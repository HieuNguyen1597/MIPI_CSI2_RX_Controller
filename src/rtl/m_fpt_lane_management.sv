// Module      : m_fpt_lane_management
// Block       : B02 - Lane Management
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.3
// Date        : 2026-10-27
// Description : Removes lane skew, merges the lane bytes into 32-bit words and manages HS bursts.
`timescale 1ns/1ps
`default_nettype none

module m_fpt_lane_management #(
  // Parameter Description: number of lane slices instantiated (write control, delay FIFO, event
  // capture); lanes at or above this value are held inactive.
  // Parameter values: 1, 2 or 4; default is 4.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_LANES      = 4,
  // Parameter Description: depth of each PHY delay FIFO; must exceed PARA_FIFO_AF_MARGIN.
  // Parameter values: any integer >= 2; default is 8.
  // Parameter unit: byte
  parameter logic [31:0] PARA_SKEW_DEPTH     = 8,
  // Parameter Description: free FIFO entries left when fifo_full_x asserts; must be smaller than
  // PARA_SKEW_DEPTH.
  // Parameter values: 1 to PARA_SKEW_DEPTH - 1; default is 2.
  // Parameter unit: byte
  parameter logic [31:0] PARA_FIFO_AF_MARGIN = 2
) (
  input  wire        i_rxbyteclkhs,
  input  wire        i_resetn,
  input  wire        i_soft_reset,
  input  wire  [1:0] i_active_lanes,
  input  wire  [7:0] i_rxdatahs_0,
  input  wire  [7:0] i_rxdatahs_1,
  input  wire  [7:0] i_rxdatahs_2,
  input  wire  [7:0] i_rxdatahs_3,
  input  wire        i_rxvalidhs_0,
  input  wire        i_rxvalidhs_1,
  input  wire        i_rxvalidhs_2,
  input  wire        i_rxvalidhs_3,
  input  wire        i_rxactivehs_0,
  input  wire        i_rxactivehs_1,
  input  wire        i_rxactivehs_2,
  input  wire        i_rxactivehs_3,
  input  wire        i_rxsynchs_0,
  input  wire        i_rxsynchs_1,
  input  wire        i_rxsynchs_2,
  input  wire        i_rxsynchs_3,
  input  wire        i_rxskewcalhs_0,
  input  wire        i_rxskewcalhs_1,
  input  wire        i_rxskewcalhs_2,
  input  wire        i_rxskewcalhs_3,
  input  wire        i_errsoths_0,
  input  wire        i_errsoths_1,
  input  wire        i_errsoths_2,
  input  wire        i_errsoths_3,
  input  wire        i_errsotsynchs_0,
  input  wire        i_errsotsynchs_1,
  input  wire        i_errsotsynchs_2,
  input  wire        i_errsotsynchs_3,
  output logic [31:0] o_merged_data,
  output logic        o_merged_valid,
  output logic  [3:0] o_merged_byte_valid,
  output logic        o_merged_sob,
  output logic        o_merged_eob,
  output logic  [2:0] o_lm_fsm_state,
  output logic        o_abort_sync_err,
  output logic        o_abort_fifo_full,
  output logic        o_abort_unarmed,
  output logic  [3:0] o_abort_lane_sync_seen,
  output logic        o_lane_abort_pulse,
  output logic        o_deskew_burst_seen,
  output logic  [1:0] o_active_lanes_applied
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_MERGED_WIDTH = 32'd32;
  localparam logic [31:0] LPARA_MERGED_BYTES = LPARA_MERGED_WIDTH / 32'd8;
  localparam int          LPARA_PORT_LANES   = 4;
  localparam int          LPARA_PTR_W        =
    (PARA_SKEW_DEPTH > 32'd2) ? $clog2(PARA_SKEW_DEPTH) : 1;
  localparam int          LPARA_CNT_W        = $clog2(PARA_SKEW_DEPTH + 32'd1);
  localparam logic [LPARA_CNT_W-1:0] LPARA_DEPTH_CNT = LPARA_CNT_W'(PARA_SKEW_DEPTH);
  localparam logic [LPARA_CNT_W-1:0] LPARA_AF_THR    =
    LPARA_CNT_W'(PARA_SKEW_DEPTH - PARA_FIFO_AF_MARGIN);
  localparam logic [LPARA_PTR_W-1:0] LPARA_PTR_LAST  = LPARA_PTR_W'(PARA_SKEW_DEPTH - 32'd1);
  // Lanes that physically exist in this build; the other port lanes are held inactive.
  localparam logic [3:0]  LPARA_LANE_MASK    = (PARA_MAX_LANES == 32'd4) ? 4'b1111 :
                                               (PARA_MAX_LANES == 32'd2) ? 4'b0011 : 4'b0001;

  // Burst FSM states, encoded as LANE_ALIGN_STATUS reports them.
  typedef enum logic [2:0] {
    ENUM_ST_IDLE    = 3'b000,
    ENUM_ST_ARMING  = 3'b001,
    ENUM_ST_ALIGNED = 3'b010,
    ENUM_ST_DRAIN   = 3'b011,
    ENUM_ST_ABORT   = 3'b100
  } lm_state_type_def;

  lm_state_type_def reg_fsm_lm_current_state;
  lm_state_type_def w_fsm_lm_nxt_state;

  // PPI inputs gathered per lane (index = lane number)
  logic [7:0] w_rxdatahs     [LPARA_PORT_LANES];
  logic       w_rxvalidhs    [LPARA_PORT_LANES];
  logic       w_rxactivehs   [LPARA_PORT_LANES];
  logic       w_rxsynchs     [LPARA_PORT_LANES];
  logic       w_rxskewcalhs  [LPARA_PORT_LANES];
  logic       w_errsotsynchs_x [LPARA_PORT_LANES];

  // Lane event capture flags and per-lane terms
  logic       reg_active_seen [LPARA_PORT_LANES];
  logic       reg_sync_seen   [LPARA_PORT_LANES];
  logic       reg_deskew_seen [LPARA_PORT_LANES];
  logic       reg_err_sync    [LPARA_PORT_LANES];
  logic       w_sync_ok       [LPARA_PORT_LANES];
  logic       w_eot_seen      [LPARA_PORT_LANES];
  logic       w_wr_en         [LPARA_PORT_LANES];
  logic       w_pop           [LPARA_PORT_LANES];
  logic       w_fifo_not_empty [LPARA_PORT_LANES];
  logic       w_fifo_full_x     [LPARA_PORT_LANES];
  logic [7:0] w_aligned_byte  [LPARA_PORT_LANES];

  // Configuration and condition reduce
  logic [1:0] reg_active_lanes_reg;
  logic [3:0] w_active;
  logic       w_burst_open;
  logic       w_burst_closed;
  logic       w_armed;
  logic       w_any_sync;
  logic       w_any_deskew;
  logic       w_deskew_burst;
  logic       w_errsotsynchs;
  logic       w_fifo_full;
  logic       w_any_lane_has_data;
  logic       w_all_lanes_ready;
  logic       w_drain_complete;
  logic [3:0] w_sync_seen_vec;

  // FSM outputs and read control
  logic       w_burst_clr;
  logic       w_cfg_apply;
  logic       w_wr_en_ok;
  logic       w_rd_en_ok;
  logic       w_burst_active;
  logic       w_pack_flush;
  logic       w_abort_active;
  logic       w_read_en;
  logic       w_abort_entry;
  logic       w_deskew_exit;

  // Lane merging
  logic       reg_tail;
  logic       w_final_place;
  logic [3:0] w_place;
  logic [7:0] w_place_data  [LPARA_MERGED_BYTES];
  logic [3:0] w_place_valid;
  logic [7:0] w_merge_data  [LPARA_MERGED_BYTES];
  logic [3:0] w_merge_valid;
  logic [2:0] w_place_cnt;
  logic [2:0] w_pos_sum;
  logic       w_word_complete;
  logic       w_any_placed;
  logic [7:0] reg_acc       [LPARA_MERGED_BYTES];
  logic [3:0] reg_acc_valid;
  logic [1:0] reg_wr_pos;
  logic [31:0] reg_hold_data;
  logic       reg_hold_vld;
  logic       reg_sob_pending;
  logic       w_packer_empty;
  logic       w_wr_hold;
  logic       w_wr_hold_last;
  logic       w_wr_part;
  logic       w_emit;
  logic [31:0] w_emit_data;
  logic [3:0] w_emit_mask;
  logic       w_emit_eob;

  // Output and status registers
  logic [31:0] reg_merged_data;
  logic        reg_merged_valid;
  logic  [3:0] reg_merged_byte_valid;
  logic        reg_merged_sob;
  logic        reg_merged_eob;
  logic        reg_abort_sync_err;
  logic        reg_abort_fifo_full;
  logic        reg_abort_unarmed;
  logic  [3:0] reg_abort_lane_sync_seen;
  logic        reg_lane_abort_pulse;
  logic        reg_deskew_burst_seen;

  // ------------------------------------------------------------------------------------------
  // Parameter check
  // ------------------------------------------------------------------------------------------
  // Illegal parameter values stop elaboration.
  generate
  if (!((PARA_MAX_LANES == 32'd1) || (PARA_MAX_LANES == 32'd2) || (PARA_MAX_LANES == 32'd4)))
  begin : gen_chk_max_lanes
    $error("PARA_MAX_LANES must be 1, 2 or 4");
  end
  if (!((PARA_FIFO_AF_MARGIN > 32'd0) && (PARA_FIFO_AF_MARGIN < PARA_SKEW_DEPTH)))
  begin : gen_chk_af_margin
    $error("PARA_FIFO_AF_MARGIN must satisfy 0 < PARA_FIFO_AF_MARGIN < PARA_SKEW_DEPTH");
  end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Input gathering
  // ------------------------------------------------------------------------------------------
  // Per-lane PPI ports into lane-indexed arrays. errsoths_x has no consumer in this block; the
  // CSR and interrupt blocks observe it.
  always_comb begin : p_com_lane_inputs
    w_rxdatahs[0]     = i_rxdatahs_0;
    w_rxdatahs[1]     = i_rxdatahs_1;
    w_rxdatahs[2]     = i_rxdatahs_2;
    w_rxdatahs[3]     = i_rxdatahs_3;
    w_rxvalidhs[0]    = i_rxvalidhs_0;
    w_rxvalidhs[1]    = i_rxvalidhs_1;
    w_rxvalidhs[2]    = i_rxvalidhs_2;
    w_rxvalidhs[3]    = i_rxvalidhs_3;
    w_rxactivehs[0]   = i_rxactivehs_0;
    w_rxactivehs[1]   = i_rxactivehs_1;
    w_rxactivehs[2]   = i_rxactivehs_2;
    w_rxactivehs[3]   = i_rxactivehs_3;
    w_rxsynchs[0]     = i_rxsynchs_0;
    w_rxsynchs[1]     = i_rxsynchs_1;
    w_rxsynchs[2]     = i_rxsynchs_2;
    w_rxsynchs[3]     = i_rxsynchs_3;
    w_rxskewcalhs[0]  = i_rxskewcalhs_0;
    w_rxskewcalhs[1]  = i_rxskewcalhs_1;
    w_rxskewcalhs[2]  = i_rxskewcalhs_2;
    w_rxskewcalhs[3]  = i_rxskewcalhs_3;
    w_errsotsynchs_x[0] = i_errsotsynchs_0;
    w_errsotsynchs_x[1] = i_errsotsynchs_1;
    w_errsotsynchs_x[2] = i_errsotsynchs_2;
    w_errsotsynchs_x[3] = i_errsotsynchs_3;
  end

  // ------------------------------------------------------------------------------------------
  // Active lane register and decoder
  // ------------------------------------------------------------------------------------------
  // active_lanes is adopted only on cfg_apply (ST_IDLE with no burst opening); soft_reset keeps it.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_active_lanes
    if (!i_resetn) begin
      reg_active_lanes_reg <= 2'b00;
    end else if (w_cfg_apply) begin
      reg_active_lanes_reg <= i_active_lanes;
    end
  end

  // The reserved code 10 selects no lane. A selected lane that is not built stays active with its
  // flags cleared, so the burst never arms and is discarded.
  always_comb begin : p_com_active_dec
    case (reg_active_lanes_reg)
      2'b00:   w_active = 4'b0001;
      2'b01:   w_active = 4'b0011;
      2'b11:   w_active = 4'b1111;
      default: w_active = 4'b0000;
    endcase
  end

  // ------------------------------------------------------------------------------------------
  // Lane slices: lane event capture, write control and PHY delay FIFO
  // ------------------------------------------------------------------------------------------
  // Lanes at or above PARA_MAX_LANES are not built and are held inactive.
  generate
  for (genvar gen_idx = 0; gen_idx < LPARA_PORT_LANES; gen_idx++) begin : gen_lane
    if (gen_idx < PARA_MAX_LANES) begin : gen_slice
      logic [7:0]             reg_fifo_mem [PARA_SKEW_DEPTH];
      logic [LPARA_PTR_W-1:0] reg_fifo_wr_ptr;
      logic [LPARA_PTR_W-1:0] reg_fifo_rd_ptr;
      logic [LPARA_CNT_W-1:0] reg_fifo_cnt;
      logic                   w_fifo_wr_acc;

      // Lane event capture: each flag is set by its PPI source on an active lane and cleared by
      // soft_reset or burst_clr, clear first, so an rxskewcalhs_x still high after the burst has
      // no effect.
      always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_flags
        if (!i_resetn) begin
          reg_active_seen[gen_idx] <= 1'b0;
          reg_sync_seen[gen_idx]   <= 1'b0;
          reg_deskew_seen[gen_idx] <= 1'b0;
          reg_err_sync[gen_idx]    <= 1'b0;
        end else if (i_soft_reset || w_burst_clr) begin
          reg_active_seen[gen_idx] <= 1'b0;
          reg_sync_seen[gen_idx]   <= 1'b0;
          reg_deskew_seen[gen_idx] <= 1'b0;
          reg_err_sync[gen_idx]    <= 1'b0;
        end else if (w_active[gen_idx]) begin
          reg_active_seen[gen_idx] <= reg_active_seen[gen_idx] | w_rxactivehs[gen_idx];
          reg_sync_seen[gen_idx]   <= reg_sync_seen[gen_idx]   | w_rxsynchs[gen_idx];
          reg_deskew_seen[gen_idx] <= reg_deskew_seen[gen_idx] | w_rxskewcalhs[gen_idx];
          reg_err_sync[gen_idx]    <= reg_err_sync[gen_idx]    | w_errsotsynchs_x[gen_idx];
        end
      end

      // Write control: the live rxsynchs_x term admits a first byte that comes with the pulse.
      // The FIFO is first-word-fall-through; a write while it holds PARA_SKEW_DEPTH bytes is
      // dropped.
      always_comb begin : p_com_write_ctrl
        w_sync_ok[gen_idx]  = reg_sync_seen[gen_idx] | w_rxsynchs[gen_idx];
        w_eot_seen[gen_idx] = reg_active_seen[gen_idx] & ~w_rxactivehs[gen_idx];
        w_wr_en[gen_idx]    = w_rxvalidhs[gen_idx] & w_sync_ok[gen_idx] & w_wr_en_ok &
                              w_active[gen_idx] & ~i_soft_reset;
        w_fifo_wr_acc       = w_wr_en[gen_idx] & (reg_fifo_cnt != LPARA_DEPTH_CNT);
        w_fifo_not_empty[gen_idx] = (reg_fifo_cnt != '0);
        w_fifo_full_x[gen_idx]      = (reg_fifo_cnt >= LPARA_AF_THR);
        w_aligned_byte[gen_idx]   = reg_fifo_mem[reg_fifo_rd_ptr];
        w_pop[gen_idx]            = w_read_en & w_fifo_not_empty[gen_idx];
      end

      // FIFO pointers and fill counter; cleared while burst_clr or soft_reset is asserted.
      always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_fifo_ctrl
        if (!i_resetn) begin
          reg_fifo_wr_ptr <= '0;
          reg_fifo_rd_ptr <= '0;
          reg_fifo_cnt    <= '0;
        end else if (i_soft_reset || w_burst_clr) begin
          reg_fifo_wr_ptr <= '0;
          reg_fifo_rd_ptr <= '0;
          reg_fifo_cnt    <= '0;
        end else begin
          if (w_fifo_wr_acc) begin
            reg_fifo_wr_ptr <= (reg_fifo_wr_ptr == LPARA_PTR_LAST) ? '0 : reg_fifo_wr_ptr + 1'b1;
          end
          if (w_pop[gen_idx]) begin
            reg_fifo_rd_ptr <= (reg_fifo_rd_ptr == LPARA_PTR_LAST) ? '0 : reg_fifo_rd_ptr + 1'b1;
          end
          reg_fifo_cnt <= reg_fifo_cnt + LPARA_CNT_W'(w_fifo_wr_acc) - LPARA_CNT_W'(w_pop[gen_idx]);
        end
      end

      // FIFO storage: payload only, read only while the FIFO is not empty, so it has no reset.
      always_ff @(posedge i_rxbyteclkhs) begin : p_ff_fifo_mem
        if (w_fifo_wr_acc) begin
          reg_fifo_mem[reg_fifo_wr_ptr] <= w_rxdatahs[gen_idx];
        end
      end
    end else begin : gen_absent
      // Lane not built: flags stay cleared and the lane never holds data.
      always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_flags
        if (!i_resetn) begin
          reg_active_seen[gen_idx] <= 1'b0;
          reg_sync_seen[gen_idx]   <= 1'b0;
          reg_deskew_seen[gen_idx] <= 1'b0;
          reg_err_sync[gen_idx]    <= 1'b0;
        end else begin
          reg_active_seen[gen_idx] <= 1'b0;
          reg_sync_seen[gen_idx]   <= 1'b0;
          reg_deskew_seen[gen_idx] <= 1'b0;
          reg_err_sync[gen_idx]    <= 1'b0;
        end
      end

      always_comb begin : p_com_write_ctrl
        w_sync_ok[gen_idx]        = 1'b0;
        w_eot_seen[gen_idx]       = 1'b0;
        w_wr_en[gen_idx]          = 1'b0;
        w_fifo_not_empty[gen_idx] = 1'b0;
        w_fifo_full_x[gen_idx]      = 1'b0;
        w_aligned_byte[gen_idx]   = 8'h00;
        w_pop[gen_idx]            = 1'b0;
      end
    end
  end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Condition reduce
  // ------------------------------------------------------------------------------------------
  // "any" terms are OR and "all" terms are AND over the active lanes; an inactive lane neither
  // asserts nor blocks a condition. rxactivehs_x of a lane that is not built never opens a burst.
  always_comb begin : p_com_cond_reduce
    w_burst_open        = 1'b0;
    w_any_sync          = 1'b0;
    w_any_deskew        = 1'b0;
    w_errsotsynchs      = 1'b0;
    w_fifo_full         = 1'b0;
    w_any_lane_has_data = 1'b0;
    w_armed             = 1'b1;
    w_all_lanes_ready   = 1'b1;
    for (int idx = 0; idx < LPARA_PORT_LANES; idx++) begin
      w_sync_seen_vec[idx] = reg_sync_seen[idx];
      if (w_active[idx]) begin
        w_burst_open        = w_burst_open | (w_rxactivehs[idx] & LPARA_LANE_MASK[idx]);
        w_any_sync          = w_any_sync | reg_sync_seen[idx];
        w_any_deskew        = w_any_deskew | reg_deskew_seen[idx];
        w_errsotsynchs      = w_errsotsynchs | reg_err_sync[idx];
        w_fifo_full         = w_fifo_full | (reg_sync_seen[idx] & w_fifo_full_x[idx]);
        w_any_lane_has_data = w_any_lane_has_data | w_fifo_not_empty[idx];
        w_armed             = w_armed & reg_sync_seen[idx];
        w_all_lanes_ready   = w_all_lanes_ready & (w_fifo_not_empty[idx] | w_eot_seen[idx]);
      end
    end
    w_burst_closed   = ~w_burst_open;
    w_deskew_burst   = w_any_deskew & ~w_any_sync;
    w_drain_complete = w_packer_empty & ~w_any_lane_has_data;
  end

  // ------------------------------------------------------------------------------------------
  // Burst FSM outputs
  // ------------------------------------------------------------------------------------------
  // In ST_IDLE the three outputs that depend on burst_open let a sync pulse, error pulse or first
  // byte arriving with the rising edge of rxactivehs_x be captured; the others depend on the
  // state only.
  always_comb begin : p_com_fsm_out
    w_abort_active = (reg_fsm_lm_current_state == ENUM_ST_ABORT);
    w_burst_clr    = ((reg_fsm_lm_current_state == ENUM_ST_IDLE) & ~w_burst_open) | w_abort_active;
    w_cfg_apply    = (reg_fsm_lm_current_state == ENUM_ST_IDLE) & ~w_burst_open;
    w_wr_en_ok     = (reg_fsm_lm_current_state == ENUM_ST_ARMING) |
                     (reg_fsm_lm_current_state == ENUM_ST_ALIGNED) |
                     ((reg_fsm_lm_current_state == ENUM_ST_IDLE) & w_burst_open);
    w_rd_en_ok     = (reg_fsm_lm_current_state == ENUM_ST_ALIGNED) |
                     (reg_fsm_lm_current_state == ENUM_ST_DRAIN);
    w_burst_active = w_rd_en_ok;
    w_pack_flush   = (reg_fsm_lm_current_state == ENUM_ST_DRAIN);
  end

  // ------------------------------------------------------------------------------------------
  // Burst FSM next state
  // ------------------------------------------------------------------------------------------
  // From ST_ARMING an errsotsynchs abort wins, armed comes second and wins over fifo_full and
  // burst_closed, then the other abort causes, then the silent return of a deskew burst. No
  // timeout; unused codes recover to ST_IDLE.
  always_comb begin : p_com_fsm_nxt
    w_fsm_lm_nxt_state = reg_fsm_lm_current_state;
    case (reg_fsm_lm_current_state)
      ENUM_ST_IDLE: begin
        if (w_burst_open) begin
          w_fsm_lm_nxt_state = ENUM_ST_ARMING;
        end
      end
      ENUM_ST_ARMING: begin
        if (w_errsotsynchs) begin
          w_fsm_lm_nxt_state = ENUM_ST_ABORT;
        end else if (w_armed) begin
          w_fsm_lm_nxt_state = ENUM_ST_ALIGNED;
        end else if (w_fifo_full || (w_burst_closed && !w_deskew_burst)) begin
          w_fsm_lm_nxt_state = ENUM_ST_ABORT;
        end else if (w_burst_closed && w_deskew_burst) begin
          w_fsm_lm_nxt_state = ENUM_ST_IDLE;
        end
      end
      ENUM_ST_ALIGNED: begin
        if (w_burst_closed) begin
          w_fsm_lm_nxt_state = ENUM_ST_DRAIN;
        end
      end
      ENUM_ST_DRAIN: begin
        if (w_drain_complete) begin
          w_fsm_lm_nxt_state = ENUM_ST_IDLE;
        end
      end
      ENUM_ST_ABORT: begin
        if (w_burst_closed) begin
          w_fsm_lm_nxt_state = ENUM_ST_IDLE;
        end
      end
      default: begin
        w_fsm_lm_nxt_state = ENUM_ST_IDLE;
      end
    endcase
    w_abort_entry = (reg_fsm_lm_current_state == ENUM_ST_ARMING) &&
                    (w_fsm_lm_nxt_state == ENUM_ST_ABORT);
    w_deskew_exit = (reg_fsm_lm_current_state == ENUM_ST_ARMING) &&
                    (w_fsm_lm_nxt_state == ENUM_ST_IDLE);
  end

  // State register; soft_reset returns the FSM to ST_IDLE.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_fsm
    if (!i_resetn) begin
      reg_fsm_lm_current_state <= ENUM_ST_IDLE;
    end else if (i_soft_reset) begin
      reg_fsm_lm_current_state <= ENUM_ST_IDLE;
    end else begin
      reg_fsm_lm_current_state <= w_fsm_lm_nxt_state;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Read control and final placement
  // ------------------------------------------------------------------------------------------
  // One read pops every non-empty FIFO. A lane is finished once its rxactivehs_x has fallen and
  // its FIFO is empty. The first read that sees a finished active lane is the last one placed:
  // only the lanes below the lowest finished lane are placed.
  always_comb begin : p_com_read_ctrl
    logic blocked;
    w_read_en     = w_rd_en_ok & w_all_lanes_ready & w_any_lane_has_data;
    w_final_place = 1'b0;
    blocked       = reg_tail;
    for (int idx = 0; idx < LPARA_PORT_LANES; idx++) begin
      if (w_active[idx] && w_eot_seen[idx] && !w_fifo_not_empty[idx]) begin
        blocked = 1'b1;
        if (w_read_en && !reg_tail) begin
          w_final_place = 1'b1;
        end
      end
      w_place[idx] = w_pop[idx] & ~blocked;
    end
  end

  // Later pops are discarded until the burst ends: the tail flag clears only when burst_active
  // falls or on reset, never on an emitted word.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_tail
    if (!i_resetn) begin
      reg_tail <= 1'b0;
    end else if (i_soft_reset || !w_burst_active) begin
      reg_tail <= 1'b0;
    end else if (w_final_place) begin
      reg_tail <= 1'b1;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Byte router, pos counter and merge mux
  // ------------------------------------------------------------------------------------------
  // Placed lanes are contiguous from lane 0, so lane x goes to word position (wr_pos + x) mod 4.
  // A read never spills into the next word: before the final placement every read places N
  // bytes from a position aligned to N, and the final placement places fewer than N.
  always_comb begin : p_com_byte_router
    logic [1:0] pos;
    w_place_cnt   = 3'd0;
    w_place_valid = 4'b0000;
    for (int idx = 0; idx < LPARA_MERGED_BYTES; idx++) begin
      w_place_data[idx] = 8'h00;
    end
    for (int idx = 0; idx < LPARA_PORT_LANES; idx++) begin
      pos = reg_wr_pos + 2'(idx);
      if (w_place[idx]) begin
        w_place_data[pos]  = w_aligned_byte[idx];
        w_place_valid[pos] = 1'b1;
        w_place_cnt        = w_place_cnt + 3'd1;
      end
    end
    w_any_placed    = (w_place_cnt != 3'd0);
    w_pos_sum       = {1'b0, reg_wr_pos} + w_place_cnt;
    w_word_complete = w_pos_sum[2];
    for (int idx = 0; idx < LPARA_MERGED_BYTES; idx++) begin
      w_merge_data[idx] = w_place_valid[idx] ? w_place_data[idx] : reg_acc[idx];
    end
    w_merge_valid = reg_acc_valid | w_place_valid;
  end

  // ------------------------------------------------------------------------------------------
  // Emit control
  // ------------------------------------------------------------------------------------------
  // A completed word is held until it is known whether it ends the burst: it is written with
  // merged_eob = 0 when the next word's first byte is placed, or with merged_eob = 1 in ST_DRAIN
  // when no byte is held or placed. A flushed partial word is always the last word; merged_sob
  // marks the first word of the burst.
  always_comb begin : p_com_emit
    w_wr_hold      = reg_hold_vld & w_any_placed;
    w_wr_hold_last = reg_hold_vld & w_pack_flush & ~w_any_placed;
    w_wr_part      = ~reg_hold_vld & w_pack_flush & ~w_any_placed & (reg_acc_valid != 4'b0000);
    w_emit         = w_wr_hold | w_wr_hold_last | w_wr_part;
    w_emit_eob     = w_wr_hold_last | w_wr_part;
    if (reg_hold_vld) begin
      w_emit_data = reg_hold_data;
      w_emit_mask = 4'b1111;
    end else begin
      w_emit_data = {reg_acc[3], reg_acc[2], reg_acc[1], reg_acc[0]};
      w_emit_mask = reg_acc_valid;
    end
    w_packer_empty = (reg_acc_valid == 4'b0000) & ~reg_hold_vld & ~reg_merged_valid;
  end

  // Word buffer, held word and first-word flag; all cleared when burst_active is deasserted.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_word_buffer
    if (!i_resetn) begin
      for (int idx = 0; idx < LPARA_MERGED_BYTES; idx++) begin
        reg_acc[idx] <= 8'h00;
      end
      reg_acc_valid   <= 4'b0000;
      reg_wr_pos      <= 2'd0;
      reg_hold_data   <= 32'h0000_0000;
      reg_hold_vld    <= 1'b0;
      reg_sob_pending <= 1'b1;
    end else if (i_soft_reset || !w_burst_active) begin
      reg_acc_valid   <= 4'b0000;
      reg_wr_pos      <= 2'd0;
      reg_hold_vld    <= 1'b0;
      reg_sob_pending <= 1'b1;
    end else begin
      for (int idx = 0; idx < LPARA_MERGED_BYTES; idx++) begin
        if (w_place_valid[idx]) begin
          reg_acc[idx] <= w_place_data[idx];
        end
      end
      if (w_word_complete) begin
        reg_hold_data <= {w_merge_data[3], w_merge_data[2], w_merge_data[1], w_merge_data[0]};
        reg_hold_vld  <= 1'b1;
        reg_acc_valid <= 4'b0000;
        reg_wr_pos    <= 2'd0;
      end else begin
        if (w_wr_hold || w_wr_hold_last) begin
          reg_hold_vld <= 1'b0;
        end
        if (w_wr_part) begin
          reg_acc_valid <= 4'b0000;
          reg_wr_pos    <= 2'd0;
        end else begin
          reg_acc_valid <= w_merge_valid;
          reg_wr_pos    <= w_pos_sum[1:0];
        end
      end
      if (w_emit) begin
        reg_sob_pending <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output register
  // ------------------------------------------------------------------------------------------
  // One word per emit, or the abort marker in the cycle after lane_abort_pulse (valid, sob and
  // eob set, byte mask 0000, data zero). The marker never meets a data word because the packer
  // is empty whenever ST_ARMING is entered.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_output
    if (!i_resetn) begin
      reg_merged_data       <= 32'h0000_0000;
      reg_merged_valid      <= 1'b0;
      reg_merged_byte_valid <= 4'b0000;
      reg_merged_sob        <= 1'b0;
      reg_merged_eob        <= 1'b0;
    end else if (i_soft_reset) begin
      reg_merged_valid      <= 1'b0;
    end else if (reg_lane_abort_pulse) begin
      reg_merged_data       <= 32'h0000_0000;
      reg_merged_valid      <= 1'b1;
      reg_merged_byte_valid <= 4'b0000;
      reg_merged_sob        <= 1'b1;
      reg_merged_eob        <= 1'b1;
    end else if (w_emit) begin
      reg_merged_data       <= w_emit_data;
      reg_merged_valid      <= 1'b1;
      reg_merged_byte_valid <= w_emit_mask;
      reg_merged_sob        <= reg_sob_pending;
      reg_merged_eob        <= w_emit_eob;
    end else begin
      reg_merged_valid      <= 1'b0;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Status report
  // ------------------------------------------------------------------------------------------
  // On the ARMING -> ABORT transition the cause bits and the sync snapshot take the values of
  // that cycle, replacing the previous abort; lane_abort_pulse is high for the first ABORT cycle
  // only. A deskew burst sets the sticky deskew_burst_seen. soft_reset clears every status bit.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn) begin : p_ff_status
    if (!i_resetn) begin
      reg_abort_sync_err       <= 1'b0;
      reg_abort_fifo_full      <= 1'b0;
      reg_abort_unarmed        <= 1'b0;
      reg_abort_lane_sync_seen <= 4'b0000;
      reg_lane_abort_pulse     <= 1'b0;
      reg_deskew_burst_seen    <= 1'b0;
    end else if (i_soft_reset) begin
      reg_abort_sync_err       <= 1'b0;
      reg_abort_fifo_full      <= 1'b0;
      reg_abort_unarmed        <= 1'b0;
      reg_abort_lane_sync_seen <= 4'b0000;
      reg_lane_abort_pulse     <= 1'b0;
      reg_deskew_burst_seen    <= 1'b0;
    end else begin
      reg_lane_abort_pulse <= w_abort_entry;
      if (w_abort_entry) begin
        reg_abort_sync_err       <= w_errsotsynchs;
        reg_abort_fifo_full      <= w_fifo_full;
        reg_abort_unarmed        <= w_burst_closed & ~w_deskew_burst;
        reg_abort_lane_sync_seen <= w_sync_seen_vec;
      end
      if (w_deskew_exit) begin
        reg_deskew_burst_seen <= 1'b1;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  assign o_merged_data          = reg_merged_data;
  assign o_merged_valid         = reg_merged_valid;
  assign o_merged_byte_valid    = reg_merged_byte_valid;
  assign o_merged_sob           = reg_merged_sob;
  assign o_merged_eob           = reg_merged_eob;
  assign o_lm_fsm_state         = reg_fsm_lm_current_state;
  assign o_abort_sync_err       = reg_abort_sync_err;
  assign o_abort_fifo_full      = reg_abort_fifo_full;
  assign o_abort_unarmed        = reg_abort_unarmed;
  assign o_abort_lane_sync_seen = reg_abort_lane_sync_seen;
  assign o_lane_abort_pulse     = reg_lane_abort_pulse;
  assign o_deskew_burst_seen    = reg_deskew_burst_seen;
  assign o_active_lanes_applied = reg_active_lanes_reg;

endmodule
`default_nettype wire
