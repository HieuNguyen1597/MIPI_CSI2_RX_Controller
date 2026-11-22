// Module      : m_fpt_control_fsm
// Block       : B10 - Control FSM
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-08
// Description : Frame-level drop-and-recover policy: collects causes, sequences flush and resume.
`timescale 1ns/1ps
`default_nettype none

module m_fpt_control_fsm (
  input  wire         i_sys_clk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  input  wire         i_core_enable,
  input  wire         i_axis_enable,
  input  wire         i_stream_follow_dma_drop,
  input  wire         i_vc_exposure_map,
  input  wire         i_tpg_dual_vc_en,
  input  wire         i_err_ecc_2bit,
  input  wire         i_err_lane_abort,
  input  wire         i_err_crc,
  input  wire         i_err_wc,
  input  wire  [1:0]  i_err_vc,
  output logic [1:0]  o_force_discard,
  input  wire         i_active_sel,
  output logic        o_rx_ctx_clear,
  input  wire         i_linebuffer_full,
  input  wire         i_linebuf_overflow,
  output logic        o_lb_flush,
  input  wire         i_fb_overflow,
  output logic        o_fb_flush,
  input  wire  [1:0]  i_capture_start,
  input  wire  [1:0]  i_capture_end,
  input  wire  [1:0]  i_capture_dropped,
  input  wire         i_ev_fs,
  input  wire  [1:0]  i_ev_vc,
  input  wire         i_lead_vc,
  output logic        o_capture_drop,
  output logic [1:0]  o_drop_vc_mask,
  output logic        o_asm_abort_req,
  output logic [1:0]  o_asm_abort_mask,
  output logic        o_asm_skip_req,
  output logic [1:0]  o_asm_skip_mask,
  input  wire         i_dma_drop,
  output logic        o_dma_abort_req,
  output logic        o_frame_drop_pulse
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  // HOLD_CYCLES is OUT_PIPE_STAGES of Byte-to-Pixel Unpacking plus one; hold_cnt counts the
  // cycles of ST_HOLD from 0 to HOLD_CYCLES - 1.
  localparam logic [31:0] LPARA_HOLD_CYCLES = 32'd2;
  localparam logic [31:0] LPARA_HOLD_CNT_W  =
    (LPARA_HOLD_CYCLES > 32'd1) ? $clog2(LPARA_HOLD_CYCLES) : 32'd1;
  localparam logic [LPARA_HOLD_CNT_W-1:0] LPARA_HOLD_LAST =
    LPARA_HOLD_CNT_W'(LPARA_HOLD_CYCLES - 32'd1);

  // Drop FSM states.
  typedef enum logic [2:0] {
    ENUM_ST_IDLE  = 3'b000,
    ENUM_ST_RUN   = 3'b001,
    ENUM_ST_HOLD  = 3'b010,
    ENUM_ST_FLUSH = 3'b011,
    ENUM_ST_SKIP  = 3'b100
  } drop_state_type_def;

  // Cause Collector
  logic                        w_rx;
  logic                        w_vc_ok;
  logic                        w_c_ecc2;
  logic                        w_c_crc;
  logic                        w_c_wc;
  logic                        w_c_lane;
  logic                        w_c_lbfull;
  logic                        w_c_lbovf;
  logic                        w_c_fbovf;
  logic                        w_c_core;
  logic                        w_abort_any;
  logic                        w_midline;
  logic                        w_stream_stop;
  logic                        w_dma_skip;
  logic                        w_dma_evt;
  logic                        reg_axis_enable;

  // Capture Watch
  logic [1:0]                  reg_cap_open;
  logic                        w_cap_busy;
  logic                        w_lead_start;
  logic                        w_lag_fs;
  logic                        reg_last_slot;
  logic [1:0]                  reg_dma_mark;
  logic [1:0]                  w_drop_end;

  // Drop Report
  logic                        reg_drop_pend;
  logic                        w_frame_drop_pulse;

  // Drop FSM
  drop_state_type_def          reg_fsm_drop_current_state;
  drop_state_type_def          w_fsm_drop_nxt_state;
  logic [LPARA_HOLD_CNT_W-1:0] reg_hold_cnt;
  logic [LPARA_HOLD_CNT_W-1:0] w_hold_cnt_nxt;
  logic                        w_hold_last;
  logic                        reg_lead_seen;
  logic                        w_lead_seen_nxt;
  logic                        w_drop_now;
  logic                        w_skip_now;
  logic                        reg_drop_now;
  logic                        reg_skip_now;
  logic                        reg_core_enable;
  logic                        reg_stream_stop;
  logic                        w_s;
  logic                        w_go_run;
  logic [1:0]                  w_clr;
  logic [1:0]                  reg_discard;

  // Command outputs
  logic                        w_capture_drop;
  logic [1:0]                  w_drop_vc_mask;
  logic                        w_asm_abort_req;
  logic [1:0]                  w_asm_abort_mask;
  logic [1:0]                  w_asm_skip_mask;
  logic                        w_dma_abort_req;
  logic                        w_lb_flush;
  logic                        w_rx_ctx_clear;

  // ------------------------------------------------------------------------------------------
  // Cause Collector
  // ------------------------------------------------------------------------------------------
  // Sensor-path causes count only while active_sel selects the sensor; err_crc and err_wc also
  // need the discard flag of their channel clear. fb_overflow, core_enable low and dma_drop act on
  // both sources. stream_stop is the falling edge of axis_enable.
  always_comb begin : p_com_cause
    w_rx          = ~i_active_sel;
    w_vc_ok       = ~reg_discard[i_err_vc[0]];
    w_c_ecc2      = i_err_ecc_2bit & w_rx;
    w_c_crc       = i_err_crc & w_vc_ok & w_rx;
    w_c_wc        = i_err_wc & w_vc_ok & w_rx;
    w_c_lane      = i_err_lane_abort & w_rx;
    w_c_lbfull    = i_linebuffer_full & w_rx;
    w_c_lbovf     = i_linebuf_overflow & w_rx;
    w_c_fbovf     = i_fb_overflow;
    w_c_core      = ~i_core_enable;
    w_abort_any   = w_c_ecc2 | w_c_crc | w_c_wc | w_c_lane | w_c_lbfull | w_c_lbovf | w_c_fbovf |
                    w_c_core;
    w_midline     = w_c_lbfull | w_c_lbovf | w_c_fbovf;
    w_stream_stop = reg_axis_enable & ~i_axis_enable;
    w_dma_skip    = i_dma_drop & i_stream_follow_dma_drop;
    w_dma_evt     = i_dma_drop;
  end

  // axis_enable delayed by one cycle; reset to 0 so that no stream_stop follows a reset.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_cause
    if (!i_resetn) begin
      reg_axis_enable <= 1'b0;
    end else if (i_soft_reset) begin
      reg_axis_enable <= 1'b0;
    end else begin
      reg_axis_enable <= i_axis_enable;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Capture Watch
  // ------------------------------------------------------------------------------------------
  // A slot is busy from its capture_start to its capture_end. A capture ends dropped when it was
  // marked dropped or when dma_drop arrived for it; dma_drop applies to the most recent slot.
  always_comb begin : p_com_cap_watch
    w_cap_busy   = |reg_cap_open;
    w_lead_start = |i_capture_start;
    w_lag_fs     = i_ev_fs & (i_ev_vc[0] != i_lead_vc);
    w_drop_end   = i_capture_end & (i_capture_dropped | reg_dma_mark);
  end

  // Per slot: capture_start wins over capture_end and clears the DMA mark of that slot.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_cap_watch
    if (!i_resetn) begin
      reg_cap_open  <= '0;
      reg_last_slot <= 1'b0;
      reg_dma_mark  <= '0;
    end else if (i_soft_reset) begin
      reg_cap_open  <= '0;
      reg_last_slot <= 1'b0;
      reg_dma_mark  <= '0;
    end else begin
      for (int idx = 0; idx < $bits(reg_cap_open); idx++) begin
        if (i_capture_start[idx]) begin
          reg_cap_open[idx] <= 1'b1;
          reg_dma_mark[idx] <= 1'b0;
        end else begin
          if (i_capture_end[idx]) begin
            reg_cap_open[idx] <= 1'b0;
          end
          if (w_dma_evt && (reg_last_slot == 1'(idx))) begin
            reg_dma_mark[idx] <= 1'b1;
          end
        end
      end
      if (i_capture_start[1]) begin
        reg_last_slot <= 1'b1;
      end else if (i_capture_start[0]) begin
        reg_last_slot <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Drop Report
  // ------------------------------------------------------------------------------------------
  // One pulse per capture that ends dropped; when both slots end dropped in one cycle the second
  // pulse follows in the next cycle.
  always_comb begin : p_com_drop_report
    w_frame_drop_pulse = (|w_drop_end) | reg_drop_pend;
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_drop_report
    if (!i_resetn) begin
      reg_drop_pend <= 1'b0;
    end else if (i_soft_reset) begin
      reg_drop_pend <= 1'b0;
    end else begin
      reg_drop_pend <= w_drop_end[0] & w_drop_end[1];
    end
  end

  // ------------------------------------------------------------------------------------------
  // Drop FSM
  // ------------------------------------------------------------------------------------------
  // Next state. ST_RUN evaluates the causes: a capture open or a mid-line cause goes through
  // ST_HOLD and ST_FLUSH, otherwise ST_SKIP, or ST_IDLE once core_enable is low. In ST_SKIP the
  // exit to ST_IDLE wins over the resume. Unused codes recover to ST_IDLE.
  always_comb begin : p_com_drop_nxt
    w_fsm_drop_nxt_state = reg_fsm_drop_current_state;
    case (reg_fsm_drop_current_state)
      ENUM_ST_IDLE: begin
        if (i_core_enable) begin
          w_fsm_drop_nxt_state = ENUM_ST_SKIP;
        end
      end
      ENUM_ST_RUN: begin
        if (w_abort_any && (w_cap_busy || w_midline)) begin
          w_fsm_drop_nxt_state = ENUM_ST_HOLD;
        end else if (w_abort_any && i_core_enable) begin
          w_fsm_drop_nxt_state = ENUM_ST_SKIP;
        end else if (!i_core_enable) begin
          w_fsm_drop_nxt_state = ENUM_ST_IDLE;
        end
      end
      ENUM_ST_HOLD: begin
        if (w_hold_last) begin
          w_fsm_drop_nxt_state = ENUM_ST_FLUSH;
        end
      end
      ENUM_ST_FLUSH: begin
        if (i_core_enable) begin
          w_fsm_drop_nxt_state = ENUM_ST_SKIP;
        end else begin
          w_fsm_drop_nxt_state = ENUM_ST_IDLE;
        end
      end
      ENUM_ST_SKIP: begin
        if (!i_core_enable) begin
          w_fsm_drop_nxt_state = ENUM_ST_IDLE;
        end else if (w_lead_start || reg_lead_seen) begin
          w_fsm_drop_nxt_state = ENUM_ST_RUN;
        end
      end
      default: begin
        w_fsm_drop_nxt_state = ENUM_ST_IDLE;
      end
    endcase
  end

  // State register: resetn, then soft_reset, then the next state.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_drop_state
    if (!i_resetn) begin
      reg_fsm_drop_current_state <= ENUM_ST_IDLE;
    end else if (i_soft_reset) begin
      reg_fsm_drop_current_state <= ENUM_ST_IDLE;
    end else begin
      reg_fsm_drop_current_state <= w_fsm_drop_nxt_state;
    end
  end

  // Decisions and flag updates. hold_cnt is 0 outside ST_HOLD and counts its cycles. lead_seen
  // records a lead capture_start in ST_HOLD or ST_FLUSH and is cleared in ST_RUN and ST_IDLE.
  // The discard flags are set in ST_IDLE, ST_HOLD, ST_FLUSH and at a drop decision; the lead
  // flag clears on resume, the lag flag at the next lag Frame Start once the lead flag is clear.
  always_comb begin : p_com_drop_ctrl
    w_hold_last     = (reg_hold_cnt == LPARA_HOLD_LAST);
    w_hold_cnt_nxt  = '0;
    if ((reg_fsm_drop_current_state == ENUM_ST_HOLD) && !w_hold_last) begin
      w_hold_cnt_nxt = LPARA_HOLD_CNT_W'(reg_hold_cnt + 1'b1);
    end
    w_lead_seen_nxt = reg_lead_seen;
    if (((reg_fsm_drop_current_state == ENUM_ST_HOLD) ||
         (reg_fsm_drop_current_state == ENUM_ST_FLUSH)) && w_lead_start) begin
      w_lead_seen_nxt = 1'b1;
    end else if ((reg_fsm_drop_current_state == ENUM_ST_RUN) ||
                 (reg_fsm_drop_current_state == ENUM_ST_IDLE)) begin
      w_lead_seen_nxt = 1'b0;
    end
    w_drop_now = (reg_fsm_drop_current_state == ENUM_ST_RUN) & w_abort_any;
    w_skip_now = (reg_fsm_drop_current_state == ENUM_ST_RUN) & w_dma_skip;
    w_s        = (reg_fsm_drop_current_state == ENUM_ST_IDLE) |
                 (reg_fsm_drop_current_state == ENUM_ST_HOLD) |
                 (reg_fsm_drop_current_state == ENUM_ST_FLUSH) | w_drop_now;
    w_go_run   = (reg_fsm_drop_current_state == ENUM_ST_SKIP) & i_core_enable &
                 (w_lead_start | reg_lead_seen);
    w_clr      = '0;
    for (int idx = 0; idx < $bits(w_clr); idx++) begin
      w_clr[idx] = (w_go_run & (i_lead_vc == 1'(idx))) |
                   (w_lag_fs & ~reg_discard[i_lead_vc] & (i_lead_vc != 1'(idx)));
    end
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_hold_cnt
    if (!i_resetn) begin
      reg_hold_cnt <= '0;
    end else if (i_soft_reset) begin
      reg_hold_cnt <= '0;
    end else begin
      reg_hold_cnt <= w_hold_cnt_nxt;
    end
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_lead_seen
    if (!i_resetn) begin
      reg_lead_seen <= 1'b0;
    end else if (i_soft_reset) begin
      reg_lead_seen <= 1'b0;
    end else begin
      reg_lead_seen <= w_lead_seen_nxt;
    end
  end

  // Both flags are set by reset; set wins over clear.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_discard
    if (!i_resetn) begin
      reg_discard <= 2'b11;
    end else if (i_soft_reset) begin
      reg_discard <= 2'b11;
    end else if (w_s) begin
      reg_discard <= 2'b11;
    end else begin
      reg_discard <= reg_discard & ~w_clr;
    end
  end

  // One-cycle delays of the decisions, of core_enable and of stream_stop, so that the commands
  // appear in the first cycle of the new state.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_cmd_q
    if (!i_resetn) begin
      reg_drop_now    <= 1'b0;
      reg_skip_now    <= 1'b0;
      reg_core_enable <= 1'b0;
      reg_stream_stop <= 1'b0;
    end else if (i_soft_reset) begin
      reg_drop_now    <= 1'b0;
      reg_skip_now    <= 1'b0;
      reg_core_enable <= 1'b0;
      reg_stream_stop <= 1'b0;
    end else begin
      reg_drop_now    <= w_drop_now;
      reg_skip_now    <= w_skip_now;
      reg_core_enable <= i_core_enable;
      reg_stream_stop <= w_stream_stop;
    end
  end

  // Commands. A drop decided with core_enable high marks every channel; a DMA skip marks the
  // lead channel only. Flush commands are decoded from the state register; the abort mask names
  // every exposure in use, and rx_ctx_clear acts on the sensor path only.
  always_comb begin : p_com_cmd_out
    w_capture_drop   = (reg_drop_now & reg_core_enable) | reg_skip_now;
    w_drop_vc_mask   = reg_drop_now ? 2'b11 : {i_lead_vc, ~i_lead_vc};
    w_asm_skip_mask  = 2'b01;
    w_lb_flush       = (reg_fsm_drop_current_state == ENUM_ST_FLUSH);
    w_rx_ctx_clear   = w_lb_flush & ~i_active_sel;
    w_asm_abort_req  = w_lb_flush | reg_stream_stop;
    w_asm_abort_mask = 2'b11;
    if (i_active_sel && !i_tpg_dual_vc_en) begin
      w_asm_abort_mask = {i_vc_exposure_map, ~i_vc_exposure_map};
    end
    w_dma_abort_req  = (reg_fsm_drop_current_state == ENUM_ST_HOLD) & (reg_hold_cnt == '0);
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  // Payload suppression and flush commands to the pipeline.
  assign o_force_discard    = reg_discard;
  assign o_lb_flush         = w_lb_flush;
  assign o_fb_flush         = w_lb_flush;
  assign o_rx_ctx_clear     = w_rx_ctx_clear;

  // Commands to Frame Context & Metadata Capture, AXI4-Stream Master and the DMA.
  assign o_capture_drop     = w_capture_drop;
  assign o_drop_vc_mask     = w_drop_vc_mask;
  assign o_asm_abort_req    = w_asm_abort_req;
  assign o_asm_abort_mask   = w_asm_abort_mask;
  assign o_asm_skip_req     = reg_skip_now;
  assign o_asm_skip_mask    = w_asm_skip_mask;
  assign o_dma_abort_req    = w_dma_abort_req;

  // Drop count towards Interrupt & Error Aggregator.
  assign o_frame_drop_pulse = w_frame_drop_pulse;

endmodule
`default_nettype wire
