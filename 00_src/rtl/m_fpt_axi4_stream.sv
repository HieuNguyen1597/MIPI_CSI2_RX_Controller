// Module      : m_fpt_axi4_stream
// Block       : B12 - AXI4-Stream Master
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.3
// Date        : 2026-11-15
// Description : AXI4-Stream output to the ISP with skid buffer, frame gate and abort sequencer
`timescale 1ns/1ps
`default_nettype none

module m_fpt_axi4_stream (
  input  wire         i_axis_aclk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  input  wire  [63:0] i_fbs_data,
  input  wire         i_fbs_valid,
  input  wire  [7:0]  i_fbs_byte_en,
  input  wire         i_fbs_sof,
  input  wire         i_fbs_eol,
  input  wire  [1:0]  i_fbs_vc_tag,
  input  wire         i_fbs_err,
  output logic        o_fbs_ready,
  input  wire         i_asm_abort_req,
  input  wire  [1:0]  i_asm_abort_mask,
  output logic        o_asm_abort_done,
  input  wire         i_asm_skip_req,
  input  wire  [1:0]  i_asm_skip_mask,
  input  wire         i_axis_enable,
  input  wire         i_vc_exposure_map,
  // AMBA AXI4-Stream (IHI0051B) transmitter towards the ISP. Optional signals present:
  // TKEEP, TLAST, TID, TUSER; not implemented: TSTRB, TDEST, TWAKEUP.
  output logic [63:0] o_m_axis_tdata,
  output logic        o_m_axis_tvalid,
  input  wire         i_m_axis_tready,
  output logic        o_m_axis_tlast,
  output logic [1:0]  o_m_axis_tuser,
  output logic [1:0]  o_m_axis_tid,
  output logic [7:0]  o_m_axis_tkeep
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_TDATA_WIDTH = 32'd64;
  localparam logic [31:0] LPARA_TKEEP_WIDTH = LPARA_TDATA_WIDTH / 32'd8;
  localparam logic [31:0] LPARA_TUSER_WIDTH = 32'd2;
  localparam logic [31:0] LPARA_TID_WIDTH   = 32'd2;

  // Finalize Sequencer states.
  typedef enum logic [1:0] {
    ENUM_ST_PASSTHROUGH = 2'b00,
    ENUM_ST_DRAIN       = 2'b01,
    ENUM_ST_INJECT      = 2'b10
  } fseq_state_type_def;

  fseq_state_type_def reg_fsm_fseq_current_state;
  fseq_state_type_def w_fsm_fseq_nxt_state;

  logic                         w_axis_in_valid;
  logic [LPARA_TDATA_WIDTH-1:0] w_axis_in_data;
  logic [LPARA_TKEEP_WIDTH-1:0] w_axis_in_keep;
  logic                         w_axis_in_last;
  logic [LPARA_TUSER_WIDTH-1:0] w_axis_in_user;
  logic [LPARA_TID_WIDTH-1:0]   w_axis_in_tid;

  logic                         w_sof_0;
  logic                         w_sof_1;
  logic [1:0]                   w_pass_nxt;
  logic                         w_gate_pass;
  logic [1:0]                   reg_pass;
  logic [1:0]                   reg_skip_arm;

  logic                         w_fbs_ready;
  logic                         w_take;
  logic                         w_accepted;
  logic                         w_out_free;

  logic                         w_sink;
  logic                         w_drained;
  logic                         w_abort_hs;
  logic [1:0]                   w_abort_mask_nxt;
  logic                         w_line_open_nxt;
  logic                         w_todo;
  logic [LPARA_TID_WIDTH-1:0]   w_inj_tid;
  logic                         w_abort_start;
  logic                         w_inject_load;
  logic                         w_asm_abort_done;
  logic [1:0]                   reg_abort_mask;

  logic                         reg_line_open;
  logic [LPARA_TID_WIDTH-1:0]   reg_line_tid;

  logic                         reg_skid_valid;
  logic [LPARA_TDATA_WIDTH-1:0] reg_skid_data;
  logic [LPARA_TKEEP_WIDTH-1:0] reg_skid_keep;
  logic                         reg_skid_last;
  logic [LPARA_TUSER_WIDTH-1:0] reg_skid_user;
  logic [LPARA_TID_WIDTH-1:0]   reg_skid_tid;

  logic                         reg_out_valid;
  logic [LPARA_TDATA_WIDTH-1:0] reg_out_data;
  logic [LPARA_TKEEP_WIDTH-1:0] reg_out_keep;
  logic                         reg_out_last;
  logic [LPARA_TUSER_WIDTH-1:0] reg_out_user;
  logic [LPARA_TID_WIDTH-1:0]   reg_out_tid;

  // ------------------------------------------------------------------------------------------
  // TID Map
  // ------------------------------------------------------------------------------------------
  // Exposure index of the beat: the VC tag, with bit 0 inverted when vc_exposure_map swaps
  // Long and Short (TID 0 = Long, TID 1 = Short).
  always_comb begin : p_com_tid_map
    w_axis_in_tid = i_fbs_vc_tag;
    if (i_vc_exposure_map) begin
      w_axis_in_tid = {i_fbs_vc_tag[1], ~i_fbs_vc_tag[0]};
    end
  end

  // ------------------------------------------------------------------------------------------
  // Ready Logic
  // ------------------------------------------------------------------------------------------
  // fbs_ready comes from registers only: 1 while an abort sequence sinks the input, otherwise
  // high while the Park Register is empty. A beat is taken on fbs_valid && fbs_ready.
  always_comb begin : p_com_ready
    w_fbs_ready = w_sink || !reg_skid_valid;
    w_take      = i_fbs_valid && w_fbs_ready;
  end

  // ------------------------------------------------------------------------------------------
  // Frame Gate
  // ------------------------------------------------------------------------------------------
  // SOF Decode, pass_next and Select by TID. A Frame Start opens a frame of TID 0 when no abort
  // is in progress and no skip is armed; TID 1 also needs TID 0 passing. TID 2 and 3 never pass.
  always_comb begin : p_com_frame_gate
    w_sof_0       = i_fbs_valid && i_fbs_sof && (w_axis_in_tid == 2'd0);
    w_sof_1       = i_fbs_valid && i_fbs_sof && (w_axis_in_tid == 2'd1);
    w_pass_nxt[0] = w_sof_0 ? (!w_sink && !reg_skip_arm[0]) : reg_pass[0];
    w_pass_nxt[1] = w_sof_1 ? (!w_sink && !reg_skip_arm[1] && reg_pass[0]) : reg_pass[1];
    w_gate_pass   = !w_axis_in_tid[1] && w_pass_nxt[w_axis_in_tid[0]];
  end

  // Pass bits: set by reset; cleared on abort_start and while axis_enable is 0 (priority over the
  // update on a take); otherwise updated on every take and held between takes.
  always_ff @(posedge i_axis_aclk or negedge i_resetn) begin : p_ff_pass
    if (!i_resetn) begin
      reg_pass <= 2'b11;
    end else if (i_soft_reset) begin
      reg_pass <= 2'b11;
    end else if (w_abort_start || !i_axis_enable) begin
      reg_pass <= '0;
    end else if (w_take) begin
      reg_pass <= w_pass_nxt;
    end
  end

  // Skip requests: a request sets the bit of each TID in the mask, with priority over the clear by
  // a taken Frame Start of that TID.
  always_ff @(posedge i_axis_aclk or negedge i_resetn) begin : p_ff_skip_arm
    if (!i_resetn) begin
      reg_skip_arm <= '0;
    end else if (i_soft_reset) begin
      reg_skip_arm <= '0;
    end else begin
      if (i_asm_skip_req && i_asm_skip_mask[0]) begin
        reg_skip_arm[0] <= 1'b1;
      end else if (w_take && w_sof_0) begin
        reg_skip_arm[0] <= 1'b0;
      end
      if (i_asm_skip_req && i_asm_skip_mask[1]) begin
        reg_skip_arm[1] <= 1'b1;
      end else if (w_take && w_sof_1) begin
        reg_skip_arm[1] <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Bundle Mapping
  // ------------------------------------------------------------------------------------------
  // The beat is forwarded unchanged; it is accepted into the Output or Park Register only when it
  // is taken in PASSTHROUGH with the path enabled and the frame passing.
  always_comb begin : p_com_bundle
    w_axis_in_valid = i_fbs_valid && i_axis_enable && w_gate_pass;
    w_axis_in_data  = i_fbs_data;
    w_axis_in_keep  = i_fbs_byte_en;
    w_axis_in_last  = i_fbs_eol;
    w_axis_in_user  = {i_fbs_err, i_fbs_sof};
    w_accepted      = w_take && w_axis_in_valid && !w_sink;
  end

  // ------------------------------------------------------------------------------------------
  // Finalize Sequencer
  // ------------------------------------------------------------------------------------------
  // Sequence conditions and pulses. A request is folded into the pending mask in every state; an
  // abort beat taken clears the mask bit of its TID. asm_abort_done is asserted on the cycle of
  // the transition that ends the sequence.
  always_comb begin : p_com_fseq_ctrl
    w_sink           = (reg_fsm_fseq_current_state != ENUM_ST_PASSTHROUGH);
    w_drained        = !reg_skid_valid && (!reg_out_valid || i_m_axis_tready);
    w_abort_hs       = (reg_fsm_fseq_current_state == ENUM_ST_INJECT) && i_m_axis_tready;
    w_abort_mask_nxt = (reg_abort_mask | (i_asm_abort_req ? i_asm_abort_mask : 2'b00)) &
                       ~(w_abort_hs ? $bits(w_abort_mask_nxt)'(2'b01 << reg_out_tid) : 2'b00);
    w_line_open_nxt  = w_abort_hs ? 1'b0 : (w_accepted ? !w_axis_in_last : reg_line_open);
    w_todo           = w_line_open_nxt || (w_abort_mask_nxt != 2'b00);
    w_inj_tid        = w_line_open_nxt ? reg_line_tid : (w_abort_mask_nxt[0] ? 2'd0 : 2'd1);
    w_abort_start    = (reg_fsm_fseq_current_state == ENUM_ST_PASSTHROUGH) && i_asm_abort_req;
    w_inject_load    = (((reg_fsm_fseq_current_state == ENUM_ST_DRAIN) && w_drained) ||
                        w_abort_hs) && w_todo;
    w_asm_abort_done = (((reg_fsm_fseq_current_state == ENUM_ST_DRAIN) && w_drained) ||
                        w_abort_hs) && !w_todo;
  end

  // Next state. PASSTHROUGH goes to DRAIN on a request; DRAIN, once out and park are drained, goes
  // to INJECT when a beat must be sent, else back to PASSTHROUGH; INJECT stays while beats remain
  // and returns to PASSTHROUGH after the last one. An unused code returns to PASSTHROUGH.
  always_comb begin : p_com_fseq_nxt
    w_fsm_fseq_nxt_state = reg_fsm_fseq_current_state;
    case (reg_fsm_fseq_current_state)
      ENUM_ST_PASSTHROUGH: begin
        if (i_asm_abort_req) begin
          w_fsm_fseq_nxt_state = ENUM_ST_DRAIN;
        end
      end
      ENUM_ST_DRAIN: begin
        if (w_drained && w_todo) begin
          w_fsm_fseq_nxt_state = ENUM_ST_INJECT;
        end else if (w_drained) begin
          w_fsm_fseq_nxt_state = ENUM_ST_PASSTHROUGH;
        end
      end
      ENUM_ST_INJECT: begin
        if (w_abort_hs && !w_todo) begin
          w_fsm_fseq_nxt_state = ENUM_ST_PASSTHROUGH;
        end
      end
      default: begin
        w_fsm_fseq_nxt_state = ENUM_ST_PASSTHROUGH;
      end
    endcase
  end

  // State register: resetn and soft_reset return to PASSTHROUGH before any transition.
  always_ff @(posedge i_axis_aclk or negedge i_resetn) begin : p_ff_fseq_state
    if (!i_resetn) begin
      reg_fsm_fseq_current_state <= ENUM_ST_PASSTHROUGH;
    end else if (i_soft_reset) begin
      reg_fsm_fseq_current_state <= ENUM_ST_PASSTHROUGH;
    end else begin
      reg_fsm_fseq_current_state <= w_fsm_fseq_nxt_state;
    end
  end

  // Pending abort mask, cleared by resetn and soft_reset.
  always_ff @(posedge i_axis_aclk or negedge i_resetn) begin : p_ff_abort_mask
    if (!i_resetn) begin
      reg_abort_mask <= '0;
    end else if (i_soft_reset) begin
      reg_abort_mask <= '0;
    end else begin
      reg_abort_mask <= w_abort_mask_nxt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Line Tracker
  // ------------------------------------------------------------------------------------------
  // line_open: set by an accepted beat without TLAST, cleared by one with TLAST or by an abort beat
  // taken; cleared by resetn and soft_reset.
  always_ff @(posedge i_axis_aclk or negedge i_resetn) begin : p_ff_line_open
    if (!i_resetn) begin
      reg_line_open <= 1'b0;
    end else if (i_soft_reset) begin
      reg_line_open <= 1'b0;
    end else begin
      reg_line_open <= w_line_open_nxt;
    end
  end

  // line_tid: TID of the last accepted beat. Not reset: it is read only while line_open is set,
  // which needs an accepted beat after reset.
  always_ff @(posedge i_axis_aclk) begin : p_ff_line_tid
    if (w_accepted) begin
      reg_line_tid <= w_axis_in_tid;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Park Register
  // ------------------------------------------------------------------------------------------
  // skid_valid: set when a beat is accepted while the Output Register holds a beat the ISP does
  // not take; cleared when the Output Register frees (the parked beat is promoted).
  always_ff @(posedge i_axis_aclk or negedge i_resetn) begin : p_ff_skid_valid
    if (!i_resetn) begin
      reg_skid_valid <= 1'b0;
    end else if (i_soft_reset) begin
      reg_skid_valid <= 1'b0;
    end else if (w_out_free) begin
      reg_skid_valid <= 1'b0;
    end else if (w_accepted) begin
      reg_skid_valid <= 1'b1;
    end
  end

  // Parked beat. Not reset: it is read only while skid_valid is set.
  always_ff @(posedge i_axis_aclk) begin : p_ff_skid_payload
    if (!w_out_free && w_accepted) begin
      reg_skid_data <= w_axis_in_data;
      reg_skid_keep <= w_axis_in_keep;
      reg_skid_last <= w_axis_in_last;
      reg_skid_user <= w_axis_in_user;
      reg_skid_tid  <= w_axis_in_tid;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output Register / Abort Override
  // ------------------------------------------------------------------------------------------
  // The register is free when empty or when the ISP accepts its beat.
  always_comb begin : p_com_out_free
    w_out_free = !reg_out_valid || i_m_axis_tready;
  end

  // out_valid: when free, loaded with the parked beat, an accepted beat or an abort beat, else
  // cleared; held while the ISP stalls. Cleared by resetn and soft_reset.
  always_ff @(posedge i_axis_aclk or negedge i_resetn) begin : p_ff_out_valid
    if (!i_resetn) begin
      reg_out_valid <= 1'b0;
    end else if (i_soft_reset) begin
      reg_out_valid <= 1'b0;
    end else if (w_out_free) begin
      reg_out_valid <= reg_skid_valid || w_accepted || w_inject_load;
    end
  end

  // Beat on the bus, in priority: parked beat, abort beat (TLAST, no byte kept,
  // TUSER = packet_error, TDATA zero, TID = inj_tid), accepted beat. Not reset: read only while
  // out_valid is set.
  always_ff @(posedge i_axis_aclk) begin : p_ff_out_payload
    if (w_out_free) begin
      if (reg_skid_valid) begin
        reg_out_data <= reg_skid_data;
        reg_out_keep <= reg_skid_keep;
        reg_out_last <= reg_skid_last;
        reg_out_user <= reg_skid_user;
        reg_out_tid  <= reg_skid_tid;
      end else if (w_inject_load) begin
        reg_out_data <= '0;
        reg_out_keep <= '0;
        reg_out_last <= 1'b1;
        reg_out_user <= 2'b10;
        reg_out_tid  <= w_inj_tid;
      end else if (w_accepted) begin
        reg_out_data <= w_axis_in_data;
        reg_out_keep <= w_axis_in_keep;
        reg_out_last <= w_axis_in_last;
        reg_out_user <= w_axis_in_user;
        reg_out_tid  <= w_axis_in_tid;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  // AXI4-Stream outputs straight from the Output Register; fbs_ready from the Ready Logic;
  // asm_abort_done from the Finalize Sequencer on the transition cycle.
  assign o_m_axis_tdata   = reg_out_data;
  assign o_m_axis_tvalid  = reg_out_valid;
  assign o_m_axis_tlast   = reg_out_last;
  assign o_m_axis_tuser   = reg_out_user;
  assign o_m_axis_tid     = reg_out_tid;
  assign o_m_axis_tkeep   = reg_out_keep;
  assign o_fbs_ready      = w_fbs_ready;
  assign o_asm_abort_done = w_asm_abort_done;

endmodule

`default_nettype wire
