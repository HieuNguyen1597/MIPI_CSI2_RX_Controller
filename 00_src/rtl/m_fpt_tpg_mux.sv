// Module      : m_fpt_tpg_mux
// Block       : B06 - TPG Mux
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.3
// Date        : 2026-10-27
// Description : Frame-safe 2:1 source select between the RX pixel path and the TPG
`timescale 1ns/1ps
`default_nettype none

module m_fpt_tpg_mux (
  // Clock and resets
  input  wire         i_sys_clk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  // RX pixel stream from Byte-to-Pixel Unpacking (valid-only, no ready returned)
  input  wire  [47:0] i_rx_px_data,
  input  wire         i_rx_px_valid,
  input  wire  [2:0]  i_rx_px_valid_count,
  input  wire         i_rx_px_sof,
  input  wire         i_rx_px_sol,
  input  wire         i_rx_px_eol,
  input  wire  [1:0]  i_rx_px_vc_tag,
  input  wire         i_rx_px_dt_sel,
  input  wire         i_rx_err_wc_align,
  // RX frame events: Frame Start / Frame End from Packet Analyzer, frame abandon from
  // Control FSM
  input  wire         i_fc_fs,
  input  wire         i_fc_fe,
  input  wire  [1:0]  i_fc_vc,
  input  wire         i_rx_ctx_clear,
  // TPG pixel stream (valid/ready) and frame events
  input  wire  [47:0] i_tpg_px_data,
  input  wire         i_tpg_px_valid,
  input  wire         i_tpg_px_sof,
  input  wire         i_tpg_px_sol,
  input  wire         i_tpg_px_eol,
  input  wire  [1:0]  i_tpg_px_vc_tag,
  input  wire         i_tpg_px_dt_sel,
  input  wire         i_tpg_fs,
  input  wire         i_tpg_eof,
  // Line Buffer levels
  input  wire         i_linebuffer_full,
  input  wire         i_linebuf_empty,
  output logic        o_tpg_px_ready,
  // TPG_CONTROL.tpg_enable, already synchronised into sys_clk
  input  wire         i_tpg_enable,
  // Output pixel stream to Line Buffer (valid-only, no ready input)
  output logic [47:0] o_px_data,
  output logic        o_px_valid,
  output logic [2:0]  o_px_valid_count,
  output logic        o_px_sof,
  output logic        o_px_sol,
  output logic        o_px_eol,
  output logic [1:0]  o_px_vc_tag,
  output logic        o_px_dt_sel,
  output logic        o_err_wc_align,
  // Status
  output logic        o_active_sel,
  output logic        o_switch_pending
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  // Fixed widths of the block (no configuration choice); they size the internal output signals.
  localparam logic [31:0] LPARA_DATA_WIDTH   = 32'd48;
  localparam logic [31:0] LPARA_VC_TAG_WIDTH = 32'd2;
  // Source encoding of active_sel, shared by every multiplexer, the Switch Control FSM and
  // the active_sel port
  localparam logic LPARA_ACTIVE_SEL_RX  = 1'b0;
  localparam logic LPARA_ACTIVE_SEL_TPG = 1'b1;

  // Switch Control FSM states. The code 2'b11 is not used and returns to STREAMING.
  typedef enum logic [1:0] {
    ENUM_ST_STREAMING = 2'b00,
    ENUM_ST_DRAINING  = 2'b01,
    ENUM_ST_ACQUIRING = 2'b10
  } mux_state_type_def;

  // Registers
  mux_state_type_def reg_fsm_mux_current_state;
  logic              reg_active_sel;
  logic              reg_armed;
  logic [1:0]        reg_vc_active;
  logic              reg_tpg_frame_active;

  // Combinational terms
  mux_state_type_def w_fsm_mux_nxt_state;
  logic w_set_0;
  logic w_set_1;
  logic w_clr_0;
  logic w_clr_1;
  logic w_pending_sel;
  logic w_rx_frame_active;
  logic w_rx_frame_start;
  logic w_rx_frame_busy;
  logic w_outgoing_active;
  logic w_outgoing_valid;
  logic w_outgoing_idle;
  logic w_incoming_idle;
  logic w_sel_mismatch;
  logic w_drain_done;
  logic w_write_en;
  logic w_switch_pending;
  logic w_incoming_sof;
  logic w_output_mask;

  // Combinational outputs
  logic                          w_tpg_px_ready;
  logic [LPARA_DATA_WIDTH-1:0]   w_px_data;
  logic                          w_px_valid;
  logic [2:0]                    w_px_valid_count;
  logic                          w_px_sof;
  logic                          w_px_sol;
  logic                          w_px_eol;
  logic [LPARA_VC_TAG_WIDTH-1:0] w_px_vc_tag;
  logic                          w_px_dt_sel;
  logic                          w_err_wc_align;

  // ------------------------------------------------------------------------------------------
  // Frame Activity Tracking
  // ------------------------------------------------------------------------------------------
  // One in-flight flag per source, set by its frame start and cleared by its frame end. RX keeps
  // one flag per Virtual Channel (VC0 Long, VC1 Short); fc_vc = 2 or 3 matches no flag.
  // rx_ctx_clear (Control FSM abandoning an RX frame) clears both RX flags.
  always_comb begin : p_com_vc_flags
    w_set_0 = i_fc_fs && (i_fc_vc == 2'b00);
    w_set_1 = i_fc_fs && (i_fc_vc == 2'b01);
    w_clr_0 = i_rx_ctx_clear || (i_fc_fe && (i_fc_vc == 2'b00));
    w_clr_1 = i_rx_ctx_clear || (i_fc_fe && (i_fc_vc == 2'b01));
  end

  // set_v wins over clr_v of the same VC and tpg_fs wins over tpg_eof. tpg_fs comes with the
  // accepted first beat, so a TPG beat held by tpg_px_ready = 0 does not open the TPG frame.
  // resetn (asynchronous) and soft_reset (synchronous) clear both flags before any update.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_frame_flags
    if (!i_resetn) begin
      reg_vc_active        <= '0;
      reg_tpg_frame_active <= 1'b0;
    end else if (i_soft_reset) begin
      reg_vc_active        <= '0;
      reg_tpg_frame_active <= 1'b0;
    end else begin
      if (w_set_0) begin
        reg_vc_active[0] <= 1'b1;
      end else if (w_clr_0) begin
        reg_vc_active[0] <= 1'b0;
      end
      if (w_set_1) begin
        reg_vc_active[1] <= 1'b1;
      end else if (w_clr_1) begin
        reg_vc_active[1] <= 1'b0;
      end
      if (i_tpg_fs) begin
        reg_tpg_frame_active <= 1'b1;
      end else if (i_tpg_eof) begin
        reg_tpg_frame_active <= 1'b0;
      end
    end
  end

  // pending_sel is tpg_enable as delivered. RX activity also counts a VC0/VC1 Frame Start of this
  // cycle (rx_frame_start), so a switch never takes effect on the edge of a Frame Start. The
  // outgoing source is idle only with no frame open and no beat presented; the incoming source
  // must have no frame open, so its first delivered beat is a frame start.
  always_comb begin : p_com_activity
    w_pending_sel     = i_tpg_enable;
    w_rx_frame_active = |reg_vc_active;
    w_rx_frame_start  = w_set_0 || w_set_1;
    w_rx_frame_busy   = w_rx_frame_active || w_rx_frame_start;
    if (reg_active_sel == LPARA_ACTIVE_SEL_TPG) begin
      w_outgoing_active = reg_tpg_frame_active;
      w_outgoing_valid  = i_tpg_px_valid;
    end else begin
      w_outgoing_active = w_rx_frame_busy;
      w_outgoing_valid  = i_rx_px_valid;
    end
    w_outgoing_idle = !w_outgoing_active && !w_outgoing_valid;
    if (w_pending_sel == LPARA_ACTIVE_SEL_TPG) begin
      w_incoming_idle = !reg_tpg_frame_active;
    end else begin
      w_incoming_idle = !w_rx_frame_busy;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Switch Control FSM
  // ------------------------------------------------------------------------------------------
  // drain_done needs both sources idle and, when the TPG is outgoing, an empty Line Buffer.
  // write_en pulses on drain_done in DRAINING; if pending_sel has returned to active_sel the
  // switch is a no-op. With no source streaming, active_sel changes on the second edge after
  // tpg_enable rises. switch_pending is 1 in every state but STREAMING.
  always_comb begin : p_com_switch_ctrl
    w_sel_mismatch   = (w_pending_sel != reg_active_sel);
    w_drain_done     = w_outgoing_idle && w_incoming_idle &&
                       ((reg_active_sel == LPARA_ACTIVE_SEL_RX) || i_linebuf_empty);
    w_write_en       = (reg_fsm_mux_current_state == ENUM_ST_DRAINING) && w_drain_done;
    w_switch_pending = (reg_fsm_mux_current_state != ENUM_ST_STREAMING);
  end

  // STREAMING -> DRAINING on a mismatch. DRAINING -> ACQUIRING on drain_done, with no timeout:
  // an RX frame that never closes waits for rx_ctx_clear. In ACQUIRING the first Start of Frame
  // of the new source (-> STREAMING) wins over a new mismatch (-> DRAINING). The unused code
  // 2'b11 returns to STREAMING.
  always_comb begin : p_com_fsm_mux_nxt
    w_fsm_mux_nxt_state = reg_fsm_mux_current_state;
    case (reg_fsm_mux_current_state)
      ENUM_ST_STREAMING: begin
        if (w_sel_mismatch) begin
          w_fsm_mux_nxt_state = ENUM_ST_DRAINING;
        end
      end
      ENUM_ST_DRAINING: begin
        if (w_drain_done) begin
          w_fsm_mux_nxt_state = ENUM_ST_ACQUIRING;
        end
      end
      ENUM_ST_ACQUIRING: begin
        if (w_incoming_sof) begin
          w_fsm_mux_nxt_state = ENUM_ST_STREAMING;
        end else if (w_sel_mismatch) begin
          w_fsm_mux_nxt_state = ENUM_ST_DRAINING;
        end
      end
      default: begin
        w_fsm_mux_nxt_state = ENUM_ST_STREAMING;
      end
    endcase
  end

  // State and active_sel registers. Both resets return to STREAMING with the RX path selected;
  // active_sel is written only by write_en.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_fsm_mux
    if (!i_resetn) begin
      reg_fsm_mux_current_state <= ENUM_ST_STREAMING;
      reg_active_sel            <= LPARA_ACTIVE_SEL_RX;
    end else if (i_soft_reset) begin
      reg_fsm_mux_current_state <= ENUM_ST_STREAMING;
      reg_active_sel            <= LPARA_ACTIVE_SEL_RX;
    end else begin
      reg_fsm_mux_current_state <= w_fsm_mux_nxt_state;
      if (w_write_en) begin
        reg_active_sel <= w_pending_sel;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // armed / output_mask
  // ------------------------------------------------------------------------------------------
  // incoming_sof is the Start of Frame of the selected source, taken from its valid before the
  // mask (on the TPG path the beat need not be accepted yet). output_mask clears on that beat, so
  // it is the first beat delivered; with sources that honour their contracts no beat is removed.
  always_comb begin : p_com_mask
    if (reg_active_sel == LPARA_ACTIVE_SEL_TPG) begin
      w_incoming_sof = i_tpg_px_valid && i_tpg_px_sof;
    end else begin
      w_incoming_sof = i_rx_px_valid && i_rx_px_sof;
    end
    w_output_mask = reg_armed && !w_incoming_sof;
  end

  // armed is set by every write_en (a no-op switch too) and cleared by incoming_sof. Being a flag
  // rather than an FSM decode, it keeps the output masked across ACQUIRING -> DRAINING. Both
  // resets clear it.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_armed
    if (!i_resetn) begin
      reg_armed <= 1'b0;
    end else if (i_soft_reset) begin
      reg_armed <= 1'b0;
    end else if (w_write_en) begin
      reg_armed <= 1'b1;
    end else if (w_incoming_sof) begin
      reg_armed <= 1'b0;
    end
  end

  // ------------------------------------------------------------------------------------------
  // TPG ready and Output Mux
  // ------------------------------------------------------------------------------------------
  // Zero latency: outputs are combinational, including linebuffer_full -> tpg_px_ready -> px_valid.
  // TPG selected: tpg_px_ready follows Line Buffer, a beat is presented only when accepted, and
  // px_valid_count = 3'b100, err_wc_align = 0. RX selected: the TPG is held while a switch to it
  // is pending and no TPG frame is open. output_mask gates px_valid and err_wc_align only.
  always_comb begin : p_com_output_mux
    if (reg_active_sel == LPARA_ACTIVE_SEL_TPG) begin
      w_tpg_px_ready   = !i_linebuffer_full;
      w_px_data        = i_tpg_px_data;
      w_px_valid       = i_tpg_px_valid && w_tpg_px_ready && !w_output_mask;
      w_px_valid_count = 3'b100;
      w_px_sof         = i_tpg_px_sof;
      w_px_sol         = i_tpg_px_sol;
      w_px_eol         = i_tpg_px_eol;
      w_px_vc_tag      = i_tpg_px_vc_tag;
      w_px_dt_sel      = i_tpg_px_dt_sel;
      w_err_wc_align   = 1'b0;
    end else begin
      w_tpg_px_ready   = !w_pending_sel || reg_tpg_frame_active;
      w_px_data        = i_rx_px_data;
      w_px_valid       = i_rx_px_valid && !w_output_mask;
      w_px_valid_count = i_rx_px_valid_count;
      w_px_sof         = i_rx_px_sof;
      w_px_sol         = i_rx_px_sol;
      w_px_eol         = i_rx_px_eol;
      w_px_vc_tag      = i_rx_px_vc_tag;
      w_px_dt_sel      = i_rx_px_dt_sel;
      w_err_wc_align   = i_rx_err_wc_align && !w_output_mask;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  // Combinational outputs come from the w_* terms above; active_sel is the register itself.
  assign o_tpg_px_ready   = w_tpg_px_ready;
  assign o_px_data        = w_px_data;
  assign o_px_valid       = w_px_valid;
  assign o_px_valid_count = w_px_valid_count;
  assign o_px_sof         = w_px_sof;
  assign o_px_sol         = w_px_sol;
  assign o_px_eol         = w_px_eol;
  assign o_px_vc_tag      = w_px_vc_tag;
  assign o_px_dt_sel      = w_px_dt_sel;
  assign o_err_wc_align   = w_err_wc_align;
  assign o_active_sel     = reg_active_sel;
  assign o_switch_pending = w_switch_pending;

endmodule

`default_nettype wire
