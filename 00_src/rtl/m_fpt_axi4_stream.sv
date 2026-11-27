// Module      : m_fpt_axi4_stream
// Block       : B12 - AXI4-Stream Master
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.5
// Date        : 2026-11-26
// Description : 16-bit AXI4-Stream output to the ISP clock domain with frame gate and abort entries
`timescale 1ns/1ps
`default_nettype none

module m_fpt_axi4_stream #(
  // Parameter Description: synchroniser flops per clock-domain crossing (FIFO pointers, flush
  // request and acknowledge); the single top-level parameter.
  // Parameter values: at least 2; default 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_SYNC_STAGES = 32'd2,
  // Parameter Description: entries of the stream FIFO, each one 64-bit beat with its sideband.
  // Parameter values: power of two from 4 to 1024; default 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_AXIS_FIFO_DEPTH = 32'd16
) (
  input  wire         i_sys_clk,
  input  wire         i_axis_aclk,
  input  wire         i_resetn,
  input  wire         i_axis_resetn,
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
  // AMBA AXI4-Stream (IHI0051B) transmitter towards the ISP, ACLK = axis_aclk. Optional signals
  // present: TKEEP, TLAST, TID, TUSER; not implemented: TSTRB, TDEST, TWAKEUP.
  output logic [15:0] o_m_axis_tdata,
  output logic        o_m_axis_tvalid,
  input  wire         i_m_axis_tready,
  output logic        o_m_axis_tlast,
  output logic [1:0]  o_m_axis_tuser,
  output logic [1:0]  o_m_axis_tid,
  output logic [1:0]  o_m_axis_tkeep
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_TDATA_WIDTH    = 32'd16;
  localparam logic [31:0] LPARA_TKEEP_WIDTH    = LPARA_TDATA_WIDTH / 32'd8;
  localparam logic [31:0] LPARA_TUSER_WIDTH    = 32'd2;
  localparam logic [31:0] LPARA_TID_WIDTH      = 32'd2;
  localparam logic [31:0] LPARA_FBS_DATA_WIDTH = 32'd64;
  localparam logic [31:0] LPARA_FBS_KEEP_WIDTH = LPARA_FBS_DATA_WIDTH / 32'd8;
  localparam logic [31:0] LPARA_XFER_NUM       = LPARA_FBS_DATA_WIDTH / LPARA_TDATA_WIDTH;
  localparam logic [31:0] LPARA_XFER_IDX_W     = $clog2(LPARA_XFER_NUM);
  localparam logic [31:0] LPARA_ENTRY_W        = LPARA_FBS_DATA_WIDTH + LPARA_FBS_KEEP_WIDTH
                                                 + 32'd1 + LPARA_TUSER_WIDTH + LPARA_TID_WIDTH;
  // axis_aclk cycles the flush request is seen before the acknowledge may return.
  localparam logic [31:0] LPARA_FLUSH_DWELL    = 32'd2;

  generate
    if ((PARA_SYNC_STAGES < 32'd2) || (PARA_AXIS_FIFO_DEPTH < 32'd4)
        || (PARA_AXIS_FIFO_DEPTH > 32'd1024)
        || ((PARA_AXIS_FIFO_DEPTH & (PARA_AXIS_FIFO_DEPTH - 32'd1)) != 32'd0))
    begin : gen_para_check
      $error("m_fpt_axi4_stream: unsupported PARA_SYNC_STAGES or PARA_AXIS_FIFO_DEPTH");
    end
  endgenerate

  // Finalize Sequencer states.
  typedef enum logic [0:0] {
    ENUM_ST_PASSTHROUGH = 1'b0,
    ENUM_ST_INJECT      = 1'b1
  } fseq_state_type_def;

  fseq_state_type_def reg_fsm_fseq_current_state;
  fseq_state_type_def w_fsm_fseq_nxt_state;

  // sys_clk side
  logic                            w_axis_in_valid;
  logic [LPARA_FBS_DATA_WIDTH-1:0] w_axis_in_data;
  logic [LPARA_FBS_KEEP_WIDTH-1:0] w_axis_in_keep;
  logic                            w_axis_in_last;
  logic [LPARA_TUSER_WIDTH-1:0]    w_axis_in_user;
  logic [LPARA_TID_WIDTH-1:0]      w_axis_in_tid;
  logic                            w_sof_0;
  logic                            w_sof_1;
  logic [1:0]                      w_pass_nxt;
  logic                            w_gate_pass;
  logic [1:0]                      reg_pass;
  logic [1:0]                      reg_skip_arm;
  logic                            w_fbs_ready;
  logic                            w_take;
  logic                            w_accepted;
  logic                            w_sink;
  logic                            w_abort_start;
  logic                            w_abort_wr;
  logic [1:0]                      w_abort_mask_nxt;
  logic                            w_line_open_nxt;
  logic                            w_todo;
  logic [LPARA_TID_WIDTH-1:0]      w_inj_tid;
  logic                            w_asm_abort_done;
  logic [1:0]                      reg_abort_mask;
  logic                            reg_line_open;
  logic [LPARA_TID_WIDTH-1:0]      reg_line_tid;
  logic                            w_fifo_wr;
  logic [LPARA_ENTRY_W-1:0]        w_fifo_din;
  logic [LPARA_ENTRY_W-1:0]        w_abort_entry;
  logic                            w_fifo_full;
  logic                            reg_flush_req;
  logic                            reg_flush_ack_sync [PARA_SYNC_STAGES];
  logic                            w_flush_pending;

  // axis_aclk side
  logic                            reg_flush_req_sync [PARA_SYNC_STAGES];
  logic                            reg_flush_ack;
  logic [1:0]                      reg_flush_dwell;
  logic                            w_flushing;
  logic [LPARA_ENTRY_W-1:0]        w_fifo_dout;
  logic                            w_fifo_empty;
  logic                            w_fifo_pop;
  logic [LPARA_FBS_DATA_WIDTH-1:0] w_hd_data;
  logic [LPARA_FBS_KEEP_WIDTH-1:0] w_hd_keep;
  logic                            w_hd_last;
  logic [LPARA_TUSER_WIDTH-1:0]    w_hd_user;
  logic [LPARA_TID_WIDTH-1:0]      w_hd_tid;
  logic [LPARA_XFER_IDX_W-1:0]     reg_conv_idx;
  logic [LPARA_XFER_IDX_W-1:0]     w_conv_k;
  logic                            w_conv_valid;
  logic                            w_conv_hand;
  logic                            w_conv_last;
  logic [LPARA_TDATA_WIDTH-1:0]    w_conv_data;
  logic [LPARA_TKEEP_WIDTH-1:0]    w_conv_keep;
  logic                            w_conv_tlast;
  logic [LPARA_TUSER_WIDTH-1:0]    w_conv_user;
  logic [LPARA_TID_WIDTH-1:0]      w_conv_tid;
  logic                            w_out_free;
  logic                            reg_out_valid;
  logic [LPARA_TDATA_WIDTH-1:0]    reg_out_data;
  logic [LPARA_TKEEP_WIDTH-1:0]    reg_out_keep;
  logic                            reg_out_last;
  logic [LPARA_TUSER_WIDTH-1:0]    reg_out_user;
  logic [LPARA_TID_WIDTH-1:0]      reg_out_tid;
  logic                            reg_skid_valid;
  logic [LPARA_TDATA_WIDTH-1:0]    reg_skid_data;
  logic [LPARA_TKEEP_WIDTH-1:0]    reg_skid_keep;
  logic                            reg_skid_last;
  logic [LPARA_TUSER_WIDTH-1:0]    reg_skid_user;
  logic [LPARA_TID_WIDTH-1:0]      reg_skid_tid;

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
  // fbs_ready comes from sys_clk registers only: 1 while an abort sequence sinks the input,
  // otherwise low while the stream FIFO is full or a soft_reset flush is pending.
  always_comb begin : p_com_ready
    w_flush_pending = (reg_flush_req != reg_flush_ack_sync[PARA_SYNC_STAGES-1]);
    w_fbs_ready     = w_sink || !(w_fifo_full || w_flush_pending);
    w_take          = i_fbs_valid && w_fbs_ready;
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
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_pass
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
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_skip_arm
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
  // The beat is kept unchanged; it is accepted when it is taken in PASSTHROUGH with the path
  // enabled and the frame passing.
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
  // abort entry written clears the mask bit of its TID and the open line. asm_abort_done is
  // asserted on the cycle that writes the last abort entry, or on abort_start with nothing to
  // close.
  always_comb begin : p_com_fseq_ctrl
    w_sink           = (reg_fsm_fseq_current_state == ENUM_ST_INJECT);
    w_abort_start    = (reg_fsm_fseq_current_state == ENUM_ST_PASSTHROUGH) && i_asm_abort_req;
    w_abort_wr       = w_sink && !w_fifo_full && !w_flush_pending;
    w_inj_tid        = reg_line_open ? reg_line_tid : (reg_abort_mask[0] ? 2'd0 : 2'd1);
    w_abort_mask_nxt = (reg_abort_mask | (i_asm_abort_req ? i_asm_abort_mask : 2'b00)) &
                       ~(w_abort_wr ? $bits(w_abort_mask_nxt)'(2'b01 << w_inj_tid) : 2'b00);
    w_line_open_nxt  = w_abort_wr ? 1'b0 : (w_accepted ? !w_axis_in_last : reg_line_open);
    w_todo           = w_line_open_nxt || (w_abort_mask_nxt != 2'b00);
    w_asm_abort_done = (w_abort_start || w_abort_wr) && !w_todo;
  end

  // Next state. PASSTHROUGH goes to INJECT on a request with something to close; INJECT returns
  // to PASSTHROUGH on the cycle that writes the last abort entry. An unused code returns to
  // PASSTHROUGH.
  always_comb begin : p_com_fseq_nxt
    w_fsm_fseq_nxt_state = reg_fsm_fseq_current_state;
    case (reg_fsm_fseq_current_state)
      ENUM_ST_PASSTHROUGH: begin
        if (w_abort_start && w_todo) begin
          w_fsm_fseq_nxt_state = ENUM_ST_INJECT;
        end
      end
      ENUM_ST_INJECT: begin
        if (w_abort_wr && !w_todo) begin
          w_fsm_fseq_nxt_state = ENUM_ST_PASSTHROUGH;
        end
      end
      default: begin
        w_fsm_fseq_nxt_state = ENUM_ST_PASSTHROUGH;
      end
    endcase
  end

  // State register: resetn and soft_reset return to PASSTHROUGH before any transition.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_fseq_state
    if (!i_resetn) begin
      reg_fsm_fseq_current_state <= ENUM_ST_PASSTHROUGH;
    end else if (i_soft_reset) begin
      reg_fsm_fseq_current_state <= ENUM_ST_PASSTHROUGH;
    end else begin
      reg_fsm_fseq_current_state <= w_fsm_fseq_nxt_state;
    end
  end

  // Pending abort mask, cleared by resetn and soft_reset.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_abort_mask
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
  // line_open: set by an accepted beat without TLAST, cleared by one with TLAST or by an abort
  // entry written; cleared by resetn and soft_reset.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_line_open
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
  always_ff @(posedge i_sys_clk) begin : p_ff_line_tid
    if (w_accepted) begin
      reg_line_tid <= w_axis_in_tid;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Abort Override
  // ------------------------------------------------------------------------------------------
  // Entry written into the stream FIFO: the accepted beat, or in INJECT the abort entry (no byte
  // kept, TLAST, TUSER = packet_error, TDATA zero, TID = inj_tid). Nothing is written on a cycle
  // with soft_reset.
  always_comb begin : p_com_fifo_wr
    w_abort_entry = {{LPARA_FBS_DATA_WIDTH{1'b0}}, {LPARA_FBS_KEEP_WIDTH{1'b0}}, 1'b1, 2'b10,
                     w_inj_tid};
    w_fifo_din    = w_abort_wr ? w_abort_entry
                               : {w_axis_in_data, w_axis_in_keep, w_axis_in_last, w_axis_in_user,
                                  w_axis_in_tid};
    w_fifo_wr     = !i_soft_reset && (w_accepted || w_abort_wr);
  end

  // ------------------------------------------------------------------------------------------
  // Flush Handshake
  // ------------------------------------------------------------------------------------------
  // Flush request (sys_clk): toggles on a soft_reset cycle that finds no flush pending; reset by
  // resetn only, so a flush in progress survives a longer soft_reset.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_flush_req
    if (!i_resetn) begin
      reg_flush_req <= 1'b0;
    end else if (i_soft_reset && !w_flush_pending) begin
      reg_flush_req <= !reg_flush_req;
    end
  end

  // Flush acknowledge brought into sys_clk through PARA_SYNC_STAGES flops, no logic in between.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_flush_ack_sync
    if (!i_resetn) begin
      for (int idx = 0; idx < PARA_SYNC_STAGES; idx++) reg_flush_ack_sync[idx] <= 1'b0;
    end else begin
      reg_flush_ack_sync[0] <= reg_flush_ack;
      for (int idx = 1; idx < PARA_SYNC_STAGES; idx++) begin
        reg_flush_ack_sync[idx] <= reg_flush_ack_sync[idx-1];
      end
    end
  end

  // Flush request brought into axis_aclk through PARA_SYNC_STAGES flops, no logic in between.
  always_ff @(posedge i_axis_aclk or negedge i_axis_resetn) begin : p_ff_flush_req_sync
    if (!i_axis_resetn) begin
      for (int idx = 0; idx < PARA_SYNC_STAGES; idx++) reg_flush_req_sync[idx] <= 1'b0;
    end else begin
      reg_flush_req_sync[0] <= reg_flush_req;
      for (int idx = 1; idx < PARA_SYNC_STAGES; idx++) begin
        reg_flush_req_sync[idx] <= reg_flush_req_sync[idx-1];
      end
    end
  end

  // axis_aclk side of the flush: while the request differs from the acknowledge, the FIFO is read
  // until empty and the output stage is cleared; the acknowledge toggles once the request has been
  // seen for LPARA_FLUSH_DWELL cycles and the FIFO is empty, so every entry written before the
  // soft_reset has been discarded.
  always_comb begin : p_com_flush
    w_flushing = (reg_flush_req_sync[PARA_SYNC_STAGES-1] != reg_flush_ack);
  end

  always_ff @(posedge i_axis_aclk or negedge i_axis_resetn) begin : p_ff_flush_ack
    if (!i_axis_resetn) begin
      reg_flush_ack   <= 1'b0;
      reg_flush_dwell <= '0;
    end else if (!w_flushing) begin
      reg_flush_dwell <= '0;
    end else if ((32'(reg_flush_dwell) >= LPARA_FLUSH_DWELL) && w_fifo_empty) begin
      reg_flush_ack   <= reg_flush_req_sync[PARA_SYNC_STAGES-1];
      reg_flush_dwell <= '0;
    end else if (32'(reg_flush_dwell) < LPARA_FLUSH_DWELL) begin
      reg_flush_dwell <= reg_flush_dwell + 2'd1;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Stream FIFO
  // ------------------------------------------------------------------------------------------
  // Asynchronous first-word-fall-through FIFO with Gray pointers from sys_clk to axis_aclk; each
  // side is reset by the reset of its own domain. The write-side occupancy count is not needed.
  m_fpt_dma_async_fifo #(
    .PARA_DEPTH       (PARA_AXIS_FIFO_DEPTH),
    .PARA_WIDTH       (LPARA_ENTRY_W),
    .PARA_ASYNC       (32'd1),
    .PARA_SYNC_STAGES (PARA_SYNC_STAGES)
  ) u_stream_fifo_0 (
    .i_wr_clk   (i_sys_clk),
    .i_wr_rst_n (i_resetn),
    .i_push     (w_fifo_wr),
    .i_din      (w_fifo_din),
    .o_full     (w_fifo_full),
    .o_wr_used  (),
    .i_rd_clk   (i_axis_aclk),
    .i_rd_rst_n (i_axis_resetn),
    .i_pop      (w_fifo_pop),
    .o_dout     (w_fifo_dout),
    .o_empty    (w_fifo_empty)
  );

  // ------------------------------------------------------------------------------------------
  // Width Converter
  // ------------------------------------------------------------------------------------------
  // The head entry is sent as 16-bit transfers in ascending byte order, skipping empty byte pairs.
  // The first transfer carries start_of_frame, the last one TLAST and packet_error; an abort entry
  // (no byte kept) gives one null transfer. Transfers are handed over only while the Park Register
  // is empty, so TREADY does not reach the FIFO read.
  always_comb begin : p_com_conv
    logic found;
    {w_hd_data, w_hd_keep, w_hd_last, w_hd_user, w_hd_tid} = w_fifo_dout;
    w_conv_k    = reg_conv_idx;
    found       = 1'b0;
    for (int idx = 0; idx < LPARA_XFER_NUM; idx++) begin
      if (!found && (idx >= int'(reg_conv_idx)) && (w_hd_keep[2*idx +: 2] != 2'b00)) begin
        w_conv_k = LPARA_XFER_IDX_W'(idx);
        found    = 1'b1;
      end
    end
    w_conv_last = 1'b1;
    for (int idx = 0; idx < LPARA_XFER_NUM; idx++) begin
      if ((idx > int'(w_conv_k)) && (w_hd_keep[2*idx +: 2] != 2'b00)) begin
        w_conv_last = 1'b0;
      end
    end
    w_conv_data = '0;
    w_conv_keep = '0;
    for (int idx = 0; idx < LPARA_XFER_NUM; idx++) begin
      if (idx == int'(w_conv_k)) begin
        w_conv_data = w_hd_data[LPARA_TDATA_WIDTH*idx +: LPARA_TDATA_WIDTH];
        w_conv_keep = w_hd_keep[LPARA_TKEEP_WIDTH*idx +: LPARA_TKEEP_WIDTH];
      end
    end
    w_conv_tlast = w_hd_last && w_conv_last;
    w_conv_user  = {w_hd_user[1] && w_conv_last, w_hd_user[0] && (reg_conv_idx == '0)};
    w_conv_tid   = w_hd_tid;
    w_conv_valid = !w_fifo_empty && !w_flushing;
    w_conv_hand  = w_conv_valid && !reg_skid_valid;
    w_fifo_pop   = (w_conv_hand && w_conv_last) || (w_flushing && !w_fifo_empty);
  end

  // Index of the next transfer of the head entry: back to 0 after the last transfer and on a flush.
  always_ff @(posedge i_axis_aclk or negedge i_axis_resetn) begin : p_ff_conv_idx
    if (!i_axis_resetn) begin
      reg_conv_idx <= '0;
    end else if (w_flushing) begin
      reg_conv_idx <= '0;
    end else if (w_conv_hand) begin
      reg_conv_idx <= w_conv_last ? '0 : w_conv_k + LPARA_XFER_IDX_W'(1);
    end
  end

  // ------------------------------------------------------------------------------------------
  // Park Register
  // ------------------------------------------------------------------------------------------
  // skid_valid: set when a transfer is handed over while the Output Register holds one the ISP
  // does not take; cleared when the Output Register frees and by a flush.
  always_ff @(posedge i_axis_aclk or negedge i_axis_resetn) begin : p_ff_skid_valid
    if (!i_axis_resetn) begin
      reg_skid_valid <= 1'b0;
    end else if (w_flushing || w_out_free) begin
      reg_skid_valid <= 1'b0;
    end else if (w_conv_hand) begin
      reg_skid_valid <= 1'b1;
    end
  end

  // Parked transfer. Not reset: it is read only while skid_valid is set.
  always_ff @(posedge i_axis_aclk) begin : p_ff_skid_payload
    if (!w_out_free && w_conv_hand) begin
      reg_skid_data <= w_conv_data;
      reg_skid_keep <= w_conv_keep;
      reg_skid_last <= w_conv_tlast;
      reg_skid_user <= w_conv_user;
      reg_skid_tid  <= w_conv_tid;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output Register
  // ------------------------------------------------------------------------------------------
  // The register is free when empty or when the ISP accepts its transfer.
  always_comb begin : p_com_out_free
    w_out_free = !reg_out_valid || i_m_axis_tready;
  end

  // out_valid: when free, loaded with the parked or the handed-over transfer, else cleared; held
  // while the ISP stalls. Cleared by axis_resetn and by a soft_reset flush.
  always_ff @(posedge i_axis_aclk or negedge i_axis_resetn) begin : p_ff_out_valid
    if (!i_axis_resetn) begin
      reg_out_valid <= 1'b0;
    end else if (w_flushing) begin
      reg_out_valid <= 1'b0;
    end else if (w_out_free) begin
      reg_out_valid <= reg_skid_valid || w_conv_hand;
    end
  end

  // Transfer on the bus: the parked one first, else the one handed over. Not reset: read only while
  // out_valid is set.
  always_ff @(posedge i_axis_aclk) begin : p_ff_out_payload
    if (w_out_free) begin
      if (reg_skid_valid) begin
        reg_out_data <= reg_skid_data;
        reg_out_keep <= reg_skid_keep;
        reg_out_last <= reg_skid_last;
        reg_out_user <= reg_skid_user;
        reg_out_tid  <= reg_skid_tid;
      end else if (w_conv_hand) begin
        reg_out_data <= w_conv_data;
        reg_out_keep <= w_conv_keep;
        reg_out_last <= w_conv_tlast;
        reg_out_user <= w_conv_user;
        reg_out_tid  <= w_conv_tid;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  // AXI4-Stream outputs straight from the Output Register; fbs_ready from the Ready Logic;
  // asm_abort_done from the Finalize Sequencer.
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
