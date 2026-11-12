// Module      : m_fpt_format_bridge
// Block       : B08 - Format Bridge
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.3
// Date        : 2026-11-07
// Description : Bypass and RAW16 formatters, stream and DMA output FIFOs with fb_ready handshake
`timescale 1ns/1ps
`default_nettype none

module m_fpt_format_bridge #(
  // Parameter Description: beats of the stream output FIFO; almost_full with one free entry.
  // Parameter values: power of two from 2 to 1024; default is 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_STREAM_FIFO_DEPTH = 32'd16,
  // Parameter Description: beats of the DMA output FIFO; almost_full with one free entry.
  // Parameter values: power of two from 2 to 1024; default is 8.
  // Parameter unit: count
  parameter logic [31:0] PARA_DMA_FIFO_DEPTH    = 32'd8
) (
  input  wire         i_sys_clk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  input  wire         i_fb_flush,
  input  wire  [47:0] i_fb_px_data,
  input  wire         i_fb_px_valid,
  input  wire  [2:0]  i_fb_px_valid_count,
  input  wire         i_fb_px_sof,
  input  wire         i_fb_px_sol,
  input  wire         i_fb_px_eol,
  input  wire  [1:0]  i_fb_px_vc_tag,
  input  wire         i_fb_px_dt_sel,
  input  wire         i_fb_err_wc_align,
  output logic        o_fb_ready,
  input  wire         i_stream_format,
  input  wire         i_dma_format,
  input  wire         i_axis_enable,
  input  wire         i_dma_enable,
  output logic [63:0] o_fbs_data,
  output logic        o_fbs_valid,
  output logic [7:0]  o_fbs_byte_en,
  output logic        o_fbs_sof,
  output logic        o_fbs_eol,
  output logic [1:0]  o_fbs_vc_tag,
  output logic        o_fbs_err,
  input  wire         i_fbs_ready,
  output logic [63:0] o_fbd_data,
  output logic        o_fbd_valid,
  output logic [7:0]  o_fbd_byte_en,
  output logic        o_fbd_sof,
  output logic        o_fbd_sol,
  output logic        o_fbd_eol,
  output logic [1:0]  o_fbd_vc_tag,
  output logic        o_fbd_err,
  input  wire         i_fbd_ready,
  output logic        o_fb_overflow,
  output logic        o_stream_fifo_almost_full,
  output logic        o_dma_fifo_almost_full
);

  // ------------------------------------------------------------------------------------------
  // Constants
  // ------------------------------------------------------------------------------------------
  // Beat width, largest residuals of the two formatters (7 and 4 bytes).
  localparam logic [31:0] LPARA_DATA_OUT_WIDTH       = 32'd64;
  localparam logic [31:0] LPARA_REPACK_BUF_WIDTH     = 32'd56;
  localparam logic [31:0] LPARA_RAW16_RESIDUAL_WIDTH = 32'd32;
  localparam logic [31:0] LPARA_BYTE_EN_WIDTH        = LPARA_DATA_OUT_WIDTH / 32'd8;
  // Largest single-cycle bypass intake: 6 bytes (RAW12, 4 pixels).
  localparam logic [31:0] LPARA_PACKED_WIDTH         = 32'd48;
  // Combining buses: residual plus intake (13 bytes bypass, 12 bytes RAW16).
  localparam logic [31:0] LPARA_BUF_NEXT_WIDTH       = LPARA_REPACK_BUF_WIDTH + LPARA_PACKED_WIDTH;
  localparam logic [31:0] LPARA_R16_BUF_NEXT_WIDTH   = LPARA_RAW16_RESIDUAL_WIDTH +
                                                       LPARA_DATA_OUT_WIDTH;
  // Byte counters: repack_cnt/total 0..13, r16_cnt 0..4, r16_total 0..12.
  localparam logic [31:0] LPARA_REPACK_CNT_WIDTH     = $clog2(LPARA_BUF_NEXT_WIDTH / 32'd8 + 32'd1);
  localparam logic [31:0] LPARA_R16_CNT_WIDTH        = $clog2(LPARA_RAW16_RESIDUAL_WIDTH / 32'd8 +
                                                              32'd1);
  localparam logic [31:0] LPARA_R16_TOTAL_WIDTH      = $clog2(LPARA_R16_BUF_NEXT_WIDTH / 32'd8 +
                                                              32'd1);
  // FIFO entries: data, byte_en, sof, eol, vc_tag[1:0], err; the DMA entry adds sol.
  localparam logic [31:0] LPARA_STREAM_ENTRY_WIDTH   = LPARA_DATA_OUT_WIDTH + LPARA_BYTE_EN_WIDTH +
                                                       32'd5;
  localparam logic [31:0] LPARA_DMA_ENTRY_WIDTH      = LPARA_DATA_OUT_WIDTH + LPARA_BYTE_EN_WIDTH +
                                                       32'd6;
  // Field positions inside an entry: byte_en above data, then sof, eol, vc_tag, err, sol.
  localparam logic [31:0] LPARA_SOF_BIT              = LPARA_DATA_OUT_WIDTH + LPARA_BYTE_EN_WIDTH;
  localparam logic [31:0] LPARA_EOL_BIT              = LPARA_SOF_BIT + 32'd1;
  localparam logic [31:0] LPARA_VC_TAG_LSB           = LPARA_SOF_BIT + 32'd2;
  localparam logic [31:0] LPARA_ERR_BIT              = LPARA_SOF_BIT + 32'd4;
  localparam logic [31:0] LPARA_SOL_BIT              = LPARA_SOF_BIT + 32'd5;
  // FIFO pointers wrap at the depth; counts hold 0..depth inclusive.
  localparam logic [31:0] LPARA_STREAM_PTR_WIDTH     = $clog2(PARA_STREAM_FIFO_DEPTH);
  localparam logic [31:0] LPARA_STREAM_CNT_WIDTH     = $clog2(PARA_STREAM_FIFO_DEPTH + 32'd1);
  localparam logic [31:0] LPARA_DMA_PTR_WIDTH        = $clog2(PARA_DMA_FIFO_DEPTH);
  localparam logic [31:0] LPARA_DMA_CNT_WIDTH        = $clog2(PARA_DMA_FIFO_DEPTH + 32'd1);
  // Supported FIFO depths.
  localparam logic [31:0] LPARA_FIFO_DEPTH_MIN       = 32'd2;
  localparam logic [31:0] LPARA_FIFO_DEPTH_MAX       = 32'd1024;

  // ------------------------------------------------------------------------------------------
  // Parameter check
  // ------------------------------------------------------------------------------------------
  // An unsupported depth stops elaboration with an error. Depths are powers of two (natural
  // pointer wrap); one entry alone could never leave almost_full.
  generate
    if ((PARA_STREAM_FIFO_DEPTH < LPARA_FIFO_DEPTH_MIN) ||
        (PARA_STREAM_FIFO_DEPTH > LPARA_FIFO_DEPTH_MAX) ||
        ((PARA_STREAM_FIFO_DEPTH & (PARA_STREAM_FIFO_DEPTH - 32'd1)) != 32'd0) ||
        (PARA_DMA_FIFO_DEPTH < LPARA_FIFO_DEPTH_MIN) ||
        (PARA_DMA_FIFO_DEPTH > LPARA_FIFO_DEPTH_MAX) ||
        ((PARA_DMA_FIFO_DEPTH & (PARA_DMA_FIFO_DEPTH - 32'd1)) != 32'd0)) begin : gen_para_check
      $error("m_fpt_format_bridge: unsupported PARA_STREAM_FIFO_DEPTH or PARA_DMA_FIFO_DEPTH");
    end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Internal signals
  // ------------------------------------------------------------------------------------------
  logic                                     w_clear;
  logic                                     w_take;
  logic                                     w_flush_cycle;
  logic                                     w_fb_ready;
  // Line Hold Register
  logic                                     reg_dt_hold;
  logic [1:0]                               reg_vc_tag_hold;
  logic                                     reg_err_hold;
  logic                                     w_dt_nxt;
  logic [1:0]                               w_vc_tag_nxt;
  logic                                     w_err_nxt;
  // RAW16 Expander
  logic [LPARA_DATA_OUT_WIDTH-1:0]          w_raw16_data;
  // Bypass Re-packer
  logic [LPARA_REPACK_BUF_WIDTH-1:0]        reg_repack_buf;
  logic [LPARA_REPACK_CNT_WIDTH-1:0]        reg_repack_cnt;
  logic                                     reg_rep_sof_pending;
  logic                                     reg_rep_sol_pending;
  logic                                     reg_rep_eol_pending;
  logic [LPARA_REPACK_CNT_WIDTH-1:0]        w_intake_bytes;
  logic [LPARA_REPACK_CNT_WIDTH-1:0]        w_new_bytes;
  logic [LPARA_PACKED_WIDTH-1:0]            w_packed_new_bytes;
  logic [LPARA_PACKED_WIDTH-1:0]            w_intake_data;
  logic [LPARA_BUF_NEXT_WIDTH-1:0]          w_rep_shifted;
  logic [LPARA_BUF_NEXT_WIDTH-1:0]          w_buf_next;
  logic [LPARA_REPACK_CNT_WIDTH-1:0]        w_total;
  logic [LPARA_REPACK_CNT_WIDTH-1:0]        w_rep_total_remain;
  logic                                     w_rep_drain;
  logic                                     w_rep_empties;
  logic [LPARA_BYTE_EN_WIDTH-1:0]           w_rep_be_flush;
  logic                                     w_rep_flush_req;
  logic                                     w_rep_sof_pending_nxt;
  logic                                     w_rep_sol_pending_nxt;
  logic                                     w_rep_eol_pending_nxt;
  logic [LPARA_DATA_OUT_WIDTH-1:0]          w_rep_data;
  logic                                     w_rep_valid;
  logic [LPARA_BYTE_EN_WIDTH-1:0]           w_rep_byte_en;
  logic                                     w_rep_sof;
  logic                                     w_rep_sol;
  logic                                     w_rep_eol;
  logic [1:0]                               w_rep_vc_tag;
  logic                                     w_rep_err;
  // RAW16 Accumulator & Drain
  logic [LPARA_RAW16_RESIDUAL_WIDTH-1:0]    reg_r16_buf;
  logic [LPARA_R16_CNT_WIDTH-1:0]           reg_r16_cnt;
  logic                                     reg_r16_sof_pending;
  logic                                     reg_r16_sol_pending;
  logic                                     reg_r16_eol_pending;
  logic [LPARA_R16_TOTAL_WIDTH-1:0]         w_r16_intake_bytes;
  logic [LPARA_R16_TOTAL_WIDTH-1:0]         w_r16_new_bytes;
  logic [LPARA_DATA_OUT_WIDTH-1:0]          w_r16_intake_data;
  logic [LPARA_R16_BUF_NEXT_WIDTH-1:0]      w_r16_shifted;
  logic [LPARA_R16_BUF_NEXT_WIDTH-1:0]      w_r16_buf_next;
  logic [LPARA_R16_TOTAL_WIDTH-1:0]         w_r16_total;
  logic [LPARA_R16_CNT_WIDTH-1:0]           w_r16_total_remain;
  logic                                     w_r16_drain;
  logic                                     w_r16_empties;
  logic [LPARA_BYTE_EN_WIDTH-1:0]           w_r16_be_flush;
  logic                                     w_r16_flush_req;
  logic                                     w_r16_sof_pending_nxt;
  logic                                     w_r16_sol_pending_nxt;
  logic                                     w_r16_eol_pending_nxt;
  logic [LPARA_DATA_OUT_WIDTH-1:0]          w_raw16_data_out;
  logic                                     w_raw16_valid;
  logic [LPARA_BYTE_EN_WIDTH-1:0]           w_raw16_byte_en;
  logic                                     w_raw16_sof;
  logic                                     w_raw16_sol;
  logic                                     w_raw16_eol;
  logic [1:0]                               w_raw16_vc_tag;
  logic                                     w_raw16_err;
  // Format Mux
  logic [LPARA_STREAM_ENTRY_WIDTH-1:0]      w_s_bundle;
  logic [LPARA_DMA_ENTRY_WIDTH-1:0]         w_d_bundle;
  logic                                     w_stream_wr;
  logic                                     w_dma_wr;
  // Stream FIFO-FWFT
  logic [LPARA_STREAM_ENTRY_WIDTH-1:0]      reg_stream_fifo_memory [PARA_STREAM_FIFO_DEPTH];
  logic [LPARA_STREAM_PTR_WIDTH-1:0]        reg_stream_fifo_wr_ptr;
  logic [LPARA_STREAM_PTR_WIDTH-1:0]        reg_stream_fifo_rd_ptr;
  logic [LPARA_STREAM_CNT_WIDTH-1:0]        reg_stream_fifo_count;
  logic                                     w_stream_fifo_full;
  logic                                     w_stream_fifo_empty;
  logic                                     w_stream_fifo_almost_full;
  logic                                     w_stream_fifo_rd;
  logic [LPARA_STREAM_ENTRY_WIDTH-1:0]      w_stream_fifo_head;
  logic                                     w_fbs_valid;
  // DMA FIFO-FWFT
  logic [LPARA_DMA_ENTRY_WIDTH-1:0]         reg_dma_fifo_memory [PARA_DMA_FIFO_DEPTH];
  logic [LPARA_DMA_PTR_WIDTH-1:0]           reg_dma_fifo_wr_ptr;
  logic [LPARA_DMA_PTR_WIDTH-1:0]           reg_dma_fifo_rd_ptr;
  logic [LPARA_DMA_CNT_WIDTH-1:0]           reg_dma_fifo_count;
  logic                                     w_dma_fifo_full;
  logic                                     w_dma_fifo_empty;
  logic                                     w_dma_fifo_almost_full;
  logic                                     w_dma_fifo_rd;
  logic [LPARA_DMA_ENTRY_WIDTH-1:0]         w_dma_fifo_head;
  logic                                     w_fbd_valid;
  logic                                     w_fb_overflow;

  // Bytes produced by one entry in bypass format: RAW12 2 px -> 3, 4 px -> 6, RAW10 4 px -> 5;
  // any other combination does not occur and yields 0.
  function automatic logic [LPARA_REPACK_CNT_WIDTH-1:0] intake_table(input logic dt,
                                                                      input logic [2:0] count);
    logic [LPARA_REPACK_CNT_WIDTH-1:0] bytes;
    case ({dt, count})
      4'b0_010: bytes = LPARA_REPACK_CNT_WIDTH'(3);
      4'b0_100: bytes = LPARA_REPACK_CNT_WIDTH'(6);
      4'b1_100: bytes = LPARA_REPACK_CNT_WIDTH'(5);
      default:  bytes = '0;
    endcase
    return bytes;
  endfunction

  // CSI-2 byte packing of the four 12-bit lanes, oldest byte in [7:0]. RAW12 per pixel pair:
  // Pn[11:4], Pn+1[11:4], {Pn+1[3:0], Pn[3:0]}; RAW10 per group: P0..P3[9:2], then the four
  // 2-bit LSB pairs, P0 in the low bits.
  function automatic logic [LPARA_PACKED_WIDTH-1:0] packed_new_bytes(input logic dt,
                                                                      input logic [47:0] px);
    logic [LPARA_PACKED_WIDTH-1:0] bytes;
    bytes = '0;
    if (dt) begin
      for (int lane = 0; lane < 4; lane++) begin
        bytes[8*lane +: 8]      = px[12*lane+2 +: 8];
        bytes[32+2*lane +: 2]   = px[12*lane +: 2];
      end
    end else begin
      for (int pair = 0; pair < 2; pair++) begin
        bytes[24*pair +: 8]     = px[24*pair+4 +: 8];
        bytes[24*pair+8 +: 8]   = px[24*pair+16 +: 8];
        bytes[24*pair+16 +: 8]  = {px[24*pair+12 +: 4], px[24*pair +: 4]};
      end
    end
    return bytes;
  endfunction

  // ------------------------------------------------------------------------------------------
  // fb_ready & take Logic
  // ------------------------------------------------------------------------------------------
  // fb_ready drops on a line-end flush cycle of either formatter and when an enabled path's FIFO
  // has at most one free entry; a disabled path never holds the input.
  always_comb begin : p_com_take
    w_clear       = i_soft_reset || i_fb_flush;
    w_flush_cycle = w_rep_flush_req || w_r16_flush_req;
    w_fb_ready    = !w_flush_cycle && !(i_axis_enable && w_stream_fifo_almost_full) &&
                    !(i_dma_enable && w_dma_fifo_almost_full);
    w_take        = i_fb_px_valid && w_fb_ready;
  end

  // ------------------------------------------------------------------------------------------
  // Line Hold Register
  // ------------------------------------------------------------------------------------------
  // Data Type and VC tag of the line, loaded on the taken start-of-line entry, and the wc_align
  // error loaded on the taken end-of-line entry. The loading entry itself uses the new value.
  always_comb begin : p_com_line_hold
    w_dt_nxt     = (i_fb_px_sol && w_take) ? i_fb_px_dt_sel : reg_dt_hold;
    w_vc_tag_nxt = (i_fb_px_sol && w_take) ? i_fb_px_vc_tag : reg_vc_tag_hold;
    w_err_nxt    = (i_fb_px_eol && w_take) ? i_fb_err_wc_align : reg_err_hold;
  end

  // Cleared by resetn, soft_reset and fb_flush.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_line_hold
    if (!i_resetn) begin
      reg_dt_hold     <= 1'b0;
      reg_vc_tag_hold <= '0;
      reg_err_hold    <= 1'b0;
    end else if (w_clear) begin
      reg_dt_hold     <= 1'b0;
      reg_vc_tag_hold <= '0;
      reg_err_hold    <= 1'b0;
    end else begin
      reg_dt_hold     <= w_dt_nxt;
      reg_vc_tag_hold <= w_vc_tag_nxt;
      reg_err_hold    <= w_err_nxt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // RAW16 Expander
  // ------------------------------------------------------------------------------------------
  // Each 12-bit lane becomes a right-aligned 16-bit container with the four MSBs at 0.
  always_comb begin : p_com_raw16_expander
    for (int lane = 0; lane < 4; lane++) begin
      w_raw16_data[16*lane +: 16] = {4'b0000, i_fb_px_data[12*lane +: 12]};
    end
  end

  // ------------------------------------------------------------------------------------------
  // Bypass Re-packer
  // ------------------------------------------------------------------------------------------
  // Intake of a taken entry: only the counted bytes are kept, so the residual keeps zeros above
  // its count; new bytes are placed above the residual (buf_next spans 0..13 bytes).
  always_comb begin : p_com_rep_intake
    w_intake_bytes     = intake_table(w_dt_nxt, i_fb_px_valid_count);
    w_new_bytes        = w_take ? w_intake_bytes : '0;
    w_packed_new_bytes = packed_new_bytes(w_dt_nxt, i_fb_px_data);
    for (int idx = 0; idx < int'(LPARA_PACKED_WIDTH / 32'd8); idx++) begin
      w_intake_data[8*idx +: 8] = (LPARA_REPACK_CNT_WIDTH'(idx) < w_new_bytes) ?
                                  w_packed_new_bytes[8*idx +: 8] : 8'h00;
    end
    w_rep_shifted = {{(LPARA_BUF_NEXT_WIDTH - LPARA_PACKED_WIDTH){1'b0}}, w_intake_data} <<
                    {reg_repack_cnt, 3'b000};
    w_buf_next    = {{(LPARA_BUF_NEXT_WIDTH - LPARA_REPACK_BUF_WIDTH){1'b0}}, reg_repack_buf} |
                    w_rep_shifted;
    w_total       = reg_repack_cnt + w_new_bytes;
  end

  // Byte Drain Control: a beat of 8 bytes leaves when 8 or more bytes are held; the line-end
  // flush emits the residual on the cycle after the end-of-line entry.
  always_comb begin : p_com_rep_drain
    w_rep_drain        = w_total >= LPARA_REPACK_CNT_WIDTH'(8);
    w_rep_empties      = w_total == LPARA_REPACK_CNT_WIDTH'(8);
    w_rep_total_remain = w_total - LPARA_REPACK_CNT_WIDTH'(8);
    w_rep_be_flush     = LPARA_BYTE_EN_WIDTH'((32'd1 << reg_repack_cnt) - 32'd1);
    w_rep_flush_req    = reg_rep_eol_pending && (reg_repack_cnt != '0);
  end

  // Pending Regs: set-or-hold flags latched on a taken entry, cleared by the beat that carries
  // the flag.
  always_comb begin : p_com_rep_pending
    w_rep_sof_pending_nxt = (i_fb_px_sof && w_take) || reg_rep_sof_pending;
    w_rep_sol_pending_nxt = (i_fb_px_sol && w_take) || reg_rep_sol_pending;
    w_rep_eol_pending_nxt = (i_fb_px_eol && w_take) || reg_rep_eol_pending;
  end

  // Output Formation: the flush beat carries the residual; sof/sol go on the first beat emitted
  // after their entry, eol on the beat that empties the line, err with eol.
  always_comb begin : p_com_rep_output
    w_rep_data    = w_rep_flush_req ?
                    {{(LPARA_DATA_OUT_WIDTH - LPARA_REPACK_BUF_WIDTH){1'b0}}, reg_repack_buf} :
                    LPARA_DATA_OUT_WIDTH'(w_buf_next);
    w_rep_valid   = w_rep_flush_req || w_rep_drain;
    w_rep_byte_en = w_rep_flush_req ? w_rep_be_flush : 8'hFF;
    w_rep_eol     = w_rep_flush_req || (w_rep_drain && w_rep_eol_pending_nxt && w_rep_empties);
    w_rep_sof     = w_rep_valid && w_rep_sof_pending_nxt;
    w_rep_sol     = w_rep_valid && w_rep_sol_pending_nxt;
    w_rep_vc_tag  = w_vc_tag_nxt;
    w_rep_err     = w_err_nxt && w_rep_eol;
  end

  // Residual register and count: cleared by resetn, soft_reset, fb_flush and the flush beat.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_rep_accumulator
    if (!i_resetn) begin
      reg_repack_buf <= '0;
      reg_repack_cnt <= '0;
    end else if (w_clear || w_rep_flush_req) begin
      reg_repack_buf <= '0;
      reg_repack_cnt <= '0;
    end else if (w_rep_drain) begin
      reg_repack_buf <= LPARA_REPACK_BUF_WIDTH'(w_buf_next >> LPARA_DATA_OUT_WIDTH);
      reg_repack_cnt <= w_rep_total_remain;
    end else begin
      reg_repack_buf <= w_buf_next[LPARA_REPACK_BUF_WIDTH-1:0];
      reg_repack_cnt <= w_total;
    end
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_rep_pending
    if (!i_resetn) begin
      reg_rep_sof_pending <= 1'b0;
      reg_rep_sol_pending <= 1'b0;
      reg_rep_eol_pending <= 1'b0;
    end else if (w_clear) begin
      reg_rep_sof_pending <= 1'b0;
      reg_rep_sol_pending <= 1'b0;
      reg_rep_eol_pending <= 1'b0;
    end else begin
      reg_rep_sof_pending <= w_rep_sof ? 1'b0 : w_rep_sof_pending_nxt;
      reg_rep_sol_pending <= w_rep_sol ? 1'b0 : w_rep_sol_pending_nxt;
      reg_rep_eol_pending <= w_rep_eol ? 1'b0 : w_rep_eol_pending_nxt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // RAW16 Accumulator & Drain
  // ------------------------------------------------------------------------------------------
  // Two bytes per pixel; only the counted bytes of a taken entry enter, above the residual.
  always_comb begin : p_com_r16_intake
    w_r16_intake_bytes = {1'b0, i_fb_px_valid_count} << 1;
    w_r16_new_bytes    = w_take ? w_r16_intake_bytes : '0;
    for (int idx = 0; idx < int'(LPARA_BYTE_EN_WIDTH); idx++) begin
      w_r16_intake_data[8*idx +: 8] = (LPARA_R16_TOTAL_WIDTH'(idx) < w_r16_new_bytes) ?
                                      w_raw16_data[8*idx +: 8] : 8'h00;
    end
    w_r16_shifted  = {{(LPARA_R16_BUF_NEXT_WIDTH - LPARA_DATA_OUT_WIDTH){1'b0}},
                      w_r16_intake_data} << {reg_r16_cnt, 3'b000};
    w_r16_buf_next = {{(LPARA_R16_BUF_NEXT_WIDTH - LPARA_RAW16_RESIDUAL_WIDTH){1'b0}},
                      reg_r16_buf} | w_r16_shifted;
    w_r16_total    = LPARA_R16_TOTAL_WIDTH'(reg_r16_cnt) + w_r16_new_bytes;
  end

  // Byte Drain Control of the RAW16 path; the residual is 0 or 4 bytes.
  always_comb begin : p_com_r16_drain
    w_r16_drain        = w_r16_total >= LPARA_R16_TOTAL_WIDTH'(8);
    w_r16_empties      = w_r16_total == LPARA_R16_TOTAL_WIDTH'(8);
    w_r16_total_remain = LPARA_R16_CNT_WIDTH'(w_r16_total - LPARA_R16_TOTAL_WIDTH'(8));
    w_r16_be_flush     = LPARA_BYTE_EN_WIDTH'((32'd1 << reg_r16_cnt) - 32'd1);
    w_r16_flush_req    = reg_r16_eol_pending && (reg_r16_cnt != '0);
  end

  // Pending Regs of the RAW16 formatter, same set-or-hold rule as the bypass formatter.
  always_comb begin : p_com_r16_pending
    w_r16_sof_pending_nxt = (i_fb_px_sof && w_take) || reg_r16_sof_pending;
    w_r16_sol_pending_nxt = (i_fb_px_sol && w_take) || reg_r16_sol_pending;
    w_r16_eol_pending_nxt = (i_fb_px_eol && w_take) || reg_r16_eol_pending;
  end

  // Output Formation of the RAW16 formatter.
  always_comb begin : p_com_r16_output
    w_raw16_data_out = w_r16_flush_req ?
                       {{(LPARA_DATA_OUT_WIDTH - LPARA_RAW16_RESIDUAL_WIDTH){1'b0}}, reg_r16_buf} :
                       LPARA_DATA_OUT_WIDTH'(w_r16_buf_next);
    w_raw16_valid    = w_r16_flush_req || w_r16_drain;
    w_raw16_byte_en  = w_r16_flush_req ? w_r16_be_flush : 8'hFF;
    w_raw16_eol      = w_r16_flush_req || (w_r16_drain && w_r16_eol_pending_nxt && w_r16_empties);
    w_raw16_sof      = w_raw16_valid && w_r16_sof_pending_nxt;
    w_raw16_sol      = w_raw16_valid && w_r16_sol_pending_nxt;
    w_raw16_vc_tag   = w_vc_tag_nxt;
    w_raw16_err      = w_err_nxt && w_raw16_eol;
  end

  // Residual register and count of the RAW16 path: cleared by resetn, soft_reset, fb_flush and
  // the flush beat.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_r16_accumulator
    if (!i_resetn) begin
      reg_r16_buf <= '0;
      reg_r16_cnt <= '0;
    end else if (w_clear || w_r16_flush_req) begin
      reg_r16_buf <= '0;
      reg_r16_cnt <= '0;
    end else if (w_r16_drain) begin
      reg_r16_buf <= LPARA_RAW16_RESIDUAL_WIDTH'(w_r16_buf_next >> LPARA_DATA_OUT_WIDTH);
      reg_r16_cnt <= w_r16_total_remain;
    end else begin
      reg_r16_buf <= w_r16_buf_next[LPARA_RAW16_RESIDUAL_WIDTH-1:0];
      reg_r16_cnt <= LPARA_R16_CNT_WIDTH'(w_r16_total);
    end
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_r16_pending
    if (!i_resetn) begin
      reg_r16_sof_pending <= 1'b0;
      reg_r16_sol_pending <= 1'b0;
      reg_r16_eol_pending <= 1'b0;
    end else if (w_clear) begin
      reg_r16_sof_pending <= 1'b0;
      reg_r16_sol_pending <= 1'b0;
      reg_r16_eol_pending <= 1'b0;
    end else begin
      reg_r16_sof_pending <= w_raw16_sof ? 1'b0 : w_r16_sof_pending_nxt;
      reg_r16_sol_pending <= w_raw16_sol ? 1'b0 : w_r16_sol_pending_nxt;
      reg_r16_eol_pending <= w_raw16_eol ? 1'b0 : w_r16_eol_pending_nxt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Format Mux
  // ------------------------------------------------------------------------------------------
  // Each path takes the whole bundle of the formatter its format bit selects (1 = RAW16); the
  // stream entry has no sol. A disabled path is not written.
  always_comb begin : p_com_format_mux
    if (i_stream_format) begin
      w_s_bundle = {w_raw16_err, w_raw16_vc_tag, w_raw16_eol, w_raw16_sof, w_raw16_byte_en,
                    w_raw16_data_out};
    end else begin
      w_s_bundle = {w_rep_err, w_rep_vc_tag, w_rep_eol, w_rep_sof, w_rep_byte_en, w_rep_data};
    end
    if (i_dma_format) begin
      w_d_bundle = {w_raw16_sol, w_raw16_err, w_raw16_vc_tag, w_raw16_eol, w_raw16_sof,
                    w_raw16_byte_en, w_raw16_data_out};
    end else begin
      w_d_bundle = {w_rep_sol, w_rep_err, w_rep_vc_tag, w_rep_eol, w_rep_sof, w_rep_byte_en,
                    w_rep_data};
    end
    w_stream_wr = i_axis_enable && (i_stream_format ? w_raw16_valid : w_rep_valid);
    w_dma_wr    = i_dma_enable && (i_dma_format ? w_raw16_valid : w_rep_valid);
  end

  // ------------------------------------------------------------------------------------------
  // Stream FIFO-FWFT
  // ------------------------------------------------------------------------------------------
  // Status from the count at the start of the cycle; the head entry is presented while not
  // empty and leaves on fbs_ready. A write into a full FIFO is refused even with a read.
  always_comb begin : p_com_stream_fifo
    w_stream_fifo_full        = 32'(reg_stream_fifo_count) == PARA_STREAM_FIFO_DEPTH;
    w_stream_fifo_empty       = reg_stream_fifo_count == '0;
    w_stream_fifo_almost_full = (PARA_STREAM_FIFO_DEPTH - 32'(reg_stream_fifo_count)) <= 32'd1;
    w_stream_fifo_rd          = i_fbs_ready && !w_stream_fifo_empty;
    w_fbs_valid               = !w_stream_fifo_empty;
    w_stream_fifo_head        = reg_stream_fifo_memory[reg_stream_fifo_rd_ptr];
  end

  // Pointers and count: cleared by resetn, soft_reset and fb_flush.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_stream_fifo
    if (!i_resetn) begin
      reg_stream_fifo_wr_ptr <= '0;
      reg_stream_fifo_rd_ptr <= '0;
      reg_stream_fifo_count  <= '0;
    end else if (w_clear) begin
      reg_stream_fifo_wr_ptr <= '0;
      reg_stream_fifo_rd_ptr <= '0;
      reg_stream_fifo_count  <= '0;
    end else begin
      if (w_stream_wr && !w_stream_fifo_full) begin
        reg_stream_fifo_wr_ptr <= reg_stream_fifo_wr_ptr + LPARA_STREAM_PTR_WIDTH'(1);
      end
      if (w_stream_fifo_rd) begin
        reg_stream_fifo_rd_ptr <= reg_stream_fifo_rd_ptr + LPARA_STREAM_PTR_WIDTH'(1);
      end
      reg_stream_fifo_count <= reg_stream_fifo_count +
                               LPARA_STREAM_CNT_WIDTH'(w_stream_wr && !w_stream_fifo_full) -
                               LPARA_STREAM_CNT_WIDTH'(w_stream_fifo_rd);
    end
  end

  // Entry storage has no reset: entries are read only while the count shows them valid.
  always_ff @(posedge i_sys_clk) begin : p_ff_stream_fifo_memory
    if (w_stream_wr && !w_stream_fifo_full) begin
      reg_stream_fifo_memory[reg_stream_fifo_wr_ptr] <= w_s_bundle;
    end
  end

  // ------------------------------------------------------------------------------------------
  // DMA FIFO-FWFT
  // ------------------------------------------------------------------------------------------
  // Same structure as the stream FIFO, read by fbd_ready.
  always_comb begin : p_com_dma_fifo
    w_dma_fifo_full        = 32'(reg_dma_fifo_count) == PARA_DMA_FIFO_DEPTH;
    w_dma_fifo_empty       = reg_dma_fifo_count == '0;
    w_dma_fifo_almost_full = (PARA_DMA_FIFO_DEPTH - 32'(reg_dma_fifo_count)) <= 32'd1;
    w_dma_fifo_rd          = i_fbd_ready && !w_dma_fifo_empty;
    w_fbd_valid            = !w_dma_fifo_empty;
    w_dma_fifo_head        = reg_dma_fifo_memory[reg_dma_fifo_rd_ptr];
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_dma_fifo
    if (!i_resetn) begin
      reg_dma_fifo_wr_ptr <= '0;
      reg_dma_fifo_rd_ptr <= '0;
      reg_dma_fifo_count  <= '0;
    end else if (w_clear) begin
      reg_dma_fifo_wr_ptr <= '0;
      reg_dma_fifo_rd_ptr <= '0;
      reg_dma_fifo_count  <= '0;
    end else begin
      if (w_dma_wr && !w_dma_fifo_full) begin
        reg_dma_fifo_wr_ptr <= reg_dma_fifo_wr_ptr + LPARA_DMA_PTR_WIDTH'(1);
      end
      if (w_dma_fifo_rd) begin
        reg_dma_fifo_rd_ptr <= reg_dma_fifo_rd_ptr + LPARA_DMA_PTR_WIDTH'(1);
      end
      reg_dma_fifo_count <= reg_dma_fifo_count +
                            LPARA_DMA_CNT_WIDTH'(w_dma_wr && !w_dma_fifo_full) -
                            LPARA_DMA_CNT_WIDTH'(w_dma_fifo_rd);
    end
  end

  // Entry storage has no reset: entries are read only while the count shows them valid.
  always_ff @(posedge i_sys_clk) begin : p_ff_dma_fifo_memory
    if (w_dma_wr && !w_dma_fifo_full) begin
      reg_dma_fifo_memory[reg_dma_fifo_wr_ptr] <= w_d_bundle;
    end
  end

  // A beat is lost when its path writes into a FIFO that is full at the start of the cycle.
  always_comb begin : p_com_overflow
    w_fb_overflow = (w_stream_wr && w_stream_fifo_full) || (w_dma_wr && w_dma_fifo_full);
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  assign o_fb_ready                = w_fb_ready;
  assign o_fbs_valid               = w_fbs_valid;
  assign o_fbs_data                = w_stream_fifo_head[LPARA_DATA_OUT_WIDTH-1:0];
  assign o_fbs_byte_en             = w_stream_fifo_head[LPARA_DATA_OUT_WIDTH+:LPARA_BYTE_EN_WIDTH];
  assign o_fbs_sof                 = w_stream_fifo_head[LPARA_SOF_BIT];
  assign o_fbs_eol                 = w_stream_fifo_head[LPARA_EOL_BIT];
  assign o_fbs_vc_tag              = w_stream_fifo_head[LPARA_VC_TAG_LSB +: 2];
  assign o_fbs_err                 = w_stream_fifo_head[LPARA_ERR_BIT];
  assign o_fbd_valid               = w_fbd_valid;
  assign o_fbd_data                = w_dma_fifo_head[LPARA_DATA_OUT_WIDTH-1:0];
  assign o_fbd_byte_en             = w_dma_fifo_head[LPARA_DATA_OUT_WIDTH+:LPARA_BYTE_EN_WIDTH];
  assign o_fbd_sof                 = w_dma_fifo_head[LPARA_SOF_BIT];
  assign o_fbd_eol                 = w_dma_fifo_head[LPARA_EOL_BIT];
  assign o_fbd_vc_tag              = w_dma_fifo_head[LPARA_VC_TAG_LSB +: 2];
  assign o_fbd_err                 = w_dma_fifo_head[LPARA_ERR_BIT];
  assign o_fbd_sol                 = w_dma_fifo_head[LPARA_SOL_BIT];
  assign o_fb_overflow             = w_fb_overflow;
  assign o_stream_fifo_almost_full = w_stream_fifo_almost_full;
  assign o_dma_fifo_almost_full    = w_dma_fifo_almost_full;

endmodule

`default_nettype wire
