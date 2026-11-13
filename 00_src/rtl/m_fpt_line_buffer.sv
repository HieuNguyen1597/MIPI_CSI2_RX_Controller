// Module      : m_fpt_line_buffer
// Block       : B07 - Line Buffer
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.3
// Date        : 2026-11-06
// Description : Single-clock pixel FIFO with a two-entry first-word-fall-through read stage
`timescale 1ns/1ps
`default_nettype none

module m_fpt_line_buffer #(
  // Parameter Description: number of buffer entries; sets the pointer and fill-level widths.
  // Parameter values: power of two from 4 to 32768; default is 2048.
  // Parameter unit: count
  parameter logic [31:0] PARA_LINE_BUFFER_DEPTH = 32'd2048,
  // Parameter Description: fill level from which linebuffer_full is asserted.
  // Parameter values: 1 to PARA_LINE_BUFFER_DEPTH - 1; default is 1740.
  // Parameter unit: count
  parameter logic [31:0] PARA_FULL_WATERMARK    = 32'd1740
) (
  input  wire                                         i_sys_clk,
  input  wire                                         i_resetn,
  input  wire                                         i_soft_reset,
  input  wire                                         i_lb_flush,
  input  wire  [47:0]                                 i_px_data,
  input  wire                                         i_px_valid,
  input  wire  [2:0]                                  i_px_valid_count,
  input  wire                                         i_px_sof,
  input  wire                                         i_px_sol,
  input  wire                                         i_px_eol,
  input  wire  [1:0]                                  i_px_vc_tag,
  input  wire                                         i_px_dt_sel,
  input  wire                                         i_err_wc_align,
  input  wire                                         i_fb_ready,
  output logic [47:0]                                 o_fb_px_data,
  output logic                                        o_fb_px_valid,
  output logic [2:0]                                  o_fb_px_valid_count,
  output logic                                        o_fb_px_sof,
  output logic                                        o_fb_px_sol,
  output logic                                        o_fb_px_eol,
  output logic [1:0]                                  o_fb_px_vc_tag,
  output logic                                        o_fb_px_dt_sel,
  output logic                                        o_fb_err_wc_align,
  output logic                                        o_linebuffer_full,
  output logic [$clog2(PARA_LINE_BUFFER_DEPTH + 1)-1:0] o_linebuf_fill,
  output logic                                        o_linebuf_overflow,
  output logic                                        o_linebuf_empty
);

  // ------------------------------------------------------------------------------------------
  // Constants
  // ------------------------------------------------------------------------------------------
  // An entry holds px_data, px_vc_tag, px_valid_count and five one-bit fields.
  localparam logic [31:0] LPARA_DATA_WIDTH        = 32'd48;
  localparam logic [31:0] LPARA_VC_TAG_WIDTH      = 32'd2;
  localparam logic [31:0] LPARA_VALID_COUNT_WIDTH = 32'd3;
  localparam logic [31:0] LPARA_ENTRY_WIDTH       = LPARA_DATA_WIDTH + LPARA_VC_TAG_WIDTH +
                                                    LPARA_VALID_COUNT_WIDTH + 32'd5;
  // Read stage: output register plus skid register.
  localparam logic [31:0] LPARA_READ_SKID_DEPTH   = 32'd2;
  // Pointers wrap at the depth; the fill level counts 0..depth inclusive.
  localparam logic [31:0] LPARA_PTR_WIDTH         = $clog2(PARA_LINE_BUFFER_DEPTH);
  localparam logic [31:0] LPARA_FILL_CNT_WIDTH    = $clog2(PARA_LINE_BUFFER_DEPTH + 1);
  // occupancy and space hold 0..READ_SKID_DEPTH + 1 before the subtraction in space.
  localparam logic [31:0] LPARA_SKID_CNT_WIDTH    = $clog2(LPARA_READ_SKID_DEPTH + 32'd2);

  // ------------------------------------------------------------------------------------------
  // Parameter check
  // ------------------------------------------------------------------------------------------
  // An unsupported depth or watermark stops elaboration with an error. The depth is a power of
  // two from 4 to 32768 (natural pointer wrap, fill level fits 16 bits); the watermark leaves at
  // least the reserved end-of-line slot above it.
  generate
    if ((PARA_LINE_BUFFER_DEPTH < 32'd4) ||
        (PARA_LINE_BUFFER_DEPTH > 32'd32768) ||
        ((PARA_LINE_BUFFER_DEPTH & (PARA_LINE_BUFFER_DEPTH - 32'd1)) != 32'd0) ||
        (PARA_FULL_WATERMARK < 32'd1) ||
        (PARA_FULL_WATERMARK > PARA_LINE_BUFFER_DEPTH - 32'd1)) begin : gen_para_check
      $error("m_fpt_line_buffer: unsupported PARA_LINE_BUFFER_DEPTH or PARA_FULL_WATERMARK");
    end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Internal signals
  // ------------------------------------------------------------------------------------------
  // Clear common to the control registers
  logic                              w_sync_clear;
  // Write Control
  logic                              w_full_hard;
  logic                              w_full_normal;
  logic                              w_write_en;
  logic                              w_write_accept;
  logic                              w_linebuf_overflow;
  logic [LPARA_PTR_WIDTH-1:0]        reg_wr_ptr;
  // Occupancy Tracking
  logic                              w_consume;
  logic [LPARA_FILL_CNT_WIDTH-1:0]   w_next_fill_cnt;
  logic                              w_linebuffer_full;
  logic                              w_linebuf_empty;
  logic [LPARA_FILL_CNT_WIDTH-1:0]   reg_fill_cnt;
  // Signal Packing and Memory
  logic [LPARA_ENTRY_WIDTH-1:0]      w_w_data;
  logic [LPARA_ENTRY_WIDTH-1:0]      reg_memory [PARA_LINE_BUFFER_DEPTH];
  logic [LPARA_ENTRY_WIDTH-1:0]      reg_r_data;
  // Read Control
  logic                              w_land;
  logic [LPARA_SKID_CNT_WIDTH-1:0]   w_occupancy;
  logic [LPARA_SKID_CNT_WIDTH-1:0]   w_space;
  logic [LPARA_FILL_CNT_WIDTH-1:0]   w_unread;
  logic                              w_rd_en;
  logic [LPARA_PTR_WIDTH-1:0]        reg_rd_ptr;
  logic                              reg_inflight;
  // Output Stage (FWFT)
  logic                              w_promote;
  logic                              w_out_free;
  logic                              w_r_to_out;
  logic                              w_r_to_skid;
  logic                              reg_fb_px_valid;
  logic                              reg_skid_valid;
  logic [LPARA_ENTRY_WIDTH-1:0]      reg_out_reg;
  logic [LPARA_ENTRY_WIDTH-1:0]      reg_skid_reg;

  // ------------------------------------------------------------------------------------------
  // Write Control
  // ------------------------------------------------------------------------------------------
  // soft_reset and lb_flush clear the control registers on the next edge and win over every
  // other update of that cycle; the write decision and linebuf_overflow keep their equations.
  always_comb begin : p_com_sync_clear
    w_sync_clear = i_soft_reset || i_lb_flush;
  end

  // A beat without pixels is not written. A normal entry is refused from depth - 1 on; an
  // end-of-line entry may still take the last slot. The fill level of the current cycle is
  // used, so a consume in the same cycle frees no slot.
  always_comb begin : p_com_write_ctrl
    w_full_hard        = (reg_fill_cnt == LPARA_FILL_CNT_WIDTH'(PARA_LINE_BUFFER_DEPTH));
    w_full_normal      = (reg_fill_cnt >= LPARA_FILL_CNT_WIDTH'(PARA_LINE_BUFFER_DEPTH - 32'd1));
    w_write_en         = i_px_valid && (i_px_valid_count != 3'd0);
    w_write_accept     = w_write_en && (!w_full_normal || (i_px_eol && !w_full_hard));
    w_linebuf_overflow = w_write_en && !w_write_accept;
  end

  // The write pointer wraps naturally at the depth.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_write_ctrl
    if (!i_resetn) begin
      reg_wr_ptr <= '0;
    end else if (w_sync_clear) begin
      reg_wr_ptr <= '0;
    end else if (w_write_accept) begin
      reg_wr_ptr <= reg_wr_ptr + LPARA_PTR_WIDTH'(1);
    end
  end

  // ------------------------------------------------------------------------------------------
  // Occupancy Tracking
  // ------------------------------------------------------------------------------------------
  // fill_cnt counts every entry held: memory, read in flight and read stage. The add is done
  // first; fill + accept stays within 0..depth and a consume needs a held entry, so nothing
  // wraps. The status levels come straight from the fill register.
  always_comb begin : p_com_occupancy
    w_consume         = reg_fb_px_valid && i_fb_ready;
    w_next_fill_cnt   = (reg_fill_cnt + LPARA_FILL_CNT_WIDTH'(w_write_accept)) -
                        LPARA_FILL_CNT_WIDTH'(w_consume);
    w_linebuffer_full = (reg_fill_cnt >= LPARA_FILL_CNT_WIDTH'(PARA_FULL_WATERMARK));
    w_linebuf_empty   = (reg_fill_cnt == '0);
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_occupancy
    if (!i_resetn) begin
      reg_fill_cnt <= '0;
    end else if (w_sync_clear) begin
      reg_fill_cnt <= '0;
    end else begin
      reg_fill_cnt <= w_next_fill_cnt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Signal Packing
  // ------------------------------------------------------------------------------------------
  // Entry layout, most significant field first; px_valid is not stored.
  always_comb begin : p_com_signal_packing
    w_w_data = {i_err_wc_align, i_px_eol, i_px_sol, i_px_sof, i_px_valid_count, i_px_vc_tag,
                i_px_dt_sel, i_px_data};
  end

  // ------------------------------------------------------------------------------------------
  // Memory
  // ------------------------------------------------------------------------------------------
  // Simple dual-port memory with a registered read port addressed by rd_ptr every cycle:
  // r_data holds the entry addressed on the previous cycle. Neither the array nor r_data is
  // reset; only entries written after the last clear are ever read and absorbed.
  always_ff @(posedge i_sys_clk) begin : p_ff_memory
    if (w_write_accept) begin
      reg_memory[reg_wr_ptr] <= w_w_data;
    end
    reg_r_data <= reg_memory[reg_rd_ptr];
  end

  // ------------------------------------------------------------------------------------------
  // Read Control
  // ------------------------------------------------------------------------------------------
  // A read is issued when an entry is still in the memory and the read stage will have room when
  // it lands one cycle later. occupancy and the read in flight never exceed fill_cnt, and space
  // stays within 0..2, so neither subtraction wraps; rd_ptr never passes wr_ptr.
  always_comb begin : p_com_read_ctrl
    w_land      = reg_inflight;
    w_occupancy = LPARA_SKID_CNT_WIDTH'(reg_fb_px_valid) + LPARA_SKID_CNT_WIDTH'(reg_skid_valid);
    w_space     = (LPARA_SKID_CNT_WIDTH'(LPARA_READ_SKID_DEPTH) + LPARA_SKID_CNT_WIDTH'(w_consume)) -
                  (w_occupancy + LPARA_SKID_CNT_WIDTH'(reg_inflight));
    w_unread    = reg_fill_cnt -
                  (LPARA_FILL_CNT_WIDTH'(w_occupancy) + LPARA_FILL_CNT_WIDTH'(reg_inflight));
    w_rd_en     = (w_unread != '0) && (w_space != '0);
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_read_ctrl
    if (!i_resetn) begin
      reg_rd_ptr   <= '0;
      reg_inflight <= 1'b0;
    end else if (w_sync_clear) begin
      reg_rd_ptr   <= '0;
      reg_inflight <= 1'b0;
    end else begin
      if (w_rd_en) begin
        reg_rd_ptr <= reg_rd_ptr + LPARA_PTR_WIDTH'(1);
      end
      reg_inflight <= w_rd_en;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output Stage (FWFT)
  // ------------------------------------------------------------------------------------------
  // Each cycle a consume first releases the output register and promotes an occupied skid
  // register into it; a landing entry then goes to the output register if it is free, otherwise
  // to the skid register.
  always_comb begin : p_com_output_stage
    w_promote   = w_consume && reg_skid_valid;
    w_out_free  = !reg_fb_px_valid || (w_consume && !reg_skid_valid);
    w_r_to_out  = w_land && w_out_free;
    w_r_to_skid = w_land && !w_out_free;
  end

  // Valid bits of the output and skid registers.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_output_valid
    if (!i_resetn) begin
      reg_fb_px_valid <= 1'b0;
      reg_skid_valid  <= 1'b0;
    end else if (w_sync_clear) begin
      reg_fb_px_valid <= 1'b0;
      reg_skid_valid  <= 1'b0;
    end else begin
      reg_fb_px_valid <= w_land || !w_out_free;
      reg_skid_valid  <= w_r_to_skid || (reg_skid_valid && !w_consume);
    end
  end

  // Entry data of the output and skid registers, not reset: fb_* carry meaning only while
  // fb_px_valid is 1, and the skid entry is read only while skid_valid is 1.
  always_ff @(posedge i_sys_clk) begin : p_ff_output_data
    if (w_r_to_out) begin
      reg_out_reg <= reg_r_data;
    end else if (w_promote) begin
      reg_out_reg <= reg_skid_reg;
    end
    if (w_r_to_skid) begin
      reg_skid_reg <= reg_r_data;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  // fb_* are the fields of the output register; the status outputs follow fill_cnt.
  assign {o_fb_err_wc_align, o_fb_px_eol, o_fb_px_sol, o_fb_px_sof, o_fb_px_valid_count,
          o_fb_px_vc_tag, o_fb_px_dt_sel, o_fb_px_data} = reg_out_reg;
  assign o_fb_px_valid      = reg_fb_px_valid;
  assign o_linebuf_fill     = reg_fill_cnt;
  assign o_linebuffer_full  = w_linebuffer_full;
  assign o_linebuf_empty    = w_linebuf_empty;
  assign o_linebuf_overflow = w_linebuf_overflow;

endmodule

`default_nettype wire
