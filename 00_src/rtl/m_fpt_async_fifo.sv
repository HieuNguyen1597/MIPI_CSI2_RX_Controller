// Module      : m_fpt_async_fifo
// Block       : Asynchronous FIFO wrapper B02-B03 (outside Blocks No. 1-14)
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-13
// Description : Gray-pointer asynchronous FIFO from Lane Management to Packet Analyzer.
`timescale 1ns/1ps
`default_nettype none

module m_fpt_async_fifo #(
  // Parameter Description: number of entries; sets the address and pointer widths.
  // Parameter values: power of two from 4 to 256; default is 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_DEPTH       = 16,
  // Parameter Description: flops in each Gray pointer synchroniser chain (both directions).
  // Parameter values: 2 to 4; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_SYNC_STAGES = 2
) (
  input  wire         i_rxbyteclkhs,
  input  wire         i_resetn_rxbyteclkhs,
  input  wire         i_merged_valid,
  input  wire  [31:0] i_merged_data,
  input  wire  [3:0]  i_merged_byte_valid,
  input  wire         i_merged_sob,
  input  wire         i_merged_eob,
  input  wire         i_sys_clk,
  input  wire         i_resetn_sys_clk,
  input  wire         i_core_enable,
  input  wire         i_soft_reset,
  input  wire         i_rd_en,
  output logic [37:0] o_rd_data,
  output logic        o_rd_empty
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_ENTRY_W = 32'd38;
  localparam logic [31:0] LPARA_ADDR_W  = $clog2(PARA_DEPTH);
  localparam logic [31:0] LPARA_PTR_W   = LPARA_ADDR_W + 32'd1;
  // Gray image of a pointer one wrap ahead: the two most significant bits inverted.
  localparam logic [LPARA_PTR_W-1:0] LPARA_FULL_MASK =
    LPARA_PTR_W'(2'b11) << (LPARA_PTR_W - 32'd2);

  // Write side (rxbyteclkhs)
  logic [LPARA_ENTRY_W-1:0] reg_mem [PARA_DEPTH];
  logic [LPARA_PTR_W-1:0]   reg_wptr_bin;
  logic [LPARA_PTR_W-1:0]   reg_wptr_gray;
  logic                     reg_wr_full;
  logic                     w_wr_en;
  logic [LPARA_PTR_W-1:0]   w_wptr_bin_nxt;
  logic [LPARA_PTR_W-1:0]   w_wptr_gray_nxt;
  logic                     w_wr_full_nxt;

  // Pointer synchronisers
  logic [LPARA_PTR_W-1:0]   reg_rptr_gray_sync [PARA_SYNC_STAGES];
  logic [LPARA_PTR_W-1:0]   reg_wptr_gray_sync [PARA_SYNC_STAGES];
  logic [LPARA_PTR_W-1:0]   w_rptr_gray_synced;
  logic [LPARA_PTR_W-1:0]   w_wptr_gray_synced;

  // Read side (sys_clk)
  logic [LPARA_PTR_W-1:0]   reg_rptr_bin;
  logic [LPARA_PTR_W-1:0]   reg_rptr_xing_bin;
  logic [LPARA_PTR_W-1:0]   reg_rptr_gray_xing;
  logic                     reg_rd_empty;
  logic                     w_flush;
  logic                     w_rd_pop;
  logic [LPARA_PTR_W-1:0]   w_rptr_bin_nxt;
  logic [LPARA_PTR_W-1:0]   w_rptr_gray_nxt;
  logic [LPARA_PTR_W-1:0]   w_rptr_xing_bin_nxt;
  logic                     w_rd_empty_nxt;
  logic [LPARA_ENTRY_W-1:0] w_rd_data;

  // Binary to Gray conversion.
  function automatic logic [LPARA_PTR_W-1:0] f_bin2gray(input logic [LPARA_PTR_W-1:0] bin);
    f_bin2gray = bin ^ (bin >> 1);
  endfunction

  // Gray to binary conversion: each binary bit is the XOR of the Gray bits at and above it.
  function automatic logic [LPARA_PTR_W-1:0] f_gray2bin(input logic [LPARA_PTR_W-1:0] gray);
    f_gray2bin = gray;
    for (int idx = int'(LPARA_PTR_W) - 2; idx >= 0; idx--) begin
      f_gray2bin[idx] = f_gray2bin[idx + 1] ^ gray[idx];
    end
  endfunction

  // ------------------------------------------------------------------------------------------
  // Parameter checks
  // ------------------------------------------------------------------------------------------
  // Illegal parameter values stop elaboration.
  if ((PARA_DEPTH < 32'd4) || (PARA_DEPTH > 32'd256) ||
      ((PARA_DEPTH & (PARA_DEPTH - 32'd1)) != 32'd0)) begin : gen_depth_check
    $error("PARA_DEPTH must be a power of two from 4 to 256");
  end
  if ((PARA_SYNC_STAGES < 32'd2) || (PARA_SYNC_STAGES > 32'd4)) begin : gen_sync_check
    $error("PARA_SYNC_STAGES must be from 2 to 4");
  end

  // ------------------------------------------------------------------------------------------
  // Write side
  // ------------------------------------------------------------------------------------------
  // A write is accepted only while not full; a write while full is dropped and changes nothing.
  // full compares the next write pointer with the synchronised read pointer one wrap ahead.
  always_comb begin : p_com_wr_ctrl
    w_wr_en         = i_merged_valid & ~reg_wr_full;
    w_wptr_bin_nxt  = reg_wptr_bin + LPARA_PTR_W'(w_wr_en);
    w_wptr_gray_nxt = f_bin2gray(w_wptr_bin_nxt);
    w_wr_full_nxt   = (w_wptr_gray_nxt == (w_rptr_gray_synced ^ LPARA_FULL_MASK));
  end

  // Write pointers and full, cleared asynchronously by the rxbyteclkhs reset.
  // reg_wptr_gray is the launch flop of the crossing towards sys_clk.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn_rxbyteclkhs) begin : p_ff_wr_ptr
    if (!i_resetn_rxbyteclkhs) begin
      reg_wptr_bin  <= '0;
      reg_wptr_gray <= '0;
      reg_wr_full   <= 1'b0;
    end else begin
      reg_wptr_bin  <= w_wptr_bin_nxt;
      reg_wptr_gray <= w_wptr_gray_nxt;
      reg_wr_full   <= w_wr_full_nxt;
    end
  end

  // Entry storage {eob, sob, byte_valid, data}. Not reset: the read side presents an entry only
  // after the synchronised write pointer shows it written since the last reset.
  always_ff @(posedge i_rxbyteclkhs) begin : p_ff_mem
    if (w_wr_en) begin
      reg_mem[reg_wptr_bin[LPARA_ADDR_W-1:0]] <=
        {i_merged_eob, i_merged_sob, i_merged_byte_valid, i_merged_data};
    end
  end

  // ------------------------------------------------------------------------------------------
  // Pointer synchronisers
  // ------------------------------------------------------------------------------------------
  // Read Gray pointer into rxbyteclkhs. Stage 0 takes the sys_clk launch flop directly and the
  // stages are chained with no logic between them.
  always_ff @(posedge i_rxbyteclkhs or negedge i_resetn_rxbyteclkhs) begin : p_ff_rptr_sync
    if (!i_resetn_rxbyteclkhs) begin
      for (int idx = 0; idx < int'(PARA_SYNC_STAGES); idx++) begin
        reg_rptr_gray_sync[idx] <= '0;
      end
    end else begin
      reg_rptr_gray_sync[0] <= reg_rptr_gray_xing;
      for (int idx = 1; idx < int'(PARA_SYNC_STAGES); idx++) begin
        reg_rptr_gray_sync[idx] <= reg_rptr_gray_sync[idx - 1];
      end
    end
  end

  // Write Gray pointer into sys_clk, same structure as above.
  always_ff @(posedge i_sys_clk or negedge i_resetn_sys_clk) begin : p_ff_wptr_sync
    if (!i_resetn_sys_clk) begin
      for (int idx = 0; idx < int'(PARA_SYNC_STAGES); idx++) begin
        reg_wptr_gray_sync[idx] <= '0;
      end
    end else begin
      reg_wptr_gray_sync[0] <= reg_wptr_gray;
      for (int idx = 1; idx < int'(PARA_SYNC_STAGES); idx++) begin
        reg_wptr_gray_sync[idx] <= reg_wptr_gray_sync[idx - 1];
      end
    end
  end

  // Last stage of each chain, used only in the destination domain.
  always_comb begin : p_com_sync_out
    w_rptr_gray_synced = reg_rptr_gray_sync[PARA_SYNC_STAGES - 32'd1];
    w_wptr_gray_synced = reg_wptr_gray_sync[PARA_SYNC_STAGES - 32'd1];
  end

  // ------------------------------------------------------------------------------------------
  // Read side
  // ------------------------------------------------------------------------------------------
  // Flush while core_enable = 0 or soft_reset = 1: the read position jumps to the synchronised
  // write pointer, so every visible entry is discarded, and rd_empty is forced to 1. Otherwise
  // the head entry is consumed on rd_en while not empty; rd_en while empty has no effect.
  always_comb begin : p_com_rd_ctrl
    w_flush  = ~i_core_enable | i_soft_reset;
    w_rd_pop = i_rd_en & ~reg_rd_empty & ~w_flush;
    if (w_flush) begin
      w_rptr_bin_nxt = f_gray2bin(w_wptr_gray_synced);
    end else begin
      w_rptr_bin_nxt = reg_rptr_bin + LPARA_PTR_W'(w_rd_pop);
    end
    w_rptr_gray_nxt = f_bin2gray(w_rptr_bin_nxt);
    w_rd_empty_nxt  = w_flush | (w_rptr_gray_nxt == w_wptr_gray_synced);
  end

  // The pointer that crosses to rxbyteclkhs steps by at most one entry per edge towards the read
  // position, so its Gray code changes by one bit even when a flush jumps the read position. It
  // never runs ahead of the read position: the write side may see freed entries late, never early.
  always_comb begin : p_com_rptr_xing
    w_rptr_xing_bin_nxt = reg_rptr_xing_bin;
    if (reg_rptr_xing_bin != w_rptr_bin_nxt) begin
      w_rptr_xing_bin_nxt = reg_rptr_xing_bin + LPARA_PTR_W'(1'b1);
    end
  end

  // Read pointers and rd_empty, cleared asynchronously by the sys_clk reset (FIFO empty).
  // reg_rptr_gray_xing is the launch flop of the crossing towards rxbyteclkhs.
  always_ff @(posedge i_sys_clk or negedge i_resetn_sys_clk) begin : p_ff_rd_ptr
    if (!i_resetn_sys_clk) begin
      reg_rptr_bin       <= '0;
      reg_rptr_xing_bin  <= '0;
      reg_rptr_gray_xing <= '0;
      reg_rd_empty       <= 1'b1;
    end else begin
      reg_rptr_bin       <= w_rptr_bin_nxt;
      reg_rptr_xing_bin  <= w_rptr_xing_bin_nxt;
      reg_rptr_gray_xing <= f_bin2gray(w_rptr_xing_bin_nxt);
      reg_rd_empty       <= w_rd_empty_nxt;
    end
  end

  // Show-ahead read data: the head entry while not empty, all zeros while empty.
  always_comb begin : p_com_rd_data
    w_rd_data = '0;
    if (!reg_rd_empty) begin
      w_rd_data = reg_mem[reg_rptr_bin[LPARA_ADDR_W-1:0]];
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  assign o_rd_data  = w_rd_data;
  assign o_rd_empty = reg_rd_empty;

endmodule

`default_nettype wire
