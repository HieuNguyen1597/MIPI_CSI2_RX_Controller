// Module      : m_fpt_dma_async_fifo
// Block       : B13 - DMA
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-09
// Description : First-word-fall-through FIFO with Gray-coded pointers, asynchronous or synchronous
`timescale 1ns/1ps
`default_nettype none
module m_fpt_dma_async_fifo #(
  // Parameter Description: number of entries of the storage; the pointers carry one extra wrap bit.
  // Parameter values: power of two, at least 2; no default use outside the DMA (default 16).
  // Parameter unit: count
  parameter logic [31:0] PARA_DEPTH = 32'd16,
  // Parameter Description: width of one entry.
  // Parameter values: at least 1; default 72.
  // Parameter unit: bit
  parameter logic [31:0] PARA_WIDTH = 32'd72,
  // Parameter Description: 1 = write and read clocks asynchronous; 0 = one clock.
  // Parameter values: 0 or 1; default 1.
  // Parameter unit: none
  parameter logic [31:0] PARA_ASYNC = 32'd1,
  // Parameter Description: synchroniser flops per pointer crossing when PARA_ASYNC = 1.
  // Parameter values: at least 2; default 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_SYNC_STAGES = 32'd2
) (
  input  wire                    i_wr_clk,
  input  wire                    i_wr_rst_n,
  input  wire                    i_push,
  input  wire  [PARA_WIDTH-1:0]  i_din,
  output logic                   o_full,
  output logic [$clog2(PARA_DEPTH):0] o_wr_used,
  input  wire                    i_rd_clk,
  input  wire                    i_rd_rst_n,
  input  wire                    i_pop,
  output logic [PARA_WIDTH-1:0]  o_dout,
  output logic                   o_empty
);

  // ------------------------------------------------------------------------------------------
  // Constants and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_ADDR_W = $clog2(PARA_DEPTH);
  localparam logic [31:0] LPARA_PTR_W  = LPARA_ADDR_W + 32'd1;
  // Gray code of a pointer one full lap ahead: the two most significant bits inverted.
  localparam logic [LPARA_PTR_W-1:0] LPARA_FULL_MASK = LPARA_PTR_W'(32'd3 << (LPARA_ADDR_W
      - 32'd1));

  generate
    if (((PARA_DEPTH & (PARA_DEPTH - 32'd1)) != 32'd0) || (PARA_DEPTH < 32'd2)
        || (PARA_WIDTH < 32'd1) ||
        (PARA_ASYNC > 32'd1) || (PARA_SYNC_STAGES < 32'd2)) begin : gen_para_check
      $error("m_fpt_dma_async_fifo: unsupported PARA_DEPTH, PARA_WIDTH, PARA_ASYNC or SYNC_STAGES");
    end
  endgenerate

  logic [PARA_WIDTH-1:0]  reg_ram [PARA_DEPTH];
  logic [LPARA_PTR_W-1:0] reg_wptr_bin, reg_wptr_gray, w_wptr_bin_nxt, w_wptr_gray_nxt;
  logic [LPARA_PTR_W-1:0] reg_rptr_bin, reg_rptr_gray, w_rptr_bin_nxt, w_rptr_gray_nxt;
  logic [LPARA_PTR_W-1:0] w_rq2_rptr, w_wq2_wptr, w_rq2_rptr_bin;
  logic                   reg_full, reg_empty, w_push_ok, w_pop_ok;
  logic [PARA_WIDTH-1:0]  reg_dout;

  function automatic logic [LPARA_PTR_W-1:0] f_bin2gray(input logic [LPARA_PTR_W-1:0] bin);
    return bin ^ (bin >> 1);
  endfunction

  function automatic logic [LPARA_PTR_W-1:0] f_gray2bin(input logic [LPARA_PTR_W-1:0] gray);
    logic [LPARA_PTR_W-1:0] bin;
    bin = gray;
    for (int idx = 1; idx < LPARA_PTR_W; idx++) bin = bin ^ (gray >> idx);
    return bin;
  endfunction

  // ------------------------------------------------------------------------------------------
  // Write side
  // ------------------------------------------------------------------------------------------
  // A push while full has no effect; full is registered from the next Gray write pointer against
  // the read pointer seen in the write domain.
  always_comb begin : p_com_wr_ptr
    w_push_ok       = i_push && !reg_full;
    w_wptr_bin_nxt  = reg_wptr_bin + LPARA_PTR_W'(w_push_ok);
    w_wptr_gray_nxt = f_bin2gray(w_wptr_bin_nxt);
    w_rq2_rptr_bin  = f_gray2bin(w_rq2_rptr);
  end

  always_ff @(posedge i_wr_clk or negedge i_wr_rst_n) begin : p_ff_wr_ptr
    if (!i_wr_rst_n) begin
      reg_wptr_bin  <= '0;
      reg_wptr_gray <= '0;
      reg_full      <= 1'b0;
    end else begin
      reg_wptr_bin  <= w_wptr_bin_nxt;
      reg_wptr_gray <= w_wptr_gray_nxt;
      reg_full      <= (w_wptr_gray_nxt == (w_rq2_rptr ^ LPARA_FULL_MASK));
    end
  end

  // Storage written at the binary write pointer; not reset, the reader ignores it until empty is
  // low.
  always_ff @(posedge i_wr_clk) begin : p_ff_ram
    if (w_push_ok) reg_ram[reg_wptr_bin[LPARA_ADDR_W-1:0]] <= i_din;
  end

  // ------------------------------------------------------------------------------------------
  // Read side
  // ------------------------------------------------------------------------------------------
  // A pop while empty has no effect; empty is registered from the next Gray read pointer against
  // the write pointer seen in the read domain. The head is read at the look-ahead address, so it is
  // valid whenever empty is low (first word fall through).
  always_comb begin : p_com_rd_ptr
    w_pop_ok        = i_pop && !reg_empty;
    w_rptr_bin_nxt  = reg_rptr_bin + LPARA_PTR_W'(w_pop_ok);
    w_rptr_gray_nxt = f_bin2gray(w_rptr_bin_nxt);
  end

  always_ff @(posedge i_rd_clk or negedge i_rd_rst_n) begin : p_ff_rd_ptr
    if (!i_rd_rst_n) begin
      reg_rptr_bin  <= '0;
      reg_rptr_gray <= '0;
      reg_empty     <= 1'b1;
    end else begin
      reg_rptr_bin  <= w_rptr_bin_nxt;
      reg_rptr_gray <= w_rptr_gray_nxt;
      reg_empty     <= (w_rptr_gray_nxt == w_wq2_wptr);
    end
  end

  // Registered head reloaded every cycle from the look-ahead read address; no reset (data path).
  always_ff @(posedge i_rd_clk) begin : p_ff_dout
    reg_dout <= reg_ram[w_rptr_bin_nxt[LPARA_ADDR_W-1:0]];
  end

  // ------------------------------------------------------------------------------------------
  // Pointer crossing
  // ------------------------------------------------------------------------------------------
  generate
    if (PARA_ASYNC == 32'd1) begin : gen_async
      logic [LPARA_PTR_W-1:0] reg_rq_rptr_sync [PARA_SYNC_STAGES];
      logic [LPARA_PTR_W-1:0] reg_wq_wptr_sync [PARA_SYNC_STAGES];

      // Gray read pointer into the write domain through PARA_SYNC_STAGES flops, no logic in
      // between.
      always_ff @(posedge i_wr_clk or negedge i_wr_rst_n) begin : p_ff_rq_sync
        if (!i_wr_rst_n) begin
          for (int idx = 0; idx < PARA_SYNC_STAGES; idx++) reg_rq_rptr_sync[idx] <= '0;
        end else begin
          reg_rq_rptr_sync[0] <= reg_rptr_gray;
          for (int idx = 1; idx < PARA_SYNC_STAGES; idx++) reg_rq_rptr_sync[idx]
              <= reg_rq_rptr_sync[idx-1];
        end
      end

      // Gray write pointer into the read domain through PARA_SYNC_STAGES flops, no logic in
      // between.
      always_ff @(posedge i_rd_clk or negedge i_rd_rst_n) begin : p_ff_wq_sync
        if (!i_rd_rst_n) begin
          for (int idx = 0; idx < PARA_SYNC_STAGES; idx++) reg_wq_wptr_sync[idx] <= '0;
        end else begin
          reg_wq_wptr_sync[0] <= reg_wptr_gray;
          for (int idx = 1; idx < PARA_SYNC_STAGES; idx++) reg_wq_wptr_sync[idx]
              <= reg_wq_wptr_sync[idx-1];
        end
      end

      always_comb begin : p_com_sync_tap
        w_rq2_rptr = reg_rq_rptr_sync[PARA_SYNC_STAGES-1];
        w_wq2_wptr = reg_wq_wptr_sync[PARA_SYNC_STAGES-1];
      end
    end else begin : gen_sync
      logic [LPARA_PTR_W-1:0] reg_wptr_dly;

      // One clock: the reader sees the write pointer one cycle late, so the registered head has
      // been reloaded from the written entry when empty falls.
      always_ff @(posedge i_rd_clk or negedge i_rd_rst_n) begin : p_ff_wptr_dly
        if (!i_rd_rst_n) reg_wptr_dly <= '0;
        else reg_wptr_dly <= reg_wptr_gray;
      end

      always_comb begin : p_com_sync_tap
        w_rq2_rptr = reg_rptr_gray;
        w_wq2_wptr = reg_wptr_dly;
      end
    end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------------------------------------
  // o_wr_used: entries seen on the write side, never below the true occupancy.
  assign o_full    = reg_full;
  assign o_wr_used = reg_wptr_bin - w_rq2_rptr_bin;
  assign o_empty   = reg_empty;
  assign o_dout    = reg_dout;

endmodule
`default_nettype wire
