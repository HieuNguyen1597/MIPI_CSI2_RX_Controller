// Module      : m_fpt_csi2rx_wrap
// Block       : TOP - MIPI CSI-2 RX Controller FTEL wrapper
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-22
// Description : FTEL wrapper of the CSI-2 RX core: port renaming only, no logic
`timescale 1ns/1ps
`default_nettype none

module m_fpt_csi2rx_wrap #(
  // Parameter Description: Physical data lanes wired to the D-PHY; shared by B01, B02, B11 and B14.
  // Parameter values: 1, 2 or 4; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_LANES = 32'd2,
  // Parameter Description: Depth of each lane deskew FIFO of B02; must exceed PARA_FIFO_AF_MARGIN.
  // Parameter values: legal set checked by B02; default is 8.
  // Parameter unit: byte
  parameter logic [31:0] PARA_SKEW_DEPTH = 32'd8,
  // Parameter Description: Free entries left in a B02 deskew FIFO when it reports almost full.
  // Parameter values: 0 < PARA_FIFO_AF_MARGIN < PARA_SKEW_DEPTH; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_FIFO_AF_MARGIN = 32'd2,
  // Parameter Description: Entries of the Async FIFO between B02 and B03.
  // Parameter values: power of two, 4 to 256; default is 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_ASYNC_FIFO_DEPTH = 32'd16,
  // Parameter Description: Depth of the generic short packet FIFO of B03.
  // Parameter values: power of two, 2 to 256; default is 8.
  // Parameter unit: count
  parameter logic [31:0] PARA_SP_FIFO_DEPTH = 32'd8,
  // Parameter Description: Register stages of the ECC engine of B03.
  // Parameter values: 0 or 1; default is 1.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_ECC_LATENCY = 32'd1,
  // Parameter Description: Entries of the B07 line memory; sets the width of linebuf_fill.
  // Parameter values: power of two, 4 to 32768; default is 2048.
  // Parameter unit: count
  parameter logic [31:0] PARA_LINE_BUFFER_DEPTH = 32'd2048,
  // Parameter Description: Fill level of B07 at which linebuffer_full is raised.
  // Parameter values: 1 to PARA_LINE_BUFFER_DEPTH - 1; default is 1740.
  // Parameter unit: count
  parameter logic [31:0] PARA_FULL_WATERMARK = 32'd1740,
  // Parameter Description: Beats of the B08 stream output FIFO.
  // Parameter values: power of two, 2 to 1024; default is 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_STREAM_FIFO_DEPTH = 32'd16,
  // Parameter Description: Beats of the B08 DMA output FIFO.
  // Parameter values: power of two, 2 to 1024; default is 8.
  // Parameter unit: count
  parameter logic [31:0] PARA_DMA_FIFO_DEPTH = 32'd8,
  // Parameter Description: Timestamp increment of B09 per sys_clk cycle, unsigned 8.24 fixed point.
  // Parameter values: any 32-bit value; default is 32'h0400_0000 (4 ns).
  // Parameter unit: none
  parameter logic [31:0] PARA_TS_INCR = 32'h0400_0000,
  // Parameter Description: Embedded data extraction slots; shared by B09 and B14.
  // Parameter values: 0, 1 or 2; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_N_EXTRACT = 32'd2,
  // Parameter Description: Entries of the B09 metadata staging FIFO.
  // Parameter values: legal set checked by B09; default is 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_META_FIFO_DEPTH = 32'd16,
  // Parameter Description: Synchroniser flops per crossing: B11, B13, B14, Async FIFO, core resets.
  // Parameter values: 2 or 3; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_SYNC_STAGES = 32'd2,
  // Parameter Description: sys_clk cycles for which B11 keeps event capture blocked after a reset.
  // Parameter values: 2 x (PARA_SYNC_STAGES + 2) + 1 to 65535; default is 256.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_RST_TAIL = 32'd256,
  // Parameter Description: Width of the AXI4 write data bus of B13 and of the core.
  // Parameter values: 32 or 64; default is 64.
  // Parameter unit: bit
  parameter logic [31:0] PARA_AXI_DATA_WIDTH = 32'd64,
  // Parameter Description: Width of the AXI4 write address of B13 and of the core.
  // Parameter values: 32 in this release; default is 32.
  // Parameter unit: bit
  parameter logic [31:0] PARA_AXI_ADDR_WIDTH = 32'd32,
  // Parameter Description: Width of the AXI4 write identifiers of B13 and of the core.
  // Parameter values: at least 2; default is 2.
  // Parameter unit: bit
  parameter logic [31:0] PARA_AXI_ID_WIDTH = 32'd2,
  // Parameter Description: Largest AXI4 burst issued by B13.
  // Parameter values: legal set checked by B13; default is 256.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_BURST_BEATS = 32'd256,
  // Parameter Description: AXI4 writes in flight in B13; read back through B14.
  // Parameter values: power of two; default is 8.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_OUTSTANDING = 32'd8,
  // Parameter Description: Beats of the B13 RAW data FIFO.
  // Parameter values: power of two; default is 512.
  // Parameter unit: count
  parameter logic [31:0] PARA_RAW_FIFO_DEPTH = 32'd512,
  // Parameter Description: Beats of the B13 metadata data FIFO.
  // Parameter values: power of two, >= 2 x PARA_META_MAX_BURST_BEATS; default is 64.
  // Parameter unit: count
  parameter logic [31:0] PARA_META_DATA_FIFO_DEPTH = 32'd64,
  // Parameter Description: Burst commands per stream in B13.
  // Parameter values: power of two; default is 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_CMD_FIFO_DEPTH = 32'd16,
  // Parameter Description: Largest metadata burst of B13.
  // Parameter values: legal set checked by B13; default is 32.
  // Parameter unit: count
  parameter logic [31:0] PARA_META_MAX_BURST_BEATS = 32'd32,
  // Parameter Description: 1: mm_aclk asynchronous to sys_clk; 0: one clock, no mm_aclk reset sync.
  // Parameter values: 0 or 1; default is 1.
  // Parameter unit: none
  parameter logic [31:0] PARA_ASYNC_MM_CLK = 32'd1,
  // Parameter Description: lite_aclk cycles of the B14 phy_core_rst pulse (at least 40 core_clk).
  // Parameter values: legal set checked by B14; default is 40.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_PHY_RST_CYCLES = 32'd40,
  // Parameter Description: Upper bound of the IRQ_STATUS write response hold in B14.
  // Parameter values: legal set checked by B14; default is 16.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_IRQ_CLR_TIMEOUT = 32'd16
) (
  // Clocks
  input  wire                              i_csi2rx_rxbyteclkhs,
  input  wire                              i_csi2rx_sys_clk,
  input  wire                              i_csi2rx_mm_aclk,
  input  wire                              i_csi2rx_lite_aclk,

  // Resets: asynchronous, active low
  input  wire                              i_csi2rx_reset_n,
  input  wire                              i_csi2rx_lite_areset_n,

  // PPI from the D-PHY
  input  wire                              i_csi2rx_cl_stopstate,
  input  wire                              i_csi2rx_stopstate_0,
  input  wire                              i_csi2rx_stopstate_1,
  input  wire                              i_csi2rx_stopstate_2,
  input  wire                              i_csi2rx_stopstate_3,
  input  wire                              i_csi2rx_cl_ulpsactivenot,
  input  wire                              i_csi2rx_ulpsactivenot_0,
  input  wire                              i_csi2rx_ulpsactivenot_1,
  input  wire                              i_csi2rx_ulpsactivenot_2,
  input  wire                              i_csi2rx_ulpsactivenot_3,
  input  wire                              i_csi2rx_cl_rxclkactivehs,
  input  wire                              i_csi2rx_cl_ulpsclknot,
  input  wire  [7:0]                       i_csi2rx_rxdatahs_0,
  input  wire  [7:0]                       i_csi2rx_rxdatahs_1,
  input  wire  [7:0]                       i_csi2rx_rxdatahs_2,
  input  wire  [7:0]                       i_csi2rx_rxdatahs_3,
  input  wire                              i_csi2rx_rxvalidhs_0,
  input  wire                              i_csi2rx_rxvalidhs_1,
  input  wire                              i_csi2rx_rxvalidhs_2,
  input  wire                              i_csi2rx_rxvalidhs_3,
  input  wire                              i_csi2rx_rxactivehs_0,
  input  wire                              i_csi2rx_rxactivehs_1,
  input  wire                              i_csi2rx_rxactivehs_2,
  input  wire                              i_csi2rx_rxactivehs_3,
  input  wire                              i_csi2rx_rxsynchs_0,
  input  wire                              i_csi2rx_rxsynchs_1,
  input  wire                              i_csi2rx_rxsynchs_2,
  input  wire                              i_csi2rx_rxsynchs_3,
  input  wire                              i_csi2rx_rxskewcalhs_0,
  input  wire                              i_csi2rx_rxskewcalhs_1,
  input  wire                              i_csi2rx_rxskewcalhs_2,
  input  wire                              i_csi2rx_rxskewcalhs_3,
  input  wire                              i_csi2rx_rxclkesc_0,
  input  wire                              i_csi2rx_rxclkesc_1,
  input  wire                              i_csi2rx_rxclkesc_2,
  input  wire                              i_csi2rx_rxclkesc_3,
  input  wire                              i_csi2rx_rxlpdtesc_0,
  input  wire                              i_csi2rx_rxlpdtesc_1,
  input  wire                              i_csi2rx_rxlpdtesc_2,
  input  wire                              i_csi2rx_rxlpdtesc_3,
  input  wire                              i_csi2rx_rxulpsesc_0,
  input  wire                              i_csi2rx_rxulpsesc_1,
  input  wire                              i_csi2rx_rxulpsesc_2,
  input  wire                              i_csi2rx_rxulpsesc_3,
  input  wire  [3:0]                       i_csi2rx_rxtriggeresc_0,
  input  wire  [3:0]                       i_csi2rx_rxtriggeresc_1,
  input  wire  [3:0]                       i_csi2rx_rxtriggeresc_2,
  input  wire  [3:0]                       i_csi2rx_rxtriggeresc_3,
  input  wire  [7:0]                       i_csi2rx_rxdataesc_0,
  input  wire  [7:0]                       i_csi2rx_rxdataesc_1,
  input  wire  [7:0]                       i_csi2rx_rxdataesc_2,
  input  wire  [7:0]                       i_csi2rx_rxdataesc_3,
  input  wire                              i_csi2rx_rxvalidesc_0,
  input  wire                              i_csi2rx_rxvalidesc_1,
  input  wire                              i_csi2rx_rxvalidesc_2,
  input  wire                              i_csi2rx_rxvalidesc_3,
  input  wire                              i_csi2rx_errsoths_0,
  input  wire                              i_csi2rx_errsoths_1,
  input  wire                              i_csi2rx_errsoths_2,
  input  wire                              i_csi2rx_errsoths_3,
  input  wire                              i_csi2rx_errsotsynchs_0,
  input  wire                              i_csi2rx_errsotsynchs_1,
  input  wire                              i_csi2rx_errsotsynchs_2,
  input  wire                              i_csi2rx_errsotsynchs_3,
  input  wire                              i_csi2rx_erresc_0,
  input  wire                              i_csi2rx_erresc_1,
  input  wire                              i_csi2rx_erresc_2,
  input  wire                              i_csi2rx_erresc_3,
  input  wire                              i_csi2rx_errsyncesc_0,
  input  wire                              i_csi2rx_errsyncesc_1,
  input  wire                              i_csi2rx_errsyncesc_2,
  input  wire                              i_csi2rx_errsyncesc_3,
  input  wire                              i_csi2rx_errcontrol_0,
  input  wire                              i_csi2rx_errcontrol_1,
  input  wire                              i_csi2rx_errcontrol_2,
  input  wire                              i_csi2rx_errcontrol_3,
  input  wire                              i_csi2rx_init_done,

  // PPI to the D-PHY
  output logic                             o_csi2rx_cl_enable,
  output logic                             o_csi2rx_dl_enable_0,
  output logic                             o_csi2rx_dl_enable_1,
  output logic                             o_csi2rx_dl_enable_2,
  output logic                             o_csi2rx_dl_enable_3,
  output logic                             o_csi2rx_forcerxmode_0,
  output logic                             o_csi2rx_forcerxmode_1,
  output logic                             o_csi2rx_forcerxmode_2,
  output logic                             o_csi2rx_forcerxmode_3,
  output logic                             o_csi2rx_phy_core_rst,

  // AXI4-Stream master to the ISP (axis_aclk = sys_clk)
  output logic [63:0]                      o_csi2rx_axis_tdata,
  output logic                             o_csi2rx_axis_tvalid,
  input  wire                              i_csi2rx_axis_tready,
  output logic                             o_csi2rx_axis_tlast,
  output logic [1:0]                       o_csi2rx_axis_tuser,
  output logic [1:0]                       o_csi2rx_axis_tid,
  output logic [7:0]                       o_csi2rx_axis_tkeep,

  // AXI4 write-only manager to DDR (mm_aclk)
  output logic [PARA_AXI_ADDR_WIDTH-1:0]   o_csi2rx_axi_awaddr,
  output logic [PARA_AXI_ID_WIDTH-1:0]     o_csi2rx_axi_awid,
  output logic [7:0]                       o_csi2rx_axi_awlen,
  output logic [1:0]                       o_csi2rx_axi_awburst,
  output logic                             o_csi2rx_axi_awlock,
  output logic [2:0]                       o_csi2rx_axi_awsize,
  output logic [3:0]                       o_csi2rx_axi_awcache,
  output logic [2:0]                       o_csi2rx_axi_awprot,
  output logic [3:0]                       o_csi2rx_axi_awqos,
  input  wire                              i_csi2rx_axi_awready,
  output logic                             o_csi2rx_axi_awvalid,
  output logic [PARA_AXI_DATA_WIDTH-1:0]   o_csi2rx_axi_wdata,
  output logic                             o_csi2rx_axi_wlast,
  output logic [PARA_AXI_DATA_WIDTH/8-1:0] o_csi2rx_axi_wstrb,
  input  wire                              i_csi2rx_axi_wready,
  output logic                             o_csi2rx_axi_wvalid,
  input  wire  [PARA_AXI_ID_WIDTH-1:0]     i_csi2rx_axi_bid,
  input  wire  [1:0]                       i_csi2rx_axi_bresp,
  input  wire                              i_csi2rx_axi_bvalid,
  output logic                             o_csi2rx_axi_bready,

  // AXI4-Lite subordinate from the host (lite_aclk)
  input  wire  [11:0]                      i_csi2rx_axil_awaddr,
  input  wire  [2:0]                       i_csi2rx_axil_awprot,
  input  wire                              i_csi2rx_axil_awvalid,
  output logic                             o_csi2rx_axil_awready,
  input  wire  [31:0]                      i_csi2rx_axil_wdata,
  input  wire  [3:0]                       i_csi2rx_axil_wstrb,
  input  wire                              i_csi2rx_axil_wvalid,
  output logic                             o_csi2rx_axil_wready,
  input  wire                              i_csi2rx_axil_bready,
  output logic [1:0]                       o_csi2rx_axil_bresp,
  output logic                             o_csi2rx_axil_bvalid,
  input  wire  [11:0]                      i_csi2rx_axil_araddr,
  input  wire  [2:0]                       i_csi2rx_axil_arprot,
  input  wire                              i_csi2rx_axil_arvalid,
  output logic                             o_csi2rx_axil_arready,
  input  wire                              i_csi2rx_axil_rready,
  output logic [31:0]                      o_csi2rx_axil_rdata,
  output logic [1:0]                       o_csi2rx_axil_rresp,
  output logic                             o_csi2rx_axil_rvalid,

  // Interrupt (sys_clk)
  output logic                             o_csi2rx_int_irq
);

  // ------------------------------------------------------------------------------------------
  // Core
  // ------------------------------------------------------------------------------------------
  // Every wrapper port connects to the core port of the same signal; resets stay active low.
  m_fpt_csi2rx_core #(
    .PARA_MAX_LANES            (PARA_MAX_LANES),
    .PARA_SKEW_DEPTH           (PARA_SKEW_DEPTH),
    .PARA_FIFO_AF_MARGIN       (PARA_FIFO_AF_MARGIN),
    .PARA_ASYNC_FIFO_DEPTH     (PARA_ASYNC_FIFO_DEPTH),
    .PARA_SP_FIFO_DEPTH        (PARA_SP_FIFO_DEPTH),
    .PARA_ECC_LATENCY          (PARA_ECC_LATENCY),
    .PARA_LINE_BUFFER_DEPTH    (PARA_LINE_BUFFER_DEPTH),
    .PARA_FULL_WATERMARK       (PARA_FULL_WATERMARK),
    .PARA_STREAM_FIFO_DEPTH    (PARA_STREAM_FIFO_DEPTH),
    .PARA_DMA_FIFO_DEPTH       (PARA_DMA_FIFO_DEPTH),
    .PARA_TS_INCR              (PARA_TS_INCR),
    .PARA_N_EXTRACT            (PARA_N_EXTRACT),
    .PARA_META_FIFO_DEPTH      (PARA_META_FIFO_DEPTH),
    .PARA_SYNC_STAGES          (PARA_SYNC_STAGES),
    .PARA_RST_TAIL             (PARA_RST_TAIL),
    .PARA_AXI_DATA_WIDTH       (PARA_AXI_DATA_WIDTH),
    .PARA_AXI_ADDR_WIDTH       (PARA_AXI_ADDR_WIDTH),
    .PARA_AXI_ID_WIDTH         (PARA_AXI_ID_WIDTH),
    .PARA_MAX_BURST_BEATS      (PARA_MAX_BURST_BEATS),
    .PARA_MAX_OUTSTANDING      (PARA_MAX_OUTSTANDING),
    .PARA_RAW_FIFO_DEPTH       (PARA_RAW_FIFO_DEPTH),
    .PARA_META_DATA_FIFO_DEPTH (PARA_META_DATA_FIFO_DEPTH),
    .PARA_CMD_FIFO_DEPTH       (PARA_CMD_FIFO_DEPTH),
    .PARA_META_MAX_BURST_BEATS (PARA_META_MAX_BURST_BEATS),
    .PARA_ASYNC_MM_CLK         (PARA_ASYNC_MM_CLK),
    .PARA_PHY_RST_CYCLES       (PARA_PHY_RST_CYCLES),
    .PARA_IRQ_CLR_TIMEOUT      (PARA_IRQ_CLR_TIMEOUT)
  ) u_csi2rx_core_0 (
    .i_rxbyteclkhs      (i_csi2rx_rxbyteclkhs),
    .i_sys_clk          (i_csi2rx_sys_clk),
    .i_mm_aclk          (i_csi2rx_mm_aclk),
    .i_lite_aclk        (i_csi2rx_lite_aclk),
    .i_resetn           (i_csi2rx_reset_n),
    .i_lite_aresetn     (i_csi2rx_lite_areset_n),
    .i_cl_stopstate     (i_csi2rx_cl_stopstate),
    .i_stopstate_0      (i_csi2rx_stopstate_0),
    .i_stopstate_1      (i_csi2rx_stopstate_1),
    .i_stopstate_2      (i_csi2rx_stopstate_2),
    .i_stopstate_3      (i_csi2rx_stopstate_3),
    .i_cl_ulpsactivenot (i_csi2rx_cl_ulpsactivenot),
    .i_ulpsactivenot_0  (i_csi2rx_ulpsactivenot_0),
    .i_ulpsactivenot_1  (i_csi2rx_ulpsactivenot_1),
    .i_ulpsactivenot_2  (i_csi2rx_ulpsactivenot_2),
    .i_ulpsactivenot_3  (i_csi2rx_ulpsactivenot_3),
    .i_cl_rxclkactivehs (i_csi2rx_cl_rxclkactivehs),
    .i_cl_ulpsclknot    (i_csi2rx_cl_ulpsclknot),
    .i_rxdatahs_0       (i_csi2rx_rxdatahs_0),
    .i_rxdatahs_1       (i_csi2rx_rxdatahs_1),
    .i_rxdatahs_2       (i_csi2rx_rxdatahs_2),
    .i_rxdatahs_3       (i_csi2rx_rxdatahs_3),
    .i_rxvalidhs_0      (i_csi2rx_rxvalidhs_0),
    .i_rxvalidhs_1      (i_csi2rx_rxvalidhs_1),
    .i_rxvalidhs_2      (i_csi2rx_rxvalidhs_2),
    .i_rxvalidhs_3      (i_csi2rx_rxvalidhs_3),
    .i_rxactivehs_0     (i_csi2rx_rxactivehs_0),
    .i_rxactivehs_1     (i_csi2rx_rxactivehs_1),
    .i_rxactivehs_2     (i_csi2rx_rxactivehs_2),
    .i_rxactivehs_3     (i_csi2rx_rxactivehs_3),
    .i_rxsynchs_0       (i_csi2rx_rxsynchs_0),
    .i_rxsynchs_1       (i_csi2rx_rxsynchs_1),
    .i_rxsynchs_2       (i_csi2rx_rxsynchs_2),
    .i_rxsynchs_3       (i_csi2rx_rxsynchs_3),
    .i_rxskewcalhs_0    (i_csi2rx_rxskewcalhs_0),
    .i_rxskewcalhs_1    (i_csi2rx_rxskewcalhs_1),
    .i_rxskewcalhs_2    (i_csi2rx_rxskewcalhs_2),
    .i_rxskewcalhs_3    (i_csi2rx_rxskewcalhs_3),
    .i_rxclkesc_0       (i_csi2rx_rxclkesc_0),
    .i_rxclkesc_1       (i_csi2rx_rxclkesc_1),
    .i_rxclkesc_2       (i_csi2rx_rxclkesc_2),
    .i_rxclkesc_3       (i_csi2rx_rxclkesc_3),
    .i_rxlpdtesc_0      (i_csi2rx_rxlpdtesc_0),
    .i_rxlpdtesc_1      (i_csi2rx_rxlpdtesc_1),
    .i_rxlpdtesc_2      (i_csi2rx_rxlpdtesc_2),
    .i_rxlpdtesc_3      (i_csi2rx_rxlpdtesc_3),
    .i_rxulpsesc_0      (i_csi2rx_rxulpsesc_0),
    .i_rxulpsesc_1      (i_csi2rx_rxulpsesc_1),
    .i_rxulpsesc_2      (i_csi2rx_rxulpsesc_2),
    .i_rxulpsesc_3      (i_csi2rx_rxulpsesc_3),
    .i_rxtriggeresc_0   (i_csi2rx_rxtriggeresc_0),
    .i_rxtriggeresc_1   (i_csi2rx_rxtriggeresc_1),
    .i_rxtriggeresc_2   (i_csi2rx_rxtriggeresc_2),
    .i_rxtriggeresc_3   (i_csi2rx_rxtriggeresc_3),
    .i_rxdataesc_0      (i_csi2rx_rxdataesc_0),
    .i_rxdataesc_1      (i_csi2rx_rxdataesc_1),
    .i_rxdataesc_2      (i_csi2rx_rxdataesc_2),
    .i_rxdataesc_3      (i_csi2rx_rxdataesc_3),
    .i_rxvalidesc_0     (i_csi2rx_rxvalidesc_0),
    .i_rxvalidesc_1     (i_csi2rx_rxvalidesc_1),
    .i_rxvalidesc_2     (i_csi2rx_rxvalidesc_2),
    .i_rxvalidesc_3     (i_csi2rx_rxvalidesc_3),
    .i_errsoths_0       (i_csi2rx_errsoths_0),
    .i_errsoths_1       (i_csi2rx_errsoths_1),
    .i_errsoths_2       (i_csi2rx_errsoths_2),
    .i_errsoths_3       (i_csi2rx_errsoths_3),
    .i_errsotsynchs_0   (i_csi2rx_errsotsynchs_0),
    .i_errsotsynchs_1   (i_csi2rx_errsotsynchs_1),
    .i_errsotsynchs_2   (i_csi2rx_errsotsynchs_2),
    .i_errsotsynchs_3   (i_csi2rx_errsotsynchs_3),
    .i_erresc_0         (i_csi2rx_erresc_0),
    .i_erresc_1         (i_csi2rx_erresc_1),
    .i_erresc_2         (i_csi2rx_erresc_2),
    .i_erresc_3         (i_csi2rx_erresc_3),
    .i_errsyncesc_0     (i_csi2rx_errsyncesc_0),
    .i_errsyncesc_1     (i_csi2rx_errsyncesc_1),
    .i_errsyncesc_2     (i_csi2rx_errsyncesc_2),
    .i_errsyncesc_3     (i_csi2rx_errsyncesc_3),
    .i_errcontrol_0     (i_csi2rx_errcontrol_0),
    .i_errcontrol_1     (i_csi2rx_errcontrol_1),
    .i_errcontrol_2     (i_csi2rx_errcontrol_2),
    .i_errcontrol_3     (i_csi2rx_errcontrol_3),
    .i_init_done        (i_csi2rx_init_done),
    .o_cl_enable        (o_csi2rx_cl_enable),
    .o_dl_enable_0      (o_csi2rx_dl_enable_0),
    .o_dl_enable_1      (o_csi2rx_dl_enable_1),
    .o_dl_enable_2      (o_csi2rx_dl_enable_2),
    .o_dl_enable_3      (o_csi2rx_dl_enable_3),
    .o_forcerxmode_0    (o_csi2rx_forcerxmode_0),
    .o_forcerxmode_1    (o_csi2rx_forcerxmode_1),
    .o_forcerxmode_2    (o_csi2rx_forcerxmode_2),
    .o_forcerxmode_3    (o_csi2rx_forcerxmode_3),
    .o_phy_core_rst     (o_csi2rx_phy_core_rst),
    .o_m_axis_tdata     (o_csi2rx_axis_tdata),
    .o_m_axis_tvalid    (o_csi2rx_axis_tvalid),
    .i_m_axis_tready    (i_csi2rx_axis_tready),
    .o_m_axis_tlast     (o_csi2rx_axis_tlast),
    .o_m_axis_tuser     (o_csi2rx_axis_tuser),
    .o_m_axis_tid       (o_csi2rx_axis_tid),
    .o_m_axis_tkeep     (o_csi2rx_axis_tkeep),
    .o_m_axi4_awaddr    (o_csi2rx_axi_awaddr),
    .o_m_axi4_awid      (o_csi2rx_axi_awid),
    .o_m_axi4_awlen     (o_csi2rx_axi_awlen),
    .o_m_axi4_awburst   (o_csi2rx_axi_awburst),
    .o_m_axi4_awlock    (o_csi2rx_axi_awlock),
    .o_m_axi4_awsize    (o_csi2rx_axi_awsize),
    .o_m_axi4_awcache   (o_csi2rx_axi_awcache),
    .o_m_axi4_awprot    (o_csi2rx_axi_awprot),
    .o_m_axi4_awqos     (o_csi2rx_axi_awqos),
    .i_m_axi4_awready   (i_csi2rx_axi_awready),
    .o_m_axi4_awvalid   (o_csi2rx_axi_awvalid),
    .o_m_axi4_wdata     (o_csi2rx_axi_wdata),
    .o_m_axi4_wlast     (o_csi2rx_axi_wlast),
    .o_m_axi4_wstrb     (o_csi2rx_axi_wstrb),
    .i_m_axi4_wready    (i_csi2rx_axi_wready),
    .o_m_axi4_wvalid    (o_csi2rx_axi_wvalid),
    .i_m_axi4_bid       (i_csi2rx_axi_bid),
    .i_m_axi4_bresp     (i_csi2rx_axi_bresp),
    .i_m_axi4_bvalid    (i_csi2rx_axi_bvalid),
    .o_m_axi4_bready    (o_csi2rx_axi_bready),
    .i_s_axi_awaddr     (i_csi2rx_axil_awaddr),
    .i_s_axi_awprot     (i_csi2rx_axil_awprot),
    .i_s_axi_awvalid    (i_csi2rx_axil_awvalid),
    .o_s_axi_awready    (o_csi2rx_axil_awready),
    .i_s_axi_wdata      (i_csi2rx_axil_wdata),
    .i_s_axi_wstrb      (i_csi2rx_axil_wstrb),
    .i_s_axi_wvalid     (i_csi2rx_axil_wvalid),
    .o_s_axi_wready     (o_csi2rx_axil_wready),
    .i_s_axi_bready     (i_csi2rx_axil_bready),
    .o_s_axi_bresp      (o_csi2rx_axil_bresp),
    .o_s_axi_bvalid     (o_csi2rx_axil_bvalid),
    .i_s_axi_araddr     (i_csi2rx_axil_araddr),
    .i_s_axi_arprot     (i_csi2rx_axil_arprot),
    .i_s_axi_arvalid    (i_csi2rx_axil_arvalid),
    .o_s_axi_arready    (o_csi2rx_axil_arready),
    .i_s_axi_rready     (i_csi2rx_axil_rready),
    .o_s_axi_rdata      (o_csi2rx_axil_rdata),
    .o_s_axi_rresp      (o_csi2rx_axil_rresp),
    .o_s_axi_rvalid     (o_csi2rx_axil_rvalid),
    .o_csi2_irq         (o_csi2rx_int_irq)
  );

endmodule

`default_nettype wire
