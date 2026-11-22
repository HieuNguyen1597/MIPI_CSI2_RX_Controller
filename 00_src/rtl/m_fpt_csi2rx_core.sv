// Module      : m_fpt_csi2rx_core
// Block       : TOP - MIPI CSI-2 RX Controller core
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-22
// Description : CSI-2 RX core: 14 blocks, B02-B03 Async FIFO and datapath reset synchronisers
`timescale 1ns/1ps
`default_nettype none

module m_fpt_csi2rx_core #(
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
  input  wire                              i_rxbyteclkhs,
  input  wire                              i_sys_clk,
  input  wire                              i_mm_aclk,
  input  wire                              i_lite_aclk,

  // Resets: asynchronous, active low
  input  wire                              i_resetn,
  input  wire                              i_lite_aresetn,

  // PPI from the D-PHY
  input  wire                              i_cl_stopstate,
  input  wire                              i_stopstate_0,
  input  wire                              i_stopstate_1,
  input  wire                              i_stopstate_2,
  input  wire                              i_stopstate_3,
  input  wire                              i_cl_ulpsactivenot,
  input  wire                              i_ulpsactivenot_0,
  input  wire                              i_ulpsactivenot_1,
  input  wire                              i_ulpsactivenot_2,
  input  wire                              i_ulpsactivenot_3,
  input  wire                              i_cl_rxclkactivehs,
  input  wire                              i_cl_ulpsclknot,
  input  wire  [7:0]                       i_rxdatahs_0,
  input  wire  [7:0]                       i_rxdatahs_1,
  input  wire  [7:0]                       i_rxdatahs_2,
  input  wire  [7:0]                       i_rxdatahs_3,
  input  wire                              i_rxvalidhs_0,
  input  wire                              i_rxvalidhs_1,
  input  wire                              i_rxvalidhs_2,
  input  wire                              i_rxvalidhs_3,
  input  wire                              i_rxactivehs_0,
  input  wire                              i_rxactivehs_1,
  input  wire                              i_rxactivehs_2,
  input  wire                              i_rxactivehs_3,
  input  wire                              i_rxsynchs_0,
  input  wire                              i_rxsynchs_1,
  input  wire                              i_rxsynchs_2,
  input  wire                              i_rxsynchs_3,
  input  wire                              i_rxskewcalhs_0,
  input  wire                              i_rxskewcalhs_1,
  input  wire                              i_rxskewcalhs_2,
  input  wire                              i_rxskewcalhs_3,
  input  wire                              i_rxclkesc_0,
  input  wire                              i_rxclkesc_1,
  input  wire                              i_rxclkesc_2,
  input  wire                              i_rxclkesc_3,
  input  wire                              i_rxlpdtesc_0,
  input  wire                              i_rxlpdtesc_1,
  input  wire                              i_rxlpdtesc_2,
  input  wire                              i_rxlpdtesc_3,
  input  wire                              i_rxulpsesc_0,
  input  wire                              i_rxulpsesc_1,
  input  wire                              i_rxulpsesc_2,
  input  wire                              i_rxulpsesc_3,
  input  wire  [3:0]                       i_rxtriggeresc_0,
  input  wire  [3:0]                       i_rxtriggeresc_1,
  input  wire  [3:0]                       i_rxtriggeresc_2,
  input  wire  [3:0]                       i_rxtriggeresc_3,
  input  wire  [7:0]                       i_rxdataesc_0,
  input  wire  [7:0]                       i_rxdataesc_1,
  input  wire  [7:0]                       i_rxdataesc_2,
  input  wire  [7:0]                       i_rxdataesc_3,
  input  wire                              i_rxvalidesc_0,
  input  wire                              i_rxvalidesc_1,
  input  wire                              i_rxvalidesc_2,
  input  wire                              i_rxvalidesc_3,
  input  wire                              i_errsoths_0,
  input  wire                              i_errsoths_1,
  input  wire                              i_errsoths_2,
  input  wire                              i_errsoths_3,
  input  wire                              i_errsotsynchs_0,
  input  wire                              i_errsotsynchs_1,
  input  wire                              i_errsotsynchs_2,
  input  wire                              i_errsotsynchs_3,
  input  wire                              i_erresc_0,
  input  wire                              i_erresc_1,
  input  wire                              i_erresc_2,
  input  wire                              i_erresc_3,
  input  wire                              i_errsyncesc_0,
  input  wire                              i_errsyncesc_1,
  input  wire                              i_errsyncesc_2,
  input  wire                              i_errsyncesc_3,
  input  wire                              i_errcontrol_0,
  input  wire                              i_errcontrol_1,
  input  wire                              i_errcontrol_2,
  input  wire                              i_errcontrol_3,
  input  wire                              i_init_done,

  // PPI to the D-PHY
  output logic                             o_cl_enable,
  output logic                             o_dl_enable_0,
  output logic                             o_dl_enable_1,
  output logic                             o_dl_enable_2,
  output logic                             o_dl_enable_3,
  output logic                             o_forcerxmode_0,
  output logic                             o_forcerxmode_1,
  output logic                             o_forcerxmode_2,
  output logic                             o_forcerxmode_3,
  output logic                             o_phy_core_rst,

  // AXI4-Stream master to the ISP (axis_aclk = sys_clk)
  output logic [63:0]                      o_m_axis_tdata,
  output logic                             o_m_axis_tvalid,
  input  wire                              i_m_axis_tready,
  output logic                             o_m_axis_tlast,
  output logic [1:0]                       o_m_axis_tuser,
  output logic [1:0]                       o_m_axis_tid,
  output logic [7:0]                       o_m_axis_tkeep,

  // AXI4 write-only manager to DDR (mm_aclk)
  output logic [PARA_AXI_ADDR_WIDTH-1:0]   o_m_axi4_awaddr,
  output logic [PARA_AXI_ID_WIDTH-1:0]     o_m_axi4_awid,
  output logic [7:0]                       o_m_axi4_awlen,
  output logic [1:0]                       o_m_axi4_awburst,
  output logic                             o_m_axi4_awlock,
  output logic [2:0]                       o_m_axi4_awsize,
  output logic [3:0]                       o_m_axi4_awcache,
  output logic [2:0]                       o_m_axi4_awprot,
  output logic [3:0]                       o_m_axi4_awqos,
  input  wire                              i_m_axi4_awready,
  output logic                             o_m_axi4_awvalid,
  output logic [PARA_AXI_DATA_WIDTH-1:0]   o_m_axi4_wdata,
  output logic                             o_m_axi4_wlast,
  output logic [PARA_AXI_DATA_WIDTH/8-1:0] o_m_axi4_wstrb,
  input  wire                              i_m_axi4_wready,
  output logic                             o_m_axi4_wvalid,
  input  wire  [PARA_AXI_ID_WIDTH-1:0]     i_m_axi4_bid,
  input  wire  [1:0]                       i_m_axi4_bresp,
  input  wire                              i_m_axi4_bvalid,
  output logic                             o_m_axi4_bready,

  // AXI4-Lite subordinate from the host (lite_aclk)
  input  wire  [11:0]                      i_s_axi_awaddr,
  input  wire  [2:0]                       i_s_axi_awprot,
  input  wire                              i_s_axi_awvalid,
  output logic                             o_s_axi_awready,
  input  wire  [31:0]                      i_s_axi_wdata,
  input  wire  [3:0]                       i_s_axi_wstrb,
  input  wire                              i_s_axi_wvalid,
  output logic                             o_s_axi_wready,
  input  wire                              i_s_axi_bready,
  output logic [1:0]                       o_s_axi_bresp,
  output logic                             o_s_axi_bvalid,
  input  wire  [11:0]                      i_s_axi_araddr,
  input  wire  [2:0]                       i_s_axi_arprot,
  input  wire                              i_s_axi_arvalid,
  output logic                             o_s_axi_arready,
  input  wire                              i_s_axi_rready,
  output logic [31:0]                      o_s_axi_rdata,
  output logic [1:0]                       o_s_axi_rresp,
  output logic                             o_s_axi_rvalid,

  // Interrupt (sys_clk)
  output logic                             o_csi2_irq
);

  // ------------------------------------------------------------------------------------------
  // Constants, elaboration guards and internal nets
  // ------------------------------------------------------------------------------------------
  // Width of the Line Buffer fill level and number of embedded data slots on the B09/B14 buses
  // (one slot is kept when PARA_N_EXTRACT is 0).
  localparam logic [31:0] LPARA_FILL_CNT_W      = $clog2(PARA_LINE_BUFFER_DEPTH + 32'd1);
  localparam logic [31:0] LPARA_N_EXTRACT_SLOTS = (PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT;

  // Elaboration guards of the core logic: a reset synchroniser needs at least two flops, and the
  // Line Buffer fill level must fit the 16-bit linebuf_fill input of B14.
  if (PARA_SYNC_STAGES < 32'd2) begin : gen_guard_sync_stages
    $error("PARA_SYNC_STAGES must be at least 2");
  end
  if (LPARA_FILL_CNT_W > 32'd16) begin : gen_guard_fill_cnt_w
    $error("PARA_LINE_BUFFER_DEPTH gives a linebuf_fill wider than 16 bits");
  end

  // Datapath reset: merged asynchronous reset and the reset of each clock domain
  logic w_rst_async_n;
  logic [PARA_SYNC_STAGES-1:0] reg_rxbyteclkhs_rst_sync;
  logic [PARA_SYNC_STAGES-1:0] reg_sys_clk_rst_sync;
  logic w_rxbyteclkhs_resetn;
  logic w_sys_resetn;
  logic w_mm_resetn;

  // Outputs of B01 PHY Protocol Interface (PPI)
  logic       w_b01_cl_stopstate;
  logic       w_b01_stopstate_0;
  logic       w_b01_stopstate_1;
  logic       w_b01_stopstate_2;
  logic       w_b01_stopstate_3;
  logic       w_b01_cl_ulpsactivenot;
  logic       w_b01_rxbyteclkhs;
  logic       w_b01_cl_rxclkactivehs;
  logic       w_b01_cl_ulpsclknot;
  logic [7:0] w_b01_rxdatahs_0;
  logic [7:0] w_b01_rxdatahs_1;
  logic [7:0] w_b01_rxdatahs_2;
  logic [7:0] w_b01_rxdatahs_3;
  logic       w_b01_rxvalidhs_0;
  logic       w_b01_rxvalidhs_1;
  logic       w_b01_rxvalidhs_2;
  logic       w_b01_rxvalidhs_3;
  logic       w_b01_rxactivehs_0;
  logic       w_b01_rxactivehs_1;
  logic       w_b01_rxactivehs_2;
  logic       w_b01_rxactivehs_3;
  logic       w_b01_rxsynchs_0;
  logic       w_b01_rxsynchs_1;
  logic       w_b01_rxsynchs_2;
  logic       w_b01_rxsynchs_3;
  logic       w_b01_rxskewcalhs_0;
  logic       w_b01_rxskewcalhs_1;
  logic       w_b01_rxskewcalhs_2;
  logic       w_b01_rxskewcalhs_3;
  logic       w_b01_errsoths_0;
  logic       w_b01_errsoths_1;
  logic       w_b01_errsoths_2;
  logic       w_b01_errsoths_3;
  logic       w_b01_errsotsynchs_0;
  logic       w_b01_errsotsynchs_1;
  logic       w_b01_errsotsynchs_2;
  logic       w_b01_errsotsynchs_3;
  logic       w_b01_init_done;

  // Outputs of B02 Lane Management
  logic [31:0] w_b02_merged_data;
  logic        w_b02_merged_valid;
  logic [3:0]  w_b02_merged_byte_valid;
  logic        w_b02_merged_sob;
  logic        w_b02_merged_eob;
  logic [2:0]  w_b02_lm_fsm_state;
  logic        w_b02_abort_sync_err;
  logic        w_b02_abort_fifo_full;
  logic        w_b02_abort_unarmed;
  logic [3:0]  w_b02_abort_lane_sync_seen;
  logic        w_b02_lane_abort_pulse;
  logic        w_b02_deskew_burst_seen;
  logic [1:0]  w_b02_active_lanes_applied;

  // Outputs of Async FIFO (B02 to B03)
  logic [37:0] w_async_fifo_rd_data;
  logic        w_async_fifo_rd_empty;

  // Outputs of B03 Packet Analyzer
  logic        w_b03_rd_en;
  logic [31:0] w_b03_bp_data;
  logic [3:0]  w_b03_bp_byte_en;
  logic        w_b03_bp_valid;
  logic        w_b03_bp_sof;
  logic        w_b03_bp_sol;
  logic        w_b03_bp_eol;
  logic [1:0]  w_b03_bp_vc_tag;
  logic        w_b03_bp_dt_sel;
  logic [31:0] w_b03_emb_data;
  logic [3:0]  w_b03_emb_byte_en;
  logic        w_b03_emb_valid;
  logic        w_b03_emb_sol;
  logic        w_b03_emb_eol;
  logic [1:0]  w_b03_emb_vc;
  logic        w_b03_fc_fs;
  logic        w_b03_fc_fe;
  logic [15:0] w_b03_fc_frame_num;
  logic [1:0]  w_b03_fc_vc;
  logic [23:0] w_b03_gsp_data;
  logic        w_b03_sp_fifo_not_empty;
  logic        w_b03_sp_fifo_full;
  logic        w_b03_sp_fifo_overflow;
  logic        w_b03_null_blank_seen;
  logic        w_b03_packet_done_pulse;
  logic        w_b03_err_ecc_1bit;
  logic        w_b03_err_ecc_2bit;
  logic        w_b03_err_crc;
  logic        w_b03_err_wc;
  logic        w_b03_err_unsupported_dt;
  logic [1:0]  w_b03_err_vc;
  logic        w_b03_err_lane_abort;
  logic        w_b03_err_frame_sync;
  logic        w_b03_err_frame_level;
  logic        w_b03_vc_unsupported_seen;
  logic        w_b03_dt_conflict;

  // Outputs of B04 Byte-to-Pixel Unpacking
  logic [47:0] w_b04_px_data;
  logic        w_b04_px_valid;
  logic [2:0]  w_b04_px_valid_count;
  logic        w_b04_px_sof;
  logic        w_b04_px_sol;
  logic        w_b04_px_eol;
  logic [1:0]  w_b04_px_vc_tag;
  logic        w_b04_px_dt_sel;
  logic        w_b04_err_wc_align;

  // Outputs of B05 Test Pattern Generator
  logic [47:0] w_b05_px_data;
  logic        w_b05_px_valid;
  logic        w_b05_px_sof;
  logic        w_b05_px_sol;
  logic        w_b05_px_eol;
  logic [1:0]  w_b05_px_vc_tag;
  logic        w_b05_px_dt_sel;
  logic        w_b05_tpg_eof;
  logic        w_b05_tpg_fs;
  logic        w_b05_tpg_fe;
  logic [1:0]  w_b05_tpg_fvc;
  logic [15:0] w_b05_tpg_height_shadow;

  // Outputs of B06 TPG Mux
  logic        w_b06_tpg_px_ready;
  logic [47:0] w_b06_px_data;
  logic        w_b06_px_valid;
  logic [2:0]  w_b06_px_valid_count;
  logic        w_b06_px_sof;
  logic        w_b06_px_sol;
  logic        w_b06_px_eol;
  logic [1:0]  w_b06_px_vc_tag;
  logic        w_b06_px_dt_sel;
  logic        w_b06_err_wc_align;
  logic        w_b06_active_sel;
  logic        w_b06_switch_pending;

  // Outputs of B07 Line Buffer
  logic [47:0]                 w_b07_fb_px_data;
  logic                        w_b07_fb_px_valid;
  logic [2:0]                  w_b07_fb_px_valid_count;
  logic                        w_b07_fb_px_sof;
  logic                        w_b07_fb_px_sol;
  logic                        w_b07_fb_px_eol;
  logic [1:0]                  w_b07_fb_px_vc_tag;
  logic                        w_b07_fb_px_dt_sel;
  logic                        w_b07_fb_err_wc_align;
  logic                        w_b07_linebuffer_full;
  logic [LPARA_FILL_CNT_W-1:0] w_b07_linebuf_fill;
  logic                        w_b07_linebuf_overflow;
  logic                        w_b07_linebuf_empty;

  // Outputs of B08 Format Bridge
  logic        w_b08_fb_ready;
  logic [63:0] w_b08_fbs_data;
  logic        w_b08_fbs_valid;
  logic [7:0]  w_b08_fbs_byte_en;
  logic        w_b08_fbs_sof;
  logic        w_b08_fbs_eol;
  logic [1:0]  w_b08_fbs_vc_tag;
  logic        w_b08_fbs_err;
  logic [63:0] w_b08_fbd_data;
  logic        w_b08_fbd_valid;
  logic [7:0]  w_b08_fbd_byte_en;
  logic        w_b08_fbd_sof;
  logic        w_b08_fbd_sol;
  logic        w_b08_fbd_eol;
  logic [1:0]  w_b08_fbd_vc_tag;
  logic        w_b08_fbd_err;
  logic        w_b08_fb_overflow;

  // Outputs of B09 Frame Context & Metadata Capture
  logic [1:0]                             w_b09_capture_start;
  logic [1:0]                             w_b09_capture_end;
  logic [1:0]                             w_b09_capture_dropped;
  logic                                   w_b09_ev_fs;
  logic [1:0]                             w_b09_ev_vc;
  logic [1:0]                             w_b09_ctx_slot;
  logic [1:0]                             w_b09_ctx_orphan;
  logic                                   w_b09_lead_vc;
  logic [31:0]                            w_b09_frame_count;
  logic [13:0]                            w_b09_frame_id_buf0;
  logic [13:0]                            w_b09_frame_id_buf1;
  logic [63:0]                            w_b09_timestamp;
  logic [15:0]                            w_b09_vc0_line_count;
  logic [15:0]                            w_b09_vc0_byte_count;
  logic [5:0]                             w_b09_vc0_data_type;
  logic                                   w_b09_vc0_width_mismatch;
  logic                                   w_b09_vc0_height_mismatch;
  logic [15:0]                            w_b09_emb_word_count_vc0;
  logic [7:0]                             w_b09_emb_line_count_vc0;
  logic [LPARA_N_EXTRACT_SLOTS-1:0][31:0] w_b09_emb_extract_val_vc0;
  logic [13:0]                            w_b09_cur_frame_id_vc0;
  logic [15:0]                            w_b09_cur_line_vc0;
  logic [15:0]                            w_b09_vc1_line_count;
  logic [15:0]                            w_b09_vc1_byte_count;
  logic [5:0]                             w_b09_vc1_data_type;
  logic                                   w_b09_vc1_width_mismatch;
  logic                                   w_b09_vc1_height_mismatch;
  logic [15:0]                            w_b09_emb_word_count_vc1;
  logic [7:0]                             w_b09_emb_line_count_vc1;
  logic [LPARA_N_EXTRACT_SLOTS-1:0][31:0] w_b09_emb_extract_val_vc1;
  logic [13:0]                            w_b09_cur_frame_id_vc1;
  logic [15:0]                            w_b09_cur_line_vc1;
  logic                                   w_b09_meta_overflow;
  logic                                   w_b09_err_frame_num;
  logic                                   w_b09_err_resolution;
  logic                                   w_b09_capture_done;
  logic [31:0]                            w_b09_meta_data;
  logic [3:0]                             w_b09_meta_byte_en;
  logic                                   w_b09_meta_valid;
  logic                                   w_b09_meta_last;
  logic                                   w_b09_meta_kind;
  logic [1:0]                             w_b09_meta_vc;
  logic                                   w_b09_meta_sof;

  // Outputs of B10 Control FSM
  logic [1:0] w_b10_force_discard;
  logic       w_b10_rx_ctx_clear;
  logic       w_b10_lb_flush;
  logic       w_b10_fb_flush;
  logic       w_b10_capture_drop;
  logic [1:0] w_b10_drop_vc_mask;
  logic       w_b10_asm_abort_req;
  logic [1:0] w_b10_asm_abort_mask;
  logic       w_b10_asm_skip_req;
  logic [1:0] w_b10_asm_skip_mask;
  logic       w_b10_dma_abort_req;
  logic       w_b10_frame_drop_pulse;

  // Outputs of B11 Interrupt & Error Aggregator
  logic [23:0] w_b11_irq_status;
  logic [31:0] w_b11_error_capture;
  logic [31:0] w_b11_ecc_err_cnt;
  logic [31:0] w_b11_crc_err_cnt;
  logic [31:0] w_b11_frame_drop_cnt;
  logic [15:0] w_b11_lane_abort_cnt;
  logic [31:0] w_b11_packet_cnt;
  logic        w_b11_linebuf_ovf_sticky;
  logic        w_b11_sp_fifo_ovf_sticky;
  logic        w_b11_dma_ovf_sticky;

  // Outputs of B12 AXI4-Stream Master
  logic w_b12_fbs_ready;

  // Outputs of B13 Direct Memory Access (DMA)
  logic        w_b13_fbd_ready;
  logic        w_b13_meta_ready;
  logic        w_b13_dma_buf_claim;
  logic        w_b13_dma_buf_index;
  logic        w_b13_dma_drop;
  logic        w_b13_dma_buf_done_ok;
  logic        w_b13_dma_buf_done_err;
  logic        w_b13_dma_axi_write_error;
  logic        w_b13_dma_buffer_overflow;
  logic        w_b13_dma_busy;
  logic        w_b13_active_buffer;
  logic [1:0]  w_b13_buf0_state;
  logic [1:0]  w_b13_buf1_state;
  logic [15:0] w_b13_buf0_last_valid_line;
  logic [15:0] w_b13_buf1_last_valid_line;
  logic [7:0]  w_b13_axi_fault_awid;
  logic [1:0]  w_b13_axi_fault_resp;
  logic [31:0] w_b13_axi_fault_awaddr;
  logic [15:0] w_b13_bresp_slverr_count;
  logic [15:0] w_b13_bresp_decerr_count;

  // Outputs of B14 CSR & AXI4-Lite Subordinate
  logic                                   w_b14_phy_core_rst;
  logic                                   w_b14_cl_enable;
  logic [3:0]                             w_b14_dl_enable;
  logic [3:0]                             w_b14_forcerxmode;
  logic                                   w_b14_full_reset_req;
  logic                                   w_b14_soft_reset_byte;
  logic [1:0]                             w_b14_active_lanes;
  logic                                   w_b14_soft_reset_sys;
  logic                                   w_b14_core_enable;
  logic                                   w_b14_vc_exposure_map;
  logic [5:0]                             w_b14_metadata_dt;
  logic                                   w_b14_metadata_capture_en;
  logic                                   w_b14_fnum_check_en;
  logic                                   w_b14_emb_extract_en;
  logic [15:0]                            w_b14_expected_width;
  logic [15:0]                            w_b14_expected_height;
  logic [31:0]                            w_b14_metadata_record_id;
  logic [LPARA_N_EXTRACT_SLOTS-1:0][31:0] w_b14_emb_extract_cfg;
  logic                                   w_b14_axis_enable;
  logic                                   w_b14_stream_format;
  logic                                   w_b14_stream_follow_dma_drop;
  logic                                   w_b14_tpg_enable;
  logic                                   w_b14_tpg_pattern;
  logic                                   w_b14_tpg_dual_vc_en;
  logic                                   w_b14_tpg_pixel_format;
  logic [1:0]                             w_b14_tpg_cfa_order;
  logic [15:0]                            w_b14_tpg_width;
  logic [15:0]                            w_b14_tpg_height;
  logic [15:0]                            w_b14_tpg_hblank;
  logic [15:0]                            w_b14_tpg_vblank;
  logic                                   w_b14_dma_enable;
  logic                                   w_b14_dma_start;
  logic                                   w_b14_dma_abort;
  logic                                   w_b14_err_frame_policy;
  logic                                   w_b14_dma_format;
  logic [31:0]                            w_b14_dma_buffer0_addr;
  logic [31:0]                            w_b14_dma_buffer1_addr;
  logic [31:0]                            w_b14_dma_buffer_size;
  logic [31:0]                            w_b14_meta_area_size;
  logic [15:0]                            w_b14_bytes_per_line;
  logic [7:0]                             w_b14_line_padding;
  logic [3:0]                             w_b14_awqos;
  logic [7:0]                             w_b14_max_burst_len;
  logic [2:0]                             w_b14_awprot;
  logic [1:0]                             w_b14_release_buf;
  logic                                   w_b14_global_irq_enable;
  logic [23:0]                            w_b14_irq_enable_mask;
  logic [23:0]                            w_b14_irq_clear_strobe;
  logic                                   w_b14_phy_rst_in_progress;
  logic                                   w_b14_gsp_pop;

  // Inputs built by the core: lane vectors, per-lane enables and the 16-bit fill level
  logic [PARA_MAX_LANES-1:0] w_b11_errsoths;
  logic [PARA_MAX_LANES-1:0] w_b11_errsotsynchs;
  logic [3:0]                w_b14_errsoths;
  logic [3:0]                w_b14_errsotsynchs;
  logic [3:0]                w_b14_rxskewcalhs;
  logic [3:0]                w_b14_stopstate;
  logic                      w_b01_dl_enable_0;
  logic                      w_b01_dl_enable_1;
  logic                      w_b01_dl_enable_2;
  logic                      w_b01_dl_enable_3;
  logic                      w_b01_forcerxmode_0;
  logic                      w_b01_forcerxmode_1;
  logic                      w_b01_forcerxmode_2;
  logic                      w_b01_forcerxmode_3;
  logic [15:0]               w_b14_linebuf_fill;

  // ------------------------------------------------------------------------------------------
  // Datapath reset
  // ------------------------------------------------------------------------------------------
  // resetn or the full_reset request of B14 (a lite_aclk flop) resets every datapath domain,
  // at once and without a running clock. B14 itself takes the raw resetn and lite_aresetn.
  always_comb begin : p_com_rst_merge
    w_rst_async_n = i_resetn & ~w_b14_full_reset_req;
  end

  // rxbyteclkhs reset synchroniser: cleared asynchronously, released on the PARA_SYNC_STAGES-th
  // rising edge of the byte clock. Stays in reset while the byte clock is stopped.
  always_ff @(posedge w_b01_rxbyteclkhs or negedge w_rst_async_n) begin : p_ff_rxbyteclkhs_rst_sync
    if (!w_rst_async_n) begin
      reg_rxbyteclkhs_rst_sync <= '0;
    end else begin
      reg_rxbyteclkhs_rst_sync <= {reg_rxbyteclkhs_rst_sync[PARA_SYNC_STAGES-2:0], 1'b1};
    end
  end

  // sys_clk reset synchroniser: same structure in the sys_clk domain.
  always_ff @(posedge i_sys_clk or negedge w_rst_async_n) begin : p_ff_sys_clk_rst_sync
    if (!w_rst_async_n) begin
      reg_sys_clk_rst_sync <= '0;
    end else begin
      reg_sys_clk_rst_sync <= {reg_sys_clk_rst_sync[PARA_SYNC_STAGES-2:0], 1'b1};
    end
  end

  // Domain resets are the last flop of each synchroniser.
  always_comb begin : p_com_domain_rst
    w_rxbyteclkhs_resetn = reg_rxbyteclkhs_rst_sync[PARA_SYNC_STAGES-1];
    w_sys_resetn         = reg_sys_clk_rst_sync[PARA_SYNC_STAGES-1];
  end

  // mm_aclk reset: own synchroniser when mm_aclk is asynchronous to sys_clk; with one clock
  // for both (PARA_ASYNC_MM_CLK = 0) the DMA back end takes the sys_clk domain reset.
  if (PARA_ASYNC_MM_CLK != 32'd0) begin : gen_mm_rst_sync
    logic [PARA_SYNC_STAGES-1:0] reg_mm_aclk_rst_sync;

    always_ff @(posedge i_mm_aclk or negedge w_rst_async_n) begin : p_ff_mm_aclk_rst_sync
      if (!w_rst_async_n) begin
        reg_mm_aclk_rst_sync <= '0;
      end else begin
        reg_mm_aclk_rst_sync <= {reg_mm_aclk_rst_sync[PARA_SYNC_STAGES-2:0], 1'b1};
      end
    end

    always_comb begin : p_com_mm_rst
      w_mm_resetn = reg_mm_aclk_rst_sync[PARA_SYNC_STAGES-1];
    end
  end else begin : gen_mm_rst_shared
    always_comb begin : p_com_mm_rst
      w_mm_resetn = w_sys_resetn;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Lane vectors and Line Buffer fill level
  // ------------------------------------------------------------------------------------------
  // Per-lane status of B01 gathered into vectors, lane x on bit x. B14 always takes the four lanes
  // (B01 drives lanes that are not built to 0); B11 takes the PARA_MAX_LANES lanes that are built.
  always_comb begin : p_com_lane_vec
    w_b14_errsoths     = {w_b01_errsoths_3, w_b01_errsoths_2, w_b01_errsoths_1, w_b01_errsoths_0};
    w_b14_errsotsynchs = {w_b01_errsotsynchs_3, w_b01_errsotsynchs_2, w_b01_errsotsynchs_1, w_b01_errsotsynchs_0};
    w_b14_rxskewcalhs  = {w_b01_rxskewcalhs_3, w_b01_rxskewcalhs_2, w_b01_rxskewcalhs_1, w_b01_rxskewcalhs_0};
    w_b14_stopstate    = {w_b01_stopstate_3, w_b01_stopstate_2, w_b01_stopstate_1, w_b01_stopstate_0};
    w_b11_errsoths     = w_b14_errsoths[PARA_MAX_LANES-1:0];
    w_b11_errsotsynchs = w_b14_errsotsynchs[PARA_MAX_LANES-1:0];
  end

  // Lane enables and forced receive mode of B14, bit x to lane x of B01.
  always_comb begin : p_com_lane_split
    w_b01_dl_enable_0   = w_b14_dl_enable[0];
    w_b01_dl_enable_1   = w_b14_dl_enable[1];
    w_b01_dl_enable_2   = w_b14_dl_enable[2];
    w_b01_dl_enable_3   = w_b14_dl_enable[3];
    w_b01_forcerxmode_0 = w_b14_forcerxmode[0];
    w_b01_forcerxmode_1 = w_b14_forcerxmode[1];
    w_b01_forcerxmode_2 = w_b14_forcerxmode[2];
    w_b01_forcerxmode_3 = w_b14_forcerxmode[3];
  end

  // Fill level of the Line Buffer (LPARA_FILL_CNT_W bits) zero-extended to the 16-bit input of B14.
  always_comb begin : p_com_fill_ext
    w_b14_linebuf_fill = $bits(w_b14_linebuf_fill)'(w_b07_linebuf_fill);
  end

  // ------------------------------------------------------------------------------------------
  // B01 - PHY Protocol Interface (PPI)
  // ------------------------------------------------------------------------------------------
  // D-PHY receive signals per lane, clock lane status and D-PHY control outputs.
  // D-PHY ports pass straight to the block; the escape mode and LP error inputs end here.
  m_fpt_ppi #(
    .PARA_MAX_LANES (PARA_MAX_LANES)
  ) u_ppi_0 (
    .i_cl_enable        (w_b14_cl_enable),
    .i_dl_enable_0      (w_b01_dl_enable_0),
    .i_dl_enable_1      (w_b01_dl_enable_1),
    .i_dl_enable_2      (w_b01_dl_enable_2),
    .i_dl_enable_3      (w_b01_dl_enable_3),
    .i_forcerxmode_0    (w_b01_forcerxmode_0),
    .i_forcerxmode_1    (w_b01_forcerxmode_1),
    .i_forcerxmode_2    (w_b01_forcerxmode_2),
    .i_forcerxmode_3    (w_b01_forcerxmode_3),
    .o_cl_enable        (o_cl_enable),
    .o_dl_enable_0      (o_dl_enable_0),
    .o_dl_enable_1      (o_dl_enable_1),
    .o_dl_enable_2      (o_dl_enable_2),
    .o_dl_enable_3      (o_dl_enable_3),
    .o_forcerxmode_0    (o_forcerxmode_0),
    .o_forcerxmode_1    (o_forcerxmode_1),
    .o_forcerxmode_2    (o_forcerxmode_2),
    .o_forcerxmode_3    (o_forcerxmode_3),
    .i_cl_stopstate     (i_cl_stopstate),
    .i_stopstate_0      (i_stopstate_0),
    .i_stopstate_1      (i_stopstate_1),
    .i_stopstate_2      (i_stopstate_2),
    .i_stopstate_3      (i_stopstate_3),
    .i_cl_ulpsactivenot (i_cl_ulpsactivenot),
    .i_ulpsactivenot_0  (i_ulpsactivenot_0),
    .i_ulpsactivenot_1  (i_ulpsactivenot_1),
    .i_ulpsactivenot_2  (i_ulpsactivenot_2),
    .i_ulpsactivenot_3  (i_ulpsactivenot_3),
    .o_cl_stopstate     (w_b01_cl_stopstate),
    .o_stopstate_0      (w_b01_stopstate_0),
    .o_stopstate_1      (w_b01_stopstate_1),
    .o_stopstate_2      (w_b01_stopstate_2),
    .o_stopstate_3      (w_b01_stopstate_3),
    .o_cl_ulpsactivenot (w_b01_cl_ulpsactivenot),
    .i_rxbyteclkhs      (i_rxbyteclkhs),
    .i_cl_rxclkactivehs (i_cl_rxclkactivehs),
    .i_cl_ulpsclknot    (i_cl_ulpsclknot),
    .o_rxbyteclkhs      (w_b01_rxbyteclkhs),
    .o_cl_rxclkactivehs (w_b01_cl_rxclkactivehs),
    .o_cl_ulpsclknot    (w_b01_cl_ulpsclknot),
    .i_rxdatahs_0       (i_rxdatahs_0),
    .i_rxdatahs_1       (i_rxdatahs_1),
    .i_rxdatahs_2       (i_rxdatahs_2),
    .i_rxdatahs_3       (i_rxdatahs_3),
    .i_rxvalidhs_0      (i_rxvalidhs_0),
    .i_rxvalidhs_1      (i_rxvalidhs_1),
    .i_rxvalidhs_2      (i_rxvalidhs_2),
    .i_rxvalidhs_3      (i_rxvalidhs_3),
    .i_rxactivehs_0     (i_rxactivehs_0),
    .i_rxactivehs_1     (i_rxactivehs_1),
    .i_rxactivehs_2     (i_rxactivehs_2),
    .i_rxactivehs_3     (i_rxactivehs_3),
    .i_rxsynchs_0       (i_rxsynchs_0),
    .i_rxsynchs_1       (i_rxsynchs_1),
    .i_rxsynchs_2       (i_rxsynchs_2),
    .i_rxsynchs_3       (i_rxsynchs_3),
    .i_rxskewcalhs_0    (i_rxskewcalhs_0),
    .i_rxskewcalhs_1    (i_rxskewcalhs_1),
    .i_rxskewcalhs_2    (i_rxskewcalhs_2),
    .i_rxskewcalhs_3    (i_rxskewcalhs_3),
    .o_rxdatahs_0       (w_b01_rxdatahs_0),
    .o_rxdatahs_1       (w_b01_rxdatahs_1),
    .o_rxdatahs_2       (w_b01_rxdatahs_2),
    .o_rxdatahs_3       (w_b01_rxdatahs_3),
    .o_rxvalidhs_0      (w_b01_rxvalidhs_0),
    .o_rxvalidhs_1      (w_b01_rxvalidhs_1),
    .o_rxvalidhs_2      (w_b01_rxvalidhs_2),
    .o_rxvalidhs_3      (w_b01_rxvalidhs_3),
    .o_rxactivehs_0     (w_b01_rxactivehs_0),
    .o_rxactivehs_1     (w_b01_rxactivehs_1),
    .o_rxactivehs_2     (w_b01_rxactivehs_2),
    .o_rxactivehs_3     (w_b01_rxactivehs_3),
    .o_rxsynchs_0       (w_b01_rxsynchs_0),
    .o_rxsynchs_1       (w_b01_rxsynchs_1),
    .o_rxsynchs_2       (w_b01_rxsynchs_2),
    .o_rxsynchs_3       (w_b01_rxsynchs_3),
    .o_rxskewcalhs_0    (w_b01_rxskewcalhs_0),
    .o_rxskewcalhs_1    (w_b01_rxskewcalhs_1),
    .o_rxskewcalhs_2    (w_b01_rxskewcalhs_2),
    .o_rxskewcalhs_3    (w_b01_rxskewcalhs_3),
    .i_rxclkesc_0       (i_rxclkesc_0),
    .i_rxclkesc_1       (i_rxclkesc_1),
    .i_rxclkesc_2       (i_rxclkesc_2),
    .i_rxclkesc_3       (i_rxclkesc_3),
    .i_rxlpdtesc_0      (i_rxlpdtesc_0),
    .i_rxlpdtesc_1      (i_rxlpdtesc_1),
    .i_rxlpdtesc_2      (i_rxlpdtesc_2),
    .i_rxlpdtesc_3      (i_rxlpdtesc_3),
    .i_rxulpsesc_0      (i_rxulpsesc_0),
    .i_rxulpsesc_1      (i_rxulpsesc_1),
    .i_rxulpsesc_2      (i_rxulpsesc_2),
    .i_rxulpsesc_3      (i_rxulpsesc_3),
    .i_rxtriggeresc_0   (i_rxtriggeresc_0),
    .i_rxtriggeresc_1   (i_rxtriggeresc_1),
    .i_rxtriggeresc_2   (i_rxtriggeresc_2),
    .i_rxtriggeresc_3   (i_rxtriggeresc_3),
    .i_rxdataesc_0      (i_rxdataesc_0),
    .i_rxdataesc_1      (i_rxdataesc_1),
    .i_rxdataesc_2      (i_rxdataesc_2),
    .i_rxdataesc_3      (i_rxdataesc_3),
    .i_rxvalidesc_0     (i_rxvalidesc_0),
    .i_rxvalidesc_1     (i_rxvalidesc_1),
    .i_rxvalidesc_2     (i_rxvalidesc_2),
    .i_rxvalidesc_3     (i_rxvalidesc_3),
    .i_errsoths_0       (i_errsoths_0),
    .i_errsoths_1       (i_errsoths_1),
    .i_errsoths_2       (i_errsoths_2),
    .i_errsoths_3       (i_errsoths_3),
    .i_errsotsynchs_0   (i_errsotsynchs_0),
    .i_errsotsynchs_1   (i_errsotsynchs_1),
    .i_errsotsynchs_2   (i_errsotsynchs_2),
    .i_errsotsynchs_3   (i_errsotsynchs_3),
    .i_erresc_0         (i_erresc_0),
    .i_erresc_1         (i_erresc_1),
    .i_erresc_2         (i_erresc_2),
    .i_erresc_3         (i_erresc_3),
    .i_errsyncesc_0     (i_errsyncesc_0),
    .i_errsyncesc_1     (i_errsyncesc_1),
    .i_errsyncesc_2     (i_errsyncesc_2),
    .i_errsyncesc_3     (i_errsyncesc_3),
    .i_errcontrol_0     (i_errcontrol_0),
    .i_errcontrol_1     (i_errcontrol_1),
    .i_errcontrol_2     (i_errcontrol_2),
    .i_errcontrol_3     (i_errcontrol_3),
    .o_errsoths_0       (w_b01_errsoths_0),
    .o_errsoths_1       (w_b01_errsoths_1),
    .o_errsoths_2       (w_b01_errsoths_2),
    .o_errsoths_3       (w_b01_errsoths_3),
    .o_errsotsynchs_0   (w_b01_errsotsynchs_0),
    .o_errsotsynchs_1   (w_b01_errsotsynchs_1),
    .o_errsotsynchs_2   (w_b01_errsotsynchs_2),
    .o_errsotsynchs_3   (w_b01_errsotsynchs_3),
    .i_phy_core_rst     (w_b14_phy_core_rst),
    .i_init_done        (i_init_done),
    .o_phy_core_rst     (o_phy_core_rst),
    .o_init_done        (w_b01_init_done)
  );

  // ------------------------------------------------------------------------------------------
  // B02 - Lane Management
  // ------------------------------------------------------------------------------------------
  // Lane deskew and merge of the active lanes into one 32-bit word stream (rxbyteclkhs).
  m_fpt_lane_management #(
    .PARA_MAX_LANES      (PARA_MAX_LANES),
    .PARA_SKEW_DEPTH     (PARA_SKEW_DEPTH),
    .PARA_FIFO_AF_MARGIN (PARA_FIFO_AF_MARGIN)
  ) u_lane_management_0 (
    .i_rxbyteclkhs          (w_b01_rxbyteclkhs),
    .i_resetn               (w_rxbyteclkhs_resetn),
    .i_soft_reset           (w_b14_soft_reset_byte),
    .i_active_lanes         (w_b14_active_lanes),
    .i_rxdatahs_0           (w_b01_rxdatahs_0),
    .i_rxdatahs_1           (w_b01_rxdatahs_1),
    .i_rxdatahs_2           (w_b01_rxdatahs_2),
    .i_rxdatahs_3           (w_b01_rxdatahs_3),
    .i_rxvalidhs_0          (w_b01_rxvalidhs_0),
    .i_rxvalidhs_1          (w_b01_rxvalidhs_1),
    .i_rxvalidhs_2          (w_b01_rxvalidhs_2),
    .i_rxvalidhs_3          (w_b01_rxvalidhs_3),
    .i_rxactivehs_0         (w_b01_rxactivehs_0),
    .i_rxactivehs_1         (w_b01_rxactivehs_1),
    .i_rxactivehs_2         (w_b01_rxactivehs_2),
    .i_rxactivehs_3         (w_b01_rxactivehs_3),
    .i_rxsynchs_0           (w_b01_rxsynchs_0),
    .i_rxsynchs_1           (w_b01_rxsynchs_1),
    .i_rxsynchs_2           (w_b01_rxsynchs_2),
    .i_rxsynchs_3           (w_b01_rxsynchs_3),
    .i_rxskewcalhs_0        (w_b01_rxskewcalhs_0),
    .i_rxskewcalhs_1        (w_b01_rxskewcalhs_1),
    .i_rxskewcalhs_2        (w_b01_rxskewcalhs_2),
    .i_rxskewcalhs_3        (w_b01_rxskewcalhs_3),
    .i_errsoths_0           (w_b01_errsoths_0),
    .i_errsoths_1           (w_b01_errsoths_1),
    .i_errsoths_2           (w_b01_errsoths_2),
    .i_errsoths_3           (w_b01_errsoths_3),
    .i_errsotsynchs_0       (w_b01_errsotsynchs_0),
    .i_errsotsynchs_1       (w_b01_errsotsynchs_1),
    .i_errsotsynchs_2       (w_b01_errsotsynchs_2),
    .i_errsotsynchs_3       (w_b01_errsotsynchs_3),
    .o_merged_data          (w_b02_merged_data),
    .o_merged_valid         (w_b02_merged_valid),
    .o_merged_byte_valid    (w_b02_merged_byte_valid),
    .o_merged_sob           (w_b02_merged_sob),
    .o_merged_eob           (w_b02_merged_eob),
    .o_lm_fsm_state         (w_b02_lm_fsm_state),
    .o_abort_sync_err       (w_b02_abort_sync_err),
    .o_abort_fifo_full      (w_b02_abort_fifo_full),
    .o_abort_unarmed        (w_b02_abort_unarmed),
    .o_abort_lane_sync_seen (w_b02_abort_lane_sync_seen),
    .o_lane_abort_pulse     (w_b02_lane_abort_pulse),
    .o_deskew_burst_seen    (w_b02_deskew_burst_seen),
    .o_active_lanes_applied (w_b02_active_lanes_applied)
  );

  // ------------------------------------------------------------------------------------------
  // Async FIFO - B02 to B03
  // ------------------------------------------------------------------------------------------
  // Word stream crossing from rxbyteclkhs to sys_clk; read side flushed by core_enable, soft_reset.
  // Write side on rxbyteclkhs, read side on sys_clk; one domain reset per side.
  m_fpt_async_fifo #(
    .PARA_DEPTH       (PARA_ASYNC_FIFO_DEPTH),
    .PARA_SYNC_STAGES (PARA_SYNC_STAGES)
  ) u_async_fifo_0 (
    .i_rxbyteclkhs        (w_b01_rxbyteclkhs),
    .i_resetn_rxbyteclkhs (w_rxbyteclkhs_resetn),
    .i_merged_valid       (w_b02_merged_valid),
    .i_merged_data        (w_b02_merged_data),
    .i_merged_byte_valid  (w_b02_merged_byte_valid),
    .i_merged_sob         (w_b02_merged_sob),
    .i_merged_eob         (w_b02_merged_eob),
    .i_sys_clk            (i_sys_clk),
    .i_resetn_sys_clk     (w_sys_resetn),
    .i_core_enable        (w_b14_core_enable),
    .i_soft_reset         (w_b14_soft_reset_sys),
    .i_rd_en              (w_b03_rd_en),
    .o_rd_data            (w_async_fifo_rd_data),
    .o_rd_empty           (w_async_fifo_rd_empty)
  );

  // ------------------------------------------------------------------------------------------
  // B03 - Packet Analyzer
  // ------------------------------------------------------------------------------------------
  // CSI-2 packet header, ECC and CRC checks; payload, frame control and packet status (sys_clk).
  m_fpt_packet_analyzer #(
    .PARA_SP_FIFO_DEPTH (PARA_SP_FIFO_DEPTH),
    .PARA_ECC_LATENCY   (PARA_ECC_LATENCY)
  ) u_packet_analyzer_0 (
    .i_sys_clk             (i_sys_clk),
    .i_resetn              (w_sys_resetn),
    .i_soft_reset          (w_b14_soft_reset_sys),
    .i_core_enable         (w_b14_core_enable),
    .i_metadata_dt         (w_b14_metadata_dt),
    .i_metadata_capture_en (w_b14_metadata_capture_en),
    .i_rd_data             (w_async_fifo_rd_data),
    .i_rd_empty            (w_async_fifo_rd_empty),
    .o_rd_en               (w_b03_rd_en),
    .i_force_discard       (w_b10_force_discard),
    .o_bp_data             (w_b03_bp_data),
    .o_bp_byte_en          (w_b03_bp_byte_en),
    .o_bp_valid            (w_b03_bp_valid),
    .o_bp_sof              (w_b03_bp_sof),
    .o_bp_sol              (w_b03_bp_sol),
    .o_bp_eol              (w_b03_bp_eol),
    .o_bp_vc_tag           (w_b03_bp_vc_tag),
    .o_bp_dt_sel           (w_b03_bp_dt_sel),
    .o_emb_data            (w_b03_emb_data),
    .o_emb_byte_en         (w_b03_emb_byte_en),
    .o_emb_valid           (w_b03_emb_valid),
    .o_emb_sol             (w_b03_emb_sol),
    .o_emb_eol             (w_b03_emb_eol),
    .o_emb_vc              (w_b03_emb_vc),
    .o_fc_fs               (w_b03_fc_fs),
    .o_fc_fe               (w_b03_fc_fe),
    .o_fc_frame_num        (w_b03_fc_frame_num),
    .o_fc_vc               (w_b03_fc_vc),
    .o_gsp_data            (w_b03_gsp_data),
    .i_gsp_pop             (w_b14_gsp_pop),
    .o_sp_fifo_not_empty   (w_b03_sp_fifo_not_empty),
    .o_sp_fifo_full        (w_b03_sp_fifo_full),
    .o_sp_fifo_overflow    (w_b03_sp_fifo_overflow),
    .o_null_blank_seen     (w_b03_null_blank_seen),
    .o_packet_done_pulse   (w_b03_packet_done_pulse),
    .o_err_ecc_1bit        (w_b03_err_ecc_1bit),
    .o_err_ecc_2bit        (w_b03_err_ecc_2bit),
    .o_err_crc             (w_b03_err_crc),
    .o_err_wc              (w_b03_err_wc),
    .o_err_unsupported_dt  (w_b03_err_unsupported_dt),
    .o_err_vc              (w_b03_err_vc),
    .o_err_lane_abort      (w_b03_err_lane_abort),
    .o_err_frame_sync      (w_b03_err_frame_sync),
    .o_err_frame_level     (w_b03_err_frame_level),
    .o_vc_unsupported_seen (w_b03_vc_unsupported_seen),
    .o_dt_conflict         (w_b03_dt_conflict)
  );

  // ------------------------------------------------------------------------------------------
  // B04 - Byte-to-Pixel Unpacking
  // ------------------------------------------------------------------------------------------
  // RAW10 / RAW12 payload bytes unpacked into pixels (sys_clk).
  m_fpt_byte_to_pixel u_byte_to_pixel_0 (
    .i_sys_clk        (i_sys_clk),
    .i_resetn         (w_sys_resetn),
    .i_soft_reset     (w_b14_soft_reset_sys),
    .i_bp_data        (w_b03_bp_data),
    .i_bp_valid       (w_b03_bp_valid),
    .i_bp_byte_en     (w_b03_bp_byte_en),
    .i_bp_sof         (w_b03_bp_sof),
    .i_bp_sol         (w_b03_bp_sol),
    .i_bp_eol         (w_b03_bp_eol),
    .i_bp_vc_tag      (w_b03_bp_vc_tag),
    .i_bp_dt_sel      (w_b03_bp_dt_sel),
    .o_px_data        (w_b04_px_data),
    .o_px_valid       (w_b04_px_valid),
    .o_px_valid_count (w_b04_px_valid_count),
    .o_px_sof         (w_b04_px_sof),
    .o_px_sol         (w_b04_px_sol),
    .o_px_eol         (w_b04_px_eol),
    .o_px_vc_tag      (w_b04_px_vc_tag),
    .o_px_dt_sel      (w_b04_px_dt_sel),
    .o_err_wc_align   (w_b04_err_wc_align)
  );

  // ------------------------------------------------------------------------------------------
  // B05 - Test Pattern Generator
  // ------------------------------------------------------------------------------------------
  // Colour bar frames built from the register configuration (sys_clk).
  m_fpt_tpg u_tpg_0 (
    .i_sys_clk           (i_sys_clk),
    .i_resetn            (w_sys_resetn),
    .i_soft_reset        (w_b14_soft_reset_sys),
    .i_core_enable       (w_b14_core_enable),
    .i_tpg_enable        (w_b14_tpg_enable),
    .i_tpg_pattern       (w_b14_tpg_pattern),
    .i_tpg_width         (w_b14_tpg_width),
    .i_tpg_height        (w_b14_tpg_height),
    .i_tpg_dual_vc_en    (w_b14_tpg_dual_vc_en),
    .i_px_ready          (w_b06_tpg_px_ready),
    .i_tpg_pixel_format  (w_b14_tpg_pixel_format),
    .i_tpg_hblank        (w_b14_tpg_hblank),
    .i_tpg_vblank        (w_b14_tpg_vblank),
    .i_tpg_cfa_order     (w_b14_tpg_cfa_order),
    .i_linebuf_empty     (w_b07_linebuf_empty),
    .o_px_data           (w_b05_px_data),
    .o_px_valid          (w_b05_px_valid),
    .o_px_sof            (w_b05_px_sof),
    .o_px_sol            (w_b05_px_sol),
    .o_px_eol            (w_b05_px_eol),
    .o_px_vc_tag         (w_b05_px_vc_tag),
    .o_px_dt_sel         (w_b05_px_dt_sel),
    .o_tpg_eof           (w_b05_tpg_eof),
    .o_tpg_fs            (w_b05_tpg_fs),
    .o_tpg_fe            (w_b05_tpg_fe),
    .o_tpg_fvc           (w_b05_tpg_fvc),
    .o_tpg_height_shadow (w_b05_tpg_height_shadow)
  );

  // ------------------------------------------------------------------------------------------
  // B06 - TPG Mux
  // ------------------------------------------------------------------------------------------
  // Selects the sensor or the pattern generator pixel stream at a frame boundary (sys_clk).
  m_fpt_tpg_mux u_tpg_mux_0 (
    .i_sys_clk           (i_sys_clk),
    .i_resetn            (w_sys_resetn),
    .i_soft_reset        (w_b14_soft_reset_sys),
    .i_rx_px_data        (w_b04_px_data),
    .i_rx_px_valid       (w_b04_px_valid),
    .i_rx_px_valid_count (w_b04_px_valid_count),
    .i_rx_px_sof         (w_b04_px_sof),
    .i_rx_px_sol         (w_b04_px_sol),
    .i_rx_px_eol         (w_b04_px_eol),
    .i_rx_px_vc_tag      (w_b04_px_vc_tag),
    .i_rx_px_dt_sel      (w_b04_px_dt_sel),
    .i_rx_err_wc_align   (w_b04_err_wc_align),
    .i_fc_fs             (w_b03_fc_fs),
    .i_fc_fe             (w_b03_fc_fe),
    .i_fc_vc             (w_b03_fc_vc),
    .i_rx_ctx_clear      (w_b10_rx_ctx_clear),
    .i_tpg_px_data       (w_b05_px_data),
    .i_tpg_px_valid      (w_b05_px_valid),
    .i_tpg_px_sof        (w_b05_px_sof),
    .i_tpg_px_sol        (w_b05_px_sol),
    .i_tpg_px_eol        (w_b05_px_eol),
    .i_tpg_px_vc_tag     (w_b05_px_vc_tag),
    .i_tpg_px_dt_sel     (w_b05_px_dt_sel),
    .i_tpg_fs            (w_b05_tpg_fs),
    .i_tpg_eof           (w_b05_tpg_eof),
    .i_linebuffer_full   (w_b07_linebuffer_full),
    .i_linebuf_empty     (w_b07_linebuf_empty),
    .o_tpg_px_ready      (w_b06_tpg_px_ready),
    .i_tpg_enable        (w_b14_tpg_enable),
    .o_px_data           (w_b06_px_data),
    .o_px_valid          (w_b06_px_valid),
    .o_px_valid_count    (w_b06_px_valid_count),
    .o_px_sof            (w_b06_px_sof),
    .o_px_sol            (w_b06_px_sol),
    .o_px_eol            (w_b06_px_eol),
    .o_px_vc_tag         (w_b06_px_vc_tag),
    .o_px_dt_sel         (w_b06_px_dt_sel),
    .o_err_wc_align      (w_b06_err_wc_align),
    .o_active_sel        (w_b06_active_sel),
    .o_switch_pending    (w_b06_switch_pending)
  );

  // ------------------------------------------------------------------------------------------
  // B07 - Line Buffer
  // ------------------------------------------------------------------------------------------
  // Pixel FIFO that absorbs output backpressure; reports fill level and overflow (sys_clk).
  m_fpt_line_buffer #(
    .PARA_LINE_BUFFER_DEPTH (PARA_LINE_BUFFER_DEPTH),
    .PARA_FULL_WATERMARK    (PARA_FULL_WATERMARK)
  ) u_line_buffer_0 (
    .i_sys_clk           (i_sys_clk),
    .i_resetn            (w_sys_resetn),
    .i_soft_reset        (w_b14_soft_reset_sys),
    .i_lb_flush          (w_b10_lb_flush),
    .i_px_data           (w_b06_px_data),
    .i_px_valid          (w_b06_px_valid),
    .i_px_valid_count    (w_b06_px_valid_count),
    .i_px_sof            (w_b06_px_sof),
    .i_px_sol            (w_b06_px_sol),
    .i_px_eol            (w_b06_px_eol),
    .i_px_vc_tag         (w_b06_px_vc_tag),
    .i_px_dt_sel         (w_b06_px_dt_sel),
    .i_err_wc_align      (w_b06_err_wc_align),
    .i_fb_ready          (w_b08_fb_ready),
    .o_fb_px_data        (w_b07_fb_px_data),
    .o_fb_px_valid       (w_b07_fb_px_valid),
    .o_fb_px_valid_count (w_b07_fb_px_valid_count),
    .o_fb_px_sof         (w_b07_fb_px_sof),
    .o_fb_px_sol         (w_b07_fb_px_sol),
    .o_fb_px_eol         (w_b07_fb_px_eol),
    .o_fb_px_vc_tag      (w_b07_fb_px_vc_tag),
    .o_fb_px_dt_sel      (w_b07_fb_px_dt_sel),
    .o_fb_err_wc_align   (w_b07_fb_err_wc_align),
    .o_linebuffer_full   (w_b07_linebuffer_full),
    .o_linebuf_fill      (w_b07_linebuf_fill),
    .o_linebuf_overflow  (w_b07_linebuf_overflow),
    .o_linebuf_empty     (w_b07_linebuf_empty)
  );

  // ------------------------------------------------------------------------------------------
  // B08 - Format Bridge
  // ------------------------------------------------------------------------------------------
  // Packs pixels into 64-bit beats for the stream and DMA outputs (sys_clk).
  // The two FIFO almost-full outputs are for observation only and stay open.
  m_fpt_format_bridge #(
    .PARA_STREAM_FIFO_DEPTH (PARA_STREAM_FIFO_DEPTH),
    .PARA_DMA_FIFO_DEPTH    (PARA_DMA_FIFO_DEPTH)
  ) u_format_bridge_0 (
    .i_sys_clk                 (i_sys_clk),
    .i_resetn                  (w_sys_resetn),
    .i_soft_reset              (w_b14_soft_reset_sys),
    .i_fb_flush                (w_b10_fb_flush),
    .i_fb_px_data              (w_b07_fb_px_data),
    .i_fb_px_valid             (w_b07_fb_px_valid),
    .i_fb_px_valid_count       (w_b07_fb_px_valid_count),
    .i_fb_px_sof               (w_b07_fb_px_sof),
    .i_fb_px_sol               (w_b07_fb_px_sol),
    .i_fb_px_eol               (w_b07_fb_px_eol),
    .i_fb_px_vc_tag            (w_b07_fb_px_vc_tag),
    .i_fb_px_dt_sel            (w_b07_fb_px_dt_sel),
    .i_fb_err_wc_align         (w_b07_fb_err_wc_align),
    .o_fb_ready                (w_b08_fb_ready),
    .i_stream_format           (w_b14_stream_format),
    .i_dma_format              (w_b14_dma_format),
    .i_axis_enable             (w_b14_axis_enable),
    .i_dma_enable              (w_b14_dma_enable),
    .o_fbs_data                (w_b08_fbs_data),
    .o_fbs_valid               (w_b08_fbs_valid),
    .o_fbs_byte_en             (w_b08_fbs_byte_en),
    .o_fbs_sof                 (w_b08_fbs_sof),
    .o_fbs_eol                 (w_b08_fbs_eol),
    .o_fbs_vc_tag              (w_b08_fbs_vc_tag),
    .o_fbs_err                 (w_b08_fbs_err),
    .i_fbs_ready               (w_b12_fbs_ready),
    .o_fbd_data                (w_b08_fbd_data),
    .o_fbd_valid               (w_b08_fbd_valid),
    .o_fbd_byte_en             (w_b08_fbd_byte_en),
    .o_fbd_sof                 (w_b08_fbd_sof),
    .o_fbd_sol                 (w_b08_fbd_sol),
    .o_fbd_eol                 (w_b08_fbd_eol),
    .o_fbd_vc_tag              (w_b08_fbd_vc_tag),
    .o_fbd_err                 (w_b08_fbd_err),
    .i_fbd_ready               (w_b13_fbd_ready),
    .o_fb_overflow             (w_b08_fb_overflow),
    .o_stream_fifo_almost_full (),
    .o_dma_fifo_almost_full    ()
  );

  // ------------------------------------------------------------------------------------------
  // B09 - Frame Context & Metadata Capture
  // ------------------------------------------------------------------------------------------
  // Per-VC frame context, timestamps, embedded data extraction and metadata records (sys_clk).
  m_fpt_frame_context_metadata #(
    .PARA_N_EXTRACT       (PARA_N_EXTRACT),
    .PARA_TS_INCR         (PARA_TS_INCR),
    .PARA_META_FIFO_DEPTH (PARA_META_FIFO_DEPTH)
  ) u_frame_context_metadata_0 (
    .i_sys_clk             (i_sys_clk),
    .i_resetn              (w_sys_resetn),
    .i_soft_reset          (w_b14_soft_reset_sys),
    .i_core_enable         (w_b14_core_enable),
    .i_metadata_capture_en (w_b14_metadata_capture_en),
    .i_fnum_check_en       (w_b14_fnum_check_en),
    .i_emb_extract_en      (w_b14_emb_extract_en),
    .i_expected_width      (w_b14_expected_width),
    .i_expected_height     (w_b14_expected_height),
    .i_metadata_record_id  (w_b14_metadata_record_id),
    .i_emb_extract_cfg     (w_b14_emb_extract_cfg),
    .i_vc_exposure_map     (w_b14_vc_exposure_map),
    .i_tpg_dual_vc_en      (w_b14_tpg_dual_vc_en),
    .i_active_sel          (w_b06_active_sel),
    .i_fc_fs               (w_b03_fc_fs),
    .i_fc_fe               (w_b03_fc_fe),
    .i_fc_frame_num        (w_b03_fc_frame_num),
    .i_fc_vc               (w_b03_fc_vc),
    .i_tpg_fs              (w_b05_tpg_fs),
    .i_tpg_fe              (w_b05_tpg_fe),
    .i_tpg_fvc             (w_b05_tpg_fvc),
    .i_bp_valid            (w_b03_bp_valid),
    .i_bp_byte_en          (w_b03_bp_byte_en),
    .i_bp_sol              (w_b03_bp_sol),
    .i_bp_eol              (w_b03_bp_eol),
    .i_bp_vc_tag           (w_b03_bp_vc_tag),
    .i_bp_dt_sel           (w_b03_bp_dt_sel),
    .i_emb_data            (w_b03_emb_data),
    .i_emb_byte_en         (w_b03_emb_byte_en),
    .i_emb_valid           (w_b03_emb_valid),
    .i_emb_sol             (w_b03_emb_sol),
    .i_emb_eol             (w_b03_emb_eol),
    .i_emb_vc              (w_b03_emb_vc),
    .i_capture_drop        (w_b10_capture_drop),
    .i_drop_vc_mask        (w_b10_drop_vc_mask),
    .i_dma_buf_claim       (w_b13_dma_buf_claim),
    .i_dma_buf_index       (w_b13_dma_buf_index),
    .i_meta_ready          (w_b13_meta_ready),
    .o_capture_start       (w_b09_capture_start),
    .o_capture_end         (w_b09_capture_end),
    .o_capture_dropped     (w_b09_capture_dropped),
    .o_ev_fs               (w_b09_ev_fs),
    .o_ev_vc               (w_b09_ev_vc),
    .o_ctx_slot            (w_b09_ctx_slot),
    .o_ctx_orphan          (w_b09_ctx_orphan),
    .o_lead_vc             (w_b09_lead_vc),
    .o_frame_count         (w_b09_frame_count),
    .o_frame_id_buf0       (w_b09_frame_id_buf0),
    .o_frame_id_buf1       (w_b09_frame_id_buf1),
    .o_timestamp           (w_b09_timestamp),
    .o_vc0_line_count      (w_b09_vc0_line_count),
    .o_vc0_byte_count      (w_b09_vc0_byte_count),
    .o_vc0_data_type       (w_b09_vc0_data_type),
    .o_vc0_width_mismatch  (w_b09_vc0_width_mismatch),
    .o_vc0_height_mismatch (w_b09_vc0_height_mismatch),
    .o_emb_word_count_vc0  (w_b09_emb_word_count_vc0),
    .o_emb_line_count_vc0  (w_b09_emb_line_count_vc0),
    .o_emb_extract_val_vc0 (w_b09_emb_extract_val_vc0),
    .o_cur_frame_id_vc0    (w_b09_cur_frame_id_vc0),
    .o_cur_line_vc0        (w_b09_cur_line_vc0),
    .o_vc1_line_count      (w_b09_vc1_line_count),
    .o_vc1_byte_count      (w_b09_vc1_byte_count),
    .o_vc1_data_type       (w_b09_vc1_data_type),
    .o_vc1_width_mismatch  (w_b09_vc1_width_mismatch),
    .o_vc1_height_mismatch (w_b09_vc1_height_mismatch),
    .o_emb_word_count_vc1  (w_b09_emb_word_count_vc1),
    .o_emb_line_count_vc1  (w_b09_emb_line_count_vc1),
    .o_emb_extract_val_vc1 (w_b09_emb_extract_val_vc1),
    .o_cur_frame_id_vc1    (w_b09_cur_frame_id_vc1),
    .o_cur_line_vc1        (w_b09_cur_line_vc1),
    .o_meta_overflow       (w_b09_meta_overflow),
    .o_err_frame_num       (w_b09_err_frame_num),
    .o_err_resolution      (w_b09_err_resolution),
    .o_capture_done        (w_b09_capture_done),
    .o_meta_data           (w_b09_meta_data),
    .o_meta_byte_en        (w_b09_meta_byte_en),
    .o_meta_valid          (w_b09_meta_valid),
    .o_meta_last           (w_b09_meta_last),
    .o_meta_kind           (w_b09_meta_kind),
    .o_meta_vc             (w_b09_meta_vc),
    .o_meta_sof            (w_b09_meta_sof)
  );

  // ------------------------------------------------------------------------------------------
  // B10 - Control FSM
  // ------------------------------------------------------------------------------------------
  // Frame admission, discard and flush control of the datapath (sys_clk).
  m_fpt_control_fsm u_control_fsm_0 (
    .i_sys_clk                (i_sys_clk),
    .i_resetn                 (w_sys_resetn),
    .i_soft_reset             (w_b14_soft_reset_sys),
    .i_core_enable            (w_b14_core_enable),
    .i_axis_enable            (w_b14_axis_enable),
    .i_stream_follow_dma_drop (w_b14_stream_follow_dma_drop),
    .i_vc_exposure_map        (w_b14_vc_exposure_map),
    .i_tpg_dual_vc_en         (w_b14_tpg_dual_vc_en),
    .i_err_ecc_2bit           (w_b03_err_ecc_2bit),
    .i_err_lane_abort         (w_b03_err_lane_abort),
    .i_err_crc                (w_b03_err_crc),
    .i_err_wc                 (w_b03_err_wc),
    .i_err_vc                 (w_b03_err_vc),
    .o_force_discard          (w_b10_force_discard),
    .i_active_sel             (w_b06_active_sel),
    .o_rx_ctx_clear           (w_b10_rx_ctx_clear),
    .i_linebuffer_full        (w_b07_linebuffer_full),
    .i_linebuf_overflow       (w_b07_linebuf_overflow),
    .o_lb_flush               (w_b10_lb_flush),
    .i_fb_overflow            (w_b08_fb_overflow),
    .o_fb_flush               (w_b10_fb_flush),
    .i_capture_start          (w_b09_capture_start),
    .i_capture_end            (w_b09_capture_end),
    .i_capture_dropped        (w_b09_capture_dropped),
    .i_ev_fs                  (w_b09_ev_fs),
    .i_ev_vc                  (w_b09_ev_vc),
    .i_lead_vc                (w_b09_lead_vc),
    .o_capture_drop           (w_b10_capture_drop),
    .o_drop_vc_mask           (w_b10_drop_vc_mask),
    .o_asm_abort_req          (w_b10_asm_abort_req),
    .o_asm_abort_mask         (w_b10_asm_abort_mask),
    .o_asm_skip_req           (w_b10_asm_skip_req),
    .o_asm_skip_mask          (w_b10_asm_skip_mask),
    .i_dma_drop               (w_b13_dma_drop),
    .o_dma_abort_req          (w_b10_dma_abort_req),
    .o_frame_drop_pulse       (w_b10_frame_drop_pulse)
  );

  // ------------------------------------------------------------------------------------------
  // B11 - Interrupt & Error Aggregator
  // ------------------------------------------------------------------------------------------
  // Event capture from rxbyteclkhs and sys_clk, error counters and the csi2_irq output.
  m_fpt_irq_aggregator #(
    .PARA_MAX_LANES   (PARA_MAX_LANES),
    .PARA_SYNC_STAGES (PARA_SYNC_STAGES),
    .PARA_RST_TAIL    (PARA_RST_TAIL)
  ) u_irq_aggregator_0 (
    .i_sys_clk             (i_sys_clk),
    .i_rxbyteclkhs         (w_b01_rxbyteclkhs),
    .i_resetn              (w_sys_resetn),
    .i_rxbyteclkhs_resetn  (w_rxbyteclkhs_resetn),
    .i_soft_reset          (w_b14_soft_reset_sys),
    .i_phy_rst_in_progress (w_b14_phy_rst_in_progress),
    .i_global_irq_enable   (w_b14_global_irq_enable),
    .i_irq_enable_mask     (w_b14_irq_enable_mask),
    .i_irq_clear_strobe    (w_b14_irq_clear_strobe),
    .o_irq_status          (w_b11_irq_status),
    .o_error_capture       (w_b11_error_capture),
    .o_ecc_err_cnt         (w_b11_ecc_err_cnt),
    .o_crc_err_cnt         (w_b11_crc_err_cnt),
    .o_frame_drop_cnt      (w_b11_frame_drop_cnt),
    .o_lane_abort_cnt      (w_b11_lane_abort_cnt),
    .o_packet_cnt          (w_b11_packet_cnt),
    .o_linebuf_ovf_sticky  (w_b11_linebuf_ovf_sticky),
    .o_sp_fifo_ovf_sticky  (w_b11_sp_fifo_ovf_sticky),
    .o_csi2_irq            (o_csi2_irq),
    .i_errsoths            (w_b11_errsoths),
    .i_errsotsynchs        (w_b11_errsotsynchs),
    .i_cl_stopstate        (w_b01_cl_stopstate),
    .i_lane_abort_pulse    (w_b02_lane_abort_pulse),
    .i_err_frame_sync      (w_b03_err_frame_sync),
    .i_err_frame_level     (w_b03_err_frame_level),
    .i_err_crc             (w_b03_err_crc),
    .i_err_ecc_1bit        (w_b03_err_ecc_1bit),
    .i_err_ecc_2bit        (w_b03_err_ecc_2bit),
    .i_err_unsupported_dt  (w_b03_err_unsupported_dt),
    .i_err_wc              (w_b03_err_wc),
    .i_err_vc              (w_b03_err_vc),
    .i_err_lane_abort      (w_b03_err_lane_abort),
    .i_packet_done_pulse   (w_b03_packet_done_pulse),
    .i_sp_fifo_overflow    (w_b03_sp_fifo_overflow),
    .i_err_wc_align        (w_b04_err_wc_align),
    .i_active_sel          (w_b06_active_sel),
    .i_linebuffer_full     (w_b07_linebuffer_full),
    .i_linebuf_overflow    (w_b07_linebuf_overflow),
    .i_fb_overflow         (w_b08_fb_overflow),
    .i_capture_done        (w_b09_capture_done),
    .i_err_resolution      (w_b09_err_resolution),
    .i_err_frame_num       (w_b09_err_frame_num),
    .i_cur_frame_id_vc0    (w_b09_cur_frame_id_vc0),
    .i_cur_frame_id_vc1    (w_b09_cur_frame_id_vc1),
    .i_cur_line_vc0        (w_b09_cur_line_vc0),
    .i_cur_line_vc1        (w_b09_cur_line_vc1),
    .i_meta_overflow       (w_b09_meta_overflow),
    .i_frame_drop_pulse    (w_b10_frame_drop_pulse),
    .i_dma_buf_done_ok     (w_b13_dma_buf_done_ok),
    .i_dma_buf_done_err    (w_b13_dma_buf_done_err),
    .i_dma_axi_write_error (w_b13_dma_axi_write_error),
    .i_dma_buffer_overflow (w_b13_dma_buffer_overflow),
    .o_dma_ovf_sticky      (w_b11_dma_ovf_sticky)
  );

  // ------------------------------------------------------------------------------------------
  // B12 - AXI4-Stream Master
  // ------------------------------------------------------------------------------------------
  // AXI4-Stream master towards the ISP (axis_aclk, the sys_clk net).
  // asm_abort_done is for observation only and stays open.
  m_fpt_axi4_stream u_axi4_stream_0 (
    .i_axis_aclk       (i_sys_clk),
    .i_resetn          (w_sys_resetn),
    .i_soft_reset      (w_b14_soft_reset_sys),
    .i_fbs_data        (w_b08_fbs_data),
    .i_fbs_valid       (w_b08_fbs_valid),
    .i_fbs_byte_en     (w_b08_fbs_byte_en),
    .i_fbs_sof         (w_b08_fbs_sof),
    .i_fbs_eol         (w_b08_fbs_eol),
    .i_fbs_vc_tag      (w_b08_fbs_vc_tag),
    .i_fbs_err         (w_b08_fbs_err),
    .o_fbs_ready       (w_b12_fbs_ready),
    .i_asm_abort_req   (w_b10_asm_abort_req),
    .i_asm_abort_mask  (w_b10_asm_abort_mask),
    .o_asm_abort_done  (),
    .i_asm_skip_req    (w_b10_asm_skip_req),
    .i_asm_skip_mask   (w_b10_asm_skip_mask),
    .i_axis_enable     (w_b14_axis_enable),
    .i_vc_exposure_map (w_b14_vc_exposure_map),
    .o_m_axis_tdata    (o_m_axis_tdata),
    .o_m_axis_tvalid   (o_m_axis_tvalid),
    .i_m_axis_tready   (i_m_axis_tready),
    .o_m_axis_tlast    (o_m_axis_tlast),
    .o_m_axis_tuser    (o_m_axis_tuser),
    .o_m_axis_tid      (o_m_axis_tid),
    .o_m_axis_tkeep    (o_m_axis_tkeep)
  );

  // ------------------------------------------------------------------------------------------
  // B13 - Direct Memory Access (DMA)
  // ------------------------------------------------------------------------------------------
  // Ping-pong buffer writer on the AXI4 write channels; sys_clk front end, mm_aclk back end.
  // i_mm_aclk is always the mm_aclk port; resetn and full_reset reset both halves.
  m_fpt_dma #(
    .PARA_AXI_DATA_WIDTH       (PARA_AXI_DATA_WIDTH),
    .PARA_AXI_ADDR_WIDTH       (PARA_AXI_ADDR_WIDTH),
    .PARA_AXI_ID_WIDTH         (PARA_AXI_ID_WIDTH),
    .PARA_MAX_BURST_BEATS      (PARA_MAX_BURST_BEATS),
    .PARA_MAX_OUTSTANDING      (PARA_MAX_OUTSTANDING),
    .PARA_RAW_FIFO_DEPTH       (PARA_RAW_FIFO_DEPTH),
    .PARA_META_DATA_FIFO_DEPTH (PARA_META_DATA_FIFO_DEPTH),
    .PARA_CMD_FIFO_DEPTH       (PARA_CMD_FIFO_DEPTH),
    .PARA_ASYNC_MM_CLK         (PARA_ASYNC_MM_CLK),
    .PARA_SYNC_STAGES          (PARA_SYNC_STAGES),
    .PARA_META_MAX_BURST_BEATS (PARA_META_MAX_BURST_BEATS)
  ) u_dma_0 (
    .i_sys_clk              (i_sys_clk),
    .i_mm_aclk              (i_mm_aclk),
    .i_resetn               (w_sys_resetn),
    .i_mm_resetn            (w_mm_resetn),
    .i_soft_reset           (w_b14_soft_reset_sys),
    .i_fbd_data             (w_b08_fbd_data),
    .i_fbd_byte_en          (w_b08_fbd_byte_en),
    .i_fbd_valid            (w_b08_fbd_valid),
    .i_fbd_sof              (w_b08_fbd_sof),
    .i_fbd_sol              (w_b08_fbd_sol),
    .i_fbd_eol              (w_b08_fbd_eol),
    .i_fbd_vc_tag           (w_b08_fbd_vc_tag),
    .i_fbd_err              (w_b08_fbd_err),
    .o_fbd_ready            (w_b13_fbd_ready),
    .i_capture_start        (w_b09_capture_start),
    .i_capture_end          (w_b09_capture_end),
    .i_capture_dropped      (w_b09_capture_dropped),
    .i_ev_fs                (w_b09_ev_fs),
    .i_ev_vc                (w_b09_ev_vc),
    .i_lead_vc              (w_b09_lead_vc),
    .i_ctx_slot             (w_b09_ctx_slot),
    .i_ctx_orphan           (w_b09_ctx_orphan),
    .i_frame_id_buf0        (w_b09_frame_id_buf0),
    .i_frame_id_buf1        (w_b09_frame_id_buf1),
    .i_meta_data            (w_b09_meta_data),
    .i_meta_byte_en         (w_b09_meta_byte_en),
    .i_meta_valid           (w_b09_meta_valid),
    .i_meta_last            (w_b09_meta_last),
    .i_meta_kind            (w_b09_meta_kind),
    .i_meta_vc              (w_b09_meta_vc),
    .i_meta_sof             (w_b09_meta_sof),
    .o_meta_ready           (w_b13_meta_ready),
    .o_dma_buf_claim        (w_b13_dma_buf_claim),
    .o_dma_buf_index        (w_b13_dma_buf_index),
    .i_active_sel           (w_b06_active_sel),
    .i_tpg_height_shadow    (w_b05_tpg_height_shadow),
    .i_dma_abort_req        (w_b10_dma_abort_req),
    .o_dma_drop             (w_b13_dma_drop),
    .o_dma_buf_done_ok      (w_b13_dma_buf_done_ok),
    .o_dma_buf_done_err     (w_b13_dma_buf_done_err),
    .o_dma_axi_write_error  (w_b13_dma_axi_write_error),
    .o_dma_buffer_overflow  (w_b13_dma_buffer_overflow),
    .i_dma_enable           (w_b14_dma_enable),
    .i_dma_start            (w_b14_dma_start),
    .i_dma_abort            (w_b14_dma_abort),
    .i_err_frame_policy     (w_b14_err_frame_policy),
    .i_dma_buffer0_addr     (w_b14_dma_buffer0_addr),
    .i_dma_buffer1_addr     (w_b14_dma_buffer1_addr),
    .i_dma_buffer_size      (w_b14_dma_buffer_size),
    .i_meta_area_size       (w_b14_meta_area_size),
    .i_bytes_per_line       (w_b14_bytes_per_line),
    .i_line_padding         (w_b14_line_padding),
    .i_awqos                (w_b14_awqos),
    .i_max_burst_len        (w_b14_max_burst_len),
    .i_awprot               (w_b14_awprot),
    .i_release_buf          (w_b14_release_buf),
    .o_dma_busy             (w_b13_dma_busy),
    .o_active_buffer        (w_b13_active_buffer),
    .o_buf0_state           (w_b13_buf0_state),
    .o_buf1_state           (w_b13_buf1_state),
    .o_buf0_last_valid_line (w_b13_buf0_last_valid_line),
    .o_buf1_last_valid_line (w_b13_buf1_last_valid_line),
    .o_axi_fault_awid       (w_b13_axi_fault_awid),
    .o_axi_fault_resp       (w_b13_axi_fault_resp),
    .o_axi_fault_awaddr     (w_b13_axi_fault_awaddr),
    .o_bresp_slverr_count   (w_b13_bresp_slverr_count),
    .o_bresp_decerr_count   (w_b13_bresp_decerr_count),
    .o_m_axi4_awid          (o_m_axi4_awid),
    .o_m_axi4_awaddr        (o_m_axi4_awaddr),
    .o_m_axi4_awlen         (o_m_axi4_awlen),
    .o_m_axi4_awsize        (o_m_axi4_awsize),
    .o_m_axi4_awburst       (o_m_axi4_awburst),
    .o_m_axi4_awlock        (o_m_axi4_awlock),
    .o_m_axi4_awcache       (o_m_axi4_awcache),
    .o_m_axi4_awprot        (o_m_axi4_awprot),
    .o_m_axi4_awqos         (o_m_axi4_awqos),
    .o_m_axi4_awvalid       (o_m_axi4_awvalid),
    .i_m_axi4_awready       (i_m_axi4_awready),
    .o_m_axi4_wdata         (o_m_axi4_wdata),
    .o_m_axi4_wstrb         (o_m_axi4_wstrb),
    .o_m_axi4_wlast         (o_m_axi4_wlast),
    .o_m_axi4_wvalid        (o_m_axi4_wvalid),
    .i_m_axi4_wready        (i_m_axi4_wready),
    .i_m_axi4_bid           (i_m_axi4_bid),
    .i_m_axi4_bresp         (i_m_axi4_bresp),
    .i_m_axi4_bvalid        (i_m_axi4_bvalid),
    .o_m_axi4_bready        (o_m_axi4_bready)
  );

  // ------------------------------------------------------------------------------------------
  // B14 - CSR & AXI4-Lite Subordinate
  // ------------------------------------------------------------------------------------------
  // Register file on AXI4-Lite (lite_aclk), its crossings and the D-PHY reset sequencer.
  // Takes the raw resetn and lite_aresetn; soft_reset leaves in an rxbyteclkhs and a sys_clk copy.
  m_fpt_csr_axi4lite #(
    .PARA_MAX_LANES       (PARA_MAX_LANES),
    .PARA_N_EXTRACT       (PARA_N_EXTRACT),
    .PARA_MAX_OUTSTANDING (PARA_MAX_OUTSTANDING),
    .PARA_SYNC_STAGES     (PARA_SYNC_STAGES),
    .PARA_PHY_RST_CYCLES  (PARA_PHY_RST_CYCLES),
    .PARA_IRQ_CLR_TIMEOUT (PARA_IRQ_CLR_TIMEOUT)
  ) u_csr_axi4lite_0 (
    .i_lite_aclk              (i_lite_aclk),
    .i_lite_aresetn           (i_lite_aresetn),
    .i_resetn                 (i_resetn),
    .i_sys_clk                (i_sys_clk),
    .i_rxbyteclkhs            (w_b01_rxbyteclkhs),
    .i_sys_resetn             (w_sys_resetn),
    .i_rxbyteclkhs_resetn     (w_rxbyteclkhs_resetn),
    .i_s_axi_awaddr           (i_s_axi_awaddr),
    .i_s_axi_awprot           (i_s_axi_awprot),
    .i_s_axi_awvalid          (i_s_axi_awvalid),
    .o_s_axi_awready          (o_s_axi_awready),
    .i_s_axi_wdata            (i_s_axi_wdata),
    .i_s_axi_wstrb            (i_s_axi_wstrb),
    .i_s_axi_wvalid           (i_s_axi_wvalid),
    .o_s_axi_wready           (o_s_axi_wready),
    .o_s_axi_bresp            (o_s_axi_bresp),
    .o_s_axi_bvalid           (o_s_axi_bvalid),
    .i_s_axi_bready           (i_s_axi_bready),
    .i_s_axi_araddr           (i_s_axi_araddr),
    .i_s_axi_arprot           (i_s_axi_arprot),
    .i_s_axi_arvalid          (i_s_axi_arvalid),
    .o_s_axi_arready          (o_s_axi_arready),
    .o_s_axi_rdata            (o_s_axi_rdata),
    .o_s_axi_rresp            (o_s_axi_rresp),
    .o_s_axi_rvalid           (o_s_axi_rvalid),
    .i_s_axi_rready           (i_s_axi_rready),
    .o_phy_core_rst           (w_b14_phy_core_rst),
    .o_cl_enable              (w_b14_cl_enable),
    .o_dl_enable              (w_b14_dl_enable),
    .o_forcerxmode            (w_b14_forcerxmode),
    .o_full_reset_req         (w_b14_full_reset_req),
    .i_cl_stopstate           (w_b01_cl_stopstate),
    .i_cl_ulpsactivenot       (w_b01_cl_ulpsactivenot),
    .i_cl_rxclkactivehs       (w_b01_cl_rxclkactivehs),
    .i_cl_ulpsclknot          (w_b01_cl_ulpsclknot),
    .i_init_done              (w_b01_init_done),
    .i_stopstate              (w_b14_stopstate),
    .i_rxskewcalhs            (w_b14_rxskewcalhs),
    .i_errsoths               (w_b14_errsoths),
    .i_errsotsynchs           (w_b14_errsotsynchs),
    .o_soft_reset_byte        (w_b14_soft_reset_byte),
    .o_active_lanes           (w_b14_active_lanes),
    .o_soft_reset_sys         (w_b14_soft_reset_sys),
    .o_core_enable            (w_b14_core_enable),
    .o_vc_exposure_map        (w_b14_vc_exposure_map),
    .o_metadata_dt            (w_b14_metadata_dt),
    .o_metadata_capture_en    (w_b14_metadata_capture_en),
    .o_fnum_check_en          (w_b14_fnum_check_en),
    .o_emb_extract_en         (w_b14_emb_extract_en),
    .o_expected_width         (w_b14_expected_width),
    .o_expected_height        (w_b14_expected_height),
    .o_metadata_record_id     (w_b14_metadata_record_id),
    .o_emb_extract_cfg        (w_b14_emb_extract_cfg),
    .o_axis_enable            (w_b14_axis_enable),
    .o_stream_format          (w_b14_stream_format),
    .o_stream_follow_dma_drop (w_b14_stream_follow_dma_drop),
    .o_tpg_enable             (w_b14_tpg_enable),
    .o_tpg_pattern            (w_b14_tpg_pattern),
    .o_tpg_dual_vc_en         (w_b14_tpg_dual_vc_en),
    .o_tpg_pixel_format       (w_b14_tpg_pixel_format),
    .o_tpg_cfa_order          (w_b14_tpg_cfa_order),
    .o_tpg_width              (w_b14_tpg_width),
    .o_tpg_height             (w_b14_tpg_height),
    .o_tpg_hblank             (w_b14_tpg_hblank),
    .o_tpg_vblank             (w_b14_tpg_vblank),
    .o_dma_enable             (w_b14_dma_enable),
    .o_dma_start              (w_b14_dma_start),
    .o_dma_abort              (w_b14_dma_abort),
    .o_err_frame_policy       (w_b14_err_frame_policy),
    .o_dma_format             (w_b14_dma_format),
    .o_dma_buffer0_addr       (w_b14_dma_buffer0_addr),
    .o_dma_buffer1_addr       (w_b14_dma_buffer1_addr),
    .o_dma_buffer_size        (w_b14_dma_buffer_size),
    .o_meta_area_size         (w_b14_meta_area_size),
    .o_bytes_per_line         (w_b14_bytes_per_line),
    .o_line_padding           (w_b14_line_padding),
    .o_awqos                  (w_b14_awqos),
    .o_max_burst_len          (w_b14_max_burst_len),
    .o_awprot                 (w_b14_awprot),
    .o_release_buf            (w_b14_release_buf),
    .o_global_irq_enable      (w_b14_global_irq_enable),
    .o_irq_enable_mask        (w_b14_irq_enable_mask),
    .o_irq_clear_strobe       (w_b14_irq_clear_strobe),
    .o_phy_rst_in_progress    (w_b14_phy_rst_in_progress),
    .i_lm_fsm_state           (w_b02_lm_fsm_state),
    .i_abort_sync_err         (w_b02_abort_sync_err),
    .i_abort_fifo_full        (w_b02_abort_fifo_full),
    .i_abort_unarmed          (w_b02_abort_unarmed),
    .i_deskew_burst_seen      (w_b02_deskew_burst_seen),
    .i_abort_lane_sync_seen   (w_b02_abort_lane_sync_seen),
    .i_active_lanes_applied   (w_b02_active_lanes_applied),
    .i_sp_fifo_not_empty      (w_b03_sp_fifo_not_empty),
    .i_sp_fifo_full           (w_b03_sp_fifo_full),
    .i_null_blank_seen        (w_b03_null_blank_seen),
    .i_dt_conflict            (w_b03_dt_conflict),
    .i_vc_unsupported_seen    (w_b03_vc_unsupported_seen),
    .i_gsp_data               (w_b03_gsp_data),
    .o_gsp_pop                (w_b14_gsp_pop),
    .i_active_sel             (w_b06_active_sel),
    .i_switch_pending         (w_b06_switch_pending),
    .i_linebuffer_full        (w_b07_linebuffer_full),
    .i_linebuf_fill           (w_b14_linebuf_fill),
    .i_frame_count            (w_b09_frame_count),
    .i_timestamp              (w_b09_timestamp),
    .i_frame_id_buf0          (w_b09_frame_id_buf0),
    .i_frame_id_buf1          (w_b09_frame_id_buf1),
    .i_vc0_line_count         (w_b09_vc0_line_count),
    .i_vc0_byte_count         (w_b09_vc0_byte_count),
    .i_vc0_data_type          (w_b09_vc0_data_type),
    .i_vc0_width_mismatch     (w_b09_vc0_width_mismatch),
    .i_vc0_height_mismatch    (w_b09_vc0_height_mismatch),
    .i_vc1_line_count         (w_b09_vc1_line_count),
    .i_vc1_byte_count         (w_b09_vc1_byte_count),
    .i_vc1_data_type          (w_b09_vc1_data_type),
    .i_vc1_width_mismatch     (w_b09_vc1_width_mismatch),
    .i_vc1_height_mismatch    (w_b09_vc1_height_mismatch),
    .i_emb_word_count_vc0     (w_b09_emb_word_count_vc0),
    .i_emb_line_count_vc0     (w_b09_emb_line_count_vc0),
    .i_emb_word_count_vc1     (w_b09_emb_word_count_vc1),
    .i_emb_line_count_vc1     (w_b09_emb_line_count_vc1),
    .i_emb_extract_val_vc0    (w_b09_emb_extract_val_vc0),
    .i_emb_extract_val_vc1    (w_b09_emb_extract_val_vc1),
    .i_meta_overflow          (w_b09_meta_overflow),
    .i_irq_status             (w_b11_irq_status),
    .i_error_capture          (w_b11_error_capture),
    .i_packet_cnt             (w_b11_packet_cnt),
    .i_ecc_err_cnt            (w_b11_ecc_err_cnt),
    .i_crc_err_cnt            (w_b11_crc_err_cnt),
    .i_frame_drop_cnt         (w_b11_frame_drop_cnt),
    .i_lane_abort_cnt         (w_b11_lane_abort_cnt),
    .i_linebuf_ovf_sticky     (w_b11_linebuf_ovf_sticky),
    .i_sp_fifo_ovf_sticky     (w_b11_sp_fifo_ovf_sticky),
    .i_dma_ovf_sticky         (w_b11_dma_ovf_sticky),
    .i_dma_busy               (w_b13_dma_busy),
    .i_active_buffer          (w_b13_active_buffer),
    .i_buf0_state             (w_b13_buf0_state),
    .i_buf1_state             (w_b13_buf1_state),
    .i_buf0_last_valid_line   (w_b13_buf0_last_valid_line),
    .i_buf1_last_valid_line   (w_b13_buf1_last_valid_line),
    .i_axi_fault_awid         (w_b13_axi_fault_awid),
    .i_axi_fault_resp         (w_b13_axi_fault_resp),
    .i_axi_fault_awaddr       (w_b13_axi_fault_awaddr),
    .i_bresp_slverr_count     (w_b13_bresp_slverr_count),
    .i_bresp_decerr_count     (w_b13_bresp_decerr_count)
  );

endmodule

`default_nettype wire
