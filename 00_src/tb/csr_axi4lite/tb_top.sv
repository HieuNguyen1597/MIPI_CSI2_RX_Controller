// File        : tb_top.sv
// Description : Testbench of m_fpt_csr_axi4lite (AXI4-Lite subordinate, Register File, control
//               and status crossings between lite_aclk, sys_clk and rxbyteclkhs, PHY Reset
//               Sequencer).
//               An AXI4-Lite manager drives the accesses. A monitor in lite_aclk checks the
//               AXI4-Lite protocol and latencies, keeps a register model written from the
//               register map and compares every read with it. Status reads are compared with the
//               history of the values the inputs held, so a read must return a value that its
//               source held within the crossing latency (one coherent value per multi-bit
//               register, per bit for single-bit fields). Destination monitors check that each
//               configuration output only shows values that were written, that strobes carry
//               the written masks in one cycle, and that the outputs reach the register value.
//               Models of the connected blocks: Interrupt & Error Aggregator flags, Generic Short
//               Packet FIFO, status sources in sys_clk and rxbyteclkhs, D-PHY levels, and the
//               top-level reset of the sys_clk and rxbyteclkhs domains.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//               TB_CLK_SET selects the clock periods (lite_aclk / sys_clk / rxbyteclkhs):
//               0 = 10 / 6.4 / 8 ns, 1 = 10 / 13 / 21 ns, 2 = 10 / 10.3 / 8 ns.
//
// Design assumptions (agreed with the block owner; the specification does not state them yet):
//   - Extra ports: sys_resetn and rxbyteclkhs_resetn (reset of each destination domain, already
//     merged with full_reset_req and released in its clock by the top level), tpg_cfa_order =
//     TPG_CONTROL[5:4]. The two soft_reset outputs are soft_reset_byte and soft_reset_sys.
//   - Lane vectors are 4 bits. Outputs of lanes not built are 0; PHY_CONFIG bits of those lanes
//     are stored and read back; their LANE_STATUS fields read 0.
//   - N_EXTRACT is 0, 1 or 2; the EMB_EXTRACT offsets of slots not built are reserved.
//     EMB_EXTRACT_CFGm bits 7:3 read 0 and are 0 at emb_extract_cfg.
//   - PHY_RST_CYCLES default is 40. phy_core_rst is high exactly PHY_RST_CYCLES lite_aclk cycles
//     for a write trigger and SYNC_STAGES + 1 + PHY_RST_CYCLES samples after the release of
//     resetn; a trigger during a sequence restarts the count.
//   - The write response of an IRQ_STATUS write waits at most IRQ_CLR_TIMEOUT + 1 cycles.
//   - A read of GENERIC_SHORT_PACKET that returns 0 issues no gsp_pop.
//   - CORE_STATUS.reset_in_progress is also 1 while the Register File is held in reset.
//   - A DECERR read returns data 0. An access in 0x100-0xFFF has no effect, also when address
//     bits 7:2 alias IRQ_STATUS or GENERIC_SHORT_PACKET.
//   - A register with a multi-bit field crosses as a whole; single-bit registers cross per bit.
//     IRQ_STATUS and BUFFER_RELEASE masks written during a transfer are OR-merged.
//   - Crossing latency bound: 2 x (SYNC_STAGES + 2) x (T_lite + T_other) + 4 x T_lite.
//   - The D-PHY enable outputs are not compared while full_reset_req is 1.
//   - Connected blocks: an irq_status bit is cleared in the sys_clk cycle after its strobe, an
//     event in that cycle keeps it set; gsp_data is the FIFO head (0 when empty) and follows a
//     gsp_pop at the next sys_clk edge; status inputs change only after an edge of their clock;
//     sys_clk and lite_aclk always run; rxbyteclkhs may stop low.
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_MAX_LANES       = 32'd4;
  parameter logic [31:0] PARA_N_EXTRACT       = 32'd2;
  parameter logic [31:0] PARA_MAX_OUTSTANDING = 32'd8;
  parameter logic [31:0] PARA_SYNC_STAGES     = 32'd2;
  parameter logic [31:0] PARA_PHY_RST_CYCLES  = 32'd40;
  parameter logic [31:0] PARA_IRQ_CLR_TIMEOUT = 32'd16;
  // Clock set of the testbench (periods in the file header).
  parameter int          TB_CLK_SET           = 0;

  localparam int NL    = int'(PARA_MAX_LANES);
  localparam int NX    = int'(PARA_N_EXTRACT);
  localparam int NXW   = (NX == 0) ? 1 : NX;
  localparam int S     = int'(PARA_SYNC_STAGES);
  localparam int N_RST = int'(PARA_PHY_RST_CYCLES);
  localparam int N_CLR = int'(PARA_IRQ_CLR_TIMEOUT);
  localparam int CFG1_IDX = (NXW > 1) ? 1 : 0;

  localparam logic [3:0] LANE_MASK  = (NL >= 4) ? 4'hF : (NL >= 2) ? 4'h3 : 4'h1;
  localparam logic [1:0] LANES_CODE = (NL >= 4) ? 2'b11 : (NL >= 2) ? 2'b01 : 2'b00;
  localparam logic [31:0] IP_VERSION = 32'h0001_0200;

  localparam real T_LITE = 10.0;
  localparam real T_SYS  = (TB_CLK_SET == 1) ? 13.0 : (TB_CLK_SET == 2) ? 10.3 : 6.4;
  localparam real T_BYTE = (TB_CLK_SET == 1) ? 21.0 : 8.0;
  localparam real LAT_SYS  = 2.0 * (S + 2) * (T_LITE + T_SYS) + 4.0 * T_LITE;
  localparam real LAT_BYTE = 2.0 * (S + 2) * (T_LITE + T_BYTE) + 4.0 * T_LITE;
  localparam real LAT_MAX  = (LAT_SYS > LAT_BYTE) ? LAT_SYS : LAT_BYTE;
  localparam real PRUNE_T  = 12.0 * LAT_MAX;

  // Word index of each register (offset / 4).
  localparam int W_IPV   = 0;
  localparam int W_CORE  = 1;
  localparam int W_PROT  = 2;
  localparam int W_PHY   = 3;
  localparam int W_MD    = 4;
  localparam int W_SC    = 5;
  localparam int W_ER    = 6;
  localparam int W_CST   = 8;
  localparam int W_PST   = 9;
  localparam int W_LST   = 10;
  localparam int W_PKT   = 11;
  localparam int W_FCNT  = 12;
  localparam int W_BST   = 13;
  localparam int W_LAS   = 14;
  localparam int W_GIE   = 16;
  localparam int W_IRQ   = 17;
  localparam int W_IE    = 18;
  localparam int W_ECC   = 19;
  localparam int W_CRC   = 20;
  localparam int W_FDR   = 21;
  localparam int W_LAB   = 22;
  localparam int W_V0I1  = 24;
  localparam int W_V0I2  = 25;
  localparam int W_V1I1  = 26;
  localparam int W_V1I2  = 27;
  localparam int W_GSP   = 28;
  localparam int W_EMB0  = 29;
  localparam int W_EMB1  = 30;
  localparam int W_RID   = 31;
  localparam int W_FID   = 32;
  localparam int W_TSLO  = 33;
  localparam int W_TSHI  = 34;
  localparam int W_DMAC  = 36;
  localparam int W_B0    = 37;
  localparam int W_B1    = 38;
  localparam int W_BSZ   = 39;
  localparam int W_ATTR  = 40;
  localparam int W_DST   = 41;
  localparam int W_LP    = 42;
  localparam int W_BOWN  = 43;
  localparam int W_REL   = 44;
  localparam int W_BERR  = 45;
  localparam int W_META  = 46;
  localparam int W_TPGC  = 48;
  localparam int W_ECAP  = 49;
  localparam int W_FTR   = 50;
  localparam int W_FAD   = 51;
  localparam int W_AXE   = 52;
  localparam int W_TR    = 53;
  localparam int W_TB    = 54;
  localparam int W_CFG0  = 56;
  localparam int W_CFG1  = 57;
  localparam int W_VAL00 = 58;
  localparam int W_VAL01 = 59;
  localparam int W_VAL10 = 60;
  localparam int W_VAL11 = 61;
  localparam int W_RIP   = 64;  // history of CORE_STATUS.reset_in_progress only

  // --------------------------------------------------------------------------------------------
  // DUT signals
  // --------------------------------------------------------------------------------------------
  logic        i_lite_aclk = 1'b0;
  logic        i_lite_aresetn;
  logic        i_resetn;
  logic        i_sys_clk = 1'b0;
  logic        i_rxbyteclkhs = 1'b0;
  logic        i_sys_resetn;
  logic        i_rxbyteclkhs_resetn;
  logic [11:0] i_s_axi_awaddr;
  logic [2:0]  i_s_axi_awprot;
  logic        i_s_axi_awvalid;
  logic        o_s_axi_awready;
  logic [31:0] i_s_axi_wdata;
  logic [3:0]  i_s_axi_wstrb;
  logic        i_s_axi_wvalid;
  logic        o_s_axi_wready;
  logic [1:0]  o_s_axi_bresp;
  logic        o_s_axi_bvalid;
  logic        i_s_axi_bready;
  logic [11:0] i_s_axi_araddr;
  logic [2:0]  i_s_axi_arprot;
  logic        i_s_axi_arvalid;
  logic        o_s_axi_arready;
  logic [31:0] o_s_axi_rdata;
  logic [1:0]  o_s_axi_rresp;
  logic        o_s_axi_rvalid;
  logic        i_s_axi_rready;
  logic        o_phy_core_rst;
  logic        o_cl_enable;
  logic [3:0]  o_dl_enable;
  logic [3:0]  o_forcerxmode;
  logic        o_full_reset_req;
  logic        i_cl_stopstate;
  logic        i_cl_ulpsactivenot;
  logic        i_cl_rxclkactivehs;
  logic        i_cl_ulpsclknot;
  logic        i_init_done;
  logic [3:0]  i_stopstate;
  logic [3:0]  i_rxskewcalhs;
  logic [3:0]  i_errsoths;
  logic [3:0]  i_errsotsynchs;
  logic        o_soft_reset_byte;
  logic [1:0]  o_active_lanes;
  logic        o_soft_reset_sys;
  logic        o_core_enable;
  logic        o_vc_exposure_map;
  logic [5:0]  o_metadata_dt;
  logic        o_metadata_capture_en;
  logic        o_fnum_check_en;
  logic        o_emb_extract_en;
  logic [15:0] o_expected_width;
  logic [15:0] o_expected_height;
  logic [31:0] o_metadata_record_id;
  logic [NXW-1:0][31:0] o_emb_extract_cfg;
  logic        o_axis_enable;
  logic        o_stream_format;
  logic        o_stream_follow_dma_drop;
  logic        o_tpg_enable;
  logic        o_tpg_pattern;
  logic        o_tpg_dual_vc_en;
  logic        o_tpg_pixel_format;
  logic [1:0]  o_tpg_cfa_order;
  logic [15:0] o_tpg_width;
  logic [15:0] o_tpg_height;
  logic [15:0] o_tpg_hblank;
  logic [15:0] o_tpg_vblank;
  logic        o_dma_enable;
  logic        o_dma_start;
  logic        o_dma_abort;
  logic        o_err_frame_policy;
  logic        o_dma_format;
  logic [31:0] o_dma_buffer0_addr;
  logic [31:0] o_dma_buffer1_addr;
  logic [31:0] o_dma_buffer_size;
  logic [31:0] o_meta_area_size;
  logic [15:0] o_bytes_per_line;
  logic [7:0]  o_line_padding;
  logic [3:0]  o_awqos;
  logic [7:0]  o_max_burst_len;
  logic [2:0]  o_awprot;
  logic [1:0]  o_release_buf;
  logic        o_global_irq_enable;
  logic [23:0] o_irq_enable_mask;
  logic [23:0] o_irq_clear_strobe;
  logic        o_phy_rst_in_progress;
  logic [2:0]  i_lm_fsm_state;
  logic        i_abort_sync_err;
  logic        i_abort_fifo_full;
  logic        i_abort_unarmed;
  logic        i_deskew_burst_seen;
  logic [3:0]  i_abort_lane_sync_seen;
  logic [1:0]  i_active_lanes_applied;
  logic        i_sp_fifo_not_empty;
  logic        i_sp_fifo_full;
  logic        i_null_blank_seen;
  logic        i_dt_conflict;
  logic        i_vc_unsupported_seen;
  logic [23:0] i_gsp_data;
  logic        o_gsp_pop;
  logic        i_active_sel;
  logic        i_switch_pending;
  logic        i_linebuffer_full;
  logic [15:0] i_linebuf_fill;
  logic [31:0] i_frame_count;
  logic [63:0] i_timestamp;
  logic [13:0] i_frame_id_buf0;
  logic [13:0] i_frame_id_buf1;
  logic [15:0] i_vc0_line_count;
  logic [15:0] i_vc0_byte_count;
  logic [5:0]  i_vc0_data_type;
  logic        i_vc0_width_mismatch;
  logic        i_vc0_height_mismatch;
  logic [15:0] i_vc1_line_count;
  logic [15:0] i_vc1_byte_count;
  logic [5:0]  i_vc1_data_type;
  logic        i_vc1_width_mismatch;
  logic        i_vc1_height_mismatch;
  logic [15:0] i_emb_word_count_vc0;
  logic [7:0]  i_emb_line_count_vc0;
  logic [15:0] i_emb_word_count_vc1;
  logic [7:0]  i_emb_line_count_vc1;
  logic [NXW-1:0][31:0] i_emb_extract_val_vc0;
  logic [NXW-1:0][31:0] i_emb_extract_val_vc1;
  logic        i_meta_overflow;
  logic [23:0] i_irq_status;
  logic [31:0] i_error_capture;
  logic [31:0] i_packet_cnt;
  logic [31:0] i_ecc_err_cnt;
  logic [31:0] i_crc_err_cnt;
  logic [31:0] i_frame_drop_cnt;
  logic [15:0] i_lane_abort_cnt;
  logic        i_linebuf_ovf_sticky;
  logic        i_sp_fifo_ovf_sticky;
  logic        i_dma_ovf_sticky;
  logic        i_dma_busy;
  logic        i_active_buffer;
  logic [1:0]  i_buf0_state;
  logic [1:0]  i_buf1_state;
  logic [15:0] i_buf0_last_valid_line;
  logic [15:0] i_buf1_last_valid_line;
  logic [7:0]  i_axi_fault_awid;
  logic [1:0]  i_axi_fault_resp;
  logic [31:0] i_axi_fault_awaddr;
  logic [15:0] i_bresp_slverr_count;
  logic [15:0] i_bresp_decerr_count;

  m_fpt_csr_axi4lite #(
    .PARA_MAX_LANES       (PARA_MAX_LANES),
    .PARA_N_EXTRACT       (PARA_N_EXTRACT),
    .PARA_MAX_OUTSTANDING (PARA_MAX_OUTSTANDING),
    .PARA_SYNC_STAGES     (PARA_SYNC_STAGES),
    .PARA_PHY_RST_CYCLES  (PARA_PHY_RST_CYCLES),
    .PARA_IRQ_CLR_TIMEOUT (PARA_IRQ_CLR_TIMEOUT)
  ) u_dut (
    .i_lite_aclk              (i_lite_aclk),
    .i_lite_aresetn           (i_lite_aresetn),
    .i_resetn                 (i_resetn),
    .i_sys_clk                (i_sys_clk),
    .i_rxbyteclkhs            (i_rxbyteclkhs),
    .i_sys_resetn             (i_sys_resetn),
    .i_rxbyteclkhs_resetn     (i_rxbyteclkhs_resetn),
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
    .o_phy_core_rst           (o_phy_core_rst),
    .o_cl_enable              (o_cl_enable),
    .o_dl_enable              (o_dl_enable),
    .o_forcerxmode            (o_forcerxmode),
    .o_full_reset_req         (o_full_reset_req),
    .i_cl_stopstate           (i_cl_stopstate),
    .i_cl_ulpsactivenot       (i_cl_ulpsactivenot),
    .i_cl_rxclkactivehs       (i_cl_rxclkactivehs),
    .i_cl_ulpsclknot          (i_cl_ulpsclknot),
    .i_init_done              (i_init_done),
    .i_stopstate              (i_stopstate),
    .i_rxskewcalhs            (i_rxskewcalhs),
    .i_errsoths               (i_errsoths),
    .i_errsotsynchs           (i_errsotsynchs),
    .o_soft_reset_byte        (o_soft_reset_byte),
    .o_active_lanes           (o_active_lanes),
    .o_soft_reset_sys         (o_soft_reset_sys),
    .o_core_enable            (o_core_enable),
    .o_vc_exposure_map        (o_vc_exposure_map),
    .o_metadata_dt            (o_metadata_dt),
    .o_metadata_capture_en    (o_metadata_capture_en),
    .o_fnum_check_en          (o_fnum_check_en),
    .o_emb_extract_en         (o_emb_extract_en),
    .o_expected_width         (o_expected_width),
    .o_expected_height        (o_expected_height),
    .o_metadata_record_id     (o_metadata_record_id),
    .o_emb_extract_cfg        (o_emb_extract_cfg),
    .o_axis_enable            (o_axis_enable),
    .o_stream_format          (o_stream_format),
    .o_stream_follow_dma_drop (o_stream_follow_dma_drop),
    .o_tpg_enable             (o_tpg_enable),
    .o_tpg_pattern            (o_tpg_pattern),
    .o_tpg_dual_vc_en         (o_tpg_dual_vc_en),
    .o_tpg_pixel_format       (o_tpg_pixel_format),
    .o_tpg_cfa_order          (o_tpg_cfa_order),
    .o_tpg_width              (o_tpg_width),
    .o_tpg_height             (o_tpg_height),
    .o_tpg_hblank             (o_tpg_hblank),
    .o_tpg_vblank             (o_tpg_vblank),
    .o_dma_enable             (o_dma_enable),
    .o_dma_start              (o_dma_start),
    .o_dma_abort              (o_dma_abort),
    .o_err_frame_policy       (o_err_frame_policy),
    .o_dma_format             (o_dma_format),
    .o_dma_buffer0_addr       (o_dma_buffer0_addr),
    .o_dma_buffer1_addr       (o_dma_buffer1_addr),
    .o_dma_buffer_size        (o_dma_buffer_size),
    .o_meta_area_size         (o_meta_area_size),
    .o_bytes_per_line         (o_bytes_per_line),
    .o_line_padding           (o_line_padding),
    .o_awqos                  (o_awqos),
    .o_max_burst_len          (o_max_burst_len),
    .o_awprot                 (o_awprot),
    .o_release_buf            (o_release_buf),
    .o_global_irq_enable      (o_global_irq_enable),
    .o_irq_enable_mask        (o_irq_enable_mask),
    .o_irq_clear_strobe       (o_irq_clear_strobe),
    .o_phy_rst_in_progress    (o_phy_rst_in_progress),
    .i_lm_fsm_state           (i_lm_fsm_state),
    .i_abort_sync_err         (i_abort_sync_err),
    .i_abort_fifo_full        (i_abort_fifo_full),
    .i_abort_unarmed          (i_abort_unarmed),
    .i_deskew_burst_seen      (i_deskew_burst_seen),
    .i_abort_lane_sync_seen   (i_abort_lane_sync_seen),
    .i_active_lanes_applied   (i_active_lanes_applied),
    .i_sp_fifo_not_empty      (i_sp_fifo_not_empty),
    .i_sp_fifo_full           (i_sp_fifo_full),
    .i_null_blank_seen        (i_null_blank_seen),
    .i_dt_conflict            (i_dt_conflict),
    .i_vc_unsupported_seen    (i_vc_unsupported_seen),
    .i_gsp_data               (i_gsp_data),
    .o_gsp_pop                (o_gsp_pop),
    .i_active_sel             (i_active_sel),
    .i_switch_pending         (i_switch_pending),
    .i_linebuffer_full        (i_linebuffer_full),
    .i_linebuf_fill           (i_linebuf_fill),
    .i_frame_count            (i_frame_count),
    .i_timestamp              (i_timestamp),
    .i_frame_id_buf0          (i_frame_id_buf0),
    .i_frame_id_buf1          (i_frame_id_buf1),
    .i_vc0_line_count         (i_vc0_line_count),
    .i_vc0_byte_count         (i_vc0_byte_count),
    .i_vc0_data_type          (i_vc0_data_type),
    .i_vc0_width_mismatch     (i_vc0_width_mismatch),
    .i_vc0_height_mismatch    (i_vc0_height_mismatch),
    .i_vc1_line_count         (i_vc1_line_count),
    .i_vc1_byte_count         (i_vc1_byte_count),
    .i_vc1_data_type          (i_vc1_data_type),
    .i_vc1_width_mismatch     (i_vc1_width_mismatch),
    .i_vc1_height_mismatch    (i_vc1_height_mismatch),
    .i_emb_word_count_vc0     (i_emb_word_count_vc0),
    .i_emb_line_count_vc0     (i_emb_line_count_vc0),
    .i_emb_word_count_vc1     (i_emb_word_count_vc1),
    .i_emb_line_count_vc1     (i_emb_line_count_vc1),
    .i_emb_extract_val_vc0    (i_emb_extract_val_vc0),
    .i_emb_extract_val_vc1    (i_emb_extract_val_vc1),
    .i_meta_overflow          (i_meta_overflow),
    .i_irq_status             (i_irq_status),
    .i_error_capture          (i_error_capture),
    .i_packet_cnt             (i_packet_cnt),
    .i_ecc_err_cnt            (i_ecc_err_cnt),
    .i_crc_err_cnt            (i_crc_err_cnt),
    .i_frame_drop_cnt         (i_frame_drop_cnt),
    .i_lane_abort_cnt         (i_lane_abort_cnt),
    .i_linebuf_ovf_sticky     (i_linebuf_ovf_sticky),
    .i_sp_fifo_ovf_sticky     (i_sp_fifo_ovf_sticky),
    .i_dma_ovf_sticky         (i_dma_ovf_sticky),
    .i_dma_busy               (i_dma_busy),
    .i_active_buffer          (i_active_buffer),
    .i_buf0_state             (i_buf0_state),
    .i_buf1_state             (i_buf1_state),
    .i_buf0_last_valid_line   (i_buf0_last_valid_line),
    .i_buf1_last_valid_line   (i_buf1_last_valid_line),
    .i_axi_fault_awid         (i_axi_fault_awid),
    .i_axi_fault_resp         (i_axi_fault_resp),
    .i_axi_fault_awaddr       (i_axi_fault_awaddr),
    .i_bresp_slverr_count     (i_bresp_slverr_count),
    .i_bresp_decerr_count     (i_bresp_decerr_count)
  );

  // --------------------------------------------------------------------------------------------
  // Error reporting and small helpers
  // --------------------------------------------------------------------------------------------
  int unsigned err_count   = 0;
  int unsigned check_count = 0;
  string       where       = "start";
  bit          started     = 1'b0;
  bit          warm        = 1'b0;  // reset checks start after the first clock edges
  bit          init_done   = 1'b0;
  int          lcyc        = 0;

  task automatic fail(input string msg);
    err_count++;
    if (err_count <= 40) $error("t=%0t lite cycle %0d [%s]: %s", $realtime, lcyc, where, msg);
  endtask

  task automatic expect_eq(input string what, input logic [63:0] got, input logic [63:0] want);
    check_count++;
    if (got !== want) fail($sformatf("%s = 0x%0h, expected 0x%0h", what, got, want));
  endtask

  task automatic expect_true(input string what, input bit cond);
    check_count++;
    if (!cond) fail(what);
  endtask

  // Wait n rising edges of lite_aclk, plus 1 ns. AXI4-Lite inputs change at that point.
  task automatic lclk(input int n = 1);
    repeat (n) @(posedge i_lite_aclk);
    #1.0;
  endtask

  // Wait for a point where sys_clk and rxbyteclkhs inputs may change (after a rising edge).
  task automatic sclk(input int n = 1);
    repeat (n) @(posedge i_sys_clk);
    #0.6;
  endtask

  task automatic bclk(input int n = 1);
    repeat (n) @(posedge i_rxbyteclkhs);
    #0.6;
  endtask

  // Wait longer than every crossing latency.
  task automatic settle();
    #(LAT_MAX + 6.0 * T_LITE + 4.0 * T_SYS + 4.0 * T_BYTE);
    lclk();
  endtask

  // --------------------------------------------------------------------------------------------
  // Clocks and the top-level reset of the sys_clk and rxbyteclkhs domains
  // --------------------------------------------------------------------------------------------
  bit      byte_run     = 1'b1;
  realtime byte_stop_t  = -1.0e9;
  realtime byte_start_t = 0.0;

  initial forever #(T_LITE / 2.0) i_lite_aclk = ~i_lite_aclk;

  initial begin
    #1.3;
    forever #(T_SYS / 2.0) i_sys_clk = ~i_sys_clk;
  end

  // rxbyteclkhs stops low when byte_run is 0.
  initial begin
    #2.7;
    forever begin
      #(T_BYTE / 2.0);
      if (byte_run || i_rxbyteclkhs) i_rxbyteclkhs = ~i_rxbyteclkhs;
    end
  end

  task automatic byte_clock(input bit on);
    if (on && !byte_run) begin
      byte_run     = 1'b1;
      byte_start_t = $realtime;
    end else if (!on && byte_run) begin
      byte_run = 1'b0;
      wait (i_rxbyteclkhs == 1'b0);
      byte_stop_t = $realtime;
    end
  endtask

  // Reset of each destination domain: asserted while resetn is 0 or full_reset_req is 1,
  // released after SYNC_STAGES edges of its clock.
  logic [31:0] sys_rst_sh  = '0;
  logic [31:0] byte_rst_sh = '0;
  wire         dst_rst_src_n = i_resetn && (o_full_reset_req !== 1'b1);

  always @(posedge i_sys_clk or negedge dst_rst_src_n) begin
    if (!dst_rst_src_n) sys_rst_sh <= '0;
    else                sys_rst_sh <= {sys_rst_sh[30:0], 1'b1};
  end

  always @(posedge i_rxbyteclkhs or negedge dst_rst_src_n) begin
    if (!dst_rst_src_n) byte_rst_sh <= '0;
    else                byte_rst_sh <= {byte_rst_sh[30:0], 1'b1};
  end

  assign i_sys_resetn         = sys_rst_sh[(S > 0) ? S - 1 : 0];
  assign i_rxbyteclkhs_resetn = byte_rst_sh[(S > 0) ? S - 1 : 0];

  realtime sys_rel_t  = 1.0e9;
  realtime byte_rel_t = 1.0e9;
  always @(posedge i_sys_resetn)         sys_rel_t  = $realtime;
  always @(posedge i_rxbyteclkhs_resetn) byte_rel_t = $realtime;

  // --------------------------------------------------------------------------------------------
  // Register model (written from the register map)
  // --------------------------------------------------------------------------------------------
  logic [31:0] rg [0:63];
  logic [31:0] ts_shadow;

  function automatic bit reserved_w(input int w);
    case (w)
      7, 15, 23, 35, 47, 55, 62, 63:  return 1'b1;
      W_CFG0, W_VAL00, W_VAL10:       return (NX < 1);
      W_CFG1, W_VAL01, W_VAL11:       return (NX < 2);
      default:                        return 1'b0;
    endcase
  endfunction

  // Bits stored by a write (RW fields).
  function automatic logic [31:0] rw_mask(input int w);
    case (w)
      W_CORE:  return 32'h0000_0003;
      W_PROT:  return 32'h0000_1003;
      W_PHY:   return 32'h0000_07FD;
      W_MD:    return 32'h0000_073F;
      W_SC:    return 32'h0000_0007;
      W_ER, W_RID, W_B0, W_B1, W_BSZ, W_META, W_TR, W_TB: return 32'hFFFF_FFFF;
      W_GIE:   return 32'h0000_0001;
      W_IE:    return 32'h00FF_FFFF;
      W_DMAC:  return 32'h0000_001F;
      W_ATTR:  return 32'h0000_7FFF;
      W_LP:    return 32'h00FF_FFFF;
      W_TPGC:  return 32'h0000_003F;
      W_CFG0:  return (NX >= 1) ? 32'hFFFF_FF07 : 32'h0;
      W_CFG1:  return (NX >= 2) ? 32'hFFFF_FF07 : 32'h0;
      default: return 32'h0;
    endcase
  endfunction

  function automatic logic [31:0] rst_val(input int w);
    case (w)
      W_CORE:  return 32'h0000_0001;
      W_PROT:  return 32'h0000_0003;
      W_PHY:   return 32'h0000_0078;
      W_MD:    return 32'h0000_0012;
      W_SC:    return 32'h0000_0001;
      W_DMAC:  return 32'h0000_0010;
      W_ATTR:  return 32'h0000_0FF0;
      default: return 32'h0;
    endcase
  endfunction

  // D-PHY enable outputs {forcerxmode, dl_enable, cl_enable} from a PHY_CONFIG value.
  function automatic logic [8:0] phy_f(input logic [31:0] v);
    return {v[10:7] & LANE_MASK, {4{v[0]}} & v[6:3] & LANE_MASK, v[0] & v[2]};
  endfunction

  // Destination groups of the multi-bit registers: the output value packed as in the register.
  localparam int NG = 16;

  function automatic int grp_w(input int g);
    case (g)
      0: return W_PROT;   1: return W_MD;     2: return W_ER;     3: return W_RID;
      4: return W_CFG0;   5: return W_CFG1;   6: return W_TPGC;   7: return W_TR;
      8: return W_TB;     9: return W_B0;     10: return W_B1;    11: return W_BSZ;
      12: return W_META;  13: return W_LP;    14: return W_ATTR;  default: return W_IE;
    endcase
  endfunction

  function automatic logic [31:0] grp_mask(input int g);
    case (g)
      0:       return 32'h0000_0003;
      1:       return 32'h0000_073F;
      4, 5:    return 32'hFFFF_FF07;
      6:       return 32'h0000_003E;
      13:      return 32'h00FF_FFFF;
      14:      return 32'h0000_7FFF;
      15:      return 32'h00FF_FFFF;
      default: return 32'hFFFF_FFFF;
    endcase
  endfunction

  function automatic bit grp_on(input int g);
    if (g == 4) return (NX >= 1);
    if (g == 5) return (NX >= 2);
    return 1'b1;
  endfunction

  function automatic logic [31:0] grp_out(input int g);
    case (g)
      0:  return {30'b0, o_active_lanes};
      1:  return {21'b0, o_emb_extract_en, o_fnum_check_en, o_metadata_capture_en, 2'b0,
                  o_metadata_dt};
      2:  return {o_expected_height, o_expected_width};
      3:  return o_metadata_record_id;
      4:  return o_emb_extract_cfg[0];
      5:  return o_emb_extract_cfg[CFG1_IDX];
      6:  return {26'b0, o_tpg_cfa_order, o_tpg_pixel_format, o_tpg_dual_vc_en, o_tpg_pattern,
                  1'b0};
      7:  return {o_tpg_height, o_tpg_width};
      8:  return {o_tpg_vblank, o_tpg_hblank};
      9:  return o_dma_buffer0_addr;
      10: return o_dma_buffer1_addr;
      11: return o_dma_buffer_size;
      12: return o_meta_area_size;
      13: return {8'b0, o_line_padding, o_bytes_per_line};
      14: return {17'b0, o_awprot, o_max_burst_len, o_awqos};
      default: return {8'b0, o_irq_enable_mask};
    endcase
  endfunction

  logic [31:0] ghist  [0:NG-1][$];
  logic [31:0] g_last [0:NG-1];

  task automatic ghist_push(input int w);
    for (int g = 0; g < NG; g++) begin
      if (grp_w(g) == w) begin
        ghist[g].push_back(rg[w] & grp_mask(g));
        if (ghist[g].size() > 64) void'(ghist[g].pop_front());
      end
    end
  endtask

  // Reset-related model state.
  bit      rf_in_reset     = 1'b1;
  int      rf_rel_cnt      = 0;
  realtime rf_last_reset_t = 0.0;
  realtime t_sr_fall       = 0.0;
  logic [31:0] phy_prev_val;
  int      phy_chg_cyc     = 0;

  task automatic model_reset();
    phy_prev_val = rg[W_PHY];
    phy_chg_cyc  = lcyc;
    for (int w = 0; w < 64; w++) rg[w] = rst_val(w);
    for (int w = 0; w < 64; w++) ghist_push(w);
    ts_shadow    = 32'h0;
  endtask

  // --------------------------------------------------------------------------------------------
  // Status history: the value of each status register computed from the inputs, with the time
  // at which it started.
  // --------------------------------------------------------------------------------------------
  typedef struct {
    realtime     t;
    logic [63:0] v;
  } hent_t;

  hent_t hist [0:64][$];

  function automatic bit is_status(input int w);
    case (w)
      W_PROT, W_CST, W_PST, W_LST, W_PKT, W_FCNT, W_BST, W_LAS, W_IRQ, W_ECC, W_CRC, W_FDR,
      W_LAB, W_V0I1, W_V0I2, W_V1I1, W_V1I2, W_EMB0, W_EMB1, W_FID, W_TSLO, W_DST, W_BOWN,
      W_BERR, W_ECAP, W_FTR, W_FAD, W_AXE:  return 1'b1;
      W_VAL00, W_VAL10:                     return (NX >= 1);
      W_VAL01, W_VAL11:                     return (NX >= 2);
      default:                              return 1'b0;
    endcase
  endfunction

  // 0: sys_clk or asynchronous source, 1: rxbyteclkhs source, 2: lite_aclk.
  function automatic int st_dom(input int w);
    if (w == W_PROT || w == W_LAS || w == W_LST) return 1;
    if (w == W_RIP) return 2;
    return 0;
  endfunction

  // Bits that exist in the register (status part only).
  function automatic logic [63:0] st_fmask(input int w);
    case (w)
      W_PROT:  return 64'h0000_0C00;
      W_CST:   return 64'h0000_10FE;
      W_PST:   return 64'h0000_001F;
      W_LST:   return 64'h0909_0909;
      W_BST:   return 64'h000F_FFFF;
      W_LAS:   return 64'h0000_0F7F;
      W_IRQ:   return 64'h00FF_FFFF;
      W_LAB:   return 64'h0000_FFFF;
      W_V0I2, W_V1I2: return 64'h0000_00FF;
      W_EMB0, W_EMB1: return 64'h00FF_FFFF;
      W_FID:   return 64'h3FFF_3FFF;
      W_TSLO:  return 64'hFFFF_FFFF_FFFF_FFFF;
      W_DST:   return 64'h0000_0003;
      W_BOWN:  return 64'h0000_000F;
      W_FTR:   return 64'h0000_03FF;
      W_RIP:   return 64'h0000_0001;
      default: return 64'hFFFF_FFFF;
    endcase
  endfunction

  // Bits read as one coherent value (multi-bit values); the other bits are checked per bit.
  function automatic logic [63:0] st_cmask(input int w);
    case (w)
      W_CST, W_PST, W_LST, W_IRQ, W_DST, W_RIP: return 64'h0;
      W_BST:   return 64'h0000_FFFF;
      W_V0I2, W_V1I2: return 64'h0000_003F;
      default: return st_fmask(w);
    endcase
  endfunction

  function automatic logic [63:0] st_val(input int w);
    logic [63:0] v;
    v = '0;
    case (w)
      W_PROT: v = {52'b0, i_active_lanes_applied, 10'b0};
      W_CST:  v = {51'b0, i_vc_unsupported_seen, 4'b0, i_dt_conflict, i_switch_pending,
                   i_active_sel, i_null_blank_seen, i_sp_fifo_full, i_sp_fifo_not_empty,
                   i_linebuffer_full, 1'b0};
      W_PST:  v = {59'b0, i_init_done, ~i_cl_ulpsclknot, i_cl_rxclkactivehs, ~i_cl_ulpsactivenot,
                   i_cl_stopstate};
      W_LST: begin
        for (int x = 0; x < 4; x++) begin
          if (x < NL) begin
            v[8 * x]     = i_stopstate[x];
            v[8 * x + 3] = i_rxskewcalhs[x];
          end
        end
      end
      W_PKT:  v = {32'b0, i_packet_cnt};
      W_FCNT: v = {32'b0, i_frame_count};
      W_BST:  v = {44'b0, i_dma_ovf_sticky, i_meta_overflow, i_sp_fifo_ovf_sticky,
                   i_linebuf_ovf_sticky, i_linebuf_fill};
      W_LAS:  v = {52'b0, i_abort_lane_sync_seen, 1'b0, i_deskew_burst_seen, i_abort_unarmed,
                   i_abort_fifo_full, i_abort_sync_err, i_lm_fsm_state};
      W_IRQ:  v = {40'b0, i_irq_status};
      W_ECC:  v = {32'b0, i_ecc_err_cnt};
      W_CRC:  v = {32'b0, i_crc_err_cnt};
      W_FDR:  v = {32'b0, i_frame_drop_cnt};
      W_LAB:  v = {48'b0, i_lane_abort_cnt};
      W_V0I1: v = {32'b0, i_vc0_line_count, i_vc0_byte_count};
      W_V0I2: v = {56'b0, i_vc0_height_mismatch, i_vc0_width_mismatch, i_vc0_data_type};
      W_V1I1: v = {32'b0, i_vc1_line_count, i_vc1_byte_count};
      W_V1I2: v = {56'b0, i_vc1_height_mismatch, i_vc1_width_mismatch, i_vc1_data_type};
      W_EMB0: v = {40'b0, i_emb_line_count_vc0, i_emb_word_count_vc0};
      W_EMB1: v = {40'b0, i_emb_line_count_vc1, i_emb_word_count_vc1};
      W_FID:  v = {34'b0, i_frame_id_buf1, 2'b0, i_frame_id_buf0};
      W_TSLO: v = i_timestamp;
      W_DST:  v = {62'b0, i_active_buffer, i_dma_busy};
      W_BOWN: v = {60'b0, i_buf1_state, i_buf0_state};
      W_BERR: v = {32'b0, i_buf1_last_valid_line, i_buf0_last_valid_line};
      W_ECAP: v = {32'b0, i_error_capture};
      W_FTR:  v = {54'b0, i_axi_fault_resp, i_axi_fault_awid};
      W_FAD:  v = {32'b0, i_axi_fault_awaddr};
      W_AXE:  v = {32'b0, i_bresp_decerr_count, i_bresp_slverr_count};
      W_VAL00: v = {32'b0, i_emb_extract_val_vc0[0]};
      W_VAL01: v = {32'b0, i_emb_extract_val_vc0[CFG1_IDX]};
      W_VAL10: v = {32'b0, i_emb_extract_val_vc1[0]};
      W_VAL11: v = {32'b0, i_emb_extract_val_vc1[CFG1_IDX]};
      default: v = '0;
    endcase
    return v;
  endfunction

  task automatic hist_push(input int w, input logic [63:0] v);
    hent_t e;
    if (hist[w].size() == 0 || hist[w][hist[w].size() - 1].v !== v) begin
      e.t = $realtime;
      e.v = v;
      hist[w].push_back(e);
      while (hist[w].size() > 2 && hist[w][1].t < $realtime - PRUNE_T) void'(hist[w].pop_front());
      while (hist[w].size() > 4096) void'(hist[w].pop_front());
    end
  endtask

  // Record the current status values; called after any change of a status input.
  task automatic rec_all();
    for (int w = 0; w < 64; w++) if (is_status(w)) hist_push(w, st_val(w));
  endtask

  // A read of a status register must return, per coherent part and per single bit, a value
  // that the source held in [t - latency, t]. After a Register File reset the value 0 is also
  // accepted until the first transfer after it.
  task automatic check_st(input int w, input logic [63:0] got, input realtime t);
    real         lat;
    real         margin;
    realtime     t_lo;
    bit          zero_ok;
    bit          ok;
    logic [63:0] fm;
    logic [63:0] cm;
    logic [63:0] pm;
    realtime     t_next;
    int          n;
    lat    = (st_dom(w) == 1) ? LAT_BYTE : (st_dom(w) == 2) ? 4.0 * T_LITE : LAT_SYS;
    margin = 4.0 * T_LITE + 2.0 * ((st_dom(w) == 1) ? T_BYTE : T_SYS);
    t_lo   = t - lat - margin;
    if (st_dom(w) == 1 && (!byte_run || t - byte_start_t < lat + margin)) begin
      if (byte_stop_t - lat - margin < t_lo) t_lo = byte_stop_t - lat - margin;
    end
    zero_ok = (st_dom(w) != 2) && (t - rf_last_reset_t < lat + margin);
    fm = st_fmask(w);
    cm = st_cmask(w) & fm;
    pm = fm & ~cm;
    n  = hist[w].size();
    check_count++;
    if ((got & ~fm) != 0) fail($sformatf("word 0x%0h: reserved bits set, read 0x%0h", w * 4, got));
    if (cm != 0) begin
      ok = zero_ok && ((got & cm) == 0);
      for (int i = 0; i < n; i++) begin
        t_next = (i + 1 < n) ? hist[w][i + 1].t : t + 1.0;
        if (hist[w][i].t <= t && t_next > t_lo && ((hist[w][i].v & cm) == (got & cm))) ok = 1'b1;
      end
      if (!ok) fail($sformatf("word 0x%0h: read 0x%0h, no source value in the window (last 0x%0h)",
                              w * 4, got & cm, (n > 0) ? (hist[w][n - 1].v & cm) : 64'h0));
    end
    for (int b = 0; b < 64; b++) begin
      if (pm[b]) begin
        ok = zero_ok && (got[b] == 1'b0);
        for (int i = 0; i < n; i++) begin
          t_next = (i + 1 < n) ? hist[w][i + 1].t : t + 1.0;
          if (hist[w][i].t <= t && t_next > t_lo && hist[w][i].v[b] == got[b]) ok = 1'b1;
        end
        if (!ok) fail($sformatf("word 0x%0h bit %0d: read %0b, source did not hold it", w * 4, b,
                                got[b]));
      end
    end
  endtask

  // --------------------------------------------------------------------------------------------
  // Models of the connected blocks and status sources
  // --------------------------------------------------------------------------------------------
  // Interrupt & Error Aggregator flags.
  logic [23:0] irq_q            = '0;
  logic [23:0] irq_events_once  = '0;
  logic [23:0] irq_persist      = '0;
  int          irq_rate         = 0;    // per mille per bit and sys_clk cycle

  // Generic Short Packet FIFO.
  logic [23:0] gsp_fifo   [$];
  logic [23:0] gsp_expect [$];
  int          gsp_pops     = 0;
  int          gsp_nz_reads = 0;
  bit          gsp_relax    = 1'b0;

  // Activity of the status sources (percent of cycles with a change).
  int          sys_act   = 0;
  int          byte_act  = 0;
  bit          async_act = 1'b0;
  int          sot_rand  = 0;           // per mille per lane and rxbyteclkhs cycle
  bit          ts_run    = 1'b0;
  logic [63:0] ts_step   = 64'd1;

  // SoT pulse requests from the tests and the time of the last pulse per lane.
  bit          sot_req [0:1][0:3];
  realtime     sot_t   [0:1][0:3];
  realtime     sot_tprev [0:1][0:3];
  logic [3:0]  sot_last  [0:1];
  bit          sot_par   [0:1][0:3];      // pulse count parity since the last byte reset

  // Strobe masks written and not yet seen at the outputs: 0 IRQ_STATUS, 1 BUFFER_RELEASE.
  logic [23:0] pend_m [0:1][$];
  realtime     pend_t [0:1][$];
  bit          pend_o [0:1][$];   // 1: may already have been merged into an earlier pulse
  int          rel_pulses [0:1];
  logic [23:0] irq_cur_mask  = '0;
  logic [23:0] irq_zero_bits = '0;  // written bits seen at 0 since the IRQ_STATUS write

  task automatic irq_set(input logic [23:0] m);
    irq_events_once = irq_events_once | m;
    sclk(2);
  endtask

  task automatic gsp_push(input logic [23:0] v);
    gsp_fifo.push_back(v);
    gsp_expect.push_back(v);
  endtask

  task automatic sys_activity();
    int f;
    f = $urandom_range(23, 0);
    case (f)
      0:  i_sp_fifo_not_empty    = $urandom;
      1:  i_sp_fifo_full         = $urandom;
      2:  i_null_blank_seen      = $urandom;
      3:  i_dt_conflict          = $urandom;
      4:  i_vc_unsupported_seen  = $urandom;
      5:  begin i_active_sel = $urandom; i_switch_pending = $urandom; end
      6:  begin i_linebuffer_full = $urandom; i_linebuf_fill = $urandom; end
      7:  i_frame_count          = i_frame_count + 1;
      8:  begin i_frame_id_buf0 = $urandom; i_frame_id_buf1 = $urandom; end
      9:  begin i_vc0_line_count = $urandom; i_vc0_byte_count = $urandom;
                i_vc0_data_type = $urandom; i_vc0_width_mismatch = $urandom;
                i_vc0_height_mismatch = $urandom; end
      10: begin i_vc1_line_count = $urandom; i_vc1_byte_count = $urandom;
                i_vc1_data_type = $urandom; i_vc1_width_mismatch = $urandom;
                i_vc1_height_mismatch = $urandom; end
      11: begin i_emb_word_count_vc0 = $urandom; i_emb_line_count_vc0 = $urandom;
                i_emb_word_count_vc1 = $urandom; i_emb_line_count_vc1 = $urandom; end
      12: for (int m = 0; m < NXW; m++) begin
            i_emb_extract_val_vc0[m] = $urandom;
            i_emb_extract_val_vc1[m] = $urandom;
          end
      13: begin i_meta_overflow = $urandom; i_linebuf_ovf_sticky = $urandom;
                i_sp_fifo_ovf_sticky = $urandom; i_dma_ovf_sticky = $urandom; end
      14: i_error_capture        = $urandom;
      15: i_packet_cnt           = i_packet_cnt + $urandom_range(3, 1);
      16: i_ecc_err_cnt          = i_ecc_err_cnt + (($urandom_range(1, 0) == 1) ? 32'h0001_0001 :
                                                                                   32'h0000_0001);
      17: i_crc_err_cnt          = i_crc_err_cnt + 1;
      18: begin i_frame_drop_cnt = i_frame_drop_cnt + 1; i_lane_abort_cnt = i_lane_abort_cnt + 1; end
      19: begin i_dma_busy = $urandom; i_active_buffer = $urandom; end
      20: begin i_buf0_state = $urandom; i_buf1_state = $urandom; end
      21: begin i_buf0_last_valid_line = $urandom; i_buf1_last_valid_line = $urandom; end
      22: begin i_axi_fault_awid = $urandom; i_axi_fault_resp = $urandom;
                i_axi_fault_awaddr = $urandom; end
      default: begin i_bresp_slverr_count = i_bresp_slverr_count + 1;
                     i_bresp_decerr_count = i_bresp_decerr_count + $urandom_range(1, 0); end
    endcase
  endtask

  task automatic byte_activity();
    case ($urandom_range(3, 0))
      0: begin i_lm_fsm_state = $urandom; i_abort_sync_err = $urandom; end
      1: begin i_abort_fifo_full = $urandom; i_abort_unarmed = $urandom;
               i_deskew_burst_seen = $urandom; i_abort_lane_sync_seen = $urandom; end
      2: i_active_lanes_applied = $urandom;
      default: i_rxskewcalhs = $urandom;
    endcase
  endtask

  task automatic gsp_pop_seen();
    if (gsp_fifo.size() == 0) begin
      if (!gsp_relax) fail("gsp_pop with the Generic Short Packet FIFO empty");
    end else begin
      void'(gsp_fifo.pop_front());
    end
    gsp_pops++;
  endtask

  // A strobe pulse is one transfer: the OR of the oldest pending masks, taken in write order,
  // never part of a mask. Masks written while a transfer is in flight are OR-merged into the
  // next one, so the cut between two transfers is not visible from the ports: after a pulse,
  // the following pending masks that it covers and that were written before it are marked
  // "may have been merged". A marked mask is dropped without error if no later pulse takes it,
  // and skipped when a later pulse does not cover it.
  task automatic pulse_check(input int k, input logic [23:0] p);
    logic [23:0] cov;
    logic [23:0] w;
    int          i;
    int          n;
    string       nm;
    nm  = k ? "release_buf" : "irq_clear_strobe";
    cov = '0;
    i   = 0;
    n   = pend_m[k].size();
    while (i < n && cov != p) begin
      w = pend_m[k][i];
      if ((w & ~p) != 0) begin
        if (pend_o[k][i]) begin
          i++;                       // already delivered by an earlier pulse
          continue;
        end
        break;
      end
      cov = cov | w;
      i++;
    end
    check_count++;
    if (cov != p) begin
      fail($sformatf("%s: pulse 0x%0h is not the OR of the oldest pending masks (%s)", nm, p,
                     pend_str(k)));
    end
    // Remove what was delivered; mark the next covered masks as possibly merged.
    for (int j = 0; j < i; j++) begin
      void'(pend_m[k].pop_front());
      void'(pend_t[k].pop_front());
      void'(pend_o[k].pop_front());
    end
    for (int j = 0; j < pend_m[k].size(); j++) begin
      if ((pend_m[k][j] & ~p) != 0 || pend_t[k][j] > $realtime) break;
      pend_o[k][j] = 1'b1;
    end
    if (k == 1) begin
      if (p[0]) rel_pulses[0]++;
      if (p[1]) rel_pulses[1]++;
    end
  endtask

  task automatic pend_clear();
    for (int k = 0; k < 2; k++) begin
      pend_m[k].delete();
      pend_t[k].delete();
      pend_o[k].delete();
    end
    irq_cur_mask = '0;
  endtask

  function automatic string pend_str(input int k);
    string s;
    s = "pending:";
    for (int j = 0; j < pend_m[k].size() && j < 8; j++)
      s = {s, $sformatf(" 0x%0h%s@%0t", pend_m[k][j], pend_o[k][j] ? "?" : "", pend_t[k][j])};
    return s;
  endfunction

  function automatic int pend_firm(input int k);
    int c;
    c = 0;
    for (int j = 0; j < pend_m[k].size(); j++) if (!pend_o[k][j]) c++;
    return c;
  endfunction

  task automatic pend_push(input int k, input logic [23:0] m);
    pend_m[k].push_back(m);
    pend_t[k].push_back($realtime);
    pend_o[k].push_back(1'b0);
  endtask

  // sys_clk monitor and source models. Outputs are sampled at the rising edge; inputs change
  // 0.3 ns after it.
  task automatic sys_drive(input logic [23:0] strb);
    logic [23:0] ev;
    if (!i_sys_resetn) begin
      irq_q = '0;
      gsp_fifo.delete();
    end else begin
      ev = '0;
      if (irq_rate > 0) begin
        for (int b = 0; b < 24; b++) if ($urandom_range(999, 0) < irq_rate) ev[b] = 1'b1;
      end
      irq_q = (irq_q & ~strb) | ev | irq_events_once | irq_persist;
    end
    irq_events_once = '0;
    i_irq_status    = irq_q;
    i_gsp_data      = (gsp_fifo.size() > 0) ? gsp_fifo[0] : 24'd0;
    if (ts_run) i_timestamp = i_timestamp + ts_step;
    if (sys_act > 0 && $urandom_range(99, 0) < sys_act) begin
      repeat ($urandom_range(3, 1)) sys_activity();
    end
    rec_all();
  endtask

  bit      phy_now     = 1'b1;
  realtime phy_chg_t   = 0.0;

  always @(posedge i_sys_clk) begin : p_sys
    logic [23:0] s_irq;
    logic [1:0]  s_rel;
    logic        s_pop;
    s_irq = o_irq_clear_strobe;
    s_rel = o_release_buf;
    s_pop = o_gsp_pop;
    if (started) begin
      if (i_sys_resetn !== 1'b1) begin
        if (warm && (s_irq !== 24'd0 || s_rel !== 2'd0 || s_pop !== 1'b0))
          fail("strobe or gsp_pop while sys_resetn is low");
      end else begin
        if (init_done && $isunknown({s_irq, s_rel, s_pop, o_soft_reset_sys, o_core_enable,
                                     o_vc_exposure_map, o_metadata_dt, o_metadata_capture_en,
                                     o_fnum_check_en, o_emb_extract_en, o_expected_width,
                                     o_expected_height, o_metadata_record_id, o_emb_extract_cfg,
                                     o_axis_enable, o_stream_format, o_stream_follow_dma_drop,
                                     o_tpg_enable, o_tpg_pattern, o_tpg_dual_vc_en,
                                     o_tpg_pixel_format, o_tpg_cfa_order, o_tpg_width,
                                     o_tpg_height, o_tpg_hblank, o_tpg_vblank, o_dma_enable,
                                     o_dma_start, o_dma_abort, o_err_frame_policy, o_dma_format,
                                     o_dma_buffer0_addr, o_dma_buffer1_addr, o_dma_buffer_size,
                                     o_meta_area_size, o_bytes_per_line, o_line_padding,
                                     o_awqos, o_max_burst_len, o_awprot, o_global_irq_enable,
                                     o_irq_enable_mask, o_phy_rst_in_progress}))
          fail("X or Z on a sys_clk output");
        if (s_irq != 24'd0) pulse_check(0, s_irq);
        if (s_rel != 2'd0)  pulse_check(1, {22'b0, s_rel});
        if (s_pop === 1'b1) gsp_pop_seen();
        for (int k = 0; k < 2; k++) begin
          while (pend_m[k].size() > 0 &&
                 $realtime - pend_t[k][0] > LAT_SYS + 2.0 * T_SYS + 4.0 * T_LITE) begin
            if (!pend_o[k][0])
              fail($sformatf("%s: mask 0x%0h not delivered in time",
                             k ? "release_buf" : "irq_clear_strobe", pend_m[k][0]));
            void'(pend_m[k].pop_front());
            void'(pend_t[k].pop_front());
            void'(pend_o[k].pop_front());
          end
        end
        if (irq_cur_mask != 0) irq_zero_bits = irq_zero_bits | (irq_cur_mask & ~i_irq_status);
        // phy_rst_in_progress follows phy_core_rst once both have been stable.
        if ($realtime - sys_rel_t > (S + 2) * T_SYS + T_LITE &&
            $realtime - phy_chg_t > (S + 2) * T_SYS + 2.0 * T_LITE) begin
          check_count++;
          if (o_phy_rst_in_progress !== phy_now)
            fail($sformatf("phy_rst_in_progress = %0b, phy_core_rst = %0b",
                           o_phy_rst_in_progress, phy_now));
        end
        if (init_done) begin
          for (int g = 1; g < NG; g++) begin
            if (grp_on(g) && grp_out(g) !== g_last[g]) grp_check(g);
          end
        end
      end
      for (int g = 1; g < NG; g++) g_last[g] = grp_out(g);
    end
    #0.3;
    if (started) sys_drive(s_irq);
  end

  // A destination group may only show a value that was written to its register.
  task automatic grp_check(input int g);
    logic [31:0] v;
    bit          found;
    v     = grp_out(g);
    found = 1'b0;
    for (int i = 0; i < ghist[g].size(); i++) if (ghist[g][i] === v) found = 1'b1;
    check_count++;
    if (!found) fail($sformatf("output of word 0x%0h shows 0x%0h, never written (mixed write?)",
                               grp_w(g) * 4, v));
  endtask

  // rxbyteclkhs monitor and source models.
  always @(posedge i_rxbyteclkhs) begin : p_byte
    // A pulse counts at the edge that samples it, with rxbyteclkhs_resetn as the flop sees it
    // there: a pulse left on a stopped clock or cut by a reset is not counted yet.
    if (started) begin
      for (int x = 0; x < 4; x++) begin
        for (int k = 0; k < 2; k++) begin
          if (i_rxbyteclkhs_resetn !== 1'b1) sot_par[k][x] = 1'b0;
          else if (k ? i_errsotsynchs[x] : i_errsoths[x]) begin
            sot_tprev[k][x] = sot_t[k][x];
            sot_t[k][x]     = $realtime;
            sot_par[k][x]   = ~sot_par[k][x];
          end
        end
      end
    end
    if (started) begin
      if (i_rxbyteclkhs_resetn === 1'b1) begin
        if (init_done && $isunknown({o_soft_reset_byte, o_active_lanes}))
          fail("X or Z on an rxbyteclkhs output");
        if (init_done && grp_out(0) !== g_last[0]) grp_check(0);
      end
      g_last[0] = grp_out(0);
    end
    #0.3;
    if (started) begin
      // One-cycle pulses only: a lane that pulsed in the previous cycle waits one cycle, so a
      // request made then is driven a cycle later.
      sot_last[0]    = i_errsoths;
      sot_last[1]    = i_errsotsynchs;
      i_errsoths     = 4'b0;
      i_errsotsynchs = 4'b0;
      for (int x = 0; x < 4; x++) begin
        for (int k = 0; k < 2; k++) begin
          if (!sot_last[k][x] &&
              (sot_req[k][x] || (sot_rand > 0 && $urandom_range(999, 0) < sot_rand))) begin
            if (k == 0) i_errsoths[x]     = 1'b1;
            else        i_errsotsynchs[x] = 1'b1;
            sot_req[k][x] = 1'b0;
          end
        end
      end
      if (byte_act > 0 && $urandom_range(99, 0) < byte_act) byte_activity();
      rec_all();
    end
  end

  // Asynchronous D-PHY levels.
  initial begin
    forever begin
      #($urandom_range(400, 20) * 1.0);
      if (started && async_act) begin
        case ($urandom_range(5, 0))
          0: i_cl_stopstate     = ~i_cl_stopstate;
          1: i_cl_ulpsactivenot = ~i_cl_ulpsactivenot;
          2: i_cl_rxclkactivehs = ~i_cl_rxclkactivehs;
          3: i_cl_ulpsclknot    = ~i_cl_ulpsclknot;
          4: i_init_done        = ~i_init_done;
          default: i_stopstate  = $urandom;
        endcase
        rec_all();
      end
    end
  end

  // --------------------------------------------------------------------------------------------
  // lite_aclk monitor: AXI4-Lite protocol and latency, register model, PHY Reset Sequencer
  // --------------------------------------------------------------------------------------------
  localparam int K_NORMAL = 0;
  localparam int K_IRQ    = 1;

  event        ev_wr_done;
  event        ev_ar_done;
  bit          aw_acc = 1'b0, w_acc = 1'b0, wr_pend = 1'b0, rd_pend = 1'b0;
  bit          bv_seen, rv_seen, rd_after_wr, rd_is_gsp;
  bit          prev_bv = 1'b0, prev_bready = 1'b0, prev_rv = 1'b0, prev_rready = 1'b0;
  logic [1:0]  prev_bresp, prev_rresp;
  logic [31:0] prev_rdata;
  logic [11:0] aw_addr_q, ar_addr_q;
  logic [31:0] w_data_q;
  logic [3:0]  w_strb_q;
  logic [1:0]  exp_bresp;
  int          wr_cyc, ar_cyc, b_cyc = -100;
  int          wr_kind;
  bit          irq_persist_hit;
  realtime     rv_t;
  int          n_writes = 0, n_reads = 0;

  // Sequencer expectations (sample index), -1 when none.
  int          seq_exp_rise = -1, seq_exp_fall = -1, fr_exp_rise = -1, fr_exp_fall = -1;
  bit          prev_phy = 1'b1, prev_fr = 1'b0, resetn_seen_low = 1'b1;
  int          phy_rise_cyc = -1, phy_fall_cyc = -1, fr_rise_cyc = -1, fr_fall_cyc = -1;
  int          resetn_rel_cyc = -1;
  int          trig_cyc [$];

  task automatic seq_trigger(input bit full);
    trig_cyc.push_back(wr_cyc);
    if (o_phy_core_rst !== 1'b1) seq_exp_rise = wr_cyc + 2;
    seq_exp_fall = wr_cyc + 2 + N_RST;
    if (full) begin
      if (o_full_reset_req !== 1'b1) fr_exp_rise = wr_cyc + 2;
      fr_exp_fall = wr_cyc + 2 + N_RST;
    end else if (o_full_reset_req === 1'b1) begin
      fr_exp_fall = wr_cyc + 2 + N_RST;
    end
  endtask

  task automatic seq_monitor();
    bit phy;
    bit fr;
    phy = (o_phy_core_rst === 1'b1);
    fr  = (o_full_reset_req === 1'b1);
    if (phy != phy_now) begin
      phy_now   = phy;
      phy_chg_t = $realtime;
    end
    if (!i_resetn) begin
      if (warm) begin
        check_count++;
        if (o_phy_core_rst !== 1'b1)   fail("phy_core_rst is not 1 while resetn is low");
        if (o_full_reset_req !== 1'b0) fail("full_reset_req is not 0 while resetn is low");
      end
      seq_exp_rise    = -1;
      seq_exp_fall    = -1;
      fr_exp_rise     = -1;
      fr_exp_fall     = -1;
      resetn_seen_low = 1'b1;
    end else begin
      if (resetn_seen_low) begin
        resetn_seen_low = 1'b0;
        resetn_rel_cyc  = lcyc;
        seq_exp_fall    = lcyc + S + N_RST + 1;
        seq_exp_rise    = -1;
      end
      if ($isunknown({o_phy_core_rst, o_full_reset_req})) fail("X or Z on phy_core_rst or full_reset_req");
      if (phy && !prev_phy) begin
        phy_rise_cyc = lcyc;
        check_count++;
        if (seq_exp_rise < 0 || lcyc != seq_exp_rise)
          fail($sformatf("phy_core_rst rose at sample %0d, expected %0d", lcyc, seq_exp_rise));
        seq_exp_rise = -1;
      end
      if (!phy && prev_phy) begin
        phy_fall_cyc = lcyc;
        check_count++;
        if (seq_exp_fall < 0 || lcyc != seq_exp_fall)
          fail($sformatf("phy_core_rst fell at sample %0d, expected %0d", lcyc, seq_exp_fall));
        seq_exp_fall = -1;
      end
      if (phy && seq_exp_fall >= 0 && lcyc == seq_exp_fall + 1)
        fail("phy_core_rst still high after PHY_RST_CYCLES");
      if (!phy && seq_exp_rise >= 0 && lcyc == seq_exp_rise + 1) fail("phy_core_rst did not rise");
      if (fr && !prev_fr) begin
        fr_rise_cyc = lcyc;
        check_count++;
        if (fr_exp_rise < 0 || lcyc != fr_exp_rise)
          fail($sformatf("full_reset_req rose at sample %0d, expected %0d", lcyc, fr_exp_rise));
        fr_exp_rise = -1;
        pend_clear();
        gsp_relax = 1'b1;
      end
      if (!fr && prev_fr) begin
        fr_fall_cyc = lcyc;
        check_count++;
        if (fr_exp_fall < 0 || lcyc != fr_exp_fall)
          fail($sformatf("full_reset_req fell at sample %0d, expected %0d", lcyc, fr_exp_fall));
        fr_exp_fall = -1;
      end
      if (fr && !phy) fail("full_reset_req high while phy_core_rst is low");
    end
    prev_phy = phy;
    prev_fr  = fr;
  endtask

  // D-PHY enable outputs: registered once from PHY_CONFIG.
  task automatic phy_out_check();
    logic [8:0] out;
    out = {o_forcerxmode, o_dl_enable, o_cl_enable};
    if (!init_done) return;
    if (!i_resetn) begin
      expect_eq("D-PHY enables while resetn is low", out, 9'h0);
    end else if (o_full_reset_req !== 1'b1) begin
      check_count++;
      if (lcyc - phy_chg_cyc <= 4) begin
        if (out !== phy_f(rg[W_PHY]) && out !== phy_f(phy_prev_val))
          fail($sformatf("D-PHY enables 0x%0h, PHY_CONFIG gives 0x%0h", out, phy_f(rg[W_PHY])));
      end else if (out !== phy_f(rg[W_PHY])) begin
        fail($sformatf("D-PHY enables 0x%0h, PHY_CONFIG gives 0x%0h", out, phy_f(rg[W_PHY])));
      end
    end
  endtask

  // Effect of a write accepted on both channels (performed in the next cycle).
  task automatic apply_write();
    int          w;
    logic [31:0] bm;
    logic [31:0] nv;
    logic [31:0] old;
    logic [23:0] m;
    w         = int'(aw_addr_q[7:2]);
    exp_bresp = (aw_addr_q >= 12'h100) ? 2'b11 : 2'b00;
    wr_kind   = K_NORMAL;
    n_writes++;
    if (aw_addr_q >= 12'h100) return;
    if (rf_in_reset) return;
    bm = {{8{w_strb_q[3]}}, {8{w_strb_q[2]}}, {8{w_strb_q[1]}}, {8{w_strb_q[0]}}};
    if (!reserved_w(w) && rw_mask(w) != 0) begin
      old   = rg[w];
      nv    = ((old & ~bm) | (w_data_q & bm)) & rw_mask(w);
      rg[w] = nv;
      ghist_push(w);
      if (w == W_PHY && nv != old) begin
        phy_prev_val = old;
        phy_chg_cyc  = lcyc;
      end
      if (w == W_CORE && old[1] && !nv[1]) t_sr_fall = $realtime;
    end
    if (w == W_CORE && w_strb_q[0] && w_data_q[2]) begin
      seq_trigger(1'b1);
      model_reset();
    end
    if (w == W_PHY && w_strb_q[0] && w_data_q[1]) seq_trigger(1'b0);
    if (w == W_IRQ) begin
      m = {{8{w_strb_q[2]}}, {8{w_strb_q[1]}}, {8{w_strb_q[0]}}} & w_data_q[23:0];
      if (m != 0) begin
        wr_kind         = K_IRQ;
        irq_persist_hit = ((m & irq_persist) != 0);
        irq_zero_bits   = '0;
        irq_cur_mask    = m;
        pend_push(0, m);
      end
    end
    if (w == W_REL) begin
      m = {22'b0, w_data_q[1:0] & {2{w_strb_q[0]}}};
      if (m != 0) begin
        pend_push(1, m);
      end
    end
  endtask

  // SoT flag of lane x (k = 0 sot_error, 1 sot_sync_error): 0, 1, or 2 when either is allowed.
  function automatic int sot_exp(input int k, input int x, input realtime t);
    realtime t_clr;
    realtime tp;
    real     m;
    if (x >= NL) return 0;
    if (rg[W_CORE][1] || rf_in_reset) return 0;
    m     = LAT_BYTE + 4.0 * T_LITE + 4.0 * T_BYTE;
    t_clr = (t_sr_fall > rf_last_reset_t) ? t_sr_fall : rf_last_reset_t;
    tp    = sot_t[k][x];
    if (tp > t_clr + m && t - tp > m) return 1;
    if (tp < t_clr - m) return 0;
    return 2;
  endfunction

  // Read data against the register model and the status history.
  task automatic check_read(input logic [11:0] addr, input logic [31:0] d, input logic [1:0] r);
    int          w;
    int          e;
    bit          found;
    logic [63:0] v;
    w = int'(addr[7:2]);
    n_reads++;
    if (addr >= 12'h100) begin
      expect_eq($sformatf("rresp at 0x%0h", addr), r, 2'b11);
      expect_eq($sformatf("rdata at 0x%0h (DECERR)", addr), d, 32'h0);
      return;
    end
    expect_eq($sformatf("rresp at 0x%0h", addr), r, 2'b00);
    if (reserved_w(w)) begin
      expect_eq($sformatf("reserved offset 0x%0h", w * 4), d, 32'h0);
      return;
    end
    case (w)
      W_IPV:  expect_eq("IP_VERSION", d, IP_VERSION);
      W_PROT: begin
        expect_eq("PROTOCOL_CONFIG RW bits", d & 32'h1003, rg[W_PROT]);
        expect_eq("PROTOCOL_CONFIG.max_lanes", d[9:8], LANES_CODE);
        expect_eq("PROTOCOL_CONFIG reserved bits", d & ~32'h1F03, 32'h0);
        check_st(W_PROT, {32'b0, d & 32'h0C00}, rv_t);
      end
      W_CST: begin
        check_st(W_CST, {32'b0, d & ~32'h1}, rv_t);
        check_st(W_RIP, {63'b0, d[0]}, rv_t);
      end
      W_LST: begin
        expect_eq("LANE_STATUS reserved bits", d & ~32'h0F0F_0F0F, 32'h0);
        check_st(W_LST, {32'b0, d & 32'h0909_0909}, rv_t);
        for (int x = 0; x < 4; x++) begin
          for (int k = 0; k < 2; k++) begin
            e = sot_exp(k, x, rv_t);
            if (e != 2) expect_eq($sformatf(
                "LANE_STATUS lane%0d %s (pulse %0t, prev %0t, clr %0t, rf rst %0t, par %0d)",
                x, k ? "sot_sync_error" : "sot_error", sot_t[k][x], sot_tprev[k][x],
                t_sr_fall, rf_last_reset_t, sot_par[k][x]), d[8 * x + 1 + k], e[0]);
          end
        end
      end
      W_GSP: begin
        expect_eq("GENERIC_SHORT_PACKET bits 31:24", d[31:24], 8'h0);
        if (d != 0) begin
          check_count++;
          if (gsp_expect.size() == 0 || gsp_expect[0] != d[23:0]) begin
            if (!gsp_relax)
              fail($sformatf("GENERIC_SHORT_PACKET read 0x%0h, next entry 0x%0h", d,
                             (gsp_expect.size() > 0) ? gsp_expect[0] : 24'h0));
          end else begin
            void'(gsp_expect.pop_front());
          end
          gsp_nz_reads++;
        end
      end
      W_TSLO: begin
        found = 1'b0;
        for (int i = 0; i < hist[W_TSLO].size(); i++) begin
          v = hist[W_TSLO][i].v;
          if (hist[W_TSLO][i].t <= rv_t && v[31:0] == d &&
              (i + 1 >= hist[W_TSLO].size() ||
               hist[W_TSLO][i + 1].t > rv_t - LAT_SYS - 4.0 * T_LITE - 2.0 * T_SYS)) begin
            found     = 1'b1;
            ts_shadow = v[63:32];
          end
        end
        if (!found && d == 0 && rv_t - rf_last_reset_t < LAT_SYS + 4.0 * T_LITE + 2.0 * T_SYS) begin
          found     = 1'b1;
          ts_shadow = 32'h0;
        end
        check_count++;
        if (!found) fail($sformatf("TIMESTAMP_LO read 0x%0h, not a recent timestamp", d));
      end
      W_TSHI: expect_eq("TIMESTAMP_HI (shadow of the last TIMESTAMP_LO read)", d, ts_shadow);
      W_REL:  expect_eq("BUFFER_RELEASE", d, 32'h0);
      W_ATTR: begin
        expect_eq("DMA_ATTR RW bits", d & 32'h7FFF, rg[W_ATTR]);
        expect_eq("DMA_ATTR.max_outstanding", d[22:16], PARA_MAX_OUTSTANDING[6:0]);
        expect_eq("DMA_ATTR reserved bits", d & ~32'h007F_7FFF, 32'h0);
      end
      default: begin
        if (is_status(w)) check_st(w, {32'b0, d}, rv_t);
        else expect_eq($sformatf("register 0x%0h", w * 4), d, rg[w]);
      end
    endcase
  endtask

  task automatic lite_monitor();
    bit aw_hs, w_hs, ar_hs, b_hs, r_hs;
    // Register File reset as seen in lite_aclk, and reset_in_progress.
    if (!i_resetn || o_full_reset_req === 1'b1) rf_rel_cnt = 0;
    else if (rf_rel_cnt < 1000) rf_rel_cnt++;
    rf_in_reset = (rf_rel_cnt < S + 2);
    if (rf_in_reset) rf_last_reset_t = $realtime;
    hist_push(W_RIP, {63'b0, (o_phy_core_rst === 1'b1) || rg[W_CORE][1] || rf_in_reset});
    seq_monitor();
    phy_out_check();

    if (!i_lite_aresetn) begin
      if (warm) begin
        check_count++;
        if (o_s_axi_bvalid !== 1'b0 || o_s_axi_rvalid !== 1'b0)
          fail("BVALID or RVALID high while lite_aresetn is low");
        if (o_s_axi_awready !== 1'b0 || o_s_axi_wready !== 1'b0 || o_s_axi_arready !== 1'b0)
          fail("READY high while lite_aresetn is low");
      end
      if (rd_pend && rd_is_gsp) gsp_relax = 1'b1;
      aw_acc  = 1'b0;
      w_acc   = 1'b0;
      wr_pend = 1'b0;
      rd_pend = 1'b0;
      prev_bv = 1'b0;
      prev_rv = 1'b0;
      return;
    end
    if (init_done && $isunknown({o_s_axi_awready, o_s_axi_wready, o_s_axi_arready,
                                 o_s_axi_bvalid, o_s_axi_rvalid}))
      fail("X or Z on an AXI4-Lite READY or VALID");

    aw_hs = (i_s_axi_awvalid === 1'b1) && (o_s_axi_awready === 1'b1);
    w_hs  = (i_s_axi_wvalid === 1'b1)  && (o_s_axi_wready === 1'b1);
    ar_hs = (i_s_axi_arvalid === 1'b1) && (o_s_axi_arready === 1'b1);
    b_hs  = (o_s_axi_bvalid === 1'b1)  && (i_s_axi_bready === 1'b1);
    r_hs  = (o_s_axi_rvalid === 1'b1)  && (i_s_axi_rready === 1'b1);

    // Write response channel.
    if (prev_bv && !prev_bready) begin
      if (o_s_axi_bvalid !== 1'b1) fail("BVALID dropped before BREADY");
      else if (o_s_axi_bresp !== prev_bresp) fail("BRESP changed while BVALID waits");
    end
    if (o_s_axi_bvalid === 1'b1) begin
      if (!wr_pend) begin
        fail("BVALID without an accepted write");
      end else if (!bv_seen) begin
        bv_seen = 1'b1;
        check_count++;
        expect_eq($sformatf("bresp at 0x%0h", aw_addr_q), o_s_axi_bresp, exp_bresp);
        if (wr_kind == K_NORMAL) begin
          if (lcyc != wr_cyc + 2)
            fail($sformatf("write latency %0d cycles, expected 2", lcyc - wr_cyc));
        end else if (irq_persist_hit) begin
          if (lcyc != wr_cyc + N_CLR + 3)
            fail($sformatf("IRQ_STATUS write held %0d cycles, expected %0d (timeout)",
                           lcyc - wr_cyc, N_CLR + 3));
        end else begin
          if (lcyc < wr_cyc + 3 || lcyc > wr_cyc + N_CLR + 3)
            fail($sformatf("IRQ_STATUS write latency %0d cycles", lcyc - wr_cyc));
          else if (lcyc < wr_cyc + N_CLR + 3 && irq_zero_bits != irq_cur_mask)
            fail("IRQ_STATUS write released before the written bits cleared");
        end
      end
    end
    if (b_hs && wr_pend) begin
      wr_pend = 1'b0;
      b_cyc   = lcyc;
    end

    // Read data channel.
    if (prev_rv && !prev_rready) begin
      if (o_s_axi_rvalid !== 1'b1) fail("RVALID dropped before RREADY");
      else if (o_s_axi_rdata !== prev_rdata || o_s_axi_rresp !== prev_rresp)
        fail("RDATA or RRESP changed while RVALID waits");
    end
    if (o_s_axi_rvalid === 1'b1) begin
      if (!rd_pend) begin
        fail("RVALID without an accepted read");
      end else if (!rv_seen) begin
        rv_seen = 1'b1;
        rv_t    = $realtime;
        check_count++;
        if (rd_after_wr) begin
          if (wr_pend) fail("read served before the write accepted in the same cycle");
          else if (rd_is_gsp ? (lcyc < b_cyc + 3) : (lcyc != b_cyc + 3))
            fail($sformatf("read after write: RVALID %0d cycles after BREADY", lcyc - b_cyc));
        end else if (rd_is_gsp ? (lcyc < ar_cyc + 2) : (lcyc != ar_cyc + 2)) begin
          fail($sformatf("read latency %0d cycles, expected 2", lcyc - ar_cyc));
        end
      end
    end
    if (r_hs && rd_pend) begin
      rd_pend = 1'b0;
      check_read(ar_addr_q, o_s_axi_rdata, o_s_axi_rresp);
    end

    // Address and data channels.
    if (aw_hs) begin
      if (aw_acc || wr_pend) fail("AW accepted while a write is in progress");
      aw_acc    = 1'b1;
      aw_addr_q = i_s_axi_awaddr;
    end
    if (w_hs) begin
      if (w_acc || wr_pend) fail("W accepted while a write is in progress");
      w_acc    = 1'b1;
      w_data_q = i_s_axi_wdata;
      w_strb_q = i_s_axi_wstrb;
    end
    if (aw_acc && w_acc) begin
      aw_acc  = 1'b0;
      w_acc   = 1'b0;
      wr_pend = 1'b1;
      bv_seen = 1'b0;
      wr_cyc  = lcyc;
      apply_write();
      -> ev_wr_done;
    end
    if (ar_hs) begin
      if (rd_pend) fail("AR accepted while a read is in progress");
      rd_pend     = 1'b1;
      rv_seen     = 1'b0;
      ar_cyc      = lcyc;
      ar_addr_q   = i_s_axi_araddr;
      rd_after_wr = wr_pend;
      rd_is_gsp   = (i_s_axi_araddr < 12'h100) && (i_s_axi_araddr[7:2] == W_GSP);
      -> ev_ar_done;
    end

    prev_bv     = (o_s_axi_bvalid === 1'b1);
    prev_bready = (i_s_axi_bready === 1'b1);
    prev_bresp  = o_s_axi_bresp;
    prev_rv     = (o_s_axi_rvalid === 1'b1);
    prev_rready = (i_s_axi_rready === 1'b1);
    prev_rdata  = o_s_axi_rdata;
    prev_rresp  = o_s_axi_rresp;
  endtask

  always @(posedge i_lite_aclk) begin
    lcyc = lcyc + 1;
    if (started) lite_monitor();
  end

  // resetn resets the Register File at once; masks and pops in flight are lost.
  always @(negedge i_resetn) begin
    if (started) begin
      model_reset();
      pend_clear();
      gsp_relax = 1'b1;
    end
  end

  // --------------------------------------------------------------------------------------------
  // AXI4-Lite manager
  // --------------------------------------------------------------------------------------------
  bit  xact_abort   = 1'b0;
  bit  rand_ready   = 1'b1;
  int  bready_delay = 0;
  int  rready_delay = 0;
  bit  rand_lowbits = 1'b1;

  function automatic logic [11:0] mk_addr(input logic [11:0] a);
    logic [1:0] lo;
    lo = rand_lowbits ? 2'($urandom) : 2'b00;
    return {a[11:2], lo};
  endfunction

  task automatic send_aw(input logic [11:0] a);
    int n;
    n = 0;
    i_s_axi_awaddr  = mk_addr(a);
    i_s_axi_awprot  = $urandom;
    i_s_axi_awvalid = 1'b1;
    forever begin
      @(posedge i_lite_aclk);
      if (xact_abort || o_s_axi_awready === 1'b1) break;
      if (++n > 2000000) begin fail("AW never accepted"); break; end
    end
    #1.0;
    i_s_axi_awvalid = 1'b0;
    i_s_axi_awaddr  = $urandom;
    i_s_axi_awprot  = $urandom;
  endtask

  task automatic send_w(input logic [31:0] d, input logic [3:0] s);
    int n;
    n = 0;
    i_s_axi_wdata  = d;
    i_s_axi_wstrb  = s;
    i_s_axi_wvalid = 1'b1;
    forever begin
      @(posedge i_lite_aclk);
      if (xact_abort || o_s_axi_wready === 1'b1) break;
      if (++n > 2000000) begin fail("W never accepted"); break; end
    end
    #1.0;
    i_s_axi_wvalid = 1'b0;
    i_s_axi_wdata  = $urandom;
    i_s_axi_wstrb  = $urandom;
  endtask

  task automatic send_ar(input logic [11:0] a);
    int n;
    n = 0;
    i_s_axi_araddr  = mk_addr(a);
    i_s_axi_arprot  = $urandom;
    i_s_axi_arvalid = 1'b1;
    forever begin
      @(posedge i_lite_aclk);
      if (xact_abort || o_s_axi_arready === 1'b1) break;
      if (++n > 2000000) begin fail("AR never accepted"); break; end
    end
    #1.0;
    i_s_axi_arvalid = 1'b0;
    i_s_axi_araddr  = $urandom;
    i_s_axi_arprot  = $urandom;
  endtask

  task automatic wait_b();
    int n;
    int wcnt;
    n    = 0;
    wcnt = 0;
    if (rand_ready) bready_delay = ($urandom_range(3, 0) == 0) ? $urandom_range(6, 1) : 0;
    i_s_axi_bready = (bready_delay == 0);
    forever begin
      @(posedge i_lite_aclk);
      if (xact_abort) break;
      if (o_s_axi_bvalid === 1'b1 && i_s_axi_bready) break;
      if (o_s_axi_bvalid === 1'b1) n++;
      if (++wcnt > 3000000) begin fail("no write response"); break; end
      #1.0;
      if (n >= bready_delay) i_s_axi_bready = 1'b1;
    end
    #1.0;
    i_s_axi_bready = 1'b0;
  endtask

  task automatic wait_r(output logic [31:0] d);
    int n;
    int wcnt;
    n    = 0;
    wcnt = 0;
    d    = '0;
    if (rand_ready) rready_delay = ($urandom_range(3, 0) == 0) ? $urandom_range(6, 1) : 0;
    i_s_axi_rready = (rready_delay == 0);
    forever begin
      @(posedge i_lite_aclk);
      if (xact_abort) break;
      if (o_s_axi_rvalid === 1'b1 && i_s_axi_rready) begin d = o_s_axi_rdata; break; end
      if (o_s_axi_rvalid === 1'b1) n++;
      if (++wcnt > 3000000) begin fail("no read data"); break; end
      #1.0;
      if (n >= rready_delay) i_s_axi_rready = 1'b1;
    end
    #1.0;
    i_s_axi_rready = 1'b0;
  endtask

  // order: 0 AW and W together, 1 AW first, 2 W first, -1 random; gap in cycles, -1 random.
  task automatic axi_write(input logic [11:0] a, input logic [31:0] d, input logic [3:0] s = 4'hF,
                           input int order = -1, input int gap = -1);
    int o;
    int g;
    o = (order < 0) ? $urandom_range(2, 0) : order;
    g = (gap < 0) ? $urandom_range(3, 0) : gap;
    fork
      begin
        if (o == 2) repeat (g) lclk();
        send_aw(a);
      end
      begin
        if (o == 1) repeat (g) lclk();
        send_w(d, s);
      end
    join
    if (!xact_abort) wait_b();
  endtask

  task automatic axi_read(input logic [11:0] a, output logic [31:0] d);
    d = '0;
    send_ar(a);
    if (!xact_abort) wait_r(d);
  endtask

  task automatic wr(input logic [11:0] a, input logic [31:0] d, input logic [3:0] s = 4'hF);
    axi_write(a, d, s);
  endtask

  task automatic rd(input logic [11:0] a);
    logic [31:0] d;
    axi_read(a, d);
  endtask

  task automatic rdv(input logic [11:0] a, output logic [31:0] d);
    axi_read(a, d);
  endtask

  // lite_aresetn asserted for n cycles; the manager drops every VALID. chk: hold the reset until
  // the crossings have delivered and check that every output kept its value.
  task automatic lite_reset(input int n, input bit chk = 1'b0);
    xact_abort      = 1'b1;
    i_lite_aresetn  = 1'b0;
    i_s_axi_awvalid = 1'b0;
    i_s_axi_wvalid  = 1'b0;
    i_s_axi_arvalid = 1'b0;
    i_s_axi_bready  = 1'b0;
    i_s_axi_rready  = 1'b0;
    lclk(n);
    if (chk) begin
      settle();
      check_outputs("lite_aresetn low");
    end
    i_lite_aresetn = 1'b1;
    lclk(2);
    xact_abort = 1'b0;
  endtask

  // --------------------------------------------------------------------------------------------
  // Test helpers
  // --------------------------------------------------------------------------------------------
  // Every configuration output against the register model. in_reset: destination domains are
  // in reset, so the rxbyteclkhs outputs are checked too.
  task automatic check_outputs(input string tag, input bit in_reset = 1'b0);
    bit          byte_ok;
    logic [31:0] e;
    byte_ok = in_reset || (byte_run && i_rxbyteclkhs_resetn === 1'b1 &&
                           $realtime - byte_start_t > LAT_BYTE + 4.0 * T_BYTE);
    if (byte_ok) begin
      expect_eq({tag, ": soft_reset_byte"}, o_soft_reset_byte, rg[W_CORE][1]);
      expect_eq({tag, ": active_lanes"}, o_active_lanes, rg[W_PROT][1:0]);
    end
    expect_eq({tag, ": soft_reset_sys"}, o_soft_reset_sys, rg[W_CORE][1]);
    expect_eq({tag, ": core_enable"}, o_core_enable, rg[W_CORE][0]);
    expect_eq({tag, ": vc_exposure_map"}, o_vc_exposure_map, rg[W_PROT][12]);
    expect_eq({tag, ": metadata_dt"}, o_metadata_dt, rg[W_MD][5:0]);
    expect_eq({tag, ": metadata_capture_en"}, o_metadata_capture_en, rg[W_MD][8]);
    expect_eq({tag, ": fnum_check_en"}, o_fnum_check_en, rg[W_MD][9]);
    expect_eq({tag, ": emb_extract_en"}, o_emb_extract_en, rg[W_MD][10]);
    expect_eq({tag, ": expected_width"}, o_expected_width, rg[W_ER][15:0]);
    expect_eq({tag, ": expected_height"}, o_expected_height, rg[W_ER][31:16]);
    expect_eq({tag, ": metadata_record_id"}, o_metadata_record_id, rg[W_RID]);
    for (int m = 0; m < NXW; m++) begin
      e = (m < NX) ? (rg[W_CFG0 + m] & 32'hFFFF_FF07) : 32'h0;
      expect_eq($sformatf("%s: emb_extract_cfg[%0d]", tag, m), o_emb_extract_cfg[m], e);
    end
    expect_eq({tag, ": axis_enable"}, o_axis_enable, rg[W_SC][0]);
    expect_eq({tag, ": stream_format"}, o_stream_format, rg[W_SC][1]);
    expect_eq({tag, ": stream_follow_dma_drop"}, o_stream_follow_dma_drop, rg[W_SC][2]);
    expect_eq({tag, ": tpg_enable"}, o_tpg_enable, rg[W_TPGC][0]);
    expect_eq({tag, ": tpg_pattern"}, o_tpg_pattern, rg[W_TPGC][1]);
    expect_eq({tag, ": tpg_dual_vc_en"}, o_tpg_dual_vc_en, rg[W_TPGC][2]);
    expect_eq({tag, ": tpg_pixel_format"}, o_tpg_pixel_format, rg[W_TPGC][3]);
    expect_eq({tag, ": tpg_cfa_order"}, o_tpg_cfa_order, rg[W_TPGC][5:4]);
    expect_eq({tag, ": tpg_width"}, o_tpg_width, rg[W_TR][15:0]);
    expect_eq({tag, ": tpg_height"}, o_tpg_height, rg[W_TR][31:16]);
    expect_eq({tag, ": tpg_hblank"}, o_tpg_hblank, rg[W_TB][15:0]);
    expect_eq({tag, ": tpg_vblank"}, o_tpg_vblank, rg[W_TB][31:16]);
    expect_eq({tag, ": dma_enable"}, o_dma_enable, rg[W_DMAC][0]);
    expect_eq({tag, ": dma_start"}, o_dma_start, rg[W_DMAC][1]);
    expect_eq({tag, ": dma_abort"}, o_dma_abort, rg[W_DMAC][2]);
    expect_eq({tag, ": dma_format"}, o_dma_format, rg[W_DMAC][3]);
    expect_eq({tag, ": err_frame_policy"}, o_err_frame_policy, rg[W_DMAC][4]);
    expect_eq({tag, ": dma_buffer0_addr"}, o_dma_buffer0_addr, rg[W_B0]);
    expect_eq({tag, ": dma_buffer1_addr"}, o_dma_buffer1_addr, rg[W_B1]);
    expect_eq({tag, ": dma_buffer_size"}, o_dma_buffer_size, rg[W_BSZ]);
    expect_eq({tag, ": meta_area_size"}, o_meta_area_size, rg[W_META]);
    expect_eq({tag, ": bytes_per_line"}, o_bytes_per_line, rg[W_LP][15:0]);
    expect_eq({tag, ": line_padding"}, o_line_padding, rg[W_LP][23:16]);
    expect_eq({tag, ": awqos"}, o_awqos, rg[W_ATTR][3:0]);
    expect_eq({tag, ": max_burst_len"}, o_max_burst_len, rg[W_ATTR][11:4]);
    expect_eq({tag, ": awprot"}, o_awprot, rg[W_ATTR][14:12]);
    expect_eq({tag, ": global_irq_enable"}, o_global_irq_enable, rg[W_GIE][0]);
    expect_eq({tag, ": irq_enable_mask"}, o_irq_enable_mask, rg[W_IE][23:0]);
    if (o_full_reset_req !== 1'b1) begin
      expect_eq({tag, ": cl_enable"}, o_cl_enable, phy_f(rg[W_PHY]) & 9'h1);
      expect_eq({tag, ": dl_enable"}, o_dl_enable, phy_f(rg[W_PHY]) >> 1 & 9'hF);
      expect_eq({tag, ": forcerxmode"}, o_forcerxmode, phy_f(rg[W_PHY]) >> 5);
    end
  endtask

  // Value for a register write that starts no command.
  function automatic logic [31:0] safe_data(input int w, input logic [31:0] d);
    logic [31:0] v;
    v = d;
    if (w == W_CORE) v[2] = 1'b0;
    if (w == W_PHY)  v[1] = 1'b0;
    return v;
  endfunction

  task automatic write_all_random();
    for (int w = 0; w < 64; w++) begin
      if (w != W_IRQ && w != W_REL) wr(12'(w * 4), safe_data(w, $urandom));
    end
  endtask

  task automatic read_all();
    for (int w = 0; w < 64; w++) rd(12'(w * 4));
  endtask

  // Wait until the PHY Reset Sequencer has finished and every domain is out of reset, then
  // until every status crossing has delivered.
  task automatic wait_ready();
    int guard;
    guard = 0;
    while (o_phy_core_rst !== 1'b0 || o_full_reset_req !== 1'b0 || i_sys_resetn !== 1'b1 ||
           (byte_run && i_rxbyteclkhs_resetn !== 1'b1) || rf_in_reset) begin
      lclk();
      if (++guard > 400000) begin fail("reset sequence did not end"); break; end
    end
    lclk(S + 6);
    settle();
    settle();
  endtask

  task automatic gsp_resync();
    settle();
    settle();
    gsp_expect.delete();
    foreach (gsp_fifo[i]) gsp_expect.push_back(gsp_fifo[i]);
    gsp_pops     = 0;
    gsp_nz_reads = 0;
    gsp_relax    = 1'b0;
  endtask

  task automatic gsp_check_counts(input string tag);
    settle();
    settle();
    expect_eq({tag, ": gsp_pop count against non-zero GENERIC_SHORT_PACKET reads"}, gsp_pops,
              gsp_nz_reads);
  endtask

  // resetn asserted at a random point of a cycle for n cycles.
  task automatic do_resetn(input int n);
    byte_clock(1'b1);
    lclk();
    #($urandom_range(6, 0) * 1.0);
    i_resetn = 1'b0;
    #0.5;
    expect_eq("phy_core_rst at once when resetn is asserted", o_phy_core_rst, 1'b1);
    lclk(n);
    #1.0;
    i_resetn = 1'b1;
    wait_ready();
    expect_eq("phy_core_rst samples after the release of resetn", phy_fall_cyc - resetn_rel_cyc,
              S + N_RST + 1);
    gsp_resync();
  endtask

  task automatic poll_rip_zero();
    logic [31:0] d;
    int          guard;
    guard = 0;
    d     = 32'h1;
    while (d[0] !== 1'b0) begin
      rdv(12'h020, d);
      if (++guard > 200000) begin fail("reset_in_progress never read 0"); break; end
    end
  endtask

  task automatic do_full_reset();
    int h;
    byte_clock(1'b1);
    wr(12'h004, rg[W_CORE] | 32'h4, 4'h1);
    h = trig_cyc[trig_cyc.size() - 1];
    while (o_full_reset_req !== 1'b0 || o_phy_core_rst !== 1'b0) lclk();
    poll_rip_zero();
    lclk(S + 4);
    wait_ready();
    expect_eq("full_reset_req width", fr_fall_cyc - fr_rise_cyc, N_RST);
    expect_eq("full_reset_req start", fr_rise_cyc - h, 2);
    gsp_resync();
  endtask

  // Single-bit status inputs at random levels.
  task automatic rand_single_bits();
    sclk();
    i_sp_fifo_not_empty   = $urandom;
    i_sp_fifo_full        = $urandom;
    i_null_blank_seen     = $urandom;
    i_dt_conflict         = $urandom;
    i_vc_unsupported_seen = $urandom;
    i_active_sel          = $urandom;
    i_switch_pending      = $urandom;
    i_linebuffer_full     = $urandom;
    i_linebuf_ovf_sticky  = $urandom;
    i_sp_fifo_ovf_sticky  = $urandom;
    i_meta_overflow       = $urandom;
    i_dma_ovf_sticky      = $urandom;
    i_dma_busy            = $urandom;
    i_active_buffer       = $urandom;
    i_vc0_width_mismatch  = $urandom;
    i_vc0_height_mismatch = $urandom;
    i_vc1_width_mismatch  = $urandom;
    i_vc1_height_mismatch = $urandom;
    rec_all();
    i_cl_stopstate     = $urandom;
    i_cl_ulpsactivenot = $urandom;
    i_cl_rxclkactivehs = $urandom;
    i_cl_ulpsclknot    = $urandom;
    i_init_done        = $urandom;
    i_stopstate        = $urandom;
    rec_all();
    if (byte_run) begin
      bclk();
      i_rxskewcalhs = $urandom;
      rec_all();
    end
  endtask

  task automatic read_status_all();
    for (int w = 0; w < 64; w++) if (is_status(w) || w == W_TSHI || w == W_GSP) rd(12'(w * 4));
  endtask

  // Registers that drive configuration outputs.
  function automatic int cfg_reg(input int i);
    case (i)
      0: return W_CORE;  1: return W_PROT;  2: return W_PHY;   3: return W_MD;
      4: return W_SC;    5: return W_ER;    6: return W_GIE;   7: return W_IE;
      8: return W_RID;   9: return W_DMAC;  10: return W_B0;   11: return W_B1;
      12: return W_BSZ;  13: return W_ATTR; 14: return W_LP;   15: return W_META;
      16: return W_TPGC; 17: return W_TR;   18: return W_TB;   19: return W_CFG0;
      default: return W_CFG1;
    endcase
  endfunction

  localparam int N_CFG_REGS = 21;

  // --------------------------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------------------------
  // Reset values of every register and output, also with resetn asserted after writes.
  task automatic test_reset_values();
    where = "reset values";
    read_all();
    check_outputs("after reset");
    expect_eq("phy_rst_in_progress", o_phy_rst_in_progress, 1'b0);
    write_all_random();
    settle();
    check_outputs("written");
    lclk();
    #2.0;
    i_resetn = 1'b0;
    lclk(S + 4);
    where = "resetn low";
    check_outputs("resetn low", 1'b1);
    expect_eq("phy_rst_in_progress while resetn is low", o_phy_rst_in_progress, 1'b1);
    expect_eq("full_reset_req while resetn is low", o_full_reset_req, 1'b0);
    expect_eq("release_buf while resetn is low", o_release_buf, 2'b0);
    expect_eq("irq_clear_strobe while resetn is low", o_irq_clear_strobe, 24'h0);
    expect_eq("gsp_pop while resetn is low", o_gsp_pop, 1'b0);
    read_all();
    lclk(5);
    #1.0;
    i_resetn = 1'b1;
    where = "after second reset";
    wait_ready();
    gsp_resync();
    read_all();
    check_outputs("after second reset");
  endtask

  // Random values to every offset, then every register read back.
  task automatic test_rw_readback();
    where = "rw readback";
    repeat (4) begin
      write_all_random();
      read_all();
      settle();
      check_outputs("rw readback");
    end
  endtask

  // Every byte-strobe pattern on the RW registers, IRQ_STATUS and BUFFER_RELEASE.
  task automatic test_write_strobes();
    int w;
    where = "write strobes";
    for (int i = 0; i < N_CFG_REGS; i++) begin
      w = cfg_reg(i);
      for (int s = 0; s < 16; s++) begin
        wr(12'(w * 4), safe_data(w, $urandom), 4'(s));
        rd(12'(w * 4));
      end
    end
    for (int s = 0; s < 16; s++) begin
      irq_set(24'hFF_FFFF);
      settle();
      wr(12'h044, $urandom, 4'(s));
      settle();
      rd(12'h044);
    end
    for (int s = 0; s < 16; s++) begin
      wr(12'h0B0, 32'h3, 4'(s));
      settle();
    end
    wr(12'h044, 32'h00FF_FFFF, 4'hF);
    settle();
    rd(12'h044);
    check_outputs("write strobes");
  endtask

  // Each RW bit seen at its output, bit by bit, then random values; reset values come back
  // with the domain resets.
  task automatic test_config_outputs();
    int          w;
    logic [31:0] mk;
    where = "config outputs";
    for (int i = 0; i < N_CFG_REGS; i++) begin
      w  = cfg_reg(i);
      mk = rw_mask(w);
      for (int b = 0; b < 32; b++) begin
        if (mk[b]) begin
          wr(12'(w * 4), 32'h1 << b);
          settle();
          check_outputs($sformatf("word 0x%0h bit %0d", w * 4, b));
        end
      end
      wr(12'(w * 4), safe_data(w, $urandom));
      settle();
      check_outputs($sformatf("word 0x%0h random", w * 4));
    end
    lclk();
    #3.0;
    i_resetn = 1'b0;
    lclk(S + 4);
    check_outputs("resetn low", 1'b1);
    lclk(3);
    #1.0;
    i_resetn = 1'b1;
    wait_ready();
    gsp_resync();
    check_outputs("after reset");
  endtask

  // Multi-bit registers rewritten faster than the crossing.
  task automatic test_control_coherency();
    int w;
    int n;
    where = "control coherency";
    repeat (12) begin
      w = cfg_reg($urandom_range(N_CFG_REGS - 1, 0));
      n = $urandom_range(12, 3);
      rand_ready = ($urandom_range(1, 0) == 1);
      if (!rand_ready) bready_delay = 0;
      repeat (n) begin
        wr(12'(w * 4), safe_data(w, $urandom));
        if ($urandom_range(2, 0) == 0) lclk($urandom_range(int'(LAT_SYS / T_LITE), 0));
      end
      if ($urandom_range(1, 0) == 1) begin
        repeat (8) begin
          w = cfg_reg($urandom_range(N_CFG_REGS - 1, 0));
          wr(12'(w * 4), safe_data(w, $urandom));
        end
      end
      rand_ready = 1'b1;
      settle();
      check_outputs("after a burst of writes");
    end
  endtask

  // BUFFER_RELEASE pulses: one cycle per write, both bits together, OR-merge of a burst.
  task automatic test_buffer_release();
    int c0;
    int c1;
    logic [31:0] d;
    where = "buffer release";
    rel_pulses[0] = 0;
    rel_pulses[1] = 0;
    wr(12'h0B0, 32'h1);  settle();
    expect_eq("release_buf[0] pulses after a write of 1", rel_pulses[0], 1);
    expect_eq("release_buf[1] pulses after a write of 1", rel_pulses[1], 0);
    wr(12'h0B0, 32'h2);  settle();
    expect_eq("release_buf[1] pulses after a write of 2", rel_pulses[1], 1);
    wr(12'h0B0, 32'h3);  settle();
    expect_eq("release_buf[0] pulses after a write of 3", rel_pulses[0], 2);
    expect_eq("release_buf[1] pulses after a write of 3", rel_pulses[1], 2);
    wr(12'h0B0, 32'h0);  settle();
    wr(12'h0B0, 32'hFFFF_FFFC);  settle();
    expect_eq("release_buf[0] pulses after writes of 0", rel_pulses[0], 2);
    expect_eq("release_buf[1] pulses after writes of 0", rel_pulses[1], 2);
    rand_ready   = 1'b0;
    bready_delay = 0;
    c0 = rel_pulses[0];
    c1 = rel_pulses[1];
    for (int i = 0; i < 8; i++) wr(12'h0B0, (i % 2) ? 32'h2 : 32'h1, 4'hF);
    rand_ready = 1'b1;
    settle();
    expect_true("release_buf[0] pulses of a burst of 4 writes in 1..4",
                rel_pulses[0] - c0 >= 1 && rel_pulses[0] - c0 <= 4);
    expect_true("release_buf[1] pulses of a burst of 4 writes in 1..4",
                rel_pulses[1] - c1 >= 1 && rel_pulses[1] - c1 <= 4);
    expect_eq("masks not delivered", pend_firm(1), 0);
    rdv(12'h0B0, d);
    expect_eq("BUFFER_RELEASE reads 0", d, 32'h0);
    check_outputs("buffer release");
  endtask

  // IRQ_STATUS writes: strobes of the written bits, response held until the bits read 0 or the
  // timeout, no wait for a mask of 0.
  task automatic test_irq_clear();
    logic [23:0] m;
    logic [23:0] c;
    logic [31:0] d;
    int          j;
    where = "irq clear";
    irq_rate = 0;
    repeat (4) begin
      m = 24'($urandom) | 24'h1;
      irq_set(m);
      settle();
      rdv(12'h044, d);
      expect_eq("IRQ_STATUS after events", d, {8'b0, irq_q});
      c = (m & 24'($urandom)) | (m & -m);
      wr(12'h044, {8'($urandom), c});
      settle();
      rdv(12'h044, d);
      expect_eq("IRQ_STATUS after a clear", d, {8'b0, irq_q});
      expect_eq("cleared bits", d[23:0] & c, 24'h0);
    end
    // Byte strobes select the bits of clr_mask; wstrb[3] and wdata[31:24] are ignored.
    irq_set(24'hFF_FFFF);
    settle();
    wr(12'h044, 32'hFFFF_FFFF, 4'b0010);
    settle();
    rdv(12'h044, d);
    expect_eq("IRQ_STATUS after a clear of byte 1", d, 32'h00FF_00FF);
    wr(12'h044, 32'hFF00_0000, 4'hF);
    wr(12'h044, 32'h00FF_FFFF, 4'b0000);
    wr(12'h044, 32'h0000_0000, 4'hF);
    settle();
    rdv(12'h044, d);
    expect_eq("IRQ_STATUS after writes of mask 0", d, 32'h00FF_00FF);
    // An event that keeps a bit set: response at the timeout.
    j = $urandom_range(23, 0);
    irq_persist = 24'h1 << j;
    sclk(2);
    settle();
    wr(12'h044, 32'h1 << j);
    wr(12'h044, (32'h1 << j) | 32'h00FF_00FF);
    irq_persist = 24'h0;
    settle();
    wr(12'h044, 32'h00FF_FFFF);
    settle();
    rdv(12'h044, d);
    expect_eq("IRQ_STATUS after the final clear", d, 32'h0);
    // A clear of a bit that is not set; back-to-back writes.
    wr(12'h044, 32'h0000_0001);
    irq_set(24'h00_0F0F);
    settle();
    rand_ready = 1'b0;
    bready_delay = 0;
    wr(12'h044, 32'h0000_0003);
    wr(12'h044, 32'h0000_0F0C);
    rand_ready = 1'b1;
    settle();
    rdv(12'h044, d);
    expect_eq("IRQ_STATUS after back-to-back clears", d, 32'h0);
  endtask

  // Single-bit status levels read after the crossing.
  task automatic test_single_bit_status();
    where = "single-bit status";
    repeat (8) begin
      rand_single_bits();
      irq_set(24'($urandom));
      settle();
      rd(12'h020); rd(12'h024); rd(12'h028); rd(12'h034); rd(12'h044);
      rd(12'h0A4); rd(12'h064); rd(12'h06C);
    end
    wr(12'h044, 32'h00FF_FFFF);
    settle();
    rd(12'h044);
  endtask

  // Multi-bit sys_clk status changing continuously; reads must be coherent.
  task automatic test_sys_status_snapshot();
    where = "sys status snapshot";
    sys_act = 100;
    ts_run  = 1'b1;
    ts_step = 64'd1 + 64'($urandom_range(1000, 0));
    repeat (40) begin
      case ($urandom_range(3, 0))
        0: read_status_all();
        default: begin
          rd(12'h02C); rd(12'h060); rd(12'h084); rd(12'h088); rd(12'h0C4); rd(12'h0D0);
        end
      endcase
    end
    repeat (200) rd(12'(4 * $urandom_range(63, 0)));
    sys_act = 0;
    ts_run  = 1'b0;
    settle();
    read_status_all();
  endtask

  // rxbyteclkhs status coherent; held while the clock is stopped; outputs wait for the clock.
  task automatic test_byte_status_snapshot();
    logic [1:0]  al;
    logic        sr;
    where = "byte status snapshot";
    byte_act = 100;
    repeat (150) begin
      rd(12'h038); rd(12'h008); rd(12'h028);
    end
    byte_act = 0;
    settle();
    rd(12'h038); rd(12'h008); rd(12'h028);
    byte_clock(1'b0);
    lclk(30);
    rd(12'h038); rd(12'h008); rd(12'h028);
    al = o_active_lanes;
    sr = o_soft_reset_byte;
    wr(12'h008, rg[W_PROT] ^ 32'h3);
    wr(12'h004, rg[W_CORE] | 32'h2);
    settle();
    expect_eq("active_lanes while rxbyteclkhs is stopped", o_active_lanes, al);
    expect_eq("soft_reset_byte while rxbyteclkhs is stopped", o_soft_reset_byte, sr);
    rd(12'h038); rd(12'h008);
    byte_clock(1'b1);
    settle();
    check_outputs("after rxbyteclkhs restarts");
    wr(12'h004, rg[W_CORE] & ~32'h2);
    byte_act = 100;
    repeat (60) begin
      rd(12'h038); rd(12'h008);
    end
    byte_act = 0;
    settle();
    rd(12'h038); rd(12'h008); rd(12'h028);
    check_outputs("byte status");
  endtask

  task automatic sot_all(input bit sync);
    for (int x = 0; x < 4; x++) sot_req[sync][x] = 1'b1;
    bclk(2);
  endtask

  task automatic expect_sot(input string what, input logic [3:0] e0, input logic [3:0] e1);
    logic [31:0] d;
    rdv(12'h028, d);
    for (int x = 0; x < 4; x++) begin
      expect_eq($sformatf("%s: lane%0d_sot_error", what, x), d[8 * x + 1], e0[x] & LANE_MASK[x]);
      expect_eq($sformatf("%s: lane%0d_sot_sync_error", what, x), d[8 * x + 2],
                e1[x] & LANE_MASK[x]);
    end
  endtask

  // SoT flags: sticky per lane, cleared by soft_reset, also with rxbyteclkhs stopped.
  task automatic test_sot_flags();
    where = "sot flags";
    expect_sot("after reset", 4'h0, 4'h0);
    for (int x = 0; x < 4; x++) begin
      sot_req[0][x] = 1'b1;
      bclk(2);
      settle();
      expect_sot($sformatf("errsoths on lane %0d", x), 4'hF >> (3 - x), 4'h0);
    end
    // A second request one cycle after a pulse is driven after one idle cycle, inside one
    // synchronizer window; the sticky flag is set.
    for (int x = 0; x < 4; x++) begin
      sot_req[1][x] = 1'b1;
      bclk(1);
      sot_req[1][x] = 1'b1;
      bclk(3);
      settle();
      expect_sot($sformatf("errsotsynchs twice on lane %0d", x), 4'hF, 4'hF >> (3 - x));
    end
    wr(12'h004, rg[W_CORE] | 32'h2);
    expect_sot("soft_reset = 1", 4'h0, 4'h0);
    wr(12'h004, rg[W_CORE] & ~32'h2);
    settle();
    expect_sot("soft_reset back to 0", 4'h0, 4'h0);
    sot_all(1'b0);
    sot_all(1'b1);
    settle();
    expect_sot("pulses on every lane", 4'hF, 4'hF);
    byte_clock(1'b0);
    wr(12'h004, rg[W_CORE] | 32'h2);
    expect_sot("soft_reset with rxbyteclkhs stopped", 4'h0, 4'h0);
    wr(12'h004, rg[W_CORE] & ~32'h2);
    lclk(10);
    expect_sot("soft_reset back to 0 with rxbyteclkhs stopped", 4'h0, 4'h0);
    byte_clock(1'b1);
    settle();
    expect_sot("rxbyteclkhs restarted", 4'h0, 4'h0);
    sot_all(1'b1);
    settle();
    expect_sot("errsotsynchs on every lane", 4'h0, 4'hF);
    check_outputs("sot flags");
  endtask

  // TIMESTAMP_LO and TIMESTAMP_HI read as one 64-bit value through the shadow.
  task automatic test_timestamp();
    logic [31:0] lo;
    logic [31:0] hi;
    where = "timestamp";
    rdv(12'h088, hi);
    expect_eq("TIMESTAMP_HI before any TIMESTAMP_LO read", hi, 32'h0);
    sclk();
    i_timestamp = {32'h1234_5678, 32'hFFFF_F000};
    rec_all();
    ts_step     = 64'd37;
    ts_run      = 1'b1;
    repeat (40) begin
      rd(12'h084);
      rd(12'h088);
      if ($urandom_range(2, 0) == 0) rd(12'h088);
    end
    ts_step = 64'h0000_0000_0100_0001;
    repeat (40) begin
      rd(12'h084);
      lclk($urandom_range(5, 0));
      rd(12'h088);
    end
    ts_run = 1'b0;
    settle();
    rdv(12'h084, lo);
    rdv(12'h088, hi);
    expect_eq("TIMESTAMP_LO with the timestamp stopped", lo, i_timestamp[31:0]);
    expect_eq("TIMESTAMP_HI with the timestamp stopped", hi, i_timestamp[63:32]);
    sclk();
    i_timestamp = i_timestamp + 64'h0000_0005_0000_0000;
    rec_all();
    settle();
    rdv(12'h088, hi);
    expect_eq("TIMESTAMP_HI without a new TIMESTAMP_LO read", hi, i_timestamp[63:32] - 32'd5);
  endtask

  // GENERIC_SHORT_PACKET: each non-zero read pops one entry, in order; empty reads give 0.
  task automatic test_generic_short_packet();
    logic [31:0] d;
    int          k;
    int          nz;
    where = "generic short packet";
    rdv(12'h070, d);
    expect_eq("GENERIC_SHORT_PACKET with the FIFO empty", d, 32'h0);
    repeat (4) begin
      k = $urandom_range(8, 0);
      for (int i = 0; i < k; i++) gsp_push(24'($urandom) | 24'h1);
      settle();
      nz = 0;
      repeat (k + 2) begin
        rdv(12'h070, d);
        if (d != 0) nz++;
      end
      expect_eq("non-zero GENERIC_SHORT_PACKET reads", nz, k);
      gsp_check_counts("round");
      expect_eq("FIFO entries left", gsp_fifo.size(), 0);
    end
    for (int i = 0; i < 3; i++) gsp_push(24'($urandom) | 24'h1);
    settle();
    rd(12'h070);
    for (int i = 0; i < 2; i++) gsp_push(24'($urandom) | 24'h1);
    repeat (8) rd(12'h070);
    settle();
    repeat (3) rd(12'h070);
    gsp_check_counts("pushes during reads");
    expect_eq("FIFO entries left after mixed pushes", gsp_fifo.size(), 0);
  endtask

  // AXI4-Lite channel orders, write before read, long READY stalls.
  task automatic test_axi_ordering();
    logic [31:0] d;
    logic [31:0] v;
    int          w;
    where = "axi ordering";
    repeat (6) begin
      w = ($urandom_range(1, 0) == 1) ? W_ER : W_RID;
      for (int o = 0; o < 3; o++) begin
        for (int g = 0; g < 6; g++) begin
          axi_write(12'(w * 4), $urandom, 4'hF, o, g);
          rd(12'(w * 4));
        end
      end
      // AR with AW and W in the same cycle: the write is served first.
      v = $urandom;
      lclk(3);
      fork
        axi_write(12'(w * 4), v, 4'hF, 0, 0);
        axi_read(12'(w * 4), d);
      join
      expect_eq("read in the cycle of a write returns the new value", d, v);
      // AR with AW only: the read is served first.
      v = $urandom;
      lclk(3);
      fork
        axi_write(12'(w * 4), v, 4'hF, 1, 6);
        rd(12'(w * 4));
      join
      rd(12'(w * 4));
      // Long READY stalls.
      rand_ready   = 1'b0;
      bready_delay = $urandom_range(40, 10);
      rready_delay = $urandom_range(40, 10);
      wr(12'(w * 4), $urandom);
      rd(12'(w * 4));
      // AW presented while the previous write waits for BREADY.
      v = $urandom;
      lclk(3);
      fork
        axi_write(12'h018, $urandom, 4'hF, 0, 0);
        begin
          lclk(6);
          send_aw(12'h07C);
        end
      join
      bready_delay = 0;
      send_w(v, 4'hF);
      wait_b();
      rd(12'h07C);
      rready_delay = 0;
      rand_ready   = 1'b1;
    end
    check_outputs("axi ordering");
  endtask

  // DECERR above 0x0FF without effect; reserved offsets read 0.
  task automatic test_address_map();
    logic [11:0] a;
    where = "address map";
    write_all_random();
    settle();
    wr(12'h104, 32'hFFFF_FFFF);
    wr(12'h10C, 32'hFFFF_FFFF);
    wr(12'h144, 32'hFFFF_FFFF);
    wr(12'h1B0, 32'hFFFF_FFFF);
    gsp_push(24'h12_3456);
    settle();
    rd(12'h170);
    rd(12'h184);
    repeat (60) begin
      a = 12'($urandom_range(12'hFFF, 12'h100));
      if ($urandom_range(1, 0) == 1) wr(a, $urandom, 4'($urandom));
      else rd(a);
    end
    for (int w = 0; w < 64; w++) begin
      if (reserved_w(w)) begin
        wr(12'(w * 4), $urandom);
        rd(12'(w * 4));
      end
    end
    settle();
    read_all();
    check_outputs("address map");
    expect_eq("phy_core_rst after DECERR writes", o_phy_core_rst, 1'b0);
    expect_eq("full_reset_req after DECERR writes", o_full_reset_req, 1'b0);
  endtask

  // lite_aresetn during transactions: no write without both channels, contents and outputs
  // kept, crossings in flight still delivered.
  task automatic test_lite_reset();
    logic [31:0] d;
    int          j;
    where = "lite reset";
    write_all_random();
    settle();
    // After AW only.
    send_aw(12'h018);
    lclk($urandom_range(3, 0));
    lite_reset($urandom_range(4, 1), 1'b1);
    // After W only.
    send_w($urandom, 4'hF);
    lclk($urandom_range(3, 0));
    lite_reset($urandom_range(4, 1), 1'b1);
    read_all();
    // During the IRQ_STATUS wait.
    j = $urandom_range(23, 0);
    irq_persist = 24'h1 << j;
    sclk(2);
    settle();
    fork
      wr(12'h044, 32'h1 << j);
      begin
        @(ev_wr_done);
        lclk(1);
        lite_reset(2, 1'b1);
      end
    join
    irq_persist = 24'h0;
    settle();
    // With BVALID stalled.
    rand_ready   = 1'b0;
    bready_delay = 1000000;
    fork
      wr(12'h07C, $urandom);
      begin
        wait (o_s_axi_bvalid === 1'b1);
        lclk(3);
        lite_reset(3, 1'b1);
      end
    join
    // With RVALID stalled.
    rready_delay = 1000000;
    fork
      rd(12'h07C);
      begin
        wait (o_s_axi_rvalid === 1'b1);
        lclk(3);
        lite_reset(3, 1'b1);
      end
    join
    bready_delay = 0;
    rready_delay = 0;
    rand_ready   = 1'b1;
    // During a GENERIC_SHORT_PACKET read held for the post-pop status.
    gsp_push(24'h00_00A1);
    gsp_push(24'h00_00B2);
    gsp_push(24'h00_00C3);
    settle();
    fork
      begin
        rd(12'h070);
        rd(12'h070);
      end
      begin
        @(ev_ar_done);
        @(ev_ar_done);
        lclk(1);
        lite_reset(3, 1'b1);
      end
    join
    gsp_resync();
    repeat (4) rd(12'h070);
    gsp_check_counts("after lite reset");
    // Contents and outputs unchanged.
    settle();
    read_all();
    check_outputs("after lite resets");
    wr(12'h044, 32'h00FF_FFFF);
    settle();
    rdv(12'h044, d);
    expect_eq("IRQ_STATUS after the final clear", d, 32'h0);
  endtask

  // PHY Reset Sequencer: width, restart, phy_rst_in_progress, reset_in_progress, resetn.
  task automatic test_phy_reset_sequencer();
    logic [31:0] d;
    logic [31:0] phy_val;
    int          h1;
    int          h2;
    where = "phy reset sequencer";
    wr(12'h00C, 32'h0000_0581);
    phy_val = rg[W_PHY];
    settle();
    repeat (2) begin
      wr(12'h00C, rg[W_PHY] | 32'h2, 4'h1);
      h1 = trig_cyc[trig_cyc.size() - 1];
      if (N_RST >= 10) begin
        rdv(12'h020, d);
        expect_eq("reset_in_progress during the sequence", d[0], 1'b1);
      end
      while (o_phy_core_rst !== 1'b0) lclk();
      lclk(2);
      expect_eq("phy_core_rst width", phy_fall_cyc - phy_rise_cyc, N_RST);
      expect_eq("phy_core_rst start", phy_rise_cyc - h1, 2);
      settle();
      rdv(12'h020, d);
      expect_eq("reset_in_progress after the sequence", d[0], 1'b0);
      rdv(12'h00C, d);
      expect_eq("PHY_CONFIG after dphy_rst_req", d, phy_val);
    end
    // Restart: the second trigger lands well inside the first sequence.
    if (N_RST >= 20) begin
      rand_ready   = 1'b0;
      bready_delay = 0;
      wr(12'h00C, rg[W_PHY] | 32'h2, 4'h1);
      h1 = trig_cyc[trig_cyc.size() - 1];
      lclk($urandom_range(N_RST / 2, 1));
      wr(12'h00C, rg[W_PHY] | 32'h2, 4'h1);
      h2 = trig_cyc[trig_cyc.size() - 1];
      rand_ready = 1'b1;
      while (o_phy_core_rst !== 1'b0) lclk();
      lclk(2);
      expect_eq("phy_core_rst width after a restart", phy_fall_cyc - phy_rise_cyc,
                h2 - h1 + N_RST);
      settle();
    end
    expect_eq("full_reset_req after dphy_rst_req", o_full_reset_req, 1'b0);
    do_resetn($urandom_range(6, 1));
    rd(12'h020);
    check_outputs("after resetn");
  endtask

  // full_reset: OKAY response, sequence width, every register and output back to reset.
  task automatic test_full_reset();
    where = "full reset";
    repeat (2) begin
      write_all_random();
      settle();
      check_outputs("before full_reset");
      do_full_reset();
      read_all();
      check_outputs("after full_reset");
    end
    write_all_random();
    read_all();
    settle();
    check_outputs("written after full_reset");
  endtask

  // soft_reset: outputs in both domains, reset_in_progress, SoT flags, nothing else.
  task automatic test_soft_reset();
    logic [31:0] d;
    where = "soft reset";
    write_all_random();
    wr(12'h004, rg[W_CORE] & ~32'h2);
    settle();
    sot_all(1'b0);
    settle();
    wr(12'h004, rg[W_CORE] | 32'h2);
    settle();
    check_outputs("soft_reset = 1");
    rdv(12'h020, d);
    expect_eq("reset_in_progress with soft_reset", d[0], 1'b1);
    expect_sot("soft_reset = 1", 4'h0, 4'h0);
    read_all();
    wr(12'h004, rg[W_CORE] & ~32'h2);
    settle();
    check_outputs("soft_reset = 0");
    rdv(12'h020, d);
    expect_eq("reset_in_progress after soft_reset", d[0], 1'b0);
    read_all();
  endtask

  // D-PHY enables: phy_enable gating, forcerxmode direct, lanes not built at 0.
  task automatic test_dphy_outputs();
    logic [31:0] d;
    logic [8:0]  e;
    where = "dphy outputs";
    for (int i = 0; i < 24; i++) begin
      d = {21'b0, 4'($urandom), 4'($urandom), i[1], 1'b0, i[0]};
      if (i >= 16) d = safe_data(W_PHY, $urandom);
      wr(12'h00C, d);
      lclk(6);
      e = phy_f(rg[W_PHY]);
      expect_eq("cl_enable", o_cl_enable, e[0]);
      expect_eq("dl_enable", o_dl_enable, e[4:1]);
      expect_eq("forcerxmode", o_forcerxmode, e[8:5]);
      rd(12'h00C);
    end
  endtask

  // Parameter read-back fields, slots and lanes not built.
  task automatic test_parameter_readback();
    logic [31:0] d;
    logic [31:0] e;
    where = "parameter readback";
    rdv(12'h000, d);
    expect_eq("IP_VERSION", d, IP_VERSION);
    wr(12'h000, 32'hFFFF_FFFF);
    wr(12'h008, 32'h0000_0F00 | rg[W_PROT]);
    wr(12'h0A0, 32'h007F_0000 | rg[W_ATTR]);
    rdv(12'h008, d);
    expect_eq("PROTOCOL_CONFIG.max_lanes", d[9:8], LANES_CODE);
    rdv(12'h0A0, d);
    expect_eq("DMA_ATTR.max_outstanding", d[22:16], PARA_MAX_OUTSTANDING[6:0]);
    rd(12'h000);
    for (int m = 0; m < 2; m++) begin
      wr(12'h0E0 + 12'(4 * m), 32'hFFFF_FFFF);
      rdv(12'h0E0 + 12'(4 * m), d);
      expect_eq($sformatf("EMB_EXTRACT_CFG%0d", m), d, (m < NX) ? 32'hFFFF_FF07 : 32'h0);
    end
    sclk();
    for (int m = 0; m < NXW; m++) begin
      i_emb_extract_val_vc0[m] = $urandom | 32'h1;
      i_emb_extract_val_vc1[m] = $urandom | 32'h1;
    end
    rec_all();
    settle();
    check_outputs("emb_extract_cfg");
    for (int w = W_VAL00; w <= W_VAL11; w++) rd(12'(w * 4));
    async_act = 1'b0;
    i_stopstate = 4'hF;
    rec_all();
    bclk();
    i_rxskewcalhs = 4'hF;
    rec_all();
    sot_all(1'b0);
    sot_all(1'b1);
    settle();
    rdv(12'h028, d);
    e = 32'h0;
    for (int x = 0; x < 4; x++) if (x < NL) e[8 * x +: 4] = 4'hF;
    expect_eq("LANE_STATUS with every lane input at 1", d, e);
    wr(12'h00C, 32'hFFFF_FFFD);
    lclk(6);
    expect_eq("dl_enable with every PHY_CONFIG bit set", o_dl_enable, LANE_MASK);
    expect_eq("forcerxmode with every PHY_CONFIG bit set", o_forcerxmode, LANE_MASK);
    rdv(12'h00C, d);
    expect_eq("PHY_CONFIG stores the bits of every lane", d, 32'h0000_07FD);
    check_outputs("parameter readback");
  endtask

  // Long random mix of accesses, status activity, commands, resets and clock stops.
  task automatic test_random_long();
    int          op;
    int          w;
    int          t_end;
    logic [31:0] d;
    where     = "random";
    sys_act   = 10;
    byte_act  = 10;
    async_act = 1'b1;
    irq_rate  = 1;
    sot_rand  = 1;
    ts_run    = 1'b1;
    ts_step   = 64'd1;
    t_end     = lcyc + 150000;
    while (lcyc < t_end) begin
      op = $urandom_range(999, 0);
      if (op < 350) begin
        w = $urandom_range(63, 0);
        if (w != W_IRQ && w != W_REL)
          wr(12'(w * 4), safe_data(w, $urandom), ($urandom_range(4, 0) == 0) ? 4'($urandom) : 4'hF);
      end else if (op < 650) begin
        rd(12'(4 * $urandom_range(63, 0)));
      end else if (op < 690) begin
        wr(12'h044, {8'($urandom), irq_q & 24'($urandom)});
      end else if (op < 710) begin
        wr(12'h0B0, $urandom);
      end else if (op < 740) begin
        repeat ($urandom_range(3, 1)) gsp_push(24'($urandom) | 24'h1);
      end else if (op < 790) begin
        rd(12'h070);
      end else if (op < 800) begin
        if ($urandom_range(1, 0) == 1) wr(12'($urandom_range(12'hFFF, 12'h100)), $urandom);
        else rd(12'($urandom_range(12'hFFF, 12'h100)));
      end else if (op < 806) begin
        if (o_phy_core_rst === 1'b0 && lcyc - phy_fall_cyc > 6)
          wr(12'h00C, rg[W_PHY] | 32'h2, 4'h1);
      end else if (op < 808) begin
        if (o_phy_core_rst === 1'b0 && lcyc - phy_fall_cyc > 6) begin
          where = "random full_reset";
          do_full_reset();
          where = "random";
        end
      end else if (op < 809) begin
        where = "random resetn";
        do_resetn($urandom_range(10, 1));
        where = "random";
      end else if (op < 814) begin
        lite_reset($urandom_range(4, 1));
      end else if (op < 824) begin
        if (byte_run && $realtime - rf_last_reset_t > 4.0 * LAT_MAX) byte_clock(1'b0);
        else byte_clock(1'b1);
      end else if (op < 834) begin
        irq_persist = ($urandom_range(2, 0) == 0) ? (24'h1 << $urandom_range(23, 0)) : 24'h0;
      end else if (op < 844) begin
        byte_clock(1'b1);
        settle();
        check_outputs("random quiet point");
      end else if (op < 900) begin
        lclk($urandom_range(50, 1));
      end else begin
        rd(12'h084);
        rd(12'h088);
      end
    end
    sys_act   = 0;
    byte_act  = 0;
    async_act = 1'b0;
    irq_rate  = 0;
    sot_rand  = 0;
    ts_run    = 1'b0;
    irq_persist = 24'h0;
    byte_clock(1'b1);
    settle();
    check_outputs("end of random");
    read_all();
    rdv(12'h044, d);
    wr(12'h044, d);
    $display("random: %0d writes, %0d reads, %0d GSP pops", n_writes, n_reads, gsp_pops);
  endtask

  // Forces a wrong core_enable; this test must end in FAIL with errors in the log.
  // It confirms that the checkers catch a fault and is not part of the regression.
  task automatic test_checker_fault();
    where = "checker fault";
    force o_core_enable = ~rg[W_CORE][0];
    lclk(3);
    check_outputs("forced core_enable");
    release o_core_enable;
    lclk(3);
  endtask

  // --------------------------------------------------------------------------------------------
  // Test selection and result
  // --------------------------------------------------------------------------------------------
  string       test_name;
  int unsigned seed;

  initial begin
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "none";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    void'($urandom(seed));
    i_lite_aresetn  = 1'b0;
    i_resetn        = 1'b0;
    i_s_axi_awaddr  = '0;
    i_s_axi_awprot  = '0;
    i_s_axi_awvalid = 1'b0;
    i_s_axi_wdata   = '0;
    i_s_axi_wstrb   = '0;
    i_s_axi_wvalid  = 1'b0;
    i_s_axi_bready  = 1'b0;
    i_s_axi_araddr  = '0;
    i_s_axi_arprot  = '0;
    i_s_axi_arvalid = 1'b0;
    i_s_axi_rready  = 1'b0;
    i_cl_stopstate     = 1'b0;
    i_cl_ulpsactivenot = 1'b1;
    i_cl_rxclkactivehs = 1'b0;
    i_cl_ulpsclknot    = 1'b1;
    i_init_done        = 1'b0;
    i_stopstate        = 4'h0;
    i_rxskewcalhs      = 4'h0;
    i_errsoths         = 4'h0;
    i_errsotsynchs     = 4'h0;
    i_lm_fsm_state         = '0;
    i_abort_sync_err       = 1'b0;
    i_abort_fifo_full      = 1'b0;
    i_abort_unarmed        = 1'b0;
    i_deskew_burst_seen    = 1'b0;
    i_abort_lane_sync_seen = '0;
    i_active_lanes_applied = '0;
    i_sp_fifo_not_empty    = 1'b0;
    i_sp_fifo_full         = 1'b0;
    i_null_blank_seen      = 1'b0;
    i_dt_conflict          = 1'b0;
    i_vc_unsupported_seen  = 1'b0;
    i_gsp_data             = '0;
    i_active_sel           = 1'b0;
    i_switch_pending       = 1'b0;
    i_linebuffer_full      = 1'b0;
    i_linebuf_fill         = '0;
    i_frame_count          = '0;
    i_timestamp            = '0;
    i_frame_id_buf0        = '0;
    i_frame_id_buf1        = '0;
    i_vc0_line_count       = '0;
    i_vc0_byte_count       = '0;
    i_vc0_data_type        = '0;
    i_vc0_width_mismatch   = 1'b0;
    i_vc0_height_mismatch  = 1'b0;
    i_vc1_line_count       = '0;
    i_vc1_byte_count       = '0;
    i_vc1_data_type        = '0;
    i_vc1_width_mismatch   = 1'b0;
    i_vc1_height_mismatch  = 1'b0;
    i_emb_word_count_vc0   = '0;
    i_emb_line_count_vc0   = '0;
    i_emb_word_count_vc1   = '0;
    i_emb_line_count_vc1   = '0;
    i_emb_extract_val_vc0  = '0;
    i_emb_extract_val_vc1  = '0;
    i_meta_overflow        = 1'b0;
    i_irq_status           = '0;
    i_error_capture        = '0;
    i_packet_cnt           = '0;
    i_ecc_err_cnt          = '0;
    i_crc_err_cnt          = '0;
    i_frame_drop_cnt       = '0;
    i_lane_abort_cnt       = '0;
    i_linebuf_ovf_sticky   = 1'b0;
    i_sp_fifo_ovf_sticky   = 1'b0;
    i_dma_ovf_sticky       = 1'b0;
    i_dma_busy             = 1'b0;
    i_active_buffer        = 1'b0;
    i_buf0_state           = '0;
    i_buf1_state           = '0;
    i_buf0_last_valid_line = '0;
    i_buf1_last_valid_line = '0;
    i_axi_fault_awid       = '0;
    i_axi_fault_resp       = '0;
    i_axi_fault_awaddr     = '0;
    i_bresp_slverr_count   = '0;
    i_bresp_decerr_count   = '0;
    for (int k = 0; k < 2; k++) begin
      for (int x = 0; x < 4; x++) begin
        sot_req[k][x] = 1'b0;
        sot_t[k][x]     = -1.0e9;
        sot_tprev[k][x] = -2.0e9;
        sot_par[k][x]   = 1'b0;
      end
      sot_last[k] = 4'h0;
    end
    rel_pulses[0] = 0;
    rel_pulses[1] = 0;
    for (int g = 0; g < NG; g++) ghist[g].delete();
    model_reset();
    for (int g = 0; g < NG; g++) g_last[g] = '0;
    rec_all();
    $display("%s: MAX_LANES=%0d N_EXTRACT=%0d MAX_OUTSTANDING=%0d SYNC_STAGES=%0d PHY_RST_CYCLES=%0d",
             test_name, NL, NX, PARA_MAX_OUTSTANDING, S, N_RST);
    $display("%s: IRQ_CLR_TIMEOUT=%0d clocks lite/sys/byte = %0.1f/%0.1f/%0.1f ns SEED=%0d",
             test_name, N_CLR, T_LITE, T_SYS, T_BYTE, seed);
    started = 1'b1;
    // Power-on: both resets low, lite_aresetn released first.
    lclk(4);
    warm           = 1'b1;
    lclk(1);
    i_lite_aresetn = 1'b1;
    lclk(4);
    #1.0;
    i_resetn = 1'b1;
    where = "power-on";
    wait_ready();
    expect_eq("phy_core_rst samples after the power-on release of resetn",
              phy_fall_cyc - resetn_rel_cyc, S + N_RST + 1);
    gsp_relax = 1'b0;
    init_done = 1'b1;
    for (int g = 0; g < NG; g++) g_last[g] = grp_out(g);
    case (test_name)
      "test_reset_values":          test_reset_values();
      "test_rw_readback":           test_rw_readback();
      "test_write_strobes":         test_write_strobes();
      "test_config_outputs":        test_config_outputs();
      "test_control_coherency":     test_control_coherency();
      "test_buffer_release":        test_buffer_release();
      "test_irq_clear":             test_irq_clear();
      "test_single_bit_status":     test_single_bit_status();
      "test_sys_status_snapshot":   test_sys_status_snapshot();
      "test_byte_status_snapshot":  test_byte_status_snapshot();
      "test_sot_flags":             test_sot_flags();
      "test_timestamp":             test_timestamp();
      "test_generic_short_packet":  test_generic_short_packet();
      "test_axi_ordering":          test_axi_ordering();
      "test_address_map":           test_address_map();
      "test_lite_reset":            test_lite_reset();
      "test_phy_reset_sequencer":   test_phy_reset_sequencer();
      "test_full_reset":            test_full_reset();
      "test_soft_reset":            test_soft_reset();
      "test_dphy_outputs":          test_dphy_outputs();
      "test_parameter_readback":    test_parameter_readback();
      "test_random_long":           test_random_long();
      "test_checker_fault":         test_checker_fault();
      default: begin
        err_count++;
        $error("unknown test '%s'", test_name);
      end
    endcase
    where = "end of test";
    irq_persist = 24'h0;
    byte_clock(1'b1);
    settle();
    if (gsp_relax) gsp_resync();
    else gsp_check_counts("end of test");
    expect_eq("IRQ_STATUS masks not delivered", pend_firm(0), 0);
    expect_eq("BUFFER_RELEASE masks not delivered", pend_firm(1), 0);
    $display("%s: %0d writes, %0d reads, %0d checks", test_name, n_writes, n_reads, check_count);
    if (err_count == 0 && check_count >= 10) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d, checks=%0d)", test_name, err_count, check_count);
    $finish;
  end

  // Timeout.
  initial begin
    #50_000_000.0;
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
