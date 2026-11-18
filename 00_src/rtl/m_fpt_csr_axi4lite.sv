// Module      : m_fpt_csr_axi4lite
// Block       : B14 - CSR & AXI4-Lite Subordinate
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.3
// Date        : 2026-11-15
// Description : AXI4-Lite register map, control and status crossings, D-PHY enables and reset.
`timescale 1ns/1ps
`default_nettype none

module m_fpt_csr_axi4lite #(
  // Parameter Description: D-PHY data lanes built; equal to MAX_LANES of Lane Management.
  // Parameter values: 1, 2 or 4; default is 4.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_LANES       = 32'd4,
  // Parameter Description: extraction slots per channel; equal to N_EXTRACT of Block No. 9.
  // Parameter values: 0, 1 or 2; default is 2. 0 removes the extraction registers.
  // Parameter unit: count
  parameter logic [31:0] PARA_N_EXTRACT       = 32'd2,
  // Parameter Description: value read in DMA_ATTR.max_outstanding; equal to MAX_OUTSTANDING of DMA.
  // Parameter values: 1 to 127; default is 8.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_OUTSTANDING = 32'd8,
  // Parameter Description: flops of every synchronizer and reset synchronizer of the block.
  // Parameter values: 2 or more; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_SYNC_STAGES     = 32'd2,
  // Parameter Description: width of phy_core_rst, at least ceil(40 x T_core_clk / T_lite_aclk).
  // Parameter values: 1 to 65535; default is 40.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_PHY_RST_CYCLES  = 32'd40,
  // Parameter Description: bound of the hold of an IRQ_STATUS write response, in lite_aclk.
  // Parameter values: 1 to 65535; default is 16.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_IRQ_CLR_TIMEOUT = 32'd16
) (
  // Clocks and resets. sys_resetn and rxbyteclkhs_resetn are released in their own domain.
  input  wire         i_lite_aclk,
  input  wire         i_lite_aresetn,
  input  wire         i_resetn,
  input  wire         i_sys_clk,
  input  wire         i_rxbyteclkhs,
  input  wire         i_sys_resetn,
  input  wire         i_rxbyteclkhs_resetn,
  // AMBA AXI4-Lite (IHI0022H), subordinate, lite_aclk. AW: addr, prot, valid/ready;
  // W: data, strb, valid/ready; B: resp, valid/ready; AR: addr, prot, valid/ready;
  // R: data, resp, valid/ready.
  input  wire  [11:0] i_s_axi_awaddr,
  input  wire  [2:0]  i_s_axi_awprot,
  input  wire         i_s_axi_awvalid,
  output logic        o_s_axi_awready,
  input  wire  [31:0] i_s_axi_wdata,
  input  wire  [3:0]  i_s_axi_wstrb,
  input  wire         i_s_axi_wvalid,
  output logic        o_s_axi_wready,
  output logic [1:0]  o_s_axi_bresp,
  output logic        o_s_axi_bvalid,
  input  wire         i_s_axi_bready,
  input  wire  [11:0] i_s_axi_araddr,
  input  wire  [2:0]  i_s_axi_arprot,
  input  wire         i_s_axi_arvalid,
  output logic        o_s_axi_arready,
  output logic [31:0] o_s_axi_rdata,
  output logic [1:0]  o_s_axi_rresp,
  output logic        o_s_axi_rvalid,
  input  wire         i_s_axi_rready,
  // To the D-PHY and to the reset path (lite_aclk)
  output logic        o_phy_core_rst,
  output logic        o_cl_enable,
  output logic [3:0]  o_dl_enable,
  output logic [3:0]  o_forcerxmode,
  output logic        o_full_reset_req,
  // From the D-PHY (asynchronous levels; rxskewcalhs, errsoths, errsotsynchs in rxbyteclkhs)
  input  wire         i_cl_stopstate,
  input  wire         i_cl_ulpsactivenot,
  input  wire         i_cl_rxclkactivehs,
  input  wire         i_cl_ulpsclknot,
  input  wire         i_init_done,
  input  wire  [3:0]  i_stopstate,
  input  wire  [3:0]  i_rxskewcalhs,
  input  wire  [3:0]  i_errsoths,
  input  wire  [3:0]  i_errsotsynchs,
  // Control to Blocks No. 1 and No. 2 (rxbyteclkhs)
  output logic        o_soft_reset_byte,
  output logic [1:0]  o_active_lanes,
  // Control to Blocks No. 3 to No. 13 (sys_clk)
  output logic        o_soft_reset_sys,
  output logic        o_core_enable,
  output logic        o_vc_exposure_map,
  output logic [5:0]  o_metadata_dt,
  output logic        o_metadata_capture_en,
  output logic        o_fnum_check_en,
  output logic        o_emb_extract_en,
  output logic [15:0] o_expected_width,
  output logic [15:0] o_expected_height,
  output logic [31:0] o_metadata_record_id,
  output logic [((PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT) - 32'd1:0][31:0]
                      o_emb_extract_cfg,
  output logic        o_axis_enable,
  output logic        o_stream_format,
  output logic        o_stream_follow_dma_drop,
  output logic        o_tpg_enable,
  output logic        o_tpg_pattern,
  output logic        o_tpg_dual_vc_en,
  output logic        o_tpg_pixel_format,
  output logic [1:0]  o_tpg_cfa_order,
  output logic [15:0] o_tpg_width,
  output logic [15:0] o_tpg_height,
  output logic [15:0] o_tpg_hblank,
  output logic [15:0] o_tpg_vblank,
  output logic        o_dma_enable,
  output logic        o_dma_start,
  output logic        o_dma_abort,
  output logic        o_err_frame_policy,
  output logic        o_dma_format,
  output logic [31:0] o_dma_buffer0_addr,
  output logic [31:0] o_dma_buffer1_addr,
  output logic [31:0] o_dma_buffer_size,
  output logic [31:0] o_meta_area_size,
  output logic [15:0] o_bytes_per_line,
  output logic [7:0]  o_line_padding,
  output logic [3:0]  o_awqos,
  output logic [7:0]  o_max_burst_len,
  output logic [2:0]  o_awprot,
  output logic [1:0]  o_release_buf,
  output logic        o_global_irq_enable,
  output logic [23:0] o_irq_enable_mask,
  output logic [23:0] o_irq_clear_strobe,
  output logic        o_phy_rst_in_progress,
  // Status from Lane Management (rxbyteclkhs)
  input  wire  [2:0]  i_lm_fsm_state,
  input  wire         i_abort_sync_err,
  input  wire         i_abort_fifo_full,
  input  wire         i_abort_unarmed,
  input  wire         i_deskew_burst_seen,
  input  wire  [3:0]  i_abort_lane_sync_seen,
  input  wire  [1:0]  i_active_lanes_applied,
  // Status from Packet Analyzer (sys_clk)
  input  wire         i_sp_fifo_not_empty,
  input  wire         i_sp_fifo_full,
  input  wire         i_null_blank_seen,
  input  wire         i_dt_conflict,
  input  wire         i_vc_unsupported_seen,
  input  wire  [23:0] i_gsp_data,
  output logic        o_gsp_pop,
  // Status from TPG Mux and Line Buffer (sys_clk)
  input  wire         i_active_sel,
  input  wire         i_switch_pending,
  input  wire         i_linebuffer_full,
  input  wire  [15:0] i_linebuf_fill,
  // Status from Frame Context & Metadata Capture (sys_clk)
  input  wire  [31:0] i_frame_count,
  input  wire  [63:0] i_timestamp,
  input  wire  [13:0] i_frame_id_buf0,
  input  wire  [13:0] i_frame_id_buf1,
  input  wire  [15:0] i_vc0_line_count,
  input  wire  [15:0] i_vc0_byte_count,
  input  wire  [5:0]  i_vc0_data_type,
  input  wire         i_vc0_width_mismatch,
  input  wire         i_vc0_height_mismatch,
  input  wire  [15:0] i_vc1_line_count,
  input  wire  [15:0] i_vc1_byte_count,
  input  wire  [5:0]  i_vc1_data_type,
  input  wire         i_vc1_width_mismatch,
  input  wire         i_vc1_height_mismatch,
  input  wire  [15:0] i_emb_word_count_vc0,
  input  wire  [7:0]  i_emb_line_count_vc0,
  input  wire  [15:0] i_emb_word_count_vc1,
  input  wire  [7:0]  i_emb_line_count_vc1,
  input  wire  [((PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT) - 32'd1:0][31:0]
                      i_emb_extract_val_vc0,
  input  wire  [((PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT) - 32'd1:0][31:0]
                      i_emb_extract_val_vc1,
  input  wire         i_meta_overflow,
  // Status from Interrupt & Error Aggregator (sys_clk)
  input  wire  [23:0] i_irq_status,
  input  wire  [31:0] i_error_capture,
  input  wire  [31:0] i_packet_cnt,
  input  wire  [31:0] i_ecc_err_cnt,
  input  wire  [31:0] i_crc_err_cnt,
  input  wire  [31:0] i_frame_drop_cnt,
  input  wire  [15:0] i_lane_abort_cnt,
  input  wire         i_linebuf_ovf_sticky,
  input  wire         i_sp_fifo_ovf_sticky,
  input  wire         i_dma_ovf_sticky,
  // Status from the DMA (sys_clk)
  input  wire         i_dma_busy,
  input  wire         i_active_buffer,
  input  wire  [1:0]  i_buf0_state,
  input  wire  [1:0]  i_buf1_state,
  input  wire  [15:0] i_buf0_last_valid_line,
  input  wire  [15:0] i_buf1_last_valid_line,
  input  wire  [7:0]  i_axi_fault_awid,
  input  wire  [1:0]  i_axi_fault_resp,
  input  wire  [31:0] i_axi_fault_awaddr,
  input  wire  [15:0] i_bresp_slverr_count,
  input  wire  [15:0] i_bresp_decerr_count
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_ADDR_WIDTH     = 32'd12;
  localparam logic [31:0] LPARA_DATA_WIDTH     = 32'd32;
  localparam logic [31:0] LPARA_IRQ_WIDTH      = 32'd24;
  localparam logic [31:0] LPARA_IP_VERSION     = 32'h0001_0200;
  // Offsets 0x00 to 0xFF are decoded; any higher address bit set gives DECERR.
  localparam logic [31:0] LPARA_MAP_BITS       = 32'd8;
  localparam logic [1:0]  LPARA_RESP_OKAY      = 2'b00;
  localparam logic [1:0]  LPARA_RESP_DECERR    = 2'b11;
  localparam logic [31:0] LPARA_RST_CNT_W      =
    (PARA_PHY_RST_CYCLES == 32'd0) ? 32'd1 : $clog2(PARA_PHY_RST_CYCLES + 32'd1);
  localparam logic [31:0] LPARA_WAIT_CNT_W     =
    (PARA_IRQ_CLR_TIMEOUT == 32'd0) ? 32'd1 : $clog2(PARA_IRQ_CLR_TIMEOUT + 32'd1);
  // Extraction ports keep one slot of width when N_EXTRACT is 0.
  localparam logic [31:0] LPARA_NX_SLOTS       = (PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT;
  // PROTOCOL_CONFIG.max_lanes uses the active_lanes encoding: 1 -> 00, 2 -> 01, 4 -> 11.
  localparam logic [1:0]  LPARA_MAX_LANES_CODE = 2'(PARA_MAX_LANES - 32'd1);
  localparam logic [3:0]  LPARA_LANE_MASK      = 4'((32'd1 << PARA_MAX_LANES) - 32'd1);
  // Flops of the rf_rst_n reset synchronizer before its last stage.
  localparam logic [31:0] LPARA_RF_SYNC_W      =
    (PARA_SYNC_STAGES > 32'd1) ? (PARA_SYNC_STAGES - 32'd1) : 32'd1;

  // Register offsets
  localparam logic [7:0]  LPARA_OFS_IP_VERSION             = 8'h00;
  localparam logic [7:0]  LPARA_OFS_CORE_CONFIG            = 8'h04;
  localparam logic [7:0]  LPARA_OFS_PROTOCOL_CONFIG        = 8'h08;
  localparam logic [7:0]  LPARA_OFS_PHY_CONFIG             = 8'h0C;
  localparam logic [7:0]  LPARA_OFS_METADATA_DT_CONFIG     = 8'h10;
  localparam logic [7:0]  LPARA_OFS_STREAM_CONFIG          = 8'h14;
  localparam logic [7:0]  LPARA_OFS_EXPECTED_RESOLUTION    = 8'h18;
  localparam logic [7:0]  LPARA_OFS_CORE_STATUS            = 8'h20;
  localparam logic [7:0]  LPARA_OFS_PHY_STATUS             = 8'h24;
  localparam logic [7:0]  LPARA_OFS_LANE_STATUS            = 8'h28;
  localparam logic [7:0]  LPARA_OFS_PACKET_COUNT           = 8'h2C;
  localparam logic [7:0]  LPARA_OFS_FRAME_COUNT            = 8'h30;
  localparam logic [7:0]  LPARA_OFS_BUFFER_STATUS          = 8'h34;
  localparam logic [7:0]  LPARA_OFS_LANE_ALIGN_STATUS      = 8'h38;
  localparam logic [7:0]  LPARA_OFS_GLOBAL_IRQ_ENABLE      = 8'h40;
  localparam logic [7:0]  LPARA_OFS_IRQ_STATUS             = 8'h44;
  localparam logic [7:0]  LPARA_OFS_IRQ_ENABLE             = 8'h48;
  localparam logic [7:0]  LPARA_OFS_ECC_ERROR_COUNT        = 8'h4C;
  localparam logic [7:0]  LPARA_OFS_CRC_ERROR_COUNT        = 8'h50;
  localparam logic [7:0]  LPARA_OFS_FRAME_DROP_COUNT       = 8'h54;
  localparam logic [7:0]  LPARA_OFS_LANE_ABORT_COUNT       = 8'h58;
  localparam logic [7:0]  LPARA_OFS_VC0_IMAGE_INFO1        = 8'h60;
  localparam logic [7:0]  LPARA_OFS_VC0_IMAGE_INFO2        = 8'h64;
  localparam logic [7:0]  LPARA_OFS_VC1_IMAGE_INFO1        = 8'h68;
  localparam logic [7:0]  LPARA_OFS_VC1_IMAGE_INFO2        = 8'h6C;
  localparam logic [7:0]  LPARA_OFS_GENERIC_SHORT_PACKET   = 8'h70;
  localparam logic [7:0]  LPARA_OFS_EMBEDDED_DATA_INFO_VC0 = 8'h74;
  localparam logic [7:0]  LPARA_OFS_EMBEDDED_DATA_INFO_VC1 = 8'h78;
  localparam logic [7:0]  LPARA_OFS_METADATA_RECORD_ID     = 8'h7C;
  localparam logic [7:0]  LPARA_OFS_FRAME_ID               = 8'h80;
  localparam logic [7:0]  LPARA_OFS_TIMESTAMP_LO           = 8'h84;
  localparam logic [7:0]  LPARA_OFS_TIMESTAMP_HI           = 8'h88;
  localparam logic [7:0]  LPARA_OFS_DMA_CONTROL            = 8'h90;
  localparam logic [7:0]  LPARA_OFS_DMA_BUFFER0_ADDR       = 8'h94;
  localparam logic [7:0]  LPARA_OFS_DMA_BUFFER1_ADDR       = 8'h98;
  localparam logic [7:0]  LPARA_OFS_DMA_BUFFER_SIZE        = 8'h9C;
  localparam logic [7:0]  LPARA_OFS_DMA_ATTR               = 8'hA0;
  localparam logic [7:0]  LPARA_OFS_DMA_STATUS             = 8'hA4;
  localparam logic [7:0]  LPARA_OFS_LINE_PITCH             = 8'hA8;
  localparam logic [7:0]  LPARA_OFS_BUFFER_OWNERSHIP       = 8'hAC;
  localparam logic [7:0]  LPARA_OFS_BUFFER_RELEASE         = 8'hB0;
  localparam logic [7:0]  LPARA_OFS_BUFFER_ERROR_INFO      = 8'hB4;
  localparam logic [7:0]  LPARA_OFS_META_AREA_SIZE         = 8'hB8;
  localparam logic [7:0]  LPARA_OFS_TPG_CONTROL            = 8'hC0;
  localparam logic [7:0]  LPARA_OFS_ERROR_CAPTURE          = 8'hC4;
  localparam logic [7:0]  LPARA_OFS_AXI_FAULT_TRACE        = 8'hC8;
  localparam logic [7:0]  LPARA_OFS_AXI_FAULT_ADDR         = 8'hCC;
  localparam logic [7:0]  LPARA_OFS_AXI_ERROR_COUNT        = 8'hD0;
  localparam logic [7:0]  LPARA_OFS_TPG_RESOLUTION         = 8'hD4;
  localparam logic [7:0]  LPARA_OFS_TPG_BLANKING           = 8'hD8;
  localparam logic [7:0]  LPARA_OFS_EMB_EXTRACT_CFG0       = 8'hE0;
  localparam logic [7:0]  LPARA_OFS_EMB_EXTRACT_CFG1       = 8'hE4;
  localparam logic [7:0]  LPARA_OFS_EMB_EXTRACT_VC0_VAL0   = 8'hE8;
  localparam logic [7:0]  LPARA_OFS_EMB_EXTRACT_VC0_VAL1   = 8'hEC;
  localparam logic [7:0]  LPARA_OFS_EMB_EXTRACT_VC1_VAL0   = 8'hF0;
  localparam logic [7:0]  LPARA_OFS_EMB_EXTRACT_VC1_VAL1   = 8'hF4;

  // Reset values of the stored registers (fields read from a parameter or a status count as 0)
  localparam logic [31:0] LPARA_RST_CORE_CONFIG         = 32'h0000_0001;
  localparam logic [31:0] LPARA_RST_PROTOCOL_CONFIG     = 32'h0000_0003;
  localparam logic [31:0] LPARA_RST_PHY_CONFIG          = 32'h0000_0078;
  localparam logic [31:0] LPARA_RST_METADATA_DT_CONFIG  = 32'h0000_0012;
  localparam logic [31:0] LPARA_RST_STREAM_CONFIG       = 32'h0000_0001;
  localparam logic [31:0] LPARA_RST_EXPECTED_RESOLUTION = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_GLOBAL_IRQ_ENABLE   = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_IRQ_ENABLE          = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_METADATA_RECORD_ID  = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_DMA_CONTROL         = 32'h0000_0010;
  localparam logic [31:0] LPARA_RST_DMA_BUFFER0_ADDR    = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_DMA_BUFFER1_ADDR    = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_DMA_BUFFER_SIZE     = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_DMA_ATTR            = 32'h0000_0FF0;
  localparam logic [31:0] LPARA_RST_LINE_PITCH          = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_META_AREA_SIZE      = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_TPG_CONTROL         = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_TPG_RESOLUTION      = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_TPG_BLANKING        = 32'h0000_0000;
  localparam logic [31:0] LPARA_RST_EMB_EXTRACT_CFG     = 32'h0000_0000;

  // Index of each register that crosses into sys_clk through its own handshake
  localparam logic [31:0] LPARA_HS_METADATA_DT_CONFIG  = 32'd0;
  localparam logic [31:0] LPARA_HS_EXPECTED_RESOLUTION = 32'd1;
  localparam logic [31:0] LPARA_HS_IRQ_ENABLE          = 32'd2;
  localparam logic [31:0] LPARA_HS_METADATA_RECORD_ID  = 32'd3;
  localparam logic [31:0] LPARA_HS_DMA_BUFFER0_ADDR    = 32'd4;
  localparam logic [31:0] LPARA_HS_DMA_BUFFER1_ADDR    = 32'd5;
  localparam logic [31:0] LPARA_HS_DMA_BUFFER_SIZE     = 32'd6;
  localparam logic [31:0] LPARA_HS_DMA_ATTR            = 32'd7;
  localparam logic [31:0] LPARA_HS_LINE_PITCH          = 32'd8;
  localparam logic [31:0] LPARA_HS_META_AREA_SIZE      = 32'd9;
  localparam logic [31:0] LPARA_HS_TPG_CONTROL         = 32'd10;
  localparam logic [31:0] LPARA_HS_TPG_RESOLUTION      = 32'd11;
  localparam logic [31:0] LPARA_HS_TPG_BLANKING        = 32'd12;
  localparam logic [31:0] LPARA_HS_EMB_EXTRACT_CFG0    = 32'd13;
  localparam logic [31:0] LPARA_HS_EMB_EXTRACT_CFG1    = 32'd14;
  localparam logic [31:0] LPARA_HS_NUM                 = 32'd13 + PARA_N_EXTRACT;

  // Access FSM states
  typedef enum logic [2:0] {
    ENUM_ST_IDLE  = 3'b000,
    ENUM_ST_WRITE = 3'b001,
    ENUM_ST_BWAIT = 3'b010,
    ENUM_ST_BRESP = 3'b011,
    ENUM_ST_READ  = 3'b100,
    ENUM_ST_RRESP = 3'b101
  } axil_state_type_def;

  // Multi-bit status of the sys_clk domain carried by one snapshot
  typedef struct packed {
    logic [31:0]                     packet_cnt;
    logic [31:0]                     ecc_err_cnt;
    logic [31:0]                     crc_err_cnt;
    logic [31:0]                     frame_drop_cnt;
    logic [15:0]                     lane_abort_cnt;
    logic [31:0]                     frame_count;
    logic [15:0]                     linebuf_fill;
    logic [63:0]                     timestamp;
    logic [13:0]                     frame_id_buf0;
    logic [13:0]                     frame_id_buf1;
    logic [15:0]                     vc0_line_count;
    logic [15:0]                     vc0_byte_count;
    logic [5:0]                      vc0_data_type;
    logic                            vc0_width_mismatch;
    logic                            vc0_height_mismatch;
    logic [15:0]                     vc1_line_count;
    logic [15:0]                     vc1_byte_count;
    logic [5:0]                      vc1_data_type;
    logic                            vc1_width_mismatch;
    logic                            vc1_height_mismatch;
    logic [15:0]                     emb_word_count_vc0;
    logic [7:0]                      emb_line_count_vc0;
    logic [15:0]                     emb_word_count_vc1;
    logic [7:0]                      emb_line_count_vc1;
    logic [LPARA_NX_SLOTS-1:0][31:0] emb_extract_val_vc0;
    logic [LPARA_NX_SLOTS-1:0][31:0] emb_extract_val_vc1;
    logic [23:0]                     gsp_data;
    logic [31:0]                     error_capture;
    logic                            dma_busy;
    logic                            active_buffer;
    logic [1:0]                      buf0_state;
    logic [1:0]                      buf1_state;
    logic [15:0]                     buf0_last_valid_line;
    logic [15:0]                     buf1_last_valid_line;
    logic [7:0]                      axi_fault_awid;
    logic [1:0]                      axi_fault_resp;
    logic [31:0]                     axi_fault_awaddr;
    logic [15:0]                     bresp_slverr_count;
    logic [15:0]                     bresp_decerr_count;
  } st_sys_type_def;

  // Multi-bit status of the rxbyteclkhs domain carried by one snapshot
  typedef struct packed {
    logic [2:0] lm_fsm_state;
    logic       abort_sync_err;
    logic       abort_fifo_full;
    logic       abort_unarmed;
    logic       deskew_burst_seen;
    logic [3:0] abort_lane_sync_seen;
    logic [1:0] active_lanes_applied;
  } st_byte_type_def;

  // Byte-lane write: a bit takes the write data when its strobe and its writable-field mask are
  // both 1; every other bit keeps its value.
  function automatic logic [31:0] f_wr_bytes(input logic [31:0] old_value,
                                             input logic [31:0] data,
                                             input logic [31:0] bm,
                                             input logic [31:0] wm);
    return (old_value & ~(bm & wm)) | (data & bm & wm);
  endfunction

  // Offset of the register carried by handshake idx.
  function automatic logic [7:0] f_hs_ofs(input logic [31:0] idx);
    logic [7:0] ofs;
    case (idx)
      LPARA_HS_METADATA_DT_CONFIG  : ofs = LPARA_OFS_METADATA_DT_CONFIG;
      LPARA_HS_EXPECTED_RESOLUTION : ofs = LPARA_OFS_EXPECTED_RESOLUTION;
      LPARA_HS_IRQ_ENABLE          : ofs = LPARA_OFS_IRQ_ENABLE;
      LPARA_HS_METADATA_RECORD_ID  : ofs = LPARA_OFS_METADATA_RECORD_ID;
      LPARA_HS_DMA_BUFFER0_ADDR    : ofs = LPARA_OFS_DMA_BUFFER0_ADDR;
      LPARA_HS_DMA_BUFFER1_ADDR    : ofs = LPARA_OFS_DMA_BUFFER1_ADDR;
      LPARA_HS_DMA_BUFFER_SIZE     : ofs = LPARA_OFS_DMA_BUFFER_SIZE;
      LPARA_HS_DMA_ATTR            : ofs = LPARA_OFS_DMA_ATTR;
      LPARA_HS_LINE_PITCH          : ofs = LPARA_OFS_LINE_PITCH;
      LPARA_HS_META_AREA_SIZE      : ofs = LPARA_OFS_META_AREA_SIZE;
      LPARA_HS_TPG_CONTROL         : ofs = LPARA_OFS_TPG_CONTROL;
      LPARA_HS_TPG_RESOLUTION      : ofs = LPARA_OFS_TPG_RESOLUTION;
      LPARA_HS_TPG_BLANKING        : ofs = LPARA_OFS_TPG_BLANKING;
      LPARA_HS_EMB_EXTRACT_CFG0    : ofs = LPARA_OFS_EMB_EXTRACT_CFG0;
      LPARA_HS_EMB_EXTRACT_CFG1    : ofs = LPARA_OFS_EMB_EXTRACT_CFG1;
      default                      : ofs = LPARA_OFS_IP_VERSION;
    endcase
    return ofs;
  endfunction

  // Reset value of the register carried by handshake idx.
  function automatic logic [31:0] f_hs_rst(input logic [31:0] idx);
    logic [31:0] rst;
    case (idx)
      LPARA_HS_METADATA_DT_CONFIG  : rst = LPARA_RST_METADATA_DT_CONFIG;
      LPARA_HS_EXPECTED_RESOLUTION : rst = LPARA_RST_EXPECTED_RESOLUTION;
      LPARA_HS_IRQ_ENABLE          : rst = LPARA_RST_IRQ_ENABLE;
      LPARA_HS_METADATA_RECORD_ID  : rst = LPARA_RST_METADATA_RECORD_ID;
      LPARA_HS_DMA_BUFFER0_ADDR    : rst = LPARA_RST_DMA_BUFFER0_ADDR;
      LPARA_HS_DMA_BUFFER1_ADDR    : rst = LPARA_RST_DMA_BUFFER1_ADDR;
      LPARA_HS_DMA_BUFFER_SIZE     : rst = LPARA_RST_DMA_BUFFER_SIZE;
      LPARA_HS_DMA_ATTR            : rst = LPARA_RST_DMA_ATTR;
      LPARA_HS_LINE_PITCH          : rst = LPARA_RST_LINE_PITCH;
      LPARA_HS_META_AREA_SIZE      : rst = LPARA_RST_META_AREA_SIZE;
      LPARA_HS_TPG_CONTROL         : rst = LPARA_RST_TPG_CONTROL;
      LPARA_HS_TPG_RESOLUTION      : rst = LPARA_RST_TPG_RESOLUTION;
      LPARA_HS_TPG_BLANKING        : rst = LPARA_RST_TPG_BLANKING;
      default                      : rst = LPARA_RST_EMB_EXTRACT_CFG;
    endcase
    return rst;
  endfunction

  // AXI4-Lite Interface and Access FSM
  axil_state_type_def              reg_fsm_axil_current_state;
  axil_state_type_def              w_fsm_axil_nxt_state;
  logic                            w_fsm_axil_bad;
  logic [LPARA_ADDR_WIDTH-1:2]     reg_aw_addr;
  logic [LPARA_ADDR_WIDTH-1:2]     reg_ar_addr;
  logic [LPARA_DATA_WIDTH-1:0]     reg_w_data;
  logic [3:0]                      reg_w_strb;
  logic                            reg_aw_got;
  logic                            reg_w_got;
  logic                            reg_ar_got;
  logic                            w_aw_got_nxt;
  logic                            w_w_got_nxt;
  logic                            w_ar_got_nxt;
  logic                            w_aw_hs;
  logic                            w_w_hs;
  logic                            w_ar_hs;
  logic                            reg_awready;
  logic                            reg_wready;
  logic                            reg_arready;
  logic                            reg_bvalid;
  logic                            reg_rvalid;
  logic                            w_awready_nxt;
  logic                            w_wready_nxt;
  logic                            w_arready_nxt;
  logic                            w_bvalid_nxt;
  logic                            w_rvalid_nxt;
  logic [1:0]                      reg_bresp;
  logic [1:0]                      reg_rresp;
  logic [LPARA_DATA_WIDTH-1:0]     reg_rdata;
  logic                            w_wr_en;
  logic                            w_rd_en;
  logic [LPARA_ADDR_WIDTH-1:2]     w_acc_addr;
  logic [LPARA_WAIT_CNT_W-1:0]     reg_wait_cnt;
  logic [LPARA_DATA_WIDTH-1:0]     w_bm;
  logic [LPARA_IRQ_WIDTH-1:0]      w_clr_mask;
  logic                            w_clr_done;

  // Register File
  logic                            w_map_hit;
  logic [7:0]                      w_ofs;
  logic [1:0]                      w_resp;
  logic                            w_is_irq;
  logic                            w_rd_hold;
  logic                            w_wr_sel;
  logic                            w_rd_done;
  logic [LPARA_DATA_WIDTH-1:0]     w_rd_data;
  logic                            w_wr_full_reset;
  logic                            w_wr_dphy_rst_req;
  logic [1:0]                      w_rel_mask;
  logic [LPARA_IRQ_WIDTH-1:0]      w_irq_clr_mask;
  logic                            w_gsp_pop_req;
  logic                            w_ts_lo_rd;
  logic                            w_reset_in_progress;
  logic [31:0]                     reg_core_config;
  logic [31:0]                     reg_protocol_config;
  logic [31:0]                     reg_phy_config;
  logic [31:0]                     reg_metadata_dt_config;
  logic [31:0]                     reg_stream_config;
  logic [31:0]                     reg_expected_resolution;
  logic [31:0]                     reg_global_irq_enable;
  logic [31:0]                     reg_irq_enable;
  logic [31:0]                     reg_metadata_record_id;
  logic [31:0]                     reg_dma_control;
  logic [31:0]                     reg_dma_buffer0_addr;
  logic [31:0]                     reg_dma_buffer1_addr;
  logic [31:0]                     reg_dma_buffer_size;
  logic [31:0]                     reg_dma_attr;
  logic [31:0]                     reg_line_pitch;
  logic [31:0]                     reg_meta_area_size;
  logic [31:0]                     reg_tpg_control;
  logic [31:0]                     reg_tpg_resolution;
  logic [31:0]                     reg_tpg_blanking;
  logic [31:0]                     reg_emb_extract_cfg [LPARA_NX_SLOTS];
  logic [31:0]                     reg_ts_hi_shadow;
  logic                            reg_gsp_stale;
  logic                            reg_gsp_arm;
  logic [LPARA_RF_SYNC_W-1:0]      reg_rf_rst_sync;
  logic                            reg_rf_rst_n;
  logic                            reg_rf_in_rst;
  logic                            w_rf_arst_n;
  logic                            w_rf_rst_n;

  // Control Synchronizer: handshakes into sys_clk
  logic [31:0]                     w_hs_src        [LPARA_HS_NUM];
  logic                            w_hs_wr         [LPARA_HS_NUM];
  logic                            w_hs_launch     [LPARA_HS_NUM];
  logic                            reg_hs_pend     [LPARA_HS_NUM];
  logic                            reg_hs_req_tgl  [LPARA_HS_NUM];
  logic [PARA_SYNC_STAGES-1:0]     reg_hs_ack_sync [LPARA_HS_NUM];
  logic [31:0]                     reg_hs_hold     [LPARA_HS_NUM];
  logic [PARA_SYNC_STAGES-1:0]     reg_hs_req_sync [LPARA_HS_NUM];
  logic                            reg_hs_ack_tgl  [LPARA_HS_NUM];
  logic [31:0]                     reg_hs_copy     [LPARA_HS_NUM];

  // Control Synchronizer: PROTOCOL_CONFIG.active_lanes into rxbyteclkhs
  logic                            w_hs_byte_wr;
  logic                            w_hs_byte_launch;
  logic                            reg_hs_byte_pend;
  logic                            reg_hs_byte_req_tgl;
  logic [PARA_SYNC_STAGES-1:0]     reg_hs_byte_ack_sync;
  logic [1:0]                      reg_hs_byte_hold;
  logic [PARA_SYNC_STAGES-1:0]     reg_hs_byte_req_sync;
  logic                            reg_hs_byte_ack_tgl;
  logic [1:0]                      reg_active_lanes;

  // Control Synchronizer: command masks into sys_clk
  logic                            w_cmd_irq_launch;
  logic [LPARA_IRQ_WIDTH-1:0]      reg_cmd_irq_pend;
  logic                            reg_cmd_irq_req_tgl;
  logic [PARA_SYNC_STAGES-1:0]     reg_cmd_irq_ack_sync;
  logic [LPARA_IRQ_WIDTH-1:0]      reg_cmd_irq_hold;
  logic [PARA_SYNC_STAGES-1:0]     reg_cmd_irq_req_sync;
  logic                            reg_cmd_irq_ack_tgl;
  logic [LPARA_IRQ_WIDTH-1:0]      reg_irq_clear_strobe;
  logic                            w_cmd_rel_launch;
  logic [1:0]                      reg_cmd_rel_pend;
  logic                            reg_cmd_rel_req_tgl;
  logic [PARA_SYNC_STAGES-1:0]     reg_cmd_rel_ack_sync;
  logic [1:0]                      reg_cmd_rel_hold;
  logic [PARA_SYNC_STAGES-1:0]     reg_cmd_rel_req_sync;
  logic                            reg_cmd_rel_ack_tgl;
  logic [1:0]                      reg_release_buf;
  logic                            w_cmd_pop_launch;
  logic                            w_pop_pend;
  logic                            reg_cmd_pop_pend;
  logic                            reg_cmd_pop_req_tgl;
  logic [PARA_SYNC_STAGES-1:0]     reg_cmd_pop_ack_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_cmd_pop_req_sync;
  logic                            reg_cmd_pop_ack_tgl;
  logic                            reg_gsp_pop;

  // Control Synchronizer: single-bit fields
  logic [PARA_SYNC_STAGES-1:0]     reg_core_enable_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_soft_reset_sys_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_vc_exposure_map_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_axis_enable_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_stream_format_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_stream_follow_dma_drop_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_tpg_enable_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_dma_enable_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_dma_start_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_dma_abort_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_dma_format_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_err_frame_policy_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_global_irq_enable_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_seq_busy_sync;
  logic [PARA_SYNC_STAGES-1:0]     reg_soft_reset_byte_sync;

  // Control Synchronizer: D-PHY enables
  logic                            reg_cl_enable;
  logic [3:0]                      reg_dl_enable;
  logic [3:0]                      reg_forcerxmode;

  // Status Synchronizer: single-bit levels
  logic [PARA_SYNC_STAGES-1:0]       reg_cl_stopstate_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_cl_ulpsactivenot_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_cl_rxclkactivehs_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_cl_ulpsclknot_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_init_done_sync;
  logic [PARA_SYNC_STAGES-1:0][3:0]  reg_stopstate_sync;
  logic [PARA_SYNC_STAGES-1:0][3:0]  reg_rxskewcalhs_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_linebuffer_full_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_sp_fifo_not_empty_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_sp_fifo_full_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_null_blank_seen_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_active_sel_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_switch_pending_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_dt_conflict_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_vc_unsupported_seen_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_linebuf_ovf_sticky_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_sp_fifo_ovf_sticky_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_meta_overflow_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_dma_ovf_sticky_sync;
  logic [PARA_SYNC_STAGES-1:0][23:0] reg_irq_status_sync;
  logic [LPARA_IRQ_WIDTH-1:0]        w_irq_status_l;

  // Status Synchronizer: Start-of-Transmission error flags
  logic [3:0]                        reg_errsoths_tgl;
  logic [3:0]                        reg_errsotsynchs_tgl;
  logic [PARA_SYNC_STAGES-1:0][3:0]  reg_errsoths_sync;
  logic [PARA_SYNC_STAGES-1:0][3:0]  reg_errsotsynchs_sync;
  logic [3:0]                        reg_errsoths_prev;
  logic [3:0]                        reg_errsotsynchs_prev;
  logic [3:0]                        reg_sot_err;
  logic [3:0]                        reg_sot_sync_err;

  // Status Synchronizer: snapshot loops
  st_sys_type_def                    w_st_sys_src;
  st_sys_type_def                    reg_snp_sys_cap;
  st_sys_type_def                    reg_st_l_sys;
  logic                              reg_snp_sys_started;
  logic                              reg_snp_sys_req_tgl;
  logic [PARA_SYNC_STAGES-1:0]       reg_snp_sys_ack_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_snp_sys_req_sync;
  logic                              reg_snp_sys_ack_tgl;
  logic                              w_snp_sys_done;
  logic                              w_snp_sys_launch;
  st_byte_type_def                   w_st_byte_src;
  st_byte_type_def                   reg_snp_byte_cap;
  st_byte_type_def                   reg_st_l_byte;
  logic                              reg_snp_byte_started;
  logic                              reg_snp_byte_req_tgl;
  logic [PARA_SYNC_STAGES-1:0]       reg_snp_byte_ack_sync;
  logic [PARA_SYNC_STAGES-1:0]       reg_snp_byte_req_sync;
  logic                              reg_snp_byte_ack_tgl;
  logic                              w_snp_byte_done;
  logic                              w_snp_byte_launch;

  // PHY Reset Sequencer
  logic [PARA_SYNC_STAGES-1:0]       reg_resetn_sync;
  logic                              reg_resetn_d;
  logic                              w_resetn_q;
  logic                              w_rst_trig;
  logic                              w_rst_cnt_gt1;
  logic [LPARA_RST_CNT_W-1:0]        reg_rst_cnt;
  logic                              reg_phy_core_rst;
  logic                              reg_full_reset_req;

  // ------------------------------------------------------------------------------------------
  // Parameter check
  // ------------------------------------------------------------------------------------------
  generate
    if ((PARA_MAX_LANES != 32'd1) && (PARA_MAX_LANES != 32'd2) && (PARA_MAX_LANES != 32'd4))
    begin : gen_para_check_lane
      $error("m_fpt_csr_axi4lite: unsupported PARA_MAX_LANES");
    end
    if (PARA_N_EXTRACT > 32'd2) begin : gen_para_check_extract
      $error("m_fpt_csr_axi4lite: unsupported PARA_N_EXTRACT");
    end
    if ((PARA_MAX_OUTSTANDING < 32'd1) || (PARA_MAX_OUTSTANDING > 32'd127))
    begin : gen_para_check_dma
      $error("m_fpt_csr_axi4lite: unsupported PARA_MAX_OUTSTANDING");
    end
    if (PARA_SYNC_STAGES < 32'd2) begin : gen_para_check_cdc
      $error("m_fpt_csr_axi4lite: unsupported PARA_SYNC_STAGES");
    end
    if ((PARA_PHY_RST_CYCLES < 32'd1) || (PARA_PHY_RST_CYCLES > 32'd65535) ||
        (PARA_IRQ_CLR_TIMEOUT < 32'd1) || (PARA_IRQ_CLR_TIMEOUT > 32'd65535))
    begin : gen_para_check_count
      $error("m_fpt_csr_axi4lite: unsupported PARA_PHY_RST_CYCLES or PARA_IRQ_CLR_TIMEOUT");
    end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // AXI4-Lite Interface
  // ------------------------------------------------------------------------------------------
  // Handshakes of the current cycle and the accepted-not-served flags: aw_got and w_got clear in
  // ST_WRITE, ar_got in ST_READ. A register access uses the read address only while rd_en is 1.
  always_comb begin : p_com_axil_hs
    w_aw_hs      = i_s_axi_awvalid & reg_awready;
    w_w_hs       = i_s_axi_wvalid & reg_wready;
    w_ar_hs      = i_s_axi_arvalid & reg_arready;
    w_aw_got_nxt = (reg_fsm_axil_current_state == ENUM_ST_WRITE) ? 1'b0 : (w_aw_hs | reg_aw_got);
    w_w_got_nxt  = (reg_fsm_axil_current_state == ENUM_ST_WRITE) ? 1'b0 : (w_w_hs | reg_w_got);
    w_ar_got_nxt = (reg_fsm_axil_current_state == ENUM_ST_READ) ? 1'b0 : (w_ar_hs | reg_ar_got);
    w_wr_en      = (reg_fsm_axil_current_state == ENUM_ST_WRITE);
    w_rd_en      = (reg_fsm_axil_current_state == ENUM_ST_READ);
    w_acc_addr   = w_rd_en ? reg_ar_addr : reg_aw_addr;
  end

  // Address and write data are captured at their handshake and used only after it; they need no
  // reset. Address bits [1:0] are not decoded.
  always_ff @(posedge i_lite_aclk) begin : p_ff_axil_chan
    if (w_aw_hs) begin
      reg_aw_addr <= i_s_axi_awaddr[LPARA_ADDR_WIDTH-1:2];
    end
    if (w_ar_hs) begin
      reg_ar_addr <= i_s_axi_araddr[LPARA_ADDR_WIDTH-1:2];
    end
    if (w_w_hs) begin
      reg_w_data <= i_s_axi_wdata;
      reg_w_strb <= i_s_axi_wstrb;
    end
  end

  // IRQ_STATUS clear: bm replicates each strobe bit over its byte; clr_done when no bit of the
  // written mask still reads 1 in lite_aclk.
  always_comb begin : p_com_irq_clr
    w_bm       = {{8{reg_w_strb[3]}}, {8{reg_w_strb[2]}}, {8{reg_w_strb[1]}}, {8{reg_w_strb[0]}}};
    w_clr_mask = w_bm[LPARA_IRQ_WIDTH-1:0] & reg_w_data[LPARA_IRQ_WIDTH-1:0];
    w_clr_done = ((w_clr_mask & w_irq_status_l) == '0);
  end

  // Access FSM next state. In ST_IDLE a write with both AW and W available is served before a
  // read; the read address stays in ar_got. An unused code returns to ST_IDLE.
  always_comb begin : p_com_axil_fsm
    w_fsm_axil_nxt_state = reg_fsm_axil_current_state;
    w_fsm_axil_bad       = 1'b0;
    case (reg_fsm_axil_current_state)
      ENUM_ST_IDLE: begin
        if ((reg_aw_got | w_aw_hs) & (reg_w_got | w_w_hs)) begin
          w_fsm_axil_nxt_state = ENUM_ST_WRITE;
        end else if (reg_ar_got | w_ar_hs) begin
          w_fsm_axil_nxt_state = ENUM_ST_READ;
        end
      end
      ENUM_ST_WRITE: begin
        if (w_is_irq & (w_clr_mask != '0)) begin
          w_fsm_axil_nxt_state = ENUM_ST_BWAIT;
        end else begin
          w_fsm_axil_nxt_state = ENUM_ST_BRESP;
        end
      end
      ENUM_ST_BWAIT: begin
        if (w_clr_done | (reg_wait_cnt == '0)) begin
          w_fsm_axil_nxt_state = ENUM_ST_BRESP;
        end
      end
      ENUM_ST_BRESP: begin
        if (i_s_axi_bready) begin
          w_fsm_axil_nxt_state = ENUM_ST_IDLE;
        end
      end
      ENUM_ST_READ: begin
        if (!w_rd_hold) begin
          w_fsm_axil_nxt_state = ENUM_ST_RRESP;
        end
      end
      ENUM_ST_RRESP: begin
        if (i_s_axi_rready) begin
          w_fsm_axil_nxt_state = ENUM_ST_IDLE;
        end
      end
      default: begin
        w_fsm_axil_nxt_state = ENUM_ST_IDLE;
        w_fsm_axil_bad       = 1'b1;
      end
    endcase
  end

  // READY and VALID come from their own flops, loaded from the next state: a READY is 1 in
  // ST_IDLE while its channel is not held; bvalid in ST_BRESP, rvalid in ST_RRESP.
  always_comb begin : p_com_axil_rdy
    w_awready_nxt = (w_fsm_axil_nxt_state == ENUM_ST_IDLE) & ~w_aw_got_nxt & ~w_fsm_axil_bad;
    w_wready_nxt  = (w_fsm_axil_nxt_state == ENUM_ST_IDLE) & ~w_w_got_nxt & ~w_fsm_axil_bad;
    w_arready_nxt = (w_fsm_axil_nxt_state == ENUM_ST_IDLE) & ~w_ar_got_nxt & ~w_fsm_axil_bad;
    w_bvalid_nxt  = (w_fsm_axil_nxt_state == ENUM_ST_BRESP);
    w_rvalid_nxt  = (w_fsm_axil_nxt_state == ENUM_ST_RRESP);
  end

  // lite_aresetn abandons the transaction: ST_IDLE, flags clear, READY and VALID low.
  always_ff @(posedge i_lite_aclk or negedge i_lite_aresetn) begin : p_ff_axil_fsm
    if (!i_lite_aresetn) begin
      reg_fsm_axil_current_state <= ENUM_ST_IDLE;
      reg_aw_got                 <= 1'b0;
      reg_w_got                  <= 1'b0;
      reg_ar_got                 <= 1'b0;
      reg_awready                <= 1'b0;
      reg_wready                 <= 1'b0;
      reg_arready                <= 1'b0;
      reg_bvalid                 <= 1'b0;
      reg_rvalid                 <= 1'b0;
    end else begin
      reg_fsm_axil_current_state <= w_fsm_axil_nxt_state;
      reg_aw_got                 <= w_aw_got_nxt;
      reg_w_got                  <= w_w_got_nxt;
      reg_ar_got                 <= w_ar_got_nxt;
      reg_awready                <= w_awready_nxt;
      reg_wready                 <= w_wready_nxt;
      reg_arready                <= w_arready_nxt;
      reg_bvalid                 <= w_bvalid_nxt;
      reg_rvalid                 <= w_rvalid_nxt;
    end
  end

  // bresp loads with wr_en, rresp and rdata with rd_en. wait_cnt loads IRQ_CLR_TIMEOUT with
  // wr_en and counts down to 0 in ST_BWAIT.
  always_ff @(posedge i_lite_aclk or negedge i_lite_aresetn) begin : p_ff_axil_rsp
    if (!i_lite_aresetn) begin
      reg_bresp    <= LPARA_RESP_OKAY;
      reg_rresp    <= LPARA_RESP_OKAY;
      reg_rdata    <= '0;
      reg_wait_cnt <= '0;
    end else begin
      if (w_wr_en) begin
        reg_bresp <= w_resp;
      end
      if (w_rd_en) begin
        reg_rresp <= w_resp;
        reg_rdata <= w_rd_data;
      end
      if (w_wr_en) begin
        reg_wait_cnt <= LPARA_WAIT_CNT_W'(PARA_IRQ_CLR_TIMEOUT);
      end else if ((reg_fsm_axil_current_state == ENUM_ST_BWAIT) && (reg_wait_cnt != '0)) begin
        reg_wait_cnt <= reg_wait_cnt - LPARA_WAIT_CNT_W'(32'd1);
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Register File
  // ------------------------------------------------------------------------------------------
  // rf_rst_n: asserted while resetn is 0 or full_reset_req is 1, released after SYNC_STAGES
  // lite_aclk edges. rf_in_rst is a copy of the last stage, used as status only.
  always_comb begin : p_com_rf_rst
    w_rf_arst_n = i_resetn & ~reg_full_reset_req;
    w_rf_rst_n  = reg_rf_rst_n;
  end

  always_ff @(posedge i_lite_aclk or negedge w_rf_arst_n) begin : p_ff_rf_rst
    if (!w_rf_arst_n) begin
      reg_rf_rst_sync <= '0;
      reg_rf_rst_n    <= 1'b0;
      reg_rf_in_rst   <= 1'b1;
    end else begin
      reg_rf_rst_sync <= $bits(reg_rf_rst_sync)'({reg_rf_rst_sync, 1'b1});
      reg_rf_rst_n    <= reg_rf_rst_sync[LPARA_RF_SYNC_W-1];
      reg_rf_in_rst   <= ~reg_rf_rst_sync[LPARA_RF_SYNC_W-1];
    end
  end

  // Decode of the word offset 0x00 to 0xFF. A DECERR access has no effect. WO bits give one-cycle
  // commands on a write of 1. Read side effects act in the ST_READ cycle that ends the read; a
  // read of GENERIC_SHORT_PACKET returning 0 requests no gsp_pop.
  always_comb begin : p_com_reg_dec
    w_map_hit         = (w_acc_addr[LPARA_ADDR_WIDTH-1:LPARA_MAP_BITS] == '0);
    w_ofs             = {w_acc_addr[LPARA_MAP_BITS-1:2], 2'b00};
    w_resp            = w_map_hit ? LPARA_RESP_OKAY : LPARA_RESP_DECERR;
    w_is_irq          = w_map_hit & (w_ofs == LPARA_OFS_IRQ_STATUS);
    w_rd_hold         = w_map_hit & (w_ofs == LPARA_OFS_GENERIC_SHORT_PACKET) & reg_gsp_stale;
    w_wr_sel          = w_wr_en & w_map_hit;
    w_rd_done         = w_rd_en & w_map_hit & ~w_rd_hold;
    w_wr_full_reset   = w_wr_sel & (w_ofs == LPARA_OFS_CORE_CONFIG) & reg_w_strb[0] &
                        reg_w_data[2];
    w_wr_dphy_rst_req = w_wr_sel & (w_ofs == LPARA_OFS_PHY_CONFIG) & reg_w_strb[0] &
                        reg_w_data[1];
    w_rel_mask        = {2{w_wr_sel & (w_ofs == LPARA_OFS_BUFFER_RELEASE) & reg_w_strb[0]}} &
                        reg_w_data[1:0];
    w_irq_clr_mask    = {LPARA_IRQ_WIDTH{w_wr_en & w_is_irq}} & w_clr_mask;
    w_gsp_pop_req     = w_rd_done & (w_ofs == LPARA_OFS_GENERIC_SHORT_PACKET) &
                        (reg_st_l_sys.gsp_data != '0);
    w_ts_lo_rd        = w_rd_done & (w_ofs == LPARA_OFS_TIMESTAMP_LO);
  end

  // CORE_STATUS.reset_in_progress also covers the cycles in which the Register File is held in
  // reset after the sequencer has finished.
  always_comb begin : p_com_rst_sts
    w_reset_in_progress = reg_phy_core_rst | reg_core_config[1] | reg_rf_in_rst;
  end

  // Configuration registers. Each write updates only the RW fields of the byte lanes whose strobe
  // is 1 (mask literal = RW bits of the register).
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_rf_config
    if (!w_rf_rst_n) begin
      reg_core_config         <= LPARA_RST_CORE_CONFIG;
      reg_protocol_config     <= LPARA_RST_PROTOCOL_CONFIG;
      reg_phy_config          <= LPARA_RST_PHY_CONFIG;
      reg_metadata_dt_config  <= LPARA_RST_METADATA_DT_CONFIG;
      reg_stream_config       <= LPARA_RST_STREAM_CONFIG;
      reg_expected_resolution <= LPARA_RST_EXPECTED_RESOLUTION;
      reg_metadata_record_id  <= LPARA_RST_METADATA_RECORD_ID;
    end else if (w_wr_sel) begin
      case (w_ofs)
        LPARA_OFS_CORE_CONFIG: begin
          reg_core_config <= f_wr_bytes(reg_core_config, reg_w_data, w_bm, 32'h0000_0003);
        end
        LPARA_OFS_PROTOCOL_CONFIG: begin
          reg_protocol_config <= f_wr_bytes(reg_protocol_config, reg_w_data, w_bm, 32'h0000_1003);
        end
        LPARA_OFS_PHY_CONFIG: begin
          reg_phy_config <= f_wr_bytes(reg_phy_config, reg_w_data, w_bm, 32'h0000_07FD);
        end
        LPARA_OFS_METADATA_DT_CONFIG: begin
          reg_metadata_dt_config <= f_wr_bytes(reg_metadata_dt_config, reg_w_data, w_bm,
                                               32'h0000_073F);
        end
        LPARA_OFS_STREAM_CONFIG: begin
          reg_stream_config <= f_wr_bytes(reg_stream_config, reg_w_data, w_bm, 32'h0000_0007);
        end
        LPARA_OFS_EXPECTED_RESOLUTION: begin
          reg_expected_resolution <= f_wr_bytes(reg_expected_resolution, reg_w_data, w_bm,
                                                32'hFFFF_FFFF);
        end
        LPARA_OFS_METADATA_RECORD_ID: begin
          reg_metadata_record_id <= f_wr_bytes(reg_metadata_record_id, reg_w_data, w_bm,
                                               32'hFFFF_FFFF);
        end
        default: begin
        end
      endcase
    end
  end

  // Interrupt enables.
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_rf_irq
    if (!w_rf_rst_n) begin
      reg_global_irq_enable <= LPARA_RST_GLOBAL_IRQ_ENABLE;
      reg_irq_enable        <= LPARA_RST_IRQ_ENABLE;
    end else if (w_wr_sel) begin
      case (w_ofs)
        LPARA_OFS_GLOBAL_IRQ_ENABLE: begin
          reg_global_irq_enable <= f_wr_bytes(reg_global_irq_enable, reg_w_data, w_bm,
                                              32'h0000_0001);
        end
        LPARA_OFS_IRQ_ENABLE: begin
          reg_irq_enable <= f_wr_bytes(reg_irq_enable, reg_w_data, w_bm, 32'h00FF_FFFF);
        end
        default: begin
        end
      endcase
    end
  end

  // DMA configuration.
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_rf_dma
    if (!w_rf_rst_n) begin
      reg_dma_control      <= LPARA_RST_DMA_CONTROL;
      reg_dma_buffer0_addr <= LPARA_RST_DMA_BUFFER0_ADDR;
      reg_dma_buffer1_addr <= LPARA_RST_DMA_BUFFER1_ADDR;
      reg_dma_buffer_size  <= LPARA_RST_DMA_BUFFER_SIZE;
      reg_dma_attr         <= LPARA_RST_DMA_ATTR;
      reg_line_pitch       <= LPARA_RST_LINE_PITCH;
      reg_meta_area_size   <= LPARA_RST_META_AREA_SIZE;
    end else if (w_wr_sel) begin
      case (w_ofs)
        LPARA_OFS_DMA_CONTROL: begin
          reg_dma_control <= f_wr_bytes(reg_dma_control, reg_w_data, w_bm, 32'h0000_001F);
        end
        LPARA_OFS_DMA_BUFFER0_ADDR: begin
          reg_dma_buffer0_addr <= f_wr_bytes(reg_dma_buffer0_addr, reg_w_data, w_bm,
                                             32'hFFFF_FFFF);
        end
        LPARA_OFS_DMA_BUFFER1_ADDR: begin
          reg_dma_buffer1_addr <= f_wr_bytes(reg_dma_buffer1_addr, reg_w_data, w_bm,
                                             32'hFFFF_FFFF);
        end
        LPARA_OFS_DMA_BUFFER_SIZE: begin
          reg_dma_buffer_size <= f_wr_bytes(reg_dma_buffer_size, reg_w_data, w_bm,
                                            32'hFFFF_FFFF);
        end
        LPARA_OFS_DMA_ATTR: begin
          reg_dma_attr <= f_wr_bytes(reg_dma_attr, reg_w_data, w_bm, 32'h0000_7FFF);
        end
        LPARA_OFS_LINE_PITCH: begin
          reg_line_pitch <= f_wr_bytes(reg_line_pitch, reg_w_data, w_bm, 32'h00FF_FFFF);
        end
        LPARA_OFS_META_AREA_SIZE: begin
          reg_meta_area_size <= f_wr_bytes(reg_meta_area_size, reg_w_data, w_bm, 32'hFFFF_FFFF);
        end
        default: begin
        end
      endcase
    end
  end

  // Test Pattern Generator configuration.
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_rf_tpg
    if (!w_rf_rst_n) begin
      reg_tpg_control    <= LPARA_RST_TPG_CONTROL;
      reg_tpg_resolution <= LPARA_RST_TPG_RESOLUTION;
      reg_tpg_blanking   <= LPARA_RST_TPG_BLANKING;
    end else if (w_wr_sel) begin
      case (w_ofs)
        LPARA_OFS_TPG_CONTROL: begin
          reg_tpg_control <= f_wr_bytes(reg_tpg_control, reg_w_data, w_bm, 32'h0000_003F);
        end
        LPARA_OFS_TPG_RESOLUTION: begin
          reg_tpg_resolution <= f_wr_bytes(reg_tpg_resolution, reg_w_data, w_bm, 32'hFFFF_FFFF);
        end
        LPARA_OFS_TPG_BLANKING: begin
          reg_tpg_blanking <= f_wr_bytes(reg_tpg_blanking, reg_w_data, w_bm, 32'hFFFF_FFFF);
        end
        default: begin
        end
      endcase
    end
  end

  // EMB_EXTRACT_CFGm, one per slot; a slot at or above N_EXTRACT is reserved and never written.
  // Bits [7:3] are reserved.
  generate
    for (genvar gen_idx = 0; gen_idx < LPARA_NX_SLOTS; gen_idx++) begin : gen_emb_cfg
      localparam logic [7:0] LPARA_SLOT_OFS =
        (gen_idx == 0) ? LPARA_OFS_EMB_EXTRACT_CFG0 : LPARA_OFS_EMB_EXTRACT_CFG1;
      localparam logic       LPARA_SLOT_ON  = (32'(gen_idx) < PARA_N_EXTRACT);

      always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_rf_extract
        if (!w_rf_rst_n) begin
          reg_emb_extract_cfg[gen_idx] <= LPARA_RST_EMB_EXTRACT_CFG;
        end else if (LPARA_SLOT_ON && w_wr_sel && (w_ofs == LPARA_SLOT_OFS)) begin
          reg_emb_extract_cfg[gen_idx] <= f_wr_bytes(reg_emb_extract_cfg[gen_idx], reg_w_data,
                                                     w_bm, 32'hFFFF_FF07);
        end
      end

      // Source of the handshake of a slot that is built.
      if (LPARA_SLOT_ON) begin : gen_hs_src
        always_comb begin : p_com_hs_src_extract
          w_hs_src[LPARA_HS_EMB_EXTRACT_CFG0 + 32'(gen_idx)] = reg_emb_extract_cfg[gen_idx];
        end
      end
    end
  endgenerate

  // Read data. Reserved offsets, reserved bits, WO registers, slots not built and DECERR read 0.
  // RO fields come from lite_aclk copies of the status or from parameters.
  always_comb begin : p_com_rd_mux
    w_rd_data = '0;
    if (w_map_hit) begin
      case (w_ofs)
        LPARA_OFS_IP_VERSION: begin
          w_rd_data = LPARA_IP_VERSION;
        end
        LPARA_OFS_CORE_CONFIG: begin
          w_rd_data = reg_core_config;
        end
        LPARA_OFS_PROTOCOL_CONFIG: begin
          w_rd_data = reg_protocol_config |
                      {20'd0, reg_st_l_byte.active_lanes_applied, LPARA_MAX_LANES_CODE, 8'd0};
        end
        LPARA_OFS_PHY_CONFIG: begin
          w_rd_data = reg_phy_config;
        end
        LPARA_OFS_METADATA_DT_CONFIG: begin
          w_rd_data = reg_metadata_dt_config;
        end
        LPARA_OFS_STREAM_CONFIG: begin
          w_rd_data = reg_stream_config;
        end
        LPARA_OFS_EXPECTED_RESOLUTION: begin
          w_rd_data = reg_expected_resolution;
        end
        LPARA_OFS_CORE_STATUS: begin
          w_rd_data = {19'd0, reg_vc_unsupported_seen_sync[PARA_SYNC_STAGES-1], 4'd0,
                       reg_dt_conflict_sync[PARA_SYNC_STAGES-1],
                       reg_switch_pending_sync[PARA_SYNC_STAGES-1],
                       reg_active_sel_sync[PARA_SYNC_STAGES-1],
                       reg_null_blank_seen_sync[PARA_SYNC_STAGES-1],
                       reg_sp_fifo_full_sync[PARA_SYNC_STAGES-1],
                       reg_sp_fifo_not_empty_sync[PARA_SYNC_STAGES-1],
                       reg_linebuffer_full_sync[PARA_SYNC_STAGES-1],
                       w_reset_in_progress};
        end
        LPARA_OFS_PHY_STATUS: begin
          w_rd_data = {27'd0, reg_init_done_sync[PARA_SYNC_STAGES-1],
                       ~reg_cl_ulpsclknot_sync[PARA_SYNC_STAGES-1],
                       reg_cl_rxclkactivehs_sync[PARA_SYNC_STAGES-1],
                       ~reg_cl_ulpsactivenot_sync[PARA_SYNC_STAGES-1],
                       reg_cl_stopstate_sync[PARA_SYNC_STAGES-1]};
        end
        LPARA_OFS_LANE_STATUS: begin
          for (int lane = 0; lane < 4; lane++) begin
            w_rd_data[lane*8 +: 4] = {reg_rxskewcalhs_sync[PARA_SYNC_STAGES-1][lane],
                                      reg_sot_sync_err[lane], reg_sot_err[lane],
                                      reg_stopstate_sync[PARA_SYNC_STAGES-1][lane]} &
                                     {4{LPARA_LANE_MASK[lane]}};
          end
        end
        LPARA_OFS_PACKET_COUNT: begin
          w_rd_data = reg_st_l_sys.packet_cnt;
        end
        LPARA_OFS_FRAME_COUNT: begin
          w_rd_data = reg_st_l_sys.frame_count;
        end
        LPARA_OFS_BUFFER_STATUS: begin
          w_rd_data = {12'd0, reg_dma_ovf_sticky_sync[PARA_SYNC_STAGES-1],
                       reg_meta_overflow_sync[PARA_SYNC_STAGES-1],
                       reg_sp_fifo_ovf_sticky_sync[PARA_SYNC_STAGES-1],
                       reg_linebuf_ovf_sticky_sync[PARA_SYNC_STAGES-1],
                       reg_st_l_sys.linebuf_fill};
        end
        LPARA_OFS_LANE_ALIGN_STATUS: begin
          w_rd_data = {20'd0, reg_st_l_byte.abort_lane_sync_seen, 1'b0,
                       reg_st_l_byte.deskew_burst_seen, reg_st_l_byte.abort_unarmed,
                       reg_st_l_byte.abort_fifo_full, reg_st_l_byte.abort_sync_err,
                       reg_st_l_byte.lm_fsm_state};
        end
        LPARA_OFS_GLOBAL_IRQ_ENABLE: begin
          w_rd_data = reg_global_irq_enable;
        end
        LPARA_OFS_IRQ_STATUS: begin
          w_rd_data = {8'd0, w_irq_status_l};
        end
        LPARA_OFS_IRQ_ENABLE: begin
          w_rd_data = reg_irq_enable;
        end
        LPARA_OFS_ECC_ERROR_COUNT: begin
          w_rd_data = reg_st_l_sys.ecc_err_cnt;
        end
        LPARA_OFS_CRC_ERROR_COUNT: begin
          w_rd_data = reg_st_l_sys.crc_err_cnt;
        end
        LPARA_OFS_FRAME_DROP_COUNT: begin
          w_rd_data = reg_st_l_sys.frame_drop_cnt;
        end
        LPARA_OFS_LANE_ABORT_COUNT: begin
          w_rd_data = {16'd0, reg_st_l_sys.lane_abort_cnt};
        end
        LPARA_OFS_VC0_IMAGE_INFO1: begin
          w_rd_data = {reg_st_l_sys.vc0_line_count, reg_st_l_sys.vc0_byte_count};
        end
        LPARA_OFS_VC0_IMAGE_INFO2: begin
          w_rd_data = {24'd0, reg_st_l_sys.vc0_height_mismatch, reg_st_l_sys.vc0_width_mismatch,
                       reg_st_l_sys.vc0_data_type};
        end
        LPARA_OFS_VC1_IMAGE_INFO1: begin
          w_rd_data = {reg_st_l_sys.vc1_line_count, reg_st_l_sys.vc1_byte_count};
        end
        LPARA_OFS_VC1_IMAGE_INFO2: begin
          w_rd_data = {24'd0, reg_st_l_sys.vc1_height_mismatch, reg_st_l_sys.vc1_width_mismatch,
                       reg_st_l_sys.vc1_data_type};
        end
        LPARA_OFS_GENERIC_SHORT_PACKET: begin
          w_rd_data = {8'd0, reg_st_l_sys.gsp_data};
        end
        LPARA_OFS_EMBEDDED_DATA_INFO_VC0: begin
          w_rd_data = {8'd0, reg_st_l_sys.emb_line_count_vc0, reg_st_l_sys.emb_word_count_vc0};
        end
        LPARA_OFS_EMBEDDED_DATA_INFO_VC1: begin
          w_rd_data = {8'd0, reg_st_l_sys.emb_line_count_vc1, reg_st_l_sys.emb_word_count_vc1};
        end
        LPARA_OFS_METADATA_RECORD_ID: begin
          w_rd_data = reg_metadata_record_id;
        end
        LPARA_OFS_FRAME_ID: begin
          w_rd_data = {2'd0, reg_st_l_sys.frame_id_buf1, 2'd0, reg_st_l_sys.frame_id_buf0};
        end
        LPARA_OFS_TIMESTAMP_LO: begin
          w_rd_data = reg_st_l_sys.timestamp[31:0];
        end
        LPARA_OFS_TIMESTAMP_HI: begin
          w_rd_data = reg_ts_hi_shadow;
        end
        LPARA_OFS_DMA_CONTROL: begin
          w_rd_data = reg_dma_control;
        end
        LPARA_OFS_DMA_BUFFER0_ADDR: begin
          w_rd_data = reg_dma_buffer0_addr;
        end
        LPARA_OFS_DMA_BUFFER1_ADDR: begin
          w_rd_data = reg_dma_buffer1_addr;
        end
        LPARA_OFS_DMA_BUFFER_SIZE: begin
          w_rd_data = reg_dma_buffer_size;
        end
        LPARA_OFS_DMA_ATTR: begin
          w_rd_data = reg_dma_attr | {9'd0, 7'(PARA_MAX_OUTSTANDING), 16'd0};
        end
        LPARA_OFS_DMA_STATUS: begin
          w_rd_data = {30'd0, reg_st_l_sys.active_buffer, reg_st_l_sys.dma_busy};
        end
        LPARA_OFS_LINE_PITCH: begin
          w_rd_data = reg_line_pitch;
        end
        LPARA_OFS_BUFFER_OWNERSHIP: begin
          w_rd_data = {28'd0, reg_st_l_sys.buf1_state, reg_st_l_sys.buf0_state};
        end
        LPARA_OFS_BUFFER_ERROR_INFO: begin
          w_rd_data = {reg_st_l_sys.buf1_last_valid_line, reg_st_l_sys.buf0_last_valid_line};
        end
        LPARA_OFS_META_AREA_SIZE: begin
          w_rd_data = reg_meta_area_size;
        end
        LPARA_OFS_TPG_CONTROL: begin
          w_rd_data = reg_tpg_control;
        end
        LPARA_OFS_ERROR_CAPTURE: begin
          w_rd_data = reg_st_l_sys.error_capture;
        end
        LPARA_OFS_AXI_FAULT_TRACE: begin
          w_rd_data = {22'd0, reg_st_l_sys.axi_fault_resp, reg_st_l_sys.axi_fault_awid};
        end
        LPARA_OFS_AXI_FAULT_ADDR: begin
          w_rd_data = reg_st_l_sys.axi_fault_awaddr;
        end
        LPARA_OFS_AXI_ERROR_COUNT: begin
          w_rd_data = {reg_st_l_sys.bresp_decerr_count, reg_st_l_sys.bresp_slverr_count};
        end
        LPARA_OFS_TPG_RESOLUTION: begin
          w_rd_data = reg_tpg_resolution;
        end
        LPARA_OFS_TPG_BLANKING: begin
          w_rd_data = reg_tpg_blanking;
        end
        LPARA_OFS_EMB_EXTRACT_CFG0: begin
          if (PARA_N_EXTRACT >= 32'd1) begin
            w_rd_data = reg_emb_extract_cfg[0];
          end
        end
        LPARA_OFS_EMB_EXTRACT_CFG1: begin
          if (PARA_N_EXTRACT >= 32'd2) begin
            w_rd_data = reg_emb_extract_cfg[LPARA_NX_SLOTS-1];
          end
        end
        LPARA_OFS_EMB_EXTRACT_VC0_VAL0: begin
          if (PARA_N_EXTRACT >= 32'd1) begin
            w_rd_data = reg_st_l_sys.emb_extract_val_vc0[0];
          end
        end
        LPARA_OFS_EMB_EXTRACT_VC0_VAL1: begin
          if (PARA_N_EXTRACT >= 32'd2) begin
            w_rd_data = reg_st_l_sys.emb_extract_val_vc0[LPARA_NX_SLOTS-1];
          end
        end
        LPARA_OFS_EMB_EXTRACT_VC1_VAL0: begin
          if (PARA_N_EXTRACT >= 32'd1) begin
            w_rd_data = reg_st_l_sys.emb_extract_val_vc1[0];
          end
        end
        LPARA_OFS_EMB_EXTRACT_VC1_VAL1: begin
          if (PARA_N_EXTRACT >= 32'd2) begin
            w_rd_data = reg_st_l_sys.emb_extract_val_vc1[LPARA_NX_SLOTS-1];
          end
        end
        default: begin
          w_rd_data = '0;
        end
      endcase
    end
  end

  // Read side effects. A TIMESTAMP_LO read copies bits [63:32] of the same snapshot into the
  // TIMESTAMP_HI shadow. gsp_stale is set by a gsp_pop request and cleared when the first sys_clk
  // snapshot launched after pop_pend returned to 0 arrives (gsp_arm marks that snapshot).
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_rd_side
    if (!w_rf_rst_n) begin
      reg_ts_hi_shadow <= '0;
      reg_gsp_stale    <= 1'b0;
      reg_gsp_arm      <= 1'b0;
    end else begin
      if (w_ts_lo_rd) begin
        reg_ts_hi_shadow <= reg_st_l_sys.timestamp[63:32];
      end
      if (w_gsp_pop_req) begin
        reg_gsp_stale <= 1'b1;
        reg_gsp_arm   <= 1'b0;
      end else begin
        if (w_snp_sys_done && reg_gsp_arm) begin
          reg_gsp_stale <= 1'b0;
        end
        if (w_snp_sys_launch) begin
          reg_gsp_arm <= ~w_pop_pend;
        end
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Control Synchronizer
  // ------------------------------------------------------------------------------------------
  // Source value of each sys_clk handshake: the whole register in lite_aclk.
  always_comb begin : p_com_hs_src
    w_hs_src[LPARA_HS_METADATA_DT_CONFIG]  = reg_metadata_dt_config;
    w_hs_src[LPARA_HS_EXPECTED_RESOLUTION] = reg_expected_resolution;
    w_hs_src[LPARA_HS_IRQ_ENABLE]          = reg_irq_enable;
    w_hs_src[LPARA_HS_METADATA_RECORD_ID]  = reg_metadata_record_id;
    w_hs_src[LPARA_HS_DMA_BUFFER0_ADDR]    = reg_dma_buffer0_addr;
    w_hs_src[LPARA_HS_DMA_BUFFER1_ADDR]    = reg_dma_buffer1_addr;
    w_hs_src[LPARA_HS_DMA_BUFFER_SIZE]     = reg_dma_buffer_size;
    w_hs_src[LPARA_HS_DMA_ATTR]            = reg_dma_attr;
    w_hs_src[LPARA_HS_LINE_PITCH]          = reg_line_pitch;
    w_hs_src[LPARA_HS_META_AREA_SIZE]      = reg_meta_area_size;
    w_hs_src[LPARA_HS_TPG_CONTROL]         = reg_tpg_control;
    w_hs_src[LPARA_HS_TPG_RESOLUTION]      = reg_tpg_resolution;
    w_hs_src[LPARA_HS_TPG_BLANKING]        = reg_tpg_blanking;
  end

  // One request/acknowledge handshake per register (toggle protocol). A write sets pend; when no
  // transfer is in flight the register is copied to hold and the request toggles. A write during
  // a transfer is sent after the acknowledge, so the destination ends with the last value. hold
  // is read in sys_clk only after the synchronized request toggles, so it needs no reset.
  generate
    for (genvar gen_idx = 0; gen_idx < LPARA_HS_NUM; gen_idx++) begin : gen_hs
      always_comb begin : p_com_hs_ctl
        w_hs_wr[gen_idx]     = w_wr_sel & (w_ofs == f_hs_ofs(32'(gen_idx)));
        w_hs_launch[gen_idx] = reg_hs_pend[gen_idx] &
                               ~(reg_hs_req_tgl[gen_idx] ^
                                 reg_hs_ack_sync[gen_idx][PARA_SYNC_STAGES-1]);
      end

      always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_hs_lite
        if (!w_rf_rst_n) begin
          reg_hs_pend[gen_idx]     <= 1'b0;
          reg_hs_req_tgl[gen_idx]  <= 1'b0;
          reg_hs_ack_sync[gen_idx] <= '0;
        end else begin
          reg_hs_pend[gen_idx]     <= w_hs_wr[gen_idx] |
                                      (reg_hs_pend[gen_idx] & ~w_hs_launch[gen_idx]);
          reg_hs_ack_sync[gen_idx] <= PARA_SYNC_STAGES'({reg_hs_ack_sync[gen_idx],
                                                         reg_hs_ack_tgl[gen_idx]});
          if (w_hs_launch[gen_idx]) begin
            reg_hs_req_tgl[gen_idx] <= ~reg_hs_req_tgl[gen_idx];
          end
        end
      end

      always_ff @(posedge i_lite_aclk) begin : p_ff_hs_hold
        if (w_hs_launch[gen_idx]) begin
          reg_hs_hold[gen_idx] <= w_hs_src[gen_idx];
        end
      end

      // Destination copy, updated in one sys_clk cycle; reset to the reset value of the register.
      always_ff @(posedge i_sys_clk or negedge i_sys_resetn) begin : p_ff_hs_sys
        if (!i_sys_resetn) begin
          reg_hs_req_sync[gen_idx] <= '0;
          reg_hs_ack_tgl[gen_idx]  <= 1'b0;
          reg_hs_copy[gen_idx]     <= f_hs_rst(32'(gen_idx));
        end else begin
          reg_hs_req_sync[gen_idx] <= PARA_SYNC_STAGES'({reg_hs_req_sync[gen_idx],
                                                         reg_hs_req_tgl[gen_idx]});
          reg_hs_ack_tgl[gen_idx]  <= reg_hs_req_sync[gen_idx][PARA_SYNC_STAGES-1];
          if (reg_hs_req_sync[gen_idx][PARA_SYNC_STAGES-1] != reg_hs_ack_tgl[gen_idx]) begin
            reg_hs_copy[gen_idx] <= reg_hs_hold[gen_idx];
          end
        end
      end
    end
  endgenerate

  // PROTOCOL_CONFIG.active_lanes into rxbyteclkhs, same handshake as above.
  always_comb begin : p_com_hs_byte_ctl
    w_hs_byte_wr     = w_wr_sel & (w_ofs == LPARA_OFS_PROTOCOL_CONFIG);
    w_hs_byte_launch = reg_hs_byte_pend &
                       ~(reg_hs_byte_req_tgl ^ reg_hs_byte_ack_sync[PARA_SYNC_STAGES-1]);
  end

  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_hs_byte_lite
    if (!w_rf_rst_n) begin
      reg_hs_byte_pend     <= 1'b0;
      reg_hs_byte_req_tgl  <= 1'b0;
      reg_hs_byte_ack_sync <= '0;
    end else begin
      reg_hs_byte_pend     <= w_hs_byte_wr | (reg_hs_byte_pend & ~w_hs_byte_launch);
      reg_hs_byte_ack_sync <= $bits(reg_hs_byte_ack_sync)'({reg_hs_byte_ack_sync,
                                                             reg_hs_byte_ack_tgl});
      if (w_hs_byte_launch) begin
        reg_hs_byte_req_tgl <= ~reg_hs_byte_req_tgl;
      end
    end
  end

  always_ff @(posedge i_lite_aclk) begin : p_ff_hs_byte_hold
    if (w_hs_byte_launch) begin
      reg_hs_byte_hold <= reg_protocol_config[1:0];
    end
  end

  always_ff @(posedge i_rxbyteclkhs or negedge i_rxbyteclkhs_resetn) begin : p_ff_hs_byte_dst
    if (!i_rxbyteclkhs_resetn) begin
      reg_hs_byte_req_sync <= '0;
      reg_hs_byte_ack_tgl  <= 1'b0;
      reg_active_lanes     <= LPARA_RST_PROTOCOL_CONFIG[1:0];
    end else begin
      reg_hs_byte_req_sync <= $bits(reg_hs_byte_req_sync)'({reg_hs_byte_req_sync,
                                                             reg_hs_byte_req_tgl});
      reg_hs_byte_ack_tgl  <= reg_hs_byte_req_sync[PARA_SYNC_STAGES-1];
      if (reg_hs_byte_req_sync[PARA_SYNC_STAGES-1] != reg_hs_byte_ack_tgl) begin
        reg_active_lanes <= reg_hs_byte_hold;
      end
    end
  end

  // Command masks of IRQ_STATUS, BUFFER_RELEASE and the gsp_pop request. A mask written while the
  // previous one is crossing is OR-merged into pend and sent after the acknowledge.
  always_comb begin : p_com_cmd_ctl
    w_cmd_irq_launch = (reg_cmd_irq_pend != '0) &
                       ~(reg_cmd_irq_req_tgl ^ reg_cmd_irq_ack_sync[PARA_SYNC_STAGES-1]);
    w_cmd_rel_launch = (reg_cmd_rel_pend != '0) &
                       ~(reg_cmd_rel_req_tgl ^ reg_cmd_rel_ack_sync[PARA_SYNC_STAGES-1]);
    w_cmd_pop_launch = reg_cmd_pop_pend &
                       ~(reg_cmd_pop_req_tgl ^ reg_cmd_pop_ack_sync[PARA_SYNC_STAGES-1]);
    w_pop_pend       = reg_cmd_pop_pend |
                       (reg_cmd_pop_req_tgl ^ reg_cmd_pop_ack_sync[PARA_SYNC_STAGES-1]);
  end

  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_cmd_lite
    if (!w_rf_rst_n) begin
      reg_cmd_irq_pend     <= '0;
      reg_cmd_irq_req_tgl  <= 1'b0;
      reg_cmd_irq_ack_sync <= '0;
      reg_cmd_rel_pend     <= '0;
      reg_cmd_rel_req_tgl  <= 1'b0;
      reg_cmd_rel_ack_sync <= '0;
      reg_cmd_pop_pend     <= 1'b0;
      reg_cmd_pop_req_tgl  <= 1'b0;
      reg_cmd_pop_ack_sync <= '0;
    end else begin
      reg_cmd_irq_pend     <= (w_cmd_irq_launch ? '0 : reg_cmd_irq_pend) | w_irq_clr_mask;
      reg_cmd_irq_ack_sync <= $bits(reg_cmd_irq_ack_sync)'({reg_cmd_irq_ack_sync,
                                                             reg_cmd_irq_ack_tgl});
      if (w_cmd_irq_launch) begin
        reg_cmd_irq_req_tgl <= ~reg_cmd_irq_req_tgl;
      end
      reg_cmd_rel_pend     <= (w_cmd_rel_launch ? '0 : reg_cmd_rel_pend) | w_rel_mask;
      reg_cmd_rel_ack_sync <= $bits(reg_cmd_rel_ack_sync)'({reg_cmd_rel_ack_sync,
                                                             reg_cmd_rel_ack_tgl});
      if (w_cmd_rel_launch) begin
        reg_cmd_rel_req_tgl <= ~reg_cmd_rel_req_tgl;
      end
      reg_cmd_pop_pend     <= (reg_cmd_pop_pend & ~w_cmd_pop_launch) | w_gsp_pop_req;
      reg_cmd_pop_ack_sync <= $bits(reg_cmd_pop_ack_sync)'({reg_cmd_pop_ack_sync,
                                                             reg_cmd_pop_ack_tgl});
      if (w_cmd_pop_launch) begin
        reg_cmd_pop_req_tgl <= ~reg_cmd_pop_req_tgl;
      end
    end
  end

  // Mask hold registers, read in sys_clk only after the synchronized request toggles.
  always_ff @(posedge i_lite_aclk) begin : p_ff_cmd_hold
    if (w_cmd_irq_launch) begin
      reg_cmd_irq_hold <= reg_cmd_irq_pend;
    end
    if (w_cmd_rel_launch) begin
      reg_cmd_rel_hold <= reg_cmd_rel_pend;
    end
  end

  // Each received mask gives one sys_clk cycle of strobes, all bits in the same cycle.
  always_ff @(posedge i_sys_clk or negedge i_sys_resetn) begin : p_ff_cmd_sys
    if (!i_sys_resetn) begin
      reg_cmd_irq_req_sync <= '0;
      reg_cmd_irq_ack_tgl  <= 1'b0;
      reg_irq_clear_strobe <= '0;
      reg_cmd_rel_req_sync <= '0;
      reg_cmd_rel_ack_tgl  <= 1'b0;
      reg_release_buf      <= '0;
      reg_cmd_pop_req_sync <= '0;
      reg_cmd_pop_ack_tgl  <= 1'b0;
      reg_gsp_pop          <= 1'b0;
    end else begin
      reg_cmd_irq_req_sync <= $bits(reg_cmd_irq_req_sync)'({reg_cmd_irq_req_sync,
                                                             reg_cmd_irq_req_tgl});
      reg_cmd_irq_ack_tgl  <= reg_cmd_irq_req_sync[PARA_SYNC_STAGES-1];
      reg_irq_clear_strobe <= (reg_cmd_irq_req_sync[PARA_SYNC_STAGES-1] != reg_cmd_irq_ack_tgl) ?
                              reg_cmd_irq_hold : '0;
      reg_cmd_rel_req_sync <= $bits(reg_cmd_rel_req_sync)'({reg_cmd_rel_req_sync,
                                                             reg_cmd_rel_req_tgl});
      reg_cmd_rel_ack_tgl  <= reg_cmd_rel_req_sync[PARA_SYNC_STAGES-1];
      reg_release_buf      <= (reg_cmd_rel_req_sync[PARA_SYNC_STAGES-1] != reg_cmd_rel_ack_tgl) ?
                              reg_cmd_rel_hold : '0;
      reg_cmd_pop_req_sync <= $bits(reg_cmd_pop_req_sync)'({reg_cmd_pop_req_sync,
                                                             reg_cmd_pop_req_tgl});
      reg_cmd_pop_ack_tgl  <= reg_cmd_pop_req_sync[PARA_SYNC_STAGES-1];
      reg_gsp_pop          <= (reg_cmd_pop_req_sync[PARA_SYNC_STAGES-1] != reg_cmd_pop_ack_tgl);
    end
  end

  // Single-bit control fields into sys_clk through SYNC_STAGES flops, each reset to the reset
  // value of its field. tpg_enable has one synchronizer shared by its consumers.
  always_ff @(posedge i_sys_clk or negedge i_sys_resetn) begin : p_ff_cfg_sys_sync
    if (!i_sys_resetn) begin
      reg_core_enable_sync            <= '1;
      reg_soft_reset_sys_sync         <= '0;
      reg_vc_exposure_map_sync        <= '0;
      reg_axis_enable_sync            <= '1;
      reg_stream_format_sync          <= '0;
      reg_stream_follow_dma_drop_sync <= '0;
      reg_tpg_enable_sync             <= '0;
      reg_dma_enable_sync             <= '0;
      reg_dma_start_sync              <= '0;
      reg_dma_abort_sync              <= '0;
      reg_dma_format_sync             <= '0;
      reg_err_frame_policy_sync       <= '1;
      reg_global_irq_enable_sync      <= '0;
    end else begin
      reg_core_enable_sync            <= PARA_SYNC_STAGES'({reg_core_enable_sync,
                                                            reg_core_config[0]});
      reg_soft_reset_sys_sync         <= PARA_SYNC_STAGES'({reg_soft_reset_sys_sync,
                                                            reg_core_config[1]});
      reg_vc_exposure_map_sync        <= PARA_SYNC_STAGES'({reg_vc_exposure_map_sync,
                                                            reg_protocol_config[12]});
      reg_axis_enable_sync            <= PARA_SYNC_STAGES'({reg_axis_enable_sync,
                                                            reg_stream_config[0]});
      reg_stream_format_sync          <= PARA_SYNC_STAGES'({reg_stream_format_sync,
                                                            reg_stream_config[1]});
      reg_stream_follow_dma_drop_sync <= PARA_SYNC_STAGES'({reg_stream_follow_dma_drop_sync,
                                                            reg_stream_config[2]});
      reg_tpg_enable_sync             <= PARA_SYNC_STAGES'({reg_tpg_enable_sync,
                                                            reg_tpg_control[0]});
      reg_dma_enable_sync             <= PARA_SYNC_STAGES'({reg_dma_enable_sync,
                                                            reg_dma_control[0]});
      reg_dma_start_sync              <= PARA_SYNC_STAGES'({reg_dma_start_sync,
                                                            reg_dma_control[1]});
      reg_dma_abort_sync              <= PARA_SYNC_STAGES'({reg_dma_abort_sync,
                                                            reg_dma_control[2]});
      reg_dma_format_sync             <= PARA_SYNC_STAGES'({reg_dma_format_sync,
                                                            reg_dma_control[3]});
      reg_err_frame_policy_sync       <= PARA_SYNC_STAGES'({reg_err_frame_policy_sync,
                                                            reg_dma_control[4]});
      reg_global_irq_enable_sync      <= PARA_SYNC_STAGES'({reg_global_irq_enable_sync,
                                                            reg_global_irq_enable[0]});
    end
  end

  // seq_busy into sys_clk as phy_rst_in_progress; the flops reset to 1 (sequencer busy in reset).
  always_ff @(posedge i_sys_clk or negedge i_sys_resetn) begin : p_ff_seq_busy_sync
    if (!i_sys_resetn) begin
      reg_seq_busy_sync <= '1;
    end else begin
      reg_seq_busy_sync <= PARA_SYNC_STAGES'({reg_seq_busy_sync, reg_phy_core_rst});
    end
  end

  // CORE_CONFIG.soft_reset into rxbyteclkhs.
  always_ff @(posedge i_rxbyteclkhs or negedge i_rxbyteclkhs_resetn) begin : p_ff_cfg_byte_sync
    if (!i_rxbyteclkhs_resetn) begin
      reg_soft_reset_byte_sync <= '0;
    end else begin
      reg_soft_reset_byte_sync <= PARA_SYNC_STAGES'({reg_soft_reset_byte_sync,
                                                     reg_core_config[1]});
    end
  end

  // D-PHY enables formed in lite_aclk and registered once; lanes not built drive 0.
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_phy_en
    if (!w_rf_rst_n) begin
      reg_cl_enable   <= 1'b0;
      reg_dl_enable   <= '0;
      reg_forcerxmode <= '0;
    end else begin
      reg_cl_enable   <= reg_phy_config[0] & reg_phy_config[2];
      reg_dl_enable   <= {4{reg_phy_config[0]}} & reg_phy_config[6:3] & LPARA_LANE_MASK;
      reg_forcerxmode <= reg_phy_config[10:7] & LPARA_LANE_MASK;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Status Synchronizer
  // ------------------------------------------------------------------------------------------
  // Single-bit levels into lite_aclk through SYNC_STAGES flops. The flops of cl_ulpsactivenot and
  // cl_ulpsclknot reset to 1, so cl_ulps and cl_ulps_line read 0 after reset.
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_st_sync
    if (!w_rf_rst_n) begin
      reg_cl_stopstate_sync        <= '0;
      reg_cl_ulpsactivenot_sync    <= '1;
      reg_cl_rxclkactivehs_sync    <= '0;
      reg_cl_ulpsclknot_sync       <= '1;
      reg_init_done_sync           <= '0;
      reg_stopstate_sync           <= '0;
      reg_rxskewcalhs_sync         <= '0;
      reg_linebuffer_full_sync     <= '0;
      reg_sp_fifo_not_empty_sync   <= '0;
      reg_sp_fifo_full_sync        <= '0;
      reg_null_blank_seen_sync     <= '0;
      reg_active_sel_sync          <= '0;
      reg_switch_pending_sync      <= '0;
      reg_dt_conflict_sync         <= '0;
      reg_vc_unsupported_seen_sync <= '0;
      reg_linebuf_ovf_sticky_sync  <= '0;
      reg_sp_fifo_ovf_sticky_sync  <= '0;
      reg_meta_overflow_sync       <= '0;
      reg_dma_ovf_sticky_sync      <= '0;
      reg_irq_status_sync          <= '0;
    end else begin
      reg_cl_stopstate_sync        <= PARA_SYNC_STAGES'({reg_cl_stopstate_sync,
                                                         i_cl_stopstate});
      reg_cl_ulpsactivenot_sync    <= PARA_SYNC_STAGES'({reg_cl_ulpsactivenot_sync,
                                                         i_cl_ulpsactivenot});
      reg_cl_rxclkactivehs_sync    <= PARA_SYNC_STAGES'({reg_cl_rxclkactivehs_sync,
                                                         i_cl_rxclkactivehs});
      reg_cl_ulpsclknot_sync       <= PARA_SYNC_STAGES'({reg_cl_ulpsclknot_sync,
                                                         i_cl_ulpsclknot});
      reg_init_done_sync           <= PARA_SYNC_STAGES'({reg_init_done_sync, i_init_done});
      reg_stopstate_sync           <= $bits(reg_stopstate_sync)'({reg_stopstate_sync,
                                                                  i_stopstate});
      reg_rxskewcalhs_sync         <= $bits(reg_rxskewcalhs_sync)'({reg_rxskewcalhs_sync,
                                                                    i_rxskewcalhs});
      reg_linebuffer_full_sync     <= PARA_SYNC_STAGES'({reg_linebuffer_full_sync,
                                                         i_linebuffer_full});
      reg_sp_fifo_not_empty_sync   <= PARA_SYNC_STAGES'({reg_sp_fifo_not_empty_sync,
                                                         i_sp_fifo_not_empty});
      reg_sp_fifo_full_sync        <= PARA_SYNC_STAGES'({reg_sp_fifo_full_sync,
                                                         i_sp_fifo_full});
      reg_null_blank_seen_sync     <= PARA_SYNC_STAGES'({reg_null_blank_seen_sync,
                                                         i_null_blank_seen});
      reg_active_sel_sync          <= PARA_SYNC_STAGES'({reg_active_sel_sync, i_active_sel});
      reg_switch_pending_sync      <= PARA_SYNC_STAGES'({reg_switch_pending_sync,
                                                         i_switch_pending});
      reg_dt_conflict_sync         <= PARA_SYNC_STAGES'({reg_dt_conflict_sync, i_dt_conflict});
      reg_vc_unsupported_seen_sync <= PARA_SYNC_STAGES'({reg_vc_unsupported_seen_sync,
                                                         i_vc_unsupported_seen});
      reg_linebuf_ovf_sticky_sync  <= PARA_SYNC_STAGES'({reg_linebuf_ovf_sticky_sync,
                                                         i_linebuf_ovf_sticky});
      reg_sp_fifo_ovf_sticky_sync  <= PARA_SYNC_STAGES'({reg_sp_fifo_ovf_sticky_sync,
                                                         i_sp_fifo_ovf_sticky});
      reg_meta_overflow_sync       <= PARA_SYNC_STAGES'({reg_meta_overflow_sync,
                                                         i_meta_overflow});
      reg_dma_ovf_sticky_sync      <= PARA_SYNC_STAGES'({reg_dma_ovf_sticky_sync,
                                                         i_dma_ovf_sticky});
      reg_irq_status_sync          <= $bits(reg_irq_status_sync)'({reg_irq_status_sync,
                                                                   i_irq_status});
    end
  end

  // irq_status_l: each irq_status bit after its own synchronizer.
  always_comb begin : p_com_irq_sts
    w_irq_status_l = reg_irq_status_sync[PARA_SYNC_STAGES-1];
  end

  // Each errsoths_x / errsotsynchs_x pulse toggles a flop in rxbyteclkhs.
  always_ff @(posedge i_rxbyteclkhs or negedge i_rxbyteclkhs_resetn) begin : p_ff_sot_tgl
    if (!i_rxbyteclkhs_resetn) begin
      reg_errsoths_tgl     <= '0;
      reg_errsotsynchs_tgl <= '0;
    end else begin
      reg_errsoths_tgl     <= reg_errsoths_tgl ^ i_errsoths;
      reg_errsotsynchs_tgl <= reg_errsotsynchs_tgl ^ i_errsotsynchs;
    end
  end

  // The toggles cross through SYNC_STAGES flops; an edge sets the sticky flag. While
  // CORE_CONFIG.soft_reset is 1 the flags are held at 0, also when rxbyteclkhs is stopped.
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_sot_flag
    if (!w_rf_rst_n) begin
      reg_errsoths_sync     <= '0;
      reg_errsotsynchs_sync <= '0;
      reg_errsoths_prev     <= '0;
      reg_errsotsynchs_prev <= '0;
      reg_sot_err           <= '0;
      reg_sot_sync_err      <= '0;
    end else begin
      reg_errsoths_sync     <= $bits(reg_errsoths_sync)'({reg_errsoths_sync, reg_errsoths_tgl});
      reg_errsotsynchs_sync <= $bits(reg_errsotsynchs_sync)'({reg_errsotsynchs_sync,
                                                              reg_errsotsynchs_tgl});
      reg_errsoths_prev     <= reg_errsoths_sync[PARA_SYNC_STAGES-1];
      reg_errsotsynchs_prev <= reg_errsotsynchs_sync[PARA_SYNC_STAGES-1];
      if (reg_core_config[1]) begin
        reg_sot_err      <= '0;
        reg_sot_sync_err <= '0;
      end else begin
        reg_sot_err      <= reg_sot_err |
                            (reg_errsoths_sync[PARA_SYNC_STAGES-1] ^ reg_errsoths_prev);
        reg_sot_sync_err <= reg_sot_sync_err |
                            (reg_errsotsynchs_sync[PARA_SYNC_STAGES-1] ^ reg_errsotsynchs_prev);
      end
    end
  end

  // Multi-bit sys_clk status gathered in one value.
  always_comb begin : p_com_snp_sys_src
    w_st_sys_src.packet_cnt           = i_packet_cnt;
    w_st_sys_src.ecc_err_cnt          = i_ecc_err_cnt;
    w_st_sys_src.crc_err_cnt          = i_crc_err_cnt;
    w_st_sys_src.frame_drop_cnt       = i_frame_drop_cnt;
    w_st_sys_src.lane_abort_cnt       = i_lane_abort_cnt;
    w_st_sys_src.frame_count          = i_frame_count;
    w_st_sys_src.linebuf_fill         = i_linebuf_fill;
    w_st_sys_src.timestamp            = i_timestamp;
    w_st_sys_src.frame_id_buf0        = i_frame_id_buf0;
    w_st_sys_src.frame_id_buf1        = i_frame_id_buf1;
    w_st_sys_src.vc0_line_count       = i_vc0_line_count;
    w_st_sys_src.vc0_byte_count       = i_vc0_byte_count;
    w_st_sys_src.vc0_data_type        = i_vc0_data_type;
    w_st_sys_src.vc0_width_mismatch   = i_vc0_width_mismatch;
    w_st_sys_src.vc0_height_mismatch  = i_vc0_height_mismatch;
    w_st_sys_src.vc1_line_count       = i_vc1_line_count;
    w_st_sys_src.vc1_byte_count       = i_vc1_byte_count;
    w_st_sys_src.vc1_data_type        = i_vc1_data_type;
    w_st_sys_src.vc1_width_mismatch   = i_vc1_width_mismatch;
    w_st_sys_src.vc1_height_mismatch  = i_vc1_height_mismatch;
    w_st_sys_src.emb_word_count_vc0   = i_emb_word_count_vc0;
    w_st_sys_src.emb_line_count_vc0   = i_emb_line_count_vc0;
    w_st_sys_src.emb_word_count_vc1   = i_emb_word_count_vc1;
    w_st_sys_src.emb_line_count_vc1   = i_emb_line_count_vc1;
    w_st_sys_src.emb_extract_val_vc0  = i_emb_extract_val_vc0;
    w_st_sys_src.emb_extract_val_vc1  = i_emb_extract_val_vc1;
    w_st_sys_src.gsp_data             = i_gsp_data;
    w_st_sys_src.error_capture        = i_error_capture;
    w_st_sys_src.dma_busy             = i_dma_busy;
    w_st_sys_src.active_buffer        = i_active_buffer;
    w_st_sys_src.buf0_state           = i_buf0_state;
    w_st_sys_src.buf1_state           = i_buf1_state;
    w_st_sys_src.buf0_last_valid_line = i_buf0_last_valid_line;
    w_st_sys_src.buf1_last_valid_line = i_buf1_last_valid_line;
    w_st_sys_src.axi_fault_awid       = i_axi_fault_awid;
    w_st_sys_src.axi_fault_resp       = i_axi_fault_resp;
    w_st_sys_src.axi_fault_awaddr     = i_axi_fault_awaddr;
    w_st_sys_src.bresp_slverr_count   = i_bresp_slverr_count;
    w_st_sys_src.bresp_decerr_count   = i_bresp_decerr_count;
  end

  // Snapshot loop, lite_aclk half: the request toggles once after reset and again each time the
  // acknowledge returns; on that acknowledge every sys_clk value is taken in one cycle.
  always_comb begin : p_com_snp_lite
    w_snp_sys_done    = reg_snp_sys_started &
                        (reg_snp_sys_ack_sync[PARA_SYNC_STAGES-1] == reg_snp_sys_req_tgl);
    w_snp_sys_launch  = ~reg_snp_sys_started | w_snp_sys_done;
    w_snp_byte_done   = reg_snp_byte_started &
                        (reg_snp_byte_ack_sync[PARA_SYNC_STAGES-1] == reg_snp_byte_req_tgl);
    w_snp_byte_launch = ~reg_snp_byte_started | w_snp_byte_done;
  end

  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_snp_sys_lite
    if (!w_rf_rst_n) begin
      reg_snp_sys_started  <= 1'b0;
      reg_snp_sys_req_tgl  <= 1'b0;
      reg_snp_sys_ack_sync <= '0;
      reg_st_l_sys         <= '0;
    end else begin
      reg_snp_sys_started  <= 1'b1;
      reg_snp_sys_ack_sync <= $bits(reg_snp_sys_ack_sync)'({reg_snp_sys_ack_sync,
                                                             reg_snp_sys_ack_tgl});
      if (w_snp_sys_launch) begin
        reg_snp_sys_req_tgl <= ~reg_snp_sys_req_tgl;
      end
      if (w_snp_sys_done) begin
        reg_st_l_sys <= reg_snp_sys_cap;
      end
    end
  end

  // Snapshot loop, sys_clk half: a new request captures every value in the same cycle and
  // returns the acknowledge. The capture is read in lite_aclk only after that acknowledge.
  always_ff @(posedge i_sys_clk or negedge i_sys_resetn) begin : p_ff_snp_sys_src
    if (!i_sys_resetn) begin
      reg_snp_sys_req_sync <= '0;
      reg_snp_sys_ack_tgl  <= 1'b0;
    end else begin
      reg_snp_sys_req_sync <= $bits(reg_snp_sys_req_sync)'({reg_snp_sys_req_sync,
                                                             reg_snp_sys_req_tgl});
      reg_snp_sys_ack_tgl  <= reg_snp_sys_req_sync[PARA_SYNC_STAGES-1];
    end
  end

  always_ff @(posedge i_sys_clk) begin : p_ff_snp_sys_cap
    if (reg_snp_sys_req_sync[PARA_SYNC_STAGES-1] != reg_snp_sys_ack_tgl) begin
      reg_snp_sys_cap <= w_st_sys_src;
    end
  end

  // Multi-bit rxbyteclkhs status gathered in one value.
  always_comb begin : p_com_snp_byte_src
    w_st_byte_src.lm_fsm_state         = i_lm_fsm_state;
    w_st_byte_src.abort_sync_err       = i_abort_sync_err;
    w_st_byte_src.abort_fifo_full      = i_abort_fifo_full;
    w_st_byte_src.abort_unarmed        = i_abort_unarmed;
    w_st_byte_src.deskew_burst_seen    = i_deskew_burst_seen;
    w_st_byte_src.abort_lane_sync_seen = i_abort_lane_sync_seen;
    w_st_byte_src.active_lanes_applied = i_active_lanes_applied;
  end

  // rxbyteclkhs loop: when rxbyteclkhs stops the loop stops and the lite_aclk copy holds.
  always_ff @(posedge i_lite_aclk or negedge w_rf_rst_n) begin : p_ff_snp_byte_lite
    if (!w_rf_rst_n) begin
      reg_snp_byte_started  <= 1'b0;
      reg_snp_byte_req_tgl  <= 1'b0;
      reg_snp_byte_ack_sync <= '0;
      reg_st_l_byte         <= '0;
    end else begin
      reg_snp_byte_started  <= 1'b1;
      reg_snp_byte_ack_sync <= $bits(reg_snp_byte_ack_sync)'({reg_snp_byte_ack_sync,
                                                               reg_snp_byte_ack_tgl});
      if (w_snp_byte_launch) begin
        reg_snp_byte_req_tgl <= ~reg_snp_byte_req_tgl;
      end
      if (w_snp_byte_done) begin
        reg_st_l_byte <= reg_snp_byte_cap;
      end
    end
  end

  always_ff @(posedge i_rxbyteclkhs or negedge i_rxbyteclkhs_resetn) begin : p_ff_snp_byte_src
    if (!i_rxbyteclkhs_resetn) begin
      reg_snp_byte_req_sync <= '0;
      reg_snp_byte_ack_tgl  <= 1'b0;
    end else begin
      reg_snp_byte_req_sync <= $bits(reg_snp_byte_req_sync)'({reg_snp_byte_req_sync,
                                                               reg_snp_byte_req_tgl});
      reg_snp_byte_ack_tgl  <= reg_snp_byte_req_sync[PARA_SYNC_STAGES-1];
    end
  end

  always_ff @(posedge i_rxbyteclkhs) begin : p_ff_snp_byte_cap
    if (reg_snp_byte_req_sync[PARA_SYNC_STAGES-1] != reg_snp_byte_ack_tgl) begin
      reg_snp_byte_cap <= w_st_byte_src;
    end
  end

  // ------------------------------------------------------------------------------------------
  // PHY Reset Sequencer
  // ------------------------------------------------------------------------------------------
  // resetn_q: resetn released synchronously in lite_aclk; resetn_q_d is its one-cycle delay.
  always_ff @(posedge i_lite_aclk or negedge i_resetn) begin : p_ff_seq_rst_sync
    if (!i_resetn) begin
      reg_resetn_sync <= '0;
      reg_resetn_d    <= 1'b0;
    end else begin
      reg_resetn_sync <= $bits(reg_resetn_sync)'({reg_resetn_sync, 1'b1});
      reg_resetn_d    <= w_resetn_q;
    end
  end

  // Triggers: release of resetn, a write of 1 to PHY_CONFIG.dphy_rst_req or CORE_CONFIG.full_reset.
  always_comb begin : p_com_seq
    w_resetn_q     = reg_resetn_sync[PARA_SYNC_STAGES-1];
    w_rst_trig     = (w_resetn_q & ~reg_resetn_d) | w_wr_dphy_rst_req | w_wr_full_reset;
    w_rst_cnt_gt1  = (reg_rst_cnt != '0) & (reg_rst_cnt != LPARA_RST_CNT_W'(32'd1));
  end

  // A trigger loads PHY_RST_CYCLES (a trigger during a sequence restarts the count), so
  // phy_core_rst is high PHY_RST_CYCLES cycles. phy_core_rst is set and full_reset_req cleared
  // asynchronously while resetn is 0; full_reset_req follows a sequence started by full_reset.
  always_ff @(posedge i_lite_aclk or negedge i_resetn) begin : p_ff_seq
    if (!i_resetn) begin
      reg_rst_cnt        <= '0;
      reg_phy_core_rst   <= 1'b1;
      reg_full_reset_req <= 1'b0;
    end else begin
      if (w_rst_trig) begin
        reg_rst_cnt <= LPARA_RST_CNT_W'(PARA_PHY_RST_CYCLES);
      end else if (reg_rst_cnt != '0) begin
        reg_rst_cnt <= reg_rst_cnt - LPARA_RST_CNT_W'(32'd1);
      end
      reg_phy_core_rst   <= ~reg_resetn_d | w_rst_trig | w_rst_cnt_gt1;
      reg_full_reset_req <= w_wr_full_reset | (reg_full_reset_req & (w_rst_trig | w_rst_cnt_gt1));
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  assign o_s_axi_awready          = reg_awready;
  assign o_s_axi_wready           = reg_wready;
  assign o_s_axi_bresp            = reg_bresp;
  assign o_s_axi_bvalid           = reg_bvalid;
  assign o_s_axi_arready          = reg_arready;
  assign o_s_axi_rdata            = reg_rdata;
  assign o_s_axi_rresp            = reg_rresp;
  assign o_s_axi_rvalid           = reg_rvalid;

  assign o_phy_core_rst           = reg_phy_core_rst;
  assign o_cl_enable              = reg_cl_enable;
  assign o_dl_enable              = reg_dl_enable;
  assign o_forcerxmode            = reg_forcerxmode;
  assign o_full_reset_req         = reg_full_reset_req;

  assign o_soft_reset_byte        = reg_soft_reset_byte_sync[PARA_SYNC_STAGES-1];
  assign o_active_lanes           = reg_active_lanes;

  assign o_soft_reset_sys         = reg_soft_reset_sys_sync[PARA_SYNC_STAGES-1];
  assign o_core_enable            = reg_core_enable_sync[PARA_SYNC_STAGES-1];
  assign o_vc_exposure_map        = reg_vc_exposure_map_sync[PARA_SYNC_STAGES-1];
  assign o_metadata_dt            = reg_hs_copy[LPARA_HS_METADATA_DT_CONFIG][5:0];
  assign o_metadata_capture_en    = reg_hs_copy[LPARA_HS_METADATA_DT_CONFIG][8];
  assign o_fnum_check_en          = reg_hs_copy[LPARA_HS_METADATA_DT_CONFIG][9];
  assign o_emb_extract_en         = reg_hs_copy[LPARA_HS_METADATA_DT_CONFIG][10];
  assign o_expected_width         = reg_hs_copy[LPARA_HS_EXPECTED_RESOLUTION][15:0];
  assign o_expected_height        = reg_hs_copy[LPARA_HS_EXPECTED_RESOLUTION][31:16];
  assign o_metadata_record_id     = reg_hs_copy[LPARA_HS_METADATA_RECORD_ID];
  assign o_axis_enable            = reg_axis_enable_sync[PARA_SYNC_STAGES-1];
  assign o_stream_format          = reg_stream_format_sync[PARA_SYNC_STAGES-1];
  assign o_stream_follow_dma_drop = reg_stream_follow_dma_drop_sync[PARA_SYNC_STAGES-1];
  assign o_tpg_enable             = reg_tpg_enable_sync[PARA_SYNC_STAGES-1];
  assign o_tpg_pattern            = reg_hs_copy[LPARA_HS_TPG_CONTROL][1];
  assign o_tpg_dual_vc_en         = reg_hs_copy[LPARA_HS_TPG_CONTROL][2];
  assign o_tpg_pixel_format       = reg_hs_copy[LPARA_HS_TPG_CONTROL][3];
  assign o_tpg_cfa_order          = reg_hs_copy[LPARA_HS_TPG_CONTROL][5:4];
  assign o_tpg_width              = reg_hs_copy[LPARA_HS_TPG_RESOLUTION][15:0];
  assign o_tpg_height             = reg_hs_copy[LPARA_HS_TPG_RESOLUTION][31:16];
  assign o_tpg_hblank             = reg_hs_copy[LPARA_HS_TPG_BLANKING][15:0];
  assign o_tpg_vblank             = reg_hs_copy[LPARA_HS_TPG_BLANKING][31:16];
  assign o_dma_enable             = reg_dma_enable_sync[PARA_SYNC_STAGES-1];
  assign o_dma_start              = reg_dma_start_sync[PARA_SYNC_STAGES-1];
  assign o_dma_abort              = reg_dma_abort_sync[PARA_SYNC_STAGES-1];
  assign o_err_frame_policy       = reg_err_frame_policy_sync[PARA_SYNC_STAGES-1];
  assign o_dma_format             = reg_dma_format_sync[PARA_SYNC_STAGES-1];
  assign o_dma_buffer0_addr       = reg_hs_copy[LPARA_HS_DMA_BUFFER0_ADDR];
  assign o_dma_buffer1_addr       = reg_hs_copy[LPARA_HS_DMA_BUFFER1_ADDR];
  assign o_dma_buffer_size        = reg_hs_copy[LPARA_HS_DMA_BUFFER_SIZE];
  assign o_meta_area_size         = reg_hs_copy[LPARA_HS_META_AREA_SIZE];
  assign o_bytes_per_line         = reg_hs_copy[LPARA_HS_LINE_PITCH][15:0];
  assign o_line_padding           = reg_hs_copy[LPARA_HS_LINE_PITCH][23:16];
  assign o_awqos                  = reg_hs_copy[LPARA_HS_DMA_ATTR][3:0];
  assign o_max_burst_len          = reg_hs_copy[LPARA_HS_DMA_ATTR][11:4];
  assign o_awprot                 = reg_hs_copy[LPARA_HS_DMA_ATTR][14:12];
  assign o_release_buf            = reg_release_buf;
  assign o_global_irq_enable      = reg_global_irq_enable_sync[PARA_SYNC_STAGES-1];
  assign o_irq_enable_mask        = reg_hs_copy[LPARA_HS_IRQ_ENABLE][23:0];
  assign o_irq_clear_strobe       = reg_irq_clear_strobe;
  assign o_phy_rst_in_progress    = reg_seq_busy_sync[PARA_SYNC_STAGES-1];
  assign o_gsp_pop                = reg_gsp_pop;

  // EMB_EXTRACT_CFGm in sys_clk; a slot not built drives 0.
  generate
    for (genvar gen_idx = 0; gen_idx < LPARA_NX_SLOTS; gen_idx++) begin : gen_emb_out
      if (32'(gen_idx) < PARA_N_EXTRACT) begin : gen_built
        assign o_emb_extract_cfg[gen_idx] = reg_hs_copy[LPARA_HS_EMB_EXTRACT_CFG0 + 32'(gen_idx)];
      end else begin : gen_none
        assign o_emb_extract_cfg[gen_idx] = '0;
      end
    end
  endgenerate

endmodule

`default_nettype wire
