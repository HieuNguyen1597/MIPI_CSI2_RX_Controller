# Signal mapping — B14 CSR & AXI4-Lite Subordinate

## Notes

- RTL module: `m_fpt_csr_axi4lite` (`00_src/rtl/m_fpt_csr_axi4lite.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- `LPARA_OFS_<REG>` is the offset of CSR register `<REG>`, `LPARA_RST_<REG>` its reset value and `LPARA_HS_<REG>` its handshake index in the Control Synchronizer (`reg_hs_*` are arrays indexed by `LPARA_HS_*`).
- `st_sys_type_def` and `st_byte_type_def` are the packed structs of the `sys_clk` and `rxbyteclkhs` status snapshots; a `<typedef>.<field>` row is one struct field.
- In a spec name, `n` in `VCn` / `_vcn` is the virtual channel index and `X` / `_x` in `laneX_` / `errsoths_x` is the lane index; the RTL has one port per channel (`_vc0` / `_vc1`) and one bit per lane in a vector. `m` in `EMB_EXTRACT_CFGm` is the extraction slot index (`reg_emb_extract_cfg` element).
- The spec suffix `(D)` is the D input (next value) of the flip-flop; in an RTL name this is `w_…_nxt`. `(added port)` marks a port that is not in the spec port table.
- `gen_para_check_*` stop elaboration for an out-of-range parameter; `gen_emb_cfg`, `gen_hs_src`, `gen_emb_out`, `gen_built` and `gen_none` build one element per extraction slot; `gen_hs` builds one handshake per synchronized register.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_csr_axi4lite` | `CSR & AXI4-Lite Subordinate (Block No. 14)` | module |
| `i_lite_aclk` | `lite_aclk` | input port |
| `i_lite_aresetn` | `lite_aresetn` | input port |
| `i_resetn` | `resetn` | input port |
| `i_sys_clk` | `sys_clk` | input port |
| `i_rxbyteclkhs` | `rxbyteclkhs` | input port |
| `i_sys_resetn` | `sys_resetn (added port)` | input port |
| `i_rxbyteclkhs_resetn` | `rxbyteclkhs_resetn (added port)` | input port |
| `i_s_axi_awaddr` | `s_axi_awaddr[11:0]` | input port |
| `i_s_axi_awprot` | `s_axi_awprot[2:0]` | input port |
| `i_s_axi_awvalid` | `s_axi_awvalid` | input port |
| `o_s_axi_awready` | `s_axi_awready` | output port |
| `i_s_axi_wdata` | `s_axi_wdata[31:0]` | input port |
| `i_s_axi_wstrb` | `s_axi_wstrb[3:0]` | input port |
| `i_s_axi_wvalid` | `s_axi_wvalid` | input port |
| `o_s_axi_wready` | `s_axi_wready` | output port |
| `o_s_axi_bresp` | `s_axi_bresp[1:0]` | output port |
| `o_s_axi_bvalid` | `s_axi_bvalid` | output port |
| `i_s_axi_bready` | `s_axi_bready` | input port |
| `i_s_axi_araddr` | `s_axi_araddr[11:0]` | input port |
| `i_s_axi_arprot` | `s_axi_arprot[2:0]` | input port |
| `i_s_axi_arvalid` | `s_axi_arvalid` | input port |
| `o_s_axi_arready` | `s_axi_arready` | output port |
| `o_s_axi_rdata` | `s_axi_rdata[31:0]` | output port |
| `o_s_axi_rresp` | `s_axi_rresp[1:0]` | output port |
| `o_s_axi_rvalid` | `s_axi_rvalid` | output port |
| `i_s_axi_rready` | `s_axi_rready` | input port |
| `o_phy_core_rst` | `phy_core_rst` | output port |
| `o_cl_enable` | `cl_enable` | output port |
| `o_dl_enable` | `dl_enable[3:0]` | output port |
| `o_forcerxmode` | `forcerxmode[3:0]` | output port |
| `o_full_reset_req` | `full_reset_req` | output port |
| `i_cl_stopstate` | `cl_stopstate` | input port |
| `i_cl_ulpsactivenot` | `cl_ulpsactivenot` | input port |
| `i_cl_rxclkactivehs` | `cl_rxclkactivehs` | input port |
| `i_cl_ulpsclknot` | `cl_ulpsclknot` | input port |
| `i_init_done` | `init_done` | input port |
| `i_stopstate` | `stopstate[3:0]` | input port |
| `i_rxskewcalhs` | `rxskewcalhs[3:0]` | input port |
| `i_errsoths` | `errsoths[3:0]` | input port |
| `i_errsotsynchs` | `errsotsynchs[3:0]` | input port |
| `o_soft_reset_byte` | `soft_reset (rxbyteclkhs copy)` | output port |
| `o_active_lanes` | `active_lanes[1:0]` | output port |
| `o_soft_reset_sys` | `soft_reset (sys_clk copy)` | output port |
| `o_core_enable` | `core_enable` | output port |
| `o_vc_exposure_map` | `vc_exposure_map` | output port |
| `o_metadata_dt` | `metadata_dt[5:0]` | output port |
| `o_metadata_capture_en` | `metadata_capture_en` | output port |
| `o_fnum_check_en` | `fnum_check_en` | output port |
| `o_emb_extract_en` | `emb_extract_en` | output port |
| `o_expected_width` | `expected_width[15:0]` | output port |
| `o_expected_height` | `expected_height[15:0]` | output port |
| `o_metadata_record_id` | `metadata_record_id[31:0]` | output port |
| `o_emb_extract_cfg` | `emb_extract_cfg[N_EXTRACT-1:0][31:0]` | output port |
| `o_axis_enable` | `axis_enable` | output port |
| `o_stream_format` | `stream_format` | output port |
| `o_stream_follow_dma_drop` | `stream_follow_dma_drop` | output port |
| `o_tpg_enable` | `tpg_enable` | output port |
| `o_tpg_pattern` | `tpg_pattern` | output port |
| `o_tpg_dual_vc_en` | `tpg_dual_vc_en` | output port |
| `o_tpg_pixel_format` | `tpg_pixel_format` | output port |
| `o_tpg_cfa_order` | `tpg_cfa_order[1:0] (added port)` | output port |
| `o_tpg_width` | `tpg_width[15:0]` | output port |
| `o_tpg_height` | `tpg_height[15:0]` | output port |
| `o_tpg_hblank` | `tpg_hblank[15:0]` | output port |
| `o_tpg_vblank` | `tpg_vblank[15:0]` | output port |
| `o_dma_enable` | `dma_enable` | output port |
| `o_dma_start` | `dma_start` | output port |
| `o_dma_abort` | `dma_abort` | output port |
| `o_err_frame_policy` | `err_frame_policy` | output port |
| `o_dma_format` | `dma_format` | output port |
| `o_dma_buffer0_addr` | `dma_buffer0_addr[31:0]` | output port |
| `o_dma_buffer1_addr` | `dma_buffer1_addr[31:0]` | output port |
| `o_dma_buffer_size` | `dma_buffer_size[31:0]` | output port |
| `o_meta_area_size` | `meta_area_size[31:0]` | output port |
| `o_bytes_per_line` | `bytes_per_line[15:0]` | output port |
| `o_line_padding` | `line_padding[7:0]` | output port |
| `o_awqos` | `awqos[3:0]` | output port |
| `o_max_burst_len` | `max_burst_len[7:0]` | output port |
| `o_awprot` | `awprot[2:0]` | output port |
| `o_release_buf` | `release_buf[1:0]` | output port |
| `o_global_irq_enable` | `global_irq_enable` | output port |
| `o_irq_enable_mask` | `irq_enable_mask[23:0]` | output port |
| `o_irq_clear_strobe` | `irq_clear_strobe[23:0]` | output port |
| `o_phy_rst_in_progress` | `phy_rst_in_progress` | output port |
| `i_lm_fsm_state` | `lm_fsm_state[2:0]` | input port |
| `i_abort_sync_err` | `abort_sync_err` | input port |
| `i_abort_fifo_full` | `abort_fifo_full` | input port |
| `i_abort_unarmed` | `abort_unarmed` | input port |
| `i_deskew_burst_seen` | `deskew_burst_seen` | input port |
| `i_abort_lane_sync_seen` | `abort_lane_sync_seen[3:0]` | input port |
| `i_active_lanes_applied` | `active_lanes_applied[1:0]` | input port |
| `i_sp_fifo_not_empty` | `sp_fifo_not_empty` | input port |
| `i_sp_fifo_full` | `sp_fifo_full` | input port |
| `i_null_blank_seen` | `null_blank_seen` | input port |
| `i_dt_conflict` | `dt_conflict` | input port |
| `i_vc_unsupported_seen` | `vc_unsupported_seen` | input port |
| `i_gsp_data` | `gsp_data[23:0]` | input port |
| `o_gsp_pop` | `gsp_pop` | output port |
| `i_active_sel` | `active_sel` | input port |
| `i_switch_pending` | `switch_pending` | input port |
| `i_linebuffer_full` | `linebuffer_full` | input port |
| `i_linebuf_fill` | `linebuf_fill[15:0]` | input port |
| `i_frame_count` | `frame_count[31:0]` | input port |
| `i_timestamp` | `timestamp[63:0]` | input port |
| `i_frame_id_buf0` | `frame_id_buf0[13:0]` | input port |
| `i_frame_id_buf1` | `frame_id_buf1[13:0]` | input port |
| `i_vc0_line_count` | `VCn geometry line_count[15:0], n = 0 (vcn_line_count)` | input port |
| `i_vc0_byte_count` | `VCn geometry byte_count[15:0], n = 0 (vcn_byte_count)` | input port |
| `i_vc0_data_type` | `VCn geometry data_type[5:0], n = 0 (vcn_data_type)` | input port |
| `i_vc0_width_mismatch` | `VCn geometry width_mismatch, n = 0 (vcn_width_mismatch)` | input port |
| `i_vc0_height_mismatch` | `VCn geometry height_mismatch, n = 0 (vcn_height_mismatch)` | input port |
| `i_vc1_line_count` | `VCn geometry line_count[15:0], n = 1 (vcn_line_count)` | input port |
| `i_vc1_byte_count` | `VCn geometry byte_count[15:0], n = 1 (vcn_byte_count)` | input port |
| `i_vc1_data_type` | `VCn geometry data_type[5:0], n = 1 (vcn_data_type)` | input port |
| `i_vc1_width_mismatch` | `VCn geometry width_mismatch, n = 1 (vcn_width_mismatch)` | input port |
| `i_vc1_height_mismatch` | `VCn geometry height_mismatch, n = 1 (vcn_height_mismatch)` | input port |
| `i_emb_word_count_vc0` | `emb_word_count_vcn[15:0], n = 0` | input port |
| `i_emb_line_count_vc0` | `emb_line_count_vcn[7:0], n = 0` | input port |
| `i_emb_word_count_vc1` | `emb_word_count_vcn[15:0], n = 1` | input port |
| `i_emb_line_count_vc1` | `emb_line_count_vcn[7:0], n = 1` | input port |
| `i_emb_extract_val_vc0` | `emb_extract_val_vcn[N_EXTRACT-1:0][31:0], n = 0` | input port |
| `i_emb_extract_val_vc1` | `emb_extract_val_vcn[N_EXTRACT-1:0][31:0], n = 1` | input port |
| `i_meta_overflow` | `meta_overflow` | input port |
| `i_irq_status` | `irq_status[23:0]` | input port |
| `i_error_capture` | `error_capture[31:0]` | input port |
| `i_packet_cnt` | `packet_cnt[31:0]` | input port |
| `i_ecc_err_cnt` | `ecc_err_cnt[31:0]` | input port |
| `i_crc_err_cnt` | `crc_err_cnt[31:0]` | input port |
| `i_frame_drop_cnt` | `frame_drop_cnt[31:0]` | input port |
| `i_lane_abort_cnt` | `lane_abort_cnt[15:0]` | input port |
| `i_linebuf_ovf_sticky` | `linebuf_ovf_sticky` | input port |
| `i_sp_fifo_ovf_sticky` | `sp_fifo_ovf_sticky` | input port |
| `i_dma_ovf_sticky` | `dma_ovf_sticky` | input port |
| `i_dma_busy` | `dma_busy` | input port |
| `i_active_buffer` | `active_buffer` | input port |
| `i_buf0_state` | `buf0_state[1:0]` | input port |
| `i_buf1_state` | `buf1_state[1:0]` | input port |
| `i_buf0_last_valid_line` | `buf0_last_valid_line[15:0]` | input port |
| `i_buf1_last_valid_line` | `buf1_last_valid_line[15:0]` | input port |
| `i_axi_fault_awid` | `axi_fault_awid[7:0]` | input port |
| `i_axi_fault_resp` | `axi_fault_resp[1:0]` | input port |
| `i_axi_fault_awaddr` | `axi_fault_awaddr[31:0]` | input port |
| `i_bresp_slverr_count` | `bresp_slverr_count[15:0]` | input port |
| `i_bresp_decerr_count` | `bresp_decerr_count[15:0]` | input port |
| `PARA_MAX_LANES` | `MAX_LANES` | parameter |
| `PARA_N_EXTRACT` | `N_EXTRACT` | parameter |
| `PARA_MAX_OUTSTANDING` | `MAX_OUTSTANDING` | parameter |
| `PARA_SYNC_STAGES` | `SYNC_STAGES` | parameter |
| `PARA_PHY_RST_CYCLES` | `PHY_RST_CYCLES` | parameter |
| `PARA_IRQ_CLR_TIMEOUT` | `IRQ_CLR_TIMEOUT` | parameter |
| `LPARA_ADDR_WIDTH` | `ADDR_WIDTH` | localparam |
| `LPARA_DATA_WIDTH` | `DATA_WIDTH` | localparam |
| `LPARA_IRQ_WIDTH` | `IRQ_WIDTH` | localparam |
| `LPARA_IP_VERSION` | `IP_VERSION` | localparam |
| `LPARA_MAP_BITS` | `0x00 to 0xFF decoded` | localparam |
| `LPARA_RESP_OKAY` | `OKAY` | localparam |
| `LPARA_RESP_DECERR` | `DECERR` | localparam |
| `LPARA_RST_CNT_W` | - | localparam |
| `LPARA_WAIT_CNT_W` | `wait_cnt width` | localparam |
| `LPARA_NX_SLOTS` | - | localparam |
| `LPARA_MAX_LANES_CODE` | `PROTOCOL_CONFIG.max_lanes` | localparam |
| `LPARA_LANE_MASK` | - | localparam |
| `LPARA_RF_SYNC_W` | - | localparam |
| `LPARA_OFS_IP_VERSION` | `IP_VERSION offset` | localparam |
| `LPARA_OFS_CORE_CONFIG` | `CORE_CONFIG offset` | localparam |
| `LPARA_OFS_PROTOCOL_CONFIG` | `PROTOCOL_CONFIG offset` | localparam |
| `LPARA_OFS_PHY_CONFIG` | `PHY_CONFIG offset` | localparam |
| `LPARA_OFS_METADATA_DT_CONFIG` | `METADATA_DT_CONFIG offset` | localparam |
| `LPARA_OFS_STREAM_CONFIG` | `STREAM_CONFIG offset` | localparam |
| `LPARA_OFS_EXPECTED_RESOLUTION` | `EXPECTED_RESOLUTION offset` | localparam |
| `LPARA_OFS_CORE_STATUS` | `CORE_STATUS offset` | localparam |
| `LPARA_OFS_PHY_STATUS` | `PHY_STATUS offset` | localparam |
| `LPARA_OFS_LANE_STATUS` | `LANE_STATUS offset` | localparam |
| `LPARA_OFS_PACKET_COUNT` | `PACKET_COUNT offset` | localparam |
| `LPARA_OFS_FRAME_COUNT` | `FRAME_COUNT offset` | localparam |
| `LPARA_OFS_BUFFER_STATUS` | `BUFFER_STATUS offset` | localparam |
| `LPARA_OFS_LANE_ALIGN_STATUS` | `LANE_ALIGN_STATUS offset` | localparam |
| `LPARA_OFS_GLOBAL_IRQ_ENABLE` | `GLOBAL_IRQ_ENABLE offset` | localparam |
| `LPARA_OFS_IRQ_STATUS` | `IRQ_STATUS offset` | localparam |
| `LPARA_OFS_IRQ_ENABLE` | `IRQ_ENABLE offset` | localparam |
| `LPARA_OFS_ECC_ERROR_COUNT` | `ECC_ERROR_COUNT offset` | localparam |
| `LPARA_OFS_CRC_ERROR_COUNT` | `CRC_ERROR_COUNT offset` | localparam |
| `LPARA_OFS_FRAME_DROP_COUNT` | `FRAME_DROP_COUNT offset` | localparam |
| `LPARA_OFS_LANE_ABORT_COUNT` | `LANE_ABORT_COUNT offset` | localparam |
| `LPARA_OFS_VC0_IMAGE_INFO1` | `VC0_IMAGE_INFO1 offset` | localparam |
| `LPARA_OFS_VC0_IMAGE_INFO2` | `VC0_IMAGE_INFO2 offset` | localparam |
| `LPARA_OFS_VC1_IMAGE_INFO1` | `VC1_IMAGE_INFO1 offset` | localparam |
| `LPARA_OFS_VC1_IMAGE_INFO2` | `VC1_IMAGE_INFO2 offset` | localparam |
| `LPARA_OFS_GENERIC_SHORT_PACKET` | `GENERIC_SHORT_PACKET offset` | localparam |
| `LPARA_OFS_EMBEDDED_DATA_INFO_VC0` | `EMBEDDED_DATA_INFO_VC0 offset` | localparam |
| `LPARA_OFS_EMBEDDED_DATA_INFO_VC1` | `EMBEDDED_DATA_INFO_VC1 offset` | localparam |
| `LPARA_OFS_METADATA_RECORD_ID` | `METADATA_RECORD_ID offset` | localparam |
| `LPARA_OFS_FRAME_ID` | `FRAME_ID offset` | localparam |
| `LPARA_OFS_TIMESTAMP_LO` | `TIMESTAMP_LO offset` | localparam |
| `LPARA_OFS_TIMESTAMP_HI` | `TIMESTAMP_HI offset` | localparam |
| `LPARA_OFS_DMA_CONTROL` | `DMA_CONTROL offset` | localparam |
| `LPARA_OFS_DMA_BUFFER0_ADDR` | `DMA_BUFFER0_ADDR offset` | localparam |
| `LPARA_OFS_DMA_BUFFER1_ADDR` | `DMA_BUFFER1_ADDR offset` | localparam |
| `LPARA_OFS_DMA_BUFFER_SIZE` | `DMA_BUFFER_SIZE offset` | localparam |
| `LPARA_OFS_DMA_ATTR` | `DMA_ATTR offset` | localparam |
| `LPARA_OFS_DMA_STATUS` | `DMA_STATUS offset` | localparam |
| `LPARA_OFS_LINE_PITCH` | `LINE_PITCH offset` | localparam |
| `LPARA_OFS_BUFFER_OWNERSHIP` | `BUFFER_OWNERSHIP offset` | localparam |
| `LPARA_OFS_BUFFER_RELEASE` | `BUFFER_RELEASE offset` | localparam |
| `LPARA_OFS_BUFFER_ERROR_INFO` | `BUFFER_ERROR_INFO offset` | localparam |
| `LPARA_OFS_META_AREA_SIZE` | `META_AREA_SIZE offset` | localparam |
| `LPARA_OFS_TPG_CONTROL` | `TPG_CONTROL offset` | localparam |
| `LPARA_OFS_ERROR_CAPTURE` | `ERROR_CAPTURE offset` | localparam |
| `LPARA_OFS_AXI_FAULT_TRACE` | `AXI_FAULT_TRACE offset` | localparam |
| `LPARA_OFS_AXI_FAULT_ADDR` | `AXI_FAULT_ADDR offset` | localparam |
| `LPARA_OFS_AXI_ERROR_COUNT` | `AXI_ERROR_COUNT offset` | localparam |
| `LPARA_OFS_TPG_RESOLUTION` | `TPG_RESOLUTION offset` | localparam |
| `LPARA_OFS_TPG_BLANKING` | `TPG_BLANKING offset` | localparam |
| `LPARA_OFS_EMB_EXTRACT_CFG0` | `EMB_EXTRACT_CFG0 offset` | localparam |
| `LPARA_OFS_EMB_EXTRACT_CFG1` | `EMB_EXTRACT_CFG1 offset` | localparam |
| `LPARA_OFS_EMB_EXTRACT_VC0_VAL0` | `EMB_EXTRACT_VC0_VAL0 offset` | localparam |
| `LPARA_OFS_EMB_EXTRACT_VC0_VAL1` | `EMB_EXTRACT_VC0_VAL1 offset` | localparam |
| `LPARA_OFS_EMB_EXTRACT_VC1_VAL0` | `EMB_EXTRACT_VC1_VAL0 offset` | localparam |
| `LPARA_OFS_EMB_EXTRACT_VC1_VAL1` | `EMB_EXTRACT_VC1_VAL1 offset` | localparam |
| `LPARA_RST_CORE_CONFIG` | `CORE_CONFIG reset value` | localparam |
| `LPARA_RST_PROTOCOL_CONFIG` | `PROTOCOL_CONFIG reset value` | localparam |
| `LPARA_RST_PHY_CONFIG` | `PHY_CONFIG reset value` | localparam |
| `LPARA_RST_METADATA_DT_CONFIG` | `METADATA_DT_CONFIG reset value` | localparam |
| `LPARA_RST_STREAM_CONFIG` | `STREAM_CONFIG reset value` | localparam |
| `LPARA_RST_EXPECTED_RESOLUTION` | `EXPECTED_RESOLUTION reset value` | localparam |
| `LPARA_RST_GLOBAL_IRQ_ENABLE` | `GLOBAL_IRQ_ENABLE reset value` | localparam |
| `LPARA_RST_IRQ_ENABLE` | `IRQ_ENABLE reset value` | localparam |
| `LPARA_RST_METADATA_RECORD_ID` | `METADATA_RECORD_ID reset value` | localparam |
| `LPARA_RST_DMA_CONTROL` | `DMA_CONTROL reset value` | localparam |
| `LPARA_RST_DMA_BUFFER0_ADDR` | `DMA_BUFFER0_ADDR reset value` | localparam |
| `LPARA_RST_DMA_BUFFER1_ADDR` | `DMA_BUFFER1_ADDR reset value` | localparam |
| `LPARA_RST_DMA_BUFFER_SIZE` | `DMA_BUFFER_SIZE reset value` | localparam |
| `LPARA_RST_DMA_ATTR` | `DMA_ATTR reset value` | localparam |
| `LPARA_RST_LINE_PITCH` | `LINE_PITCH reset value` | localparam |
| `LPARA_RST_META_AREA_SIZE` | `META_AREA_SIZE reset value` | localparam |
| `LPARA_RST_TPG_CONTROL` | `TPG_CONTROL reset value` | localparam |
| `LPARA_RST_TPG_RESOLUTION` | `TPG_RESOLUTION reset value` | localparam |
| `LPARA_RST_TPG_BLANKING` | `TPG_BLANKING reset value` | localparam |
| `LPARA_RST_EMB_EXTRACT_CFG` | `EMB_EXTRACT_CFGm reset value` | localparam |
| `LPARA_HS_METADATA_DT_CONFIG` | - | localparam |
| `LPARA_HS_EXPECTED_RESOLUTION` | - | localparam |
| `LPARA_HS_IRQ_ENABLE` | - | localparam |
| `LPARA_HS_METADATA_RECORD_ID` | - | localparam |
| `LPARA_HS_DMA_BUFFER0_ADDR` | - | localparam |
| `LPARA_HS_DMA_BUFFER1_ADDR` | - | localparam |
| `LPARA_HS_DMA_BUFFER_SIZE` | - | localparam |
| `LPARA_HS_DMA_ATTR` | - | localparam |
| `LPARA_HS_LINE_PITCH` | - | localparam |
| `LPARA_HS_META_AREA_SIZE` | - | localparam |
| `LPARA_HS_TPG_CONTROL` | - | localparam |
| `LPARA_HS_TPG_RESOLUTION` | - | localparam |
| `LPARA_HS_TPG_BLANKING` | - | localparam |
| `LPARA_HS_EMB_EXTRACT_CFG0` | - | localparam |
| `LPARA_HS_EMB_EXTRACT_CFG1` | - | localparam |
| `LPARA_HS_NUM` | - | localparam |
| `LPARA_SLOT_OFS` | - | localparam |
| `LPARA_SLOT_ON` | - | localparam |
| `axil_state_type_def` | `Access FSM state` | typedef (enum) |
| `st_sys_type_def` | `st_sys_*` | typedef (struct) |
| `st_byte_type_def` | `st_byte_*` | typedef (struct) |
| `ENUM_ST_IDLE` | `ST_IDLE` | enum member |
| `ENUM_ST_WRITE` | `ST_WRITE` | enum member |
| `ENUM_ST_BWAIT` | `ST_BWAIT` | enum member |
| `ENUM_ST_BRESP` | `ST_BRESP` | enum member |
| `ENUM_ST_READ` | `ST_READ` | enum member |
| `ENUM_ST_RRESP` | `ST_RRESP` | enum member |
| `st_sys_type_def.packet_cnt` | `packet_cnt[31:0]` | struct member |
| `st_sys_type_def.ecc_err_cnt` | `ecc_err_cnt[31:0]` | struct member |
| `st_sys_type_def.crc_err_cnt` | `crc_err_cnt[31:0]` | struct member |
| `st_sys_type_def.frame_drop_cnt` | `frame_drop_cnt[31:0]` | struct member |
| `st_sys_type_def.lane_abort_cnt` | `lane_abort_cnt[15:0]` | struct member |
| `st_sys_type_def.frame_count` | `frame_count[31:0]` | struct member |
| `st_sys_type_def.linebuf_fill` | `linebuf_fill[15:0]` | struct member |
| `st_sys_type_def.timestamp` | `timestamp[63:0]` | struct member |
| `st_sys_type_def.frame_id_buf0` | `frame_id_buf0[13:0]` | struct member |
| `st_sys_type_def.frame_id_buf1` | `frame_id_buf1[13:0]` | struct member |
| `st_sys_type_def.vc0_line_count` | `VCn geometry line_count[15:0], n = 0 (vcn_line_count)` | struct member |
| `st_sys_type_def.vc0_byte_count` | `VCn geometry byte_count[15:0], n = 0 (vcn_byte_count)` | struct member |
| `st_sys_type_def.vc0_data_type` | `VCn geometry data_type[5:0], n = 0 (vcn_data_type)` | struct member |
| `st_sys_type_def.vc0_width_mismatch` | `VCn geometry width_mismatch, n = 0 (vcn_width_mismatch)` | struct member |
| `st_sys_type_def.vc0_height_mismatch` | `VCn geometry height_mismatch, n = 0 (vcn_height_mismatch)` | struct member |
| `st_sys_type_def.vc1_line_count` | `VCn geometry line_count[15:0], n = 1 (vcn_line_count)` | struct member |
| `st_sys_type_def.vc1_byte_count` | `VCn geometry byte_count[15:0], n = 1 (vcn_byte_count)` | struct member |
| `st_sys_type_def.vc1_data_type` | `VCn geometry data_type[5:0], n = 1 (vcn_data_type)` | struct member |
| `st_sys_type_def.vc1_width_mismatch` | `VCn geometry width_mismatch, n = 1 (vcn_width_mismatch)` | struct member |
| `st_sys_type_def.vc1_height_mismatch` | `VCn geometry height_mismatch, n = 1 (vcn_height_mismatch)` | struct member |
| `st_sys_type_def.emb_word_count_vc0` | `emb_word_count_vcn[15:0], n = 0` | struct member |
| `st_sys_type_def.emb_line_count_vc0` | `emb_line_count_vcn[7:0], n = 0` | struct member |
| `st_sys_type_def.emb_word_count_vc1` | `emb_word_count_vcn[15:0], n = 1` | struct member |
| `st_sys_type_def.emb_line_count_vc1` | `emb_line_count_vcn[7:0], n = 1` | struct member |
| `st_sys_type_def.emb_extract_val_vc0` | `emb_extract_val_vcn[N_EXTRACT-1:0][31:0], n = 0` | struct member |
| `st_sys_type_def.emb_extract_val_vc1` | `emb_extract_val_vcn[N_EXTRACT-1:0][31:0], n = 1` | struct member |
| `st_sys_type_def.gsp_data` | `gsp_data[23:0]` | struct member |
| `st_sys_type_def.error_capture` | `error_capture[31:0]` | struct member |
| `st_sys_type_def.dma_busy` | `dma_busy` | struct member |
| `st_sys_type_def.active_buffer` | `active_buffer` | struct member |
| `st_sys_type_def.buf0_state` | `buf0_state[1:0]` | struct member |
| `st_sys_type_def.buf1_state` | `buf1_state[1:0]` | struct member |
| `st_sys_type_def.buf0_last_valid_line` | `buf0_last_valid_line[15:0]` | struct member |
| `st_sys_type_def.buf1_last_valid_line` | `buf1_last_valid_line[15:0]` | struct member |
| `st_sys_type_def.axi_fault_awid` | `axi_fault_awid[7:0]` | struct member |
| `st_sys_type_def.axi_fault_resp` | `axi_fault_resp[1:0]` | struct member |
| `st_sys_type_def.axi_fault_awaddr` | `axi_fault_awaddr[31:0]` | struct member |
| `st_sys_type_def.bresp_slverr_count` | `bresp_slverr_count[15:0]` | struct member |
| `st_sys_type_def.bresp_decerr_count` | `bresp_decerr_count[15:0]` | struct member |
| `st_byte_type_def.lm_fsm_state` | `lm_fsm_state[2:0]` | struct member |
| `st_byte_type_def.abort_sync_err` | `abort_sync_err` | struct member |
| `st_byte_type_def.abort_fifo_full` | `abort_fifo_full` | struct member |
| `st_byte_type_def.abort_unarmed` | `abort_unarmed` | struct member |
| `st_byte_type_def.deskew_burst_seen` | `deskew_burst_seen` | struct member |
| `st_byte_type_def.abort_lane_sync_seen` | `abort_lane_sync_seen[3:0]` | struct member |
| `st_byte_type_def.active_lanes_applied` | `active_lanes_applied[1:0]` | struct member |
| `reg_fsm_axil_current_state` | `Access FSM state` | flip-flop |
| `w_fsm_axil_nxt_state` | `Access FSM next state` | combinational |
| `w_fsm_axil_bad` | - | combinational |
| `reg_aw_addr` | `aw_addr_q[11:0]` | flip-flop |
| `reg_ar_addr` | `ar_addr_q[11:0]` | flip-flop |
| `reg_w_data` | `w_data_q[31:0] / wr_data` | flip-flop |
| `reg_w_strb` | `w_strb_q[3:0] / wr_strb` | flip-flop |
| `reg_aw_got` | `aw_got` | flip-flop |
| `reg_w_got` | `w_got` | flip-flop |
| `reg_ar_got` | `ar_got` | flip-flop |
| `w_aw_got_nxt` | `aw_got (D)` | combinational |
| `w_w_got_nxt` | `w_got (D)` | combinational |
| `w_ar_got_nxt` | `ar_got (D)` | combinational |
| `w_aw_hs` | `aw_hs` | combinational |
| `w_w_hs` | `w_hs` | combinational |
| `w_ar_hs` | `ar_hs` | combinational |
| `reg_awready` | `s_axi_awready` | flip-flop |
| `reg_wready` | `s_axi_wready` | flip-flop |
| `reg_arready` | `s_axi_arready` | flip-flop |
| `reg_bvalid` | `s_axi_bvalid` | flip-flop |
| `reg_rvalid` | `s_axi_rvalid` | flip-flop |
| `w_awready_nxt` | `s_axi_awready (D)` | combinational |
| `w_wready_nxt` | `s_axi_wready (D)` | combinational |
| `w_arready_nxt` | `s_axi_arready (D)` | combinational |
| `w_bvalid_nxt` | `s_axi_bvalid (D)` | combinational |
| `w_rvalid_nxt` | `s_axi_rvalid (D)` | combinational |
| `reg_bresp` | `s_axi_bresp` | flip-flop |
| `reg_rresp` | `s_axi_rresp` | flip-flop |
| `reg_rdata` | `s_axi_rdata` | flip-flop |
| `w_wr_en` | `wr_en` | combinational |
| `w_rd_en` | `rd_en` | combinational |
| `w_acc_addr` | `acc_addr[11:0]` | combinational |
| `reg_wait_cnt` | `wait_cnt` | flip-flop |
| `w_bm` | `bm` | combinational |
| `w_clr_mask` | `clr_mask[23:0]` | combinational |
| `w_clr_done` | `clr_done` | combinational |
| `w_map_hit` | - | combinational |
| `w_ofs` | - | combinational |
| `w_resp` | `resp[1:0]` | combinational |
| `w_is_irq` | `is_irq` | combinational |
| `w_rd_hold` | `rd_hold` | combinational |
| `w_wr_sel` | - | combinational |
| `w_rd_done` | - | combinational |
| `w_rd_data` | `rd_data[31:0]` | combinational |
| `w_wr_full_reset` | `wr_full_reset` | combinational |
| `w_wr_dphy_rst_req` | `wr_dphy_rst_req` | combinational |
| `w_rel_mask` | `rel_mask[1:0]` | combinational |
| `w_irq_clr_mask` | `irq_clr_mask[23:0]` | combinational |
| `w_gsp_pop_req` | `gsp_pop_req` | combinational |
| `w_ts_lo_rd` | - | combinational |
| `w_reset_in_progress` | `CORE_STATUS.reset_in_progress` | combinational |
| `reg_core_config` | `CORE_CONFIG` | flip-flop |
| `reg_protocol_config` | `PROTOCOL_CONFIG` | flip-flop |
| `reg_phy_config` | `PHY_CONFIG` | flip-flop |
| `reg_metadata_dt_config` | `METADATA_DT_CONFIG` | flip-flop |
| `reg_stream_config` | `STREAM_CONFIG` | flip-flop |
| `reg_expected_resolution` | `EXPECTED_RESOLUTION` | flip-flop |
| `reg_global_irq_enable` | `GLOBAL_IRQ_ENABLE` | flip-flop |
| `reg_irq_enable` | `IRQ_ENABLE` | flip-flop |
| `reg_metadata_record_id` | `METADATA_RECORD_ID` | flip-flop |
| `reg_dma_control` | `DMA_CONTROL` | flip-flop |
| `reg_dma_buffer0_addr` | `DMA_BUFFER0_ADDR` | flip-flop |
| `reg_dma_buffer1_addr` | `DMA_BUFFER1_ADDR` | flip-flop |
| `reg_dma_buffer_size` | `DMA_BUFFER_SIZE` | flip-flop |
| `reg_dma_attr` | `DMA_ATTR` | flip-flop |
| `reg_line_pitch` | `LINE_PITCH` | flip-flop |
| `reg_meta_area_size` | `META_AREA_SIZE` | flip-flop |
| `reg_tpg_control` | `TPG_CONTROL` | flip-flop |
| `reg_tpg_resolution` | `TPG_RESOLUTION` | flip-flop |
| `reg_tpg_blanking` | `TPG_BLANKING` | flip-flop |
| `reg_emb_extract_cfg` | `EMB_EXTRACT_CFG0/1` | flip-flop |
| `reg_ts_hi_shadow` | `TIMESTAMP_HI shadow` | flip-flop |
| `reg_gsp_stale` | `gsp_stale` | flip-flop |
| `reg_gsp_arm` | - | flip-flop |
| `reg_rf_rst_sync` | `rf_rst_n (leading stages)` | flip-flop |
| `reg_rf_rst_n` | `rf_rst_n` | flip-flop |
| `reg_rf_in_rst` | - | flip-flop |
| `w_rf_arst_n` | `resetn AND NOT full_reset_req` | combinational |
| `w_rf_rst_n` | `rf_rst_n` | combinational |
| `w_hs_src` | `reg_* (stored value)` | combinational |
| `w_hs_wr` | `reg_wr[k]` | combinational |
| `w_hs_launch` | - | combinational |
| `reg_hs_pend` | - | flip-flop |
| `reg_hs_req_tgl` | - | flip-flop |
| `reg_hs_ack_sync` | - | flip-flop |
| `reg_hs_hold` | - | flip-flop |
| `reg_hs_req_sync` | - | flip-flop |
| `reg_hs_ack_tgl` | - | flip-flop |
| `reg_hs_copy` | `cfg_sys_* (destination copy)` | flip-flop |
| `w_hs_byte_wr` | `reg_wr[PROTOCOL_CONFIG]` | combinational |
| `w_hs_byte_launch` | - | combinational |
| `reg_hs_byte_pend` | - | flip-flop |
| `reg_hs_byte_req_tgl` | - | flip-flop |
| `reg_hs_byte_ack_sync` | - | flip-flop |
| `reg_hs_byte_hold` | - | flip-flop |
| `reg_hs_byte_req_sync` | - | flip-flop |
| `reg_hs_byte_ack_tgl` | - | flip-flop |
| `reg_active_lanes` | `active_lanes (cfg_byte_*)` | flip-flop |
| `w_cmd_irq_launch` | - | combinational |
| `reg_cmd_irq_pend` | - | flip-flop |
| `reg_cmd_irq_req_tgl` | - | flip-flop |
| `reg_cmd_irq_ack_sync` | - | flip-flop |
| `reg_cmd_irq_hold` | - | flip-flop |
| `reg_cmd_irq_req_sync` | - | flip-flop |
| `reg_cmd_irq_ack_tgl` | - | flip-flop |
| `reg_irq_clear_strobe` | `irq_clear_strobe` | flip-flop |
| `w_cmd_rel_launch` | - | combinational |
| `reg_cmd_rel_pend` | - | flip-flop |
| `reg_cmd_rel_req_tgl` | - | flip-flop |
| `reg_cmd_rel_ack_sync` | - | flip-flop |
| `reg_cmd_rel_hold` | - | flip-flop |
| `reg_cmd_rel_req_sync` | - | flip-flop |
| `reg_cmd_rel_ack_tgl` | - | flip-flop |
| `reg_release_buf` | `release_buf` | flip-flop |
| `w_cmd_pop_launch` | - | combinational |
| `w_pop_pend` | `pop_pend` | combinational |
| `reg_cmd_pop_pend` | - | flip-flop |
| `reg_cmd_pop_req_tgl` | - | flip-flop |
| `reg_cmd_pop_ack_sync` | - | flip-flop |
| `reg_cmd_pop_req_sync` | - | flip-flop |
| `reg_cmd_pop_ack_tgl` | - | flip-flop |
| `reg_gsp_pop` | `gsp_pop` | flip-flop |
| `reg_core_enable_sync` | `synchronizer core_enable` | flip-flop |
| `reg_soft_reset_sys_sync` | `synchronizer soft_reset` | flip-flop |
| `reg_vc_exposure_map_sync` | `synchronizer vc_exposure_map` | flip-flop |
| `reg_axis_enable_sync` | `synchronizer axis_enable` | flip-flop |
| `reg_stream_format_sync` | `synchronizer stream_format` | flip-flop |
| `reg_stream_follow_dma_drop_sync` | `synchronizer stream_follow_dma_drop` | flip-flop |
| `reg_tpg_enable_sync` | `synchronizer tpg_enable` | flip-flop |
| `reg_dma_enable_sync` | `synchronizer dma_enable` | flip-flop |
| `reg_dma_start_sync` | `synchronizer dma_start` | flip-flop |
| `reg_dma_abort_sync` | `synchronizer dma_abort` | flip-flop |
| `reg_dma_format_sync` | `synchronizer dma_format` | flip-flop |
| `reg_err_frame_policy_sync` | `synchronizer err_frame_policy` | flip-flop |
| `reg_global_irq_enable_sync` | `synchronizer global_irq_enable` | flip-flop |
| `reg_seq_busy_sync` | `synchronizer seq_busy -> phy_rst_in_progress` | flip-flop |
| `reg_soft_reset_byte_sync` | `synchronizer soft_reset` | flip-flop |
| `reg_cl_enable` | `cl_enable (phy_en_*)` | flip-flop |
| `reg_dl_enable` | `dl_enable (phy_en_*)` | flip-flop |
| `reg_forcerxmode` | `forcerxmode (phy_en_*)` | flip-flop |
| `reg_cl_stopstate_sync` | `synchronizer cl_stopstate` | flip-flop |
| `reg_cl_ulpsactivenot_sync` | `synchronizer cl_ulpsactivenot` | flip-flop |
| `reg_cl_rxclkactivehs_sync` | `synchronizer cl_rxclkactivehs` | flip-flop |
| `reg_cl_ulpsclknot_sync` | `synchronizer cl_ulpsclknot` | flip-flop |
| `reg_init_done_sync` | `synchronizer init_done` | flip-flop |
| `reg_stopstate_sync` | `synchronizer stopstate[3:0]` | flip-flop |
| `reg_rxskewcalhs_sync` | `synchronizer rxskewcalhs[3:0]` | flip-flop |
| `reg_linebuffer_full_sync` | `synchronizer linebuffer_full` | flip-flop |
| `reg_sp_fifo_not_empty_sync` | `synchronizer sp_fifo_not_empty` | flip-flop |
| `reg_sp_fifo_full_sync` | `synchronizer sp_fifo_full` | flip-flop |
| `reg_null_blank_seen_sync` | `synchronizer null_blank_seen` | flip-flop |
| `reg_active_sel_sync` | `synchronizer active_sel` | flip-flop |
| `reg_switch_pending_sync` | `synchronizer switch_pending` | flip-flop |
| `reg_dt_conflict_sync` | `synchronizer dt_conflict` | flip-flop |
| `reg_vc_unsupported_seen_sync` | `synchronizer vc_unsupported_seen` | flip-flop |
| `reg_linebuf_ovf_sticky_sync` | `synchronizer linebuf_ovf_sticky` | flip-flop |
| `reg_sp_fifo_ovf_sticky_sync` | `synchronizer sp_fifo_ovf_sticky` | flip-flop |
| `reg_meta_overflow_sync` | `synchronizer meta_overflow` | flip-flop |
| `reg_dma_ovf_sticky_sync` | `synchronizer dma_ovf_sticky` | flip-flop |
| `reg_irq_status_sync` | `synchronizer irq_status[23:0]` | flip-flop |
| `w_irq_status_l` | `irq_status_l[23:0]` | combinational |
| `reg_errsoths_tgl` | `toggle flop errsoths_x` | flip-flop |
| `reg_errsotsynchs_tgl` | `toggle flop errsotsynchs_x` | flip-flop |
| `reg_errsoths_sync` | `synchronizer toggle errsoths_x` | flip-flop |
| `reg_errsotsynchs_sync` | `synchronizer toggle errsotsynchs_x` | flip-flop |
| `reg_errsoths_prev` | `edge detector errsoths_x` | flip-flop |
| `reg_errsotsynchs_prev` | `edge detector errsotsynchs_x` | flip-flop |
| `reg_sot_err` | `laneX_sot_error` | flip-flop |
| `reg_sot_sync_err` | `laneX_sot_sync_error` | flip-flop |
| `w_st_sys_src` | `st_sys_*` | combinational |
| `reg_snp_sys_cap` | - | flip-flop |
| `reg_st_l_sys` | `st_l_* (sys_clk part)` | flip-flop |
| `reg_snp_sys_started` | - | flip-flop |
| `reg_snp_sys_req_tgl` | - | flip-flop |
| `reg_snp_sys_ack_sync` | - | flip-flop |
| `reg_snp_sys_req_sync` | - | flip-flop |
| `reg_snp_sys_ack_tgl` | - | flip-flop |
| `w_snp_sys_done` | - | combinational |
| `w_snp_sys_launch` | - | combinational |
| `w_st_byte_src` | `st_byte_*` | combinational |
| `reg_snp_byte_cap` | - | flip-flop |
| `reg_st_l_byte` | `st_l_* (rxbyteclkhs part)` | flip-flop |
| `reg_snp_byte_started` | - | flip-flop |
| `reg_snp_byte_req_tgl` | - | flip-flop |
| `reg_snp_byte_ack_sync` | - | flip-flop |
| `reg_snp_byte_req_sync` | - | flip-flop |
| `reg_snp_byte_ack_tgl` | - | flip-flop |
| `w_snp_byte_done` | - | combinational |
| `w_snp_byte_launch` | - | combinational |
| `reg_resetn_sync` | `resetn_q` | flip-flop |
| `reg_resetn_d` | `resetn_q_d` | flip-flop |
| `w_resetn_q` | `resetn_q` | combinational |
| `w_rst_trig` | `rst_trig` | combinational |
| `w_rst_cnt_gt1` | `rst_cnt > 1` | combinational |
| `reg_rst_cnt` | `rst_cnt` | flip-flop |
| `reg_phy_core_rst` | `phy_core_rst / seq_busy` | flip-flop |
| `reg_full_reset_req` | `full_reset_req` | flip-flop |
| `f_wr_bytes` | - | function |
| `f_hs_ofs` | - | function |
| `f_hs_rst` | - | function |
| `gen_para_check_lane` | - | generate block |
| `gen_para_check_extract` | - | generate block |
| `gen_para_check_dma` | - | generate block |
| `gen_para_check_cdc` | - | generate block |
| `gen_para_check_count` | - | generate block |
| `gen_emb_cfg` | - | generate block |
| `gen_hs_src` | - | generate block |
| `gen_hs` | - | generate block |
| `gen_emb_out` | - | generate block |
| `gen_built` | - | generate block |
| `gen_none` | - | generate block |
