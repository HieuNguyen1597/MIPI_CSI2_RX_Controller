# Signal mapping — TOP MIPI CSI-2 RX Controller

## Notes

- RTL modules: `m_fpt_csi2rx_core` (`00_src/rtl/m_fpt_csi2rx_core.sv`), which connects the 14 blocks, the asynchronous FIFO wrapper and the reset synchronisers, and the FTEL wrapper `m_fpt_csi2rx_wrap` (`00_src/rtl/m_fpt_csi2rx_wrap.sv`), which only renames the ports.
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- `m_fpt_csi2rx_wrap` has the same parameters as `m_fpt_csi2rx_core` and passes each one to `u_csi2rx_core_0` under the same name; they are listed once, in the core table.
- Instance `u_<block>_0` is the block whose module is `m_fpt_<block>`.
- Net `w_bNN_<port>` is driven by port `o_<port>` of Block No. NN, and `w_async_<port>` by the asynchronous FIFO wrapper. A combinational `w_bNN_<port>` instead feeds input `i_<port>` of Block No. NN and is assembled from other nets (lane vectors, one bit of a vector).
- In a spec name, `_x` / `_0` … `_3` is the data lane index; the lane ports keep four lanes for every `MAX_LANES`.
- The wrapper ports follow the FTEL wrapper naming (`i_csi2rx_…`, `o_csi2rx_…`, with `axis`, `axi` or `axil` for a bus port); their spec name is the port name of the core in the specs.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping — m_fpt_csi2rx_core

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_csi2rx_core` | `TOP of the MIPI CSI-2 RX Controller (core)` | module |
| `i_rxbyteclkhs` | `rxbyteclkhs` | input port |
| `i_sys_clk` | `sys_clk` | input port |
| `i_mm_aclk` | `mm_aclk` | input port |
| `i_lite_aclk` | `lite_aclk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_lite_aresetn` | `lite_aresetn` | input port |
| `i_cl_stopstate` | `cl_stopstate` | input port |
| `i_stopstate_0` | `stopstate_x` | input port |
| `i_stopstate_1` | `stopstate_x` | input port |
| `i_stopstate_2` | `stopstate_x` | input port |
| `i_stopstate_3` | `stopstate_x` | input port |
| `i_cl_ulpsactivenot` | `cl_ulpsactivenot` | input port |
| `i_ulpsactivenot_0` | `ulpsactivenot_x` | input port |
| `i_ulpsactivenot_1` | `ulpsactivenot_x` | input port |
| `i_ulpsactivenot_2` | `ulpsactivenot_x` | input port |
| `i_ulpsactivenot_3` | `ulpsactivenot_x` | input port |
| `i_cl_rxclkactivehs` | `cl_rxclkactivehs` | input port |
| `i_cl_ulpsclknot` | `cl_ulpsclknot` | input port |
| `i_rxdatahs_0` | `rxdatahs_x` | input port |
| `i_rxdatahs_1` | `rxdatahs_x` | input port |
| `i_rxdatahs_2` | `rxdatahs_x` | input port |
| `i_rxdatahs_3` | `rxdatahs_x` | input port |
| `i_rxvalidhs_0` | `rxvalidhs_x` | input port |
| `i_rxvalidhs_1` | `rxvalidhs_x` | input port |
| `i_rxvalidhs_2` | `rxvalidhs_x` | input port |
| `i_rxvalidhs_3` | `rxvalidhs_x` | input port |
| `i_rxactivehs_0` | `rxactivehs_x` | input port |
| `i_rxactivehs_1` | `rxactivehs_x` | input port |
| `i_rxactivehs_2` | `rxactivehs_x` | input port |
| `i_rxactivehs_3` | `rxactivehs_x` | input port |
| `i_rxsynchs_0` | `rxsynchs_x` | input port |
| `i_rxsynchs_1` | `rxsynchs_x` | input port |
| `i_rxsynchs_2` | `rxsynchs_x` | input port |
| `i_rxsynchs_3` | `rxsynchs_x` | input port |
| `i_rxskewcalhs_0` | `rxskewcalhs_x` | input port |
| `i_rxskewcalhs_1` | `rxskewcalhs_x` | input port |
| `i_rxskewcalhs_2` | `rxskewcalhs_x` | input port |
| `i_rxskewcalhs_3` | `rxskewcalhs_x` | input port |
| `i_rxclkesc_0` | `rxclkesc_x` | input port |
| `i_rxclkesc_1` | `rxclkesc_x` | input port |
| `i_rxclkesc_2` | `rxclkesc_x` | input port |
| `i_rxclkesc_3` | `rxclkesc_x` | input port |
| `i_rxlpdtesc_0` | `rxlpdtesc_x` | input port |
| `i_rxlpdtesc_1` | `rxlpdtesc_x` | input port |
| `i_rxlpdtesc_2` | `rxlpdtesc_x` | input port |
| `i_rxlpdtesc_3` | `rxlpdtesc_x` | input port |
| `i_rxulpsesc_0` | `rxulpsesc_x` | input port |
| `i_rxulpsesc_1` | `rxulpsesc_x` | input port |
| `i_rxulpsesc_2` | `rxulpsesc_x` | input port |
| `i_rxulpsesc_3` | `rxulpsesc_x` | input port |
| `i_rxtriggeresc_0` | `rxtriggeresc_x` | input port |
| `i_rxtriggeresc_1` | `rxtriggeresc_x` | input port |
| `i_rxtriggeresc_2` | `rxtriggeresc_x` | input port |
| `i_rxtriggeresc_3` | `rxtriggeresc_x` | input port |
| `i_rxdataesc_0` | `rxdataesc_x` | input port |
| `i_rxdataesc_1` | `rxdataesc_x` | input port |
| `i_rxdataesc_2` | `rxdataesc_x` | input port |
| `i_rxdataesc_3` | `rxdataesc_x` | input port |
| `i_rxvalidesc_0` | `rxvalidesc_x` | input port |
| `i_rxvalidesc_1` | `rxvalidesc_x` | input port |
| `i_rxvalidesc_2` | `rxvalidesc_x` | input port |
| `i_rxvalidesc_3` | `rxvalidesc_x` | input port |
| `i_errsoths_0` | `errsoths_x` | input port |
| `i_errsoths_1` | `errsoths_x` | input port |
| `i_errsoths_2` | `errsoths_x` | input port |
| `i_errsoths_3` | `errsoths_x` | input port |
| `i_errsotsynchs_0` | `errsotsynchs_x` | input port |
| `i_errsotsynchs_1` | `errsotsynchs_x` | input port |
| `i_errsotsynchs_2` | `errsotsynchs_x` | input port |
| `i_errsotsynchs_3` | `errsotsynchs_x` | input port |
| `i_erresc_0` | `erresc_x` | input port |
| `i_erresc_1` | `erresc_x` | input port |
| `i_erresc_2` | `erresc_x` | input port |
| `i_erresc_3` | `erresc_x` | input port |
| `i_errsyncesc_0` | `errsyncesc_x` | input port |
| `i_errsyncesc_1` | `errsyncesc_x` | input port |
| `i_errsyncesc_2` | `errsyncesc_x` | input port |
| `i_errsyncesc_3` | `errsyncesc_x` | input port |
| `i_errcontrol_0` | `errcontrol_x` | input port |
| `i_errcontrol_1` | `errcontrol_x` | input port |
| `i_errcontrol_2` | `errcontrol_x` | input port |
| `i_errcontrol_3` | `errcontrol_x` | input port |
| `i_init_done` | `init_done` | input port |
| `o_cl_enable` | `cl_enable` | output port |
| `o_dl_enable_0` | `dl_enable_x` | output port |
| `o_dl_enable_1` | `dl_enable_x` | output port |
| `o_dl_enable_2` | `dl_enable_x` | output port |
| `o_dl_enable_3` | `dl_enable_x` | output port |
| `o_forcerxmode_0` | `forcerxmode_x` | output port |
| `o_forcerxmode_1` | `forcerxmode_x` | output port |
| `o_forcerxmode_2` | `forcerxmode_x` | output port |
| `o_forcerxmode_3` | `forcerxmode_x` | output port |
| `o_phy_core_rst` | `phy_core_rst` | output port |
| `o_m_axis_tdata` | `m_axis_tdata` | output port |
| `o_m_axis_tvalid` | `m_axis_tvalid` | output port |
| `i_m_axis_tready` | `m_axis_tready` | input port |
| `o_m_axis_tlast` | `m_axis_tlast` | output port |
| `o_m_axis_tuser` | `m_axis_tuser` | output port |
| `o_m_axis_tid` | `m_axis_tid` | output port |
| `o_m_axis_tkeep` | `m_axis_tkeep` | output port |
| `o_m_axi4_awaddr` | `m_axi4_awaddr` | output port |
| `o_m_axi4_awid` | `m_axi4_awid` | output port |
| `o_m_axi4_awlen` | `m_axi4_awlen` | output port |
| `o_m_axi4_awburst` | `m_axi4_awburst` | output port |
| `o_m_axi4_awlock` | `m_axi4_awlock` | output port |
| `o_m_axi4_awsize` | `m_axi4_awsize` | output port |
| `o_m_axi4_awcache` | `m_axi4_awcache` | output port |
| `o_m_axi4_awprot` | `m_axi4_awprot` | output port |
| `o_m_axi4_awqos` | `m_axi4_awqos` | output port |
| `i_m_axi4_awready` | `m_axi4_awready` | input port |
| `o_m_axi4_awvalid` | `m_axi4_awvalid` | output port |
| `o_m_axi4_wdata` | `m_axi4_wdata` | output port |
| `o_m_axi4_wlast` | `m_axi4_wlast` | output port |
| `o_m_axi4_wstrb` | `m_axi4_wstrb` | output port |
| `i_m_axi4_wready` | `m_axi4_wready` | input port |
| `o_m_axi4_wvalid` | `m_axi4_wvalid` | output port |
| `i_m_axi4_bid` | `m_axi4_bid` | input port |
| `i_m_axi4_bresp` | `m_axi4_bresp` | input port |
| `i_m_axi4_bvalid` | `m_axi4_bvalid` | input port |
| `o_m_axi4_bready` | `m_axi4_bready` | output port |
| `i_s_axi_awaddr` | `s_axi_awaddr` | input port |
| `i_s_axi_awprot` | `s_axi_awprot` | input port |
| `i_s_axi_awvalid` | `s_axi_awvalid` | input port |
| `o_s_axi_awready` | `s_axi_awready` | output port |
| `i_s_axi_wdata` | `s_axi_wdata` | input port |
| `i_s_axi_wstrb` | `s_axi_wstrb` | input port |
| `i_s_axi_wvalid` | `s_axi_wvalid` | input port |
| `o_s_axi_wready` | `s_axi_wready` | output port |
| `i_s_axi_bready` | `s_axi_bready` | input port |
| `o_s_axi_bresp` | `s_axi_bresp` | output port |
| `o_s_axi_bvalid` | `s_axi_bvalid` | output port |
| `i_s_axi_araddr` | `s_axi_araddr` | input port |
| `i_s_axi_arprot` | `s_axi_arprot` | input port |
| `i_s_axi_arvalid` | `s_axi_arvalid` | input port |
| `o_s_axi_arready` | `s_axi_arready` | output port |
| `i_s_axi_rready` | `s_axi_rready` | input port |
| `o_s_axi_rdata` | `s_axi_rdata` | output port |
| `o_s_axi_rresp` | `s_axi_rresp` | output port |
| `o_s_axi_rvalid` | `s_axi_rvalid` | output port |
| `o_csi2_irq` | `csi2_irq` | output port |
| `PARA_MAX_LANES` | `MAX_LANES` | parameter |
| `PARA_SKEW_DEPTH` | `SKEW_DEPTH` | parameter |
| `PARA_FIFO_AF_MARGIN` | `FIFO_AF_MARGIN` | parameter |
| `PARA_ASYNC_FIFO_DEPTH` | `DEPTH (Async FIFO)` | parameter |
| `PARA_SP_FIFO_DEPTH` | `SP_FIFO_DEPTH` | parameter |
| `PARA_ECC_LATENCY` | `ECC_LATENCY` | parameter |
| `PARA_LINE_BUFFER_DEPTH` | `LINE_BUFFER_DEPTH` | parameter |
| `PARA_FULL_WATERMARK` | `FULL_WATERMARK` | parameter |
| `PARA_STREAM_FIFO_DEPTH` | `STREAM_FIFO_DEPTH` | parameter |
| `PARA_DMA_FIFO_DEPTH` | `DMA_FIFO_DEPTH` | parameter |
| `PARA_TS_INCR` | `TS_INCR` | parameter |
| `PARA_N_EXTRACT` | `N_EXTRACT` | parameter |
| `PARA_META_FIFO_DEPTH` | `META_FIFO_DEPTH` | parameter |
| `PARA_SYNC_STAGES` | `SYNC_STAGES` | parameter |
| `PARA_RST_TAIL` | `RST_TAIL` | parameter |
| `PARA_AXI_DATA_WIDTH` | `AXI_DATA_WIDTH` | parameter |
| `PARA_AXI_ADDR_WIDTH` | `AXI_ADDR_WIDTH` | parameter |
| `PARA_AXI_ID_WIDTH` | `AXI_ID_WIDTH` | parameter |
| `PARA_MAX_BURST_BEATS` | `MAX_BURST_BEATS` | parameter |
| `PARA_MAX_OUTSTANDING` | `MAX_OUTSTANDING` | parameter |
| `PARA_RAW_FIFO_DEPTH` | `RAW_FIFO_DEPTH` | parameter |
| `PARA_META_DATA_FIFO_DEPTH` | `META_DATA_FIFO_DEPTH` | parameter |
| `PARA_CMD_FIFO_DEPTH` | `CMD_FIFO_DEPTH` | parameter |
| `PARA_META_MAX_BURST_BEATS` | `META_MAX_BURST_BEATS` | parameter |
| `PARA_ASYNC_MM_CLK` | `ASYNC_MM_CLK` | parameter |
| `PARA_PHY_RST_CYCLES` | `PHY_RST_CYCLES` | parameter |
| `PARA_IRQ_CLR_TIMEOUT` | `IRQ_CLR_TIMEOUT` | parameter |
| `LPARA_FILL_CNT_W` | `FILL_CNT_WIDTH` | localparam |
| `LPARA_N_EXTRACT_SLOTS` | - | localparam |
| `gen_guard_sync_stages` | - | generate block |
| `gen_guard_fill_cnt_w` | - | generate block |
| `u_ppi_0` | `PHY Protocol Interface (PPI)` | instance |
| `u_lane_management_0` | `Lane Management` | instance |
| `u_async_fifo_0` | `Async FIFO` | instance |
| `u_packet_analyzer_0` | `Packet Analyzer` | instance |
| `u_byte_to_pixel_0` | `Byte-to-Pixel Unpacking` | instance |
| `u_tpg_0` | `Test Pattern Generator` | instance |
| `u_tpg_mux_0` | `TPG Mux` | instance |
| `u_line_buffer_0` | `Line Buffer` | instance |
| `u_format_bridge_0` | `Format Bridge` | instance |
| `u_frame_context_metadata_0` | `Frame Context & Metadata Capture` | instance |
| `u_control_fsm_0` | `Control FSM` | instance |
| `u_irq_aggregator_0` | `Interrupt & Error Aggregator` | instance |
| `u_axi4_stream_0` | `AXI4-Stream Master` | instance |
| `u_dma_0` | `Direct Memory Access (DMA)` | instance |
| `u_csr_axi4lite_0` | `CSR & AXI4-Lite Subordinate` | instance |
| `w_rst_async_n` | `resetn AND NOT full_reset_req` | combinational |
| `reg_rxbyteclkhs_rst_sync` | - | flip-flop |
| `reg_sys_clk_rst_sync` | - | flip-flop |
| `reg_mm_aclk_rst_sync` | - | flip-flop |
| `w_rxbyteclkhs_resetn` | `resetn (rxbyteclkhs domain)` | combinational |
| `w_sys_resetn` | `resetn (sys_clk domain)` | combinational |
| `w_mm_resetn` | `resetn (mm_aclk domain)` | combinational |
| `w_b01_rxbyteclkhs` | `rxbyteclkhs` | net |
| `w_b01_errsoths_0` | `errsoths_0` | net |
| `w_b01_errsoths_1` | `errsoths_1` | net |
| `w_b01_errsoths_2` | `errsoths_2` | net |
| `w_b01_errsoths_3` | `errsoths_3` | net |
| `w_b01_errsotsynchs_0` | `errsotsynchs_0` | net |
| `w_b01_errsotsynchs_1` | `errsotsynchs_1` | net |
| `w_b01_errsotsynchs_2` | `errsotsynchs_2` | net |
| `w_b01_errsotsynchs_3` | `errsotsynchs_3` | net |
| `w_b01_rxactivehs_0` | `rxactivehs_0` | net |
| `w_b01_rxactivehs_1` | `rxactivehs_1` | net |
| `w_b01_rxactivehs_2` | `rxactivehs_2` | net |
| `w_b01_rxactivehs_3` | `rxactivehs_3` | net |
| `w_b01_rxdatahs_0` | `rxdatahs_0` | net |
| `w_b01_rxdatahs_1` | `rxdatahs_1` | net |
| `w_b01_rxdatahs_2` | `rxdatahs_2` | net |
| `w_b01_rxdatahs_3` | `rxdatahs_3` | net |
| `w_b01_rxskewcalhs_0` | `rxskewcalhs_0` | net |
| `w_b01_rxskewcalhs_1` | `rxskewcalhs_1` | net |
| `w_b01_rxskewcalhs_2` | `rxskewcalhs_2` | net |
| `w_b01_rxskewcalhs_3` | `rxskewcalhs_3` | net |
| `w_b01_rxsynchs_0` | `rxsynchs_0` | net |
| `w_b01_rxsynchs_1` | `rxsynchs_1` | net |
| `w_b01_rxsynchs_2` | `rxsynchs_2` | net |
| `w_b01_rxsynchs_3` | `rxsynchs_3` | net |
| `w_b01_rxvalidhs_0` | `rxvalidhs_0` | net |
| `w_b01_rxvalidhs_1` | `rxvalidhs_1` | net |
| `w_b01_rxvalidhs_2` | `rxvalidhs_2` | net |
| `w_b01_rxvalidhs_3` | `rxvalidhs_3` | net |
| `w_b01_cl_stopstate` | `cl_stopstate` | net |
| `w_b01_cl_rxclkactivehs` | `cl_rxclkactivehs` | net |
| `w_b01_cl_ulpsactivenot` | `cl_ulpsactivenot` | net |
| `w_b01_cl_ulpsclknot` | `cl_ulpsclknot` | net |
| `w_b01_init_done` | `init_done` | net |
| `w_b01_stopstate_0` | `stopstate_0` | net |
| `w_b01_stopstate_1` | `stopstate_1` | net |
| `w_b01_stopstate_2` | `stopstate_2` | net |
| `w_b01_stopstate_3` | `stopstate_3` | net |
| `w_b02_merged_byte_valid` | `merged_byte_valid` | net |
| `w_b02_merged_data` | `merged_data` | net |
| `w_b02_merged_eob` | `merged_eob` | net |
| `w_b02_merged_sob` | `merged_sob` | net |
| `w_b02_merged_valid` | `merged_valid` | net |
| `w_b02_lane_abort_pulse` | `lane_abort_pulse` | net |
| `w_b02_abort_fifo_full` | `abort_fifo_full` | net |
| `w_b02_abort_lane_sync_seen` | `abort_lane_sync_seen` | net |
| `w_b02_abort_sync_err` | `abort_sync_err` | net |
| `w_b02_abort_unarmed` | `abort_unarmed` | net |
| `w_b02_active_lanes_applied` | `active_lanes_applied` | net |
| `w_b02_deskew_burst_seen` | `deskew_burst_seen` | net |
| `w_b02_lm_fsm_state` | `lm_fsm_state` | net |
| `w_async_fifo_rd_data` | `rd_data` | net |
| `w_async_fifo_rd_empty` | `rd_empty` | net |
| `w_b03_rd_en` | `rd_en` | net |
| `w_b03_bp_byte_en` | `bp_byte_en` | net |
| `w_b03_bp_data` | `bp_data` | net |
| `w_b03_bp_dt_sel` | `bp_dt_sel` | net |
| `w_b03_bp_eol` | `bp_eol` | net |
| `w_b03_bp_sof` | `bp_sof` | net |
| `w_b03_bp_sol` | `bp_sol` | net |
| `w_b03_bp_valid` | `bp_valid` | net |
| `w_b03_bp_vc_tag` | `bp_vc_tag` | net |
| `w_b03_fc_fe` | `fc_fe` | net |
| `w_b03_fc_fs` | `fc_fs` | net |
| `w_b03_fc_vc` | `fc_vc` | net |
| `w_b03_emb_byte_en` | `emb_byte_en` | net |
| `w_b03_emb_data` | `emb_data` | net |
| `w_b03_emb_eol` | `emb_eol` | net |
| `w_b03_emb_sol` | `emb_sol` | net |
| `w_b03_emb_valid` | `emb_valid` | net |
| `w_b03_emb_vc` | `emb_vc` | net |
| `w_b03_fc_frame_num` | `fc_frame_num` | net |
| `w_b03_err_crc` | `err_crc` | net |
| `w_b03_err_ecc_2bit` | `err_ecc_2bit` | net |
| `w_b03_err_lane_abort` | `err_lane_abort` | net |
| `w_b03_err_vc` | `err_vc` | net |
| `w_b03_err_wc` | `err_wc` | net |
| `w_b03_err_ecc_1bit` | `err_ecc_1bit` | net |
| `w_b03_err_frame_level` | `err_frame_level` | net |
| `w_b03_err_frame_sync` | `err_frame_sync` | net |
| `w_b03_err_unsupported_dt` | `err_unsupported_dt` | net |
| `w_b03_packet_done_pulse` | `packet_done_pulse` | net |
| `w_b03_sp_fifo_overflow` | `sp_fifo_overflow` | net |
| `w_b03_dt_conflict` | `dt_conflict` | net |
| `w_b03_gsp_data` | `gsp_data` | net |
| `w_b03_null_blank_seen` | `null_blank_seen` | net |
| `w_b03_sp_fifo_full` | `sp_fifo_full` | net |
| `w_b03_sp_fifo_not_empty` | `sp_fifo_not_empty` | net |
| `w_b03_vc_unsupported_seen` | `vc_unsupported_seen` | net |
| `w_b04_err_wc_align` | `err_wc_align` | net |
| `w_b04_px_data` | `px_data` | net |
| `w_b04_px_dt_sel` | `px_dt_sel` | net |
| `w_b04_px_eol` | `px_eol` | net |
| `w_b04_px_sof` | `px_sof` | net |
| `w_b04_px_sol` | `px_sol` | net |
| `w_b04_px_valid` | `px_valid` | net |
| `w_b04_px_valid_count` | `px_valid_count` | net |
| `w_b04_px_vc_tag` | `px_vc_tag` | net |
| `w_b05_px_data` | `px_data` | net |
| `w_b05_px_dt_sel` | `px_dt_sel` | net |
| `w_b05_px_eol` | `px_eol` | net |
| `w_b05_px_sof` | `px_sof` | net |
| `w_b05_px_sol` | `px_sol` | net |
| `w_b05_px_valid` | `px_valid` | net |
| `w_b05_px_vc_tag` | `px_vc_tag` | net |
| `w_b05_tpg_eof` | `tpg_eof` | net |
| `w_b05_tpg_fs` | `tpg_fs` | net |
| `w_b05_tpg_fe` | `tpg_fe` | net |
| `w_b05_tpg_fvc` | `tpg_fvc` | net |
| `w_b05_tpg_height_shadow` | `tpg_height_shadow` | net |
| `w_b06_tpg_px_ready` | `tpg_px_ready` | net |
| `w_b06_err_wc_align` | `err_wc_align` | net |
| `w_b06_px_data` | `px_data` | net |
| `w_b06_px_dt_sel` | `px_dt_sel` | net |
| `w_b06_px_eol` | `px_eol` | net |
| `w_b06_px_sof` | `px_sof` | net |
| `w_b06_px_sol` | `px_sol` | net |
| `w_b06_px_valid` | `px_valid` | net |
| `w_b06_px_valid_count` | `px_valid_count` | net |
| `w_b06_px_vc_tag` | `px_vc_tag` | net |
| `w_b06_active_sel` | `active_sel` | net |
| `w_b06_switch_pending` | `switch_pending` | net |
| `w_b07_linebuf_empty` | `linebuf_empty` | net |
| `w_b07_linebuffer_full` | `linebuffer_full` | net |
| `w_b07_fb_err_wc_align` | `fb_err_wc_align` | net |
| `w_b07_fb_px_data` | `fb_px_data` | net |
| `w_b07_fb_px_dt_sel` | `fb_px_dt_sel` | net |
| `w_b07_fb_px_eol` | `fb_px_eol` | net |
| `w_b07_fb_px_sof` | `fb_px_sof` | net |
| `w_b07_fb_px_sol` | `fb_px_sol` | net |
| `w_b07_fb_px_valid` | `fb_px_valid` | net |
| `w_b07_fb_px_valid_count` | `fb_px_valid_count` | net |
| `w_b07_fb_px_vc_tag` | `fb_px_vc_tag` | net |
| `w_b07_linebuf_overflow` | `linebuf_overflow` | net |
| `w_b07_linebuf_fill` | `linebuf_fill` | net |
| `w_b08_fb_ready` | `fb_ready` | net |
| `w_b08_fb_overflow` | `fb_overflow` | net |
| `w_b08_fbs_byte_en` | `fbs_byte_en` | net |
| `w_b08_fbs_data` | `fbs_data` | net |
| `w_b08_fbs_eol` | `fbs_eol` | net |
| `w_b08_fbs_err` | `fbs_err` | net |
| `w_b08_fbs_sof` | `fbs_sof` | net |
| `w_b08_fbs_valid` | `fbs_valid` | net |
| `w_b08_fbs_vc_tag` | `fbs_vc_tag` | net |
| `w_b08_fbd_byte_en` | `fbd_byte_en` | net |
| `w_b08_fbd_data` | `fbd_data` | net |
| `w_b08_fbd_eol` | `fbd_eol` | net |
| `w_b08_fbd_err` | `fbd_err` | net |
| `w_b08_fbd_sof` | `fbd_sof` | net |
| `w_b08_fbd_sol` | `fbd_sol` | net |
| `w_b08_fbd_valid` | `fbd_valid` | net |
| `w_b08_fbd_vc_tag` | `fbd_vc_tag` | net |
| `w_b09_capture_dropped` | `capture_dropped` | net |
| `w_b09_capture_end` | `capture_end` | net |
| `w_b09_capture_start` | `capture_start` | net |
| `w_b09_ev_fs` | `ev_fs` | net |
| `w_b09_ev_vc` | `ev_vc` | net |
| `w_b09_lead_vc` | `lead_vc` | net |
| `w_b09_capture_done` | `capture_done` | net |
| `w_b09_cur_frame_id_vc0` | `cur_frame_id_vcn` | net |
| `w_b09_cur_frame_id_vc1` | `cur_frame_id_vcn` | net |
| `w_b09_cur_line_vc0` | `cur_line_vcn` | net |
| `w_b09_cur_line_vc1` | `cur_line_vcn` | net |
| `w_b09_err_frame_num` | `err_frame_num` | net |
| `w_b09_err_resolution` | `err_resolution` | net |
| `w_b09_meta_overflow` | `meta_overflow` | net |
| `w_b09_ctx_orphan` | `ctx_orphan` | net |
| `w_b09_ctx_slot` | `ctx_slot` | net |
| `w_b09_frame_id_buf0` | `frame_id_buf0` | net |
| `w_b09_frame_id_buf1` | `frame_id_buf1` | net |
| `w_b09_meta_byte_en` | `meta_byte_en` | net |
| `w_b09_meta_data` | `meta_data` | net |
| `w_b09_meta_kind` | `meta_kind` | net |
| `w_b09_meta_last` | `meta_last` | net |
| `w_b09_meta_sof` | `meta_sof` | net |
| `w_b09_meta_valid` | `meta_valid` | net |
| `w_b09_meta_vc` | `meta_vc` | net |
| `w_b09_emb_extract_val_vc0` | `emb_extract_val_vcn` | net |
| `w_b09_emb_extract_val_vc1` | `emb_extract_val_vcn` | net |
| `w_b09_emb_line_count_vc0` | `emb_line_count_vcn` | net |
| `w_b09_emb_line_count_vc1` | `emb_line_count_vcn` | net |
| `w_b09_emb_word_count_vc0` | `emb_word_count_vcn` | net |
| `w_b09_emb_word_count_vc1` | `emb_word_count_vcn` | net |
| `w_b09_frame_count` | `frame_count` | net |
| `w_b09_timestamp` | `timestamp` | net |
| `w_b09_vc0_byte_count` | `vcn_byte_count` | net |
| `w_b09_vc1_byte_count` | `vcn_byte_count` | net |
| `w_b09_vc0_data_type` | `vcn_data_type` | net |
| `w_b09_vc1_data_type` | `vcn_data_type` | net |
| `w_b09_vc0_height_mismatch` | `vcn_height_mismatch` | net |
| `w_b09_vc1_height_mismatch` | `vcn_height_mismatch` | net |
| `w_b09_vc0_line_count` | `vcn_line_count` | net |
| `w_b09_vc1_line_count` | `vcn_line_count` | net |
| `w_b09_vc0_width_mismatch` | `vcn_width_mismatch` | net |
| `w_b09_vc1_width_mismatch` | `vcn_width_mismatch` | net |
| `w_b10_force_discard` | `force_discard` | net |
| `w_b10_rx_ctx_clear` | `rx_ctx_clear` | net |
| `w_b10_lb_flush` | `lb_flush` | net |
| `w_b10_fb_flush` | `fb_flush` | net |
| `w_b10_capture_drop` | `capture_drop` | net |
| `w_b10_drop_vc_mask` | `drop_vc_mask` | net |
| `w_b10_frame_drop_pulse` | `frame_drop_pulse` | net |
| `w_b10_asm_abort_mask` | `asm_abort_mask` | net |
| `w_b10_asm_abort_req` | `asm_abort_req` | net |
| `w_b10_asm_skip_mask` | `asm_skip_mask` | net |
| `w_b10_asm_skip_req` | `asm_skip_req` | net |
| `w_b10_dma_abort_req` | `dma_abort_req` | net |
| `w_b11_crc_err_cnt` | `crc_err_cnt` | net |
| `w_b11_dma_ovf_sticky` | `dma_ovf_sticky` | net |
| `w_b11_ecc_err_cnt` | `ecc_err_cnt` | net |
| `w_b11_error_capture` | `error_capture` | net |
| `w_b11_frame_drop_cnt` | `frame_drop_cnt` | net |
| `w_b11_irq_status` | `irq_status` | net |
| `w_b11_lane_abort_cnt` | `lane_abort_cnt` | net |
| `w_b11_linebuf_ovf_sticky` | `linebuf_ovf_sticky` | net |
| `w_b11_packet_cnt` | `packet_cnt` | net |
| `w_b11_sp_fifo_ovf_sticky` | `sp_fifo_ovf_sticky` | net |
| `w_b12_fbs_ready` | `fbs_ready` | net |
| `w_b13_fbd_ready` | `fbd_ready` | net |
| `w_b13_dma_buf_claim` | `dma_buf_claim` | net |
| `w_b13_dma_buf_index` | `dma_buf_index` | net |
| `w_b13_meta_ready` | `meta_ready` | net |
| `w_b13_dma_drop` | `dma_drop` | net |
| `w_b13_dma_axi_write_error` | `dma_axi_write_error` | net |
| `w_b13_dma_buf_done_err` | `dma_buf_done_err` | net |
| `w_b13_dma_buf_done_ok` | `dma_buf_done_ok` | net |
| `w_b13_dma_buffer_overflow` | `dma_buffer_overflow` | net |
| `w_b13_active_buffer` | `active_buffer` | net |
| `w_b13_axi_fault_awaddr` | `axi_fault_awaddr` | net |
| `w_b13_axi_fault_awid` | `axi_fault_awid` | net |
| `w_b13_axi_fault_resp` | `axi_fault_resp` | net |
| `w_b13_bresp_decerr_count` | `bresp_decerr_count` | net |
| `w_b13_bresp_slverr_count` | `bresp_slverr_count` | net |
| `w_b13_buf0_last_valid_line` | `buf0_last_valid_line` | net |
| `w_b13_buf0_state` | `buf0_state` | net |
| `w_b13_buf1_last_valid_line` | `buf1_last_valid_line` | net |
| `w_b13_buf1_state` | `buf1_state` | net |
| `w_b13_dma_busy` | `dma_busy` | net |
| `w_b14_soft_reset_byte` | `soft_reset` | net |
| `w_b14_soft_reset_sys` | `soft_reset` | net |
| `w_b14_full_reset_req` | `full_reset_req` | net |
| `w_b14_cl_enable` | `cl_enable` | net |
| `w_b14_dl_enable` | `dl_enable` | net |
| `w_b14_forcerxmode` | `forcerxmode` | net |
| `w_b14_phy_core_rst` | `phy_core_rst` | net |
| `w_b14_core_enable` | `core_enable` | net |
| `w_b14_active_lanes` | `active_lanes` | net |
| `w_b14_gsp_pop` | `gsp_pop` | net |
| `w_b14_metadata_capture_en` | `metadata_capture_en` | net |
| `w_b14_metadata_dt` | `metadata_dt` | net |
| `w_b14_tpg_dual_vc_en` | `tpg_dual_vc_en` | net |
| `w_b14_tpg_enable` | `tpg_enable` | net |
| `w_b14_tpg_hblank` | `tpg_hblank` | net |
| `w_b14_tpg_height` | `tpg_height` | net |
| `w_b14_tpg_pattern` | `tpg_pattern` | net |
| `w_b14_tpg_pixel_format` | `tpg_pixel_format` | net |
| `w_b14_tpg_vblank` | `tpg_vblank` | net |
| `w_b14_tpg_width` | `tpg_width` | net |
| `w_b14_axis_enable` | `axis_enable` | net |
| `w_b14_dma_enable` | `dma_enable` | net |
| `w_b14_dma_format` | `dma_format` | net |
| `w_b14_stream_format` | `stream_format` | net |
| `w_b14_emb_extract_cfg` | `emb_extract_cfg` | net |
| `w_b14_emb_extract_en` | `emb_extract_en` | net |
| `w_b14_expected_height` | `expected_height` | net |
| `w_b14_expected_width` | `expected_width` | net |
| `w_b14_fnum_check_en` | `fnum_check_en` | net |
| `w_b14_metadata_record_id` | `metadata_record_id` | net |
| `w_b14_vc_exposure_map` | `vc_exposure_map` | net |
| `w_b14_stream_follow_dma_drop` | `stream_follow_dma_drop` | net |
| `w_b14_global_irq_enable` | `global_irq_enable` | net |
| `w_b14_irq_clear_strobe` | `irq_clear_strobe` | net |
| `w_b14_irq_enable_mask` | `irq_enable_mask` | net |
| `w_b14_phy_rst_in_progress` | `phy_rst_in_progress` | net |
| `w_b14_awprot` | `awprot` | net |
| `w_b14_awqos` | `awqos` | net |
| `w_b14_bytes_per_line` | `bytes_per_line` | net |
| `w_b14_dma_abort` | `dma_abort` | net |
| `w_b14_dma_buffer0_addr` | `dma_buffer0_addr` | net |
| `w_b14_dma_buffer1_addr` | `dma_buffer1_addr` | net |
| `w_b14_dma_buffer_size` | `dma_buffer_size` | net |
| `w_b14_dma_start` | `dma_start` | net |
| `w_b14_err_frame_policy` | `err_frame_policy` | net |
| `w_b14_line_padding` | `line_padding` | net |
| `w_b14_max_burst_len` | `max_burst_len` | net |
| `w_b14_meta_area_size` | `meta_area_size` | net |
| `w_b14_release_buf` | `release_buf` | net |
| `w_b14_tpg_cfa_order` | `tpg_cfa_order` | net |
| `w_b11_errsoths` | `errsoths` | combinational |
| `w_b11_errsotsynchs` | `errsotsynchs` | combinational |
| `w_b14_errsoths` | `errsoths` | combinational |
| `w_b14_errsotsynchs` | `errsotsynchs` | combinational |
| `w_b14_rxskewcalhs` | `rxskewcalhs` | combinational |
| `w_b14_stopstate` | `stopstate` | combinational |
| `w_b01_dl_enable_0` | `dl_enable_0` | combinational |
| `w_b01_dl_enable_1` | `dl_enable_1` | combinational |
| `w_b01_dl_enable_2` | `dl_enable_2` | combinational |
| `w_b01_dl_enable_3` | `dl_enable_3` | combinational |
| `w_b01_forcerxmode_0` | `forcerxmode_0` | combinational |
| `w_b01_forcerxmode_1` | `forcerxmode_1` | combinational |
| `w_b01_forcerxmode_2` | `forcerxmode_2` | combinational |
| `w_b01_forcerxmode_3` | `forcerxmode_3` | combinational |
| `w_b14_linebuf_fill` | `linebuf_fill` | combinational |
| `gen_mm_rst_sync` | - | generate block |
| `gen_mm_rst_shared` | - | generate block |

## Mapping — m_fpt_csi2rx_wrap

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_csi2rx_wrap` | `TOP of the MIPI CSI-2 RX Controller (FTEL wrapper)` | module |
| `i_csi2rx_rxbyteclkhs` | `rxbyteclkhs` | input port |
| `i_csi2rx_sys_clk` | `sys_clk` | input port |
| `i_csi2rx_mm_aclk` | `mm_aclk` | input port |
| `i_csi2rx_lite_aclk` | `lite_aclk` | input port |
| `i_csi2rx_reset_n` | `resetn` | input port |
| `i_csi2rx_lite_areset_n` | `lite_aresetn` | input port |
| `i_csi2rx_cl_stopstate` | `cl_stopstate` | input port |
| `i_csi2rx_stopstate_0` | `stopstate_x` | input port |
| `i_csi2rx_stopstate_1` | `stopstate_x` | input port |
| `i_csi2rx_stopstate_2` | `stopstate_x` | input port |
| `i_csi2rx_stopstate_3` | `stopstate_x` | input port |
| `i_csi2rx_cl_ulpsactivenot` | `cl_ulpsactivenot` | input port |
| `i_csi2rx_ulpsactivenot_0` | `ulpsactivenot_x` | input port |
| `i_csi2rx_ulpsactivenot_1` | `ulpsactivenot_x` | input port |
| `i_csi2rx_ulpsactivenot_2` | `ulpsactivenot_x` | input port |
| `i_csi2rx_ulpsactivenot_3` | `ulpsactivenot_x` | input port |
| `i_csi2rx_cl_rxclkactivehs` | `cl_rxclkactivehs` | input port |
| `i_csi2rx_cl_ulpsclknot` | `cl_ulpsclknot` | input port |
| `i_csi2rx_rxdatahs_0` | `rxdatahs_x` | input port |
| `i_csi2rx_rxdatahs_1` | `rxdatahs_x` | input port |
| `i_csi2rx_rxdatahs_2` | `rxdatahs_x` | input port |
| `i_csi2rx_rxdatahs_3` | `rxdatahs_x` | input port |
| `i_csi2rx_rxvalidhs_0` | `rxvalidhs_x` | input port |
| `i_csi2rx_rxvalidhs_1` | `rxvalidhs_x` | input port |
| `i_csi2rx_rxvalidhs_2` | `rxvalidhs_x` | input port |
| `i_csi2rx_rxvalidhs_3` | `rxvalidhs_x` | input port |
| `i_csi2rx_rxactivehs_0` | `rxactivehs_x` | input port |
| `i_csi2rx_rxactivehs_1` | `rxactivehs_x` | input port |
| `i_csi2rx_rxactivehs_2` | `rxactivehs_x` | input port |
| `i_csi2rx_rxactivehs_3` | `rxactivehs_x` | input port |
| `i_csi2rx_rxsynchs_0` | `rxsynchs_x` | input port |
| `i_csi2rx_rxsynchs_1` | `rxsynchs_x` | input port |
| `i_csi2rx_rxsynchs_2` | `rxsynchs_x` | input port |
| `i_csi2rx_rxsynchs_3` | `rxsynchs_x` | input port |
| `i_csi2rx_rxskewcalhs_0` | `rxskewcalhs_x` | input port |
| `i_csi2rx_rxskewcalhs_1` | `rxskewcalhs_x` | input port |
| `i_csi2rx_rxskewcalhs_2` | `rxskewcalhs_x` | input port |
| `i_csi2rx_rxskewcalhs_3` | `rxskewcalhs_x` | input port |
| `i_csi2rx_rxclkesc_0` | `rxclkesc_x` | input port |
| `i_csi2rx_rxclkesc_1` | `rxclkesc_x` | input port |
| `i_csi2rx_rxclkesc_2` | `rxclkesc_x` | input port |
| `i_csi2rx_rxclkesc_3` | `rxclkesc_x` | input port |
| `i_csi2rx_rxlpdtesc_0` | `rxlpdtesc_x` | input port |
| `i_csi2rx_rxlpdtesc_1` | `rxlpdtesc_x` | input port |
| `i_csi2rx_rxlpdtesc_2` | `rxlpdtesc_x` | input port |
| `i_csi2rx_rxlpdtesc_3` | `rxlpdtesc_x` | input port |
| `i_csi2rx_rxulpsesc_0` | `rxulpsesc_x` | input port |
| `i_csi2rx_rxulpsesc_1` | `rxulpsesc_x` | input port |
| `i_csi2rx_rxulpsesc_2` | `rxulpsesc_x` | input port |
| `i_csi2rx_rxulpsesc_3` | `rxulpsesc_x` | input port |
| `i_csi2rx_rxtriggeresc_0` | `rxtriggeresc_x` | input port |
| `i_csi2rx_rxtriggeresc_1` | `rxtriggeresc_x` | input port |
| `i_csi2rx_rxtriggeresc_2` | `rxtriggeresc_x` | input port |
| `i_csi2rx_rxtriggeresc_3` | `rxtriggeresc_x` | input port |
| `i_csi2rx_rxdataesc_0` | `rxdataesc_x` | input port |
| `i_csi2rx_rxdataesc_1` | `rxdataesc_x` | input port |
| `i_csi2rx_rxdataesc_2` | `rxdataesc_x` | input port |
| `i_csi2rx_rxdataesc_3` | `rxdataesc_x` | input port |
| `i_csi2rx_rxvalidesc_0` | `rxvalidesc_x` | input port |
| `i_csi2rx_rxvalidesc_1` | `rxvalidesc_x` | input port |
| `i_csi2rx_rxvalidesc_2` | `rxvalidesc_x` | input port |
| `i_csi2rx_rxvalidesc_3` | `rxvalidesc_x` | input port |
| `i_csi2rx_errsoths_0` | `errsoths_x` | input port |
| `i_csi2rx_errsoths_1` | `errsoths_x` | input port |
| `i_csi2rx_errsoths_2` | `errsoths_x` | input port |
| `i_csi2rx_errsoths_3` | `errsoths_x` | input port |
| `i_csi2rx_errsotsynchs_0` | `errsotsynchs_x` | input port |
| `i_csi2rx_errsotsynchs_1` | `errsotsynchs_x` | input port |
| `i_csi2rx_errsotsynchs_2` | `errsotsynchs_x` | input port |
| `i_csi2rx_errsotsynchs_3` | `errsotsynchs_x` | input port |
| `i_csi2rx_erresc_0` | `erresc_x` | input port |
| `i_csi2rx_erresc_1` | `erresc_x` | input port |
| `i_csi2rx_erresc_2` | `erresc_x` | input port |
| `i_csi2rx_erresc_3` | `erresc_x` | input port |
| `i_csi2rx_errsyncesc_0` | `errsyncesc_x` | input port |
| `i_csi2rx_errsyncesc_1` | `errsyncesc_x` | input port |
| `i_csi2rx_errsyncesc_2` | `errsyncesc_x` | input port |
| `i_csi2rx_errsyncesc_3` | `errsyncesc_x` | input port |
| `i_csi2rx_errcontrol_0` | `errcontrol_x` | input port |
| `i_csi2rx_errcontrol_1` | `errcontrol_x` | input port |
| `i_csi2rx_errcontrol_2` | `errcontrol_x` | input port |
| `i_csi2rx_errcontrol_3` | `errcontrol_x` | input port |
| `i_csi2rx_init_done` | `init_done` | input port |
| `o_csi2rx_cl_enable` | `cl_enable` | output port |
| `o_csi2rx_dl_enable_0` | `dl_enable_x` | output port |
| `o_csi2rx_dl_enable_1` | `dl_enable_x` | output port |
| `o_csi2rx_dl_enable_2` | `dl_enable_x` | output port |
| `o_csi2rx_dl_enable_3` | `dl_enable_x` | output port |
| `o_csi2rx_forcerxmode_0` | `forcerxmode_x` | output port |
| `o_csi2rx_forcerxmode_1` | `forcerxmode_x` | output port |
| `o_csi2rx_forcerxmode_2` | `forcerxmode_x` | output port |
| `o_csi2rx_forcerxmode_3` | `forcerxmode_x` | output port |
| `o_csi2rx_phy_core_rst` | `phy_core_rst` | output port |
| `o_csi2rx_axis_tdata` | `m_axis_tdata` | output port |
| `o_csi2rx_axis_tvalid` | `m_axis_tvalid` | output port |
| `i_csi2rx_axis_tready` | `m_axis_tready` | input port |
| `o_csi2rx_axis_tlast` | `m_axis_tlast` | output port |
| `o_csi2rx_axis_tuser` | `m_axis_tuser` | output port |
| `o_csi2rx_axis_tid` | `m_axis_tid` | output port |
| `o_csi2rx_axis_tkeep` | `m_axis_tkeep` | output port |
| `o_csi2rx_axi_awaddr` | `m_axi4_awaddr` | output port |
| `o_csi2rx_axi_awid` | `m_axi4_awid` | output port |
| `o_csi2rx_axi_awlen` | `m_axi4_awlen` | output port |
| `o_csi2rx_axi_awburst` | `m_axi4_awburst` | output port |
| `o_csi2rx_axi_awlock` | `m_axi4_awlock` | output port |
| `o_csi2rx_axi_awsize` | `m_axi4_awsize` | output port |
| `o_csi2rx_axi_awcache` | `m_axi4_awcache` | output port |
| `o_csi2rx_axi_awprot` | `m_axi4_awprot` | output port |
| `o_csi2rx_axi_awqos` | `m_axi4_awqos` | output port |
| `i_csi2rx_axi_awready` | `m_axi4_awready` | input port |
| `o_csi2rx_axi_awvalid` | `m_axi4_awvalid` | output port |
| `o_csi2rx_axi_wdata` | `m_axi4_wdata` | output port |
| `o_csi2rx_axi_wlast` | `m_axi4_wlast` | output port |
| `o_csi2rx_axi_wstrb` | `m_axi4_wstrb` | output port |
| `i_csi2rx_axi_wready` | `m_axi4_wready` | input port |
| `o_csi2rx_axi_wvalid` | `m_axi4_wvalid` | output port |
| `i_csi2rx_axi_bid` | `m_axi4_bid` | input port |
| `i_csi2rx_axi_bresp` | `m_axi4_bresp` | input port |
| `i_csi2rx_axi_bvalid` | `m_axi4_bvalid` | input port |
| `o_csi2rx_axi_bready` | `m_axi4_bready` | output port |
| `i_csi2rx_axil_awaddr` | `s_axi_awaddr` | input port |
| `i_csi2rx_axil_awprot` | `s_axi_awprot` | input port |
| `i_csi2rx_axil_awvalid` | `s_axi_awvalid` | input port |
| `o_csi2rx_axil_awready` | `s_axi_awready` | output port |
| `i_csi2rx_axil_wdata` | `s_axi_wdata` | input port |
| `i_csi2rx_axil_wstrb` | `s_axi_wstrb` | input port |
| `i_csi2rx_axil_wvalid` | `s_axi_wvalid` | input port |
| `o_csi2rx_axil_wready` | `s_axi_wready` | output port |
| `i_csi2rx_axil_bready` | `s_axi_bready` | input port |
| `o_csi2rx_axil_bresp` | `s_axi_bresp` | output port |
| `o_csi2rx_axil_bvalid` | `s_axi_bvalid` | output port |
| `i_csi2rx_axil_araddr` | `s_axi_araddr` | input port |
| `i_csi2rx_axil_arprot` | `s_axi_arprot` | input port |
| `i_csi2rx_axil_arvalid` | `s_axi_arvalid` | input port |
| `o_csi2rx_axil_arready` | `s_axi_arready` | output port |
| `i_csi2rx_axil_rready` | `s_axi_rready` | input port |
| `o_csi2rx_axil_rdata` | `s_axi_rdata` | output port |
| `o_csi2rx_axil_rresp` | `s_axi_rresp` | output port |
| `o_csi2rx_axil_rvalid` | `s_axi_rvalid` | output port |
| `o_csi2rx_int_irq` | `csi2_irq` | output port |
| `u_csi2rx_core_0` | `TOP (core)` | instance |
