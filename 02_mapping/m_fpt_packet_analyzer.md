# Signal mapping — B03 Packet Analyzer

## Notes

- RTL module: `m_fpt_packet_analyzer` (`00_src/rtl/m_fpt_packet_analyzer.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- `gen_ecc_reg.` is the generate block built when `PARA_ECC_LATENCY` = 1 (registered ECC stage and Delay Register); `gen_ecc_comb` is built when `PARA_ECC_LATENCY` = 0.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_packet_analyzer` | Packet Analyzer (Block No. 3) | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_core_enable` | `core_enable` | input port |
| `i_metadata_dt` | `metadata_dt[5:0]` | input port |
| `i_metadata_capture_en` | `metadata_capture_en` | input port |
| `i_rd_data` | `rd_data[37:0]` | input port |
| `i_rd_empty` | `rd_empty` | input port |
| `o_rd_en` | `rd_en` | output port |
| `i_force_discard` | `force_discard[1:0]` | input port |
| `o_bp_data` | `bp_data[31:0]` | output port |
| `o_bp_byte_en` | `bp_byte_en[3:0]` | output port |
| `o_bp_valid` | `bp_valid` | output port |
| `o_bp_sof` | `bp_sof` | output port |
| `o_bp_sol` | `bp_sol` | output port |
| `o_bp_eol` | `bp_eol` | output port |
| `o_bp_vc_tag` | `bp_vc_tag[1:0]` | output port |
| `o_bp_dt_sel` | `bp_dt_sel` | output port |
| `o_emb_data` | `emb_data[31:0]` | output port |
| `o_emb_byte_en` | `emb_byte_en[3:0]` | output port |
| `o_emb_valid` | `emb_valid` | output port |
| `o_emb_sol` | `emb_sol` | output port |
| `o_emb_eol` | `emb_eol` | output port |
| `o_emb_vc` | `emb_vc[1:0]` | output port |
| `o_fc_fs` | `fc_fs` | output port |
| `o_fc_fe` | `fc_fe` | output port |
| `o_fc_frame_num` | `fc_frame_num[15:0]` | output port |
| `o_fc_vc` | `fc_vc[1:0]` | output port |
| `o_gsp_data` | `gsp_data[23:0]` | output port |
| `i_gsp_pop` | `gsp_pop` | input port |
| `o_sp_fifo_not_empty` | `sp_fifo_not_empty` | output port |
| `o_sp_fifo_full` | `sp_fifo_full` | output port |
| `o_sp_fifo_overflow` | `sp_fifo_overflow` | output port |
| `o_null_blank_seen` | `null_blank_seen` | output port |
| `o_packet_done_pulse` | `packet_done_pulse` | output port |
| `o_err_ecc_1bit` | `err_ecc_1bit` | output port |
| `o_err_ecc_2bit` | `err_ecc_2bit` | output port |
| `o_err_crc` | `err_crc` | output port |
| `o_err_wc` | `err_wc` | output port |
| `o_err_unsupported_dt` | `err_unsupported_dt` | output port |
| `o_err_vc` | `err_vc[1:0]` | output port |
| `o_err_lane_abort` | `err_lane_abort` | output port |
| `o_err_frame_sync` | `err_frame_sync` | output port |
| `o_err_frame_level` | `err_frame_level` | output port |
| `o_vc_unsupported_seen` | `vc_unsupported_seen` | output port |
| `o_dt_conflict` | `dt_conflict` | output port |
| `PARA_SP_FIFO_DEPTH` | `SP_FIFO_DEPTH` | parameter |
| `PARA_ECC_LATENCY` | `ECC_LATENCY` | parameter |
| `LPARA_WORD_WIDTH` | `WORD_WIDTH` | localparam |
| `LPARA_SP_FIFO_WIDTH` | `SP_FIFO_WIDTH` | localparam |
| `LPARA_BYTE_NUM` | `WORD_WIDTH / 8` | localparam |
| `LPARA_HDR_DATA_W` | - | localparam |
| `LPARA_PTR_W` | - | localparam |
| `LPARA_CNT_W` | - | localparam |
| `LPARA_DEPTH_CNT` | - | localparam |
| `LPARA_DT_FS` | `Frame Start code 0x00` | localparam |
| `LPARA_DT_FE` | `Frame End code 0x01` | localparam |
| `LPARA_DT_GSP_HI` | `codes 0x08 to 0x0F` | localparam |
| `LPARA_DT_NULL` | `Null 0x10` | localparam |
| `LPARA_DT_BLANK` | `Blanking 0x11` | localparam |
| `LPARA_DT_RAW10` | `RAW10 0x2B` | localparam |
| `LPARA_DT_RAW12` | `RAW12 0x2C` | localparam |
| `LPARA_CRC_POLY` | `x16 + x12 + x5 + 1` | localparam |
| `LPARA_CRC_INIT` | `0xFFFF` | localparam |
| `pkt_state_type_def` | `Packet FSM states` | typedef (enum) |
| `ENUM_ST_IDLE` | `ST_IDLE` | enum member |
| `ENUM_ST_HEADER` | `ST_HEADER` | enum member |
| `ENUM_ST_PAYLOAD` | `ST_PAYLOAD` | enum member |
| `ENUM_ST_DISCARD` | `ST_DISCARD` | enum member |
| `ENUM_ST_DONE` | `ST_DONE` | enum member |
| `reg_fsm_pkt_current_state` | `Packet FSM state` | flip-flop |
| `w_fsm_pkt_nxt_state` | `Packet FSM next state` | combinational |
| `reg_eob_latched` | `eob_latched` | flip-flop |
| `w_eob_latched_nxt` | - | combinational |
| `w_pyld_eob` | - | combinational |
| `w_flush` | - | combinational |
| `w_rd_req` | - | combinational |
| `w_rd_en` | `rd_en` | combinational |
| `reg_rd_valid` | - | flip-flop |
| `reg_input_data` | `input_data[31:0]` | flip-flop |
| `reg_input_byte_valid` | `input_byte_valid[3:0]` | flip-flop |
| `reg_input_sob` | `input_sob` | flip-flop |
| `reg_input_eob` | `input_eob` | flip-flop |
| `w_input_valid` | `input_valid` | combinational |
| `w_abort_marker` | `abort_marker` | combinational |
| `reg_word0_reg` | `word0_reg[29:0]` | flip-flop |
| `reg_hdr_load` | - | flip-flop |
| `w_ecc_parity` | - | combinational |
| `w_syndrome` | - | combinational |
| `w_ecc_hit` | - | combinational |
| `w_ecc_parity_err` | - | combinational |
| `w_ecc_corrected` | - | combinational |
| `w_ecc_1bit_raw` | - | combinational |
| `w_ecc_2bit_raw` | - | combinational |
| `gen_ecc_reg.reg_ecc_valid` | - | flip-flop |
| `gen_ecc_reg.reg_ecc_data` | - | flip-flop |
| `gen_ecc_reg.reg_ecc_1bit` | - | flip-flop |
| `gen_ecc_reg.reg_ecc_2bit` | - | flip-flop |
| `gen_ecc_reg.reg_dly_valid` | `Delay Register` | flip-flop |
| `gen_ecc_reg.reg_dly_marker` | `Delay Register` | flip-flop |
| `gen_ecc_reg.reg_dly_eob` | `Delay Register` | flip-flop |
| `gen_ecc_reg.reg_dly_byte_valid` | `Delay Register` | flip-flop |
| `gen_ecc_reg.reg_dly_data` | `Delay Register` | flip-flop |
| `w_hdr_valid` | `hdr_valid` | combinational |
| `w_hdr_data` | - | combinational |
| `w_err_ecc_1bit` | `err_ecc_1bit` | combinational |
| `w_err_ecc_2bit` | `err_ecc_2bit` | combinational |
| `w_hdr_vc` | `hdr_vc[1:0]` | combinational |
| `w_hdr_dt` | `hdr_dt[5:0]` | combinational |
| `w_hdr_wc` | `hdr_wc[15:0]` | combinational |
| `w_hdr_field2` | `hdr_field2[15:0]` | combinational |
| `w_is_short` | `is_short` | combinational |
| `w_hdr_ok` | - | combinational |
| `w_vc_ok` | - | combinational |
| `w_long_ok` | - | combinational |
| `w_hdr_pyld_valid` | `hdr_pyld_valid` | combinational |
| `w_hdr_pyld_marker` | `abort_marker` | combinational |
| `w_hdr_pyld_eob` | `hdr_pyld_eob` | combinational |
| `w_hdr_pyld_byte_valid` | `hdr_pyld_byte_valid[3:0]` | combinational |
| `w_hdr_pyld_data` | `hdr_pyld_data[31:0]` | combinational |
| `w_dt_meta_hit` | - | combinational |
| `w_dt_reserved` | - | combinational |
| `w_pkt_is_embedded` | `pkt_is_embedded` | combinational |
| `w_pkt_is_image` | `pkt_is_image` | combinational |
| `w_dt_meta_drop` | - | combinational |
| `w_dt_null` | - | combinational |
| `w_dt_unsupported` | - | combinational |
| `w_bp_dt_sel` | `bp_dt_sel` | combinational |
| `w_dt_conflict` | `dt_conflict` | combinational |
| `w_fc_fs_raw` | `fc_fs_raw` | combinational |
| `w_fc_fe_raw` | `fc_fe_raw` | combinational |
| `w_fc_fs` | `fc_fs` | combinational |
| `w_fc_fe` | `fc_fe` | combinational |
| `w_gsp_wr` | - | combinational |
| `w_gsp_not_empty` | `sp_fifo_not_empty` | combinational |
| `w_gsp_full` | `sp_fifo_full` | combinational |
| `w_gsp_pop_ok` | - | combinational |
| `w_gsp_push` | - | combinational |
| `w_sp_fifo_overflow` | `sp_fifo_overflow` | combinational |
| `w_gsp_data` | `gsp_data[23:0]` | combinational |
| `reg_gsp_mem` | `Generic Short Packet FIFO` | flip-flop |
| `reg_gsp_wr_ptr` | - | flip-flop |
| `reg_gsp_rd_ptr` | - | flip-flop |
| `reg_gsp_cnt` | - | flip-flop |
| `w_load` | - | combinational |
| `w_armed` | `wc_armed` | combinational |
| `w_rem_cur` | `remaining_bytes` | combinational |
| `w_footer_cnt_cur` | - | combinational |
| `w_crc_cur` | - | combinational |
| `w_word_active` | - | combinational |
| `w_word_bytes` | - | combinational |
| `w_pay_n` | - | combinational |
| `w_payload_byte_en` | `payload_byte_en[3:0]` | combinational |
| `w_payload_valid` | `payload_valid` | combinational |
| `w_payload_last` | `payload_last` | combinational |
| `w_rem_nxt` | - | combinational |
| `w_crc_avail` | - | combinational |
| `w_crc_need` | - | combinational |
| `w_crc_in` | - | combinational |
| `w_footer_cnt_nxt` | - | combinational |
| `w_crc_ls` | - | combinational |
| `w_crc_ms` | - | combinational |
| `w_payload_done` | `payload_done` | combinational |
| `w_crc_nxt` | `crc_accumulated[15:0]` | combinational |
| `w_crc_mismatch` | - | combinational |
| `w_pkt_end` | - | combinational |
| `w_err_wc_pyld` | - | combinational |
| `reg_wc_armed` | `wc_armed` | flip-flop |
| `reg_remaining_bytes` | `remaining_bytes[15:0]` | flip-flop |
| `reg_footer_cnt` | `Footer Collector` | flip-flop |
| `reg_crc` | `crc_accumulated[15:0]` | flip-flop |
| `reg_crc_lo` | `Footer Collector` | flip-flop |
| `w_cur_vc` | - | combinational |
| `w_cur_vc_ok` | - | combinational |
| `w_cur_image` | `pkt_is_image` | combinational |
| `w_cur_emb` | `pkt_is_embedded` | combinational |
| `w_cur_sof` | - | combinational |
| `w_sol_pend_cur` | - | combinational |
| `reg_pkt_vc` | - | flip-flop |
| `reg_pkt_vc_ok` | - | flip-flop |
| `reg_pkt_is_image` | `pkt_is_image` | flip-flop |
| `reg_pkt_is_embedded` | `pkt_is_embedded` | flip-flop |
| `reg_pkt_sof` | - | flip-flop |
| `reg_sol_pend` | - | flip-flop |
| `reg_pkt_long` | - | flip-flop |
| `w_hdr_fa` | - | combinational |
| `w_hdr_err_pend` | - | combinational |
| `w_err_frame_sync` | `err_frame_sync` | combinational |
| `w_err_frame_level` | `err_frame_level` | combinational |
| `w_err_unsupported_dt` | `err_unsupported_dt` | combinational |
| `w_err_wc_hdr` | - | combinational |
| `w_pkt_err_set` | - | combinational |
| `w_frame_active_nxt` | - | combinational |
| `w_first_image_seen_nxt` | - | combinational |
| `w_pkt_err_in_frame_nxt` | - | combinational |
| `reg_frame_active` | `frame_active[1:0]` | flip-flop |
| `reg_first_image_seen` | `first_image_seen` | flip-flop |
| `reg_pkt_err_in_frame` | `pkt_err_in_frame` | flip-flop |
| `w_discard_gate` | `discard_gate` | combinational |
| `w_fwd` | - | combinational |
| `w_bp_valid` | `bp_valid` | combinational |
| `w_bp_sof` | `bp_sof` | combinational |
| `w_bp_sol` | `bp_sol` | combinational |
| `w_bp_eol` | `bp_eol` | combinational |
| `w_emb_valid` | `emb_valid` | combinational |
| `w_emb_sol` | `emb_sol` | combinational |
| `w_emb_eol` | `emb_eol` | combinational |
| `w_err_ecc_1bit_ev` | `err_ecc_1bit` | combinational |
| `w_err_ecc_2bit_ev` | `err_ecc_2bit` | combinational |
| `w_err_crc_det` | `err_crc` | combinational |
| `w_err_wc` | `err_wc` | combinational |
| `w_err_lane_abort` | `err_lane_abort` | combinational |
| `w_hdr_err_any` | - | combinational |
| `w_err_vc` | `err_vc[1:0]` | combinational |
| `w_packet_done_pulse` | `packet_done_pulse` | combinational |
| `w_null_set` | - | combinational |
| `w_vc_unsup_set` | - | combinational |
| `reg_err_crc_d` | - | flip-flop |
| `reg_err_crc_vc_d` | - | flip-flop |
| `reg_bp_data` | `bp_data` | flip-flop |
| `reg_bp_byte_en` | `bp_byte_en` | flip-flop |
| `reg_bp_valid` | `bp_valid` | flip-flop |
| `reg_bp_sof` | `bp_sof` | flip-flop |
| `reg_bp_sol` | `bp_sol` | flip-flop |
| `reg_bp_eol` | `bp_eol` | flip-flop |
| `reg_bp_vc_tag` | `bp_vc_tag` | flip-flop |
| `reg_bp_dt_sel` | `bp_dt_sel` | flip-flop |
| `reg_emb_data` | `emb_data` | flip-flop |
| `reg_emb_byte_en` | `emb_byte_en` | flip-flop |
| `reg_emb_valid` | `emb_valid` | flip-flop |
| `reg_emb_sol` | `emb_sol` | flip-flop |
| `reg_emb_eol` | `emb_eol` | flip-flop |
| `reg_emb_vc` | `emb_vc` | flip-flop |
| `reg_fc_fs` | `fc_fs` | flip-flop |
| `reg_fc_fe` | `fc_fe` | flip-flop |
| `reg_fc_frame_num` | `fc_frame_num` | flip-flop |
| `reg_fc_vc` | `fc_vc` | flip-flop |
| `reg_sp_fifo_overflow` | `sp_fifo_overflow` | flip-flop |
| `reg_packet_done_pulse` | `packet_done_pulse` | flip-flop |
| `reg_err_ecc_1bit` | `err_ecc_1bit` | flip-flop |
| `reg_err_ecc_2bit` | `err_ecc_2bit` | flip-flop |
| `reg_err_crc` | `err_crc` | flip-flop |
| `reg_err_wc` | `err_wc` | flip-flop |
| `reg_err_unsupported_dt` | `err_unsupported_dt` | flip-flop |
| `reg_err_vc` | `err_vc` | flip-flop |
| `reg_err_lane_abort` | `err_lane_abort` | flip-flop |
| `reg_err_frame_sync` | `err_frame_sync` | flip-flop |
| `reg_err_frame_level` | `err_frame_level` | flip-flop |
| `reg_null_blank_seen` | `null_blank_seen` | flip-flop |
| `reg_vc_unsupported_seen` | `vc_unsupported_seen` | flip-flop |
| `reg_dt_conflict` | `dt_conflict` | flip-flop |
| `f_ecc_code` | `ECC parity rules` | function |
| `f_crc16_byte` | `CRC-16 Engine (one stage)` | function |
| `f_byte_cnt` | - | function |
| `f_byte_mask` | - | function |
| `f_byte_sel` | - | function |
| `gen_ecc_reg` | `ECC Engine stages / Delay Register` | generate block |
| `gen_ecc_comb` | `ECC Engine stages` | generate block |
| `gen_chk_fifo_depth` | - | generate block |
| `gen_chk_ecc_latency` | - | generate block |
