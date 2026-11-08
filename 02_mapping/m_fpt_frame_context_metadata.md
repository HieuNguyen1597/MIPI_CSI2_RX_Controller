# Signal mapping — B09 Frame Context & Metadata Capture

## Notes

- RTL module: `m_fpt_frame_context_metadata` (`00_src/rtl/m_fpt_frame_context_metadata.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- The `_vcn` suffix in a spec name is the virtual channel index; output ports are split into one port per channel, `_vc0` / `_vc1`. In an RTL name, `[n]` is the per-channel array element (n = 0 … 1) and `[s]` is the capture slot element (s = 0 … 1); the spec names use the `_vcn` / `_s` suffix.
- `gen_extract.` is the generate block built when `PARA_N_EXTRACT` > 0 (embedded data field extraction); `gen_no_extract.` is built when `PARA_N_EXTRACT` = 0.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_frame_context_metadata` | Frame Context & Metadata Capture (Block No. 9) | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_core_enable` | `core_enable` | input port |
| `i_metadata_capture_en` | `metadata_capture_en` | input port |
| `i_fnum_check_en` | `fnum_check_en` | input port |
| `i_emb_extract_en` | `emb_extract_en` | input port |
| `i_expected_width` | `expected_width[15:0]` | input port |
| `i_expected_height` | `expected_height[15:0]` | input port |
| `i_metadata_record_id` | `metadata_record_id[31:0]` | input port |
| `i_emb_extract_cfg` | `emb_extract_cfg[N_EXTRACT-1:0][31:0]` | input port |
| `i_vc_exposure_map` | `vc_exposure_map` | input port |
| `i_tpg_dual_vc_en` | `tpg_dual_vc_en` | input port |
| `i_active_sel` | `active_sel` | input port |
| `i_fc_fs` | `fc_fs` | input port |
| `i_fc_fe` | `fc_fe` | input port |
| `i_fc_frame_num` | `fc_frame_num[15:0]` | input port |
| `i_fc_vc` | `fc_vc[1:0]` | input port |
| `i_tpg_fs` | `tpg_fs` | input port |
| `i_tpg_fe` | `tpg_fe` | input port |
| `i_tpg_fvc` | `tpg_fvc[1:0]` | input port |
| `i_bp_valid` | `bp_valid` | input port |
| `i_bp_byte_en` | `bp_byte_en[3:0]` | input port |
| `i_bp_sol` | `bp_sol` | input port |
| `i_bp_eol` | `bp_eol` | input port |
| `i_bp_vc_tag` | `bp_vc_tag[1:0]` | input port |
| `i_bp_dt_sel` | `bp_dt_sel` | input port |
| `i_emb_data` | `emb_data[31:0]` | input port |
| `i_emb_byte_en` | `emb_byte_en[3:0]` | input port |
| `i_emb_valid` | `emb_valid` | input port |
| `i_emb_sol` | `emb_sol` | input port |
| `i_emb_eol` | `emb_eol` | input port |
| `i_emb_vc` | `emb_vc[1:0]` | input port |
| `i_capture_drop` | `capture_drop` | input port |
| `i_drop_vc_mask` | `drop_vc_mask[1:0]` | input port |
| `i_dma_buf_claim` | `dma_buf_claim` | input port |
| `i_dma_buf_index` | `dma_buf_index` | input port |
| `i_meta_ready` | `meta_ready` | input port |
| `o_capture_start` | `capture_start[1:0]` | output port |
| `o_capture_end` | `capture_end[1:0]` | output port |
| `o_capture_dropped` | `capture_dropped[1:0]` | output port |
| `o_ev_fs` | `ev_fs` | output port |
| `o_ev_vc` | `ev_vc[1:0]` | output port |
| `o_ctx_slot` | `ctx_slot[1:0]` | output port |
| `o_ctx_orphan` | `ctx_orphan[1:0]` | output port |
| `o_lead_vc` | `lead_vc` | output port |
| `o_frame_count` | `frame_count[31:0]` | output port |
| `o_frame_id_buf0` | `frame_id_buf0[13:0]` | output port |
| `o_frame_id_buf1` | `frame_id_buf1[13:0]` | output port |
| `o_timestamp` | `timestamp[63:0]` | output port |
| `o_vc0_line_count` | `vcn_line_count[15:0]` | output port |
| `o_vc0_byte_count` | `vcn_byte_count[15:0]` | output port |
| `o_vc0_data_type` | `vcn_data_type[5:0]` | output port |
| `o_vc0_width_mismatch` | `vcn_width_mismatch` | output port |
| `o_vc0_height_mismatch` | `vcn_height_mismatch` | output port |
| `o_emb_word_count_vc0` | `emb_word_count_vcn[15:0]` | output port |
| `o_emb_line_count_vc0` | `emb_line_count_vcn[7:0]` | output port |
| `o_emb_extract_val_vc0` | `emb_extract_val_vcn[N_EXTRACT-1:0][31:0]` | output port |
| `o_cur_frame_id_vc0` | `cur_frame_id_vcn[13:0]` | output port |
| `o_cur_line_vc0` | `cur_line_vcn[15:0]` | output port |
| `o_vc1_line_count` | `vcn_line_count[15:0]` | output port |
| `o_vc1_byte_count` | `vcn_byte_count[15:0]` | output port |
| `o_vc1_data_type` | `vcn_data_type[5:0]` | output port |
| `o_vc1_width_mismatch` | `vcn_width_mismatch` | output port |
| `o_vc1_height_mismatch` | `vcn_height_mismatch` | output port |
| `o_emb_word_count_vc1` | `emb_word_count_vcn[15:0]` | output port |
| `o_emb_line_count_vc1` | `emb_line_count_vcn[7:0]` | output port |
| `o_emb_extract_val_vc1` | `emb_extract_val_vcn[N_EXTRACT-1:0][31:0]` | output port |
| `o_cur_frame_id_vc1` | `cur_frame_id_vcn[13:0]` | output port |
| `o_cur_line_vc1` | `cur_line_vcn[15:0]` | output port |
| `o_meta_overflow` | `meta_overflow` | output port |
| `o_err_frame_num` | `err_frame_num` | output port |
| `o_err_resolution` | `err_resolution` | output port |
| `o_capture_done` | `capture_done` | output port |
| `o_meta_data` | `meta_data[31:0]` | output port |
| `o_meta_byte_en` | `meta_byte_en[3:0]` | output port |
| `o_meta_valid` | `meta_valid` | output port |
| `o_meta_last` | `meta_last` | output port |
| `o_meta_kind` | `meta_kind` | output port |
| `o_meta_vc` | `meta_vc[1:0]` | output port |
| `o_meta_sof` | `meta_sof` | output port |
| `PARA_N_EXTRACT` | `N_EXTRACT` | parameter |
| `PARA_TS_INCR` | `TS_INCR` | parameter |
| `PARA_META_FIFO_DEPTH` | `META_FIFO_DEPTH` | parameter |
| `LPARA_FRAME_ID_WIDTH` | `FRAME_ID_WIDTH` | localparam |
| `LPARA_CAPTURE_SLOTS` | `CAPTURE_SLOTS` | localparam |
| `LPARA_RECORD_WORDS` | `RECORD_WORDS` | localparam |
| `LPARA_CHANNEL_NUM` | - | localparam |
| `LPARA_EXTRACT_PORTS` | - | localparam |
| `LPARA_TS_INT_W` | `64 integer bits` | localparam |
| `LPARA_TS_FRAC_W` | `24 fractional bits` | localparam |
| `LPARA_META_WIDTH` | `41 bits` | localparam |
| `LPARA_PTR_W` | - | localparam |
| `LPARA_CNT_W` | - | localparam |
| `LPARA_WORD_IDX_W` | - | localparam |
| `LPARA_DEPTH_CNT` | - | localparam |
| `LPARA_WORD_LAST` | - | localparam |
| `LPARA_FRAME_ID_MAX` | - | localparam |
| `LPARA_RECORD_VERSION` | `version` | localparam |
| `LPARA_DT_RAW10` | `RAW10 0x2B` | localparam |
| `LPARA_DT_RAW12` | `RAW12 0x2C` | localparam |
| `rec_state_type_def` | `Record FSM states` | typedef (enum) |
| `ENUM_ST_IDLE` | `ST_IDLE` | enum member |
| `ENUM_ST_RECORD` | `ST_RECORD` | enum member |
| `ENUM_ST_POP` | `ST_POP` | enum member |
| `reg_fsm_rec_current_state` | `Record FSM state` | flip-flop |
| `w_fsm_rec_nxt_state` | `Record FSM next state` | combinational |
| `w_ev_fs` | `ev_fs` | combinational |
| `w_ev_fe` | `ev_fe` | combinational |
| `w_ev_vc` | `ev_vc[1:0]` | combinational |
| `w_ev_fnum` | `ev_fnum[15:0]` | combinational |
| `w_ev_rx` | `ev_rx` | combinational |
| `w_lead_vc` | `lead_vc` | combinational |
| `w_dual_vc` | `dual_vc` | combinational |
| `w_ev_take` | - | combinational |
| `w_fs_take` | - | combinational |
| `w_fe_take` | - | combinational |
| `w_ev_ch` | - | combinational |
| `w_is_lead` | - | combinational |
| `w_ctx_open` | `ctx_open[1:0]` | combinational |
| `w_ctx_close` | `ctx_close[1:0]` | combinational |
| `w_ctx_sync_close` | `ctx_sync_close[1:0]` | combinational |
| `w_ctx_abort` | `ctx_abort[1:0]` | combinational |
| `reg_ctx_active[n]` | `ctx_active` | flip-flop |
| `reg_ctx_orphan[n]` | `ctx_orphan` | flip-flop |
| `reg_ctx_slot[n]` | `ctx_slot` | flip-flop |
| `reg_last_fnum[n]` | `last_fnum` | flip-flop |
| `reg_fnum_err_flag[n]` | `fnum_err_flag` | flip-flop |
| `reg_slot_open[s]` | `slot_open` | flip-flop |
| `reg_slot_wait_lag[s]` | `slot_wait_lag` | flip-flop |
| `reg_slot_dropped[s]` | `slot_dropped` | flip-flop |
| `reg_last_slot` | - | flip-flop |
| `reg_core_enable_d` | - | flip-flop |
| `reg_active_sel_d` | - | flip-flop |
| `w_core_fall` | - | combinational |
| `w_active_sel_chg` | - | combinational |
| `w_slot_free[s]` | - | combinational |
| `w_alloc_en` | - | combinational |
| `w_alloc_slot` | - | combinational |
| `w_lag_join` | - | combinational |
| `w_capture_start` | `capture_start[1:0]` | combinational |
| `w_recent_slot` | - | combinational |
| `w_drop_hit[s]` | - | combinational |
| `w_slot_busy[s]` | - | combinational |
| `w_slot_eligible[s]` | - | combinational |
| `w_end_en` | - | combinational |
| `w_capture_end` | `capture_end[1:0]` | combinational |
| `w_capture_dropped` | `capture_dropped[1:0]` | combinational |
| `w_capture_done` | `capture_done` | combinational |
| `w_fnum_mismatch` | - | combinational |
| `w_err_frame_num` | `err_frame_num` | combinational |
| `reg_ts_cnt` | `timestamp counter` | flip-flop |
| `reg_frame_id_cnt` | `frame_id_cnt` | flip-flop |
| `w_frame_id_nxt` | - | combinational |
| `reg_slot_frame_id[s]` | `slot_frame_id_s[13:0]` | flip-flop |
| `reg_slot_timestamp[s]` | `slot_timestamp_s[63:0]` | flip-flop |
| `reg_slot_record_id[s]` | `slot_record_id_s[31:0]` | flip-flop |
| `reg_frame_id_buf` | `frame_id_buf0, frame_id_buf1` | flip-flop |
| `w_cur_frame_id[n]` | `cur_frame_id_vcn[13:0]` | combinational |
| `w_bp_take` | - | combinational |
| `w_bp_ch` | - | combinational |
| `w_line_bytes` | `line_bytes[15:0]` | combinational |
| `w_line_done` | `line_done[1:0]` | combinational |
| `reg_byte_acc[n]` | `byte_acc` | flip-flop |
| `reg_acc_byte_count[n]` | `acc_byte_count_vcn[15:0]` | flip-flop |
| `reg_acc_line_count[n]` | `acc_line_count_vcn[15:0]` | flip-flop |
| `reg_acc_dt[n]` | `acc_dt_vcn[5:0]` | flip-flop |
| `reg_expected_width[n]` | `expected_width (sampled)` | flip-flop |
| `reg_expected_height[n]` | `expected_height (sampled)` | flip-flop |
| `w_bytes_ref` | - | combinational |
| `reg_width_mismatch[n]` | `width_mismatch_vcn` | flip-flop |
| `w_height_mismatch[n]` | `height_mismatch_vcn` | combinational |
| `w_err_resolution` | `err_resolution` | combinational |
| `w_emb_fwd_valid` | `emb_fwd_valid` | combinational |
| `w_emb_ch` | - | combinational |
| `reg_emb_word_cnt[n]` | `emb_word_cnt_vcn[15:0]` | flip-flop |
| `reg_emb_line_cnt[n]` | `emb_line_cnt_vcn[7:0]` | flip-flop |
| `w_extract_pub[n]` | `emb_extract_val_vcn` | combinational |
| `w_rec_extract` | - | combinational |
| `gen_extract.reg_emb_byte_ptr` | `byte pointer` | flip-flop |
| `gen_extract.w_emb_ptr` | - | combinational |
| `gen_extract.w_extract_cur` | - | combinational |
| `gen_extract.w_extract_nxt` | - | combinational |
| `gen_extract.w_slot_cfg` | - | combinational |
| `gen_extract.w_slot_val` | - | combinational |
| `gen_extract.w_field_len` | - | combinational |
| `gen_extract.w_field_start` | - | combinational |
| `gen_extract.w_byte_pos` | - | combinational |
| `gen_extract.w_field_pos` | - | combinational |
| `gen_extract.reg_extract_val[n]` | `extract_val_vcn[N_EXTRACT-1:0][31:0]` | flip-flop |
| `gen_extract.reg_bank_extract_val[n]` | `bank_extract_val_vcn[N_EXTRACT-1:0][31:0]` | flip-flop |
| `gen_extract.reg_pub_extract_val[n]` | `emb_extract_val_vcn` | flip-flop |
| `gen_extract.w_bank_extract_cur` | - | combinational |
| `gen_extract.w_unused_cfg` | - | combinational |
| `gen_no_extract.w_unused_extract` | - | combinational |
| `w_close_ok[n]` | - | combinational |
| `w_bank_load[n]` | - | combinational |
| `w_pub_load[n]` | - | combinational |
| `reg_bank_line_count[n]` | `bank_line_count_vcn[15:0]` | flip-flop |
| `reg_bank_byte_count[n]` | `bank_byte_count_vcn[15:0]` | flip-flop |
| `reg_bank_dt[n]` | `bank_dt_vcn[5:0]` | flip-flop |
| `reg_bank_width_mismatch[n]` | `bank_width_mismatch_vcn` | flip-flop |
| `reg_bank_height_mismatch[n]` | `bank_height_mismatch_vcn` | flip-flop |
| `reg_bank_emb_word_cnt[n]` | `bank_emb_word_cnt_vcn[15:0]` | flip-flop |
| `reg_bank_emb_line_cnt[n]` | `bank_emb_line_cnt_vcn[7:0]` | flip-flop |
| `reg_bank_fnum_err[n]` | `bank_fnum_err_vcn` | flip-flop |
| `reg_bank_ovf[n]` | `bank_ovf_vcn` | flip-flop |
| `reg_pub_line_count[n]` | `vcn_line_count` | flip-flop |
| `reg_pub_byte_count[n]` | `vcn_byte_count` | flip-flop |
| `reg_pub_data_type[n]` | `vcn_data_type` | flip-flop |
| `reg_pub_width_mismatch[n]` | `vcn_width_mismatch` | flip-flop |
| `reg_pub_height_mismatch[n]` | `vcn_height_mismatch` | flip-flop |
| `reg_pub_emb_word_count[n]` | `emb_word_count_vcn` | flip-flop |
| `reg_pub_emb_line_count[n]` | `emb_line_count_vcn` | flip-flop |
| `reg_timestamp` | `timestamp` | flip-flop |
| `reg_frame_count` | `frame_count` | flip-flop |
| `reg_queue_vc` | - | flip-flop |
| `reg_queue_slot` | - | flip-flop |
| `reg_queue_abort` | - | flip-flop |
| `reg_queue_cnt` | - | flip-flop |
| `w_queue_empty` | `queue_empty` | combinational |
| `w_more_pending` | `more_pending` | combinational |
| `w_cur_vc` | `cur_vc` | combinational |
| `w_cur_slot` | `cur_slot` | combinational |
| `w_cur_abort` | `cur_abort` | combinational |
| `w_nxt_slot` | `nxt_slot` | combinational |
| `w_vc_queued[n]` | - | combinational |
| `w_push` | `push` | combinational |
| `w_push_vc` | `push_vc` | combinational |
| `w_push_slot` | - | combinational |
| `w_push_abort` | - | combinational |
| `w_record_lost` | - | combinational |
| `w_queue_wr_idx` | - | combinational |
| `reg_rec_pending` | `rec_pending[1:0]` | flip-flop |
| `w_rec_pending_set[s]` | - | combinational |
| `w_rec_pending_clr[s]` | - | combinational |
| `w_rec_active` | `rec_active` | combinational |
| `w_cnt_clr` | `cnt_clr` | combinational |
| `w_queue_pop` | `queue_pop` | combinational |
| `reg_word_idx` | `word_idx` | flip-flop |
| `w_word_last` | `word_last` | combinational |
| `w_cur_timestamp` | - | combinational |
| `w_err_summary` | `err_summary` | combinational |
| `w_rec_word` | `rec_word` | combinational |
| `w_fifo_full` | `fifo_full` | combinational |
| `w_fifo_empty` | `fifo_empty` | combinational |
| `w_rec_grant` | `rec_grant` | combinational |
| `w_pay_wr` | `pay_wr` | combinational |
| `w_pay_lost` | `pay_lost` | combinational |
| `reg_ovf_frame` | `ovf_frame[1:0]` | flip-flop |
| `reg_sof_pend` | `sof_pend[1:0]` | flip-flop |
| `w_sof_bit` | `sof_bit` | combinational |
| `reg_meta_overflow` | `meta_overflow` | flip-flop |
| `reg_fifo_mem` | - | flip-flop |
| `reg_fifo_wr_ptr` | - | flip-flop |
| `reg_fifo_rd_ptr` | - | flip-flop |
| `reg_fifo_cnt` | - | flip-flop |
| `w_fifo_wr` | - | combinational |
| `w_fifo_rd` | - | combinational |
| `w_fifo_din` | - | combinational |
| `w_fifo_dout` | - | combinational |
| `w_meta_data` | `meta_data[31:0]` | combinational |
| `w_meta_byte_en` | `meta_byte_en[3:0]` | combinational |
| `w_meta_valid` | `meta_valid` | combinational |
| `w_meta_last` | `meta_last` | combinational |
| `w_meta_kind` | `meta_kind` | combinational |
| `w_meta_vc` | `meta_vc[1:0]` | combinational |
| `w_meta_sof` | `meta_sof` | combinational |
| `f_byte_num` | - | function |
| `gen_chk_n_extract` | - | generate block |
| `gen_chk_ts_incr` | - | generate block |
| `gen_chk_fifo_depth` | - | generate block |
| `gen_extract` | - | generate block |
| `gen_no_extract` | - | generate block |
