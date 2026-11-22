# Signal mapping — B13 Direct Memory Access (DMA)

## Notes

- RTL modules: `m_fpt_dma` (`00_src/rtl/m_fpt_dma.sv`) and its FIFO `m_fpt_dma_async_fifo` (`00_src/rtl/m_fpt_dma_async_fifo.sv`), instantiated five times as `u_raw_cmd_fifo_0`, `u_raw_data_fifo_0`, `u_meta_cmd_fifo_0`, `u_meta_data_fifo_0` and `u_cpl_fifo_0`.
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- In a spec name, `[b]` is the buffer index, `[x]` the virtual channel index and `[s]` the capture slot index; the RTL keeps one element per buffer, channel or slot in a vector or array. `shadow[b].<field>`, `pending[x].<field>` and similar names are one field of a per-buffer or per-channel record, held in its own RTL signal.
- `gen_buf` builds one Buffer FSM per buffer; `buf_state_type_def` / `ENUM_ST_*` are its states.
- `gen_para_check*` stop elaboration for an out-of-range parameter. In `m_fpt_dma_async_fifo`, `gen_async` builds the Gray-pointer clock crossing (`PARA_ASYNC = 1`) and `gen_sync` the single-clock FIFO (`PARA_ASYNC = 0`).
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping — m_fpt_dma

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_dma` | `DMA (Block No. 13)` | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_mm_aclk` | `mm_aclk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_mm_resetn` | `mm_resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_fbd_data` | `fbd_data` | input port |
| `i_fbd_byte_en` | `fbd_byte_en` | input port |
| `i_fbd_valid` | `fbd_valid` | input port |
| `i_fbd_sof` | `fbd_sof` | input port |
| `i_fbd_sol` | `fbd_sol` | input port |
| `i_fbd_eol` | `fbd_eol` | input port |
| `i_fbd_vc_tag` | `fbd_vc_tag` | input port |
| `i_fbd_err` | `fbd_err` | input port |
| `o_fbd_ready` | `fbd_ready` | output port |
| `i_capture_start` | `capture_start` | input port |
| `i_capture_end` | `capture_end` | input port |
| `i_capture_dropped` | `capture_dropped` | input port |
| `i_ev_fs` | `ev_fs` | input port |
| `i_ev_vc` | `ev_vc` | input port |
| `i_lead_vc` | `lead_vc` | input port |
| `i_ctx_slot` | `ctx_slot` | input port |
| `i_ctx_orphan` | `ctx_orphan` | input port |
| `i_frame_id_buf0` | `frame_id_buf0` | input port |
| `i_frame_id_buf1` | `frame_id_buf1` | input port |
| `i_meta_data` | `meta_data` | input port |
| `i_meta_byte_en` | `meta_byte_en` | input port |
| `i_meta_valid` | `meta_valid` | input port |
| `i_meta_last` | `meta_last` | input port |
| `i_meta_kind` | `meta_kind` | input port |
| `i_meta_vc` | `meta_vc` | input port |
| `i_meta_sof` | `meta_sof` | input port |
| `o_meta_ready` | `meta_ready` | output port |
| `o_dma_buf_claim` | `dma_buf_claim` | output port |
| `o_dma_buf_index` | `dma_buf_index` | output port |
| `i_active_sel` | `active_sel` | input port |
| `i_tpg_height_shadow` | `tpg_height_shadow` | input port |
| `i_dma_abort_req` | `dma_abort_req` | input port |
| `o_dma_drop` | `dma_drop` | output port |
| `o_dma_buf_done_ok` | `dma_buf_done_ok` | output port |
| `o_dma_buf_done_err` | `dma_buf_done_err` | output port |
| `o_dma_axi_write_error` | `dma_axi_write_error` | output port |
| `o_dma_buffer_overflow` | `dma_buffer_overflow` | output port |
| `i_dma_enable` | `dma_enable` | input port |
| `i_dma_start` | `dma_start` | input port |
| `i_dma_abort` | `dma_abort` | input port |
| `i_err_frame_policy` | `err_frame_policy` | input port |
| `i_dma_buffer0_addr` | `dma_buffer0_addr` | input port |
| `i_dma_buffer1_addr` | `dma_buffer1_addr` | input port |
| `i_dma_buffer_size` | `dma_buffer_size` | input port |
| `i_meta_area_size` | `meta_area_size` | input port |
| `i_bytes_per_line` | `bytes_per_line` | input port |
| `i_line_padding` | `line_padding` | input port |
| `i_awqos` | `awqos` | input port |
| `i_max_burst_len` | `max_burst_len` | input port |
| `i_awprot` | `awprot` | input port |
| `i_release_buf` | `release_buf` | input port |
| `o_dma_busy` | `dma_busy` | output port |
| `o_active_buffer` | `active_buffer` | output port |
| `o_buf0_state` | `buf0_state` | output port |
| `o_buf1_state` | `buf1_state` | output port |
| `o_buf0_last_valid_line` | `buf0_last_valid_line` | output port |
| `o_buf1_last_valid_line` | `buf1_last_valid_line` | output port |
| `o_axi_fault_awid` | `axi_fault_awid` | output port |
| `o_axi_fault_resp` | `axi_fault_resp` | output port |
| `o_axi_fault_awaddr` | `axi_fault_awaddr` | output port |
| `o_bresp_slverr_count` | `bresp_slverr_count` | output port |
| `o_bresp_decerr_count` | `bresp_decerr_count` | output port |
| `o_m_axi4_awid` | `m_axi4_awid` | output port |
| `o_m_axi4_awaddr` | `m_axi4_awaddr` | output port |
| `o_m_axi4_awlen` | `m_axi4_awlen` | output port |
| `o_m_axi4_awsize` | `m_axi4_awsize` | output port |
| `o_m_axi4_awburst` | `m_axi4_awburst` | output port |
| `o_m_axi4_awlock` | `m_axi4_awlock` | output port |
| `o_m_axi4_awcache` | `m_axi4_awcache` | output port |
| `o_m_axi4_awprot` | `m_axi4_awprot` | output port |
| `o_m_axi4_awqos` | `m_axi4_awqos` | output port |
| `o_m_axi4_awvalid` | `m_axi4_awvalid` | output port |
| `i_m_axi4_awready` | `m_axi4_awready` | input port |
| `o_m_axi4_wdata` | `m_axi4_wdata` | output port |
| `o_m_axi4_wstrb` | `m_axi4_wstrb` | output port |
| `o_m_axi4_wlast` | `m_axi4_wlast` | output port |
| `o_m_axi4_wvalid` | `m_axi4_wvalid` | output port |
| `i_m_axi4_wready` | `m_axi4_wready` | input port |
| `i_m_axi4_bid` | `m_axi4_bid` | input port |
| `i_m_axi4_bresp` | `m_axi4_bresp` | input port |
| `i_m_axi4_bvalid` | `m_axi4_bvalid` | input port |
| `o_m_axi4_bready` | `m_axi4_bready` | output port |
| `PARA_AXI_DATA_WIDTH` | `AXI_DATA_WIDTH` | parameter |
| `PARA_AXI_ADDR_WIDTH` | `AXI_ADDR_WIDTH` | parameter |
| `PARA_AXI_ID_WIDTH` | `AXI_ID_WIDTH` | parameter |
| `PARA_MAX_BURST_BEATS` | `MAX_BURST_BEATS` | parameter |
| `PARA_MAX_OUTSTANDING` | `MAX_OUTSTANDING` | parameter |
| `PARA_RAW_FIFO_DEPTH` | `RAW_FIFO_DEPTH` | parameter |
| `PARA_META_DATA_FIFO_DEPTH` | `META_DATA_FIFO_DEPTH` | parameter |
| `PARA_CMD_FIFO_DEPTH` | `CMD_FIFO_DEPTH` | parameter |
| `PARA_ASYNC_MM_CLK` | `ASYNC_MM_CLK` | parameter |
| `PARA_SYNC_STAGES` | `SYNC_STAGES` | parameter |
| `PARA_META_MAX_BURST_BEATS` | `META_MAX_BURST_BEATS` | parameter |
| `LPARA_FIFO_DATA_W` | - | localparam |
| `LPARA_FIFO_STRB_W` | - | localparam |
| `LPARA_BEAT_SHIFT` | - | localparam |
| `LPARA_DATA_ENTRY_W` | `data {wdata, wstrb}` | localparam |
| `LPARA_CMD_W` | `cmd_*` | localparam |
| `LPARA_CPL_W` | `cpl_*` | localparam |
| `LPARA_TAG_W` | - | localparam |
| `LPARA_ADDR_EXT_W` | - | localparam |
| `LPARA_RA_WORDS` | `Record Assembler` | localparam |
| `LPARA_N_BUFFERS` | `N_BUFFERS` | localparam |
| `LPARA_MAX_BURST_LEN_WIDTH` | `MAX_BURST_LEN_WIDTH` | localparam |
| `LPARA_AWSIZE` | `AWSIZE` | localparam |
| `LPARA_BEATS_W` | - | localparam |
| `LPARA_OUTST_W` | - | localparam |
| `LPARA_TAG_PTR_W` | - | localparam |
| `LPARA_INFL_W` | `inflight` | localparam |
| `LPARA_WQ_DEPTH` | `W order queue` | localparam |
| `LPARA_WQ_PTR_W` | - | localparam |
| `buf_state_type_def` | `Buffer FSM states` | typedef (enum) |
| `ENUM_ST_FREE` | `ST_FREE` | enum member |
| `ENUM_ST_WRITE` | `ST_WRITE` | enum member |
| `ENUM_ST_DRAIN` | `ST_DRAIN` | enum member |
| `ENUM_ST_ABORT` | `ST_ABORT` | enum member |
| `ENUM_ST_SW_VALID` | `ST_SW_VALID` | enum member |
| `ENUM_ST_SW_ERROR` | `ST_SW_ERROR` | enum member |
| `reg_fsm_buf_current_state` | `state[b]` | flip-flop |
| `w_buf_free` | `buf_free[1:0]` | combinational |
| `w_buf_writing` | `buf_writing[1:0]` | combinational |
| `w_buf_leave` | `buf_leave[1:0]` | combinational |
| `w_done_ok` | `dma_buf_done_ok (per buffer)` | combinational |
| `w_done_err` | `dma_buf_done_err (per buffer)` | combinational |
| `w_abort_req` | `abort_req[b]` | combinational |
| `reg_soft_reset_seen` | `soft_reset_seen[b]` | flip-flop |
| `reg_sh_buf_addr` | `shadow[b].buf_addr` | flip-flop |
| `reg_sh_raw_size` | `shadow[b].raw_size` | flip-flop |
| `reg_sh_meta_area_size` | `shadow[b].meta_area_size` | flip-flop |
| `reg_sh_bytes_per_line` | `shadow[b].bytes_per_line` | flip-flop |
| `reg_sh_line_padding` | `shadow[b].line_padding` | flip-flop |
| `reg_sh_max_burst_len` | `shadow[b].max_burst_len` | flip-flop |
| `reg_sh_awqos` | `shadow[b].awqos` | flip-flop |
| `reg_sh_awprot` | `shadow[b].awprot` | flip-flop |
| `reg_sh_policy` | `shadow[b].policy` | flip-flop |
| `reg_sh_tpg` | `shadow[b].tpg` | flip-flop |
| `reg_sh_tpg_height` | `shadow[b].tpg_height` | flip-flop |
| `reg_joined` | `joined[b][x]` | flip-flop |
| `reg_rec_seen` | `rec_seen[b][x]` | flip-flop |
| `reg_moved` | `moved[b][x]` | flip-flop |
| `reg_rec_lines` | `rec_lines[b][x]` | flip-flop |
| `reg_lines` | `lines[b][x]` | flip-flop |
| `reg_ended` | `ended[b]` | flip-flop |
| `reg_inflight` | `inflight[b]` | flip-flop |
| `reg_cap_start` | `cap_start_q` | flip-flop |
| `reg_ev_fs` | `ev_fs_q` | flip-flop |
| `reg_ev_vc` | `ev_vc_q` | flip-flop |
| `reg_lead_vc` | `lead_vc_q` | flip-flop |
| `reg_slot_ok` | `slot_ok[s]` | flip-flop |
| `reg_slot_buf` | `slot_buf[s]` | flip-flop |
| `reg_last_buf` | `last_buf` | flip-flop |
| `reg_active_buffer` | `active_buffer` | flip-flop |
| `reg_pending_valid` | `pending[x].valid` | flip-flop |
| `reg_pending_buf` | `pending[x].buf` | flip-flop |
| `w_run` | `run` | combinational |
| `w_abort_all` | `abort_all` | combinational |
| `w_clr` | `clr` | combinational |
| `w_pick` | `pick` | combinational |
| `w_pick_none` | `pick == NONE` | combinational |
| `w_claim` | `claim` | combinational |
| `w_dma_drop` | `dma_drop` | combinational |
| `w_bind_end` | `bind_end[b]` | combinational |
| `w_bind_drop` | `bind_drop[b]` | combinational |
| `w_load` | `load[x]` | combinational |
| `w_fs_valid` | `fs_val[x].valid` | combinational |
| `w_fs_buf` | `fs_val[x].buf` | combinational |
| `w_pend_load` | `pend_load[x]` | combinational |
| `w_bind_join` | `bind_join[x]` | combinational |
| `w_leave_hit` | `leave_hit[x]` | combinational |
| `reg_pw_fresh` | `fresh[x] (Pixel Writer)` | flip-flop |
| `reg_pw_active_valid` | `active[x].valid (Pixel Writer)` | flip-flop |
| `reg_pw_active_buf` | `active[x].buf (Pixel Writer)` | flip-flop |
| `reg_line_idx` | `line_idx[x]` | flip-flop |
| `reg_line_bytes` | `line_bytes` | flip-flop |
| `reg_beats` | `beats` | flip-flop |
| `reg_burst_addr` | `burst_addr` | flip-flop |
| `reg_burst_vc` | `burst_vc` | flip-flop |
| `w_vc` | `vc` | combinational |
| `w_beat` | `beat` | combinational |
| `w_sof` | `sof` | combinational |
| `w_sol` | `sol` | combinational |
| `w_eol` | `eol` | combinational |
| `w_bind_tpg` | `bind_tpg` | combinational |
| `w_raw_full` | `raw_full` | combinational |
| `w_fbd_ready` | `fbd_ready` | combinational |
| `w_sof_x` | `sof_x` | combinational |
| `w_eol_x` | `eol_x` | combinational |
| `w_nb_valid` | `nb[x].valid` | combinational |
| `w_nb_buf` | `nb[x].buf` | combinational |
| `w_bind_valid` | `bind[x].valid` | combinational |
| `w_bind_buf` | `bind[x].buf` | combinational |
| `w_lidx` | `lidx[x]` | combinational |
| `w_cur_valid` | `cur.valid` | combinational |
| `w_cur_buf` | `cur.buf` | combinational |
| `w_clip` | `clip` | combinational |
| `w_lose` | `lose` | combinational |
| `w_wbeat` | `wbeat` | combinational |
| `w_pw_err` | `err (Pixel Writer)` | combinational |
| `w_write` | `write` | combinational |
| `w_stride` | `stride` | combinational |
| `w_lb_prev` | `lb_prev` | combinational |
| `w_lb` | `lb` | combinational |
| `w_line_addr` | `line_addr` | combinational |
| `w_region_end` | `buf_addr + (vc + 1) x raw_size` | combinational |
| `w_beat_addr` | `beat_addr` | combinational |
| `w_bstart` | `bstart` | combinational |
| `w_to_4k` | `to_4k` | combinational |
| `w_n` | `n` | combinational |
| `w_lim` | `max_burst_len + 1 (64-bit beats)` | combinational |
| `w_close` | `close` | combinational |
| `w_skip` | `skip` | combinational |
| `w_raw_cmd_push` | `cmd_push` | combinational |
| `w_raw_data_push` | `data_push` | combinational |
| `w_raw_cmd_din` | `cmd` | combinational |
| `w_raw_data_din` | `data {wdata, wstrb}` | combinational |
| `w_wr_issue_pw` | `wr_issue[b] (Pixel Writer)` | combinational |
| `w_wr_err_pw` | `wr_err[b] (Pixel Writer)` | combinational |
| `w_miss_evt` | `miss_evt` | combinational |
| `w_eol_evt` | `eol_evt` | combinational |
| `w_sof_evt` | `sof_evt` | combinational |
| `w_sof_prev_valid` | `sof_evt.prev_valid` | combinational |
| `w_sof_prev_buf` | `sof_evt.prev_buf` | combinational |
| `reg_mw_fresh` | `fresh[x] (Metadata Writer)` | flip-flop |
| `reg_mw_active_valid` | `active metadata binding valid` | flip-flop |
| `reg_mw_active_buf` | `active metadata binding buf` | flip-flop |
| `reg_pay_ptr` | `pay_ptr[x]` | flip-flop |
| `reg_pay_beats` | `pay_beats` | flip-flop |
| `reg_pay_addr` | - | flip-flop |
| `reg_pay_vc` | - | flip-flop |
| `reg_pay_buf` | - | flip-flop |
| `reg_pay_open` | - | flip-flop |
| `reg_pay_data` | - | flip-flop |
| `reg_pay_strb` | - | flip-flop |
| `reg_ra_word` | `Record Assembler` | flip-flop |
| `reg_ra_cnt` | - | flip-flop |
| `reg_ra_vc` | `meta_vc of the record` | flip-flop |
| `reg_ra_full` | `ra_full` | flip-flop |
| `reg_ra_busy` | `ra_busy` | flip-flop |
| `reg_rec_buf_valid` | `rec_buf != DROP` | flip-flop |
| `reg_rec_buf` | `rec_buf` | flip-flop |
| `reg_rec_beat` | - | flip-flop |
| `reg_rec_bn` | - | flip-flop |
| `reg_rec_start` | `rec_ptr` | flip-flop |
| `reg_rec_addr` | - | flip-flop |
| `w_mv` | `mv` | combinational |
| `w_mx` | `meta_vc[0]` | combinational |
| `w_rec_wait` | `rec_wait` | combinational |
| `w_meta_ready` | `meta_ready` | combinational |
| `w_pay_bind_valid` | `pay_bind.valid` | combinational |
| `w_pay_cut` | `pay_cut` | combinational |
| `w_pay_skip` | `pay_skip` | combinational |
| `w_pay_detach` | - | combinational |
| `w_ra_go` | `ra_go` | combinational |
| `w_meta_data_push` | - | combinational |
| `w_meta_cmd_push` | - | combinational |
| `w_rec_done` | `rec_done` | combinational |
| `w_rec_done_buf` | `rec_done.b` | combinational |
| `w_rec_done_x` | `rec_done.x` | combinational |
| `w_ra_we` | - | combinational |
| `w_rec_lines` | `rec_lines.count` | combinational |
| `w_ra_len` | `RECORD_WORDS` | combinational |
| `w_ra_widx` | - | combinational |
| `w_meta_data_din` | `data {wdata, wstrb}` | combinational |
| `w_meta_cmd_din` | `cmd` | combinational |
| `w_wr_issue_mw` | `wr_issue[b] (Metadata Writer)` | combinational |
| `w_wr_err_mw` | `wr_err[b] (Metadata Writer)` | combinational |
| `w_pay_beats_nxt` | - | combinational |
| `w_rec_bn_nxt` | - | combinational |
| `w_pay_addr_nxt` | - | combinational |
| `w_rec_start_nxt` | - | combinational |
| `w_rec_addr_nxt` | - | combinational |
| `w_pay_vc_nxt` | - | combinational |
| `w_pay_buf_nxt` | - | combinational |
| `w_pay_open_nxt` | - | combinational |
| `w_pay_data_nxt` | - | combinational |
| `w_pay_strb_nxt` | - | combinational |
| `w_pay_ptr_nxt` | - | combinational |
| `w_ra_cnt_nxt` | - | combinational |
| `w_ra_vc_nxt` | - | combinational |
| `w_ra_full_nxt` | - | combinational |
| `w_ra_busy_nxt` | - | combinational |
| `w_rec_buf_valid_nxt` | - | combinational |
| `w_rec_buf_nxt` | - | combinational |
| `w_rec_beat_nxt` | - | combinational |
| `w_mw_sof_x` | - | combinational |
| `w_mw_word_x` | - | combinational |
| `w_mw_act_valid_nxt` | - | combinational |
| `w_mw_act_buf_nxt` | - | combinational |
| `w_tok_valid` | `cpl_token` | combinational |
| `w_cpl_pop` | - | combinational |
| `w_cpl_dout` | `cpl_*` | combinational |
| `w_tok_resp` | `cpl.resp` | combinational |
| `w_tok_id` | `cpl.id` | combinational |
| `w_tok_addr` | `cpl.addr` | combinational |
| `w_tok_vc` | `cpl.vc` | combinational |
| `w_tok_eol` | `cpl.eol` | combinational |
| `w_tok_line` | `cpl.line` | combinational |
| `w_axi_err` | `axi_err[b]` | combinational |
| `w_cpl_token` | `cpl_token[b]` | combinational |
| `w_axi_write_error` | `dma_axi_write_error` | combinational |
| `w_overflow` | `dma_buffer_overflow` | combinational |
| `w_cpl_empty` | `empty (completion FIFO)` | combinational |
| `reg_bresp_slverr_count` | `bresp_slverr_count` | flip-flop |
| `reg_bresp_decerr_count` | `bresp_decerr_count` | flip-flop |
| `reg_first_fault` | - | flip-flop |
| `reg_axi_fault_awid` | `AXI_FAULT_TRACE.awid` | flip-flop |
| `reg_axi_fault_resp` | `AXI_FAULT_TRACE.resp` | flip-flop |
| `reg_axi_fault_awaddr` | `AXI_FAULT_ADDR` | flip-flop |
| `reg_lvl` | - | flip-flop |
| `reg_lvl_has` | - | flip-flop |
| `reg_lvl_stop` | - | flip-flop |
| `reg_lvl_claimed` | - | flip-flop |
| `w_buf_last_valid_line` | `bufN_last_valid_line` | combinational |
| `w_dma_busy` | `dma_busy` | combinational |
| `w_buf_state_code` | `BUFFER_OWNERSHIP` | combinational |
| `w_raw_cmd_full` | `raw_cmd_full` | combinational |
| `w_raw_cmd_empty` | - | combinational |
| `w_raw_cmd_pop` | - | combinational |
| `w_raw_data_full` | `raw_data_full` | combinational |
| `w_raw_data_empty` | - | combinational |
| `w_raw_data_pop` | - | combinational |
| `w_meta_cmd_full` | `meta_cmd_full` | combinational |
| `w_meta_cmd_empty` | - | combinational |
| `w_meta_cmd_pop` | - | combinational |
| `w_meta_data_full` | `meta_data_full` | combinational |
| `w_meta_data_empty` | - | combinational |
| `w_meta_data_pop` | - | combinational |
| `w_cpl_full` | - | combinational |
| `w_cpl_push` | - | combinational |
| `w_raw_cmd_dout` | - | combinational |
| `w_meta_cmd_dout` | - | combinational |
| `w_raw_data_dout` | - | combinational |
| `w_meta_data_dout` | - | combinational |
| `w_cpl_din` | `cpl_*` | combinational |
| `w_cpl_used` | `cpl_used` | combinational |
| `reg_awvalid` | `m_axi4_awvalid` | flip-flop |
| `reg_awid` | `m_axi4_awid` | flip-flop |
| `reg_awaddr` | `m_axi4_awaddr` | flip-flop |
| `reg_awlen` | `m_axi4_awlen` | flip-flop |
| `reg_awqos` | `m_axi4_awqos` | flip-flop |
| `reg_awprot` | `m_axi4_awprot` | flip-flop |
| `reg_aw_outst` | - | flip-flop |
| `reg_wq_stream` | `W order queue.stream` | flip-flop |
| `reg_wq_skip` | `W order queue.skip` | flip-flop |
| `reg_wq_beats` | `W order queue.length` | flip-flop |
| `reg_wq_wr_ptr` | - | flip-flop |
| `reg_wq_rd_ptr` | - | flip-flop |
| `reg_w_cnt` | - | flip-flop |
| `reg_w_half` | - | flip-flop |
| `reg_wvalid` | `m_axi4_wvalid` | flip-flop |
| `reg_wlast` | `m_axi4_wlast` | flip-flop |
| `reg_wdata` | `m_axi4_wdata` | flip-flop |
| `reg_wstrb` | `m_axi4_wstrb` | flip-flop |
| `reg_tag_mem` | `tag FIFO` | flip-flop |
| `reg_tag_wr_ptr` | - | flip-flop |
| `reg_tag_rd_ptr` | - | flip-flop |
| `w_inflight_mm` | `inflight_mm` | combinational |
| `w_issue_room` | `inflight_mm < MAX_OUTSTANDING` | combinational |
| `w_meta_cmd_ok` | - | combinational |
| `w_raw_cmd_ok` | - | combinational |
| `w_wq_full` | - | combinational |
| `w_wq_empty` | - | combinational |
| `w_take_meta` | - | combinational |
| `w_take_raw` | - | combinational |
| `w_take_write` | `issue_ok` | combinational |
| `w_wq_push` | - | combinational |
| `w_wq_pop` | - | combinational |
| `w_sel_cmd` | - | combinational |
| `w_head_stream` | - | combinational |
| `w_head_skip` | - | combinational |
| `w_data_avail` | - | combinational |
| `w_w_free` | - | combinational |
| `w_w_load` | - | combinational |
| `w_head_last` | - | combinational |
| `w_head_beats` | - | combinational |
| `w_head_data` | - | combinational |
| `w_axi_last_beat` | - | combinational |
| `w_b_hs` | - | combinational |
| `w_b_id` | `m_axi4_bid[1:0]` | combinational |
| `w_fsm_buf_nxt_state` | `state_next[b]` | combinational |
| `w_exp_lines` | `exp[b][x]` | combinational |
| `w_ch_done` | `ch_done[b][x]` | combinational |
| `w_all_done` | `all_done[b]` | combinational |
| `w_line_err` | `line_err[b]` | combinational |
| `w_claim_b` | `claim && pick == b` | combinational |
| `u_raw_cmd_fifo_0` | `RAW command FIFO` | instance |
| `u_raw_data_fifo_0` | `RAW data FIFO` | instance |
| `u_meta_cmd_fifo_0` | `metadata command FIFO (DMA_ASYNC_FIFO_METADATA)` | instance |
| `u_meta_data_fifo_0` | `metadata data FIFO (DMA_ASYNC_FIFO_METADATA)` | instance |
| `u_cpl_fifo_0` | `completion FIFO` | instance |
| `gen_para_check_axi` | - | generate block |
| `gen_para_check_burst` | - | generate block |
| `gen_para_check_fifo` | - | generate block |
| `gen_para_check_cdc` | - | generate block |
| `gen_buf` | `Buffer FSM (one per buffer)` | generate block |
| `f_popcount8` | `popcount` | function |
| `f_raw_lim` | - | function |
| `f_meta_lim` | - | function |
| `f_meta_base` | `meta_base[b][x]` | function |

## Mapping — m_fpt_dma_async_fifo

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_dma_async_fifo` | `Asynchronous FIFO (template of the five FIFOs)` | module |
| `i_wr_clk` | `wr_clk` | input port |
| `i_wr_rst_n` | `wr_rst` | input port |
| `i_push` | `push` | input port |
| `i_din` | `din` | input port |
| `o_full` | `full` | output port |
| `o_wr_used` | `cpl_used` | output port |
| `i_rd_clk` | `rd_clk` | input port |
| `i_rd_rst_n` | `rd_rst` | input port |
| `i_pop` | `pop` | input port |
| `o_dout` | `dout` | output port |
| `o_empty` | `empty` | output port |
| `PARA_DEPTH` | `RAM DEPTH` | parameter |
| `PARA_WIDTH` | `RAM WIDTH` | parameter |
| `PARA_ASYNC` | `ASYNC_MM_CLK` | parameter |
| `PARA_SYNC_STAGES` | `SYNC_STAGES` | parameter |
| `LPARA_ADDR_W` | - | localparam |
| `LPARA_PTR_W` | - | localparam |
| `LPARA_FULL_MASK` | - | localparam |
| `reg_ram` | `RAM` | flip-flop |
| `reg_wptr_bin` | `wptr_bin` | flip-flop |
| `reg_wptr_gray` | `wptr_gray` | flip-flop |
| `w_wptr_bin_nxt` | - | combinational |
| `w_wptr_gray_nxt` | `wptr_gray_next` | combinational |
| `reg_rptr_bin` | `rptr_bin` | flip-flop |
| `reg_rptr_gray` | `rptr_gray` | flip-flop |
| `w_rptr_bin_nxt` | - | combinational |
| `w_rptr_gray_nxt` | `rptr_gray_next` | combinational |
| `w_rq2_rptr` | `rq2_rptr` | combinational |
| `w_wq2_wptr` | `wq2_wptr` | combinational |
| `w_rq2_rptr_bin` | - | combinational |
| `reg_full` | `full` | flip-flop |
| `reg_empty` | `empty` | flip-flop |
| `w_push_ok` | - | combinational |
| `w_pop_ok` | - | combinational |
| `reg_dout` | `dout` | flip-flop |
| `reg_rq_rptr_sync` | `rq2_rptr` | flip-flop |
| `reg_wq_wptr_sync` | `wq2_wptr` | flip-flop |
| `reg_wptr_dly` | - | flip-flop |
| `gen_para_check` | - | generate block |
| `gen_async` | `ASYNC_MM_CLK = 1` | generate block |
| `gen_sync` | `ASYNC_MM_CLK = 0` | generate block |
| `f_bin2gray` | - | function |
| `f_gray2bin` | - | function |
