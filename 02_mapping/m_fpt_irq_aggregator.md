# Signal mapping — B11 Interrupt & Error Aggregator

## Notes

- RTL module: `m_fpt_irq_aggregator` (`00_src/rtl/m_fpt_irq_aggregator.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- `gen_es` is the generate block of the Event Synchronizers ES0 … ES2; index 0 = ES0 (`errsoths`), 1 = ES1 (`errsotsynchs`), 2 = ES2 (`lane_abort_pulse`). Their signals (`w_ev_s`, `w_idle`, `w_send`, `reg_req_tgl`, `reg_pend`, `reg_ack_sync`, `reg_req_sync`, `reg_req`, `w_ev_d`) are vectors or arrays with one element per channel.
- Each `reg_x_*` row is the spec register `x_q` of the edge detector for the source named after `reg_x_`; the matching `w_rise_*` row is its rising edge.
- In the spec names, `_d` / `_n` is the next value and `_q` the registered value (not active-low); in an RTL name these are `w_…` / `w_…_nxt` and `reg_…`. `fid_q[0:1]` is indexed by virtual channel (VC0, VC1).
- `S` in `ack_sync[S-1:0]`, `req_sync[S-1:0]` and `stop_sync[S-1:0]` is `SYNC_STAGES` (`PARA_SYNC_STAGES`).
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_irq_aggregator` | `Interrupt & Error Aggregator (Block No. 11)` | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_rxbyteclkhs` | `rxbyteclkhs` | input port |
| `i_resetn` | `resetn` | input port |
| `i_rxbyteclkhs_resetn` | - | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_phy_rst_in_progress` | `phy_rst_in_progress` | input port |
| `i_global_irq_enable` | `global_irq_enable` | input port |
| `i_irq_enable_mask` | `irq_enable_mask[23:0]` | input port |
| `i_irq_clear_strobe` | `irq_clear_strobe[23:0]` | input port |
| `o_irq_status` | `irq_status[23:0]` | output port |
| `o_error_capture` | `error_capture[31:0]` | output port |
| `o_ecc_err_cnt` | `ecc_err_cnt[31:0]` | output port |
| `o_crc_err_cnt` | `crc_err_cnt[31:0]` | output port |
| `o_frame_drop_cnt` | `frame_drop_cnt[31:0]` | output port |
| `o_lane_abort_cnt` | `lane_abort_cnt[15:0]` | output port |
| `o_packet_cnt` | `packet_cnt[31:0]` | output port |
| `o_linebuf_ovf_sticky` | `linebuf_ovf_sticky` | output port |
| `o_sp_fifo_ovf_sticky` | `sp_fifo_ovf_sticky` | output port |
| `o_csi2_irq` | `csi2_irq` | output port |
| `i_errsoths` | `errsoths[MAX_LANES-1:0]` | input port |
| `i_errsotsynchs` | `errsotsynchs[MAX_LANES-1:0]` | input port |
| `i_cl_stopstate` | `cl_stopstate` | input port |
| `i_lane_abort_pulse` | `lane_abort_pulse` | input port |
| `i_err_frame_sync` | `err_frame_sync` | input port |
| `i_err_frame_level` | `err_frame_level` | input port |
| `i_err_crc` | `err_crc` | input port |
| `i_err_ecc_1bit` | `err_ecc_1bit` | input port |
| `i_err_ecc_2bit` | `err_ecc_2bit` | input port |
| `i_err_unsupported_dt` | `err_unsupported_dt` | input port |
| `i_err_wc` | `err_wc` | input port |
| `i_err_vc` | `err_vc[1:0]` | input port |
| `i_err_lane_abort` | `err_lane_abort` | input port |
| `i_packet_done_pulse` | `packet_done_pulse` | input port |
| `i_sp_fifo_overflow` | `sp_fifo_overflow` | input port |
| `i_err_wc_align` | `err_wc_align` | input port |
| `i_active_sel` | `active_sel` | input port |
| `i_linebuffer_full` | `linebuffer_full` | input port |
| `i_linebuf_overflow` | `linebuf_overflow` | input port |
| `i_fb_overflow` | `fb_overflow` | input port |
| `i_capture_done` | `capture_done` | input port |
| `i_err_resolution` | `err_resolution` | input port |
| `i_err_frame_num` | `err_frame_num` | input port |
| `i_cur_frame_id_vc0` | `cur_frame_id_vc0[13:0]` | input port |
| `i_cur_frame_id_vc1` | `cur_frame_id_vc1[13:0]` | input port |
| `i_cur_line_vc0` | `cur_line_vc0[15:0]` | input port |
| `i_cur_line_vc1` | `cur_line_vc1[15:0]` | input port |
| `i_meta_overflow` | `meta_overflow` | input port |
| `i_frame_drop_pulse` | `frame_drop_pulse` | input port |
| `i_dma_buf_done_ok` | `dma_buf_done_ok` | input port |
| `i_dma_buf_done_err` | `dma_buf_done_err` | input port |
| `i_dma_axi_write_error` | `dma_axi_write_error` | input port |
| `i_dma_buffer_overflow` | `dma_buffer_overflow` | input port |
| `o_dma_ovf_sticky` | `dma_ovf_sticky` | output port |
| `PARA_MAX_LANES` | `MAX_LANES` | parameter |
| `PARA_SYNC_STAGES` | `SYNC_STAGES` | parameter |
| `PARA_RST_TAIL` | `RST_TAIL` | parameter |
| `LPARA_IRQ_WIDTH` | `IRQ_WIDTH` | localparam |
| `LPARA_FRAME_ID_WIDTH` | `FRAME_ID_WIDTH` | localparam |
| `LPARA_TAIL_CNT_WIDTH` | - | localparam |
| `LPARA_ES_NUM` | - | localparam |
| `LPARA_TRG_NUM` | - | localparam |
| `LPARA_TRG_KNOWN` | `known[4:0]` | localparam |
| `LPARA_BIT_FRAME_SYNC_VC0` | - | localparam |
| `LPARA_BIT_FRAME_SYNC_VC1` | - | localparam |
| `LPARA_BIT_CRC_ERROR` | - | localparam |
| `LPARA_BIT_ECC_1BIT_ERROR` | - | localparam |
| `LPARA_BIT_ECC_2BIT_ERROR` | - | localparam |
| `LPARA_BIT_WC_CORRUPTION` | - | localparam |
| `LPARA_BIT_FRAME_DROPPED` | - | localparam |
| `LPARA_BIT_LANE_ALIGN_ABORT` | - | localparam |
| `w_ev_s` | `ev_s` | combinational |
| `w_idle` | `idle` | combinational |
| `w_send` | `send` | combinational |
| `reg_req_tgl` | `req_tgl` | flip-flop |
| `reg_pend` | `pend` | flip-flop |
| `reg_ack_sync` | `ack_sync[S-1:0]` | flip-flop |
| `reg_req_sync` | `req_sync[S-1:0]` | flip-flop |
| `reg_req` | `req_q` | flip-flop |
| `w_ev_d` | `ev_d` | combinational |
| `reg_stop_sync` | `stop_sync[S-1:0]` | flip-flop |
| `reg_stop` | `stop_q` | flip-flop |
| `w_ev_stop` | `ev_stop` | combinational |
| `w_x_linebuffer_full` | `linebuffer_full && !active_sel` | combinational |
| `reg_x_linebuffer_full` | `x_q` | flip-flop |
| `reg_x_linebuf_overflow` | `x_q` | flip-flop |
| `reg_x_sp_fifo_overflow` | `x_q` | flip-flop |
| `reg_x_meta_overflow` | `x_q` | flip-flop |
| `reg_x_dma_buffer_overflow` | `x_q` | flip-flop |
| `w_rise_linebuffer_full` | `rise(linebuffer_full && !active_sel)` | combinational |
| `w_rise_linebuf_overflow` | `rise(linebuf_overflow)` | combinational |
| `w_rise_sp_fifo_overflow` | `rise(sp_fifo_overflow)` | combinational |
| `w_rise_meta_overflow` | `rise(meta_overflow)` | combinational |
| `w_rise_dma_buffer_overflow` | `rise(dma_buffer_overflow)` | combinational |
| `w_evt` | `evt[23:0]` | combinational |
| `w_gate_src` | `gate_src` | combinational |
| `reg_tail_cnt` | `tail_cnt` | flip-flop |
| `w_evt_block` | `evt_block` | combinational |
| `w_evt_g` | `evt_g[23:0]` | combinational |
| `w_pkt_g` | `pkt_g` | combinational |
| `w_lb_ovf_g` | `lb_ovf_g` | combinational |
| `w_sp_ovf_g` | `sp_ovf_g` | combinational |
| `w_dma_ovf_g` | `dma_ovf_g` | combinational |
| `w_err_lane_abort_g` | `err_lane_abort_g` | combinational |
| `w_flag_d` | `flag_d[23:0]` | combinational |
| `reg_flag` | `flag_q[23:0]` | flip-flop |
| `w_irq_d` | `irq_d` | combinational |
| `reg_csi2_irq` | `csi2_irq` | flip-flop |
| `reg_ecc_1bit_count` | `ecc_1bit_count` | flip-flop |
| `reg_ecc_2bit_count` | `ecc_2bit_count` | flip-flop |
| `reg_crc_error_count` | `crc_error_count` | flip-flop |
| `reg_frame_drop_count` | `frame_drop_count` | flip-flop |
| `reg_lane_abort_count` | `lane_abort_count` | flip-flop |
| `reg_packet_count` | `packet_count` | flip-flop |
| `reg_lb_ovf` | `lb_ovf_q` | flip-flop |
| `reg_sp_ovf` | `sp_ovf_q` | flip-flop |
| `reg_dma_ovf` | `dma_ovf_q` | flip-flop |
| `reg_fid` | `fid_q[0:1]` | flip-flop |
| `w_chg` | `chg[1:0]` | combinational |
| `w_new_cap` | `new_cap` | combinational |
| `w_new_id` | `new_id` | combinational |
| `reg_newest_fid` | `newest_fid` | flip-flop |
| `reg_prev_fid` | `prev_fid` | flip-flop |
| `w_newest_nxt` | `newest_n` | combinational |
| `w_prev_nxt` | `prev_n` | combinational |
| `reg_arm_new` | `arm_new` | flip-flop |
| `reg_arm_prev` | `arm_prev` | flip-flop |
| `w_arm_new_nxt` | `arm_new_n` | combinational |
| `w_arm_prev_nxt` | `arm_prev_n` | combinational |
| `w_trg` | `trg[4:0]` | combinational |
| `w_vc_k` | `vc_k` | combinational |
| `w_fid_k` | `fid_k` | combinational |
| `w_line_k` | `line_k` | combinational |
| `w_hit_new` | `hit_new[4:0]` | combinational |
| `w_hit_prev` | `hit_prev[4:0]` | combinational |
| `w_hit_sel` | - | combinational |
| `w_rec_new` | `rec_new` | combinational |
| `w_rec_prev` | `rec_prev` | combinational |
| `w_known_sel` | `known_sel` | combinational |
| `w_fault_vc` | `fault_vc` | combinational |
| `w_fault_frame_id` | `fault_frame_id` | combinational |
| `w_fault_line_number` | `fault_line_number` | combinational |
| `reg_error_capture` | `error_capture` | flip-flop |
| `gen_para_check` | - | generate block |
| `gen_es` | `ES0 to ES2` | generate block |
