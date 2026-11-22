# Signal mapping — B10 Control FSM

## Notes

- RTL module: `m_fpt_control_fsm` (`00_src/rtl/m_fpt_control_fsm.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- `drop_state_type_def` is the state type of the Drop FSM, `ENUM_ST_*` are its states and `reg_fsm_drop_current_state` / `w_fsm_drop_nxt_state` are the state register and its next value.
- In a spec name, `[s]` is the capture slot index and `[x]` the virtual channel index; the RTL keeps one bit per slot or channel in a vector (`reg_cap_open`, `reg_dma_mark`, `reg_discard`).
- The spec suffix `_q` is the registered value; in an RTL name this is `reg_…`.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_control_fsm` | `Control FSM (Block No. 10)` | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_core_enable` | `core_enable` | input port |
| `i_axis_enable` | `axis_enable` | input port |
| `i_stream_follow_dma_drop` | `stream_follow_dma_drop` | input port |
| `i_vc_exposure_map` | `vc_exposure_map` | input port |
| `i_tpg_dual_vc_en` | `tpg_dual_vc_en` | input port |
| `i_err_ecc_2bit` | `err_ecc_2bit` | input port |
| `i_err_lane_abort` | `err_lane_abort` | input port |
| `i_err_crc` | `err_crc` | input port |
| `i_err_wc` | `err_wc` | input port |
| `i_err_vc` | `err_vc[1:0]` | input port |
| `o_force_discard` | `force_discard[1:0]` | output port |
| `i_active_sel` | `active_sel` | input port |
| `o_rx_ctx_clear` | `rx_ctx_clear` | output port |
| `i_linebuffer_full` | `linebuffer_full` | input port |
| `i_linebuf_overflow` | `linebuf_overflow` | input port |
| `o_lb_flush` | `lb_flush` | output port |
| `i_fb_overflow` | `fb_overflow` | input port |
| `o_fb_flush` | `fb_flush` | output port |
| `i_capture_start` | `capture_start[1:0]` | input port |
| `i_capture_end` | `capture_end[1:0]` | input port |
| `i_capture_dropped` | `capture_dropped[1:0]` | input port |
| `i_ev_fs` | `ev_fs` | input port |
| `i_ev_vc` | `ev_vc[1:0]` | input port |
| `i_lead_vc` | `lead_vc` | input port |
| `o_capture_drop` | `capture_drop` | output port |
| `o_drop_vc_mask` | `drop_vc_mask[1:0]` | output port |
| `o_asm_abort_req` | `asm_abort_req` | output port |
| `o_asm_abort_mask` | `asm_abort_mask[1:0]` | output port |
| `o_asm_skip_req` | `asm_skip_req` | output port |
| `o_asm_skip_mask` | `asm_skip_mask[1:0]` | output port |
| `i_dma_drop` | `dma_drop` | input port |
| `o_dma_abort_req` | `dma_abort_req` | output port |
| `o_frame_drop_pulse` | `frame_drop_pulse` | output port |
| `LPARA_HOLD_CYCLES` | `HOLD_CYCLES` | localparam |
| `LPARA_HOLD_CNT_W` | - | localparam |
| `LPARA_HOLD_LAST` | `HOLD_CYCLES - 1` | localparam |
| `drop_state_type_def` | `Drop FSM states` | typedef (enum) |
| `ENUM_ST_IDLE` | `ST_IDLE` | enum member |
| `ENUM_ST_RUN` | `ST_RUN` | enum member |
| `ENUM_ST_HOLD` | `ST_HOLD` | enum member |
| `ENUM_ST_FLUSH` | `ST_FLUSH` | enum member |
| `ENUM_ST_SKIP` | `ST_SKIP` | enum member |
| `w_rx` | `rx` | combinational |
| `w_vc_ok` | `vc_ok` | combinational |
| `w_c_ecc2` | `c_ecc2` | combinational |
| `w_c_crc` | `c_crc` | combinational |
| `w_c_wc` | `c_wc` | combinational |
| `w_c_lane` | `c_lane` | combinational |
| `w_c_lbfull` | `c_lbfull` | combinational |
| `w_c_lbovf` | `c_lbovf` | combinational |
| `w_c_fbovf` | `c_fbovf` | combinational |
| `w_c_core` | `c_core` | combinational |
| `w_abort_any` | `abort_any` | combinational |
| `w_midline` | `midline` | combinational |
| `w_stream_stop` | `stream_stop` | combinational |
| `w_dma_skip` | `dma_skip` | combinational |
| `w_dma_evt` | `dma_evt / c_dma` | combinational |
| `reg_axis_enable` | `axis_enable_q` | flip-flop |
| `reg_cap_open` | `cap_open[s] / capture_open[s]` | flip-flop |
| `w_cap_busy` | `cap_busy` | combinational |
| `w_lead_start` | `lead_start` | combinational |
| `w_lag_fs` | `lag_fs` | combinational |
| `reg_last_slot` | `last_slot` | flip-flop |
| `reg_dma_mark` | `dma_mark[s]` | flip-flop |
| `w_drop_end` | `drop_end[s]` | combinational |
| `reg_drop_pend` | `drop_pend` | flip-flop |
| `w_frame_drop_pulse` | `frame_drop_pulse` | combinational |
| `reg_fsm_drop_current_state` | `state` | flip-flop |
| `w_fsm_drop_nxt_state` | `Drop FSM next state` | combinational |
| `reg_hold_cnt` | `hold_cnt` | flip-flop |
| `w_hold_cnt_nxt` | - | combinational |
| `w_hold_last` | `hold_cnt == HOLD_CYCLES - 1` | combinational |
| `reg_lead_seen` | `lead_seen` | flip-flop |
| `w_lead_seen_nxt` | - | combinational |
| `w_drop_now` | `drop_now` | combinational |
| `w_skip_now` | `skip_now` | combinational |
| `reg_drop_now` | `drop_now_q` | flip-flop |
| `reg_skip_now` | `skip_now_q` | flip-flop |
| `reg_core_enable` | `core_enable_q` | flip-flop |
| `reg_stream_stop` | `stream_stop_q` | flip-flop |
| `w_s` | `S` | combinational |
| `w_go_run` | `go_run` | combinational |
| `w_clr` | `clr[x]` | combinational |
| `reg_discard` | `discard[x]` | flip-flop |
| `w_capture_drop` | `capture_drop` | combinational |
| `w_drop_vc_mask` | `drop_vc_mask` | combinational |
| `w_asm_abort_req` | `asm_abort_req` | combinational |
| `w_asm_abort_mask` | `asm_abort_mask` | combinational |
| `w_asm_skip_mask` | `asm_skip_mask` | combinational |
| `w_dma_abort_req` | `dma_abort_req` | combinational |
| `w_lb_flush` | `lb_flush, fb_flush` | combinational |
| `w_rx_ctx_clear` | `rx_ctx_clear` | combinational |
