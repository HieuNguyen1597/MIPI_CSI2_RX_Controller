# Signal mapping — B06 TPG Mux

## Notes

- RTL module: `m_fpt_tpg_mux` (`00_src/rtl/m_fpt_tpg_mux.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- In `set_v` / `clr_v`, `v` is the virtual channel index; the RTL has one signal per channel, `_0` / `_1`.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_tpg_mux` | `TPG Mux (Block No. 6)` | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_rx_px_data` | `rx_px_data` | input port |
| `i_rx_px_valid` | `rx_px_valid` | input port |
| `i_rx_px_valid_count` | `rx_px_valid_count` | input port |
| `i_rx_px_sof` | `rx_px_sof` | input port |
| `i_rx_px_sol` | `rx_px_sol` | input port |
| `i_rx_px_eol` | `rx_px_eol` | input port |
| `i_rx_px_vc_tag` | `rx_px_vc_tag` | input port |
| `i_rx_px_dt_sel` | `rx_px_dt_sel` | input port |
| `i_rx_err_wc_align` | `rx_err_wc_align` | input port |
| `i_fc_fs` | `fc_fs` | input port |
| `i_fc_fe` | `fc_fe` | input port |
| `i_fc_vc` | `fc_vc` | input port |
| `i_rx_ctx_clear` | `rx_ctx_clear` | input port |
| `i_tpg_px_data` | `tpg_px_data` | input port |
| `i_tpg_px_valid` | `tpg_px_valid` | input port |
| `i_tpg_px_sof` | `tpg_px_sof` | input port |
| `i_tpg_px_sol` | `tpg_px_sol` | input port |
| `i_tpg_px_eol` | `tpg_px_eol` | input port |
| `i_tpg_px_vc_tag` | `tpg_px_vc_tag` | input port |
| `i_tpg_px_dt_sel` | `tpg_px_dt_sel` | input port |
| `i_tpg_fs` | `tpg_fs` | input port |
| `i_tpg_eof` | `tpg_eof` | input port |
| `i_linebuffer_full` | `linebuffer_full` | input port |
| `i_linebuf_empty` | `linebuf_empty` | input port |
| `o_tpg_px_ready` | `tpg_px_ready` | output port |
| `i_tpg_enable` | `tpg_enable` | input port |
| `o_px_data` | `px_data` | output port |
| `o_px_valid` | `px_valid` | output port |
| `o_px_valid_count` | `px_valid_count` | output port |
| `o_px_sof` | `px_sof` | output port |
| `o_px_sol` | `px_sol` | output port |
| `o_px_eol` | `px_eol` | output port |
| `o_px_vc_tag` | `px_vc_tag` | output port |
| `o_px_dt_sel` | `px_dt_sel` | output port |
| `o_err_wc_align` | `err_wc_align` | output port |
| `o_active_sel` | `active_sel` | output port |
| `o_switch_pending` | `switch_pending` | output port |
| `LPARA_DATA_WIDTH` | `DATA_WIDTH` | localparam |
| `LPARA_VC_TAG_WIDTH` | `VC_TAG_WIDTH` | localparam |
| `LPARA_ACTIVE_SEL_RX` | `ACTIVE_SEL_ENCODING (0 = RX)` | localparam |
| `LPARA_ACTIVE_SEL_TPG` | `ACTIVE_SEL_ENCODING (1 = TPG)` | localparam |
| `mux_state_type_def` | - | typedef (enum) |
| `ENUM_ST_STREAMING` | `STREAMING` | enum member |
| `ENUM_ST_DRAINING` | `DRAINING` | enum member |
| `ENUM_ST_ACQUIRING` | `ACQUIRING` | enum member |
| `reg_fsm_mux_current_state` | `state (Switch Control FSM)` | flip-flop |
| `reg_active_sel` | `active_sel` | flip-flop |
| `reg_armed` | `armed` | flip-flop |
| `reg_vc_active` | `vc_active[1:0]` | flip-flop |
| `reg_tpg_frame_active` | `tpg_frame_active` | flip-flop |
| `w_fsm_mux_nxt_state` | - | combinational |
| `w_set_0` | `set_v (v = 0)` | combinational |
| `w_set_1` | `set_v (v = 1)` | combinational |
| `w_clr_0` | `clr_v (v = 0)` | combinational |
| `w_clr_1` | `clr_v (v = 1)` | combinational |
| `w_pending_sel` | `pending_sel` | combinational |
| `w_rx_frame_active` | `rx_frame_active` | combinational |
| `w_rx_frame_start` | - | combinational |
| `w_rx_frame_busy` | - | combinational |
| `w_outgoing_active` | `outgoing_active` | combinational |
| `w_outgoing_valid` | `outgoing_valid` | combinational |
| `w_outgoing_idle` | `outgoing_idle` | combinational |
| `w_incoming_idle` | `incoming_idle` | combinational |
| `w_sel_mismatch` | `sel_mismatch` | combinational |
| `w_drain_done` | `drain_done` | combinational |
| `w_write_en` | `write_en` | combinational |
| `w_switch_pending` | `switch_pending` | combinational |
| `w_incoming_sof` | `incoming_sof` | combinational |
| `w_output_mask` | `output_mask` | combinational |
| `w_tpg_px_ready` | `tpg_px_ready` | combinational |
| `w_px_data` | `px_data` | combinational |
| `w_px_valid` | `px_valid` | combinational |
| `w_px_valid_count` | `px_valid_count` | combinational |
| `w_px_sof` | `px_sof` | combinational |
| `w_px_sol` | `px_sol` | combinational |
| `w_px_eol` | `px_eol` | combinational |
| `w_px_vc_tag` | `px_vc_tag` | combinational |
| `w_px_dt_sel` | `px_dt_sel` | combinational |
| `w_err_wc_align` | `err_wc_align` | combinational |
