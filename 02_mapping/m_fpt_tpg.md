# Signal mapping — B05 Test Pattern Generator (TPG)

## Notes

- RTL module: `m_fpt_tpg` (`00_src/rtl/m_fpt_tpg.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4, Section 6.7: Table 6-23 (parameters), Table 6-24 (interfaces), Table 6-26 (Control FSM state outputs), Figures 6-18 to 6-22; draw.io diagram page "#5 TPG".
- In an RTL name, `[k]` is the band threshold index (k = 1 … 7) and `[n]` is the pixel index within one cycle (n = 0 … 3); the spec names use the `_k` / `_n` suffix.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_tpg` | Test Pattern Generator (Block No. 5) | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_core_enable` | `core_enable` | input port |
| `i_tpg_enable` | `tpg_enable` | input port |
| `i_tpg_pattern` | `tpg_pattern` | input port |
| `i_tpg_width` | `tpg_width` | input port |
| `i_tpg_height` | `tpg_height` | input port |
| `i_tpg_dual_vc_en` | `tpg_dual_vc_en` | input port |
| `i_px_ready` | `px_ready` | input port |
| `i_tpg_pixel_format` | `tpg_pixel_format` | input port |
| `i_tpg_hblank` | `tpg_hblank` | input port |
| `i_tpg_vblank` | `tpg_vblank` | input port |
| `i_tpg_cfa_order` | `tpg_cfa_order` | input port |
| `i_linebuf_empty` | `linebuf_empty` | input port |
| `o_px_data` | `px_data` | output port |
| `o_px_valid` | `px_valid` | output port |
| `o_px_sof` | `px_sof` | output port |
| `o_px_sol` | `px_sol` | output port |
| `o_px_eol` | `px_eol` | output port |
| `o_px_vc_tag` | `px_vc_tag` | output port |
| `o_px_dt_sel` | `px_dt_sel` | output port |
| `o_tpg_eof` | `tpg_eof` | output port |
| `o_tpg_fs` | `tpg_fs` | output port |
| `o_tpg_fe` | `tpg_fe` | output port |
| `o_tpg_fvc` | `tpg_fvc` | output port |
| `o_tpg_height_shadow` | `tpg_height_shadow` | output port |
| `LPARA_N_BANDS` | `N_BANDS` | localparam |
| `LPARA_PIXELS_PER_CYCLE` | `PIXELS_PER_CYCLE` | localparam |
| `LPARA_BAR_LEVEL` | `BAR_LEVEL` | localparam |
| `LPARA_GUARD_DIMENSION` | `GUARD_DIMENSION` | localparam |
| `LPARA_BLANK_CNT_WIDTH` | `BLANK_CNT_WIDTH` | localparam |
| `LPARA_VBLANK_LINE_CNT_WIDTH` | `VBLANK_LINE_CNT_WIDTH` | localparam |
| `LPARA_QUAD_SHIFT` | - | localparam |
| `LPARA_BAND_IDX_W` | - | localparam |
| `LPARA_PX_DATA_W` | - | localparam |
| `LPARA_X_STEP` | - | localparam |
| `LPARA_GUARD_SIZE` | - | localparam |
| `LPARA_WIDTH_MASK` | - | localparam |
| `tpg_state_type_def` | - | typedef (enum) |
| `ENUM_ST_IDLE` | `IDLE` | enum member |
| `ENUM_ST_CALC_THRESHOLDS` | `CALC_THRESHOLDS` | enum member |
| `ENUM_ST_RUNNING` | `RUNNING` | enum member |
| `ENUM_ST_STOPPING` | `STOPPING` | enum member |
| `reg_fsm_tpg_current_state` | `state` | flip-flop |
| `w_fsm_tpg_nxt_state` | - | combinational |
| `w_run_en` | `run_en` | combinational |
| `w_guard_ok` | `guard_ok` | combinational |
| `w_tpg_width_truncated` | `{tpg_width[15:2], 2'b00}` | combinational |
| `w_gen_active` | `gen_active` | combinational |
| `w_latch_en` | `latch_en` | combinational |
| `w_calc_done` | `calc_done` | combinational |
| `w_blank_clr` | `blank_clr` | combinational |
| `w_clr_pend` | `clr_pend` | combinational |
| `reg_width_shadow` | `width_shadow` | flip-flop |
| `reg_height_shadow` | `height_shadow` | flip-flop |
| `reg_pattern_shadow` | `pattern_shadow` | flip-flop |
| `reg_format_shadow` | `format_shadow` | flip-flop |
| `reg_dual_vc_shadow` | `dual_vc_shadow` | flip-flop |
| `reg_hblank_shadow` | `hblank_shadow` | flip-flop |
| `reg_vblank_shadow` | `vblank_shadow` | flip-flop |
| `reg_cfa_shadow` | `cfa_shadow` | flip-flop |
| `w_band_base` | `band_base` | combinational |
| `w_threshold[k]` | `threshold_k` | combinational |
| `reg_threshold[k]` | `threshold_k / thr_1..7` | flip-flop |
| `reg_x_cnt` | `x_cnt` | flip-flop |
| `reg_y_cnt` | `y_cnt` | flip-flop |
| `reg_vc_toggle` | `vc_toggle` | flip-flop |
| `w_x_first` | `x_first` | combinational |
| `w_x_last` | `x_last` | combinational |
| `w_y_first` | `y_first` | combinational |
| `w_y_last` | `y_last` | combinational |
| `w_row_done` | `row_done` | combinational |
| `w_eof` | `EOF` | combinational |
| `w_target_coord[n]` | `target_coord_n` | combinational |
| `w_band_idx[n]` | `band_idx_n` | combinational |
| `w_a[n]` | `a_n` | combinational |
| `w_b` | `b` | combinational |
| `w_on[n]` | `on_n` | combinational |
| `w_pix12[n]` | `pix12_n` | combinational |
| `w_pixel[n]` | `pixel_n / pixel_n_val` | combinational |
| `w_px_data` | `px_data` | combinational |
| `w_px_valid` | `px_valid` | combinational |
| `w_accept` | `accept` | combinational |
| `w_cnt_en` | `cnt_en` | combinational |
| `w_px_sof` | `px_sof` | combinational |
| `w_px_sol` | `px_sol` | combinational |
| `w_px_eol` | `px_eol` | combinational |
| `w_px_vc_tag` | `px_vc_tag` | combinational |
| `w_line_done` | `line_done` | combinational |
| `w_frame_done` | `frame_done` | combinational |
| `w_tpg_fs` | `tpg_fs` | combinational |
| `w_tpg_fe` | `tpg_fe` | combinational |
| `reg_eof_pend` | `eof_pend` | flip-flop |
| `reg_blank_cnt` | `blank_cnt` | flip-flop |
| `w_line_period` | `line_period` | combinational |
| `w_blank_nxt` | `blank_nxt` | combinational |
| `w_blank_active` | `blank_active` | combinational |
| `reg_vblank_line_cnt` | `vblank_line_cnt` | flip-flop |
| `w_vblank_nxt` | `vblank_nxt` | combinational |
| `w_vblank_reload` | `vblank_reload` | combinational |
