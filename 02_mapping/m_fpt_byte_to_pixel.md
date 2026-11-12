# Signal mapping — B04 Byte-to-Pixel Unpacking

## Notes

- RTL module: `m_fpt_byte_to_pixel` (`00_src/rtl/m_fpt_byte_to_pixel.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
- All spec parameters are fixed values, so the RTL declares them as `localparam`; they cannot be overridden at instantiation. `OUT_PIPE_STAGES` (fixed 1) has no RTL name: it is realized as the single output register stage (`reg_px_*`, `reg_err_wc_align`), giving 1 `sys_clk` cycle of latency.
- In the spec names `sof_pend_n` and `sol_pend_n`, the `_n` suffix means "next value", not active-low.
- A `reg_px_*` / `reg_err_wc_align` row maps to the Output Register stage of the spec signal; the matching `o_*` port is driven from it.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_byte_to_pixel` | Byte-to-Pixel Unpacking (Block No. 4) | module |
| `LPARA_DATA_IN_WIDTH` | `DATA_IN_WIDTH` | localparam |
| `LPARA_DATA_OUT_WIDTH` | `DATA_OUT_WIDTH` | localparam |
| `LPARA_MAX_PIXELS_PER_CYCLE` | `MAX_PIXELS_PER_CYCLE` | localparam |
| `LPARA_K_RAW12` | `K_RAW12` | localparam |
| `LPARA_K_RAW10` | `K_RAW10` | localparam |
| `LPARA_CARRY_BUF_WIDTH` | `CARRY_BUF_WIDTH` | localparam |
| `LPARA_WORK_BUF_WIDTH` | `WORK_BUF_WIDTH` | localparam |
| `LPARA_BYTE_NUM` | - | localparam |
| `LPARA_VALID_COUNT_WIDTH` | - | localparam |
| `LPARA_NEW_BYTE_CNT_WIDTH` | - | localparam |
| `LPARA_CARRY_CNT_WIDTH` | - | localparam |
| `LPARA_K_WIDTH` | - | localparam |
| `LPARA_WORK_CNT_WIDTH` | - | localparam |
| `LPARA_UNITS_WIDTH` | - | localparam |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_bp_data` | `bp_data` | input port |
| `i_bp_valid` | `bp_valid` | input port |
| `i_bp_byte_en` | `bp_byte_en` | input port |
| `i_bp_sof` | `bp_sof` | input port |
| `i_bp_sol` | `bp_sol` | input port |
| `i_bp_eol` | `bp_eol` | input port |
| `i_bp_vc_tag` | `bp_vc_tag` | input port |
| `i_bp_dt_sel` | `bp_dt_sel` | input port |
| `o_px_data` | `px_data` | output port |
| `o_px_valid` | `px_valid` | output port |
| `o_px_valid_count` | `px_valid_count` | output port |
| `o_px_sof` | `px_sof` | output port |
| `o_px_sol` | `px_sol` | output port |
| `o_px_eol` | `px_eol` | output port |
| `o_px_vc_tag` | `px_vc_tag` | output port |
| `o_px_dt_sel` | `px_dt_sel` | output port |
| `o_err_wc_align` | `err_wc_align` | output port |
| `reg_carry_buf` | `carry_buf` | flip-flop |
| `reg_carry_cnt` | `carry_cnt` | flip-flop |
| `reg_sof_pend` | `sof_pend` | flip-flop |
| `reg_sol_pend` | `sol_pend` | flip-flop |
| `reg_px_data` | `px_data` | flip-flop |
| `reg_px_valid` | `px_valid` | flip-flop |
| `reg_px_valid_count` | `px_valid_count` | flip-flop |
| `reg_px_sof` | `px_sof` | flip-flop |
| `reg_px_sol` | `px_sol` | flip-flop |
| `reg_px_eol` | `px_eol` | flip-flop |
| `reg_px_vc_tag` | `px_vc_tag` | flip-flop |
| `reg_px_dt_sel` | `px_dt_sel` | flip-flop |
| `reg_err_wc_align` | `err_wc_align` | flip-flop |
| `w_bp_data_m` | `bp_data_m` | combinational |
| `w_new_byte_cnt` | `new_byte_cnt` | combinational |
| `w_line_start` | `line_start` | combinational |
| `w_line_end` | `line_end` | combinational |
| `w_carry_in` | `carry_in` | combinational |
| `w_work_cnt` | `work_cnt` | combinational |
| `w_k` | `K` | combinational |
| `w_two_k_raw12` | - | combinational |
| `w_units_extracted` | `units_extracted` | combinational |
| `w_raw_next_carry` | `raw_next_carry` | combinational |
| `w_pad_unit` | `pad_unit` | combinational |
| `w_units_out` | `units_out` | combinational |
| `w_px_emit` | `px_emit` | combinational |
| `w_next_carry_cnt` | `next_carry_cnt` | combinational |
| `w_err_wc_align_pre` | - | combinational |
| `w_sof_pend_n` | `sof_pend_n` | combinational |
| `w_sol_pend_n` | `sol_pend_n` | combinational |
| `w_px_sof_pre` | `px_sof_pre` | combinational |
| `w_px_sol_pre` | `px_sol_pre` | combinational |
| `w_px_eol_pre` | `px_eol_pre` | combinational |
| `w_carry_bytes` | `carry_bytes` | combinational |
| `w_work_buf` | `work_buf` | combinational |
| `w_work_buf_extended` | - | combinational |
| `w_consumed_cnt` | - | combinational |
| `w_carry_next` | `carry_next` | combinational |
| `w_raw12_data` | - | combinational |
| `w_raw10_data` | - | combinational |
| `w_px_data_pre` | - | combinational |
| `w_px_valid_count_pre` | - | combinational |
| `p_com_byte_mask` | Byte Mask + Byte-Count Resolver | always_comb |
| `p_com_state_logic` | State Logic | always_comb |
| `p_com_sync_pending` | Sync Pending Logic | always_comb |
| `p_com_work_buffer` | Work Buffer Assembler + Carry Buffer Update | always_comb |
| `p_com_raw12_extractor` | RAW12 Extractor | always_comb |
| `p_com_raw10_extractor` | RAW10 Extractor | always_comb |
| `p_com_format_output_mux` | Format Output Mux | always_comb |
| `p_ff_state` | Carry Buffer + `carry_cnt` + `sof_pend` / `sol_pend` | always_ff |
| `p_ff_output` | Output Register | always_ff |
