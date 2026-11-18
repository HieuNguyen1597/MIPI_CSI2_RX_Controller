# Signal mapping — B12 AXI4-Stream Master

## Notes

- RTL module: `m_fpt_axi4_stream` (`00_src/rtl/m_fpt_axi4_stream.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- All spec parameters are fixed values, so the RTL declares them as `localparam`; they cannot be overridden at instantiation. `SKID_BUFFER_DEPTH` (fixed 2) has no RTL name: it is realized as the Output Register (`reg_out_*`) plus the Park Register (`reg_skid_*`).
- The spec suffix `(next)` / `_next` is the next value of a register; in an RTL name this is `w_…_nxt`.
- `reg_out_*` drive the `o_m_axis_*` ports; the abort beat of the Abort Override is loaded into the same Output Register.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_axi4_stream` | `AXI4-Stream Master (Block No. 12)` | module |
| `i_axis_aclk` | `axis_aclk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_fbs_data` | `fbs_data` | input port |
| `i_fbs_valid` | `fbs_valid` | input port |
| `i_fbs_byte_en` | `fbs_byte_en` | input port |
| `i_fbs_sof` | `fbs_sof` | input port |
| `i_fbs_eol` | `fbs_eol` | input port |
| `i_fbs_vc_tag` | `fbs_vc_tag` | input port |
| `i_fbs_err` | `fbs_err` | input port |
| `o_fbs_ready` | `fbs_ready` | output port |
| `i_asm_abort_req` | `asm_abort_req` | input port |
| `i_asm_abort_mask` | `asm_abort_mask` | input port |
| `o_asm_abort_done` | `asm_abort_done` | output port |
| `i_asm_skip_req` | `asm_skip_req` | input port |
| `i_asm_skip_mask` | `asm_skip_mask` | input port |
| `i_axis_enable` | `axis_enable` | input port |
| `i_vc_exposure_map` | `vc_exposure_map` | input port |
| `o_m_axis_tdata` | `m_axis_tdata` | output port |
| `o_m_axis_tvalid` | `m_axis_tvalid` | output port |
| `i_m_axis_tready` | `m_axis_tready` | input port |
| `o_m_axis_tlast` | `m_axis_tlast` | output port |
| `o_m_axis_tuser` | `m_axis_tuser` | output port |
| `o_m_axis_tid` | `m_axis_tid` | output port |
| `o_m_axis_tkeep` | `m_axis_tkeep` | output port |
| `LPARA_TDATA_WIDTH` | `TDATA_WIDTH` | localparam |
| `LPARA_TKEEP_WIDTH` | `TKEEP_WIDTH` | localparam |
| `LPARA_TUSER_WIDTH` | `TUSER_WIDTH` | localparam |
| `LPARA_TID_WIDTH` | `TID_WIDTH` | localparam |
| `fseq_state_type_def` | `fseq_state` | typedef (enum) |
| `ENUM_ST_PASSTHROUGH` | `PASSTHROUGH` | enum member |
| `ENUM_ST_DRAIN` | `DRAIN` | enum member |
| `ENUM_ST_INJECT` | `INJECT` | enum member |
| `reg_fsm_fseq_current_state` | `fseq_state` | flip-flop |
| `w_fsm_fseq_nxt_state` | `fseq_state (next)` | combinational |
| `w_axis_in_valid` | `axis_in_valid` | combinational |
| `w_axis_in_data` | `axis_in_data` | combinational |
| `w_axis_in_keep` | `axis_in_keep` | combinational |
| `w_axis_in_last` | `axis_in_last` | combinational |
| `w_axis_in_user` | `axis_in_user` | combinational |
| `w_axis_in_tid` | `axis_in_tid` | combinational |
| `w_sof_0` | `sof_0` | combinational |
| `w_sof_1` | `sof_1` | combinational |
| `w_pass_nxt` | `pass_next` | combinational |
| `w_gate_pass` | `gate_pass` | combinational |
| `reg_pass` | `pass` | flip-flop |
| `reg_skip_arm` | `skip_arm` | flip-flop |
| `w_fbs_ready` | `fbs_ready` | combinational |
| `w_take` | `take` | combinational |
| `w_accepted` | `accepted` | combinational |
| `w_out_free` | - | combinational |
| `w_sink` | `sink` | combinational |
| `w_drained` | `drained` | combinational |
| `w_abort_hs` | `abort_hs` | combinational |
| `w_abort_mask_nxt` | `abort_mask_next` | combinational |
| `w_line_open_nxt` | `line_open_next` | combinational |
| `w_todo` | `todo` | combinational |
| `w_inj_tid` | `inj_tid` | combinational |
| `w_abort_start` | `abort_start` | combinational |
| `w_inject_load` | `inject_load` | combinational |
| `w_asm_abort_done` | `asm_abort_done` | combinational |
| `reg_abort_mask` | `abort_mask` | flip-flop |
| `reg_line_open` | `line_open` | flip-flop |
| `reg_line_tid` | `line_tid` | flip-flop |
| `reg_skid_valid` | `skid_valid` | flip-flop |
| `reg_skid_data` | `skid_data` | flip-flop |
| `reg_skid_keep` | `skid_keep` | flip-flop |
| `reg_skid_last` | `skid_last` | flip-flop |
| `reg_skid_user` | `skid_user` | flip-flop |
| `reg_skid_tid` | `skid_tid` | flip-flop |
| `reg_out_valid` | `out_valid` | flip-flop |
| `reg_out_data` | `out_data` | flip-flop |
| `reg_out_keep` | `out_keep` | flip-flop |
| `reg_out_last` | `out_last` | flip-flop |
| `reg_out_user` | `out_user` | flip-flop |
| `reg_out_tid` | `out_tid` | flip-flop |
