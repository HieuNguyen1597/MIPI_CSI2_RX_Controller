# Signal mapping — B12 AXI4-Stream Master

## Notes

- RTL module: `m_fpt_axi4_stream` (`00_src/rtl/m_fpt_axi4_stream.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.5.
- The block has two clock domains. The intake side (TID Map, Ready Logic, Frame Gate, Bundle Mapping, Finalize Sequencer, Line Tracker, Abort Override, write side of the flush) runs on `sys_clk` and is reset by `resetn`; the output side (Width Converter, Park Register, Output Register, read side of the flush) runs on `axis_aclk` and is reset by `axis_resetn`.
- The stream FIFO between the two domains is instance `u_stream_fifo_0` of `m_fpt_dma_async_fifo` (`00_src/rtl/m_fpt_dma_async_fifo.sv`), the asynchronous FIFO of Block No. 13; its signals are listed in `m_fpt_dma.md`. One FIFO entry is one 64-bit beat with its sideband: `{data, keep, last, user, tid}`.
- `SKID_BUFFER_DEPTH` (fixed 2) has no RTL name: it is realized as the Output Register (`reg_out_*`) plus the Park Register (`reg_skid_*`), both on `axis_aclk`. `reg_out_*` drive the `o_m_axis_*` ports.
- `reg_flush_req_sync` and `reg_flush_ack_sync` are synchroniser chains of `PARA_SYNC_STAGES` flops; each carries the flush request or acknowledge toggle into the other clock domain.
- `gen_para_check` stops elaboration for an out-of-range `PARA_SYNC_STAGES` or `PARA_AXIS_FIFO_DEPTH`.
- The spec suffix `(next)` / `_next` is the next value of a register; in an RTL name this is `w_…_nxt`.
- In `transfer k`, `k` is the index of the 16-bit transfer within the 64-bit head entry (0 to 3, ascending byte order).
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_axi4_stream` | `AXI4-Stream Master (Block No. 12)` | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_axis_aclk` | `axis_aclk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_axis_resetn` | `axis_resetn` | input port |
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
| `PARA_SYNC_STAGES` | `SYNC_STAGES` | parameter |
| `PARA_AXIS_FIFO_DEPTH` | `AXIS_FIFO_DEPTH` | parameter |
| `LPARA_TDATA_WIDTH` | `TDATA_WIDTH` | localparam |
| `LPARA_TKEEP_WIDTH` | `TKEEP_WIDTH` | localparam |
| `LPARA_TUSER_WIDTH` | `TUSER_WIDTH` | localparam |
| `LPARA_TID_WIDTH` | `TID_WIDTH` | localparam |
| `LPARA_FBS_DATA_WIDTH` | `DATA_OUT_WIDTH (Block No. 8)` | localparam |
| `LPARA_FBS_KEEP_WIDTH` | `fbs_byte_en width` | localparam |
| `LPARA_XFER_NUM` | - | localparam |
| `LPARA_XFER_IDX_W` | - | localparam |
| `LPARA_ENTRY_W` | - | localparam |
| `LPARA_FLUSH_DWELL` | - | localparam |
| `gen_para_check` | - | generate block |
| `u_stream_fifo_0` | `stream FIFO` | instance |
| `fseq_state_type_def` | `fseq_state` | typedef (enum) |
| `ENUM_ST_PASSTHROUGH` | `PASSTHROUGH` | enum member |
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
| `w_sink` | `sink` | combinational |
| `w_abort_start` | `abort_start` | combinational |
| `w_abort_wr` | `abort_wr` | combinational |
| `w_abort_mask_nxt` | `abort_mask_next` | combinational |
| `w_line_open_nxt` | `line_open_next` | combinational |
| `w_todo` | `todo` | combinational |
| `w_inj_tid` | `inj_tid` | combinational |
| `w_asm_abort_done` | `asm_abort_done` | combinational |
| `reg_abort_mask` | `abort_mask` | flip-flop |
| `reg_line_open` | `line_open` | flip-flop |
| `reg_line_tid` | `line_tid` | flip-flop |
| `w_fifo_wr` | `FIFO write` | combinational |
| `w_fifo_din` | `FIFO entry` | combinational |
| `w_abort_entry` | `abort beat` | combinational |
| `w_fifo_full` | `FIFO full` | combinational |
| `reg_flush_req` | `flush request` | flip-flop |
| `reg_flush_ack_sync` | `flush acknowledge` | flip-flop |
| `w_flush_pending` | `flush pending` | combinational |
| `reg_flush_req_sync` | `flush request` | flip-flop |
| `reg_flush_ack` | `flush acknowledge` | flip-flop |
| `reg_flush_dwell` | - | flip-flop |
| `w_flushing` | `flush` | combinational |
| `w_fifo_dout` | `FIFO head` | combinational |
| `w_fifo_empty` | `FIFO empty` | combinational |
| `w_fifo_pop` | `FIFO read` | combinational |
| `w_hd_data` | `FIFO head fields (data)` | combinational |
| `w_hd_keep` | `FIFO head fields (keep)` | combinational |
| `w_hd_last` | `FIFO head fields (last)` | combinational |
| `w_hd_user` | `FIFO head fields (user)` | combinational |
| `w_hd_tid` | `FIFO head fields (tid)` | combinational |
| `reg_conv_idx` | `transfer index k` | flip-flop |
| `w_conv_k` | `k` | combinational |
| `found` | - | combinational |
| `w_conv_valid` | - | combinational |
| `w_conv_hand` | - | combinational |
| `w_conv_last` | - | combinational |
| `w_conv_data` | `transfer k (TDATA)` | combinational |
| `w_conv_keep` | `transfer k (TKEEP)` | combinational |
| `w_conv_tlast` | `transfer k (TLAST)` | combinational |
| `w_conv_user` | `transfer k (TUSER)` | combinational |
| `w_conv_tid` | `transfer k (TID)` | combinational |
| `w_out_free` | - | combinational |
| `reg_out_valid` | `out_valid` | flip-flop |
| `reg_out_data` | `out_data` | flip-flop |
| `reg_out_keep` | `out_keep` | flip-flop |
| `reg_out_last` | `out_last` | flip-flop |
| `reg_out_user` | `out_user` | flip-flop |
| `reg_out_tid` | `out_tid` | flip-flop |
| `reg_skid_valid` | `skid_valid` | flip-flop |
| `reg_skid_data` | `skid_data` | flip-flop |
| `reg_skid_keep` | `skid_keep` | flip-flop |
| `reg_skid_last` | `skid_last` | flip-flop |
| `reg_skid_user` | `skid_user` | flip-flop |
| `reg_skid_tid` | `skid_tid` | flip-flop |
