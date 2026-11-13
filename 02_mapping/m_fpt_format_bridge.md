# Signal mapping — B08 Format Bridge

## Notes

- RTL module: `m_fpt_format_bridge` (`00_src/rtl/m_fpt_format_bridge.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- The Bypass Re-packer and the RAW16 Accumulator & Drain each have their own copy of the spec names `sof_pending` / `sol_pending` / `eol_pending`, `shifted`, `drain`, `empties`, `be_flush` and `total_remain`; the RTL prefixes are `rep_` and `r16_`, and the spec name gives the owner in parentheses.
- The spec suffix `_q` is the registered value and `_n` is the next value (not active-low); in an RTL name these are `reg_…` and `w_…_nxt`.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_format_bridge` | `Format Bridge (Block No. 8)` | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_fb_flush` | `fb_flush` | input port |
| `i_fb_px_data` | `fb_px_data` | input port |
| `i_fb_px_valid` | `fb_px_valid` | input port |
| `i_fb_px_valid_count` | `fb_px_valid_count` | input port |
| `i_fb_px_sof` | `fb_px_sof` | input port |
| `i_fb_px_sol` | `fb_px_sol` | input port |
| `i_fb_px_eol` | `fb_px_eol` | input port |
| `i_fb_px_vc_tag` | `fb_px_vc_tag` | input port |
| `i_fb_px_dt_sel` | `fb_px_dt_sel` | input port |
| `i_fb_err_wc_align` | `fb_err_wc_align` | input port |
| `o_fb_ready` | `fb_ready` | output port |
| `i_stream_format` | `stream_format` | input port |
| `i_dma_format` | `dma_format` | input port |
| `i_axis_enable` | `axis_enable` | input port |
| `i_dma_enable` | `dma_enable` | input port |
| `o_fbs_data` | `fbs_data` | output port |
| `o_fbs_valid` | `fbs_valid` | output port |
| `o_fbs_byte_en` | `fbs_byte_en` | output port |
| `o_fbs_sof` | `fbs_sof` | output port |
| `o_fbs_eol` | `fbs_eol` | output port |
| `o_fbs_vc_tag` | `fbs_vc_tag` | output port |
| `o_fbs_err` | `fbs_err` | output port |
| `i_fbs_ready` | `fbs_ready` | input port |
| `o_fbd_data` | `fbd_data` | output port |
| `o_fbd_valid` | `fbd_valid` | output port |
| `o_fbd_byte_en` | `fbd_byte_en` | output port |
| `o_fbd_sof` | `fbd_sof` | output port |
| `o_fbd_sol` | `fbd_sol` | output port |
| `o_fbd_eol` | `fbd_eol` | output port |
| `o_fbd_vc_tag` | `fbd_vc_tag` | output port |
| `o_fbd_err` | `fbd_err` | output port |
| `i_fbd_ready` | `fbd_ready` | input port |
| `o_fb_overflow` | `fb_overflow` | output port |
| `o_stream_fifo_almost_full` | `stream_fifo_almost_full` | output port |
| `o_dma_fifo_almost_full` | `dma_fifo_almost_full` | output port |
| `PARA_STREAM_FIFO_DEPTH` | `STREAM_FIFO_DEPTH` | parameter |
| `PARA_DMA_FIFO_DEPTH` | `DMA_FIFO_DEPTH` | parameter |
| `LPARA_DATA_OUT_WIDTH` | `DATA_OUT_WIDTH` | localparam |
| `LPARA_REPACK_BUF_WIDTH` | `REPACK_BUF_WIDTH` | localparam |
| `LPARA_RAW16_RESIDUAL_WIDTH` | `RAW16_RESIDUAL_WIDTH` | localparam |
| `LPARA_BYTE_EN_WIDTH` | - | localparam |
| `LPARA_PACKED_WIDTH` | - | localparam |
| `LPARA_BUF_NEXT_WIDTH` | `buf_next (104-bit)` | localparam |
| `LPARA_R16_BUF_NEXT_WIDTH` | `r16_buf_next (96-bit)` | localparam |
| `LPARA_REPACK_CNT_WIDTH` | `repack_cnt (4-bit)` | localparam |
| `LPARA_R16_CNT_WIDTH` | `r16_cnt (3-bit)` | localparam |
| `LPARA_R16_TOTAL_WIDTH` | `r16_total (4-bit)` | localparam |
| `LPARA_STREAM_ENTRY_WIDTH` | `Stream FIFO entry (77 bits)` | localparam |
| `LPARA_DMA_ENTRY_WIDTH` | `DMA FIFO entry (78 bits)` | localparam |
| `LPARA_SOF_BIT` | - | localparam |
| `LPARA_EOL_BIT` | - | localparam |
| `LPARA_VC_TAG_LSB` | - | localparam |
| `LPARA_ERR_BIT` | - | localparam |
| `LPARA_SOL_BIT` | - | localparam |
| `LPARA_STREAM_PTR_WIDTH` | - | localparam |
| `LPARA_STREAM_CNT_WIDTH` | `stream_fifo_count width` | localparam |
| `LPARA_DMA_PTR_WIDTH` | - | localparam |
| `LPARA_DMA_CNT_WIDTH` | `dma_fifo_count width` | localparam |
| `LPARA_FIFO_DEPTH_MIN` | - | localparam |
| `LPARA_FIFO_DEPTH_MAX` | - | localparam |
| `w_clear` | - | combinational |
| `w_take` | `take` | combinational |
| `w_flush_cycle` | `flush_cycle (flush_cycle_n)` | combinational |
| `w_fb_ready` | `fb_ready` | combinational |
| `reg_dt_hold` | `dt_hold` | flip-flop |
| `reg_vc_tag_hold` | `vc_tag_hold` | flip-flop |
| `reg_err_hold` | `err_hold` | flip-flop |
| `w_dt_nxt` | `dt_hold (value being loaded)` | combinational |
| `w_vc_tag_nxt` | `vc_tag_hold (value being loaded)` | combinational |
| `w_err_nxt` | `err_hold (value being loaded)` | combinational |
| `w_raw16_data` | `raw16_data` | combinational |
| `reg_repack_buf` | `repack_buf` | flip-flop |
| `reg_repack_cnt` | `repack_cnt` | flip-flop |
| `reg_rep_sof_pending` | `sof_pending (sof_pending_q, Bypass Re-packer)` | flip-flop |
| `reg_rep_sol_pending` | `sol_pending (sol_pending_q, Bypass Re-packer)` | flip-flop |
| `reg_rep_eol_pending` | `eol_pending (eol_pending_q, Bypass Re-packer)` | flip-flop |
| `w_rep_sof_pending_nxt` | `sof_pending_n (Bypass Re-packer)` | combinational |
| `w_rep_sol_pending_nxt` | `sol_pending_n (Bypass Re-packer)` | combinational |
| `w_rep_eol_pending_nxt` | `eol_pending_n (Bypass Re-packer)` | combinational |
| `w_intake_bytes` | `intake_bytes` | combinational |
| `w_new_bytes` | `new_bytes` | combinational |
| `w_packed_new_bytes` | `packed_new_bytes` | combinational |
| `w_intake_data` | - | combinational |
| `w_rep_shifted` | `shifted (104bit, Bypass Re-packer)` | combinational |
| `w_buf_next` | `buf_next` | combinational |
| `w_total` | `total` | combinational |
| `w_rep_total_remain` | `total_remain (Bypass Re-packer)` | combinational |
| `w_rep_drain` | `drain (Bypass Re-packer)` | combinational |
| `w_rep_empties` | `empties (Bypass Re-packer)` | combinational |
| `w_rep_be_flush` | `be_flush (Bypass Re-packer)` | combinational |
| `w_rep_flush_req` | `rep_flush_req` | combinational |
| `w_rep_data` | `rep_data` | combinational |
| `w_rep_valid` | `rep_valid` | combinational |
| `w_rep_byte_en` | `rep_byte_en` | combinational |
| `w_rep_sof` | `rep_sof` | combinational |
| `w_rep_sol` | `rep_sol` | combinational |
| `w_rep_eol` | `rep_eol` | combinational |
| `w_rep_vc_tag` | `rep_vc_tag` | combinational |
| `w_rep_err` | `rep_err` | combinational |
| `reg_r16_buf` | `r16_buf` | flip-flop |
| `reg_r16_cnt` | `r16_cnt` | flip-flop |
| `reg_r16_sof_pending` | `sof_pending (sof_pending_q, RAW16 Accumulator & Drain)` | flip-flop |
| `reg_r16_sol_pending` | `sol_pending (sol_pending_q, RAW16 Accumulator & Drain)` | flip-flop |
| `reg_r16_eol_pending` | `eol_pending (eol_pending_q, RAW16 Accumulator & Drain)` | flip-flop |
| `w_r16_sof_pending_nxt` | `sof_pending_n (RAW16 Accumulator & Drain)` | combinational |
| `w_r16_sol_pending_nxt` | `sol_pending_n (RAW16 Accumulator & Drain)` | combinational |
| `w_r16_eol_pending_nxt` | `eol_pending_n (RAW16 Accumulator & Drain)` | combinational |
| `w_r16_intake_bytes` | `r16_intake_bytes` | combinational |
| `w_r16_new_bytes` | `r16_new_bytes` | combinational |
| `w_r16_intake_data` | - | combinational |
| `w_r16_shifted` | `shifted (96bit, RAW16 Accumulator & Drain)` | combinational |
| `w_r16_buf_next` | `r16_buf_next` | combinational |
| `w_r16_total` | `r16_total` | combinational |
| `w_r16_total_remain` | `r16_total_remain` | combinational |
| `w_r16_drain` | `drain (RAW16 Accumulator & Drain)` | combinational |
| `w_r16_empties` | `empties (RAW16 Accumulator & Drain)` | combinational |
| `w_r16_be_flush` | `be_flush (RAW16 Accumulator & Drain)` | combinational |
| `w_r16_flush_req` | `r16_flush_req` | combinational |
| `w_raw16_data_out` | `raw16_data_out` | combinational |
| `w_raw16_valid` | `raw16_valid` | combinational |
| `w_raw16_byte_en` | `raw16_byte_en` | combinational |
| `w_raw16_sof` | `raw16_sof` | combinational |
| `w_raw16_sol` | `raw16_sol` | combinational |
| `w_raw16_eol` | `raw16_eol` | combinational |
| `w_raw16_vc_tag` | `raw16_vc_tag` | combinational |
| `w_raw16_err` | `raw16_err` | combinational |
| `w_s_bundle` | `s_bundle` | combinational |
| `w_d_bundle` | `d_bundle` | combinational |
| `w_stream_wr` | `stream_wr` | combinational |
| `w_dma_wr` | `dma_wr` | combinational |
| `reg_stream_fifo_memory` | - | flip-flop |
| `reg_stream_fifo_wr_ptr` | - | flip-flop |
| `reg_stream_fifo_rd_ptr` | - | flip-flop |
| `reg_stream_fifo_count` | `stream_fifo_count` | flip-flop |
| `w_stream_fifo_full` | `stream_fifo_full` | combinational |
| `w_stream_fifo_empty` | `fifo_empty (stream)` | combinational |
| `w_stream_fifo_almost_full` | `stream_fifo_almost_full` | combinational |
| `w_stream_fifo_rd` | - | combinational |
| `w_stream_fifo_head` | - | combinational |
| `w_fbs_valid` | `fbs_valid` | combinational |
| `reg_dma_fifo_memory` | - | flip-flop |
| `reg_dma_fifo_wr_ptr` | - | flip-flop |
| `reg_dma_fifo_rd_ptr` | - | flip-flop |
| `reg_dma_fifo_count` | `dma_fifo_count` | flip-flop |
| `w_dma_fifo_full` | `dma_fifo_full` | combinational |
| `w_dma_fifo_empty` | `fifo_empty (DMA)` | combinational |
| `w_dma_fifo_almost_full` | `dma_fifo_almost_full` | combinational |
| `w_dma_fifo_rd` | - | combinational |
| `w_dma_fifo_head` | - | combinational |
| `w_fbd_valid` | `fbd_valid` | combinational |
| `w_fb_overflow` | `fb_overflow` | combinational |
| `intake_table` | `intake_table()` | function |
| `packed_new_bytes` | `packed_new_bytes()` | function |
| `gen_para_check` | - | generate block |
