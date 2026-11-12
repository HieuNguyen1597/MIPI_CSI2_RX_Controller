# Signal mapping — B07 Line Buffer

## Notes

- RTL module: `m_fpt_line_buffer` (`00_src/rtl/m_fpt_line_buffer.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- `reg_wr_ptr` and `reg_rd_ptr` are used directly as the memory write and read addresses, so each maps to two spec names (`wr_ptr` / `w_addr`, `rd_ptr` / `r_addr`).
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_line_buffer` | `Line Buffer (Block No. 7)` | module |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_lb_flush` | `lb_flush` | input port |
| `i_px_data` | `px_data` | input port |
| `i_px_valid` | `px_valid` | input port |
| `i_px_valid_count` | `px_valid_count` | input port |
| `i_px_sof` | `px_sof` | input port |
| `i_px_sol` | `px_sol` | input port |
| `i_px_eol` | `px_eol` | input port |
| `i_px_vc_tag` | `px_vc_tag` | input port |
| `i_px_dt_sel` | `px_dt_sel` | input port |
| `i_err_wc_align` | `err_wc_align` | input port |
| `i_fb_ready` | `fb_ready` | input port |
| `o_fb_px_data` | `fb_px_data` | output port |
| `o_fb_px_valid` | `fb_px_valid` | output port |
| `o_fb_px_valid_count` | `fb_px_valid_count` | output port |
| `o_fb_px_sof` | `fb_px_sof` | output port |
| `o_fb_px_sol` | `fb_px_sol` | output port |
| `o_fb_px_eol` | `fb_px_eol` | output port |
| `o_fb_px_vc_tag` | `fb_px_vc_tag` | output port |
| `o_fb_px_dt_sel` | `fb_px_dt_sel` | output port |
| `o_fb_err_wc_align` | `fb_err_wc_align` | output port |
| `o_linebuffer_full` | `linebuffer_full` | output port |
| `o_linebuf_fill` | `linebuf_fill` | output port |
| `o_linebuf_overflow` | `linebuf_overflow` | output port |
| `o_linebuf_empty` | `linebuf_empty` | output port |
| `PARA_LINE_BUFFER_DEPTH` | `LINE_BUFFER_DEPTH (LBD)` | parameter |
| `PARA_FULL_WATERMARK` | `FULL_WATERMARK` | parameter |
| `LPARA_DATA_WIDTH` | `DATA_WIDTH` | localparam |
| `LPARA_VC_TAG_WIDTH` | `VC_TAG_WIDTH` | localparam |
| `LPARA_VALID_COUNT_WIDTH` | `VALID_COUNT_WIDTH` | localparam |
| `LPARA_ENTRY_WIDTH` | `ENTRY_WIDTH` | localparam |
| `LPARA_READ_SKID_DEPTH` | `READ_SKID_DEPTH` | localparam |
| `LPARA_PTR_WIDTH` | `PTR_WIDTH` | localparam |
| `LPARA_FILL_CNT_WIDTH` | `FILL_CNT_WIDTH` | localparam |
| `LPARA_SKID_CNT_WIDTH` | - | localparam |
| `reg_wr_ptr` | `wr_ptr / w_addr` | flip-flop |
| `reg_fill_cnt` | `fill_cnt` | flip-flop |
| `reg_memory` | `Memory (dual-port SRAM)` | flip-flop |
| `reg_r_data` | `r_data` | flip-flop |
| `reg_rd_ptr` | `rd_ptr / r_addr` | flip-flop |
| `reg_inflight` | `inflight` | flip-flop |
| `reg_fb_px_valid` | `fb_px_valid` | flip-flop |
| `reg_skid_valid` | `skid_valid` | flip-flop |
| `reg_out_reg` | `out_reg (output register)` | flip-flop |
| `reg_skid_reg` | `skid_reg (skid register)` | flip-flop |
| `w_sync_clear` | - | combinational |
| `w_full_hard` | `full_hard` | combinational |
| `w_full_normal` | `full_normal` | combinational |
| `w_write_en` | `write_en` | combinational |
| `w_write_accept` | `write_accept` | combinational |
| `w_linebuf_overflow` | `linebuf_overflow` | combinational |
| `w_consume` | `consume` | combinational |
| `w_next_fill_cnt` | `next_fill_cnt` | combinational |
| `w_linebuffer_full` | `linebuffer_full` | combinational |
| `w_linebuf_empty` | `linebuf_empty` | combinational |
| `w_w_data` | `w_data` | combinational |
| `w_land` | `land` | combinational |
| `w_occupancy` | `occupancy` | combinational |
| `w_space` | `space` | combinational |
| `w_unread` | `unread` | combinational |
| `w_rd_en` | `rd_en` | combinational |
| `w_promote` | `promote` | combinational |
| `w_out_free` | `out_free` | combinational |
| `w_r_to_out` | `r_to_out` | combinational |
| `w_r_to_skid` | `r_to_skid` | combinational |
| `gen_para_check` | - | generate block |
