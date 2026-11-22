# Signal mapping — Asynchronous FIFO wrapper (Block No. 2 to Block No. 3)

## Notes

- RTL module: `m_fpt_async_fifo` (`00_src/rtl/m_fpt_async_fifo.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- The write side runs on `rxbyteclkhs` (word stream from Lane Management) and the read side on `sys_clk` (towards Packet Analyzer). `i_resetn_rxbyteclkhs` and `i_resetn_sys_clk` are `resetn` after the reset synchroniser of each domain at the top level.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_async_fifo` | `Asynchronous FIFO wrapper between Block No. 2 and Block No. 3` | module |
| `i_rxbyteclkhs` | `rxbyteclkhs` | input port |
| `i_resetn_rxbyteclkhs` | `resetn` | input port |
| `i_merged_valid` | `merged_valid` | input port |
| `i_merged_data` | `merged_data[31:0]` | input port |
| `i_merged_byte_valid` | `merged_byte_valid[3:0]` | input port |
| `i_merged_sob` | `merged_sob` | input port |
| `i_merged_eob` | `merged_eob` | input port |
| `i_sys_clk` | `sys_clk` | input port |
| `i_resetn_sys_clk` | `resetn` | input port |
| `i_core_enable` | `core_enable` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_rd_en` | `rd_en` | input port |
| `o_rd_data` | `rd_data[37:0]` | output port |
| `o_rd_empty` | `rd_empty` | output port |
| `PARA_DEPTH` | - | parameter |
| `PARA_SYNC_STAGES` | - | parameter |
| `LPARA_ENTRY_W` | - | localparam |
| `LPARA_ADDR_W` | - | localparam |
| `LPARA_PTR_W` | - | localparam |
| `LPARA_FULL_MASK` | - | localparam |
| `reg_mem` | - | flip-flop |
| `reg_wptr_bin` | - | flip-flop |
| `reg_wptr_gray` | - | flip-flop |
| `reg_wr_full` | - | flip-flop |
| `w_wr_en` | - | combinational |
| `w_wptr_bin_nxt` | - | combinational |
| `w_wptr_gray_nxt` | - | combinational |
| `w_wr_full_nxt` | - | combinational |
| `reg_rptr_gray_sync` | - | flip-flop |
| `reg_wptr_gray_sync` | - | flip-flop |
| `w_rptr_gray_synced` | - | combinational |
| `w_wptr_gray_synced` | - | combinational |
| `reg_rptr_bin` | - | flip-flop |
| `reg_rptr_xing_bin` | - | flip-flop |
| `reg_rptr_gray_xing` | - | flip-flop |
| `reg_rd_empty` | `rd_empty` | flip-flop |
| `w_flush` | - | combinational |
| `w_rd_pop` | - | combinational |
| `w_rptr_bin_nxt` | - | combinational |
| `w_rptr_gray_nxt` | - | combinational |
| `w_rptr_xing_bin_nxt` | - | combinational |
| `w_rd_empty_nxt` | - | combinational |
| `w_rd_data` | `rd_data[37:0]` | combinational |
