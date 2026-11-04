# Signal mapping — B02 Lane Management

## Notes

- RTL module: `m_fpt_lane_management` (`00_src/rtl/m_fpt_lane_management.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4, Section 6.4: Table 6-7 (parameters), Table 6-8 (port list), Table 6-9 (Burst FSM states and outputs), Figures 6-6 to 6-8 (lane alignment and lane merging); draw.io diagram page "#2 Lane Management" (Lane Event Capture, Condition Reduce).
- The `_x` suffix in a spec name is the lane index; `(x=0)` gives the lane of a split port. In an RTL name, `[x]` is the per-lane array element (x = 0 … 3) and `gen_lane[x].gen_slice.` is the per-lane generate block.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_lane_management` | Lane Management (Block No. 2) | module |
| `PARA_MAX_LANES` | `MAX_LANES` | parameter |
| `PARA_SKEW_DEPTH` | `SKEW_DEPTH` | parameter |
| `PARA_FIFO_AF_MARGIN` | `FIFO_AF_MARGIN` | parameter |
| `LPARA_MERGED_WIDTH` | `MERGED_WIDTH` | localparam |
| `LPARA_MERGED_BYTES` | `MERGED_WIDTH / 8` | localparam |
| `LPARA_PORT_LANES` | - | localparam |
| `LPARA_PTR_W` | - | localparam |
| `LPARA_CNT_W` | - | localparam |
| `LPARA_DEPTH_CNT` | - | localparam |
| `LPARA_AF_THR` | - | localparam |
| `LPARA_PTR_LAST` | - | localparam |
| `LPARA_LANE_MASK` | - | localparam |
| `i_rxbyteclkhs` | `rxbyteclkhs` | input port |
| `i_resetn` | `resetn` | input port |
| `i_soft_reset` | `soft_reset` | input port |
| `i_active_lanes` | `active_lanes[1:0]` | input port |
| `i_rxdatahs_0` | `rxdatahs_x (x=0)` | input port |
| `i_rxdatahs_1` | `rxdatahs_x (x=1)` | input port |
| `i_rxdatahs_2` | `rxdatahs_x (x=2)` | input port |
| `i_rxdatahs_3` | `rxdatahs_x (x=3)` | input port |
| `i_rxvalidhs_0` | `rxvalidhs_x (x=0)` | input port |
| `i_rxvalidhs_1` | `rxvalidhs_x (x=1)` | input port |
| `i_rxvalidhs_2` | `rxvalidhs_x (x=2)` | input port |
| `i_rxvalidhs_3` | `rxvalidhs_x (x=3)` | input port |
| `i_rxactivehs_0` | `rxactivehs_x (x=0)` | input port |
| `i_rxactivehs_1` | `rxactivehs_x (x=1)` | input port |
| `i_rxactivehs_2` | `rxactivehs_x (x=2)` | input port |
| `i_rxactivehs_3` | `rxactivehs_x (x=3)` | input port |
| `i_rxsynchs_0` | `rxsynchs_x (x=0)` | input port |
| `i_rxsynchs_1` | `rxsynchs_x (x=1)` | input port |
| `i_rxsynchs_2` | `rxsynchs_x (x=2)` | input port |
| `i_rxsynchs_3` | `rxsynchs_x (x=3)` | input port |
| `i_rxskewcalhs_0` | `rxskewcalhs_x (x=0)` | input port |
| `i_rxskewcalhs_1` | `rxskewcalhs_x (x=1)` | input port |
| `i_rxskewcalhs_2` | `rxskewcalhs_x (x=2)` | input port |
| `i_rxskewcalhs_3` | `rxskewcalhs_x (x=3)` | input port |
| `i_errsoths_0` | `errsoths_x (x=0)` | input port |
| `i_errsoths_1` | `errsoths_x (x=1)` | input port |
| `i_errsoths_2` | `errsoths_x (x=2)` | input port |
| `i_errsoths_3` | `errsoths_x (x=3)` | input port |
| `i_errsotsynchs_0` | `errsotsynchs_x (x=0)` | input port |
| `i_errsotsynchs_1` | `errsotsynchs_x (x=1)` | input port |
| `i_errsotsynchs_2` | `errsotsynchs_x (x=2)` | input port |
| `i_errsotsynchs_3` | `errsotsynchs_x (x=3)` | input port |
| `o_merged_data` | `merged_data[31:0]` | output port |
| `o_merged_valid` | `merged_valid` | output port |
| `o_merged_byte_valid` | `merged_byte_valid[3:0]` | output port |
| `o_merged_sob` | `merged_sob` | output port |
| `o_merged_eob` | `merged_eob` | output port |
| `o_lm_fsm_state` | `lm_fsm_state[2:0]` | output port |
| `o_abort_sync_err` | `abort_sync_err` | output port |
| `o_abort_fifo_full` | `abort_fifo_full` | output port |
| `o_abort_unarmed` | `abort_unarmed` | output port |
| `o_abort_lane_sync_seen` | `abort_lane_sync_seen[3:0]` | output port |
| `o_lane_abort_pulse` | `lane_abort_pulse` | output port |
| `o_deskew_burst_seen` | `deskew_burst_seen` | output port |
| `o_active_lanes_applied` | `active_lanes_applied[1:0]` | output port |
| `reg_active_lanes_reg` | `active_lanes_reg` | flip-flop |
| `reg_fsm_lm_current_state` | - | flip-flop |
| `reg_active_seen[x]` | `active_seen_x` | flip-flop |
| `reg_sync_seen[x]` | `sync_seen_x` | flip-flop |
| `reg_deskew_seen[x]` | `deskew_seen_x` | flip-flop |
| `reg_err_sync[x]` | `err_sync_x` | flip-flop |
| `gen_lane[x].gen_slice.reg_fifo_mem` | - | flip-flop |
| `gen_lane[x].gen_slice.reg_fifo_wr_ptr` | - | flip-flop |
| `gen_lane[x].gen_slice.reg_fifo_rd_ptr` | - | flip-flop |
| `gen_lane[x].gen_slice.reg_fifo_cnt` | - | flip-flop |
| `reg_tail` | - | flip-flop |
| `reg_acc` | `acc` | flip-flop |
| `reg_acc_valid` | `acc_valid[3:0]` | flip-flop |
| `reg_wr_pos` | `wr_pos[1:0]` | flip-flop |
| `reg_hold_data` | - | flip-flop |
| `reg_hold_vld` | - | flip-flop |
| `reg_sob_pending` | `sob_pending` | flip-flop |
| `reg_merged_data` | `merged_data` | flip-flop |
| `reg_merged_valid` | `merged_valid` | flip-flop |
| `reg_merged_byte_valid` | `merged_byte_valid` | flip-flop |
| `reg_merged_sob` | `merged_sob` | flip-flop |
| `reg_merged_eob` | `merged_eob` | flip-flop |
| `reg_abort_sync_err` | `abort_sync_err` | flip-flop |
| `reg_abort_fifo_full` | `abort_fifo_full` | flip-flop |
| `reg_abort_unarmed` | `abort_unarmed` | flip-flop |
| `reg_abort_lane_sync_seen` | `abort_lane_sync_seen` | flip-flop |
| `reg_lane_abort_pulse` | `lane_abort_pulse` | flip-flop |
| `reg_deskew_burst_seen` | `deskew_burst_seen` | flip-flop |
| `w_fsm_lm_nxt_state` | - | combinational |
| `w_rxdatahs[x] .. w_rxskewcalhs[x]` | `rxdatahs_x .. rxskewcalhs_x` | combinational |
| `w_errsotsynchs_x[x]` | `errsotsynchs_x` | combinational |
| `w_sync_ok[x]` | `sync_ok_x` | combinational |
| `w_eot_seen[x]` | `eot_seen_x` | combinational |
| `w_wr_en[x]` | `wr_en_x` | combinational |
| `w_pop[x]` | `pop_x` | combinational |
| `w_fifo_not_empty[x]` | `fifo_not_empty_x` | combinational |
| `w_fifo_full_x[x]` | `fifo_full_x` | combinational |
| `w_aligned_byte[x]` | `aligned_byte_x` | combinational |
| `gen_lane[x].gen_slice.w_fifo_wr_acc` | - | combinational |
| `w_active` | `active_0..active_3` | combinational |
| `w_burst_open` | `burst_open` | combinational |
| `w_burst_closed` | `burst_closed` | combinational |
| `w_armed` | `armed` | combinational |
| `w_any_sync` | `any_sync` | combinational |
| `w_any_deskew` | `any_deskew` | combinational |
| `w_deskew_burst` | `deskew_burst` | combinational |
| `w_errsotsynchs` | `errsotsynchs` | combinational |
| `w_fifo_full` | `fifo_full` | combinational |
| `w_any_lane_has_data` | `any_lane_has_data` | combinational |
| `w_all_lanes_ready` | `all_lanes_ready` | combinational |
| `w_drain_complete` | `drain_complete` | combinational |
| `w_sync_seen_vec` | - | combinational |
| `w_burst_clr` | `burst_clr` | combinational |
| `w_cfg_apply` | `cfg_apply` | combinational |
| `w_wr_en_ok` | `wr_en_ok` | combinational |
| `w_rd_en_ok` | `rd_en_ok` | combinational |
| `w_burst_active` | `burst_active` | combinational |
| `w_pack_flush` | `pack_flush` | combinational |
| `w_abort_active` | `abort_active` | combinational |
| `w_read_en` | `read_en` | combinational |
| `w_abort_entry` | - | combinational |
| `w_deskew_exit` | - | combinational |
| `w_final_place` | - | combinational |
| `w_place` | - | combinational |
| `w_place_data` | `place_data` | combinational |
| `w_place_valid` | `place_valid` | combinational |
| `w_merge_data` | - | combinational |
| `w_merge_valid` | - | combinational |
| `w_place_cnt` | - | combinational |
| `w_pos_sum` | `sum[2:0]` | combinational |
| `w_word_complete` | `word_complete` | combinational |
| `w_any_placed` | - | combinational |
| `w_packer_empty` | `packer_empty` | combinational |
| `w_wr_hold` | - | combinational |
| `w_wr_hold_last` | - | combinational |
| `w_wr_part` | - | combinational |
| `w_emit` | `emit` | combinational |
| `w_emit_data` | - | combinational |
| `w_emit_mask` | - | combinational |
| `w_emit_eob` | - | combinational |
