// Module      : m_fpt_frame_context_metadata
// Block       : B09 - Frame Context & Metadata Capture
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-06
// Description : Pairs channels into captures, tracks identity and geometry, writes metadata.
`timescale 1ns/1ps
`default_nettype none

module m_fpt_frame_context_metadata #(
  // Parameter Description: extraction slots in the embedded-data path; 0 removes the extractor.
  // Parameter values: 0, 1 or 2; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_N_EXTRACT       = 32'd2,
  // Parameter Description: nanoseconds added to the timestamp counter every sys_clk cycle,
  // unsigned with 8 integer and 24 fractional bits; follows the sys_clk frequency.
  // Parameter values: any non-zero value; default is 32'h0400_0000 (4 ns at 250 MHz).
  // Parameter unit: none
  parameter logic [31:0] PARA_TS_INCR         = 32'h0400_0000,
  // Parameter Description: staging FIFO depth towards the DMA, one word and its side-band bits.
  // Parameter values: power of two, at least 2; default is 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_META_FIFO_DEPTH = 32'd16
) (
  input  wire         i_sys_clk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  input  wire         i_core_enable,
  input  wire         i_metadata_capture_en,
  input  wire         i_fnum_check_en,
  input  wire         i_emb_extract_en,
  input  wire  [15:0] i_expected_width,
  input  wire  [15:0] i_expected_height,
  input  wire  [31:0] i_metadata_record_id,
  input  wire  [((PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT) - 32'd1:0][31:0] i_emb_extract_cfg,
  input  wire         i_vc_exposure_map,
  input  wire         i_tpg_dual_vc_en,
  input  wire         i_active_sel,
  input  wire         i_fc_fs,
  input  wire         i_fc_fe,
  input  wire  [15:0] i_fc_frame_num,
  input  wire  [1:0]  i_fc_vc,
  input  wire         i_tpg_fs,
  input  wire         i_tpg_fe,
  input  wire  [1:0]  i_tpg_fvc,
  input  wire         i_bp_valid,
  input  wire  [3:0]  i_bp_byte_en,
  input  wire         i_bp_sol,
  input  wire         i_bp_eol,
  input  wire  [1:0]  i_bp_vc_tag,
  input  wire         i_bp_dt_sel,
  input  wire  [31:0] i_emb_data,
  input  wire  [3:0]  i_emb_byte_en,
  input  wire         i_emb_valid,
  input  wire         i_emb_sol,
  input  wire         i_emb_eol,
  input  wire  [1:0]  i_emb_vc,
  input  wire         i_capture_drop,
  input  wire  [1:0]  i_drop_vc_mask,
  input  wire         i_dma_buf_claim,
  input  wire         i_dma_buf_index,
  input  wire         i_meta_ready,
  output logic [1:0]  o_capture_start,
  output logic [1:0]  o_capture_end,
  output logic [1:0]  o_capture_dropped,
  output logic        o_ev_fs,
  output logic [1:0]  o_ev_vc,
  output logic [1:0]  o_ctx_slot,
  output logic [1:0]  o_ctx_orphan,
  output logic        o_lead_vc,
  output logic [31:0] o_frame_count,
  output logic [13:0] o_frame_id_buf0,
  output logic [13:0] o_frame_id_buf1,
  output logic [63:0] o_timestamp,
  output logic [15:0] o_vc0_line_count,
  output logic [15:0] o_vc0_byte_count,
  output logic [5:0]  o_vc0_data_type,
  output logic        o_vc0_width_mismatch,
  output logic        o_vc0_height_mismatch,
  output logic [15:0] o_emb_word_count_vc0,
  output logic [7:0]  o_emb_line_count_vc0,
  output logic [((PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT) - 32'd1:0][31:0] o_emb_extract_val_vc0,
  output logic [13:0] o_cur_frame_id_vc0,
  output logic [15:0] o_cur_line_vc0,
  output logic [15:0] o_vc1_line_count,
  output logic [15:0] o_vc1_byte_count,
  output logic [5:0]  o_vc1_data_type,
  output logic        o_vc1_width_mismatch,
  output logic        o_vc1_height_mismatch,
  output logic [15:0] o_emb_word_count_vc1,
  output logic [7:0]  o_emb_line_count_vc1,
  output logic [((PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT) - 32'd1:0][31:0] o_emb_extract_val_vc1,
  output logic [13:0] o_cur_frame_id_vc1,
  output logic [15:0] o_cur_line_vc1,
  output logic        o_meta_overflow,
  output logic        o_err_frame_num,
  output logic        o_err_resolution,
  output logic        o_capture_done,
  output logic [31:0] o_meta_data,
  output logic [3:0]  o_meta_byte_en,
  output logic        o_meta_valid,
  output logic        o_meta_last,
  output logic        o_meta_kind,
  output logic [1:0]  o_meta_vc,
  output logic        o_meta_sof
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_FRAME_ID_WIDTH = 32'd14;
  localparam logic [31:0] LPARA_CAPTURE_SLOTS  = 32'd2;
  localparam logic [31:0] LPARA_RECORD_WORDS   = 32'd7 + PARA_N_EXTRACT;
  localparam logic [31:0] LPARA_CHANNEL_NUM    = 32'd2;
  localparam logic [31:0] LPARA_EXTRACT_PORTS  = (PARA_N_EXTRACT == 32'd0) ? 32'd1 : PARA_N_EXTRACT;
  localparam logic [31:0] LPARA_TS_INT_W       = 32'd64;
  localparam logic [31:0] LPARA_TS_FRAC_W      = 32'd24;
  localparam logic [31:0] LPARA_META_WIDTH     = 32'd41;
  localparam logic [31:0] LPARA_PTR_W          = $clog2(PARA_META_FIFO_DEPTH);
  localparam logic [31:0] LPARA_CNT_W          = $clog2(PARA_META_FIFO_DEPTH + 32'd1);
  localparam logic [31:0] LPARA_WORD_IDX_W     = $clog2(LPARA_RECORD_WORDS);
  localparam logic [LPARA_CNT_W-1:0]          LPARA_DEPTH_CNT = LPARA_CNT_W'(PARA_META_FIFO_DEPTH);
  localparam logic [LPARA_WORD_IDX_W-1:0]     LPARA_WORD_LAST =
    LPARA_WORD_IDX_W'(LPARA_RECORD_WORDS - 32'd1);
  localparam logic [LPARA_FRAME_ID_WIDTH-1:0] LPARA_FRAME_ID_MAX = '1;

  // Record version and the image Data Type codes reported in the record.
  localparam logic [7:0]  LPARA_RECORD_VERSION = 8'h01;
  localparam logic [5:0]  LPARA_DT_RAW10       = 6'h2B;
  localparam logic [5:0]  LPARA_DT_RAW12       = 6'h2C;

  // Record FSM states.
  typedef enum logic [1:0] {
    ENUM_ST_IDLE   = 2'b00,
    ENUM_ST_RECORD = 2'b01,
    ENUM_ST_POP    = 2'b10
  } rec_state_type_def;

  // Event Source Select
  logic                                        w_ev_fs;
  logic                                        w_ev_fe;
  logic [1:0]                                  w_ev_vc;
  logic [15:0]                                 w_ev_fnum;
  logic                                        w_ev_rx;
  logic                                        w_lead_vc;
  logic                                        w_dual_vc;

  // Capture Tracker
  logic                                        w_ev_take;
  logic                                        w_fs_take;
  logic                                        w_fe_take;
  logic                                        w_ev_ch;
  logic                                        w_is_lead;
  logic [LPARA_CHANNEL_NUM-1:0]                w_ctx_open;
  logic [LPARA_CHANNEL_NUM-1:0]                w_ctx_close;
  logic [LPARA_CHANNEL_NUM-1:0]                w_ctx_sync_close;
  logic [LPARA_CHANNEL_NUM-1:0]                w_ctx_abort;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_ctx_active;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_ctx_orphan;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_ctx_slot;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_last_fnum;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_fnum_err_flag;
  logic [LPARA_CAPTURE_SLOTS-1:0]              reg_slot_open;
  logic [LPARA_CAPTURE_SLOTS-1:0]              reg_slot_wait_lag;
  logic [LPARA_CAPTURE_SLOTS-1:0]              reg_slot_dropped;
  logic                                        reg_last_slot;
  logic                                        reg_core_enable_d;
  logic                                        reg_active_sel_d;
  logic                                        w_core_fall;
  logic                                        w_active_sel_chg;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_slot_free;
  logic                                        w_alloc_en;
  logic                                        w_alloc_slot;
  logic                                        w_lag_join;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_capture_start;
  logic                                        w_recent_slot;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_drop_hit;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_slot_busy;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_slot_eligible;
  logic                                        w_end_en;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_capture_end;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_capture_dropped;
  logic                                        w_capture_done;
  logic                                        w_fnum_mismatch;
  logic                                        w_err_frame_num;

  // Identity Generator
  logic [LPARA_TS_INT_W+LPARA_TS_FRAC_W-1:0]   reg_ts_cnt;
  logic [LPARA_FRAME_ID_WIDTH-1:0]             reg_frame_id_cnt;
  logic [LPARA_FRAME_ID_WIDTH-1:0]             w_frame_id_nxt;
  logic [LPARA_CAPTURE_SLOTS-1:0][LPARA_FRAME_ID_WIDTH-1:0] reg_slot_frame_id;
  logic [LPARA_CAPTURE_SLOTS-1:0][LPARA_TS_INT_W-1:0]       reg_slot_timestamp;
  logic [LPARA_CAPTURE_SLOTS-1:0][31:0]        reg_slot_record_id;
  logic [1:0][LPARA_FRAME_ID_WIDTH-1:0]        reg_frame_id_buf;
  logic [LPARA_CHANNEL_NUM-1:0][LPARA_FRAME_ID_WIDTH-1:0]   w_cur_frame_id;

  // Geometry Accumulator
  logic                                        w_bp_take;
  logic                                        w_bp_ch;
  logic [15:0]                                 w_line_bytes;
  logic [LPARA_CHANNEL_NUM-1:0]                w_line_done;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_byte_acc;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_acc_byte_count;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_acc_line_count;
  logic [LPARA_CHANNEL_NUM-1:0][5:0]           reg_acc_dt;

  // Resolution Checker
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_expected_width;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_expected_height;
  logic [16:0]                                 w_bytes_ref;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_width_mismatch;
  logic [LPARA_CHANNEL_NUM-1:0]                w_height_mismatch;
  logic                                        w_err_resolution;

  // Embedded Data Path
  logic                                        w_emb_fwd_valid;
  logic                                        w_emb_ch;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_emb_word_cnt;
  logic [LPARA_CHANNEL_NUM-1:0][7:0]           reg_emb_line_cnt;
  logic [LPARA_CHANNEL_NUM-1:0][LPARA_EXTRACT_PORTS-1:0][31:0] w_extract_pub;
  logic [31:0]                                 w_rec_extract;

  // Context Latch
  logic [LPARA_CHANNEL_NUM-1:0]                w_close_ok;
  logic [LPARA_CHANNEL_NUM-1:0]                w_bank_load;
  logic [LPARA_CHANNEL_NUM-1:0]                w_pub_load;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_bank_line_count;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_bank_byte_count;
  logic [LPARA_CHANNEL_NUM-1:0][5:0]           reg_bank_dt;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_bank_width_mismatch;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_bank_height_mismatch;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_bank_emb_word_cnt;
  logic [LPARA_CHANNEL_NUM-1:0][7:0]           reg_bank_emb_line_cnt;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_bank_fnum_err;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_bank_ovf;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_pub_line_count;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_pub_byte_count;
  logic [LPARA_CHANNEL_NUM-1:0][5:0]           reg_pub_data_type;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_pub_width_mismatch;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_pub_height_mismatch;
  logic [LPARA_CHANNEL_NUM-1:0][15:0]          reg_pub_emb_word_count;
  logic [LPARA_CHANNEL_NUM-1:0][7:0]           reg_pub_emb_line_count;
  logic [LPARA_TS_INT_W-1:0]                   reg_timestamp;
  logic [31:0]                                 reg_frame_count;

  // Record Builder
  logic [1:0]                                  reg_queue_vc;
  logic [1:0]                                  reg_queue_slot;
  logic [1:0]                                  reg_queue_abort;
  logic [1:0]                                  reg_queue_cnt;
  logic                                        w_queue_empty;
  logic                                        w_more_pending;
  logic                                        w_cur_vc;
  logic                                        w_cur_slot;
  logic                                        w_cur_abort;
  logic                                        w_nxt_slot;
  logic [LPARA_CHANNEL_NUM-1:0]                w_vc_queued;
  logic                                        w_push;
  logic                                        w_push_vc;
  logic                                        w_push_slot;
  logic                                        w_push_abort;
  logic                                        w_record_lost;
  logic                                        w_queue_wr_idx;
  logic [LPARA_CAPTURE_SLOTS-1:0]              reg_rec_pending;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_rec_pending_set;
  logic [LPARA_CAPTURE_SLOTS-1:0]              w_rec_pending_clr;
  rec_state_type_def                           reg_fsm_rec_current_state;
  rec_state_type_def                           w_fsm_rec_nxt_state;
  logic                                        w_rec_active;
  logic                                        w_cnt_clr;
  logic                                        w_queue_pop;
  logic [LPARA_WORD_IDX_W-1:0]                 reg_word_idx;
  logic                                        w_word_last;
  logic [LPARA_TS_INT_W-1:0]                   w_cur_timestamp;
  logic [4:0]                                  w_err_summary;
  logic [31:0]                                 w_rec_word;
  logic                                        w_fifo_full;
  logic                                        w_fifo_empty;
  logic                                        w_rec_grant;
  logic                                        w_pay_wr;
  logic                                        w_pay_lost;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_ovf_frame;
  logic [LPARA_CHANNEL_NUM-1:0]                reg_sof_pend;
  logic                                        w_sof_bit;
  logic                                        reg_meta_overflow;
  logic [LPARA_META_WIDTH-1:0]                 reg_fifo_mem [PARA_META_FIFO_DEPTH-1:0];
  logic [LPARA_PTR_W-1:0]                      reg_fifo_wr_ptr;
  logic [LPARA_PTR_W-1:0]                      reg_fifo_rd_ptr;
  logic [LPARA_CNT_W-1:0]                      reg_fifo_cnt;
  logic                                        w_fifo_wr;
  logic                                        w_fifo_rd;
  logic [LPARA_META_WIDTH-1:0]                 w_fifo_din;
  logic [LPARA_META_WIDTH-1:0]                 w_fifo_dout;
  logic [31:0]                                 w_meta_data;
  logic [3:0]                                  w_meta_byte_en;
  logic                                        w_meta_valid;
  logic                                        w_meta_last;
  logic                                        w_meta_kind;
  logic [1:0]                                  w_meta_vc;
  logic                                        w_meta_sof;

  // Number of bytes marked valid in one 32-bit beat.
  function automatic logic [2:0] f_byte_num(input logic [3:0] byte_en);
    f_byte_num = 3'd0;
    for (int idx = 0; idx < 4; idx++) begin
      f_byte_num = f_byte_num + 3'(byte_en[idx]);
    end
  endfunction

  // ------------------------------------------------------------------------------------------
  // Parameter checks
  // ------------------------------------------------------------------------------------------
  generate
  if (PARA_N_EXTRACT > 32'd2) begin : gen_chk_n_extract
    $error("PARA_N_EXTRACT must be 0, 1 or 2");
  end
  if (PARA_TS_INCR == 32'd0) begin : gen_chk_ts_incr
    $error("PARA_TS_INCR must not be 0");
  end
  if (!((PARA_META_FIFO_DEPTH >= 32'd2) &&
        ((PARA_META_FIFO_DEPTH & (PARA_META_FIFO_DEPTH - 32'd1)) == 32'd0)))
  begin : gen_chk_fifo_depth
    $error("PARA_META_FIFO_DEPTH must be a power of two, at least 2");
  end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Event Source Select
  // ------------------------------------------------------------------------------------------
  // Frame events and pairing mode of the source chosen by active_sel. The source carries a frame
  // number and geometry only on the sensor path.
  always_comb begin : p_com_event_select
    w_ev_fs   = i_fc_fs;
    w_ev_fe   = i_fc_fe;
    w_ev_vc   = i_fc_vc;
    w_ev_fnum = i_fc_frame_num;
    w_ev_rx   = 1'b1;
    w_lead_vc = i_vc_exposure_map;
    w_dual_vc = 1'b1;
    if (i_active_sel) begin
      w_ev_fs   = i_tpg_fs;
      w_ev_fe   = i_tpg_fe;
      w_ev_vc   = i_tpg_fvc;
      w_ev_fnum = 16'd0;
      w_ev_rx   = 1'b0;
      w_lead_vc = 1'b0;
      w_dual_vc = i_tpg_dual_vc_en;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Capture Tracker
  // ------------------------------------------------------------------------------------------
  // A frame event is taken only while enabled and out of reset, and only on channel 0 or 1. The
  // sources deliver at most one frame event per cycle.
  always_comb begin : p_com_event_take
    w_ev_take   = i_resetn & ~i_soft_reset & i_core_enable & ~w_ev_vc[1];
    w_fs_take   = w_ev_fs & w_ev_take;
    w_fe_take   = w_ev_fe & w_ev_take;
    w_ev_ch     = w_ev_vc[0];
    w_is_lead   = (w_ev_ch == w_lead_vc);
    w_core_fall = ~i_core_enable & reg_core_enable_d;
    w_active_sel_chg = i_active_sel ^ reg_active_sel_d;
  end

  // Frame Start opens the channel and closes a frame still open as a sync close; Frame End closes
  // an open frame and is ignored otherwise.
  always_comb begin : p_com_ctx_event
    w_ctx_open       = '0;
    w_ctx_close      = '0;
    w_ctx_sync_close = '0;
    if (w_fs_take) begin
      w_ctx_open[w_ev_ch]       = 1'b1;
      w_ctx_close[w_ev_ch]      = reg_ctx_active[w_ev_ch];
      w_ctx_sync_close[w_ev_ch] = reg_ctx_active[w_ev_ch];
    end else if (w_fe_take) begin
      w_ctx_close[w_ev_ch]      = reg_ctx_active[w_ev_ch];
    end
  end

  // A lead Frame Start takes slot 0 first. A lag Frame Start joins only the most recently
  // allocated slot, and only while that slot waits for it. A slot with a queued record is not free.
  always_comb begin : p_com_slot_alloc
    w_slot_free = ~reg_slot_open & ~reg_rec_pending;
    w_alloc_en  = w_fs_take & w_is_lead & (|w_slot_free);
    w_alloc_slot = ~w_slot_free[0];
    w_lag_join  = w_fs_take & ~w_is_lead & w_dual_vc & reg_slot_wait_lag[reg_last_slot];
    w_capture_start = '0;
    w_capture_start[w_alloc_slot] = w_alloc_en;
  end

  // A drop marks the open slot of each named channel, otherwise the most recently allocated open
  // slot. It uses the attachments held before a frame event of the same cycle, and it counts as
  // earlier than a close of the same cycle.
  always_comb begin : p_com_drop
    w_recent_slot = reg_slot_open[reg_last_slot] ? reg_last_slot : ~reg_last_slot;
    w_drop_hit    = '0;
    for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
      if (i_capture_drop && i_drop_vc_mask[idx]) begin
        if (!reg_ctx_orphan[idx] && reg_slot_open[reg_ctx_slot[idx]]) begin
          w_drop_hit[reg_ctx_slot[idx]] = 1'b1;
        end else if (reg_slot_open[w_recent_slot]) begin
          w_drop_hit[w_recent_slot] = 1'b1;
        end
      end
    end
    for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
      w_ctx_abort[idx] = ~reg_ctx_orphan[idx] &
                         (reg_slot_dropped[reg_ctx_slot[idx]] | w_drop_hit[reg_ctx_slot[idx]]);
    end
  end

  // A slot ends when it no longer waits and none of its channels is open. Slot 0 goes first when
  // both qualify. The first cycle of core_enable low only marks the slots; they end after it.
  always_comb begin : p_com_capture_end
    w_slot_busy = '0;
    for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
      if (reg_ctx_active[idx] && !reg_ctx_orphan[idx]) begin
        w_slot_busy[reg_ctx_slot[idx]] = 1'b1;
      end
    end
    w_slot_eligible   = reg_slot_open & ~reg_slot_wait_lag & ~w_slot_busy;
    w_end_en          = ~i_soft_reset & ~w_core_fall;
    w_capture_end[0]  = w_end_en & w_slot_eligible[0];
    w_capture_end[1]  = w_end_en & w_slot_eligible[1] & ~w_slot_eligible[0];
    w_capture_dropped = reg_slot_dropped | w_drop_hit;
    w_capture_done    = |(w_capture_end & ~w_capture_dropped);
  end

  // Frame numbers 0 and 1 are exempt from the sequence check, which runs on the sensor path only.
  always_comb begin : p_com_fnum
    w_fnum_mismatch = w_ev_rx & i_fnum_check_en & (w_ev_fnum != 16'd0) & (w_ev_fnum != 16'd1) &
                      (w_ev_fnum != (reg_last_fnum[w_ev_ch] + 16'd1));
    w_err_frame_num = w_fs_take & w_fnum_mismatch;
  end

  // Channel context. Reset leaves both channels orphan; core_enable low detaches them without a
  // close. ctx_slot holds while a channel is orphan.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_ctx
    if (!i_resetn) begin
      reg_ctx_active    <= '0;
      reg_ctx_orphan    <= '1;
      reg_ctx_slot      <= '0;
      reg_last_fnum     <= '0;
      reg_fnum_err_flag <= '0;
    end else if (i_soft_reset) begin
      reg_ctx_active    <= '0;
      reg_ctx_orphan    <= '1;
      reg_ctx_slot      <= '0;
      reg_last_fnum     <= '0;
      reg_fnum_err_flag <= '0;
    end else if (!i_core_enable) begin
      reg_ctx_active    <= '0;
      reg_ctx_orphan    <= '1;
    end else if (w_fs_take) begin
      reg_ctx_active[w_ev_ch]    <= 1'b1;
      reg_fnum_err_flag[w_ev_ch] <= w_fnum_mismatch;
      if (w_ev_rx) begin
        reg_last_fnum[w_ev_ch]   <= w_ev_fnum;
      end
      if (w_alloc_en) begin
        reg_ctx_orphan[w_ev_ch]  <= 1'b0;
        reg_ctx_slot[w_ev_ch]    <= w_alloc_slot;
      end else if (w_lag_join) begin
        reg_ctx_orphan[w_ev_ch]  <= 1'b0;
        reg_ctx_slot[w_ev_ch]    <= reg_last_slot;
      end else begin
        reg_ctx_orphan[w_ev_ch]  <= 1'b1;
      end
    end else if (w_fe_take) begin
      reg_ctx_active[w_ev_ch]    <= 1'b0;
    end
  end

  // Capture slots. Allocation has priority over the marks of the same cycle on that slot, which
  // was free. Every lead Frame Start, core_enable low and a source change stop the lag wait.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_slot
    if (!i_resetn) begin
      reg_slot_open     <= '0;
      reg_slot_wait_lag <= '0;
      reg_slot_dropped  <= '0;
      reg_last_slot     <= 1'b0;
      reg_core_enable_d <= 1'b1;
      reg_active_sel_d  <= 1'b0;
    end else if (i_soft_reset) begin
      reg_slot_open     <= '0;
      reg_slot_wait_lag <= '0;
      reg_slot_dropped  <= '0;
      reg_last_slot     <= 1'b0;
      reg_core_enable_d <= 1'b1;
      reg_active_sel_d  <= 1'b0;
    end else begin
      reg_core_enable_d <= i_core_enable;
      reg_active_sel_d  <= i_active_sel;
      reg_slot_open     <= reg_slot_open & ~w_capture_end;
      reg_slot_dropped  <= reg_slot_dropped | w_drop_hit;
      if (!i_core_enable) begin
        reg_slot_dropped  <= reg_slot_dropped | w_drop_hit | reg_slot_open;
      end
      if (!i_core_enable || w_active_sel_chg || (w_fs_take && w_is_lead)) begin
        reg_slot_wait_lag <= '0;
      end else if (w_lag_join) begin
        reg_slot_wait_lag[reg_last_slot] <= 1'b0;
      end
      if (w_alloc_en) begin
        reg_slot_open[w_alloc_slot]     <= 1'b1;
        reg_slot_wait_lag[w_alloc_slot] <= w_dual_vc;
        reg_slot_dropped[w_alloc_slot]  <= 1'b0;
        reg_last_slot                   <= w_alloc_slot;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Identity Generator
  // ------------------------------------------------------------------------------------------
  // frame_id counts from 1 and skips 0 on wrap. A channel with no capture reports frame_id 0.
  always_comb begin : p_com_identity
    w_frame_id_nxt = (reg_frame_id_cnt == LPARA_FRAME_ID_MAX) ?
                     LPARA_FRAME_ID_WIDTH'(1) : reg_frame_id_cnt + LPARA_FRAME_ID_WIDTH'(1);
    for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
      w_cur_frame_id[idx] = reg_ctx_orphan[idx] ? '0 : reg_slot_frame_id[reg_ctx_slot[idx]];
    end
  end

  // The time base and the frame_id count survive soft_reset, so only resetn clears them.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_time_base
    if (!i_resetn) begin
      reg_ts_cnt       <= '0;
      reg_frame_id_cnt <= '0;
    end else begin
      reg_ts_cnt       <= reg_ts_cnt + (LPARA_TS_INT_W + LPARA_TS_FRAC_W)'(PARA_TS_INCR);
      if (w_alloc_en) begin
        reg_frame_id_cnt <= w_frame_id_nxt;
      end
    end
  end

  // Identity of a capture holds until its slot is allocated again. A buffer claim names the most
  // recently allocated slot.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_identity
    if (!i_resetn) begin
      reg_slot_frame_id  <= '0;
      reg_slot_timestamp <= '0;
      reg_slot_record_id <= '0;
      reg_frame_id_buf   <= '0;
    end else if (i_soft_reset) begin
      reg_slot_frame_id  <= '0;
      reg_slot_timestamp <= '0;
      reg_slot_record_id <= '0;
      reg_frame_id_buf   <= '0;
    end else begin
      if (w_alloc_en) begin
        reg_slot_frame_id[w_alloc_slot]  <= w_frame_id_nxt;
        reg_slot_timestamp[w_alloc_slot] <= reg_ts_cnt[LPARA_TS_INT_W+LPARA_TS_FRAC_W-1:LPARA_TS_FRAC_W];
        reg_slot_record_id[w_alloc_slot] <= i_metadata_record_id;
      end
      if (i_dma_buf_claim) begin
        reg_frame_id_buf[i_dma_buf_index] <= reg_slot_frame_id[reg_last_slot];
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Geometry Accumulator
  // ------------------------------------------------------------------------------------------
  // Beats count on the sensor path only. bp_sol restarts the sum with its own beat; the sum at
  // bp_eol is the length of the line.
  always_comb begin : p_com_geometry
    w_bp_take    = i_bp_valid & w_ev_rx & ~i_bp_vc_tag[1];
    w_bp_ch      = i_bp_vc_tag[0];
    w_line_bytes = (i_bp_sol ? 16'd0 : reg_byte_acc[w_bp_ch]) + 16'(f_byte_num(i_bp_byte_en));
    w_line_done  = '0;
    w_line_done[w_bp_ch] = w_bp_take & i_bp_eol;
  end

  // ctx_open clears a channel on either path, so a frame of the generator reports zero geometry.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_geometry
    if (!i_resetn) begin
      reg_byte_acc       <= '0;
      reg_acc_byte_count <= '0;
      reg_acc_line_count <= '0;
      reg_acc_dt         <= '0;
    end else if (i_soft_reset) begin
      reg_byte_acc       <= '0;
      reg_acc_byte_count <= '0;
      reg_acc_line_count <= '0;
      reg_acc_dt         <= '0;
    end else begin
      if (w_bp_take) begin
        reg_byte_acc[w_bp_ch] <= w_line_bytes;
        if (i_bp_eol) begin
          reg_acc_byte_count[w_bp_ch] <= w_line_bytes;
          reg_acc_line_count[w_bp_ch] <= reg_acc_line_count[w_bp_ch] + 16'd1;
          reg_acc_dt[w_bp_ch]         <= i_bp_dt_sel ? LPARA_DT_RAW10 : LPARA_DT_RAW12;
        end
      end
      for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
        if (w_ctx_open[idx]) begin
          reg_byte_acc[idx]       <= 16'd0;
          reg_acc_byte_count[idx] <= 16'd0;
          reg_acc_line_count[idx] <= 16'd0;
          reg_acc_dt[idx]         <= 6'd0;
        end
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Resolution Checker
  // ------------------------------------------------------------------------------------------
  // Bytes per line of the sampled width, RAW10 when bp_dt_sel is set and RAW12 otherwise, kept at
  // full precision. Height is compared only on the sensor path. A dropped capture, an orphan
  // channel and a sync close report no error.
  always_comb begin : p_com_resolution
    w_bytes_ref = i_bp_dt_sel ? 17'((19'(reg_expected_width[w_bp_ch]) * 19'd5) >> 2) :
                                17'((18'(reg_expected_width[w_bp_ch]) * 18'd3) >> 1);
    w_err_resolution = 1'b0;
    for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
      w_height_mismatch[idx] = w_ev_rx & (reg_acc_line_count[idx] != reg_expected_height[idx]);
      if (w_ctx_close[idx] && !w_ctx_sync_close[idx] && w_ev_rx && !reg_ctx_orphan[idx] &&
          !w_ctx_abort[idx] && (reg_width_mismatch[idx] || w_height_mismatch[idx])) begin
        w_err_resolution = 1'b1;
      end
    end
  end

  // The expected size is sampled at each Frame Start; width_mismatch is sticky until the next one.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_resolution
    if (!i_resetn) begin
      reg_expected_width  <= '0;
      reg_expected_height <= '0;
      reg_width_mismatch  <= '0;
    end else if (i_soft_reset) begin
      reg_expected_width  <= '0;
      reg_expected_height <= '0;
      reg_width_mismatch  <= '0;
    end else begin
      for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
        if (w_ctx_open[idx]) begin
          reg_expected_width[idx]  <= i_expected_width;
          reg_expected_height[idx] <= i_expected_height;
          reg_width_mismatch[idx]  <= 1'b0;
        end else if (w_line_done[idx] && ({1'b0, w_line_bytes} != w_bytes_ref)) begin
          reg_width_mismatch[idx]  <= 1'b1;
        end
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Embedded Data Path
  // ------------------------------------------------------------------------------------------
  // Payload passes unchanged on the sensor path while capture is enabled. Beats are counted here,
  // so a word later lost on a full staging FIFO is still counted.
  always_comb begin : p_com_embedded
    w_emb_fwd_valid = i_emb_valid & w_ev_rx & i_metadata_capture_en & ~i_emb_vc[1];
    w_emb_ch        = i_emb_vc[0];
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_embedded
    if (!i_resetn) begin
      reg_emb_word_cnt <= '0;
      reg_emb_line_cnt <= '0;
    end else if (i_soft_reset) begin
      reg_emb_word_cnt <= '0;
      reg_emb_line_cnt <= '0;
    end else begin
      if (w_emb_fwd_valid) begin
        reg_emb_word_cnt[w_emb_ch] <= reg_emb_word_cnt[w_emb_ch] + 16'd1;
        if (i_emb_eol) begin
          reg_emb_line_cnt[w_emb_ch] <= reg_emb_line_cnt[w_emb_ch] + 8'd1;
        end
      end
      for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
        if (w_ctx_open[idx]) begin
          reg_emb_word_cnt[idx] <= 16'd0;
          reg_emb_line_cnt[idx] <= 8'd0;
        end
      end
    end
  end

  generate
  if (PARA_N_EXTRACT != 32'd0) begin : gen_extract
    logic [15:0]                                     reg_emb_byte_ptr;
    logic [15:0]                                     w_emb_ptr;
    logic [PARA_N_EXTRACT-1:0][31:0]                 w_extract_cur;
    logic [PARA_N_EXTRACT-1:0][31:0]                 w_extract_nxt;
    logic [31:0]                                     w_slot_cfg;
    logic [31:0]                                     w_slot_val;
    logic [2:0]                                      w_field_len;
    logic [16:0]                                     w_field_start;
    logic [16:0]                                     w_byte_pos;
    logic [16:0]                                     w_field_pos;
    logic [LPARA_CHANNEL_NUM-1:0][PARA_N_EXTRACT-1:0][31:0] reg_extract_val;
    logic [LPARA_CHANNEL_NUM-1:0][PARA_N_EXTRACT-1:0][31:0] reg_bank_extract_val;
    logic [LPARA_CHANNEL_NUM-1:0][PARA_N_EXTRACT-1:0][31:0] reg_pub_extract_val;
    logic [PARA_N_EXTRACT-1:0][31:0]                 w_bank_extract_cur;
    logic                                            w_unused_cfg;

    // The pointer counts payload bytes from emb_sol. A field longer than four bytes is taken as
    // four. The value clears at the first byte of the field and bytes land from bit 0 upward; a
    // line ending early keeps the bytes captured so far.
    always_comb begin : p_com_extract
      w_emb_ptr     = i_emb_sol ? 16'd0 : reg_emb_byte_ptr;
      w_extract_cur = reg_extract_val[w_emb_ch];
      w_extract_nxt = w_extract_cur;
      w_slot_cfg    = 32'd0;
      w_slot_val    = 32'd0;
      w_field_len   = 3'd0;
      w_field_start = 17'd0;
      w_byte_pos    = 17'd0;
      w_field_pos   = 17'd0;
      w_unused_cfg  = 1'b0;
      for (int slot_idx = 0; slot_idx < PARA_N_EXTRACT; slot_idx++) begin
        w_slot_cfg    = i_emb_extract_cfg[slot_idx];
        w_slot_val    = w_extract_cur[slot_idx];
        w_unused_cfg  = w_unused_cfg | (|w_slot_cfg[7:3]);
        w_field_len   = (w_slot_cfg[2:0] > 3'd4) ? 3'd4 : w_slot_cfg[2:0];
        w_field_start = 17'(w_slot_cfg[23:8]);
        if (w_slot_cfg[31:24] == reg_emb_line_cnt[w_emb_ch]) begin
          for (int byte_idx = 0; byte_idx < 4; byte_idx++) begin
            w_byte_pos  = 17'(w_emb_ptr) + 17'(byte_idx);
            w_field_pos = w_byte_pos - w_field_start;
            if (i_emb_byte_en[byte_idx] && (w_byte_pos >= w_field_start) &&
                (w_field_pos < 17'(w_field_len))) begin
              if (w_field_pos == 17'd0) begin
                w_slot_val = 32'd0;
              end
              w_slot_val[{w_field_pos[1:0], 3'b000} +: 8] = i_emb_data[(8 * byte_idx) +: 8];
            end
          end
        end
        w_extract_nxt[slot_idx] = w_slot_val;
      end
    end

    // Extracted values survive ctx_open, so a frame with no embedded lines carries the previous
    // ones. The bank and the published copy load with the other fields of a close.
    always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_extract
      if (!i_resetn) begin
        reg_emb_byte_ptr     <= 16'd0;
        reg_extract_val      <= '0;
        reg_bank_extract_val <= '0;
        reg_pub_extract_val  <= '0;
      end else if (i_soft_reset) begin
        reg_emb_byte_ptr     <= 16'd0;
        reg_extract_val      <= '0;
        reg_bank_extract_val <= '0;
        reg_pub_extract_val  <= '0;
      end else begin
        if (w_emb_fwd_valid) begin
          reg_emb_byte_ptr <= w_emb_ptr + 16'(f_byte_num(i_emb_byte_en));
          if (i_emb_extract_en) begin
            reg_extract_val[w_emb_ch] <= w_extract_nxt;
          end
        end
        for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
          if (w_bank_load[idx]) begin
            reg_bank_extract_val[idx] <= reg_extract_val[idx];
          end
          if (w_pub_load[idx]) begin
            reg_pub_extract_val[idx]  <= reg_extract_val[idx];
          end
        end
      end
    end

    // Extraction words follow the fixed words of a record.
    always_comb begin : p_com_extract_out
      w_bank_extract_cur = reg_bank_extract_val[w_cur_vc];
      w_rec_extract      = 32'd0;
      for (int slot_idx = 0; slot_idx < PARA_N_EXTRACT; slot_idx++) begin
        if (reg_word_idx == LPARA_WORD_IDX_W'(slot_idx + 7)) begin
          w_rec_extract = w_bank_extract_cur[slot_idx];
        end
      end
      for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
        w_extract_pub[idx] = reg_pub_extract_val[idx];
      end
    end
  end else begin : gen_no_extract
    logic w_unused_extract;

    // With no extraction slot the values read zero and these inputs have no function.
    always_comb begin : p_com_no_extract
      w_rec_extract    = 32'd0;
      w_extract_pub    = '0;
      w_unused_extract = &{1'b0, i_emb_sol, i_emb_extract_en, i_emb_extract_cfg};
    end
  end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Context Latch
  // ------------------------------------------------------------------------------------------
  // Channels that already have a record waiting in the queue.
  always_comb begin : p_com_vc_queued
    w_vc_queued = '0;
    if (reg_queue_cnt != 2'd0) begin
      w_vc_queued[reg_queue_vc[0]] = 1'b1;
    end
    if (reg_queue_cnt == 2'd2) begin
      w_vc_queued[reg_queue_vc[1]] = 1'b1;
    end
  end

  // The bank of a channel loads at a close that is queued for a record. Publication needs a sensor
  // frame of a capture that is not dropped, and does not depend on the queue.
  always_comb begin : p_com_context_latch
    w_close_ok  = w_ctx_close & ~reg_ctx_orphan;
    w_bank_load = w_close_ok & ~w_vc_queued;
    w_pub_load  = w_close_ok & ~w_ctx_abort & {LPARA_CHANNEL_NUM{w_ev_rx}};
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_bank
    if (!i_resetn) begin
      reg_bank_line_count      <= '0;
      reg_bank_byte_count      <= '0;
      reg_bank_dt              <= '0;
      reg_bank_width_mismatch  <= '0;
      reg_bank_height_mismatch <= '0;
      reg_bank_emb_word_cnt    <= '0;
      reg_bank_emb_line_cnt    <= '0;
      reg_bank_fnum_err        <= '0;
      reg_bank_ovf             <= '0;
    end else if (i_soft_reset) begin
      reg_bank_line_count      <= '0;
      reg_bank_byte_count      <= '0;
      reg_bank_dt              <= '0;
      reg_bank_width_mismatch  <= '0;
      reg_bank_height_mismatch <= '0;
      reg_bank_emb_word_cnt    <= '0;
      reg_bank_emb_line_cnt    <= '0;
      reg_bank_fnum_err        <= '0;
      reg_bank_ovf             <= '0;
    end else begin
      for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
        if (w_bank_load[idx]) begin
          reg_bank_line_count[idx]      <= reg_acc_line_count[idx];
          reg_bank_byte_count[idx]      <= reg_acc_byte_count[idx];
          reg_bank_dt[idx]              <= reg_acc_dt[idx];
          reg_bank_width_mismatch[idx]  <= reg_width_mismatch[idx];
          reg_bank_height_mismatch[idx] <= w_height_mismatch[idx];
          reg_bank_emb_word_cnt[idx]    <= reg_emb_word_cnt[idx];
          reg_bank_emb_line_cnt[idx]    <= reg_emb_line_cnt[idx];
          reg_bank_fnum_err[idx]        <= reg_fnum_err_flag[idx];
          reg_bank_ovf[idx]             <= reg_ovf_frame[idx];
        end
      end
    end
  end

  // Published channel registers keep the last frame that met the publication condition.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_publish
    if (!i_resetn) begin
      reg_pub_line_count      <= '0;
      reg_pub_byte_count      <= '0;
      reg_pub_data_type       <= '0;
      reg_pub_width_mismatch  <= '0;
      reg_pub_height_mismatch <= '0;
      reg_pub_emb_word_count  <= '0;
      reg_pub_emb_line_count  <= '0;
    end else if (i_soft_reset) begin
      reg_pub_line_count      <= '0;
      reg_pub_byte_count      <= '0;
      reg_pub_data_type       <= '0;
      reg_pub_width_mismatch  <= '0;
      reg_pub_height_mismatch <= '0;
      reg_pub_emb_word_count  <= '0;
      reg_pub_emb_line_count  <= '0;
    end else begin
      for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
        if (w_pub_load[idx]) begin
          reg_pub_line_count[idx]      <= reg_acc_line_count[idx];
          reg_pub_byte_count[idx]      <= reg_acc_byte_count[idx];
          reg_pub_data_type[idx]       <= reg_acc_dt[idx];
          reg_pub_width_mismatch[idx]  <= reg_width_mismatch[idx];
          reg_pub_height_mismatch[idx] <= w_height_mismatch[idx];
          reg_pub_emb_word_count[idx]  <= reg_emb_word_cnt[idx];
          reg_pub_emb_line_count[idx]  <= reg_emb_line_cnt[idx];
        end
      end
    end
  end

  // frame_count advances at every capture end. The timestamp loads only at the end of a capture
  // that is not dropped.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_capture_pub
    if (!i_resetn) begin
      reg_timestamp   <= '0;
      reg_frame_count <= 32'd0;
    end else if (i_soft_reset) begin
      reg_timestamp   <= '0;
      reg_frame_count <= 32'd0;
    end else begin
      if (|w_capture_end) begin
        reg_frame_count <= reg_frame_count + 32'd1;
      end
      for (int idx = 0; idx < LPARA_CAPTURE_SLOTS; idx++) begin
        if (w_capture_end[idx] && !w_capture_dropped[idx]) begin
          reg_timestamp <= reg_slot_timestamp[idx];
        end
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Record Builder
  // ------------------------------------------------------------------------------------------
  // A close enters the queue only when no record of its channel is queued, which also bounds the
  // queue to two entries. Otherwise the older record and its bank stay intact and the loss is
  // reported on meta_overflow.
  always_comb begin : p_com_queue
    w_queue_empty  = (reg_queue_cnt == 2'd0);
    w_more_pending = (reg_queue_cnt == 2'd2);
    w_cur_vc       = reg_queue_vc[0];
    w_cur_slot     = reg_queue_slot[0];
    w_cur_abort    = reg_queue_abort[0];
    w_nxt_slot     = reg_queue_slot[1];
    w_push         = |w_bank_load;
    w_record_lost  = |(w_close_ok & w_vc_queued);
    w_push_vc      = w_close_ok[1];
    w_push_slot    = reg_ctx_slot[w_push_vc];
    w_push_abort   = w_ctx_abort[w_push_vc];
    w_queue_wr_idx = reg_queue_cnt[0] & ~w_queue_pop;
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_queue
    if (!i_resetn) begin
      reg_queue_vc    <= '0;
      reg_queue_slot  <= '0;
      reg_queue_abort <= '0;
      reg_queue_cnt   <= 2'd0;
    end else if (i_soft_reset) begin
      reg_queue_vc    <= '0;
      reg_queue_slot  <= '0;
      reg_queue_abort <= '0;
      reg_queue_cnt   <= 2'd0;
    end else begin
      if (w_queue_pop) begin
        reg_queue_vc[0]    <= reg_queue_vc[1];
        reg_queue_slot[0]  <= reg_queue_slot[1];
        reg_queue_abort[0] <= reg_queue_abort[1];
      end
      if (w_push) begin
        reg_queue_vc[w_queue_wr_idx]    <= w_push_vc;
        reg_queue_slot[w_queue_wr_idx]  <= w_push_slot;
        reg_queue_abort[w_queue_wr_idx] <= w_push_abort;
      end
      reg_queue_cnt <= reg_queue_cnt + 2'(w_push) - 2'(w_queue_pop);
    end
  end

  // rec_pending holds a slot from the first push naming it until the last such entry leaves.
  always_comb begin : p_com_rec_pending
    w_rec_pending_set = '0;
    w_rec_pending_clr = '0;
    w_rec_pending_set[w_push_slot] = w_push;
    w_rec_pending_clr[w_cur_slot]  = w_queue_pop & ~(w_more_pending & (w_nxt_slot == w_cur_slot));
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_rec_pending
    if (!i_resetn) begin
      reg_rec_pending <= '0;
    end else if (i_soft_reset) begin
      reg_rec_pending <= '0;
    end else begin
      reg_rec_pending <= (reg_rec_pending & ~w_rec_pending_clr) | w_rec_pending_set;
    end
  end

  // Moore outputs of Record FSM.
  always_comb begin : p_com_fsm_rec_out
    w_rec_active = (reg_fsm_rec_current_state == ENUM_ST_RECORD);
    w_cnt_clr    = (reg_fsm_rec_current_state != ENUM_ST_RECORD);
    w_queue_pop  = (reg_fsm_rec_current_state == ENUM_ST_POP);
  end

  // Record FSM sequences one record: IDLE leaves on a queued entry, RECORD leaves on the grant of
  // the last word, POP lasts one cycle. Unused codes recover to IDLE.
  always_comb begin : p_com_fsm_rec
    w_fsm_rec_nxt_state = reg_fsm_rec_current_state;
    case (reg_fsm_rec_current_state)
      ENUM_ST_IDLE: begin
        if (!w_queue_empty) begin
          w_fsm_rec_nxt_state = ENUM_ST_RECORD;
        end
      end
      ENUM_ST_RECORD: begin
        if (w_rec_grant && w_word_last) begin
          w_fsm_rec_nxt_state = ENUM_ST_POP;
        end
      end
      ENUM_ST_POP: begin
        if (w_more_pending) begin
          w_fsm_rec_nxt_state = ENUM_ST_RECORD;
        end else begin
          w_fsm_rec_nxt_state = ENUM_ST_IDLE;
        end
      end
      default: begin
        w_fsm_rec_nxt_state = ENUM_ST_IDLE;
      end
    endcase
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_fsm_rec
    if (!i_resetn) begin
      reg_fsm_rec_current_state <= ENUM_ST_IDLE;
      reg_word_idx              <= '0;
    end else if (i_soft_reset) begin
      reg_fsm_rec_current_state <= ENUM_ST_IDLE;
      reg_word_idx              <= '0;
    end else begin
      reg_fsm_rec_current_state <= w_fsm_rec_nxt_state;
      if (w_cnt_clr) begin
        reg_word_idx <= '0;
      end else if (w_rec_grant) begin
        reg_word_idx <= reg_word_idx + LPARA_WORD_IDX_W'(1);
      end
    end
  end

  // Record words of the queue head: identity from its capture slot, geometry from its channel bank.
  always_comb begin : p_com_rec_word
    w_word_last   = (reg_word_idx == LPARA_WORD_LAST);
    w_cur_timestamp = reg_slot_timestamp[w_cur_slot];
    w_err_summary = {reg_bank_ovf[w_cur_vc], reg_bank_height_mismatch[w_cur_vc],
                     reg_bank_width_mismatch[w_cur_vc], reg_bank_fnum_err[w_cur_vc], w_cur_abort};
    case (4'(reg_word_idx))
      4'd0:    w_rec_word = {11'd0, w_err_summary, 6'd0, 1'b0, w_cur_vc, LPARA_RECORD_VERSION};
      4'd1:    w_rec_word = {18'd0, reg_slot_frame_id[w_cur_slot]};
      4'd2:    w_rec_word = w_cur_timestamp[31:0];
      4'd3:    w_rec_word = w_cur_timestamp[63:32];
      4'd4:    w_rec_word = reg_slot_record_id[w_cur_slot];
      4'd5:    w_rec_word = {reg_bank_line_count[w_cur_vc], reg_bank_byte_count[w_cur_vc]};
      4'd6:    w_rec_word = {2'd0, reg_bank_dt[w_cur_vc], reg_bank_emb_line_cnt[w_cur_vc],
                             reg_bank_emb_word_cnt[w_cur_vc]};
      default: w_rec_word = w_rec_extract;
    endcase
  end

  // Payload has priority on the staging FIFO. A record word waits for space and a free cycle; a
  // payload beat that meets a full FIFO is lost.
  always_comb begin : p_com_fifo_write
    w_fifo_full  = (reg_fifo_cnt == LPARA_DEPTH_CNT);
    w_fifo_empty = (reg_fifo_cnt == '0);
    w_pay_wr     = w_emb_fwd_valid & ~w_fifo_full;
    w_pay_lost   = w_emb_fwd_valid & w_fifo_full;
    w_rec_grant  = w_rec_active & ~w_emb_fwd_valid & ~w_fifo_full;
    w_sof_bit    = reg_sof_pend[w_emb_ch];
    w_fifo_wr    = w_pay_wr | w_rec_grant;
    w_fifo_rd    = ~w_fifo_empty & i_meta_ready;
    w_fifo_din   = w_pay_wr ?
                   {i_emb_data, i_emb_byte_en, i_emb_eol, 1'b1, w_sof_bit, i_emb_vc} :
                   {w_rec_word, 4'b1111, w_word_last, 1'b0, 1'b0, 1'b0, w_cur_vc};
  end

  // ovf_frame and sof_pend restart at ctx_open of their channel. meta_overflow stays set until a
  // reset; it also reports a record that could not be queued.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_payload_flags
    if (!i_resetn) begin
      reg_ovf_frame     <= '0;
      reg_sof_pend      <= '0;
      reg_meta_overflow <= 1'b0;
    end else if (i_soft_reset) begin
      reg_ovf_frame     <= '0;
      reg_sof_pend      <= '0;
      reg_meta_overflow <= 1'b0;
    end else begin
      if (w_pay_lost) begin
        reg_ovf_frame[w_emb_ch] <= 1'b1;
      end
      if (w_pay_wr) begin
        reg_sof_pend[w_emb_ch]  <= 1'b0;
      end
      for (int idx = 0; idx < LPARA_CHANNEL_NUM; idx++) begin
        if (w_ctx_open[idx]) begin
          reg_ovf_frame[idx] <= 1'b0;
          reg_sof_pend[idx]  <= 1'b1;
        end
      end
      if (w_pay_lost || w_record_lost) begin
        reg_meta_overflow <= 1'b1;
      end
    end
  end

  // The staging memory has no reset: an entry is read only after it has been written.
  always_ff @(posedge i_sys_clk) begin : p_ff_fifo_mem
    if (w_fifo_wr) begin
      reg_fifo_mem[reg_fifo_wr_ptr] <= w_fifo_din;
    end
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_fifo_ctrl
    if (!i_resetn) begin
      reg_fifo_wr_ptr <= '0;
      reg_fifo_rd_ptr <= '0;
      reg_fifo_cnt    <= '0;
    end else if (i_soft_reset) begin
      reg_fifo_wr_ptr <= '0;
      reg_fifo_rd_ptr <= '0;
      reg_fifo_cnt    <= '0;
    end else begin
      if (w_fifo_wr) begin
        reg_fifo_wr_ptr <= reg_fifo_wr_ptr + LPARA_PTR_W'(1);
      end
      if (w_fifo_rd) begin
        reg_fifo_rd_ptr <= reg_fifo_rd_ptr + LPARA_PTR_W'(1);
      end
      reg_fifo_cnt <= reg_fifo_cnt + LPARA_CNT_W'(w_fifo_wr) - LPARA_CNT_W'(w_fifo_rd);
    end
  end

  // The head entry drives the stream directly; every field reads zero while the FIFO is empty.
  always_comb begin : p_com_meta_out
    w_fifo_dout    = reg_fifo_mem[reg_fifo_rd_ptr];
    w_meta_valid   = ~w_fifo_empty;
    w_meta_data    = 32'd0;
    w_meta_byte_en = 4'd0;
    w_meta_last    = 1'b0;
    w_meta_kind    = 1'b0;
    w_meta_sof     = 1'b0;
    w_meta_vc      = 2'd0;
    if (!w_fifo_empty) begin
      {w_meta_data, w_meta_byte_en, w_meta_last, w_meta_kind, w_meta_sof, w_meta_vc} = w_fifo_dout;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  // Capture events and frame events.
  assign o_capture_start   = w_capture_start;
  assign o_capture_end     = w_capture_end;
  assign o_capture_dropped = w_capture_dropped;
  assign o_ev_fs           = w_ev_fs;
  assign o_ev_vc           = w_ev_vc;
  assign o_ctx_slot        = reg_ctx_slot;
  assign o_ctx_orphan      = reg_ctx_orphan;
  assign o_lead_vc         = w_lead_vc;

  // Published capture registers.
  assign o_frame_count     = reg_frame_count;
  assign o_frame_id_buf0   = reg_frame_id_buf[0];
  assign o_frame_id_buf1   = reg_frame_id_buf[1];
  assign o_timestamp       = reg_timestamp;

  // Published and live registers of each channel.
  assign o_vc0_line_count      = reg_pub_line_count[0];
  assign o_vc0_byte_count      = reg_pub_byte_count[0];
  assign o_vc0_data_type       = reg_pub_data_type[0];
  assign o_vc0_width_mismatch  = reg_pub_width_mismatch[0];
  assign o_vc0_height_mismatch = reg_pub_height_mismatch[0];
  assign o_emb_word_count_vc0  = reg_pub_emb_word_count[0];
  assign o_emb_line_count_vc0  = reg_pub_emb_line_count[0];
  assign o_emb_extract_val_vc0 = w_extract_pub[0];
  assign o_cur_frame_id_vc0    = w_cur_frame_id[0];
  assign o_cur_line_vc0        = reg_acc_line_count[0];
  assign o_vc1_line_count      = reg_pub_line_count[1];
  assign o_vc1_byte_count      = reg_pub_byte_count[1];
  assign o_vc1_data_type       = reg_pub_data_type[1];
  assign o_vc1_width_mismatch  = reg_pub_width_mismatch[1];
  assign o_vc1_height_mismatch = reg_pub_height_mismatch[1];
  assign o_emb_word_count_vc1  = reg_pub_emb_word_count[1];
  assign o_emb_line_count_vc1  = reg_pub_emb_line_count[1];
  assign o_emb_extract_val_vc1 = w_extract_pub[1];
  assign o_cur_frame_id_vc1    = w_cur_frame_id[1];
  assign o_cur_line_vc1        = reg_acc_line_count[1];

  // Error pulses and interrupt sources.
  assign o_meta_overflow   = reg_meta_overflow;
  assign o_err_frame_num   = w_err_frame_num;
  assign o_err_resolution  = w_err_resolution;
  assign o_capture_done    = w_capture_done;

  // Metadata stream towards the DMA.
  assign o_meta_data       = w_meta_data;
  assign o_meta_byte_en    = w_meta_byte_en;
  assign o_meta_valid      = w_meta_valid;
  assign o_meta_last       = w_meta_last;
  assign o_meta_kind       = w_meta_kind;
  assign o_meta_vc         = w_meta_vc;
  assign o_meta_sof        = w_meta_sof;

endmodule
`default_nettype wire
