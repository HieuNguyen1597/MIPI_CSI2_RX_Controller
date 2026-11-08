// File        : tb_top.sv
// Description : Testbench of m_fpt_frame_context_metadata (capture pairing of the two Virtual
//               Channels, frame identity and timestamp, geometry and resolution check, embedded
//               data forwarding and field extraction, publication, metadata records and the
//               staging FIFO towards the DMA).
//               A cycle-based reference model, written from the specification, predicts every
//               output cycle by cycle; a checker compares the outputs with it at each sys_clk edge.
//               A second set of checkers tests rules that hold whatever the traffic: event source
//               selection, capture pulse rules, cur_frame_id of an orphan channel, the meta_*
//               stream (stable under stall, zero while idle, record shape), registers that may
//               change only after their event, reset values and X/Z. Tests drive stimulus and
//               check explicit expected values.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions (agreed with the block owner; the specification does not state them yet):
//   - ev_fs, ev_vc, lead_vc, capture_start, err_frame_num and err_resolution are combinational
//     in the cycle of the frame event. ctx_slot, ctx_orphan, cur_frame_id and cur_line change at
//     the clock edge that ends that cycle. capture_end and capture_done come from registered
//     state only, at the earliest in the cycle after the close that empties the slot.
//   - After resetn and after soft_reset every channel is an orphan, so cur_frame_id is 0.
//   - When both slots are free, slot 0 is allocated. Two slots able to end in the same cycle
//     end in order, slot 1 one cycle after slot 0; frame_count advances by one per capture_end.
//   - Frame events on VC2 and VC3 have no effect on the capture state; ev_fs and ev_vc still
//     follow the selected source.
//   - A change of active_sel clears the lag wait of every open slot.
//   - The frame-number check compares modulo 2^16; last_fnum is cleared by resetn and
//     soft_reset and holds while core_enable is low. A Frame Start of an orphan channel is
//     checked and updates last_fnum. On the Test Pattern Generator path there is no check,
//     last_fnum holds and the fnum_err flag of the channel is cleared.
//   - The resolution flags are computed on every sensor close, a ctx_sync_close included, and
//     reach the record and the published registers; only the err_resolution pulse is
//     suppressed. On the Test Pattern Generator path both flags stay 0. The bytes-per-line
//     reference is computed without truncation, and expected values of 0 are not a disable.
//   - data_type is 0x2B for bp_dt_sel = 1 and 0x2C for bp_dt_sel = 0. The 16-bit and 8-bit
//     counters wrap.
//   - emb_word_count counts every embedded beat taken, also the beats lost on a full FIFO.
//     Payload of an orphan channel is still forwarded.
//   - ctx_open clears the per-channel counters on both paths; it does not clear the extracted
//     values, which keep their content until a field is matched again.
//   - Extractor: len 0 disables a slot, len above 4 acts as 4. The value is cleared when the
//     first byte of the field is taken. A field is taken from one line only; when the line ends
//     first, the bytes taken stay. emb_line_count advances at emb_eol, and the line index of a
//     line is the number of embedded lines completed before it, so a line cut before emb_eol
//     does not advance it.
//   - Record queue: a close that finds a record of its channel in the queue (the record leaving
//     in the ST_POP cycle included) is not queued, its bank is not loaded and meta_overflow is
//     set; publication still takes place. An entry pushed in the ST_POP cycle is taken up from
//     ST_IDLE. A slot is held while a record that names it is queued.
//   - A capture_drop in the cycle of a close counts as earlier than the close. A channel that is
//     an orphan, or whose slot has ended, names the most recently allocated open slot.
//   - core_enable low: the first low cycle detaches both channels and marks every open slot at
//     its closing edge; capture_end follows from the next cycle, even if core_enable is high
//     again. Geometry and embedded accounting are not gated by core_enable.
//   - In a soft_reset cycle no frame event is taken and capture_start, capture_end,
//     capture_done and err_* stay 0.
//   - Staging FIFO: first-word fall-through; meta_* read 0 while it is empty. Payload has
//     priority; a record word is written only with no payload beat and a FIFO that is not full
//     at the start of the cycle.
//   - Parameter ranges: PARA_N_EXTRACT 0 to 2 (at 0 the extractor ports stay one slot wide and
//     read 0, and the record has 7 words), PARA_META_FIFO_DEPTH a power of two of at least 2,
//     PARA_TS_INCR not 0.
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_N_EXTRACT       = 32'd2;
  parameter logic [31:0] PARA_TS_INCR         = 32'h0400_0000;
  parameter logic [31:0] PARA_META_FIFO_DEPTH = 32'd16;

  localparam int NX    = int'(PARA_N_EXTRACT);
  localparam int NP    = (NX == 0) ? 1 : NX;
  localparam int RW    = 7 + NX;
  localparam int DEPTH = int'(PARA_META_FIFO_DEPTH);

  // sys_clk, 200 MHz.
  localparam real T_SYS_CLK = 5.0;

  // Data Type codes and the default frame of the tests: width 64 gives 80 bytes per RAW10 line
  // and 96 bytes per RAW12 line.
  localparam logic [5:0] DT_RAW10 = 6'h2B;
  localparam logic [5:0] DT_RAW12 = 6'h2C;
  localparam int W_DEF = 64;
  localparam int H_DEF = 3;
  localparam int B10   = 80;
  localparam int B12   = 96;

  // DUT inputs.
  logic                 i_sys_clk, i_resetn, i_soft_reset, i_core_enable;
  logic                 i_metadata_capture_en, i_fnum_check_en, i_emb_extract_en;
  logic [15:0]          i_expected_width, i_expected_height;
  logic [31:0]          i_metadata_record_id;
  logic [NP-1:0][31:0]  i_emb_extract_cfg;
  logic                 i_vc_exposure_map, i_tpg_dual_vc_en, i_active_sel;
  logic                 i_fc_fs, i_fc_fe;
  logic [15:0]          i_fc_frame_num;
  logic [1:0]           i_fc_vc;
  logic                 i_tpg_fs, i_tpg_fe;
  logic [1:0]           i_tpg_fvc;
  logic                 i_bp_valid;
  logic [3:0]           i_bp_byte_en;
  logic                 i_bp_sol, i_bp_eol;
  logic [1:0]           i_bp_vc_tag;
  logic                 i_bp_dt_sel;
  logic [31:0]          i_emb_data;
  logic [3:0]           i_emb_byte_en;
  logic                 i_emb_valid, i_emb_sol, i_emb_eol;
  logic [1:0]           i_emb_vc;
  logic                 i_capture_drop;
  logic [1:0]           i_drop_vc_mask;
  logic                 i_dma_buf_claim, i_dma_buf_index;
  logic                 i_meta_ready;

  // DUT outputs.
  logic [1:0]           o_capture_start, o_capture_end, o_capture_dropped;
  logic                 o_ev_fs;
  logic [1:0]           o_ev_vc;
  logic [1:0]           o_ctx_slot, o_ctx_orphan;
  logic                 o_lead_vc;
  logic [31:0]          o_frame_count;
  logic [13:0]          o_frame_id_buf0, o_frame_id_buf1;
  logic [63:0]          o_timestamp;
  logic [15:0]          o_vc0_line_count, o_vc0_byte_count;
  logic [5:0]           o_vc0_data_type;
  logic                 o_vc0_width_mismatch, o_vc0_height_mismatch;
  logic [15:0]          o_emb_word_count_vc0;
  logic [7:0]           o_emb_line_count_vc0;
  logic [NP-1:0][31:0]  o_emb_extract_val_vc0;
  logic [13:0]          o_cur_frame_id_vc0;
  logic [15:0]          o_cur_line_vc0;
  logic [15:0]          o_vc1_line_count, o_vc1_byte_count;
  logic [5:0]           o_vc1_data_type;
  logic                 o_vc1_width_mismatch, o_vc1_height_mismatch;
  logic [15:0]          o_emb_word_count_vc1;
  logic [7:0]           o_emb_line_count_vc1;
  logic [NP-1:0][31:0]  o_emb_extract_val_vc1;
  logic [13:0]          o_cur_frame_id_vc1;
  logic [15:0]          o_cur_line_vc1;
  logic                 o_meta_overflow, o_err_frame_num, o_err_resolution, o_capture_done;
  logic [31:0]          o_meta_data;
  logic [3:0]           o_meta_byte_en;
  logic                 o_meta_valid, o_meta_last, o_meta_kind;
  logic [1:0]           o_meta_vc;
  logic                 o_meta_sof;

  m_fpt_frame_context_metadata #(
    .PARA_N_EXTRACT       (PARA_N_EXTRACT),
    .PARA_TS_INCR         (PARA_TS_INCR),
    .PARA_META_FIFO_DEPTH (PARA_META_FIFO_DEPTH)
  ) u_dut (
    .i_sys_clk             (i_sys_clk),
    .i_resetn              (i_resetn),
    .i_soft_reset          (i_soft_reset),
    .i_core_enable         (i_core_enable),
    .i_metadata_capture_en (i_metadata_capture_en),
    .i_fnum_check_en       (i_fnum_check_en),
    .i_emb_extract_en      (i_emb_extract_en),
    .i_expected_width      (i_expected_width),
    .i_expected_height     (i_expected_height),
    .i_metadata_record_id  (i_metadata_record_id),
    .i_emb_extract_cfg     (i_emb_extract_cfg),
    .i_vc_exposure_map     (i_vc_exposure_map),
    .i_tpg_dual_vc_en      (i_tpg_dual_vc_en),
    .i_active_sel          (i_active_sel),
    .i_fc_fs               (i_fc_fs),
    .i_fc_fe               (i_fc_fe),
    .i_fc_frame_num        (i_fc_frame_num),
    .i_fc_vc               (i_fc_vc),
    .i_tpg_fs              (i_tpg_fs),
    .i_tpg_fe              (i_tpg_fe),
    .i_tpg_fvc             (i_tpg_fvc),
    .i_bp_valid            (i_bp_valid),
    .i_bp_byte_en          (i_bp_byte_en),
    .i_bp_sol              (i_bp_sol),
    .i_bp_eol              (i_bp_eol),
    .i_bp_vc_tag           (i_bp_vc_tag),
    .i_bp_dt_sel           (i_bp_dt_sel),
    .i_emb_data            (i_emb_data),
    .i_emb_byte_en         (i_emb_byte_en),
    .i_emb_valid           (i_emb_valid),
    .i_emb_sol             (i_emb_sol),
    .i_emb_eol             (i_emb_eol),
    .i_emb_vc              (i_emb_vc),
    .i_capture_drop        (i_capture_drop),
    .i_drop_vc_mask        (i_drop_vc_mask),
    .i_dma_buf_claim       (i_dma_buf_claim),
    .i_dma_buf_index       (i_dma_buf_index),
    .i_meta_ready          (i_meta_ready),
    .o_capture_start       (o_capture_start),
    .o_capture_end         (o_capture_end),
    .o_capture_dropped     (o_capture_dropped),
    .o_ev_fs               (o_ev_fs),
    .o_ev_vc               (o_ev_vc),
    .o_ctx_slot            (o_ctx_slot),
    .o_ctx_orphan          (o_ctx_orphan),
    .o_lead_vc             (o_lead_vc),
    .o_frame_count         (o_frame_count),
    .o_frame_id_buf0       (o_frame_id_buf0),
    .o_frame_id_buf1       (o_frame_id_buf1),
    .o_timestamp           (o_timestamp),
    .o_vc0_line_count      (o_vc0_line_count),
    .o_vc0_byte_count      (o_vc0_byte_count),
    .o_vc0_data_type       (o_vc0_data_type),
    .o_vc0_width_mismatch  (o_vc0_width_mismatch),
    .o_vc0_height_mismatch (o_vc0_height_mismatch),
    .o_emb_word_count_vc0  (o_emb_word_count_vc0),
    .o_emb_line_count_vc0  (o_emb_line_count_vc0),
    .o_emb_extract_val_vc0 (o_emb_extract_val_vc0),
    .o_cur_frame_id_vc0    (o_cur_frame_id_vc0),
    .o_cur_line_vc0        (o_cur_line_vc0),
    .o_vc1_line_count      (o_vc1_line_count),
    .o_vc1_byte_count      (o_vc1_byte_count),
    .o_vc1_data_type       (o_vc1_data_type),
    .o_vc1_width_mismatch  (o_vc1_width_mismatch),
    .o_vc1_height_mismatch (o_vc1_height_mismatch),
    .o_emb_word_count_vc1  (o_emb_word_count_vc1),
    .o_emb_line_count_vc1  (o_emb_line_count_vc1),
    .o_emb_extract_val_vc1 (o_emb_extract_val_vc1),
    .o_cur_frame_id_vc1    (o_cur_frame_id_vc1),
    .o_cur_line_vc1        (o_cur_line_vc1),
    .o_meta_overflow       (o_meta_overflow),
    .o_err_frame_num       (o_err_frame_num),
    .o_err_resolution      (o_err_resolution),
    .o_capture_done        (o_capture_done),
    .o_meta_data           (o_meta_data),
    .o_meta_byte_en        (o_meta_byte_en),
    .o_meta_valid          (o_meta_valid),
    .o_meta_last           (o_meta_last),
    .o_meta_kind           (o_meta_kind),
    .o_meta_vc             (o_meta_vc),
    .o_meta_sof            (o_meta_sof)
  );

  // --------------------------------------------------------------------------------------------
  // Clock, error reporting and small helpers
  // --------------------------------------------------------------------------------------------
  int          cyc = 0;
  bit          started = 1'b0;
  int unsigned err_count;
  int unsigned check_count;
  string       where = "start";

  initial begin
    i_sys_clk = 1'b0;
    forever #(T_SYS_CLK / 2.0) i_sys_clk = ~i_sys_clk;
  end

  always @(posedge i_sys_clk) cyc <= cyc + 1;

  task automatic fail(input string msg);
    err_count++;
    if (err_count <= 30) $error("cycle %0d [%s]: %s", cyc, where, msg);
  endtask

  // An explicit check of a test.
  task automatic expect_eq(input string what, input logic [63:0] got, input logic [63:0] want);
    check_count++;
    if (got !== want) fail($sformatf("%s = %0h, expected %0h", what, got, want));
  endtask

  task automatic expect_true(input string what, input bit cond);
    check_count++;
    if (!cond) fail(what);
  endtask

  // Wait n rising edges, plus 0.5 ns. Tests change inputs at that point.
  task automatic cycles(input int n);
    repeat (n) @(posedge i_sys_clk);
    #0.5;
  endtask

  // Lets combinational outputs follow inputs set in the current cycle.
  task automatic settle();
    #0.1;
  endtask

  function automatic int ones4(input logic [3:0] v);
    return int'(v[0]) + int'(v[1]) + int'(v[2]) + int'(v[3]);
  endfunction

  function automatic int rand_range(input int lo, input int hi);
    return int'($urandom_range(hi, lo));
  endfunction

  function automatic bit rand_pct(input int pct);
    return (rand_range(0, 99) < pct);
  endfunction

  function automatic bit rand_pm(input int per_mille);
    return (rand_range(0, 999) < per_mille);
  endfunction

  function automatic logic [3:0] be_of(input int nbytes);
    case (nbytes)
      0:       return 4'b0000;
      1:       return 4'b0001;
      2:       return 4'b0011;
      3:       return 4'b0111;
      default: return 4'b1111;
    endcase
  endfunction

  // --------------------------------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------------------------------
  // Pulse inputs are cleared 0.2 ns after every rising edge, so a value set by a test lasts one
  // cycle. Fields without their qualifier carry random values: they mean nothing then.
  // meta_ready: mode 0 high, 1 low, 2 random with ready_pct, 3 ready_lo cycles low then ready_hi
  // cycles high.
  int ready_mode = 0;
  int ready_pct  = 50;
  int ready_lo   = 1;
  int ready_hi   = 1;
  int ready_cnt  = 0;

  task automatic idle_inputs();
    logic [31:0] r;
    r = $urandom;
    i_fc_fs         = 1'b0;
    i_fc_fe         = 1'b0;
    i_tpg_fs        = 1'b0;
    i_tpg_fe        = 1'b0;
    i_bp_valid      = 1'b0;
    i_emb_valid     = 1'b0;
    i_capture_drop  = 1'b0;
    i_dma_buf_claim = 1'b0;
    i_bp_sol        = r[0];
    i_bp_eol        = r[1];
    i_bp_byte_en    = r[5:2];
    i_bp_vc_tag     = r[7:6];
    i_bp_dt_sel     = r[8];
    i_emb_sol       = r[9];
    i_emb_eol       = r[10];
    i_emb_byte_en   = r[14:11];
    i_emb_vc        = r[16:15];
    i_fc_vc         = r[18:17];
    i_tpg_fvc       = r[20:19];
    i_drop_vc_mask  = r[22:21];
    i_dma_buf_index = r[23];
    i_fc_frame_num  = r[31:16];
    i_emb_data      = $urandom;
    case (ready_mode)
      0: i_meta_ready = 1'b1;
      1: i_meta_ready = 1'b0;
      2: i_meta_ready = rand_pct(ready_pct);
      default: begin
        i_meta_ready = (ready_cnt >= ready_lo);
        ready_cnt    = (ready_cnt + 1 >= ready_lo + ready_hi) ? 0 : ready_cnt + 1;
      end
    endcase
  endtask

  always @(posedge i_sys_clk) begin
    #0.2;
    idle_inputs();
  end

  // Frame events. Each fs setter also arms the expected meta_sof of the channel.
  logic [15:0] fnum_next [0:1] = '{16'd1, 16'd1};
  bit          exp_sof   [0:1] = '{1'b0, 1'b0};

  task automatic set_sfsn(input bit vc, input logic [15:0] fn);
    i_fc_fs        = 1'b1;
    i_fc_vc        = {1'b0, vc};
    i_fc_frame_num = fn;
    fnum_next[vc]  = fn + 16'd1;
    exp_sof[vc]    = 1'b1;
  endtask
  task automatic set_sfs(input bit vc);
    set_sfsn(vc, fnum_next[vc]);
  endtask
  task automatic set_sfe(input bit vc);
    i_fc_fe = 1'b1;
    i_fc_vc = {1'b0, vc};
  endtask
  task automatic set_tfs(input bit vc);
    i_tpg_fs    = 1'b1;
    i_tpg_fvc   = {1'b0, vc};
    exp_sof[vc] = 1'b1;
  endtask
  task automatic set_tfe(input bit vc);
    i_tpg_fe  = 1'b1;
    i_tpg_fvc = {1'b0, vc};
  endtask
  task automatic set_drop(input logic [1:0] mask);
    i_capture_drop = 1'b1;
    i_drop_vc_mask = mask;
  endtask
  task automatic set_claim(input bit idx);
    i_dma_buf_claim = 1'b1;
    i_dma_buf_index = idx;
  endtask

  task automatic do_sfs(input bit vc);
    set_sfs(vc);
    cycles(1);
  endtask
  task automatic do_sfsn(input bit vc, input logic [15:0] fn);
    set_sfsn(vc, fn);
    cycles(1);
  endtask
  task automatic do_sfe(input bit vc);
    set_sfe(vc);
    cycles(1);
  endtask
  task automatic do_tfs(input bit vc);
    set_tfs(vc);
    cycles(1);
  endtask
  task automatic do_tfe(input bit vc);
    set_tfe(vc);
    cycles(1);
  endtask
  task automatic do_drop(input logic [1:0] mask);
    set_drop(mask);
    cycles(1);
  endtask
  task automatic soft_reset_pulse(input int n = 1);
    i_soft_reset = 1'b1;
    cycles(n);
    i_soft_reset = 1'b0;
  endtask

  // Image lines of nbytes; only the final beat is partial. no_eol leaves the line cut (use a
  // multiple of four then). gap_pct inserts idle cycles between beats.
  task automatic bp_line(input bit vc, input int nbytes, input bit raw10, input int gap_pct = 0,
                         input bit no_eol = 1'b0);
    int left;
    int k;
    bit first;
    left  = nbytes;
    first = 1'b1;
    do begin
      while ((gap_pct != 0) && rand_pct(gap_pct)) cycles(1);
      k = (left >= 4) ? 4 : left;
      i_bp_valid   = 1'b1;
      i_bp_vc_tag  = {1'b0, vc};
      i_bp_dt_sel  = raw10;
      i_bp_byte_en = be_of(k);
      i_bp_sol     = first;
      i_bp_eol     = (left <= 4) && !no_eol;
      cycles(1);
      left  = left - k;
      first = 1'b0;
    end while (left > 0);
  endtask

  task automatic bp_lines(input bit vc, input int n, input int nbytes, input bit raw10);
    for (int i = 0; i < n; i++) bp_line(vc, nbytes, raw10);
  endtask

  // Embedded bytes are a fixed function of channel, line salt and byte position, so a test can
  // compute the extracted value it expects.
  function automatic logic [7:0] emb_byte(input bit vc, input int pos, input int salt);
    int v;
    v = salt * 41 + pos * 13 + (pos >> 8) * 7 + (vc ? 97 : 29);
    return 8'(v) | 8'h01;
  endfunction

  // Expected value of a field: len bytes from off of a line of line_bytes, first byte in 7:0.
  function automatic logic [31:0] emb_field(input bit vc, input int off, input int len,
                                            input int line_bytes, input int salt);
    logic [31:0] v;
    int          n;
    v = '0;
    n = (len > 4) ? 4 : len;
    for (int i = 0; i < n; i++) begin
      if (off + i < line_bytes) v[8*i +: 8] = emb_byte(vc, off + i, salt);
    end
    return v;
  endfunction

  // Extraction descriptor k: len 2:0, byte_offset 23:8, line_idx 31:24.
  task automatic set_ext(input int k, input int line_idx, input int off, input int len);
    if (k < NP) i_emb_extract_cfg[k] = {8'(line_idx), 16'(off), 5'd0, 3'(len)};
  endtask

  // One stream word.
  typedef struct packed {
    logic [31:0] data;
    logic [3:0]  be;
    logic        last;
    logic        kind;
    logic [1:0]  vc;
    logic        sof;
  } mword_t;

  // Payload words sent, in order, for comparison with the words accepted at meta_*.
  mword_t sent_q[$];

  // One embedded line of nbytes >= 1; unused byte lanes of the final beat carry noise.
  task automatic emb_line(input bit vc, input int nbytes, input int salt, input int gap_pct = 0,
                          input bit no_eol = 1'b0, input int gap_cycles = 0);
    int          left;
    int          pos;
    int          k;
    logic [31:0] d;
    left = nbytes;
    pos  = 0;
    do begin
      while ((gap_pct != 0) && rand_pct(gap_pct)) cycles(1);
      if (gap_cycles != 0) cycles(gap_cycles);
      k = (left >= 4) ? 4 : left;
      d = $urandom;
      for (int i = 0; i < k; i++) d[8*i +: 8] = emb_byte(vc, pos + i, salt);
      i_emb_valid   = 1'b1;
      i_emb_data    = d;
      i_emb_byte_en = be_of(k);
      i_emb_vc      = {1'b0, vc};
      i_emb_sol     = (pos == 0);
      i_emb_eol     = (left <= 4) && !no_eol;
      sent_q.push_back('{data: d, be: be_of(k), last: (left <= 4) && !no_eol, kind: 1'b1,
                         vc: {1'b0, vc}, sof: 1'b0});
      cycles(1);
      pos  = pos + k;
      left = left - k;
    end while (left > 0);
  endtask

  // --------------------------------------------------------------------------------------------
  // Reference model
  // --------------------------------------------------------------------------------------------
  // Record bank and published channel registers of one channel.
  typedef struct packed {
    logic [15:0]         lines;
    logic [15:0]         bytes;
    logic [5:0]          dt;
    logic                wm;
    logic                hm;
    logic [15:0]         ewords;
    logic [7:0]          elines;
    logic [NP-1:0][31:0] ext;
    logic                fnum_err;
    logic                ovf;
  } bank_t;

  // Record queue entry.
  typedef struct packed {
    logic vc;
    logic slot;
    logic aborted;
  } qent_t;

  localparam int FSM_IDLE = 0;
  localparam int FSM_REC  = 1;
  localparam int FSM_POP  = 2;

  // Channel contexts and capture slots.
  logic [1:0]          m_active, m_orphan, m_slot, m_fnum_err;
  logic [15:0]         m_last_fnum [0:1];
  logic [1:0]          m_open, m_wait, m_dropped;
  logic                m_last_alloc;
  logic [13:0]         m_slot_fid [0:1];
  logic [63:0]         m_slot_ts  [0:1];
  logic [31:0]         m_slot_rid [0:1];
  // Identity: frame_id counter, timestamp with 64 integer and 24 fractional bits, FRAME_ID.
  logic [13:0]         m_fid_cnt;
  logic [87:0]         m_ts;
  logic [13:0]         m_fid_buf [0:1];
  // Geometry and resolution, per channel.
  logic [15:0]         m_acc [0:1];
  logic [15:0]         m_blast [0:1];
  logic [15:0]         m_lines [0:1];
  logic [5:0]          m_dt [0:1];
  logic [15:0]         m_exp_w [0:1];
  logic [15:0]         m_exp_h [0:1];
  logic [1:0]          m_wmm;
  // Embedded data, per channel.
  logic [15:0]         m_ewords [0:1];
  logic [7:0]          m_elines [0:1];
  logic [15:0]         m_eptr [0:1];
  logic [NP-1:0][31:0] m_ext [0:1];
  logic [1:0]          m_ovf, m_sofp;
  // Context Latch.
  bank_t               m_bank [0:1];
  bank_t               m_pub [0:1];
  logic [31:0]         m_fcount;
  logic [63:0]         m_pts;
  // Record Builder.
  qent_t               m_q[$];
  int                  m_fsm;
  int                  m_widx;
  mword_t              m_fifo[$];
  logic                m_movf;
  // Input history.
  logic                m_ce_d, m_sel_d;

  // Values of the current cycle, from the inputs and the model state.
  logic        x_rx, x_dual, x_ev_fs, x_ev_fe, x_lead, x_take, x_fs, x_fe, x_ch, x_lead_fs;
  logic [1:0]  x_ev_vc, x_mark, x_elig, x_end, x_cdrop, x_pend, x_free, x_start;
  logic        x_first_low, x_done, x_alloc, x_alloc_ok, x_close, x_sync, x_cslot, x_abort;
  logic        x_hmm, x_queued, x_efn, x_eres, x_lag_join, x_fw, x_full, x_grant, x_meta_valid;
  mword_t      x_meta;
  logic [13:0] x_cur_fid [0:1];

  task automatic model_reset(input bit keep_counters);
    m_active     = 2'b00;
    m_orphan     = 2'b11;
    m_slot       = 2'b00;
    m_fnum_err   = 2'b00;
    m_open       = 2'b00;
    m_wait       = 2'b00;
    m_dropped    = 2'b00;
    m_last_alloc = 1'b0;
    m_wmm        = 2'b00;
    m_ovf        = 2'b00;
    m_sofp       = 2'b00;
    m_fcount     = '0;
    m_pts        = '0;
    m_movf       = 1'b0;
    m_fsm        = FSM_IDLE;
    m_widx       = 0;
    m_q.delete();
    m_fifo.delete();
    for (int c = 0; c < 2; c++) begin
      m_last_fnum[c] = '0;
      m_slot_fid[c]  = '0;
      m_slot_ts[c]   = '0;
      m_slot_rid[c]  = '0;
      m_fid_buf[c]   = '0;
      m_acc[c]       = '0;
      m_blast[c]     = '0;
      m_lines[c]     = '0;
      m_dt[c]        = '0;
      m_exp_w[c]     = '0;
      m_exp_h[c]     = '0;
      m_ewords[c]    = '0;
      m_elines[c]    = '0;
      m_eptr[c]      = '0;
      m_ext[c]       = '0;
      m_bank[c]      = '0;
      m_pub[c]       = '0;
    end
    if (!keep_counters) begin
      m_fid_cnt = '0;
      m_ts      = '0;
      m_ce_d    = 1'b0;
      m_sel_d   = 1'b0;
    end
  endtask

  task automatic model_eval();
    bit held;
    // Event source selection.
    x_rx    = !i_active_sel;
    x_dual  = i_active_sel ? i_tpg_dual_vc_en : 1'b1;
    x_ev_fs = i_active_sel ? i_tpg_fs : i_fc_fs;
    x_ev_fe = i_active_sel ? i_tpg_fe : i_fc_fe;
    x_ev_vc = i_active_sel ? i_tpg_fvc : i_fc_vc;
    x_lead  = i_active_sel ? 1'b0 : i_vc_exposure_map;
    // Events are taken on VC0/VC1, outside soft_reset and while core_enable is high.
    x_take    = i_core_enable && !i_soft_reset && !x_ev_vc[1];
    x_fs      = x_ev_fs && x_take;
    x_fe      = x_ev_fe && x_take;
    x_ch      = x_ev_vc[0];
    x_lead_fs = x_fs && (x_ch == x_lead);
    // capture_drop acts on the attachments held before the frame event of the cycle.
    x_mark = 2'b00;
    if (i_capture_drop && !i_soft_reset) begin
      for (int c = 0; c < 2; c++) begin
        if (i_drop_vc_mask[c]) begin
          if (!m_orphan[c] && m_open[m_slot[c]]) x_mark[m_slot[c]] = 1'b1;
          else if (m_open[m_last_alloc])         x_mark[m_last_alloc] = 1'b1;
          else if (m_open[!m_last_alloc])        x_mark[!m_last_alloc] = 1'b1;
        end
      end
    end
    // A slot ends when it waits for no lag and no channel of it has a frame open.
    for (int k = 0; k < 2; k++) begin
      held = 1'b0;
      for (int c = 0; c < 2; c++) begin
        if (m_active[c] && !m_orphan[c] && (m_slot[c] == 1'(k))) held = 1'b1;
      end
      x_elig[k] = m_open[k] && !m_wait[k] && !held;
    end
    x_first_low = !i_core_enable && m_ce_d;
    x_end       = 2'b00;
    if (!i_soft_reset && !x_first_low) begin
      if (x_elig[0])      x_end = 2'b01;
      else if (x_elig[1]) x_end = 2'b10;
    end
    x_cdrop = m_dropped | x_mark;
    x_done  = |(x_end & ~x_cdrop);
    // Allocation: a slot is free while it is closed and no queued record names it.
    x_pend = 2'b00;
    foreach (m_q[i]) x_pend[m_q[i].slot] = 1'b1;
    x_free     = ~m_open & ~x_pend;
    x_alloc    = x_free[0] ? 1'b0 : 1'b1;
    x_alloc_ok = x_lead_fs && (x_free != 2'b00);
    x_start    = 2'b00;
    if (x_alloc_ok) x_start[x_alloc] = 1'b1;
    // Close at Frame End, and at Frame Start as a ctx_sync_close, of a channel with a frame open.
    x_close  = (x_fs || x_fe) && m_active[x_ch];
    x_sync   = x_fs && m_active[x_ch];
    x_cslot  = m_slot[x_ch];
    x_abort  = m_dropped[x_cslot] || x_mark[x_cslot];
    x_hmm    = x_rx && (m_lines[x_ch] != m_exp_h[x_ch]);
    x_queued = 1'b0;
    foreach (m_q[i]) if (m_q[i].vc == x_ch) x_queued = 1'b1;
    x_efn  = x_fs && x_rx && i_fnum_check_en && (i_fc_frame_num > 16'd1) &&
             (i_fc_frame_num != 16'(m_last_fnum[x_ch] + 16'd1));
    x_eres = x_close && !x_sync && x_rx && !m_orphan[x_ch] && !x_abort && (m_wmm[x_ch] || x_hmm);
    x_lag_join = x_dual && m_wait[m_last_alloc];
    // Record Builder: payload first, record words only into a FIFO that has space.
    x_fw         = i_emb_valid && x_rx && i_metadata_capture_en && !i_emb_vc[1];
    x_full       = (m_fifo.size() == DEPTH);
    x_grant      = (m_fsm == FSM_REC) && !x_fw && !x_full;
    x_meta_valid = (m_fifo.size() != 0);
    x_meta       = x_meta_valid ? m_fifo[0] : '0;
    for (int c = 0; c < 2; c++) x_cur_fid[c] = m_orphan[c] ? 14'd0 : m_slot_fid[m_slot[c]];
  endtask

  // Word idx of the record at the head of the queue.
  function automatic logic [31:0] rec_word(input int idx);
    qent_t      h;
    bank_t      b;
    logic [4:0] es;
    h  = m_q[0];
    b  = m_bank[h.vc];
    es = {b.ovf, b.hm, b.wm, b.fnum_err, h.aborted};
    case (idx)
      0:       return {11'd0, es, 6'd0, 1'b0, h.vc, 8'h01};
      1:       return {18'd0, m_slot_fid[h.slot]};
      2:       return m_slot_ts[h.slot][31:0];
      3:       return m_slot_ts[h.slot][63:32];
      4:       return m_slot_rid[h.slot];
      5:       return {b.lines, b.bytes};
      6:       return {2'd0, b.dt, b.elines, b.ewords};
      default: return b.ext[idx - 7];
    endcase
  endfunction

  task automatic cmp(input string what, input logic [63:0] got, input logic [63:0] want);
    if (got !== want) fail($sformatf("%s = %0h, model expects %0h", what, got, want));
  endtask

  task automatic model_check();
    check_count++;
    cmp("capture_start", o_capture_start, x_start);
    cmp("capture_end", o_capture_end, x_end);
    cmp("capture_dropped (with capture_end)", o_capture_dropped & x_end, x_cdrop & x_end);
    cmp("capture_done", o_capture_done, x_done);
    cmp("ev_fs", o_ev_fs, x_ev_fs);
    cmp("ev_vc", o_ev_vc, x_ev_vc);
    cmp("lead_vc", o_lead_vc, x_lead);
    cmp("ctx_orphan", o_ctx_orphan, m_orphan);
    cmp("ctx_slot (joined channels)", o_ctx_slot & ~m_orphan, m_slot & ~m_orphan);
    cmp("frame_count", o_frame_count, m_fcount);
    cmp("frame_id_buf0", o_frame_id_buf0, m_fid_buf[0]);
    cmp("frame_id_buf1", o_frame_id_buf1, m_fid_buf[1]);
    cmp("timestamp", o_timestamp, m_pts);
    cmp("vc0_line_count", o_vc0_line_count, m_pub[0].lines);
    cmp("vc0_byte_count", o_vc0_byte_count, m_pub[0].bytes);
    cmp("vc0_data_type", o_vc0_data_type, m_pub[0].dt);
    cmp("vc0_width_mismatch", o_vc0_width_mismatch, m_pub[0].wm);
    cmp("vc0_height_mismatch", o_vc0_height_mismatch, m_pub[0].hm);
    cmp("emb_word_count_vc0", o_emb_word_count_vc0, m_pub[0].ewords);
    cmp("emb_line_count_vc0", o_emb_line_count_vc0, m_pub[0].elines);
    cmp("emb_extract_val_vc0", o_emb_extract_val_vc0, m_pub[0].ext);
    cmp("vc1_line_count", o_vc1_line_count, m_pub[1].lines);
    cmp("vc1_byte_count", o_vc1_byte_count, m_pub[1].bytes);
    cmp("vc1_data_type", o_vc1_data_type, m_pub[1].dt);
    cmp("vc1_width_mismatch", o_vc1_width_mismatch, m_pub[1].wm);
    cmp("vc1_height_mismatch", o_vc1_height_mismatch, m_pub[1].hm);
    cmp("emb_word_count_vc1", o_emb_word_count_vc1, m_pub[1].ewords);
    cmp("emb_line_count_vc1", o_emb_line_count_vc1, m_pub[1].elines);
    cmp("emb_extract_val_vc1", o_emb_extract_val_vc1, m_pub[1].ext);
    cmp("cur_frame_id_vc0", o_cur_frame_id_vc0, x_cur_fid[0]);
    cmp("cur_frame_id_vc1", o_cur_frame_id_vc1, x_cur_fid[1]);
    cmp("cur_line_vc0", o_cur_line_vc0, m_lines[0]);
    cmp("cur_line_vc1", o_cur_line_vc1, m_lines[1]);
    cmp("meta_overflow", o_meta_overflow, m_movf);
    cmp("err_frame_num", o_err_frame_num, x_efn);
    cmp("err_resolution", o_err_resolution, x_eres);
    cmp("meta_valid", o_meta_valid, x_meta_valid);
    cmp("meta_data", o_meta_data, x_meta.data);
    cmp("meta_byte_en", o_meta_byte_en, x_meta.be);
    cmp("meta_last", o_meta_last, x_meta.last);
    cmp("meta_kind", o_meta_kind, x_meta.kind);
    cmp("meta_vc", o_meta_vc, x_meta.vc);
    cmp("meta_sof", o_meta_sof, x_meta.sof);
  endtask

  task automatic model_update();
    logic [87:0] ts_old;
    bank_t       b;
    qent_t       e;
    bit          push;
    bit          wr;
    bit          rd;
    mword_t      w;
    bit          c;
    logic [15:0] acc;
    logic [31:0] ref_bytes;
    logic [31:0] cfg;
    logic [13:0] nid;
    int          fsm_old;
    int          ptr;
    int          nb;
    int          len;
    int          off;
    int          p;
    ts_old = m_ts;
    m_ts   = m_ts + 88'(PARA_TS_INCR);
    if (i_soft_reset) begin
      model_reset(1'b1);
    end else begin
      // FRAME_ID entry, from the most recently allocated slot before this edge.
      if (i_dma_buf_claim) m_fid_buf[i_dma_buf_index] = m_slot_fid[m_last_alloc];
      m_dropped = m_dropped | x_mark;
      if (i_active_sel != m_sel_d) m_wait = 2'b00;

      // capture_end: frame_count and the published timestamp.
      for (int k = 0; k < 2; k++) begin
        if (x_end[k]) begin
          m_open[k] = 1'b0;
          m_fcount  = m_fcount + 32'd1;
          if (!x_cdrop[k]) m_pts = m_slot_ts[k];
        end
      end

      // Close: record bank, queue entry, publication.
      push = 1'b0;
      if (x_close) begin
        if (!m_orphan[x_ch]) begin
          b.lines    = m_lines[x_ch];
          b.bytes    = m_blast[x_ch];
          b.dt       = m_dt[x_ch];
          b.wm       = m_wmm[x_ch];
          b.hm       = x_hmm;
          b.ewords   = m_ewords[x_ch];
          b.elines   = m_elines[x_ch];
          b.ext      = m_ext[x_ch];
          b.fnum_err = m_fnum_err[x_ch];
          b.ovf      = m_ovf[x_ch];
          if (x_queued) begin
            m_movf = 1'b1;
          end else begin
            m_bank[x_ch] = b;
            push         = 1'b1;
            e.vc         = x_ch;
            e.slot       = x_cslot;
            e.aborted    = x_abort;
          end
          if (x_rx && !x_abort) m_pub[x_ch] = b;
        end
        if (x_fe) m_active[x_ch] = 1'b0;
      end

      // Frame Start: ctx_open, frame number, pairing.
      if (x_fs) begin
        m_active[x_ch] = 1'b1;
        m_acc[x_ch]    = '0;
        m_blast[x_ch]  = '0;
        m_lines[x_ch]  = '0;
        m_dt[x_ch]     = '0;
        m_wmm[x_ch]    = 1'b0;
        m_exp_w[x_ch]  = i_expected_width;
        m_exp_h[x_ch]  = i_expected_height;
        m_ewords[x_ch] = '0;
        m_elines[x_ch] = '0;
        m_eptr[x_ch]   = '0;
        m_ovf[x_ch]    = 1'b0;
        m_sofp[x_ch]   = 1'b1;
        if (x_rx) begin
          m_last_fnum[x_ch] = i_fc_frame_num;
          m_fnum_err[x_ch]  = x_efn;
        end else begin
          m_fnum_err[x_ch]  = 1'b0;
        end
        if (x_lead_fs) begin
          m_wait = 2'b00;
          if (x_alloc_ok) begin
            nid                  = (m_fid_cnt == 14'h3FFF) ? 14'd1 : m_fid_cnt + 14'd1;
            m_open[x_alloc]      = 1'b1;
            m_wait[x_alloc]      = x_dual;
            m_dropped[x_alloc]   = 1'b0;
            m_last_alloc         = x_alloc;
            m_slot[x_ch]         = x_alloc;
            m_orphan[x_ch]       = 1'b0;
            m_fid_cnt            = nid;
            m_slot_fid[x_alloc]  = nid;
            m_slot_ts[x_alloc]   = ts_old[87:24];
            m_slot_rid[x_alloc]  = i_metadata_record_id;
          end else begin
            m_orphan[x_ch] = 1'b1;
          end
        end else if (x_lag_join) begin
          m_slot[x_ch]         = m_last_alloc;
          m_orphan[x_ch]       = 1'b0;
          m_wait[m_last_alloc] = 1'b0;
        end else begin
          m_orphan[x_ch] = 1'b1;
        end
      end

      // core_enable low: both channels detached, every open slot marked.
      if (!i_core_enable) begin
        m_active  = 2'b00;
        m_orphan  = 2'b11;
        m_dropped = m_dropped | m_open;
        m_wait    = 2'b00;
      end

      // Geometry Accumulator and Resolution Checker.
      if (i_bp_valid && x_rx && !i_bp_vc_tag[1]) begin
        c        = i_bp_vc_tag[0];
        acc      = i_bp_sol ? 16'd0 : m_acc[c];
        acc      = 16'(acc + 16'(ones4(i_bp_byte_en)));
        m_acc[c] = acc;
        if (i_bp_eol) begin
          m_blast[c] = acc;
          m_lines[c] = m_lines[c] + 16'd1;
          m_dt[c]    = i_bp_dt_sel ? DT_RAW10 : DT_RAW12;
          ref_bytes  = i_bp_dt_sel ? ((32'(m_exp_w[c]) * 32'd5) >> 2)
                                   : ((32'(m_exp_w[c]) * 32'd3) >> 1);
          if (32'(acc) != ref_bytes) m_wmm[c] = 1'b1;
        end
      end

      // Embedded Data Path, then the word written into the staging FIFO.
      wr = 1'b0;
      if (x_fw) begin
        c           = i_emb_vc[0];
        m_ewords[c] = m_ewords[c] + 16'd1;
        ptr         = i_emb_sol ? 0 : int'(m_eptr[c]);
        nb          = ones4(i_emb_byte_en);
        if (i_emb_extract_en) begin
          for (int k = 0; k < NX; k++) begin
            cfg = i_emb_extract_cfg[k];
            len = (int'(cfg[2:0]) > 4) ? 4 : int'(cfg[2:0]);
            off = int'(cfg[23:8]);
            if ((len != 0) && (m_elines[c] == cfg[31:24])) begin
              for (int i = 0; i < nb; i++) begin
                p = ptr + i;
                if ((p >= off) && (p < off + len)) begin
                  if (p == off) m_ext[c][k] = '0;
                  m_ext[c][k][8*(p-off) +: 8] = i_emb_data[8*i +: 8];
                end
              end
            end
          end
        end
        m_eptr[c] = 16'(ptr + nb);
        if (i_emb_eol) m_elines[c] = m_elines[c] + 8'd1;
        if (!x_full) begin
          wr        = 1'b1;
          w.data    = i_emb_data;
          w.be      = i_emb_byte_en;
          w.last    = i_emb_eol;
          w.kind    = 1'b1;
          w.vc      = {1'b0, c};
          w.sof     = m_sofp[c];
          m_sofp[c] = 1'b0;
        end else begin
          m_ovf[c] = 1'b1;
          m_movf   = 1'b1;
        end
      end else if (x_grant) begin
        wr     = 1'b1;
        w.data = rec_word(m_widx);
        w.be   = 4'b1111;
        w.last = (m_widx == RW - 1);
        w.kind = 1'b0;
        w.vc   = {1'b0, m_q[0].vc};
        w.sof  = 1'b0;
      end

      // Staging FIFO.
      rd = (m_fifo.size() != 0) && i_meta_ready;
      if (rd) void'(m_fifo.pop_front());
      if (wr) m_fifo.push_back(w);

      // Record FSM.
      fsm_old = m_fsm;
      case (m_fsm)
        FSM_IDLE: begin
          m_widx = 0;
          if (m_q.size() != 0) m_fsm = FSM_REC;
        end
        FSM_REC: begin
          if (x_grant) begin
            if (m_widx == RW - 1) m_fsm = FSM_POP;
            m_widx = m_widx + 1;
          end
        end
        default: begin
          m_widx = 0;
          m_fsm  = (m_q.size() > 1) ? FSM_REC : FSM_IDLE;
        end
      endcase

      // Queue: the pop of ST_POP, then the entry of this cycle.
      if (fsm_old == FSM_POP) void'(m_q.pop_front());
      if (push) m_q.push_back(e);
    end
    m_ce_d  = i_core_enable;
    m_sel_d = i_active_sel;
  endtask

  // Every output other than ev_fs, ev_vc, lead_vc (the source mux), ctx_slot and
  // capture_dropped (meaningful only with their qualifier) is at its reset value.
  task automatic check_reset_outputs();
    if ({o_capture_start, o_capture_end, o_capture_done, o_err_frame_num, o_err_resolution,
         o_frame_count, o_frame_id_buf0, o_frame_id_buf1, o_timestamp,
         o_vc0_line_count, o_vc0_byte_count, o_vc0_data_type, o_vc0_width_mismatch,
         o_vc0_height_mismatch, o_emb_word_count_vc0, o_emb_line_count_vc0, o_emb_extract_val_vc0,
         o_cur_frame_id_vc0, o_cur_line_vc0,
         o_vc1_line_count, o_vc1_byte_count, o_vc1_data_type, o_vc1_width_mismatch,
         o_vc1_height_mismatch, o_emb_word_count_vc1, o_emb_line_count_vc1, o_emb_extract_val_vc1,
         o_cur_frame_id_vc1, o_cur_line_vc1,
         o_meta_overflow, o_meta_data, o_meta_byte_en, o_meta_valid, o_meta_last, o_meta_kind,
         o_meta_vc, o_meta_sof} !== '0)
      fail("an output is not at its reset value while resetn is low");
    if (o_ctx_orphan !== 2'b11) fail("ctx_orphan is not 11 while resetn is low");
  endtask

  // Reference model and checker, at each rising edge.
  always @(posedge i_sys_clk) begin
    if (started) begin
      if (i_resetn !== 1'b1) begin
        check_reset_outputs();
        model_reset(1'b0);
      end else begin
        model_eval();
        model_check();
        model_update();
      end
    end
  end

  // resetn is asynchronous: the outputs go to their reset value without waiting for an edge.
  always @(negedge i_resetn) begin
    if (started) begin
      model_reset(1'b0);
      #0.2;
      check_reset_outputs();
    end
  end

  // --------------------------------------------------------------------------------------------
  // Port monitor: records and payload accepted at meta_*
  // --------------------------------------------------------------------------------------------
  typedef struct packed {
    logic [1:0]       vc;
    logic [8:0][31:0] w;
  } rec_t;

  rec_t   rec_q[$];
  mword_t pay_q[$];
  rec_t   mon_rec;
  int     mon_idx      = 0;
  int     n_interleave = 0;
  int     n_start      = 0;
  int     n_end        = 0;
  int     n_done       = 0;
  int     n_err_fnum   = 0;
  int     n_err_res    = 0;
  logic [1:0] last_start = 2'b00;

  always @(posedge i_sys_clk) begin
    mword_t w;
    if (started) begin
      w = '{data: o_meta_data, be: o_meta_byte_en, last: o_meta_last, kind: o_meta_kind,
            vc: o_meta_vc, sof: o_meta_sof};
      if (i_resetn !== 1'b1 || i_soft_reset === 1'b1) begin
        mon_idx = 0;
      end else if (o_meta_valid === 1'b1 && i_meta_ready === 1'b1) begin
        if (w.kind === 1'b0) begin
          if (mon_idx == 0) mon_rec.vc = w.vc;
          else if (mon_rec.vc !== w.vc) fail("meta_vc changed inside a record");
          if (mon_idx < 9) mon_rec.w[mon_idx] = w.data;
          mon_idx++;
          if (w.last !== ((mon_idx == RW) ? 1'b1 : 1'b0))
            fail($sformatf("meta_last = %b on record word %0d", w.last, mon_idx - 1));
          if (mon_idx == RW) begin
            rec_q.push_back(mon_rec);
            mon_idx = 0;
          end
        end else begin
          if (mon_idx != 0) n_interleave++;
          pay_q.push_back(w);
        end
      end
      if (i_resetn === 1'b1) begin
        if (o_capture_start !== 2'b00) n_start++;
        if (o_capture_end !== 2'b00) n_end++;
        if (o_capture_done === 1'b1) n_done++;
        if (o_err_frame_num === 1'b1) n_err_fnum++;
        if (o_err_resolution === 1'b1) n_err_res++;
      end
      last_start = o_capture_start;
    end
  end

  // --------------------------------------------------------------------------------------------
  // Protocol checkers (independent of the model)
  // --------------------------------------------------------------------------------------------
  bit          p_valid = 1'b0;
  bit          relax_vc = 1'b0;  // a test sends frame events on VC2/VC3 on purpose
  logic        p_soft, p_evfs, p_ce, p_done, p_claim, p_claim_idx, p_stall, p_movf;
  logic [1:0]  p_start, p_end, p_orphan, p_slot;
  logic [31:0] p_fcount;
  logic [63:0] p_ts;
  logic [13:0] p_fid0, p_fid1;
  logic [15:0] p_line0, p_line1;
  logic [1:0]  p_line_ev;
  mword_t      p_word;

  always @(posedge i_sys_clk) begin
    mword_t w;
    logic   sel_fs, sel_fe;
    logic [1:0] sel_vc;
    if (started && i_resetn === 1'b1) begin
      w = '{data: o_meta_data, be: o_meta_byte_en, last: o_meta_last, kind: o_meta_kind,
            vc: o_meta_vc, sof: o_meta_sof};
      if ($isunknown({o_capture_start, o_capture_end, o_capture_dropped, o_ev_fs, o_ev_vc,
                      o_ctx_slot, o_ctx_orphan, o_lead_vc, o_frame_count, o_frame_id_buf0,
                      o_frame_id_buf1, o_timestamp, o_vc0_line_count, o_vc0_byte_count,
                      o_vc0_data_type, o_vc0_width_mismatch, o_vc0_height_mismatch,
                      o_emb_word_count_vc0, o_emb_line_count_vc0, o_emb_extract_val_vc0,
                      o_cur_frame_id_vc0, o_cur_line_vc0, o_vc1_line_count, o_vc1_byte_count,
                      o_vc1_data_type, o_vc1_width_mismatch, o_vc1_height_mismatch,
                      o_emb_word_count_vc1, o_emb_line_count_vc1, o_emb_extract_val_vc1,
                      o_cur_frame_id_vc1, o_cur_line_vc1, o_meta_overflow, o_err_frame_num,
                      o_err_resolution, o_capture_done, o_meta_data, o_meta_byte_en,
                      o_meta_valid, o_meta_last, o_meta_kind, o_meta_vc, o_meta_sof}))
        fail("X or Z on an output");
      // Event source selection.
      sel_fs = i_active_sel ? i_tpg_fs : i_fc_fs;
      sel_fe = i_active_sel ? i_tpg_fe : i_fc_fe;
      sel_vc = i_active_sel ? i_tpg_fvc : i_fc_vc;
      if ({o_ev_fs, o_ev_vc} !== {sel_fs, sel_vc})
        fail("ev_fs/ev_vc differ from the selected source");
      if (o_lead_vc !== (i_active_sel ? 1'b0 : i_vc_exposure_map))
        fail("lead_vc differs from its selection");
      // Capture events.
      if (o_capture_end === 2'b11) fail("capture_end on both slots in one cycle");
      if (o_capture_start === 2'b11) fail("capture_start on both slots in one cycle");
      if ((o_capture_start !== 2'b00) && !(o_ev_fs && (o_ev_vc === {1'b0, o_lead_vc})))
        fail("capture_start without a lead Frame Start");
      if (o_capture_done !== |(o_capture_end & ~o_capture_dropped))
        fail("capture_done differs from capture_end of a capture not dropped");
      if (o_ctx_orphan[0] && (o_cur_frame_id_vc0 !== 14'd0))
        fail("cur_frame_id_vc0 not 0 while orphan");
      if (o_ctx_orphan[1] && (o_cur_frame_id_vc1 !== 14'd0))
        fail("cur_frame_id_vc1 not 0 while orphan");
      if (o_err_frame_num && !(!i_active_sel && i_fc_fs && i_fnum_check_en))
        fail("err_frame_num without a sensor Frame Start and fnum_check_en");
      if (o_err_resolution && !(!i_active_sel && i_fc_fe))
        fail("err_resolution without a sensor Frame End");
      // meta_* stream.
      if (o_meta_valid === 1'b0 && w !== '0) fail("meta fields not 0 while meta_valid is low");
      if (o_meta_valid === 1'b1 && w.kind === 1'b0 && (w.be !== 4'b1111 || w.sof !== 1'b0))
        fail("record word with a partial byte enable or meta_sof");
      if (o_meta_valid === 1'b1 && w.kind === 1'b1 && w.last !== 1'b1 && w.be !== 4'b1111)
        fail("partial payload word that is not the last of its line");
      if (o_meta_valid === 1'b1 && !(w.be inside {4'b0000, 4'b0001, 4'b0011, 4'b0111, 4'b1111}))
        fail("meta_byte_en not contiguous from bit 0");
      if (o_meta_valid === 1'b1 && w.vc[1]) fail("meta_vc outside VC0/VC1");
      if (p_valid) begin
        // Pulses of one capture event last one cycle.
        if ((p_start & o_capture_start) != 2'b00)
          fail("capture_start of a slot in two cycles in a row");
        if ((p_end & o_capture_end) != 2'b00) fail("capture_end of a slot in two cycles in a row");
        if (p_stall && !p_soft) begin
          if (o_meta_valid !== 1'b1) fail("meta_valid fell while meta_ready was low");
          else if (w !== p_word) fail("meta word changed while meta_ready was low");
        end
        if (!p_soft) begin
          if (p_movf && !o_meta_overflow) fail("meta_overflow cleared without a reset");
          if (o_frame_count !== p_fcount + ((p_end != 2'b00) ? 32'd1 : 32'd0))
            fail("frame_count changed other than by one per capture_end");
          if (!p_done && o_timestamp !== p_ts) fail("timestamp changed without capture_done");
          if (!(p_claim && !p_claim_idx) && o_frame_id_buf0 !== p_fid0)
            fail("frame_id_buf0 changed without dma_buf_claim");
          if (!(p_claim && p_claim_idx) && o_frame_id_buf1 !== p_fid1)
            fail("frame_id_buf1 changed without dma_buf_claim");
          if (!p_evfs && p_ce && o_ctx_orphan !== p_orphan)
            fail("ctx_orphan changed without a Frame Start or core_enable low");
          if (!p_evfs && ((o_ctx_slot ^ p_slot) & ~o_ctx_orphan) != 2'b00)
            fail("ctx_slot changed without a Frame Start");
          if (!p_evfs && !p_line_ev[0] && o_cur_line_vc0 !== p_line0)
            fail("cur_line_vc0 changed without bp_eol or a Frame Start");
          if (!p_evfs && !p_line_ev[1] && o_cur_line_vc1 !== p_line1)
            fail("cur_line_vc1 changed without bp_eol or a Frame Start");
        end
      end
      // Stimulus rules.
      if ((i_fc_fs && i_fc_fe) || (i_tpg_fs && i_tpg_fe))
        fail("stimulus: Frame Start and Frame End of one source in one cycle");
      if (i_bp_valid && i_emb_valid) fail("stimulus: bp_valid and emb_valid together");
      if ((i_bp_valid || i_emb_valid) && (sel_fs || sel_fe))
        fail("stimulus: payload beat in a frame event cycle");
      if (i_bp_valid && i_bp_vc_tag[1]) fail("stimulus: bp_vc_tag outside VC0/VC1");
      if (i_emb_valid && i_emb_vc[1]) fail("stimulus: emb_vc outside VC0/VC1");
      if (!relax_vc && (sel_fs || sel_fe) && sel_vc[1])
        fail("stimulus: frame event outside VC0/VC1");
      if (i_emb_valid && !(i_emb_byte_en inside {4'b0001, 4'b0011, 4'b0111, 4'b1111}))
        fail("stimulus: emb_byte_en");
      if (i_bp_valid && !(i_bp_byte_en inside {4'b0000, 4'b0001, 4'b0011, 4'b0111, 4'b1111}))
        fail("stimulus: bp_byte_en");
      p_valid     = 1'b1;
      p_soft      = i_soft_reset;
      p_evfs      = o_ev_fs;
      p_ce        = i_core_enable;
      p_done      = o_capture_done;
      p_claim     = i_dma_buf_claim;
      p_claim_idx = i_dma_buf_index;
      p_stall     = o_meta_valid && !i_meta_ready;
      p_movf      = o_meta_overflow;
      p_start     = o_capture_start;
      p_end       = o_capture_end;
      p_orphan    = o_ctx_orphan;
      p_slot      = o_ctx_slot;
      p_fcount    = o_frame_count;
      p_ts        = o_timestamp;
      p_fid0      = o_frame_id_buf0;
      p_fid1      = o_frame_id_buf1;
      p_line0     = o_cur_line_vc0;
      p_line1     = o_cur_line_vc1;
      p_line_ev   = 2'b00;
      if (i_bp_valid && i_bp_eol && !i_active_sel) p_line_ev[i_bp_vc_tag[0]] = 1'b1;
      p_word      = w;
    end else begin
      p_valid = 1'b0;
    end
  end

  // --------------------------------------------------------------------------------------------
  // Test library
  // --------------------------------------------------------------------------------------------
  // Clock edges since resetn was released; the timestamp of a capture is the integer part of
  // edges x TS_INCR at the capture_start cycle.
  longint unsigned n_edges = 0;
  always @(posedge i_sys_clk or negedge i_resetn) begin
    if (!i_resetn) n_edges <= 0;
    else           n_edges <= n_edges + 1;
  end

  function automatic logic [63:0] exp_ts();
    return 64'((n_edges * longint'(PARA_TS_INCR)) >> 24);
  endfunction

  function automatic logic [15:0] pub_lines(input bit vc);
    return vc ? o_vc1_line_count : o_vc0_line_count;
  endfunction
  function automatic logic [15:0] pub_bytes(input bit vc);
    return vc ? o_vc1_byte_count : o_vc0_byte_count;
  endfunction
  function automatic logic [7:0] pub_elines(input bit vc);
    return vc ? o_emb_line_count_vc1 : o_emb_line_count_vc0;
  endfunction
  function automatic logic [15:0] pub_ewords(input bit vc);
    return vc ? o_emb_word_count_vc1 : o_emb_word_count_vc0;
  endfunction
  function automatic logic [31:0] pub_ext(input bit vc, input int k);
    return vc ? o_emb_extract_val_vc1[k] : o_emb_extract_val_vc0[k];
  endfunction
  function automatic logic [13:0] cur_fid(input bit vc);
    return vc ? o_cur_frame_id_vc1 : o_cur_frame_id_vc0;
  endfunction
  function automatic logic [15:0] cur_line(input bit vc);
    return vc ? o_cur_line_vc1 : o_cur_line_vc0;
  endfunction

  // err_summary bits: aborted, fnum_err, width_mismatch, height_mismatch, meta_overflow.
  function automatic logic [4:0] errs(input bit aborted, input bit fnum, input bit wm,
                                      input bit hm, input bit ovf);
    return {ovf, hm, wm, fnum, aborted};
  endfunction

  // capture_end with its capture_dropped, and capture_done, in the current cycle.
  task automatic expect_end(input logic [1:0] e, input logic [1:0] dr);
    expect_eq("{capture_end, capture_dropped, capture_done}",
              {o_capture_end, o_capture_dropped & e, o_capture_done}, {e, dr & e, |(e & ~dr)});
  endtask

  // Words 0, 1, 5 and 6 of a record.
  task automatic expect_rec(input rec_t r, input bit vc, input logic [4:0] es,
                            input logic [13:0] fid,
                            input int lines, input int bytes, input int ew, input int el,
                            input logic [5:0] dt);
    expect_eq("record meta_vc", r.vc, {1'b0, vc});
    expect_eq("record word 0", r.w[0], {11'd0, es, 6'd0, 1'b0, vc, 8'h01});
    expect_eq("record word 1", r.w[1], {18'd0, fid});
    expect_eq("record word 5", r.w[5], {16'(lines), 16'(bytes)});
    expect_eq("record word 6", r.w[6], {2'd0, dt, 8'(el), 16'(ew)});
  endtask

  // Words 2 to 4 of a record.
  task automatic expect_rec_id(input rec_t r, input logic [63:0] ts, input logic [31:0] rid);
    expect_eq("record words 3, 2", {r.w[3], r.w[2]}, ts);
    expect_eq("record word 4", r.w[4], rid);
  endtask

  task automatic expect_rec_ext(input rec_t r, input int k, input logic [31:0] val);
    if (k < NX) expect_eq($sformatf("record word %0d", 7 + k), r.w[7 + k], val);
  endtask

  task automatic expect_no_rec();
    expect_eq("records emitted", rec_q.size(), 0);
    rec_q.delete();
  endtask

  // Published channel registers.
  task automatic expect_pub(input bit vc, input int lines, input int bytes, input logic [5:0] dt,
                            input bit wm, input bit hm, input int ew, input int el);
    expect_eq($sformatf({"VC%0d {line_count, byte_count, data_type, width_mismatch, ",
                         "height_mismatch, emb_word_count, emb_line_count}"}, vc),
              vc ? {o_vc1_line_count, o_vc1_byte_count, o_vc1_data_type, o_vc1_width_mismatch,
                    o_vc1_height_mismatch, o_emb_word_count_vc1, o_emb_line_count_vc1}
                 : {o_vc0_line_count, o_vc0_byte_count, o_vc0_data_type, o_vc0_width_mismatch,
                    o_vc0_height_mismatch, o_emb_word_count_vc0, o_emb_line_count_vc0},
              {16'(lines), 16'(bytes), dt, wm, hm, 16'(ew), 8'(el)});
  endtask

  task automatic expect_pub_ext(input bit vc, input int k, input logic [31:0] val);
    if (k < NX) expect_eq($sformatf("emb_extract_val_vc%0d[%0d]", vc, k), pub_ext(vc, k), val);
  endtask

  // Payload accepted so far equals the words sent, in order, with meta_sof on the first word
  // of each frame. Only for runs in which no word was lost.
  task automatic expect_payload();
    mword_t a;
    mword_t b;
    expect_eq("payload words accepted", pay_q.size(), sent_q.size());
    while (pay_q.size() != 0 && sent_q.size() != 0) begin
      a     = pay_q.pop_front();
      b     = sent_q.pop_front();
      b.sof = exp_sof[b.vc[0]];
      exp_sof[b.vc[0]] = 1'b0;
      if (a !== b) fail($sformatf("payload word %h, expected %h", a, b));
    end
    pay_q.delete();
    sent_q.delete();
  endtask

  task automatic clear_streams();
    rec_q.delete();
    pay_q.delete();
    sent_q.delete();
  endtask

  // Every status output at its reset value.
  task automatic expect_reset_outputs();
    expect_eq("{capture_start, capture_end, capture_done, err_frame_num, err_resolution}",
              {o_capture_start, o_capture_end, o_capture_done, o_err_frame_num,
               o_err_resolution}, 0);
    expect_eq("ctx_orphan", o_ctx_orphan, 2'b11);
    expect_eq("frame_count", o_frame_count, 0);
    expect_eq("{frame_id_buf0, frame_id_buf1}", {o_frame_id_buf0, o_frame_id_buf1}, 0);
    expect_eq("timestamp", o_timestamp, 0);
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    expect_pub(1'b1, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    expect_eq("{cur_frame_id_vc0, cur_frame_id_vc1}", {o_cur_frame_id_vc0, o_cur_frame_id_vc1}, 0);
    expect_eq("{cur_line_vc0, cur_line_vc1}", {o_cur_line_vc0, o_cur_line_vc1}, 0);
    expect_eq("emb_extract_val_vc0", o_emb_extract_val_vc0, 0);
    expect_eq("emb_extract_val_vc1", o_emb_extract_val_vc1, 0);
    expect_eq("{meta_valid, meta_overflow, meta_data}",
              {o_meta_valid, o_meta_overflow, o_meta_data}, 0);
  endtask

  // Next record accepted at meta_*, waiting for it with a limit.
  task automatic get_rec(output rec_t r);
    int guard;
    guard = 0;
    while (rec_q.size() == 0 && guard < 5000) begin
      cycles(1);
      guard++;
    end
    if (rec_q.size() == 0) begin
      fail("record expected, none was emitted");
      r = '0;
    end else begin
      r = rec_q.pop_front();
    end
  endtask

  // Until the record queue, the Record FSM and the staging FIFO are empty, meta_ready high. The
  // model state only paces the stimulus here.
  task automatic drain();
    int guard;
    ready_mode = 0;
    guard      = 0;
    cycles(1);
    while ((m_q.size() != 0 || m_fsm != FSM_IDLE || m_fifo.size() != 0) && guard < 5000) begin
      cycles(1);
      guard++;
    end
    if (guard >= 5000) fail("record path did not become idle");
    cycles(2);
  endtask

  // Until every capture slot has ended, with a limit.
  task automatic wait_slots_end();
    int guard;
    guard = 0;
    while (m_open != 2'b00 && guard < 5000) begin
      cycles(1);
      guard++;
    end
    if (guard >= 5000) fail("capture slots did not end");
  endtask

  // A clean sensor capture: lead and lag, each with H_DEF good RAW10 lines, both closed.
  task automatic capture_pair(input bit lead = 1'b0);
    do_sfs(lead);
    do_sfs(!lead);
    bp_lines(lead, H_DEF, B10, 1'b1);
    bp_lines(!lead, H_DEF, B10, 1'b1);
    do_sfe(lead);
    do_sfe(!lead);
  endtask

  // --------------------------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------------------------
  // resetn values, and resetn applied asynchronously in the middle of activity.
  task automatic test_reset_values();
    rec_t        r;
    int          ends;
    logic [63:0] ts;
    where = "after power-up resetn";
    settle();
    expect_reset_outputs();
    cycles(3);
    settle();
    expect_reset_outputs();

    where = "activity before resetn";
    do_sfs(1'b0);
    set_claim(1'b0);
    do_sfs(1'b1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    settle();
    expect_eq("frame_count", o_frame_count, 1);
    expect_eq("frame_id_buf0", o_frame_id_buf0, 1);
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);
    rec_q.delete();
    ready_mode = 1;
    cycles(1);
    do_sfs(1'b0);
    emb_line(1'b0, (DEPTH + 1) * 4, 1);
    do_sfe(1'b0);
    do_sfs(1'b0);
    cycles(1);
    do_sfs(1'b1);
    settle();
    expect_eq("meta_overflow before resetn", o_meta_overflow, 1);
    expect_eq("meta_valid before resetn", o_meta_valid, 1);

    where = "resetn between clock edges";
    ends = n_end;
    #1.0;
    i_resetn = 1'b0;
    #0.5;
    expect_reset_outputs();
    ready_mode = 0;
    cycles(3);
    settle();
    where = "resetn held";
    expect_reset_outputs();
    i_resetn = 1'b1;
    cycles(6);
    settle();
    where = "after release: nothing survives";
    expect_reset_outputs();
    expect_eq("capture_end pulses after reset", n_end, ends);
    expect_no_rec();
    clear_streams();

    where = "first capture after resetn: frame_id and time base restart";
    i_metadata_record_id = 32'h1234_5678;
    set_sfs(1'b0);
    settle();
    ts = exp_ts();
    expect_eq("capture_start", o_capture_start, 2'b01);
    cycles(1);
    settle();
    expect_eq("frame_id restarts", cur_fid(1'b0), 1);
    do_sfs(1'b1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    expect_rec_id(r, ts, 32'h1234_5678);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    settle();
    expect_eq("published timestamp", o_timestamp, ts);
  endtask

  // soft_reset in each activity; the timestamp and frame_id counters continue.
  task automatic test_soft_reset();
    rec_t        r;
    logic [63:0] ts;
    int          ends;
    int          fid;
    where = "capture before soft_reset";
    do_sfs(1'b0);
    set_claim(1'b1);
    do_sfs(1'b1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    settle();
    expect_eq("frame_count", o_frame_count, 1);
    expect_eq("frame_id_buf1", o_frame_id_buf1, 1);
    expect_true("timestamp is not 0", o_timestamp != 0);
    fid = 1;

    where = "soft_reset in idle";
    i_soft_reset = 1'b1;
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    i_soft_reset = 1'b0;
    settle();
    expect_reset_outputs();

    where = "frame_id and the time base continue after soft_reset";
    set_sfs(1'b0);
    settle();
    ts = exp_ts();
    cycles(1);
    settle();
    fid++;
    expect_eq("frame_id continues", cur_fid(1'b0), fid);

    where = "soft_reset with a capture open and a line in flight";
    do_sfs(1'b1);
    bp_line(1'b0, B10, 1'b1);
    bp_line(1'b1, B10, 1'b1);
    bp_line(1'b0, 8, 1'b1, 0, 1'b1);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 1);
    ends = n_end;
    i_soft_reset = 1'b1;
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    i_soft_reset = 1'b0;
    settle();
    expect_reset_outputs();
    cycles(4);
    expect_eq("no capture_end for a cleared slot", n_end, ends);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    settle();
    expect_no_rec();
    expect_reset_outputs();

    where = "soft_reset in the capture_end cycle";
    i_expected_height = 16'd0;
    do_sfs(1'b0);
    do_sfs(1'b1);
    fid++;
    do_sfe(1'b0);
    do_sfe(1'b1);
    ends = n_end;
    i_soft_reset = 1'b1;
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    i_soft_reset = 1'b0;
    settle();
    expect_reset_outputs();
    cycles(2);
    expect_eq("capture_end masked", n_end, ends);
    rec_q.delete();

    where = "frame events in a soft_reset cycle are not taken";
    i_fnum_check_en = 1'b1;
    i_soft_reset    = 1'b1;
    set_sfsn(1'b0, 16'd77);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b00);
    expect_eq("ev_fs follows the source", o_ev_fs, 1);
    expect_eq("ev_vc follows the source", o_ev_vc, 0);
    expect_eq("err_frame_num", o_err_frame_num, 0);
    cycles(1);
    i_soft_reset = 1'b0;
    settle();
    expect_eq("ctx_orphan", o_ctx_orphan, 2'b11);
    i_fnum_check_en = 1'b0;
    do_sfs(1'b0);
    settle();
    fid++;
    expect_eq("the ignored Frame Start used no frame_id", cur_fid(1'b0), fid);
    do_sfs(1'b1);
    bp_line(1'b0, 12, 1'b1);
    i_expected_height = 16'd9;
    i_soft_reset      = 1'b1;
    set_sfe(1'b0);
    settle();
    expect_eq("err_resolution in a soft_reset cycle", o_err_resolution, 0);
    cycles(1);
    i_soft_reset = 1'b0;
    cycles(3);
    drain();
    expect_no_rec();

    where = "soft_reset in the middle of a record under a stall";
    i_expected_height = 16'd0;
    ready_mode = 1;
    cycles(1);
    do_sfs(1'b0);
    fid++;
    emb_line(1'b0, (DEPTH + 2) * 4, 2);
    do_sfe(1'b0);
    cycles(2);
    settle();
    expect_eq("meta_overflow", o_meta_overflow, 1);
    expect_eq("meta_valid", o_meta_valid, 1);
    soft_reset_pulse();
    settle();
    expect_reset_outputs();
    ready_mode = 0;
    cycles(RW + DEPTH + 6);
    expect_no_rec();
    expect_eq("no word survives", pay_q.size(), 0);
    sent_q.delete();

    where = "soft_reset of several cycles with noise on the inputs";
    do_sfs(1'b0);
    fid++;
    do_sfs(1'b1);
    i_soft_reset = 1'b1;
    cycles(3);
    i_soft_reset = 1'b0;
    settle();
    expect_reset_outputs();

    where = "identity after all the soft_resets";
    i_metadata_record_id = 32'hABCD_0002;
    set_sfs(1'b0);
    settle();
    ts = exp_ts();
    cycles(1);
    fid++;
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'(fid), 0, 0, 0, 0, 6'd0);
    expect_rec_id(r, ts, 32'hABCD_0002);
    get_rec(r);
    settle();
    expect_eq("timestamp continues", o_timestamp, ts);
  endtask

  // Sensor event selection with vc_exposure_map 0 and 1; generator events have no effect.
  task automatic test_sensor_event_select();
    bit         lead;
    logic [1:0] o0;
    i_expected_height = 16'd0;
    for (int m = 0; m < 2; m++) begin
      lead  = 1'(m);
      where = $sformatf("vc_exposure_map = %0d", m);
      i_vc_exposure_map = lead;
      settle();
      expect_eq("lead_vc", o_lead_vc, lead);
      o0 = o_ctx_orphan;
      cycles(1);
      set_tfs(lead);
      settle();
      expect_eq("ev_fs ignores tpg_fs", o_ev_fs, 0);
      expect_eq("capture_start", o_capture_start, 2'b00);
      cycles(1);
      set_tfe(lead);
      settle();
      expect_eq("ev_fs", o_ev_fs, 0);
      cycles(2);
      settle();
      expect_eq("ctx_orphan unchanged", o_ctx_orphan, o0);
      set_sfs(lead);
      settle();
      expect_eq("ev_fs", o_ev_fs, 1);
      expect_eq("ev_vc", o_ev_vc, {1'b0, lead});
      expect_true("capture_start at the lead Frame Start", o_capture_start != 2'b00);
      cycles(1);
      set_sfs(!lead);
      settle();
      expect_eq("ev_fs", o_ev_fs, 1);
      expect_eq("ev_vc", o_ev_vc, {1'b0, !lead});
      expect_eq("no capture_start at the lag Frame Start", o_capture_start, 2'b00);
      cycles(1);
      settle();
      expect_eq("both channels joined", o_ctx_orphan, 2'b00);
      do_sfe(lead);
      do_sfe(!lead);
      drain();
      do_sfs(!lead);
      settle();
      expect_eq("a lag Frame Start first is an orphan", o_ctx_orphan[!lead], 1);
      do_sfe(!lead);
      drain();
      rec_q.delete();
    end

    where = "active_sel = 1";
    i_vc_exposure_map = 1'b1;
    i_active_sel      = 1'b1;
    settle();
    expect_eq("lead_vc is VC0 on the generator path", o_lead_vc, 0);
    cycles(1);
    set_sfsn(1'b0, 16'd5);
    settle();
    expect_eq("ev_fs ignores fc_fs", o_ev_fs, 0);
    expect_eq("capture_start", o_capture_start, 2'b00);
    cycles(1);
    set_tfs(1'b0);
    settle();
    expect_eq("ev_fs", o_ev_fs, 1);
    expect_eq("ev_vc", o_ev_vc, 0);
    expect_true("capture_start", o_capture_start != 2'b00);
    cycles(1);
    set_tfs(1'b1);
    settle();
    expect_eq("ev_vc", o_ev_vc, 1);
    cycles(1);
    do_tfe(1'b0);
    do_tfe(1'b1);
    drain();
    i_active_sel = 1'b0;
    settle();
    expect_eq("lead_vc back on the sensor path", o_lead_vc, 1);
    cycles(1);
    rec_q.delete();
  endtask

  // Test Pattern Generator with one channel per capture.
  task automatic test_tpg_single_vc();
    rec_t r;
    i_active_sel      = 1'b1;
    i_tpg_dual_vc_en  = 1'b0;
    i_vc_exposure_map = 1'b1;
    cycles(1);
    settle();
    expect_eq("lead_vc", o_lead_vc, 0);

    where = "a VC0 capture ends alone";
    set_tfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b01);
    cycles(1);
    settle();
    expect_eq("VC0 joined", o_ctx_orphan[0], 0);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 1);
    do_tfe(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("frame_count", o_frame_count, 1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);

    where = "a VC1 Frame Start is an orphan with tpg_dual_vc_en = 0";
    drain();
    set_tfs(1'b1);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b00);
    cycles(1);
    settle();
    expect_eq("VC1 orphan", o_ctx_orphan[1], 1);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 0);
    do_tfe(1'b1);
    drain();
    expect_no_rec();
    settle();
    expect_eq("frame_count", o_frame_count, 1);

    where = "sensor events are ignored while active_sel = 1";
    set_sfsn(1'b0, 16'd7);
    settle();
    expect_eq("ev_fs", o_ev_fs, 0);
    expect_eq("capture_start", o_capture_start, 2'b00);
    cycles(1);
    set_sfe(1'b0);
    settle();
    expect_end(2'b00, 2'b00);
    cycles(2);
    settle();
    expect_eq("frame_count", o_frame_count, 1);
  endtask

  // Test Pattern Generator with two channels per capture.
  task automatic test_tpg_dual_vc();
    rec_t        r0;
    rec_t        r1;
    logic [63:0] ts;
    i_active_sel         = 1'b1;
    i_tpg_dual_vc_en     = 1'b1;
    i_metadata_record_id = 32'hFACE_0001;
    cycles(1);

    where = "VC0 then VC1, closed in the same order";
    set_tfs(1'b0);
    settle();
    ts = exp_ts();
    expect_eq("capture_start", o_capture_start, 2'b01);
    cycles(1);
    set_tfs(1'b1);
    settle();
    expect_eq("no capture_start at the lag", o_capture_start, 2'b00);
    cycles(1);
    settle();
    expect_eq("both joined", o_ctx_orphan, 2'b00);
    expect_eq("VC1 in slot 0", o_ctx_slot[1], 0);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 1);
    do_tfe(1'b0);
    settle();
    expect_end(2'b00, 2'b00);
    cycles(3);
    settle();
    expect_end(2'b00, 2'b00);
    do_tfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    get_rec(r0);
    get_rec(r1);
    expect_rec(r0, 1'b0, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    expect_rec(r1, 1'b1, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    expect_rec_id(r0, ts, 32'hFACE_0001);
    expect_rec_id(r1, ts, 32'hFACE_0001);
    drain();

    where = "a capture waits for VC1 until the next VC0 Frame Start";
    do_tfs(1'b0);
    do_tfe(1'b0);
    cycles(4);
    settle();
    expect_end(2'b00, 2'b00);
    drain();
    do_tfs(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    do_tfs(1'b1);
    do_tfe(1'b1);
    do_tfe(1'b0);
    settle();
    expect_end(2'b10, 2'b00);
    drain();
    rec_q.delete();
  endtask

  // A normal Long and Short capture, then a second one in the same slot.
  task automatic test_basic_capture();
    rec_t        r;
    logic [63:0] ts;
    where = "lead Frame Start";
    i_metadata_record_id = 32'hC0DE_0001;
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start in the event cycle, slot 0 first", o_capture_start, 2'b01);
    expect_eq("ev_fs", o_ev_fs, 1);
    expect_eq("ev_vc", o_ev_vc, 0);
    ts = exp_ts();
    cycles(1);
    settle();
    expect_eq("capture_start lasts one cycle", o_capture_start, 2'b00);
    expect_eq("ctx_orphan[0]", o_ctx_orphan[0], 0);
    expect_eq("ctx_slot[0]", o_ctx_slot[0], 0);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 1);
    expect_eq("cur_frame_id_vc1 while orphan", cur_fid(1'b1), 0);
    // A later write does not alter the identity of the capture.
    i_metadata_record_id = 32'hDEAD_0002;

    where = "lag Frame Start";
    set_sfs(1'b1);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b00);
    cycles(1);
    settle();
    expect_eq("ctx_orphan[1]", o_ctx_orphan[1], 0);
    expect_eq("ctx_slot[1]", o_ctx_slot[1], 0);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B12, 1'b0);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), H_DEF);
    expect_eq("cur_line_vc1", cur_line(1'b1), H_DEF);

    where = "lead Frame End: the open lag holds the capture";
    set_sfe(1'b0);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);
    cycles(3);
    settle();
    expect_end(2'b00, 2'b00);

    where = "lag Frame End: capture_end one cycle later";
    set_sfe(1'b1);
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b00);
    expect_eq("timestamp not yet", o_timestamp, 0);
    expect_pub(1'b1, H_DEF, B12, DT_RAW12, 1'b0, 1'b0, 0, 0);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("timestamp", o_timestamp, ts);
    expect_eq("frame_count", o_frame_count, 1);

    where = "records of the capture";
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    expect_rec_id(r, ts, 32'hC0DE_0001);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, H_DEF, B12, 0, 0, DT_RAW12);
    expect_rec_id(r, ts, 32'hC0DE_0001);
    drain();

    where = "second capture: the channel state restarts, slot 0 again";
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b01);
    expect_eq("cur_line_vc0 before the edge", cur_line(1'b0), H_DEF);
    cycles(1);
    settle();
    expect_eq("cur_line_vc0 restarts", cur_line(1'b0), 0);
    expect_eq("frame_id advances", cur_fid(1'b0), 2);
    do_sfs(1'b1);
    settle();
    expect_eq("cur_line_vc1 restarts", cur_line(1'b1), 0);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b1);
    do_sfe(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_eq("frame_count", o_frame_count, 2);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd2, H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, H_DEF, B10, 0, 0, DT_RAW10);
    expect_eq("one capture_done per capture", n_done, 2);
    expect_eq("one capture_start per capture", n_start, 2);
    expect_eq("no resolution error", n_err_res, 0);
  endtask

  // The Short frame is still open when the next Long frame starts.
  task automatic test_short_open_at_next_long();
    rec_t r;
    i_expected_height = 16'd0;
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    drain();

    where = "next Long while the Short is open";
    set_sfs(1'b0);
    settle();
    expect_eq("the second slot is allocated", o_capture_start, 2'b10);
    cycles(1);
    settle();
    expect_eq("ctx_slot[0]", o_ctx_slot[0], 1);
    expect_eq("ctx_slot[1]", o_ctx_slot[1], 0);
    expect_eq("ctx_orphan", o_ctx_orphan, 2'b00);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 2);
    expect_eq("the Short keeps its identity", cur_fid(1'b1), 1);
    expect_end(2'b00, 2'b00);

    where = "the capture ends at the Short Frame End";
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("frame_count", o_frame_count, 1);
    expect_eq("ctx_slot[1] until its next Frame Start", o_ctx_slot[1], 0);
    expect_eq("cur_frame_id_vc1 holds", cur_fid(1'b1), 1);
    drain();

    where = "the next Short joins the second slot";
    do_sfs(1'b1);
    settle();
    expect_eq("ctx_slot[1]", o_ctx_slot[1], 1);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 2);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b10, 2'b00);

    where = "the records keep their identity";
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
  endtask

  // The lag Frame Start is lost: the capture waits until the next lead Frame Start.
  task automatic test_lag_fs_lost();
    rec_t r;
    i_expected_height = 16'd0;
    where = "the capture waits for its lag";
    do_sfs(1'b0);
    do_sfe(1'b0);
    cycles(4);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("no capture_end yet", n_end, 0);
    drain();

    where = "the next lead Frame Start stops the wait";
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b10);
    expect_end(2'b00, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    cycles(RW + 3);
    expect_no_rec();

    where = "the lag joins the most recent capture";
    do_sfs(1'b1);
    settle();
    expect_eq("ctx_orphan[1]", o_ctx_orphan[1], 0);
    expect_eq("ctx_slot[1]", o_ctx_slot[1], 1);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 2);
    do_sfe(1'b1);
    do_sfe(1'b0);
    settle();
    expect_end(2'b10, 2'b00);
    drain();
    rec_q.delete();

    where = "a lag Frame Start with no wait set is an orphan";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    set_sfs(1'b1);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b00);
    cycles(1);
    settle();
    expect_eq("ctx_orphan[1]", o_ctx_orphan[1], 1);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 0);
    do_sfe(1'b1);
    drain();
    expect_no_rec();
    do_sfe(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    drain();
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd3, 0, 0, 0, 0, 6'd0);
    expect_no_rec();
  endtask

  // The lead Frame Start is lost: the lag frame is an orphan, counted but not recorded.
  task automatic test_lead_fs_lost();
    rec_t r;
    where = "lag Frame Start with no capture";
    set_sfs(1'b1);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b00);
    cycles(1);
    settle();
    expect_eq("ctx_orphan", o_ctx_orphan, 2'b11);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 0);

    where = "orphan lines are counted";
    bp_lines(1'b1, 2, B10, 1'b1);
    settle();
    expect_eq("cur_line_vc1", cur_line(1'b1), 2);

    where = "orphan close: no record, nothing published";
    set_sfe(1'b1);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    drain();
    settle();
    expect_no_rec();
    expect_pub(1'b1, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    expect_eq("frame_count", o_frame_count, 0);
    expect_eq("no capture_end", n_end, 0);

    where = "the following capture is normal";
    capture_pair(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
  endtask

  // A Frame End is lost on each channel in turn: ctx_sync_close.
  task automatic test_fe_lost();
    rec_t r;
    where = "lead Frame End lost";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_lines(1'b0, 2, B10, 1'b1);
    set_sfs(1'b0);
    settle();
    expect_eq("no err_resolution on ctx_sync_close", o_err_resolution, 0);
    expect_eq("capture_start", o_capture_start, 2'b10);
    cycles(1);
    settle();
    // The flags are computed as usual and reach the record and the published registers.
    expect_pub(1'b0, 2, B10, DT_RAW10, 1'b0, 1'b1, 0, 0);
    get_rec(r);
    expect_rec(r, 1'b0, errs(0, 0, 0, 1, 0), 14'd1, 2, B10, 0, 0, DT_RAW10);
    expect_end(2'b00, 2'b00);

    where = "the old capture still ends at its Short Frame End";
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    drain();

    where = "lag Frame End lost: the next lag Frame Start closes it and joins";
    do_sfs(1'b1);
    settle();
    expect_eq("VC1 joined slot 1", o_ctx_slot[1], 1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, 1, B10, 1'b1);
    do_sfe(1'b0);
    drain();
    rec_q.delete();
    set_sfs(1'b0);
    settle();
    expect_eq("the free slot 0 is allocated", o_capture_start, 2'b01);
    cycles(1);
    set_sfs(1'b1);
    settle();
    expect_eq("no err_resolution on ctx_sync_close", o_err_resolution, 0);
    cycles(1);
    settle();
    expect_end(2'b10, 2'b00);
    expect_eq("VC1 now in slot 0", o_ctx_slot[1], 0);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 3);
    expect_pub(1'b1, 1, B10, DT_RAW10, 1'b0, 1'b1, 0, 0);
    get_rec(r);
    expect_rec(r, 1'b1, errs(0, 0, 0, 1, 0), 14'd2, 1, B10, 0, 0, DT_RAW10);
    expect_eq("err_resolution never pulsed", n_err_res, 0);

    where = "lag Frame End lost with no capture waiting: orphan";
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b0);
    drain();
    rec_q.delete();
    set_sfs(1'b1);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    settle();
    expect_eq("VC1 orphan", o_ctx_orphan[1], 1);
    expect_end(2'b01, 2'b00);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd3, H_DEF, B10, 0, 0, DT_RAW10);
  endtask

  // A Frame End with no frame open is ignored.
  task automatic test_fe_without_frame();
    rec_t r;
    int   ends;
    where = "Frame End out of reset";
    do_sfe(1'b0);
    do_sfe(1'b1);
    cycles(3);
    settle();
    expect_reset_outputs();
    expect_no_rec();

    where = "a normal capture";
    capture_pair(1'b0);
    drain();
    get_rec(r);
    get_rec(r);
    settle();
    expect_eq("frame_count", o_frame_count, 1);
    ends = n_end;

    where = "a second Frame End on a closed channel";
    do_sfe(1'b0);
    do_sfe(1'b1);
    cycles(3);
    drain();
    settle();
    expect_no_rec();
    expect_eq("frame_count", o_frame_count, 1);
    expect_eq("capture_end", n_end, ends);
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);
    expect_pub(1'b1, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);

    where = "an open channel holds its capture";
    i_expected_height = 16'd0;
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b1);
    cycles(3);
    settle();
    expect_end(2'b00, 2'b00);
    do_sfe(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    drain();
    rec_q.delete();

    where = "the lead holds its capture the other way round";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    cycles(3);
    settle();
    expect_end(2'b00, 2'b00);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
  endtask

  // Both slots in use at a lead Frame Start, also when held by queued records.
  task automatic test_slots_full();
    rec_t r;
    i_expected_height = 16'd0;
    where = "two captures open";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    drain();
    do_sfs(1'b0);
    do_sfe(1'b0);
    drain();
    rec_q.delete();

    where = "a third lead Frame Start finds no free slot";
    set_sfs(1'b0);
    settle();
    expect_eq("no capture_start", o_capture_start, 2'b00);
    cycles(1);
    settle();
    expect_eq("the lead is an orphan", o_ctx_orphan[0], 1);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 0);
    expect_end(2'b10, 2'b00);
    cycles(1);
    do_sfe(1'b0);
    cycles(RW + 3);
    expect_no_rec();
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    drain();
    expect_no_rec();

    where = "the refused capture used no frame_id";
    do_sfs(1'b0);
    settle();
    expect_eq("frame_id", cur_fid(1'b0), 3);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();

    where = "slots held by queued records";
    ready_mode = 1;
    cycles(1);
    do_sfs(1'b0);
    emb_line(1'b0, DEPTH * 4, 3);
    sent_q.delete();
    do_sfe(1'b0);
    set_sfs(1'b0);
    settle();
    expect_eq("the second slot is taken", o_capture_start, 2'b10);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    do_sfs(1'b1);
    do_sfe(1'b1);
    do_sfe(1'b0);
    settle();
    expect_end(2'b10, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    set_sfs(1'b0);
    settle();
    expect_eq("no slot is free while its record is queued", o_capture_start, 2'b00);
    cycles(1);
    settle();
    expect_eq("orphan", o_ctx_orphan[0], 1);
    do_sfe(1'b0);
    ready_mode = 0;
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd4, 0, 0, DEPTH, 1, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd5, 0, 0, 0, 0, 6'd0);
    drain();
    pay_q.delete();

    where = "slots free again after the records left";
    set_sfs(1'b0);
    settle();
    expect_eq("slot 0 again", o_capture_start, 2'b01);
    cycles(1);
  endtask

  // Drop before the lag Frame Start: the lag still joins the dropped capture.
  task automatic test_drop_before_lag();
    rec_t r;
    where = "drop, then the lag still joins";
    do_sfs(1'b0);
    do_drop(2'b01);
    do_sfs(1'b1);
    settle();
    expect_eq("the lag joined", o_ctx_orphan[1], 0);
    expect_eq("slot", o_ctx_slot[1], 0);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 1);

    where = "dropped capture: no publication, no capture_done";
    bp_line(1'b0, B10, 1'b1);
    bp_line(1'b1, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);
    settle();
    expect_eq("frame_count counts a dropped capture", o_frame_count, 1);
    expect_eq("timestamp holds", o_timestamp, 0);
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    expect_pub(1'b1, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    expect_eq("err_resolution suppressed", n_err_res, 0);
    get_rec(r);
    expect_rec(r, 1'b0, errs(1, 0, 0, 1, 0), 14'd1, 1, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, errs(1, 0, 0, 1, 0), 14'd1, 1, B10, 0, 0, DT_RAW10);
    drain();

    where = "the following capture is unaffected";
    capture_pair(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd2, H_DEF, B10, 0, 0, DT_RAW10);
  endtask

  // Drop after one channel has closed: aborted only in the record of the later close.
  task automatic test_drop_after_close();
    rec_t        r;
    logic [63:0] ts;
    set_sfs(1'b0);
    settle();
    ts = exp_ts();
    cycles(1);
    do_sfs(1'b1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B12, 1'b0);

    where = "the channel closed before the drop";
    do_sfe(1'b0);
    settle();
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);

    where = "the channel closed after the drop";
    do_drop(2'b10);
    set_sfe(1'b1);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b01);
    expect_pub(1'b1, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    cycles(1);
    settle();
    expect_eq("timestamp holds", o_timestamp, 0);
    expect_eq("frame_count", o_frame_count, 1);
    get_rec(r);
    expect_rec(r, 1'b1, errs(1, 0, 0, 0, 0), 14'd1, H_DEF, B12, 0, 0, DT_RAW12);
    expect_rec_id(r, ts, 32'd0);

    where = "no channel is closed by a drop";
    drain();
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_drop(2'b11);
    cycles(RW + 3);
    settle();
    expect_no_rec();
    expect_end(2'b00, 2'b00);
    expect_eq("still attached", o_ctx_orphan, 2'b00);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 2);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);
  endtask

  // drop_vc_mask combinations, channels without a slot of their own, drops in event cycles.
  task automatic test_drop_masks();
    rec_t        r;
    logic [63:0] ts_before;
    i_expected_height = 16'd0;
    for (int m = 1; m < 4; m++) begin
      where = $sformatf("mask %0d on one open capture", m);
      do_sfs(1'b0);
      do_sfs(1'b1);
      do_drop(2'(m));
      do_sfe(1'b0);
      do_sfe(1'b1);
      settle();
      expect_end(2'b01, 2'b01);
      cycles(1);
      get_rec(r);
      expect_eq("aborted", r.w[0][16], 1);
      get_rec(r);
      expect_eq("aborted", r.w[0][16], 1);
      drain();
    end

    where = "mask 00 names no channel";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_drop(2'b00);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    drain();
    rec_q.delete();

    where = "no open slot: no effect";
    do_drop(2'b11);
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    drain();
    rec_q.delete();

    where = "an orphan channel names the most recent open slot";
    do_sfs(1'b1);
    do_sfs(1'b0);
    do_drop(2'b10);
    do_sfe(1'b1);
    do_sfe(1'b0);
    drain();
    get_rec(r);
    expect_eq("aborted", r.w[0][16], 1);
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b10);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);

    where = "a channel whose slot has ended names the most recent open slot";
    do_sfs(1'b1);
    do_sfe(1'b1);
    do_sfe(1'b0);
    settle();
    expect_end(2'b10, 2'b00);
    drain();
    rec_q.delete();
    do_sfs(1'b0);
    settle();
    expect_eq("VC0 in slot 0", o_ctx_slot[0], 0);
    expect_eq("VC1 still names slot 1", o_ctx_slot[1], 1);
    do_drop(2'b10);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    drain();
    rec_q.delete();

    where = "two open slots: each channel names its own";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    drain();
    do_sfs(1'b0);
    do_drop(2'b10);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    drain();
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b10, 2'b00);
    drain();
    rec_q.delete();

    where = "a drop in the cycle of a lead Frame Start hits the old attachment";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    drain();
    set_sfs(1'b0);
    set_drop(2'b01);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b10);
    cycles(1);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    drain();
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b10, 2'b00);
    drain();
    rec_q.delete();

    where = "a drop in the cycle of a ctx_sync_close counts first";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10, 1'b1);
    set_sfs(1'b0);
    set_drop(2'b01);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b10);
    cycles(1);
    settle();
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    get_rec(r);
    expect_eq("aborted set", r.w[0][16], 1);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    drain();
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b10, 2'b00);
    cycles(1);
    drain();
    rec_q.delete();

    where = "a drop in the capture_end cycle";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    ts_before = o_timestamp;
    set_drop(2'b11);
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);
    settle();
    expect_eq("timestamp holds", o_timestamp, ts_before);
    get_rec(r);
    expect_eq("closed before the drop: aborted clear", r.w[0][16], 0);
    get_rec(r);
    expect_eq("closed before the drop: aborted clear", r.w[0][16], 0);
    drain();

    where = "a drop in the cycle of a Frame End counts first";
    i_expected_height = 16'd3;
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10, 1'b1);
    set_sfe(1'b0);
    set_drop(2'b01);
    settle();
    expect_eq("err_resolution suppressed", o_err_resolution, 0);
    cycles(1);
    get_rec(r);
    expect_eq("aborted set", r.w[0][16], 1);
    i_expected_height = 16'd0;
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);
  endtask

  // core_enable low with 0, 1 and 2 open slots, short low pulses, and re-enable.
  task automatic test_core_enable();
    rec_t        r;
    logic [31:0] fc;
    logic [15:0] l0;
    logic [15:0] b0;
    where = "low with nothing open: frame events are not taken";
    i_core_enable = 1'b0;
    cycles(2);
    set_sfsn(1'b0, 16'd9);
    settle();
    expect_eq("ev_fs follows the source", o_ev_fs, 1);
    expect_eq("capture_start", o_capture_start, 2'b00);
    cycles(1);
    set_sfe(1'b0);
    cycles(1);
    i_core_enable = 1'b1;
    cycles(1);
    settle();
    expect_eq("ctx_orphan", o_ctx_orphan, 2'b11);
    expect_eq("frame_count", o_frame_count, 0);

    where = "the first Frame Start after re-enabling opens a capture";
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b01);
    cycles(1);

    where = "one open slot";
    do_sfs(1'b1);
    bp_lines(1'b0, 2, B10, 1'b1);
    bp_lines(1'b1, 1, B10, 1'b1);
    i_core_enable = 1'b0;
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b01);
    expect_eq("both channels detached", o_ctx_orphan, 2'b11);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 0);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 0);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("frame_count", o_frame_count, 1);
    expect_eq("timestamp holds", o_timestamp, 0);

    where = "detached without a record; the accumulators hold";
    do_sfe(1'b0);
    do_sfe(1'b1);
    cycles(2);
    drain();
    settle();
    expect_no_rec();
    expect_eq("cur_line_vc0 holds", cur_line(1'b0), 2);
    expect_eq("cur_line_vc1 holds", cur_line(1'b1), 1);
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    i_core_enable = 1'b1;
    cycles(1);

    where = "two open slots: slot 1 one cycle after slot 0";
    i_expected_height = 16'd0;
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    drain();
    rec_q.delete();
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b10);
    cycles(1);
    settle();
    fc = o_frame_count;
    i_core_enable = 1'b0;
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);
    settle();
    expect_end(2'b10, 2'b10);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("frame_count", o_frame_count, fc + 2);
    i_core_enable = 1'b1;
    cycles(1);
    drain();
    expect_no_rec();

    where = "one low cycle: the capture still ends";
    do_sfs(1'b0);
    do_sfs(1'b1);
    i_core_enable = 1'b0;
    cycles(1);
    i_core_enable = 1'b1;
    settle();
    expect_end(2'b01, 2'b01);
    expect_eq("ctx_orphan", o_ctx_orphan, 2'b11);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);

    where = "one low cycle with a lag wait pending";
    do_sfs(1'b0);
    i_core_enable = 1'b0;
    cycles(1);
    i_core_enable = 1'b1;
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);
    do_sfs(1'b1);
    settle();
    expect_eq("no capture waits any more", o_ctx_orphan[1], 1);
    do_sfe(1'b1);
    drain();
    expect_no_rec();

    where = "queued records and published registers across core_enable low";
    i_expected_height = 16'(H_DEF);
    ready_mode = 1;
    cycles(1);
    capture_pair(1'b0);
    cycles(2);
    settle();
    l0 = pub_lines(1'b0);
    b0 = pub_bytes(1'b0);
    expect_eq("published before", l0, H_DEF);
    i_core_enable = 1'b0;
    cycles(2);
    ready_mode = 0;
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, r.w[1][13:0], H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, r.w[1][13:0], H_DEF, B10, 0, 0, DT_RAW10);
    settle();
    expect_eq("published holds", pub_lines(1'b0), l0);
    expect_eq("published holds", pub_bytes(1'b0), b0);
    i_core_enable = 1'b1;
    cycles(1);
    drain();

    where = "beats and last_fnum while low";
    i_fnum_check_en = 1'b1;
    do_sfsn(1'b0, 16'd100);
    do_sfsn(1'b1, 16'd200);
    i_core_enable = 1'b0;
    cycles(1);
    bp_line(1'b0, 16, 1'b1);
    settle();
    expect_eq("beats are still counted", cur_line(1'b0), 1);
    set_sfsn(1'b0, 16'd500);
    settle();
    expect_eq("no frame-number check while low", o_err_frame_num, 0);
    cycles(1);
    i_core_enable = 1'b1;
    cycles(1);
    set_sfsn(1'b0, 16'd101);
    settle();
    expect_eq("last_fnum held across the low time", o_err_frame_num, 0);
    expect_eq("capture_start", o_capture_start, 2'b01);
    cycles(1);
    do_sfsn(1'b1, 16'd201);
    i_fnum_check_en   = 1'b0;
    i_expected_height = 16'd0;
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();

    // vc_exposure_map changes with frames open here, as a test of sampling.
    where = "a lead Frame Start makes two slots able to end";
    i_vc_exposure_map = 1'b1;
    cycles(1);
    do_sfs(1'b1);
    do_sfs(1'b0);
    do_sfe(1'b0);
    drain();
    i_vc_exposure_map = 1'b0;
    cycles(1);
    do_sfs(1'b0);
    do_sfe(1'b0);
    drain();
    rec_q.delete();
    settle();
    fc = o_frame_count;
    i_vc_exposure_map = 1'b1;
    set_sfs(1'b1);
    settle();
    expect_eq("no free slot", o_capture_start, 2'b00);
    expect_end(2'b00, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b10, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    expect_eq("frame_count advanced once per capture", o_frame_count, fc + 2);
    expect_eq("the lead is an orphan", o_ctx_orphan[1], 1);
    i_vc_exposure_map = 1'b0;
    cycles(1);
  endtask

  // A change of active_sel clears the lag wait (the source changes inside a frame on purpose).
  task automatic test_active_sel_change();
    i_expected_height = 16'd0;
    where = "sensor path, lead closed, lag wait pending";
    do_sfs(1'b0);
    do_sfe(1'b0);
    cycles(3);
    settle();
    expect_end(2'b00, 2'b00);
    i_active_sel = 1'b1;
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_end(2'b00, 2'b00);
    cycles(1);
    i_active_sel = 1'b0;
    cycles(2);
    drain();
    rec_q.delete();

    where = "generator path, lead closed, lag wait pending";
    i_active_sel     = 1'b1;
    i_tpg_dual_vc_en = 1'b1;
    cycles(2);
    do_tfs(1'b0);
    do_tfe(1'b0);
    cycles(3);
    settle();
    expect_end(2'b00, 2'b00);
    i_active_sel = 1'b0;
    cycles(1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    drain();
    rec_q.delete();

    where = "lead still open during the change: it ends once the lead has closed";
    do_sfs(1'b0);
    i_active_sel = 1'b1;
    cycles(1);
    i_active_sel = 1'b0;
    cycles(3);
    settle();
    expect_end(2'b00, 2'b00);
    do_sfe(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    drain();
    rec_q.delete();

    where = "after the change no capture waits: a late lag is an orphan";
    do_sfs(1'b0);
    i_active_sel = 1'b1;
    cycles(1);
    i_active_sel = 1'b0;
    cycles(1);
    do_sfs(1'b1);
    settle();
    expect_eq("VC1 orphan", o_ctx_orphan[1], 1);
    do_sfe(1'b1);
    do_sfe(1'b0);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    drain();
    rec_q.delete();

    where = "a change with no capture open has no effect";
    i_active_sel = 1'b1;
    cycles(2);
    i_active_sel = 1'b0;
    cycles(2);
    do_sfs(1'b0);
    do_sfs(1'b1);
    settle();
    expect_eq("VC1 joined", o_ctx_orphan[1], 0);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
  endtask

  // Frame events on VC2 and VC3 follow the source mux and change nothing else.
  task automatic test_vc2_vc3_events();
    rec_t r;
    relax_vc          = 1'b1;
    i_fnum_check_en   = 1'b1;
    i_expected_height = 16'd0;
    for (int v = 2; v < 4; v++) begin
      where = $sformatf("sensor path, VC%0d, nothing open", v);
      i_fc_fs        = 1'b1;
      i_fc_vc        = 2'(v);
      i_fc_frame_num = 16'd55;
      settle();
      expect_eq("ev_fs", o_ev_fs, 1);
      expect_eq("ev_vc", o_ev_vc, v);
      expect_eq("capture_start", o_capture_start, 2'b00);
      expect_eq("err_frame_num", o_err_frame_num, 0);
      cycles(1);
      settle();
      expect_eq("ctx_orphan", o_ctx_orphan, 2'b11);
      i_fc_fe = 1'b1;
      i_fc_vc = 2'(v);
      cycles(1);

      where = $sformatf("generator path, VC%0d, nothing open", v);
      i_active_sel = 1'b1;
      cycles(1);
      i_tpg_fs  = 1'b1;
      i_tpg_fvc = 2'(v);
      settle();
      expect_eq("ev_fs", o_ev_fs, 1);
      expect_eq("ev_vc", o_ev_vc, v);
      expect_eq("capture_start", o_capture_start, 2'b00);
      cycles(1);
      i_tpg_fe  = 1'b1;
      i_tpg_fvc = 2'(v);
      cycles(1);
      i_active_sel = 1'b0;
      cycles(1);
    end
    cycles(1);
    settle();
    expect_eq("frame_count", o_frame_count, 0);
    expect_no_rec();

    where = "VC2 and VC3 events during an open capture";
    i_fnum_check_en = 1'b0;
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10, 1'b1);
    for (int v = 2; v < 4; v++) begin
      i_fc_fe = 1'b1;
      i_fc_vc = 2'(v);
      cycles(1);
      i_fc_fs = 1'b1;
      i_fc_vc = 2'(v);
      settle();
      expect_eq("capture_start", o_capture_start, 2'b00);
      cycles(1);
    end
    cycles(RW + 3);
    settle();
    expect_end(2'b00, 2'b00);
    expect_no_rec();
    expect_eq("ctx_orphan", o_ctx_orphan, 2'b00);
    expect_eq("cur_line_vc0 holds", cur_line(1'b0), 1);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 1);

    where = "the capture ends only at its own Frame Ends";
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    get_rec(r);
    expect_rec(r, 1'b0, errs(0, 0, 0, 1, 0), 14'd1, 1, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    drain();
    relax_vc = 1'b0;
  endtask

  // One capture with explicit frame numbers; e0, e1: err_frame_num expected at each Frame Start.
  task automatic fnum_pair(input logic [15:0] fn0, input logic [15:0] fn1, input bit e0,
                           input bit e1);
    rec_t r;
    set_sfsn(1'b0, fn0);
    settle();
    expect_eq("err_frame_num at the VC0 Frame Start", o_err_frame_num, e0);
    cycles(1);
    settle();
    expect_eq("err_frame_num lasts one cycle", o_err_frame_num, 0);
    set_sfsn(1'b1, fn1);
    settle();
    expect_eq("err_frame_num at the VC1 Frame Start", o_err_frame_num, e1);
    cycles(1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_eq("fnum_err in the record of VC0", r.w[0][17], e0);
    get_rec(r);
    expect_eq("fnum_err in the record of VC1", r.w[0][17], e1);
    drain();
  endtask

  // Frame-number sequence check.
  task automatic test_frame_number_check();
    rec_t r;
    int   errs0;
    i_expected_height = 16'd0;
    i_fnum_check_en   = 1'b1;
    where = "VC0: first after reset, +1, jump, exempt 0 and 1, wrap";
    fnum_pair(16'd5,    16'd1,  1'b1, 1'b0);
    fnum_pair(16'd6,    16'd2,  1'b0, 1'b0);
    fnum_pair(16'd11,   16'd3,  1'b1, 1'b0);
    fnum_pair(16'd12,   16'd4,  1'b0, 1'b0);
    fnum_pair(16'd0,    16'd5,  1'b0, 1'b0);
    fnum_pair(16'd1,    16'd6,  1'b0, 1'b0);
    fnum_pair(16'd2,    16'd7,  1'b0, 1'b0);
    fnum_pair(16'hFFFF, 16'd8,  1'b1, 1'b0);
    fnum_pair(16'd0,    16'd9,  1'b0, 1'b0);
    fnum_pair(16'd1,    16'd10, 1'b0, 1'b0);
    fnum_pair(16'd9,    16'd11, 1'b1, 1'b0);
    fnum_pair(16'hFFFE, 16'd12, 1'b1, 1'b0);
    fnum_pair(16'hFFFF, 16'd13, 1'b0, 1'b0);

    where = "VC1: the channels are independent";
    fnum_pair(16'd2,    16'd20, 1'b1, 1'b1);
    fnum_pair(16'd3,    16'd21, 1'b0, 1'b0);
    fnum_pair(16'd4,    16'd1,  1'b0, 1'b0);

    where = "fnum_check_en = 0: no pulse, no flag, last_fnum still follows";
    i_fnum_check_en = 1'b0;
    fnum_pair(16'd100, 16'd200, 1'b0, 1'b0);
    i_fnum_check_en = 1'b1;
    fnum_pair(16'd101, 16'd201, 1'b0, 1'b0);

    where = "an orphan frame is checked and updates last_fnum";
    do_sfsn(1'b0, 16'd102);
    do_sfsn(1'b1, 16'd202);
    do_sfe(1'b1);
    set_sfsn(1'b1, 16'd300);
    settle();
    expect_eq("err_frame_num on the orphan", o_err_frame_num, 1);
    cycles(1);
    settle();
    expect_eq("VC1 orphan", o_ctx_orphan[1], 1);
    do_sfe(1'b1);
    do_sfe(1'b0);
    drain();
    rec_q.delete();
    fnum_pair(16'd103, 16'd301, 1'b0, 1'b0);

    where = "generator path: no check, last_fnum unchanged, flag cleared";
    set_sfsn(1'b0, 16'd500);
    settle();
    expect_eq("err_frame_num", o_err_frame_num, 1);
    cycles(1);
    do_sfsn(1'b1, 16'd302);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_eq("fnum_err", r.w[0][17], 1);
    get_rec(r);
    drain();
    i_active_sel = 1'b1;
    cycles(1);
    errs0 = n_err_fnum;
    do_tfs(1'b0);
    do_tfs(1'b1);
    do_tfe(1'b0);
    do_tfe(1'b1);
    get_rec(r);
    expect_eq("generator record fnum_err", r.w[0][17], 0);
    get_rec(r);
    expect_eq("generator record fnum_err", r.w[0][17], 0);
    expect_eq("no err_frame_num on the generator path", n_err_fnum, errs0);
    drain();
    i_active_sel = 1'b0;
    cycles(1);
    fnum_pair(16'd501, 16'd303, 1'b0, 1'b0);

    where = "resets clear last_fnum";
    soft_reset_pulse();
    fnum_pair(16'd7, 16'd1, 1'b1, 1'b0);
    soft_reset_pulse();
    fnum_pair(16'd1, 16'd2, 1'b0, 1'b1);
    fnum_pair(16'd2, 16'd3, 1'b0, 1'b0);
  endtask

  // frame_id counts from 1 and skips 0 at the wrap, by real stimulus (more than 16383 captures).
  task automatic test_frame_id_wrap();
    int expect_id;
    expect_eq("frame_id_buf0 width", $bits(o_frame_id_buf0), 14);
    expect_eq("frame_id_buf1 width", $bits(o_frame_id_buf1), 14);
    expect_eq("cur_frame_id_vc0 width", $bits(o_cur_frame_id_vc0), 14);
    expect_eq("cur_frame_id_vc1 width", $bits(o_cur_frame_id_vc1), 14);
    i_active_sel     = 1'b1;
    i_tpg_dual_vc_en = 1'b0;
    cycles(1);
    expect_id = 1;
    where     = "frame_id sequence";
    for (int i = 0; i < 16400; i++) begin
      // The model state only paces the stimulus: queue and slots empty before each capture.
      while (m_q.size() != 0 || m_open != 2'b00) cycles(1);
      do_tfs(1'b0);
      settle();
      if (cur_fid(1'b0) !== 14'(expect_id))
        fail($sformatf("capture %0d: frame_id %0d, expected %0d", i, cur_fid(1'b0), expect_id));
      if (cur_fid(1'b0) === 14'd0) fail("frame_id 0 assigned to a capture");
      check_count++;
      do_tfe(1'b0);
      expect_id = (expect_id == 16383) ? 1 : expect_id + 1;
      // soft_reset does not clear the count.
      if (i == 100) begin
        wait_slots_end();
        cycles(1);
        soft_reset_pulse();
        cycles(1);
      end
      rec_q.delete();
    end
    drain();
    rec_q.delete();
    where = "the wrap was passed";
    expect_eq("next value after the wrap", 14'(expect_id), 18);
  endtask

  // Record and published timestamps against TS_INCR, across soft_reset and core_enable low.
  task automatic test_timestamp();
    rec_t        r;
    logic [63:0] ts;
    i_expected_height = 16'd0;
    where = "captures at known cycle counts";
    for (int i = 0; i < 40; i++) begin
      cycles(rand_range(0, 400));
      if (i == 20) begin
        where = "the time base continues across soft_reset";
        soft_reset_pulse();
      end
      if (i == 30) begin
        where = "the time base continues across core_enable low";
        i_core_enable = 1'b0;
        cycles(rand_range(1, 20));
        i_core_enable = 1'b1;
        cycles(1);
      end
      set_sfs(1'b0);
      settle();
      ts = exp_ts();
      expect_true("a slot is free", o_capture_start != 2'b00);
      cycles(1);
      cycles(rand_range(0, 30));
      do_sfs(1'b1);
      do_sfe(1'b0);
      cycles(rand_range(0, 3));
      do_sfe(1'b1);
      get_rec(r);
      expect_rec_id(r, ts, 32'd0);
      get_rec(r);
      expect_rec_id(r, ts, 32'd0);
      drain();
      settle();
      expect_eq("published timestamp", o_timestamp, ts);
      // One long interval accumulates the fractional part of TS_INCR.
      if (i == 35) cycles(20000);
    end
  endtask

  // dma_buf_claim writes the FRAME_ID entry it selects.
  task automatic test_frame_id_buf();
    i_expected_height = 16'd0;
    where = "claim of index 0";
    do_sfs(1'b0);
    set_claim(1'b0);
    settle();
    expect_eq("not before the clock edge", o_frame_id_buf0, 0);
    cycles(1);
    settle();
    expect_eq("frame_id_buf0", o_frame_id_buf0, 1);
    expect_eq("frame_id_buf1 holds", o_frame_id_buf1, 0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    cycles(4);
    settle();
    expect_eq("frame_id_buf0 holds after the capture ends", o_frame_id_buf0, 1);
    drain();
    rec_q.delete();

    where = "claim of index 1";
    do_sfs(1'b0);
    set_claim(1'b1);
    cycles(1);
    settle();
    expect_eq("frame_id_buf1", o_frame_id_buf1, 2);
    expect_eq("frame_id_buf0 holds", o_frame_id_buf0, 1);
    do_sfs(1'b1);
    do_sfe(1'b0);
    drain();
    rec_q.delete();

    where = "the entry takes the most recently allocated slot";
    do_sfs(1'b0);
    settle();
    expect_eq("slot 1 was allocated", o_ctx_slot[0], 1);
    set_claim(1'b0);
    cycles(1);
    settle();
    expect_eq("frame_id_buf0", o_frame_id_buf0, 3);
    expect_eq("frame_id_buf1 holds", o_frame_id_buf1, 2);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();

    where = "no claim: both entries hold";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    settle();
    expect_eq("frame_id_buf0", o_frame_id_buf0, 3);
    expect_eq("frame_id_buf1", o_frame_id_buf1, 2);

    where = "a claim in the cycle of the next capture_start names the earlier capture";
    do_sfs(1'b0);
    set_claim(1'b1);
    set_sfs(1'b0);
    settle();
    expect_true("capture_start in the claim cycle", o_capture_start != 2'b00);
    cycles(1);
    settle();
    expect_eq("frame_id_buf1", o_frame_id_buf1, 5);
    set_claim(1'b0);
    cycles(1);
    settle();
    expect_eq("frame_id_buf0", o_frame_id_buf0, 6);
    expect_eq("frame_id_buf1 holds", o_frame_id_buf1, 5);
    drain();
    rec_q.delete();
  endtask

  // cur_frame_id timing: the cycle after ev_fs, held to the next ev_fs, follows a reused slot.
  task automatic test_cur_frame_id();
    i_expected_height = 16'd0;
    where = "the cycle after ev_fs of the channel";
    settle();
    expect_eq("cur_frame_id_vc0 at reset", cur_fid(1'b0), 0);
    set_sfs(1'b0);
    settle();
    expect_eq("not in the ev_fs cycle", cur_fid(1'b0), 0);
    cycles(1);
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 1);
    expect_eq("cur_frame_id_vc1 still an orphan", cur_fid(1'b1), 0);
    set_sfs(1'b1);
    settle();
    expect_eq("not in the ev_fs cycle", cur_fid(1'b1), 0);
    cycles(1);
    settle();
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 1);

    where = "held from the Frame End until the next ev_fs";
    do_sfe(1'b0);
    do_sfe(1'b1);
    cycles(5);
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 1);
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 1);
    drain();
    rec_q.delete();

    where = "slot reused by the lead while the lag stays: both follow in one cycle";
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b01);
    expect_eq("cur_frame_id_vc0 before the edge", cur_fid(1'b0), 1);
    expect_eq("cur_frame_id_vc1 before the edge", cur_fid(1'b1), 1);
    cycles(1);
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 2);
    expect_eq("cur_frame_id_vc1 follows the slot", cur_fid(1'b1), 2);
    do_sfs(1'b1);
    settle();
    expect_eq("cur_frame_id_vc1 after it joined", cur_fid(1'b1), 2);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();

    where = "an orphan reads 0";
    do_sfs(1'b1);
    settle();
    expect_eq("cur_frame_id_vc1", cur_fid(1'b1), 0);
    do_sfe(1'b1);
    cycles(2);
    settle();
    expect_eq("cur_frame_id_vc1 after its Frame End", cur_fid(1'b1), 0);
    expect_eq("cur_frame_id_vc0 still names its slot", cur_fid(1'b0), 2);

    where = "core_enable low reads 0";
    do_sfs(1'b0);
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 3);
    i_core_enable = 1'b0;
    settle();
    expect_eq("not before the edge", cur_fid(1'b0), 3);
    cycles(1);
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 0);
    i_core_enable = 1'b1;
    cycles(3);

    where = "soft_reset reads 0";
    do_sfs(1'b0);
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 4);
    soft_reset_pulse();
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 0);

    where = "directly from the previous identifier to the new one";
    do_sfs(1'b0);
    settle();
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 5);
    do_sfe(1'b0);
    drain();
    rec_q.delete();
    set_sfs(1'b0);
    settle();
    expect_eq("in the ev_fs cycle", cur_fid(1'b0), 5);
    cycles(1);
    settle();
    expect_eq("in the next cycle", cur_fid(1'b0), 6);
  endtask

  // Geometry on RAW10 and RAW12: beats, byte enables, Data Type, cur_line, 16-bit byte count.
  task automatic test_geometry();
    rec_t r;
    where = "beats with gaps, noise on sol and eol while bp_valid is low";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10, 1'b1, 40);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 1);

    where = "a single-beat line; cur_line in the cycle after bp_eol";
    i_bp_valid   = 1'b1;
    i_bp_vc_tag  = 2'd0;
    i_bp_dt_sel  = 1'b1;
    i_bp_byte_en = 4'b0111;
    i_bp_sol     = 1'b1;
    i_bp_eol     = 1'b1;
    settle();
    expect_eq("not in the bp_eol cycle", cur_line(1'b0), 1);
    cycles(1);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 2);

    where = "sol and eol without bp_valid are not taken";
    i_bp_sol     = 1'b1;
    i_bp_eol     = 1'b1;
    i_bp_vc_tag  = 2'd0;
    i_bp_byte_en = 4'b1111;
    cycles(1);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 2);

    where = "final beat byte enables";
    bp_line(1'b0, 9, 1'b1);
    bp_line(1'b0, 10, 1'b1);
    bp_line(1'b0, 11, 1'b1);
    bp_line(1'b0, 12, 1'b1);
    // The CRC bytes are already removed, so a final beat may carry no payload byte.
    bp_line(1'b0, 8, 1'b1, 0, 1'b1);
    i_bp_valid   = 1'b1;
    i_bp_vc_tag  = 2'd0;
    i_bp_dt_sel  = 1'b1;
    i_bp_byte_en = 4'b0000;
    i_bp_sol     = 1'b0;
    i_bp_eol     = 1'b1;
    cycles(1);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 7);
    do_sfe(1'b0);
    settle();
    expect_pub(1'b0, 7, 8, DT_RAW10, 1'b1, 1'b1, 0, 0);

    where = "the Data Type is that of the last line";
    bp_line(1'b1, B12, 1'b0);
    bp_line(1'b1, B10, 1'b1);
    bp_line(1'b1, B12, 1'b0);
    do_sfe(1'b1);
    settle();
    expect_pub(1'b1, 3, B12, DT_RAW12, 1'b0, 1'b0, 0, 0);
    get_rec(r);
    expect_rec(r, 1'b0, errs(0, 0, 1, 1, 0), 14'd1, 7, 8, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 3, B12, 0, 0, DT_RAW12);
    drain();

    where = "the other Data Type on the other channel";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_lines(1'b0, H_DEF, B12, 1'b0);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_pub(1'b0, H_DEF, B12, DT_RAW12, 1'b0, 1'b0, 0, 0);
    expect_pub(1'b1, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);
    drain();
    rec_q.delete();

    where = "a single-beat line as the last line";
    i_expected_height = 16'd1;
    for (int n = 1; n <= 4; n++) begin
      do_sfs(1'b0);
      do_sfs(1'b1);
      bp_line(1'b0, n, 1'b1);
      bp_line(1'b1, n, 1'b0);
      do_sfe(1'b0);
      do_sfe(1'b1);
      settle();
      expect_pub(1'b0, 1, n, DT_RAW10, 1'b1, 1'b0, 0, 0);
      expect_pub(1'b1, 1, n, DT_RAW12, 1'b1, 1'b0, 0, 0);
      drain();
      rec_q.delete();
    end
    i_expected_height = 16'(H_DEF);

    where = "the generator path adds nothing";
    i_active_sel = 1'b1;
    cycles(1);
    do_tfs(1'b0);
    do_tfs(1'b1);
    bp_line(1'b0, B10, 1'b1);
    bp_line(1'b1, B10, 1'b1);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 0);
    expect_eq("cur_line_vc1", cur_line(1'b1), 0);
    do_tfe(1'b0);
    do_tfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd7, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd7, 0, 0, 0, 0, 6'd0);
    drain();
    i_active_sel = 1'b0;
    cycles(1);

    where = "a line of 65540 bytes wraps the 16-bit byte count";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, 65540, 1'b1);
    do_sfe(1'b0);
    settle();
    expect_eq("published byte_count", pub_bytes(1'b0), 4);
    expect_eq("published line_count", pub_lines(1'b0), 1);
    do_sfe(1'b1);
    cycles(1);
  endtask

  // A line without bp_eol is not counted; the next bp_sol restarts the sum.
  task automatic test_line_without_eol();
    rec_t r;
    where = "a cut line in the middle of the frame";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10, 1'b1);
    bp_line(1'b0, 40, 1'b1, 0, 1'b1);
    settle();
    expect_eq("not counted", cur_line(1'b0), 1);
    bp_line(1'b0, B10, 1'b1);
    settle();
    expect_eq("the next line starts a new sum", cur_line(1'b0), 2);
    bp_line(1'b0, B10, 1'b1);

    where = "a cut last line";
    bp_line(1'b0, 8, 1'b1, 0, 1'b1);
    do_sfe(1'b0);
    settle();
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);

    where = "a cut first line on the other channel";
    bp_line(1'b1, 16, 1'b0, 0, 1'b1);
    settle();
    expect_eq("not counted", cur_line(1'b1), 0);
    bp_lines(1'b1, H_DEF, B12, 1'b0);
    do_sfe(1'b1);
    settle();
    expect_pub(1'b1, H_DEF, B12, DT_RAW12, 1'b0, 1'b0, 0, 0);
    expect_eq("no resolution error", n_err_res, 0);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, H_DEF, B12, 0, 0, DT_RAW12);
  endtask

  // One frame on channel vc as the lead; line bad_line carries bad_bytes instead of good.
  task automatic res_frame(input bit vc, input int w, input int h, input bit raw10, input int good,
                           input int nlines, input int bad_line, input int bad_bytes, input bit wm,
                           input bit hm);
    rec_t       r;
    int         last;
    logic [5:0] dt;
    i_vc_exposure_map = vc;
    i_expected_width  = 16'(w);
    i_expected_height = 16'(h);
    do_sfs(vc);
    // Values written after ctx_open have no effect on this frame.
    i_expected_width  = 16'(w + 2);
    i_expected_height = 16'(h + 1);
    last = 0;
    for (int i = 0; i < nlines; i++) begin
      last = (i == bad_line) ? bad_bytes : good;
      bp_line(vc, last, raw10);
    end
    dt = (nlines == 0) ? 6'd0 : (raw10 ? DT_RAW10 : DT_RAW12);
    set_sfe(vc);
    settle();
    expect_eq("err_resolution in the close cycle", o_err_resolution, wm || hm);
    cycles(1);
    settle();
    expect_eq("err_resolution lasts one cycle", o_err_resolution, 0);
    expect_pub(vc, nlines, last, dt, wm, hm, 0, 0);
    get_rec(r);
    expect_rec(r, vc, errs(0, 0, wm, hm, 0), r.w[1][13:0], nlines, last, 0, 0, dt);
    drain();
  endtask

  // Width and height checks against EXPECTED_RESOLUTION sampled at ctx_open.
  task automatic test_resolution_mismatch();
    bit vc;
    for (int c = 0; c < 2; c++) begin
      vc    = 1'(c);
      where = $sformatf("VC%0d: lines equal to the reference raise no flag", c);
      // RAW10: (w x 5) >> 2; RAW12: (w x 3) >> 1.
      res_frame(vc, 64, 2, 1'b1, 80,  2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 64, 2, 1'b0, 96,  2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 65, 2, 1'b1, 81,  2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 65, 2, 1'b0, 97,  2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 66, 2, 1'b1, 82,  2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 66, 2, 1'b0, 99,  2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 67, 2, 1'b1, 83,  2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 67, 2, 1'b0, 100, 2, -1, 0, 1'b0, 1'b0);

      where = $sformatf("VC%0d: one bad line among good ones", c);
      res_frame(vc, 64, 3, 1'b1, 80, 3, 0, 81, 1'b1, 1'b0);
      res_frame(vc, 64, 3, 1'b1, 80, 3, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 64, 3, 1'b1, 80, 3, 1, 79, 1'b1, 1'b0);
      res_frame(vc, 64, 3, 1'b0, 96, 3, 2, 80, 1'b1, 1'b0);
      res_frame(vc, 64, 3, 1'b0, 96, 3, -1, 0, 1'b0, 1'b0);

      where = $sformatf("VC%0d: height", c);
      res_frame(vc, 64, 3, 1'b1, 80, 2, -1, 0, 1'b0, 1'b1);
      res_frame(vc, 64, 1, 1'b1, 80, 2, -1, 0, 1'b0, 1'b1);
      res_frame(vc, 64, 2, 1'b1, 80, 2, -1, 0, 1'b0, 1'b0);
      res_frame(vc, 64, 5, 1'b1, 80, 2, 0, 12, 1'b1, 1'b1);
    end

    where = "the reference is not truncated to 16 bits";
    // 65535 x 5 >> 2 = 81918 and 65535 x 3 >> 1 = 98302: their low 16 bits must not match.
    res_frame(1'b0, 65535, 1, 1'b1, 16382, 1, -1, 0, 1'b1, 1'b0);
    res_frame(1'b0, 65535, 1, 1'b0, 32766, 1, -1, 0, 1'b1, 1'b0);
    // 52428 x 5 >> 2 = 65535: the largest line that can match.
    res_frame(1'b0, 52428, 1, 1'b1, 65535, 1, -1, 0, 1'b0, 1'b0);

    where = "expected values of 0 are not a disable";
    res_frame(1'b0, 0, 0, 1'b1, 4, 0, -1, 0, 1'b0, 1'b0);
    res_frame(1'b0, 0, 0, 1'b1, 4, 1, -1, 0, 1'b1, 1'b1);
  endtask

  // err_resolution is suppressed for an orphan, a dropped capture, a ctx_sync_close and the
  // generator path.
  task automatic test_err_resolution_suppress();
    int n;
    where = "no suppression: one cycle, in the ctx_close cycle";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10 + 1, 1'b1);
    bp_lines(1'b0, 2, B10, 1'b1);
    bp_lines(1'b1, 2, B10, 1'b1);
    set_sfe(1'b0);
    settle();
    expect_eq("width mismatch", o_err_resolution, 1);
    cycles(1);
    settle();
    expect_eq("one cycle", o_err_resolution, 0);
    set_sfe(1'b1);
    settle();
    expect_eq("height mismatch", o_err_resolution, 1);
    cycles(1);
    settle();
    expect_eq("one cycle", o_err_resolution, 0);
    expect_eq("two pulses", n_err_res, 2);
    drain();
    rec_q.delete();
    n = n_err_res;

    where = "suppressed for an orphan";
    do_sfs(1'b1);
    bp_line(1'b1, B10 + 1, 1'b1);
    set_sfe(1'b1);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    drain();

    where = "suppressed for a dropped capture";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_drop(2'b11);
    bp_line(1'b0, B10 + 1, 1'b1);
    bp_line(1'b1, B10 + 1, 1'b1);
    set_sfe(1'b0);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    set_sfe(1'b1);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    drain();
    rec_q.delete();

    where = "suppressed for a ctx_sync_close";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10 + 1, 1'b1);
    bp_line(1'b1, B10 + 1, 1'b1);
    drain();
    i_expected_height = 16'd0;
    set_sfs(1'b0);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    drain();
    set_sfs(1'b1);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    drain();
    rec_q.delete();
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();

    where = "off while the generator path is selected";
    i_active_sel      = 1'b1;
    i_expected_height = 16'd3;
    cycles(1);
    do_tfs(1'b0);
    do_tfs(1'b1);
    set_tfe(1'b0);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    set_tfe(1'b1);
    settle();
    expect_eq("err_resolution", o_err_resolution, 0);
    cycles(1);
    drain();
    i_active_sel = 1'b0;
    cycles(1);
    expect_eq("no further pulse", n_err_res, n);
  endtask

  // Embedded counters and forwarding of the payload, unchanged, with meta_last and meta_sof.
  task automatic test_embedded_forwarding();
    rec_t r;
    i_expected_height = 16'd0;
    where = "metadata_capture_en = 0: no payload, counters stay 0";
    i_metadata_capture_en = 1'b0;
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 16, 1);
    emb_line(1'b1, 16, 1);
    sent_q.delete();
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    drain();
    settle();
    expect_eq("no payload word", pay_q.size(), 0);
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);

    where = "metadata_capture_en = 1: payload unchanged, meta_last at emb_eol, one meta_sof";
    i_metadata_capture_en = 1'b1;
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 10, 2);
    emb_line(1'b1, 7, 2);
    emb_line(1'b0, 8, 3);
    emb_line(1'b1, 12, 3);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 5, 2);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, 0, 0, 5, 2, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd2, 0, 0, 5, 2, 6'd0);
    drain();
    settle();
    expect_pub(1'b1, 0, 0, 6'd0, 1'b0, 1'b0, 5, 2);
    expect_payload();

    where = "every partial final beat, with gaps; the counters restart";
    do_sfs(1'b0);
    do_sfs(1'b1);
    for (int n = 1; n <= 9; n++) begin
      emb_line(1'b0, n, 3 + n, 30);
      emb_line(1'b1, 10 - n, 3 + n, 30);
    end
    do_sfe(1'b1);
    do_sfe(1'b0);
    settle();
    expect_pub(1'b1, 0, 0, 6'd0, 1'b0, 1'b0, 15, 9);
    drain();
    settle();
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 15, 9);
    expect_payload();
    rec_q.delete();

    where = "payload of an orphan channel still enters the FIFO";
    do_sfs(1'b1);
    settle();
    expect_eq("VC1 orphan", o_ctx_orphan[1], 1);
    emb_line(1'b1, 8, 15);
    do_sfe(1'b1);
    drain();
    expect_payload();
    expect_no_rec();

    where = "noise on sol and eol while emb_valid is low is not taken";
    do_sfs(1'b0);
    do_sfs(1'b1);
    i_emb_sol     = 1'b1;
    i_emb_eol     = 1'b1;
    i_emb_vc      = 2'd0;
    i_emb_byte_en = 4'b1111;
    cycles(1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    cycles(1);
  endtask

  // Extractor: every len, offsets on each byte lane, line index 0 to 2, both channels.
  task automatic test_extract_fields();
    rec_t        r;
    int          l1;
    int          o1;
    int          n1;
    logic [31:0] e0 [0:1];
    logic [31:0] e1 [0:1];
    if (NX == 0) begin
      fail("test_extract_fields needs PARA_N_EXTRACT of at least 1");
      return;
    end
    i_expected_height = 16'd0;
    i_emb_extract_en  = 1'b1;
    for (int li = 0; li < 3; li++) begin
      for (int off = 0; off < 8; off++) begin
        for (int len = 1; len <= 4; len++) begin
          where = $sformatf("line_idx %0d, byte_offset %0d, len %0d", li, off, len);
          l1 = (li + 1) % 3;
          o1 = 7 - off;
          n1 = 5 - len;
          set_ext(0, li, off, len);
          set_ext(1, l1, o1, n1);
          do_sfs(1'b0);
          do_sfs(1'b1);
          for (int ln = 0; ln < 3; ln++) begin
            emb_line(1'b0, 12, 10 + ln);
            emb_line(1'b1, 12, 10 + ln);
          end
          do_sfe(1'b0);
          do_sfe(1'b1);
          for (int c = 0; c < 2; c++) begin
            e0[c] = emb_field(1'(c), off, len, 12, 10 + li);
            e1[c] = emb_field(1'(c), o1, n1, 12, 10 + l1);
          end
          expect_true("different data on each channel", e0[0] != e0[1]);
          get_rec(r);
          expect_eq("record vc", r.vc, 0);
          expect_rec_ext(r, 0, e0[0]);
          expect_rec_ext(r, 1, e1[0]);
          get_rec(r);
          expect_eq("record vc", r.vc, 1);
          expect_rec_ext(r, 0, e0[1]);
          expect_rec_ext(r, 1, e1[1]);
          drain();
          settle();
          expect_pub_ext(1'b0, 0, e0[0]);
          expect_pub_ext(1'b0, 1, e1[0]);
          expect_pub_ext(1'b1, 0, e0[1]);
          expect_pub_ext(1'b1, 1, e1[1]);
          sent_q.delete();
          pay_q.delete();
        end
      end
    end
  endtask

  // One capture: nlines embedded lines of nbytes on both channels, line ln with salt base + ln.
  task automatic ext_frame(input int base, input int nlines, input int nbytes);
    do_sfs(1'b0);
    do_sfs(1'b1);
    for (int ln = 0; ln < nlines; ln++) begin
      emb_line(1'b0, nbytes, base + ln);
      emb_line(1'b1, nbytes, base + ln);
    end
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    clear_streams();
    settle();
  endtask

  task automatic ext_expect(input int k, input int salt, input int off, input int len,
                            input int nbytes);
    expect_pub_ext(1'b0, k, emb_field(1'b0, off, len, nbytes, salt));
    expect_pub_ext(1'b1, k, emb_field(1'b1, off, len, nbytes, salt));
  endtask

  // Extractor corner cases and its enable.
  task automatic test_extract_corners();
    if (NX == 0) begin
      fail("test_extract_corners needs PARA_N_EXTRACT of at least 1");
      return;
    end
    i_expected_height = 16'd0;
    i_emb_extract_en  = 1'b1;
    where = "a first value";
    set_ext(0, 0, 2, 2);
    ext_frame(20, 1, 12);
    ext_expect(0, 20, 2, 2, 12);

    where = "len = 0 disables the slot: the value stays";
    set_ext(0, 0, 2, 0);
    ext_frame(30, 1, 12);
    ext_expect(0, 20, 2, 2, 12);

    for (int l = 5; l <= 7; l++) begin
      where = $sformatf("len %0d acts as 4", l);
      set_ext(0, 0, 1, l);
      ext_frame(30 + l, 1, 12);
      ext_expect(0, 30 + l, 1, 4, 12);
    end

    where = "a field across a word boundary";
    set_ext(0, 0, 2, 4);
    ext_frame(40, 1, 12);
    ext_expect(0, 40, 2, 4, 12);
    set_ext(0, 0, 3, 2);
    ext_frame(41, 1, 12);
    ext_expect(0, 41, 3, 2, 12);
    set_ext(0, 0, 3, 4);
    ext_frame(42, 1, 12);
    ext_expect(0, 42, 3, 4, 12);

    where = "a field past the end of the line: cleared at the match, the bytes so far stay";
    set_ext(0, 0, 0, 4);
    ext_frame(43, 1, 8);
    ext_expect(0, 43, 0, 4, 8);
    set_ext(0, 0, 6, 4);
    ext_frame(44, 1, 8);
    ext_expect(0, 44, 6, 4, 8);

    where = "a field starting in a partial final beat";
    set_ext(0, 0, 8, 4);
    ext_frame(45, 1, 9);
    ext_expect(0, 45, 8, 4, 9);

    where = "an offset beyond the line: no match, the value stays";
    set_ext(0, 0, 20, 2);
    ext_frame(46, 1, 8);
    ext_expect(0, 45, 8, 4, 9);

    where = "a line index beyond the frame: no match, the value stays";
    set_ext(0, 200, 0, 2);
    ext_frame(47, 3, 8);
    ext_expect(0, 45, 8, 4, 9);

    where = "emb_extract_en = 0: counted and forwarded, the value stays";
    set_ext(0, 0, 0, 4);
    i_emb_extract_en = 1'b0;
    ext_frame(48, 2, 8);
    ext_expect(0, 45, 8, 4, 9);
    expect_eq("emb_line_count", pub_elines(1'b0), 2);
    expect_eq("emb_word_count", pub_ewords(1'b0), 4);
    i_emb_extract_en = 1'b1;

    if (NX >= 2) begin
      where = "two overlapping descriptors";
      set_ext(0, 1, 1, 3);
      set_ext(1, 1, 2, 2);
      ext_frame(50, 2, 8);
      ext_expect(0, 51, 1, 3, 8);
      ext_expect(1, 51, 2, 2, 8);
      set_ext(1, 0, 0, 0);
    end

    // A line cut before emb_eol is not counted, so the line index does not advance.
    where = "a cut line keeps the line index";
    set_ext(0, 1, 0, 2);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 8, 60, 0, 1'b1);
    emb_line(1'b0, 8, 61);
    emb_line(1'b0, 8, 62);
    emb_line(1'b1, 8, 60);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    settle();
    expect_eq("emb_line_count", pub_elines(1'b0), 2);
    expect_eq("emb_word_count", pub_ewords(1'b0), 6);
    expect_pub_ext(1'b0, 0, emb_field(1'b0, 0, 2, 8, 62));
    sent_q.delete();
    pay_q.delete();

    where = "the line after a cut one matches the same index again";
    set_ext(0, 0, 0, 2);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 8, 63, 0, 1'b1);
    emb_line(1'b0, 8, 64);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    settle();
    expect_pub_ext(1'b0, 0, emb_field(1'b0, 0, 2, 8, 64));
    sent_q.delete();
    pay_q.delete();

    where = "a field cut by a line without emb_eol keeps the bytes taken";
    set_ext(0, 0, 6, 4);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 8, 65, 0, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    settle();
    expect_pub_ext(1'b0, 0, emb_field(1'b0, 6, 4, 8, 65));
    clear_streams();
  endtask

  // Extracted values hold through frames without a match; resets clear them.
  task automatic test_extract_hold();
    rec_t        r;
    logic [31:0] v0;
    logic [31:0] v1;
    logic [31:0] w0;
    if (NX == 0) begin
      fail("test_extract_hold needs PARA_N_EXTRACT of at least 1");
      return;
    end
    i_expected_height = 16'd0;
    i_emb_extract_en  = 1'b1;
    set_ext(0, 0, 0, 4);
    set_ext(1, 0, 4, 4);

    where = "a frame with values";
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 12, 70);
    do_sfe(1'b0);
    do_sfe(1'b1);
    v0 = emb_field(1'b0, 0, 4, 12, 70);
    v1 = emb_field(1'b0, 4, 4, 12, 70);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, 0, 0, 3, 1, 6'd0);
    expect_rec_ext(r, 0, v0);
    expect_rec_ext(r, 1, v1);
    get_rec(r);
    drain();
    settle();
    expect_pub_ext(1'b0, 0, v0);
    expect_pub_ext(1'b0, 1, v1);
    pay_q.delete();
    sent_q.delete();

    where = "a frame without embedded lines keeps the values, the counters read 0";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
    expect_rec_ext(r, 0, v0);
    expect_rec_ext(r, 1, v1);
    get_rec(r);
    drain();
    settle();
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    expect_pub_ext(1'b0, 0, v0);
    expect_pub_ext(1'b0, 1, v1);

    where = "a generator frame carries the held values";
    i_active_sel = 1'b1;
    cycles(1);
    do_tfs(1'b0);
    do_tfs(1'b1);
    do_tfe(1'b0);
    do_tfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd3, 0, 0, 0, 0, 6'd0);
    expect_rec_ext(r, 0, v0);
    expect_rec_ext(r, 1, v1);
    get_rec(r);
    drain();
    i_active_sel = 1'b0;
    cycles(1);

    where = "a dropped capture: the record shows the new value, the published value holds";
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 12, 71);
    w0 = emb_field(1'b0, 0, 4, 12, 71);
    do_drop(2'b11);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec_ext(r, 0, w0);
    get_rec(r);
    drain();
    settle();
    expect_pub_ext(1'b0, 0, v0);
    expect_pub_ext(1'b0, 1, v1);
    pay_q.delete();
    sent_q.delete();

    where = "soft_reset clears the values";
    soft_reset_pulse();
    settle();
    expect_pub_ext(1'b0, 0, 32'd0);
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec_ext(r, 0, 32'd0);
    expect_rec_ext(r, 1, 32'd0);
    get_rec(r);
    drain();

    where = "resetn clears the values";
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 12, 72);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    clear_streams();
    settle();
    expect_true("value present", pub_ext(1'b0, 0) != 32'd0);
    i_resetn = 1'b0;
    cycles(2);
    i_resetn = 1'b1;
    cycles(2);
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec_ext(r, 0, 32'd0);
    get_rec(r);
  endtask

  // Publication: each channel at its own close; not for dropped or orphan frames; the
  // generator path loads only the timestamp.
  task automatic test_publication();
    logic [63:0] ts1;
    logic [63:0] ts2;
    where = "normal capture: each channel at its own close";
    set_sfs(1'b0);
    settle();
    ts1 = exp_ts();
    cycles(1);
    do_sfs(1'b1);
    emb_line(1'b0, 8, 1);
    emb_line(1'b1, 4, 1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B12, 1'b0);
    set_sfe(1'b0);
    settle();
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    cycles(1);
    settle();
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 2, 1);
    expect_pub(1'b1, 0, 0, 6'd0, 1'b0, 1'b0, 0, 0);
    set_sfe(1'b1);
    cycles(1);
    settle();
    expect_pub(1'b1, H_DEF, B12, DT_RAW12, 1'b0, 1'b0, 1, 1);
    expect_end(2'b01, 2'b00);
    expect_eq("timestamp at capture_done", o_timestamp, 0);
    cycles(1);
    settle();
    expect_eq("timestamp after capture_done", o_timestamp, ts1);
    expect_eq("frame_count", o_frame_count, 1);
    drain();
    clear_streams();

    where = "dropped capture: nothing published";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, 40, 1'b1);
    do_drop(2'b11);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b01);
    cycles(1);
    settle();
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 2, 1);
    expect_pub(1'b1, H_DEF, B12, DT_RAW12, 1'b0, 1'b0, 1, 1);
    expect_eq("timestamp holds", o_timestamp, ts1);
    expect_eq("frame_count counts it", o_frame_count, 2);
    drain();
    rec_q.delete();

    where = "orphan frame: nothing published";
    do_sfs(1'b1);
    bp_line(1'b1, 40, 1'b1);
    do_sfe(1'b1);
    cycles(1);
    settle();
    expect_pub(1'b1, H_DEF, B12, DT_RAW12, 1'b0, 1'b0, 1, 1);
    drain();
    expect_no_rec();

    where = "ctx_sync_close publishes";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, 40, 1'b1);
    do_sfs(1'b0);
    settle();
    expect_pub(1'b0, 1, 40, DT_RAW10, 1'b1, 1'b1, 0, 0);
    drain();
    rec_q.delete();
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    settle();

    where = "generator frame: channel registers hold, the timestamp is loaded";
    expect_eq("VC0 published before", pub_bytes(1'b0), 0);
    i_active_sel = 1'b1;
    cycles(1);
    set_tfs(1'b0);
    settle();
    ts2 = exp_ts();
    cycles(1);
    do_tfs(1'b1);
    do_tfe(1'b0);
    do_tfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_eq("timestamp", o_timestamp, ts2);
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b1, 0, 0);
    drain();
    rec_q.delete();
  endtask

  // Record layout word by word, with every err_summary bit.
  task automatic test_record_layout();
    rec_t        r;
    logic [63:0] ts0;
    logic [63:0] ts1;
    logic [31:0] e0;
    logic [31:0] e1;
    where = "two slots with different identity and geometry";
    i_emb_extract_en = 1'b1;
    set_ext(0, 0, 1, 3);
    set_ext(1, 1, 0, 4);
    i_metadata_record_id = 32'h1111_2222;
    set_sfs(1'b0);
    settle();
    ts0 = exp_ts();
    cycles(1);
    i_metadata_record_id = 32'h3333_4444;
    do_sfs(1'b1);
    emb_line(1'b0, 20, 80);
    e0 = emb_field(1'b0, 1, 3, 20, 80);
    emb_line(1'b0, 3, 81);
    e1 = emb_field(1'b0, 0, 4, 3, 81);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, 5, 44, 1'b1);
    do_sfe(1'b0);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, H_DEF, B10, 6, 2, DT_RAW10);
    expect_rec_id(r, ts0, 32'h1111_2222);
    expect_rec_ext(r, 0, e0);
    expect_rec_ext(r, 1, e1);
    drain();
    pay_q.delete();
    sent_q.delete();
    set_sfs(1'b0);
    settle();
    ts1 = exp_ts();
    expect_eq("second slot", o_capture_start, 2'b10);
    cycles(1);
    i_metadata_record_id = 32'h5555_6666;
    bp_lines(1'b0, H_DEF, B12, 1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b1, errs(0, 0, 1, 1, 0), 14'd1, 5, 44, 0, 0, DT_RAW10);
    expect_rec_id(r, ts0, 32'h1111_2222);
    expect_rec_ext(r, 0, 32'd0);
    expect_rec_ext(r, 1, 32'd0);
    do_sfe(1'b0);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, H_DEF, B12, 0, 0, DT_RAW12);
    expect_rec_id(r, ts1, 32'h3333_4444);
    expect_rec_ext(r, 0, e0);
    expect_rec_ext(r, 1, e1);
    drain();
    do_sfs(1'b1);
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    i_emb_extract_en = 1'b0;

    where = "aborted";
    i_expected_height = 16'd0;
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_drop(2'b10);
    do_sfe(1'b1);
    do_sfe(1'b0);
    get_rec(r);
    expect_rec(r, 1'b1, errs(1, 0, 0, 0, 0), 14'd3, 0, 0, 0, 0, 6'd0);
    drain();
    rec_q.delete();

    where = "fnum_err on VC1 only";
    i_fnum_check_en = 1'b1;
    do_sfsn(1'b0, 16'd1);
    do_sfsn(1'b1, 16'd9);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, r.w[1][13:0], 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, errs(0, 1, 0, 0, 0), r.w[1][13:0], 0, 0, 0, 0, 6'd0);
    drain();
    i_fnum_check_en = 1'b0;

    where = "width_mismatch only";
    i_expected_height = 16'd1;
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, 16, 1'b1);
    bp_line(1'b1, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, errs(0, 0, 1, 0, 0), r.w[1][13:0], 1, 16, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, r.w[1][13:0], 1, B10, 0, 0, DT_RAW10);
    drain();

    where = "height_mismatch only";
    i_expected_height = 16'd2;
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, B10, 1'b1);
    bp_lines(1'b1, 2, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, errs(0, 0, 0, 1, 0), r.w[1][13:0], 1, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, r.w[1][13:0], 2, B10, 0, 0, DT_RAW10);
    drain();

    where = "every flag together on VC0, meta_overflow on VC1, words in order under a stall";
    i_expected_height = 16'd2;
    i_fnum_check_en   = 1'b1;
    ready_mode = 1;
    cycles(1);
    do_sfsn(1'b0, 16'd40);
    do_sfs(1'b1);
    emb_line(1'b0, DEPTH * 4, 82);
    emb_line(1'b0, 8, 83);
    bp_line(1'b0, 16, 1'b1);
    do_drop(2'b01);
    do_sfe(1'b0);
    ready_mode = 2;
    ready_pct  = 30;
    get_rec(r);
    expect_rec(r, 1'b0, errs(1, 1, 1, 1, 1), r.w[1][13:0], 1, 16, DEPTH + 2, 2, DT_RAW10);
    cycles(RW + 3);
    drain();
    pay_q.delete();
    sent_q.delete();
    i_fnum_check_en = 1'b0;
    ready_mode = 1;
    cycles(1);
    emb_line(1'b1, DEPTH * 4, 84);
    emb_line(1'b1, 8, 85);
    bp_lines(1'b1, 2, B10, 1'b1);
    do_sfe(1'b1);
    ready_mode = 3;
    ready_lo   = 3;
    ready_hi   = 1;
    ready_cnt  = 0;
    get_rec(r);
    expect_rec(r, 1'b1, errs(1, 0, 0, 0, 1), r.w[1][13:0], 2, B10, DEPTH + 2, 2, DT_RAW10);
    drain();
    clear_streams();
  endtask

  // Test Pattern Generator records: zero geometry and counters, held extracted values.
  task automatic test_tpg_record();
    rec_t        r0;
    rec_t        r1;
    logic [63:0] ts;
    logic [31:0] v0;
    logic [31:0] v1;
    int          nd;
    int          nf;
    int          nr;
    where = "a sensor frame first";
    i_fnum_check_en  = 1'b1;
    i_emb_extract_en = 1'b1;
    set_ext(0, 0, 0, 4);
    set_ext(1, 0, 4, 2);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 16, 90);
    emb_line(1'b1, 16, 90);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    clear_streams();
    v0 = emb_field(1'b0, 0, 4, 16, 90);
    v1 = emb_field(1'b0, 4, 2, 16, 90);
    settle();
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 4, 1);

    where = "generator frames carry zero geometry and counters";
    nd = n_done;
    nf = n_err_fnum;
    nr = n_err_res;
    i_active_sel         = 1'b1;
    i_tpg_dual_vc_en     = 1'b1;
    i_metadata_record_id = 32'h7E57_0001;
    cycles(1);
    set_tfs(1'b0);
    settle();
    ts = exp_ts();
    cycles(1);
    settle();
    expect_eq("ctx_open clears on the generator path", cur_line(1'b0), 0);
    do_tfs(1'b1);
    settle();
    expect_eq("ctx_open clears on the generator path", cur_line(1'b1), 0);
    // Beats during a generator frame add nothing and are not forwarded.
    bp_line(1'b0, 40, 1'b1);
    emb_line(1'b0, 8, 91);
    bp_line(1'b1, 40, 1'b1);
    emb_line(1'b1, 8, 91);
    sent_q.delete();
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 0);
    do_tfe(1'b0);
    do_tfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_eq("capture_done", n_done, nd + 1);
    expect_eq("timestamp", o_timestamp, ts);
    get_rec(r0);
    get_rec(r1);
    expect_rec(r0, 1'b0, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
    expect_rec(r1, 1'b1, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
    expect_rec_id(r0, ts, 32'h7E57_0001);
    expect_rec_id(r1, ts, 32'h7E57_0001);
    expect_rec_ext(r0, 0, v0);
    expect_rec_ext(r0, 1, v1);
    expect_eq("no payload", pay_q.size(), 0);
    expect_eq("no err_frame_num", n_err_fnum, nf);
    expect_eq("no err_resolution", n_err_res, nr);

    where = "the published channel registers hold";
    drain();
    settle();
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 4, 1);
    expect_pub(1'b1, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 4, 1);
    expect_pub_ext(1'b0, 0, v0);

    where = "a sensor frame afterwards restarts from 0";
    i_active_sel = 1'b0;
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    settle();
    expect_eq("cur_line_vc0", cur_line(1'b0), 0);
    bp_lines(1'b0, H_DEF, B12, 1'b0);
    bp_lines(1'b1, H_DEF, B12, 1'b0);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r0);
    expect_rec(r0, 1'b0, 5'd0, 14'd3, H_DEF, B12, 0, 0, DT_RAW12);
    get_rec(r1);
    expect_rec(r1, 1'b1, 5'd0, 14'd3, H_DEF, B12, 0, 0, DT_RAW12);
    expect_eq("no err_resolution", n_err_res, nr);
  endtask

  // Backpressure: exactly META_FIFO_DEPTH words before loss, meta_sof moved by a loss, payload
  // priority over record words, capture events independent of meta_ready.
  task automatic test_backpressure();
    rec_t        r;
    mword_t      a;
    mword_t      b;
    logic [31:0] fc;
    i_expected_height = 16'd0;
    where = "exactly META_FIFO_DEPTH words are accepted before loss";
    ready_mode = 1;
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, DEPTH * 4, 1, 0, 1'b1);
    settle();
    expect_eq("nothing lost yet", o_meta_overflow, 0);
    expect_eq("meta_valid", o_meta_valid, 1);
    emb_line(1'b0, 12, 2);
    settle();
    expect_eq("meta_overflow", o_meta_overflow, 1);
    ready_mode = 0;
    cycles(DEPTH + 2);
    settle();
    expect_eq("words accepted", pay_q.size(), DEPTH);
    for (int i = 0; i < DEPTH; i++) begin
      a = pay_q.pop_front();
      b = sent_q.pop_front();
      if ({a.data, a.be, a.last, a.vc} !== {b.data, b.be, b.last, b.vc})
        fail($sformatf("kept word %0d differs", i));
      expect_eq("meta_sof", a.sof, (i == 0) ? 1'b1 : 1'b0);
    end
    sent_q.delete();
    exp_sof[0] = 1'b0;
    expect_eq("meta_valid after the FIFO emptied", o_meta_valid, 0);

    where = "err_summary of that frame; emb_word_count counts the lost beats";
    do_sfe(1'b0);
    settle();
    expect_pub(1'b0, 0, 0, 6'd0, 1'b0, 1'b0, DEPTH + 3, 1);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, errs(0, 0, 0, 0, 1), 14'd1, 0, 0, DEPTH + 3, 1, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    drain();

    where = "the next frame is clear, meta_overflow stays";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    drain();
    settle();
    expect_eq("meta_overflow is sticky", o_meta_overflow, 1);
    soft_reset_pulse();
    settle();
    expect_eq("cleared by soft_reset", o_meta_overflow, 0);

    where = "a first word lost moves meta_sof to the next word written";
    ready_mode = 1;
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, DEPTH * 4, 5);
    emb_line(1'b1, 8, 5);
    ready_mode = 0;
    cycles(DEPTH + 2);
    settle();
    expect_eq("words accepted", pay_q.size(), DEPTH);
    a = pay_q.pop_front();
    expect_eq("VC0 first word", a.sof, 1);
    pay_q.delete();
    sent_q.delete();
    emb_line(1'b1, 8, 6);
    cycles(3);
    expect_eq("words accepted", pay_q.size(), 2);
    a = pay_q.pop_front();
    expect_eq("the flag moved to this VC1 word", a.sof, 1);
    expect_eq("meta_vc", a.vc, 1);
    a = pay_q.pop_front();
    expect_eq("once per frame", a.sof, 0);
    sent_q.delete();

    where = "payload has priority during a record, the record words stay in order";
    n_interleave = 0;
    set_sfe(1'b0);
    cycles(1);
    cycles(3);
    emb_line(1'b1, 16, 7);
    cycles(1);
    emb_line(1'b1, 16, 8);
    get_rec(r);
    expect_eq("record of VC0", r.vc, 0);
    expect_eq("frame_id", r.w[1], 3);
    expect_true("payload entered between record words", n_interleave > 0);
    drain();
    expect_eq("payload words", pay_q.size(), 8);
    pay_q.delete();
    sent_q.delete();

    where = "payload at full rate with meta_ready high loses nothing";
    do_sfe(1'b1);
    drain();
    rec_q.delete();
    soft_reset_pulse();
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 40, 9);
    emb_line(1'b1, 40, 9);
    settle();
    expect_eq("meta_overflow", o_meta_overflow, 0);
    drain();
    expect_payload();
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();

    where = "stalled records are written once the FIFO has room; the capture events do not wait";
    settle();
    fc = o_frame_count;
    ready_mode = 1;
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, DEPTH * 4, 10);
    sent_q.delete();
    do_sfe(1'b0);
    do_sfe(1'b1);
    cycles(RW + 4);
    settle();
    expect_no_rec();
    expect_eq("frame_count", o_frame_count, fc + 1);
    ready_mode = 0;
    get_rec(r);
    expect_eq("record of VC0", r.vc, 0);
    get_rec(r);
    expect_eq("record of VC1", r.vc, 1);
    drain();
    pay_q.delete();
  endtask

  // The record queue is full or holds a record of the closing channel.
  task automatic test_record_queue_full();
    rec_t r;
    where = "two closes queue, a close of a queued channel does not";
    ready_mode = 1;
    cycles(1);
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_line(1'b0, 40, 1'b1);
    bp_lines(1'b1, 2, 44, 1'b1);
    // A full FIFO keeps both records in the queue.
    emb_line(1'b0, DEPTH * 4, 1);
    sent_q.delete();
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    expect_eq("nothing refused yet", o_meta_overflow, 0);
    cycles(1);

    where = "slot 0 stays allocated while its records wait";
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b10);
    cycles(1);
    do_sfs(1'b1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);

    where = "the refused close: publication still takes place, meta_overflow set";
    set_sfe(1'b0);
    settle();
    expect_eq("meta_overflow before the edge", o_meta_overflow, 0);
    cycles(1);
    settle();
    expect_eq("meta_overflow", o_meta_overflow, 1);
    expect_pub(1'b0, H_DEF, B10, DT_RAW10, 1'b0, 1'b0, 0, 0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b10, 2'b00);
    cycles(1);

    where = "the older records are intact, the refused ones never appear";
    ready_mode = 0;
    get_rec(r);
    expect_rec(r, 1'b0, errs(0, 0, 1, 1, 0), 14'd1, 1, 40, DEPTH, 1, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, errs(0, 0, 1, 1, 0), 14'd1, 2, 44, 0, 0, DT_RAW10);
    drain();
    expect_no_rec();
    pay_q.delete();

    where = "slot 0 is free again";
    i_expected_height = 16'd0;
    set_sfs(1'b0);
    settle();
    expect_eq("capture_start", o_capture_start, 2'b01);
    cycles(1);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    rec_q.delete();

    // The record leaving in ST_POP still counts as queued. The model state paces the stimulus.
    where = "a close of the same channel in the ST_POP cycle is refused";
    soft_reset_pulse();
    i_active_sel     = 1'b1;
    i_tpg_dual_vc_en = 1'b0;
    cycles(2);
    do_tfs(1'b0);
    do_tfe(1'b0);
    do_tfs(1'b0);
    while (m_fsm != FSM_POP) cycles(1);
    settle();
    expect_eq("nothing refused yet", o_meta_overflow, 0);
    do_tfe(1'b0);
    settle();
    expect_eq("meta_overflow", o_meta_overflow, 1);
    drain();
    get_rec(r);
    expect_eq("the first record", r.w[1], 4);
    expect_no_rec();

    where = "a close of the other channel in the ST_POP cycle is queued";
    soft_reset_pulse();
    i_tpg_dual_vc_en = 1'b1;
    cycles(2);
    do_tfs(1'b0);
    do_tfs(1'b1);
    do_tfe(1'b0);
    while (m_fsm != FSM_POP) cycles(1);
    do_tfe(1'b1);
    settle();
    expect_eq("not refused", o_meta_overflow, 0);
    expect_end(2'b01, 2'b00);
    cycles(1);
    // The new entry names the same slot, so the slot stays held.
    set_tfs(1'b0);
    settle();
    expect_eq("the slot is still held", o_capture_start, 2'b10);
    cycles(1);
    get_rec(r);
    expect_eq("first record", r.vc, 0);
    get_rec(r);
    expect_eq("second record", r.vc, 1);
    expect_eq("same capture", r.w[1], 6);
    do_tfe(1'b0);
    drain();
    rec_q.delete();
  endtask

  // The meta_* stream under meta_ready patterns of every length.
  task automatic test_meta_stall();
    rec_t r;
    int   lows [0:5];
    int   fid;
    lows[0] = 1;
    lows[1] = 2;
    lows[2] = 3;
    lows[3] = 5;
    lows[4] = 8;
    lows[5] = 2 * DEPTH + 1;
    i_expected_height = 16'd0;
    fid = 0;
    for (int l = 0; l < 6; l++) begin
      for (int h = 1; h <= 3; h++) begin
        where      = $sformatf("meta_ready low %0d, high %0d", lows[l], h);
        ready_mode = 3;
        ready_lo   = lows[l];
        ready_hi   = h;
        ready_cnt  = 0;
        do_sfs(1'b0);
        do_sfs(1'b1);
        fid++;
        emb_line(1'b0, 20, l + h, 0, 1'b0, (lows[l] + h) * 2);
        emb_line(1'b1, 11, l + h, 0, 1'b0, (lows[l] + h) * 2);
        do_sfe(1'b0);
        get_rec(r);
        expect_rec(r, 1'b0, 5'd0, 14'(fid), 0, 0, 5, 1, 6'd0);
        do_sfe(1'b1);
        get_rec(r);
        expect_rec(r, 1'b1, 5'd0, 14'(fid), 0, 0, 3, 1, 6'd0);
        drain();
        settle();
        expect_eq("nothing lost", o_meta_overflow, 0);
        expect_payload();
      end
    end

    where = "random meta_ready, one long stall inside a record";
    for (int p = 10; p <= 90; p += 20) begin
      ready_mode = 2;
      ready_pct  = p;
      do_sfs(1'b0);
      do_sfs(1'b1);
      fid++;
      do_sfe(1'b0);
      do_sfe(1'b1);
      get_rec(r);
      expect_rec(r, 1'b0, 5'd0, 14'(fid), 0, 0, 0, 0, 6'd0);
      get_rec(r);
      expect_rec(r, 1'b1, 5'd0, 14'(fid), 0, 0, 0, 0, 6'd0);
      drain();
    end
    do_sfs(1'b0);
    do_sfs(1'b1);
    fid++;
    ready_mode = 1;
    do_sfe(1'b0);
    cycles(5);
    settle();
    expect_eq("meta_valid holds through a stall", o_meta_valid, 1);
    cycles(40);
    settle();
    expect_eq("meta_valid holds through a stall", o_meta_valid, 1);
    ready_mode = 0;
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'(fid), 0, 0, 0, 0, 6'd0);
    do_sfe(1'b1);
    get_rec(r);
  endtask

  // Record sequencing: two queued records leave whole and in order, with a gap for ST_POP.
  task automatic test_record_sequence();
    rec_t r;
    int   t_last;
    int   t_next;
    int   gap;
    i_expected_height = 16'd0;
    where = "two queued records leave in queue order";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b1);
    do_sfe(1'b0);
    // After the final word of the first record meta_valid is low for at least a cycle, since
    // ST_POP grants no word and meta_ready is high.
    t_last = -1;
    t_next = -1;
    gap    = 0;
    for (int i = 0; (i < 3 * RW + 20) && (t_next < 0); i++) begin
      settle();
      if (t_last >= 0 && o_meta_valid === 1'b1) t_next = i;
      else if (t_last >= 0 && o_meta_valid === 1'b0) gap++;
      if (t_last < 0 && o_meta_valid === 1'b1 && o_meta_kind === 1'b0 && o_meta_last === 1'b1)
        t_last = i;
      cycles(1);
    end
    expect_true("the second record began", t_next >= 0);
    expect_true("a cycle without a record word between the records", gap >= 1);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, 0, 0, 0, 0, 6'd0);
    drain();

    where = "one record per close of a joined channel, none for an orphan";
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd2, 0, 0, 0, 0, 6'd0);
    drain();
    expect_no_rec();
    do_sfs(1'b1);
    do_sfe(1'b1);
    drain();
    expect_no_rec();

    where = "two records with payload around them";
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 8, 7);
    do_sfe(1'b0);
    emb_line(1'b1, 4, 7);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd3, 0, 0, 2, 1, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd3, 0, 0, 1, 1, 6'd0);
    drain();
    expect_payload();
  endtask

  // PARA_N_EXTRACT = 0: 7-word records, value ports read 0, descriptor inputs ignored.
  task automatic test_no_extractor();
    rec_t r;
    if (NX != 0) begin
      fail("test_no_extractor needs PARA_N_EXTRACT = 0");
      return;
    end
    expect_eq("value port is one slot wide", $bits(o_emb_extract_val_vc0), 32);
    expect_eq("value port is one slot wide", $bits(o_emb_extract_val_vc1), 32);
    expect_eq("descriptor port is one slot wide", $bits(i_emb_extract_cfg), 32);
    i_expected_height = 16'd0;
    i_emb_extract_en  = 1'b1;
    i_emb_extract_cfg = {8'd0, 16'd0, 5'd0, 3'd4};
    where = "the extractor has no effect";
    do_sfs(1'b0);
    do_sfs(1'b1);
    emb_line(1'b0, 12, 9);
    emb_line(1'b1, 12, 9);
    do_sfe(1'b0);
    do_sfe(1'b1);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, 0, 0, 3, 1, 6'd0);
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, 0, 0, 3, 1, 6'd0);
    drain();
    settle();
    expect_eq("emb_extract_val_vc0", o_emb_extract_val_vc0, 0);
    expect_eq("emb_extract_val_vc1", o_emb_extract_val_vc1, 0);
    expect_payload();
    // The port monitor fails unless meta_last marks word 6 of each record.
    expect_no_rec();
  endtask

  // Drop of an orphan channel while both slots are open, in the cycle of its Frame End.
  task automatic test_drop_orphan_two_slots();
    rec_t r;
    where = "capture 1 in slot 0, both channels";
    do_sfs(1'b0);
    do_sfs(1'b1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b1);

    where = "lag Frame Start with no wait: VC1 is an orphan, slot 0 stays open for VC0";
    do_sfs(1'b1);
    settle();
    expect_eq("VC1 orphan", o_ctx_orphan[1], 1);
    expect_eq("cur_frame_id_vc1 of an orphan", cur_fid(1'b1), 0);
    expect_eq("no capture ends", o_capture_end, 2'b00);

    where = "lead Frame Start allocates slot 1; slot 0 ends in the next cycle";
    do_sfs(1'b0);
    settle();
    expect_eq("VC0 joined slot 1", o_ctx_slot[0], 1);
    expect_eq("cur_frame_id_vc0", cur_fid(1'b0), 2);

    where = "drop of the orphan VC1 with its Frame End: slot 1 is marked, slot 0 is not";
    set_drop(2'b10);
    set_sfe(1'b1);
    settle();
    expect_end(2'b01, 2'b00);
    cycles(1);
    settle();
    expect_eq("frame_count", o_frame_count, 1);

    where = "VC1 joins slot 1; slot 1 ends dropped";
    do_sfs(1'b1);
    settle();
    expect_eq("VC1 joined", o_ctx_orphan[1], 0);
    expect_eq("VC1 slot", o_ctx_slot[1], 1);
    bp_lines(1'b0, H_DEF, B10, 1'b1);
    bp_lines(1'b1, H_DEF, B10, 1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    settle();
    expect_end(2'b10, 2'b10);
    cycles(1);
    settle();
    expect_eq("frame_count", o_frame_count, 2);

    where = "records: capture 1 not aborted, capture 2 aborted on both channels";
    get_rec(r);
    expect_rec(r, 1'b1, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b0, 5'd0, 14'd1, H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b0, errs(1, 0, 0, 0, 0), 14'd2, H_DEF, B10, 0, 0, DT_RAW10);
    get_rec(r);
    expect_rec(r, 1'b1, errs(1, 0, 0, 0, 0), 14'd2, H_DEF, B10, 0, 0, DT_RAW10);
  endtask

  // Constrained random traffic of ncyc cycles against the model. The stimulus stays legal:
  // one frame event per cycle, beats only on a channel with a frame open and never in a frame
  // event cycle, configuration changes only with no frame open, dma_buf_claim one cycle after
  // capture_start. The model state selects legal stimulus only.
  int pe, pbeat, ppkt, pbad, pdrop, pce, psoft;

  task automatic set_profile(input int ready, input int ev, input int beat, input int pkt,
                             input int bad, input int drop, input int ce, input int soft_pm);
    ready_pct = ready;
    pe        = ev;
    pbeat     = beat;
    ppkt      = pkt;
    pbad      = bad;
    pdrop     = drop;
    pce       = ce;
    psoft     = soft_pm;
  endtask

  task automatic random_traffic(input int ncyc);
    int          phase_left;
    int          ce_left;
    int          pkt_left;
    bit          pkt_on;
    bit          pkt_emb;
    bit          pkt_vc;
    bit          pkt_raw;
    bit          pkt_first;
    bit          pkt_noeol;
    bit          beat;
    bit          cfg_cycle;
    bit          lead;
    bit          vc;
    bit          is_fs;
    int          ev;
    int          k;
    int          w;
    logic [15:0] fn;
    where      = "random";
    phase_left = 0;
    set_profile(100, 3, 90, 30, 1, 1, 0, 0);
    ce_left    = 0;
    pkt_on     = 1'b0;
    pkt_left   = 0;
    ready_mode = 2;
    for (int cy = 0; cy < ncyc; cy++) begin
      // Traffic profile of the next few hundred cycles.
      if (phase_left == 0) begin
        phase_left = rand_range(200, 3000);
        // Profiles: meta_ready %, frame event %, beat %, packet start %, bad event, drop
        // and core_enable pulse per mille, soft_reset per mille.
        case (rand_range(0, 5))
          0:       set_profile(100, 3,  90, 30, 1,  1,  0, 0);
          1:       set_profile(50,  5,  70, 40, 5,  5,  1, 0);
          2:       set_profile(10,  5,  90, 60, 5,  5,  1, 0);
          3:       set_profile(100, 30, 30, 10, 20, 20, 5, 1);
          4:       set_profile(30,  30, 30, 10, 20, 10, 0, 0);
          default: begin
            set_profile(0, 20, 50, 30, 5, 5, 0, 0);
            phase_left = rand_range(50, 300);
          end
        endcase
      end
      phase_left--;
      i_soft_reset = 1'b0;
      cfg_cycle    = 1'b0;

      // core_enable low pulses of one to six cycles.
      if (ce_left > 0) begin
        ce_left--;
        if (ce_left == 0) i_core_enable = 1'b1;
      end else if (rand_pm(pce)) begin
        i_core_enable = 1'b0;
        ce_left       = rand_range(1, 6);
      end
      if (rand_pm(psoft)) begin
        i_soft_reset = 1'b1;
        pkt_on       = 1'b0;
      end

      // Inputs sampled at an event may change in any cycle.
      if (rand_pct(1)) i_metadata_record_id = $urandom;
      if (rand_pct(1)) begin
        i_expected_width  = 16'(rand_range(32, 67));
        i_expected_height = 16'(rand_range(0, 3));
      end

      // The other configuration changes only with no frame open and no packet in flight.
      if (m_active == 2'b00 && !pkt_on && ce_left == 0 && rand_pct(2)) begin
        i_vc_exposure_map     = rand_pct(50);
        i_active_sel          = rand_pct(25);
        i_tpg_dual_vc_en      = rand_pct(70);
        i_metadata_capture_en = rand_pct(90);
        i_emb_extract_en      = rand_pct(70);
        i_fnum_check_en       = rand_pct(60);
        for (int i = 0; i < NP; i++)
          set_ext(i, rand_range(0, 2), rand_range(0, 12), rand_range(0, 7));
        cfg_cycle = 1'b1;
      end

      // One long packet at a time, on a channel with a frame open.
      if (!pkt_on && m_active != 2'b00 && rand_pct(ppkt)) begin
        pkt_on    = 1'b1;
        pkt_vc    = (m_active == 2'b11) ? rand_pct(50) : m_active[1];
        pkt_emb   = rand_pct(35);
        pkt_raw   = rand_pct(50);
        pkt_first = 1'b1;
        pkt_noeol = rand_pct(5);
        w         = int'(i_expected_width);
        if (pkt_noeol)        pkt_left = 4 * rand_range(1, 8);
        else if (pkt_emb)     pkt_left = rand_range(1, 40);
        else if (rand_pct(70)) pkt_left = pkt_raw ? ((w * 5) >> 2) : ((w * 3) >> 1);
        else                  pkt_left = rand_range(1, 64);
      end
      beat = 1'b0;
      if (pkt_on && rand_pct(pbeat)) begin
        k = (pkt_left >= 4) ? 4 : pkt_left;
        if (pkt_emb) begin
          i_emb_valid   = 1'b1;
          i_emb_data    = $urandom;
          i_emb_byte_en = be_of(k);
          i_emb_vc      = {1'b0, pkt_vc};
          i_emb_sol     = pkt_first;
          i_emb_eol     = (pkt_left <= 4) && !pkt_noeol;
        end else begin
          i_bp_valid   = 1'b1;
          i_bp_byte_en = be_of(k);
          i_bp_vc_tag  = {1'b0, pkt_vc};
          i_bp_dt_sel  = pkt_raw;
          i_bp_sol     = pkt_first;
          i_bp_eol     = (pkt_left <= 4) && !pkt_noeol;
        end
        pkt_left  = pkt_left - k;
        pkt_first = 1'b0;
        beat      = 1'b1;
        if (pkt_left == 0) pkt_on = 1'b0;
      end

      // Frame events of the selected source, never in a beat cycle.
      lead = i_active_sel ? 1'b0 : i_vc_exposure_map;
      ev   = 0;
      if (!m_active[lead] && rand_pct(2 * pe))                         ev = 1;
      else if (m_wait[m_last_alloc] && !m_active[!lead] && rand_pct(2 * pe)) ev = 2;
      else if (m_active[lead] && rand_pct(pe))                          ev = 3;
      else if (m_active[!lead] && rand_pct(pe))                         ev = 4;
      else if (rand_pm(pbad))                                           ev = rand_range(1, 4);
      if (ev != 0 && !beat && !cfg_cycle) begin
        vc    = (ev == 1 || ev == 3) ? lead : !lead;
        is_fs = (ev <= 2);
        fn    = fnum_next[vc];
        if (rand_pct(5))      fn = 16'($urandom);
        else if (rand_pct(5)) fn = 16'(rand_range(0, 1));
        if (i_active_sel) begin
          if (is_fs) set_tfs(vc);
          else       set_tfe(vc);
        end else begin
          if (is_fs) set_sfsn(vc, fn);
          else       set_sfe(vc);
        end
        // A line in flight on that channel ends here without its eol.
        if (pkt_on && pkt_vc == vc) pkt_on = 1'b0;
      end

      // Events of the other source have no effect.
      if (rand_pct(3) && !beat) begin
        vc    = rand_pct(50);
        is_fs = rand_pct(50);
        if (i_active_sel) begin
          if (is_fs) set_sfsn(vc, fnum_next[vc]);
          else       set_sfe(vc);
        end else begin
          if (is_fs) set_tfs(vc);
          else       set_tfe(vc);
        end
      end

      if (rand_pm(pdrop)) set_drop(2'(rand_range(0, 3)));
      // The DMA claims one cycle after capture_start.
      if (last_start != 2'b00 && rand_pct(75)) set_claim(rand_pct(50));
      cycles(1);
    end
    i_soft_reset  = 1'b0;
    i_core_enable = 1'b1;
    ready_mode    = 0;
    cycles(2);
    drain();
    clear_streams();
    expect_true("captures were exercised", n_start > 0);
  endtask

  task automatic test_random_traffic();
    random_traffic(100000);
  endtask

  task automatic test_random_long();
    random_traffic(500000);
  endtask

  // Forces a wrong frame_count after a capture; this test must end in FAIL with errors in the
  // log. It confirms that the checkers catch a fault and is not part of the regression.
  task automatic test_checker_fault();
    i_expected_height = 16'd0;
    do_sfs(1'b0);
    do_sfs(1'b1);
    do_sfe(1'b0);
    do_sfe(1'b1);
    drain();
    force o_frame_count = 32'd0;
    cycles(3);
    release o_frame_count;
    cycles(2);
  endtask

  // --------------------------------------------------------------------------------------------
  // Test selection and result
  // --------------------------------------------------------------------------------------------
  string       test_name;
  int unsigned seed;

  initial begin
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "none";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    void'($urandom(seed));
    i_resetn              = 1'b0;
    i_soft_reset          = 1'b0;
    i_core_enable         = 1'b1;
    i_metadata_capture_en = 1'b1;
    i_fnum_check_en       = 1'b0;
    i_emb_extract_en      = 1'b0;
    i_expected_width      = 16'(W_DEF);
    i_expected_height     = 16'(H_DEF);
    i_metadata_record_id  = '0;
    i_emb_extract_cfg     = '0;
    i_vc_exposure_map     = 1'b0;
    i_tpg_dual_vc_en      = 1'b1;
    i_active_sel          = 1'b0;
    idle_inputs();
    i_meta_ready          = 1'b1;
    model_reset(1'b0);
    $display("%s: PARA_N_EXTRACT=%0d PARA_META_FIFO_DEPTH=%0d PARA_TS_INCR=%h SEED=%0d",
             test_name, NX, DEPTH, PARA_TS_INCR, seed);
    #1.0;
    started = 1'b1;
    cycles(4);
    i_resetn = 1'b1;
    cycles(3);
    case (test_name)
      "test_reset_values":            test_reset_values();
      "test_soft_reset":              test_soft_reset();
      "test_sensor_event_select":     test_sensor_event_select();
      "test_tpg_single_vc":           test_tpg_single_vc();
      "test_tpg_dual_vc":             test_tpg_dual_vc();
      "test_basic_capture":           test_basic_capture();
      "test_short_open_at_next_long": test_short_open_at_next_long();
      "test_lag_fs_lost":             test_lag_fs_lost();
      "test_lead_fs_lost":            test_lead_fs_lost();
      "test_fe_lost":                 test_fe_lost();
      "test_fe_without_frame":        test_fe_without_frame();
      "test_slots_full":              test_slots_full();
      "test_drop_before_lag":         test_drop_before_lag();
      "test_drop_after_close":        test_drop_after_close();
      "test_drop_masks":              test_drop_masks();
      "test_core_enable":             test_core_enable();
      "test_active_sel_change":       test_active_sel_change();
      "test_vc2_vc3_events":          test_vc2_vc3_events();
      "test_frame_number_check":      test_frame_number_check();
      "test_frame_id_wrap":           test_frame_id_wrap();
      "test_timestamp":               test_timestamp();
      "test_frame_id_buf":            test_frame_id_buf();
      "test_cur_frame_id":            test_cur_frame_id();
      "test_geometry":                test_geometry();
      "test_line_without_eol":        test_line_without_eol();
      "test_resolution_mismatch":     test_resolution_mismatch();
      "test_err_resolution_suppress": test_err_resolution_suppress();
      "test_embedded_forwarding":     test_embedded_forwarding();
      "test_extract_fields":          test_extract_fields();
      "test_extract_corners":         test_extract_corners();
      "test_extract_hold":            test_extract_hold();
      "test_publication":             test_publication();
      "test_record_layout":           test_record_layout();
      "test_tpg_record":              test_tpg_record();
      "test_backpressure":            test_backpressure();
      "test_record_queue_full":       test_record_queue_full();
      "test_meta_stall":              test_meta_stall();
      "test_record_sequence":         test_record_sequence();
      "test_no_extractor":            test_no_extractor();
      "test_drop_orphan_two_slots":   test_drop_orphan_two_slots();
      "test_random_traffic":          test_random_traffic();
      "test_random_long":             test_random_long();
      "test_checker_fault":           test_checker_fault();
      default: begin
        err_count++;
        $error("unknown test '%s'", test_name);
      end
    endcase
    where = "end of test";
    drain();
    cycles(5);
    if (err_count == 0 && check_count >= 10) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d, checks=%0d)", test_name, err_count, check_count);
    $finish;
  end

  // Timeout.
  initial begin
    #100_000_000.0;
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
