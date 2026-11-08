// File        : tb_top.sv
// Description : Testbench of m_fpt_lane_management (lane deskew, lane merging, burst FSM).
//               A cycle-based reference model, written from the specification, predicts the FSM
//               state, the status outputs and the merged byte stream; a checker compares every
//               output with it at each byte clock cycle. A second, independent check compares the
//               bytes of each delivered burst with the payload the test sent, for every burst in
//               which no FIFO write was dropped. Tests only drive
//               stimulus and state the expected result of each burst.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions (agreed with the block owner; the specification does not state them yet):
//   - The bytes of a burst are spread over the active lanes in turn (lane 0, lane 1, ...). The
//     merged stream is lane 0 byte 0, lane 1 byte 0, ..., lane 0 byte 1, ...; the first byte of a
//     word is merged_data[7:0].
//   - rxsynchs, errsotsynchs and the first byte may arrive in the same cycle as the rising edge of
//     rxactivehs. The first byte comes with rxsynchs or in a later cycle.
//   - Each lane FIFO is first-word-fall-through. fifo_full of a lane is set while its fill level
//     is at least SKEW_DEPTH - FIFO_AF_MARGIN, and only counts on a lane that has synchronised.
//     A write into a FIFO that holds SKEW_DEPTH bytes is dropped, even if a byte is read in the
//     same cycle. A lane that runs about SKEW_DEPTH - 1 bytes ahead when the read starts (possible
//     with FIFO_AF_MARGIN = 1) therefore loses bytes; for such a burst the delivered bytes are
//     checked against the reference model only, not against the payload sent.
//   - errsotsynchs is taken from the stored per-lane flag, so it acts one cycle after the pulse.
//   - From ST_ARMING: errsotsynchs wins, then armed (which wins over fifo_full and over the burst
//     closing), then the other abort causes, then the silent return of a deskew burst. Every
//     cause present in the abort cycle is latched; abort_lane_sync_seen is the sync flags then.
//   - active_lanes 10 selects no lane and every burst is ignored. A lane outside the active set
//     has no effect. A selected lane that is not built never synchronises.
//   - active_lanes_applied resets to 00, is not cleared by soft_reset, and takes the new value in
//     any ST_IDLE cycle in which no burst opens.
//   - soft_reset acts like resetn at the next clock edge, except for active_lanes, and wins over
//     a burst that opens.
//   - A completed word is held until it is known whether it is the last one, so merged_eob is set
//     on the last word even when rxactivehs falls several cycles after the last byte. A burst that
//     synchronises but carries no byte produces no word.
//   - The first read in which an active lane has finished (rxactivehs low, FIFO empty) is the last
//     one placed in words: only the lanes below the lowest finished lane are placed; later bytes
//     of the burst are popped and dropped.
//   - lane_abort_pulse is high in the first ST_ABORT cycle; the abort marker follows one cycle
//     later. errsoths has no effect on the block.
//   - The gap between two bursts is longer than the time the FSM needs to drain or abort, and
//     merged_* has no backpressure.
//   - The exact cycle in which the FSM leaves ST_DRAIN is not specified: the checker accepts it
//     once all bytes have been sent, within 4 cycles. Words may lag the bytes placed by at most
//     7 bytes (one held word and one word being filled).
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_MAX_LANES      = 4;
  parameter logic [31:0] PARA_SKEW_DEPTH     = 8;
  parameter logic [31:0] PARA_FIFO_AF_MARGIN = 2;

  // Lane x is built when x < PARA_MAX_LANES.
  localparam logic [3:0] BUILT = (PARA_MAX_LANES == 32'd1) ? 4'b0001 :
                                 (PARA_MAX_LANES == 32'd2) ? 4'b0011 : 4'b1111;
  localparam int MAXN    = (PARA_MAX_LANES == 32'd1) ? 1 : (PARA_MAX_LANES == 32'd2) ? 2 : 4;
  localparam int DEPTH   = int'(PARA_SKEW_DEPTH);
  // Fill level at which fifo_full of a lane is set.
  localparam int THR     = int'(PARA_SKEW_DEPTH) - int'(PARA_FIFO_AF_MARGIN);
  // Largest lane-to-lane skew, in cycles, that never raises fifo_full before the burst is armed.
  localparam int SKEW_OK = (THR > 1) ? THR - 1 : 0;

  // Byte clock, 224 MHz.
  localparam real T_BYTE_CLK = 4.464;

  localparam logic [2:0] ST_IDLE    = 3'b000;
  localparam logic [2:0] ST_ARMING  = 3'b001;
  localparam logic [2:0] ST_ALIGNED = 3'b010;
  localparam logic [2:0] ST_DRAIN   = 3'b011;
  localparam logic [2:0] ST_ABORT   = 3'b100;

  // Abort causes as {abort_sync_err, abort_fifo_full, abort_unarmed}.
  localparam logic [2:0] C_NONE    = 3'b000;
  localparam logic [2:0] C_SYNC    = 3'b100;
  localparam logic [2:0] C_FULL    = 3'b010;
  localparam logic [2:0] C_UNARMED = 3'b001;

  // DUT inputs; the vectors carry one bit per lane.
  logic        i_rxbyteclkhs, i_resetn, i_soft_reset;
  logic [1:0]  i_active_lanes;
  logic [7:0]  i_rxdatahs [4];
  logic [3:0]  i_rxvalidhs, i_rxactivehs, i_rxsynchs, i_rxskewcalhs, i_errsoths, i_errsotsynchs;

  // DUT outputs.
  logic [31:0] o_merged_data;
  logic        o_merged_valid, o_merged_sob, o_merged_eob;
  logic [3:0]  o_merged_byte_valid;
  logic [2:0]  o_lm_fsm_state;
  logic        o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed;
  logic [3:0]  o_abort_lane_sync_seen;
  logic        o_lane_abort_pulse, o_deskew_burst_seen;
  logic [1:0]  o_active_lanes_applied;

  m_fpt_lane_management #(
    .PARA_MAX_LANES      (PARA_MAX_LANES),
    .PARA_SKEW_DEPTH     (PARA_SKEW_DEPTH),
    .PARA_FIFO_AF_MARGIN (PARA_FIFO_AF_MARGIN)
  ) u_dut (
    .i_rxbyteclkhs          (i_rxbyteclkhs),
    .i_resetn               (i_resetn),
    .i_soft_reset           (i_soft_reset),
    .i_active_lanes         (i_active_lanes),
    .i_rxdatahs_0           (i_rxdatahs[0]),
    .i_rxdatahs_1           (i_rxdatahs[1]),
    .i_rxdatahs_2           (i_rxdatahs[2]),
    .i_rxdatahs_3           (i_rxdatahs[3]),
    .i_rxvalidhs_0          (i_rxvalidhs[0]),
    .i_rxvalidhs_1          (i_rxvalidhs[1]),
    .i_rxvalidhs_2          (i_rxvalidhs[2]),
    .i_rxvalidhs_3          (i_rxvalidhs[3]),
    .i_rxactivehs_0         (i_rxactivehs[0]),
    .i_rxactivehs_1         (i_rxactivehs[1]),
    .i_rxactivehs_2         (i_rxactivehs[2]),
    .i_rxactivehs_3         (i_rxactivehs[3]),
    .i_rxsynchs_0           (i_rxsynchs[0]),
    .i_rxsynchs_1           (i_rxsynchs[1]),
    .i_rxsynchs_2           (i_rxsynchs[2]),
    .i_rxsynchs_3           (i_rxsynchs[3]),
    .i_rxskewcalhs_0        (i_rxskewcalhs[0]),
    .i_rxskewcalhs_1        (i_rxskewcalhs[1]),
    .i_rxskewcalhs_2        (i_rxskewcalhs[2]),
    .i_rxskewcalhs_3        (i_rxskewcalhs[3]),
    .i_errsoths_0           (i_errsoths[0]),
    .i_errsoths_1           (i_errsoths[1]),
    .i_errsoths_2           (i_errsoths[2]),
    .i_errsoths_3           (i_errsoths[3]),
    .i_errsotsynchs_0       (i_errsotsynchs[0]),
    .i_errsotsynchs_1       (i_errsotsynchs[1]),
    .i_errsotsynchs_2       (i_errsotsynchs[2]),
    .i_errsotsynchs_3       (i_errsotsynchs[3]),
    .o_merged_data          (o_merged_data),
    .o_merged_valid         (o_merged_valid),
    .o_merged_byte_valid    (o_merged_byte_valid),
    .o_merged_sob           (o_merged_sob),
    .o_merged_eob           (o_merged_eob),
    .o_lm_fsm_state         (o_lm_fsm_state),
    .o_abort_sync_err       (o_abort_sync_err),
    .o_abort_fifo_full      (o_abort_fifo_full),
    .o_abort_unarmed        (o_abort_unarmed),
    .o_abort_lane_sync_seen (o_abort_lane_sync_seen),
    .o_lane_abort_pulse     (o_lane_abort_pulse),
    .o_deskew_burst_seen    (o_deskew_burst_seen),
    .o_active_lanes_applied (o_active_lanes_applied)
  );

  // --------------------------------------------------------------------------------------------
  // Clock, error reporting and small helpers
  // --------------------------------------------------------------------------------------------
  // The byte clock toggles while clk_run is 1.
  bit clk_run = 1'b1;
  int unsigned cyc;

  initial begin
    i_rxbyteclkhs = 1'b0;
    forever begin
      #(T_BYTE_CLK / 2.0);
      if (clk_run) i_rxbyteclkhs = ~i_rxbyteclkhs;
    end
  end

  always @(posedge i_rxbyteclkhs) cyc++;

  int unsigned err_count;
  int unsigned check_count;

  task automatic fail(input string msg);
    err_count++;
    if (err_count <= 30) $error("%0t ns: %s", $realtime, msg);
  endtask

  // Wait n rising clock edges, plus 0.5 ns. Stimulus always changes at that point.
  task automatic cycles(input int n);
    repeat (n) @(posedge i_rxbyteclkhs);
    #0.5;
  endtask

  // Lanes selected by an active_lanes code.
  function automatic logic [3:0] lanes_of(input logic [1:0] al);
    case (al)
      2'b00:   return 4'b0001;
      2'b01:   return 4'b0011;
      2'b11:   return 4'b1111;
      default: return 4'b0000;
    endcase
  endfunction

  function automatic int lanes_n(input logic [1:0] al);
    return (al == 2'b00) ? 1 : (al == 2'b01) ? 2 : (al == 2'b11) ? 4 : 0;
  endfunction

  function automatic logic [1:0] code_of(input int n);
    return (n == 1) ? 2'b00 : (n == 2) ? 2'b01 : 2'b11;
  endfunction

  function automatic logic [3:0] lane_bit(input int l);
    return 4'b0001 << l;
  endfunction

  // Number of bytes that lane l carries when "total" bytes are spread over nl lanes.
  function automatic int lane_bytes(input int total, input int nl, input int l);
    return (total > l) ? (total - l + nl - 1) / nl : 0;
  endfunction

  // Set every PPI lane input to 0.
  task automatic idle_lanes();
    i_rxvalidhs = '0;  i_rxactivehs = '0;  i_rxsynchs = '0;  i_rxskewcalhs = '0;
    i_errsoths = '0;  i_errsotsynchs = '0;
    for (int l = 0; l < 4; l++) i_rxdatahs[l] = 8'h00;
  endtask

  // --------------------------------------------------------------------------------------------
  // Reference model
  // --------------------------------------------------------------------------------------------
  // Updated at each rising clock edge from the inputs of the cycle that ends there; the checker
  // compares the DUT with it at the falling edge.
  logic [2:0]   m_state;
  logic [1:0]   m_al;
  logic [3:0]   m_act_seen, m_sync_seen, m_desk_seen, m_err_sync;
  byte unsigned m_fifo [4][$];
  bit           m_tail;
  byte unsigned m_stream [$];     // bytes placed in words and not yet seen on merged_*
  bit           m_sob_due;        // the next data word is the first of the burst
  bit           m_last_eob;       // the last data word of the burst had merged_eob
  logic         m_pulse;
  logic [2:0]   m_cause;
  logic [3:0]   m_seen;
  logic         m_desk_sticky;
  bit           m_marker_due;     // abort marker expected on merged_* now
  bit           m_soft_seen;      // soft_reset was applied at the last edge
  int           m_drain_wait;
  int unsigned  m_drop_count;     // writes dropped on a full FIFO
  int unsigned  m_discard_count;  // bytes popped after the last placement

  task automatic model_reset();
    m_state = ST_IDLE;  m_al = 2'b00;
    m_act_seen = '0;  m_sync_seen = '0;  m_desk_seen = '0;  m_err_sync = '0;
    for (int l = 0; l < 4; l++) m_fifo[l].delete();
    m_tail = 1'b0;  m_stream.delete();  m_sob_due = 1'b1;  m_last_eob = 1'b1;
    m_pulse = 1'b0;  m_cause = C_NONE;  m_seen = '0;  m_desk_sticky = 1'b0;
    m_marker_due = 1'b0;  m_soft_seen = 1'b0;  m_drain_wait = 0;
  endtask

  function automatic bit fifos_empty();
    for (int l = 0; l < 4; l++) if (m_fifo[l].size() != 0) return 1'b0;
    return 1'b1;
  endfunction

  task automatic model_step();
    logic [3:0] sel, act, vld, syn, skc, ers, nempty, full, fin, seen_now;
    int         size_now [4];
    bit         sreset, burst_open, armed, any_sync, any_desk, errsync, fifo_full, has_data;
    bit         all_ready, deskew_burst, burst_clr, cfg_apply, wr_en_ok, rd_en_ok, read_en;
    bit         abort_entry, desk_exit, final_place;
    logic [2:0] nxt;
    int         jstar;
    byte unsigned b;

    sreset   = (i_soft_reset === 1'b1);
    sel      = lanes_of(m_al);
    seen_now = m_sync_seen;
    for (int l = 0; l < 4; l++) begin
      act[l]      = (i_rxactivehs[l] === 1'b1);
      vld[l]      = (i_rxvalidhs[l] === 1'b1);
      syn[l]      = (i_rxsynchs[l] === 1'b1);
      skc[l]      = (i_rxskewcalhs[l] === 1'b1);
      ers[l]      = (i_errsotsynchs[l] === 1'b1);
      size_now[l] = m_fifo[l].size();
      nempty[l]   = (size_now[l] != 0);
      full[l]     = (size_now[l] >= THR);
    end

    // Reduced conditions over the active lanes.
    burst_open   = |(act & sel & BUILT);
    any_sync     = |(m_sync_seen & sel);
    any_desk     = |(m_desk_seen & sel);
    errsync      = |(m_err_sync & sel);
    fifo_full    = |(m_sync_seen & full & sel);
    has_data     = |(nempty & sel);
    armed        = ((m_sync_seen | ~sel) == 4'b1111);
    all_ready    = ((nempty | (m_act_seen & ~act) | ~sel) == 4'b1111);
    fin          = sel & m_act_seen & ~act & ~nempty;
    deskew_burst = any_desk && !any_sync;

    // FSM outputs.
    burst_clr = ((m_state == ST_IDLE) && !burst_open) || (m_state == ST_ABORT);
    cfg_apply = (m_state == ST_IDLE) && !burst_open;
    wr_en_ok  = (m_state == ST_ARMING) || (m_state == ST_ALIGNED) ||
                ((m_state == ST_IDLE) && burst_open);
    rd_en_ok  = (m_state == ST_ALIGNED) || (m_state == ST_DRAIN);
    read_en   = rd_en_ok && all_ready && has_data;

    // Next state; ST_DRAIN is left when the checker sees the DUT leave it.
    nxt = m_state;
    case (m_state)
      ST_IDLE:    if (burst_open) nxt = ST_ARMING;
      ST_ARMING: begin
        if (errsync)                                  nxt = ST_ABORT;
        else if (armed)                               nxt = ST_ALIGNED;
        else if (fifo_full || (!burst_open && !deskew_burst)) nxt = ST_ABORT;
        else if (!burst_open)                         nxt = ST_IDLE;
      end
      ST_ALIGNED: if (!burst_open) nxt = ST_DRAIN;
      ST_DRAIN:   nxt = ST_DRAIN;
      ST_ABORT:   if (!burst_open) nxt = ST_IDLE;
      default:    nxt = ST_IDLE;
    endcase
    abort_entry = (m_state == ST_ARMING) && (nxt == ST_ABORT);
    desk_exit   = (m_state == ST_ARMING) && (nxt == ST_IDLE);

    // Lane FIFOs: one common read, then the writes of this cycle.
    final_place = 1'b0;
    if (sreset || burst_clr) begin
      for (int l = 0; l < 4; l++) m_fifo[l].delete();
    end else begin
      if (read_en) begin
        jstar = 4;
        for (int l = 3; l >= 0; l--) if (fin[l]) jstar = l;
        for (int l = 0; l < 4; l++) begin
          if (nempty[l]) begin
            b = m_fifo[l].pop_front();
            if (!m_tail && (l < jstar)) m_stream.push_back(b);
            else m_discard_count++;
          end
        end
        final_place = !m_tail && (jstar < 4);
      end
      for (int l = 0; l < 4; l++) begin
        if (sel[l] && BUILT[l] && vld[l] && (m_sync_seen[l] || syn[l]) && wr_en_ok) begin
          if (size_now[l] < DEPTH) m_fifo[l].push_back(byte'(i_rxdatahs[l]));
          else m_drop_count++;
        end
      end
    end

    // Tail of the burst and first-word flag.
    if (sreset || !rd_en_ok) begin
      m_tail = 1'b0;  m_sob_due = 1'b1;  m_last_eob = 1'b1;
    end else if (final_place) begin
      m_tail = 1'b1;
    end
    if (sreset) m_stream.delete();

    // Per-lane flags.
    if (sreset || burst_clr) begin
      m_act_seen = '0;  m_sync_seen = '0;  m_desk_seen = '0;  m_err_sync = '0;
    end else begin
      for (int l = 0; l < 4; l++) begin
        if (sel[l] && BUILT[l]) begin
          m_act_seen[l]  = m_act_seen[l]  | act[l];
          m_sync_seen[l] = m_sync_seen[l] | syn[l];
          m_desk_seen[l] = m_desk_seen[l] | skc[l];
          m_err_sync[l]  = m_err_sync[l]  | ers[l];
        end
      end
    end

    // Status, abort marker and configuration.
    m_marker_due = m_pulse && !sreset;
    if (sreset) begin
      m_pulse = 1'b0;  m_cause = C_NONE;  m_seen = '0;  m_desk_sticky = 1'b0;
    end else begin
      m_pulse = abort_entry;
      if (abort_entry) begin
        m_cause = {errsync, fifo_full, !burst_open && !deskew_burst};
        m_seen  = seen_now;
      end
      if (desk_exit) m_desk_sticky = 1'b1;
    end
    if (cfg_apply) m_al = i_active_lanes;
    m_state     = sreset ? ST_IDLE : nxt;
    m_soft_seen = sreset;
  endtask

  always @(posedge i_rxbyteclkhs or negedge i_resetn) begin
    if (i_resetn !== 1'b1) begin
      model_reset();
    end else begin
      model_step();
    end
  end

  // --------------------------------------------------------------------------------------------
  // Checker: DUT against the model, every cycle
  // --------------------------------------------------------------------------------------------
  int unsigned st_cnt [8];    // cycles spent in each state, cleared by tests
  int unsigned n_pulse;       // lane_abort_pulse cycles seen
  int unsigned word_cyc [$];  // clock cycle of each data word, cleared by tests

  task automatic expect_reset_outputs(input string what);
    check_count++;
    if (o_merged_valid !== 1'b0 || o_lane_abort_pulse !== 1'b0 || o_lm_fsm_state !== ST_IDLE ||
        {o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed} !== C_NONE ||
        o_abort_lane_sync_seen !== 4'b0000 || o_deskew_burst_seen !== 1'b0 ||
        o_active_lanes_applied !== 2'b00)
      fail($sformatf("%s: outputs not at their reset values (valid=%b pulse=%b state=%b causes=%b%b%b seen=%b deskew=%b applied=%b)",
                     what, o_merged_valid, o_lane_abort_pulse, o_lm_fsm_state, o_abort_sync_err,
                     o_abort_fifo_full, o_abort_unarmed, o_abort_lane_sync_seen,
                     o_deskew_burst_seen, o_active_lanes_applied));
  endtask

  task automatic check_data_word();
    int n;
    case (o_merged_byte_valid)
      4'b0001: n = 1;
      4'b0011: n = 2;
      4'b0111: n = 3;
      4'b1111: n = 4;
      default: n = 0;
    endcase
    word_cyc.push_back(cyc);
    if (n == 0) begin
      fail($sformatf("data word with byte mask %b", o_merged_byte_valid));
      return;
    end
    if (o_merged_sob !== m_sob_due)
      fail($sformatf("merged_sob is %b, expected %b", o_merged_sob, m_sob_due));
    if (m_stream.size() < n) begin
      fail($sformatf("data word of %0d bytes, but only %0d bytes were placed", n, m_stream.size()));
      m_stream.delete();
      return;
    end
    for (int k = 0; k < n; k++) begin
      if (o_merged_data[8*k +: 8] !== m_stream[0])
        fail($sformatf("merged_data byte %0d is %h, expected %h", k, o_merged_data[8*k +: 8],
                       m_stream[0]));
      void'(m_stream.pop_front());
    end
    if (n < 4 && o_merged_eob !== 1'b1) fail("partial data word without merged_eob");
    if (o_merged_eob === 1'b1) begin
      if (m_stream.size() != 0)
        fail($sformatf("merged_eob while %0d placed bytes are still to be sent", m_stream.size()));
      if (!(m_state == ST_DRAIN && (m_tail || fifos_empty())))
        fail("merged_eob before the end of the burst");
    end else if (o_merged_eob === 1'b0) begin
      if (m_stream.size() == 0) fail("word sent without merged_eob, but no further byte was placed");
    end else begin
      fail("merged_eob is X");
    end
    m_sob_due  = 1'b0;
    m_last_eob = (o_merged_eob === 1'b1);
  endtask

  task automatic check_cycle();
    bit drain_done;
    check_count++;
    if ($isunknown({o_merged_valid, o_lm_fsm_state, o_abort_sync_err, o_abort_fifo_full,
                    o_abort_unarmed, o_abort_lane_sync_seen, o_lane_abort_pulse,
                    o_deskew_burst_seen, o_active_lanes_applied}))
      fail("X or Z on a control or status output");

    // Word stream.
    if (m_marker_due) begin
      if (o_merged_valid !== 1'b1 || o_merged_sob !== 1'b1 || o_merged_eob !== 1'b1 ||
          o_merged_byte_valid !== 4'b0000 || o_merged_data !== 32'h0)
        fail($sformatf("abort marker expected: valid=%b sob=%b eob=%b mask=%b data=%h",
                       o_merged_valid, o_merged_sob, o_merged_eob, o_merged_byte_valid,
                       o_merged_data));
    end else if (o_merged_valid === 1'b1) begin
      if (m_soft_seen) fail("merged_valid in the cycle after soft_reset");
      else check_data_word();
    end
    if (m_stream.size() > 7)
      fail($sformatf("%0d placed bytes not yet sent: Lane Merging falls behind", m_stream.size()));

    // FSM state.
    drain_done = (m_stream.size() == 0) && m_last_eob && fifos_empty();
    if (m_state == ST_DRAIN && o_lm_fsm_state === ST_IDLE) begin
      if (!drain_done) fail("ST_DRAIN left before every byte was sent");
      m_state = ST_IDLE;
      m_drain_wait = 0;
    end else begin
      if (o_lm_fsm_state !== m_state)
        fail($sformatf("lm_fsm_state is %b, expected %b", o_lm_fsm_state, m_state));
      if (m_state == ST_DRAIN && drain_done) begin
        m_drain_wait++;
        if (m_drain_wait == 5) fail("ST_DRAIN not left after every byte was sent");
      end else begin
        m_drain_wait = 0;
      end
    end

    // Status outputs.
    if (o_lane_abort_pulse !== m_pulse)
      fail($sformatf("lane_abort_pulse is %b, expected %b", o_lane_abort_pulse, m_pulse));
    if ({o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed} !== m_cause)
      fail($sformatf("abort causes (sync_err, fifo_full, unarmed) are %b%b%b, expected %b",
                     o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed, m_cause));
    if (o_abort_lane_sync_seen !== m_seen)
      fail($sformatf("abort_lane_sync_seen is %b, expected %b", o_abort_lane_sync_seen, m_seen));
    if (o_deskew_burst_seen !== m_desk_sticky)
      fail($sformatf("deskew_burst_seen is %b, expected %b", o_deskew_burst_seen, m_desk_sticky));
    if (o_active_lanes_applied !== m_al)
      fail($sformatf("active_lanes_applied is %b, expected %b", o_active_lanes_applied, m_al));

    if (!$isunknown(o_lm_fsm_state)) st_cnt[o_lm_fsm_state]++;
    if (o_lane_abort_pulse === 1'b1) n_pulse++;
  endtask

  always @(negedge i_rxbyteclkhs) begin
    if (i_resetn === 1'b1) check_cycle();
    else if ($realtime > 0.0) expect_reset_outputs("while resetn is low");
  end

  // --------------------------------------------------------------------------------------------
  // Burst collector: what the block delivered, burst by burst
  // --------------------------------------------------------------------------------------------
  localparam int GOT_DATA = 0, GOT_MARKER = 1, GOT_CUT = 2;
  int           got_kind  [$];
  int           got_len   [$];
  byte unsigned got_bytes [$];
  byte unsigned cur_bytes [$];
  bit           cur_open;

  task automatic got_close(input int kind);
    got_kind.push_back(kind);
    got_len.push_back(cur_bytes.size());
    foreach (cur_bytes[k]) got_bytes.push_back(cur_bytes[k]);
    cur_bytes.delete();
    cur_open = 1'b0;
  endtask

  task automatic collect();
    int n;
    if (i_resetn !== 1'b1 || m_soft_seen) begin
      if (cur_open) got_close(GOT_CUT);
      return;
    end
    if (o_merged_valid !== 1'b1) return;
    if (o_merged_byte_valid === 4'b0000) begin
      if (cur_open) got_close(GOT_CUT);
      got_close(GOT_MARKER);
      return;
    end
    if (o_merged_sob === 1'b1) begin
      if (cur_open) got_close(GOT_CUT);
      cur_open = 1'b1;
    end
    if (!cur_open) return;
    n = (o_merged_byte_valid == 4'b1111) ? 4 : (o_merged_byte_valid == 4'b0111) ? 3 :
        (o_merged_byte_valid == 4'b0011) ? 2 : 1;
    for (int k = 0; k < n; k++) cur_bytes.push_back(o_merged_data[8*k +: 8]);
    if (o_merged_eob === 1'b1) got_close(GOT_DATA);
  endtask

  always @(negedge i_rxbyteclkhs) collect();
  always @(negedge i_resetn) if (cur_open) got_close(GOT_CUT);

  byte unsigned e_bytes [$];

  // Take the oldest delivered entry into e_bytes; returns its kind, or -1 if there is none.
  function automatic int take_entry();
    int kind, len;
    e_bytes.delete();
    if (got_kind.size() == 0) return -1;
    kind = got_kind.pop_front();
    len  = got_len.pop_front();
    for (int k = 0; k < len; k++) e_bytes.push_back(got_bytes.pop_front());
    return kind;
  endfunction

  function automatic string kind_name(input int kind);
    case (kind)
      GOT_DATA:   return "a data burst";
      GOT_MARKER: return "an abort marker";
      GOT_CUT:    return "a burst cut by a reset";
      default:    return "nothing";
    endcase
  endfunction

  task automatic flush_got();
    got_kind.delete();  got_len.delete();  got_bytes.delete();
  endtask

  // --------------------------------------------------------------------------------------------
  // Burst plan and stimulus
  // --------------------------------------------------------------------------------------------
  // Lane kinds:
  //   LK_OFF     no activity
  //   LK_DATA    rxactivehs, rxsynchs, then this lane's share of p_payload and p_trail extra bytes
  //   LK_NOSYNC  rxactivehs for p_len cycles, never rxsynchs; p_bytes bytes with rxvalidhs
  //   LK_SYNCERR as LK_NOSYNC, with errsotsynchs at p_sync
  //   LK_DESKEW  rxactivehs for p_len cycles; rxskewcalhs from p_sync until p_hold cycles after
  //              rxactivehs falls
  //   LK_NOISE   random values on every input of the lane for p_len cycles
  localparam int LK_OFF = 0, LK_DATA = 1, LK_NOSYNC = 2, LK_SYNCERR = 3, LK_DESKEW = 4,
                 LK_NOISE = 5;
  localparam int MAXT = 2048;

  int           p_kind    [4];
  int           p_act     [4];   // cycle in which rxactivehs rises
  int           p_sync    [4];   // cycle of rxsynchs, errsotsynchs or rising rxskewcalhs
  int           p_first   [4];   // cycle of the first byte
  int           p_trail   [4];   // LK_DATA: bytes after the data bytes
  int           p_late    [4];   // LK_DATA: extra cycles of rxactivehs after the last byte
  int           p_len     [4];   // cycles of rxactivehs (LK_NOSYNC, LK_SYNCERR, LK_DESKEW, LK_NOISE)
  int           p_bytes   [4];   // LK_NOSYNC, LK_SYNCERR: bytes sent with rxvalidhs
  int           p_hold    [4];   // LK_DESKEW
  bit           p_soterr  [4];   // LK_DATA: errsoths together with rxsynchs
  int           p_err_at  [4];   // LK_DATA: errsotsynchs pulse in this cycle, -1 for none
  int           p_stall_k [4];   // LK_DATA: the lane pauses before its byte p_stall_k ...
  int           p_stall_len [4]; // ... for this many cycles
  int           p_gap     [$];   // idle cycles before byte k, the same on every lane
  byte unsigned p_payload [$];
  int           p_lanes;         // lanes the payload is spread over
  int           p_base;          // first byte cycle of a lane with no skew
  int           p_soft_at, p_soft_len, p_rst_at, p_rst_len, p_al_at;
  real          p_rst_phase;
  logic [1:0]   p_al_val;
  bit           x_mode;          // drive X on inputs that must not matter

  bit           s_act [4][MAXT], s_vld [4][MAXT], s_syn [4][MAXT], s_skc [4][MAXT];
  bit           s_soh [4][MAXT], s_ers [4][MAXT], s_noise [4][MAXT];
  byte unsigned s_dat [4][MAXT];
  int           s_end;           // last cycle of the schedule
  int unsigned  s_drops;         // dropped FIFO writes counted by the model when the burst began
  int           s_close;         // first cycle in which every lane has rxactivehs low

  task automatic plan_clear();
    for (int l = 0; l < 4; l++) begin
      p_kind[l] = LK_OFF;  p_act[l] = 0;  p_sync[l] = 0;  p_first[l] = 0;  p_trail[l] = 0;
      p_late[l] = 0;  p_len[l] = 0;  p_bytes[l] = 0;  p_hold[l] = 0;  p_soterr[l] = 1'b0;
      p_err_at[l] = -1;  p_stall_k[l] = -1;  p_stall_len[l] = 0;
    end
    p_gap.delete();  p_payload.delete();  p_lanes = 1;
    p_soft_at = -1;  p_soft_len = 0;  p_rst_at = -1;  p_rst_len = 0;  p_rst_phase = 1.0;
    p_al_at = -1;  p_al_val = 2'b00;
  endtask

  // A burst of len payload bytes over lanes 0 .. nl-1. The first byte of each lane comes 0 to
  // skew cycles after p_base. sync_mode 0: first byte with rxsynchs; 1: one or two cycles after
  // it; 2: either. gap_max: idle cycles between bytes (same on every lane, so the lanes keep their
  // skew). late_max, trail_max: random rxactivehs tail and extra bytes per lane.
  task automatic plan_clean(input int nl, input int len, input int skew, input int sync_mode,
                            input int gap_max, input int late_max, input int trail_max);
    int s;
    plan_clear();
    p_lanes = nl;
    for (int k = 0; k < len; k++) p_payload.push_back(8'($urandom));
    p_gap.push_back(0);
    for (int k = 1; k < len + 8; k++)
      p_gap.push_back((gap_max > 0 && $urandom_range(0, 3) == 0) ? $urandom_range(1, gap_max) : 0);
    for (int l = 0; l < nl; l++) begin
      p_kind[l]  = LK_DATA;
      p_first[l] = p_base + $urandom_range(0, skew);
      s          = (sync_mode == 0) ? 0 : (sync_mode == 1) ? $urandom_range(1, 2) :
                                                             $urandom_range(0, 2);
      p_sync[l]  = p_first[l] - s;
      p_act[l]   = p_sync[l] - $urandom_range(0, 3);
      p_late[l]  = $urandom_range(0, late_max);
      p_trail[l] = $urandom_range(0, trail_max);
    end
  endtask

  task automatic end_at(input int c);
    if (c >= MAXT - 4) fail("burst plan longer than the schedule");
    if (c > s_end) s_end = c;
  endtask

  // Turn the plan into per-cycle values of every lane input.
  task automatic build_schedule();
    int t, fall, n, last_rise;
    for (int l = 0; l < 4; l++) begin
      for (int c = 0; c <= s_end + 2 && c < MAXT; c++) begin
        s_act[l][c] = 1'b0;  s_vld[l][c] = 1'b0;  s_syn[l][c] = 1'b0;  s_skc[l][c] = 1'b0;
        s_soh[l][c] = 1'b0;  s_ers[l][c] = 1'b0;  s_noise[l][c] = 1'b0;  s_dat[l][c] = 8'h00;
      end
    end
    s_end = 0;
    s_close = 0;
    s_drops = m_drop_count;
    // The data lanes overlap in High-Speed mode: none leaves it before every one has entered it.
    last_rise = 0;
    for (int l = 0; l < 4; l++) if (p_kind[l] == LK_DATA && p_act[l] > last_rise) last_rise = p_act[l];
    for (int l = 0; l < 4; l++) begin
      fall = 0;
      case (p_kind[l])
        LK_DATA: begin
          n = lane_bytes(p_payload.size(), p_lanes, l);
          if (p_act[l] < 0 || p_sync[l] < p_act[l] || p_first[l] < p_sync[l])
            fail($sformatf("bad burst plan on lane %0d", l));
          s_syn[l][p_sync[l]] = 1'b1;
          if (p_soterr[l]) s_soh[l][p_sync[l]] = 1'b1;
          if (p_err_at[l] >= 0) s_ers[l][p_err_at[l]] = 1'b1;
          t = p_first[l];
          for (int k = 0; k < n + p_trail[l]; k++) begin
            if (k < p_gap.size()) t += p_gap[k];
            if (k == p_stall_k[l]) t += p_stall_len[l];
            end_at(t);
            s_vld[l][t] = 1'b1;
            s_dat[l][t] = (k < n) ? p_payload[k * p_lanes + l] : 8'($urandom);
            t++;
          end
          if (n + p_trail[l] == 0) t = p_sync[l] + 1;
          fall = t + p_late[l];
          if (fall <= last_rise) fall = last_rise + 1;
          end_at(fall);
          for (int c = p_act[l]; c < fall; c++) s_act[l][c] = 1'b1;
        end
        LK_NOSYNC, LK_SYNCERR: begin
          fall = p_act[l] + p_len[l];
          end_at(fall);
          for (int c = p_act[l]; c < fall; c++) s_act[l][c] = 1'b1;
          if (p_kind[l] == LK_SYNCERR) s_ers[l][p_sync[l]] = 1'b1;
          for (int k = 0; k < p_bytes[l]; k++) begin
            if (p_first[l] + k >= fall) fail($sformatf("bytes after rxactivehs on lane %0d", l));
            s_vld[l][p_first[l] + k] = 1'b1;
            s_dat[l][p_first[l] + k] = 8'($urandom);
          end
        end
        LK_DESKEW: begin
          fall = p_act[l] + p_len[l];
          end_at(fall + p_hold[l]);
          for (int c = p_act[l]; c < fall; c++) s_act[l][c] = 1'b1;
          for (int c = p_sync[l]; c < fall + p_hold[l]; c++) s_skc[l][c] = 1'b1;
        end
        LK_NOISE: begin
          end_at(p_act[l] + p_len[l]);
          for (int c = p_act[l]; c < p_act[l] + p_len[l]; c++) s_noise[l][c] = 1'b1;
        end
        default: ;
      endcase
      if (p_kind[l] != LK_NOISE && fall > s_close) s_close = fall;
    end
  endtask

  task automatic drive_lane(input int l, input int t);
    bit outside;
    outside = !(lanes_of(m_al)[l] && BUILT[l]);
    if (s_noise[l][t]) begin
      i_rxactivehs[l] = $urandom;  i_rxvalidhs[l] = $urandom;  i_rxsynchs[l] = $urandom;
      i_rxskewcalhs[l] = $urandom;  i_errsoths[l] = $urandom;  i_errsotsynchs[l] = $urandom;
      i_rxdatahs[l] = 8'($urandom);
    end else begin
      i_rxactivehs[l]   = s_act[l][t];
      i_rxvalidhs[l]    = s_vld[l][t];
      i_rxsynchs[l]     = s_syn[l][t];
      i_rxskewcalhs[l]  = s_skc[l][t];
      i_errsoths[l]     = s_soh[l][t];
      i_errsotsynchs[l] = s_ers[l][t];
      i_rxdatahs[l]     = s_vld[l][t] ? s_dat[l][t] : (x_mode ? 8'hxx : 8'($urandom));
    end
    if (x_mode) begin
      i_errsoths[l] = 1'bx;
      if (outside) begin
        i_rxactivehs[l] = 1'bx;  i_rxvalidhs[l] = 1'bx;  i_rxsynchs[l] = 1'bx;
        i_rxskewcalhs[l] = 1'bx;  i_errsotsynchs[l] = 1'bx;  i_rxdatahs[l] = 8'hxx;
      end
    end
  endtask

  task automatic play_schedule();
    for (int t = 0; t <= s_end + 1; t++) begin
      if (t == p_al_at) i_active_lanes = p_al_val;
      if (t == p_soft_at) i_soft_reset = 1'b1;
      if (t == p_soft_at + p_soft_len) i_soft_reset = 1'b0;
      if (t == p_rst_at + p_rst_len) i_resetn = 1'b1;
      for (int l = 0; l < 4; l++) drive_lane(l, t);
      if (t == p_rst_at) begin
        #(p_rst_phase);
        i_resetn = 1'b0;
      end
      cycles(1);
    end
    i_soft_reset = 1'b0;
    i_resetn = 1'b1;
    idle_lanes();
  endtask

  // Wait until the DUT and the model are back in ST_IDLE, then "extra" more cycles.
  task automatic wait_idle(input int extra = 2);
    int n;
    n = 0;
    while (!(o_lm_fsm_state === ST_IDLE && m_state == ST_IDLE)) begin
      cycles(1);
      n++;
      if (n > 1000) begin
        fail("the FSM does not return to ST_IDLE");
        break;
      end
    end
    cycles(extra);
  endtask

  task automatic run_burst();
    build_schedule();
    play_schedule();
    wait_idle();
  endtask

  // Select n lanes between bursts.
  task automatic use_lanes(input int n);
    i_active_lanes = code_of(n);
    cycles(2);
    if (o_active_lanes_applied !== code_of(n))
      fail($sformatf("active_lanes_applied is %b after selecting %0d lanes", o_active_lanes_applied, n));
  endtask

  // --------------------------------------------------------------------------------------------
  // Expected result of a burst
  // --------------------------------------------------------------------------------------------
  // The burst delivered the planned payload; with prefix = 1 extra bytes may follow it.
  task automatic expect_data(input string what, input bit prefix = 1'b0);
    int kind;
    check_count++;
    kind = take_entry();
    if (kind != GOT_DATA) begin
      fail($sformatf("%s: expected a data burst, got %s", what, kind_name(kind)));
      flush_got();
      return;
    end
    if (m_drop_count != s_drops) begin
      // Bytes were dropped on a full FIFO: the reference model has already checked every word.
      $display("%s: %0d FIFO writes dropped, payload compared with the model only", what,
               m_drop_count - s_drops);
    end else if (prefix ? (e_bytes.size() < p_payload.size()) : (e_bytes.size() != p_payload.size())) begin
      fail($sformatf("%s: %0d bytes delivered, %0d sent", what, e_bytes.size(), p_payload.size()));
    end else begin
      foreach (p_payload[k]) begin
        if (e_bytes[k] != p_payload[k]) begin
          fail($sformatf("%s: byte %0d delivered as %h, sent %h", what, k, e_bytes[k], p_payload[k]));
          break;
        end
      end
    end
    if (got_kind.size() != 0) begin
      fail($sformatf("%s: more output than one burst", what));
      flush_got();
    end
  endtask

  // The burst was discarded: one abort marker, and the latched causes and sync flags match on the
  // bits selected by care and seen_care.
  task automatic expect_marker(input string what, input logic [2:0] cause, input logic [2:0] care,
                               input logic [3:0] seen, input logic [3:0] seen_care);
    int kind;
    check_count++;
    kind = take_entry();
    if (kind != GOT_MARKER) begin
      fail($sformatf("%s: expected an abort marker, got %s", what, kind_name(kind)));
      flush_got();
      return;
    end
    if (({o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed} & care) !== (cause & care))
      fail($sformatf("%s: abort causes (sync_err, fifo_full, unarmed) %b%b%b, expected %b", what,
                     o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed, cause));
    if ((o_abort_lane_sync_seen & seen_care) !== (seen & seen_care))
      fail($sformatf("%s: abort_lane_sync_seen %b, expected %b", what, o_abort_lane_sync_seen, seen));
    if (got_kind.size() != 0) begin
      fail($sformatf("%s: more output than one abort marker", what));
      flush_got();
    end
  endtask

  task automatic expect_nothing(input string what);
    int kind;
    check_count++;
    if (got_kind.size() != 0 || cur_open) begin
      kind = take_entry();
      fail($sformatf("%s: expected no output, got %s", what, kind_name(kind)));
    end
    flush_got();
  endtask

  // --------------------------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------------------------

  // Reset values, active_lanes adopted after reset and kept through soft_reset, resetn clearing
  // the latched status.
  task automatic test_reset_values();
    i_active_lanes = code_of(MAXN);
    i_resetn = 1'b0;
    cycles(4);
    expect_reset_outputs("resetn held low");
    i_resetn = 1'b1;
    cycles(1);
    if (o_active_lanes_applied !== code_of(MAXN)) fail("active_lanes not adopted after reset");
    i_soft_reset = 1'b1;
    cycles(3);
    i_soft_reset = 1'b0;
    cycles(2);
    if (o_active_lanes_applied !== code_of(MAXN)) fail("soft_reset changed active_lanes_applied");
    use_lanes(1);
    plan_clear();
    p_kind[0] = LK_NOSYNC;  p_act[0] = 4;  p_len[0] = 6;
    run_burst();
    expect_marker("lane 0 never synchronises", C_UNARMED, 3'b111, 4'b0000, 4'b1111);
    plan_clear();
    p_kind[0] = LK_DESKEW;  p_act[0] = 4;  p_sync[0] = 5;  p_len[0] = 8;  p_hold[0] = 2;
    run_burst();
    expect_nothing("deskew burst");
    if (o_deskew_burst_seen !== 1'b1) fail("deskew_burst_seen not set");
    @(negedge i_rxbyteclkhs);
    #0.3;
    i_resetn = 1'b0;
    #0.5;
    expect_reset_outputs("resetn applied between clock edges");
    cycles(3);
    i_resetn = 1'b1;
    cycles(3);
    plan_clean(1, 9, 0, 0, 0, 0, 0);
    run_burst();
    expect_data("burst after reset");
  endtask

  // Bursts of every short length on every lane count, lanes aligned, first byte with rxsynchs.
  task automatic test_basic_bursts();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int len = 1; len <= 4 * n + 5; len++) begin
        plan_clean(n, len, 0, 0, 0, 0, 0);
        run_burst();
        expect_data($sformatf("lanes=%0d length=%0d", n, len));
      end
    end
  endtask

  // Lengths 1 to 64 with random skew within the limit, rxsynchs timing and rxactivehs tail.
  task automatic test_burst_lengths();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int len = 1; len <= 64; len++) begin
        plan_clean(n, len, SKEW_OK, 2, 0, 2, 0);
        run_burst();
        expect_data($sformatf("lanes=%0d length=%0d", n, len));
      end
    end
  endtask

  // rxvalidhs pauses inside the burst.
  task automatic test_valid_gaps();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int i = 0; i < 20; i++) begin
        plan_clean(n, $urandom_range(8, 80), SKEW_OK, 2, 3, 2, 0);
        run_burst();
        expect_data($sformatf("lanes=%0d gaps, burst %0d", n, i));
      end
    end
  endtask

  // One lane late or early by 0 .. SKEW_OK cycles, first byte with rxsynchs or one cycle later.
  task automatic test_lane_skew();
    for (int n = 2; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int d = 0; d <= SKEW_OK; d++) begin
        for (int l = 0; l < n; l++) begin
          for (int mode = 0; mode < 4; mode++) begin
            plan_clean(n, (DEPTH + 6) * n + l, 0, 0, 0, 1, 0);
            for (int k = 0; k < n; k++) begin
              if (mode < 2) p_first[k] = (k == l) ? 8 + d : 8;
              else          p_first[k] = (k == l) ? 8 : 8 + d;
              p_sync[k] = p_first[k] - (mode % 2);
              p_act[k]  = p_sync[k] - (k % 3);
            end
            run_burst();
            expect_data($sformatf("lanes=%0d skew=%0d lane=%0d mode=%0d", n, d, l, mode));
          end
        end
      end
    end
    use_lanes(1);
    for (int s = 0; s < 3; s++) begin
      plan_clean(1, 13, 0, 0, 0, 1, 0);
      p_sync[0] = p_first[0] - s;  p_act[0] = p_sync[0] - 1;
      run_burst();
      expect_data($sformatf("one lane, rxsynchs %0d cycles before the first byte", s));
    end
  endtask

  // A lane synchronising THR cycles or more after the others: fifo_full abort. At SKEW_OK the
  // burst is armed in the same cycle as fifo_full and is delivered.
  task automatic test_skew_beyond_limit();
    logic [3:0] sel;
    for (int n = 2; n <= MAXN; n *= 2) begin
      use_lanes(n);
      sel = lanes_of(code_of(n));
      for (int l = 0; l < n; l++) begin
        for (int d = SKEW_OK; d <= SKEW_OK + 3; d++) begin
          plan_clean(n, n * (THR + 10), 0, 0, 0, 0, 0);
          for (int k = 0; k < n; k++) begin
            p_first[k] = (k == l) ? 8 + d : 8;
            p_sync[k]  = p_first[k];
            p_act[k]   = 6;
          end
          run_burst();
          if (d <= SKEW_OK)
            expect_data($sformatf("lanes=%0d lane %0d late by %0d", n, l, d));
          else
            expect_marker($sformatf("lanes=%0d lane %0d late by %0d", n, l, d), C_FULL, 3'b111,
                          sel & ~lane_bit(l), 4'b1111);
        end
      end
    end
    use_lanes(1);
    plan_clean(1, 4 * DEPTH, 0, 0, 0, 0, 0);
    run_burst();
    expect_data("one lane, long burst");
  endtask

  // errsotsynchs: on one lane instead of rxsynchs, with or without bytes, with rxactivehs, with
  // rxsynchs in the same cycle (wins over armed), on every lane, and after the burst is aligned.
  task automatic test_sync_error();
    logic [3:0] sel;
    logic [2:0] full;
    int         n;
    n = MAXN;
    use_lanes(n);
    sel  = lanes_of(code_of(n));
    // The other lanes wrote one byte when the abort is taken.
    full = (THR <= 1 && n > 1) ? C_FULL : C_NONE;
    for (int l = 0; l < n; l++) begin
      for (int form = 0; form < 3; form++) begin
        plan_clean(n, 8 * n, 0, 0, 0, 0, 0);
        p_kind[l]  = LK_SYNCERR;  p_sync[l] = 8;  p_act[l] = (form == 2) ? 8 : 6;
        p_len[l]   = 20;  p_first[l] = 8;  p_bytes[l] = (form == 1) ? 8 : 0;
        if (form == 2) for (int k = 0; k < n; k++) if (k != l) p_act[k] = 8;
        run_burst();
        expect_marker($sformatf("errsotsynchs on lane %0d, form %0d", l, form), C_SYNC | full,
                      3'b111, sel & ~lane_bit(l), 4'b1111);
      end
      // Lane l has synchronised too and holds one byte.
      plan_clean(n, 8 * n, 0, 0, 0, 0, 0);
      p_err_at[l] = p_sync[l];
      run_burst();
      expect_marker($sformatf("errsotsynchs with rxsynchs on lane %0d", l),
                    C_SYNC | ((THR <= 1) ? C_FULL : C_NONE), 3'b111, sel, 4'b1111);
      plan_clean(n, 16 * n, 0, 0, 0, 0, 0);
      p_err_at[l] = p_first[l] + 6;
      run_burst();
      expect_data($sformatf("errsotsynchs on lane %0d after alignment", l));
    end
    plan_clear();
    for (int k = 0; k < n; k++) begin
      p_kind[k] = LK_SYNCERR;  p_act[k] = 6;  p_sync[k] = 8;  p_len[k] = 10;
    end
    run_burst();
    expect_marker("errsotsynchs on every lane", C_SYNC, 3'b111, 4'b0000, 4'b1111);
  endtask

  // An active lane that never synchronises: unarmed when the burst closes first, fifo_full when a
  // synchronised lane fills first; also a lane that never enters High-Speed mode.
  task automatic test_lane_never_syncs();
    logic [3:0] sel;
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      sel = lanes_of(code_of(n));
      for (int l = 0; l < n; l++) begin
        plan_clean(n, 0, 0, 0, 0, 0, 0);
        p_kind[l] = LK_NOSYNC;  p_act[l] = 6;  p_len[l] = 10;
        run_burst();
        expect_marker($sformatf("lanes=%0d lane %0d never synchronises", n, l), C_UNARMED, 3'b111,
                      sel & ~lane_bit(l), 4'b1111);
        plan_clean(n, 0, 0, 0, 0, 0, 0);
        p_kind[l] = LK_NOSYNC;  p_act[l] = 6;  p_len[l] = 12;  p_first[l] = 8;  p_bytes[l] = 4;
        run_burst();
        expect_marker($sformatf("lanes=%0d lane %0d sends bytes without rxsynchs", n, l), C_UNARMED,
                      3'b111, sel & ~lane_bit(l), 4'b1111);
        if (n > 1) begin
          plan_clean(n, 0, 0, 0, 0, 0, 0);
          p_kind[l] = LK_OFF;
          run_burst();
          expect_marker($sformatf("lanes=%0d lane %0d never active", n, l), C_UNARMED, 3'b111,
                        sel & ~lane_bit(l), 4'b1111);
          plan_clean(n, n * (DEPTH + 6), 0, 0, 0, 0, 0);
          p_kind[l] = LK_NOSYNC;  p_act[l] = 6;  p_len[l] = DEPTH + 40;
          run_burst();
          expect_marker($sformatf("lanes=%0d lane %0d never synchronises, long burst", n, l),
                        C_FULL, 3'b111, sel & ~lane_bit(l), 4'b1111);
        end
      end
    end
  endtask

  // Several abort causes in the same cycle; each one present is latched.
  task automatic test_abort_causes_together();
    if (MAXN >= 2) begin
      use_lanes(2);
      for (int c = 0; c < 3; c++) begin
        // Lane 0 writes THR bytes: fifo_full in cycle 8 + THR.
        plan_clear();
        p_lanes = 1;
        for (int k = 0; k < THR; k++) p_payload.push_back(8'($urandom));
        p_kind[0] = LK_DATA;  p_act[0] = 6;  p_sync[0] = 8;  p_first[0] = 8;
        p_late[0] = (c == 1) ? 20 : 0;
        if (c == 0) begin
          p_kind[1] = LK_NOSYNC;  p_act[1] = 6;  p_len[1] = 2 + THR;
        end else begin
          p_kind[1] = LK_SYNCERR;  p_act[1] = 6;  p_sync[1] = 7 + THR;
          p_len[1] = (c == 1) ? 40 : 2 + THR;
        end
        run_burst();
        case (c)
          0: expect_marker("fifo_full as the burst closes", C_FULL | C_UNARMED, 3'b111, 4'b0001, 4'b1111);
          1: expect_marker("errsotsynchs with fifo_full", C_SYNC | C_FULL, 3'b111, 4'b0001, 4'b1111);
          default: expect_marker("all three causes", C_SYNC | C_FULL | C_UNARMED, 3'b111, 4'b0001,
                                 4'b1111);
        endcase
      end
      plan_clear();
      p_kind[0] = LK_NOSYNC;  p_act[0] = 6;  p_len[0] = 14;
      p_kind[1] = LK_SYNCERR;  p_act[1] = 6;  p_len[1] = 14;  p_sync[1] = 19;
      run_burst();
      expect_marker("errsotsynchs as the burst closes, two lanes", C_SYNC | C_UNARMED, 3'b111,
                    4'b0000, 4'b1111);
    end
    use_lanes(1);
    plan_clear();
    p_kind[0] = LK_SYNCERR;  p_act[0] = 6;  p_len[0] = 14;  p_sync[0] = 19;
    run_burst();
    expect_marker("errsotsynchs as the burst closes, one lane", C_SYNC | C_UNARMED, 3'b111, 4'b0000,
                  4'b1111);
  endtask

  // Deskew bursts: silent, deskew_burst_seen sticky; rxskewcalhs held after the burst has no
  // effect; deskew mixed with rxsynchs or errsotsynchs is not a deskew burst.
  task automatic test_deskew_burst();
    logic [3:0] sel;
    int         n;
    n = MAXN;
    use_lanes(n);
    sel = lanes_of(code_of(n));
    plan_clear();
    for (int k = 0; k < n; k++) begin
      p_kind[k] = LK_DESKEW;  p_act[k] = 6;  p_sync[k] = 6 + (k % 2);  p_len[k] = 30;
      p_hold[k] = 12;
    end
    for (int i = 0; i < 8; i++) st_cnt[i] = 0;
    run_burst();
    expect_nothing("deskew burst on every lane");
    if (o_deskew_burst_seen !== 1'b1) fail("deskew_burst_seen not set");
    if (st_cnt[ST_ARMING] == 0 || st_cnt[ST_ABORT] != 0 || st_cnt[ST_ALIGNED] != 0)
      fail("deskew burst did not return from ST_ARMING to ST_IDLE directly");
    // rxskewcalhs stayed high in ST_IDLE; the next burst without rxsynchs is unarmed.
    plan_clear();
    for (int k = 0; k < n; k++) begin
      p_kind[k] = LK_NOSYNC;  p_act[k] = 4;  p_len[k] = 8;
    end
    run_burst();
    expect_marker("burst without rxsynchs after a deskew burst", C_UNARMED, 3'b111, 4'b0000, 4'b1111);
    plan_clean(n, 24, SKEW_OK, 2, 1, 2, 0);
    run_burst();
    expect_data("data burst after a deskew burst");
    if (o_deskew_burst_seen !== 1'b1) fail("deskew_burst_seen is not sticky");
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset = 1'b0;
    cycles(1);
    if (o_deskew_burst_seen !== 1'b0) fail("soft_reset does not clear deskew_burst_seen");
    // Calibration on lane 0 only, rising with rxactivehs; the other lanes neither synchronise nor
    // calibrate.
    plan_clear();
    p_kind[0] = LK_DESKEW;  p_act[0] = 6;  p_sync[0] = 6;  p_len[0] = 20;  p_hold[0] = 3;
    for (int k = 1; k < n; k++) begin
      p_kind[k] = LK_NOSYNC;  p_act[k] = 6;  p_len[k] = 20;
    end
    run_burst();
    expect_nothing("deskew burst on lane 0 only");
    if (o_deskew_burst_seen !== 1'b1) fail("deskew_burst_seen not set by a one-lane deskew burst");
    if (n > 1) begin
      plan_clean(n, 0, 0, 0, 0, 0, 0);
      p_kind[0] = LK_DESKEW;  p_act[0] = 6;  p_sync[0] = 7;  p_len[0] = 12;  p_hold[0] = 2;
      run_burst();
      expect_marker("calibration on lane 0, rxsynchs on the others", C_UNARMED, 3'b111,
                    sel & ~4'b0001, 4'b1111);
    end
    plan_clear();
    for (int k = 0; k < n; k++) begin
      p_kind[k] = LK_DESKEW;  p_act[k] = 6;  p_sync[k] = 7;  p_len[k] = 20;  p_hold[k] = 2;
    end
    p_kind[n - 1] = LK_SYNCERR;  p_sync[n - 1] = 9;
    run_burst();
    expect_marker("errsotsynchs during calibration", C_SYNC, 3'b111, 4'b0000, 4'b1111);
  endtask

  // errsoths with rxsynchs on random lanes: bursts delivered as usual, no status change.
  task automatic test_sot_error_ignored();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int i = 0; i < 8; i++) begin
        plan_clean(n, $urandom_range(4, 40), SKEW_OK, 2, 1, 2, 0);
        for (int k = 0; k < n; k++) p_soterr[k] = (i == 0) ? 1'b1 : 1'($urandom);
        run_burst();
        expect_data($sformatf("lanes=%0d errsoths, burst %0d", n, i));
      end
    end
    if (n_pulse != 0 || {o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed} !== C_NONE)
      fail("errsoths caused an abort");
  endtask

  // active_lanes changed during a burst or in the cycle a burst opens: adopted only in ST_IDLE.
  // The reserved code 10 ignores every burst.
  task automatic test_active_lanes_change();
    int first_act;
    for (int a = 1; a <= MAXN; a *= 2) begin
      for (int b = 1; b <= MAXN; b *= 2) begin
        if (a == b) continue;
        use_lanes(a);
        plan_clean(a, 12 * a, SKEW_OK, 2, 1, 1, 0);
        p_al_at = 0;
        for (int k = 0; k < a; k++) if (p_act[k] >= p_al_at) p_al_at = p_act[k] + 1;
        p_al_val = code_of(b);
        run_burst();
        expect_data($sformatf("%0d lanes, changed to %0d during the burst", a, b));
        if (o_active_lanes_applied !== code_of(b))
          fail($sformatf("active_lanes %b not adopted after the burst", code_of(b)));
        plan_clean(b, 12 * b, SKEW_OK, 2, 1, 1, 0);
        run_burst();
        expect_data($sformatf("next burst on %0d lanes", b));
        // Change in the ST_IDLE cycle in which the burst opens.
        use_lanes(a);
        plan_clean(a, 10 * a, SKEW_OK, 2, 0, 1, 0);
        first_act = MAXT;
        for (int k = 0; k < a; k++) if (p_act[k] < first_act) first_act = p_act[k];
        p_al_at = first_act;  p_al_val = code_of(b);
        run_burst();
        expect_data($sformatf("%0d lanes, changed to %0d as the burst opens", a, b));
      end
    end
    i_active_lanes = 2'b10;
    cycles(2);
    if (o_active_lanes_applied !== 2'b10) fail("reserved active_lanes 10 not read back");
    for (int i = 0; i < 8; i++) st_cnt[i] = 0;
    for (int i = 0; i < 3; i++) begin
      plan_clean(MAXN, 20, SKEW_OK, 2, 1, 1, 0);
      if (i == 1) for (int k = 0; k < MAXN; k++) begin p_kind[k] = LK_DESKEW; p_len[k] = 10; end
      if (i == 2) p_err_at[0] = p_sync[0];
      run_burst();
      expect_nothing($sformatf("active_lanes 10, burst %0d", i));
    end
    if (st_cnt[ST_ARMING] + st_cnt[ST_ALIGNED] + st_cnt[ST_DRAIN] + st_cnt[ST_ABORT] != 0)
      fail("the FSM left ST_IDLE with active_lanes 10");
    use_lanes(MAXN);
    plan_clean(MAXN, 20, SKEW_OK, 2, 1, 1, 0);
    run_burst();
    expect_data("burst after active_lanes 10");
  endtask

  // active_lanes selecting lanes that are not built: those lanes never synchronise.
  task automatic test_unbuilt_lanes_selected();
    logic [1:0] code;
    int         nsel;
    for (int c = 0; c < 2; c++) begin
      code = (c == 0) ? 2'b01 : 2'b11;
      nsel = lanes_n(code);
      if (nsel <= MAXN) continue;
      i_active_lanes = code;
      cycles(2);
      if (o_active_lanes_applied !== code) fail("active_lanes_applied wrong");
      plan_clean(nsel, 0, 0, 0, 0, 0, 0);
      run_burst();
      expect_marker($sformatf("active_lanes %b, no byte", code), C_UNARMED, 3'b111, BUILT, 4'b1111);
      plan_clean(nsel, nsel * (DEPTH + 6), 0, 0, 0, 0, 0);
      run_burst();
      expect_marker($sformatf("active_lanes %b, long burst", code), C_FULL, 3'b111, BUILT, 4'b1111);
    end
    use_lanes(MAXN);
    plan_clean(MAXN, 32, SKEW_OK, 2, 1, 1, 0);
    run_burst();
    expect_data("burst on the built lanes");
  endtask

  // Random activity on the lanes outside the active set, during and between bursts.
  task automatic test_inactive_lane_noise();
    for (int n = 1; n < 4 && n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int i = 0; i < 10; i++) begin
        plan_clean(n, $urandom_range(1, 48), SKEW_OK, 2, 2, 2, 0);
        for (int k = n; k < 4; k++) begin
          p_kind[k] = LK_NOISE;  p_act[k] = 0;  p_len[k] = 60 + p_payload.size();
        end
        run_burst();
        expect_data($sformatf("lanes=%0d with noise on the other lanes, burst %0d", n, i));
      end
    end
    if (MAXN == 4) begin
      use_lanes(4);
      plan_clean(4, 40, SKEW_OK, 2, 1, 1, 0);
      run_burst();
      expect_data("four lanes");
    end
  endtask

  // Extra bytes after the data, different per lane: the payload is delivered first.
  task automatic test_trailer_bytes();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int i = 0; i < 16; i++) begin
        plan_clean(n, $urandom_range(1, 40), SKEW_OK, 2, 1, 2, 3);
        run_burst();
        expect_data($sformatf("lanes=%0d trailer bytes, burst %0d", n, i), 1'b1);
      end
      if (n > 1) begin
        for (int len = 4 * n - 1; len <= 4 * n + 2; len++) begin
          plan_clean(n, len, 0, 0, 0, 1, 0);
          for (int k = 1; k < n; k++) p_trail[k] = 1 + (k % 3);
          run_burst();
          expect_data($sformatf("lanes=%0d length=%0d, lane 0 without trailer", n, len), 1'b1);
        end
      end
    end
  endtask

  // rxactivehs falls 0 to 8 cycles after the last byte, differently per lane.
  task automatic test_late_activehs_fall();
    int lens [6];
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      lens = '{4 * n, 8 * n, 4 * n + 1, 4 * n + 3, 3, 12 * n + 2};
      foreach (lens[i]) begin
        for (int late = 0; late <= 8; late += 2) begin
          plan_clean(n, lens[i], SKEW_OK, 2, 0, 0, 0);
          for (int k = 0; k < n; k++) p_late[k] = late + $urandom_range(0, 2);
          run_burst();
          expect_data($sformatf("lanes=%0d length=%0d late=%0d", n, lens[i], late));
        end
      end
    end
  endtask

  // Every lane synchronises but no byte comes: ST_ALIGNED and ST_DRAIN, no output. In the first
  // burst every lane leaves High-Speed mode in the cycle after rxsynchs, so armed and the burst
  // closing come together and armed wins.
  task automatic test_sync_without_bytes();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int i = 0; i < 4; i++) begin
        if (i == 0) plan_clean(n, 0, 0, 0, 0, 0, 0);
        else        plan_clean(n, 0, SKEW_OK, 2, 0, 3, 0);
        for (int s = 0; s < 8; s++) st_cnt[s] = 0;
        run_burst();
        expect_nothing($sformatf("lanes=%0d synchronised burst without bytes", n));
        if (st_cnt[ST_ALIGNED] == 0 || st_cnt[ST_DRAIN] == 0 || st_cnt[ST_ABORT] != 0)
          fail("burst without bytes did not pass ST_ALIGNED and ST_DRAIN");
      end
    end
  endtask

  // Bursts of every kind one after the other, with the shortest idle time.
  task automatic test_back_to_back_bursts();
    int n;
    n = MAXN;
    use_lanes(n);
    p_base = 5;
    for (int i = 0; i < 24; i++) begin
      case (i % 4)
        0, 3: plan_clean(n, (i % 4 == 0) ? 40 : 3, SKEW_OK, 2, 1, 1, 0);
        1: begin
          plan_clean(n, 0, 0, 0, 0, 0, 0);
          p_kind[n - 1] = LK_NOSYNC;  p_act[n - 1] = 0;  p_len[n - 1] = 6;
        end
        default: begin
          plan_clear();
          for (int k = 0; k < n; k++) begin
            p_kind[k] = LK_DESKEW;  p_act[k] = 0;  p_sync[k] = 1;  p_len[k] = 6;
          end
        end
      endcase
      build_schedule();
      play_schedule();
      wait_idle(1);
      case (i % 4)
        0, 3:    expect_data($sformatf("back-to-back burst %0d", i));
        1:       expect_marker($sformatf("back-to-back burst %0d", i), C_UNARMED, 3'b111,
                               lanes_of(code_of(n)) & ~lane_bit(n - 1), 4'b1111);
        default: expect_nothing($sformatf("back-to-back burst %0d", i));
      endcase
    end
    p_base = 8;
  endtask

  // soft_reset in ST_IDLE and in each state of a burst; the next burst is processed normally.
  task automatic test_soft_reset();
    int n, kind;
    n = MAXN;
    use_lanes(n);
    plan_clear();
    p_kind[0] = LK_NOSYNC;  p_act[0] = 4;  p_len[0] = 6;
    run_burst();
    expect_marker("abort before soft_reset", C_UNARMED, 3'b111, 4'b0000, 4'b1111);
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset = 1'b0;
    cycles(1);
    if ({o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed} !== C_NONE ||
        o_abort_lane_sync_seen !== 4'b0000 || o_active_lanes_applied !== code_of(n))
      fail("soft_reset in ST_IDLE: status not cleared or active_lanes lost");
    for (int where = 0; where < 6; where++) begin
      plan_clean(n, 16 * n, 0, 0, 0, 0, 0);
      for (int k = 0; k < n; k++) p_act[k] = 5;
      case (where)
        0: begin p_soft_at = 6;  p_soft_len = 1; end   // ST_ARMING before rxsynchs
        1: begin p_soft_at = 9;  p_soft_len = 1; end   // ST_ARMING after rxsynchs
        2: begin p_soft_at = 18; p_soft_len = 2; end   // ST_ALIGNED
        3: ;                                           // ST_DRAIN, set below
        4: begin                                       // ST_ABORT
          plan_clear();
          for (int k = 0; k < n; k++) begin
            p_kind[k] = LK_NOSYNC;  p_act[k] = 5;  p_len[k] = 30;
          end
          p_kind[0] = LK_SYNCERR;  p_sync[0] = 8;
          p_soft_at = 14;  p_soft_len = 2;
        end
        default: begin p_soft_at = 4;  p_soft_len = 8; end  // held while the burst opens
      endcase
      build_schedule();
      if (where == 3) begin p_soft_at = s_close + 1;  p_soft_len = 1; end
      play_schedule();
      wait_idle();
      case (where)
        0: expect_data("soft_reset in ST_ARMING before rxsynchs");
        1: expect_marker("soft_reset in ST_ARMING after rxsynchs", C_UNARMED, 3'b111, 4'b0000, 4'b1111);
        2, 5: begin
          kind = take_entry();
          while (kind == GOT_CUT) kind = take_entry();
          if (kind != GOT_MARKER)
            fail($sformatf("soft_reset case %0d: expected an abort marker, got %s", where, kind_name(kind)));
          if ({o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed} !== C_UNARMED)
            fail($sformatf("soft_reset case %0d: abort cause is not unarmed", where));
          flush_got();
        end
        3: begin
          while (got_kind.size() != 0)
            if (take_entry() == GOT_MARKER) fail("soft_reset in ST_DRAIN produced an abort marker");
        end
        default: begin
          kind = take_entry();
          if (kind != GOT_MARKER) fail($sformatf("soft_reset in ST_ABORT: first entry is %s", kind_name(kind)));
          kind = take_entry();
          if (kind != GOT_MARKER) fail($sformatf("soft_reset in ST_ABORT: second entry is %s", kind_name(kind)));
          if ({o_abort_sync_err, o_abort_fifo_full, o_abort_unarmed} !== C_UNARMED)
            fail("soft_reset in ST_ABORT: second abort is not unarmed");
          flush_got();
        end
      endcase
      plan_clean(n, 20, SKEW_OK, 2, 1, 1, 0);
      run_burst();
      expect_data($sformatf("burst after soft_reset case %0d", where));
    end
  endtask

  // resetn at random times and phases during bursts, and with the byte clock stopped.
  task automatic test_async_reset();
    int n;
    n = MAXN;
    for (int i = 0; i < 12; i++) begin
      use_lanes(n);
      plan_clean(n, $urandom_range(8, 60), SKEW_OK, 2, 1, 2, 0);
      build_schedule();
      p_rst_at    = $urandom_range(2, s_close + 4);
      p_rst_len   = $urandom_range(1, 4);
      p_rst_phase = 0.1 * $urandom_range(1, 35);
      play_schedule();
      wait_idle();
      flush_got();
      use_lanes(n);
      plan_clean(n, 24, SKEW_OK, 2, 1, 1, 0);
      run_burst();
      expect_data($sformatf("burst after resetn %0d", i));
    end
    plan_clear();
    p_kind[0] = LK_NOSYNC;  p_act[0] = 4;  p_len[0] = 6;
    run_burst();
    expect_marker("abort before resetn", C_UNARMED, 3'b111, 4'b0000, 4'b1111);
    clk_run = 1'b0;
    #20.0;
    i_resetn = 1'b0;
    #1.0;
    expect_reset_outputs("resetn with the byte clock stopped");
    #20.0;
    i_resetn = 1'b1;
    #5.0;
    clk_run = 1'b1;
    cycles(3);
    use_lanes(n);
    plan_clean(n, 24, SKEW_OK, 2, 1, 1, 0);
    run_burst();
    expect_data("burst after resetn with the clock stopped");
  endtask

  // One lane pauses longer than the FIFOs can absorb: writes on full FIFOs are dropped and the
  // block recovers with the next burst.
  task automatic test_fifo_overflow();
    int unsigned drops;
    for (int n = 2; n <= MAXN; n *= 2) begin
      use_lanes(n);
      for (int l = 0; l < n; l++) begin
        drops = m_drop_count;
        plan_clean(n, n * 3 * DEPTH, 0, 0, 0, 0, 0);
        p_stall_k[l] = 4;  p_stall_len[l] = DEPTH + 3;
        run_burst();
        flush_got();
        if (m_drop_count == drops) fail($sformatf("lanes=%0d lane %0d: no write was dropped", n, l));
        plan_clean(n, 16 * n, SKEW_OK, 2, 1, 1, 0);
        run_burst();
        expect_data($sformatf("lanes=%0d burst after an overflow", n));
      end
    end
    use_lanes(1);
    plan_clean(1, 40, 0, 0, 0, 0, 0);
    p_stall_k[0] = 10;  p_stall_len[0] = DEPTH + 3;
    run_burst();
    expect_data("one lane with a long pause");
  endtask

  // rxactivehs held for hundreds of cycles in ST_ARMING, ST_ALIGNED and ST_ABORT; no timeout,
  // and soft_reset alone sends ST_ARMING back to ST_ARMING.
  task automatic test_stuck_activehs();
    int n;
    n = MAXN;
    use_lanes(n);
    plan_clear();
    for (int k = 0; k < n; k++) begin
      p_kind[k] = LK_NOSYNC;  p_act[k] = 5;  p_len[k] = 400;
    end
    p_soft_at = 200;  p_soft_len = 1;
    for (int s = 0; s < 8; s++) st_cnt[s] = 0;
    run_burst();
    expect_marker("rxactivehs held without rxsynchs", C_UNARMED, 3'b111, 4'b0000, 4'b1111);
    if (st_cnt[ST_ARMING] < 390) fail("ST_ARMING not held while rxactivehs stays high");
    plan_clean(n, 10 * n + 1, SKEW_OK, 2, 0, 0, 0);
    for (int k = 0; k < n; k++) p_late[k] = 300;
    for (int s = 0; s < 8; s++) st_cnt[s] = 0;
    run_burst();
    expect_data("rxactivehs held after the last byte");
    if (st_cnt[ST_ALIGNED] < 290) fail("ST_ALIGNED not held while rxactivehs stays high");
    plan_clear();
    for (int k = 0; k < n; k++) begin
      p_kind[k] = LK_NOSYNC;  p_act[k] = 5;  p_len[k] = 300;
    end
    p_kind[0] = LK_SYNCERR;  p_sync[0] = 6;
    for (int s = 0; s < 8; s++) st_cnt[s] = 0;
    n_pulse = 0;
    run_burst();
    expect_marker("rxactivehs held after errsotsynchs", C_SYNC, 3'b111, 4'b0000, 4'b1111);
    if (st_cnt[ST_ABORT] < 290) fail("ST_ABORT not held while rxactivehs stays high");
    if (n_pulse != 1) fail($sformatf("%0d lane_abort_pulse cycles for one abort", n_pulse));
  endtask

  // X on inputs that must not matter: rxdatahs while rxvalidhs is low, errsoths, and every input
  // of the lanes outside the active set. No output may become X.
  task automatic test_four_state_inputs();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      x_mode = 1'b1;
      for (int i = 0; i < 6; i++) begin
        plan_clean(n, $urandom_range(1, 40), SKEW_OK, 2, 2, 2, 0);
        run_burst();
        expect_data($sformatf("lanes=%0d with X on unused inputs, burst %0d", n, i));
      end
      x_mode = 1'b0;
      idle_lanes();
    end
  endtask

  // A continuous stream: one word every 4/N cycles.
  task automatic test_full_rate_stream();
    for (int n = 1; n <= MAXN; n *= 2) begin
      use_lanes(n);
      plan_clean(n, 64, 0, 0, 0, 0, 0);
      word_cyc.delete();
      run_burst();
      expect_data($sformatf("lanes=%0d continuous stream", n));
      if (m_drop_count != s_drops) begin
        // Bytes dropped on a full FIFO (small SKEW_DEPTH): the word count is not that of the payload.
      end else if (word_cyc.size() != 16) begin
        fail($sformatf("lanes=%0d: %0d words for 64 bytes", n, word_cyc.size()));
      end else begin
        for (int w = 1; w < 15; w++)
          if (word_cyc[w] - word_cyc[w - 1] != 4 / n)
            fail($sformatf("lanes=%0d: word %0d came %0d cycles after the previous one", n, w,
                           word_cyc[w] - word_cyc[w - 1]));
      end
    end
  endtask

  // Random mix of every kind of burst, lane setting, noise, soft_reset and resetn.
  task automatic test_random_bursts();
    int         n, nd, kind, l, r, inject;
    logic [1:0] code;
    bit         exact, prefix;
    for (int b = 0; b < 150; b++) begin
      if (b % 10 == 0) begin
        r = $urandom_range(0, 19);
        if (r == 0)                  code = 2'b10;
        else if (r == 1 && MAXN < 4) code = 2'b11;
        else                         code = code_of(1 << $urandom_range(0, $clog2(MAXN)));
        i_active_lanes = code;
        cycles(2);
      end
      code   = m_al;
      n      = lanes_n(code);
      nd     = (n == 0) ? MAXN : n;
      kind   = $urandom_range(0, 9);
      inject = ($urandom_range(0, 19) == 0) ? $urandom_range(1, 2) : 0;
      plan_clean(nd, $urandom_range(0, 64), SKEW_OK, 2, 2, 3, 0);
      l = $urandom_range(0, nd - 1);
      exact  = (n != 0) && (n <= MAXN) && (kind <= 4) && (inject == 0);
      prefix = (n != 0) && (n <= MAXN) && (kind == 5) && (inject == 0);
      case (kind)
        5: for (int k = 0; k < nd; k++) p_trail[k] = $urandom_range(0, 3);
        6: begin
          p_first[l] += $urandom_range(THR, THR + 4);
          p_sync[l] = p_first[l];
        end
        7: begin
          p_kind[l] = ($urandom_range(0, 1) == 0) ? LK_OFF : LK_NOSYNC;
          p_act[l] = 5;  p_len[l] = $urandom_range(2, 30);
        end
        8: p_err_at[l] = p_act[l] + $urandom_range(0, 6);
        9: for (int k = 0; k < nd; k++) begin
          p_kind[k] = LK_DESKEW;  p_sync[k] = p_act[k] + $urandom_range(0, 2);
          p_len[k] = $urandom_range(4, 30);  p_hold[k] = $urandom_range(0, 6);
        end
        default: ;
      endcase
      // An empty payload gives no word (and with trailers maybe none either): no data expected.
      if (p_payload.size() == 0) begin
        exact  = 1'b0;
        prefix = 1'b0;
      end
      if (nd < 4 && $urandom_range(0, 2) == 0) begin
        for (int k = nd; k < 4; k++) begin
          p_kind[k] = LK_NOISE;  p_act[k] = 0;  p_len[k] = 40 + p_payload.size();
        end
      end
      build_schedule();
      if (inject == 1) begin
        p_soft_at = $urandom_range(1, s_close + 2);  p_soft_len = $urandom_range(1, 3);
      end else if (inject == 2) begin
        p_rst_at = $urandom_range(1, s_close + 2);  p_rst_len = $urandom_range(1, 3);
        p_rst_phase = 0.1 * $urandom_range(1, 35);
      end
      play_schedule();
      wait_idle();
      // resetn returns active_lanes_applied to 00, so the rest of the burst runs on lane 0.
      if (n == 0 && inject != 2) begin
        expect_nothing($sformatf("random burst %0d with active_lanes 10", b));
      end else if (exact) begin
        expect_data($sformatf("random burst %0d", b));
      end else if (prefix) begin
        expect_data($sformatf("random burst %0d", b), 1'b1);
      end else begin
        flush_got();
      end
      if (inject == 2) begin
        i_active_lanes = code;
        cycles(2);
      end
    end
  endtask

  // Forces a wrong word on merged_*; this test must end in FAIL with errors in the log. It
  // confirms that the checker catches a fault and is not part of the regression.
  task automatic test_checker_fault();
    use_lanes(MAXN);
    plan_clean(MAXN, 40, 0, 0, 0, 0, 0);
    build_schedule();
    fork
      play_schedule();
      begin
        cycles(14);
        force o_merged_valid = 1'b1;
        force o_merged_data  = 32'h0000_0000;
        cycles(3);
        release o_merged_valid;
        release o_merged_data;
      end
    join
    wait_idle();
    flush_got();
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
    idle_lanes();
    plan_clear();
    p_base = 8;
    x_mode = 1'b0;
    s_end = 0;
    i_soft_reset = 1'b0;
    i_active_lanes = code_of(MAXN);
    i_resetn = 1'b0;
    #20.0;
    @(posedge i_rxbyteclkhs);
    #0.5;
    i_resetn = 1'b1;
    cycles(3);
    case (test_name)
      "test_reset_values":            test_reset_values();
      "test_basic_bursts":            test_basic_bursts();
      "test_burst_lengths":           test_burst_lengths();
      "test_valid_gaps":              test_valid_gaps();
      "test_lane_skew":               test_lane_skew();
      "test_skew_beyond_limit":       test_skew_beyond_limit();
      "test_sync_error":              test_sync_error();
      "test_lane_never_syncs":        test_lane_never_syncs();
      "test_abort_causes_together":   test_abort_causes_together();
      "test_deskew_burst":            test_deskew_burst();
      "test_sot_error_ignored":       test_sot_error_ignored();
      "test_active_lanes_change":     test_active_lanes_change();
      "test_unbuilt_lanes_selected":  test_unbuilt_lanes_selected();
      "test_inactive_lane_noise":     test_inactive_lane_noise();
      "test_trailer_bytes":           test_trailer_bytes();
      "test_late_activehs_fall":      test_late_activehs_fall();
      "test_sync_without_bytes":      test_sync_without_bytes();
      "test_back_to_back_bursts":     test_back_to_back_bursts();
      "test_soft_reset":              test_soft_reset();
      "test_async_reset":             test_async_reset();
      "test_fifo_overflow":           test_fifo_overflow();
      "test_stuck_activehs":          test_stuck_activehs();
      "test_four_state_inputs":       test_four_state_inputs();
      "test_full_rate_stream":        test_full_rate_stream();
      "test_random_bursts":           test_random_bursts();
      "test_checker_fault":           test_checker_fault();
      default: begin
        err_count++;
        $error("unknown test '%s'", test_name);
      end
    endcase
    cycles(5);
    if (err_count == 0 && check_count >= 10) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d, checks=%0d)", test_name, err_count, check_count);
    $finish;
  end

  // Timeout.
  initial begin
    #20_000_000.0;
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
