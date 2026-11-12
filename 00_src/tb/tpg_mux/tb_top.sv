// File        : tb_top.sv
// Description : TPG Mux tests with an input-history model and interface checkers.
// Run         : +TEST=<name> +SEED=<n>; one final PASS or FAIL line.
//
// Design assumptions agreed by the block owner:
//   - Controls are synchronous. Reset is asynchronous low; soft reset is synchronous high.
//   - Reset clears control history; the combinational pixel outputs still follow the RX input.
//     A retained enable restarts source selection after reset.
//   - RX valid means count 2 or 4. Sync and alignment error accompany a valid RX beat.
//     RX Frame Start precedes pixels; the last pixel is no later than Frame End.
//   - A normal frame starts with SoF. Recovery clear follows discarded payload; no old pixel
//     follows that clear. Malformed-head tests explicitly relax these source conditions.
//   - TPG holds valid and beat fields until acceptance. Frame events accompany accepted beats.
//     Dual-channel EOF follows the last VC1 row; a new frame starts after the previous EOF.
//   - Data, sync, count, channel and format are muxed even on invalid output cycles.
//     Presented SoF can clear pending before acceptance when the TPG is stalled.
//   - Both idle sources take two request edges to change selection. The request is not latched.
//     A current RX Frame Start on channel 0 or 1 delays the switch. Start wins end or clear.
//   - Full and empty are registered occupancy indications and cannot both be asserted.
//     Switching away from TPG additionally waits for an empty Line Buffer.
//   - There is no hardware timeout. Clock and settling delays here are simulation choices.
`timescale 1ns/1ps

module tb_top;
  localparam real CLOCK_PERIOD = 10.0;
  localparam int CYCLE_LIMIT = 100000;

  typedef struct packed {
    logic resetn, soft_reset;
    logic [47:0] rx_data;
    logic rx_valid;
    logic [2:0] rx_count;
    logic rx_sof, rx_sol, rx_eol;
    logic [1:0] rx_vc;
    logic rx_format, rx_error;
    logic fs, fe;
    logic [1:0] event_vc;
    logic clear_rx;
    logic [47:0] tpg_data;
    logic tpg_valid, tpg_sof, tpg_sol, tpg_eol;
    logic [1:0] tpg_vc;
    logic tpg_format;
    logic full, empty, enable_tpg;
  } stimulus_t;

  typedef struct packed {
    logic ready;
    logic [47:0] data;
    logic valid;
    logic [2:0] count;
    logic sof, sol, eol;
    logic [1:0] vc;
    logic pixel_format, align_error, selection, pending;
  } response_t;

  typedef enum {SETTLED, WAIT_QUIET, WAIT_HEAD} phase_t;
  stimulus_t s;
  response_t actual;
  logic clk = 0;
  logic tpg_fs, tpg_eof;
  bit last_tpg_beat = 0, collide_tpg_events = 0;

  m_fpt_tpg_mux u_dut (
    .i_sys_clk(clk), .i_resetn(s.resetn), .i_soft_reset(s.soft_reset),
    .i_rx_px_data(s.rx_data), .i_rx_px_valid(s.rx_valid),
    .i_rx_px_valid_count(s.rx_count), .i_rx_px_sof(s.rx_sof),
    .i_rx_px_sol(s.rx_sol), .i_rx_px_eol(s.rx_eol), .i_rx_px_vc_tag(s.rx_vc),
    .i_rx_px_dt_sel(s.rx_format), .i_rx_err_wc_align(s.rx_error),
    .i_fc_fs(s.fs), .i_fc_fe(s.fe), .i_fc_vc(s.event_vc), .i_rx_ctx_clear(s.clear_rx),
    .i_tpg_px_data(s.tpg_data), .i_tpg_px_valid(s.tpg_valid),
    .i_tpg_px_sof(s.tpg_sof), .i_tpg_px_sol(s.tpg_sol), .i_tpg_px_eol(s.tpg_eol),
    .i_tpg_px_vc_tag(s.tpg_vc), .i_tpg_px_dt_sel(s.tpg_format),
    .i_tpg_fs(tpg_fs), .i_tpg_eof(tpg_eof), .i_linebuffer_full(s.full),
    .i_linebuf_empty(s.empty), .o_tpg_px_ready(actual.ready), .i_tpg_enable(s.enable_tpg),
    .o_px_data(actual.data), .o_px_valid(actual.valid), .o_px_valid_count(actual.count),
    .o_px_sof(actual.sof), .o_px_sol(actual.sol), .o_px_eol(actual.eol),
    .o_px_vc_tag(actual.vc), .o_px_dt_sel(actual.pixel_format),
    .o_err_wc_align(actual.align_error), .o_active_sel(actual.selection),
    .o_switch_pending(actual.pending)
  );

  always #(CLOCK_PERIOD / 2) clk = ~clk;
  assign tpg_fs = s.tpg_valid && actual.ready && s.tpg_sof;
  assign tpg_eof = s.tpg_valid && actual.ready &&
                   (last_tpg_beat || (collide_tpg_events && s.tpg_sof));

  phase_t phase = SETTLED;
  bit selected = 0, waiting_sof = 0, tpg_open = 0;
  bit [1:0] rx_open = 0;
  bit [1:0] source_rx_open = 0, source_rx_first = 0, source_tpg_seen = 0;
  bit source_tpg_open = 0, stalled = 0;
  logic [53:0] held_tpg;
  logic [53:0] expected_beats[$];
  bit allow_missing_sof = 0, allow_closed_rx = 0;
  bit allow_tpg_collision = 0, allow_rx_collision = 0;
  bit checking = 0, finished = 0, fault_injected = 0;
  string test_name;
  int seed, errors = 0, cycles = 0, comparisons = 0, anchors = 0, scenarios = 0;
  int source_checks = 0, accepted_beats = 0, emitted_beats = 0, sequence_checks = 0;
  int stall_checks = 0, rx_checks = 0, event_checks = 0, serial = 0;
  realtime edge_time = -1.0;

  task automatic conclude(input bit timed_out = 0);
    if (!finished) begin
      finished = 1;
      $display("checked cycles=%0d comparisons=%0d anchors=%0d scenarios=%0d",
               cycles, comparisons, anchors, scenarios);
      $display("source_checks=%0d accepted=%0d emitted=%0d sequence_checks=%0d",
               source_checks, accepted_beats, emitted_beats, sequence_checks);
      if (timed_out) $display("%s FAIL (timeout)", test_name);
      else if (errors == 0 && comparisons > 0 && anchors > 0 && scenarios > 0)
        $display("%s PASS", test_name);
      else $display("%s FAIL (errors=%0d)", test_name, errors);
      $finish;
    end
  endtask

  task automatic check(input bit okay, input string message);
    if (!okay) begin
      errors++;
      if (errors <= 20) $error("cycle=%0d %s", cycles, message);
      if (errors == 20) conclude();
    end
  endtask

  task automatic expect_status(input bit sel, input bit pending, input string message);
    anchors++;
    check(actual.selection === sel && actual.pending === pending, message);
  endtask

  task automatic anchor(input bit okay, input string message);
    anchors++;
    check(okay, message);
  endtask

  function automatic bit head_present();
    return selected ? (s.tpg_valid && s.tpg_sof) : (s.rx_valid && s.rx_sof);
  endfunction

  function automatic response_t expected_response();
    response_t r;
    bit masked;
    masked = waiting_sof && !head_present();
    r = '0;
    r.selection = selected;
    r.pending = phase != SETTLED;
    r.ready = selected ? !s.full : (!s.enable_tpg || tpg_open);
    r.data = selected ? s.tpg_data : s.rx_data;
    r.count = selected ? 3'd4 : s.rx_count;
    r.sof = selected ? s.tpg_sof : s.rx_sof;
    r.sol = selected ? s.tpg_sol : s.rx_sol;
    r.eol = selected ? s.tpg_eol : s.rx_eol;
    r.vc = selected ? s.tpg_vc : s.rx_vc;
    r.pixel_format = selected ? s.tpg_format : s.rx_format;
    r.valid = !masked && (selected ? s.tpg_valid && r.ready : s.rx_valid);
    r.align_error = !selected && !masked && s.rx_error;
    return r;
  endfunction

  task automatic field_check(input string name, input logic [63:0] got,
                             input logic [63:0] want);
    check(got === want, $sformatf("%s got=%h expected=%h", name, got, want));
  endtask

  task automatic compare_outputs();
    response_t want, got;
    if (!checking || finished) return;
    want = expected_response();
    got = actual;
    if (test_name == "test_checker_fault" && !fault_injected && got.valid) begin
      got.data[0] = ~got.data[0];
      fault_injected = 1;
    end
    comparisons++;
    check(!$isunknown({actual.selection, actual.pending}), "unknown status");
    if (s.resetn) check(!$isunknown(actual), "unknown output after reset");
    field_check("active_sel", got.selection, want.selection);
    field_check("switch_pending", got.pending, want.pending);
    field_check("tpg_px_ready", got.ready, want.ready);
    field_check("px_data", got.data, want.data);
    field_check("px_valid", got.valid, want.valid);
    field_check("px_valid_count", got.count, want.count);
    field_check("px_sof", got.sof, want.sof);
    field_check("px_sol", got.sol, want.sol);
    field_check("px_eol", got.eol, want.eol);
    field_check("px_vc_tag", got.vc, want.vc);
    field_check("px_dt_sel", got.pixel_format, want.pixel_format);
    field_check("err_wc_align", got.align_error, want.align_error);
  endtask

  task automatic clear_history();
    phase = SETTLED;
    selected = 0;
    waiting_sof = 0;
    rx_open = 0;
    tpg_open = 0;
    source_rx_open = 0;
    source_rx_first = 0;
    source_tpg_open = 0;
    source_tpg_seen = 0;
    stalled = 0;
    held_tpg = '0;
    expected_beats.delete();
  endtask

  task automatic advance_history();
    bit rx_busy, outgoing_busy, outgoing_valid, incoming_busy, complete, sof;
    rx_busy = (|rx_open) || (s.fs && s.event_vc < 2);
    outgoing_busy = selected ? tpg_open : rx_busy;
    outgoing_valid = selected ? s.tpg_valid : s.rx_valid;
    incoming_busy = s.enable_tpg ? tpg_open : rx_busy;
    complete = !outgoing_busy && !outgoing_valid && !incoming_busy &&
               (!selected || s.empty);
    sof = head_present();
    if (phase == WAIT_QUIET && complete) begin
      selected = s.enable_tpg;
      waiting_sof = 1;
      phase = WAIT_HEAD;
    end else begin
      if (sof) waiting_sof = 0;
      case (phase)
        SETTLED: if (s.enable_tpg != selected) phase = WAIT_QUIET;
        WAIT_HEAD: begin
          if (sof) phase = SETTLED;
          else if (s.enable_tpg != selected) phase = WAIT_QUIET;
        end
        default: begin end
      endcase
    end
    for (int v = 0; v < 2; v++) begin
      if (s.fs && s.event_vc == v) rx_open[v] = 1;
      else if (s.clear_rx || (s.fe && s.event_vc == v)) rx_open[v] = 0;
    end
    if (tpg_fs) tpg_open = 1;
    else if (tpg_eof) tpg_open = 0;
  endtask

  task automatic check_sources();
    logic [53:0] payload;
    payload = {s.tpg_data, s.tpg_sof, s.tpg_sol, s.tpg_eol, s.tpg_vc, s.tpg_format};
    source_checks++;
    check(s.rx_count inside {0, 2, 4}, "source RX count outside 0/2/4");
    check(s.rx_valid == (s.rx_count != 0), "source RX count/valid disagreement");
    check(!(s.rx_sof || s.rx_sol || s.rx_eol || s.rx_error) || s.rx_valid,
          "source RX sync/error without valid");
    if (s.rx_valid) begin
      rx_checks++;
      if (!allow_closed_rx)
        check(s.rx_vc < 2 && source_rx_open[s.rx_vc[0]], "source RX outside frame");
      if (!allow_missing_sof && source_rx_first[s.rx_vc[0]])
        check(s.rx_sof, "source RX first beat lacks SoF");
    end
    if (stalled) begin
      stall_checks++;
      check(s.tpg_valid && payload === held_tpg, "source TPG changed held beat");
    end
    check(tpg_fs == (s.tpg_valid && actual.ready && s.tpg_sof), "source TPG FS");
    if (tpg_fs) begin
      event_checks++;
      check(s.tpg_vc < 2 && !source_tpg_seen[s.tpg_vc[0]], "source repeated TPG FS");
    end
    if (!allow_tpg_collision && tpg_eof)
      check(s.tpg_valid && actual.ready && s.tpg_eol && source_tpg_open &&
            (!source_tpg_seen[1] || s.tpg_vc == 1), "source TPG EOF position");
    if (!allow_missing_sof && !source_tpg_open && s.tpg_valid)
      check(s.tpg_sof, "source TPG first beat lacks SoF");
    check(!(s.full && s.empty), "source full and empty together");
    if (!allow_tpg_collision) check(!(tpg_fs && tpg_eof), "source TPG event collision");
    if (!allow_rx_collision) check(!(s.fs && s.fe), "source RX event collision");
    stalled = s.tpg_valid && !actual.ready;
    held_tpg = payload;
    for (int v = 0; v < 2; v++) begin
      if (s.fs && s.event_vc == v) begin
        source_rx_open[v] = 1;
        source_rx_first[v] = 1;
      end else if (s.clear_rx || (s.fe && s.event_vc == v)) begin
        source_rx_open[v] = 0;
        source_rx_first[v] = 0;
      end else if (s.rx_valid && s.rx_vc == v) source_rx_first[v] = 0;
    end
    if (tpg_fs) begin
      source_tpg_open = 1;
      source_tpg_seen[s.tpg_vc[0]] = 1;
    end else if (tpg_eof) begin
      source_tpg_open = 0;
      source_tpg_seen = 0;
    end
  endtask

  task automatic check_transfers();
    logic [53:0] head;
    bit masked;
    masked = waiting_sof && !head_present();
    if (!allow_missing_sof && !allow_closed_rx)
      check(actual.valid == (selected ? s.tpg_valid && actual.ready : s.rx_valid),
            "normal source transfer was suppressed");
    if (selected && s.tpg_valid && actual.ready && !masked) begin
      expected_beats.push_back({s.tpg_data, s.tpg_sof, s.tpg_sol, s.tpg_eol,
                               s.tpg_vc, s.tpg_format});
      accepted_beats++;
    end
    if (selected && actual.valid) begin
      emitted_beats++;
      if (expected_beats.size() == 0) check(0, "emitted beat without acceptance");
      else begin
        head = expected_beats.pop_front();
        sequence_checks++;
        check(head === {actual.data, actual.sof, actual.sol, actual.eol,
                        actual.vc, actual.pixel_format}, "accepted/emitted beat sequence");
      end
    end
    check(expected_beats.size() == 0, "accepted beat missing at output");
  endtask

  always @(posedge clk) begin
    edge_time = $realtime;
    if (checking) compare_outputs();
    if (!s.resetn || s.soft_reset) clear_history();
    else begin
      if (checking) begin check_sources(); check_transfers(); end
      advance_history();
    end
    cycles++;
    if (cycles > CYCLE_LIMIT) conclude(1);
    #0.1;
    compare_outputs();
  end

  always @(negedge s.resetn) begin
    clear_history();
    #0.1;
    compare_outputs();
  end

  always @(actual.selection or actual.pending) begin
    if (checking && s.resetn)
      check($realtime == edge_time, "status changed away from rising edge");
  end

  task automatic tick();
    #0.2;
    compare_outputs();
    @(posedge clk);
    #0.2;
    @(negedge clk);
    #0.2;
  endtask

  task automatic quiet();
    s.rx_valid = 0; s.rx_count = 0;
    s.rx_sof = 0; s.rx_sol = 0; s.rx_eol = 0; s.rx_error = 0;
    s.fs = 0; s.fe = 0; s.clear_rx = 0;
    s.tpg_valid = 0; s.tpg_sof = 0; s.tpg_sol = 0; s.tpg_eol = 0;
    last_tpg_beat = 0; collide_tpg_events = 0;
  endtask

  task automatic idle(input int n = 1);
    quiet();
    repeat (n) tick();
  endtask

  task automatic restart();
    quiet();
    s = '0; s.empty = 1;
    repeat (2) tick();
    checking = 1;
    s.resetn = 1;
    #0.2;
    compare_outputs();
  endtask

  task automatic rx_start(input int vc);
    quiet(); s.fs = 1; s.event_vc = 2'(vc); tick(); quiet();
  endtask

  task automatic rx_end(input int vc);
    quiet(); s.fe = 1; s.event_vc = 2'(vc); tick(); quiet();
  endtask

  function automatic logic [47:0] data_word();
    serial++;
    return {16'(serial), $urandom};
  endfunction

  task automatic rx_drive(input int vc, input int count = 4, input bit sof = 0,
                          input bit eol = 0, input bit fmt = 0, input bit err = 0);
    quiet();
    s.rx_data = data_word(); s.rx_vc = 2'(vc); s.rx_format = fmt;
    s.rx_valid = count != 0; s.rx_count = 3'(count);
    s.rx_sof = sof; s.rx_sol = sof; s.rx_eol = eol; s.rx_error = err;
  endtask

  task automatic rx_send(input int vc, input int count = 4, input bit sof = 0,
                         input bit eol = 0, input bit fmt = 0, input bit err = 0);
    rx_drive(vc, count, sof, eol, fmt, err); tick(); quiet();
  endtask

  task automatic choose_tpg();
    idle(); s.enable_tpg = 1; s.empty = 1; s.full = 0;
    tick(); expect_status(0, 1, "idle request first edge");
    tick(); expect_status(1, 1, "idle request second edge");
  endtask

  task automatic tpg_prepare(input int index, input bit dual = 0, input bit fmt = 0);
    int row, channel, column;
    quiet();
    row = index / (dual ? 4 : 2);
    channel = dual ? ((index / 2) % 2) : 0;
    column = index % 2;
    s.tpg_data = data_word(); s.tpg_valid = 1;
    s.tpg_sof = row == 0 && column == 0;
    s.tpg_sol = column == 0; s.tpg_eol = column == 1;
    s.tpg_vc = 2'(channel); s.tpg_format = fmt;
    last_tpg_beat = index == (dual ? 31 : 15);
  endtask

  task automatic tpg_send(input int index, input bit dual = 0, input bit fmt = 0,
                          input int stalls = 0);
    tpg_prepare(index, dual, fmt);
    if (stalls > 0) begin
      s.full = 1; s.empty = 0;
      repeat (stalls) begin
        tick(); anchor(!actual.ready && !actual.valid, "stall emits no beat");
      end
    end
    s.full = 0;
    #0.2;
    anchor(actual.ready, "prepared TPG beat can be accepted");
    tick(); quiet();
  endtask

  task automatic tpg_tail(input int start = 0, input bit dual = 0, input bit fmt = 0,
                          input bit with_stalls = 0);
    for (int i = start; i < (dual ? 32 : 16); i++)
      tpg_send(i, dual, fmt, with_stalls ? 1 + int'($urandom_range(0, 3)) : 0);
    idle();
  endtask

  task automatic settled_tpg();
    choose_tpg(); tpg_tail(); expect_status(1, 0, "TPG frame settled");
  endtask

  task automatic choose_rx();
    quiet(); s.enable_tpg = 0; s.full = 0; s.empty = 1;
    tick(); tick(); expect_status(0, 1, "quiet TPG switches to RX");
  endtask

  task automatic establish(input bit sel, input int stage);
    restart();
    if (sel) settled_tpg();
    if (stage == 1) begin
      if (sel) begin tpg_send(0); s.enable_tpg = 0; end
      else begin rx_start(0); rx_send(0, 4, 1); s.enable_tpg = 1; end
      idle(); expect_status(sel, 1, "drain setup");
    end else if (stage == 2) begin
      if (sel) begin restart(); choose_tpg(); end
      else begin settled_tpg(); choose_rx(); end
      expect_status(sel, 1, "first SoF setup");
    end
  endtask

  // Checks the fixed interface widths and both source encodings.
  task automatic test_interface();
    anchor($bits(actual) == 62 && $bits(actual.data) == 48 && $bits(actual.vc) == 2,
           "fixed output interface widths");
    anchor($bits(s.rx_data) == 48 && $bits(s.tpg_data) == 48 && $bits(s.event_vc) == 2,
           "fixed source interface widths");
    expect_status(0, 0, "reset selects RX encoding");
    choose_tpg(); scenarios++;
  endtask

  // Resets each source and switching phase between clock edges.
  task automatic test_async_reset();
    for (int sel = 0; sel < 2; sel++) for (int stage = 0; stage < 3; stage++) begin
      establish(1'(sel), stage);
      #1.0; quiet(); s.resetn = 0;
      #0.2; expect_status(0, 0, "async reset clears status before clock");
      restart(); choose_tpg(); scenarios++;
    end
  endtask

  // Gives soft reset priority over events and restarts with retained enable.
  task automatic test_soft_reset();
    for (int sel = 0; sel < 2; sel++) for (int stage = 0; stage < 3; stage++)
      for (int ending = 0; ending < 2; ending++) begin
        establish(1'(sel), stage);
        quiet(); s.soft_reset = 1; s.enable_tpg = 1;
        s.fs = 1; s.event_vc = 1; s.clear_rx = 1;
        s.rx_valid = 1; s.rx_count = 4; s.rx_sof = 1;
        s.tpg_valid = 1; s.tpg_sof = ending == 0;
        s.tpg_eol = ending != 0; last_tpg_beat = ending != 0;
        tick(); expect_status(0, 0, "soft reset wins concurrent activity");
        tick(); expect_status(0, 0, "held soft reset remains active");
        quiet(); s.soft_reset = 0;
        tick(); expect_status(0, 1, "retained request restarts drain");
        tick(); expect_status(1, 1, "retained request completes drain");
        scenarios++;
      end
  endtask

  // Checks nonzero mux values in reset and around a sampled soft reset.
  task automatic test_reset_mux();
    s.resetn = 0;
    for (int count = 0; count <= 4; count += 2) begin
      rx_drive(0, count, count != 0, count != 0, 1, count != 0);
      tick(); anchor(actual.data === s.rx_data && actual.count == count,
                     "reset leaves RX mux combinational");
      anchor(actual.valid == (count != 0), "reset does not gate RX valid");
      scenarios++;
    end
    restart(); choose_tpg();
    rx_drive(0, 2, 1, 1, 0, 1); s.soft_reset = 1;
    #0.2; anchor(actual.selection && actual.count == 4, "soft reset waits for edge");
    tick(); anchor(!actual.selection && actual.valid && actual.align_error,
                   "sampled soft reset selects nonzero RX bus");
    quiet(); s.soft_reset = 0; scenarios++;
  endtask

  // Checks two idle request edges with absent and already held first beats.
  task automatic test_idle_switch();
    choose_tpg(); idle(4); expect_status(1, 1, "absent first SoF keeps pending");
    tpg_tail(); expect_status(1, 0, "first presented SoF finishes switch");
    scenarios++;
    restart(); s.enable_tpg = 1; tpg_prepare(0);
    tick(); expect_status(0, 1, "held first beat while drain starts");
    tick(); expect_status(1, 1, "second edge selects held source");
    tick(); expect_status(1, 0, "held SoF is delivered after switch");
    quiet(); tpg_tail(1); scenarios++;
  endtask

  // Requests a switch at the first, middle and last VC0 beat in either format.
  task automatic test_rx_vc0_boundary();
    for (int fmt = 0; fmt < 2; fmt++) for (int request_at = 0; request_at < 3;
         request_at++) begin
      restart(); rx_start(0);
      for (int i = 0; i < 3; i++) begin
        rx_drive(0, i == 1 ? 2 : 4, i == 0, i == 2, 1'(fmt), i == 2);
        if (i == request_at) s.enable_tpg = 1;
        tick(); anchor(!actual.selection, "VC0 frame cannot be truncated");
      end
      rx_end(0); expect_status(0, 1, "FE closes history before selection changes");
      idle(); expect_status(1, 1, "quiet closed VC0 permits switch"); scenarios++;
    end
  endtask

  // Checks VC1 independently and both orders of closing a dual-channel frame.
  task automatic test_rx_vc1_boundary();
    for (int fmt = 0; fmt < 2; fmt++) for (int order = 0; order < 3; order++) begin
      restart(); rx_start(1); rx_send(1, 2, 1, 0, 1'(fmt));
      if (order != 0) begin rx_start(0); rx_send(0, 4, 1); end
      s.enable_tpg = 1; idle(2);
      rx_send(1, 4, 0, 1, 1'(fmt));
      if (order == 2) begin
        rx_end(0); idle(); expect_status(0, 1, "VC1 remains open after VC0 closes");
      end
      rx_end(1);
      if (order == 1) begin
        idle(); expect_status(0, 1, "VC0 remains open after VC1 closes"); rx_end(0);
      end
      idle(); expect_status(1, 1, "all supported frames closed"); scenarios++;
    end
  endtask

  // Maintains continuous overlap across successive long and short exposures.
  task automatic test_rx_overlap();
    rx_start(0); rx_send(0, 4, 1); rx_start(1); rx_send(1, 4, 1);
    s.enable_tpg = 1; idle();
    repeat (4) begin
      rx_end(0); rx_start(0); rx_send(0, 4, 1);
      expect_status(0, 1, "next long overlaps previous short");
      rx_end(1); rx_start(1); rx_send(1, 4, 1);
      expect_status(0, 1, "next short overlaps current long"); scenarios++;
    end
    rx_end(0); idle(3); expect_status(0, 1, "one remaining exposure blocks switch");
    rx_end(1); idle(); expect_status(1, 1, "end of overlap releases switch");
  endtask

  // Checks unsupported frame events with idle and open supported contexts.
  task automatic test_unsupported_vc();
    for (int busy = 0; busy < 2; busy++) for (int vc = 2; vc < 4; vc++) begin
      restart();
      if (busy) rx_start(0);
      s.enable_tpg = 1; idle();
      s.event_vc = 2'(vc); s.fs = 1; tick(); quiet();
      s.event_vc = 2'(vc); s.fe = 1; tick(); quiet();
      if (busy) begin
        expect_status(0, 1, "unsupported events cannot close VC0"); rx_end(0); idle();
      end
      expect_status(1, 1, "unsupported events cannot keep RX busy"); scenarios++;
    end
  endtask

  // Releases abandoned RX channels through clear and starts a fresh RX frame later.
  task automatic test_rx_context_clear();
    for (int mask = 1; mask < 4; mask++) begin
      restart();
      for (int vc = 0; vc < 2; vc++) if ((mask & (1 << vc)) != 0) begin
        rx_start(vc); rx_send(vc, 4, 1);
      end
      s.enable_tpg = 1; idle(37);
      expect_status(0, 1, "missing FE keeps context open");
      s.clear_rx = 1; tick(); idle();
      expect_status(1, 1, "recovery clear releases pending switch");
      tpg_tail(); choose_rx(); rx_start(0); rx_send(0, 4, 1, 1); rx_end(0);
      expect_status(0, 0, "fresh RX frame resumes after clear"); scenarios++;
    end
  endtask

  // Gives a same-cycle supported start priority over recovery clear.
  task automatic test_rx_start_clear();
    for (int vc = 0; vc < 2; vc++) begin
      restart(); rx_start(0); rx_start(1); s.enable_tpg = 1; idle();
      s.fs = 1; s.event_vc = 2'(vc); s.clear_rx = 1; tick(); idle(5);
      expect_status(0, 1, "new frame survives simultaneous clear");
      rx_send(vc, 4, 1, 1); rx_end(vc); idle();
      expect_status(1, 1, "other channel was cleared"); scenarios++;
    end
  endtask

  // Forwards the last valid RX beat when FE is reported in the same cycle.
  task automatic test_rx_last_beat();
    for (int vc = 0; vc < 2; vc++) for (int count = 2; count <= 4; count += 2) begin
      restart(); rx_start(vc); s.enable_tpg = 1; idle();
      rx_drive(vc, count, 1, 1); s.fe = 1; s.event_vc = 2'(vc);
      tick(); anchor(actual.valid && !actual.selection, "last RX beat survives coincident FE");
      idle(); expect_status(1, 1, "only later quiet cycle completes drain"); scenarios++;
    end
  endtask

  // Keeps an open RAW10 frame busy through bubbles and long line gaps.
  task automatic test_rx_gaps();
    rx_start(0); rx_send(0, 4, 1, 0, 1); s.enable_tpg = 1;
    repeat (4) begin
      repeat (4) rx_send(0, 4, 0, 0, 1);
      idle(); expect_status(0, 1, "RAW10 bubble does not close frame"); scenarios++;
    end
    idle(23); s.enable_tpg = 0; idle(7); s.enable_tpg = 1; idle(13);
    expect_status(0, 1, "request changes during line gap do not close frame");
    rx_send(0, 2, 0, 1, 1); rx_end(0); idle();
    expect_status(1, 1, "frame end releases gapped RX stream");
  endtask

  // Waits for accepted TPG EOF and both short and long empty-buffer delays.
  task automatic test_tpg_drain();
    for (int dual = 0; dual < 2; dual++) for (int delay_kind = 0; delay_kind < 2;
         delay_kind++) begin
      restart(); choose_tpg(); tpg_send(0, 1'(dual));
      s.enable_tpg = 0; s.empty = 0; idle();
      for (int i = 1; i < (dual ? 31 : 15); i++) tpg_send(i, 1'(dual));
      tpg_prepare(dual ? 31 : 15, 1'(dual)); s.full = 1;
      repeat (3) tick(); expect_status(1, 1, "unaccepted EOF cannot release TPG");
      s.full = 0; tick(); quiet();
      expect_status(1, 1, "EOF edge still belongs to TPG");
      idle(delay_kind ? 11 : 1);
      expect_status(1, 1, "nonempty buffer delays switch after EOF");
      s.empty = 1; tick(); expect_status(0, 1, "empty buffer permits RX selection");
      rx_start(0); rx_send(0, 4, 1, 1); rx_end(0);
      expect_status(0, 0, "RX SoF clears pending"); scenarios++;
    end
  endtask

  // Allows presented first SoF to clear pending before its eventual acceptance.
  task automatic test_first_sof_stall();
    choose_tpg(); tpg_prepare(0); s.full = 1; s.empty = 0;
    repeat (4) begin
      tick(); expect_status(1, 0, "presented stalled SoF clears pending");
      anchor(!actual.valid && actual.sof && actual.count == 4, "stalled SoF stays muxed");
    end
    s.full = 0; tick(); quiet(); tpg_tail(1);
    anchor(accepted_beats == 16 && emitted_beats == 16, "first SoF emitted exactly once");
    scenarios++;
  endtask

  // Stalls all beat positions in single/dual-channel frames of both pixel formats.
  task automatic test_tpg_backpressure();
    for (int dual = 0; dual < 2; dual++) for (int fmt = 0; fmt < 2; fmt++) begin
      restart(); choose_tpg(); tpg_tail(0, 1'(dual), 1'(fmt), 1);
      anchor(sequence_checks > 0 && accepted_beats == emitted_beats,
             "TPG accepted and emitted sequence matches"); scenarios++;
    end
  endtask

  // Cancels then renews a request while a background TPG frame is still in flight.
  task automatic test_cancel_reenable();
    rx_start(0); rx_send(0, 4, 1); s.enable_tpg = 1; idle();
    s.enable_tpg = 0; tpg_send(0); s.enable_tpg = 1; idle();
    rx_end(0); idle(5);
    expect_status(0, 1, "incoming background frame blocks selection");
    anchor(actual.ready, "background frame remains allowed to finish");
    tpg_tail(1); expect_status(1, 1, "background EOF releases selection");
    anchor(emitted_beats == 0, "background frame never reaches selected output");
    tpg_tail(); anchor(emitted_beats == 16, "next full frame is first delivered"); scenarios++;
  endtask

  // Completes cancelled requests without changing the selected source.
  task automatic test_noop_switch();
    for (int sel = 0; sel < 2; sel++) begin
      restart(); if (sel) settled_tpg();
      s.enable_tpg = !sel; tick(); s.enable_tpg = 1'(sel); tick(); idle(3);
      expect_status(1'(sel), 1, "cancelled request still waits for same-source SoF");
      if (sel) tpg_tail();
      else begin rx_start(0); rx_send(0, 4, 1, 1); rx_end(0); end
      expect_status(1'(sel), 0, "same-source SoF completes no-op"); scenarios++;
    end
  endtask

  // Reverses each switch before the first SoF has been presented.
  task automatic test_retoggle_before_sof();
    for (int sel = 0; sel < 2; sel++) begin
      establish(1'(sel), 2);
      s.enable_tpg = !sel; s.empty = 0; tick();
      expect_status(1'(sel), 1, "re-toggle returns to drain with pending set");
      s.empty = 1; tick(); expect_status(!sel, 1, "re-drain samples new request");
      if (sel) begin rx_start(1); rx_send(1, 4, 1, 1); rx_end(1); end
      else tpg_tail();
      expect_status(!sel, 0, "eventual new-source SoF clears pending"); scenarios++;
    end
  endtask

  // Gives presented SoF priority over a simultaneous request reversal.
  task automatic test_sof_request_priority();
    for (int sel = 0; sel < 2; sel++) for (int hold_head = 0; hold_head < 2;
         hold_head++) begin
      establish(1'(sel), 2);
      if (sel) begin
        tpg_prepare(0); s.full = 1'(hold_head); s.empty = 0;
      end else begin rx_start(0); rx_drive(0, 4, 1); end
      s.enable_tpg = !sel; tick();
      expect_status(1'(sel), 0, "presented SoF wins request mismatch");
      if (sel && !hold_head) tpg_prepare(1);
      else if (!sel) quiet();
      tick(); expect_status(1'(sel), 1, "mismatch enters drain one edge later");
      if (sel) begin
        if (hold_head) begin s.full = 0; tick(); quiet(); tpg_tail(1); end
        else begin quiet(); tpg_tail(2); end
        s.empty = 1; idle();
      end else begin rx_end(0); idle(); end
      expect_status(!sel, 1, "frame completes before reversed selection"); scenarios++;
    end
  endtask

  // Passes the current source's first SoF during re-drain and preserves its frame.
  task automatic test_sof_during_redrain();
    for (int sel = 0; sel < 2; sel++) begin
      establish(1'(sel), 2); s.enable_tpg = !sel; tick();
      if (sel) begin
        tpg_send(0); expect_status(1, 1, "TPG head in re-drain keeps current source");
        tpg_tail(1);
      end else begin
        rx_start(0); rx_send(0, 4, 1);
        expect_status(0, 1, "RX head in re-drain keeps current source");
        rx_send(0, 2, 0, 1); rx_end(0); idle();
      end
      expect_status(!sel, 1, "later frame end permits reversed switch"); scenarios++;
    end
  endtask

  function automatic logic [47:0] pattern_word(input int n);
    if (n == 0) return '0;
    if (n == 1) return '1;
    if (n < 50) return 48'b1 << (n - 2);
    return data_word();
  endfunction

  // Checks every mux field with valid, invalid, zero, one and walking-bit data.
  task automatic test_mux_fields();
    for (int vc = 0; vc < 2; vc++) for (int fmt = 0; fmt < 2; fmt++) begin
      rx_start(vc);
      for (int i = 0; i < 52; i++) begin
        rx_drive(vc, (i % 2) ? 2 : 4, i == 0, i == 51, 1'(fmt), i == 51);
        s.rx_data = pattern_word(i); s.tpg_data = ~s.rx_data;
        tick(); anchor(actual.data === s.rx_data, "valid RX mux responds without pipeline");
      end
      rx_end(vc); scenarios++;
    end
    for (int i = 0; i < 52; i++) begin
      quiet(); s.rx_data = pattern_word(i); s.rx_vc = 2'(i % 4);
      s.rx_format = 1'(i % 2); s.tpg_data = ~s.rx_data; tick();
      anchor(!actual.valid && actual.data === s.rx_data, "invalid RX mux remains live");
    end
    choose_tpg();
    for (int i = 0; i < 52; i++) begin
      quiet(); s.tpg_data = pattern_word(i); s.tpg_vc = 2'(i % 4);
      s.tpg_format = 1'(i % 2); s.rx_data = ~s.tpg_data; tick();
      anchor(actual.data === s.tpg_data && actual.count == 4 && !actual.align_error,
             "invalid TPG mux retains data and fixed count");
    end
    tpg_tail(0, 1, 0); tpg_tail(0, 1, 1); scenarios++;
  endtask

  // Masks malformed first beats across re-drain while leaving mux sidebands visible.
  task automatic test_malformed_head();
    allow_missing_sof = 1;
    for (int sel = 0; sel < 2; sel++) begin
      establish(1'(sel), 2);
      if (sel) begin tpg_prepare(1); end
      else begin rx_start(0); rx_drive(0, 4, 0, 1, 1, 1); end
      tick(); anchor(!actual.valid && !actual.align_error, "malformed head is masked");
      anchor(actual.eol, "masked sideband still passes through mux");
      s.enable_tpg = !sel; tick();
      anchor(!actual.valid && !actual.align_error, "mask persists while re-draining");
      if (sel) begin tpg_send(0); tpg_tail(1); end
      else begin rx_send(0, 4, 1, 1, 1); rx_end(0); idle(); end
      expect_status(!sel, 1, "valid SoF restores frame before later switch"); scenarios++;
    end
    restart(); allow_missing_sof = 0;
  endtask

  // Shows both reset types remove a pending mask using a malformed post-reset RX beat.
  task automatic test_reset_clears_mask();
    allow_missing_sof = 1; allow_closed_rx = 1;
    for (int reset_kind = 0; reset_kind < 2; reset_kind++) begin
      establish(0, 2); rx_drive(0, 4, 0, 1, 0, 1);
      tick(); anchor(!actual.valid && !actual.align_error, "mask setup is observable");
      if (reset_kind) begin s.soft_reset = 1; tick(); s.soft_reset = 0; end
      else restart();
      rx_drive(0, 4, 0, 1, 0, 1); tick();
      anchor(actual.valid && actual.align_error, "reset removed mask without a new SoF");
      scenarios++;
    end
    restart(); allow_missing_sof = 0; allow_closed_rx = 0;
  endtask

  // Checks three blockers over varied finite waits before releasing the exact blocker.
  task automatic test_wait_without_timeout();
    for (int kind = 0; kind < 3; kind++) begin
      restart();
      if (kind == 0) begin rx_start(0); rx_send(0, 4, 1); s.enable_tpg = 1; end
      else begin
        choose_tpg();
        if (kind == 1) tpg_send(0);
        else tpg_tail();
        s.enable_tpg = 0; s.empty = kind == 1;
      end
      idle(257 + int'($urandom_range(0, 127)));
      expect_status(kind != 0, 1, "first finite wait has no spontaneous switch");
      idle(769 + int'($urandom_range(0, 127)));
      expect_status(kind != 0, 1, "second finite wait has no spontaneous switch");
      if (kind == 0) begin rx_end(0); idle(); end
      else if (kind == 1) tpg_tail(1);
      else begin s.empty = 1; tick(); end
      expect_status(kind == 0, 1, "exact blocker release permits switch"); scenarios++;
    end
  endtask

  // Samples the live request and its corresponding incoming-busy condition.
  task automatic test_live_request();
    rx_start(0); rx_send(0, 4, 1); s.enable_tpg = 1; idle();
    s.enable_tpg = 0; tpg_send(0); s.enable_tpg = 1; rx_end(0); idle(3);
    expect_status(0, 1, "requested TPG is still busy");
    s.enable_tpg = 0; tick();
    rx_start(0); rx_send(0, 4, 1, 1); rx_end(0);
    expect_status(0, 0, "live RX request completes no-op despite busy TPG");
    s.enable_tpg = 1; idle(5); expect_status(0, 1, "renewed TPG request waits for EOF");
    tpg_tail(1); expect_status(1, 1, "current TPG request wins at completion"); scenarios++;
  endtask

  // Crosses three switch types with all four frame-event channel encodings.
  task automatic test_rx_start_at_switch();
    for (int mode = 0; mode < 3; mode++) for (int vc = 0; vc < 4; vc++) begin
      restart(); if (mode == 1) settled_tpg();
      s.enable_tpg = mode != 1; tick();
      if (mode == 2) s.enable_tpg = 0;
      s.fs = 1; s.event_vc = 2'(vc); tick(); quiet();
      if (vc < 2) begin
        expect_status(mode == 1, 1, "supported current FS defers switch edge");
        rx_send(vc, 4, 1, 1); rx_end(vc); idle();
      end
      expect_status(mode == 0, 1, "only supported FS contributes to RX busy"); scenarios++;
    end
  endtask

  // Exercises legal source count, ordering, event, stall and occupancy monitors.
  task automatic test_source_monitors();
    rx_start(0); rx_send(0, 4, 1); idle(); rx_send(0, 2, 0, 1, 1, 1); rx_end(0);
    choose_tpg(); tpg_tail(0, 1, 1, 1);
    anchor(rx_checks > 0 && event_checks > 0 && stall_checks > 0 && source_checks > 0,
           "source monitors exercised by real traffic"); scenarios++;
  endtask

  // Checks normal transfers across both directions, resets, no-op switches and full stalls.
  task automatic test_normal_transfers();
    for (int dual = 0; dual < 2; dual++) begin
      restart(); choose_tpg(); tpg_tail(0, 1'(dual), 1'(dual), 1);
      choose_rx(); rx_start(dual); rx_send(dual, 4, 1, 1); rx_end(dual);
      expect_status(0, 0, "normal RX transfer completes return switch"); scenarios++;
    end
    test_noop_switch();
    anchor(accepted_beats > 0 && accepted_beats == emitted_beats,
           "normal selected TPG transfers are neither lost nor duplicated");
  endtask

  // Observes the mux boundary when RX FE precedes an external recovery flush.
  task automatic test_recovery_boundary();
    rx_start(0); rx_send(0, 4, 1, 1); s.enable_tpg = 1; idle();
    rx_end(0); idle(); expect_status(1, 1, "FE permits selection before external flush");
    tpg_send(0); expect_status(1, 0, "TPG head delivered at mux boundary");
    idle(2); tpg_tail(1);
    anchor(emitted_beats == 16, "mux completes selected frame at its own boundary"); scenarios++;
  endtask

  // Gives simultaneous accepted TPG start and end events start priority.
  task automatic test_tpg_event_priority();
    allow_tpg_collision = 1; tpg_prepare(0); collide_tpg_events = 1;
    #0.2; anchor(tpg_fs && tpg_eof, "both TPG events are present"); tick();
    quiet(); s.enable_tpg = 1; idle(41);
    expect_status(0, 1, "TPG start wins end and keeps incoming frame busy");
    anchor(actual.ready, "started background TPG frame still receives ready");
    tpg_tail(1); expect_status(1, 1, "later EOF releases busy TPG");
    restart(); allow_tpg_collision = 0; scenarios++;
  endtask

  // Gives simultaneous RX start/end events start priority for both supported channels.
  task automatic test_rx_event_priority();
    allow_rx_collision = 1;
    for (int vc = 0; vc < 2; vc++) for (int clear_it = 0; clear_it < 2; clear_it++) begin
      restart(); s.enable_tpg = 1; tick(); s.fs = 1; s.fe = 1; s.event_vc = 2'(vc);
      tick(); idle(13); expect_status(0, 1, "RX start wins end at would-be switch");
      rx_send(vc, 4, 1, 1);
      if (clear_it) begin s.clear_rx = 1; tick(); quiet(); end
      else rx_end(vc);
      idle(); expect_status(1, 1, "later end or clear releases RX context"); scenarios++;
    end
    restart(); allow_rx_collision = 0;
  endtask

  // Holds drain with RX valid even when frame context is absent after either reset.
  task automatic test_rx_valid_without_context();
    allow_missing_sof = 1; allow_closed_rx = 1;
    for (int reset_kind = 0; reset_kind < 2; reset_kind++) begin
      restart();
      if (reset_kind) begin s.soft_reset = 1; tick(); s.soft_reset = 0; end
      rx_drive(0, 4, 0, 1, 0, 1); s.enable_tpg = 1;
      repeat (5) tick(); expect_status(0, 1, "outgoing valid alone prevents drain");
      idle(); expect_status(1, 1, "dropping outgoing valid permits drain"); scenarios++;
    end
    restart(); allow_missing_sof = 0; allow_closed_rx = 0;
  endtask

  task automatic short_reset();
    quiet(); @(posedge clk); #1.0;
    s.resetn = 0;
    #0.2; expect_status(0, 0, "short pulse resets status without a receiving edge");
    @(negedge clk); #0.2; s.resetn = 1;
    #0.2; compare_outputs();
  endtask

  // Clears RX history, background TPG history and masking with pulses between rising edges.
  task automatic test_short_async_reset();
    allow_missing_sof = 1; allow_closed_rx = 1;
    rx_start(0); s.enable_tpg = 1; idle(); short_reset();
    tick(); tick(); expect_status(1, 1, "short pulse erased open RX history"); scenarios++;
    restart(); tpg_send(0); short_reset(); s.enable_tpg = 1;
    #0.2; anchor(!actual.ready, "short pulse erased background TPG history");
    tick(); tick(); expect_status(1, 1, "stale TPG frame cannot block selection"); scenarios++;
    restart(); s.enable_tpg = 1; tick(); s.enable_tpg = 0; tick(); short_reset();
    rx_drive(0, 4, 0, 1, 0, 1);
    #0.2; compare_outputs();
    anchor(actual.valid && actual.align_error, "short pulse removed suppression before clock");
    tick(); scenarios++;
    restart(); allow_missing_sof = 0; allow_closed_rx = 0;
  endtask

  // Corrupts one observed data sample to prove the ordinary comparator reports an error.
  task automatic test_checker_fault();
    rx_start(0); rx_send(0, 4, 1, 1); rx_end(0);
    anchor(fault_injected, "intentional checker fault was exercised"); scenarios++;
  endtask

  initial begin
    s = '0; s.empty = 1;
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "none";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    void'($urandom(seed));
    restart();
    case (test_name)
      "test_interface": test_interface();
      "test_async_reset": test_async_reset();
      "test_soft_reset": test_soft_reset();
      "test_reset_mux": test_reset_mux();
      "test_idle_switch": test_idle_switch();
      "test_rx_vc0_boundary": test_rx_vc0_boundary();
      "test_rx_vc1_boundary": test_rx_vc1_boundary();
      "test_rx_overlap": test_rx_overlap();
      "test_unsupported_vc": test_unsupported_vc();
      "test_rx_context_clear": test_rx_context_clear();
      "test_rx_start_clear": test_rx_start_clear();
      "test_rx_last_beat": test_rx_last_beat();
      "test_rx_gaps": test_rx_gaps();
      "test_tpg_drain": test_tpg_drain();
      "test_first_sof_stall": test_first_sof_stall();
      "test_tpg_backpressure": test_tpg_backpressure();
      "test_cancel_reenable": test_cancel_reenable();
      "test_noop_switch": test_noop_switch();
      "test_retoggle_before_sof": test_retoggle_before_sof();
      "test_sof_request_priority": test_sof_request_priority();
      "test_sof_during_redrain": test_sof_during_redrain();
      "test_mux_fields": test_mux_fields();
      "test_malformed_head": test_malformed_head();
      "test_reset_clears_mask": test_reset_clears_mask();
      "test_wait_without_timeout": test_wait_without_timeout();
      "test_live_request": test_live_request();
      "test_rx_start_at_switch": test_rx_start_at_switch();
      "test_source_monitors": test_source_monitors();
      "test_normal_transfers": test_normal_transfers();
      "test_recovery_boundary": test_recovery_boundary();
      "test_tpg_event_priority": test_tpg_event_priority();
      "test_rx_event_priority": test_rx_event_priority();
      "test_rx_valid_without_context": test_rx_valid_without_context();
      "test_short_async_reset": test_short_async_reset();
      "test_checker_fault": test_checker_fault();
      default: check(0, "unknown TEST name");
    endcase
    #0.2; compare_outputs(); conclude();
  end

  initial begin
    #(CLOCK_PERIOD * (CYCLE_LIMIT + 10));
    conclude(1);
  end
endmodule
