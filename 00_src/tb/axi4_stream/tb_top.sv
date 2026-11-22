// File        : tb_top.sv
// Description : Testbench of m_fpt_axi4_stream (AXI4-Stream Master).
//               A cycle-by-cycle reference model built from the specification equations tracks the
//               skid-buffer queue, the Finalize Sequencer state, the open line and its exposure,
//               the pending abort mask and the frame-gate pass and skip bits, and predicts every
//               output at each rising edge of axis_aclk. Protocol checkers verify the reset state,
//               the AXI4-Stream handshake stability, packet TID continuity, abort beat format,
//               byte-qualifier shapes, exposure range, fbs_ready timing, asm_abort_done pulse
//               rules, input discipline and X/Z absence.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions (agreed with the block owner; the specification does not state them yet):
//   - Every register is clocked on the rising edge of axis_aclk and samples its inputs there.
//   - resetn is an asynchronous active-low reset; soft_reset is a synchronous active-high level
//     that takes priority over every other update on the same clock edge.
//   - resetn and soft_reset clear out_valid, skid_valid, the Finalize Sequencer state, line_open,
//     abort_mask and skip_arm to 0, and set both frame-gate pass bits to 2'b11. The payload fields
//     of out_* and skid_* and line_tid need not be reset and are meaningful only while
//     m_axis_tvalid is 1 (or line_open is 1).
//   - On a cycle with soft_reset high (while resetn is high), the combinational outputs fbs_ready
//     and asm_abort_done and the registered m_axis_tvalid still reflect the state at the start of
//     that cycle, and the state is cleared at the rising clock edge.
//   - A beat from Format Bridge is accepted into out_* or skid_* only on an input transfer
//     (fbs_valid && fbs_ready) in PASSTHROUGH with axis_enable high and gate_pass high.
//   - On every rising clock edge with axis_enable low, both frame-gate pass bits are cleared to
//     2'b00 (same priority as abort_start, above the update on an input transfer), so after
//     axis_enable is set again the stream resumes at the next Long-exposure (TID 0) Frame Start.
//   - Reserved exposure indices 2 and 3 (when fbs_vc_tag[1] is 1) always yield gate_pass = 0 and
//     are discarded by the frame gate.
//   - Every abort beat is a null transfer with TVALID = 1, TLAST = 1, TKEEP = 8'h00,
//     TUSER = 2'b10 (TUSER[0] = 0) and TDATA = 64'h0.
//   - asm_abort_done is a combinational one-cycle pulse asserted in the cycle of the transition
//     that ends the abort sequence.
//   - When asm_skip_req and an input Frame Start transfer of the same exposure occur on the same
//     clock edge, setting skip_arm wins so the skip applies to the following frame of that TID.
//   - When asm_abort_req arrives on the cycle an abort beat of exposure k completes its handshake,
//     that beat serves bit k of the new request and no second beat is sent for k.
`timescale 1ns/1ps

module tb_top;

  // Clock period (10 ns = 100 MHz) and simulation watchdog limit.
  localparam real T_CLK      = 10.0;
  localparam int  MAX_PRINT  = 30;
  localparam int  WAIT_LIMIT = 600000;

  typedef struct packed {
    logic [63:0] data;
    logic [7:0]  keep;
    logic        last;
    logic [1:0]  user;
    logic [1:0]  tid;
  } out_beat_t;

  typedef struct packed {
    logic [63:0] data;
    logic [7:0]  be;
    logic        sof;
    logic        eol;
    logic [1:0]  vc;
    logic        err;
  } in_beat_t;

  localparam int ST_PASS  = 0;
  localparam int ST_DRAIN = 1;
  localparam int ST_INJ   = 2;

  // ----------------------------------------------------------------------------------------------
  // DUT Signals and Instance
  // ----------------------------------------------------------------------------------------------
  logic        i_axis_aclk;
  logic        i_resetn;
  logic        i_soft_reset;
  logic [63:0] i_fbs_data;
  logic        i_fbs_valid;
  logic [7:0]  i_fbs_byte_en;
  logic        i_fbs_sof;
  logic        i_fbs_eol;
  logic [1:0]  i_fbs_vc_tag;
  logic        i_fbs_err;
  logic        o_fbs_ready;
  logic        i_asm_abort_req;
  logic [1:0]  i_asm_abort_mask;
  logic        o_asm_abort_done;
  logic        i_asm_skip_req;
  logic [1:0]  i_asm_skip_mask;
  logic        i_axis_enable;
  logic        i_vc_exposure_map;
  logic [63:0] o_m_axis_tdata;
  logic        o_m_axis_tvalid;
  logic        i_m_axis_tready;
  logic        o_m_axis_tlast;
  logic [1:0]  o_m_axis_tuser;
  logic [1:0]  o_m_axis_tid;
  logic [7:0]  o_m_axis_tkeep;

  m_fpt_axi4_stream u_dut (
    .i_axis_aclk       (i_axis_aclk),
    .i_resetn          (i_resetn),
    .i_soft_reset      (i_soft_reset),
    .i_fbs_data        (i_fbs_data),
    .i_fbs_valid       (i_fbs_valid),
    .i_fbs_byte_en     (i_fbs_byte_en),
    .i_fbs_sof         (i_fbs_sof),
    .i_fbs_eol         (i_fbs_eol),
    .i_fbs_vc_tag      (i_fbs_vc_tag),
    .i_fbs_err         (i_fbs_err),
    .o_fbs_ready       (o_fbs_ready),
    .i_asm_abort_req   (i_asm_abort_req),
    .i_asm_abort_mask  (i_asm_abort_mask),
    .o_asm_abort_done  (o_asm_abort_done),
    .i_asm_skip_req    (i_asm_skip_req),
    .i_asm_skip_mask   (i_asm_skip_mask),
    .i_axis_enable     (i_axis_enable),
    .i_vc_exposure_map (i_vc_exposure_map),
    .o_m_axis_tdata    (o_m_axis_tdata),
    .o_m_axis_tvalid   (o_m_axis_tvalid),
    .i_m_axis_tready   (i_m_axis_tready),
    .o_m_axis_tlast    (o_m_axis_tlast),
    .o_m_axis_tuser    (o_m_axis_tuser),
    .o_m_axis_tid      (o_m_axis_tid),
    .o_m_axis_tkeep    (o_m_axis_tkeep)
  );

  initial begin
    i_axis_aclk = 1'b0;
    forever #(T_CLK / 2.0) i_axis_aclk = ~i_axis_aclk;
  end

  // ----------------------------------------------------------------------------------------------
  // Simulation Control and Random Number Generation
  // ----------------------------------------------------------------------------------------------
  string       test_name;
  int          seed;
  int unsigned cyc;
  int          err_count;
  longint      check_count;
  int          max_cycles;
  int          rst_release_cyc;
  bit          checking_enabled;
  bit          stim_checks_on;
  bit          inject_checker_fault;

  function automatic int rnd(input int unsigned modulo);
    int unsigned r;
    r = $urandom();
    return int'(r % modulo);
  endfunction

  function automatic int urange(input int lo, input int hi);
    return int'($urandom_range(hi, lo));
  endfunction

  task automatic report_error(input string msg);
    err_count++;
    if (err_count <= MAX_PRINT) begin
      $error("[%s] cyc %0d: %s", test_name, cyc, msg);
    end
  endtask

  task automatic tick();
    @(negedge i_axis_aclk);
    #1.0;
  endtask

  // ----------------------------------------------------------------------------------------------
  // Source Driver (Format Bridge FWFT Stream FIFO)
  // ----------------------------------------------------------------------------------------------
  in_beat_t src_q[$];
  in_beat_t cur;
  bit       cur_valid;
  bit       last_take;
  bit       src_drop;
  int       src_drop_keep;
  int       src_gap_pct;
  longint   total_takes;

  task automatic drive_src_fields();
    logic [31:0] noise;
    i_fbs_valid = cur_valid;
    if (cur_valid) begin
      i_fbs_data    = cur.data;
      i_fbs_byte_en = cur.be;
      i_fbs_sof     = cur.sof;
      i_fbs_eol     = cur.eol;
      i_fbs_vc_tag  = cur.vc;
      i_fbs_err     = cur.err;
    end else begin
      noise         = $urandom();
      i_fbs_data    = {noise, ~noise};
      i_fbs_byte_en = noise[7:0];
      i_fbs_sof     = noise[8];
      i_fbs_eol     = noise[9];
      i_fbs_vc_tag  = noise[11:10];
      i_fbs_err     = noise[12];
    end
  endtask

  task automatic drive_src();
    if (!i_resetn) begin
      cur_valid = 1'b0;
      src_q.delete();
    end
    if (cur_valid && last_take) cur_valid = 1'b0;
    if (src_drop) begin
      cur_valid = 1'b0;
      if (src_drop_keep < 0) src_q.delete();
      else while (src_q.size() > src_drop_keep) void'(src_q.pop_front());
      src_drop = 1'b0;
    end
    if (!cur_valid && i_resetn && (src_q.size() > 0) && (rnd(100) >= src_gap_pct)) begin
      cur       = src_q.pop_front();
      cur_valid = 1'b1;
    end
    drive_src_fields();
  endtask

  // ----------------------------------------------------------------------------------------------
  // Downstream ISP Driver (m_axis_tready) and Control FSM Command Pulses
  // ----------------------------------------------------------------------------------------------
  int rdy_pct;
  int rdy_burst;
  int rdy_low_left;
  int rdy_force;
  bit rdy_toggle;

  task automatic drive_ready();
    int unsigned r;
    r = $urandom();
    if ((rdy_force == 0) || (rdy_force == 1)) begin
      i_m_axis_tready = rdy_force[0];
    end else if (rdy_force == 2) begin
      rdy_toggle      = !rdy_toggle;
      i_m_axis_tready = rdy_toggle;
    end else if (rdy_low_left > 0) begin
      rdy_low_left--;
      i_m_axis_tready = 1'b0;
    end else begin
      i_m_axis_tready = (int'(r[30:24]) % 100) < rdy_pct;
      if ((rdy_burst > 0) && (r[23:17] == 7'd0)) begin
        rdy_low_left = 1 + (int'(r[16:4]) % rdy_burst);
      end
    end
  endtask

  task automatic set_ready(input int pct, input int burst);
    rdy_pct      = pct;
    rdy_burst    = burst;
    rdy_force    = -1;
    rdy_low_left = 0;
  endtask

  bit         auto_abort;
  logic [1:0] abort_mask_cfg;
  bit         ae_c1, ae_c2;

  task automatic drive_cmds();
    i_asm_abort_req  = 1'b0;
    i_asm_abort_mask = 2'b00;
    i_asm_skip_req   = 1'b0;
    i_asm_skip_mask  = 2'b00;
    if (auto_abort && ae_c2 && !ae_c1 && i_resetn) begin
      i_asm_abort_req  = 1'b1;
      i_asm_abort_mask = abort_mask_cfg;
    end
  endtask

  task automatic abort_now(input logic [1:0] mask, input bit flush);
    i_asm_abort_req  = 1'b1;
    i_asm_abort_mask = i_asm_abort_mask | mask;
    if (flush) begin
      src_drop      = 1'b1;
      src_drop_keep = -1;
    end
  endtask

  task automatic skip_now(input logic [1:0] mask);
    i_asm_skip_req  = 1'b1;
    i_asm_skip_mask = mask;
  endtask

  // ----------------------------------------------------------------------------------------------
  // Cycle-by-Cycle Reference Model
  // ----------------------------------------------------------------------------------------------
  out_beat_t  hq [2];
  int         hn;
  int         ph;
  bit         lopen;
  logic [1:0] ltid;
  logic [1:0] pmask;
  logic [1:0] pass_b;
  logic [1:0] skip_b;

  bit         e_tvalid, e_fready, e_done, e_take, e_acc, e_astart, e_ahs, e_hs;
  out_beat_t  e_head;
  out_beat_t  n_hq [2];
  int         n_hn, n_ph;
  bit         n_lopen;
  logic [1:0] n_ltid, n_pmask, n_pass, n_skip;
  logic [1:0] e_tin;
  bit         e_sof0, e_sof1, e_gate, e_todo;
  int         cov_park;
  longint     n_xfer;

  task automatic model_reset();
    hn     = 0;
    ph     = ST_PASS;
    lopen  = 1'b0;
    ltid   = 2'd0;
    pmask  = 2'b00;
    pass_b = 2'b11;
    skip_b = 2'b00;
    hq[0]  = '0;
    hq[1]  = '0;
  endtask

  task automatic model_eval();
    bit         sink, hs, ahs, drained, onext, gate;
    logic [1:0] tin, pn, mn, itid, clr;
    in_beat_t   ib;
    out_beat_t  ab;

    if (!i_resetn) model_reset();
    sink     = (ph != ST_PASS);
    e_tvalid = (hn > 0);
    e_head   = (hn > 0) ? hq[0] : '0;
    e_fready = sink ? 1'b1 : (hn < 2);
    e_take   = i_resetn && i_fbs_valid && e_fready;

    tin      = i_vc_exposure_map ? {i_fbs_vc_tag[1], ~i_fbs_vc_tag[0]} : i_fbs_vc_tag;
    e_tin    = tin;
    e_sof0   = i_fbs_valid && i_fbs_sof && (tin == 2'd0);
    e_sof1   = i_fbs_valid && i_fbs_sof && (tin == 2'd1);
    pn[0]    = e_sof0 ? (!sink && !skip_b[0]) : pass_b[0];
    pn[1]    = e_sof1 ? (!sink && !skip_b[1] && pass_b[0]) : pass_b[1];
    gate     = (tin[1] == 1'b0) ? pn[tin[0]] : 1'b0;
    e_gate   = gate;
    e_acc    = e_take && !sink && i_axis_enable && gate;

    ib.data  = i_fbs_data;
    ib.be    = i_fbs_byte_en;
    ib.sof   = i_fbs_sof;
    ib.eol   = i_fbs_eol;
    ib.vc    = i_fbs_vc_tag;
    ib.err   = i_fbs_err;

    hs       = e_tvalid && i_m_axis_tready;
    ahs      = (ph == ST_INJ) && i_m_axis_tready;
    e_hs     = hs;
    e_ahs    = ahs;
    clr      = ahs ? (2'b01 << e_head.tid) : 2'b00;
    mn       = (pmask | (i_asm_abort_req ? i_asm_abort_mask : 2'b00)) & ~clr;
    onext    = ahs ? 1'b0 : (e_acc ? !i_fbs_eol : lopen);
    e_todo   = onext || (mn != 2'b00);
    drained  = (hn == 0) || ((hn == 1) && i_m_axis_tready);
    itid     = onext ? ltid : (mn[0] ? 2'd0 : 2'd1);
    e_done   = (((ph == ST_DRAIN) && drained) || ahs) && !e_todo;
    e_astart = (ph == ST_PASS) && i_asm_abort_req;

    ab.data  = 64'h0;
    ab.keep  = 8'h00;
    ab.last  = 1'b1;
    ab.user  = 2'b10;
    ab.tid   = itid;

    n_hq[0]  = hq[0];
    n_hq[1]  = hq[1];
    n_hn     = hn;
    if (hs) begin
      n_hq[0] = n_hq[1];
      n_hn--;
    end
    if ((ph == ST_PASS) && e_acc) begin
      n_hq[n_hn] = '{data: ib.data, keep: ib.be, last: ib.eol, user: {ib.err, ib.sof}, tid: tin};
      n_hn++;
    end
    if (((ph == ST_DRAIN) && drained && e_todo) || ((ph == ST_INJ) && ahs && e_todo)) begin
      n_hq[n_hn] = ab;
      n_hn++;
    end

    n_ph = ph;
    case (ph)
      ST_PASS:  if (i_asm_abort_req) n_ph = ST_DRAIN;
      ST_DRAIN: if (drained) n_ph = e_todo ? ST_INJ : ST_PASS;
      default:  if (ahs) n_ph = e_todo ? ST_INJ : ST_PASS;
    endcase

    if (e_astart || !i_axis_enable) n_pass = 2'b00;
    else if (e_take)                n_pass = pn;
    else                            n_pass = pass_b;

    for (int k = 0; k < 2; k++) begin
      if (i_asm_skip_req && i_asm_skip_mask[k])                      n_skip[k] = 1'b1;
      else if (e_take && ((k == 0) ? e_sof0 : e_sof1))               n_skip[k] = 1'b0;
      else                                                           n_skip[k] = skip_b[k];
    end
    n_lopen = onext;
    n_ltid  = e_acc ? tin : ltid;
    n_pmask = mn;
  endtask

  task automatic model_commit();
    if (!i_resetn || i_soft_reset) begin
      model_reset();
      return;
    end
    hq[0]  = n_hq[0];
    hq[1]  = n_hq[1];
    hn     = n_hn;
    ph     = n_ph;
    lopen  = n_lopen;
    ltid   = n_ltid;
    pmask  = n_pmask;
    pass_b = n_pass;
    skip_b = n_skip;
  endtask

  // ----------------------------------------------------------------------------------------------
  // Output Comparators and Protocol Checkers
  // ----------------------------------------------------------------------------------------------
  logic        prev_valid_active;
  logic        prev_tvalid_stalled;
  logic [63:0] prev_tdata;
  logic [7:0]  prev_tkeep;
  logic        prev_tlast;
  logic [1:0]  prev_tuser;
  logic [1:0]  prev_tid;
  logic        prev_fready;
  logic        prev_take;
  logic        prev_done;
  logic        prev_soft_reset;
  logic        pkt_open;
  logic [1:0]  pkt_tid;
  logic        in_line_open;
  logic [1:0]  in_line_vc;
  logic        prev_in_hold;
  logic [63:0] prev_fbs_data;
  logic [7:0]  prev_fbs_be;
  logic        prev_fbs_sof;
  logic        prev_fbs_eol;
  logic [1:0]  prev_fbs_vc;
  logic        prev_fbs_err;

  task automatic reset_protocol_trackers();
    prev_valid_active   = 1'b0;
    prev_tvalid_stalled = 1'b0;
    prev_fready         = 1'b1;
    prev_take           = 1'b0;
    prev_done           = 1'b0;
    prev_soft_reset     = 1'b0;
    pkt_open            = 1'b0;
    pkt_tid             = 2'b00;
    in_line_open        = 1'b0;
    in_line_vc          = 2'b00;
    prev_in_hold        = 1'b0;
  endtask

  // Immediate asynchronous reset check when resetn falls.
  always @(negedge i_resetn) begin
    #0.1;
    if (checking_enabled && (i_resetn === 1'b0)) begin
      check_count++;
      if (o_m_axis_tvalid !== 1'b0) begin
        report_error("async resetn did not clear o_m_axis_tvalid immediately");
      end
      if (o_asm_abort_done !== 1'b0) begin
        report_error("async resetn did not clear o_asm_abort_done immediately");
      end
    end
  end

  task automatic cmp_bit(input string label, input logic got, input logic exp);
    check_count++;
    if (got !== exp) begin
      report_error($sformatf("%s got=%b expected=%b", label, got, exp));
    end
  endtask

  task automatic check_outputs_and_protocol();
    logic [63:0] samp_tdata;
    logic        take_now, xfer_now;

    if (!checking_enabled) return;

    samp_tdata = inject_checker_fault ? (o_m_axis_tdata ^ 64'h1) : o_m_axis_tdata;

    if (!i_resetn) begin
      cmp_bit("o_m_axis_tvalid (resetn)", o_m_axis_tvalid, 1'b0);
      cmp_bit("o_asm_abort_done (resetn)", o_asm_abort_done, 1'b0);
      reset_protocol_trackers();
      return;
    end

    // Reference model comparison.
    cmp_bit("o_m_axis_tvalid", o_m_axis_tvalid, e_tvalid);
    cmp_bit("o_fbs_ready", o_fbs_ready, e_fready);
    cmp_bit("o_asm_abort_done", o_asm_abort_done, e_done);
    if (e_tvalid && (o_m_axis_tvalid === 1'b1)) begin
      check_count++;
      if ({samp_tdata, o_m_axis_tkeep, o_m_axis_tlast, o_m_axis_tuser, o_m_axis_tid} !==
          {e_head.data, e_head.keep, e_head.last, e_head.user, e_head.tid}) begin
        report_error($sformatf(
          "beat mismatch: got data=%h keep=%h last=%b user=%b tid=%0d, exp %h %h %b %b %0d",
          samp_tdata, o_m_axis_tkeep, o_m_axis_tlast, o_m_axis_tuser, o_m_axis_tid,
          e_head.data, e_head.keep, e_head.last, e_head.user, e_head.tid));
      end
    end

    // X/Z check after reset release.
    if (cyc > (rst_release_cyc + 1)) begin
      if ($isunknown({o_m_axis_tvalid, o_fbs_ready, o_asm_abort_done})) begin
        report_error("unknown value on o_m_axis_tvalid, o_fbs_ready or o_asm_abort_done");
      end
      if ((o_m_axis_tvalid === 1'b1) &&
          $isunknown({samp_tdata, o_m_axis_tkeep, o_m_axis_tlast, o_m_axis_tuser, o_m_axis_tid}))
      begin
        report_error("unknown field on a valid output beat");
      end
    end

    // Protocol rule checks on outputs.
    if (prev_soft_reset && (o_m_axis_tvalid !== 1'b0)) begin
      report_error("o_m_axis_tvalid high on the cycle after soft_reset");
    end
    if (prev_valid_active && prev_tvalid_stalled && !prev_soft_reset) begin
      if ((o_m_axis_tvalid !== 1'b1) || (samp_tdata !== prev_tdata) ||
          (o_m_axis_tkeep !== prev_tkeep) || (o_m_axis_tlast !== prev_tlast) ||
          (o_m_axis_tuser !== prev_tuser) || (o_m_axis_tid !== prev_tid)) begin
        report_error("offered output beat dropped or changed while stalled");
      end
    end
    if (o_m_axis_tvalid === 1'b1) begin
      if ((o_m_axis_tkeep == 8'h00) &&
          (!o_m_axis_tlast || (o_m_axis_tuser != 2'b10) || (samp_tdata != 64'h0))) begin
        report_error("null beat is not a well-formed abort beat");
      end
      if (!o_m_axis_tlast && (o_m_axis_tkeep != 8'hFF)) begin
        report_error("non-final beat does not keep all eight bytes");
      end
      if (o_m_axis_tlast &&
          !(o_m_axis_tkeep inside {8'h00, 8'h01, 8'h03, 8'h07, 8'h0F, 8'h1F, 8'h3F, 8'h7F, 8'hFF}))
      begin
        report_error("illegal o_m_axis_tkeep value on a TLAST beat");
      end
      if (o_m_axis_tid[1] !== 1'b0) begin
        report_error("reserved TID 2 or 3 driven on output");
      end
    end

    take_now = i_fbs_valid && o_fbs_ready;
    xfer_now = (o_m_axis_tvalid === 1'b1) && i_m_axis_tready;

    if (xfer_now && pkt_open && !prev_soft_reset && (o_m_axis_tid !== pkt_tid)) begin
      report_error("o_m_axis_tid changed inside an open packet before TLAST");
    end
    if (prev_valid_active && !prev_soft_reset) begin
      if (!o_fbs_ready && !prev_tvalid_stalled) begin
        report_error("o_fbs_ready low without a stalled output beat on the previous cycle");
      end
      if (prev_fready && !o_fbs_ready && !(prev_take && prev_tvalid_stalled)) begin
        report_error("o_fbs_ready fell without parking a beat while stalled");
      end
      if (prev_done && o_asm_abort_done) begin
        report_error("o_asm_abort_done pulse longer than one cycle");
      end
    end
    if (o_asm_abort_done && (o_m_axis_tvalid === 1'b1) && !i_m_axis_tready) begin
      report_error("o_asm_abort_done asserted while output beat is stalled");
    end

    // Input stimulus discipline checks (enabled during standard tests).
    if (prev_in_hold && (!i_fbs_valid || (i_fbs_data !== prev_fbs_data) ||
        (i_fbs_byte_en !== prev_fbs_be) || (i_fbs_sof !== prev_fbs_sof) ||
        (i_fbs_eol !== prev_fbs_eol) || (i_fbs_vc_tag !== prev_fbs_vc) ||
        (i_fbs_err !== prev_fbs_err))) begin
      report_error("FWFT source changed while stalled without flush or soft_reset");
    end
    if (stim_checks_on) begin
      if (i_fbs_valid && (i_fbs_vc_tag[1] != 1'b0)) begin
        report_error("stimulus presented reserved VC2/VC3 while standard check is active");
      end
      if (i_asm_abort_req && (i_asm_abort_mask == 2'b00)) begin
        report_error("stimulus issued zero abort mask while standard check is active");
      end
      if (i_asm_skip_req && (i_asm_skip_mask != 2'b01)) begin
        report_error("stimulus issued non-01 skip mask while standard check is active");
      end
    end
    if (take_now && in_line_open && !i_soft_reset && !i_asm_abort_req &&
        (i_fbs_vc_tag != in_line_vc)) begin
      report_error("input VC changed mid-line without flush or soft_reset");
    end

    // Update trackers for the next cycle.
    if (i_soft_reset) begin
      reset_protocol_trackers();
      prev_soft_reset = 1'b1;
    end else begin
      prev_valid_active   = 1'b1;
      prev_soft_reset     = 1'b0;
      prev_tvalid_stalled = (o_m_axis_tvalid === 1'b1) && !i_m_axis_tready;
      prev_tdata          = samp_tdata;
      prev_tkeep          = o_m_axis_tkeep;
      prev_tlast          = o_m_axis_tlast;
      prev_tuser          = o_m_axis_tuser;
      prev_tid            = o_m_axis_tid;
      prev_fready         = o_fbs_ready;
      prev_take           = take_now;
      prev_done           = o_asm_abort_done;
      if (xfer_now) begin
        pkt_open = !o_m_axis_tlast;
        pkt_tid  = o_m_axis_tid;
      end
      if (i_asm_abort_req) begin
        in_line_open = 1'b0;
      end else if (take_now) begin
        in_line_open = !i_fbs_eol;
        in_line_vc   = i_fbs_vc_tag;
      end
      prev_in_hold  = i_fbs_valid && !o_fbs_ready && !i_asm_abort_req;
      prev_fbs_data = i_fbs_data;
      prev_fbs_be   = i_fbs_byte_en;
      prev_fbs_sof  = i_fbs_sof;
      prev_fbs_eol  = i_fbs_eol;
      prev_fbs_vc   = i_fbs_vc_tag;
      prev_fbs_err  = i_fbs_err;
    end
  endtask

  // ----------------------------------------------------------------------------------------------
  // Central Cycle Loop
  // ----------------------------------------------------------------------------------------------
  initial begin : p_cycle
    @(negedge i_axis_aclk);
    forever begin
      drive_cmds();
      drive_src();
      drive_ready();
      #2.0;
      model_eval();
      #2.0;
      last_take = i_fbs_valid && o_fbs_ready && i_resetn;
      check_outputs_and_protocol();
      if (i_resetn && !e_fready) cov_park++;
      if (last_take) total_takes++;
      if (i_resetn && (o_m_axis_tvalid === 1'b1) && i_m_axis_tready) n_xfer++;
      if (i_soft_reset && i_resetn) begin
        src_drop      = 1'b1;
        src_drop_keep = -1;
      end
      model_commit();
      ae_c2 = ae_c1;
      ae_c1 = i_axis_enable;
      cyc++;
      if (int'(cyc) > max_cycles) begin
        $display("%s FAIL (timeout)", test_name);
        $finish;
      end
      @(negedge i_axis_aclk);
    end
  end

  // ----------------------------------------------------------------------------------------------
  // Stimulus Helper Tasks
  // ----------------------------------------------------------------------------------------------
  task automatic gen_line(input logic [1:0] vc, input int nbytes, input bit sof, input bit err);
    in_beat_t b;
    int       left;
    left = nbytes;
    while (left > 0) begin
      for (int k = 0; k < 8; k++) b.data[8*k +: 8] = 8'(rnd(256));
      b.be  = (left >= 8) ? 8'hFF : 8'((9'd1 << left) - 9'd1);
      b.sof = sof && (left == nbytes);
      b.eol = (left <= 8);
      b.vc  = vc;
      b.err = err && (left <= 8);
      src_q.push_back(b);
      left -= 8;
    end
  endtask

  task automatic gen_partial(input logic [1:0] vc, input int nbeats, input bit sof);
    in_beat_t b;
    for (int i = 0; i < nbeats; i++) begin
      for (int k = 0; k < 8; k++) b.data[8*k +: 8] = 8'(rnd(256));
      b.be  = 8'hFF;
      b.sof = sof && (i == 0);
      b.eol = 1'b0;
      b.vc  = vc;
      b.err = 1'b0;
      src_q.push_back(b);
    end
  endtask

  task automatic wait_src_empty(input int max_wait);
    int n;
    n = 0;
    while (((src_q.size() > 0) || cur_valid) && (n < max_wait)) begin
      tick();
      n++;
    end
    if ((src_q.size() > 0) || cur_valid) report_error("wait_src_empty: source not consumed");
  endtask

  task automatic gen_frame(
    input logic [1:0] vc, input int nlines, input int lo, input int hi, input int err_pct
  );
    for (int l = 0; l < nlines; l++) begin
      gen_line(vc, urange(lo, hi), l == 0, rnd(100) < err_pct);
    end
  endtask

  task automatic gen_capture(
    input int nlong, input int nshort, input int lo, input int hi, input int err_pct
  );
    logic [1:0] vc_l, vc_s;
    int         il, is_cnt;
    vc_l   = i_vc_exposure_map ? 2'd1 : 2'd0;
    vc_s   = i_vc_exposure_map ? 2'd0 : 2'd1;
    il     = 0;
    is_cnt = 0;
    while ((il < nlong) || (is_cnt < nshort)) begin
      if ((is_cnt < nshort) && (il > 0) && ((il >= nlong) || (rnd(2) == 1))) begin
        gen_line(vc_s, urange(lo, hi), is_cnt == 0, rnd(100) < err_pct);
        is_cnt++;
      end else begin
        gen_line(vc_l, urange(lo, hi), il == 0, rnd(100) < err_pct);
        il++;
      end
    end
  endtask

  task automatic gen_single(input int nlines, input int lo, input int hi);
    gen_frame(i_vc_exposure_map ? 2'd1 : 2'd0, nlines, lo, hi, 10);
  endtask

  function automatic bit model_idle();
    return (src_q.size() == 0) && !cur_valid && (hn == 0) && (ph == ST_PASS);
  endfunction

  task automatic wait_idle(input int max_wait);
    int n;
    n = 0;
    while (!model_idle()) begin
      tick();
      n++;
      if (n == (max_wait / 2)) begin
        rdy_force    = 1;
        rdy_low_left = 0;
      end
      if (n > max_wait) begin
        report_error($sformatf("wait_idle timeout after %0d cycles", max_wait));
        return;
      end
    end
    tick();
  endtask

  task automatic wait_cycles(input int n);
    repeat (n) tick();
  endtask

  task automatic wait_takes(input int ntakes, input int max_wait);
    longint t0;
    int     n;
    t0 = total_takes;
    n  = 0;
    while (((total_takes - t0) < ntakes) && (n < max_wait)) begin
      tick();
      n++;
    end
    if ((total_takes - t0) < ntakes) begin
      report_error($sformatf("wait_takes: %0d of %0d takes in %0d cycles",
                             total_takes - t0, ntakes, max_wait));
    end
  endtask

  // Waits for up to ntakes input transfers and stops early once the source is empty and idle.
  task automatic wait_takes_upto(input int ntakes, input int max_wait);
    longint t0;
    int     n;
    t0 = total_takes;
    n  = 0;
    while (((total_takes - t0) < ntakes) && (n < max_wait) && !model_idle()) begin
      tick();
      n++;
    end
  endtask

  // Reports a stimulus precondition that was not reached, so no scenario is skipped silently.
  task automatic need(input bit ok, input string what);
    if (!ok) report_error($sformatf("precondition not reached: %s", what));
  endtask

  function automatic bit exp_ready();
    return (ph != ST_PASS) || (hn < 2);
  endfunction

  function automatic bit beat_kind(input int kind);
    if (!(i_fbs_valid && exp_ready() && (ph == ST_PASS))) return 1'b0;
    case (kind)
      0:       return !i_fbs_sof && !i_fbs_eol;
      1:       return i_fbs_sof;
      default: return i_fbs_eol;
    endcase
  endfunction

  task automatic wait_take_kind(input int kind, input int max_wait, output bit ok);
    int n;
    n  = 0;
    ok = 1'b0;
    while (n < max_wait) begin
      if (beat_kind(kind)) begin
        ok = 1'b1;
        return;
      end
      tick();
      n++;
    end
  endtask

  // Waits for a take of the given kind. busy = 1 also needs exactly one beat in out_* and drives
  // TREADY low on that cycle (the beat is parked); busy = 0 drives TREADY high (out_* free).
  task automatic wait_take_at(input int kind, input int busy, input int max_wait, output bit ok);
    int n;
    n  = 0;
    ok = 1'b0;
    while (n < max_wait) begin
      if (beat_kind(kind) && ((busy == 0) || (hn == 1))) begin
        i_m_axis_tready = (busy == 0);
        ok              = 1'b1;
        return;
      end
      tick();
      n++;
    end
  endtask

  task automatic wait_phase(input int p, input int max_wait, output bit ok);
    int n;
    n  = 0;
    ok = 1'b0;
    while (n < max_wait) begin
      if (ph == p) begin
        ok = 1'b1;
        return;
      end
      tick();
      n++;
    end
  endtask

  task automatic wait_held(input int nh, input int max_wait, output bit ok);
    int n;
    n  = 0;
    ok = 1'b0;
    while (n < max_wait) begin
      if ((hn == nh) && (ph == ST_PASS)) begin
        ok = 1'b1;
        return;
      end
      tick();
      n++;
    end
  endtask

  // Asserts resetn mid-cycle (away from clock edges) and releases it at negedge.
  task automatic do_reset(input int ncyc);
    #1.3;
    i_resetn        = 1'b0;
    cur_valid       = 1'b0;
    src_q.delete();
    drive_src_fields();
    i_asm_abort_req = 1'b0;
    i_asm_skip_req  = 1'b0;
    model_reset();
    reset_protocol_trackers();
    repeat (ncyc) tick();
    @(negedge i_axis_aclk);
    #0.5;
    i_resetn        = 1'b1;
    rst_release_cyc = int'(cyc);
    #0.5;
  endtask

  task automatic do_soft_reset(input int ncyc);
    i_soft_reset = 1'b1;
    repeat (ncyc) tick();
    i_soft_reset = 1'b0;
  endtask

  task automatic set_axis_enable(input bit v);
    i_axis_enable = v;
  endtask

  // ----------------------------------------------------------------------------------------------
  // Test Cases
  // ----------------------------------------------------------------------------------------------

  // Checks reset outputs, both pass bits enabled after reset, and resetn with two beats held.
  task automatic test_reset_and_idle();
    bit ok;
    wait_cycles(20);
    gen_frame(2'd1, 3, 8, 40, 0);
    wait_idle(3000);
    do_reset(3);
    gen_frame(2'd0, 3, 1, 30, 0);
    wait_idle(3000);
    set_ready(0, 0);
    gen_frame(2'd0, 2, 64, 64, 0);
    wait_held(2, 400, ok);
    if (!ok) report_error("two beats not held before resetn");
    do_reset(2);
    set_ready(100, 0);
    gen_capture(2, 2, 8, 24, 0);
    wait_idle(4000);
  endtask

  // Checks full-rate passthrough with 1-beat lines, 1..8 last-beat byte counts and 1-cycle latency.
  task automatic test_passthrough_full_rate();
    set_ready(100, 0);
    src_gap_pct = 0;
    cov_park    = 0;
    gen_line(2'd0, 5, 1'b1, 1'b0);
    for (int b = 1; b <= 24; b++) gen_line(2'd0, b, 1'b0, (b % 5) == 0);
    gen_frame(2'd0, 8, 8, 200, 20);
    gen_frame(2'd0, 4, 64, 64, 0);
    wait_idle(6000);
    if (cov_park != 0) begin
      report_error($sformatf("fbs_ready went low on %0d cycles with TREADY always 1", cov_park));
    end
  endtask

  // Checks random TREADY backpressure (25/50/75/90%), source gaps, HDR interleaving and VC swap.
  task automatic test_random_backpressure();
    int pcts [4];
    pcts[0] = 25;
    pcts[1] = 50;
    pcts[2] = 75;
    pcts[3] = 90;
    for (int m = 0; m < 2; m++) begin
      i_vc_exposure_map = m[0];
      for (int i = 0; i < 4; i++) begin
        set_ready(pcts[i], 0);
        src_gap_pct = ((i % 2) == 1) ? 30 : 0;
        gen_capture(3, 3, 8, 120, 10);
        gen_capture(2, 4, 1, 64, 10);
        wait_idle(30000);
      end
    end
    src_gap_pct       = 0;
    i_vc_exposure_map = 1'b0;
  endtask

  // Checks long TREADY stalls, every-cycle TREADY toggling, and TREADY falls with/without a take.
  task automatic test_stall_patterns();
    set_ready(95, 300);
    gen_capture(4, 4, 16, 200, 0);
    wait_idle(60000);
    rdy_force = 2;
    gen_capture(3, 3, 8, 100, 0);
    wait_idle(30000);
    set_ready(100, 0);
    gen_frame(2'd0, 2, 160, 160, 0);
    wait_takes(5, 200);
    rdy_force = 0;
    wait_cycles(400);
    set_ready(100, 0);
    wait_idle(4000);
    src_gap_pct = 50;
    set_ready(60, 20);
    gen_capture(3, 3, 8, 80, 0);
    wait_idle(30000);
    src_gap_pct = 0;
  endtask

  // Checks axis_enable cleared mid-line with beats held and Frame Start taken while disabled.
  task automatic test_axis_enable_gating();
    set_ready(100, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_frame(2'd0, 3, 40, 40, 0);
    gen_line(2'd0, 24, 1'b1, 1'b0);
    gen_line(2'd0, 24, 1'b0, 1'b0);
    wait_takes(2, 400);
    set_axis_enable(1'b0);
    wait_cycles(30);
    set_ready(70, 0);
    wait_idle(6000);
    set_axis_enable(1'b1);
    gen_line(2'd0, 24, 1'b0, 1'b0);
    gen_line(2'd0, 16, 1'b0, 1'b0);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(6000);
    set_ready(100, 0);
    gen_single(6, 40, 80);
    wait_takes(8, 400);
    set_axis_enable(1'b0);
    wait_takes(6, 400);
    set_axis_enable(1'b1);
    wait_idle(8000);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    set_ready(100, 0);
    gen_frame(2'd0, 3, 40, 40, 0);
    gen_line(2'd0, 24, 1'b1, 1'b0);
    wait_takes(3, 400);
    set_axis_enable(1'b0);
    wait_src_empty(400);
    wait_idle(400);
    set_axis_enable(1'b1);
    gen_line(2'd0, 24, 1'b0, 1'b0);
    gen_line(2'd0, 24, 1'b0, 1'b0);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
  endtask

  // Checks abort sequences with masks 11 and 01, open lines on TID 0/1, and 0/1/2 held beats.
  task automatic test_abort_sequences();
    bit ok;
    set_ready(100, 0);
    wait_cycles(5);
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle(300);
    abort_now(2'b01, 1'b0);
    tick();
    wait_idle(300);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    gen_partial(2'd0, 3, 1'b1);
    wait_src_empty(400);
    wait_cycles(4);
    abort_now(2'b11, 1'b1);
    tick();
    wait_idle(400);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_partial(2'd0, 1, 1'b1);
    wait_src_empty(400);
    wait_cycles(3);
    abort_now(2'b11, 1'b1);
    wait_cycles(10);
    set_ready(100, 0);
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_partial(2'd0, 5, 1'b1);
    wait_held(2, 400, ok);
    if (!ok) report_error("two beats not held");
    abort_now(2'b11, 1'b1);
    wait_cycles(10);
    set_ready(100, 0);
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    gen_line(2'd0, 16, 1'b1, 1'b0);
    gen_partial(2'd1, 3, 1'b1);
    wait_src_empty(400);
    wait_cycles(2);
    abort_now(2'b11, 1'b1);
    tick();
    wait_idle(400);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    gen_partial(2'd0, 4, 1'b1);
    wait_src_empty(400);
    abort_now(2'b01, 1'b1);
    tick();
    wait_idle(400);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle(400);
    gen_capture(3, 3, 8, 64, 10);
    wait_idle(6000);
  endtask

  // Checks abort under TREADY stall during DRAIN and INJECT and at the last-real-beat boundary.
  task automatic test_abort_under_backpressure();
    bit ok;
    rdy_force = 0;
    gen_partial(2'd0, 6, 1'b1);
    wait_held(2, 400, ok);
    if (!ok) report_error("two beats not held");
    abort_now(2'b11, 1'b1);
    tick();
    gen_capture(2, 2, 8, 40, 0);
    wait_cycles(200);
    rdy_force = 2;
    wait_phase(ST_INJ, 400, ok);
    if (!ok) report_error("INJECT not reached");
    rdy_force = 0;
    wait_cycles(300);
    set_ready(100, 0);
    wait_idle(8000);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_line(2'd0, 16, 1'b1, 1'b0);
    gen_partial(2'd1, 3, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b11, 1'b1);
    tick();
    rdy_force = 1;
    tick();
    rdy_force = 0;
    wait_cycles(5);
    rdy_force = 1;
    tick();
    rdy_force = 0;
    wait_cycles(150);
    set_ready(50, 0);
    wait_idle(4000);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
  endtask

  // Checks asm_abort_req on a mid-line, Frame Start or TLAST beat take with out free and out busy;
  // the rest of the cut frames keeps arriving and must be discarded by the frame gate.
  task automatic test_abort_on_take_cycle();
    bit ok;
    for (int busy = 0; busy < 2; busy++) begin
      for (int kind = 0; kind < 3; kind++) begin
        set_ready(busy ? 50 : 100, 0);
        gen_capture(3, 3, 24, 40, 0);
        gen_capture(3, 3, 24, 40, 0);
        wait_take_at(kind, busy, 4000, ok);
        need(ok, $sformatf("take of kind %0d with out busy = %0d", kind, busy));
        if (ok) abort_now(2'b11, 1'b0);
        tick();
        set_ready(100, 0);
        wait_idle(6000);
      end
    end
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
  endtask

  // Checks asm_abort_req re-assertion during DRAIN, INJECT and on DRAIN/INJECT exit cycles.
  task automatic test_abort_reassertion();
    bit ok;
    int n;
    rdy_force = 0;
    gen_partial(2'd0, 4, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b01, 1'b1);
    tick();
    wait_cycles(3);
    abort_now(2'b10, 1'b0);
    tick();
    set_ready(100, 0);
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);

    rdy_force = 0;
    gen_partial(2'd0, 2, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b01, 1'b1);
    tick();
    rdy_force = 1;
    wait_phase(ST_INJ, 100, ok);
    rdy_force = 0;
    if (!ok) report_error("INJECT not reached");
    i_m_axis_tready = 1'b0;
    abort_now(2'b01, 1'b0);
    tick();
    wait_cycles(4);
    abort_now(2'b10, 1'b0);
    tick();
    set_ready(100, 0);
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);

    rdy_force = 0;
    gen_line(2'd0, 8, 1'b1, 1'b0);
    wait_src_empty(200);
    wait_cycles(2);
    abort_now(2'b11, 1'b0);
    tick();
    wait_cycles(3);
    if (!((ph == ST_DRAIN) && (hn == 1))) report_error("not in DRAIN with one beat held");
    i_m_axis_tready = 1'b1;
    rdy_force       = 1;
    abort_now(2'b01, 1'b0);
    tick();
    set_ready(100, 0);
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);

    for (int v = 0; v < 2; v++) begin
      rdy_force = 0;
      abort_now(2'b11, 1'b0);
      tick();
      n = 0;
      while (!((ph == ST_INJ) && (hn == 1) && (hq[0].tid == 2'd1)) && (n < 200)) begin
        if ((ph == ST_INJ) && (hn == 1) && (hq[0].tid == 2'd0)) i_m_axis_tready = 1'b1;
        tick();
        n++;
      end
      if (n >= 200) report_error("last abort beat not reached");
      i_m_axis_tready = 1'b1;
      rdy_force       = 1;
      abort_now((v == 0) ? 2'b10 : 2'b01, 1'b0);
      tick();
      set_ready(100, 0);
      wait_idle(600);
    end
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
  endtask

  // Checks frame gate discarding remainder after abort, SOF during sink, and Short before Long.
  task automatic test_frame_gate_after_abort();
    bit ok;
    set_ready(100, 0);
    gen_capture(4, 4, 24, 60, 0);
    wait_takes(12, 600);
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle(8000);
    gen_capture(2, 2, 16, 32, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_partial(2'd0, 3, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b11, 1'b1);
    tick();
    gen_capture(2, 2, 16, 32, 0);
    wait_cycles(60);
    set_ready(100, 0);
    wait_idle(6000);
    gen_frame(2'd1, 2, 16, 24, 0);
    gen_capture(2, 2, 16, 32, 0);
    wait_idle(6000);
    gen_line(2'd0, 16, 1'b1, 1'b0);
    gen_line(2'd0, 16, 1'b0, 1'b0);
    wait_src_empty(400);
    wait_cycles(3);
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle(400);
    gen_line(2'd1, 16, 1'b1, 1'b0);
    gen_line(2'd0, 16, 1'b0, 1'b0);
    gen_line(2'd1, 16, 1'b0, 1'b0);
    gen_capture(2, 2, 16, 32, 0);
    wait_idle(6000);
  endtask

  // Checks frame skip (mask 01), set-wins priority on same-cycle SOF take, and skip during abort.
  task automatic test_frame_skip();
    bit ok;
    set_ready(100, 0);
    gen_capture(3, 3, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    if (!ok) report_error("no Frame Start take");
    tick();
    skip_now(2'b01);
    gen_capture(3, 3, 16, 40, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(10000);
    gen_capture(2, 2, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    if (!ok) report_error("no Frame Start take");
    skip_now(2'b01);
    tick();
    gen_capture(2, 2, 16, 40, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(10000);
    gen_capture(2, 2, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    need(ok, "Frame Start take");
    tick();
    skip_now(2'b01);
    wait_takes(4, 400);
    rdy_force = 0;
    abort_now(2'b11, 1'b1);
    tick();
    gen_capture(2, 2, 16, 32, 0);
    wait_cycles(80);
    set_ready(100, 0);
    wait_idle(6000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
  endtask

  // Checks single-channel TPG operation with vc_exposure_map = 0, TID 0 only and abort mask 01.
  task automatic test_single_channel_tpg_mode();
    abort_mask_cfg = 2'b01;
    set_ready(80, 0);
    gen_single(4, 16, 80);
    gen_single(4, 16, 80);
    wait_takes(10, 1000);
    abort_now(2'b01, 1'b1);
    tick();
    wait_idle(4000);
    gen_single(4, 16, 80);
    wait_takes(6, 1000);
    set_axis_enable(1'b0);
    wait_cycles(20);
    set_axis_enable(1'b1);
    wait_idle(6000);
    gen_single(3, 16, 80);
    wait_idle(4000);
    abort_now(2'b01, 1'b0);
    tick();
    wait_idle(400);
    gen_single(3, 16, 80);
    wait_idle(4000);
    abort_mask_cfg = 2'b11;
  endtask

  // Checks vc_exposure_map = 1 swapping VC0/VC1 onto TID 1/0 for data beats and line-closing abort.
  task automatic test_vc_exposure_swap();
    i_vc_exposure_map = 1'b1;
    set_ready(85, 0);
    gen_capture(3, 3, 8, 64, 10);
    wait_idle(8000);
    gen_partial(2'd1, 3, 1'b1);
    wait_src_empty(400);
    abort_now(2'b11, 1'b1);
    tick();
    wait_idle(600);
    gen_line(2'd1, 16, 1'b1, 1'b0);
    gen_partial(2'd0, 2, 1'b1);
    wait_src_empty(400);
    abort_now(2'b11, 1'b1);
    tick();
    wait_idle(600);
    gen_capture(2, 2, 8, 64, 10);
    wait_idle(6000);
    i_vc_exposure_map = 1'b0;
  endtask

  // Checks soft_reset across all states, coinciding with commands/handshakes, and post-reset state.
  task automatic test_soft_reset();
    bit ok;
    set_ready(100, 0);
    gen_capture(3, 3, 24, 60, 0);
    wait_takes(10, 600);
    do_soft_reset(1);
    wait_idle(4000);
    gen_capture(2, 2, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    need(ok, "Frame Start take");
    i_soft_reset = 1'b1;
    abort_now(2'b11, 1'b0);
    skip_now(2'b01);
    tick();
    i_soft_reset = 1'b0;
    wait_idle(4000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    rdy_force = 0;
    gen_partial(2'd0, 4, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b11, 1'b1);
    tick();
    wait_cycles(3);
    do_soft_reset(5);
    set_ready(100, 0);
    wait_idle(2000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    rdy_force = 0;
    gen_partial(2'd1, 1, 1'b0);
    wait_src_empty(400);
    abort_now(2'b11, 1'b1);
    tick();
    rdy_force = 1;
    wait_phase(ST_INJ, 100, ok);
    need(ok, "INJECT reached");
    rdy_force = 1;
    do_soft_reset(1);
    set_ready(100, 0);
    wait_idle(2000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    set_ready(100, 0);
    gen_capture(2, 2, 40, 80, 0);
    wait_takes(7, 600);
    do_soft_reset(2);
    wait_idle(4000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    do_soft_reset(1);
    wait_cycles(2);
    gen_frame(2'd1, 2, 16, 24, 0);
    wait_idle(4000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    gen_line(2'd0, 16, 1'b1, 1'b0);
    gen_line(2'd1, 16, 1'b1, 1'b0);
    wait_idle(4000);
    do_soft_reset(1);
    wait_cycles(2);
    abort_now(2'b01, 1'b0);
    tick();
    wait_idle(600);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    // One cycle in DRAIN with a TID 0 line open; the next abort (mask 10) sends one TID 1 beat.
    rdy_force = 0;
    gen_partial(2'd0, 4, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b11, 1'b1);
    tick();
    wait_cycles(2);
    need(ph == ST_DRAIN, "DRAIN before soft_reset");
    do_soft_reset(1);
    set_ready(100, 0);
    wait_cycles(2);
    abort_now(2'b10, 1'b0);
    tick();
    wait_idle(600);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    // Three cycles in INJECT with mask 11 pending; the next abort (mask 01) sends one TID 0 beat.
    rdy_force = 0;
    abort_now(2'b11, 1'b0);
    tick();
    wait_phase(ST_INJ, 100, ok);
    need(ok, "INJECT reached");
    wait_cycles(2);
    do_soft_reset(3);
    set_ready(100, 0);
    wait_cycles(2);
    abort_now(2'b01, 1'b0);
    tick();
    wait_idle(600);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);

    // Skip armed for the next Long frame, then soft_reset: the next capture passes whole.
    gen_capture(2, 2, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    need(ok, "Frame Start take");
    tick();
    skip_now(2'b01);
    tick();
    wait_idle(4000);
    do_soft_reset(1);
    wait_cycles(2);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
  endtask

  // Checks asynchronous resetn assertion mid-line, with held beats, in DRAIN/INJECT and with skip.
  task automatic test_async_resetn();
    bit ok;
    set_ready(100, 0);
    gen_capture(3, 3, 24, 60, 0);
    wait_takes(10, 600);
    do_reset(2);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_partial(2'd0, 4, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    do_reset(3);
    set_ready(100, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_partial(2'd0, 4, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b11, 1'b1);
    tick();
    wait_cycles(3);
    do_reset(2);
    set_ready(100, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    abort_now(2'b11, 1'b0);
    tick();
    wait_phase(ST_INJ, 100, ok);
    need(ok, "INJECT reached");
    wait_cycles(5);
    do_reset(2);
    set_ready(100, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    gen_capture(2, 2, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    need(ok, "Frame Start take");
    tick();
    skip_now(2'b01);
    tick();
    wait_idle(4000);
    do_reset(2);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
  endtask

  // Checks upstream Format Bridge flush coinciding with asm_abort_req when taken, stalled or TPG.
  task automatic test_upstream_flush();
    bit ok;
    set_ready(100, 0);
    gen_capture(3, 3, 32, 64, 0);
    wait_take_kind(0, 2000, ok);
    need(ok, "mid-line take");
    abort_now(2'b11, 1'b1);
    tick();
    wait_idle(4000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    rdy_force = 0;
    gen_partial(2'd0, 4, 1'b1);
    wait_held(2, 400, ok);
    need(ok, "two beats held");
    abort_now(2'b11, 1'b1);
    tick();
    wait_cycles(5);
    set_ready(100, 0);
    wait_idle(4000);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(4000);
    abort_mask_cfg = 2'b01;
    gen_single(6, 32, 64);
    wait_takes(12, 1000);
    abort_now(2'b01, 1'b0);
    src_drop      = 1'b1;
    src_drop_keep = 20;
    tick();
    wait_idle(6000);
    gen_single(3, 32, 64);
    wait_idle(4000);
    abort_mask_cfg = 2'b11;
  endtask

  // Checks long random streaming with HDR/single captures, stalls, aborts, skips and resets.
  task automatic test_random_streaming();
    bit ok;
    int ev;
    for (int c = 0; c < 40; c++) begin
      if (model_idle() && (rnd(4) == 0)) i_vc_exposure_map = !i_vc_exposure_map;
      case (rnd(5))
        0: set_ready(100, 0);
        1: set_ready(urange(20, 95), 0);
        2: set_ready(90, 200);
        3: begin
          set_ready(100, 0);
          rdy_force = 2;
        end
        default: set_ready(urange(40, 80), 30);
      endcase
      src_gap_pct = (rnd(3) == 0) ? urange(10, 60) : 0;
      if (rnd(3) == 0) gen_single(urange(1, 4), 1, 120);
      else             gen_capture(urange(1, 4), urange(1, 4), 1, 120, 10);
      ev = rnd(8);
      if (ev == 0) begin
        wait_takes_upto(urange(1, 20), 4000);
        abort_now(2'b11, rnd(2) == 1);
        tick();
      end else if (ev == 1) begin
        wait_take_kind(1, 4000, ok);
        if (ok) begin
          tick();
          skip_now(2'b01);
        end
        gen_capture(urange(1, 3), urange(1, 3), 1, 80, 0);
      end else if (ev == 2) begin
        wait_takes_upto(urange(1, 20), 4000);
        set_axis_enable(1'b0);
        wait_cycles(urange(1, 60));
        set_axis_enable(1'b1);
      end else if ((ev == 3) && ((c % 5) == 0)) begin
        wait_takes_upto(urange(1, 20), 4000);
        do_soft_reset(urange(1, 3));
      end
      wait_idle(60000);
      src_gap_pct = 0;
    end
  endtask

  // Checks robustness with asm_abort_mask = 00 and mask = 10 when a line is open on TID 0.
  task automatic test_abort_mask_robustness();
    bit ok;
    stim_checks_on = 1'b0;
    set_ready(100, 0);
    wait_cycles(4);
    abort_now(2'b00, 1'b0);
    tick();
    wait_idle(300);
    rdy_force = 0;
    gen_partial(2'd0, 3, 1'b1);
    wait_held(2, 400, ok);
    need(ok && lopen, "two beats held with a TID 0 line open");
    abort_now(2'b00, 1'b0);
    tick();
    wait_cycles(6);
    set_ready(100, 0);
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    gen_partial(2'd0, 3, 1'b1);
    wait_src_empty(400);
    abort_now(2'b00, 1'b1);
    tick();
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    gen_partial(2'd0, 3, 1'b1);
    wait_src_empty(400);
    abort_now(2'b10, 1'b1);
    tick();
    wait_idle(600);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(4000);
    stim_checks_on = 1'b1;
  endtask

  // Checks robustness with VC2/VC3 frames, skip masks 10/11 and TID 1 beats right after reset.
  task automatic test_vc_and_skip_robustness();
    bit ok;
    stim_checks_on = 1'b0;
    set_ready(80, 0);
    gen_capture(2, 2, 8, 40, 0);
    gen_frame(2'd2, 2, 8, 40, 0);
    gen_frame(2'd3, 2, 8, 40, 0);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle(8000);
    i_vc_exposure_map = 1'b1;
    gen_frame(2'd2, 2, 8, 40, 0);
    gen_capture(2, 2, 8, 40, 0);
    gen_frame(2'd3, 1, 8, 40, 0);
    wait_idle(8000);
    i_vc_exposure_map = 1'b0;
    gen_capture(2, 2, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    need(ok, "Frame Start take");
    tick();
    skip_now(2'b10);
    gen_capture(2, 2, 16, 40, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(8000);
    gen_capture(2, 2, 16, 40, 0);
    wait_take_kind(1, 2000, ok);
    need(ok, "Frame Start take");
    tick();
    skip_now(2'b11);
    gen_capture(2, 2, 16, 40, 0);
    gen_capture(2, 2, 16, 40, 0);
    wait_idle(8000);
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle(400);
    gen_frame(2'd1, 2, 16, 24, 0);
    gen_capture(2, 2, 16, 32, 0);
    wait_idle(6000);
    do_reset(2);
    set_ready(100, 0);
    gen_line(2'd1, 16, 1'b0, 1'b0);
    gen_line(2'd1, 12, 1'b0, 1'b0);
    wait_idle(2000);
    gen_capture(2, 2, 16, 32, 0);
    wait_idle(4000);
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle(400);
    do_soft_reset(1);
    wait_cycles(2);
    gen_line(2'd1, 16, 1'b0, 1'b0);
    gen_line(2'd0, 16, 1'b0, 1'b0);
    wait_idle(2000);
    gen_capture(2, 2, 16, 32, 0);
    wait_idle(4000);
    stim_checks_on = 1'b1;
  endtask

  // Checks robustness when axis_enable is cleared and re-enabled without asm_abort_req.
  task automatic test_disable_without_abort();
    bit ok;
    stim_checks_on = 1'b0;
    auto_abort     = 1'b0;
    set_ready(100, 0);
    gen_single(6, 24, 24);
    wait_takes(4, 400);
    wait_take_kind(2, 2000, ok);
    need(ok, "TLAST take");
    tick();
    set_axis_enable(1'b0);
    wait_takes(3, 400);
    set_axis_enable(1'b1);
    wait_idle(6000);
    gen_capture(2, 2, 16, 32, 0);
    wait_idle(6000);
    gen_single(4, 40, 40);
    wait_takes(7, 400);
    set_axis_enable(1'b0);
    wait_cycles(10);
    set_axis_enable(1'b1);
    wait_idle(6000);
    gen_single(2, 16, 32);
    wait_idle(4000);
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle(600);
    auto_abort     = 1'b1;
    stim_checks_on = 1'b1;
  endtask

  // Perturbs the sampled output data on purpose; this test must end in FAIL with errors.
  task automatic test_checker_fault();
    set_ready(100, 0);
    gen_line(2'd0, 32, 1'b1, 1'b0);
    wait_takes(1, 100);
    inject_checker_fault = 1'b1;
    tick();
    tick();
    inject_checker_fault = 1'b0;
    wait_idle(500);
  endtask

  // ----------------------------------------------------------------------------------------------
  // Test Selection and Final Summary
  // ----------------------------------------------------------------------------------------------
  initial begin : p_main
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "test_reset_and_idle";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    if (!$value$plusargs("MAX_CYCLES=%d", max_cycles)) max_cycles = WAIT_LIMIT;
    void'($urandom(seed));

    err_count            = 0;
    check_count          = 0;
    cyc                  = 0;
    total_takes          = 0;
    n_xfer               = 0;
    cov_park             = 0;
    rst_release_cyc      = 0;
    checking_enabled     = 1'b0;
    stim_checks_on       = 1'b1;
    inject_checker_fault = 1'b0;
    cur_valid            = 1'b0;
    last_take            = 1'b0;
    src_drop             = 1'b0;
    src_drop_keep        = -1;
    src_gap_pct          = 0;
    set_ready(100, 0);
    rdy_toggle           = 1'b0;
    auto_abort           = 1'b1;
    abort_mask_cfg       = 2'b11;
    ae_c1                = 1'b1;
    ae_c2                = 1'b1;

    model_reset();
    reset_protocol_trackers();
    i_resetn          = 1'b0;
    i_soft_reset      = 1'b0;
    i_asm_abort_req   = 1'b0;
    i_asm_abort_mask  = 2'b00;
    i_asm_skip_req    = 1'b0;
    i_asm_skip_mask   = 2'b00;
    i_axis_enable     = 1'b1;
    i_vc_exposure_map = 1'b0;
    i_m_axis_tready   = 1'b1;
    drive_src_fields();

    repeat (2) tick();
    checking_enabled = 1'b1;
    repeat (2) tick();
    @(negedge i_axis_aclk);
    #0.5;
    i_resetn        = 1'b1;
    rst_release_cyc = int'(cyc);
    #0.5;

    case (test_name)
      "test_reset_and_idle":           test_reset_and_idle();
      "test_passthrough_full_rate":    test_passthrough_full_rate();
      "test_random_backpressure":      test_random_backpressure();
      "test_stall_patterns":           test_stall_patterns();
      "test_axis_enable_gating":       test_axis_enable_gating();
      "test_abort_sequences":          test_abort_sequences();
      "test_abort_under_backpressure": test_abort_under_backpressure();
      "test_abort_on_take_cycle":      test_abort_on_take_cycle();
      "test_abort_reassertion":        test_abort_reassertion();
      "test_frame_gate_after_abort":   test_frame_gate_after_abort();
      "test_frame_skip":               test_frame_skip();
      "test_single_channel_tpg_mode":  test_single_channel_tpg_mode();
      "test_vc_exposure_swap":         test_vc_exposure_swap();
      "test_soft_reset":               test_soft_reset();
      "test_async_resetn":             test_async_resetn();
      "test_upstream_flush":           test_upstream_flush();
      "test_random_streaming":         test_random_streaming();
      "test_abort_mask_robustness":    test_abort_mask_robustness();
      "test_vc_and_skip_robustness":   test_vc_and_skip_robustness();
      "test_disable_without_abort":    test_disable_without_abort();
      "test_checker_fault":            test_checker_fault();
      default: report_error($sformatf("unknown test '%s'", test_name));
    endcase

    repeat (4) tick();
    $display("cycles=%0d, takes=%0d, transfers=%0d, checks=%0d, errors=%0d",
             cyc, total_takes, n_xfer, check_count, err_count);
    if ((err_count == 0) && (n_xfer > 0) && (check_count > 0)) begin
      $display("%s PASS", test_name);
    end else begin
      $display("%s FAIL (errors=%0d)", test_name, err_count);
    end
    $finish;
  end

endmodule
