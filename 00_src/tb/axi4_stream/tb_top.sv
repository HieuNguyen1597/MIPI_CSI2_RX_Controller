// File        : tb_top.sv
// Description : Testbench of m_fpt_axi4_stream (AXI4-Stream Master: 16-bit stream to the ISP in
//               the clock domain axis_aclk, intake from Format Bridge in sys_clk).
//               The testbench plays Format Bridge, Control FSM, the CSR block and the ISP. A
//               reference model in sys_clk, written from the specification, follows the TID map,
//               the frame gate, the skip bits, the open line and the Finalize Sequencer, and turns
//               every accepted beat and every abort entry into the expected list of 16-bit
//               transfers. While the stream FIFO may be full the model keeps every case of abort
//               entry writes that the rules allow and drops the cases that asm_abort_done or the
//               transfers seen at the ISP rule out. A monitor in axis_aclk compares every
//               handshake with the list and checks the AXI4-Stream rules; a monitor in sys_clk
//               checks fbs_ready and asm_abort_done. Tests only drive stimulus.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//               The seed selects the clocks: seeds 1, 4, 7 run axis_aclk 1.818 ns (550 MHz) with
//               sys_clk 4.464 ns; seeds 2, 5, 8 run both at 4.464 ns; seeds 3, 6, 9 run axis_aclk
//               10 ns with sys_clk 4.464 ns. The phase of axis_aclk is drawn from the seed.
//
// Design assumptions (decided by the block owner; the specification does not state them yet):
//   - sys_clk and axis_aclk are asynchronous. resetn resets the sys_clk side and axis_resetn the
//     axis_aclk side; both are asserted together and each is released in its own clock domain,
//     in either order. A beat accepted while the axis_aclk side is in reset is delivered later.
//   - A beat accepted on the sys_clk side is written into the stream FIFO as one entry. Abort
//     entries are written into the same FIFO behind the real beats while the Finalize Sequencer
//     is in INJECT, one per cycle while the FIFO is not full and no soft_reset flush is pending.
//   - asm_abort_done pulses in the sys_clk cycle that writes the last abort entry of a sequence,
//     or in the cycle of the request when there is nothing to close.
//   - fbs_ready is 1 in INJECT; otherwise it is 0 while the FIFO is full or a soft_reset flush is
//     pending. The full flag rises in the cycle after the write that fills the FIFO and clears no
//     later than SYNC_STAGES + 2 sys_clk edges after an entry is freed.
//   - soft_reset clears the sys_clk state like resetn, and the cycle after its first cycle has
//     fbs_ready = 0 unless an earlier flush may still be pending (it then serves this one).
//     Entries written before it may still reach the ISP during SYNC_STAGES + 3 axis_aclk
//     edges, as a prefix of the old list; then TVALID is low. A beat accepted after it is never
//     lost. A cycle with soft_reset ignores the abort and skip requests.
//   - The first transfer of an entry written into an empty block is presented no later than
//     SYNC_STAGES + 3 axis_aclk edges after the write.
//   - An entry is sent as 16-bit transfers k = 0..3 in ascending order, transfer k carrying
//     fbs_data[16k+15:16k] and its pair of byte enables; a pair 2'b00 is not sent. TUSER[0] is on
//     the first transfer, TLAST and TUSER[1] on the last, TID on every transfer. An abort entry is
//     one null transfer (TKEEP 2'b00, TLAST 1, TUSER 2'b10, TDATA 0).
//   - TVALID may be low for some cycles between the last real transfer of a line and its abort
//     transfer. With TREADY held high and continuous input a line has no TVALID gap.
//   - Stimulus: byte enables are contiguous from bit 0 and only the last beat of a line is
//     partial; fbs_err only with fbs_eol; the Long exposure starts each capture; the command
//     masks are random while no request is issued. Some tests use requests outside the normal
//     command rules (mask 2'b00, back-to-back aborts, skip masks 2'b10 and 2'b11, reserved
//     channels).
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_SYNC_STAGES     = 32'd2;
  parameter logic [31:0] PARA_AXIS_FIFO_DEPTH = 32'd16;

  localparam int SYNC      = int'(PARA_SYNC_STAGES);
  localparam int DEPTH     = int'(PARA_AXIS_FIFO_DEPTH);
  localparam int MAX_PRINT = 40;
  localparam int MAXG      = 256;      // abort sequences waiting in the expected list
  localparam int MAXB      = 32;       // open cases of one abort sequence
  localparam int MAXSEQ    = 64;       // abort entries of one sequence (or of joined ones)
  localparam int WAIT_MAX  = 400000;   // sys_clk cycles of one wait
  localparam int REL_MAX   = 3;        // entries released beyond the presented ones

  // ------------------------------------------------------------------------------------------
  // Clocks and resets
  // ------------------------------------------------------------------------------------------
  real  t_sys = 4.464;
  real  t_ax  = 1.818;
  real  t_ph  = 0.5;
  bit   clk_go = 1'b0;
  logic sys_clk     = 1'b0;
  logic axis_aclk   = 1'b0;
  logic resetn      = 1'b0;
  logic axis_resetn = 1'b0;

  initial begin
    wait (clk_go);
    forever #(t_sys / 2.0) sys_clk = ~sys_clk;
  end

  initial begin
    wait (clk_go);
    #(t_ph);
    forever #(t_ax / 2.0) axis_aclk = ~axis_aclk;
  end

  // ------------------------------------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------------------------------------
  logic        soft_reset      = 1'b0;
  logic [63:0] fbs_data;
  logic        fbs_valid       = 1'b0;
  logic [7:0]  fbs_byte_en;
  logic        fbs_sof;
  logic        fbs_eol;
  logic [1:0]  fbs_vc_tag;
  logic        fbs_err;
  logic        fbs_ready;
  logic        asm_abort_req   = 1'b0;
  logic [1:0]  asm_abort_mask  = 2'b00;
  logic        asm_abort_done;
  logic        asm_skip_req    = 1'b0;
  logic [1:0]  asm_skip_mask   = 2'b00;
  logic        axis_enable     = 1'b1;
  logic        vc_exposure_map = 1'b0;
  logic [15:0] m_axis_tdata;
  logic        m_axis_tvalid;
  logic        m_axis_tready   = 1'b1;
  logic        m_axis_tlast;
  logic [1:0]  m_axis_tuser;
  logic [1:0]  m_axis_tid;
  logic [1:0]  m_axis_tkeep;

  m_fpt_axi4_stream #(
    .PARA_SYNC_STAGES     (PARA_SYNC_STAGES),
    .PARA_AXIS_FIFO_DEPTH (PARA_AXIS_FIFO_DEPTH)
  ) u_dut (
    .i_sys_clk         (sys_clk),
    .i_axis_aclk       (axis_aclk),
    .i_resetn          (resetn),
    .i_axis_resetn     (axis_resetn),
    .i_soft_reset      (soft_reset),
    .i_fbs_data        (fbs_data),
    .i_fbs_valid       (fbs_valid),
    .i_fbs_byte_en     (fbs_byte_en),
    .i_fbs_sof         (fbs_sof),
    .i_fbs_eol         (fbs_eol),
    .i_fbs_vc_tag      (fbs_vc_tag),
    .i_fbs_err         (fbs_err),
    .o_fbs_ready       (fbs_ready),
    .i_asm_abort_req   (asm_abort_req),
    .i_asm_abort_mask  (asm_abort_mask),
    .o_asm_abort_done  (asm_abort_done),
    .i_asm_skip_req    (asm_skip_req),
    .i_asm_skip_mask   (asm_skip_mask),
    .i_axis_enable     (axis_enable),
    .i_vc_exposure_map (vc_exposure_map),
    .o_m_axis_tdata    (m_axis_tdata),
    .o_m_axis_tvalid   (m_axis_tvalid),
    .i_m_axis_tready   (m_axis_tready),
    .o_m_axis_tlast    (m_axis_tlast),
    .o_m_axis_tuser    (m_axis_tuser),
    .o_m_axis_tid      (m_axis_tid),
    .o_m_axis_tkeep    (m_axis_tkeep)
  );

  // ------------------------------------------------------------------------------------------
  // Reporting
  // ------------------------------------------------------------------------------------------
  string  test_name;
  int     seed;
  int     err_count = 0;
  longint checks    = 0;
  bit     chk_on    = 1'b0;
  bit     fault_on  = 1'b0;
  bit     dbg       = 1'b0;     // +DBG prints the abort sequences and soft_reset events

  task automatic report(input string msg);
    err_count++;
    if (err_count <= MAX_PRINT) $error("%s", msg);
    else if (err_count == MAX_PRINT + 1) $display("further errors are counted, not printed");
  endtask

  task automatic need(input bit ok, input string what);
    checks++;
    if (!ok) report({"stimulus did not reach: ", what});
  endtask

  // ------------------------------------------------------------------------------------------
  // Format Bridge source (sys_clk): FWFT, a beat stays until it is taken
  // ------------------------------------------------------------------------------------------
  typedef struct packed {
    logic [63:0] data;
    logic [7:0]  be;
    logic        sof;
    logic        eol;
    logic [1:0]  vc;
    logic        err;
  } beat_t;

  beat_t src_q[$];
  beat_t cur;
  bit    cur_valid  = 1'b0;
  bit    last_take  = 1'b0;
  bit    src_flush  = 1'b0;
  int    src_budget = -1;     // beats still allowed to be presented; -1 = no limit
  int    gap_pct    = 0;
  int    line_seq   = 0;
  int    abort_idle = -1;     // value of asm_abort_mask without a request; -1 = random
  int    skip_idle  = -1;     // value of asm_skip_mask without a request; -1 = random

  function automatic logic [1:0] idle_mask(input int v);
    if (v < 0) return 2'($urandom_range(0, 3));
    return 2'(v);
  endfunction

  task automatic drive_fields();
    fbs_valid = cur_valid;
    if (cur_valid) begin
      fbs_data = cur.data; fbs_byte_en = cur.be; fbs_sof = cur.sof; fbs_eol = cur.eol;
      fbs_vc_tag = cur.vc; fbs_err = cur.err;
    end else begin
      fbs_data = 'x; fbs_byte_en = 'x; fbs_sof = 1'bx; fbs_eol = 1'bx; fbs_vc_tag = 'x;
      fbs_err = 1'bx;
    end
  endtask

  // Inputs change 0.1 ns after the falling edge; command pulses last one cycle.
  always @(negedge sys_clk) begin : p_drive_sys
    #0.1;
    asm_abort_req  = 1'b0;
    asm_skip_req   = 1'b0;
    asm_abort_mask = idle_mask(abort_idle);
    asm_skip_mask  = idle_mask(skip_idle);
    if (resetn !== 1'b1) begin
      cur_valid = 1'b0; src_q.delete(); src_flush = 1'b0;
    end else begin
      if (last_take) cur_valid = 1'b0;
      if (src_flush) begin
        cur_valid = 1'b0; src_q.delete(); src_flush = 1'b0; src_budget = -1;
      end
      if (!cur_valid && (src_q.size() > 0) && (src_budget != 0) &&
          ((gap_pct == 0) || ($urandom_range(0, 99) >= gap_pct))) begin
        cur = src_q.pop_front();
        cur_valid = 1'b1;
        if (src_budget > 0) src_budget--;
      end
    end
    last_take = 1'b0;
    drive_fields();
  end

  // ------------------------------------------------------------------------------------------
  // ISP sink (axis_aclk): TREADY pattern, optional holds at the abort boundary
  // ------------------------------------------------------------------------------------------
  int rdy_pct       = 100;
  int hold_null     = 0;     // TREADY low for this many cycles when an abort transfer appears
  int hold_last     = 0;     // same, when the last real transfer before an abort appears
  int hold_cnt      = 0;
  int n_hold_null   = 0;
  int n_hold_last   = 0;
  bit ax_hs_edge    = 1'b0;  // handshake at the last rising edge
  bit ax_tv_before  = 1'b0;  // TVALID before the last rising edge

  typedef struct {
    bit          grp;     // abort sequence placeholder
    int          gid;
    logic [15:0] data;
    logic [1:0]  keep;
    logic        last;
    logic [1:0]  user;
    logic [1:0]  tid;
    bit          elast;   // last transfer of its entry
    int          epoch;
  } exp_t;

  exp_t q[$];

  always @(negedge axis_aclk) begin : p_drive_axis
    bit newp;
    #0.1;
    newp = (m_axis_tvalid === 1'b1) && (ax_hs_edge || !ax_tv_before);
    if (hold_cnt > 0) begin
      hold_cnt--;
      m_axis_tready = 1'b0;
    end else if (newp && (hold_null > 0) && (m_axis_tkeep == 2'b00)) begin
      hold_cnt = hold_null - 1; n_hold_null++;
      m_axis_tready = 1'b0;
    end else if (newp && (hold_last > 0) && (m_axis_tkeep != 2'b00) && (q.size() > 1) &&
                 !q[0].grp && q[1].grp) begin
      hold_cnt = hold_last - 1; n_hold_last++;
      m_axis_tready = 1'b0;
    end else begin
      m_axis_tready = ($urandom_range(0, 99) < rdy_pct);
    end
  end

  // ------------------------------------------------------------------------------------------
  // Reference model state (sys_clk)
  // ------------------------------------------------------------------------------------------
  localparam bit ST_PT  = 1'b0;   // PASSTHROUGH
  localparam bit ST_INJ = 1'b1;   // INJECT

  // One case of an abort sequence: open line, pending mask and the entries written so far.
  typedef struct packed {
    logic        lo;
    logic        lt;
    logic [1:0]  mask;
    logic [6:0]  len;
    logic [63:0] seq;
  } br_t;

  typedef struct {
    realtime t;
    longint  n;
    bit      adj;
  } cpl_t;

  bit         m_st;
  bit         m_lo;
  bit         m_lt;
  logic [1:0] m_pass;
  logic [1:0] m_skip;
  int         epoch = 0;

  // FIFO occupancy as seen by the write side, current epoch: bounds from the writes, the
  // presented entries (at most REL_MAX more are released: output, park and FIFO head
  // registers) and the completed entries (freed after SYNC_STAGES + 2 sys_clk edges).
  int         w_real, aw_min_c, aw_max_c, p_cur, fr_cur;
  bit         fl_clear;          // no soft_reset flush is pending
  int         ub_hold;           // entries of the last flush may still count as used
  cpl_t       cpl_q[$];
  longint     sys_n = 0;

  bit         sr_open;           // transfers of the old list may still appear
  realtime    sr_t;
  int         sr_cnt;
  bit         prev_sr, p010;

  bit         probe_on;
  realtime    probe_t;
  int         probe_cnt;

  br_t        g_br [MAXG][MAXB];
  int         g_nb [MAXG];
  bit         g_closed [MAXG];
  int         g_obs [MAXG];
  int         g_mn [MAXG];
  int         g_mx [MAXG];
  int         g_next, g_open, g_live;
  int         g_last_nb, g_last_len;

  // statistics used by the tests
  longint n_sys = 0, n_take = 0, n_acc = 0, n_take_sink = 0, n_take_ae0 = 0, n_full = 0;
  longint n_hs = 0, n_hs_real = 0, n_keep01 = 0, n_push = 0, n_take_sr = 0;
  int     n_null [2];
  int     n_groups = 0, n_done_now = 0, n_sof_inj = 0, n_sof_skip = 0, n_sof_blocked = 0;
  int     n_probe = 0, n_reserved = 0, n_sr = 0;
  bit     thr_on = 1'b0;

  task automatic model_reset();
    q.delete(); cpl_q.delete();
    m_st = ST_PT; m_lo = 1'b0; m_lt = 1'b0; m_pass = 2'b11; m_skip = 2'b00;
    w_real = 0; aw_min_c = 0; aw_max_c = 0; p_cur = 0; fr_cur = 0;
    fl_clear = 1'b1; ub_hold = 0;
    sr_open = 1'b0; prev_sr = 1'b0; p010 = 1'b0; probe_on = 1'b0;
    g_open = -1; g_live = 0;
    epoch++;
  endtask

  function automatic exp_t new_exp();
    exp_t e;
    e.grp = 1'b0; e.gid = 0; e.data = '0; e.keep = '0; e.last = 1'b0; e.user = '0;
    e.tid = '0; e.elast = 1'b0; e.epoch = epoch;
    return e;
  endfunction

  function automatic logic [1:0] tid_of(input logic [1:0] vc);
    return vc_exposure_map ? {vc[1], ~vc[0]} : vc;
  endfunction

  function automatic logic [1:0] vc_of(input bit t);
    return {1'b0, t ^ vc_exposure_map};
  endfunction

  task automatic drop_item();
    int gi;
    if (q[0].grp) begin
      gi = q[0].gid % MAXG;
      if (g_closed[gi] && (q[0].epoch == epoch) && (g_nb[gi] > 0) &&
          (int'(g_br[gi][0].len) == g_obs[gi])) begin
        aw_min_c += g_obs[gi] - g_mn[gi];
        aw_max_c -= g_mx[gi] - g_obs[gi];
      end
      g_live--;
    end
    void'(q.pop_front());
  endtask

  task automatic drop_old();
    while ((q.size() > 0) && (q[0].epoch != epoch)) drop_item();
  endtask

  function automatic bit group_done(input int gi);
    if (!g_closed[gi]) return 1'b0;
    for (int i = 0; i < g_nb[gi]; i++) if (int'(g_br[gi][i].len) != g_obs[gi]) return 1'b0;
    return 1'b1;
  endfunction

  task automatic try_pop_groups();
    while ((q.size() > 0) && q[0].grp && group_done(q[0].gid % MAXG)) drop_item();
  endtask

  function automatic bit q_idle();
    foreach (q[i]) begin
      if (!q[i].grp || !g_closed[q[i].gid % MAXG]) return 1'b0;
    end
    return 1'b1;
  endfunction

  // Keep the cases whose next written entry carries TID t.
  function automatic bit group_accept(input int gi, input bit t);
    br_t kept [$];
    for (int i = 0; i < g_nb[gi]; i++) begin
      if ((int'(g_br[gi][i].len) > g_obs[gi]) && (g_br[gi][i].seq[g_obs[gi]] == t))
        kept.push_back(g_br[gi][i]);
    end
    if (kept.size() == 0) return 1'b0;
    g_nb[gi] = kept.size();
    foreach (kept[i]) g_br[gi][i] = kept[i];
    g_obs[gi]++;
    return 1'b1;
  endfunction

  // Keep the cases in which every entry of the sequence has been seen.
  function automatic bit group_complete(input int gi);
    br_t kept [$];
    for (int i = 0; i < g_nb[gi]; i++)
      if (int'(g_br[gi][i].len) == g_obs[gi]) kept.push_back(g_br[gi][i]);
    if (kept.size() == 0) return 1'b0;
    g_nb[gi] = kept.size();
    foreach (kept[i]) g_br[gi][i] = kept[i];
    return 1'b1;
  endfunction

  task automatic add_br(ref br_t l[$], input br_t b);
    if (!b.lo) b.lt = 1'b0;
    foreach (l[i]) if (l[i] == b) return;
    l.push_back(b);
  endtask

  // A sequence that starts right behind one whose length is still open (no real transfer in
  // between) continues it: its cases carry on from every case of the earlier one.
  task automatic open_group(input bit lo, input bit lt, input logic [1:0] mask);
    exp_t e;
    int   gi;
    if ((q.size() > 0) && q[q.size() - 1].grp && (q[q.size() - 1].epoch == epoch)) begin
      gi = q[q.size() - 1].gid % MAXG;
      if (g_closed[gi] && (g_nb[gi] > 1)) begin
        aw_min_c -= g_mn[gi]; aw_max_c -= g_mx[gi];
        for (int i = 0; i < g_nb[gi]; i++) begin
          g_br[gi][i].lo = lo; g_br[gi][i].lt = lo ? lt : 1'b0; g_br[gi][i].mask = mask;
        end
        g_closed[gi] = 1'b0;
        g_open = q[q.size() - 1].gid;
        if (dbg) $display("%t sequence %0d continues with line %0d tid %0d mask %b", $realtime,
                          g_open, lo, lt, mask);
        return;
      end
    end
    if (g_live >= MAXG - 1) report("too many abort sequences waiting in the expected list");
    gi = g_next % MAXG;
    g_br[gi][0] = '0;
    g_br[gi][0].lo = lo; g_br[gi][0].lt = lo ? lt : 1'b0; g_br[gi][0].mask = mask;
    g_nb[gi] = 1; g_closed[gi] = 1'b0; g_obs[gi] = 0;
    e = new_exp(); e.grp = 1'b1; e.gid = g_next;
    q.push_back(e);
    g_open = g_next; g_next++; g_live++;
    if (dbg) $display("%t open sequence %0d: line %0d tid %0d mask %b, fl_clear %0d", $realtime,
                      g_open, lo, lt, mask, fl_clear);
  endtask

  task automatic close_group();
    int gi, mn, mx;
    gi = g_open % MAXG;
    mn = MAXSEQ + 1; mx = 0;
    for (int i = 0; i < g_nb[gi]; i++) begin
      if (int'(g_br[gi][i].len) < mn) mn = int'(g_br[gi][i].len);
      if (int'(g_br[gi][i].len) > mx) mx = int'(g_br[gi][i].len);
    end
    aw_min_c += mn; aw_max_c += mx;
    g_mn[gi] = mn; g_mx[gi] = mx;
    g_closed[gi] = 1'b1;
    g_last_nb = g_nb[gi]; g_last_len = (mn == mx) ? mn : -1;
    n_groups++;
    if (dbg) $display("%t close sequence %0d: %0d cases, length %0d..%0d", $realtime, g_open,
                      g_nb[gi], mn, mx);
    m_st = ST_PT; m_lo = 1'b0; m_lt = 1'b0; g_open = -1;
    try_pop_groups();
  endtask

  // One INJECT cycle: every case writes the next abort entry or, while the FIFO may be full or
  // a flush may be pending, waits; the observed asm_abort_done keeps the consistent cases.
  task automatic inject_step(input bit done_obs, input bit areq, input logic [1:0] amask);
    br_t        nb [$];
    br_t        b, w;
    int         gi, lbb, ubb;
    bit         can_w, must_w, dw, inj;
    logic [1:0] mi, nm;
    gi = g_open % MAXG;
    for (int i = 0; i < g_nb[gi]; i++) begin
      b      = g_br[gi][i];
      mi     = b.mask | (areq ? amask : 2'b00);
      lbb    = w_real + aw_min_c + int'(b.len) - p_cur - REL_MAX;
      ubb    = w_real + aw_max_c + int'(b.len) - fr_cur;
      can_w  = (lbb < DEPTH) && (int'(b.len) < MAXSEQ);
      must_w = fl_clear && (ub_hold == 0) && (ubb < DEPTH);
      if (can_w) begin
        inj = b.lo ? b.lt : !b.mask[0];
        nm  = mi & ~(2'b01 << inj);
        dw  = (nm == 2'b00);
        if (dw == done_obs) begin
          w = b; w.lo = 1'b0; w.mask = nm; w.seq[b.len] = inj; w.len = b.len + 7'd1;
          add_br(nb, w);
        end
      end
      if (!must_w && !done_obs) begin
        w = b; w.mask = mi;
        add_br(nb, w);
      end
    end
    if (nb.size() == 0) begin
      report($sformatf("asm_abort_done = %0d does not fit any abort entry writes allowed",
                       done_obs));
      if (done_obs) close_group();
      return;
    end
    if (nb.size() > MAXB) begin
      report("too many open cases of one abort sequence");
      while (nb.size() > MAXB) void'(nb.pop_back());
    end
    g_nb[gi] = nb.size();
    foreach (nb[i]) g_br[gi][i] = nb[i];
    if (dbg && (areq || (nb.size() > 1)))
      $display("%t sequence %0d: request %0d mask %b, %0d cases", $realtime, g_open, areq,
               amask, nb.size());
    if (done_obs) close_group();
  endtask

  // Accepted beat: one entry, its non-empty byte pairs as transfers in ascending order.
  task automatic push_entry(input beat_t b, input bit t);
    exp_t e;
    int   first, last;
    bit   empty;
    empty = (q.size() == 0) && (m_axis_tvalid !== 1'b1) && (axis_resetn === 1'b1) && fl_clear &&
            !sr_open;
    first = -1; last = -1;
    for (int k = 0; k < 4; k++) begin
      if (b.be[2*k +: 2] != 2'b00) begin
        if (first < 0) first = k;
        last = k;
      end
    end
    for (int k = 0; k < 4; k++) begin
      if (b.be[2*k +: 2] != 2'b00) begin
        e = new_exp();
        e.data  = b.data[16*k +: 16];
        e.keep  = b.be[2*k +: 2];
        e.last  = b.eol && (k == last);
        e.user  = {b.err && (k == last), b.sof && (k == first)};
        e.tid   = {1'b0, t};
        e.elast = (k == last);
        n_push++;
        if (fault_on && (n_push == 5)) e.data[0] = ~e.data[0];
        q.push_back(e);
      end
    end
    w_real++;
    if (empty && !probe_on) begin
      probe_on = 1'b1; probe_t = $realtime; probe_cnt = 0;
    end
  endtask

  task automatic soft_reset_model();
    if (dbg) $display("%t soft_reset: open sequence %0d, %0d items", $realtime, g_open, q.size());
    if (g_open >= 0) begin
      g_closed[g_open % MAXG] = 1'b1;
      g_mn[g_open % MAXG] = 0; g_mx[g_open % MAXG] = 0;
      g_open = -1;
    end
    m_st = ST_PT; m_lo = 1'b0; m_lt = 1'b0; m_pass = 2'b11; m_skip = 2'b00;
    if ((q.size() > 0) && (q[q.size() - 1].epoch == epoch)) begin
      epoch++;
      sr_open = 1'b1; sr_t = $realtime; sr_cnt = 0;
    end
    w_real = 0; aw_min_c = 0; aw_max_c = 0; p_cur = 0; fr_cur = 0;
    cpl_q.delete();
    fl_clear = 1'b0; probe_on = 1'b0;
  endtask

  // ------------------------------------------------------------------------------------------
  // sys_clk monitor: model step with the values before each rising edge
  // ------------------------------------------------------------------------------------------
  task automatic sys_step();
    bit         v, rdy, take, sr, sink, done, sof0, sof1, pn0, pn1, gate, inval, acc, astart;
    bit         todo, lo_n, lt_n, done_exp, ae, fl0;
    logic [1:0] tid, mask_n;
    int         lb, ub;
    sys_n++;
    for (int i = cpl_q.size() - 1; i >= 0; i--) begin
      if (cpl_q[i].t != $realtime) break;
      cpl_q[i].adj = 1'b1;
    end
    while ((cpl_q.size() > 0) &&
           ((sys_n - cpl_q[0].n - longint'(cpl_q[0].adj)) >= longint'(SYNC + 3))) begin
      void'(cpl_q.pop_front());
      fr_cur++;
    end
    if (ub_hold > 0) ub_hold--;
    if (!chk_on) return;
    n_sys++;
    if (resetn !== 1'b1) begin
      checks++;
      if ((fbs_ready !== 1'b1) || (asm_abort_done !== 1'b0))
        report("fbs_ready must be 1 and asm_abort_done 0 while resetn is low");
      last_take = 1'b0; prev_sr = 1'b0; p010 = 1'b0;
      return;
    end
    if ($isunknown({fbs_ready, asm_abort_done}))
      report("fbs_ready or asm_abort_done is unknown");
    v    = (fbs_valid === 1'b1);
    rdy  = (fbs_ready === 1'b1);
    done = (asm_abort_done === 1'b1);
    take = v && rdy;
    last_take = take;
    sr   = (soft_reset === 1'b1);
    ae   = (axis_enable === 1'b1);
    sink = (m_st == ST_INJ);
    tid  = 2'b00;
    if (v) tid = tid_of(fbs_vc_tag);
    sof0 = v && (fbs_sof === 1'b1) && (tid == 2'd0);
    sof1 = v && (fbs_sof === 1'b1) && (tid == 2'd1);
    pn0  = sof0 ? (!sink && !m_skip[0]) : m_pass[0];
    pn1  = sof1 ? (!sink && !m_skip[1] && m_pass[0]) : m_pass[1];
    gate = v && !tid[1] && (tid[0] ? pn1 : pn0);
    inval = v && ae && gate;
    acc    = take && !sink && inval && !sr;
    astart = !sink && (asm_abort_req === 1'b1) && !sr;
    if (take) begin
      n_take++;
      if (sink) n_take_sink++;
      if (!ae) n_take_ae0++;
      if (sr) n_take_sr++;
      if (tid[1]) n_reserved++;
      if (sink && (sof0 || sof1)) n_sof_inj++;
      if (!sink && ((sof0 && m_skip[0]) || (sof1 && m_skip[1]))) n_sof_skip++;
      if (!sink && sof1 && !m_skip[1] && !m_pass[0]) n_sof_blocked++;
    end
    if (v && !rdy && !sink) n_full++;

    // fbs_ready
    checks++;
    if (p010 && rdy) report("fbs_ready is high in the cycle after the first soft_reset cycle");
    if (sink) begin
      if (!rdy) report("fbs_ready is low during an abort sequence");
    end else begin
      lb = w_real + aw_min_c - p_cur - REL_MAX;
      ub = w_real + aw_max_c - fr_cur;
      if ((lb >= DEPTH) && rdy)
        report($sformatf("fbs_ready is high with at least %0d entries in the stream FIFO", lb));
      if (fl_clear && (ub_hold == 0) && (ub < DEPTH) && !rdy)
        report($sformatf("fbs_ready is low with at most %0d entries in the stream FIFO", ub));
    end

    // asm_abort_done in PASSTHROUGH
    lo_n = m_lo; lt_n = m_lt; mask_n = 2'b00; todo = 1'b0;
    if (!sink) begin
      if (acc) begin
        lo_n = !(fbs_eol === 1'b1);
        lt_n = tid[0];
      end
      if (astart) mask_n = asm_abort_mask;
      todo     = lo_n || (mask_n != 2'b00);
      done_exp = astart && !todo;
      if (!sr) begin
        checks++;
        if (done != done_exp)
          report($sformatf("asm_abort_done = %0d, expected %0d (no abort sequence open)",
                           done, done_exp));
      end
    end

    // state update at the edge
    fl0 = fl_clear;
    if (sr) begin
      if (!prev_sr) n_sr++;
      soft_reset_model();
    end else begin
      if (astart || !ae) m_pass = 2'b00;
      else if (take) m_pass = {pn1, pn0};
      for (int k = 0; k < 2; k++) begin
        if ((asm_skip_req === 1'b1) && (asm_skip_mask[k] === 1'b1)) m_skip[k] = 1'b1;
        else if (take && ((k == 0) ? sof0 : sof1)) m_skip[k] = 1'b0;
      end
      if (!sink) begin
        if (acc) begin
          n_acc++;
          push_entry('{fbs_data, fbs_byte_en, fbs_sof, fbs_eol, fbs_vc_tag, fbs_err}, tid[0]);
          m_lo = lo_n; m_lt = lt_n;
        end
        if (astart && !todo) n_done_now++;
        if (astart && todo) begin
          open_group(lo_n, lt_n, mask_n);
          m_st = ST_INJ;
        end
      end else begin
        inject_step(done, (asm_abort_req === 1'b1), asm_abort_mask);
      end
      if (!sink && rdy && !fl_clear) begin
        fl_clear = 1'b1;
        ub_hold  = SYNC + 3;
      end
    end
    // a soft_reset that finds a flush possibly pending may be served by that flush
    p010    = sr && !prev_sr && fl0;
    prev_sr = sr;
  endtask

  always @(posedge sys_clk) sys_step();

  always @(negedge resetn) begin : p_resetn
    model_reset();
    if (chk_on) begin
      #0.1;
      checks++;
      if ((fbs_ready !== 1'b1) || (asm_abort_done !== 1'b0))
        report("resetn did not clear fbs_ready / asm_abort_done at once");
    end
  end

  // ------------------------------------------------------------------------------------------
  // axis_aclk monitor: AXI4-Stream rules and transfer scoreboard
  // ------------------------------------------------------------------------------------------
  bit          ax_rst_seen  = 1'b1;
  bit          ax_prev_tv   = 1'b0;
  bit          ax_prev_rd   = 1'b0;
  bit          ax_prev_hs   = 1'b0;
  logic [22:0] ax_prev_pl;
  bit          ax_in_line   = 1'b0;
  logic [1:0]  ax_line_tid;

  task automatic ax_reset_trackers();
    ax_prev_tv = 1'b0; ax_prev_rd = 1'b0; ax_prev_hs = 1'b0; ax_in_line = 1'b0;
  endtask

  always @(negedge axis_resetn) begin : p_axis_resetn
    ax_reset_trackers();
    ax_rst_seen = 1'b1;
    if (chk_on) begin
      #0.1;
      checks++;
      if (m_axis_tvalid !== 1'b0) report("axis_resetn did not clear TVALID at once");
    end
  end

  task automatic scoreboard();
    exp_t        h;
    bit          nul, ok;
    int          gi;
    logic [22:0] got;
    nul = (m_axis_tkeep == 2'b00);
    got = {m_axis_tdata, m_axis_tkeep, m_axis_tlast, m_axis_tuser, m_axis_tid};
    forever begin
      try_pop_groups();
      if (q.size() == 0) begin
        report($sformatf("unexpected transfer data %h keep %b last %b user %b tid %0d",
                         m_axis_tdata, m_axis_tkeep, m_axis_tlast, m_axis_tuser, m_axis_tid));
        return;
      end
      h = q[0];
      if ((h.epoch != epoch) && !sr_open) begin
        drop_item();
        continue;
      end
      if (h.grp) begin
        gi = h.gid % MAXG;
        ok = nul && (m_axis_tid[1] == 1'b0) && (m_axis_tdata == 16'h0) && m_axis_tlast &&
             (m_axis_tuser == 2'b10);
        if (dbg) begin
          $display("%t abort transfer tid %0d at sequence %0d (seen %0d, closed %0d)", $realtime,
                   m_axis_tid, h.gid, g_obs[gi], g_closed[gi]);
          for (int i = 0; i < g_nb[gi]; i++)
            $display("    case %0d: len %0d seq %b", i, g_br[gi][i].len, g_br[gi][i].seq[15:0]);
        end
        if (ok && group_accept(gi, m_axis_tid[0])) begin
          if (h.epoch == epoch) cpl_q.push_back('{$realtime, sys_n, 1'b0});
          try_pop_groups();
          return;
        end
        if (!nul && g_closed[gi] && group_complete(gi)) begin
          drop_item();
          continue;
        end
        if (h.epoch != epoch) begin
          drop_old();
          continue;
        end
        report($sformatf("transfer keep %b tid %0d does not fit the abort sequence in progress",
                         m_axis_tkeep, m_axis_tid));
        if (nul) return;
        drop_item();
        continue;
      end
      if (got === {h.data, h.keep, h.last, h.user, h.tid}) begin
        if (h.elast && (h.epoch == epoch)) cpl_q.push_back('{$realtime, sys_n, 1'b0});
        drop_item();
        return;
      end
      if (h.epoch != epoch) begin
        drop_old();
        continue;
      end
      report($sformatf("transfer data %h keep %b last %b user %b tid %0d, expected %h %b %b %b %0d",
                       m_axis_tdata, m_axis_tkeep, m_axis_tlast, m_axis_tuser, m_axis_tid,
                       h.data, h.keep, h.last, h.user, h.tid));
      drop_item();
      return;
    end
  endtask

  task automatic axis_step();
    bit          tv, rd, hs, newp;
    logic [22:0] pl;
    int          hi;
    ax_hs_edge   = 1'b0;
    ax_tv_before = (m_axis_tvalid === 1'b1);
    if (!chk_on) return;
    if (axis_resetn !== 1'b1) begin
      checks++;
      if (m_axis_tvalid !== 1'b0) report("TVALID is high while axis_resetn is low");
      ax_rst_seen = 1'b1;
      ax_reset_trackers();
      return;
    end
    if (ax_rst_seen) begin
      checks++;
      if (m_axis_tvalid !== 1'b0) report("TVALID is high at the first edge after axis_resetn");
      ax_rst_seen = 1'b0;
    end
    if (resetn !== 1'b1) begin
      checks++;
      if (m_axis_tvalid !== 1'b0) report("a transfer is presented while resetn is low");
    end
    if (sr_open && ($realtime > sr_t)) begin
      sr_cnt++;
      if (sr_cnt >= SYNC + 4) begin
        drop_old();
        sr_open = 1'b0;
        checks++;
        if ((m_axis_tvalid === 1'b1) && (q.size() == 0))
          report("TVALID is still high SYNC_STAGES + 3 axis_aclk edges after soft_reset");
        ax_reset_trackers();
      end
    end
    if ($isunknown(m_axis_tvalid)) report("TVALID is unknown");
    tv = (m_axis_tvalid === 1'b1);
    rd = (m_axis_tready === 1'b1);
    pl = {m_axis_tdata, m_axis_tkeep, m_axis_tlast, m_axis_tuser, m_axis_tid};
    if (tv && $isunknown(pl)) report("TDATA, TKEEP, TLAST, TUSER or TID is unknown with TVALID");

    // handshake rule: no change before TREADY
    if (ax_prev_tv && !ax_prev_rd && !sr_open) begin
      checks++;
      if (!tv || (pl !== ax_prev_pl)) report("TVALID or the payload changed before TREADY");
    end

    // transfer form
    if (tv) begin
      checks++;
      if (m_axis_tkeep == 2'b10) report("TKEEP = 2'b10");
      if (!m_axis_tlast && (m_axis_tkeep != 2'b11)) report("TKEEP not 2'b11 without TLAST");
      if ((m_axis_tkeep == 2'b00) &&
          (!m_axis_tlast || (m_axis_tuser != 2'b10) || (m_axis_tdata != 16'h0)))
        report("null transfer without TLAST = 1, TUSER = 2'b10, TDATA = 0");
      if (m_axis_tuser[1] && !m_axis_tlast) report("TUSER[1] without TLAST");
      if (m_axis_tuser[0] && (m_axis_tkeep == 2'b00)) report("TUSER[0] on a null transfer");
      if (m_axis_tid[1]) report("TID 2 or 3");
    end

    // latency into an empty block
    if (probe_on && ($realtime > probe_t)) begin
      probe_cnt++;
      if (tv) begin
        probe_on = 1'b0; n_probe++; checks++;
      end else if (probe_cnt >= SYNC + 4) begin
        report("first transfer later than SYNC_STAGES + 3 axis_aclk edges after the write");
        probe_on = 1'b0;
      end
    end

    if (thr_on && ax_in_line && !tv) report("TVALID gap inside a line with TREADY held high");

    // entries released by the read side: at most the presented ones and one more
    try_pop_groups();
    newp = tv && (!ax_prev_tv || ax_prev_hs);
    if (newp && (q.size() > 0)) begin
      hi = 0;
      if (q[0].grp && (m_axis_tkeep != 2'b00) && (q.size() > 1)) hi = 1;
      if (q[hi].epoch == epoch) begin
        if (q[hi].grp || q[hi].elast) p_cur++;
      end
    end

    hs = tv && rd;
    if (hs) begin
      n_hs++;
      checks++;
      if (m_axis_tkeep == 2'b00) n_null[m_axis_tid[0]]++;
      else n_hs_real++;
      if ((m_axis_tkeep == 2'b01) && m_axis_tlast) n_keep01++;
      if (ax_in_line && (m_axis_tid !== ax_line_tid) && !sr_open)
        report("TID changed inside a packet (no TLAST before it)");
      scoreboard();
      ax_in_line  = !m_axis_tlast;
      ax_line_tid = m_axis_tid;
    end
    ax_prev_tv = tv; ax_prev_rd = rd; ax_prev_pl = pl; ax_prev_hs = hs;
    ax_hs_edge = hs;
  endtask

  always @(posedge axis_aclk) axis_step();

  // ------------------------------------------------------------------------------------------
  // Stimulus helpers
  // ------------------------------------------------------------------------------------------
  task automatic tick();
    @(negedge sys_clk);
    #0.2;
  endtask

  task automatic ticks(input int n);
    repeat (n) tick();
  endtask

  task automatic atick();
    @(negedge axis_aclk);
    #0.2;
  endtask

  function automatic int urange(input int lo, input int hi);
    return int'($urandom_range(lo, hi));
  endfunction

  // A line of nbytes bytes; lane k of beat n carries {line number, pixel 4n + k}.
  task automatic gen_line(input logic [1:0] vc, input int nbytes, input bit sof, input bit err);
    beat_t b;
    int    nb, r;
    nb = (nbytes + 7) / 8;
    for (int i = 0; i < nb; i++) begin
      r = nbytes - 8 * i;
      b.be = (r >= 8) ? 8'hFF : 8'((1 << r) - 1);
      for (int k = 0; k < 4; k++) b.data[16*k +: 16] = {6'(line_seq), 10'(4 * i + k)};
      for (int j = 0; j < 8; j++) if (!b.be[j]) b.data[8*j +: 8] = 8'($urandom);
      b.sof = sof && (i == 0);
      b.eol = (i == nb - 1);
      b.vc  = vc;
      b.err = err && (i == nb - 1);
      src_q.push_back(b);
    end
    line_seq++;
  endtask

  // Long frame first, then Long and Short lines interleaved in random order.
  task automatic gen_capture(input int nl, input int ns, input int lo, input int hi,
                             input int err_pct);
    int il, is;
    gen_line(vc_of(1'b0), urange(lo, hi), 1'b1, urange(0, 99) < err_pct);
    il = 1; is = 0;
    while ((il < nl) || (is < ns)) begin
      if ((is < ns) && ((il >= nl) || (urange(0, 1) == 1))) begin
        gen_line(vc_of(1'b1), urange(lo, hi), is == 0, urange(0, 99) < err_pct);
        is++;
      end else begin
        gen_line(vc_of(1'b0), urange(lo, hi), 1'b0, urange(0, 99) < err_pct);
        il++;
      end
    end
  endtask

  task automatic start_capture(input bit with_short);
    gen_line(vc_of(1'b0), 16, 1'b1, 1'b0);
    if (with_short) gen_line(vc_of(1'b1), 16, 1'b1, 1'b0);
  endtask

  task automatic abort_now(input logic [1:0] mask, input bit flush);
    asm_abort_req = 1'b1; asm_abort_mask = mask;
    if (flush) src_flush = 1'b1;
  endtask

  task automatic skip_now(input logic [1:0] mask);
    asm_skip_req = 1'b1; asm_skip_mask = mask;
  endtask

  task automatic sr_pulse(input int n, input bit flush);
    soft_reset = 1'b1;
    if (flush) src_flush = 1'b1;
    ticks(n);
    soft_reset = 1'b0;
  endtask

  function automatic bit tb_idle();
    return (src_q.size() == 0) && !cur_valid && q_idle() && (m_st == ST_PT) && fl_clear &&
           (ub_hold == 0) && (cpl_q.size() == 0) && (m_axis_tvalid !== 1'b1) && !sr_open &&
           !probe_on;
  endfunction

  task automatic wait_idle(input string what);
    int n;
    n = 0;
    while (!tb_idle() && (n < WAIT_MAX)) begin
      tick(); n++;
    end
    need(tb_idle(), {"idle after ", what});
  endtask

  function automatic bit tb_drained();
    return !cur_valid && q_idle() && (m_st == ST_PT) && fl_clear && (ub_hold == 0) &&
           (cpl_q.size() == 0) && (m_axis_tvalid !== 1'b1) && !sr_open && !probe_on;
  endfunction

  task automatic wait_drained(input string what);
    int n;
    n = 0;
    while (!tb_drained() && (n < WAIT_MAX)) begin
      tick(); n++;
    end
    need(tb_drained(), {"empty block in ", what});
  endtask

  task automatic wait_tvalid(input bit with_last, input string what);
    int n;
    n = 0;
    while (!((m_axis_tvalid === 1'b1) && (!with_last || (m_axis_tlast === 1'b0))) &&
           (n < WAIT_MAX)) begin
      tick(); n++;
    end
    need(m_axis_tvalid === 1'b1, {"transfer presented in ", what});
  endtask

  task automatic wait_src_empty(input string what);
    int n;
    n = 0;
    while (((src_q.size() > 0) || cur_valid) && (n < WAIT_MAX)) begin
      tick(); n++;
    end
    need((src_q.size() == 0) && !cur_valid, {"source empty in ", what});
  endtask

  task automatic wait_budget(input string what);
    int n;
    n = 0;
    while (((src_budget != 0) || cur_valid) && (n < WAIT_MAX)) begin
      tick(); n++;
    end
    need((src_budget == 0) && !cur_valid, {"beats taken in ", what});
  endtask

  task automatic wait_seq_end(input string what);
    int n;
    n = 0;
    while ((m_st != ST_PT) && (n < WAIT_MAX)) begin
      tick(); n++;
    end
    need(m_st == ST_PT, {"end of the abort sequence in ", what});
  endtask

  // fbs_valid = 1 with fbs_ready = 0 for ncyc consecutive cycles (FIFO full).
  task automatic wait_full(input int ncyc, input string what);
    int n, run;
    n = 0; run = 0;
    while ((run < ncyc) && (n < WAIT_MAX)) begin
      tick(); n++;
      if ((fbs_valid === 1'b1) && (fbs_ready === 1'b0) && (m_st == ST_PT)) run++;
      else run = 0;
    end
    need(run >= ncyc, {"stream FIFO full in ", what});
  endtask

  // Presented beat that will be taken in this cycle.
  localparam int P_ANY      = 0;
  localparam int P_MID      = 1;   // not the first, not the last beat of a line
  localparam int P_SOF_LONG = 2;   // Frame Start of the Long exposure
  localparam int P_EOL      = 3;   // last beat of a line
  localparam int P_SOF_SHRT = 4;   // Frame Start of the Short exposure

  function automatic bit pres_is(input int kind);
    if (!cur_valid || (fbs_ready !== 1'b1)) return 1'b0;
    case (kind)
      P_MID:      return !cur.sof && !cur.eol;
      P_SOF_LONG: return cur.sof && (tid_of(cur.vc) == 2'd0);
      P_EOL:      return cur.eol;
      P_SOF_SHRT: return cur.sof && (tid_of(cur.vc) == 2'd1);
      default:    return 1'b1;
    endcase
  endfunction

  task automatic wait_present(input int kind, input string what);
    int n;
    n = 0;
    tick();
    while (!pres_is(kind) && (n < WAIT_MAX)) begin
      tick(); n++;
    end
    need(pres_is(kind), {"presented beat in ", what});
  endtask

  // Both resets assert together between clock edges; each is released in its own domain.
  task automatic rst_assert();
    #0.37;
    resetn = 1'b0; axis_resetn = 1'b0;
    soft_reset = 1'b0;
    cur_valid = 1'b0; src_q.delete(); src_budget = -1; src_flush = 1'b0;
    drive_fields();
  endtask

  task automatic rst_release_sys();
    @(negedge sys_clk);
    #0.05;
    resetn = 1'b1;
  endtask

  task automatic rst_release_axis();
    @(negedge axis_aclk);
    #0.05;
    axis_resetn = 1'b1;
  endtask

  task automatic do_reset(input int hold, input bit axis_first, input int dly);
    rst_assert();
    repeat (hold) @(posedge sys_clk);
    if (axis_first) begin
      rst_release_axis();
      repeat (dly) @(negedge sys_clk);
      rst_release_sys();
    end else begin
      rst_release_sys();
      repeat (dly) @(negedge axis_aclk);
      rst_release_axis();
    end
    tick();
  endtask

  // Fill the stream FIFO with TREADY low and leave a Long line open at the intake side.
  task automatic fill_open_line(input string what);
    rdy_pct = 0;
    start_capture(1'b0);
    gen_line(vc_of(1'b0), 8 * (DEPTH + 64), 1'b0, 1'b0);
    wait_full(16, what);
  endtask

  // ------------------------------------------------------------------------------------------
  // Tests
  // ------------------------------------------------------------------------------------------

  // Reset of both domains: outputs during reset, first beats after each release order.
  task automatic test_reset_both_domains();
    rdy_pct = 100;
    gen_line(vc_of(1'b0), 5, 1'b1, 1'b0);
    wait_idle("first beat after reset");
    need(n_probe >= 1, "latency of the first beat");
    gen_capture(2, 2, 8, 40, 0);
    ticks(6);
    do_reset(4, 1'b1, 9);
    gen_line(vc_of(1'b0), 13, 1'b1, 1'b0);
    wait_idle("axis_aclk side released first");
    gen_capture(2, 1, 8, 40, 0);
    ticks(5);
    rst_assert();
    repeat (3) @(posedge sys_clk);
    rst_release_sys();
    tick();
    gen_capture(1, 1, 8, 24, 0);
    ticks(10);
    rst_release_axis();
    wait_idle("sys_clk side released first");
    need(n_hs_real > 0, "transfers after reset");
  endtask

  // Every line length from 1 to 20 beats with every last-beat byte count, both exposures.
  task automatic test_line_lengths();
    int nline, total;
    longint base;
    rdy_pct = 100; gap_pct = 0;
    nline = 0; total = 0;
    base = n_hs_real;
    for (int nb = 1; nb <= 20; nb++) begin
      for (int r = 1; r <= 8; r++) begin
        gen_line(vc_of(nline[0]), 8 * (nb - 1) + r, (nline % 40) < 2, (nline % 5) == 4);
        total += (8 * (nb - 1) + r + 1) / 2;
        nline++;
      end
      wait_src_empty("line lengths");
    end
    wait_idle("line lengths");
    need((n_hs_real - base) == total, "every transfer of the line length table");
  endtask

  // Random TREADY, HDR captures in random line order, gapped and continuous input.
  task automatic test_random_streaming();
    int pcts [3] = '{25, 50, 90};
    for (int c = 0; c < 8; c++) begin
      rdy_pct = pcts[urange(0, 2)];
      gap_pct = (urange(0, 2) == 0) ? 30 : 0;
      gen_capture(urange(1, 4), urange(0, 4), 1, 120, 20);
      for (int i = 0; i < 20; i++) begin
        ticks(urange(10, 60));
        rdy_pct = pcts[urange(0, 2)];
      end
    end
    rdy_pct = 90;
    wait_idle("random streaming");
    gap_pct = 0;
  endtask

  // Long TREADY stall: the FIFO fills, fbs_ready falls, nothing is lost; exact capacity with
  // one-transfer entries.
  task automatic test_fifo_full_stall();
    longint t0;
    rdy_pct = 0;
    start_capture(1'b1);
    for (int i = 0; i < (DEPTH + 40) / 8 + 1; i++) gen_line(vc_of(i % 2), 64, 1'b0, 1'b0);
    wait_full(17, "long stall");
    repeat (4 * DEPTH * 4) atick();
    need(fbs_ready === 1'b0, "fbs_ready low through the long stall");
    rdy_pct = 100;
    wait_idle("long stall");
    rdy_pct = 0;
    for (int i = 0; i < DEPTH + 12; i++) gen_line(vc_of(1'b0), urange(1, 2), 1'b0, 1'b0);
    wait_full(17, "one-transfer entries");
    ticks(40);
    rdy_pct = 50;
    wait_idle("one-transfer entries");
    rdy_pct = 0;
    gen_capture(3 + DEPTH / 16, 3 + DEPTH / 16, 64, 200, 10);
    wait_full(10, "stall with HDR lines");
    t0 = n_hs;
    rdy_pct = 25;
    wait_idle("release at 25 %");
    need(n_hs > t0, "transfers after the stall");
  endtask

  // TREADY held high, continuous input: no TVALID gap inside a line.
  task automatic test_throughput();
    rdy_pct = 100; gap_pct = 0;
    start_capture(1'b1);
    for (int i = 0; i < 24; i++) gen_line(vc_of(1'b0), urange(9, 250), 1'b0, i % 7 == 3);
    gen_capture(6, 6, 16, 200, 10);
    thr_on = (DEPTH >= 16);
    wait_src_empty("throughput");
    wait_idle("throughput");
    thr_on = 1'b0;
    need(n_hs_real > 1000, "transfers of the throughput lines");
  endtask

  // Single beats into an empty block: first transfer within SYNC_STAGES + 3 axis_aclk edges.
  task automatic test_latency();
    int p0;
    p0 = n_probe;
    rdy_pct = 100;
    start_capture(1'b1);
    wait_idle("latency start");
    for (int i = 0; i < 24; i++) begin
      ticks(urange(0, 9));
      rdy_pct = (i % 2 == 0) ? 100 : 0;
      gen_line(vc_of(i % 3 == 2), urange(1, 8), 1'b0, 1'b0);
      repeat (SYNC + 8) atick();
      rdy_pct = 100;
      wait_idle("latency");
    end
    need((n_probe - p0) >= 24, "latency of every single beat");
  endtask

  // Abort in many positions, masks 01 / 10 / 11, TREADY changing at the abort boundary.
  task automatic abort_case(input int w, input logic [1:0] mask, input int mode);
    rdy_pct = 100; hold_last = 0; hold_null = 0;
    wait_idle("abort case start");
    case (mode)
      1: rdy_pct = 50;
      2: hold_last = 6;
      3: begin hold_null = 6; rdy_pct = 60; end
      default: rdy_pct = 100;
    endcase
    start_capture(1'b1);
    wait_src_empty("abort case capture");
    case (w)
      0, 1: begin
        gen_line(vc_of(w[0]), 64, 1'b0, 1'b0);
        src_budget = urange(2, 5);
        wait_budget("open line");
        ticks(urange(0, 3));
        abort_now(mask, 1'b1);
      end
      2: begin
        ticks(urange(0, 5));
        abort_now(mask, urange(0, 1));
      end
      3: begin
        gen_line(vc_of(urange(0, 1)), 64, 1'b0, 1'b0);
        wait_present(P_MID, "abort on a take without TLAST");
        abort_now(mask, 1'b1);
      end
      4: begin
        gen_line(vc_of(1'b0), 40, 1'b1, 1'b0);
        wait_present(P_SOF_LONG, "abort on a Frame Start take");
        abort_now(mask, 1'b1);
      end
      default: begin
        gen_line(vc_of(urange(0, 1)), 40, 1'b0, urange(0, 1));
        wait_present(P_EOL, "abort on a take with TLAST");
        abort_now(mask, 1'b1);
      end
    endcase
    tick();
    wait_seq_end("abort case");
    wait_idle("abort case end");
    hold_last = 0; hold_null = 0;
  endtask

  task automatic test_abort_cases();
    logic [1:0] masks [3] = '{2'b01, 2'b10, 2'b11};
    int g0;
    g0 = n_groups;
    for (int w = 0; w < 6; w++)
      for (int m = 0; m < 3; m++) abort_case(w, masks[m], (w * 3 + m + seed) % 4);
    need((n_groups - g0) == 18, "an abort sequence for every case");
    need(n_hold_last > 0, "TREADY low on the last real transfer before an abort");
    need(n_hold_null > 0, "TREADY low on an abort transfer");
    need((n_null[0] > 0) && (n_null[1] > 0), "abort transfers on both TIDs");
  endtask

  // Abort with TREADY low and a full FIFO: sink mode, no timeout, held abort transfer.
  task automatic test_abort_stalled();
    longint s0;
    int     g0;
    g0 = n_groups;
    fill_open_line("abort with full FIFO");
    gen_capture(4, 4, 64, 160, 0);
    abort_now(2'b11, 1'b0);
    s0 = n_take_sink;
    ticks(200);
    need((n_take_sink - s0) > 100, "beats taken in sink mode");
    repeat (300) atick();
    need(m_st == ST_INJ, "abort entries waiting while the FIFO is full");
    hold_null = 8;
    rdy_pct = 100;
    wait_seq_end("abort with full FIFO");
    wait_src_empty("abort with full FIFO");
    start_capture(1'b1);
    wait_idle("abort with full FIFO");
    need(n_hold_null > 0, "abort transfer held with TREADY low");
    hold_null = 0;
    // Short line open, mask 01, TREADY at 25 % while the FIFO is full
    rdy_pct = 0;
    start_capture(1'b1);
    gen_line(vc_of(1'b1), 8 * (DEPTH + 64), 1'b0, 1'b0);
    wait_full(16, "abort with Short line open");
    abort_now(2'b01, 1'b0);
    ticks(50);
    rdy_pct = 25;
    wait_seq_end("abort with Short line open");
    rdy_pct = 100;
    wait_src_empty("abort with Short line open");
    wait_idle("abort with Short line open");
    need((n_groups - g0) == 2, "two abort sequences");
  endtask

  // Re-assertion at given cycles after the request; FIFO not full, so the writes are exact.
  task automatic reassert_case(input int lt, input logic [1:0] m0, input int c1,
                               input logic [1:0] m1, input int c2, input logic [1:0] m2,
                               input int exp_len, input string what);
    rdy_pct = 100;
    wait_idle({"start of ", what});
    start_capture(1'b1);
    wait_src_empty(what);
    if (lt >= 0) begin
      gen_line(vc_of(lt[0]), 48, 1'b0, 1'b0);
      src_budget = 1;
      wait_budget(what);
    end
    wait_drained({"line open in ", what});
    abort_now(m0, 1'b0);
    for (int i = 1; i <= 4; i++) begin
      tick();
      if (i == c1) abort_now(m1, 1'b0);
      if (i == c2) abort_now(m2, 1'b0);
    end
    wait_seq_end(what);
    need((g_last_nb == 1) && (g_last_len == exp_len), {"exact abort entries in ", what});
    src_flush = 1'b1;
    tick();
    wait_idle(what);
  endtask

  task automatic test_abort_reassert();
    reassert_case(0, 2'b11, 2, 2'b10, 0, 2'b00, 2, "mask of the TID written on the exit cycle");
    reassert_case(0, 2'b11, 2, 2'b01, 0, 2'b00, 3, "other TID on the exit cycle");
    reassert_case(0, 2'b11, 1, 2'b01, 0, 2'b00, 2, "TID written on the same cycle");
    reassert_case(0, 2'b11, 1, 2'b10, 0, 2'b00, 2, "pending TID on the first write");
    reassert_case(0, 2'b11, 1, 2'b11, 2, 2'b11, 3, "back-to-back requests");
    reassert_case(1, 2'b01, 1, 2'b10, 0, 2'b00, 2, "Short line, its TID on its write");
    reassert_case(1, 2'b01, 2, 2'b10, 0, 2'b00, 3, "Short line, its TID after its write");
    reassert_case(-1, 2'b11, 1, 2'b11, 0, 2'b00, 2, "no line open");
    reassert_case(0, 2'b11, 3, 2'b01, 0, 2'b00, 1, "new request after the end");
  endtask

  // Nothing to terminate: mask 00 without an open line, after reset and after soft_reset.
  task automatic test_abort_nothing_open();
    int d0, g0;
    d0 = n_done_now; g0 = n_groups;
    abort_idle = 0;
    rst_assert();
    repeat (3) @(posedge sys_clk);
    rst_release_axis();
    rst_release_sys();
    #0.2;
    abort_now(2'b00, 1'b0);
    tick();
    need(n_done_now == d0 + 1, "asm_abort_done on the request cycle after reset");
    rdy_pct = 100;
    start_capture(1'b1);
    wait_idle("capture");
    abort_now(2'b00, 1'b0);
    tick();
    gen_line(vc_of(1'b0), 30, 1'b0, 1'b0);
    gen_line(vc_of(1'b1), 30, 1'b0, 1'b0);
    start_capture(1'b1);
    wait_idle("frame gate cleared by an empty abort");
    // Short line open, then resetn: the line is not closed later
    gen_line(vc_of(1'b1), 64, 1'b0, 1'b0);
    src_budget = 3;
    wait_budget("Short line before resetn");
    ticks(2);
    do_reset(3, 1'b0, 2);
    abort_now(2'b00, 1'b0);
    tick();
    gen_line(vc_of(1'b1), 20, 1'b0, 1'b0);
    wait_idle("abort after resetn");
    // Short line open, then soft_reset
    start_capture(1'b1);
    gen_line(vc_of(1'b1), 64, 1'b0, 1'b0);
    wait_src_empty("before soft_reset");
    wait_idle("before soft_reset");
    gen_line(vc_of(1'b1), 64, 1'b0, 1'b0);
    src_budget = 2;
    wait_budget("Short line before soft_reset");
    sr_pulse(1, 1'b1);
    wait_idle("soft_reset");
    abort_now(2'b00, 1'b0);
    tick();
    wait_idle("abort after soft_reset");
    need(n_done_now == d0 + 4, "asm_abort_done on every request with nothing to close");
    need(n_groups == g0, "no abort entry for an empty request");
    // mask 00 with a line open closes that line only
    start_capture(1'b0);
    wait_src_empty("line open with mask 00");
    gen_line(vc_of(1'b0), 64, 1'b0, 1'b0);
    src_budget = 2;
    wait_budget("line open with mask 00");
    abort_now(2'b00, 1'b1);
    tick();
    wait_seq_end("line open with mask 00");
    wait_idle("line open with mask 00");
    need((g_last_nb == 1) && (g_last_len == 1), "one abort transfer for the open line");
    abort_idle = -1;
  endtask

  // Frame gate: Short only with Long, frames cut by aborts, Frame Starts during an abort.
  task automatic test_frame_gate();
    int b0, f0;
    rdy_pct = 70;
    gen_capture(3, 3, 8, 80, 0);
    wait_idle("capture");
    abort_now(2'b11, 1'b1);
    tick();
    wait_seq_end("abort before the Short frame");
    b0 = n_sof_blocked;
    gen_line(vc_of(1'b1), 24, 1'b1, 1'b0);
    gen_line(vc_of(1'b1), 24, 1'b0, 1'b0);
    gen_line(vc_of(1'b0), 24, 1'b0, 1'b0);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle("Short Frame Start while Long is not passing");
    need(n_sof_blocked > b0, "Short Frame Start blocked");
    // frame cut by an abort without flushing the source
    gen_capture(6, 6, 32, 120, 0);
    gen_capture(2, 2, 8, 40, 0);
    ticks(40);
    abort_now(2'b11, 1'b0);
    wait_idle("frame cut by an abort");
    // Frame Starts taken during the abort sequence
    f0 = n_sof_inj;
    fill_open_line("Frame Start during abort");
    abort_now(2'b11, 1'b0);
    tick();
    src_flush = 1'b1;
    tick();
    gen_capture(2, 2, 16, 40, 0);
    gen_line(vc_of(1'b0), 40, 1'b0, 1'b0);
    gen_line(vc_of(1'b1), 40, 1'b0, 1'b0);
    ticks(150);
    need(m_st == ST_INJ, "abort sequence still open");
    rdy_pct = 100;
    wait_seq_end("Frame Start during abort");
    gen_capture(2, 2, 16, 40, 0);
    wait_idle("Frame Start during abort");
    need((n_sof_inj - f0) >= 2, "Frame Starts taken during an abort sequence");
  endtask

  // Skip: mask 01 at a Frame Start and between frames, during an abort, while disabled.
  task automatic test_skip();
    int s0;
    s0 = n_sof_skip;
    rdy_pct = 80;
    // request on the cycle of the Long Frame Start of B: B passes, C is skipped
    gen_capture(2, 2, 8, 40, 0);
    gen_capture(2, 2, 8, 40, 0);
    gen_capture(2, 2, 8, 40, 0);
    gen_capture(2, 2, 8, 40, 0);
    wait_present(P_SOF_LONG, "Frame Start of A");
    wait_present(P_SOF_LONG, "Frame Start of B");
    skip_now(2'b01);
    tick();
    wait_idle("skip on a Frame Start");
    need(n_sof_skip == s0 + 1, "one capture skipped");
    // request between frames
    skip_now(2'b01);
    tick();
    gen_capture(2, 2, 8, 40, 0);
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("skip between frames");
    // request during an abort sequence, no Frame Start in it
    fill_open_line("skip during abort");
    abort_now(2'b11, 1'b1);
    tick();
    skip_now(2'b01);
    ticks(20);
    rdy_pct = 100;
    wait_seq_end("skip during abort");
    gen_capture(1, 1, 8, 40, 0);
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("skip during abort");
    // request during an abort sequence, then a Long Frame Start in it
    fill_open_line("skip cleared during abort");
    abort_now(2'b11, 1'b1);
    tick();
    skip_now(2'b01);
    tick();
    gen_line(vc_of(1'b0), 16, 1'b1, 1'b0);
    ticks(30);
    rdy_pct = 100;
    wait_seq_end("skip cleared during abort");
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("skip cleared during abort");
    // request while axis_enable = 0, no frame meanwhile: the first capture after is skipped
    axis_enable = 1'b0;
    ticks(5);
    skip_now(2'b01);
    ticks(5);
    axis_enable = 1'b1;
    gen_capture(2, 1, 8, 40, 0);
    gen_capture(2, 1, 8, 40, 0);
    wait_idle("skip while disabled");
    // request while disabled, then a capture taken while disabled clears it
    axis_enable = 1'b0;
    skip_now(2'b01);
    tick();
    gen_capture(1, 1, 8, 24, 0);
    wait_src_empty("capture while disabled");
    axis_enable = 1'b1;
    gen_capture(2, 1, 8, 40, 0);
    wait_idle("skip cleared while disabled");
    // mask held at 01 without a request: nothing is skipped
    skip_idle = 1;
    gen_capture(2, 2, 8, 40, 0);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle("mask held without request");
    skip_idle = -1;
    // mask 10 and 11 on separate captures
    skip_now(2'b10);
    tick();
    gen_capture(2, 2, 8, 40, 0);
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("skip mask 10");
    skip_now(2'b11);
    tick();
    gen_capture(2, 2, 8, 40, 0);
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("skip mask 11");
    need(n_sof_skip == s0 + 8, "skipped Frame Starts of every case");
  endtask

  // axis_enable cleared mid-line and between frames, with and without a following abort.
  task automatic test_axis_enable();
    longint a0;
    a0 = n_take_ae0;
    rdy_pct = 30;
    gen_capture(4, 4, 64, 160, 0);
    gen_capture(1, 1, 8, 40, 0);
    ticks(60);
    axis_enable = 1'b0;
    ticks(20);
    abort_now(2'b11, 1'b1);
    ticks(10);
    axis_enable = 1'b1;
    gen_line(vc_of(1'b0), 40, 1'b0, 1'b0);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle("disable with abort");
    start_capture(1'b1);
    gen_line(vc_of(1'b0), 800, 1'b0, 1'b0);
    gen_line(vc_of(1'b1), 40, 1'b0, 1'b0);
    gen_capture(2, 1, 8, 40, 0);
    wait_present(P_MID, "disable without abort");
    ticks(urange(0, 20));
    axis_enable = 1'b0;
    ticks(30);
    axis_enable = 1'b1;
    wait_idle("disable without abort");
    axis_enable = 1'b0;
    ticks(10);
    axis_enable = 1'b1;
    gen_capture(2, 2, 8, 40, 0);
    wait_idle("disable between frames");
    gen_capture(2, 2, 8, 40, 0);
    wait_present(P_SOF_LONG, "Frame Start while disabled");
    axis_enable = 1'b0;
    ticks(8);
    axis_enable = 1'b1;
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("Frame Start while disabled");
    need(n_take_ae0 > a0, "beats taken while axis_enable is 0");
  endtask

  // soft_reset in many positions; the beat presented after it reaches the ISP.
  task automatic test_soft_reset();
    int     r0;
    longint s0;
    r0 = n_sr;
    // mid-line, new capture right after
    rdy_pct = 60;
    gen_capture(3, 3, 40, 160, 0);
    ticks(urange(20, 60));
    sr_pulse(1, 1'b1);
    gen_capture(2, 2, 8, 40, 0);
    wait_idle("soft_reset mid-line");
    // FIFO full
    rdy_pct = 0;
    gen_capture(4 + DEPTH / 16, 4 + DEPTH / 16, 64, 160, 0);
    wait_full(16, "soft_reset with full FIFO");
    sr_pulse(3, 1'b1);
    gen_capture(1, 1, 8, 40, 0);
    ticks(10);
    rdy_pct = 100;
    wait_idle("soft_reset with full FIFO");
    // TREADY low with a transfer presented
    rdy_pct = 0;
    gen_capture(2, 2, 8, 40, 0);
    wait_tvalid(1'b0, "soft_reset with TREADY low");
    ticks(urange(0, 4));
    sr_pulse(1, 1'b0);
    ticks(10);
    rdy_pct = 100;
    wait_idle("soft_reset with TREADY low");
    // during an abort sequence
    fill_open_line("soft_reset during abort");
    abort_now(2'b11, 1'b0);
    ticks(urange(2, 20));
    sr_pulse(2, 1'b1);
    need(m_st == ST_PT, "abort sequence cleared by soft_reset");
    gen_capture(1, 1, 8, 40, 0);
    rdy_pct = 100;
    wait_idle("soft_reset during abort");
    // repeated while the flush is pending, then a long level
    rdy_pct = 50;
    gen_capture(3, 3, 40, 120, 0);
    ticks(30);
    sr_pulse(1, 1'b1);
    ticks(1);
    need(!fl_clear, "soft_reset again while the flush is pending");
    sr_pulse(1, 1'b0);
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("repeated soft_reset");
    gen_capture(2, 2, 40, 120, 0);
    ticks(25);
    sr_pulse(25, 1'b1);
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("long soft_reset");
    // long level with Format Bridge still presenting: beats taken while it is 1 are discarded
    s0 = n_take_sr;
    gen_capture(4 + DEPTH / 16, 4 + DEPTH / 16, 160, 400, 0);
    ticks(urange(10, 30));
    soft_reset = 1'b1;
    ticks(2);
    for (int n = 0; (n < WAIT_MAX) && (fbs_ready !== 1'b1); n++) tick();
    ticks(20);
    soft_reset = 1'b0;
    need(n_take_sr > s0, "a beat taken during a long soft_reset");
    wait_idle("long soft_reset, source not flushed");
    // requests on a soft_reset cycle have no effect
    rdy_pct = 100;
    gen_capture(2, 2, 40, 80, 0);
    ticks(20);
    soft_reset = 1'b1;
    abort_now(2'b11, 1'b1);
    skip_now(2'b01);
    tick();
    soft_reset = 1'b0;
    need(m_st == ST_PT, "abort request ignored on a soft_reset cycle");
    gen_capture(1, 1, 8, 40, 0);
    wait_idle("requests on a soft_reset cycle");
    // after soft_reset both pass bits are set: lines without Frame Start are delivered
    sr_pulse(1, 1'b0);
    gen_line(vc_of(1'b1), 24, 1'b0, 1'b0);
    gen_line(vc_of(1'b0), 24, 1'b0, 1'b0);
    wait_idle("lines without Frame Start after soft_reset");
    need((n_sr - r0) >= 9, "every soft_reset case");
  endtask

  // Release order of resetn and axis_resetn with several delays, reset during a transfer.
  task automatic test_reset_release_order();
    int     dl [3] = '{0, 3, 60};
    longint t0;
    for (int o = 0; o < 2; o++) begin
      for (int d = 0; d < 3; d++) begin
        rdy_pct = 70;
        gen_capture(3, 3, 40, 160, 0);
        wait_tvalid(1'b1, "reset during a transfer");
        rst_assert();
        repeat (3) @(posedge sys_clk);
        if (o == 0) begin
          rst_release_sys();
          #0.2;
          t0 = n_take;
          gen_line(vc_of(1'b1), 40, 1'b0, 1'b0);
          gen_capture(2, 2, 8, 40, 0);
          repeat (dl[d]) @(negedge axis_aclk);
          if (d == 2) need(n_take > t0, "beats taken while axis_aclk side is in reset");
          rst_release_axis();
        end else begin
          rst_release_axis();
          repeat (dl[d]) @(negedge sys_clk);
          rst_release_sys();
          #0.2;
          gen_line(vc_of(1'b0), 40, 1'b0, 1'b0);
          gen_capture(2, 2, 8, 40, 0);
        end
        tick();
        wait_idle("reset release order");
      end
    end
  endtask

  // vc_exposure_map 0 and 1 between frames; reserved channels are discarded.
  task automatic test_vc_map_and_reserved();
    beat_t b;
    int    r0;
    r0 = n_reserved;
    rdy_pct = 80;
    gen_capture(3, 3, 8, 60, 10);
    wait_idle("map 0");
    vc_exposure_map = 1'b1;
    gen_capture(3, 3, 8, 60, 10);
    wait_idle("map 1");
    // reserved channels: separate lines, a Frame Start, a beat inside a line
    gen_capture(1, 1, 8, 24, 0);
    gen_line(2'b10, 40, 1'b0, 1'b0);
    gen_line(2'b11, 24, 1'b1, 1'b0);
    gen_line(vc_of(1'b0), 24, 1'b0, 1'b0);
    gen_line(vc_of(1'b0), 32, 1'b0, 1'b0);
    b = src_q[src_q.size() - 1];
    void'(src_q.pop_back());
    b.vc = 2'b10; b.data = {$urandom, $urandom};
    src_q.push_back(b);
    gen_line(vc_of(1'b0), 8, 1'b0, 1'b0);
    wait_idle("reserved channels, map 1");
    vc_exposure_map = 1'b0;
    gen_capture(1, 1, 8, 24, 0);
    gen_line(2'b11, 40, 1'b0, 1'b0);
    gen_line(2'b10, 16, 1'b1, 1'b0);
    gen_line(vc_of(1'b1), 24, 1'b0, 1'b0);
    wait_idle("reserved channels, map 0");
    need((n_reserved - r0) >= 10, "reserved-channel beats taken");
  endtask

  // Long random run: captures, aborts, skips, enable toggles, soft_reset, random TREADY.
  task automatic test_random_all();
    int a, pcts [4] = '{10, 40, 75, 100};
    for (int s = 0; s < 60; s++) begin
      a = urange(0, 99);
      if (a < 35) begin
        gap_pct = (urange(0, 3) == 0) ? 25 : 0;
        gen_capture(urange(1, 4), urange(0, 4), 1, 160, 15);
      end else if (a < 50) begin
        abort_now((urange(0, 3) == 0) ? 2'b01 : 2'b11, urange(0, 1));
      end else if (a < 55) begin
        abort_now(2'b11, 1'b0);
        ticks(urange(2, 4));
        abort_now((urange(0, 1) == 0) ? 2'b01 : 2'b11, 1'b0);
      end else if (a < 63) begin
        skip_now(2'b01);
      end else if (a < 71) begin
        axis_enable = 1'b0;
        ticks(urange(1, 40));
        abort_now(2'b11, 1'b1);
        ticks(urange(1, 10));
        axis_enable = 1'b1;
      end else if (a < 79) begin
        sr_pulse(urange(1, 4), urange(0, 1));
      end else if (a < 92) begin
        rdy_pct = pcts[urange(0, 3)];
      end else begin
        rdy_pct = 100;
        wait_idle("random run, map change");
        vc_exposure_map = ~vc_exposure_map;
      end
      tick();
      ticks(urange(0, 120));
    end
    rdy_pct = 100; axis_enable = 1'b1; gap_pct = 0;
    wait_idle("random run");
    vc_exposure_map = 1'b0;
  endtask

  // Packet rules: odd byte counts, one-byte lines, abort transfers on both TIDs.
  task automatic test_packet_rules();
    longint k0;
    k0 = n_keep01;
    rdy_pct = 60;
    start_capture(1'b1);
    for (int i = 0; i < 16; i++) gen_line(vc_of(i % 2), 2 * i + 1, 1'b0, i % 4 == 1);
    for (int i = 0; i < 6; i++) gen_line(vc_of(i % 2), 1, 1'b0, 1'b0);
    wait_idle("odd byte counts");
    abort_now(2'b11, 1'b0);
    tick();
    wait_idle("abort between lines");
    start_capture(1'b1);
    wait_src_empty("Short line open");
    gen_line(vc_of(1'b1), 33, 1'b0, 1'b0);
    src_budget = 2;
    wait_budget("Short line open");
    abort_now(2'b11, 1'b1);
    tick();
    wait_idle("abort with Short line open");
    need((n_keep01 - k0) >= 16, "TKEEP 2'b01 on the last transfer of odd lines");
    need((n_null[0] >= 2) && (n_null[1] >= 2), "abort transfers on both TIDs");
  endtask

  // The expected data of one transfer is altered: the run must end in FAIL.
  task automatic test_checker_fault();
    fault_on = 1'b1;
    rdy_pct = 100;
    gen_capture(2, 1, 16, 40, 0);
    wait_idle("checker fault");
  endtask

  // ------------------------------------------------------------------------------------------
  // Test selection and final summary
  // ------------------------------------------------------------------------------------------
  initial begin : p_watchdog
    #(20000000.0);
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

  initial begin : p_main
    int    ratio;
    string rname;
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "test_reset_both_domains";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    dbg = $test$plusargs("DBG");
    void'($urandom(seed));
    n_null[0] = 0; n_null[1] = 0;
    ratio = (seed < 1) ? 0 : ((seed - 1) % 3);
    case (ratio)
      0:       begin t_ax = 1.818; t_sys = 4.464; rname = "550/224 MHz"; end
      1:       begin t_ax = 4.464; t_sys = 4.464; rname = "equal"; end
      default: begin t_ax = 10.0;  t_sys = 4.464; rname = "100/224 MHz"; end
    endcase
    t_ph = real'(urange(150, int'(t_ax * 1000.0) - 150)) / 1000.0;
    $display("%s seed %0d: SYNC_STAGES %0d, AXIS_FIFO_DEPTH %0d, clocks %s, phase %.3f ns",
             test_name, seed, SYNC, DEPTH, rname, t_ph);
    model_reset();
    drive_fields();
    clk_go = 1'b1;
    repeat (2) @(posedge sys_clk);
    chk_on = 1'b1;
    repeat (3) @(posedge sys_clk);
    rst_release_sys();
    repeat (2) @(negedge axis_aclk);
    rst_release_axis();
    #0.2;

    case (test_name)
      "test_reset_both_domains":   test_reset_both_domains();
      "test_line_lengths":         test_line_lengths();
      "test_random_streaming":     test_random_streaming();
      "test_fifo_full_stall":      test_fifo_full_stall();
      "test_throughput":           test_throughput();
      "test_latency":              test_latency();
      "test_abort_cases":          test_abort_cases();
      "test_abort_stalled":        test_abort_stalled();
      "test_abort_reassert":       test_abort_reassert();
      "test_abort_nothing_open":   test_abort_nothing_open();
      "test_frame_gate":           test_frame_gate();
      "test_skip":                 test_skip();
      "test_axis_enable":          test_axis_enable();
      "test_soft_reset":           test_soft_reset();
      "test_reset_release_order":  test_reset_release_order();
      "test_vc_map_and_reserved":  test_vc_map_and_reserved();
      "test_random_all":           test_random_all();
      "test_packet_rules":         test_packet_rules();
      "test_checker_fault":        test_checker_fault();
      default: report({"unknown test ", test_name});
    endcase

    ticks(20);
    $display("sys cycles %0d, takes %0d, accepted %0d, transfers %0d (abort %0d), checks %0d",
             n_sys, n_take, n_acc, n_hs, n_null[0] + n_null[1], checks);
    $display("abort sequences %0d, soft_resets %0d, latency probes %0d, errors %0d",
             n_groups, n_sr, n_probe, err_count);
    if ((err_count == 0) && (n_hs > 0) && (checks > 0)) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d)", test_name, err_count);
    $finish;
  end

endmodule
