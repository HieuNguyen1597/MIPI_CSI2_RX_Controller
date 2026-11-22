// File        : tb_top.sv
// Description : Testbench of m_fpt_control_fsm (drop causes, Capture Watch, Drop Report and the
//               Drop FSM with its command outputs and discard flags).
//               A cycle-based reference model, written from the specification equations, predicts
//               every output cycle by cycle; a checker compares the outputs with it at each sys_clk
//               edge. A second set of checkers tests rules that hold whatever the traffic: flush
//               and hold sequence, one-cycle commands, masks with their request, reset values,
//               X/Z. A small model of Frame Context & Metadata Capture and of the DMA produces the
//               capture events, and a scoreboard counts the dropped captures against
//               frame_drop_pulse. Tests drive stimulus and check explicit expected values.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions (agreed with the block owner; the specification does not state them yet):
//   - hold_cnt is 0 outside ST_HOLD and counts the ST_HOLD cycles, so ST_HOLD lasts HOLD_CYCLES
//     = 2 cycles, dma_abort_req is in its first cycle and lb_flush follows two cycles later.
//   - lead_seen is cleared in ST_RUN and also in ST_IDLE: a lead capture opened in ST_HOLD or
//     ST_FLUSH does not resume reception after the block has gone through ST_IDLE.
//   - When two slots end dropped in the same cycle, the second frame_drop_pulse follows one cycle
//     later; a new dropped end in that cycle gives one pulse only.
//   - dma_drop with stream_follow_dma_drop = 1 is followed on the stream path only in ST_RUN. A
//     dma_drop for a capture opened in ST_HOLD or ST_FLUSH gives no asm_skip_req and no
//     capture_drop; that capture is still counted as dropped at its capture_end.
//   - An abort cause and a DMA skip in the same ST_RUN cycle give both capture_drop (mask 11) and
//     asm_skip_req (mask 01) in the next cycle, then the abort sequence.
//   - rx_ctx_clear is issued only in ST_FLUSH with active_sel = 0; core_enable low with no
//     capture open and no mid-line cause goes to ST_IDLE without rx_ctx_clear.
//   - resetn and soft_reset reset every register, the one-cycle delays and axis_enable_q
//     included: an event in a soft_reset cycle gives no command in the next cycle.
//   - cap_busy does not include a capture_start of the current cycle.
//   - err_vc[1] and ev_vc[1] are not used: Virtual Channel 2 acts as 0 and 3 as 1.
//   - core_enable low in ST_RUN is a drop cause in every cycle it is low.
//   - asm_abort_req is one pulse per cycle in which ST_FLUSH or stream_stop_q holds.
//   - drop_vc_mask, asm_abort_mask and asm_skip_mask are meaningful only with their request.
//   - Connected blocks: capture_start has at most one bit, only with a Frame Start of the lead
//     channel, for a free slot; capture_end of an open slot at least two cycles after its
//     capture_start, at most one per cycle; capture_dropped is set for a capture open during a
//     capture_drop or while core_enable is low; dma_drop comes one cycle after capture_start;
//     no capture event while resetn is low or soft_reset is high; ev_fs carries Virtual Channel 0
//     or 1; lead_vc is vc_exposure_map on the sensor path and 0 on the Test Pattern Generator
//     path; active_sel, vc_exposure_map and tpg_dual_vc_en change only with no capture open.
//     Some tests break one of these rules on purpose and turn its stimulus check off.
`timescale 1ns/1ps

module tb_top;

  // ST_HOLD length of the specification (OUT_PIPE_STAGES of Byte-to-Pixel Unpacking plus one).
  localparam int HOLD_CYCLES = 2;

  // sys_clk, 200 MHz.
  localparam real T_SYS_CLK = 5.0;

  // Command outputs as {capture_drop, dma_abort_req, lb_flush, asm_abort_req, asm_skip_req,
  // rx_ctx_clear, frame_drop_pulse}.
  localparam logic [6:0] C_NONE      = 7'b000_0000;
  localparam logic [6:0] C_DROP      = 7'b100_0000;
  localparam logic [6:0] C_DROP_HOLD = 7'b110_0000;
  localparam logic [6:0] C_HOLD      = 7'b010_0000;
  localparam logic [6:0] C_FLUSH_S   = 7'b001_1010;
  localparam logic [6:0] C_FLUSH_T   = 7'b001_1000;
  localparam logic [6:0] C_STOP      = 7'b000_1000;
  localparam logic [6:0] C_SKIP      = 7'b100_0100;
  localparam logic [6:0] C_PULSE     = 7'b000_0001;

  // Causes driven by cause(): pulses; linebuffer_full and core_enable are levels set by the tests.
  localparam int K_ECC2  = 0;
  localparam int K_CRC   = 1;
  localparam int K_WC    = 2;
  localparam int K_LANE  = 3;
  localparam int K_LBOVF = 4;
  localparam int K_FBOVF = 5;

  // DUT inputs.
  logic       i_sys_clk, i_resetn, i_soft_reset;
  logic       i_core_enable, i_axis_enable, i_stream_follow_dma_drop;
  logic       i_vc_exposure_map, i_tpg_dual_vc_en;
  logic       i_err_ecc_2bit, i_err_lane_abort, i_err_crc, i_err_wc;
  logic [1:0] i_err_vc;
  logic       i_active_sel;
  logic       i_linebuffer_full, i_linebuf_overflow, i_fb_overflow;
  logic [1:0] i_capture_start, i_capture_end, i_capture_dropped;
  logic       i_ev_fs;
  logic [1:0] i_ev_vc;
  logic       i_lead_vc;
  logic       i_dma_drop;

  // DUT outputs.
  logic [1:0] o_force_discard;
  logic       o_rx_ctx_clear, o_lb_flush, o_fb_flush;
  logic       o_capture_drop;
  logic [1:0] o_drop_vc_mask;
  logic       o_asm_abort_req;
  logic [1:0] o_asm_abort_mask;
  logic       o_asm_skip_req;
  logic [1:0] o_asm_skip_mask;
  logic       o_dma_abort_req, o_frame_drop_pulse;

  m_fpt_control_fsm u_dut (
    .i_sys_clk                (i_sys_clk),
    .i_resetn                 (i_resetn),
    .i_soft_reset             (i_soft_reset),
    .i_core_enable            (i_core_enable),
    .i_axis_enable            (i_axis_enable),
    .i_stream_follow_dma_drop (i_stream_follow_dma_drop),
    .i_vc_exposure_map        (i_vc_exposure_map),
    .i_tpg_dual_vc_en         (i_tpg_dual_vc_en),
    .i_err_ecc_2bit           (i_err_ecc_2bit),
    .i_err_lane_abort         (i_err_lane_abort),
    .i_err_crc                (i_err_crc),
    .i_err_wc                 (i_err_wc),
    .i_err_vc                 (i_err_vc),
    .o_force_discard          (o_force_discard),
    .i_active_sel             (i_active_sel),
    .o_rx_ctx_clear           (o_rx_ctx_clear),
    .i_linebuffer_full        (i_linebuffer_full),
    .i_linebuf_overflow       (i_linebuf_overflow),
    .o_lb_flush               (o_lb_flush),
    .i_fb_overflow            (i_fb_overflow),
    .o_fb_flush               (o_fb_flush),
    .i_capture_start          (i_capture_start),
    .i_capture_end            (i_capture_end),
    .i_capture_dropped        (i_capture_dropped),
    .i_ev_fs                  (i_ev_fs),
    .i_ev_vc                  (i_ev_vc),
    .i_lead_vc                (i_lead_vc),
    .o_capture_drop           (o_capture_drop),
    .o_drop_vc_mask           (o_drop_vc_mask),
    .o_asm_abort_req          (o_asm_abort_req),
    .o_asm_abort_mask         (o_asm_abort_mask),
    .o_asm_skip_req           (o_asm_skip_req),
    .o_asm_skip_mask          (o_asm_skip_mask),
    .i_dma_drop               (i_dma_drop),
    .o_dma_abort_req          (o_dma_abort_req),
    .o_frame_drop_pulse       (o_frame_drop_pulse)
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
  task automatic expect_eq(input string what, input logic [31:0] got, input logic [31:0] want);
    check_count++;
    if (got !== want) fail($sformatf("%s = %0b, expected %0b", what, got, want));
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

  function automatic int rand_range(input int lo, input int hi);
    return int'($urandom_range(hi, lo));
  endfunction

  function automatic bit rand_pct(input int pct);
    return (rand_range(0, 99) < pct);
  endfunction

  function automatic bit rand_pm(input int per_mille);
    return (rand_range(0, 999) < per_mille);
  endfunction

  function automatic logic [1:0] onehot(input logic vc);
    return vc ? 2'b10 : 2'b01;
  endfunction

  function automatic logic [6:0] cmds();
    return {o_capture_drop, o_dma_abort_req, o_lb_flush, o_asm_abort_req, o_asm_skip_req,
            o_rx_ctx_clear, o_frame_drop_pulse};
  endfunction

  // --------------------------------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------------------------------
  // lead_vc of Frame Context & Metadata Capture.
  assign i_lead_vc = i_active_sel ? 1'b0 : i_vc_exposure_map;

  // Stimulus rules that some tests break on purpose.
  bit relax_two_ends  = 1'b0;  // two capture_end in one cycle
  bit relax_end_age   = 1'b0;  // capture_end of a closed slot or too early after capture_start
  bit relax_dma       = 1'b0;  // dma_drop not one cycle after capture_start
  bit relax_vc23      = 1'b0;  // ev_fs on Virtual Channel 2 or 3
  bit relax_soft_ev   = 1'b0;  // capture event in a soft_reset cycle

  logic [1:0] env_noise = 2'b00;

  // Pulse inputs are cleared 0.2 ns after every rising edge, so a value set by a test lasts one
  // cycle. err_vc, ev_vc and capture_dropped without their qualifier carry random values.
  task automatic clear_pulses();
    logic [31:0] r;
    r = $urandom;
    i_err_ecc_2bit     = 1'b0;
    i_err_lane_abort   = 1'b0;
    i_err_crc          = 1'b0;
    i_err_wc           = 1'b0;
    i_linebuf_overflow = 1'b0;
    i_fb_overflow      = 1'b0;
    i_capture_start    = 2'b00;
    i_capture_end      = 2'b00;
    i_ev_fs            = 1'b0;
    i_dma_drop         = 1'b0;
    i_err_vc           = {1'b0, r[0]};
    i_ev_vc            = {1'b0, r[1]};
    env_noise          = r[3:2];
  endtask

  always @(posedge i_sys_clk) begin
    #0.2;
    clear_pulses();
  end

  // --------------------------------------------------------------------------------------------
  // Frame Context & Metadata Capture and DMA, as seen by this block
  // --------------------------------------------------------------------------------------------
  // env_open: slot open; env_dflag: capture already marked dropped; env_dma: dma_drop received
  // for the capture of the slot; env_age: clock edges since the capture_start of the slot.
  logic [1:0] env_open  = 2'b00;
  logic [1:0] env_dflag = 2'b00;
  logic [1:0] env_dma   = 2'b00;
  logic       env_last  = 1'b0;
  int         env_age [2];

  // A capture is marked when it is open in the cycle of a capture_drop, including the cycle of
  // its own capture_end. Tests that need another value of capture_dropped override it.
  bit         env_ovr     = 1'b0;
  logic [1:0] env_ovr_val = 2'b00;
  logic [1:0] env_dropped_now;

  always_comb begin
    for (int s = 0; s < 2; s++)
      env_dropped_now[s] = env_dflag[s] | (o_capture_drop & env_open[s]);
  end

  assign i_capture_dropped = env_ovr ? env_ovr_val
                                     : ((i_capture_end & env_dropped_now) |
                                        (~i_capture_end & env_noise));

  always @(posedge i_sys_clk or negedge i_resetn) begin
    if (!i_resetn) begin
      env_open  <= 2'b00;
      env_dflag <= 2'b00;
      env_dma   <= 2'b00;
      env_last  <= 1'b0;
      env_age   <= '{0, 0};
    end else if (i_soft_reset) begin
      // Frame Context & Metadata Capture takes the same soft_reset.
      env_open  <= 2'b00;
      env_dflag <= 2'b00;
      env_dma   <= 2'b00;
      env_last  <= 1'b0;
    end else begin
      for (int s = 0; s < 2; s++) begin
        if (i_capture_start[s]) begin
          env_open[s]  <= 1'b1;
          env_dflag[s] <= 1'b0;
          env_dma[s]   <= 1'b0;
          env_age[s]   <= 0;
        end else begin
          if (i_capture_end[s]) env_open[s] <= 1'b0;
          if (env_open[s] && (o_capture_drop || !i_core_enable)) env_dflag[s] <= 1'b1;
          if (i_dma_drop && (env_last == 1'(s))) env_dma[s] <= 1'b1;
          env_age[s] <= env_age[s] + 1;
        end
      end
      if (i_capture_start[1])      env_last <= 1'b1;
      else if (i_capture_start[0]) env_last <= 1'b0;
    end
  end

  // Scoreboard: frame_drop_pulse events equal the capture_end events of captures marked by
  // capture_dropped or by a dma_drop.
  bit sb_off    = 1'b0;
  int sb_expect = 0;
  int sb_pulses = 0;
  int n_fdp     = 0;

  always @(posedge i_sys_clk) begin
    if (started && i_resetn === 1'b1) begin
      if (o_frame_drop_pulse === 1'b1) n_fdp++;
      if (!sb_off) begin
        for (int s = 0; s < 2; s++)
          if (i_capture_end[s] && (i_capture_dropped[s] || env_dma[s])) sb_expect++;
        if (o_frame_drop_pulse === 1'b1) sb_pulses++;
      end
    end
  end

  task automatic sb_clear();
    sb_expect = 0;
    sb_pulses = 0;
  endtask

  // --------------------------------------------------------------------------------------------
  // Reference model
  // --------------------------------------------------------------------------------------------
  typedef enum logic [2:0] {S_IDLE, S_RUN, S_HOLD, S_FLUSH, S_SKIP} st_t;

  st_t        m_st;
  int         m_hold;
  logic       m_lead_seen, m_last, m_pend;
  logic       m_axis_q, m_drop_q, m_skip_q, m_core_q, m_stop_q;
  logic [1:0] m_discard, m_open, m_mark;

  // Values of the current cycle.
  logic       w_abort_any, w_midline, w_stop, w_dskip, w_lead_start, w_lag_fs;
  logic       w_drop_now, w_skip_now;
  logic [1:0] w_drop_end;
  logic [1:0] e_fd, e_dmask, e_amask;
  logic       e_flush, e_rxclr, e_abort, e_dmaab, e_cdrop, e_skip, e_fdp;

  // Event counts, read by the random test.
  int n_hold = 0, n_skip_entry = 0, n_idle_entry = 0, n_resume = 0, n_seen_resume = 0;
  int n_dma_skip = 0, n_flush_s = 0, n_flush_t = 0, n_stop = 0;

  task automatic model_reset();
    m_st        = S_IDLE;
    m_hold      = 0;
    m_lead_seen = 1'b0;
    m_last      = 1'b0;
    m_pend      = 1'b0;
    m_axis_q    = 1'b0;
    m_drop_q    = 1'b0;
    m_skip_q    = 1'b0;
    m_core_q    = 1'b0;
    m_stop_q    = 1'b0;
    m_discard   = 2'b11;
    m_open      = 2'b00;
    m_mark      = 2'b00;
  endtask

  task automatic model_eval();
    logic rx, c_ecc2, c_crc, c_wc, c_lane, c_lbfull, c_lbovf, c_fbovf, c_core;
    rx           = ~i_active_sel;
    c_ecc2       = i_err_ecc_2bit & rx;
    c_crc        = i_err_crc & rx & ~m_discard[i_err_vc[0]];
    c_wc         = i_err_wc & rx & ~m_discard[i_err_vc[0]];
    c_lane       = i_err_lane_abort & rx;
    c_lbfull     = i_linebuffer_full & rx;
    c_lbovf      = i_linebuf_overflow & rx;
    c_fbovf      = i_fb_overflow;
    c_core       = ~i_core_enable;
    w_abort_any  = c_ecc2 | c_crc | c_wc | c_lane | c_lbfull | c_lbovf | c_fbovf | c_core;
    w_midline    = c_lbfull | c_lbovf | c_fbovf;
    w_stop       = m_axis_q & ~i_axis_enable;
    w_dskip      = i_dma_drop & i_stream_follow_dma_drop;
    w_lead_start = |i_capture_start;
    w_lag_fs     = i_ev_fs & (i_ev_vc[0] != i_lead_vc);
    w_drop_end   = i_capture_end & (i_capture_dropped | m_mark);
    w_drop_now   = (m_st == S_RUN) & w_abort_any;
    w_skip_now   = (m_st == S_RUN) & w_dskip;
    e_fd         = m_discard;
    e_flush      = (m_st == S_FLUSH);
    e_rxclr      = e_flush & ~i_active_sel;
    e_abort      = e_flush | m_stop_q;
    e_amask      = (i_active_sel && !i_tpg_dual_vc_en) ? {i_vc_exposure_map, ~i_vc_exposure_map}
                                                       : 2'b11;
    e_dmaab      = (m_st == S_HOLD) && (m_hold == 0);
    e_cdrop      = (m_drop_q & m_core_q) | m_skip_q;
    e_dmask      = m_drop_q ? 2'b11 : onehot(i_lead_vc);
    e_skip       = m_skip_q;
    e_fdp        = (|w_drop_end) | m_pend;
  endtask

  task automatic cmp(input string what, input logic [1:0] got, input logic [1:0] want);
    if (got !== want)
      fail($sformatf("%s = %b, model expects %b (state %s)", what, got, want, m_st.name()));
  endtask

  // Masks are compared only in the cycle of their request.
  task automatic model_check();
    cmp("force_discard", o_force_discard, e_fd);
    cmp("lb_flush", o_lb_flush, e_flush);
    cmp("fb_flush", o_fb_flush, e_flush);
    cmp("rx_ctx_clear", o_rx_ctx_clear, e_rxclr);
    cmp("asm_abort_req", o_asm_abort_req, e_abort);
    cmp("dma_abort_req", o_dma_abort_req, e_dmaab);
    cmp("capture_drop", o_capture_drop, e_cdrop);
    cmp("asm_skip_req", o_asm_skip_req, e_skip);
    cmp("frame_drop_pulse", o_frame_drop_pulse, e_fdp);
    if (e_cdrop) cmp("drop_vc_mask", o_drop_vc_mask, e_dmask);
    if (e_abort) cmp("asm_abort_mask", o_asm_abort_mask, e_amask);
    if (e_skip)  cmp("asm_skip_mask", o_asm_skip_mask, 2'b01);
  endtask

  task automatic model_update();
    st_t        nst;
    int         nhold;
    logic       nseen, s_set, go_run, nlast;
    logic [1:0] clr, nopen, nmark;
    logic       busy;
    busy = |m_open;
    nst  = m_st;
    case (m_st)
      S_IDLE: if (i_core_enable) nst = S_SKIP;
      S_RUN: begin
        if (w_abort_any && (busy || w_midline))                           nst = S_HOLD;
        else if (w_abort_any && !busy && !w_midline && i_core_enable)     nst = S_SKIP;
        else if (!i_core_enable && !busy && !w_midline)                   nst = S_IDLE;
      end
      S_HOLD:  if (m_hold == HOLD_CYCLES - 1) nst = S_FLUSH;
      S_FLUSH: nst = i_core_enable ? S_SKIP : S_IDLE;
      S_SKIP: begin
        if (!i_core_enable)                      nst = S_IDLE;
        else if (w_lead_start || m_lead_seen)    nst = S_RUN;
      end
      default: nst = S_IDLE;
    endcase
    if (m_st == S_RUN && nst == S_HOLD) n_hold++;
    if (m_st == S_RUN && nst == S_SKIP) n_skip_entry++;
    if (m_st == S_RUN && nst == S_IDLE) n_idle_entry++;
    if (m_st == S_SKIP && nst == S_RUN) begin
      n_resume++;
      if (!w_lead_start) n_seen_resume++;
    end
    if (w_skip_now) n_dma_skip++;
    if (m_st == S_FLUSH && !i_active_sel) n_flush_s++;
    if (m_st == S_FLUSH && i_active_sel)  n_flush_t++;
    if (m_stop_q) n_stop++;

    nhold = (m_st == S_HOLD && m_hold != HOLD_CYCLES - 1) ? m_hold + 1 : 0;
    if ((m_st == S_HOLD || m_st == S_FLUSH) && w_lead_start) nseen = 1'b1;
    else if (m_st == S_RUN || m_st == S_IDLE)               nseen = 1'b0;
    else                                                    nseen = m_lead_seen;

    s_set  = (m_st == S_IDLE) || (m_st == S_HOLD) || (m_st == S_FLUSH) || w_drop_now;
    go_run = (m_st == S_SKIP) && i_core_enable && (w_lead_start || m_lead_seen);
    for (int x = 0; x < 2; x++)
      clr[x] = (go_run && (i_lead_vc == 1'(x))) ||
               (w_lag_fs && !m_discard[i_lead_vc] && (i_lead_vc != 1'(x)));

    nopen = m_open;
    nmark = m_mark;
    for (int s = 0; s < 2; s++) begin
      if (i_capture_start[s]) begin
        nopen[s] = 1'b1;
        nmark[s] = 1'b0;
      end else begin
        if (i_capture_end[s]) nopen[s] = 1'b0;
        if (i_dma_drop && (m_last == 1'(s))) nmark[s] = 1'b1;
      end
    end
    nlast = i_capture_start[1] ? 1'b1 : (i_capture_start[0] ? 1'b0 : m_last);

    m_pend      = w_drop_end[0] & w_drop_end[1];
    m_discard   = s_set ? 2'b11 : (m_discard & ~clr);
    m_st        = nst;
    m_hold      = nhold;
    m_lead_seen = nseen;
    m_open      = nopen;
    m_mark      = nmark;
    m_last      = nlast;
    m_drop_q    = w_drop_now;
    m_skip_q    = w_skip_now;
    m_core_q    = i_core_enable;
    m_stop_q    = w_stop;
    m_axis_q    = i_axis_enable;
  endtask

  // While resetn is low: both discard flags set, every command low.
  task automatic check_reset_outputs();
    if (o_force_discard !== 2'b11) fail("force_discard is not 11 while resetn is low");
    if ({cmds(), o_fb_flush} !== 8'd0) fail("a command is not 0 while resetn is low");
  endtask

  // Reference model and checker, at each rising edge.
  always @(posedge i_sys_clk) begin
    if (started) begin
      if (i_resetn !== 1'b1) begin
        check_reset_outputs();
        model_reset();
      end else begin
        model_eval();
        model_check();
        if (i_soft_reset === 1'b1) model_reset();
        else                       model_update();
      end
    end
  end

  // resetn is asynchronous: the outputs go to their reset value without waiting for an edge.
  always @(negedge i_resetn) begin
    if (started) begin
      model_reset();
      #0.2;
      check_reset_outputs();
    end
  end

  // --------------------------------------------------------------------------------------------
  // Protocol checkers (independent of the model)
  // --------------------------------------------------------------------------------------------
  bit         p_valid = 1'b0;   // the previous edge was sampled with resetn high
  bit         p_rst   = 1'b1;   // the previous cycle was in reset or had soft_reset
  logic       p_lb, p_rx, p_dab, p_core;
  logic [1:0] p_end;
  logic [1:0] dab_hist = 2'b00; // dma_abort_req one and two cycles ago
  logic       x_axis_q = 1'b0;  // axis_enable delayed, as the specification defines axis_enable_q
  logic       x_stop_q = 1'b0;
  logic       p_start_any = 1'b0;

  always @(posedge i_sys_clk) begin
    logic stop_now;
    if (started) begin
      if ($isunknown({o_force_discard, o_rx_ctx_clear, o_lb_flush, o_fb_flush, o_capture_drop,
                      o_drop_vc_mask, o_asm_abort_req, o_asm_abort_mask, o_asm_skip_req,
                      o_asm_skip_mask, o_dma_abort_req, o_frame_drop_pulse}))
        fail("X or Z on an output");
      // Stimulus rules.
      if (i_resetn !== 1'b1 && (i_capture_start != 2'b00 || i_capture_end != 2'b00))
        fail("stimulus: capture event while resetn is low");
      if (!relax_soft_ev && i_soft_reset && (i_capture_start != 2'b00 || i_capture_end != 2'b00))
        fail("stimulus: capture event in a soft_reset cycle");
      if (i_capture_start == 2'b11) fail("stimulus: capture_start on both slots");
      if (i_capture_start != 2'b00 && !(i_ev_fs && i_ev_vc == {1'b0, i_lead_vc}))
        fail($sformatf("stimulus: capture_start %b without a lead Frame Start (ev_fs %b ev_vc %0d)",
                       i_capture_start, i_ev_fs, i_ev_vc));
      if (!relax_two_ends && i_capture_end == 2'b11) fail("stimulus: capture_end on both slots");
      for (int s = 0; s < 2; s++)
        if (!relax_end_age && i_capture_end[s] && !(env_open[s] && env_age[s] >= 1))
          fail($sformatf("stimulus: capture_end of slot %0d not open or too early", s));
      if (!relax_dma && i_dma_drop && !p_start_any)
        fail("stimulus: dma_drop not one cycle after capture_start");
      if (!relax_vc23 && i_ev_fs && i_ev_vc[1]) fail("stimulus: ev_fs outside VC0/VC1");
      if ((i_err_crc || i_err_wc) && i_err_vc[1]) fail("stimulus: err_vc outside VC0/VC1");
      p_start_any = (i_capture_start != 2'b00) && (i_resetn === 1'b1) && !i_soft_reset;

      if (i_resetn !== 1'b1) begin
        p_valid  = 1'b0;
        p_rst    = 1'b1;
        dab_hist = 2'b00;
        x_axis_q = 1'b0;
        x_stop_q = 1'b0;
      end else begin
        // First cycle after a reset: discard flags set, no command.
        if (p_rst && (o_force_discard !== 2'b11 ||
                      {o_capture_drop, o_dma_abort_req, o_lb_flush, o_asm_abort_req,
                       o_asm_skip_req, o_rx_ctx_clear} !== 6'd0))
          fail("command or force_discard after a reset");
        // Flush commands.
        if (o_fb_flush !== o_lb_flush) fail("fb_flush differs from lb_flush");
        if (o_rx_ctx_clear !== (o_lb_flush & ~i_active_sel))
          fail("rx_ctx_clear differs from lb_flush on the sensor path");
        if (o_lb_flush && (o_force_discard !== 2'b11)) fail("flush with force_discard not 11");
        if (o_dma_abort_req && (o_force_discard !== 2'b11))
          fail("dma_abort_req with force_discard not 11");
        if (dab_hist[0] && (o_lb_flush || o_force_discard !== 2'b11))
          fail("flush or force_discard not 11 one cycle after dma_abort_req");
        if (dab_hist[1] && !o_lb_flush) fail("no flush two cycles after dma_abort_req");
        if (o_lb_flush && !dab_hist[1]) fail("flush without dma_abort_req two cycles before");
        // asm_abort_req: ST_FLUSH, or axis_enable fell in the previous cycle.
        if (o_asm_abort_req !== (o_lb_flush | x_stop_q))
          fail("asm_abort_req differs from flush or a fall of axis_enable");
        // Masks with their request.
        if (o_asm_abort_req && (o_asm_abort_mask !==
            ((i_active_sel && !i_tpg_dual_vc_en) ? {i_vc_exposure_map, ~i_vc_exposure_map}
                                                 : 2'b11)))
          fail("asm_abort_mask differs from the exposures in use");
        if (o_asm_skip_req && !o_capture_drop) fail("asm_skip_req without capture_drop");
        if (o_asm_skip_req && o_asm_skip_mask !== 2'b01) fail("asm_skip_mask not 01");
        if (o_asm_skip_req && !(o_drop_vc_mask inside {2'b11, onehot(i_lead_vc)}))
          fail("drop_vc_mask with asm_skip_req");
        if (o_capture_drop && !o_asm_skip_req && o_drop_vc_mask !== 2'b11)
          fail("drop_vc_mask not 11 for a drop");
        if (o_capture_drop && !o_asm_skip_req && (p_rst || !p_core))
          fail("capture_drop for a drop decided with core_enable low");
        // core_enable low: both flags set in the next cycle.
        if (p_valid && !p_core && o_force_discard !== 2'b11)
          fail("force_discard not 11 after a cycle with core_enable low");
        // One-cycle commands.
        if (p_valid && p_lb && o_lb_flush) fail("lb_flush in two cycles in a row");
        if (p_valid && p_rx && o_rx_ctx_clear) fail("rx_ctx_clear in two cycles in a row");
        if (p_valid && p_dab && o_dma_abort_req) fail("dma_abort_req in two cycles in a row");
        // frame_drop_pulse only at a capture_end, or one cycle after two capture_end.
        if (o_frame_drop_pulse && i_capture_end == 2'b00 && !(p_valid && !p_rst && p_end == 2'b11))
          fail("frame_drop_pulse without capture_end");

        stop_now = x_axis_q & ~i_axis_enable;
        x_stop_q = i_soft_reset ? 1'b0 : stop_now;
        x_axis_q = i_soft_reset ? 1'b0 : i_axis_enable;
        dab_hist = i_soft_reset ? 2'b00 : {dab_hist[0], o_dma_abort_req};
        p_valid  = 1'b1;
        p_rst    = i_soft_reset;
        p_lb     = o_lb_flush;
        p_rx     = o_rx_ctx_clear;
        p_dab    = o_dma_abort_req;
        p_core   = i_core_enable;
        p_end    = i_capture_end;
      end
    end
  end

  // --------------------------------------------------------------------------------------------
  // Test library
  // --------------------------------------------------------------------------------------------
  // Lead channel of the current input values. Tests use it instead of the lead_vc net, which
  // follows a change of active_sel or vc_exposure_map only after the current time step.
  function automatic logic lead_ch();
    return i_active_sel ? 1'b0 : i_vc_exposure_map;
  endfunction

  task automatic expect_cmd(input string what, input logic [6:0] want);
    settle();
    expect_eq({what, ": {capture_drop, dma_abort_req, lb_flush, asm_abort_req, asm_skip_req, ",
               "rx_ctx_clear, frame_drop_pulse}"}, cmds(), want);
  endtask

  task automatic expect_fd(input string what, input logic [1:0] want);
    settle();
    expect_eq({what, ": force_discard"}, o_force_discard, want);
  endtask

  task automatic expect_state(input string what, input st_t st);
    expect_true($sformatf("%s: model in %s, expected %s", what, m_st.name(), st.name()),
                m_st == st);
  endtask

  // n cycles with no command and force_discard at fd.
  task automatic quiet(input int n, input logic [1:0] fd, input string what);
    repeat (n) begin
      expect_cmd(what, C_NONE);
      expect_fd(what, fd);
      cycles(1);
    end
  endtask

  // resetn asserted between two edges, held for two edges, released between edges.
  task automatic do_resetn();
    #0.5;
    i_resetn = 1'b0;
    cycles(2);
    i_resetn = 1'b1;
  endtask

  // Frame Start of the lead channel; Frame Context & Metadata Capture opens the first free slot
  // when core_enable is high. Returns the slot, or -1.
  function automatic int lead_fs();
    int s;
    s       = -1;
    i_ev_fs = 1'b1;
    i_ev_vc = {1'b0, lead_ch()};
    if (i_core_enable && !i_soft_reset) begin
      if (!env_open[0])      s = 0;
      else if (!env_open[1]) s = 1;
      if (s >= 0) i_capture_start[s] = 1'b1;
    end
    return s;
  endfunction

  // Lead Frame Start that opens slot s, whatever the state of the slot.
  function automatic void lead_fs_slot(input int s);
    i_ev_fs            = 1'b1;
    i_ev_vc            = {1'b0, lead_ch()};
    i_capture_start[s] = 1'b1;
  endfunction

  // Frame Start of the lag channel.
  function automatic void lag_fs();
    i_ev_fs = 1'b1;
    i_ev_vc = {1'b0, ~lead_ch()};
  endfunction

  function automatic void cause(input int k, input logic vc = 1'b0);
    case (k)
      K_ECC2:  i_err_ecc_2bit = 1'b1;
      K_CRC:   begin i_err_crc = 1'b1; i_err_vc = {1'b0, vc}; end
      K_WC:    begin i_err_wc = 1'b1; i_err_vc = {1'b0, vc}; end
      K_LANE:  i_err_lane_abort = 1'b1;
      K_LBOVF: i_linebuf_overflow = 1'b1;
      default: i_fb_overflow = 1'b1;
    endcase
  endfunction

  function automatic string cause_name(input int k);
    case (k)
      K_ECC2:  return "err_ecc_2bit";
      K_CRC:   return "err_crc";
      K_WC:    return "err_wc";
      K_LANE:  return "err_lane_abort";
      K_LBOVF: return "linebuf_overflow";
      default: return "fb_overflow";
    endcase
  endfunction

  // From ST_IDLE or ST_SKIP to ST_RUN through a lead Frame Start that opens a capture; returns the
  // slot. The lead flag clears in the cycle after the resume. dma: dma_drop in the first ST_RUN
  // cycle (one cycle after capture_start). lag: then a lag Frame Start, so both flags are clear.
  // On return the test is in the cycle after the last step.
  task automatic to_run(output int slot, input bit dma = 1'b0, input bit lag = 1'b1);
    i_core_enable = 1'b1;
    if (m_st == S_IDLE) begin
      expect_fd("to ST_SKIP", 2'b11);
      cycles(1);
    end
    expect_state("before the resume", S_SKIP);
    slot = lead_fs();
    expect_true("lead Frame Start opens a capture", slot >= 0);
    expect_cmd("resume cycle", C_NONE);
    expect_fd("resume cycle", 2'b11);
    cycles(1);
    expect_fd("first ST_RUN cycle", onehot(~lead_ch()));
    if (dma) i_dma_drop = 1'b1;
    if (lag) begin
      lag_fs();
      cycles(1);
      expect_fd("after the lag Frame Start", 2'b00);
    end else if (dma) begin
      cycles(1);
    end
  endtask

  // From ST_IDLE after a reset to the first cycle of st, with a capture open and a DMA mark
  // pending in ST_RUN, ST_HOLD, ST_FLUSH and ST_SKIP.
  task automatic goto_state(input st_t st);
    int s;
    do_resetn();
    if (st == S_IDLE) begin
      i_core_enable = 1'b0;
      cycles(1);
      expect_state("goto", S_IDLE);
      return;
    end
    i_core_enable = 1'b1;
    cycles(1);
    to_run(s, 1'b1, 1'b1);
    if (st != S_RUN) begin
      cause(K_ECC2);
      cycles(1);
      if (st != S_HOLD) cycles(HOLD_CYCLES);
      if (st == S_SKIP) cycles(1);
    end
    expect_state("goto", st);
  endtask

  // From the cycle after a decision with a capture open or a mid-line cause: dma_abort_req (with
  // capture_drop when capdrop), ST_HOLD for HOLD_CYCLES cycles, one ST_FLUSH cycle, then no
  // command. Returns in the first cycle after ST_FLUSH.
  task automatic chk_abort_seq(input bit capdrop, input string what);
    expect_cmd({what, ", first ST_HOLD cycle"}, capdrop ? C_DROP_HOLD : C_HOLD);
    expect_fd({what, ", first ST_HOLD cycle"}, 2'b11);
    if (capdrop) expect_eq({what, ": drop_vc_mask"}, o_drop_vc_mask, 2'b11);
    cycles(1);
    for (int k = 1; k < HOLD_CYCLES; k++) begin
      expect_cmd({what, ", ST_HOLD"}, C_NONE);
      expect_fd({what, ", ST_HOLD"}, 2'b11);
      cycles(1);
    end
    expect_cmd({what, ", ST_FLUSH"}, i_active_sel ? C_FLUSH_T : C_FLUSH_S);
    expect_eq({what, ", ST_FLUSH: fb_flush"}, o_fb_flush, 1'b1);
    expect_fd({what, ", ST_FLUSH"}, 2'b11);
    cycles(1);
    expect_cmd({what, ", after ST_FLUSH"}, C_NONE);
    expect_fd({what, ", after ST_FLUSH"}, 2'b11);
  endtask

  // From the cycle after a packet-class decision with no capture open (ST_RUN to ST_SKIP):
  // capture_drop with mask 11 when capdrop, then no dma_abort_req and no flush.
  task automatic chk_skip_seq(input bit capdrop, input string what);
    expect_cmd({what, ", decision + 1"}, capdrop ? C_DROP : C_NONE);
    expect_fd({what, ", decision + 1"}, 2'b11);
    if (capdrop) expect_eq({what, ": drop_vc_mask"}, o_drop_vc_mask, 2'b11);
    cycles(1);
    quiet(HOLD_CYCLES + 1, 2'b11, {what, ", no flush"});
  endtask

  // capture_end of slot s once it may end; frame_drop_pulse must be exp in that cycle.
  task automatic end_chk(input int s, input bit exp, input string what = "capture_end");
    int n;
    n = 0;
    while (!(env_open[s] && env_age[s] >= 1) && n < 4) begin
      cycles(1);
      n++;
    end
    if (!env_open[s]) begin
      fail($sformatf("%s: slot %0d is not open", what, s));
      return;
    end
    i_capture_end[s] = 1'b1;
    settle();
    expect_eq($sformatf("%s of slot %0d: frame_drop_pulse", what, s), o_frame_drop_pulse, exp);
    cycles(1);
  endtask

  // Ends every open capture, one per cycle; the expected pulse follows the marks of the
  // connected blocks.
  task automatic close_all();
    for (int s = 0; s < 2; s++) begin
      if (env_open[s]) begin
        int n;
        n = 0;
        while (env_age[s] < 1 && n < 4) begin
          cycles(1);
          n++;
        end
        i_capture_end[s] = 1'b1;
        settle();
        expect_eq($sformatf("close of slot %0d: frame_drop_pulse", s), o_frame_drop_pulse,
                  env_dropped_now[s] | env_dma[s]);
        cycles(1);
      end
    end
  endtask

  // Sets every pulse cause of the sensor and the generator path at once.
  function automatic void all_causes();
    for (int k = K_ECC2; k <= K_FBOVF; k++) cause(k, lead_ch());
  endfunction

  // --------------------------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------------------------
  // Port widths, values while resetn is low, and resetn asserted in every state with captures
  // open, a DMA mark pending and lead_seen set.
  task automatic test_reset_values();
    int   s;
    st_t  sts [5];
    sts = '{S_IDLE, S_SKIP, S_RUN, S_HOLD, S_FLUSH};
    where = "port widths";
    expect_eq("width of err_vc", $bits(u_dut.i_err_vc), 2);
    expect_eq("width of force_discard", $bits(u_dut.o_force_discard), 2);
    expect_eq("width of capture_start", $bits(u_dut.i_capture_start), 2);
    expect_eq("width of capture_end", $bits(u_dut.i_capture_end), 2);
    expect_eq("width of capture_dropped", $bits(u_dut.i_capture_dropped), 2);
    expect_eq("width of ev_vc", $bits(u_dut.i_ev_vc), 2);
    expect_eq("width of drop_vc_mask", $bits(u_dut.o_drop_vc_mask), 2);
    expect_eq("width of asm_abort_mask", $bits(u_dut.o_asm_abort_mask), 2);
    expect_eq("width of asm_skip_mask", $bits(u_dut.o_asm_skip_mask), 2);
    expect_eq("width of one-bit ports",
              $bits({u_dut.i_sys_clk, u_dut.i_resetn, u_dut.i_soft_reset, u_dut.i_core_enable,
                     u_dut.i_axis_enable, u_dut.i_stream_follow_dma_drop,
                     u_dut.i_vc_exposure_map, u_dut.i_tpg_dual_vc_en, u_dut.i_err_ecc_2bit,
                     u_dut.i_err_lane_abort, u_dut.i_err_crc, u_dut.i_err_wc,
                     u_dut.i_active_sel, u_dut.o_rx_ctx_clear, u_dut.i_linebuffer_full,
                     u_dut.i_linebuf_overflow, u_dut.o_lb_flush, u_dut.i_fb_overflow,
                     u_dut.o_fb_flush, u_dut.i_ev_fs, u_dut.i_lead_vc, u_dut.o_capture_drop,
                     u_dut.o_asm_abort_req, u_dut.o_asm_skip_req, u_dut.i_dma_drop,
                     u_dut.o_dma_abort_req, u_dut.o_frame_drop_pulse}), 27);

    where = "while resetn is low";
    #1.0;
    i_resetn = 1'b0;
    #0.5;
    expect_fd("resetn low, between edges", 2'b11);
    expect_cmd("resetn low, between edges", C_NONE);
    cycles(2);
    expect_fd("resetn low", 2'b11);
    expect_cmd("resetn low", C_NONE);
    i_resetn = 1'b1;
    quiet(4, 2'b11, "after resetn, no lead Frame Start");

    foreach (sts[k]) begin
      where = $sformatf("resetn in %s", sts[k].name());
      goto_state(sts[k]);
      if (sts[k] == S_HOLD) begin
        s = lead_fs();
        expect_true("capture opened in ST_HOLD", s >= 0);
        cycles(1);
      end
      #1.0;
      i_resetn = 1'b0;
      #0.5;
      expect_fd("resetn low, before the next edge", 2'b11);
      expect_cmd("resetn low, before the next edge", C_NONE);
      cycles(2);
      i_resetn      = 1'b1;
      i_core_enable = 1'b1;
      quiet(4, 2'b11, "no resume after resetn");
      // The DMA mark of slot 0 and the open flags were cleared: a late capture_end is not
      // counted.
      relax_end_age    = 1'b1;
      i_capture_end[0] = 1'b1;
      expect_cmd("capture_end after resetn", C_NONE);
      cycles(1);
      relax_end_age    = 1'b0;
      to_run(s);
      close_all();
    end
  endtask

  // soft_reset in every state, and in the cycle of a cause, a dma_drop, a lag Frame Start and a
  // lead capture_start.
  task automatic test_soft_reset();
    int   s;
    st_t  sts [5];
    sts = '{S_IDLE, S_SKIP, S_RUN, S_HOLD, S_FLUSH};
    foreach (sts[k]) begin
      where = $sformatf("soft_reset in %s", sts[k].name());
      goto_state(sts[k]);
      if (sts[k] == S_HOLD) begin
        s = lead_fs();
        cycles(1);
      end
      i_soft_reset = 1'b1;
      cycles(1);
      i_soft_reset  = 1'b0;
      i_core_enable = 1'b1;
      expect_cmd("first cycle after soft_reset", C_NONE);
      expect_fd("first cycle after soft_reset", 2'b11);
      cycles(1);
      quiet(3, 2'b11, "no resume after soft_reset");
      relax_end_age    = 1'b1;
      i_capture_end[0] = 1'b1;
      expect_cmd("capture_end after soft_reset", C_NONE);
      cycles(1);
      relax_end_age    = 1'b0;
      to_run(s);
      close_all();
    end

    where = "soft_reset with a cause";
    do_resetn();
    to_run(s);
    cause(K_ECC2);
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset = 1'b0;
    quiet(4, 2'b11, "no command after the cause");

    where = "soft_reset with a dma_drop";
    i_stream_follow_dma_drop = 1'b1;
    cycles(1);
    s = lead_fs();
    cycles(1);
    expect_state("before the dma_drop", S_RUN);
    i_dma_drop   = 1'b1;
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset = 1'b0;
    quiet(4, 2'b11, "no skip after the dma_drop");
    i_stream_follow_dma_drop = 1'b0;

    where = "soft_reset with a lag Frame Start";
    to_run(s, 1'b0, 1'b0);
    lag_fs();
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset = 1'b0;
    quiet(3, 2'b11, "flags set after soft_reset");

    where = "soft_reset with a lead capture_start";
    expect_state("before the lead capture_start", S_SKIP);
    relax_soft_ev = 1'b1;
    lead_fs_slot(0);
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset  = 1'b0;
    relax_soft_ev = 1'b0;
    quiet(4, 2'b11, "no resume on a capture_start in the soft_reset cycle");
    to_run(s);
    close_all();
  endtask

  // Each packet-class cause with one and two captures open: abort through ST_HOLD and ST_FLUSH,
  // capture_drop with mask 11, each capture counted at its end.
  task automatic test_packet_causes();
    int s0, s1;
    for (int k = K_ECC2; k <= K_LANE; k++) begin
      for (int n = 1; n <= 2; n++) begin
        where = $sformatf("%s with %0d capture(s) open", cause_name(k), n);
        do_resetn();
        to_run(s0);
        if (n == 2) begin
          s1 = lead_fs();
          expect_eq("second capture slot", s1, 1);
          cycles(1);
        end
        cause(k, lead_ch());
        expect_cmd("decision cycle", C_NONE);
        cycles(1);
        chk_abort_seq(1'b1, cause_name(k));
        end_chk(s0, 1'b1);
        if (n == 2) end_chk(1, 1'b1);
      end
    end
  endtask

  // Packet-class cause with no capture open: ST_SKIP without flush, the orphan lag frame stays
  // suppressed, nothing is counted.
  task automatic test_packet_cause_no_capture();
    int s, n0;
    for (int map = 0; map < 2; map++) begin
      where = $sformatf("lead_vc %0d", map);
      do_resetn();
      i_vc_exposure_map = 1'(map);
      to_run(s);
      end_chk(s, 1'b0);
      n0 = n_fdp;
      cause(K_ECC2);
      cycles(1);
      chk_skip_seq(1'b1, "packet cause, no capture");
      lag_fs();
      expect_cmd("orphan lag Frame Start", C_NONE);
      cycles(1);
      quiet(2, 2'b11, "orphan lag frame suppressed");
      s = lead_fs();
      cycles(1);
      expect_fd("resume", onehot(~lead_ch()));
      expect_eq("frame_drop_pulse count", n_fdp - n0, 0);
      close_all();
      i_vc_exposure_map = 1'b0;
    end
  endtask

  // Each mid-line cause with and without a capture open: always through ST_HOLD and ST_FLUSH;
  // fb_overflow also on the Test Pattern Generator path.
  task automatic test_midline_causes();
    int s;
    string name;
    for (int k = 0; k < 3; k++) begin
      for (int cap = 0; cap < 2; cap++) begin
        if (k == 0) name = "linebuffer_full";
        else        name = cause_name(K_LBOVF + k - 1);
        where = $sformatf("%s, capture open %0d", name, cap);
        do_resetn();
        to_run(s);
        if (cap == 0) end_chk(s, 1'b0);
        if (k == 0) i_linebuffer_full = 1'b1;
        else        cause(K_LBOVF + k - 1);
        cycles(1);
        i_linebuffer_full = 1'b0;
        chk_abort_seq(1'b1, where);
        if (cap == 1) end_chk(s, 1'b1);
      end
    end
    where = "fb_overflow on the Test Pattern Generator path";
    do_resetn();
    i_active_sel = 1'b1;
    to_run(s);
    cause(K_FBOVF);
    cycles(1);
    chk_abort_seq(1'b1, where);
    end_chk(s, 1'b1);
    i_active_sel = 1'b0;
  endtask

  // active_sel = 1 masks the sensor causes; err_crc and err_wc on a discarded channel are
  // ignored; err_ecc_2bit and err_lane_abort are taken while a channel is discarded.
  task automatic test_cause_qualifiers();
    int s;
    where = "sensor causes with active_sel = 1";
    do_resetn();
    i_active_sel = 1'b1;
    to_run(s);
    for (int k = K_ECC2; k <= K_LBOVF; k++) begin
      cause(k, 1'b0);
      quiet(2, 2'b00, cause_name(k));
    end
    i_linebuffer_full = 1'b1;
    quiet(3, 2'b00, "linebuffer_full");
    i_linebuffer_full = 1'b0;
    close_all();
    expect_cmd("after the sensor causes", C_NONE);
    cycles(1);
    // Lead channel resumed, lag channel still discarded.
    i_active_sel = 1'b0;
    for (int k = 0; k < 5; k++) begin
      do_resetn();
      to_run(s, 1'b0, 1'b0);
      case (k)
        0: begin where = "err_crc on the discarded lag channel"; cause(K_CRC, ~lead_ch()); end
        1: begin where = "err_wc on the discarded lag channel"; cause(K_WC, ~lead_ch()); end
        2: begin where = "err_ecc_2bit, lag channel discarded"; cause(K_ECC2); end
        3: begin where = "err_lane_abort, lag channel discarded"; cause(K_LANE); end
        default: begin where = "err_crc on the lead channel"; cause(K_CRC, lead_ch()); end
      endcase
      if (k < 2) begin
        quiet(3, onehot(~lead_ch()), "ignored");
        close_all();
      end else begin
        cycles(1);
        chk_abort_seq(1'b1, where);
        end_chk(s, 1'b1);
      end
    end
  endtask

  // core_enable falling with 0, 1 and 2 captures open, low in ST_HOLD, and re-enable.
  task automatic test_core_enable();
    int s;
    where = "core_enable low, no capture";
    do_resetn();
    to_run(s);
    end_chk(s, 1'b0);
    i_core_enable = 1'b0;
    expect_cmd("cycle of the fall", C_NONE);
    cycles(1);
    quiet(5, 2'b11, "ST_IDLE, no capture_drop");
    expect_state("core_enable low", S_IDLE);
    i_core_enable = 1'b1;
    cycles(1);
    quiet(2, 2'b11, "ST_SKIP");
    to_run(s);
    close_all();

    for (int n = 1; n <= 2; n++) begin
      where = $sformatf("core_enable low with %0d capture(s) open", n);
      do_resetn();
      to_run(s);
      if (n == 2) begin
        s = lead_fs();
        cycles(1);
      end
      i_core_enable = 1'b0;
      cycles(1);
      chk_abort_seq(1'b0, where);
      expect_state("after ST_FLUSH with core_enable low", S_IDLE);
      end_chk(0, 1'b1, "capture ended while core_enable is low");
      if (n == 2) end_chk(1, 1'b1, "capture ended while core_enable is low");
      quiet(2, 2'b11, "ST_IDLE");
      i_core_enable = 1'b1;
      cycles(1);
      to_run(s);
      close_all();
    end

    where = "core_enable low in ST_HOLD";
    do_resetn();
    to_run(s);
    cause(K_ECC2);
    cycles(1);
    i_core_enable = 1'b0;
    chk_abort_seq(1'b1, where);
    expect_state("after ST_FLUSH with core_enable low", S_IDLE);
    quiet(2, 2'b11, "ST_IDLE");
    end_chk(s, 1'b1);
    i_core_enable = 1'b1;
    cycles(1);
    to_run(s);
    close_all();
  endtask

  // dma_drop with stream_follow_dma_drop = 1 in ST_RUN: capture_drop on the lead channel with
  // asm_skip_req, no abort, no discard.
  task automatic test_dma_drop_follow();
    int s;
    for (int mode = 0; mode < 3; mode++) begin
      if (mode == 2) where = "DMA skip, Test Pattern Generator";
      else           where = $sformatf("DMA skip, lead_vc %0d", mode);
      do_resetn();
      i_stream_follow_dma_drop = 1'b1;
      i_vc_exposure_map        = (mode == 1);
      i_active_sel             = (mode == 2);
      to_run(s, 1'b1, 1'b1);
      expect_cmd("skip", C_SKIP);
      expect_eq("drop_vc_mask", o_drop_vc_mask, onehot(lead_ch()));
      expect_eq("asm_skip_mask", o_asm_skip_mask, 2'b01);
      expect_fd("skip", 2'b00);
      cycles(1);
      quiet(3, 2'b00, "no abort after the skip");
      s = lead_fs();
      cycles(1);
      i_dma_drop = 1'b1;
      cycles(1);
      expect_cmd("second skip", C_SKIP);
      expect_eq("drop_vc_mask", o_drop_vc_mask, onehot(lead_ch()));
      cycles(1);
      quiet(2, 2'b00, "no abort after the second skip");
      end_chk(0, 1'b1);
      end_chk(1, 1'b1);
      i_stream_follow_dma_drop = 1'b0;
      i_vc_exposure_map        = 1'b0;
      i_active_sel             = 1'b0;
    end
  endtask

  // dma_drop with stream_follow_dma_drop = 0: no command, the most recent slot is counted at its
  // end, the mark is cleared at the next capture_start of that slot; dma_drop outside ST_RUN.
  task automatic test_dma_drop_no_follow();
    int s0, s1;
    where = "DMA drop, follow 0";
    do_resetn();
    to_run(s0, 1'b1, 1'b1);
    quiet(2, 2'b00, "no command for the DMA drop");
    s1 = lead_fs();
    cycles(1);
    i_dma_drop = 1'b1;
    quiet(2, 2'b00, "no command for the second DMA drop");
    end_chk(0, 1'b1, "first DMA-dropped capture");
    end_chk(1, 1'b1, "second DMA-dropped capture");
    s0 = lead_fs();
    expect_eq("reused slot", s0, 0);
    cycles(2);
    end_chk(0, 1'b0, "new capture in a slot that had a DMA mark");

    where = "DMA drop, follow 0, Test Pattern Generator";
    do_resetn();
    i_active_sel = 1'b1;
    to_run(s0, 1'b1, 1'b1);
    quiet(2, 2'b00, "no command");
    end_chk(s0, 1'b1);
    i_active_sel = 1'b0;

    where = "DMA drop outside ST_RUN, follow 1";
    do_resetn();
    i_stream_follow_dma_drop = 1'b1;
    to_run(s0);
    cause(K_ECC2);
    cycles(1);
    expect_cmd("first ST_HOLD cycle", C_DROP_HOLD);
    s1 = lead_fs();
    cycles(1);
    i_dma_drop = 1'b1;
    expect_cmd("dma_drop in ST_HOLD", C_NONE);
    cycles(1);
    expect_cmd("ST_FLUSH, no skip", C_FLUSH_S);
    cycles(1);
    expect_cmd("ST_SKIP", C_NONE);
    cycles(1);
    expect_cmd("resume through the capture opened in ST_HOLD", C_NONE);
    expect_fd("resume through the capture opened in ST_HOLD", onehot(~lead_ch()));
    end_chk(s0, 1'b1, "dropped capture");
    end_chk(s1, 1'b1, "capture of the DMA drop outside ST_RUN");
    i_stream_follow_dma_drop = 1'b0;
  endtask

  // Abort cause and DMA skip in the same ST_RUN cycle.
  task automatic test_abort_and_skip();
    int s0, s1;
    where = "abort and skip";
    do_resetn();
    i_stream_follow_dma_drop = 1'b1;
    to_run(s0);
    s1 = lead_fs();
    cycles(1);
    i_dma_drop = 1'b1;
    cause(K_ECC2);
    cycles(1);
    expect_cmd("first ST_HOLD cycle", 7'b110_0100);
    expect_eq("drop_vc_mask", o_drop_vc_mask, 2'b11);
    expect_eq("asm_skip_mask", o_asm_skip_mask, 2'b01);
    expect_fd("first ST_HOLD cycle", 2'b11);
    cycles(HOLD_CYCLES);
    expect_cmd("ST_FLUSH", C_FLUSH_S);
    cycles(1);
    quiet(2, 2'b11, "ST_SKIP");
    end_chk(s0, 1'b1);
    end_chk(s1, 1'b1);
    i_stream_follow_dma_drop = 1'b0;
  endtask

  // Every cause class in ST_IDLE, ST_SKIP, ST_HOLD and ST_FLUSH: no capture_drop, no further
  // dma_abort_req or flush.
  task automatic test_causes_outside_run();
    int s0, s1;
    where = "causes in ST_IDLE";
    do_resetn();
    i_core_enable = 1'b0;
    i_linebuffer_full = 1'b1;
    repeat (3) begin
      all_causes();
      expect_cmd("ST_IDLE", C_NONE);
      cycles(1);
    end
    quiet(2, 2'b11, "after the causes in ST_IDLE");
    i_core_enable = 1'b1;
    cycles(1);
    where = "causes in ST_SKIP";
    expect_state("causes in ST_SKIP", S_SKIP);
    repeat (3) begin
      all_causes();
      expect_cmd("ST_SKIP", C_NONE);
      cycles(1);
    end
    i_linebuffer_full = 1'b0;
    quiet(2, 2'b11, "after the causes in ST_SKIP");

    where = "causes in ST_HOLD and ST_FLUSH";
    i_stream_follow_dma_drop = 1'b1;
    to_run(s0);
    cause(K_ECC2);
    cycles(1);
    i_linebuffer_full = 1'b1;
    all_causes();
    s1 = lead_fs();
    expect_cmd("first ST_HOLD cycle", C_DROP_HOLD);
    cycles(1);
    all_causes();
    i_dma_drop = 1'b1;
    expect_cmd("second ST_HOLD cycle", C_NONE);
    cycles(1);
    all_causes();
    expect_cmd("ST_FLUSH", C_FLUSH_S);
    cycles(1);
    i_linebuffer_full = 1'b0;
    expect_cmd("ST_SKIP", C_NONE);
    cycles(1);
    expect_cmd("first ST_RUN cycle", C_NONE);
    expect_fd("first ST_RUN cycle", onehot(~lead_ch()));
    cycles(1);
    quiet(2, onehot(~lead_ch()), "no late command");
    end_chk(s0, 1'b1);
    end_chk(s1, 1'b1);
    i_stream_follow_dma_drop = 1'b0;
  endtask

  // ST_HOLD length, dma_abort_req in its first cycle, the ST_FLUSH command set on both sources,
  // and a fall of axis_enable before and in the ST_FLUSH cycle.
  task automatic test_hold_and_flush();
    int s;
    for (int asel = 0; asel < 2; asel++) begin
      where = $sformatf("flush with active_sel %0d", asel);
      do_resetn();
      i_active_sel = 1'(asel);
      to_run(s);
      cause(K_FBOVF);
      cycles(1);
      chk_abort_seq(1'b1, where);
      expect_state("after ST_FLUSH with core_enable high", S_SKIP);
      end_chk(s, 1'b1);
    end
    i_active_sel = 1'b0;

    where = "axis_enable falls in the last ST_HOLD cycle";
    do_resetn();
    to_run(s);
    cause(K_ECC2);
    cycles(HOLD_CYCLES);
    i_axis_enable = 1'b0;
    expect_cmd("last ST_HOLD cycle", C_NONE);
    cycles(1);
    expect_cmd("ST_FLUSH, one asm_abort_req for both", C_FLUSH_S);
    cycles(1);
    expect_cmd("after ST_FLUSH", C_NONE);
    i_axis_enable = 1'b1;
    cycles(1);
    end_chk(s, 1'b1);

    where = "axis_enable falls in the ST_FLUSH cycle";
    to_run(s);
    cause(K_ECC2);
    cycles(HOLD_CYCLES + 1);
    i_axis_enable = 1'b0;
    expect_cmd("ST_FLUSH", C_FLUSH_S);
    cycles(1);
    expect_cmd("after ST_FLUSH", C_STOP);
    i_axis_enable = 1'b1;
    cycles(1);
    expect_cmd("no third pulse", C_NONE);
    end_chk(s, 1'b1);
  endtask

  // ST_SKIP exits: lead capture_start on either slot, lead_seen from ST_HOLD and ST_FLUSH,
  // priority of core_enable low, lead_seen cleared through ST_IDLE.
  task automatic test_skip_exit();
    int s0, s1;
    for (int map = 0; map < 2; map++) begin
      where = $sformatf("resume on slot 1, lead_vc %0d", map);
      do_resetn();
      i_vc_exposure_map = 1'(map);
      to_run(s0);
      cause(K_ECC2);
      cycles(1);
      chk_abort_seq(1'b1, where);
      cycles(1);
      quiet(2, 2'b11, "ST_SKIP, slot 0 still open");
      s1 = lead_fs();
      expect_eq("slot of the resume", s1, 1);
      cycles(1);
      expect_fd("resume on slot 1", onehot(~lead_ch()));
      end_chk(0, 1'b1);
      end_chk(1, 1'b0);

      for (int at_flush = 0; at_flush < 2; at_flush++) begin
        where = $sformatf("lead_seen from %s, lead_vc %0d", at_flush ? "ST_FLUSH" : "ST_HOLD",
                          map);
        do_resetn();
        to_run(s0);
        cause(K_ECC2);
        cycles(1);
        if (at_flush) cycles(HOLD_CYCLES);
        s1 = lead_fs();
        expect_true("capture opened", s1 == 1);
        if (!at_flush) cycles(HOLD_CYCLES);
        expect_cmd("ST_FLUSH", C_FLUSH_S);
        cycles(1);
        expect_cmd("ST_SKIP", C_NONE);
        expect_fd("ST_SKIP", 2'b11);
        cycles(1);
        expect_fd("resume on entry into ST_SKIP", onehot(~lead_ch()));
        expect_cmd("resume on entry into ST_SKIP", C_NONE);
        end_chk(s0, 1'b1);
        end_chk(s1, 1'b0, "capture opened after the drop decision");
      end
    end
    i_vc_exposure_map = 1'b0;

    where = "core_enable low with a lead capture_start in ST_SKIP";
    do_resetn();
    cycles(1);
    expect_state("ST_SKIP", S_SKIP);
    i_core_enable = 1'b0;
    lead_fs_slot(0);
    cycles(1);
    expect_state("ST_IDLE taken first", S_IDLE);
    quiet(2, 2'b11, "no resume");
    i_core_enable = 1'b1;
    cycles(1);
    quiet(3, 2'b11, "no resume after core_enable returns");
    close_all();

    where = "lead_seen cleared through ST_IDLE";
    do_resetn();
    to_run(s0);
    cause(K_ECC2);
    cycles(1);
    s1 = lead_fs();
    cycles(1);
    i_core_enable = 1'b0;
    cycles(1);
    expect_cmd("ST_FLUSH", C_FLUSH_S);
    cycles(1);
    expect_state("after ST_FLUSH with core_enable low", S_IDLE);
    quiet(2, 2'b11, "ST_IDLE");
    i_core_enable = 1'b1;
    cycles(1);
    quiet(4, 2'b11, "no resume through the old lead_seen");
    close_all();
    to_run(s0);
    close_all();
  endtask

  // Discard flags: set points, lead clear at the resume, lag clear at the first lag Frame Start
  // with the lead flag clear, for both lead channels.
  task automatic test_discard_flags();
    int s0, s1;
    for (int map = 0; map < 2; map++) begin
      where = $sformatf("discard flags, lead_vc %0d", map);
      do_resetn();
      i_vc_exposure_map = 1'(map);
      cycles(1);
      lag_fs();
      cycles(1);
      expect_fd("lag Frame Start in ST_SKIP", 2'b11);
      to_run(s0, 1'b0, 1'b0);
      expect_fd("lead flag clear, lag flag set", onehot(~lead_ch()));
      cycles(2);
      s1 = lead_fs();
      cycles(1);
      expect_fd("lead Frame Start does not clear the lag flag", onehot(~lead_ch()));
      lag_fs();
      cycles(1);
      expect_fd("lag Frame Start clears the lag flag", 2'b00);
      cause(K_ECC2);
      expect_fd("decision cycle", 2'b00);
      cycles(1);
      chk_abort_seq(1'b1, "drop");
      cycles(1);
      lag_fs();
      cycles(1);
      expect_fd("lag Frame Start in ST_SKIP", 2'b11);
      end_chk(s0, 1'b1);
      end_chk(s1, 1'b1);
      s0 = lead_fs();
      cycles(1);
      expect_fd("resume", onehot(~lead_ch()));
      lag_fs();
      cycles(1);
      expect_fd("lag Frame Start after the resume", 2'b00);
      close_all();
    end
    i_vc_exposure_map = 1'b0;
  endtask

  // axis_enable falling in each state: asm_abort_req one cycle after the fall only; nothing on
  // the rising edge or after a reset with axis_enable high.
  task automatic axis_fall(input logic [1:0] fd, input string what);
    i_axis_enable = 1'b0;
    expect_cmd({what, ", cycle of the fall"}, C_NONE);
    cycles(1);
    expect_cmd({what, ", one cycle after the fall"}, C_STOP);
    expect_fd(what, fd);
    cycles(1);
    expect_cmd({what, ", two cycles after the fall"}, C_NONE);
    i_axis_enable = 1'b1;
    cycles(1);
    quiet(2, fd, {what, ", rising edge"});
  endtask

  task automatic test_axis_enable_fall();
    int s0, s1;
    where = "axis_enable after resetn";
    do_resetn();
    i_core_enable = 1'b0;
    quiet(3, 2'b11, "axis_enable high after resetn");
    where = "axis_enable in ST_IDLE";
    axis_fall(2'b11, where);
    i_core_enable = 1'b1;
    cycles(1);
    where = "axis_enable in ST_SKIP";
    axis_fall(2'b11, where);
    where = "axis_enable in ST_RUN with two captures open";
    to_run(s0);
    s1 = lead_fs();
    cycles(1);
    axis_fall(2'b00, where);
    where = "axis_enable in ST_HOLD";
    cause(K_ECC2);
    cycles(1);
    i_axis_enable = 1'b0;
    expect_cmd("first ST_HOLD cycle", C_DROP_HOLD);
    cycles(1);
    expect_cmd("second ST_HOLD cycle", C_STOP);
    i_axis_enable = 1'b1;
    cycles(1);
    expect_cmd("ST_FLUSH", C_FLUSH_S);
    cycles(1);
    expect_cmd("after ST_FLUSH", C_NONE);
    end_chk(s0, 1'b1);
    end_chk(s1, 1'b1);
  endtask

  // asm_abort_mask on the sensor path, in dual-channel and in single-channel Test Pattern
  // Generator mode, on a fall of axis_enable and on a flush.
  task automatic test_abort_mask();
    int s;
    logic [1:0] want;
    for (int mode = 0; mode < 5; mode++) begin
      do_resetn();
      case (mode)
        0: begin i_active_sel = 1'b0; i_vc_exposure_map = 1'b0; want = 2'b11; end
        1: begin i_active_sel = 1'b0; i_vc_exposure_map = 1'b1; want = 2'b11; end
        2: begin i_active_sel = 1'b1; i_tpg_dual_vc_en = 1'b1; want = 2'b11; end
        3: begin i_active_sel = 1'b1; i_tpg_dual_vc_en = 1'b0; i_vc_exposure_map = 1'b0;
                 want = 2'b01; end
        default: begin i_active_sel = 1'b1; i_tpg_dual_vc_en = 1'b0; i_vc_exposure_map = 1'b1;
                       want = 2'b10; end
      endcase
      where = $sformatf("abort mask, active_sel %b tpg_dual_vc_en %b vc_exposure_map %b",
                        i_active_sel, i_tpg_dual_vc_en, i_vc_exposure_map);
      cycles(1);
      i_axis_enable = 1'b0;
      cycles(1);
      expect_eq("asm_abort_req on the fall", o_asm_abort_req, 1'b1);
      expect_eq("asm_abort_mask on the fall", o_asm_abort_mask, want);
      i_axis_enable = 1'b1;
      cycles(1);
      to_run(s);
      cause(K_FBOVF);
      cycles(HOLD_CYCLES + 1);
      settle();
      expect_eq("lb_flush", o_lb_flush, 1'b1);
      expect_eq("asm_abort_mask on the flush", o_asm_abort_mask, want);
      cycles(1);
      end_chk(s, 1'b1);
    end
    i_active_sel      = 1'b0;
    i_tpg_dual_vc_en  = 1'b1;
    i_vc_exposure_map = 1'b0;
  endtask

  // Capture Watch: slot 1 alone keeps the block busy, a slot that ends and starts in one cycle
  // stays open, a capture_start in the cycle of a cause does not make the block busy.
  task automatic test_capture_watch();
    int s0, s1;
    where = "slot 1 open alone";
    do_resetn();
    to_run(s0);
    s1 = lead_fs();
    cycles(1);
    end_chk(0, 1'b0);
    cause(K_ECC2);
    cycles(1);
    chk_abort_seq(1'b1, where);
    end_chk(1, 1'b1);

    where = "no capture open after the ends";
    to_run(s0);
    end_chk(s0, 1'b0);
    cause(K_ECC2);
    cycles(1);
    chk_skip_seq(1'b1, where);

    where = "end and start of slot 0 in one cycle";
    do_resetn();
    to_run(s0);
    cycles(2);
    i_capture_end[0] = 1'b1;
    lead_fs_slot(0);
    expect_cmd("end of the old capture", C_NONE);
    cycles(1);
    cause(K_ECC2);
    cycles(1);
    chk_abort_seq(1'b1, "slot 0 still open");
    end_chk(0, 1'b1);

    where = "cause in the cycle of the first capture_start";
    do_resetn();
    to_run(s0);
    end_chk(s0, 1'b0);
    s0 = lead_fs();
    cause(K_ECC2);
    cycles(1);
    chk_skip_seq(1'b1, where);
    end_chk(s0, 1'b1, "capture marked by capture_drop");
  endtask

  // Drop Report: capture_end with capture_dropped, with a DMA mark and clean; two dropped ends in
  // consecutive cycles; a slot reused after a DMA mark.
  task automatic test_drop_report();
    int s0, s1;
    where = "drop report";
    do_resetn();
    to_run(s0);
    s1 = lead_fs();
    cycles(1);
    cause(K_ECC2);
    cycles(1);
    chk_abort_seq(1'b1, "drop of both slots");
    end_chk(0, 1'b1, "first dropped end");
    i_capture_end[1] = 1'b1;
    expect_cmd("second dropped end in the next cycle", C_PULSE);
    cycles(1);
    expect_cmd("no pulse after the two ends", C_NONE);
    to_run(s0, 1'b1, 1'b1);
    end_chk(s0, 1'b1, "capture with a DMA mark");
    s0 = lead_fs();
    cycles(2);
    end_chk(s0, 1'b0, "clean capture");
    s0 = lead_fs();
    cycles(1);
    i_dma_drop = 1'b1;
    cycles(1);
    end_chk(s0, 1'b1, "capture with a DMA mark");
    s1 = lead_fs();
    expect_eq("slot reused", s1, s0);
    cycles(2);
    end_chk(s1, 1'b0, "new capture after the DMA mark");
  endtask

  // Error on S(N) while L(N+1) is open: both dropped, reception resumes at L(N+2) on the lead
  // channel and at the following Frame Start on the lag channel.
  task automatic test_error_on_overlap();
    int sn, sn1, sn2;
    where = "error on S(N) with L(N+1) open";
    do_resetn();
    to_run(sn);
    cycles(3);
    sn1 = lead_fs();
    cycles(2);
    cause(K_CRC, ~lead_ch());
    cycles(1);
    chk_abort_seq(1'b1, "error on S(N)");
    end_chk(sn, 1'b1, "capture N");
    lag_fs();
    cycles(1);
    expect_fd("S(N+1) Frame Start", 2'b11);
    cycles(2);
    sn2 = lead_fs();
    expect_eq("slot of L(N+2)", sn2, sn);
    cycles(1);
    expect_fd("resume at L(N+2), tail of S(N+1) suppressed", onehot(~lead_ch()));
    end_chk(sn1, 1'b1, "capture N+1");
    expect_fd("lag channel still discarded", onehot(~lead_ch()));
    lag_fs();
    cycles(1);
    expect_fd("S(N+2) Frame Start", 2'b00);
    end_chk(sn2, 1'b0, "capture N+2");
  endtask

  // Test Pattern Generator selected: sensor causes ignored, fb_overflow, core_enable and dma_drop
  // act, the flush has no rx_ctx_clear.
  task automatic test_tpg_selected();
    int s;
    where = "Test Pattern Generator";
    do_resetn();
    i_active_sel = 1'b1;
    to_run(s);
    cause(K_ECC2);
    cause(K_CRC, 1'b0);
    cause(K_LANE);
    cause(K_LBOVF);
    i_linebuffer_full = 1'b1;
    cycles(1);
    i_linebuffer_full = 1'b0;
    quiet(2, 2'b00, "sensor causes ignored");
    cause(K_FBOVF);
    cycles(1);
    chk_abort_seq(1'b1, "fb_overflow");
    end_chk(s, 1'b1);
    to_run(s);
    i_core_enable = 1'b0;
    cycles(1);
    chk_abort_seq(1'b0, "core_enable low");
    end_chk(s, 1'b1);
    i_core_enable            = 1'b1;
    i_stream_follow_dma_drop = 1'b1;
    cycles(1);
    to_run(s, 1'b1, 1'b1);
    expect_cmd("DMA skip", C_SKIP);
    expect_eq("drop_vc_mask", o_drop_vc_mask, 2'b01);
    cycles(1);
    quiet(2, 2'b00, "after the skip");
    end_chk(s, 1'b1);
    i_stream_follow_dma_drop = 1'b0;
    i_active_sel             = 1'b0;
  endtask

  // Frame that is never closed: core_enable cleared drops the open captures with rx_ctx_clear;
  // with no capture open core_enable low gives no rx_ctx_clear.
  task automatic test_frame_never_closed();
    int s0, s1;
    where = "frame never closed";
    do_resetn();
    to_run(s0);
    s1 = lead_fs();
    cycles(20);
    quiet(5, 2'b00, "no timeout");
    i_core_enable = 1'b0;
    cycles(1);
    chk_abort_seq(1'b0, "core_enable cleared");
    end_chk(s0, 1'b1);
    end_chk(s1, 1'b1);
    where = "core_enable low with no capture open";
    i_core_enable = 1'b1;
    cycles(1);
    to_run(s0);
    end_chk(s0, 1'b0);
    i_core_enable = 1'b0;
    cycles(1);
    quiet(4, 2'b11, "no rx_ctx_clear");
    i_core_enable = 1'b1;
  endtask

  // Two dropped captures ending in the same cycle: two pulses in consecutive cycles; a third
  // dropped end in the pending cycle merges into one pulse.
  task automatic test_two_ends_same_cycle();
    int s0, s1;
    where = "two ends in one cycle";
    do_resetn();
    to_run(s0);
    s1 = lead_fs();
    cycles(2);
    cause(K_ECC2);
    cycles(1);
    chk_abort_seq(1'b1, "drop of both slots");
    sb_off         = 1'b1;
    relax_two_ends = 1'b1;
    relax_end_age  = 1'b1;
    i_capture_end  = 2'b11;
    expect_cmd("both ends", C_PULSE);
    cycles(1);
    expect_cmd("second pulse", C_PULSE);
    cycles(1);
    expect_cmd("no third pulse", C_NONE);
    to_run(s0);
    s1 = lead_fs();
    cycles(2);
    cause(K_ECC2);
    cycles(1);
    chk_abort_seq(1'b1, "drop of both slots");
    where = "merge";
    i_capture_end = 2'b11;
    lead_fs_slot(0);
    expect_cmd("both ends", C_PULSE);
    cycles(1);
    env_ovr          = 1'b1;
    env_ovr_val      = 2'b01;
    i_capture_end[0] = 1'b1;
    expect_cmd("pending pulse and a new dropped end", C_PULSE);
    cycles(1);
    env_ovr = 1'b0;
    expect_cmd("one pulse for the two", C_NONE);
    relax_two_ends = 1'b0;
    relax_end_age  = 1'b0;
    cycles(2);
    sb_clear();
    sb_off = 1'b0;
  endtask

  // dma_drop not preceded by capture_start, in ST_IDLE, ST_SKIP and ST_RUN.
  task automatic test_dma_drop_without_capture();
    int s;
    relax_dma = 1'b1;
    where = "dma_drop in ST_IDLE";
    do_resetn();
    i_stream_follow_dma_drop = 1'b1;
    i_core_enable = 1'b0;
    cycles(1);
    i_dma_drop = 1'b1;
    quiet(3, 2'b11, where);
    where = "dma_drop in ST_SKIP";
    i_core_enable = 1'b1;
    cycles(1);
    expect_state(where, S_SKIP);
    i_dma_drop = 1'b1;
    quiet(3, 2'b11, where);
    where = "dma_drop in ST_RUN, follow 1";
    to_run(s);
    cycles(2);
    i_dma_drop = 1'b1;
    cycles(1);
    expect_cmd("skip", C_SKIP);
    expect_eq("drop_vc_mask", o_drop_vc_mask, onehot(lead_ch()));
    cycles(1);
    end_chk(0, 1'b1, "slot 0 marked");
    where = "dma_drop in ST_RUN, follow 0";
    i_stream_follow_dma_drop = 1'b0;
    s = lead_fs();
    cycles(3);
    i_dma_drop = 1'b1;
    quiet(2, 2'b00, where);
    end_chk(s, 1'b1, "slot marked by the late dma_drop");
    relax_dma = 1'b0;
  endtask

  // ev_fs on Virtual Channels 2 and 3: only bit 0 of ev_vc is used.
  task automatic test_vc2_vc3_frame_start();
    int s;
    for (int map = 0; map < 2; map++) begin
      where = $sformatf("ev_vc 2 and 3, lead_vc %0d", map);
      do_resetn();
      i_vc_exposure_map = 1'(map);
      to_run(s, 1'b0, 1'b0);
      relax_vc23 = 1'b1;
      i_ev_fs = 1'b1;
      i_ev_vc = {1'b1, lead_ch()};
      cycles(1);
      quiet(2, onehot(~lead_ch()), "Frame Start on the lead channel + 2");
      i_ev_fs = 1'b1;
      i_ev_vc = {1'b1, ~lead_ch()};
      cycles(1);
      expect_fd("Frame Start on the lag channel + 2", 2'b00);
      relax_vc23 = 1'b0;
      close_all();
    end
    i_vc_exposure_map = 1'b0;
  endtask

  // core_enable low for exactly one cycle in ST_RUN, ST_HOLD, ST_FLUSH and ST_SKIP.
  task automatic test_core_enable_one_cycle();
    int s, s1;
    where = "one cycle in ST_RUN, no capture";
    do_resetn();
    to_run(s);
    end_chk(s, 1'b0);
    i_core_enable = 1'b0;
    cycles(1);
    i_core_enable = 1'b1;
    expect_cmd("no capture_drop", C_NONE);
    expect_fd(where, 2'b11);
    expect_state(where, S_IDLE);
    cycles(1);
    quiet(2, 2'b11, "ST_SKIP");
    to_run(s);

    where = "one cycle in ST_RUN, capture open";
    i_core_enable = 1'b0;
    cycles(1);
    i_core_enable = 1'b1;
    chk_abort_seq(1'b0, where);
    expect_state(where, S_SKIP);
    end_chk(s, 1'b1);

    where = "one cycle in ST_HOLD";
    to_run(s);
    cause(K_ECC2);
    cycles(1);
    i_core_enable = 1'b0;
    expect_cmd("first ST_HOLD cycle", C_DROP_HOLD);
    cycles(1);
    i_core_enable = 1'b1;
    expect_cmd("second ST_HOLD cycle", C_NONE);
    cycles(1);
    expect_cmd("ST_FLUSH", C_FLUSH_S);
    cycles(1);
    expect_state(where, S_SKIP);
    end_chk(s, 1'b1);

    where = "one cycle in ST_FLUSH";
    to_run(s);
    cause(K_ECC2);
    cycles(HOLD_CYCLES + 1);
    i_core_enable = 1'b0;
    expect_cmd("ST_FLUSH", C_FLUSH_S);
    cycles(1);
    i_core_enable = 1'b1;
    expect_state(where, S_IDLE);
    expect_cmd("ST_IDLE", C_NONE);
    cycles(1);
    expect_state(where, S_SKIP);
    end_chk(s, 1'b1);

    where = "one cycle in ST_SKIP";
    i_core_enable = 1'b0;
    cycles(1);
    i_core_enable = 1'b1;
    expect_state(where, S_IDLE);
    s1 = lead_fs();
    expect_true("capture opened", s1 >= 0);
    cycles(1);
    quiet(3, 2'b11, "capture_start in ST_IDLE ignored");
    close_all();
    to_run(s);
    close_all();
  endtask

  // active_sel toggled during ST_HOLD and in the ST_FLUSH cycle: rx_ctx_clear and asm_abort_mask
  // follow active_sel of the ST_FLUSH cycle.
  task automatic test_active_sel_in_drop();
    int s;
    i_tpg_dual_vc_en = 1'b0;
    for (int last = 0; last < 2; last++) begin
      where = $sformatf("active_sel %0d in the ST_FLUSH cycle", last);
      do_resetn();
      to_run(s);
      cause(K_ECC2);
      cycles(1);
      i_active_sel = 1'b1;
      cycles(HOLD_CYCLES);
      i_active_sel = 1'(last);
      expect_cmd("ST_FLUSH", last ? C_FLUSH_T : C_FLUSH_S);
      expect_eq("asm_abort_mask", o_asm_abort_mask, last ? 2'b01 : 2'b11);
      cycles(1);
      i_active_sel = 1'b0;
      expect_cmd("after ST_FLUSH", C_NONE);
      end_chk(s, 1'b1);
    end
    i_tpg_dual_vc_en = 1'b1;
  endtask

  // linebuffer_full rising in ST_RUN: both flags set in the next cycle.
  task automatic test_linebuffer_full();
    int s;
    for (int cap = 0; cap < 2; cap++) begin
      where = $sformatf("linebuffer_full, capture open %0d", cap);
      do_resetn();
      to_run(s);
      if (cap == 0) end_chk(s, 1'b0);
      i_linebuffer_full = 1'b1;
      expect_fd("cycle of linebuffer_full", 2'b00);
      cycles(1);
      expect_fd("next cycle", 2'b11);
      i_linebuffer_full = 1'b0;
      chk_abort_seq(1'b1, where);
      if (cap == 1) end_chk(s, 1'b1);
    end
  endtask

  // Events in the first cycle after a reset or after ST_IDLE is entered.
  task automatic test_first_cycle_events();
    int s, s1;
    where = "lead capture_start in the first cycle after resetn";
    #0.5;
    i_resetn = 1'b0;
    cycles(2);
    i_resetn      = 1'b1;
    i_core_enable = 1'b1;
    s = lead_fs();
    expect_true("capture opened", s >= 0);
    cycles(1);
    expect_cmd("ST_IDLE ignores it", C_NONE);
    expect_fd("ST_IDLE ignores it", 2'b11);
    cycles(1);
    quiet(3, 2'b11, "ST_SKIP waits");
    close_all();
    to_run(s);
    close_all();

    where = "axis_enable low in the cycle after soft_reset";
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset  = 1'b0;
    i_axis_enable = 1'b0;
    expect_cmd("cycle after soft_reset", C_NONE);
    cycles(1);
    expect_cmd("no asm_abort_req", C_NONE);
    i_axis_enable = 1'b1;
    cycles(2);
    where = "axis_enable low two cycles after soft_reset";
    i_soft_reset = 1'b1;
    cycles(1);
    i_soft_reset = 1'b0;
    cycles(1);
    i_axis_enable = 1'b0;
    cycles(1);
    expect_cmd("asm_abort_req", C_STOP);
    i_axis_enable = 1'b1;
    cycles(2);

    where = "lead capture_start in ST_IDLE after one cycle of core_enable low";
    i_core_enable = 1'b1;
    cycles(1);
    to_run(s);
    end_chk(s, 1'b0);
    i_core_enable = 1'b0;
    cycles(1);
    i_core_enable = 1'b1;
    expect_state(where, S_IDLE);
    s = lead_fs();
    cycles(1);
    quiet(3, 2'b11, "ignored");
    close_all();

    where = "lead capture_start in ST_IDLE after ST_FLUSH";
    to_run(s);
    i_core_enable = 1'b0;
    cycles(1);
    expect_cmd("first ST_HOLD cycle", C_HOLD);
    cycles(HOLD_CYCLES + 1);
    i_core_enable = 1'b1;
    expect_state(where, S_IDLE);
    s1 = lead_fs();
    expect_true("capture opened", s1 >= 0);
    cycles(1);
    quiet(3, 2'b11, "capture_start in the first ST_IDLE cycle ignored");
    close_all();
  endtask

  // Random traffic within the rules of the connected blocks, checked by the reference model, the
  // checkers and the scoreboard.
  task automatic random_traffic(input int ncyc);
    int lb_len, s;
    bit start_q;
    lb_len  = 0;
    start_q = 1'b0;
    where   = "random";
    i_core_enable = 1'b1;
    for (int c = 0; c < ncyc; c++) begin
      // Configuration levels; the path settings only with no capture open.
      if (rand_pm(3)) i_core_enable = !i_core_enable;
      if (!i_core_enable && rand_pm(30)) i_core_enable = 1'b1;
      if (rand_pm(5)) i_axis_enable = !i_axis_enable;
      if (rand_pm(3)) i_stream_follow_dma_drop = !i_stream_follow_dma_drop;
      if (env_open == 2'b00 && rand_pm(4)) begin
        i_active_sel      = rand_pct(30);
        i_vc_exposure_map = rand_pct(50);
        i_tpg_dual_vc_en  = rand_pct(50);
      end
      if (lb_len > 0)      lb_len--;
      else if (rand_pm(3)) lb_len = rand_range(1, 4);
      i_linebuffer_full = (lb_len > 0);
      // Rare resets; no capture event in a soft_reset cycle.
      if (rand_pm(1)) begin
        if (rand_pct(80)) begin
          i_soft_reset = 1'b1;
          cycles(1);
          i_soft_reset = 1'b0;
        end else begin
          do_resetn();
        end
        start_q = 1'b0;
        continue;
      end
      // Error pulses at a packet boundary.
      if (rand_pm(4)) cause(K_ECC2);
      if (rand_pm(4)) cause(K_LANE);
      if (rand_pm(6)) cause(K_CRC, rand_pct(50));
      if (rand_pm(6)) cause(K_WC, rand_pct(50));
      if (rand_pm(3)) cause(K_LBOVF);
      if (rand_pm(3)) cause(K_FBOVF);
      // dma_drop one cycle after a capture_start.
      if (start_q && rand_pct(30)) i_dma_drop = 1'b1;
      start_q = 1'b0;
      // Frame events.
      if (rand_pm(25)) begin
        s = lead_fs();
        start_q = (s >= 0);
      end else if (rand_pm(25)) begin
        lag_fs();
      end
      // At most one capture_end per cycle; open captures end sooner while core_enable is low.
      for (int k = 0; k < 2; k++) begin
        if (i_capture_end == 2'b00 && !i_capture_start[k] && env_open[k] && env_age[k] >= 1 &&
            rand_pm(i_core_enable ? 30 : 200))
          i_capture_end[k] = 1'b1;
      end
      cycles(1);
    end
    i_linebuffer_full        = 1'b0;
    i_core_enable            = 1'b1;
    i_axis_enable            = 1'b1;
    i_stream_follow_dma_drop = 1'b0;
    cycles(4);
    close_all();
    $display("%s: ST_HOLD %0d, ST_RUN to ST_SKIP %0d, ST_RUN to ST_IDLE %0d, resume %0d",
             where, n_hold, n_skip_entry, n_idle_entry, n_resume);
    $display("%s: resume through lead_seen %0d, DMA skip %0d, frame_drop_pulse %0d",
             where, n_seen_resume, n_dma_skip, n_fdp);
    expect_true("drops through ST_HOLD were exercised", n_hold > 0);
    expect_true("drops without flush were exercised", n_skip_entry > 0);
    expect_true("ST_RUN to ST_IDLE was exercised", n_idle_entry > 0);
    expect_true("resumes were exercised", n_resume > 0);
    expect_true("resumes through lead_seen were exercised", n_seen_resume > 0);
    expect_true("DMA skips were exercised", n_dma_skip > 0);
    expect_true("frame_drop_pulse was exercised", n_fdp > 0);
    expect_true("flushes on the sensor path were exercised", n_flush_s > 0);
    expect_true("flushes on the generator path were exercised", n_flush_t > 0);
    expect_true("falls of axis_enable were exercised", n_stop > 0);
  endtask

  task automatic test_random_traffic();
    random_traffic(200000);
  endtask

  // Forces a wrong force_discard in ST_RUN; this test must end in FAIL with errors in the log.
  // It confirms that the checkers catch a fault and is not part of the regression.
  task automatic test_checker_fault();
    int s;
    to_run(s);
    force o_force_discard = 2'b11;
    cycles(3);
    release o_force_discard;
    cycles(2);
    close_all();
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
    env_age[0]               = 0;
    env_age[1]               = 0;
    i_resetn                 = 1'b1;
    i_soft_reset             = 1'b0;
    i_core_enable            = 1'b1;
    i_axis_enable            = 1'b1;
    i_stream_follow_dma_drop = 1'b0;
    i_vc_exposure_map        = 1'b0;
    i_tpg_dual_vc_en         = 1'b1;
    i_active_sel             = 1'b0;
    i_linebuffer_full        = 1'b0;
    clear_pulses();
    model_reset();
    $display("%s: HOLD_CYCLES=%0d SEED=%0d", test_name, HOLD_CYCLES, seed);
    started = 1'b1;
    // Power-on reset: a falling edge of resetn before the first clock edge.
    #1.0;
    i_resetn = 1'b0;
    cycles(4);
    i_resetn = 1'b1;
    cycles(2);
    case (test_name)
      "test_reset_values":             test_reset_values();
      "test_soft_reset":               test_soft_reset();
      "test_packet_causes":            test_packet_causes();
      "test_packet_cause_no_capture":  test_packet_cause_no_capture();
      "test_midline_causes":           test_midline_causes();
      "test_cause_qualifiers":         test_cause_qualifiers();
      "test_core_enable":              test_core_enable();
      "test_dma_drop_follow":          test_dma_drop_follow();
      "test_dma_drop_no_follow":       test_dma_drop_no_follow();
      "test_abort_and_skip":           test_abort_and_skip();
      "test_causes_outside_run":       test_causes_outside_run();
      "test_hold_and_flush":           test_hold_and_flush();
      "test_skip_exit":                test_skip_exit();
      "test_discard_flags":            test_discard_flags();
      "test_axis_enable_fall":         test_axis_enable_fall();
      "test_abort_mask":               test_abort_mask();
      "test_capture_watch":            test_capture_watch();
      "test_drop_report":              test_drop_report();
      "test_error_on_overlap":         test_error_on_overlap();
      "test_tpg_selected":             test_tpg_selected();
      "test_frame_never_closed":       test_frame_never_closed();
      "test_two_ends_same_cycle":      test_two_ends_same_cycle();
      "test_dma_drop_without_capture": test_dma_drop_without_capture();
      "test_vc2_vc3_frame_start":      test_vc2_vc3_frame_start();
      "test_core_enable_one_cycle":    test_core_enable_one_cycle();
      "test_active_sel_in_drop":       test_active_sel_in_drop();
      "test_linebuffer_full":          test_linebuffer_full();
      "test_first_cycle_events":       test_first_cycle_events();
      "test_random_traffic":           test_random_traffic();
      "test_checker_fault":            test_checker_fault();
      default: begin
        err_count++;
        $error("unknown test '%s'", test_name);
      end
    endcase
    where = "end of test";
    cycles(4);
    if (sb_expect != sb_pulses)
      fail($sformatf("scoreboard: %0d dropped capture ends, %0d frame_drop_pulse",
                     sb_expect, sb_pulses));
    else
      $display("scoreboard: %0d dropped capture ends, %0d frame_drop_pulse", sb_expect,
               sb_pulses);
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
