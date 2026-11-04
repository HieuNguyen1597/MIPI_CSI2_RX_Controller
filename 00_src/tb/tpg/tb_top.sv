// File        : tb_top.sv
// Description : Testbench of m_fpt_tpg (Test Pattern Generator).
//               A reference model builds every beat of a frame from the configuration latched for
//               that frame. A monitor compares the bus, the frame events and the shadow outputs
//               with it at every clock edge, and checks the valid/ready rules, the line gaps and,
//               when a test asks for it, the gaps between frames. Tests drive stimulus and check
//               start and stop times.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions (decided by the block owner; the specification does not state them yet):
//   - Every register is clocked on the rising edge of sys_clk and samples its inputs there.
//   - The configuration shadows and thresholds are cleared by resetn and kept across soft_reset,
//     so tpg_height_shadow and px_dt_sel are 0 after resetn until the first frame is latched.
//   - CALC_THRESHOLDS lasts exactly one cycle and always moves to RUNNING. The first beat of a
//     frame is therefore presented in the cycle right after the one in which the eight fields
//     are latched, and a start from IDLE shows the first beat two cycles after the cycle in
//     which run_en, linebuf_empty and the size guard are all met.
//   - In RUNNING a drop of run_en wins over the loop-back; in STOPPING run_en wins over the
//     return to IDLE.
//   - x_cnt and vc_toggle move only on an accepted beat; y_cnt moves on the accepted last beat
//     of a line that completes a row.
//   - The band of a pixel is min(7, coordinate / band_base): every band is band_base pixels or
//     lines wide and band 7 also takes the remainder.
//   - px_vc_tag and the blank line-time use the latched values (dual_vc_shadow, hblank_shadow).
//   - resetn (at once) and soft_reset (at the next edge) may drop px_valid while a beat waits;
//     the beat and the frame in progress are discarded.
//   - The size guard is checked only when leaving IDLE. Software keeps the truncated width and
//     the height at 8 or more while the generator runs, so no test writes a smaller value after
//     the start.
//   - The sys_clk frequency is not fixed yet; the testbench uses 200 MHz, which changes no
//     behaviour.
`timescale 1ns/1ps

module tb_top;

  // sys_clk period, 200 MHz.
  localparam real T_CLK = 5.0;

  // Value of a colour component that is on.
  localparam logic [11:0] BAR_LEVEL = 12'hFFF;

  // Longest wait for a frame event before a test reports an error, in clock cycles.
  localparam longint unsigned WAIT_LIMIT = 64'd4_000_000;

  // ----------------------------------------------------------------------------------------------
  // DUT
  // ----------------------------------------------------------------------------------------------
  logic        i_sys_clk;
  logic        i_resetn;
  logic        i_soft_reset;
  logic        i_core_enable;
  logic        i_tpg_enable;
  logic        i_tpg_pattern;
  logic [15:0] i_tpg_width;
  logic [15:0] i_tpg_height;
  logic        i_tpg_dual_vc_en;
  logic        i_px_ready;
  logic        i_tpg_pixel_format;
  logic [15:0] i_tpg_hblank;
  logic [15:0] i_tpg_vblank;
  logic [1:0]  i_tpg_cfa_order;
  logic        i_linebuf_empty;

  logic [47:0] o_px_data;
  logic        o_px_valid;
  logic        o_px_sof;
  logic        o_px_sol;
  logic        o_px_eol;
  logic [1:0]  o_px_vc_tag;
  logic        o_px_dt_sel;
  logic        o_tpg_eof;
  logic        o_tpg_fs;
  logic        o_tpg_fe;
  logic [1:0]  o_tpg_fvc;
  logic [15:0] o_tpg_height_shadow;

  m_fpt_tpg u_dut (
    .i_sys_clk           (i_sys_clk),
    .i_resetn            (i_resetn),
    .i_soft_reset        (i_soft_reset),
    .i_core_enable       (i_core_enable),
    .i_tpg_enable        (i_tpg_enable),
    .i_tpg_pattern       (i_tpg_pattern),
    .i_tpg_width         (i_tpg_width),
    .i_tpg_height        (i_tpg_height),
    .i_tpg_dual_vc_en    (i_tpg_dual_vc_en),
    .i_px_ready          (i_px_ready),
    .i_tpg_pixel_format  (i_tpg_pixel_format),
    .i_tpg_hblank        (i_tpg_hblank),
    .i_tpg_vblank        (i_tpg_vblank),
    .i_tpg_cfa_order     (i_tpg_cfa_order),
    .i_linebuf_empty     (i_linebuf_empty),
    .o_px_data           (o_px_data),
    .o_px_valid          (o_px_valid),
    .o_px_sof            (o_px_sof),
    .o_px_sol            (o_px_sol),
    .o_px_eol            (o_px_eol),
    .o_px_vc_tag         (o_px_vc_tag),
    .o_px_dt_sel         (o_px_dt_sel),
    .o_tpg_eof           (o_tpg_eof),
    .o_tpg_fs            (o_tpg_fs),
    .o_tpg_fe            (o_tpg_fe),
    .o_tpg_fvc           (o_tpg_fvc),
    .o_tpg_height_shadow (o_tpg_height_shadow)
  );

  initial begin
    i_sys_clk = 1'b0;
    forever #(T_CLK / 2.0) i_sys_clk = ~i_sys_clk;
  end

  // ----------------------------------------------------------------------------------------------
  // Configuration and reference model
  // ----------------------------------------------------------------------------------------------
  // The eight fields latched for each frame.
  typedef struct packed {
    logic [15:0] width;
    logic [15:0] height;
    logic        pattern;
    logic        format;
    logic        dual_vc;
    logic [1:0]  cfa;
    logic [15:0] hblank;
    logic [15:0] vblank;
  } cfg_t;

  // Expected content of one beat.
  typedef struct packed {
    logic [47:0] data;
    logic        sof;
    logic        sol;
    logic        eol;
    logic [1:0]  vc;
    logic        last_row;   // last beat of a line of the last row: tpg_fe when accepted
    logic        last_beat;  // last beat of the frame: tpg_eof when accepted
  } beat_t;

  function automatic cfg_t make_cfg(input int unsigned width, input int unsigned height,
                                    input bit pattern, input bit format, input bit dual_vc,
                                    input int unsigned cfa, input int unsigned hblank,
                                    input int unsigned vblank);
    cfg_t c;
    c.width   = 16'(width);
    c.height  = 16'(height);
    c.pattern = pattern;
    c.format  = format;
    c.dual_vc = dual_vc;
    c.cfa     = 2'(cfa);
    c.hblank  = 16'(hblank);
    c.vblank  = 16'(vblank);
    return c;
  endfunction

  function automatic string cfg_text(input cfg_t c);
    return {$sformatf("width=%0d height=%0d pattern=%0d format=%0d ", c.width, c.height,
                      c.pattern, c.format),
            $sformatf("dual_vc=%0d cfa=%0d hblank=%0d vblank=%0d", c.dual_vc, c.cfa, c.hblank,
                      c.vblank)};
  endfunction

  // Width used by the datapath: tpg_width truncated to a multiple of 4.
  function automatic int unsigned width_used(input cfg_t c);
    return 32'({c.width[15:2], 2'b00});
  endfunction

  function automatic bit guard_ok(input cfg_t c);
    return (width_used(c) >= 8) && (c.height >= 16'd8);
  endfunction

  function automatic int unsigned beats_per_line(input cfg_t c);
    return width_used(c) / 4;
  endfunction

  function automatic int unsigned lines_per_row(input cfg_t c);
    return c.dual_vc ? 2 : 1;
  endfunction

  function automatic int unsigned frame_lines(input cfg_t c);
    return 32'(c.height) * lines_per_row(c);
  endfunction

  function automatic int unsigned frame_beats(input cfg_t c);
    return frame_lines(c) * beats_per_line(c);
  endfunction

  // One blank line-time.
  function automatic int unsigned line_period(input cfg_t c);
    return beats_per_line(c) + 32'(c.hblank);
  endfunction

  // Idle cycles between the last beat of a frame and the first beat of the next one when
  // generation goes on: hblank, vblank line-times and the two frame-boundary cycles.
  function automatic longint unsigned frame_gap(input cfg_t c);
    return longint'(c.hblank) + longint'(c.vblank) * longint'(line_period(c)) + 64'd2;
  endfunction

  // Cycles from the first beat of a frame to the first beat of the next one, px_ready held at 1.
  function automatic longint unsigned frame_period(input cfg_t c);
    return (longint'(frame_lines(c)) + longint'(c.vblank)) * longint'(line_period(c)) + 64'd2;
  endfunction

  // Idle time checked after a stop: long enough to see a frame that should not follow.
  function automatic longint unsigned idle_after_stop(input cfg_t c);
    longint unsigned n;
    n = frame_gap(c) + 64'd20;
    return (n > 64'd3000) ? 64'd3000 : n;
  endfunction

  // Pixel at column col and row row: colour bar from the band index, one Bayer component of it.
  function automatic logic [11:0] exp_pixel(input cfg_t c, input int unsigned col,
                                            input int unsigned row);
    int unsigned band_base;
    int unsigned coord;
    int unsigned band;
    logic [2:0]  idx;
    logic        col_odd;
    logic        row_odd;
    logic        on;
    logic [11:0] pix12;
    band_base = c.pattern ? (32'(c.height) / 8) : (width_used(c) / 8);
    if (band_base == 0) band_base = 1;
    coord = c.pattern ? row : col;
    band  = coord / band_base;
    idx   = (band > 7) ? 3'd7 : 3'(band);
    col_odd = col[0];
    row_odd = row[0];
    // cfa gives the R site of the 2x2 cell (bit 0 odd column, bit 1 odd row); B is diagonal to it.
    if (col_odd == c.cfa[0] && row_odd == c.cfa[1])      on = !idx[1];
    else if (col_odd != c.cfa[0] && row_odd != c.cfa[1]) on = !idx[0];
    else                                                  on = !idx[2];
    pix12 = on ? BAR_LEVEL : 12'h000;
    return c.format ? {2'b00, pix12[11:2]} : pix12;
  endfunction

  // Beat p of a frame: rows in order, VC0 then VC1 line per row in dual-VC mode, 4 pixels a beat.
  function automatic beat_t exp_beat(input cfg_t c, input int unsigned p);
    beat_t       b;
    int unsigned bpl;
    int unsigned lpr;
    int unsigned line;
    int unsigned x;
    int unsigned y;
    int unsigned vc;
    bpl  = beats_per_line(c);
    lpr  = lines_per_row(c);
    line = p / bpl;
    x    = (p % bpl) * 4;
    y    = line / lpr;
    vc   = line % lpr;
    b.sol       = (x == 0);
    b.eol       = (x == width_used(c) - 4);
    b.sof       = b.sol && (y == 0);
    b.vc        = 2'(vc);
    b.last_row  = b.eol && (y == 32'(c.height) - 1);
    b.last_beat = b.last_row && (vc == lpr - 1);
    for (int n = 0; n < 4; n++) b.data[n*12 +: 12] = exp_pixel(c, x + n, y);
    return b;
  endfunction

  function automatic cfg_t live_cfg();
    cfg_t c;
    c.width   = i_tpg_width;
    c.height  = i_tpg_height;
    c.pattern = i_tpg_pattern;
    c.format  = i_tpg_pixel_format;
    c.dual_vc = i_tpg_dual_vc_en;
    c.cfa     = i_tpg_cfa_order;
    c.hblank  = i_tpg_hblank;
    c.vblank  = i_tpg_vblank;
    return c;
  endfunction

  // ----------------------------------------------------------------------------------------------
  // Monitor
  // ----------------------------------------------------------------------------------------------
  int unsigned     err_count         = 0;
  int unsigned     check_count       = 0;
  longint unsigned cyc               = 0;    // rising edges so far; edge n samples cycle n
  bit              started           = 1'b0; // the first reset has defined the outputs
  bit              async_reset_seen  = 1'b0;

  bit              in_frame          = 1'b0;
  cfg_t            frame_cfg         = '0;   // configuration of the current or last frame
  int unsigned     pos               = 0;    // index of the beat on the bus within the frame
  cfg_t            prev_cfg          = '0;   // inputs sampled at the previous edge
  bit              frame_stalled     = 1'b0;

  bit              shadow_known      = 1'b0;
  logic            exp_dt_sel        = 1'b0;
  logic [15:0]     exp_height_shadow = '0;

  bit              wait_hold         = 1'b0; // a beat was waiting for px_ready at the last edge
  bit              mid_line          = 1'b0; // a beat that does not end a line was accepted
  bit              line_gap_on       = 1'b0;
  int unsigned     line_gap_cnt      = 0;
  int unsigned     line_gap_exp      = 0;

  bit              chk_frame_gap     = 1'b0; // set by a test: generation goes on between frames
  bit              frame_gap_ref     = 1'b0; // the last frame ended normally
  longint unsigned frame_gap_cnt     = 0;
  longint unsigned frame_gap_exp     = 0;
  cfg_t            done_cfg          = '0;
  longint unsigned done_start        = 0;
  bit              done_stalled      = 1'b0;

  int unsigned     frames_started    = 0;
  int unsigned     frames_done       = 0;
  int unsigned     frames_aborted    = 0;
  longint unsigned last_start_cyc    = 0;
  longint unsigned beats_accepted    = 0;
  longint unsigned valid_cycles      = 0;

  task automatic report_error(input string msg);
    err_count++;
    if (err_count <= 30) $error("edge %0d: %s", cyc, msg);
  endtask

  // Drops the frame in progress (reset).
  task automatic abort_frame();
    if (in_frame) frames_aborted++;
    in_frame      = 1'b0;
    wait_hold     = 1'b0;
    mid_line      = 1'b0;
    line_gap_on   = 1'b0;
    frame_gap_ref = 1'b0;
  endtask

  // resetn clears the outputs at once, without waiting for a clock edge.
  always @(negedge i_resetn) begin
    if (started) begin
      async_reset_seen = 1'b1;
      #0.1;
      if ({o_px_valid, o_px_sof, o_px_sol, o_px_eol, o_tpg_fs, o_tpg_fe, o_tpg_eof} !== 7'b0)
        report_error("resetn did not clear px_valid and the flags at once");
      if (o_tpg_height_shadow !== 16'd0 || o_px_dt_sel !== 1'b0 || o_px_vc_tag !== 2'b00)
        report_error("resetn did not clear the shadows and the VC tag at once");
    end
  end

  // Every rising edge: the values seen here are those of the cycle that ends at this edge.
  always @(posedge i_sys_clk) begin : p_monitor
    cfg_t  live;
    beat_t eb;
    logic  accept;
    bit    skip;
    cyc    = cyc + 1;
    live   = live_cfg();
    eb     = '0;
    accept = 1'b0;
    skip   = 1'b0;
    if (!started && cyc >= 2 && i_resetn === 1'b0) started = 1'b1;
    if (started) begin
      check_count++;
      if ($isunknown({o_px_data, o_px_valid, o_px_sof, o_px_sol, o_px_eol, o_px_vc_tag,
                      o_px_dt_sel, o_tpg_eof, o_tpg_fs, o_tpg_fe, o_tpg_fvc, o_tpg_height_shadow}))
        report_error("X or Z on an output");

      if (i_resetn !== 1'b1 || async_reset_seen) begin
        if (i_resetn !== 1'b1 && o_px_valid !== 1'b0) report_error("px_valid high during resetn");
        abort_frame();
        shadow_known      = 1'b1;
        exp_dt_sel        = 1'b0;
        exp_height_shadow = '0;
        async_reset_seen  = 1'b0;
        skip              = (i_resetn !== 1'b1);
      end

      if (!skip) begin
        accept = o_px_valid && i_px_ready;
        if (o_px_valid) valid_cycles++;

        // Once presented, a beat stays until accepted; a line has no gap inside.
        if (wait_hold && !o_px_valid)
          report_error("px_valid fell while a beat waited for px_ready");
        if (mid_line && !o_px_valid) report_error("px_valid fell inside a line");

        // tpg_hblank idle cycles after each line that does not end the frame.
        if (line_gap_on) begin
          if (o_px_valid) begin
            if (line_gap_cnt != line_gap_exp)
              report_error($sformatf("line gap of %0d cycles, expected hblank = %0d",
                                     line_gap_cnt, line_gap_exp));
            line_gap_on = 1'b0;
          end else begin
            line_gap_cnt++;
            if (line_gap_cnt > line_gap_exp) begin
              report_error($sformatf("no beat after a line gap of %0d cycles (hblank = %0d)",
                                     line_gap_cnt, line_gap_exp));
              line_gap_on = 1'b0;
            end
          end
        end

        // Gap between frames while generation goes on.
        if (!in_frame && frame_gap_ref && chk_frame_gap && !o_px_valid) begin
          frame_gap_cnt++;
          if (frame_gap_cnt > frame_gap_exp) begin
            report_error($sformatf("no frame start after a gap of %0d cycles, expected %0d",
                                   frame_gap_cnt, frame_gap_exp));
            frame_gap_ref = 1'b0;
          end
        end

        if (o_px_valid) begin
          if (!in_frame) begin
            // First beat of a frame: its configuration was latched at the previous edge.
            if (chk_frame_gap && frame_gap_ref) begin
              if (frame_gap_cnt != frame_gap_exp)
                report_error($sformatf("frame gap of %0d cycles, expected %0d (%s)",
                                       frame_gap_cnt, frame_gap_exp, cfg_text(done_cfg)));
              if (!done_stalled && (cyc - done_start) != frame_period(done_cfg))
                report_error($sformatf("frame period of %0d cycles, expected %0d (%s)",
                                       cyc - done_start, frame_period(done_cfg),
                                       cfg_text(done_cfg)));
            end
            frame_gap_ref     = 1'b0;
            frame_cfg         = prev_cfg;
            in_frame          = 1'b1;
            pos               = 0;
            frame_stalled     = 1'b0;
            frames_started++;
            last_start_cyc    = cyc;
            if (!guard_ok(frame_cfg))
              report_error({"frame started below the size guard: ", cfg_text(frame_cfg)});
            shadow_known      = 1'b1;
            exp_dt_sel        = frame_cfg.format;
            exp_height_shadow = frame_cfg.height;
          end

          eb = exp_beat(frame_cfg, pos);
          if (o_px_data !== eb.data)
            report_error($sformatf("px_data %012h, expected %012h (beat %0d, %s)",
                                   o_px_data, eb.data, pos, cfg_text(frame_cfg)));
          if ({o_px_sof, o_px_sol, o_px_eol} !== {eb.sof, eb.sol, eb.eol})
            report_error($sformatf("sof/sol/eol %b, expected %b (beat %0d)",
                                   {o_px_sof, o_px_sol, o_px_eol}, {eb.sof, eb.sol, eb.eol}, pos));
          if (o_px_vc_tag !== eb.vc)
            report_error($sformatf("px_vc_tag %0d, expected %0d (beat %0d)",
                                   o_px_vc_tag, eb.vc, pos));
          if ({o_tpg_fs, o_tpg_fe, o_tpg_eof} !==
              {accept && eb.sof, accept && eb.last_row, accept && eb.last_beat})
            report_error($sformatf("tpg_fs/fe/eof %b, expected %b (beat %0d, px_ready %b)",
                                   {o_tpg_fs, o_tpg_fe, o_tpg_eof},
                                   {accept && eb.sof, accept && eb.last_row,
                                    accept && eb.last_beat},
                                   pos, i_px_ready));
          if ((o_tpg_fs || o_tpg_fe) && o_tpg_fvc !== eb.vc)
            report_error($sformatf("tpg_fvc %0d, expected %0d (beat %0d)", o_tpg_fvc, eb.vc, pos));
          if (!i_px_ready) frame_stalled = 1'b1;

          if (accept) begin
            beats_accepted++;
            if (eb.last_beat) begin
              in_frame      = 1'b0;
              frames_done++;
              done_cfg      = frame_cfg;
              done_start    = last_start_cyc;
              done_stalled  = frame_stalled;
              frame_gap_ref = 1'b1;
              frame_gap_cnt = 0;
              frame_gap_exp = frame_gap(frame_cfg);
            end else if (eb.eol) begin
              line_gap_on  = 1'b1;
              line_gap_cnt = 0;
              line_gap_exp = 32'(frame_cfg.hblank);
            end
            pos++;
          end
        end else begin
          if ({o_px_sof, o_px_sol, o_px_eol} !== 3'b000)
            report_error("sof/sol/eol set without px_valid");
          if ({o_tpg_fs, o_tpg_fe, o_tpg_eof} !== 3'b000)
            report_error("tpg_fs/fe/eof set without an accepted beat");
        end

        wait_hold = o_px_valid && !i_px_ready;
        mid_line  = accept && !eb.eol;

        // soft_reset seen at this edge drops the frame in progress from the next cycle.
        if (i_soft_reset) abort_frame();
      end

      // Shadow outputs: values latched for the current or last frame, 0 after resetn.
      if (shadow_known) begin
        if (o_px_dt_sel !== exp_dt_sel)
          report_error($sformatf("px_dt_sel %b, expected %b", o_px_dt_sel, exp_dt_sel));
        if (o_tpg_height_shadow !== exp_height_shadow)
          report_error($sformatf("tpg_height_shadow %0d, expected %0d",
                                 o_tpg_height_shadow, exp_height_shadow));
      end
    end
    prev_cfg = live;
  end

  // ----------------------------------------------------------------------------------------------
  // Stimulus helpers; every input changes right after a falling edge
  // ----------------------------------------------------------------------------------------------
  // px_ready: 0 always high, 1 high on ready_pct percent of cycles, 2 long stalls,
  // 3 set by the test.
  int unsigned ready_mode = 0;
  int unsigned ready_pct  = 100;
  int unsigned stall_left = 0;

  always @(negedge i_sys_clk) begin
    case (ready_mode)
      0: i_px_ready = 1'b1;
      1: i_px_ready = ($urandom_range(1, 100) <= ready_pct);
      2: begin
        if (stall_left > 0) begin
          i_px_ready = 1'b0;
          stall_left--;
        end else begin
          i_px_ready = 1'b1;
          if ($urandom_range(1, 20) == 1) stall_left = $urandom_range(1, 200);
        end
      end
      default: ;
    endcase
  end

  task automatic tick(input longint unsigned n = 1);
    repeat (n) @(negedge i_sys_clk);
  endtask

  task automatic apply_cfg(input cfg_t c);
    i_tpg_width        = c.width;
    i_tpg_height       = c.height;
    i_tpg_pattern      = c.pattern;
    i_tpg_pixel_format = c.format;
    i_tpg_dual_vc_en   = c.dual_vc;
    i_tpg_cfa_order    = c.cfa;
    i_tpg_hblank       = c.hblank;
    i_tpg_vblank       = c.vblank;
  endtask

  // Legal configuration with random fields; big allows longer lines and more rows.
  function automatic cfg_t random_cfg(input bit big);
    cfg_t c;
    c.width   = 16'($urandom_range(8, big ? 160 : 64));
    c.height  = 16'($urandom_range(8, big ? 16 : 12));
    c.pattern = 1'($urandom);
    c.format  = 1'($urandom);
    c.dual_vc = 1'($urandom);
    c.cfa     = 2'($urandom);
    c.hblank  = 16'($urandom_range(0, big ? 8 : 4));
    c.vblank  = 16'($urandom_range(0, 2));
    return c;
  endfunction

  // Writes one field, or all eight, with a new legal value.
  task automatic change_cfg();
    cfg_t c;
    c = live_cfg();
    case ($urandom_range(0, 9))
      0:       c.width   = 16'($urandom_range(8, 96));
      1:       c.height  = 16'($urandom_range(8, 14));
      2:       c.pattern = ~c.pattern;
      3:       c.format  = ~c.format;
      4:       c.dual_vc = ~c.dual_vc;
      5:       c.cfa     = 2'($urandom);
      6:       c.hblank  = 16'($urandom_range(0, 5));
      7:       c.vblank  = 16'($urandom_range(0, 2));
      default: c = random_cfg(1'b0);
    endcase
    apply_cfg(c);
  endtask

  // The next frame must start with its first beat sampled at edge exp_edge.
  task automatic expect_start_at(input longint unsigned exp_edge, input string what);
    int unsigned n0;
    n0 = frames_started;
    while (frames_started == n0 && cyc < exp_edge + 100) tick(1);
    if (frames_started == n0)
      report_error({what, ": no frame started"});
    else if (last_start_cyc != exp_edge)
      report_error($sformatf("%s: first beat at edge %0d, expected %0d", what, last_start_cyc,
                             exp_edge));
  endtask

  // No beat for n cycles.
  task automatic expect_idle(input longint unsigned n, input string what);
    longint unsigned v0;
    v0 = valid_cycles;
    tick(n);
    if (valid_cycles != v0) report_error({what, ": px_valid was asserted"});
  endtask

  task automatic wait_frame_start(input string what);
    int unsigned     n0;
    longint unsigned limit;
    n0    = frames_started;
    limit = cyc + WAIT_LIMIT;
    while (frames_started == n0 && cyc < limit) tick(1);
    if (frames_started == n0) report_error({what, ": no frame start"});
  endtask

  // Returns in the cycle after the accepted last beat of a frame.
  task automatic wait_frame_end(input string what);
    int unsigned     d0;
    longint unsigned limit;
    d0    = frames_done;
    limit = cyc + WAIT_LIMIT;
    while (frames_done == d0 && cyc < limit) tick(1);
    if (frames_done == d0) report_error({what, ": no frame end"});
  endtask

  // Returns when beat p of the current (or next) frame is on the bus.
  task automatic wait_beat(input int unsigned p, input string what);
    longint unsigned limit;
    limit = cyc + WAIT_LIMIT;
    while (!(in_frame && pos >= p) && cyc < limit) tick(1);
    if (!(in_frame && pos >= p)) report_error($sformatf("%s: beat %0d never came", what, p));
  endtask

  // After a stop request: the frame in progress must end and no frame may follow.
  task automatic finish_after_stop(input string what);
    longint unsigned limit;
    chk_frame_gap = 1'b0;
    limit = cyc + 20 * frame_period(frame_cfg) + 10000;
    while (in_frame && cyc < limit) tick(1);
    if (in_frame) report_error({what, ": frame not completed after the stop request"});
    expect_idle(idle_after_stop(frame_cfg), {what, ": a frame followed the stop"});
    frame_gap_ref = 1'b0;
  endtask

  task automatic stop_after_frame(input string what);
    chk_frame_gap = 1'b0;
    i_tpg_enable  = 1'b0;
    finish_after_stop(what);
  endtask

  // From IDLE: generates exactly n frames with c (the stop is requested during frame n).
  task automatic run_frames(input cfg_t c, input int unsigned n, input bit check_gaps);
    int unsigned f0;
    apply_cfg(c);
    chk_frame_gap = check_gaps;
    f0            = frames_started;
    i_tpg_enable  = 1'b1;
    expect_start_at(cyc + 3, {"start from IDLE, ", cfg_text(c)});
    while (frames_started < f0 + n) wait_frame_start("next frame");
    stop_after_frame({"stop, ", cfg_text(c)});
  endtask

  // soft_reset for n cycles; with run_en high a new frame starts from row 0 three cycles after
  // the last cycle of soft_reset.
  task automatic soft_reset_restart(input int unsigned n, input string what);
    i_soft_reset = 1'b1;
    tick(n);
    i_soft_reset = 1'b0;
    expect_start_at(cyc + 3, what);
  endtask

  // resetn asserted offset ns after a rising edge, released on a falling edge three cycles later.
  task automatic pulse_resetn(input real offset);
    @(posedge i_sys_clk);
    #(offset);
    i_resetn = 1'b0;
    tick(3);
    i_resetn = 1'b1;
  endtask

  // ----------------------------------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------------------------------

  // Values after resetn, then each start condition held back alone.
  task automatic test_reset_state();
    if ({o_px_valid, o_px_sof, o_px_sol, o_px_eol, o_tpg_fs, o_tpg_fe, o_tpg_eof} !== 7'b0)
      report_error("px_valid or a flag is set after resetn");
    if (o_px_vc_tag !== 2'b00 || o_tpg_fvc !== 2'b00) report_error("VC tag is not 0 after resetn");
    if (o_tpg_height_shadow !== 16'd0 || o_px_dt_sel !== 1'b0)
      report_error("tpg_height_shadow or px_dt_sel is not 0 after resetn");
    apply_cfg(make_cfg(32, 8, 0, 1, 0, 0, 0, 0));
    i_tpg_enable    = 1'b0;
    expect_idle(40, "tpg_enable low");
    i_tpg_enable    = 1'b1;
    i_core_enable   = 1'b0;
    expect_idle(40, "core_enable low");
    i_core_enable   = 1'b1;
    i_linebuf_empty = 1'b0;
    expect_idle(40, "Line Buffer not empty");
    i_linebuf_empty = 1'b1;
    expect_start_at(cyc + 3, "start once every condition is met");
    stop_after_frame("stop");
  endtask

  // Vertical bars in RAW12: one-pixel bands, widths that are multiples of 8 and with a remainder.
  task automatic test_vertical_bars();
    run_frames(make_cfg(8,    8,  0, 0, 0, 0, 0, 0), 1, 1'b0);
    run_frames(make_cfg(64,   8,  0, 0, 0, 0, 0, 0), 2, 1'b1);
    run_frames(make_cfg(100,  9,  0, 0, 0, 0, 0, 0), 1, 1'b0);
    run_frames(make_cfg(76,   10, 0, 0, 0, 1, 0, 0), 1, 1'b0);
    run_frames(make_cfg(1920, 8,  0, 0, 0, 0, 0, 0), 1, 1'b0);
  endtask

  // Horizontal bars in RAW12: heights that are multiples of 8 and with a remainder.
  task automatic test_horizontal_bars();
    run_frames(make_cfg(16, 8,   1, 0, 0, 0, 0, 0), 1, 1'b0);
    run_frames(make_cfg(16, 16,  1, 0, 0, 0, 0, 0), 2, 1'b1);
    run_frames(make_cfg(24, 20,  1, 0, 0, 0, 0, 0), 1, 1'b0);
    run_frames(make_cfg(32, 13,  1, 0, 0, 2, 0, 0), 1, 1'b0);
    run_frames(make_cfg(8,  64,  1, 0, 0, 0, 0, 0), 1, 1'b0);
    run_frames(make_cfg(12, 100, 1, 0, 0, 3, 0, 0), 1, 1'b0);
  endtask

  // RAW10: components 0x3FF or 0, bits 11:10 at 0, px_dt_sel high.
  task automatic test_raw10_format();
    run_frames(make_cfg(64, 8,  0, 1, 0, 0, 0, 0), 2, 1'b1);
    run_frames(make_cfg(32, 16, 1, 1, 0, 0, 0, 0), 1, 1'b0);
    run_frames(make_cfg(48, 8,  0, 1, 0, 2, 0, 0), 1, 1'b0);
    run_frames(make_cfg(16, 9,  1, 1, 1, 3, 0, 0), 1, 1'b0);
    // RAW12 again: px_dt_sel returns to 0 with the next frame.
    run_frames(make_cfg(16, 8,  0, 0, 0, 0, 0, 0), 1, 1'b0);
  endtask

  // The four Bayer orders on both patterns and both formats.
  task automatic test_cfa_orders();
    for (int cfa = 0; cfa < 4; cfa++) begin
      for (int pat = 0; pat < 2; pat++) begin
        run_frames(make_cfg(32, 10, pat[0], cfa[0], 0, cfa, 0, 0), 1, 1'b0);
      end
    end
  endtask

  // Widths that are not a multiple of 4 are truncated; the line ends on the truncated width.
  task automatic test_width_truncation();
    int unsigned widths [12] = '{8, 9, 10, 11, 12, 13, 14, 15, 66, 67, 1023, 1022};
    foreach (widths[k]) run_frames(make_cfg(widths[k], 8, 0, 0, 0, 0, 0, 0), 1, 1'b0);
    run_frames(make_cfg(13, 9, 1, 1, 1, 1, 0, 0), 1, 1'b0);
  endtask

  // Width (after truncation) or height below 8 blocks the start; correcting it starts at once.
  task automatic test_dimension_guard();
    int unsigned bad_w [9] = '{0, 3, 4, 7, 7, 8, 100, 65535, 65535};
    int unsigned bad_h [9] = '{8, 8, 8, 8, 65535, 7, 0, 7, 0};
    i_tpg_enable = 1'b1;
    foreach (bad_w[k]) begin
      apply_cfg(make_cfg(bad_w[k], bad_h[k], 0, 0, 0, 0, 0, 0));
      expect_idle(30, $sformatf("guard, width=%0d height=%0d", bad_w[k], bad_h[k]));
    end
    apply_cfg(make_cfg(8, 8, 0, 0, 0, 0, 0, 0));
    expect_start_at(cyc + 3, "start once the size is corrected");
    stop_after_frame("stop");
    // Width 11 truncates to 8 and passes; width 7 truncates to 4 and is held back.
    apply_cfg(make_cfg(7, 8, 0, 0, 0, 0, 0, 0));
    i_tpg_enable = 1'b1;
    expect_idle(30, "guard, width=7");
    apply_cfg(make_cfg(11, 8, 0, 0, 0, 0, 0, 0));
    expect_start_at(cyc + 3, "start with width 11");
    stop_after_frame("stop");
  endtask

  // linebuf_empty holds back the start and the loop-back, never a frame in progress.
  task automatic test_linebuf_empty_wait();
    cfg_t c0;
    cfg_t c1;
    c0 = make_cfg(32, 8, 0, 0, 0, 0, 0, 0);
    c1 = make_cfg(32, 8, 0, 0, 0, 0, 3, 2);
    apply_cfg(c0);
    i_linebuf_empty = 1'b0;
    i_tpg_enable    = 1'b1;
    expect_idle(40, "start while Line Buffer is not empty");
    i_linebuf_empty = 1'b1;
    expect_start_at(cyc + 3, "start once Line Buffer is empty");
    // Random changes inside the frame do not stall it (checked by the monitor).
    while (frames_done == 0 || in_frame) begin
      i_linebuf_empty = 1'($urandom);
      tick(1);
    end
    // Loop-back held until Line Buffer is empty.
    i_linebuf_empty = 1'b0;
    expect_idle(60, "loop-back while Line Buffer is not empty");
    i_linebuf_empty = 1'b1;
    expect_start_at(cyc + 3, "loop-back once Line Buffer is empty");
    // With blanking: empty again before the blanking ends, so no delay.
    apply_cfg(c1);
    wait_frame_end("frame with c0");
    wait_frame_start("frame with c1");
    chk_frame_gap = 1'b1;
    wait_frame_end("first frame with c1");
    i_linebuf_empty = 1'b0;
    tick(5);
    i_linebuf_empty = 1'b1;
    wait_frame_end("second frame with c1");
    // Not empty past the blanking: the loop-back waits for it.
    chk_frame_gap   = 1'b0;
    i_linebuf_empty = 1'b0;
    expect_idle(frame_gap(c1) + 10, "loop-back after blanking while Line Buffer is not empty");
    i_linebuf_empty = 1'b1;
    expect_start_at(cyc + 3, "loop-back after blanking once Line Buffer is empty");
    stop_after_frame("stop");
  endtask

  // Drops run_en at one point of the second frame: the frame ends, none follows, and a restart
  // skips the remaining blanking. point < 0: -1 right after the last beat, -2 in the blanking.
  task automatic stop_at(input cfg_t c, input int point, input bit use_core);
    string what;
    what = $sformatf("stop at %0d with %s", point, use_core ? "core_enable" : "tpg_enable");
    apply_cfg(c);
    i_core_enable = 1'b1;
    i_tpg_enable  = 1'b1;
    expect_start_at(cyc + 3, {what, ": start"});
    wait_frame_start({what, ": second frame"});
    if (point >= 0) begin
      wait_beat(point, what);
    end else begin
      wait_frame_end({what, ": end of the second frame"});
      if (point == -2) tick(32'(c.hblank) + 4);
    end
    if (use_core) i_core_enable = 1'b0;
    else          i_tpg_enable  = 1'b0;
    finish_after_stop(what);
    if (use_core) i_core_enable = 1'b1;
    else          i_tpg_enable  = 1'b1;
    expect_start_at(cyc + 3, {what, ": restart"});
    stop_after_frame({what, ": final stop"});
  endtask

  // tpg_enable cleared at several points: the frame in progress is always completed.
  task automatic test_stop_at_frame_end();
    cfg_t c;
    cfg_t cd;
    c  = make_cfg(32, 8, 0, 0, 0, 0, 3, 2);
    cd = make_cfg(16, 8, 1, 0, 1, 2, 1, 1);
    stop_at(c, 0, 1'b0);
    stop_at(c, frame_beats(c) / 2, 1'b0);
    stop_at(c, frame_beats(c) - 2, 1'b0);
    stop_at(c, -1, 1'b0);
    stop_at(c, -2, 1'b0);
    stop_at(cd, 6 * beats_per_line(cd) + 1, 1'b0);
    stop_at(cd, 7 * beats_per_line(cd) + 1, 1'b0);
  endtask

  // core_enable cleared at several points: same behaviour as tpg_enable.
  task automatic test_core_enable_stop();
    cfg_t c;
    cfg_t cd;
    c  = make_cfg(24, 8, 1, 1, 0, 1, 2, 1);
    cd = make_cfg(16, 9, 0, 0, 1, 0, 0, 2);
    stop_at(c, 0, 1'b1);
    stop_at(c, frame_beats(c) / 2, 1'b1);
    stop_at(c, -1, 1'b1);
    stop_at(c, -2, 1'b1);
    stop_at(cd, 4 * beats_per_line(cd) + 2, 1'b1);
  endtask

  // run_en dropped and raised again: no frame is lost and the frame gaps stay the same.
  task automatic test_quick_reenable();
    cfg_t c;
    c = make_cfg(32, 8, 0, 0, 0, 0, 2, 1);
    apply_cfg(c);
    chk_frame_gap = 1'b1;
    i_tpg_enable  = 1'b1;
    expect_start_at(cyc + 3, "start");
    // Short drops inside a frame.
    for (int d = 1; d <= 4; d++) begin
      wait_beat(5 + 7 * d, "drop point");
      if (d % 2 == 1) i_tpg_enable  = 1'b0;
      else            i_core_enable = 1'b0;
      tick(d);
      i_tpg_enable  = 1'b1;
      i_core_enable = 1'b1;
      wait_frame_start("frame after a short drop");
    end
    // Dropped on the last line, back in the cycle after the last beat: the next frame follows
    // after the normal gap.
    wait_beat(frame_beats(c) - 3, "last line");
    i_tpg_enable = 1'b0;
    wait_frame_end("frame with run_en low");
    i_tpg_enable = 1'b1;
    wait_frame_start("frame after re-enable at the frame end");
    // Same, back one cycle later: the block is in IDLE and starts again without blanking.
    wait_beat(frame_beats(c) - 3, "last line");
    i_tpg_enable = 1'b0;
    wait_frame_end("frame with run_en low");
    chk_frame_gap = 1'b0;
    frame_gap_ref = 1'b0;
    tick(1);
    i_tpg_enable = 1'b1;
    expect_start_at(cyc + 3, "start after re-enable in IDLE");
    // Dropped for longer than a frame, then back: a new start.
    i_tpg_enable = 1'b0;
    finish_after_stop("long drop");
    i_tpg_enable = 1'b1;
    expect_start_at(cyc + 3, "start after a long drop");
    stop_after_frame("stop");
  endtask

  // Dual-VC line interleave: VC0 then VC1 per row, a full row pair even when stopped mid-pair.
  task automatic test_dual_vc_interleave();
    cfg_t c;
    run_frames(make_cfg(16, 8,  0, 0, 1, 0, 0, 0), 2, 1'b1);
    run_frames(make_cfg(32, 8,  1, 1, 1, 3, 0, 0), 2, 1'b1);
    run_frames(make_cfg(8,  9,  0, 0, 1, 1, 1, 1), 2, 1'b1);
    run_frames(make_cfg(24, 10, 1, 0, 1, 2, 3, 2), 2, 1'b1);
    // Stop requested on the VC0 line, then on the VC1 line of row 3.
    c = make_cfg(16, 8, 0, 1, 1, 1, 1, 0);
    for (int vc = 0; vc < 2; vc++) begin
      apply_cfg(c);
      i_tpg_enable = 1'b1;
      expect_start_at(cyc + 3, "start");
      wait_beat((6 + vc) * beats_per_line(c) + 1, "row 3");
      stop_after_frame($sformatf("stop on the VC%0d line of row 3", vc));
    end
  endtask

  // Field writes at any time while streaming: each frame uses the values of its latch cycle.
  task automatic test_per_frame_config();
    apply_cfg(random_cfg(1'b0));
    chk_frame_gap = 1'b1;
    ready_mode    = 1;
    ready_pct     = 80;
    i_tpg_enable  = 1'b1;
    expect_start_at(cyc + 3, "start");
    for (int f = 0; f < 16; f++) begin
      if (!in_frame) wait_frame_start("frame");
      wait_beat($urandom_range(0, frame_beats(frame_cfg) - 1), "write point");
      change_cfg();
      wait_frame_end("frame end");
      tick(f % 4);
      change_cfg();
    end
    ready_mode = 0;
    stop_after_frame("stop");
  endtask

  // Line gaps of tpg_hblank and frame periods of (lines + tpg_vblank) * line_period + 2.
  task automatic test_blanking_timing();
    run_frames(make_cfg(32, 8,  0, 0, 0, 0, 0,  0), 3, 1'b1);
    run_frames(make_cfg(32, 8,  0, 0, 0, 0, 1,  0), 3, 1'b1);
    run_frames(make_cfg(32, 8,  0, 0, 0, 0, 0,  1), 3, 1'b1);
    run_frames(make_cfg(32, 8,  0, 0, 0, 0, 3,  2), 3, 1'b1);
    run_frames(make_cfg(64, 8,  1, 0, 0, 1, 17, 5), 3, 1'b1);
    run_frames(make_cfg(16, 8,  0, 1, 1, 0, 2,  1), 3, 1'b1);
    run_frames(make_cfg(8,  8,  0, 0, 0, 0, 0,  3), 3, 1'b1);
    run_frames(make_cfg(12, 10, 1, 0, 1, 3, 5,  0), 3, 1'b1);
  endtask

  // px_ready low at random, for long periods and on the first, end-of-line and last beats.
  task automatic test_backpressure();
    apply_cfg(make_cfg(16, 8, 0, 0, 1, 0, 1, 1));
    chk_frame_gap = 1'b1;
    ready_mode    = 1;
    ready_pct     = 50;
    i_tpg_enable  = 1'b1;
    expect_start_at(cyc + 3, "start");
    wait_frame_end("frame, px_ready 50%");
    wait_frame_end("frame, px_ready 50%");
    ready_pct = 10;
    wait_frame_end("frame, px_ready 10%");
    apply_cfg(make_cfg(24, 9, 1, 1, 0, 3, 0, 0));
    ready_mode = 2;
    wait_frame_end("frame, long stalls");
    wait_frame_end("frame, long stalls");
    // Chosen beats held for 30 cycles.
    ready_mode = 3;
    i_px_ready = 1'b1;
    for (int k = 0; k < 3; k++) begin
      case (k)
        0:       do tick(1); while (!(o_px_valid && o_px_sof));
        1:       do tick(1); while (!(o_px_valid && o_px_eol && !o_px_sof));
        default: do tick(1); while (!(o_px_valid && in_frame && pos == frame_beats(frame_cfg) - 1));
      endcase
      i_px_ready = 1'b0;
      tick(30);
      i_px_ready = 1'b1;
    end
    ready_mode = 0;
    wait_frame_start("frame after the stalls");
    stop_after_frame("stop");
  endtask

  // soft_reset at several points: the frame is dropped, the next starts from row 0 and the
  // shadows keep their values until it is latched.
  task automatic test_soft_reset();
    cfg_t ca;
    cfg_t cb;
    cfg_t cz;
    ca = make_cfg(32, 8,  0, 0, 0, 0, 3, 1);
    cb = make_cfg(48, 12, 1, 1, 1, 2, 0, 0);
    cz = make_cfg(16, 8,  0, 1, 0, 1, 0, 0);
    apply_cfg(ca);
    i_tpg_enable = 1'b1;
    expect_start_at(cyc + 3, "start");
    // Inside a line, after a write: ca stays in the shadows until the restart latches cb.
    wait_beat(10, "inside a line");
    apply_cfg(cb);
    tick(2);
    soft_reset_restart(1, "restart after soft_reset inside a line");
    wait_beat(7, "inside a line");
    soft_reset_restart(3, "restart after a three-cycle soft_reset");
    // While a beat waits for px_ready.
    ready_mode = 3;
    wait_beat(5, "stall");
    i_px_ready = 1'b0;
    tick(4);
    soft_reset_restart(1, "restart after soft_reset during a stall");
    i_px_ready = 1'b1;
    ready_mode = 0;
    // In the line gap and in the vertical blanking.
    apply_cfg(ca);
    wait_beat(3, "inside a line");
    soft_reset_restart(1, "restart with ca");
    do tick(1); while (!line_gap_on);
    soft_reset_restart(1, "restart after soft_reset in the line gap");
    wait_frame_end("frame with ca");
    tick(5);
    soft_reset_restart(1, "restart after soft_reset in the vertical blanking");
    // Back-to-back frames: the cycle after the last beat, then the latch cycle.
    apply_cfg(cz);
    wait_beat(3, "inside a line");
    soft_reset_restart(1, "restart with cz");
    wait_frame_end("frame with cz");
    soft_reset_restart(1, "restart after soft_reset in the cycle after the last beat");
    wait_frame_end("frame with cz");
    tick(1);
    soft_reset_restart(1, "restart after soft_reset in the latch cycle");
    // While stopping: the frame is dropped and no frame follows.
    wait_beat(9, "inside a line");
    i_tpg_enable = 1'b0;
    tick(2);
    i_soft_reset = 1'b1;
    tick(1);
    i_soft_reset = 1'b0;
    expect_idle(60, "soft_reset while stopping");
    // In IDLE: no effect.
    i_soft_reset = 1'b1;
    tick(3);
    i_soft_reset = 1'b0;
    expect_idle(20, "soft_reset in IDLE");
    i_tpg_enable = 1'b1;
    expect_start_at(cyc + 3, "start after soft_reset in IDLE");
    stop_after_frame("stop");
  endtask

  // resetn between clock edges: outputs and shadows cleared at once, a new frame after release.
  task automatic test_async_resetn();
    cfg_t c;
    c = make_cfg(32, 8, 1, 1, 1, 1, 4, 1);
    apply_cfg(c);
    i_tpg_enable = 1'b1;
    expect_start_at(cyc + 3, "start");
    wait_beat(9, "inside a line");
    pulse_resetn(1.3);
    expect_start_at(cyc + 3, "restart after resetn inside a line");
    ready_mode = 3;
    wait_beat(4, "stall");
    i_px_ready = 1'b0;
    tick(3);
    pulse_resetn(0.7);
    expect_start_at(cyc + 3, "restart after resetn during a stall");
    i_px_ready = 1'b1;
    ready_mode = 0;
    do tick(1); while (!line_gap_on);
    pulse_resetn(2.1);
    expect_start_at(cyc + 3, "restart after resetn in the line gap");
    wait_frame_end("frame");
    pulse_resetn(1.9);
    expect_start_at(cyc + 3, "restart after resetn in the blanking after the last beat");
    stop_after_frame("stop");
  endtask

  // Largest field values: line period at the 17-bit limit, 65535 rows, 65535 blank lines.
  task automatic test_max_dimensions();
    run_frames(make_cfg(16'hFFFF, 8,        0, 0, 0, 0, 16'hFFFF, 2),        2, 1'b1);
    run_frames(make_cfg(8,        16'hFFFF, 1, 1, 0, 2, 0,        0),        1, 1'b0);
    run_frames(make_cfg(16'hFFFD, 8,        0, 1, 1, 3, 0,        1),        2, 1'b1);
    run_frames(make_cfg(8,        8,        0, 0, 0, 0, 0,        16'hFFFF), 2, 1'b1);
  endtask

  // Random writes, px_ready, Line Buffer state, run_en drops and resets for 40000 cycles.
  bit rnd_run = 1'b0;

  task automatic test_random_streaming();
    apply_cfg(random_cfg(1'b1));
    ready_mode   = 1;
    ready_pct    = 70;
    i_tpg_enable = 1'b1;
    rnd_run      = 1'b1;
    fork
      begin
        while (rnd_run) begin
          tick(1);
          if ($urandom_range(1, 150) == 1) begin
            if ($urandom_range(0, 3) == 0) apply_cfg(random_cfg(1'b1));
            else                           change_cfg();
          end
        end
      end
      begin
        while (rnd_run) begin
          tick($urandom_range(500, 3000));
          ready_mode = $urandom_range(0, 2);
          ready_pct  = $urandom_range(10, 100);
        end
      end
      begin
        while (rnd_run) begin
          tick($urandom_range(50, 600));
          i_linebuf_empty = 1'b0;
          tick($urandom_range(1, 40));
          i_linebuf_empty = 1'b1;
        end
      end
      begin
        while (rnd_run) begin
          tick($urandom_range(500, 4000));
          if ($urandom_range(0, 1) == 1) i_tpg_enable  = 1'b0;
          else                           i_core_enable = 1'b0;
          tick($urandom_range(1, 600));
          i_tpg_enable  = 1'b1;
          i_core_enable = 1'b1;
        end
      end
      begin
        while (rnd_run) begin
          tick($urandom_range(2000, 8000));
          while (rnd_run && !o_px_valid) tick(1);
          if (rnd_run) begin
            i_soft_reset = 1'b1;
            tick(1);
            i_soft_reset = 1'b0;
          end
        end
      end
      begin
        tick($urandom_range(5000, 30000));
        if ($urandom_range(0, 1) == 1) pulse_resetn(0.1 * $urandom_range(1, 24));
      end
    join_none
    tick(40000);
    rnd_run = 1'b0;
    disable fork;
    i_resetn        = 1'b1;
    i_soft_reset    = 1'b0;
    i_linebuf_empty = 1'b1;
    i_core_enable   = 1'b1;
    ready_mode      = 0;
    if (frames_done < 4) report_error($sformatf("only %0d frames completed", frames_done));
  endtask

  // Forces a wrong px_data; this test must end in FAIL with errors in the log.
  // It confirms that the monitor catches a fault and is not part of the regression.
  task automatic test_checker_fault();
    apply_cfg(make_cfg(32, 8, 0, 0, 0, 0, 0, 0));
    i_tpg_enable = 1'b1;
    expect_start_at(cyc + 3, "start");
    wait_beat(5, "fault point");
    force o_px_data = 48'h0123_4567_89AB;
    tick(2);
    release o_px_data;
    stop_after_frame("stop");
  endtask

  // ----------------------------------------------------------------------------------------------
  // Test selection and result
  // ----------------------------------------------------------------------------------------------
  string       test_name;
  int unsigned seed;

  initial begin
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "none";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    void'($urandom(seed));
    i_resetn        = 1'b0;
    i_soft_reset    = 1'b0;
    i_core_enable   = 1'b1;
    i_tpg_enable    = 1'b0;
    i_linebuf_empty = 1'b1;
    i_px_ready      = 1'b1;
    apply_cfg(make_cfg(32, 8, 0, 0, 0, 0, 0, 0));
    tick(4);
    i_resetn = 1'b1;
    tick(2);
    case (test_name)
      "test_reset_state":        test_reset_state();
      "test_vertical_bars":      test_vertical_bars();
      "test_horizontal_bars":    test_horizontal_bars();
      "test_raw10_format":       test_raw10_format();
      "test_cfa_orders":         test_cfa_orders();
      "test_width_truncation":   test_width_truncation();
      "test_dimension_guard":    test_dimension_guard();
      "test_linebuf_empty_wait": test_linebuf_empty_wait();
      "test_stop_at_frame_end":  test_stop_at_frame_end();
      "test_core_enable_stop":   test_core_enable_stop();
      "test_quick_reenable":     test_quick_reenable();
      "test_dual_vc_interleave": test_dual_vc_interleave();
      "test_per_frame_config":   test_per_frame_config();
      "test_blanking_timing":    test_blanking_timing();
      "test_backpressure":       test_backpressure();
      "test_soft_reset":         test_soft_reset();
      "test_async_resetn":       test_async_resetn();
      "test_max_dimensions":     test_max_dimensions();
      "test_random_streaming":   test_random_streaming();
      "test_checker_fault":      test_checker_fault();
      default: report_error($sformatf("unknown test '%s'", test_name));
    endcase
    tick(20);
    $display("frames started=%0d done=%0d dropped=%0d, beats=%0d, edges checked=%0d",
             frames_started, frames_done, frames_aborted, beats_accepted, check_count);
    if (err_count == 0 && beats_accepted > 0) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d)", test_name, err_count);
    $finish;
  end

  // Timeout.
  initial begin
    #100_000_000.0;
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
