// File        : tb_top.sv
// Description : Testbench of m_fpt_byte_to_pixel (Byte-to-Pixel Unpacking).
//               A cycle model written from the specification equations predicts every output at
//               every rising edge of sys_clk. A line checker rebuilds each line from the bytes
//               sent and the line-end rules, and compares the pixels and flags of every line.
//               Rule checkers test the output relations that hold for any traffic, the reset
//               values, the asynchronous reset, X/Z and the absence of combinational paths.
//               Tests drive stimulus and check counts and flags at given cycles.
// Run         : +TEST=<name> +SEED=<n> [+N_LINES=<n>]; the last log line is "<name> PASS" or
//               "<name> FAIL".
//
// Design assumptions (decided by the block owner; the specification does not state them yet):
//   - Every register is clocked on the rising edge of sys_clk and samples its inputs there.
//   - soft_reset is active high and synchronous. It wins over the beat of the same cycle: that
//     beat is discarded and gives no px_eol.
//   - resetn (at once) and soft_reset (at the edge) set every output to 0, px_data, px_vc_tag
//     and px_dt_sel included.
//   - The output register loads every cycle: px_vc_tag and px_dt_sel are the bp_vc_tag and
//     bp_dt_sel of the previous cycle, also on cycles without pixels.
//   - A line that ends mid-unit ends with its last whole units and the partial bytes are
//     dropped. Only when the last beat leaves no whole unit do the bytes on hand form one
//     zero-padded unit.
//   - A last beat without payload byte pads the bytes still carried (all zero when none).
//   - After a bp_eol beat nothing is carried, whatever follows. A line whose first beat has no
//     bp_sol is unpacked from an empty carry and gets no px_sol.
//   - The producer keeps bp_byte_en contiguous, sends 4 bytes on every beat but the last of a
//     line, and holds bp_dt_sel and bp_vc_tag from the bp_sol beat to the bp_eol beat, idle
//     cycles included. A few tests send on purpose a last beat without payload byte or a line
//     without bp_sol.
//   - The sys_clk frequency is not fixed yet; the testbench uses 200 MHz, which changes no
//     behaviour.
`timescale 1ns/1ps

module tb_top;

  // sys_clk period, 200 MHz.
  localparam real T_CLK = 5.0;

  // Error messages printed at most; every error is still counted.
  localparam int MAX_PRINT = 30;

  // Lines of test_random_traffic when +N_LINES is not given.
  localparam int RANDOM_LINES = 2000;

  typedef logic [11:0] pixel_t;
  typedef logic [7:0]  byte_t;

  // ----------------------------------------------------------------------------------------------
  // DUT
  // ----------------------------------------------------------------------------------------------
  logic        i_sys_clk;
  logic        i_resetn;
  logic        i_soft_reset;
  logic [31:0] i_bp_data;
  logic        i_bp_valid;
  logic [3:0]  i_bp_byte_en;
  logic        i_bp_sof;
  logic        i_bp_sol;
  logic        i_bp_eol;
  logic [1:0]  i_bp_vc_tag;
  logic        i_bp_dt_sel;

  logic [47:0] o_px_data;
  logic        o_px_valid;
  logic [2:0]  o_px_valid_count;
  logic        o_px_sof;
  logic        o_px_sol;
  logic        o_px_eol;
  logic [1:0]  o_px_vc_tag;
  logic        o_px_dt_sel;
  logic        o_err_wc_align;

  m_fpt_byte_to_pixel u_dut (
    .i_sys_clk        (i_sys_clk),
    .i_resetn         (i_resetn),
    .i_soft_reset     (i_soft_reset),
    .i_bp_data        (i_bp_data),
    .i_bp_valid       (i_bp_valid),
    .i_bp_byte_en     (i_bp_byte_en),
    .i_bp_sof         (i_bp_sof),
    .i_bp_sol         (i_bp_sol),
    .i_bp_eol         (i_bp_eol),
    .i_bp_vc_tag      (i_bp_vc_tag),
    .i_bp_dt_sel      (i_bp_dt_sel),
    .o_px_data        (o_px_data),
    .o_px_valid       (o_px_valid),
    .o_px_valid_count (o_px_valid_count),
    .o_px_sof         (o_px_sof),
    .o_px_sol         (o_px_sol),
    .o_px_eol         (o_px_eol),
    .o_px_vc_tag      (o_px_vc_tag),
    .o_px_dt_sel      (o_px_dt_sel),
    .o_err_wc_align   (o_err_wc_align)
  );

  initial begin
    i_sys_clk = 1'b0;
    forever #(T_CLK / 2.0) i_sys_clk = ~i_sys_clk;
  end

  // ----------------------------------------------------------------------------------------------
  // Bookkeeping
  // ----------------------------------------------------------------------------------------------
  int unsigned err_count      = 0;
  int unsigned edge_checks    = 0;
  int unsigned px_compared    = 0;
  int unsigned lines_compared = 0;
  int          cyc            = 0;     // rising edges so far
  realtime     t_edge         = 0.0;   // time of the last rising edge
  bit          started        = 1'b0;  // the first reset has defined the outputs

  task automatic report_error(input string msg);
    err_count++;
    if (err_count <= MAX_PRINT) $error("edge %0d: %s", cyc, msg);
  endtask

  // ----------------------------------------------------------------------------------------------
  // Cycle model
  // ----------------------------------------------------------------------------------------------
  // State: Carry Buffer, carry_cnt and the two sync pending flags.
  logic [31:0] m_carry_buf = '0;
  int          m_carry_cnt = 0;
  bit          m_sof_pend  = 1'b0;
  bit          m_sol_pend  = 1'b0;
  // Outputs expected after the last rising edge.
  logic [47:0] x_data      = '0;
  logic [2:0]  x_cnt       = '0;
  logic        x_sof       = 1'b0;
  logic        x_sol       = 1'b0;
  logic        x_eol       = 1'b0;
  logic        x_err       = 1'b0;
  logic [1:0]  x_vc        = '0;
  logic        x_dt        = 1'b0;

  task automatic model_clear();
    m_carry_buf = '0;
    m_carry_cnt = 0;
    m_sof_pend  = 1'b0;
    m_sol_pend  = 1'b0;
    x_data      = '0;
    x_cnt       = '0;
    x_sof       = 1'b0;
    x_sol       = 1'b0;
    x_eol       = 1'b0;
    x_err       = 1'b0;
    x_vc        = '0;
    x_dt        = 1'b0;
  endtask

  // One rising edge with the inputs sampled there: byte mask, carry read with line_start, work
  // buffer, units of K bytes, pad unit on line_end, RAW12/RAW10 slices, carry written back
  // (cleared on line_end) and the sync pending flags.
  task automatic model_step();
    int          k;
    int          ppu;
    bit          line_start;
    bit          line_end;
    int          new_bytes;
    logic [31:0] data_m;
    int          carry_in;
    logic [31:0] carry_bytes;
    logic [63:0] work;
    int          work_cnt;
    int          units;
    int          rem;
    int          units_out;
    bit          emit;
    bit          sof_n;
    bit          sol_n;
    byte_t       b [8];
    logic [47:0] lanes;
    if ((i_resetn !== 1'b1) || (i_soft_reset === 1'b1)) begin
      model_clear();
      return;
    end
    k          = i_bp_dt_sel ? 5 : 3;
    ppu        = i_bp_dt_sel ? 4 : 2;
    line_start = i_bp_valid && i_bp_sol;
    line_end   = i_bp_valid && i_bp_eol;
    new_bytes  = 0;
    data_m     = '0;
    for (int i = 0; i < 4; i++) begin
      if (i_bp_valid && i_bp_byte_en[i]) begin
        data_m[8*i +: 8] = i_bp_data[8*i +: 8];
        new_bytes++;
      end
    end
    carry_in    = line_start ? 0 : m_carry_cnt;
    carry_bytes = line_start ? 32'd0 : m_carry_buf;
    work        = {32'd0, carry_bytes} | ({32'd0, data_m} << (8 * carry_in));
    work_cnt    = carry_in + new_bytes;
    units       = work_cnt / k;
    rem         = work_cnt % k;
    units_out   = (line_end && (units == 0)) ? 1 : units;
    emit        = (units_out != 0);
    for (int n = 0; n < 8; n++) b[n] = work[8*n +: 8];
    lanes = '0;
    if (!i_bp_dt_sel) begin
      for (int u = 0; u < units_out; u++) begin
        lanes[24*u +: 12]      = {b[3*u],     b[3*u+2][3:0]};
        lanes[24*u + 12 +: 12] = {b[3*u + 1], b[3*u+2][7:4]};
      end
    end else if (emit) begin
      for (int j = 0; j < 4; j++) lanes[12*j +: 12] = {2'b00, b[j], b[4][2*j +: 2]};
    end
    sof_n       = (i_bp_valid && i_bp_sof) || m_sof_pend;
    sol_n       = line_start || m_sol_pend;
    x_data      = lanes;
    x_cnt       = 3'(units_out * ppu);
    x_sof       = emit && sof_n;
    x_sol       = emit && sol_n;
    x_eol       = line_end;
    x_err       = line_end && ((rem != 0) || (units == 0));
    x_vc        = i_bp_vc_tag;
    x_dt        = i_bp_dt_sel;
    m_sof_pend  = sof_n && !x_sof;
    m_sol_pend  = sol_n && !x_sol;
    m_carry_buf = line_end ? 32'd0 : 32'(work >> (8 * k * units_out));
    m_carry_cnt = line_end ? 0 : rem;
  endtask

  // ----------------------------------------------------------------------------------------------
  // Output checkers
  // ----------------------------------------------------------------------------------------------
  // Inputs of the previous rising edge, for the rules that relate an output to its beat.
  // pv_kind: 0 resetn low (or fallen since), 1 soft_reset, 2 normal.
  int          pv_kind  = 0;
  bit          pv_valid = 1'b0;
  bit          pv_eol   = 1'b0;
  logic [1:0]  pv_vc    = '0;
  bit          pv_dt    = 1'b0;

  function automatic logic [58:0] out_bus();
    return {o_px_data, o_px_valid, o_px_valid_count, o_px_sof, o_px_sol, o_px_eol, o_px_vc_tag,
            o_px_dt_sel, o_err_wc_align};
  endfunction

  // Outputs against the cycle model, then the rules that hold for any traffic.
  task automatic check_outputs();
    logic [58:0] got;
    logic [58:0] exp;
    got = out_bus();
    if ($isunknown(got)) begin
      report_error($sformatf("X or Z on an output: %b", got));
      return;
    end
    exp = {x_data, (x_cnt != 3'd0), x_cnt, x_sof, x_sol, x_eol, x_vc, x_dt, x_err};
    if (got !== exp) begin
      report_error($sformatf("px_data %012h count %0d sof/sol/eol %b%b%b vc %0d dt %b err %b",
                             o_px_data, o_px_valid_count, o_px_sof, o_px_sol, o_px_eol,
                             o_px_vc_tag, o_px_dt_sel, o_err_wc_align));
      report_error($sformatf("expected %012h count %0d sof/sol/eol %b%b%b vc %0d dt %b err %b",
                             x_data, x_cnt, x_sof, x_sol, x_eol, x_vc, x_dt, x_err));
    end
    if (o_px_valid != (o_px_valid_count != 3'd0))
      report_error("px_valid does not match px_valid_count");
    if (!(o_px_valid_count inside {3'd0, 3'd2, 3'd4}))
      report_error($sformatf("px_valid_count %0d is not 0, 2 or 4", o_px_valid_count));
    if (o_px_dt_sel && (o_px_valid_count == 3'd2))
      report_error("two pixels on a RAW10 cycle");
    if ((o_px_valid_count == 3'd2) && (o_px_data[47:24] != 24'd0))
      report_error("lanes 2 and 3 are not 0 with two pixels");
    if ((o_px_valid_count == 3'd0) && (o_px_data != 48'd0))
      report_error("px_data is not 0 on a cycle without pixels");
    if (o_px_dt_sel && ({o_px_data[47:46], o_px_data[35:34], o_px_data[23:22],
                         o_px_data[11:10]} != 8'd0))
      report_error("bits 11:10 of a RAW10 lane are not 0");
    if ((o_px_sof || o_px_sol || o_px_eol) && (o_px_valid_count == 3'd0))
      report_error("px_sof, px_sol or px_eol on a cycle without pixels");
    if (o_err_wc_align && !o_px_eol)
      report_error("err_wc_align without px_eol");
    if (pv_kind != 2) begin
      if (got != 59'd0) report_error("outputs are not 0 after resetn or soft_reset");
    end else begin
      if (o_px_eol != pv_eol)
        report_error("px_eol does not follow a bp_eol beat of the previous cycle");
      if ((o_px_vc_tag != pv_vc) || (o_px_dt_sel != pv_dt))
        report_error("px_vc_tag or px_dt_sel is not the input of the previous cycle");
      if (!pv_valid && (o_px_valid || o_err_wc_align))
        report_error("pixels or err_wc_align after an idle input cycle");
    end
  endtask

  // Per-edge log, indexed by the edge that sampled the input: {sof, sol, eol, err}, {vc, dt}.
  logic [47:0] log_data [int];
  logic [2:0]  log_cnt  [int];
  logic [3:0]  log_flag [int];
  logic [2:0]  log_vcdt [int];

  // resetn clears the outputs at once, without waiting for a clock edge.
  always @(negedge i_resetn) begin
    pv_kind = 0;
    model_clear();
    if (started) begin
      #0.1;
      if (out_bus() !== 59'd0) report_error("resetn did not clear the outputs at once");
    end
  end

  // Outputs move only at a rising edge of sys_clk, or when resetn falls.
  always @(o_px_data or o_px_valid or o_px_valid_count or o_px_sof or o_px_sol or o_px_eol or
           o_px_vc_tag or o_px_dt_sel or o_err_wc_align) begin
    if (started && (i_resetn === 1'b1) && ($realtime != t_edge))
      report_error("an output changed between clock edges");
  end

  // ----------------------------------------------------------------------------------------------
  // Line checker
  // ----------------------------------------------------------------------------------------------
  // One line as seen on the outputs (or expected): pixels, flags of its first output, end.
  typedef struct {
    int         len;
    bit         sof;
    bit         sol;
    bit         term;
    bit         err;
    logic [1:0] vc;
    bit         dt;
  } rec_t;

  pixel_t act_px  [$];
  rec_t   act_rec [$];
  rec_t   cur;
  bit     cur_open = 1'b0;

  pixel_t exp_px  [$];
  rec_t   exp_rec [$];
  pixel_t tmp_px  [$];
  bit     exp_sof_pend = 1'b0;   // bp_sof of a line that gave no pixel yet
  bit     exp_sol_pend = 1'b0;   // bp_sol of a line that gave no pixel yet

  task automatic line_close(input bit term, input bit err);
    cur.term = term;
    cur.err  = err;
    act_rec.push_back(cur);
    cur_open = 1'b0;
  endtask

  // A line starts with px_sol, or with the first output after a closed line; px_eol ends it.
  task automatic line_collect();
    int n;
    n = int'(o_px_valid_count);
    if (o_px_sol || !cur_open) begin
      if (cur_open) line_close(1'b0, 1'b0);
      cur_open = 1'b1;
      cur.len  = 0;
      cur.sof  = o_px_sof;
      cur.sol  = o_px_sol;
      cur.vc   = o_px_vc_tag;
      cur.dt   = o_px_dt_sel;
    end else begin
      if (o_px_sof) report_error("px_sof on an output that is not the first of a line");
      if ((o_px_vc_tag != cur.vc) || (o_px_dt_sel != cur.dt))
        report_error("px_vc_tag or px_dt_sel changed inside a line");
    end
    for (int i = 0; (i < n) && (i < 4); i++) act_px.push_back(o_px_data[12*i +: 12]);
    cur.len = cur.len + n;
    if (o_px_eol) line_close(1'b1, o_err_wc_align);
  endtask

  // Bytes of the line being sent, enabled bytes of each beat, edge that sampled each beat.
  byte_t ln_bytes  [$];
  int    ln_beats  [$];
  int    beat_edge [$];

  // One unit from line byte 'base'; bytes past the bytes received read 0.
  task automatic exp_unit(input int base, input bit dt);
    byte_t u [5];
    for (int i = 0; i < 5; i++) u[i] = ((base + i) < ln_bytes.size()) ? ln_bytes[base + i] : 8'd0;
    if (!dt) begin
      tmp_px.push_back({u[0], u[2][3:0]});
      tmp_px.push_back({u[1], u[2][7:4]});
    end else begin
      for (int j = 0; j < 4; j++) tmp_px.push_back({2'b00, u[j], u[4][2*j +: 2]});
    end
  endtask

  // Expected record of the line just sent. Whole units come out as their last byte arrives. On
  // the bp_eol beat the whole units come out and the partial bytes are dropped (error when there
  // are any); when no whole unit is on hand, one zero-padded unit comes out (error).
  task automatic exp_line(input bit dt, input logic [1:0] vc, input bit sof, input bit sol,
                          input bit term);
    int   k;
    int   carry;
    int   pos;
    int   work;
    int   units;
    bit   err;
    rec_t r;
    k     = dt ? 5 : 3;
    carry = 0;
    pos   = 0;
    err   = 1'b0;
    tmp_px.delete();
    for (int bi = 0; bi < ln_beats.size(); bi++) begin
      work  = carry + ln_beats[bi];
      units = work / k;
      if (term && (bi == ln_beats.size() - 1)) begin
        if (units == 0) begin
          exp_unit(pos, dt);
          err = 1'b1;
        end else begin
          for (int u = 0; u < units; u++) exp_unit(pos + u * k, dt);
          err = ((work % k) != 0);
        end
      end else begin
        for (int u = 0; u < units; u++) exp_unit(pos + u * k, dt);
        pos   = pos + units * k;
        carry = work - units * k;
      end
    end
    if (tmp_px.size() > 0) begin
      foreach (tmp_px[i]) exp_px.push_back(tmp_px[i]);
      r.len  = tmp_px.size();
      r.sof  = sof || exp_sof_pend;
      r.sol  = sol || exp_sol_pend;
      r.term = term;
      r.err  = term && err;
      r.vc   = vc;
      r.dt   = dt;
      exp_rec.push_back(r);
      exp_sof_pend = 1'b0;
      exp_sol_pend = 1'b0;
    end else begin
      exp_sof_pend = exp_sof_pend || sof;
      exp_sol_pend = exp_sol_pend || sol;
    end
  endtask

  task automatic lines_clear();
    act_px.delete();
    act_rec.delete();
    exp_px.delete();
    exp_rec.delete();
    cur_open     = 1'b0;
    exp_sof_pend = 1'b0;
    exp_sol_pend = 1'b0;
  endtask

  // Compares the lines seen with the lines expected since the last call, then clears both.
  task automatic check_lines(input string tag);
    int n;
    if (cur_open) line_close(1'b0, 1'b0);
    if (act_rec.size() != exp_rec.size())
      report_error($sformatf("%s: %0d lines seen, %0d expected", tag, act_rec.size(),
                             exp_rec.size()));
    n = (act_rec.size() < exp_rec.size()) ? act_rec.size() : exp_rec.size();
    for (int r = 0; r < n; r++) begin
      if (act_rec[r].len != exp_rec[r].len)
        report_error($sformatf("%s line %0d: %0d pixels, expected %0d", tag, r,
                               act_rec[r].len, exp_rec[r].len));
      if ((act_rec[r].sof != exp_rec[r].sof) || (act_rec[r].sol != exp_rec[r].sol))
        report_error($sformatf("%s line %0d: px_sof/px_sol %b%b on its first output, expected %b%b",
                               tag, r, act_rec[r].sof, act_rec[r].sol, exp_rec[r].sof,
                               exp_rec[r].sol));
      if ((act_rec[r].term != exp_rec[r].term) || (act_rec[r].err != exp_rec[r].err))
        report_error($sformatf("%s line %0d: px_eol/err_wc_align %b%b, expected %b%b", tag, r,
                               act_rec[r].term, act_rec[r].err, exp_rec[r].term,
                               exp_rec[r].err));
      if ((act_rec[r].vc != exp_rec[r].vc) || (act_rec[r].dt != exp_rec[r].dt))
        report_error($sformatf("%s line %0d: px_vc_tag/px_dt_sel %0d/%b, expected %0d/%b", tag,
                               r, act_rec[r].vc, act_rec[r].dt, exp_rec[r].vc, exp_rec[r].dt));
    end
    if (act_px.size() != exp_px.size())
      report_error($sformatf("%s: %0d pixels seen, %0d expected", tag, act_px.size(),
                             exp_px.size()));
    for (int i = 0; (i < act_px.size()) && (i < exp_px.size()); i++) begin
      if (act_px[i] !== exp_px[i]) begin
        report_error($sformatf("%s: pixel %0d is %03h, expected %03h", tag, i, act_px[i],
                               exp_px[i]));
        break;
      end
    end
    px_compared    = px_compared + exp_px.size();
    lines_compared = lines_compared + exp_rec.size();
    $display("%s: %0d lines, %0d pixels compared", tag, exp_rec.size(), exp_px.size());
    lines_clear();
  endtask

  // ----------------------------------------------------------------------------------------------
  // Monitor: every rising edge sees the outputs of the cycle that ends there
  // ----------------------------------------------------------------------------------------------
  always @(posedge i_sys_clk) begin : p_monitor
    cyc    = cyc + 1;
    t_edge = $realtime;
    if (!started && (cyc >= 2) && (i_resetn === 1'b0)) started = 1'b1;
    if (started) begin
      edge_checks++;
      check_outputs();
      log_data[cyc - 1] = o_px_data;
      log_cnt[cyc - 1]  = o_px_valid_count;
      log_flag[cyc - 1] = {o_px_sof, o_px_sol, o_px_eol, o_err_wc_align};
      log_vcdt[cyc - 1] = {o_px_vc_tag, o_px_dt_sel};
      if ((i_resetn === 1'b1) && (o_px_valid === 1'b1)) line_collect();
    end
    model_step();
    if (i_resetn !== 1'b1)          pv_kind = 0;
    else if (i_soft_reset === 1'b1) pv_kind = 1;
    else                            pv_kind = 2;
    pv_valid = i_bp_valid;
    pv_eol   = i_bp_valid && i_bp_eol;
    pv_vc    = i_bp_vc_tag;
    pv_dt    = i_bp_dt_sel;
  end

  // ----------------------------------------------------------------------------------------------
  // Stimulus helpers; every input changes right after a falling edge
  // ----------------------------------------------------------------------------------------------
  bit         in_line   = 1'b0;   // a line was started and not ended: vc and dt are held
  logic [1:0] cur_vc    = '0;
  bit         cur_dt    = 1'b0;
  int         last_edge = 0;      // edge that samples the input driven last

  task automatic drive(input bit valid, input logic [31:0] data, input logic [3:0] be,
                       input bit sof, input bit sol, input bit eol, input logic [1:0] vc,
                       input bit dt, input bit sr);
    @(negedge i_sys_clk);
    i_bp_valid   = valid;
    i_bp_data    = data;
    i_bp_byte_en = be;
    i_bp_sof     = sof;
    i_bp_sol     = sol;
    i_bp_eol     = eol;
    i_bp_vc_tag  = vc;
    i_bp_dt_sel  = dt;
    i_soft_reset = sr;
    last_edge    = cyc + 1;
  endtask

  // Idle cycle; with garbage the ignored inputs carry random values, and between lines
  // bp_vc_tag and bp_dt_sel change too.
  task automatic drive_idle(input bit garbage);
    if (garbage) begin
      if (!in_line) begin
        cur_vc = 2'($urandom);
        cur_dt = 1'($urandom);
      end
      drive(1'b0, $urandom, 4'($urandom), 1'($urandom), 1'($urandom), 1'($urandom), cur_vc,
            cur_dt, 1'b0);
    end else begin
      drive(1'b0, 32'd0, 4'd0, 1'b0, 1'b0, 1'b0, cur_vc, cur_dt, 1'b0);
    end
  endtask

  task automatic idle(input int n);
    repeat (n) drive_idle(1'b0);
  endtask

  task automatic fill_random(input int wc);
    ln_bytes.delete();
    for (int i = 0; i < wc; i++) ln_bytes.push_back(8'($urandom));
  endtask

  // Sends ln_bytes as one line: beats of 4 bytes, the last one partial; disabled byte lanes
  // carry random data.
  //   sol       : bp_sol on the first beat
  //   term      : bp_eol on the last beat (0: the line is abandoned)
  //   gap_max   : random idle cycles (0 to gap_max) between beats
  //   empty_eol : bp_eol on an extra beat without payload byte
  //   sof_beat2 : bp_sof also on the second beat
  task automatic send_prepared(input bit dt, input logic [1:0] vc, input bit sof, input bit sol,
                               input bit term, input int gap_max, input bit garbage,
                               input bit empty_eol, input bit sof_beat2);
    int          pos;
    int          nb;
    int          bi;
    bit          last;
    logic [31:0] data;
    logic [3:0]  be;
    pos = 0;
    bi  = 0;
    ln_beats.delete();
    beat_edge.delete();
    in_line = 1'b1;
    cur_vc  = vc;
    cur_dt  = dt;
    while (pos < ln_bytes.size()) begin
      nb   = ((ln_bytes.size() - pos) >= 4) ? 4 : (ln_bytes.size() - pos);
      last = ((pos + nb) >= ln_bytes.size()) && term && !empty_eol;
      data = $urandom;
      be   = 4'd0;
      for (int i = 0; i < nb; i++) begin
        data[8*i +: 8] = ln_bytes[pos + i];
        be[i]          = 1'b1;
      end
      drive(1'b1, data, be, ((bi == 0) && sof) || ((bi == 1) && sof_beat2), (bi == 0) && sol,
            last, vc, dt, 1'b0);
      beat_edge.push_back(last_edge);
      ln_beats.push_back(nb);
      pos = pos + nb;
      bi  = bi + 1;
      if ((gap_max > 0) && ((pos < ln_bytes.size()) || empty_eol))
        repeat ($urandom_range(gap_max, 0)) drive_idle(garbage);
    end
    if (empty_eol) begin
      drive(1'b1, $urandom, 4'b0000, (bi == 0) && sof, (bi == 0) && sol, 1'b1, vc, dt, 1'b0);
      beat_edge.push_back(last_edge);
      ln_beats.push_back(0);
    end
    in_line = !term;
    exp_line(dt, vc, sof, sol, term);
  endtask

  task automatic send_line(input int wc, input bit dt, input logic [1:0] vc, input bit sof,
                           input bit term, input int gap_max, input bit garbage);
    fill_random(wc);
    send_prepared(dt, vc, sof, 1'b1, term, gap_max, garbage, 1'b0, 1'b0);
  endtask

  // soft_reset on n idle cycles, then one idle cycle; pending requests are dropped.
  task automatic soft_reset_idle(input int n);
    repeat (n) drive(1'b0, 32'd0, 4'd0, 1'b0, 1'b0, 1'b0, cur_vc, cur_dt, 1'b1);
    drive_idle(1'b0);
    in_line      = 1'b0;
    exp_sof_pend = 1'b0;
    exp_sol_pend = 1'b0;
  endtask

  // Called right after a falling edge: resetn falls offset ns later (after_pos = 0) or offset ns
  // after the next rising edge (after_pos = 1), and the producer stops at once; resetn is
  // released on a falling edge 3 cycles later.
  task automatic async_resetn(input bit after_pos, input real offset);
    if (after_pos) @(posedge i_sys_clk);
    #(offset);
    i_resetn     = 1'b0;
    i_bp_valid   = 1'b0;
    i_bp_sof     = 1'b0;
    i_bp_sol     = 1'b0;
    i_bp_eol     = 1'b0;
    i_soft_reset = 1'b0;
    repeat (3) @(negedge i_sys_clk);
    i_resetn = 1'b1;
    in_line  = 1'b0;
    lines_clear();
  endtask

  // Count, flags {sof, sol, eol, err} and data of the output of the input sampled at edge e.
  task automatic expect_out(input int e, input logic [2:0] cnt, input logic [3:0] flags,
                            input string tag);
    if (!log_cnt.exists(e))
      report_error($sformatf("%s: no output logged for edge %0d", tag, e));
    else if ((log_cnt[e] !== cnt) || (log_flag[e] !== flags))
      report_error($sformatf("%s: count %0d flags %b, expected count %0d flags %b", tag,
                             log_cnt[e], log_flag[e], cnt, flags));
  endtask

  task automatic expect_data(input int e, input logic [47:0] d, input string tag);
    if (!log_data.exists(e))
      report_error($sformatf("%s: no output logged for edge %0d", tag, e));
    else if (log_data[e] !== d)
      report_error($sformatf("%s: px_data %012h, expected %012h", tag, log_data[e], d));
  endtask

  // Every output 0, px_vc_tag and px_dt_sel included.
  task automatic expect_zero(input int e, input string tag);
    expect_out(e, 3'd0, 4'b0000, tag);
    expect_data(e, 48'd0, tag);
    if (log_vcdt.exists(e) && (log_vcdt[e] !== 3'd0))
      report_error($sformatf("%s: px_vc_tag/px_dt_sel %b, expected 0", tag, log_vcdt[e]));
  endtask

  // Logged edges in [e0, e1] with flag idx set (3 sof, 2 sol, 1 eol, 0 err).
  function automatic int count_flag(input int e0, input int e1, input int idx);
    int n;
    n = 0;
    for (int e = e0; e <= e1; e++) if (log_flag.exists(e) && log_flag[e][idx]) n++;
    return n;
  endfunction

  // RAW12 unit of b0, b1, b2 on lanes 0 and 1.
  function automatic logic [23:0] raw12_pair(input byte_t b0, input byte_t b1, input byte_t b2);
    return {b1, b2[7:4], b0, b2[3:0]};
  endfunction

  // RAW10 unit of the line bytes from base, lanes 0 to 3.
  function automatic logic [47:0] raw10_unit(input int base);
    logic [47:0] d;
    for (int j = 0; j < 4; j++)
      d[12*j +: 12] = {2'b00, ln_bytes[base + j], ln_bytes[base + 4][2*j +: 2]};
    return d;
  endfunction

  // ----------------------------------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------------------------------

  // Outputs at 0 after resetn, then one RAW12 and one RAW10 line.
  task automatic test_reset_values();
    if (out_bus() !== 59'd0) report_error("outputs are not 0 after resetn");
    idle(2);
    send_line(24, 1'b0, 2'd2, 1'b1, 1'b1, 0, 1'b0);
    send_line(40, 1'b1, 2'd1, 1'b0, 1'b1, 0, 1'b0);
    idle(3);
    check_lines("reset values");
  endtask

  // resetn between edges: with pixels on the outputs, while idle with a carry and a pending
  // bp_sof, before a beat without bp_sol, before the edge of a bp_eol beat, with px_eol shown.
  task automatic test_async_resetn();
    int e;
    send_line(12, 1'b0, 2'd3, 1'b1, 1'b1, 0, 1'b0);
    idle(2);
    check_lines("line before resetn");
    // RAW12 line cut after 8 bytes (two units out, carry 2); resetn while pixels are shown.
    fill_random(8);
    send_prepared(1'b0, 2'd3, 1'b1, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    drive_idle(1'b0);
    if (o_px_valid !== 1'b1) report_error("pixels expected on the outputs before resetn");
    async_resetn(1'b0, 0.7);
    // RAW10 first beat only (carry 4, bp_sof pending); resetn in the high phase of sys_clk.
    fill_random(4);
    send_prepared(1'b1, 2'd1, 1'b1, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    idle(2);
    async_resetn(1'b1, 1.2);
    send_line(20, 1'b1, 2'd1, 1'b0, 1'b1, 0, 1'b0);
    idle(2);
    check_lines("no px_sof after resetn");
    // RAW12 beat of 4 bytes (carry 1), resetn, then a line without bp_sol: empty carry.
    fill_random(4);
    send_prepared(1'b0, 2'd0, 1'b0, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    drive_idle(1'b0);
    async_resetn(1'b0, 1.9);
    fill_random(6);
    send_prepared(1'b0, 2'd0, 1'b0, 1'b0, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(2);
    expect_out(beat_edge[0], 3'd2, 4'b0000, "first beat after resetn");
    expect_data(beat_edge[0], {24'd0, raw12_pair(ln_bytes[0], ln_bytes[1], ln_bytes[2])},
                "first beat after resetn, empty carry");
    check_lines("line without bp_sol after resetn");
    // RAW12 single-beat line; resetn falls before the edge that would sample it.
    fill_random(3);
    send_prepared(1'b0, 2'd1, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    e = last_edge;
    #1.0;
    i_resetn   = 1'b0;
    i_bp_valid = 1'b0;
    i_bp_sol   = 1'b0;
    i_bp_eol   = 1'b0;
    repeat (3) @(negedge i_sys_clk);
    i_resetn = 1'b1;
    in_line  = 1'b0;
    lines_clear();
    idle(3);
    if (count_flag(e, cyc, 1) != 0) report_error("px_eol for a bp_eol beat cut by resetn");
    // RAW10 single-beat line; resetn while its px_eol is on the outputs.
    fill_random(4);
    send_prepared(1'b1, 2'd2, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    drive_idle(1'b0);
    if (o_px_eol !== 1'b1) report_error("px_eol expected on the outputs before resetn");
    async_resetn(1'b0, 1.0);
    send_line(24, 1'b0, 2'd2, 1'b1, 1'b1, 0, 1'b0);
    idle(2);
    check_lines("line after resetn");
  endtask

  // soft_reset idle, inside a line with a carry, with a pending bp_sof, on the bp_eol beat,
  // for three cycles of beats, and before a beat without bp_sol.
  task automatic test_soft_reset();
    int e;
    send_line(12, 1'b0, 2'd2, 1'b1, 1'b1, 0, 1'b0);
    idle(2);
    check_lines("line before soft_reset");
    drive(1'b0, 32'd0, 4'd0, 1'b0, 1'b0, 1'b0, 2'd2, 1'b0, 1'b1);
    e = last_edge;
    idle(2);
    expect_zero(e, "soft_reset on an idle cycle");
    // RAW10 line cut after 12 bytes (carry 2), soft_reset, new line.
    fill_random(12);
    send_prepared(1'b1, 2'd1, 1'b1, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    soft_reset_idle(1);
    send_line(20, 1'b1, 2'd1, 1'b0, 1'b1, 0, 1'b0);
    idle(2);
    check_lines("soft_reset inside a line");
    // RAW10 first beat with bp_sof (no pixel), soft_reset, a line without bp_sof.
    fill_random(4);
    send_prepared(1'b1, 2'd0, 1'b1, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    soft_reset_idle(2);
    send_line(20, 1'b1, 2'd0, 1'b0, 1'b1, 0, 1'b0);
    idle(2);
    check_lines("soft_reset with a pending bp_sof");
    // soft_reset on the bp_eol beat of a RAW12 line.
    drive(1'b1, $urandom, 4'b1111, 1'b0, 1'b1, 1'b0, 2'd3, 1'b0, 1'b0);
    drive(1'b1, $urandom, 4'b1111, 1'b0, 1'b0, 1'b0, 2'd3, 1'b0, 1'b0);
    drive(1'b1, $urandom, 4'b1111, 1'b0, 1'b0, 1'b1, 2'd3, 1'b0, 1'b1);
    e = last_edge;
    idle(4);
    expect_zero(e, "soft_reset on the bp_eol beat");
    if (count_flag(e, e + 3, 1) != 0) report_error("px_eol for a beat discarded by soft_reset");
    lines_clear();
    // soft_reset for three cycles with beats carrying bp_sof, bp_sol and bp_eol.
    drive(1'b1, $urandom, 4'b1111, 1'b1, 1'b1, 1'b0, 2'd1, 1'b0, 1'b1);
    e = last_edge;
    drive(1'b1, $urandom, 4'b1111, 1'b0, 1'b0, 1'b0, 2'd1, 1'b0, 1'b1);
    drive(1'b1, $urandom, 4'b1111, 1'b0, 1'b0, 1'b1, 2'd1, 1'b0, 1'b1);
    cur_vc = 2'd1;
    cur_dt = 1'b0;
    idle(3);
    for (int i = 0; i < 3; i++) expect_zero(e + i, "three-cycle soft_reset with beats");
    expect_out(e + 3, 3'd0, 4'b0000, "idle cycle after the three-cycle soft_reset");
    lines_clear();
    // RAW12 beat of 4 bytes (carry 1), soft_reset, then a line without bp_sol: empty carry.
    fill_random(4);
    send_prepared(1'b0, 2'd0, 1'b0, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    soft_reset_idle(1);
    lines_clear();
    fill_random(6);
    send_prepared(1'b0, 2'd0, 1'b0, 1'b0, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(2);
    expect_data(beat_edge[0], {24'd0, raw12_pair(ln_bytes[0], ln_bytes[1], ln_bytes[2])},
                "first beat after soft_reset, empty carry");
    check_lines("line without bp_sol after soft_reset");
    send_line(24, 1'b0, 2'd1, 1'b1, 1'b1, 0, 1'b0);
    idle(2);
    check_lines("line after soft_reset");
  endtask

  // RAW12 lines of full beats, WC 12, 24 and 120.
  task automatic test_raw12_aligned();
    int wcs [3] = '{12, 24, 120};
    send_line(12, 1'b0, 2'd0, 1'b1, 1'b1, 0, 1'b0);
    idle(3);
    expect_out(beat_edge[0],     3'd2, 4'b1100, "RAW12 WC 12 beat 1");
    expect_out(beat_edge[1],     3'd2, 4'b0000, "RAW12 WC 12 beat 2");
    expect_out(beat_edge[2],     3'd4, 4'b0010, "RAW12 WC 12 beat 3");
    expect_out(beat_edge[2] + 1, 3'd0, 4'b0000, "idle cycle after the line");
    for (int i = 0; i < 3; i++) begin
      send_line(wcs[i], 1'b0, 2'(i), 1'b0, 1'b1, 0, 1'b0);
      idle(1);
    end
    idle(2);
    check_lines("RAW12 aligned");
  endtask

  // RAW10 lines of full beats, WC 20, 40 and 200; px_sof and px_sol move to beat 2.
  task automatic test_raw10_aligned();
    int wcs [3] = '{20, 40, 200};
    send_line(20, 1'b1, 2'd1, 1'b1, 1'b1, 0, 1'b0);
    idle(3);
    expect_out(beat_edge[0],     3'd0, 4'b0000, "RAW10 WC 20 beat 1");
    expect_out(beat_edge[1],     3'd4, 4'b1100, "RAW10 WC 20 beat 2");
    expect_out(beat_edge[2],     3'd4, 4'b0000, "RAW10 WC 20 beat 3");
    expect_out(beat_edge[3],     3'd4, 4'b0000, "RAW10 WC 20 beat 4");
    expect_out(beat_edge[4],     3'd4, 4'b0010, "RAW10 WC 20 beat 5");
    expect_out(beat_edge[4] + 1, 3'd0, 4'b0000, "idle cycle after the line");
    for (int i = 0; i < 3; i++) begin
      send_line(wcs[i], 1'b1, 2'(i + 1), 1'b0, 1'b1, 0, 1'b0);
      idle(1);
    end
    idle(2);
    check_lines("RAW10 aligned");
  endtask

  // RAW12 WC a multiple of 3 but not of 4: partial last beat, no err_wc_align.
  task automatic test_raw12_partial_last_beat();
    int s;
    int wcs [5] = '{3, 9, 15, 21, 33};
    s = cyc + 1;
    for (int i = 0; i < 5; i++) begin
      send_line(wcs[i], 1'b0, 2'(i), (i == 0), 1'b1, 0, 1'b0);
      idle(1);
    end
    idle(2);
    if (count_flag(s, cyc, 0) != 0) report_error("err_wc_align on a line with WC a multiple of 3");
    if (count_flag(s, cyc, 1) != 5) report_error("px_eol count differs from the 5 lines");
    check_lines("RAW12 partial last beat");
  endtask

  // RAW10 WC a multiple of 5 but not of 4: partial last beat, no err_wc_align.
  task automatic test_raw10_partial_last_beat();
    int s;
    int wcs [5] = '{5, 10, 15, 25, 35};
    s = cyc + 1;
    for (int i = 0; i < 5; i++) begin
      send_line(wcs[i], 1'b1, 2'(i), (i == 0), 1'b1, 0, 1'b0);
      idle(1);
    end
    idle(2);
    if (count_flag(s, cyc, 0) != 0) report_error("err_wc_align on a line with WC a multiple of 5");
    if (count_flag(s, cyc, 1) != 5) report_error("px_eol count differs from the 5 lines");
    check_lines("RAW10 partial last beat");
  endtask

  // Line ends with whole units and a remainder: partial bytes dropped, err_wc_align.
  task automatic test_misaligned_whole_units();
    int s;
    int wcs12 [3] = '{7, 8, 10};
    int wcs10 [3] = '{8, 12, 16};
    s = cyc + 1;
    send_line(8, 1'b1, 2'd0, 1'b0, 1'b1, 0, 1'b0);
    idle(3);
    expect_out(beat_edge[0], 3'd0, 4'b0000, "RAW10 WC 8 beat 1");
    expect_out(beat_edge[1], 3'd4, 4'b0111, "RAW10 WC 8 bp_eol beat");
    expect_data(beat_edge[1], raw10_unit(0), "RAW10 WC 8 unit of bytes 0 to 4");
    for (int i = 0; i < 3; i++) begin
      send_line(wcs12[i], 1'b0, 2'(i), 1'b0, 1'b1, 0, 1'b0);
      idle(1);
    end
    for (int i = 0; i < 3; i++) begin
      send_line(wcs10[i], 1'b1, 2'(i), 1'b0, 1'b1, 0, 1'b0);
      idle(1);
    end
    // An aligned line afterwards: no dropped byte crosses the line boundary.
    send_line(12, 1'b0, 2'd3, 1'b0, 1'b1, 0, 1'b0);
    idle(3);
    if (count_flag(s, cyc, 0) != 7) report_error("err_wc_align count differs from 7");
    if (count_flag(s, cyc, 1) != 8) report_error("px_eol count differs from 8");
    check_lines("misaligned with whole units");
  endtask

  // Line ends with no whole unit: one zero-padded unit with px_eol and err_wc_align.
  task automatic test_misaligned_pad_unit();
    int s;
    int wcs12 [3] = '{1, 2, 5};
    int wcs10 [6] = '{1, 2, 3, 4, 9, 13};
    s = cyc + 1;
    // RAW10 bytes 5A C3 with bp_sof: {5A, 00} = 168, {C3, 00} = 30C, lanes 2 and 3 = 0.
    ln_bytes.delete();
    ln_bytes.push_back(8'h5A);
    ln_bytes.push_back(8'hC3);
    send_prepared(1'b1, 2'd0, 1'b1, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(2);
    expect_out(beat_edge[0], 3'd4, 4'b1111, "RAW10 WC 2 padded unit");
    expect_data(beat_edge[0], 48'h000000_30C168, "RAW10 WC 2 padded pixels");
    // RAW12 bytes AB CD: {AB, 0} = AB0, {CD, 0} = CD0.
    ln_bytes.delete();
    ln_bytes.push_back(8'hAB);
    ln_bytes.push_back(8'hCD);
    send_prepared(1'b0, 2'd1, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(2);
    expect_out(beat_edge[0], 3'd2, 4'b0111, "RAW12 WC 2 padded unit");
    expect_data(beat_edge[0], 48'h000000_CD0AB0, "RAW12 WC 2 padded pixels");
    for (int i = 0; i < 3; i++) begin
      send_line(wcs12[i], 1'b0, 2'(i), 1'b0, 1'b1, 0, 1'b0);
      idle(1);
    end
    for (int i = 0; i < 6; i++) begin
      send_line(wcs10[i], 1'b1, 2'(i), 1'b0, 1'b1, 0, 1'b0);
      idle(1);
    end
    idle(2);
    if (count_flag(s, cyc, 0) != 11) report_error("err_wc_align count differs from 11");
    check_lines("misaligned pad unit");
  endtask

  // bp_eol on a beat without payload byte, with an empty and with a non-empty carry.
  task automatic test_eol_without_payload();
    int s;
    s = cyc + 1;
    // RAW12, 12 bytes, empty carry: one all-zero unit.
    fill_random(12);
    send_prepared(1'b0, 2'd0, 1'b1, 1'b1, 1'b1, 0, 1'b0, 1'b1, 1'b0);
    idle(2);
    expect_out(beat_edge[3], 3'd2, 4'b0011, "RAW12 empty bp_eol beat, empty carry");
    expect_data(beat_edge[3], 48'd0, "RAW12 all-zero unit");
    // RAW12 bytes 12 34 56 78: unit 126/345, carry 78; then {78, 0} = 780 and 000.
    ln_bytes.delete();
    ln_bytes.push_back(8'h12);
    ln_bytes.push_back(8'h34);
    ln_bytes.push_back(8'h56);
    ln_bytes.push_back(8'h78);
    send_prepared(1'b0, 2'd1, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b1, 1'b0);
    idle(2);
    expect_out(beat_edge[0], 3'd2, 4'b0100, "RAW12 beat 12 34 56 78");
    expect_data(beat_edge[0], 48'h000000_345126, "RAW12 unit 12 34 56");
    expect_out(beat_edge[1], 3'd2, 4'b0011, "RAW12 empty bp_eol beat, carry 1");
    expect_data(beat_edge[1], 48'h000000_000780, "RAW12 padded unit of byte 78");
    // RAW10, 20 bytes, empty carry: one all-zero unit.
    fill_random(20);
    send_prepared(1'b1, 2'd2, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b1, 1'b0);
    idle(2);
    expect_out(beat_edge[5], 3'd4, 4'b0011, "RAW10 empty bp_eol beat, empty carry");
    expect_data(beat_edge[5], 48'd0, "RAW10 all-zero unit");
    // RAW10 bytes 01 02 03 04 05 9C AB CD: unit 005/009/00C/010, carry 9C AB CD; then the
    // padded unit 270/2AC/334/000.
    ln_bytes.delete();
    ln_bytes.push_back(8'h01);
    ln_bytes.push_back(8'h02);
    ln_bytes.push_back(8'h03);
    ln_bytes.push_back(8'h04);
    ln_bytes.push_back(8'h05);
    ln_bytes.push_back(8'h9C);
    ln_bytes.push_back(8'hAB);
    ln_bytes.push_back(8'hCD);
    send_prepared(1'b1, 2'd3, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b1, 1'b0);
    idle(3);
    expect_out(beat_edge[1], 3'd4, 4'b0100, "RAW10 second beat");
    expect_data(beat_edge[1], 48'h01000C_009005, "RAW10 unit 01 to 05");
    expect_out(beat_edge[2], 3'd4, 4'b0011, "RAW10 empty bp_eol beat, carry 3");
    expect_data(beat_edge[2], 48'h000334_2AC270, "RAW10 padded unit of 9C AB CD");
    if (count_flag(s, cyc, 0) != 4) report_error("err_wc_align count differs from 4");
    if (count_flag(s, cyc, 1) != 4) report_error("px_eol count differs from 4");
    check_lines("eol without payload");
  endtask

  // Random idle gaps of 0 to 3 cycles between beats, both formats, every carry value.
  task automatic test_idle_gaps();
    int wcs12 [3] = '{60, 23, 7};
    int wcs10 [3] = '{60, 23, 8};
    for (int rep = 0; rep < 3; rep++) begin
      for (int i = 0; i < 3; i++) begin
        send_line(wcs12[i], 1'b0, 2'(rep), (i == 0), 1'b1, 3, 1'b0);
        idle(1);
        send_line(wcs10[i], 1'b1, 2'(rep), 1'b0, 1'b1, 3, 1'b0);
        idle(1);
      end
    end
    idle(2);
    check_lines("idle gaps");
  endtask

  // One-beat lines (bp_sol and bp_eol together) of 1 to 4 bytes, both formats.
  task automatic test_single_beat_lines();
    bit err;
    for (int dt = 0; dt < 2; dt++) begin
      for (int nb = 1; nb <= 4; nb++) begin
        send_line(nb, 1'(dt), 2'(nb - 1), (nb == 1), 1'b1, 0, 1'b0);
        err = (dt == 1) || (nb != 3);
        idle(2);
        expect_out(beat_edge[0], (dt == 1) ? 3'd4 : 3'd2, {(nb == 1), 1'b1, 1'b1, err},
                   $sformatf("single beat of %0d bytes, dt %0d", nb, dt));
      end
    end
    idle(1);
    check_lines("single-beat lines");
  endtask

  // A line left without bp_eol, then a new bp_sol beat while bytes are carried.
  task automatic test_line_start_safety_net();
    int s;
    s = cyc + 1;
    // RAW12 cut after 8 bytes (carry 2), new RAW12 line.
    fill_random(8);
    send_prepared(1'b0, 2'd0, 1'b1, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    send_line(12, 1'b0, 2'd0, 1'b0, 1'b1, 0, 1'b0);
    idle(2);
    // RAW10 cut after 12 bytes (carry 2), new RAW10 line.
    fill_random(12);
    send_prepared(1'b1, 2'd1, 1'b0, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    send_line(20, 1'b1, 2'd1, 1'b0, 1'b1, 0, 1'b0);
    idle(2);
    // RAW10 cut after its first beat with bp_sof (no pixel): px_sof comes with the new line.
    fill_random(4);
    send_prepared(1'b1, 2'd2, 1'b1, 1'b1, 1'b0, 0, 1'b0, 1'b0, 1'b0);
    send_line(20, 1'b1, 2'd2, 1'b0, 1'b1, 0, 1'b0);
    idle(3);
    if (count_flag(s, cyc, 0) != 0) report_error("err_wc_align on a line start without bp_eol");
    if (count_flag(s, cyc, 1) != 3) report_error("px_eol count differs from the 3 ended lines");
    check_lines("line start safety net");
  endtask

  // The remainder of a full first beat is kept for the second beat.
  task automatic test_first_beat_remainder();
    byte_t b12 [12] = '{8'h11, 8'h22, 8'h33, 8'h44, 8'h55, 8'h66, 8'h77, 8'h88, 8'h99, 8'hAA,
                        8'hBB, 8'hCC};
    byte_t b10 [20] = '{8'hA0, 8'hB1, 8'hC2, 8'hD3, 8'hE4, 8'hF5, 8'h06, 8'h17, 8'h28, 8'h39,
                        8'h4A, 8'h5B, 8'h6C, 8'h7D, 8'h8E, 8'h9F, 8'hA0, 8'hB1, 8'hC2, 8'hD3};
    // RAW12: 113/223, then 446/556 from the carried byte 44, then 779/889/AAC/BBC.
    ln_bytes.delete();
    foreach (b12[i]) ln_bytes.push_back(b12[i]);
    send_prepared(1'b0, 2'd0, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(3);
    expect_out(beat_edge[0], 3'd2, 4'b0100, "RAW12 beat 1");
    expect_data(beat_edge[0], 48'h000000_223113, "RAW12 beat 1 pixels");
    expect_out(beat_edge[1], 3'd2, 4'b0000, "RAW12 beat 2");
    expect_data(beat_edge[1], 48'h000000_556446, "RAW12 beat 2 pixels from the remainder");
    expect_out(beat_edge[2], 3'd4, 4'b0010, "RAW12 beat 3");
    expect_data(beat_edge[2], 48'hBBCAAC_889779, "RAW12 beat 3 pixels");
    // RAW10: no pixel, then 280/2C5/30A/34F with the four carried bytes, then 3D5/01A/05F/0A0.
    ln_bytes.delete();
    foreach (b10[i]) ln_bytes.push_back(b10[i]);
    send_prepared(1'b1, 2'd1, 1'b1, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(3);
    expect_out(beat_edge[0], 3'd0, 4'b0000, "RAW10 beat 1");
    expect_data(beat_edge[0], 48'd0, "RAW10 beat 1 pixels");
    expect_out(beat_edge[1], 3'd4, 4'b1100, "RAW10 beat 2");
    expect_data(beat_edge[1], 48'h34F30A_2C5280, "RAW10 beat 2 pixels from the remainder");
    expect_out(beat_edge[2], 3'd4, 4'b0000, "RAW10 beat 3");
    expect_data(beat_edge[2], 48'h0A005F_01A3D5, "RAW10 beat 3 pixels");
    check_lines("first beat remainder");
  endtask

  // px_sof and px_sol held until the first output with pixels.
  task automatic test_sync_pending();
    int e0;
    int e2;
    // RAW10 WC 20, bp_sof on the first beat, which gives no pixel.
    send_line(20, 1'b1, 2'd0, 1'b1, 1'b1, 0, 1'b0);
    e0 = beat_edge[0];
    idle(3);
    expect_out(e0,     3'd0, 4'b0000, "RAW10 first beat without pixel");
    expect_out(e0 + 1, 3'd4, 4'b1100, "RAW10 px_sof and px_sol with beat 2");
    if (count_flag(e0 + 2, e0 + 4, 3) != 0) report_error("extra px_sof in the RAW10 line");
    // RAW12 WC 12: px_sof and px_sol with the first pixels.
    send_line(12, 1'b0, 2'd1, 1'b1, 1'b1, 0, 1'b0);
    idle(2);
    expect_out(beat_edge[0], 3'd2, 4'b1100, "RAW12 px_sof and px_sol with beat 1");
    // RAW10 WC 2: px_sof, px_sol and px_eol on the single padded output.
    send_line(2, 1'b1, 2'd2, 1'b1, 1'b1, 0, 1'b0);
    idle(2);
    expect_out(beat_edge[0], 3'd4, 4'b1111, "single padded output with every flag");
    // RAW10 WC 20 with bp_sof on beats 1 and 2: one px_sof.
    fill_random(20);
    send_prepared(1'b1, 2'd3, 1'b1, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b1);
    e0 = beat_edge[0];
    idle(3);
    expect_out(e0 + 1, 3'd4, 4'b1100, "repeated bp_sof released with beat 2");
    if (count_flag(e0, e0 + 4, 3) != 1) report_error("repeated bp_sof gave more than one px_sof");
    // RAW10 WC 20: beat 1 with bp_sof and bp_sol, three idle cycles, beats 2 to 5.
    fill_random(20);
    ln_beats.delete();
    beat_edge.delete();
    in_line = 1'b1;
    cur_vc  = 2'd0;
    cur_dt  = 1'b1;
    for (int bi = 0; bi < 5; bi++) begin
      drive(1'b1, {ln_bytes[4*bi + 3], ln_bytes[4*bi + 2], ln_bytes[4*bi + 1], ln_bytes[4*bi]},
            4'b1111, (bi == 0), (bi == 0), (bi == 4), 2'd0, 1'b1, 1'b0);
      beat_edge.push_back(last_edge);
      ln_beats.push_back(4);
      if (bi == 0) idle(3);
    end
    in_line = 1'b0;
    exp_line(1'b1, 2'd0, 1'b1, 1'b1, 1'b1);
    idle(3);
    e0 = beat_edge[0];
    e2 = beat_edge[1];
    for (int e = e0; e < e2; e++) expect_out(e, 3'd0, 4'b0000, "no flag before beat 2");
    expect_out(e2, 3'd4, 4'b1100, "px_sof and px_sol with beat 2 after the gap");
    // A line without bp_sof: no px_sof.
    send_line(12, 1'b0, 2'd1, 1'b0, 1'b1, 0, 1'b0);
    idle(3);
    check_lines("sync pending");
  endtask

  // bp_vc_tag 0 to 3 and bp_dt_sel between lines and on idle cycles; VC does not change pixels.
  task automatic test_vc_dt_passthrough();
    for (int i = 0; i < 8; i++) begin
      send_line(((i % 2) == 1) ? 20 : 12, 1'(i % 2), 2'(i % 4), (i == 0), 1'b1, 0, 1'b0);
      repeat (3) drive_idle(1'b1);
    end
    fill_random(24);
    send_prepared(1'b0, 2'd0, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(1);
    send_prepared(1'b0, 2'd3, 1'b0, 1'b1, 1'b1, 0, 1'b0, 1'b0, 1'b0);
    idle(3);
    check_lines("vc and dt pass-through");
  endtask

  // Lines with no idle cycle between a bp_eol beat and the next bp_sol beat, formats mixed.
  task automatic test_back_to_back_lines();
    int dts [10] = '{0, 1, 0, 1, 0, 1, 1, 0, 1, 0};
    int wcs [10] = '{12, 20, 7, 8, 5, 9, 40, 24, 13, 3};
    for (int i = 0; i < 10; i++) send_line(wcs[i], 1'(dts[i]), 2'(i), (i == 0), 1'b1, 0, 1'b0);
    idle(3);
    check_lines("back-to-back lines");
  endtask

  // Random values on every input that bp_valid = 0 makes irrelevant, inside and between lines.
  task automatic test_idle_inputs_ignored();
    for (int i = 0; i < 6; i++) begin
      send_line(((i % 2) == 1) ? 40 : 36, 1'(i % 2), 2'(i % 4), (i == 0), 1'b1, 2, 1'b1);
      repeat (5) drive_idle(1'b1);
    end
    send_line(13, 1'b1, 2'd2, 1'b0, 1'b1, 2, 1'b1);
    repeat (5) drive_idle(1'b1);
    send_line(5, 1'b0, 2'd1, 1'b0, 1'b1, 2, 1'b1);
    idle(3);
    check_lines("idle inputs ignored");
  endtask

  // Sensor line lengths: RAW12 WC 5760, RAW10 WC 4800 and 3360, bp_eol on the last payload word.
  task automatic test_sensor_line_lengths();
    int s;
    int total;
    int zero;
    int wcs [3] = '{5760, 4800, 3360};
    int dts [3] = '{0, 1, 1};
    int pxs [3] = '{3840, 3840, 2688};
    for (int i = 0; i < 3; i++) begin
      s = cyc + 1;
      send_line(wcs[i], 1'(dts[i]), 2'd0, 1'b1, 1'b1, 0, 1'b0);
      idle(3);
      total = 0;
      for (int e = s; e < cyc; e++) if (log_cnt.exists(e)) total = total + int'(log_cnt[e]);
      zero = 0;
      foreach (beat_edge[b]) if (log_cnt[beat_edge[b]] == 3'd0) zero++;
      if (total != pxs[i])
        report_error($sformatf("WC %0d gave %0d pixels, expected %0d", wcs[i], total, pxs[i]));
      if (zero != (dts[i] ? beat_edge.size() / 5 : 0))
        report_error($sformatf("WC %0d: %0d beats without pixel", wcs[i], zero));
      if (count_flag(s, cyc, 0) != 0) report_error("err_wc_align on a sensor line");
      if (count_flag(s, cyc, 1) != 1) report_error("px_eol count of a sensor line differs from 1");
      check_lines($sformatf("sensor line WC %0d", wcs[i]));
    end
  endtask

  // A line ends, then the next line comes without bp_sol on its first beat, both formats, line
  // ends aligned or not, directly or after two idle cycles; it is unpacked from an empty carry.
  task automatic test_missing_sol_after_eol();
    int s;
    int e0;
    int wc_end;
    bit sof;
    s = cyc + 1;
    for (int dt = 0; dt < 2; dt++) begin
      for (int gap = 0; gap < 2; gap++) begin
        for (int part = 0; part < 2; part++) begin
          wc_end = (dt == 1) ? ((part == 1) ? 8 : 20) : ((part == 1) ? 7 : 12);
          sof    = (gap == 1) && (part == 0);
          send_line(wc_end, 1'(dt), 2'(gap * 2 + part), 1'b0, 1'b1, 0, 1'b0);
          if (gap == 1) idle(2);
          fill_random((dt == 1) ? 20 : 12);
          send_prepared(1'(dt), 2'(gap * 2 + part), sof, 1'b0, 1'b1, 0, 1'b0, 1'b0, 1'b0);
          e0 = beat_edge[0];
          idle(2);
          if (dt == 0) begin
            expect_out(e0, 3'd2, {sof, 3'b000},
                       $sformatf("RAW12 first beat without bp_sol, gap %0d part %0d", gap, part));
            expect_data(e0, {24'd0, raw12_pair(ln_bytes[0], ln_bytes[1], ln_bytes[2])},
                        $sformatf("RAW12 pixels from an empty carry, gap %0d part %0d", gap,
                                  part));
          end else begin
            expect_out(e0, 3'd0, 4'b0000,
                       $sformatf("RAW10 first beat without bp_sol, gap %0d part %0d", gap, part));
            expect_out(e0 + 1, 3'd4, {sof, 3'b000},
                       $sformatf("RAW10 second beat, gap %0d part %0d", gap, part));
            expect_data(e0 + 1, raw10_unit(0),
                        $sformatf("RAW10 pixels from an empty carry, gap %0d part %0d", gap,
                                  part));
          end
        end
      end
    end
    send_line(24, 1'b0, 2'd3, 1'b0, 1'b1, 0, 1'b0);
    idle(3);
    if (count_flag(s, cyc, 2) != 9)  report_error("px_sol count differs from 9");
    if (count_flag(s, cyc, 3) != 2)  report_error("px_sof count differs from 2");
    if (count_flag(s, cyc, 1) != 17) report_error("px_eol count differs from 17");
    if (count_flag(s, cyc, 0) != 4)  report_error("err_wc_align count differs from 4");
    check_lines("missing bp_sol after bp_eol");
  endtask

  // Seeded random traffic: format, VC, word count, bp_sof, gaps, values on idle cycles; a few
  // lines with a last beat without payload byte, abandoned lines, soft_reset after them.
  task automatic test_random_traffic();
    int         n_lines;
    int         k;
    int         wc;
    int         kind;
    int         gaps;
    bit         dt;
    bit         sof;
    bit         garbage;
    logic [1:0] vc;
    int         n_err;
    if (!$value$plusargs("N_LINES=%d", n_lines)) n_lines = RANDOM_LINES;
    n_err = 0;
    for (int l = 0; l < n_lines; l++) begin
      dt      = 1'($urandom);
      vc      = 2'($urandom);
      k       = dt ? 5 : 3;
      wc      = ($urandom_range(99, 0) < 50) ? (k * $urandom_range(30, 1)) : $urandom_range(80, 1);
      sof     = ($urandom_range(7, 0) == 0);
      gaps    = $urandom_range(3, 0);
      garbage = 1'($urandom);
      kind    = $urandom_range(99, 0);
      if (kind < 4) begin
        fill_random(((wc + 3) / 4) * 4);
        send_prepared(dt, vc, sof, 1'b1, 1'b1, gaps, garbage, 1'b1, 1'b0);
      end else if (kind < 10) begin
        send_line(((wc + 3) / 4) * 4, dt, vc, sof, 1'b0, gaps, garbage);
      end else if (kind < 13) begin
        send_line(((wc + 3) / 4) * 4, dt, vc, sof, 1'b0, gaps, garbage);
        soft_reset_idle($urandom_range(2, 1));
      end else if ((kind < 16) && dt) begin
        // bp_sof repeated on beat 2, before the RAW10 first beat has given a pixel.
        fill_random(wc);
        send_prepared(dt, vc, 1'b1, 1'b1, 1'b1, gaps, garbage, 1'b0, 1'b1);
      end else begin
        send_line(wc, dt, vc, sof, 1'b1, gaps, garbage);
      end
      repeat ($urandom_range(2, 0)) drive_idle(garbage);
    end
    idle(3);
    foreach (exp_rec[r]) if (exp_rec[r].err) n_err++;
    $display("random traffic: %0d lines sent, %0d expected with err_wc_align", n_lines, n_err);
    check_lines("random traffic");
  endtask

  // Forces a wrong px_data for two cycles inside a line; this test must end in FAIL with errors.
  // It confirms that the checkers catch a fault and is not part of the regression.
  task automatic test_checker_fault();
    fork
      send_line(48, 1'b0, 2'd0, 1'b1, 1'b1, 0, 1'b0);
      begin
        repeat (6) @(posedge i_sys_clk);
        force o_px_data = 48'h0123_4567_89AB;
        repeat (2) @(posedge i_sys_clk);
        release o_px_data;
      end
    join
    idle(3);
    check_lines("checker fault");
  endtask

  // ----------------------------------------------------------------------------------------------
  // Test selection and result
  // ----------------------------------------------------------------------------------------------
  string test_name;
  int    seed;

  initial begin : p_main
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "none";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    void'($urandom(seed));
    i_resetn     = 1'b0;
    i_soft_reset = 1'b0;
    i_bp_data    = 32'd0;
    i_bp_valid   = 1'b0;
    i_bp_byte_en = 4'd0;
    i_bp_sof     = 1'b0;
    i_bp_sol     = 1'b0;
    i_bp_eol     = 1'b0;
    i_bp_vc_tag  = 2'd0;
    i_bp_dt_sel  = 1'b0;
    repeat (4) @(negedge i_sys_clk);
    i_resetn = 1'b1;
    repeat (2) @(negedge i_sys_clk);
    case (test_name)
      "test_reset_values":            test_reset_values();
      "test_async_resetn":            test_async_resetn();
      "test_soft_reset":              test_soft_reset();
      "test_raw12_aligned":           test_raw12_aligned();
      "test_raw10_aligned":           test_raw10_aligned();
      "test_raw12_partial_last_beat": test_raw12_partial_last_beat();
      "test_raw10_partial_last_beat": test_raw10_partial_last_beat();
      "test_misaligned_whole_units":  test_misaligned_whole_units();
      "test_misaligned_pad_unit":     test_misaligned_pad_unit();
      "test_eol_without_payload":     test_eol_without_payload();
      "test_idle_gaps":               test_idle_gaps();
      "test_single_beat_lines":       test_single_beat_lines();
      "test_line_start_safety_net":   test_line_start_safety_net();
      "test_first_beat_remainder":    test_first_beat_remainder();
      "test_sync_pending":            test_sync_pending();
      "test_vc_dt_passthrough":       test_vc_dt_passthrough();
      "test_back_to_back_lines":      test_back_to_back_lines();
      "test_idle_inputs_ignored":     test_idle_inputs_ignored();
      "test_sensor_line_lengths":     test_sensor_line_lengths();
      "test_missing_sol_after_eol":   test_missing_sol_after_eol();
      "test_random_traffic":          test_random_traffic();
      "test_checker_fault":           test_checker_fault();
      default: report_error($sformatf("unknown test '%s'", test_name));
    endcase
    idle(4);
    $display("edges checked=%0d, lines compared=%0d, pixels compared=%0d, errors=%0d",
             edge_checks, lines_compared, px_compared, err_count);
    if ((err_count == 0) && (px_compared > 0)) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d)", test_name, err_count);
    $finish;
  end

  // Timeout.
  initial begin
    #20_000_000.0;
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
