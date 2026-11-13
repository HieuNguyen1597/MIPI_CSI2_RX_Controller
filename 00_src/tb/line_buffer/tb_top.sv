// File        : tb_top.sv
// Description : Testbench of m_fpt_line_buffer (Line Buffer).
//               A cycle-accurate reference model written from the specification equations predicts
//               every output at every rising edge of sys_clk. An independent FIFO queue scoreboard
//               verifies end-to-end entry ordering and bit-for-bit integrity of all 57 stored bits.
//               Rule checkers verify output hold under backpressure, occupancy flag relations,
//               asynchronous reset response, registered output stability, and absence of X/Z.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions agreed by the block owner:
//   - Every register is clocked on the rising edge of sys_clk and samples its inputs there.
//   - Memory reads are synchronous (r_data captures mem[rd_ptr] on the clock edge after rd_en,
//     with inflight tracking the pending read). A beat accepted into an empty buffer at cycle t is
//     read at t + 1, lands in out_reg at t + 2, and is presented with fb_px_valid = 1 at t + 3.
//   - The two-entry output stage (out_reg and skid_reg) promotes skid_reg into out_reg on
//     consume && skid_valid. A landing read (inflight == 1) loads into out_reg when out_reg is
//     free (!fb_px_valid || (consume && !skid_valid)), and otherwise loads into skid_reg.
//   - Asynchronous active-low resetn and synchronous active-high soft_reset and lb_flush clear
//     wr_ptr, rd_ptr, fill_cnt, inflight, fb_px_valid, and skid_valid to zero, winning over any
//     write, read issue, landing read, or output consumption in the same cycle. On a synchronous
//     clear cycle, the combinational linebuf_overflow output keeps its pre-edge equation. At the
//     module ports, soft_reset and lb_flush have identical behavior.
//   - Memory and data/sideband registers (mem, r_data, out_reg, skid_reg) are not cleared by reset
//     or flush; only valid flags and pointers are cleared. All payload and sideband outputs
//     (fb_px_data, fb_px_valid_count, fb_px_sof, fb_px_sol, fb_px_eol, fb_px_vc_tag, fb_px_dt_sel,
//     fb_err_wc_align) are per-entry fields qualified by fb_px_valid == 1 and held stable across
//     stall cycles (fb_px_valid && !fb_ready).
//   - linebuf_empty (fill_cnt == 0), linebuffer_full (fill_cnt >= PARA_FULL_WATERMARK), and
//     linebuf_fill (fill_cnt) reflect the registered occupancy count and update one cycle after an
//     accepted write or output consumption. Write acceptance evaluates fill_cnt at the start of
//     the cycle (an output consumption in the same cycle does not free a slot for that cycle's
//     write). Normal beats (px_eol == 0) are accepted when fill_cnt < PARA_LINE_BUFFER_DEPTH - 1;
//     the last slot (fill_cnt == PARA_LINE_BUFFER_DEPTH - 1) is reserved for an end-of-line beat
//     (px_eol == 1); at fill_cnt == PARA_LINE_BUFFER_DEPTH all writes are refused and
//     linebuf_overflow asserts on every refused write cycle.
//   - A write is requested whenever px_valid && (px_valid_count != 0). The block stores and
//     forwards all 57 bits of an accepted entry without modifying or filtering non-standard
//     px_valid_count values (1, 3, 5, 6, 7), sync/error flag combinations, or upper pixel lanes.
//   - PARA_LINE_BUFFER_DEPTH is a power of two in 4..32768 and PARA_FULL_WATERMARK is in
//     1..PARA_LINE_BUFFER_DEPTH - 1.
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_LINE_BUFFER_DEPTH = 32'd2048;
  parameter logic [31:0] PARA_FULL_WATERMARK    = 32'd1740;

  localparam int LBD   = int'(PARA_LINE_BUFFER_DEPTH);
  localparam int FW    = int'(PARA_FULL_WATERMARK);
  localparam int PTR_W = (LBD > 1) ? $clog2(LBD) : 1;
  localparam int CNT_W = $clog2(LBD + 1);

  localparam real T_CLK      = 5.0;
  localparam int  MAX_PRINT  = 30;
  localparam int  MAX_CYCLES = (LBD > 2048) ? (40 * LBD + 50000) : 250000;

  typedef struct packed {
    logic        err;
    logic        eol;
    logic        sol;
    logic        sof;
    logic [2:0]  count;
    logic [1:0]  tag;
    logic        dt;
    logic [47:0] data;
  } tb_entry_t;

  // ----------------------------------------------------------------------------------------------
  // DUT signals and instantiation
  // ----------------------------------------------------------------------------------------------
  logic             i_sys_clk;
  logic             i_resetn;
  logic             i_soft_reset;
  logic             i_lb_flush;
  logic [47:0]      i_px_data;
  logic             i_px_valid;
  logic [2:0]       i_px_valid_count;
  logic             i_px_sof;
  logic             i_px_sol;
  logic             i_px_eol;
  logic [1:0]       i_px_vc_tag;
  logic             i_px_dt_sel;
  logic             i_err_wc_align;
  logic [47:0]      o_fb_px_data;
  logic             o_fb_px_valid;
  logic [2:0]       o_fb_px_valid_count;
  logic             o_fb_px_sof;
  logic             o_fb_px_sol;
  logic             o_fb_px_eol;
  logic [1:0]       o_fb_px_vc_tag;
  logic             o_fb_px_dt_sel;
  logic             o_fb_err_wc_align;
  logic             i_fb_ready;
  logic             o_linebuffer_full;
  logic             o_linebuf_empty;
  logic             o_linebuf_overflow;
  logic [CNT_W-1:0] o_linebuf_fill;

  m_fpt_line_buffer #(
    .PARA_LINE_BUFFER_DEPTH (PARA_LINE_BUFFER_DEPTH),
    .PARA_FULL_WATERMARK    (PARA_FULL_WATERMARK)
  ) u_dut (
    .i_sys_clk           (i_sys_clk),
    .i_resetn            (i_resetn),
    .i_soft_reset        (i_soft_reset),
    .i_lb_flush          (i_lb_flush),
    .i_px_data           (i_px_data),
    .i_px_valid          (i_px_valid),
    .i_px_valid_count    (i_px_valid_count),
    .i_px_sof            (i_px_sof),
    .i_px_sol            (i_px_sol),
    .i_px_eol            (i_px_eol),
    .i_px_vc_tag         (i_px_vc_tag),
    .i_px_dt_sel         (i_px_dt_sel),
    .i_err_wc_align      (i_err_wc_align),
    .o_fb_px_data        (o_fb_px_data),
    .o_fb_px_valid       (o_fb_px_valid),
    .o_fb_px_valid_count (o_fb_px_valid_count),
    .o_fb_px_sof         (o_fb_px_sof),
    .o_fb_px_sol         (o_fb_px_sol),
    .o_fb_px_eol         (o_fb_px_eol),
    .o_fb_px_vc_tag      (o_fb_px_vc_tag),
    .o_fb_px_dt_sel      (o_fb_px_dt_sel),
    .o_fb_err_wc_align   (o_fb_err_wc_align),
    .i_fb_ready          (i_fb_ready),
    .o_linebuffer_full   (o_linebuffer_full),
    .o_linebuf_empty     (o_linebuf_empty),
    .o_linebuf_overflow  (o_linebuf_overflow),
    .o_linebuf_fill      (o_linebuf_fill)
  );

  initial begin
    i_sys_clk = 1'b0;
    forever #(T_CLK / 2.0) i_sys_clk = ~i_sys_clk;
  end

  // ----------------------------------------------------------------------------------------------
  // Bookkeeping and error reporting
  // ----------------------------------------------------------------------------------------------
  string       test_name          = "none";
  int          seed               = 1;
  int unsigned err_count          = 0;
  int unsigned cyc                = 0;
  int unsigned edge_checks        = 0;
  int unsigned entries_accepted   = 0;
  int unsigned entries_compared   = 0;
  bit          chk_en             = 1'b0;
  bit          chk_robust         = 1'b0;
  bit          chk_tpg_mode       = 1'b0;
  bit          fault_inject_arm   = 1'b0;
  bit          fault_injected     = 1'b0;

  task automatic fail(input string msg);
    err_count++;
    if (err_count <= MAX_PRINT) $error("cycle %0d: %s", cyc, msg);
  endtask

  task automatic require_hit(input int count, input string label);
    if (count <= 0) fail($sformatf("scenario not reached: %s", label));
  endtask

  function automatic tb_entry_t cur_in_entry();
    tb_entry_t e;
    e.err   = i_err_wc_align;
    e.eol   = i_px_eol;
    e.sol   = i_px_sol;
    e.sof   = i_px_sof;
    e.count = i_px_valid_count;
    e.tag   = i_px_vc_tag;
    e.dt    = i_px_dt_sel;
    e.data  = i_px_data;
    return e;
  endfunction

  function automatic tb_entry_t cur_out_entry();
    tb_entry_t e;
    e.err   = o_fb_err_wc_align;
    e.eol   = o_fb_px_eol;
    e.sol   = o_fb_px_sol;
    e.sof   = o_fb_px_sof;
    e.count = o_fb_px_valid_count;
    e.tag   = o_fb_px_vc_tag;
    e.dt    = o_fb_px_dt_sel;
    e.data  = o_fb_px_data;
    return e;
  endfunction

  // ----------------------------------------------------------------------------------------------
  // Reference model and independent FIFO scoreboard
  // ----------------------------------------------------------------------------------------------
  tb_entry_t        m_mem [LBD];
  logic [PTR_W-1:0] m_wr_ptr;
  logic [PTR_W-1:0] m_rd_ptr;
  logic [CNT_W-1:0] m_fill_cnt;
  logic             m_inflight;
  tb_entry_t        m_r_data;
  tb_entry_t        m_out_reg;
  logic             m_out_valid;
  tb_entry_t        m_skid_reg;
  logic             m_skid_valid;

  tb_entry_t        sb_q [$];
  int               sb_fill;
  int               sb_accepted_since_clear;

  // Stall-hold and registered-output stability state.
  bit               prev_stalled;
  tb_entry_t        prev_stall_entry;
  bit               snap_valid;
  logic             snap_fb_valid;
  tb_entry_t        snap_fb_entry;
  logic             snap_full;
  logic             snap_empty;
  logic [CNT_W-1:0] snap_fill;
  bit               prev_full_for_tpg;

  // Scenario hit counters for test self-checks.
  int cov_clear_state [3][8];
  int cov_clear_accept, cov_clear_consume, cov_clear_land, cov_clear_ovf;
  int cov_v1c0, cov_v0c4, cov_v0c0;
  int cov_norm_ref_lbdm1, cov_eol_acc_lbdm1, cov_eol_ref_lbd, cov_norm_ref_lbd;
  int cov_consec_ovf, cov_budget_ok;
  int cov_full_rise, cov_full_fall, cov_empty_rise, cov_empty_fall, cov_fill_lbd;
  int cov_wrcons_at [5];
  int cov_single_stall, cov_land_skid_stall, cov_cons_land, cov_long_stall;
  int cov_wrap;
  int cov_tag_change, cov_tag [4], cov_dt [2], cov_err_eol, cov_cnt2, cov_cnt4;
  int cov_tput, cov_lat3;
  int cov_tpg_hold;
  int cov_after_clear_write;
  int cov_robust_cnt [8];

  int consec_ovf_run;
  int consec_cons_run;
  int stall_run;
  int empty_write_age;
  logic [1:0] last_out_tag;
  bit have_last_out_tag;
  bit prev_full_obs, prev_empty_obs;

  function automatic void clear_model_and_sb();
    m_wr_ptr                = '0;
    m_rd_ptr                = '0;
    m_fill_cnt              = '0;
    m_inflight              = 1'b0;
    m_out_valid             = 1'b0;
    m_skid_valid            = 1'b0;
    sb_q.delete();
    sb_fill                 = 0;
    sb_accepted_since_clear = 0;
    prev_stalled            = 1'b0;
    snap_valid              = 1'b0;
    consec_ovf_run          = 0;
    consec_cons_run         = 0;
    stall_run               = 0;
    empty_write_age         = -1;
  endfunction

  function automatic void record_clear_coverage(
    input int   kind,
    input logic wr_acc,
    input logic cons,
    input logic land,
    input logic ovf
  );
    if (m_fill_cnt == '0 && !m_inflight && !m_out_valid && !m_skid_valid)
      cov_clear_state[kind][0]++;
    if (wr_acc || cons || m_inflight)
      cov_clear_state[kind][1]++;
    if (m_out_valid)
      cov_clear_state[kind][2]++;
    if (m_inflight)
      cov_clear_state[kind][3]++;
    if (m_skid_valid && m_out_valid && !i_fb_ready)
      cov_clear_state[kind][4]++;
    if (int'(m_fill_cnt) == FW)
      cov_clear_state[kind][5]++;
    if (int'(m_fill_cnt) == LBD - 1)
      cov_clear_state[kind][6]++;
    if (int'(m_fill_cnt) == LBD)
      cov_clear_state[kind][7]++;
    if (wr_acc) cov_clear_accept++;
    if (cons)   cov_clear_consume++;
    if (land)   cov_clear_land++;
    if (ovf)    cov_clear_ovf++;
  endfunction

  // Asynchronous reset immediate check and state clear (covers sub-cycle reset pulses too).
  always @(negedge i_resetn) begin
    if (chk_en) begin
      record_clear_coverage(0, 1'b0, 1'b0, m_inflight, 1'b0);
    end
    clear_model_and_sb();
    #1ps;
    if (chk_en && (i_resetn === 1'b0)) begin
      if (o_fb_px_valid !== 1'b0)
        fail($sformatf("async resetn: fb_px_valid=%b expected 0", o_fb_px_valid));
      if (o_linebuffer_full !== 1'b0)
        fail($sformatf("async resetn: linebuffer_full=%b expected 0", o_linebuffer_full));
      if (o_linebuf_empty !== 1'b1)
        fail($sformatf("async resetn: linebuf_empty=%b expected 1", o_linebuf_empty));
      if (o_linebuf_fill !== '0)
        fail($sformatf("async resetn: linebuf_fill=%0d expected 0", o_linebuf_fill));
    end
  end

  // Main cycle checker and reference model update at posedge i_sys_clk.
  always @(posedge i_sys_clk) begin
    cyc++;
    if (!chk_en) begin
      if (!i_resetn || i_soft_reset || i_lb_flush) clear_model_and_sb();
    end else if (!i_resetn) begin
      record_clear_coverage(0, 1'b0, 1'b0, m_inflight, 1'b0);
      clear_model_and_sb();
      if (o_fb_px_valid !== 1'b0 || o_linebuffer_full !== 1'b0 ||
          o_linebuf_empty !== 1'b1 || o_linebuf_fill !== '0)
        fail("outputs not in reset state while resetn is low at clock edge");
    end else begin
      logic        write_en;
      logic        write_accept;
      logic        consume;
      logic        land;
      logic        promote;
      logic        out_free;
      logic        r_to_out;
      logic        r_to_skid;
      logic [1:0]  pipe_occ;
      logic [1:0]  space;
      logic        mem_Empty;
      logic        rd_en;
      logic        exp_full;
      logic        exp_empty;
      logic        exp_ovf;
      tb_entry_t   in_e;
      tb_entry_t   obs_out_e;
      bit          was_empty_before;

      edge_checks++;
      in_e      = cur_in_entry();
      obs_out_e = cur_out_entry();

      // Fault injection hook for test_checker_fault (perturbs comparator input only, no force).
      if (fault_inject_arm && !fault_injected && (o_fb_px_valid === 1'b1)) begin
        obs_out_e.data[0] = ~obs_out_e.data[0];
        fault_injected    = 1'b1;
      end

      // 1. Check that registered outputs did not glitch between the post-NBA sample and this edge.
      if (snap_valid) begin
        if (o_fb_px_valid !== snap_fb_valid || o_linebuffer_full !== snap_full ||
            o_linebuf_empty !== snap_empty || o_linebuf_fill !== snap_fill)
          fail("registered status/valid output changed between rising clock edges");
        if (snap_fb_valid && (cur_out_entry() !== snap_fb_entry))
          fail("registered payload/sideband output changed between rising clock edges");
      end

      // 2. Check for X/Z on inputs and outputs.
      if ($isunknown({i_soft_reset, i_lb_flush, i_px_valid, i_fb_ready}))
        fail("X/Z on control inputs");
      if (i_px_valid === 1'b1 &&
          $isunknown({i_px_data, i_px_valid_count, i_px_sof, i_px_sol, i_px_eol,
                      i_px_vc_tag, i_px_dt_sel, i_err_wc_align}))
        fail("X/Z on active pixel inputs");
      if ($isunknown({o_fb_px_valid, o_linebuffer_full, o_linebuf_empty,
                      o_linebuf_overflow, o_linebuf_fill}))
        fail("X/Z on status or valid outputs");
      if ((o_fb_px_valid === 1'b1) && $isunknown(obs_out_e))
        fail("X/Z on payload or sideband outputs while fb_px_valid is high");

      // 3. Source assumptions monitor (unless explicitly paused by the active test).
      if (!chk_robust && (i_px_valid === 1'b1)) begin
        if (!(i_px_valid_count inside {3'd0, 3'd2, 3'd4}))
          fail($sformatf("source monitor: invalid px_valid_count=%0d", i_px_valid_count));
        if ((i_px_valid_count == 3'd0) &&
            (i_px_sof || i_px_sol || i_px_eol || i_err_wc_align))
          fail("source monitor: sync or error flag asserted with px_valid_count=0");
        if (i_err_wc_align && !i_px_eol)
          fail("source monitor: err_wc_align asserted without px_eol");
      end
      if (chk_tpg_mode) begin
        if (i_px_valid_count !== 3'd4)
          fail($sformatf("TPG source monitor: px_valid_count=%0d expected 4", i_px_valid_count));
        if ((o_linebuffer_full === 1'b1) && (i_px_valid === 1'b1))
          fail("TPG source monitor: px_valid high while linebuffer_full is asserted");
      end
      prev_full_for_tpg = (o_linebuffer_full === 1'b1);

      // 4. Pre-edge reference model equations from MAS and approved specification changes.
      write_en     = i_px_valid && (i_px_valid_count != 3'd0);
      write_accept = write_en &&
                     ((int'(m_fill_cnt) < (LBD - 1)) ||
                      ((int'(m_fill_cnt) == (LBD - 1)) && i_px_eol));
      exp_ovf      = write_en && !write_accept;
      exp_empty    = (m_fill_cnt == '0);
      exp_full     = (int'(m_fill_cnt) >= FW);

      consume      = m_out_valid && i_fb_ready;
      land         = m_inflight;
      promote      = consume && m_skid_valid;
      out_free     = !m_out_valid || (consume && !m_skid_valid);
      r_to_out     = land && out_free;
      r_to_skid    = land && !out_free;

      pipe_occ     = 2'(m_out_valid) + 2'(m_skid_valid) + 2'(m_inflight) - 2'(consume);
      space        = 2'd2 - pipe_occ;
      mem_Empty    = (int'(m_fill_cnt) <=
                      (int'(m_out_valid) + int'(m_skid_valid) + int'(m_inflight)));
      rd_en        = !mem_Empty && (space > 2'd0);

      // 5. Compare all 12 DUT outputs against the pre-edge reference model.
      if (o_fb_px_valid !== m_out_valid)
        fail($sformatf("fb_px_valid got=%b expected=%b", o_fb_px_valid, m_out_valid));
      if (o_linebuffer_full !== exp_full)
        fail($sformatf("linebuffer_full got=%b expected=%b (fill=%0d)",
                       o_linebuffer_full, exp_full, m_fill_cnt));
      if (o_linebuf_empty !== exp_empty)
        fail($sformatf("linebuf_empty got=%b expected=%b (fill=%0d)",
                       o_linebuf_empty, exp_empty, m_fill_cnt));
      if (o_linebuf_overflow !== exp_ovf)
        fail($sformatf("linebuf_overflow got=%b expected=%b", o_linebuf_overflow, exp_ovf));
      if (o_linebuf_fill !== m_fill_cnt)
        fail($sformatf("linebuf_fill got=%0d expected=%0d", o_linebuf_fill, m_fill_cnt));

      if (m_out_valid) begin
        if (obs_out_e !== m_out_reg) begin
          fail($sformatf("payload/sideband mismatch: got=%h expected=%h", obs_out_e, m_out_reg));
        end
      end

      // 6. Stall-hold check and status relation check.
      if (prev_stalled) begin
        if (o_fb_px_valid !== 1'b1)
          fail("fb_px_valid dropped during stall without reset or flush");
        if (obs_out_e !== prev_stall_entry)
          fail("payload or sideband changed while stalled (fb_px_valid && !fb_ready)");
      end
      if ((o_linebuf_empty === 1'b1) && (o_linebuffer_full === 1'b1))
        fail("linebuf_empty and linebuffer_full both high");

      // 7. Scenario coverage tracking.
      if (i_px_valid && (i_px_valid_count == 3'd0))  cov_v1c0++;
      if (!i_px_valid && (i_px_valid_count == 3'd4)) cov_v0c4++;
      if (!i_px_valid && (i_px_valid_count == 3'd0)) cov_v0c0++;

      if (!i_soft_reset && !i_lb_flush) begin
        if (write_en && !i_px_eol && (int'(m_fill_cnt) == LBD - 1) && !write_accept)
          cov_norm_ref_lbdm1++;
        if (write_en && i_px_eol && (int'(m_fill_cnt) == LBD - 1) && write_accept)
          cov_eol_acc_lbdm1++;
        if (write_en && i_px_eol && (int'(m_fill_cnt) == LBD) && !write_accept)
          cov_eol_ref_lbd++;
        if (write_en && !i_px_eol && (int'(m_fill_cnt) == LBD) && !write_accept)
          cov_norm_ref_lbd++;
        if (exp_ovf) begin
          consec_ovf_run++;
          if (consec_ovf_run >= 2) cov_consec_ovf++;
        end else begin
          consec_ovf_run = 0;
        end

        if (!prev_full_obs && exp_full)   cov_full_rise++;
        if (prev_full_obs && !exp_full)   cov_full_fall++;
        if (!prev_empty_obs && exp_empty) cov_empty_rise++;
        if (prev_empty_obs && !exp_empty) cov_empty_fall++;
        if (int'(m_fill_cnt) == LBD)      cov_fill_lbd++;
        prev_full_obs  = exp_full;
        prev_empty_obs = exp_empty;

        if (write_en && consume) begin
          if (int'(m_fill_cnt) == 1)       cov_wrcons_at[0]++;
          if (int'(m_fill_cnt) == FW - 1)  cov_wrcons_at[1]++;
          if (int'(m_fill_cnt) == FW)      cov_wrcons_at[2]++;
          if (int'(m_fill_cnt) == LBD - 1) cov_wrcons_at[3]++;
          if (int'(m_fill_cnt) == LBD)     cov_wrcons_at[4]++;
        end

        if (m_out_valid && !i_fb_ready) begin
          stall_run++;
          if (stall_run > LBD) cov_long_stall++;
        end else begin
          if (stall_run == 1 && m_out_valid && i_fb_ready) cov_single_stall++;
          stall_run = 0;
        end
        if (r_to_skid && m_out_valid && !i_fb_ready) cov_land_skid_stall++;
        if (consume && land) cov_cons_land++;

        if (consume) begin
          consec_cons_run++;
          if (consec_cons_run >= LBD) cov_tput++;
          cov_tag[m_out_reg.tag]++;
          cov_dt[m_out_reg.dt]++;
          if (m_out_reg.err && m_out_reg.eol) cov_err_eol++;
          if (m_out_reg.count == 3'd2) cov_cnt2++;
          if (m_out_reg.count == 3'd4) cov_cnt4++;
          cov_robust_cnt[m_out_reg.count]++;
          if (have_last_out_tag && (m_out_reg.tag != last_out_tag)) cov_tag_change++;
          last_out_tag      = m_out_reg.tag;
          have_last_out_tag = 1'b1;
        end else begin
          consec_cons_run = 0;
        end

        if (chk_tpg_mode && exp_full && !i_px_valid && !i_fb_ready) cov_tpg_hold++;
      end

      // Track t + 3 empty-to-valid latency.
      was_empty_before = (m_fill_cnt == '0) && !m_inflight && !m_out_valid && !m_skid_valid;
      if (i_soft_reset || i_lb_flush) begin
        empty_write_age = -1;
      end else begin
        if (empty_write_age >= 0) begin
          empty_write_age++;
          if ((empty_write_age == 3) && m_out_valid) cov_lat3++;
          if (empty_write_age >= 3) empty_write_age = -1;
        end
        if (was_empty_before && write_accept && i_fb_ready) empty_write_age = 0;
      end

      // 8. State and scoreboard updates on the clock edge.
      if (i_soft_reset || i_lb_flush) begin
        if (i_soft_reset)
          record_clear_coverage(1, write_accept, consume, land, exp_ovf);
        if (i_lb_flush)
          record_clear_coverage(2, write_accept, consume, land, exp_ovf);
        clear_model_and_sb();
        prev_full_obs  = 1'b0;
        prev_empty_obs = 1'b1;
      end else begin
        // Independent FIFO scoreboard check on consume and push on write_accept.
        if (consume) begin
          entries_compared++;
          if (sb_q.size() == 0) begin
            fail("scoreboard underflow on output consumption");
          end else begin
            tb_entry_t exp_sb;
            exp_sb = sb_q.pop_front();
            sb_fill--;
            if (obs_out_e !== exp_sb)
              fail($sformatf("scoreboard mismatch: got=%h expected=%h", obs_out_e, exp_sb));
          end
        end
        if (write_accept) begin
          entries_accepted++;
          sb_q.push_back(in_e);
          sb_fill++;
          sb_accepted_since_clear++;
          if (sb_accepted_since_clear > 2 * LBD) cov_wrap++;
        end

        // Update reference model state (order preserves pre-edge values).
        if (promote) begin
          m_out_reg   = m_skid_reg;
          m_out_valid = 1'b1;
        end else if (r_to_out) begin
          m_out_reg   = m_r_data;
          m_out_valid = 1'b1;
        end else if (consume) begin
          m_out_valid = 1'b0;
        end

        if (r_to_skid) begin
          m_skid_reg   = m_r_data;
          m_skid_valid = 1'b1;
        end else if (promote) begin
          m_skid_valid = 1'b0;
        end

        if (rd_en) begin
          m_r_data = m_mem[m_rd_ptr];
          m_rd_ptr = m_rd_ptr + PTR_W'(1);
        end
        m_inflight = rd_en;

        if (write_accept) begin
          m_mem[m_wr_ptr] = in_e;
          m_wr_ptr        = m_wr_ptr + PTR_W'(1);
        end

        case ({write_accept, consume})
          2'b10:   m_fill_cnt = m_fill_cnt + CNT_W'(1);
          2'b01:   m_fill_cnt = m_fill_cnt - CNT_W'(1);
          default: m_fill_cnt = m_fill_cnt;
        endcase

        if (sb_fill != int'(m_fill_cnt))
          fail($sformatf("internal model/scoreboard fill mismatch: sb=%0d model=%0d",
                         sb_fill, m_fill_cnt));

        prev_stalled     = m_out_valid && !i_fb_ready;
        prev_stall_entry = m_out_reg;
      end
    end

    // Post-NBA sample for registered output stability verification.
    #1ps;
    if (chk_en && (i_resetn === 1'b1)) begin
      snap_valid    = 1'b1;
      snap_fb_valid = o_fb_px_valid;
      snap_fb_entry = cur_out_entry();
      snap_full     = o_linebuffer_full;
      snap_empty    = o_linebuf_empty;
      snap_fill     = o_linebuf_fill;
    end else begin
      snap_valid = 1'b0;
    end
  end

  // ----------------------------------------------------------------------------------------------
  // Stimulus helpers (all inputs driven on negedge i_sys_clk)
  // ----------------------------------------------------------------------------------------------
  typedef enum int {RD_ONE, RD_ZERO, RD_RAND, RD_MANUAL} rd_mode_e;
  rd_mode_e rd_mode = RD_ONE;
  int       rd_pct  = 70;

  task automatic drive_idle();
    i_soft_reset     = 1'b0;
    i_lb_flush       = 1'b0;
    i_px_data        = '0;
    i_px_valid       = 1'b0;
    i_px_valid_count = 3'd0;
    i_px_sof         = 1'b0;
    i_px_sol         = 1'b0;
    i_px_eol         = 1'b0;
    i_px_vc_tag      = 2'd0;
    i_px_dt_sel      = 1'b0;
    i_err_wc_align   = 1'b0;
  endtask

  task automatic drive_entry(input tb_entry_t e, input logic valid = 1'b1);
    i_soft_reset     = 1'b0;
    i_lb_flush       = 1'b0;
    i_px_data        = e.data;
    i_px_valid       = valid;
    i_px_valid_count = e.count;
    i_px_sof         = e.sof;
    i_px_sol         = e.sol;
    i_px_eol         = e.eol;
    i_px_vc_tag      = e.tag;
    i_px_dt_sel      = e.dt;
    i_err_wc_align   = e.err;
  endtask

  task automatic step(input int n = 1);
    repeat (n) begin
      @(negedge i_sys_clk);
      case (rd_mode)
        RD_ONE:  i_fb_ready = 1'b1;
        RD_ZERO: i_fb_ready = 1'b0;
        RD_RAND: i_fb_ready = ($urandom_range(99, 0) < rd_pct);
        default: ;
      endcase
    end
  endtask

  task automatic do_reset();
    chk_en       = 1'b0;
    i_resetn     = 1'b0;
    i_fb_ready   = 1'b0;
    drive_idle();
    clear_model_and_sb();
    repeat (3) @(negedge i_sys_clk);
    i_resetn       = 1'b1;
    prev_full_obs  = 1'b0;
    prev_empty_obs = 1'b1;
    chk_en         = 1'b1;
    @(negedge i_sys_clk);
  endtask

  function automatic tb_entry_t rand_entry(
    input logic       sof = 1'b0,
    input logic       sol = 1'b0,
    input logic       eol = 1'b0,
    input logic [1:0] tag = 2'd0,
    input logic       dt  = 1'b0
  );
    tb_entry_t e;
    e.sof   = sof;
    e.sol   = sol;
    e.eol   = eol;
    e.err   = eol ? 1'($urandom_range(1, 0) == 0 && ($urandom_range(7, 0) == 0)) : 1'b0;
    e.count = eol ? (($urandom_range(1, 0) == 0) ? 3'd2 : 3'd4) : 3'd4;
    e.tag   = tag;
    e.dt    = dt;
    e.data  = {$urandom, 16'($urandom)};
    if (e.count == 3'd2) e.data[47:24] = 24'd0;
    if (!dt) begin
      e.data[11:10] = 2'b00;
      e.data[23:22] = 2'b00;
      e.data[35:34] = 2'b00;
      e.data[47:46] = 2'b00;
    end
    return e;
  endfunction

  task automatic send(input tb_entry_t e);
    step();
    drive_entry(e, 1'b1);
  endtask

  task automatic idle(input int n = 1);
    for (int i = 0; i < n; i++) begin
      step();
      drive_idle();
    end
  endtask

  task automatic send_line(
    input int         beats,
    input logic [1:0] tag = 2'd0,
    input logic       dt  = 1'b0,
    input logic       sof = 1'b0
  );
    for (int i = 0; i < beats; i++) begin
      send(rand_entry(sof && (i == 0), i == 0, i == (beats - 1), tag, dt));
    end
    idle(1);
  endtask

  task automatic wait_empty(input int limit = 8 * LBD + 64);
    int k;
    rd_mode = RD_ONE;
    k = 0;
    while (((o_linebuf_empty !== 1'b1) || (o_fb_px_valid !== 1'b0) || (sb_fill > 0)) &&
           (k < limit)) begin
      idle(1);
      k++;
    end
    if (k >= limit) fail("timeout waiting for buffer to drain to empty");
  endtask

  task automatic fill_to(input int target);
    wait_empty();
    rd_mode = RD_ZERO;
    for (int i = 0; i < target; i++) begin
      send(rand_entry(i == 0, i == 0, (i == (target - 1)) || (i == (LBD - 1))));
    end
    idle(2);
    if (sb_fill != target)
      fail($sformatf("fill_to(%0d) reached %0d", target, sb_fill));
  endtask

  task automatic apply_clear(input int kind, input int cycles = 1);
    if (kind == 0) begin
      // Assert resetn asynchronously after the rising edge and release at negedge.
      @(posedge i_sys_clk);
      #(T_CLK * 0.25);
      i_resetn = 1'b0;
      drive_idle();
      repeat (cycles) @(negedge i_sys_clk);
      i_resetn = 1'b1;
    end else begin
      for (int c = 0; c < cycles; c++) begin
        if (c > 0) step();
        if (kind == 1) i_soft_reset = 1'b1;
        else           i_lb_flush   = 1'b1;
      end
      step();
      drive_idle();
    end
  endtask

  task automatic after_clear_check();
    if (o_fb_px_valid !== 1'b0 || o_linebuffer_full !== 1'b0 ||
        o_linebuf_empty !== 1'b1 || o_linebuf_fill !== '0)
      fail("outputs not cleared immediately after reset or flush");
    rd_mode = RD_ONE;
    send_line(4, 2'd1, 1'b1, 1'b1);
    wait_empty();
  endtask

  task automatic clear_states(input int kind);
    // (0) Empty.
    apply_clear(kind);
    after_clear_check();
    // (1) During active read/write traffic.
    rd_mode = RD_ONE;
    send_line(12, 2'd0, 1'b0, 1'b1);
    apply_clear(kind);
    after_clear_check();
    // (2) Reading continuously.
    fill_to((LBD - 1 < 3) ? (LBD - 1) : 3);
    rd_mode = RD_ONE;
    step(2);
    apply_clear(kind);
    after_clear_check();
    // (3) Read in flight (two cycles after writing into an empty buffer).
    rd_mode = RD_ONE;
    send(rand_entry(1'b0, 1'b1, 1'b1));
    idle(1);
    step();
    apply_clear(kind);
    after_clear_check();
    // (4) Skid entry held under stall.
    fill_to(3);
    apply_clear(kind);
    after_clear_check();
    // (5) At watermark.
    fill_to(FW);
    apply_clear(kind);
    after_clear_check();
    // (6) At PARA_LINE_BUFFER_DEPTH - 1.
    fill_to(LBD - 1);
    apply_clear(kind);
    after_clear_check();
    // (7) At PARA_LINE_BUFFER_DEPTH.
    fill_to(LBD);
    apply_clear(kind);
    after_clear_check();
  endtask

  task automatic clear_same_cycle(input int kind);
    // (a) Accepted write in the clear cycle.
    rd_mode = RD_ONE;
    send_line(3);
    send(rand_entry(1'b0, 1'b1, 1'b0));
    apply_clear(kind);
    after_clear_check();
    // (b) Output consumption in the clear cycle.
    fill_to((LBD - 1 < 5) ? (LBD - 1) : 5);
    rd_mode = RD_MANUAL;
    step();
    i_fb_ready = 1'b1;
    apply_clear(kind);
    rd_mode = RD_ONE;
    after_clear_check();
    // (c) Refused write (linebuf_overflow high) in the clear cycle.
    fill_to(LBD);
    send(rand_entry());
    apply_clear(kind);
    after_clear_check();
  endtask

  // ----------------------------------------------------------------------------------------------
  // Directed and randomized tests
  // ----------------------------------------------------------------------------------------------

  // Port widths, legal parameter range, and basic line transfer with t + 3 empty latency.
  task automatic test_interface();
    if ($bits(i_px_data) != 48 || $bits(o_fb_px_data) != 48)
      fail("px_data or fb_px_data width is not 48");
    if ($bits(i_px_valid_count) != 3 || $bits(o_fb_px_valid_count) != 3)
      fail("valid_count width is not 3");
    if ($bits(i_px_vc_tag) != 2 || $bits(o_fb_px_vc_tag) != 2)
      fail("vc_tag width is not 2");
    if ($bits(o_linebuf_fill) != $clog2(LBD + 1))
      fail("linebuf_fill width does not match clog2(PARA_LINE_BUFFER_DEPTH + 1)");
    if (LBD < 4 || LBD > 32768 || ((LBD & (LBD - 1)) != 0) || FW < 1 || FW > (LBD - 1))
      fail("active configuration is outside the valid parameter domain");
    rd_mode = RD_ONE;
    send_line(6, 2'd0, 1'b0, 1'b1);
    send_line(4, 2'd1, 1'b1, 1'b0);
    wait_empty();
    require_hit(cov_lat3, "first presentation at t + 3");
  endtask

  // Asynchronous resetn across all 8 states plus a sub-cycle glitch between rising clock edges.
  task automatic test_async_resetn();
    clear_states(0);
    // Sub-cycle asynchronous reset pulse entirely between two rising clock edges.
    fill_to(3);
    @(posedge i_sys_clk);
    #(T_CLK * 0.25);
    i_resetn = 1'b0;
    #(T_CLK * 0.35);
    i_resetn = 1'b1;
    @(negedge i_sys_clk);
    after_clear_check();
    for (int s = 0; s < 8; s++)
      require_hit(cov_clear_state[0][s], $sformatf("resetn state %0d", s));
  endtask

  // Synchronous soft_reset across all 8 states, 4 same-cycle events, and multi-cycle hold.
  task automatic test_soft_reset();
    clear_states(1);
    clear_same_cycle(1);
    rd_mode = RD_RAND;
    rd_pct  = 70;
    send_line(10);
    send(rand_entry());
    apply_clear(1, 3);
    after_clear_check();
    for (int s = 0; s < 8; s++)
      require_hit(cov_clear_state[1][s], $sformatf("soft_reset state %0d", s));
    require_hit(cov_clear_accept, "clear with accepted write");
    require_hit(cov_clear_consume, "clear with consume");
    require_hit(cov_clear_land, "clear with landing read");
    require_hit(cov_clear_ovf, "clear with linebuf_overflow");
  endtask

  // Deterministic trace helper to compare soft_reset and lb_flush port-for-port.
  task automatic history_trace(input int kind, input int hseed, output logic [63:0] trace [40]);
    logic [31:0] h;
    tb_entry_t   e;
    do_reset();
    h       = 32'(hseed);
    rd_mode = RD_MANUAL;
    for (int i = 0; i < 52; i++) begin
      step();
      h          = h * 32'd1664525 + 32'd1013904223;
      i_fb_ready = ((int'(h[31:16]) % 100) < 40);
      e.err      = 1'b0;
      e.eol      = (i == 9 || i == 31);
      e.sol      = (i == 0 || i == 12);
      e.sof      = (i == 0);
      e.count    = 3'd4;
      e.tag      = 2'd1;
      e.dt       = h[3];
      e.data     = {h, h[15:0]};
      if (i == 10) begin
        drive_entry(e, 1'b1);
        if (kind == 1) i_soft_reset = 1'b1;
        else           i_lb_flush   = 1'b1;
      end else begin
        i_soft_reset = 1'b0;
        i_lb_flush   = 1'b0;
        if (i < 10 || (i > 11 && i < 32)) drive_entry(e, 1'b1);
        else                              drive_idle();
      end
      if (i >= 12) begin
        trace[i - 12] = {o_fb_px_valid, o_linebuffer_full, o_linebuf_empty, o_linebuf_overflow,
                         12'(o_linebuf_fill), o_fb_px_valid ? o_fb_px_data : 48'd0};
      end
    end
    drive_idle();
    wait_empty();
  endtask

  // Synchronous lb_flush across all 8 states, 4 same-cycle events, and equivalence to soft_reset.
  task automatic test_lb_flush();
    logic [63:0] trace_soft [40];
    logic [63:0] trace_flush [40];
    clear_states(2);
    clear_same_cycle(2);
    history_trace(1, seed + 1000, trace_soft);
    history_trace(2, seed + 1000, trace_flush);
    for (int i = 0; i < 40; i++) begin
      if (trace_soft[i] !== trace_flush[i])
        fail($sformatf("post-clear trace mismatch at step %0d: soft=%h flush=%h",
                       i, trace_soft[i], trace_flush[i]));
    end
    for (int s = 0; s < 8; s++)
      require_hit(cov_clear_state[2][s], $sformatf("lb_flush state %0d", s));
  endtask

  task automatic send_non_writes(input int n);
    tb_entry_t e;
    for (int i = 0; i < n; i++) begin
      e = rand_entry();
      case (i % 3)
        0: begin
          e.count = 3'd0;
          e.sof   = 1'b0;
          e.sol   = 1'b0;
          e.eol   = 1'b0;
          e.err   = 1'b0;
          send(e);
        end
        1: begin
          e.count = 3'd4;
          e.sof   = 1'($urandom);
          e.sol   = 1'($urandom);
          e.eol   = 1'($urandom);
          e.err   = 1'($urandom);
          step();
          drive_entry(e, 1'b0);
        end
        default: begin
          e.count = 3'd0;
          step();
          drive_entry(e, 1'b0);
        end
      endcase
    end
    idle(1);
  endtask

  task automatic check_level_unchanged(input int level, input int n);
    int fill_before;
    fill_to(level);
    fill_before = int'(o_linebuf_fill);
    send_non_writes(n);
    step(3);
    if ((int'(o_linebuf_fill) != fill_before) || (sb_fill != level))
      fail($sformatf("fill changed from %0d to %0d on non-writing beats",
                     fill_before, o_linebuf_fill));
  endtask

  // Write qualification: valid=1/count=0 and valid=0/count=0,4 ignored at 4 levels and in traffic.
  task automatic test_write_qualification();
    rd_mode = RD_ZERO;
    send_non_writes(12);
    step(3);
    if (o_linebuf_fill !== '0) fail("non-writing beats modified an empty buffer");
    check_level_unchanged(LBD / 2, 12);
    check_level_unchanged(LBD - 1, 12);
    check_level_unchanged(LBD, 12);
    wait_empty();
    rd_mode = RD_RAND;
    rd_pct  = 60;
    for (int i = 0; i < 4 * LBD + 40; i++) begin
      if ($urandom_range(1, 0) == 1)
        send(rand_entry(1'b0, 1'b0, ($urandom_range(7, 0) == 0)));
      else
        send_non_writes(1);
    end
    wait_empty();
    require_hit(cov_v1c0, "px_valid=1 with px_valid_count=0");
    require_hit(cov_v0c4, "px_valid=0 with px_valid_count=4");
    require_hit(cov_v0c0, "px_valid=0 with px_valid_count=0");
  endtask

  // Empty-to-output latency (t + 3), bursts 1..2*DEPTH, 3*DEPTH stream, and DEPTH back-to-back.
  task automatic test_latency_throughput();
    int lens [5];
    rd_mode = RD_ONE;
    for (int k = 0; k < 4; k++) begin
      send(rand_entry(1'b0, 1'b1, 1'b1));
      idle(8);
    end
    require_hit(cov_lat3, "first presentation at t + 3");
    lens = '{1, 2, 3, 5, 2 * LBD};
    foreach (lens[i]) begin
      send_line(lens[i]);
      wait_empty();
      idle(3);
    end
    send_line(3 * LBD);
    wait_empty();
    fill_to(LBD);
    rd_mode = RD_ONE;
    wait_empty();
    require_hit(cov_tput, "PARA_LINE_BUFFER_DEPTH consecutive consumes");
  endtask

  // Random backpressure phases, single-cycle line-end stalls, and stalls longer than depth.
  task automatic test_random_backpressure();
    int phases;
    int wr_pct;
    int len;
    phases = 24;
    for (int p = 0; p < phases; p++) begin
      wr_pct = ((p % 4) == 3) ? 100 : $urandom_range(100, 0);
      case (p % 3)
        0: begin rd_mode = RD_RAND; rd_pct = $urandom_range(99, 10); end
        1: begin rd_mode = RD_ZERO; end
        default: begin rd_mode = RD_RAND; rd_pct = 95; end
      endcase
      for (int l = 0; l < 3; l++) begin
        len = $urandom_range((LBD < 64) ? LBD : 64, 1);
        for (int i = 0; i < len; i++) begin
          if ($urandom_range(99, 0) < wr_pct)
            send(rand_entry((l == 0) && (i == 0), i == 0, i == (len - 1), p[1:0]));
          else
            idle(1);
        end
        if (rd_mode != RD_ZERO) begin
          rd_mode = RD_MANUAL;
          step();
          drive_idle();
          i_fb_ready = 1'b0;
          step();
          i_fb_ready = 1'b1;
          rd_mode = RD_RAND;
        end
      end
      if ((p % 6) == 5) begin
        rd_mode = RD_ZERO;
        for (int i = 0; i < LBD + 8; i++) begin
          if ((i % 2) == 0) send(rand_entry(1'b0, 1'b0, (i % 16) == 14));
          else              idle(1);
        end
        rd_mode = RD_ONE;
        idle(4);
      end
    end
    wait_empty();
    require_hit(cov_single_stall, "single-cycle fb_ready=0 in a stream");
    require_hit(cov_land_skid_stall, "landing into skid_reg under stall");
    require_hit(cov_cons_land, "consume and landing read in the same cycle");
    require_hit(cov_long_stall, "stall longer than PARA_LINE_BUFFER_DEPTH");
  endtask

  // Interleaved VC tags 0..3, formats 0/1, sync flags, err_wc_align, and counts 2/4 under stalls.
  task automatic test_field_passthrough();
    tb_entry_t e;
    int        len;
    for (int l = 0; l < 40; l++) begin
      rd_mode = RD_RAND;
      rd_pct  = $urandom_range(99, 30);
      len     = $urandom_range(13, 2);
      for (int i = 0; i < len; i++) begin
        e = rand_entry(((l % 8) == 0) && (i == 0), i == 0, i == (len - 1),
                       2'($urandom_range(3, 0)), l[0]);
        if (i == (len - 1)) e.err = 1'($urandom_range(1, 0));
        send(e);
      end
      if ((l % 5) == 4) wait_empty();
    end
    wait_empty();
    require_hit(cov_tag_change, "VC tag change between consecutive entries");
    for (int t = 0; t < 4; t++)
      require_hit(cov_tag[t], $sformatf("VC tag %0d", t));
    require_hit(cov_dt[0], "px_dt_sel 0");
    require_hit(cov_dt[1], "px_dt_sel 1");
    require_hit(cov_err_eol, "err_wc_align with px_eol");
    require_hit(cov_cnt2, "px_valid_count 2");
    require_hit(cov_cnt4, "px_valid_count 4");
  endtask

  task automatic consume_one();
    rd_mode = RD_MANUAL;
    step();
    drive_idle();
    i_fb_ready = 1'b1;
    step();
    i_fb_ready = 1'b0;
    step(3);
  endtask

  // Exact watermark/empty/full transitions at FW - 1, FW, FW + 1, 0, and PARA_LINE_BUFFER_DEPTH.
  task automatic test_occupancy_thresholds();
    fill_to(FW - 1);
    if (o_linebuffer_full !== 1'b0) fail("linebuffer_full high at FW - 1");
    fill_to(FW);
    if (o_linebuffer_full !== 1'b1) fail("linebuffer_full low at FW");
    if ((FW + 1) <= (LBD - 1)) begin
      fill_to(FW + 1);
      consume_one();
    end
    consume_one();
    if (o_linebuffer_full !== 1'b0) fail("linebuffer_full high below FW");
    rd_mode = RD_ONE;
    wait_empty();
    idle(2);
    if (o_linebuf_empty !== 1'b1) fail("linebuf_empty low on empty buffer");
    send(rand_entry(1'b0, 1'b1, 1'b1));
    step();
    if (o_linebuf_empty !== 1'b0) fail("linebuf_empty high one cycle after accepted write");
    wait_empty();
    fill_to(LBD);
    wait_empty();
    require_hit(cov_full_rise, "linebuffer_full rise");
    require_hit(cov_full_fall, "linebuffer_full fall");
    require_hit(cov_empty_rise, "linebuf_empty rise");
    require_hit(cov_empty_fall, "linebuf_empty fall");
    require_hit(cov_fill_lbd, "linebuf_fill == PARA_LINE_BUFFER_DEPTH");
  endtask

  // Reaction budget (DEPTH - 1 - FW), normal refusal at DEPTH - 1, reserved EoL slot, full refusal.
  task automatic test_overflow_eol_slot();
    int after_full;
    int fill_before;
    bit refused;
    rd_mode = RD_ZERO;
    send(rand_entry(1'b1, 1'b1, 1'b0));
    step();
    drive_idle();
    after_full = 0;
    refused    = 1'b0;
    for (int i = 0; (i < 2 * LBD) && !refused; i++) begin
      fill_before = sb_fill;
      send(rand_entry());
      step();
      drive_idle();
      if ((sb_fill > fill_before) && (fill_before >= FW)) after_full++;
      if (sb_fill == fill_before) refused = 1'b1;
    end
    if (after_full != (LBD - 1 - FW))
      fail($sformatf("accepted %0d normal entries after watermark, expected %0d",
                     after_full, LBD - 1 - FW));
    else
      cov_budget_ok++;
    for (int i = 0; i < 3; i++) send(rand_entry());
    send(rand_entry(1'b0, 1'b0, 1'b1));
    send(rand_entry());
    send(rand_entry(1'b0, 1'b0, 1'b1));
    send(rand_entry(1'b0, 1'b1, 1'b0));
    idle(3);
    if (sb_fill != LBD)
      fail($sformatf("fill after reserved EoL slot is %0d, expected %0d", sb_fill, LBD));
    if ((sb_q.size() > 0) && (sb_q[sb_q.size() - 1].eol !== 1'b1))
      fail("last stored entry in full buffer is not the end-of-line entry");
    wait_empty();
    require_hit(cov_norm_ref_lbdm1, "normal beat refused at PARA_LINE_BUFFER_DEPTH - 1");
    require_hit(cov_eol_acc_lbdm1, "EoL beat accepted at PARA_LINE_BUFFER_DEPTH - 1");
    require_hit(cov_eol_ref_lbd, "EoL beat refused at PARA_LINE_BUFFER_DEPTH");
    require_hit(cov_norm_ref_lbd, "normal beat refused at PARA_LINE_BUFFER_DEPTH");
    require_hit(cov_consec_ovf, "consecutive overflow cycles");
    require_hit(cov_budget_ok, "reaction budget PARA_LINE_BUFFER_DEPTH - 1 - PARA_FULL_WATERMARK");
  endtask

  task automatic write_with_consume(input int level, input logic eol);
    fill_to(level);
    rd_mode = RD_MANUAL;
    step();
    drive_entry(rand_entry(1'b0, 1'b0, eol), 1'b1);
    i_fb_ready = 1'b1;
    step();
    drive_idle();
    i_fb_ready = 1'b0;
    step(3);
    rd_mode = RD_ONE;
    wait_empty();
  endtask

  // Simultaneous write and consume at fill 1, FW - 1, FW, DEPTH - 1, DEPTH, and near-empty.
  task automatic test_sim_write_consume();
    int levels [5];
    levels = '{1, FW - 1, FW, LBD - 1, LBD};
    foreach (levels[i]) begin
      if (levels[i] >= 1) begin
        write_with_consume(levels[i], 1'b0);
        write_with_consume(levels[i], 1'b1);
      end
    end
    rd_mode = RD_RAND;
    rd_pct  = 50;
    for (int i = 0; i < 200; i++) begin
      send(rand_entry(1'b0, 1'b0, (i % 10) == 9));
      if ($urandom_range(1, 0) == 1) idle($urandom_range(3, 1));
    end
    wait_empty();
    require_hit(cov_wrcons_at[0], "write + consume at fill 1");
    if (FW >= 2) require_hit(cov_wrcons_at[1], "write + consume at FW - 1");
    require_hit(cov_wrcons_at[2], "write + consume at FW");
    require_hit(cov_wrcons_at[3], "write + consume at PARA_LINE_BUFFER_DEPTH - 1");
    require_hit(cov_wrcons_at[4], "write + consume at PARA_LINE_BUFFER_DEPTH");
  endtask

  // More than 3 * PARA_LINE_BUFFER_DEPTH accepted entries without clear to wrap both pointers.
  task automatic test_pointer_wrap();
    int i;
    i = 0;
    while ((sb_accepted_since_clear < (3 * LBD + 7)) && (i < (40 * LBD))) begin
      if ((i % (2 * LBD)) == 0) rd_mode = RD_ZERO;
      if ((rd_mode == RD_ZERO) && (sb_fill >= (LBD - 4))) begin
        rd_mode = RD_RAND;
        rd_pct  = 70;
      end
      send(rand_entry(1'b0, (i % 37) == 0, (i % 37) == 36, 2'(i / 37)));
      if ($urandom_range(3, 0) == 0) idle(1);
      i++;
    end
    wait_empty();
    require_hit(cov_wrap, "more than 2 * PARA_LINE_BUFFER_DEPTH entries without clear");
  endtask

  // TPG-path source behavior: count held at 4, valid gated by !linebuffer_full, no overflow.
  task automatic test_tpg_source();
    tb_entry_t e;
    int        len;
    int        held;
    int        guard;
    i_px_valid_count  = 3'd4;
    chk_tpg_mode      = 1'b1;
    prev_full_for_tpg = 1'b0;
    for (int l = 0; l < 12; l++) begin
      rd_mode = ((l % 3) == 1) ? RD_ZERO : RD_RAND;
      rd_pct  = $urandom_range(79, 20);
      len     = (rd_mode == RD_ZERO) ? (FW + 8) : (LBD / 2 + 3);
      held    = 0;
      guard   = 0;
      for (int i = 0; (i < len) && (guard < (64 * LBD)); guard++) begin
        e       = rand_entry((l == 0) && (i == 0), i == 0, i == (len - 1), 2'd0, 1'b0);
        e.count = 3'd4;
        step();
        drive_entry(e, o_linebuffer_full === 1'b0);
        if (o_linebuffer_full === 1'b0) i++;
        if ((rd_mode == RD_ZERO) && (o_linebuffer_full === 1'b1)) held++;
        if ((rd_mode == RD_ZERO) && (held >= 16)) begin
          rd_mode = RD_RAND;
          rd_pct  = 30;
        end
      end
      if (guard >= (64 * LBD)) fail("TPG-path line did not complete");
    end
    step();
    i_px_valid       = 1'b0;
    i_px_valid_count = 3'd4;
    step();
    chk_tpg_mode     = 1'b0;
    wait_empty();
    require_hit(cov_tpg_hold, "TPG source held at watermark");
  endtask

  // Out-of-range/unpoliced input combinations: odd counts 1/3/5/6/7, flags on count 0, etc.
  task automatic test_unpoliced_inputs();
    tb_entry_t  e;
    logic [2:0] odd [5];
    odd        = '{3'd1, 3'd3, 3'd5, 3'd6, 3'd7};
    chk_robust = 1'b1;
    rd_mode    = RD_RAND;
    rd_pct     = 50;
    for (int r = 0; r < 20; r++) begin
      foreach (odd[i]) begin
        e       = rand_entry();
        e.count = odd[i];
        send(e);
      end
      e       = rand_entry(1'b1, 1'b1, 1'b1);
      e.count = 3'd0;
      e.err   = 1'b1;
      send(e);
      e       = rand_entry();
      e.err   = 1'b1;
      send(e);
      e             = rand_entry();
      e.count       = 3'd2;
      e.data[47:24] = 24'($urandom) | 24'h1;
      send(e);
    end
    wait_empty();
    chk_robust = 1'b0;
    foreach (odd[i])
      require_hit(cov_robust_cnt[odd[i]], $sformatf("count %0d stored and returned", odd[i]));
  endtask

  // Long random traffic with random resetn, soft_reset, and lb_flush assertions.
  task automatic test_random_stress_clears();
    int n;
    int r;
    n = (30 * LBD < 20000) ? 20000 : ((30 * LBD < 60000) ? (30 * LBD) : 60000);
    for (int i = 0; i < n; i++) begin
      if ((i % 97) == 0) begin
        case ($urandom_range(3, 0))
          0: rd_mode = RD_ONE;
          1: rd_mode = RD_ZERO;
          default: begin
            rd_mode = RD_RAND;
            rd_pct  = $urandom_range(100, 0);
          end
        endcase
      end
      r = $urandom_range(999, 0);
      if (r < 3) begin
        apply_clear(0);
      end else if (r < 8) begin
        apply_clear(1, $urandom_range(2, 1));
      end else if (r < 14) begin
        apply_clear(2);
      end else if (r < 650) begin
        send(rand_entry($urandom_range(63, 0) == 0, $urandom_range(15, 0) == 0,
                        $urandom_range(15, 0) == 0, 2'($urandom_range(1, 0)),
                        1'($urandom_range(1, 0))));
      end else begin
        idle(1);
      end
    end
    wait_empty();
    require_hit(cov_clear_state[0][0] + cov_clear_state[0][1], "resetn during random stress");
    require_hit(cov_clear_state[1][1] + cov_clear_state[1][2], "soft_reset during random stress");
    require_hit(cov_clear_state[2][1] + cov_clear_state[2][2], "lb_flush during random stress");
  endtask

  localparam logic [47:0] MARKER = 48'hDEAD_BEEF_CAFE;
  bit markers_cleared = 1'b0;

  always @(posedge i_sys_clk) begin
    if (chk_en && markers_cleared && (o_fb_px_valid === 1'b1) &&
        (o_fb_px_data[47:12] == MARKER[47:12])) begin
      fail("stale marker entry written before clear reappeared on output");
    end
  end

  task automatic marker_fill();
    tb_entry_t e;
    markers_cleared = 1'b0;
    rd_mode         = RD_RAND;
    rd_pct          = 30;
    for (int i = 0; i < (LBD + LBD / 2); i++) begin
      e       = rand_entry();
      e.count = 3'd4;
      e.data  = MARKER ^ 48'(i);
      send(e);
    end
    fill_to(LBD - 1);
  endtask

  // Pre-clear memory contents never reappear after resetn, soft_reset, or lb_flush.
  task automatic test_no_stale_entries();
    for (int kind = 0; kind < 3; kind++) begin
      marker_fill();
      apply_clear(kind);
      markers_cleared = 1'b1;
      drive_idle();
      rd_mode = RD_ONE;
      for (int i = 0; i < 10; i++) begin
        step();
        if ((o_fb_px_valid !== 1'b0) || (o_linebuf_empty !== 1'b1))
          fail("entry presented after clear with nothing written");
      end
      send_line(3, 2'd1);
      wait_empty();
      marker_fill();
      apply_clear(kind);
      markers_cleared = 1'b1;
      send_line(LBD + 5, 2'd2, 1'b1);
      if (sb_accepted_since_clear > 0) cov_after_clear_write++;
      wait_empty();
    end
    markers_cleared = 1'b0;
    require_hit(cov_after_clear_write, "write after clear over filled memory");
  endtask

  task automatic run_ready_pattern(input string pat);
    rd_mode = RD_MANUAL;
    foreach (pat[i]) begin
      step();
      i_fb_ready = (pat[i] == "1");
      drive_entry(rand_entry(1'b0, i == 0, i == (pat.len() - 1)), 1'b1);
    end
    step();
    drive_idle();
    rd_mode = RD_ONE;
    wait_empty();
  endtask

  // Directed fb_ready patterns exercising out_reg, skid_reg, promote, and single-cycle stalls.
  task automatic test_output_stage_corners();
    run_ready_pattern("0000111111111111");
    run_ready_pattern("1111110111111111");
    run_ready_pattern("1111001111001111");
    run_ready_pattern("1010101010101010");
    run_ready_pattern("0011011100111011");
    run_ready_pattern("1111111111111101");
    require_hit(cov_cons_land, "consume and landing read in the same cycle");
    require_hit(cov_land_skid_stall, "landing into skid_reg under stall");
    require_hit(cov_single_stall, "single-cycle fb_ready=0 in a stream");
  endtask

  // Negative self-check of the comparator and scoreboard: must fail with errors.
  task automatic test_checker_fault();
    fault_inject_arm = 1'b1;
    rd_mode          = RD_ONE;
    send_line(4, 2'd0, 1'b0, 1'b1);
    wait_empty();
    fault_inject_arm = 1'b0;
  endtask

  // ----------------------------------------------------------------------------------------------
  // Test dispatcher and final result summary
  // ----------------------------------------------------------------------------------------------
  initial begin : p_main
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "test_interface";
    if (!$value$plusargs("SEED=%d", seed))      seed      = 1;
    void'($urandom(seed));

    do_reset();

    case (test_name)
      "test_interface":               test_interface();
      "test_async_resetn":            test_async_resetn();
      "test_soft_reset":              test_soft_reset();
      "test_lb_flush":                test_lb_flush();
      "test_write_qualification":     test_write_qualification();
      "test_latency_throughput":      test_latency_throughput();
      "test_random_backpressure":     test_random_backpressure();
      "test_field_passthrough":       test_field_passthrough();
      "test_occupancy_thresholds":    test_occupancy_thresholds();
      "test_overflow_eol_slot":       test_overflow_eol_slot();
      "test_sim_write_consume":       test_sim_write_consume();
      "test_pointer_wrap":            test_pointer_wrap();
      "test_tpg_source":              test_tpg_source();
      "test_unpoliced_inputs":        test_unpoliced_inputs();
      "test_random_stress_clears":    test_random_stress_clears();
      "test_no_stale_entries":        test_no_stale_entries();
      "test_output_stage_corners":    test_output_stage_corners();
      "test_checker_fault":           test_checker_fault();
      default: fail($sformatf("unknown TEST=%s", test_name));
    endcase

    idle(4);
    $display("edges checked=%0d, entries accepted=%0d, entries compared=%0d, errors=%0d",
             edge_checks, entries_accepted, entries_compared, err_count);
    if ((err_count == 0) && (edge_checks > 0) && (entries_compared > 0))
      $display("%s PASS", test_name);
    else
      $display("%s FAIL (errors=%0d)", test_name, err_count);
    $finish;
  end

  initial begin : p_watchdog
    #(T_CLK * MAX_CYCLES);
    $error("watchdog timeout after %0d cycles", MAX_CYCLES);
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
