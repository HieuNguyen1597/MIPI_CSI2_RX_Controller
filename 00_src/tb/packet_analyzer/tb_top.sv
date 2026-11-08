// File        : tb_top.sv
// Description : Testbench of m_fpt_packet_analyzer (header ECC, payload CRC, Data Type routing,
//               frame supervision, Generic Short Packet FIFO, Packet FSM).
//               A source models the show-ahead FIFO ahead of the block. A cycle-based reference
//               model, written from the specification and the MIPI CSI-2 v1.3 ECC and CRC rules,
//               predicts every output cycle by cycle; a checker compares the outputs with it at each
//               sys_clk edge. A second set of checkers tests protocol rules that hold whatever the
//               traffic: rd_en, pulse widths, valid qualifiers, byte enables, sticky flags, reset
//               values and X/Z. Tests only drive stimulus.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions (agreed with the block owner; the specification does not state them yet):
//   - The FIFO ahead of the block is show-ahead: while rd_empty is low, rd_data is the head entry
//     and a cycle with rd_en consumes it. That FIFO is held flushed while core_enable is low, and
//     the testbench also flushes it on soft_reset and resetn.
//   - rd_en = resetn AND NOT rd_empty AND core_enable AND NOT soft_reset, in every cycle.
//   - Output timing, with r the cycle in which an entry is read and L = PARA_ECC_LATENCY: payload
//     beat of a word at r + 2 + L; header events of word 0 (fc_*, err_ecc_*, err_frame_*,
//     err_unsupported_dt, sticky flags, FIFO write) at r + 3 + L; err_wc at r + 2 + L of the word
//     carrying eob, or with the header events when eob is on word 0; err_crc at r + 3 + L of the
//     word completing the footer; err_lane_abort at r + 2 + L of the marker; force_discard acts on
//     a beat when it is high at r + 1 + L of that word.
//   - packet_done_pulse comes exactly once per long packet whose header passed ECC, between the
//     header events (r + 3 + L of word 0) and r + 4 + L of the last entry of the burst.
//   - err_wc is raised when eob arrives before both CRC bytes have been received, also when only
//     CRC bytes are missing. Words after a complete footer are ignored.
//   - err_frame_sync for a long packet outside a frame is raised for image and embedded packets
//     only. Null, Blanking and unsupported Data Types keep their own handling outside a frame.
//   - A packet whose Data Type equals metadata_dt while metadata_capture_en is clear, and which
//     is not Null, Blanking, RAW10 or RAW12, is dropped silently (no error, no flag).
//   - dt_conflict is set in the cycle after any cycle with the conflicting setting and held until
//     soft_reset or resetn. After soft_reset it is set again one cycle later if the setting holds.
//   - core_enable low, or soft_reset, abandons the packet in progress: no beat, event pulse or
//     packet_done_pulse of it from the next cycle on, and both frame contexts are closed.
//     core_enable low keeps the FIFO and the sticky flags; soft_reset clears them.
//   - Two bursts, or a burst and an abort marker, are at least L + 2 cycles apart at the FIFO
//     read port. line_count is not implemented.
//   - Frame Start, Frame End and generic short packets are taken only when the header has no
//     uncorrectable error. A generic short packet entry is laid out like GENERIC_SHORT_PACKET:
//     data_type in bits 5:0, virtual_channel in 7:6, data in 23:8. Codes 0x02 to 0x07 are ignored.
//   - A write into a full Generic Short Packet FIFO in the same cycle as a gsp_pop is accepted.
//   - err_vc carries the header Virtual Channel with err_ecc_1bit, which may be 2 or 3.
//   - Data buses, fc_frame_num, fc_vc and err_vc are 0 outside their valid cycle; bp_vc_tag,
//     bp_dt_sel and emb_vc hold the value of the current packet.
//   - A packet that ends in the cycle its header resolves (Word Count 1 or 2, or a burst cut after
//     word 1) is retired normally.
//   - A partial beat without eol is the last beat of its line. Line checks restart after any cycle
//     with force_discard set, since a suppressed sol or eol beat cannot be seen.
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_SP_FIFO_DEPTH = 8;
  parameter logic [31:0] PARA_ECC_LATENCY   = 1;

  localparam int L     = int'(PARA_ECC_LATENCY);
  localparam int DEPTH = int'(PARA_SP_FIFO_DEPTH);

  // sys_clk, 200 MHz.
  localparam real T_SYS_CLK = 5.0;

  // Data Types.
  localparam logic [5:0] DT_FS     = 6'h00;
  localparam logic [5:0] DT_FE     = 6'h01;
  localparam logic [5:0] DT_NULL   = 6'h10;
  localparam logic [5:0] DT_BLANK  = 6'h11;
  localparam logic [5:0] DT_EMB    = 6'h12;
  localparam logic [5:0] DT_RAW10  = 6'h2B;
  localparam logic [5:0] DT_RAW12  = 6'h2C;

  // Payload class of a long packet.
  localparam int CLS_NONE = 0;
  localparam int CLS_IMG  = 1;
  localparam int CLS_EMB  = 2;

  // DUT inputs.
  logic        i_sys_clk, i_resetn, i_soft_reset, i_core_enable;
  logic [5:0]  i_metadata_dt;
  logic        i_metadata_capture_en;
  logic [37:0] i_rd_data;
  logic        i_rd_empty;
  logic [1:0]  i_force_discard;
  logic        i_gsp_pop;

  // DUT outputs.
  logic        o_rd_en;
  logic [31:0] o_bp_data;
  logic [3:0]  o_bp_byte_en;
  logic        o_bp_valid, o_bp_sof, o_bp_sol, o_bp_eol;
  logic [1:0]  o_bp_vc_tag;
  logic        o_bp_dt_sel;
  logic [31:0] o_emb_data;
  logic [3:0]  o_emb_byte_en;
  logic        o_emb_valid, o_emb_sol, o_emb_eol;
  logic [1:0]  o_emb_vc;
  logic        o_fc_fs, o_fc_fe;
  logic [15:0] o_fc_frame_num;
  logic [1:0]  o_fc_vc;
  logic [23:0] o_gsp_data;
  logic        o_sp_fifo_not_empty, o_sp_fifo_full, o_sp_fifo_overflow;
  logic        o_null_blank_seen, o_packet_done_pulse;
  logic        o_err_ecc_1bit, o_err_ecc_2bit, o_err_crc, o_err_wc, o_err_unsupported_dt;
  logic [1:0]  o_err_vc;
  logic        o_err_lane_abort, o_err_frame_sync, o_err_frame_level;
  logic        o_vc_unsupported_seen, o_dt_conflict;

  m_fpt_packet_analyzer #(
    .PARA_SP_FIFO_DEPTH (PARA_SP_FIFO_DEPTH),
    .PARA_ECC_LATENCY   (PARA_ECC_LATENCY)
  ) u_dut (
    .i_sys_clk             (i_sys_clk),
    .i_resetn              (i_resetn),
    .i_soft_reset          (i_soft_reset),
    .i_core_enable         (i_core_enable),
    .i_metadata_dt         (i_metadata_dt),
    .i_metadata_capture_en (i_metadata_capture_en),
    .i_rd_data             (i_rd_data),
    .i_rd_empty            (i_rd_empty),
    .o_rd_en               (o_rd_en),
    .i_force_discard       (i_force_discard),
    .o_bp_data             (o_bp_data),
    .o_bp_byte_en          (o_bp_byte_en),
    .o_bp_valid            (o_bp_valid),
    .o_bp_sof              (o_bp_sof),
    .o_bp_sol              (o_bp_sol),
    .o_bp_eol              (o_bp_eol),
    .o_bp_vc_tag           (o_bp_vc_tag),
    .o_bp_dt_sel           (o_bp_dt_sel),
    .o_emb_data            (o_emb_data),
    .o_emb_byte_en         (o_emb_byte_en),
    .o_emb_valid           (o_emb_valid),
    .o_emb_sol             (o_emb_sol),
    .o_emb_eol             (o_emb_eol),
    .o_emb_vc              (o_emb_vc),
    .o_fc_fs               (o_fc_fs),
    .o_fc_fe               (o_fc_fe),
    .o_fc_frame_num        (o_fc_frame_num),
    .o_fc_vc               (o_fc_vc),
    .o_gsp_data            (o_gsp_data),
    .i_gsp_pop             (i_gsp_pop),
    .o_sp_fifo_not_empty   (o_sp_fifo_not_empty),
    .o_sp_fifo_full        (o_sp_fifo_full),
    .o_sp_fifo_overflow    (o_sp_fifo_overflow),
    .o_null_blank_seen     (o_null_blank_seen),
    .o_packet_done_pulse   (o_packet_done_pulse),
    .o_err_ecc_1bit        (o_err_ecc_1bit),
    .o_err_ecc_2bit        (o_err_ecc_2bit),
    .o_err_crc             (o_err_crc),
    .o_err_wc              (o_err_wc),
    .o_err_unsupported_dt  (o_err_unsupported_dt),
    .o_err_vc              (o_err_vc),
    .o_err_lane_abort      (o_err_lane_abort),
    .o_err_frame_sync      (o_err_frame_sync),
    .o_err_frame_level     (o_err_frame_level),
    .o_vc_unsupported_seen (o_vc_unsupported_seen),
    .o_dt_conflict         (o_dt_conflict)
  );

  // --------------------------------------------------------------------------------------------
  // Clock, error reporting and small helpers
  // --------------------------------------------------------------------------------------------
  // cyc is the number of the next sampling edge: inputs set after edge k - 1 are sampled at edge
  // k, and the outputs seen at edge k are the outputs of cycle k.
  int          cyc = 0;
  bit          started = 1'b0;
  int unsigned err_count;
  int unsigned check_count;

  initial begin
    i_sys_clk = 1'b0;
    forever #(T_SYS_CLK / 2.0) i_sys_clk = ~i_sys_clk;
  end

  task automatic fail(input string msg);
    err_count++;
    if (err_count <= 30) $error("cycle %0d: %s", cyc, msg);
  endtask

  // Wait n rising edges, plus 0.5 ns. Tests change inputs at that point.
  task automatic cycles(input int n);
    repeat (n) @(posedge i_sys_clk);
    #0.5;
  endtask

  function automatic int min2(input int a, input int b);
    return (a < b) ? a : b;
  endfunction

  function automatic int ones4(input logic [3:0] v);
    return int'(v[0]) + int'(v[1]) + int'(v[2]) + int'(v[3]);
  endfunction

  function automatic int rand_range(input int lo, input int hi);
    return int'($urandom_range(hi, lo));
  endfunction

  // --------------------------------------------------------------------------------------------
  // CSI-2 v1.3 header ECC (Table 5) and payload CRC-16 (Figure 54)
  // --------------------------------------------------------------------------------------------
  // Parity bits of the 24-bit header {WC MS byte, WC LS byte, Data Identifier}.
  function automatic logic [5:0] ecc_parity(input logic [23:0] d);
    logic [5:0] p;
    p[0] = d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[10] ^ d[11] ^ d[13] ^ d[16] ^ d[20] ^
           d[21] ^ d[22] ^ d[23];
    p[1] = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[10] ^ d[12] ^ d[14] ^ d[17] ^ d[20] ^
           d[21] ^ d[22] ^ d[23];
    p[2] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[9] ^ d[11] ^ d[12] ^ d[15] ^ d[18] ^ d[20] ^
           d[21] ^ d[22];
    p[3] = d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[19] ^ d[20] ^
           d[21] ^ d[23];
    p[4] = d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^
           d[22] ^ d[23];
    p[5] = d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^
           d[21] ^ d[22] ^ d[23];
    return p;
  endfunction

  // CRC-16, x^16 + x^12 + x^5 + 1, bits processed least significant first.
  function automatic logic [15:0] crc_byte(input logic [15:0] c, input logic [7:0] b);
    logic [15:0] x;
    x = c;
    for (int i = 0; i < 8; i++) begin
      if (x[0] ^ b[i]) x = (x >> 1) ^ 16'h8408;
      else             x = x >> 1;
    end
    return x;
  endfunction

  // Header decode: returns the corrected 24 header bits and the error class
  // (0 = none, 1 = single-bit error corrected or in the ECC byte, 2 = uncorrectable).
  task automatic ecc_decode(input logic [31:0] w, output logic [23:0] d, output int cls);
    logic [5:0] syn;
    d   = w[23:0];
    syn = w[29:24] ^ ecc_parity(w[23:0]);
    cls = 0;
    if (syn != 6'h00) begin
      cls = 2;
      for (int i = 0; i < 24; i++) begin
        if (syn == ecc_parity(24'(1) << i)) begin
          d[i] = ~d[i];
          cls  = 1;
        end
      end
      if ((syn & (syn - 6'd1)) == 6'h00) cls = 1;
    end
  endtask

  // --------------------------------------------------------------------------------------------
  // Source: show-ahead FIFO ahead of the block
  // --------------------------------------------------------------------------------------------
  // One item per entry {eob, sob, byte_valid, data}; an idle item is one cycle with rd_empty high.
  typedef struct packed {
    logic        idle;
    logic [37:0] ent;
  } src_item_t;
  src_item_t src_q[$];

  // Stimulus settings.
  int g_space    = 0;  // extra empty cycles after each burst (the minimum gap is L + 2)
  int g_hole     = 0;  // up to g_hole empty cycles between two words of a burst
  int g_hdr_hole = 0;  // empty cycles between word 0 and word 1
  bit g_pop_with_write = 1'b0;  // pulse gsp_pop in the FIFO write cycle of each generic packet

  logic [7:0] bq[$];   // bytes of the burst being built
  logic [7:0] pl[$];   // payload of the next long packet

  // Pending gsp_pop pulses, by cycle.
  bit pop_req[int];

  task automatic push_gap(input int n);
    for (int i = 0; i < n; i++) src_q.push_back('{idle: 1'b1, ent: '0});
  endtask

  // Packs bq into entries, byte 0 in data[7:0], then leaves the minimum gap.
  task automatic push_burst();
    int nw;
    nw = (bq.size() + 3) / 4;
    for (int w = 0; w < nw; w++) begin
      logic [31:0] d;
      logic [3:0]  bv;
      d  = '0;
      bv = '0;
      for (int b = 0; b < 4; b++) begin
        if (4 * w + b < bq.size()) begin
          d[8*b +: 8] = bq[4 * w + b];
          bv[b]       = 1'b1;
        end
      end
      src_q.push_back('{idle: 1'b0, ent: {(w == nw - 1), (w == 0), bv, d}});
      if (w != nw - 1) begin
        if (w == 0 && g_hdr_hole > 0) push_gap(g_hdr_hole);
        else if (g_hole > 0) push_gap(rand_range(0, g_hole));
      end
    end
    push_gap(L + 2 + g_space);
  endtask

  // Header bytes {DI, WC LS, WC MS, ECC} into bq. flip inverts code bits: header bits 23:0 are
  // flip[23:0], ECC bits 5:0 are flip[29:24]. ecc76 sets the two unused ECC bits.
  task automatic header_bytes(input logic [1:0] vc, input logic [5:0] dt, input logic [15:0] f2,
                              input logic [29:0] flip, input logic [1:0] ecc76);
    logic [23:0] d;
    logic [31:0] w;
    d = {f2, vc, dt};
    w = {ecc76, ecc_parity(d), d} ^ {2'b00, flip};
    bq.delete();
    for (int b = 0; b < 4; b++) bq.push_back(w[8*b +: 8]);
  endtask

  // Short packet; extra adds bytes to the same burst.
  task automatic send_short(input logic [1:0] vc, input logic [5:0] dt, input logic [15:0] data,
                            input logic [29:0] flip = '0, input logic [1:0] ecc76 = 2'b00,
                            input int extra = 0);
    header_bytes(vc, dt, data, flip, ecc76);
    for (int i = 0; i < extra; i++) bq.push_back(8'($urandom));
    push_burst();
  endtask

  // Long packet with payload pl. crc_xor corrupts the footer, bad_byte corrupts that payload byte
  // after the CRC is computed, cut keeps the first cut bytes of the burst (header included,
  // -1 = all), extra adds bytes after the footer.
  task automatic send_long(input logic [1:0] vc, input logic [5:0] dt,
                           input logic [15:0] crc_xor = '0, input int cut = -1,
                           input logic [29:0] flip = '0, input int extra = 0,
                           input logic [1:0] ecc76 = 2'b00, input int bad_byte = -1);
    logic [15:0] c;
    c = 16'hFFFF;
    foreach (pl[i]) c = crc_byte(c, pl[i]);
    c = c ^ crc_xor;
    header_bytes(vc, dt, 16'(pl.size()), flip, ecc76);
    foreach (pl[i]) bq.push_back((i == bad_byte) ? (pl[i] ^ 8'h10) : pl[i]);
    bq.push_back(c[7:0]);
    bq.push_back(c[15:8]);
    for (int i = 0; i < extra; i++) bq.push_back(8'($urandom));
    if (cut >= 0) while (bq.size() > cut) void'(bq.pop_back());
    push_burst();
  endtask

  task automatic random_payload(input int n);
    pl.delete();
    for (int i = 0; i < n; i++) pl.push_back(8'($urandom));
  endtask

  task automatic send_line(input logic [1:0] vc, input logic [5:0] dt, input int wc);
    random_payload(wc);
    send_long(vc, dt);
  endtask

  task automatic send_marker();
    src_q.push_back('{idle: 1'b0, ent: {1'b1, 1'b1, 4'b0000, 32'h0}});
    push_gap(L + 2 + g_space);
  endtask

  task automatic frame_start(input logic [1:0] vc, input logic [15:0] fn = 16'h0001,
                             input logic [29:0] flip = '0);
    send_short(vc, DT_FS, fn, flip);
  endtask

  task automatic frame_end(input logic [1:0] vc, input logic [15:0] fn = 16'h0001,
                           input logic [29:0] flip = '0);
    send_short(vc, DT_FE, fn, flip);
  endtask

  // gsp_pop for one cycle, in the cycle sampled at the next edge.
  task automatic pop_pulse();
    pop_req[cyc] = 1'b1;
  endtask

  task automatic soft_reset_pulse(input int n = 1);
    i_soft_reset = 1'b1;
    cycles(n);
    i_soft_reset = 1'b0;
  endtask

  // The source and gsp_pop change at the falling edge.
  always @(negedge i_sys_clk) begin
    i_rd_empty = (src_q.size() == 0) || src_q[0].idle;
    i_rd_data  = (src_q.size() != 0 && !src_q[0].idle) ? src_q[0].ent : '0;
    i_gsp_pop  = pop_req.exists(cyc);
    if (pop_req.exists(cyc)) pop_req.delete(cyc);
  end

  // --------------------------------------------------------------------------------------------
  // Reference model
  // --------------------------------------------------------------------------------------------
  // Expected beat and pulse outputs of one cycle; every field not set is expected to be 0.
  typedef struct packed {
    logic        bp_valid, bp_sof, bp_sol, bp_eol;
    logic [31:0] bp_data;
    logic [3:0]  bp_byte_en;
    logic [1:0]  bp_vc_tag;
    logic        bp_dt_sel;
    logic        emb_valid, emb_sol, emb_eol;
    logic [31:0] emb_data;
    logic [3:0]  emb_byte_en;
    logic [1:0]  emb_vc;
    logic        fc_fs, fc_fe;
    logic [15:0] fc_frame_num;
    logic [1:0]  fc_vc;
    logic        err_ecc_1bit, err_ecc_2bit, err_crc, err_wc, err_unsupported_dt;
    logic        err_lane_abort, err_frame_sync, err_frame_level;
    logic [1:0]  err_vc;
    logic        sp_fifo_overflow;
  } ev_t;
  ev_t exp_ev[int];

  function automatic ev_t ev_at(input int c);
    if (exp_ev.exists(c)) return exp_ev[c];
    return '0;
  endfunction

  // Entries read, by read cycle, with the burst number they belong to.
  logic [37:0] rd_hist[int];
  int          rd_burst[int];
  int          burst_num;
  int          burst_last[int];

  // packet_done_pulse windows: lo..hi, hi = -1 while the burst has not ended.
  typedef struct {
    int lo;
    int hi;
    int burst;
  } win_t;
  win_t win_q[$];

  // Packet whose payload is being received.
  bit          m_armed;
  logic [1:0]  m_vc;
  int          m_cls;
  bit          m_sof, m_sol_pend;
  logic        m_raw10;
  int          m_rem, m_fcnt;
  logic [7:0]  m_crc_lo, m_crc_hi;
  logic [15:0] m_crc;

  // Per-channel frame context: frame_active, first_image_seen, pkt_err_in_frame.
  logic [1:0]  m_fa, m_fis, m_perr;

  // Generic Short Packet FIFO and sticky flags.
  logic [23:0] m_gsp[$];
  bit          m_gsp_wr;
  logic [23:0] m_gsp_wdata;
  bit          m_nbs, m_vcu, m_dtc;

  // Level outputs expected in the current cycle.
  logic [23:0] x_gsp_data;
  bit          x_ne, x_full, x_nbs, x_vcu, x_dtc;

  task automatic model_reset();
    rd_hist.delete();
    rd_burst.delete();
    burst_last.delete();
    exp_ev.delete();
    win_q.delete();
    m_armed = 1'b0;
    m_fa    = '0;
    m_fis   = '0;
    m_perr  = '0;
    m_gsp.delete();
    m_nbs = 1'b0;
    m_vcu = 1'b0;
    m_dtc = 1'b0;
    x_gsp_data = '0;
    {x_ne, x_full, x_nbs, x_vcu, x_dtc} = '0;
  endtask

  // Header of a burst resolved in cycle k; its events appear at k + 1.
  task automatic do_header(input int k, input logic [37:0] e, input int burst);
    logic [23:0] d;
    int          cls;
    logic [1:0]  vc;
    logic [5:0]  dt;
    logic [15:0] f2;
    bit          is_meta, is_emb, is_drop, is_null, is_img, is_uns;
    ev_t         nx;
    nx = ev_at(k + 1);
    ecc_decode(e[31:0], d, cls);
    m_armed = 1'b0;
    if (cls == 2) begin
      nx.err_ecc_2bit = 1'b1;
      exp_ev[k + 1] = nx;
      return;
    end
    vc = d[7:6];
    dt = d[5:0];
    f2 = d[23:8];
    if (cls == 1) begin
      nx.err_ecc_1bit = 1'b1;
      nx.err_vc       = vc;
    end
    // Short packets.
    if (dt[5:4] == 2'b00) begin
      if (vc[1]) m_vcu = 1'b1;
      else if (dt == DT_FS) begin
        nx.fc_fs = 1'b1; nx.fc_frame_num = f2; nx.fc_vc = vc;
        if (m_fa[vc[0]]) begin nx.err_frame_sync = 1'b1; nx.err_vc = vc; end
        m_fa[vc[0]]   = 1'b1;
        m_fis[vc[0]]  = 1'b0;
        m_perr[vc[0]] = 1'b0;
      end else if (dt == DT_FE) begin
        nx.fc_fe = 1'b1; nx.fc_frame_num = f2; nx.fc_vc = vc;
        if (!m_fa[vc[0]]) begin
          nx.err_frame_sync = 1'b1; nx.err_vc = vc;
        end else begin
          if (m_perr[vc[0]]) begin nx.err_frame_level = 1'b1; nx.err_vc = vc; end
          m_fa[vc[0]] = 1'b0;
        end
      end else if (dt[3]) begin
        m_gsp_wr    = 1'b1;
        m_gsp_wdata = {f2, vc, dt};
      end
      exp_ev[k + 1] = nx;
      return;
    end
    // Long packet with a good header: one packet_done_pulse.
    win_q.push_back('{lo: k + 1, hi: burst_last.exists(burst) ? burst_last[burst] + 4 + L : -1,
                      burst: burst});
    if (vc[1]) begin
      m_vcu = 1'b1;
      exp_ev[k + 1] = nx;
      return;
    end
    is_meta = (dt == i_metadata_dt);
    is_emb  = is_meta && i_metadata_capture_en;
    is_null = !is_emb && (dt == DT_NULL || dt == DT_BLANK);
    is_img  = !is_emb && (dt == DT_RAW10 || dt == DT_RAW12);
    is_drop = is_meta && !is_emb && !is_null && !is_img;
    is_uns  = !is_emb && !is_null && !is_img && !is_drop;
    if (is_null) m_nbs = 1'b1;
    if (is_uns) begin nx.err_unsupported_dt = 1'b1; nx.err_vc = vc; end
    if ((is_img || is_emb) && !m_fa[vc[0]]) begin nx.err_frame_sync = 1'b1; nx.err_vc = vc; end
    m_sof = 1'b0;
    if (is_img) begin
      m_sof = !m_fis[vc[0]];
      m_fis[vc[0]] = 1'b1;
    end
    if (e[37]) begin
      // The burst ended on the header: footer missing.
      nx.err_wc = 1'b1; nx.err_vc = vc;
      m_perr[vc[0]] = 1'b1;
    end else begin
      m_armed    = 1'b1;
      m_vc       = vc;
      m_cls      = is_img ? CLS_IMG : (is_emb ? CLS_EMB : CLS_NONE);
      m_raw10    = (dt == DT_RAW10);
      m_sol_pend = 1'b1;
      m_rem      = int'(f2);
      m_fcnt     = 0;
      m_crc      = 16'hFFFF;
    end
    exp_ev[k + 1] = nx;
  endtask

  // Payload word (word 1 or later) reaching Payload Path in cycle k.
  task automatic do_payload(input int k, input logic [37:0] e);
    int          nb, pay, take;
    logic [3:0]  mask;
    logic [31:0] data;
    bit          last, fwd;
    ev_t         nx;
    if (!m_armed) return;
    nx   = ev_at(k + 1);
    data = e[31:0];
    nb   = ones4(e[35:32]);
    pay  = min2(m_rem, nb);
    take = min2(nb - pay, 2 - m_fcnt);
    mask = 4'((1 << pay) - 1);
    last = (pay > 0) && (m_rem == pay);
    for (int b = 0; b < pay; b++) m_crc = crc_byte(m_crc, data[8*b +: 8]);
    if (pay > 0) begin
      fwd = (m_cls != CLS_NONE) && m_fa[m_vc[0]] && !i_force_discard[m_vc[0]];
      if (fwd && m_cls == CLS_IMG) begin
        nx.bp_valid = 1'b1; nx.bp_data = data; nx.bp_byte_en = mask;
        nx.bp_sol = m_sol_pend; nx.bp_eol = last; nx.bp_sof = m_sol_pend && m_sof;
        nx.bp_vc_tag = m_vc; nx.bp_dt_sel = m_raw10;
      end
      if (fwd && m_cls == CLS_EMB) begin
        nx.emb_valid = 1'b1; nx.emb_data = data; nx.emb_byte_en = mask;
        nx.emb_sol = m_sol_pend; nx.emb_eol = last; nx.emb_vc = m_vc;
      end
      m_sol_pend = 1'b0;
    end
    m_rem = m_rem - pay;
    for (int b = pay; b < pay + take; b++) begin
      if (m_fcnt == 0) m_crc_lo = data[8*b +: 8];
      else             m_crc_hi = data[8*b +: 8];
      m_fcnt++;
    end
    if (m_fcnt == 2) begin
      if ({m_crc_hi, m_crc_lo} != m_crc) begin
        ev_t n2;
        n2 = ev_at(k + 2);
        n2.err_crc = 1'b1;
        n2.err_vc  = m_vc;
        exp_ev[k + 2] = n2;
        m_perr[m_vc[0]] = 1'b1;
      end
      m_armed = 1'b0;
    end else if (e[37]) begin
      nx.err_wc = 1'b1;
      nx.err_vc = m_vc;
      m_perr[m_vc[0]] = 1'b1;
      m_armed = 1'b0;
    end
    exp_ev[k + 1] = nx;
  endtask

  task automatic do_abort(input int k);
    ev_t nx;
    nx = ev_at(k + 1);
    nx.err_lane_abort = 1'b1;
    exp_ev[k + 1] = nx;
    m_perr = m_perr | m_fa;
  endtask

  function automatic bit is_marker(input logic [37:0] e);
    return e[37] && e[36] && (e[35:32] == 4'b0000);
  endfunction

  // One model step with the inputs of cycle k; computes the level outputs of cycle k + 1.
  task automatic model_step(input int k);
    bit flush, pop_ok, full0;
    flush    = i_soft_reset || !i_core_enable;
    m_gsp_wr = 1'b0;
    if (flush) begin
      int later[$];
      rd_hist.delete();
      rd_burst.delete();
      foreach (exp_ev[c]) if (c > k) later.push_back(c);
      foreach (later[i]) exp_ev.delete(later[i]);
      win_q.delete();
      m_armed = 1'b0;
      m_fa    = '0;
      m_fis   = '0;
      m_perr  = '0;
    end else begin
      // Word 0 read at k - 2 - L resolves now; words read at k - 1 - L reach Payload Path now.
      int kh, kp;
      kh = k - 2 - L;
      kp = k - 1 - L;
      if (rd_hist.exists(kh) && rd_hist[kh][36] && !is_marker(rd_hist[kh])) begin
        do_header(k, rd_hist[kh], rd_burst[kh]);
        rd_hist.delete(kh);
        rd_burst.delete(kh);
      end
      if (rd_hist.exists(kp)) begin
        if (is_marker(rd_hist[kp])) do_abort(k);
        else if (!rd_hist[kp][36]) do_payload(k, rd_hist[kp]);
        if (!rd_hist[kp][36] || is_marker(rd_hist[kp])) begin
          rd_hist.delete(kp);
          rd_burst.delete(kp);
        end
      end
    end
    // Generic Short Packet FIFO: pop and write use the occupancy at the start of the cycle.
    pop_ok = (i_gsp_pop === 1'b1) && (m_gsp.size() != 0);
    full0  = (m_gsp.size() == DEPTH);
    if (i_soft_reset) m_gsp.delete();
    else begin
      if (m_gsp_wr && full0 && !pop_ok) begin
        ev_t nx;
        nx = ev_at(k + 1);
        nx.sp_fifo_overflow = 1'b1;
        exp_ev[k + 1] = nx;
      end
      if (pop_ok) void'(m_gsp.pop_front());
      if (m_gsp_wr && (!full0 || pop_ok)) m_gsp.push_back(m_gsp_wdata);
    end
    // Sticky flags.
    if (i_soft_reset) begin
      m_nbs = 1'b0;
      m_vcu = 1'b0;
      m_dtc = 1'b0;
    end else if (i_metadata_capture_en &&
                 (i_metadata_dt inside {DT_NULL, DT_BLANK, DT_RAW10, DT_RAW12})) begin
      m_dtc = 1'b1;
    end
    // Entry read in cycle k.
    if (o_rd_en === 1'b1 && i_rd_empty === 1'b0) begin
      logic [37:0] e;
      e = i_rd_data;
      if (e[36] && !is_marker(e)) burst_num++;
      rd_hist[k]  = e;
      rd_burst[k] = burst_num;
      if (e[37] && !is_marker(e)) begin
        burst_last[burst_num] = k;
        foreach (win_q[i]) if (win_q[i].burst == burst_num && win_q[i].hi < 0) win_q[i].hi = k + 4 + L;
      end
      if (g_pop_with_write && e[36] && !is_marker(e) && !e[7] && e[5:3] == 3'b001)
        pop_req[k + 2 + L] = 1'b1;
    end
    x_gsp_data = (m_gsp.size() != 0) ? m_gsp[0] : '0;
    x_ne   = (m_gsp.size() != 0);
    x_full = (m_gsp.size() == DEPTH);
    x_nbs  = m_nbs;
    x_vcu  = m_vcu;
    x_dtc  = m_dtc;
  endtask

  // --------------------------------------------------------------------------------------------
  // Output checker against the model
  // --------------------------------------------------------------------------------------------
  task automatic check_outputs(input int k);
    ev_t e;
    e = ev_at(k);
    check_count++;
    if ({o_bp_valid, o_bp_sof, o_bp_sol, o_bp_eol} !== {e.bp_valid, e.bp_sof, e.bp_sol, e.bp_eol})
      fail($sformatf("bp valid/sof/sol/eol = %b%b%b%b, expected %b%b%b%b", o_bp_valid, o_bp_sof,
                     o_bp_sol, o_bp_eol, e.bp_valid, e.bp_sof, e.bp_sol, e.bp_eol));
    if ({o_bp_data, o_bp_byte_en} !== {e.bp_data, e.bp_byte_en})
      fail($sformatf("bp_data/bp_byte_en = %h/%b, expected %h/%b", o_bp_data, o_bp_byte_en,
                     e.bp_data, e.bp_byte_en));
    if (e.bp_valid && {o_bp_vc_tag, o_bp_dt_sel} !== {e.bp_vc_tag, e.bp_dt_sel})
      fail($sformatf("bp_vc_tag/bp_dt_sel = %0d/%b, expected %0d/%b", o_bp_vc_tag, o_bp_dt_sel,
                     e.bp_vc_tag, e.bp_dt_sel));
    if ({o_emb_valid, o_emb_sol, o_emb_eol} !== {e.emb_valid, e.emb_sol, e.emb_eol})
      fail($sformatf("emb valid/sol/eol = %b%b%b, expected %b%b%b", o_emb_valid, o_emb_sol,
                     o_emb_eol, e.emb_valid, e.emb_sol, e.emb_eol));
    if ({o_emb_data, o_emb_byte_en} !== {e.emb_data, e.emb_byte_en})
      fail($sformatf("emb_data/emb_byte_en = %h/%b, expected %h/%b", o_emb_data, o_emb_byte_en,
                     e.emb_data, e.emb_byte_en));
    if (e.emb_valid && o_emb_vc !== e.emb_vc)
      fail($sformatf("emb_vc = %0d, expected %0d", o_emb_vc, e.emb_vc));
    if ({o_fc_fs, o_fc_fe, o_fc_frame_num, o_fc_vc} !== {e.fc_fs, e.fc_fe, e.fc_frame_num, e.fc_vc})
      fail($sformatf("fc fs/fe/frame_num/vc = %b/%b/%h/%0d, expected %b/%b/%h/%0d", o_fc_fs,
                     o_fc_fe, o_fc_frame_num, o_fc_vc, e.fc_fs, e.fc_fe, e.fc_frame_num, e.fc_vc));
    if ({o_err_ecc_1bit, o_err_ecc_2bit, o_err_crc, o_err_wc, o_err_unsupported_dt,
         o_err_lane_abort, o_err_frame_sync, o_err_frame_level} !==
        {e.err_ecc_1bit, e.err_ecc_2bit, e.err_crc, e.err_wc, e.err_unsupported_dt,
         e.err_lane_abort, e.err_frame_sync, e.err_frame_level})
      fail($sformatf({"errors ecc1/ecc2/crc/wc/unsupported_dt/lane_abort/frame_sync/frame_level",
                      " = %b%b%b%b%b%b%b%b, expected %b%b%b%b%b%b%b%b"},
                     o_err_ecc_1bit, o_err_ecc_2bit, o_err_crc, o_err_wc, o_err_unsupported_dt,
                     o_err_lane_abort, o_err_frame_sync, o_err_frame_level,
                     e.err_ecc_1bit, e.err_ecc_2bit, e.err_crc, e.err_wc, e.err_unsupported_dt,
                     e.err_lane_abort, e.err_frame_sync, e.err_frame_level));
    if (o_err_vc !== e.err_vc) fail($sformatf("err_vc = %0d, expected %0d", o_err_vc, e.err_vc));
    if (o_sp_fifo_overflow !== e.sp_fifo_overflow)
      fail($sformatf("sp_fifo_overflow = %b, expected %b", o_sp_fifo_overflow, e.sp_fifo_overflow));
    if (o_gsp_data !== x_gsp_data)
      fail($sformatf("gsp_data = %h, expected %h", o_gsp_data, x_gsp_data));
    if ({o_sp_fifo_not_empty, o_sp_fifo_full} !== {x_ne, x_full})
      fail($sformatf("sp_fifo_not_empty/full = %b%b, expected %b%b", o_sp_fifo_not_empty,
                     o_sp_fifo_full, x_ne, x_full));
    if ({o_null_blank_seen, o_vc_unsupported_seen, o_dt_conflict} !== {x_nbs, x_vcu, x_dtc})
      fail($sformatf("null_blank_seen/vc_unsupported_seen/dt_conflict = %b%b%b, expected %b%b%b",
                     o_null_blank_seen, o_vc_unsupported_seen, o_dt_conflict, x_nbs, x_vcu,
                     x_dtc));
    // packet_done_pulse: one pulse inside the window of each long packet, oldest window first.
    if (o_packet_done_pulse === 1'b1) begin
      if (win_q.size() != 0 && k >= win_q[0].lo && (win_q[0].hi < 0 || k <= win_q[0].hi))
        void'(win_q.pop_front());
      else fail("packet_done_pulse with no long packet to retire");
    end else if (o_packet_done_pulse !== 1'b0) fail("packet_done_pulse is X or Z");
    while (win_q.size() != 0 && win_q[0].hi >= 0 && k >= win_q[0].hi) begin
      fail($sformatf("packet_done_pulse missing in cycles %0d..%0d", win_q[0].lo, win_q[0].hi));
      void'(win_q.pop_front());
    end
    exp_ev.delete(k);
  endtask

  // Every output is 0 while resetn is low.
  task automatic check_reset_outputs();
    if ({o_rd_en, o_bp_data, o_bp_byte_en, o_bp_valid, o_bp_sof, o_bp_sol, o_bp_eol, o_bp_vc_tag,
         o_bp_dt_sel, o_emb_data, o_emb_byte_en, o_emb_valid, o_emb_sol, o_emb_eol, o_emb_vc,
         o_fc_fs, o_fc_fe, o_fc_frame_num, o_fc_vc, o_gsp_data, o_sp_fifo_not_empty,
         o_sp_fifo_full, o_sp_fifo_overflow, o_null_blank_seen, o_packet_done_pulse,
         o_err_ecc_1bit, o_err_ecc_2bit, o_err_crc, o_err_wc, o_err_unsupported_dt, o_err_vc,
         o_err_lane_abort, o_err_frame_sync, o_err_frame_level, o_vc_unsupported_seen,
         o_dt_conflict} !== '0)
      fail("an output is not 0 while resetn is low");
  endtask

  // Reference model, checker and source, at each rising edge.
  always @(posedge i_sys_clk) begin
    int k;
    k = cyc;
    if (started) begin
      if (i_resetn !== 1'b1) begin
        check_reset_outputs();
        model_reset();
      end else begin
        check_outputs(k);
        model_step(k);
      end
      if (i_resetn !== 1'b1 || i_core_enable !== 1'b1 || i_soft_reset !== 1'b0) src_q.delete();
      else if (src_q.size() != 0 && (src_q[0].idle || o_rd_en === 1'b1)) void'(src_q.pop_front());
    end
    cyc <= cyc + 1;
  end

  // resetn is asynchronous: the outputs go to 0 without waiting for an edge.
  always @(negedge i_resetn) begin
    #0.2;
    if (started) check_reset_outputs();
  end

  // --------------------------------------------------------------------------------------------
  // Protocol checkers (independent of the model)
  // --------------------------------------------------------------------------------------------
  logic [1:0]  p_fd;
  logic        p_flush, p_soft, p_dtc_cond, p_full_nopop;
  logic        p_nbs, p_vcu, p_dtc;
  logic [16:0] p_pulses;
  bit          p_valid_prev = 1'b0;
  // Line state per output: partial beat seen without eol; line open and its attributes.
  bit          bp_short_seen, emb_short_seen;
  bit          bp_in_line, bp_moved;
  logic [2:0]  bp_line_attr;
  // Stimulus format: inside a burst, cycle of the last end of burst or marker.
  bit          s_in_burst;
  int          s_last_end;

  always @(posedge i_sys_clk) begin
    logic [16:0] pulses;
    if (started && i_resetn === 1'b1) begin
      // rd_en.
      if (o_rd_en !== (!i_rd_empty && i_core_enable && !i_soft_reset))
        fail($sformatf("rd_en = %b with rd_empty=%b core_enable=%b soft_reset=%b", o_rd_en,
                       i_rd_empty, i_core_enable, i_soft_reset));
      // No X or Z after reset.
      if ($isunknown({o_rd_en, o_bp_data, o_bp_byte_en, o_bp_valid, o_bp_sof, o_bp_sol, o_bp_eol,
                      o_bp_vc_tag, o_bp_dt_sel, o_emb_data, o_emb_byte_en, o_emb_valid,
                      o_emb_sol, o_emb_eol, o_emb_vc, o_fc_fs, o_fc_fe, o_fc_frame_num, o_fc_vc,
                      o_gsp_data, o_sp_fifo_not_empty, o_sp_fifo_full, o_sp_fifo_overflow,
                      o_null_blank_seen, o_packet_done_pulse, o_err_ecc_1bit, o_err_ecc_2bit,
                      o_err_crc, o_err_wc, o_err_unsupported_dt, o_err_vc, o_err_lane_abort,
                      o_err_frame_sync, o_err_frame_level, o_vc_unsupported_seen,
                      o_dt_conflict}))
        fail("X or Z on an output");
      // Pulses last one cycle.
      pulses = {o_bp_sof, o_bp_sol, o_bp_eol, o_emb_sol, o_emb_eol, o_fc_fs, o_fc_fe,
                o_packet_done_pulse, o_sp_fifo_overflow, o_err_ecc_1bit, o_err_ecc_2bit,
                o_err_crc, o_err_wc, o_err_unsupported_dt, o_err_lane_abort, o_err_frame_sync,
                o_err_frame_level};
      if (p_valid_prev && ((pulses & p_pulses) != '0))
        fail($sformatf("pulse output high in two cycles in a row (%b)", pulses & p_pulses));
      // Flags only with their valid; one valid at a time.
      if ((o_bp_sof || o_bp_sol || o_bp_eol) && !o_bp_valid) fail("bp_sof/sol/eol without bp_valid");
      if ((o_emb_sol || o_emb_eol) && !o_emb_valid) fail("emb_sol/eol without emb_valid");
      if (o_bp_sof && !o_bp_sol) fail("bp_sof without bp_sol");
      if (o_bp_valid && o_emb_valid) fail("bp_valid and emb_valid together");
      // Byte enables contiguous from bit 0; buses 0 when not valid.
      if (o_bp_valid && !(o_bp_byte_en inside {4'b0001, 4'b0011, 4'b0111, 4'b1111}))
        fail($sformatf("bp_byte_en = %b", o_bp_byte_en));
      if (!o_bp_valid && (o_bp_data != '0 || o_bp_byte_en != '0)) fail("bp_data/byte_en not 0 when idle");
      if (o_emb_valid && !(o_emb_byte_en inside {4'b0001, 4'b0011, 4'b0111, 4'b1111}))
        fail($sformatf("emb_byte_en = %b", o_emb_byte_en));
      if (!o_emb_valid && (o_emb_data != '0 || o_emb_byte_en != '0)) fail("emb_data/byte_en not 0 when idle");
      // Supported channels only on the payload interfaces.
      if (o_bp_valid && o_bp_vc_tag[1]) fail("bp_vc_tag outside VC0/VC1");
      if (o_emb_valid && o_emb_vc[1]) fail("emb_vc outside VC0/VC1");
      // Frame events.
      if (o_fc_fs && o_fc_fe) fail("fc_fs and fc_fe together");
      if ((o_fc_fs || o_fc_fe) && o_fc_vc[1]) fail("fc_vc outside VC0/VC1");
      if (!o_fc_fs && !o_fc_fe && (o_fc_frame_num != '0 || o_fc_vc != '0)) fail("fc_frame_num/fc_vc not 0 when idle");
      // err_vc.
      if ((o_err_frame_sync || o_err_frame_level || o_err_crc || o_err_wc || o_err_unsupported_dt)
          && o_err_vc[1]) fail("err_vc outside VC0/VC1 with a packet error");
      if (!(o_err_frame_sync || o_err_frame_level || o_err_crc || o_err_wc || o_err_unsupported_dt ||
            o_err_ecc_1bit) && o_err_vc != '0) fail("err_vc not 0 without an error pulse");
      if (o_err_frame_level && !o_fc_fe) fail("err_frame_level without fc_fe");
      if ((o_err_frame_level || (o_err_frame_sync && (o_fc_fs || o_fc_fe))) && o_err_vc != o_fc_vc)
        fail("err_vc differs from fc_vc on a frame error");
      // Generic Short Packet FIFO.
      if (!o_sp_fifo_not_empty && o_gsp_data != '0) fail("gsp_data not 0 with the FIFO empty");
      if (o_sp_fifo_full && !o_sp_fifo_not_empty) fail("sp_fifo_full with the FIFO empty");
      if (p_valid_prev && o_sp_fifo_overflow && !p_full_nopop)
        fail("sp_fifo_overflow without a full FIFO and no gsp_pop");
      if (p_valid_prev) begin
        // Sticky flags fall only after soft_reset.
        if (!p_soft && ((p_nbs && !o_null_blank_seen) || (p_vcu && !o_vc_unsupported_seen) ||
                        (p_dtc && !o_dt_conflict))) fail("sticky flag cleared without soft_reset");
        if (p_dtc_cond && !p_soft && !o_dt_conflict) fail("dt_conflict not set after its condition");
        // Payload suppression by force_discard of the previous cycle.
        if (o_bp_valid && p_fd[o_bp_vc_tag[0]]) fail("bp beat of a channel under force_discard");
        if (o_emb_valid && p_fd[o_emb_vc[0]]) fail("emb beat of a channel under force_discard");
        // Nothing of an abandoned packet after soft_reset or core_enable low.
        if (p_flush && (o_bp_valid || o_emb_valid || o_fc_fs || o_fc_fe || o_err_crc || o_err_wc ||
                        o_err_frame_sync || o_err_frame_level || o_err_ecc_1bit ||
                        o_err_ecc_2bit || o_err_unsupported_dt || o_err_lane_abort ||
                        o_packet_done_pulse)) fail("output event in the cycle after a flush");
        if (p_soft && (o_null_blank_seen || o_vc_unsupported_seen || o_dt_conflict ||
                       o_sp_fifo_not_empty)) fail("flag or FIFO not cleared by soft_reset");
      end
      // Line structure: after a partial beat without eol the next beat starts a line; the line
      // attributes stay fixed from sol to eol. Both checks restart after force_discard.
      if (i_force_discard != 2'b00) begin
        bp_short_seen  = 1'b0;
        emb_short_seen = 1'b0;
        bp_in_line     = 1'b0;
      end
      if (o_bp_valid) begin
        if (bp_short_seen && !o_bp_sol) fail("bp beat after a partial beat without a new bp_sol");
        if (o_bp_sol) begin
          bp_in_line   = 1'b1;
          bp_moved     = 1'b0;
          bp_line_attr = {o_bp_vc_tag, o_bp_dt_sel};
        end else if (bp_in_line && (bp_moved || bp_line_attr != {o_bp_vc_tag, o_bp_dt_sel}))
          fail("bp_vc_tag/bp_dt_sel changed inside a line");
        if (o_bp_eol) bp_in_line = 1'b0;
        bp_short_seen = !o_bp_eol && (o_bp_byte_en != 4'b1111);
      end else if (bp_in_line && bp_line_attr != {o_bp_vc_tag, o_bp_dt_sel}) begin
        bp_moved = 1'b1;
      end
      if (o_emb_valid) begin
        if (emb_short_seen && !o_emb_sol) fail("emb beat after a partial beat without a new emb_sol");
        emb_short_seen = !o_emb_eol && (o_emb_byte_en != 4'b1111);
      end
      p_valid_prev = 1'b1;
    end else begin
      p_valid_prev   = 1'b0;
      bp_short_seen  = 1'b0;
      emb_short_seen = 1'b0;
      bp_in_line     = 1'b0;
    end
    if (started && i_resetn !== 1'b1) begin
      if (o_rd_en !== 1'b0) fail("rd_en high while resetn is low");
    end
    p_fd         = i_force_discard;
    p_flush      = i_soft_reset || !i_core_enable;
    p_soft       = i_soft_reset;
    p_dtc_cond   = i_metadata_capture_en && (i_metadata_dt inside {DT_NULL, DT_BLANK, DT_RAW10, DT_RAW12});
    p_full_nopop = o_sp_fifo_full && !i_gsp_pop;
    p_nbs        = o_null_blank_seen;
    p_vcu        = o_vc_unsupported_seen;
    p_dtc        = o_dt_conflict;
    p_pulses     = {o_bp_sof, o_bp_sol, o_bp_eol, o_emb_sol, o_emb_eol, o_fc_fs, o_fc_fe,
                    o_packet_done_pulse, o_sp_fifo_overflow, o_err_ecc_1bit, o_err_ecc_2bit,
                    o_err_crc, o_err_wc, o_err_unsupported_dt, o_err_lane_abort,
                    o_err_frame_sync, o_err_frame_level};
  end

  // Stimulus checks: the entries read follow the Lane Management format and the minimum gap.
  always @(posedge i_sys_clk) begin
    if (!started || i_resetn !== 1'b1 || i_soft_reset || !i_core_enable) begin
      s_in_burst = 1'b0;
      s_last_end = -1000;
    end else if (o_rd_en === 1'b1 && i_rd_empty === 1'b0) begin
      logic [37:0] e;
      e = i_rd_data;
      if (is_marker(e)) begin
        if (e[31:0] != '0) fail("stimulus: abort marker with data");
        if (s_in_burst) fail("stimulus: abort marker inside a burst");
        if (cyc - s_last_end < L + 3) fail("stimulus: abort marker closer than the minimum gap");
        s_last_end = cyc;
      end else begin
        if (!(e[35:32] inside {4'b0001, 4'b0011, 4'b0111, 4'b1111}))
          fail("stimulus: byte_valid not contiguous");
        if (e[35:32] != 4'b1111 && !e[37]) fail("stimulus: partial entry before the end of burst");
        if (e[36]) begin
          if (s_in_burst) fail("stimulus: sob inside a burst");
          if (cyc - s_last_end < L + 3) fail("stimulus: burst closer than the minimum gap");
          s_in_burst = 1'b1;
        end else if (!s_in_burst) fail("stimulus: entry outside a burst");
        if (e[37]) begin
          s_in_burst = 1'b0;
          s_last_end = cyc;
        end
      end
    end
  end

  // Waits until every entry has been read and every expected output has been seen.
  task automatic wait_quiet();
    int guard;
    guard = 0;
    while ((src_q.size() != 0 || rd_hist.size() != 0 || exp_ev.size() != 0 ||
            win_q.size() != 0) && guard < 400000) begin
      cycles(1);
      guard++;
    end
    if (guard >= 400000) fail("traffic did not drain");
    cycles(L + 6);
  endtask

  // --------------------------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------------------------
  // Idle outputs after reset; a second reset clears the flags and the FIFO.
  task automatic test_reset_values();
    cycles(20);
    send_line(2'd0, DT_NULL, 4);
    send_short(2'd2, 6'h08, 16'h0001);
    send_short(2'd0, 6'h0A, 16'hABCD);
    wait_quiet();
    i_metadata_dt = DT_RAW10;
    cycles(2);
    i_metadata_dt = DT_EMB;
    i_resetn = 1'b0;
    cycles(3);
    i_resetn = 1'b1;
    cycles(5);
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 8); frame_end(2'd0);
  endtask

  // RAW10 and RAW12 frames, every Word Count mod 4, an embedded line first.
  task automatic test_basic_frames();
    frame_start(2'd0, 16'h0001);
    send_line(2'd0, DT_EMB, 9);
    for (int wc = 40; wc < 44; wc++) send_line(2'd0, DT_RAW10, wc);
    for (int wc = 36; wc < 40; wc++) send_line(2'd0, DT_RAW12, wc);
    send_line(2'd0, DT_RAW10, 3360);
    frame_end(2'd0, 16'h0001);
    frame_start(2'd1, 16'h0010);
    send_line(2'd1, DT_RAW12, 18);
    send_line(2'd1, DT_RAW10, 25);
    frame_end(2'd1, 16'h0010);
    frame_start(2'd0, 16'h0002);
    send_line(2'd0, DT_RAW12, 18);
    frame_end(2'd0, 16'h0002);
  endtask

  // Word Counts 0 to 12 on image and embedded packets.
  task automatic test_small_word_counts();
    frame_start(2'd0);
    for (int wc = 0; wc <= 12; wc++) send_line(2'd0, DT_RAW10, wc);
    for (int wc = 0; wc <= 12; wc++) send_line(2'd0, DT_RAW12, wc);
    for (int wc = 0; wc <= 8; wc++) send_line(2'd0, DT_EMB, wc);
    frame_end(2'd0);
  endtask

  // CRC: CSI-2 examples, corrupted footer and payload bytes, every packet class.
  task automatic test_crc_check();
    logic [7:0]  ex1[24] = '{8'hFF, 8'h00, 8'h00, 8'h02, 8'hB9, 8'hDC, 8'hF3, 8'h72, 8'hBB, 8'hD4,
                             8'hB8, 8'h5A, 8'hC8, 8'h75, 8'hC2, 8'h7C, 8'h81, 8'hF8, 8'h05, 8'hDF,
                             8'hFF, 8'h00, 8'h00, 8'h01};
    logic [7:0]  ex2[24] = '{8'hFF, 8'h00, 8'h00, 8'h00, 8'h1E, 8'hF0, 8'h1E, 8'hC7, 8'h4F, 8'h82,
                             8'h78, 8'hC5, 8'h82, 8'hE0, 8'h8C, 8'h70, 8'hD2, 8'h3C, 8'h78, 8'hE9,
                             8'hFF, 8'h00, 8'h00, 8'h01};
    logic [15:0] c;
    frame_start(2'd0);
    pl.delete(); foreach (ex1[i]) pl.push_back(ex1[i]);
    c = 16'hFFFF; foreach (pl[i]) c = crc_byte(c, pl[i]);
    if (c != 16'h00F0) fail($sformatf("CRC of CSI-2 example 1 = %h, expected F0 00", c));
    send_long(2'd0, DT_RAW10);
    pl.delete(); foreach (ex2[i]) pl.push_back(ex2[i]);
    c = 16'hFFFF; foreach (pl[i]) c = crc_byte(c, pl[i]);
    if (c != 16'hE569) fail($sformatf("CRC of CSI-2 example 2 = %h, expected 69 E5", c));
    send_long(2'd0, DT_RAW10);
    // Word Count 0 carries FF FF.
    pl.delete(); send_long(2'd0, DT_RAW10);
    pl.delete(); send_long(2'd0, DT_RAW10, 16'h0100);
    for (int wc = 20; wc < 24; wc++) begin
      random_payload(wc); send_long(2'd0, DT_RAW10, 16'h0001);
      random_payload(wc); send_long(2'd0, DT_RAW12, 16'h8000);
      random_payload(wc); send_long(2'd0, DT_EMB, 16'h0100);
      random_payload(wc); send_long(2'd0, DT_RAW10, 16'h0000, -1, '0, 0, 2'b00, wc - 1);
      random_payload(wc); send_long(2'd0, DT_RAW12, 16'h0000, -1, '0, 0, 2'b00, 0);
    end
    random_payload(8); send_long(2'd0, DT_NULL, 16'h0004);
    random_payload(8); send_long(2'd0, DT_BLANK, 16'h0004);
    random_payload(8); send_long(2'd0, 6'h30, 16'h0004);
    wait_quiet();
    i_force_discard = 2'b01;
    random_payload(12); send_long(2'd0, DT_RAW10, 16'h0040);
    wait_quiet();
    i_force_discard = 2'b00;
    frame_end(2'd0);
    random_payload(12); send_long(2'd0, DT_RAW10, 16'h0200);
    random_payload(12); send_long(2'd1, DT_EMB, 16'h0002);
  endtask

  // Single-bit header errors at the 24 data and 6 parity positions, CSI-2 Figure 46 header.
  task automatic test_ecc_single_bit();
    if (ecc_parity(24'h01F037) != 6'h3F) fail("ECC of the CSI-2 Figure 46 header is not 0x3F");
    frame_start(2'd0);
    random_payload(16'h01F0); send_long(2'd0, 6'h37);
    for (int b = 0; b < 30; b++) begin
      random_payload(8);
      send_long(2'd0, DT_RAW10, 16'h0000, -1, 30'(1) << b);
      send_short(2'(b % 2), 6'h08 + 6'(b % 8), 16'(b * 16'h0101), 30'(1) << b);
      frame_end(2'd0, 16'(b), 30'(1) << b);
      frame_start(2'd0, 16'(b + 1), 30'(1) << b);
      send_short(2'(2 + b % 2), 6'h09, 16'h5A5A, 30'(1) << b);
      pop_pulse();
    end
    wait_quiet();
    for (int i = 0; i < DEPTH; i++) begin pop_pulse(); cycles(2); end
    frame_end(2'd0);
  endtask

  // Two-bit errors at every pair of code bits, random multi-bit errors, unused ECC bits.
  task automatic test_ecc_multi_bit();
    frame_start(2'd0);
    frame_start(2'd1);
    for (int a = 0; a < 30; a++) begin
      for (int b = a + 1; b < 30; b++) begin
        random_payload(4);
        send_long(2'd0, DT_RAW10, 16'h0000, -1, (30'(1) << a) | (30'(1) << b));
      end
    end
    // Two-bit errors on every packet class.
    for (int i = 0; i < 40; i++) begin
      int a, b;
      logic [29:0] f;
      a = rand_range(0, 29);
      b = (a + rand_range(1, 29)) % 30;
      f = (30'(1) << a) | (30'(1) << b);
      case (i % 8)
        0: send_short(2'd0, DT_FS, 16'(i), f);
        1: send_short(2'd1, DT_FE, 16'(i), f);
        2: send_short(2'd0, 6'h0B, 16'(i), f);
        3: begin random_payload(10); send_long(2'd1, DT_RAW12, 16'h0000, -1, f); end
        4: begin random_payload(10); send_long(2'd0, DT_EMB, 16'h0000, -1, f); end
        5: begin random_payload(10); send_long(2'd1, DT_NULL, 16'h0000, -1, f); end
        6: begin random_payload(10); send_long(2'd3, DT_RAW10, 16'h0000, -1, f); end
        default: send_short(2'd2, 6'h0C, 16'(i), f);
      endcase
    end
    // Three or more flipped bits; the result follows the decode rule.
    for (int i = 0; i < 20; i++) begin
      logic [29:0] f;
      f = 30'($urandom) | (30'(1) << rand_range(0, 29));
      random_payload(rand_range(1, 9));
      send_long(2'd0, DT_RAW10, 16'h0000, -1, f);
    end
    wait_quiet();
    // The two unused ECC bits are ignored.
    frame_start(2'd0, 16'h0100);
    for (int i = 0; i < 12; i++) begin
      random_payload(6);
      send_long(2'd0, DT_RAW10, 16'h0000, -1, '0, 0, 2'(i));
      send_short(2'd0, DT_FE, 16'h0100, '0, 2'(i + 1));
      send_short(2'd0, DT_FS, 16'h0100, '0, 2'(i + 2));
    end
    frame_end(2'd0);
    frame_end(2'd1);
    wait_quiet();
    for (int i = 0; i < DEPTH; i++) begin pop_pulse(); cycles(2); end
  endtask

  // Bursts cut at every byte position, header-only bursts, words after the footer.
  task automatic test_truncated_bursts();
    frame_start(2'd0);
    for (int wc = 5; wc <= 8; wc++) begin
      for (int cut = 4; cut <= wc + 5; cut++) begin
        random_payload(wc);
        send_long(2'd0, DT_RAW10, 16'h0000, cut);
      end
    end
    pl.delete(); send_long(2'd0, DT_RAW10, 16'h0000, 4);
    random_payload(1); send_long(2'd0, DT_RAW10, 16'h0000, 4);
    random_payload(100); send_long(2'd0, DT_RAW12, 16'h0000, 4);
    random_payload(9); send_long(2'd0, DT_EMB, 16'h0000, 7);
    random_payload(9); send_long(2'd0, DT_NULL, 16'h0000, 11);
    random_payload(9); send_long(2'd0, 6'h31, 16'h0000, 12);
    random_payload(2); send_long(2'd0, DT_RAW10, 16'h0000, 5);
    for (int x = 1; x <= 8; x++) begin
      random_payload(rand_range(1, 13));
      send_long(2'd0, DT_RAW10, 16'h0000, -1, '0, x);
    end
    random_payload(10); send_long(2'd0, DT_RAW10, 16'h0001, -1, '0, 5);
    random_payload(10); send_long(2'd1, DT_RAW10, 16'h0000, 9);
    frame_end(2'd0);
  endtask

  // Short packets: frame numbers, Line Start/End, reserved codes, generic codes, gsp_pop.
  task automatic test_short_packets();
    pop_pulse();
    cycles(3);
    for (int v = 0; v < 2; v++) begin
      frame_start(2'(v), 16'(16'h1234 + v));
      send_short(2'(v), 6'h02, 16'h0007);
      send_short(2'(v), 6'h03, 16'h0007);
      for (int d = 4; d < 8; d++) send_short(2'(v), 6'(d), 16'hAAAA);
      frame_end(2'(v), 16'(16'h1234 + v));
    end
    for (int d = 8; d < 16; d++) send_short(2'(d % 2), 6'(d), 16'($urandom));
    send_short(2'd0, 6'h0F, 16'h0F0F, '0, 2'b00, 4);
    send_short(2'd1, DT_FS, 16'h0042, '0, 2'b00, 7);
    send_short(2'd1, DT_FE, 16'h0042, '0, 2'b00, 2);
    wait_quiet();
    for (int i = 0; i < 12; i++) begin pop_pulse(); cycles(1); end
    wait_quiet();
    for (int d = 8; d < 11; d++) send_short(2'd0, 6'(d), 16'($urandom));
    wait_quiet();
    for (int i = 0; i < 4; i++) begin pop_pulse(); cycles(3); end
  endtask

  // Generic Short Packet FIFO: fill, overflow, write and pop on a full FIFO, pop on empty.
  task automatic test_gsp_fifo();
    for (int i = 0; i < DEPTH + 3; i++) send_short(2'(i % 2), 6'h0C, 16'(i));
    wait_quiet();
    g_pop_with_write = 1'b1;
    send_short(2'd0, 6'h0D, 16'hBEEF);
    send_short(2'd1, 6'h0E, 16'hBEE0);
    wait_quiet();
    g_pop_with_write = 1'b0;
    i_force_discard = 2'b11;
    pop_pulse();
    cycles(2);
    send_short(2'd0, 6'h0E, 16'hCAFE);
    wait_quiet();
    i_force_discard = 2'b00;
    for (int i = 0; i < DEPTH + 2; i++) begin pop_pulse(); cycles(2); end
    for (int i = 0; i < 3; i++) send_short(2'd1, 6'h0F, 16'(i));
    wait_quiet();
    for (int i = 0; i < 5; i++) begin pop_pulse(); cycles(1); end
  endtask

  // Data Type routing: every long Data Type with several metadata_dt settings.
  task automatic test_dt_routing();
    logic [5:0] metas[4] = '{DT_EMB, 6'h30, DT_RAW10, DT_NULL};
    frame_start(2'd0);
    frame_start(2'd1);
    for (int m = 0; m < 4; m++) begin
      for (int en = 0; en < 2; en++) begin
        wait_quiet();
        i_metadata_dt         = metas[m];
        i_metadata_capture_en = 1'(en);
        for (int dt = 16; dt < 64; dt++) send_line(2'(dt % 2), 6'(dt), 5);
      end
    end
    wait_quiet();
    i_metadata_dt         = DT_EMB;
    i_metadata_capture_en = 1'b1;
    frame_end(2'd0);
    frame_end(2'd1);
  endtask

  // dt_conflict: set by each conflicting setting, held, cleared only by soft_reset.
  task automatic test_dt_conflict();
    logic [5:0] cf[4] = '{DT_NULL, DT_BLANK, DT_RAW10, DT_RAW12};
    frame_start(2'd0);
    for (int i = 0; i < 4; i++) begin
      wait_quiet();
      i_metadata_dt         = cf[i];
      i_metadata_capture_en = 1'b0;
      cycles(3);
      send_line(2'd0, cf[i], 6);
      wait_quiet();
      i_metadata_capture_en = 1'b1;
      send_line(2'd0, cf[i], 7);
      wait_quiet();
      i_metadata_dt = DT_EMB;
      cycles(5);
      i_core_enable = 1'b0;
      cycles(3);
      i_core_enable = 1'b1;
      cycles(3);
      soft_reset_pulse(1);
      cycles(5);
      frame_start(2'd0);
    end
    wait_quiet();
    i_metadata_dt = DT_RAW10;
    soft_reset_pulse(2);
    cycles(4);
    i_metadata_capture_en = 1'b0;
    cycles(3);
    i_metadata_dt         = DT_EMB;
    i_metadata_capture_en = 1'b1;
  endtask

  // Frame errors: Frame Start on an open frame, Frame End on a closed one, packets outside frames.
  task automatic test_frame_sync_errors();
    frame_start(2'd0, 16'd1);
    frame_start(2'd0, 16'd2);
    frame_end(2'd1, 16'd9);
    send_line(2'd1, DT_RAW10, 10);
    send_line(2'd1, DT_EMB, 10);
    send_line(2'd1, DT_NULL, 10);
    send_line(2'd1, DT_BLANK, 10);
    send_line(2'd1, 6'h31, 10);
    random_payload(10); send_long(2'd1, DT_RAW12, 16'h0000, 4);
    wait_quiet();
    i_metadata_capture_en = 1'b0;
    send_line(2'd1, DT_EMB, 10);
    wait_quiet();
    i_metadata_capture_en = 1'b1;
    frame_start(2'd1, 16'd3);
    send_line(2'd1, DT_RAW12, 12);
    frame_start(2'd1, 16'd4);
    send_line(2'd1, DT_RAW12, 12);
    frame_end(2'd1, 16'd4);
    frame_end(2'd1, 16'd4);
    send_line(2'd1, DT_RAW10, 6);
    frame_end(2'd0, 16'd2);
    send_line(2'd0, DT_EMB, 6);
  endtask

  // err_frame_level after CRC, Word Count and abort errors; not after ECC or Data Type errors.
  task automatic test_frame_level_errors();
    for (int v = 0; v < 2; v++) begin
      frame_start(2'(v)); random_payload(10); send_long(2'(v), DT_RAW10, 16'h0001); frame_end(2'(v));
      frame_start(2'(v)); random_payload(10); send_long(2'(v), DT_RAW10, 16'h0000, 9); frame_end(2'(v));
      frame_start(2'(v)); send_marker(); frame_end(2'(v));
      frame_start(2'(v)); random_payload(10); send_long(2'(v), DT_RAW10, 16'h0000, -1, 30'h4);
      frame_end(2'(v));
      frame_start(2'(v)); random_payload(10); send_long(2'(v), DT_RAW10, 16'h0000, -1, 30'h5);
      frame_end(2'(v));
      frame_start(2'(v)); send_line(2'(v), 6'h33, 10); frame_end(2'(v));
      frame_start(2'(v)); random_payload(9); send_long(2'(v), DT_NULL, 16'h0010); frame_end(2'(v));
      frame_start(2'(v)); random_payload(9); send_long(2'(v), DT_EMB, 16'h0000, 4); frame_end(2'(v));
      frame_start(2'(v)); random_payload(9); send_long(2'(1 - v), DT_RAW10, 16'h0001);
      frame_end(2'(v));
      frame_start(2'(v)); send_line(2'(v), DT_RAW10, 10); frame_end(2'(v));
    end
    // An error, then a new Frame Start without Frame End: the next frame starts clean.
    frame_start(2'd0); random_payload(10); send_long(2'd0, DT_RAW10, 16'h0001);
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 10); frame_end(2'd0);
  endtask

  // Abort markers with no, one and two frames open.
  task automatic test_abort_markers();
    send_marker();
    send_marker();
    frame_start(2'd0); send_marker(); frame_end(2'd0);
    frame_start(2'd1); send_marker(); frame_end(2'd1);
    frame_start(2'd0); frame_start(2'd1); send_marker(); send_line(2'd0, DT_RAW10, 8);
    frame_end(2'd0); frame_end(2'd1);
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 8); send_marker(); send_marker();
    send_line(2'd0, DT_RAW12, 9); frame_end(2'd0);
    frame_start(2'd0); frame_end(2'd0);
  endtask

  // bp_sof placement: embedded and Null lines first, first image line suppressed, one-beat line.
  task automatic test_sof_placement();
    frame_start(2'd0);
    send_line(2'd0, DT_EMB, 6); send_line(2'd0, DT_EMB, 13); send_line(2'd0, DT_NULL, 4);
    send_line(2'd0, DT_RAW10, 20); send_line(2'd0, DT_RAW10, 20); send_line(2'd0, DT_EMB, 5);
    frame_end(2'd0);
    frame_start(2'd0);
    wait_quiet();
    i_force_discard = 2'b01;
    send_line(2'd0, DT_RAW10, 20);
    wait_quiet();
    i_force_discard = 2'b00;
    send_line(2'd0, DT_RAW10, 20); send_line(2'd0, DT_EMB, 3);
    frame_end(2'd0);
    frame_start(2'd1); send_line(2'd1, DT_RAW12, 3); send_line(2'd1, DT_RAW12, 3); frame_end(2'd1);
    frame_start(2'd1); send_line(2'd1, DT_RAW12, 0); send_line(2'd1, DT_RAW12, 6); frame_end(2'd1);
  endtask

  // Interleaved VC0 and VC1 frames; an error on one channel does not reach the other.
  task automatic test_interleaved_vc();
    for (int f = 0; f < 3; f++) begin
      frame_start(2'd0, 16'(f + 1));
      frame_start(2'd1, 16'(f + 1));
      for (int l = 0; l < 4; l++) begin
        send_line(2'd0, DT_RAW10, 40 + l);
        if (f == 1 && l == 2) begin random_payload(41); send_long(2'd1, DT_RAW10, 16'h0300); end
        else send_line(2'd1, DT_RAW12, 39 + 3 * l);
        if (l == 1) send_line(2'(f % 2), DT_EMB, 7);
      end
      frame_end(2'd0, 16'(f + 1));
      frame_end(2'd1, 16'(f + 1));
    end
  endtask

  // force_discard per channel, between lines and at random cycles; other outputs unchanged.
  task automatic test_force_discard();
    frame_start(2'd0);
    frame_start(2'd1);
    wait_quiet();
    i_force_discard = 2'b01;
    send_line(2'd0, DT_RAW10, 20); send_line(2'd1, DT_RAW10, 20); send_line(2'd0, DT_EMB, 9);
    wait_quiet();
    i_force_discard = 2'b10;
    send_line(2'd0, DT_RAW12, 18); send_line(2'd1, DT_RAW12, 18); send_line(2'd1, DT_EMB, 9);
    random_payload(17); send_long(2'd1, DT_RAW10, 16'h0001);
    send_short(2'd1, 6'h09, 16'h0909);
    wait_quiet();
    i_force_discard = 2'b00;
    for (int i = 0; i < 30; i++) begin
      send_line(2'(i % 2), (i % 3 == 0) ? DT_EMB : DT_RAW10, rand_range(9, 60));
      if (i % 5 == 0) send_short(2'd0, 6'h09, 16'(i));
      if (i % 4 == 0) begin random_payload(17); send_long(2'(i % 2), DT_RAW12, 16'h0001); end
    end
    for (int t = 0; t < 2000 && src_q.size() != 0; t++) begin
      if (rand_range(0, 9) == 0) i_force_discard = 2'($urandom);
      cycles(1);
    end
    wait_quiet();
    i_force_discard = 2'b00;
    frame_end(2'd0);
    frame_end(2'd1);
    wait_quiet();
    for (int i = 0; i < DEPTH; i++) begin pop_pulse(); cycles(2); end
  endtask

  // Packets of every class on VC2 and VC3.
  task automatic test_unsupported_vc();
    frame_start(2'd0);
    for (int v = 2; v < 4; v++) begin
      frame_start(2'(v)); frame_end(2'(v));
      send_short(2'(v), 6'h09, 16'h1111); send_short(2'(v), 6'h02, 16'h0001);
      send_line(2'(v), DT_RAW10, 12); send_line(2'(v), DT_EMB, 12);
      send_line(2'(v), DT_NULL, 12); send_line(2'(v), 6'h30, 12);
      random_payload(12); send_long(2'(v), DT_RAW10, 16'h0001);
      random_payload(12); send_long(2'(v), DT_RAW10, 16'h0000, 9);
      random_payload(12); send_long(2'(v), DT_RAW10, 16'h0000, 4);
      random_payload(12); send_long(2'(v), DT_RAW10, 16'h0000, -1, 30'h8);
      random_payload(12); send_long(2'(v), DT_RAW10, 16'h0000, -1, 30'h9);
      send_short(2'(v), DT_FS, 16'h0002, 30'h0100_0000);
    end
    send_line(2'd0, DT_RAW10, 12);
    wait_quiet();
    i_core_enable = 1'b0;
    cycles(3);
    i_core_enable = 1'b1;
    cycles(3);
    frame_start(2'd0);
    wait_quiet();
    soft_reset_pulse(1);
    cycles(4);
    frame_start(2'd0);
    send_line(2'd3, DT_RAW10, 4);
    frame_end(2'd0);
  endtask

  // core_enable low between packets and frames: frame contexts closed, FIFO and flags kept.
  task automatic test_core_enable_idle();
    frame_start(2'd0); frame_start(2'd1);
    send_line(2'd0, DT_RAW10, 20); send_short(2'd0, 6'h0A, 16'h5555);
    send_short(2'd1, 6'h0B, 16'h6666); send_line(2'd0, DT_NULL, 4); send_line(2'd2, DT_RAW10, 4);
    wait_quiet();
    i_metadata_dt = DT_RAW12;
    cycles(2);
    i_metadata_dt = DT_EMB;
    i_core_enable = 1'b0;
    pop_pulse();
    cycles(6);
    i_core_enable = 1'b1;
    send_line(2'd0, DT_RAW10, 20);
    send_line(2'd1, DT_EMB, 8);
    frame_end(2'd1);
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 20); frame_end(2'd0);
    wait_quiet();
    i_core_enable = 1'b0;
    cycles(2);
    i_core_enable = 1'b1;
    frame_start(2'd1); send_line(2'd1, DT_RAW12, 9); frame_end(2'd1);
    wait_quiet();
    for (int i = 0; i < 3; i++) begin pop_pulse(); cycles(2); end
  endtask

  // core_enable falling during the header, the payload and an uncorrectable-header burst.
  task automatic test_core_enable_mid_packet();
    for (int d = 0; d < 16; d++) begin
      frame_start(2'd0);
      wait_quiet();
      random_payload(40);
      if (d % 3 == 2) send_long(2'd0, DT_RAW10, 16'h0000, -1, 30'h3);
      else if (d % 3 == 1) send_long(2'd0, DT_RAW10, 16'h0001);
      else send_long(2'd0, DT_RAW10);
      cycles(d);
      i_core_enable = 1'b0;
      cycles(1 + d % 3);
      i_core_enable = 1'b1;
      wait_quiet();
    end
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 16); frame_end(2'd0);
  endtask

  // soft_reset in every phase of a packet; a burst that never ends is recovered by soft_reset.
  task automatic test_soft_reset();
    for (int d = 0; d < 14; d++) begin
      frame_start(2'd0);
      send_short(2'd0, 6'h0B, 16'(d));
      send_line(2'd2, DT_NULL, 3);
      wait_quiet();
      random_payload(32);
      send_long(2'd0, (d % 2) ? DT_RAW10 : DT_EMB, (d % 4 == 3) ? 16'h0001 : 16'h0000);
      cycles(d);
      soft_reset_pulse(1 + d % 2);
      wait_quiet();
    end
    // soft_reset at each cycle after an abort marker, with a frame open.
    for (int d = 0; d < L + 4; d++) begin
      frame_start(2'd0);
      wait_quiet();
      send_marker();
      cycles(d);
      soft_reset_pulse(1);
      wait_quiet();
    end
    // Header of a 64-byte packet followed by three words and no end of burst.
    frame_start(2'd0);
    wait_quiet();
    header_bytes(2'd0, DT_RAW10, 16'd64, '0, 2'b00);
    src_q.push_back('{idle: 1'b0, ent: {1'b0, 1'b1, 4'hF, bq[3], bq[2], bq[1], bq[0]}});
    for (int w = 0; w < 3; w++) src_q.push_back('{idle: 1'b0, ent: {1'b0, 1'b0, 4'hF, 32'($urandom)}});
    cycles(30);
    soft_reset_pulse(1);
    cycles(3);
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 12); frame_end(2'd0);
  endtask

  // resetn asserted between clock edges in the middle of a packet.
  task automatic test_async_reset();
    for (int d = 0; d < 10; d++) begin
      frame_start(2'd0);
      send_short(2'd0, 6'h0F, 16'(d));
      send_line(2'd0, DT_NULL, 2);
      send_line(2'd0, DT_RAW10, 28);
      cycles(4 + 2 * d);
      #1.3;
      i_resetn = 1'b0;
      cycles(2 + d % 2);
      i_resetn = 1'b1;
      cycles(3);
    end
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 8); frame_end(2'd0);
  endtask

  // rd_en against rd_empty, core_enable, soft_reset and resetn.
  task automatic test_rd_en();
    g_hole = 3;
    for (int rep = 0; rep < 8; rep++) begin
      frame_start(2'd0);
      for (int i = 0; i < 6; i++) send_line(2'd0, DT_RAW10, rand_range(1, 40));
      for (int t = 0; t < 400 && src_q.size() != 0; t++) begin
        int r;
        r = rand_range(0, 99);
        if (r < 3) begin
          i_core_enable = 1'b0; cycles(rand_range(1, 3)); i_core_enable = 1'b1;
        end else if (r < 5) begin
          soft_reset_pulse(rand_range(1, 2));
        end else if (r == 5 && rep % 2 == 1) begin
          i_resetn = 1'b0; cycles(rand_range(1, 2)); i_resetn = 1'b1;
        end else if (r == 6) begin
          i_core_enable = 1'b0; i_soft_reset = 1'b1; cycles(1);
          i_core_enable = 1'b1; cycles(1); i_soft_reset = 1'b0;
        end else cycles(1);
      end
      wait_quiet();
    end
    g_hole = 0;
    frame_start(2'd0); send_line(2'd0, DT_RAW10, 9); frame_end(2'd0);
  endtask

  // Bursts and markers at the minimum gap.
  task automatic test_min_gap_bursts();
    frame_start(2'd0);
    for (int i = 0; i < 6; i++) begin
      send_line(2'd0, DT_RAW10, 1);
      send_line(2'd0, DT_RAW10, 2);
      send_short(2'd0, 6'h09, 16'(i));
      send_marker();
      send_short(2'd0, 6'h08, 16'h0000, 30'h11);
      random_payload(9); send_long(2'd0, DT_RAW10, 16'h0000, 4);
      random_payload(9); send_long(2'd0, DT_RAW10, 16'h0000, -1, 30'h0030_0000);
      send_line(2'd0, DT_RAW12, 6);
      random_payload(5); send_long(2'd0, DT_RAW10, 16'h0000, 6);
      send_line(2'd0, DT_EMB, 4);
      pop_pulse();
    end
    frame_end(2'd0);
    wait_quiet();
    for (int i = 0; i < DEPTH; i++) begin pop_pulse(); cycles(1); end
  endtask

  // Empty cycles inside bursts and a long wait between header and payload.
  task automatic test_rd_empty_gaps();
    g_hole = 6;
    frame_start(2'd0);
    frame_start(2'd1);
    for (int i = 0; i < 16; i++) send_line(2'(i % 2), (i % 2) ? DT_RAW12 : DT_RAW10, rand_range(1, 50));
    random_payload(30); send_long(2'd0, DT_RAW10, 16'h0001);
    random_payload(30); send_long(2'd1, DT_RAW10, 16'h0000, 21);
    g_hole     = 0;
    g_hdr_hole = 40;
    send_line(2'd0, DT_RAW10, 12);
    send_line(2'd1, DT_EMB, 5);
    send_line(2'd0, DT_RAW12, 1);
    g_hdr_hole = 0;
    frame_end(2'd0);
    frame_end(2'd1);
  endtask

  // Output timing of every class of event.
  task automatic test_latency();
    frame_start(2'd0);
    send_line(2'd0, DT_RAW10, 6);
    random_payload(6); send_long(2'd0, DT_RAW10, 16'h0001);
    random_payload(7); send_long(2'd0, DT_RAW10, 16'h0002);
    random_payload(10); send_long(2'd0, DT_RAW10, 16'h0000, 7);
    random_payload(10); send_long(2'd0, DT_RAW10, 16'h0000, 4);
    send_short(2'd0, 6'h09, 16'h1234);
    send_marker();
    send_line(2'd0, 6'h30, 4);
    send_line(2'd0, DT_EMB, 5);
    send_line(2'd0, DT_NULL, 5);
    send_line(2'd2, DT_NULL, 5);
    random_payload(4); send_long(2'd0, DT_RAW10, 16'h0000, -1, 30'h1);
    random_payload(4); send_long(2'd0, DT_RAW10, 16'h0000, -1, 30'h3);
    frame_end(2'd0);
    frame_end(2'd0);
    wait_quiet();
    i_metadata_dt = DT_RAW12;
    cycles(1);
    i_metadata_dt = DT_EMB;
    pop_pulse();
    cycles(4);
  endtask

  // Random mixed traffic: frames, channels, Data Types, errors, discards, pops.
  task automatic test_random_traffic();
    g_space = 1;
    frame_start(2'd0);
    frame_start(2'd1);
    for (int p = 0; p < 150; p++) begin
      logic [1:0] v;
      int r;
      v = 2'(rand_range(0, 1));
      r = rand_range(0, 99);
      if (r < 40)      send_line(v, (rand_range(0, 1) != 0) ? DT_RAW10 : DT_RAW12, rand_range(0, 80));
      else if (r < 47) send_line(v, DT_EMB, rand_range(1, 30));
      else if (r < 51) begin random_payload(rand_range(1, 40)); send_long(v, DT_RAW10, 16'($urandom) | 16'h1); end
      else if (r < 55) begin random_payload(rand_range(2, 40)); send_long(v, DT_RAW12, 16'h0000, rand_range(4, 20)); end
      else if (r < 59) begin random_payload(rand_range(1, 20)); send_long(v, DT_RAW10, 16'h0000, -1, 30'(1) << rand_range(0, 29)); end
      else if (r < 61) begin random_payload(rand_range(1, 20)); send_long(v, DT_RAW10, 16'h0000, -1, 30'h0000_0081); end
      else if (r < 65) send_short(v, 6'(rand_range(8, 15)), 16'($urandom));
      else if (r < 68) send_marker();
      else if (r < 72) begin frame_end(v, 16'(p)); frame_start(v, 16'(p + 1)); end
      else if (r < 75) send_line(2'(rand_range(2, 3)), DT_RAW10, rand_range(1, 20));
      else if (r < 78) send_line(v, 6'(rand_range(16, 63)), rand_range(1, 20));
      else if (r < 80) begin random_payload(rand_range(1, 20)); send_long(v, DT_RAW10, 16'h0000, -1, '0, rand_range(1, 6)); end
      else if (r < 83) pop_pulse();
      else if (r < 85) frame_start(v, 16'(p));
      else             send_line(v, DT_RAW10, rand_range(30, 200));
      // Keep the source short so that force_discard changes land inside the traffic.
      while (src_q.size() > 40) cycles(1);
      if (rand_range(0, 25) == 0) i_force_discard = 2'($urandom);
      else if (i_force_discard != 2'b00 && rand_range(0, 3) == 0) i_force_discard = 2'b00;
      if (rand_range(0, 7) == 0) cycles(rand_range(1, 6));
    end
    wait_quiet();
    i_force_discard = 2'b00;
    frame_end(2'd0);
    frame_end(2'd1);
    g_space = 0;
    wait_quiet();
    for (int i = 0; i < DEPTH + 1; i++) begin pop_pulse(); cycles(1); end
  endtask

  // An uncorrectable header in the middle of a frame does not affect the following lines.
  task automatic test_ecc_error_mid_frame();
    frame_start(2'd0);
    send_line(2'd0, DT_RAW10, 24); send_line(2'd0, DT_RAW10, 24);
    random_payload(24); send_long(2'd0, DT_RAW10, 16'h0000, -1, 30'h0000_0006);
    send_line(2'd0, DT_RAW10, 24);
    send_short(2'd0, DT_FE, 16'h0001, 30'h0000_0300);
    send_line(2'd0, DT_RAW10, 24);
    frame_end(2'd0);
  endtask

  // Forces wrong payload data on bp_data; this test must end in FAIL with errors in the log. It
  // confirms that the checker catches a fault and is not part of the regression.
  task automatic test_checker_fault();
    frame_start(2'd0);
    send_line(2'd0, DT_RAW10, 40);
    while (o_bp_valid !== 1'b1) cycles(1);
    force o_bp_data = 32'h0000_0000;
    cycles(3);
    release o_bp_data;
    frame_end(2'd0);
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
    i_metadata_dt         = DT_EMB;
    i_metadata_capture_en = 1'b1;
    i_force_discard       = 2'b00;
    i_gsp_pop             = 1'b0;
    i_rd_empty            = 1'b1;
    i_rd_data             = '0;
    model_reset();
    burst_num = 0;
    $display("%s: PARA_ECC_LATENCY=%0d PARA_SP_FIFO_DEPTH=%0d SEED=%0d", test_name, L, DEPTH, seed);
    #1.0;
    started = 1'b1;
    cycles(4);
    i_resetn = 1'b1;
    cycles(3);
    case (test_name)
      "test_reset_values":           test_reset_values();
      "test_basic_frames":           test_basic_frames();
      "test_small_word_counts":      test_small_word_counts();
      "test_crc_check":              test_crc_check();
      "test_ecc_single_bit":         test_ecc_single_bit();
      "test_ecc_multi_bit":          test_ecc_multi_bit();
      "test_truncated_bursts":       test_truncated_bursts();
      "test_short_packets":          test_short_packets();
      "test_gsp_fifo":               test_gsp_fifo();
      "test_dt_routing":             test_dt_routing();
      "test_dt_conflict":            test_dt_conflict();
      "test_frame_sync_errors":      test_frame_sync_errors();
      "test_frame_level_errors":     test_frame_level_errors();
      "test_abort_markers":          test_abort_markers();
      "test_sof_placement":          test_sof_placement();
      "test_interleaved_vc":         test_interleaved_vc();
      "test_force_discard":          test_force_discard();
      "test_unsupported_vc":         test_unsupported_vc();
      "test_core_enable_idle":       test_core_enable_idle();
      "test_core_enable_mid_packet": test_core_enable_mid_packet();
      "test_soft_reset":             test_soft_reset();
      "test_async_reset":            test_async_reset();
      "test_rd_en":                  test_rd_en();
      "test_min_gap_bursts":         test_min_gap_bursts();
      "test_rd_empty_gaps":          test_rd_empty_gaps();
      "test_latency":                test_latency();
      "test_random_traffic":         test_random_traffic();
      "test_ecc_error_mid_frame":    test_ecc_error_mid_frame();
      "test_checker_fault":          test_checker_fault();
      default: begin
        err_count++;
        $error("unknown test '%s'", test_name);
      end
    endcase
    wait_quiet();
    if (win_q.size() != 0) fail("packet_done_pulse still expected at the end of the test");
    cycles(5);
    if (err_count == 0 && check_count >= 10) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d, checks=%0d)", test_name, err_count, check_count);
    $finish;
  end

  // Timeout.
  initial begin
    #50_000_000.0;
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
