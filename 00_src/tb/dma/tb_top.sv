// File        : tb_top.sv
// Description : Testbench of m_fpt_dma (DMA: two ping-pong buffers written over an AXI4 write
//               manager interface) and of its asynchronous FIFO m_fpt_dma_async_fifo.
//               The testbench plays Format Bridge, Frame Context & Metadata Capture, Control FSM,
//               the CSR block, software and an AXI4 subordinate with a byte memory. A reference
//               model, written from the specification, follows the bindings of every channel,
//               the state of both buffers and the bytes each capture must place in memory. A
//               monitor checks at every sys_clk edge the claims, drops, fbd_ready, the buffer
//               ownership and the events; at every mm_aclk edge the AXI rules, every written
//               byte, the burst limits and the outstanding count. A buffer released as valid
//               must hold exactly the expected bytes. Tests only drive stimulus.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//               The seed also selects the clock ratio: seeds 1, 4, 7 run sys_clk 5 ns with
//               mm_aclk 13 ns, seeds 2, 5, 8 run 5 ns with 5.3 ns, seeds 3, 6, 9 run 10 ns with
//               6 ns; the phase of mm_aclk is drawn from the seed. With PARA_ASYNC_MM_CLK = 0 one
//               clock drives both domains.
//
// Design assumptions (decided by the block owner; the specification does not state them yet):
//   - resetn resets the sys_clk side and mm_resetn the mm_aclk side; both are asserted together
//     and each is released in its own clock domain.
//   - Each capture slot remembers whether its capture holds a buffer. That flag is cleared when
//     the buffer leaves the writing state and by soft_reset, so the end, the drop or a late
//     channel of an old capture never acts on a buffer claimed again by a later capture.
//   - A pending binding replaced before the pixel path took it counts as the channel having
//     moved on: a frame none of whose beats reaches the DMA is caught by the line count check
//     once the record of the frame is known.
//   - No claim and no dma_drop are made in a cycle with soft_reset; a capture_start or ev_fs
//     seen in a soft_reset cycle is forgotten.
//   - A record is written in as many bursts as the 4 KB boundary and the burst limit require.
//   - Write data never waits for AWREADY.
//   - soft_reset in the cycle in which an aborted buffer leaves the abort state counts as seen:
//     the buffer returns to free whatever err_frame_policy.
//   - The first claim after resetn or soft_reset takes buffer 0 when it is free.
//     DMA_STATUS.active_buffer resets to 0 and keeps its value over soft_reset.
//   - Metadata words advance by four bytes. A word whose address has bit 2 clear uses data bits
//     31:0, a word with bit 2 set uses bits 63:32; AWADDR of a burst is 8-byte aligned. Each
//     channel has its own payload pointer.
//   - The length of a record is the number of words up to meta_last, at most ten.
//   - META_MAX_BURST_BEATS counts AXI beats; with a 32-bit bus a writer closes a burst at half
//     that number of 64-bit beats, at least one.
//   - soft_reset clears the bindings, the line counters, AXI_FAULT_TRACE, AXI_FAULT_ADDR and
//     AXI_ERROR_COUNT. It keeps the buffers owned by software, BUFFER_ERROR_INFO,
//     active_buffer and the writes in flight.
//   - Metadata responses never advance bufN_last_valid_line, but one that is not OKAY stops it.
//     bufN_last_valid_line resets to 0.
//   - A metadata payload burst whose binding is lost is released at once; the next word starts
//     a new burst.
//   - The cycle at which a buffer reaches software, the completion latency across the clock
//     domains and the cycle of a FIFO loss are not fixed by the specification: checks on them
//     use windows. Claims, drops, fbd_ready on the sensor path, the AXI rules and the memory
//     contents are checked exactly.
//   - Stimulus: every line ends with fbd_eol before the next line starts, byte enables are
//     contiguous from bit 0 and only the last beat of a line is partial; records are never
//     interleaved with records; the stride never makes two lines overlap; every region lies
//     below 4 GB; the subordinate never answers EXOKAY.
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_AXI_DATA_WIDTH       = 32'd64;
  parameter logic [31:0] PARA_AXI_ADDR_WIDTH       = 32'd32;
  parameter logic [31:0] PARA_AXI_ID_WIDTH         = 32'd2;
  parameter logic [31:0] PARA_MAX_BURST_BEATS      = 32'd256;
  parameter logic [31:0] PARA_MAX_OUTSTANDING      = 32'd8;
  parameter logic [31:0] PARA_RAW_FIFO_DEPTH       = 32'd512;
  parameter logic [31:0] PARA_META_DATA_FIFO_DEPTH = 32'd64;
  parameter logic [31:0] PARA_CMD_FIFO_DEPTH       = 32'd16;
  parameter logic [31:0] PARA_ASYNC_MM_CLK         = 32'd1;
  parameter logic [31:0] PARA_SYNC_STAGES          = 32'd2;
  parameter logic [31:0] PARA_META_MAX_BURST_BEATS = 32'd32;

  localparam int DW   = int'(PARA_AXI_DATA_WIDTH);
  localparam int NB   = int'(PARA_AXI_DATA_WIDTH) / 8;
  localparam int IDW  = int'(PARA_AXI_ID_WIDTH);
  localparam int AWW  = int'(PARA_AXI_ADDR_WIDTH);
  localparam int MBB  = int'(PARA_MAX_BURST_BEATS);
  localparam int MMBB = int'(PARA_META_MAX_BURST_BEATS);
  localparam int MOUT = int'(PARA_MAX_OUTSTANDING);
  localparam int SYNC = int'(PARA_SYNC_STAGES);

  // Tri-state used by the model when the specification leaves a cycle open.
  localparam int NO  = 0;
  localparam int YES = 1;
  localparam int MAY = 2;

  // Model states of a buffer.
  localparam int M_FREE  = 0;
  localparam int M_WRITE = 1;
  localparam int M_COMPL = 2;   // every input of the capture received; must end as valid
  localparam int M_ABORT = 3;   // aborted; ends free or as error
  localparam int M_SWV   = 4;
  localparam int M_SWE   = 5;

  // Kinds of expected bytes.
  localparam int K_RAW = 1;
  localparam int K_REC = 2;
  localparam int K_PAY = 3;

  // --------------------------------------------------------------------------------------------
  // Clocks and resets
  // --------------------------------------------------------------------------------------------
  real  t_sys = 5.0;
  real  t_mm  = 13.0;
  real  t_ph  = 1.0;
  int   clk_sel = 0;
  bit   clk_go  = 1'b0;
  logic sys_clk = 1'b0;
  logic mm_src  = 1'b0;
  logic mm_aclk;
  logic resetn    = 1'b0;
  logic mm_resetn = 1'b0;

  assign mm_aclk = (PARA_ASYNC_MM_CLK == 32'd0) ? sys_clk : mm_src;

  initial begin
    wait (clk_go);
    forever #(t_sys / 2.0) sys_clk = ~sys_clk;
  end

  initial begin
    wait (clk_go);
    #(t_ph);
    forever #(t_mm / 2.0) mm_src = ~mm_src;
  end

  // --------------------------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------------------------
  logic                 soft_reset;
  logic [63:0]          fbd_data;
  logic [7:0]           fbd_byte_en;
  logic                 fbd_valid, fbd_sof, fbd_sol, fbd_eol, fbd_err;
  logic [1:0]           fbd_vc_tag;
  logic                 fbd_ready;
  logic [1:0]           capture_start, capture_end, capture_dropped;
  logic                 ev_fs, lead_vc;
  logic [1:0]           ev_vc, ctx_slot, ctx_orphan;
  logic [13:0]          frame_id_buf0, frame_id_buf1;
  logic [31:0]          meta_data;
  logic [3:0]           meta_byte_en;
  logic                 meta_valid, meta_last, meta_kind, meta_sof, meta_ready;
  logic [1:0]           meta_vc;
  logic                 dma_buf_claim, dma_buf_index;
  logic                 active_sel;
  logic [15:0]          tpg_height_shadow;
  logic                 dma_abort_req, dma_drop;
  logic                 dma_buf_done_ok, dma_buf_done_err, dma_axi_write_error;
  logic                 dma_buffer_overflow;
  logic                 dma_enable, dma_start, dma_abort, err_frame_policy;
  logic [31:0]          dma_buffer0_addr, dma_buffer1_addr, dma_buffer_size, meta_area_size;
  logic [15:0]          bytes_per_line;
  logic [7:0]           line_padding;
  logic [3:0]           awqos;
  logic [7:0]           max_burst_len;
  logic [2:0]           awprot;
  logic [1:0]           release_buf;
  logic                 dma_busy, active_buffer;
  logic [1:0]           buf0_state, buf1_state;
  logic [15:0]          buf0_last_valid_line, buf1_last_valid_line;
  logic [7:0]           axi_fault_awid;
  logic [1:0]           axi_fault_resp;
  logic [31:0]          axi_fault_awaddr;
  logic [15:0]          bresp_slverr_count, bresp_decerr_count;
  logic [IDW-1:0]       m_axi4_awid;
  logic [AWW-1:0]       m_axi4_awaddr;
  logic [7:0]           m_axi4_awlen;
  logic [2:0]           m_axi4_awsize;
  logic [1:0]           m_axi4_awburst;
  logic                 m_axi4_awlock;
  logic [3:0]           m_axi4_awcache;
  logic [2:0]           m_axi4_awprot;
  logic [3:0]           m_axi4_awqos;
  logic                 m_axi4_awvalid, m_axi4_awready;
  logic [DW-1:0]        m_axi4_wdata;
  logic [NB-1:0]        m_axi4_wstrb;
  logic                 m_axi4_wlast, m_axi4_wvalid, m_axi4_wready;
  logic [IDW-1:0]       m_axi4_bid;
  logic [1:0]           m_axi4_bresp;
  logic                 m_axi4_bvalid, m_axi4_bready;

  m_fpt_dma #(
    .PARA_AXI_DATA_WIDTH       (PARA_AXI_DATA_WIDTH),
    .PARA_AXI_ADDR_WIDTH       (PARA_AXI_ADDR_WIDTH),
    .PARA_AXI_ID_WIDTH         (PARA_AXI_ID_WIDTH),
    .PARA_MAX_BURST_BEATS      (PARA_MAX_BURST_BEATS),
    .PARA_MAX_OUTSTANDING      (PARA_MAX_OUTSTANDING),
    .PARA_RAW_FIFO_DEPTH       (PARA_RAW_FIFO_DEPTH),
    .PARA_META_DATA_FIFO_DEPTH (PARA_META_DATA_FIFO_DEPTH),
    .PARA_CMD_FIFO_DEPTH       (PARA_CMD_FIFO_DEPTH),
    .PARA_ASYNC_MM_CLK         (PARA_ASYNC_MM_CLK),
    .PARA_SYNC_STAGES          (PARA_SYNC_STAGES),
    .PARA_META_MAX_BURST_BEATS (PARA_META_MAX_BURST_BEATS)
  ) u_dut (
    .i_sys_clk              (sys_clk),
    .i_mm_aclk              (mm_aclk),
    .i_resetn               (resetn),
    .i_mm_resetn            (mm_resetn),
    .i_soft_reset           (soft_reset),
    .i_fbd_data             (fbd_data),
    .i_fbd_byte_en          (fbd_byte_en),
    .i_fbd_valid            (fbd_valid),
    .i_fbd_sof              (fbd_sof),
    .i_fbd_sol              (fbd_sol),
    .i_fbd_eol              (fbd_eol),
    .i_fbd_vc_tag           (fbd_vc_tag),
    .i_fbd_err              (fbd_err),
    .o_fbd_ready            (fbd_ready),
    .i_capture_start        (capture_start),
    .i_capture_end          (capture_end),
    .i_capture_dropped      (capture_dropped),
    .i_ev_fs                (ev_fs),
    .i_ev_vc                (ev_vc),
    .i_lead_vc              (lead_vc),
    .i_ctx_slot             (ctx_slot),
    .i_ctx_orphan           (ctx_orphan),
    .i_frame_id_buf0        (frame_id_buf0),
    .i_frame_id_buf1        (frame_id_buf1),
    .i_meta_data            (meta_data),
    .i_meta_byte_en         (meta_byte_en),
    .i_meta_valid           (meta_valid),
    .i_meta_last            (meta_last),
    .i_meta_kind            (meta_kind),
    .i_meta_vc              (meta_vc),
    .i_meta_sof             (meta_sof),
    .o_meta_ready           (meta_ready),
    .o_dma_buf_claim        (dma_buf_claim),
    .o_dma_buf_index        (dma_buf_index),
    .i_active_sel           (active_sel),
    .i_tpg_height_shadow    (tpg_height_shadow),
    .i_dma_abort_req        (dma_abort_req),
    .o_dma_drop             (dma_drop),
    .o_dma_buf_done_ok      (dma_buf_done_ok),
    .o_dma_buf_done_err     (dma_buf_done_err),
    .o_dma_axi_write_error  (dma_axi_write_error),
    .o_dma_buffer_overflow  (dma_buffer_overflow),
    .i_dma_enable           (dma_enable),
    .i_dma_start            (dma_start),
    .i_dma_abort            (dma_abort),
    .i_err_frame_policy     (err_frame_policy),
    .i_dma_buffer0_addr     (dma_buffer0_addr),
    .i_dma_buffer1_addr     (dma_buffer1_addr),
    .i_dma_buffer_size      (dma_buffer_size),
    .i_meta_area_size       (meta_area_size),
    .i_bytes_per_line       (bytes_per_line),
    .i_line_padding         (line_padding),
    .i_awqos                (awqos),
    .i_max_burst_len        (max_burst_len),
    .i_awprot               (awprot),
    .i_release_buf          (release_buf),
    .o_dma_busy             (dma_busy),
    .o_active_buffer        (active_buffer),
    .o_buf0_state           (buf0_state),
    .o_buf1_state           (buf1_state),
    .o_buf0_last_valid_line (buf0_last_valid_line),
    .o_buf1_last_valid_line (buf1_last_valid_line),
    .o_axi_fault_awid       (axi_fault_awid),
    .o_axi_fault_resp       (axi_fault_resp),
    .o_axi_fault_awaddr     (axi_fault_awaddr),
    .o_bresp_slverr_count   (bresp_slverr_count),
    .o_bresp_decerr_count   (bresp_decerr_count),
    .o_m_axi4_awid          (m_axi4_awid),
    .o_m_axi4_awaddr        (m_axi4_awaddr),
    .o_m_axi4_awlen         (m_axi4_awlen),
    .o_m_axi4_awsize        (m_axi4_awsize),
    .o_m_axi4_awburst       (m_axi4_awburst),
    .o_m_axi4_awlock        (m_axi4_awlock),
    .o_m_axi4_awcache       (m_axi4_awcache),
    .o_m_axi4_awprot        (m_axi4_awprot),
    .o_m_axi4_awqos         (m_axi4_awqos),
    .o_m_axi4_awvalid       (m_axi4_awvalid),
    .i_m_axi4_awready       (m_axi4_awready),
    .o_m_axi4_wdata         (m_axi4_wdata),
    .o_m_axi4_wstrb         (m_axi4_wstrb),
    .o_m_axi4_wlast         (m_axi4_wlast),
    .o_m_axi4_wvalid        (m_axi4_wvalid),
    .i_m_axi4_wready        (m_axi4_wready),
    .i_m_axi4_bid           (m_axi4_bid),
    .i_m_axi4_bresp         (m_axi4_bresp),
    .i_m_axi4_bvalid        (m_axi4_bvalid),
    .o_m_axi4_bready        (m_axi4_bready)
  );

  // --------------------------------------------------------------------------------------------
  // Asynchronous FIFO, unit level: four instances driven only by the FIFO tests.
  // 0: depth 2, width 8, two clocks, 3 stages; 1: depth 16, width 72, two clocks, 2 stages;
  // 2: depth 512, width 72, two clocks, 2 stages; 3: depth 16, width 85, one clock.
  // --------------------------------------------------------------------------------------------
  localparam int NF = 4;
  real  f_wper = 5.0;
  real  f_rper = 13.0;
  bit   f_run  = 1'b0;
  logic f_wclk = 1'b0;
  logic f_rclk = 1'b0;
  logic f_wrst_n = 1'b0;
  logic f_rrst_n = 1'b0;
  logic        f_push [NF];
  logic        f_pop  [NF];
  logic [84:0] f_din  [NF];
  logic        f_full [NF];
  logic        f_empty[NF];
  logic [84:0] f_dout [NF];
  logic [9:0]  f_used [NF];
  int          f_depth[NF] = '{2, 16, 512, 16};
  int          f_width[NF] = '{8, 72, 72, 85};
  int          f_sync [NF] = '{3, 2, 2, 2};
  int          f_async[NF] = '{1, 1, 1, 0};

  initial begin
    wait (f_run);
    forever begin
      #(f_wper / 2.0) f_wclk = ~f_wclk;
    end
  end

  initial begin
    wait (f_run);
    #(1.3);
    forever begin
      #(f_rper / 2.0) f_rclk = ~f_rclk;
    end
  end

  logic [1:0]  f0_used;
  logic [4:0]  f1_used;
  logic [9:0]  f2_used;
  logic [4:0]  f3_used;
  logic [7:0]  f0_dout;
  logic [71:0] f1_dout;
  logic [71:0] f2_dout;
  logic [84:0] f3_dout;

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(32'd2), .PARA_WIDTH(32'd8), .PARA_ASYNC(32'd1), .PARA_SYNC_STAGES(32'd3)
  ) u_fifo_0 (
    .i_wr_clk (f_wclk), .i_wr_rst_n (f_wrst_n), .i_push (f_push[0]), .i_din (f_din[0][7:0]),
    .o_full (f_full[0]), .o_wr_used (f0_used), .i_rd_clk (f_rclk), .i_rd_rst_n (f_rrst_n),
    .i_pop (f_pop[0]), .o_dout (f0_dout), .o_empty (f_empty[0])
  );

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(32'd16), .PARA_WIDTH(32'd72), .PARA_ASYNC(32'd1), .PARA_SYNC_STAGES(32'd2)
  ) u_fifo_1 (
    .i_wr_clk (f_wclk), .i_wr_rst_n (f_wrst_n), .i_push (f_push[1]), .i_din (f_din[1][71:0]),
    .o_full (f_full[1]), .o_wr_used (f1_used), .i_rd_clk (f_rclk), .i_rd_rst_n (f_rrst_n),
    .i_pop (f_pop[1]), .o_dout (f1_dout), .o_empty (f_empty[1])
  );

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(32'd512), .PARA_WIDTH(32'd72), .PARA_ASYNC(32'd1), .PARA_SYNC_STAGES(32'd2)
  ) u_fifo_2 (
    .i_wr_clk (f_wclk), .i_wr_rst_n (f_wrst_n), .i_push (f_push[2]), .i_din (f_din[2][71:0]),
    .o_full (f_full[2]), .o_wr_used (f2_used), .i_rd_clk (f_rclk), .i_rd_rst_n (f_rrst_n),
    .i_pop (f_pop[2]), .o_dout (f2_dout), .o_empty (f_empty[2])
  );

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(32'd16), .PARA_WIDTH(32'd85), .PARA_ASYNC(32'd0), .PARA_SYNC_STAGES(32'd2)
  ) u_fifo_3 (
    .i_wr_clk (f_wclk), .i_wr_rst_n (f_wrst_n), .i_push (f_push[3]), .i_din (f_din[3]),
    .o_full (f_full[3]), .o_wr_used (f3_used), .i_rd_clk (f_wclk), .i_rd_rst_n (f_wrst_n),
    .i_pop (f_pop[3]), .o_dout (f3_dout), .o_empty (f_empty[3])
  );

  always_comb begin
    f_used[0] = 10'(f0_used);
    f_used[1] = 10'(f1_used);
    f_used[2] = 10'(f2_used);
    f_used[3] = 10'(f3_used);
    f_dout[0] = 85'(f0_dout);
    f_dout[1] = 85'(f1_dout);
    f_dout[2] = 85'(f2_dout);
    f_dout[3] = f3_dout;
  end

  // --------------------------------------------------------------------------------------------
  // Reporting
  // --------------------------------------------------------------------------------------------
  string test_name = "none";
  int    seed      = 1;
  int    n_err     = 0;
  int    n_chk     = 0;
  int    max_cyc   = 3000000;

  function automatic void fail(input string msg);
    n_err++;
    if (n_err <= 30) $error("%0s", msg);
    else if (n_err == 31) $error("%0s", "more errors follow, not printed");
  endfunction

  function automatic int popc8(input logic [7:0] v);
    int n = 0;
    for (int i = 0; i < 8; i++) n += int'(v[i]);
    return n;
  endfunction

  function automatic int and3(input int a, input int b);
    if (a == NO || b == NO) return NO;
    if (a == YES && b == YES) return YES;
    return MAY;
  endfunction

  function automatic int max3(input int a, input int b);
    return (a > b) ? a : b;
  endfunction

  // --------------------------------------------------------------------------------------------
  // Expected bytes and written bytes of each buffer, keyed by byte address
  // --------------------------------------------------------------------------------------------
  logic [7:0] ev0 [int unsigned];
  logic [7:0] ev1 [int unsigned];
  int         et0 [int unsigned];
  int         et1 [int unsigned];
  bit         wm0 [int unsigned];
  bit         wm1 [int unsigned];

  bit fault_inject = 1'b0;   // test_checker_fault: one expected byte made wrong
  bit fault_done   = 1'b0;

  function automatic void exp_put(input int b, input longint a, input logic [7:0] v,
                                  input int tag);
    int unsigned k = int'(a);
    if (fault_inject && !fault_done && (tag >> 24) == K_RAW) begin
      v = ~v;
      fault_done = 1'b1;
    end
    if (b == 0) begin ev0[k] = v; et0[k] = tag; end
    else        begin ev1[k] = v; et1[k] = tag; end
  endfunction

  function automatic bit exp_has(input int b, input int unsigned a);
    return (b == 0) ? ev0.exists(a) : ev1.exists(a);
  endfunction

  function automatic logic [7:0] exp_val(input int b, input int unsigned a);
    return (b == 0) ? ev0[a] : ev1[a];
  endfunction

  function automatic void exp_clear(input int b);
    if (b == 0) begin ev0.delete(); et0.delete(); wm0.delete(); end
    else        begin ev1.delete(); et1.delete(); wm1.delete(); end
  endfunction

  function automatic bit wm_has(input int b, input int unsigned a);
    return (b == 0) ? wm0.exists(a) : wm1.exists(a);
  endfunction

  function automatic void wm_set(input int b, input int unsigned a);
    if (b == 0) wm0[a] = 1'b1;
    else        wm1[a] = 1'b1;
  endfunction

  // Tag of an expected byte: kind, channel and line index.
  function automatic int mk_tag(input int kind, input int x, input int line);
    return (kind << 24) | (x << 20) | (line & 32'hFFFFF);
  endfunction

  // --------------------------------------------------------------------------------------------
  // Reference model state (sys_clk side)
  // --------------------------------------------------------------------------------------------
  // Buffers: state, configuration sampled at the claim, per-channel progress.
  int     m_st      [2];
  int     m_wrunc   [2];   // the cycle at which the buffer stops being written is not known
  int     m_mayab   [2];   // an abort cause arrived while the buffer was completing
  int     m_maysr   [2];   // soft_reset arrived while the buffer was completing
  int     m_srseen  [2];
  int     m_allowswe[2];
  int     m_policy  [2];
  int     m_tpg     [2];
  int     m_tpgh    [2];
  longint m_base    [2];
  longint m_size    [2];
  longint m_meta    [2];
  int     m_bpl     [2];
  int     m_pad     [2];
  int     m_mbl     [2];
  int     m_qos     [2];
  int     m_prot    [2];
  int     m_ended   [2];
  int     m_join    [2][2];
  int     m_recseen [2][2];
  int     m_expl    [2][2];
  int     m_lines   [2][2];
  int     m_moved   [2][2];
  int     m_claimcyc[2];
  int     m_stcyc   [2];
  int     m_nout    [2];
  int     m_lvl     [2][2];
  int     m_lvlstop [2];
  int     m_ever    [2];
  int     m_relcyc  [2];
  int     m_fid     [2];
  int     m_out     [2];   // outcome of the last claim: 0 open, M_SWV, M_SWE or M_FREE

  // Lines written into each buffer (for the end-of-line flag of a write and the burst checks).
  typedef struct {
    int     x;
    int     lidx;
    longint start;
    int     len;
    int     wr;
    bit     done;
  } line_t;
  line_t  lq0 [$];
  line_t  lq1 [$];
  int     cur_le [2][2];

  // Bindings: pending (shared by both writers), active pixel and metadata bindings, fresh flags.
  int     pend_v [2];
  int     pend_b [2];
  int     actp_v [2];
  int     actp_b [2];
  int     actm_v [2];
  int     actm_b [2];
  int     fresh_p[2];
  int     fresh_m[2];
  int     line_idx[2];
  int     line_bytes;
  int     pay_ptr[2];
  int     slot_v [2];
  int     slot_b [2];

  // Record assembler.
  int          ra_n;
  int          ra_vc;
  int          ra_rv;
  int          ra_rb;
  logic [31:0] ra_d [10];
  logic [3:0]  ra_be[10];

  // Registered capture events, last claimed buffer.
  logic [1:0] q_cs;
  int         q_fs;
  int         q_evc;
  int         q_lead;
  int         last_buf;
  int         act_exp;
  int         act_cyc;

  // Monitor bookkeeping.
  bit     sys_run = 1'b0;
  bit     mm_run  = 1'b0;
  int     cyc     = 0;
  int     mcyc    = 0;
  logic [1:0] p_so [2];
  int     a_open_ok;
  int     a_open_err;
  int     busy0;
  int     bh_ok [2];
  int     bh_x  [2];
  int     bh_v  [2];
  int     bh_b  [2];
  int     bh_tpg[2];
  typedef struct {
    int lo;
    int hi;
    bit must;
  } ovf_t;
  ovf_t   ovf_q [$];
  int     cap_res [$];
  int     n_cap_res;
  int     n_claim, n_drop, n_swv, n_swe, n_free_ab, n_ovf_pulse, n_aerr_pulse;
  int     meta_low_seen, fbd_low_seen, n_loss;
  bit     lenient = 1'b0;
  realtime t_sr = 0.0;
  logic [13:0] last_alloc_fid;
  bit     fidw_pend = 1'b0;
  int     fidw_idx;
  logic [13:0] fidw_val;
  bit     ovf_quiet = 1'b0;   // set by the fault test: no overflow check
  bit     dbg = 1'b0;         // +DBG: trace of the model events

  // AXI side counters shared with the model.
  int     n_aw, n_bh, n_err_resp, max_out, n_wbytes;
  typedef struct {
    realtime t;
    int      id;
    longint  addr;
    int      resp;
  } er_t;
  er_t    er_q [$];

  function automatic void set_state(input int b, input int st);
    m_st[b]   = st;
    m_stcyc[b] = cyc;
  endfunction

  function automatic int writing(input int b);
    if (b < 0 || b > 1) return NO;
    case (m_st[b])
      M_WRITE: return (m_wrunc[b] != 0) ? MAY : YES;
      M_COMPL: return (m_wrunc[b] != 0) ? MAY : NO;
      M_ABORT: return (m_wrunc[b] != 0) ? MAY : NO;
      default: return NO;
    endcase
  endfunction

  function automatic void set_abort(input int b, input int unc);
    if (m_st[b] == M_WRITE || m_st[b] == M_COMPL) begin
      set_state(b, M_ABORT);
      m_wrunc[b] = unc;
      m_mayab[b] = 0;
      m_maysr[b] = 0;
    end
  endfunction

  function automatic int maybe_free(input int b);
    if (m_st[b] == M_ABORT)
      return int'(m_policy[b] == 0 || m_srseen[b] != 0 || m_allowswe[b] != 0);
    if (m_st[b] == M_COMPL)
      return int'(m_maysr[b] != 0 || (m_mayab[b] != 0 && m_policy[b] == 0));
    return 0;
  endfunction

  function automatic void resolve_free(input int b);
    if (maybe_free(b) == 0)
      fail($sformatf("buffer %0d became free at cycle %0d; model state %0d, policy %0d",
                     b, cyc, m_st[b], m_policy[b]));
    set_state(b, M_FREE);
    m_out[b] = M_FREE;
    n_free_ab++;
  endfunction

  function automatic int swe_ok(input int b);
    if (m_st[b] == M_ABORT)
      return int'(m_policy[b] == 1 && (m_srseen[b] == 0 || m_allowswe[b] != 0));
    if (m_st[b] == M_COMPL)
      return int'(m_mayab[b] != 0 && m_policy[b] == 1 && (m_maysr[b] == 0 || lenient));
    return 0;
  endfunction

  function automatic longint stride(input int b);
    return longint'(((m_bpl[b] + m_pad[b]) >> 3) << 3);
  endfunction

  function automatic longint line_addr(input int b, input int x, input int lidx);
    return m_base[b] + longint'(x) * m_size[b] + longint'(lidx) * stride(b);
  endfunction

  function automatic longint meta_base(input int b, input int x);
    return m_base[b] + 2 * m_size[b] + longint'(x) * m_meta[b];
  endfunction

  // Largest burst in 64-bit beats of the RAW and metadata writers of buffer b.
  function automatic int lim64_raw(input int b);
    int l = m_mbl[b] + 1;
    if (DW == 64) return l;
    return (l / 2 < 1) ? 1 : l / 2;
  endfunction

  function automatic int lim64_meta(input int b);
    int l = m_mbl[b] + 1;
    if (MMBB < l) l = MMBB;
    if (DW == 64) return l;
    return (l / 2 < 1) ? 1 : l / 2;
  endfunction

  function automatic void set_moved(input int b, input int x, input int v);
    if (b < 0 || b > 1) return;
    if (v == YES) m_moved[b][x] = YES;
    else if (m_moved[b][x] == NO) m_moved[b][x] = MAY;
  endfunction

  function automatic void add_ovf(input int lo, input int hi, input bit must);
    ovf_t o;
    o.lo = lo;
    o.hi = hi;
    o.must = must;
    ovf_q.push_back(o);
  endfunction

  // Line bookkeeping for the AXI side.
  function automatic void line_note(input int b, input int x, input int lidx, input longint la,
                                    input int pc, input bit sol, input bit eol);
    line_t e;
    int i;
    if (sol) begin
      e.x = x; e.lidx = lidx; e.start = la; e.len = pc; e.wr = 0; e.done = eol;
      if (b == 0) begin lq0.push_back(e); cur_le[0][x] = lq0.size() - 1; end
      else        begin lq1.push_back(e); cur_le[1][x] = lq1.size() - 1; end
    end else begin
      i = cur_le[b][x];
      if (i >= 0) begin
        if (b == 0) begin lq0[i].len += pc; if (eol) lq0[i].done = 1'b1; end
        else        begin lq1[i].len += pc; if (eol) lq1[i].done = 1'b1; end
      end
    end
  endfunction

  // A buffer just claimed: every binding still naming it belongs to an earlier capture.
  function automatic void stale_clear(input int b);
    for (int y = 0; y < 2; y++) begin
      if (actp_b[y] == b) actp_v[y] = NO;
      if (actm_b[y] == b) actm_v[y] = NO;
      if (pend_b[y] == b) pend_v[y] = NO;
      if (slot_b[y] == b) slot_v[y] = NO;
    end
    if (ra_rb == b) ra_rv = NO;
  endfunction

  function automatic void do_claim(input int b, input int lead);
    set_state(b, M_WRITE);
    m_out[b] = 0;
    exp_clear(b);
    if (b == 0) lq0.delete(); else lq1.delete();
    cur_le[b][0] = -1;  cur_le[b][1] = -1;
    m_wrunc[b] = 0;  m_mayab[b] = 0;  m_maysr[b] = 0;  m_srseen[b] = 0;  m_allowswe[b] = 0;
    m_base[b]  = longint'(((b == 0) ? dma_buffer0_addr : dma_buffer1_addr) & 32'hFFFF_FFF8);
    m_size[b]  = longint'(dma_buffer_size & 32'hFFFF_FFF8);
    m_meta[b]  = longint'(meta_area_size & 32'hFFFF_FFF8);
    m_bpl[b]   = int'(bytes_per_line);
    m_pad[b]   = int'(line_padding);
    m_mbl[b]   = (int'(max_burst_len) + 1 > MBB) ? MBB - 1 : int'(max_burst_len);
    m_qos[b]   = int'(awqos);
    m_prot[b]  = int'(awprot);
    m_policy[b] = int'(err_frame_policy);
    m_tpg[b]   = int'(active_sel);
    m_tpgh[b]  = int'(tpg_height_shadow);
    m_ended[b] = 0;
    for (int x = 0; x < 2; x++) begin
      m_join[b][x] = (x == lead) ? YES : NO;
      m_recseen[b][x] = 0;  m_expl[b][x] = 0;  m_lines[b][x] = 0;  m_moved[b][x] = NO;
      m_lvl[b][x] = -1;
    end
    m_lvlstop[b] = 0;
    m_claimcyc[b] = cyc;
    m_ever[b] = 1;
    m_fid[b] = int'(last_alloc_fid);
    last_buf = b;
    act_exp = b;
    act_cyc = cyc;
    n_claim++;
    fidw_pend = 1'b1;
    fidw_idx  = b;
    fidw_val  = last_alloc_fid;
  endfunction

  function automatic void handle_sr(input int b);
    case (m_st[b])
      M_WRITE: begin set_abort(b, 0); m_srseen[b] = 1; end
      M_COMPL: begin
        if (m_nout[b] > 0) begin set_abort(b, 0); m_srseen[b] = 1; end
        else begin m_maysr[b] = 1; m_wrunc[b] = 0; end
      end
      M_ABORT: begin
        if (lenient && m_policy[b] == 1 && m_srseen[b] == 0) m_allowswe[b] = 1;
        m_srseen[b] = 1;
        m_wrunc[b] = 0;
      end
      default: ;
    endcase
  endfunction

  // A record ended with meta_last.
  function automatic void record_done();
    int n = (ra_n > 10) ? 10 : ra_n;
    int x = ra_vc;
    int b = ra_rb;
    if (ra_rv == NO || n < 2) return;
    if (longint'(4 * n) > m_meta[b]) begin
      add_ovf(cyc, cyc + 5000, ra_rv == YES);
      if (dbg) $display("dbg %0d: record of %0d words past the area of buffer %0d", cyc, n, b);
      if (ra_rv == YES && m_st[b] == M_WRITE) set_abort(b, 1);
      return;
    end
    for (int i = 0; i < n; i++)
      for (int k = 0; k < 4; k++)
        if (ra_be[i][k]) exp_put(b, meta_base(b, x) + 4 * i + k, ra_d[i][8*k +: 8],
                                 mk_tag(K_REC, x, 0));
    if (ra_rv == YES) begin
      m_recseen[b][x] = 1;
      m_expl[b][x] = (m_tpg[b] != 0) ? m_tpgh[b] : int'(ra_d[5][31:16]);
    end
  endfunction

  // Is an expected byte required in a buffer released as valid?
  function automatic bit tag_req(input int b, input int tag);
    int kind = tag >> 24;
    int x    = (tag >> 20) & 15;
    int line = tag & 32'hFFFFF;
    if (kind == K_RAW) return (m_join[b][x] == YES) && (line < m_expl[b][x]);
    return 1'b1;
  endfunction

  // A buffer released as valid holds exactly the expected bytes.
  function automatic void check_valid_buffer(input int b);
    int nreq = 0;
    int nmiss = 0;
    int nextra = 0;
    if (b == 0) begin
      foreach (ev0[a]) if (tag_req(0, et0[a])) begin
        nreq++;
        if (!wm0.exists(a)) begin
          nmiss++;
          if (nmiss <= 3) fail($sformatf("buffer 0 valid but byte %08h not written", a));
        end
      end
      foreach (wm0[a]) if (!tag_req(0, et0[a])) begin
        nextra++;
        if (nextra <= 3) fail($sformatf("buffer 0 valid but holds extra byte %08h", a));
      end
    end else begin
      foreach (ev1[a]) if (tag_req(1, et1[a])) begin
        nreq++;
        if (!wm1.exists(a)) begin
          nmiss++;
          if (nmiss <= 3) fail($sformatf("buffer 1 valid but byte %08h not written", a));
        end
      end
      foreach (wm1[a]) if (!tag_req(1, et1[a])) begin
        nextra++;
        if (nextra <= 3) fail($sformatf("buffer 1 valid but holds extra byte %08h", a));
      end
    end
    for (int x = 0; x < 2; x++)
      if (m_join[b][x] == YES && m_recseen[b][x] == 0)
        fail($sformatf("buffer %0d valid without the record of channel %0d", b, x));
    n_chk += nreq + 1;
  endfunction

  function automatic void model_reset();
    for (int b = 0; b < 2; b++) begin
      m_st[b] = M_FREE;  m_stcyc[b] = 0;  m_wrunc[b] = 0;  m_mayab[b] = 0;  m_maysr[b] = 0;
      m_srseen[b] = 0;  m_allowswe[b] = 0;  m_policy[b] = 1;  m_tpg[b] = 0;  m_tpgh[b] = 0;
      m_base[b] = 0;  m_size[b] = 0;  m_meta[b] = 0;  m_bpl[b] = 0;  m_pad[b] = 0;
      m_mbl[b] = 0;  m_qos[b] = 0;  m_prot[b] = 0;  m_ended[b] = 0;  m_claimcyc[b] = 0;
      m_nout[b] = 0;  m_lvlstop[b] = 0;  m_ever[b] = 0;  m_relcyc[b] = -100;  m_fid[b] = 0;
      m_out[b] = 0;
      for (int x = 0; x < 2; x++) begin
        m_join[b][x] = NO;  m_recseen[b][x] = 0;  m_expl[b][x] = 0;  m_lines[b][x] = 0;
        m_moved[b][x] = NO;  m_lvl[b][x] = -1;  cur_le[b][x] = -1;
      end
      exp_clear(b);
      pend_v[b] = NO;  pend_b[b] = 0;  actp_v[b] = NO;  actp_b[b] = 0;
      actm_v[b] = NO;  actm_b[b] = 0;  fresh_p[b] = 0;  fresh_m[b] = 0;  line_idx[b] = 0;
      pay_ptr[b] = 0;  slot_v[b] = NO;  slot_b[b] = 0;  p_so[b] = 2'b00;
      bh_ok[b] = 0;  bh_x[b] = 0;  bh_v[b] = NO;  bh_b[b] = 0;  bh_tpg[b] = 0;
    end
    lq0.delete();  lq1.delete();
    line_bytes = 0;  ra_n = 0;  ra_vc = 0;  ra_rv = NO;  ra_rb = 0;
    q_cs = 2'b00;  q_fs = 0;  q_evc = 0;  q_lead = 0;
    last_buf = 1;  act_exp = 0;  act_cyc = -1;
    a_open_ok = 0;  a_open_err = 0;  busy0 = 0;
    ovf_q.delete();
    er_q.delete();
    n_aw = 0;  n_bh = 0;  n_err_resp = 0;  n_aerr_pulse = 0;
    fidw_pend = 1'b0;
  endfunction

  // --------------------------------------------------------------------------------------------
  // sys_clk monitor: one call per rising edge, in the order the specification evaluates a cycle
  // --------------------------------------------------------------------------------------------
  task automatic mon_sys();
    int sr, aa, ne, run, ok, pk, match, beat, x, y, s, b, k, lidx, lbp, lb, pc, cur, clip;
    int nbv, nbb, mv, mx, w, off, fid, c0, c1, lx, clr, fsv, fsb, must_busy, settled, matched;
    int found, n01, n10, le, dn, ex;
    int claim_now, claim_b;
    int fr [2];
    int f  [2];
    int bind_v[2];
    int bind_b[2];
    int bm_v [2];
    int bm_b [2];
    int sof_tk[2];
    int msof_tk[2];
    int wr_exact[2];
    int doom_unc[2];
    int cause[2];
    int lvn[2];
    int pl[2];
    int bend[2];
    int bdrop[2];
    logic [1:0] so [2];
    longint la, rend;

    cyc++;
    sr  = int'(soft_reset);
    aa  = int'(dma_abort_req || dma_abort);
    ne  = int'(!dma_enable);
    run = int'(dma_enable && dma_start && !dma_abort);
    if (sr) t_sr = $realtime;
    for (int i = 0; i < 2; i++) begin
      sof_tk[i] = 0;  msof_tk[i] = 0;  wr_exact[i] = 0;  doom_unc[i] = 0;
    end

    // X/Z on outputs
    if ($isunknown({fbd_ready, meta_ready, dma_buf_claim, dma_buf_index, dma_drop,
                    dma_buf_done_ok, dma_buf_done_err, dma_axi_write_error,
                    dma_buffer_overflow, dma_busy, active_buffer, buf0_state, buf1_state,
                    buf0_last_valid_line, buf1_last_valid_line, axi_fault_awid,
                    axi_fault_resp, axi_fault_awaddr, bresp_slverr_count,
                    bresp_decerr_count}))
      fail($sformatf("X or Z on a sys_clk output at cycle %0d", cyc));

    // ---- claim and dma_drop, one cycle after capture_start
    claim_now = 0;
    claim_b = -1;
    if (dma_buf_claim && dma_drop) fail($sformatf("claim and dma_drop together at cycle %0d", cyc));
    if (q_cs != 2'b00 && run && !sr) begin
      for (int i = 0; i < 2; i++)
        fr[i] = (m_st[i] == M_FREE) ? YES : ((maybe_free(i) != 0) ? MAY : NO);
      ok = 0;
      for (int c = 0; c < 4; c++) begin
        if (fr[0] != MAY && (c & 1) != 0) continue;
        if (fr[1] != MAY && (c & 2) != 0) continue;
        f[0] = int'(fr[0] == YES || (c & 1) != 0);
        f[1] = int'(fr[1] == YES || (c & 2) != 0);
        pk = (f[1 - last_buf] != 0) ? 1 - last_buf : ((f[last_buf] != 0) ? last_buf : -1);
        if (pk < 0) match = int'(dma_drop && !dma_buf_claim);
        else match = int'(dma_buf_claim && !dma_drop && int'(dma_buf_index) == pk);
        if (match != 0) ok = 1;
      end
      if (ok == 0)
        fail($sformatf("cycle %0d: claim %0b index %0b drop %0b; free %0d %0d, last buffer %0d",
                       cyc, dma_buf_claim, dma_buf_index, dma_drop, fr[0], fr[1], last_buf));
      else n_chk++;
      if (dma_buf_claim) begin
        claim_now = 1;
        claim_b = int'(dma_buf_index);
        if (m_st[claim_b] != M_FREE) resolve_free(claim_b);
        stale_clear(claim_b);
      end
      if (dma_drop) n_drop++;
      cap_res.push_back(claim_now ? claim_b : -1);
      n_cap_res++;
    end else begin
      if (dma_buf_claim || dma_drop)
        fail($sformatf("claim or dma_drop at cycle %0d without a capture to claim", cyc));
      if (q_cs != 2'b00) begin
        cap_res.push_back(-2);
        n_cap_res++;
      end
    end

    // ---- pixel path: fbd_ready, bindings, line address and clip, expected bytes
    beat = int'(fbd_valid && fbd_ready);
    x = int'(fbd_vc_tag[0]);
    if (fbd_valid) begin
      if (fbd_sof) begin nbv = (fresh_p[x] != 0) ? pend_v[x] : NO; nbb = pend_b[x]; end
      else begin nbv = actp_v[x]; nbb = actp_b[x]; end
      if (!fbd_ready) begin
        fbd_low_seen++;
        if (nbv == NO || writing(nbb) == NO || m_tpg[nbb] == 0)
          fail($sformatf("fbd_ready low at cycle %0d for a beat not bound to a test pattern buffer",
                         cyc));
      end
    end
    for (y = 0; y < 2; y++) begin
      if (beat != 0 && fbd_sof && x == y) begin
        nbv = (fresh_p[y] != 0) ? pend_v[y] : NO;
        nbb = pend_b[y];
        sof_tk[y] = 1;
      end else begin
        nbv = actp_v[y];
        nbb = actp_b[y];
      end
      bind_v[y] = (sr != 0) ? NO : and3(nbv, writing(nbb));
      bind_b[y] = nbb;
    end
    bh_ok[1] = bh_ok[0];  bh_x[1] = bh_x[0];  bh_v[1] = bh_v[0];  bh_b[1] = bh_b[0];
    bh_tpg[1] = bh_tpg[0];
    bh_ok[0] = 0;
    if (beat != 0) begin
      if (fbd_sof && actp_v[x] != NO) set_moved(actp_b[x], x, actp_v[x]);
      cur = bind_v[x];
      b = bind_b[x];
      lidx = fbd_sof ? 0 : line_idx[x];
      lbp = fbd_sol ? 0 : line_bytes;
      pc = popc8(fbd_byte_en);
      lb = lbp + pc;
      if (dbg) $display("dbg %0d: beat ch %0d sof %0b sol %0b eol %0b %02h bytes %0d bind %0d %0d",
                        cyc, x, fbd_sof, fbd_sol, fbd_eol, fbd_byte_en, lb, cur, b);
      if (cur != NO) begin
        la = line_addr(b, x, lidx);
        rend = m_base[b] + longint'(x + 1) * m_size[b];
        clip = int'(lb > m_bpl[b] || la + longint'(lb) > rend);
        if (clip != 0) begin
          add_ovf(cyc, cyc + 1, cur == YES);
          if (dbg) $display("dbg %0d: clip ch %0d line %0d bytes %0d buffer %0d bpl %0d", cyc,
                            x, lidx, lb, b, m_bpl[b]);
          if (cur == YES) wr_exact[b] = 1;
        end else begin
          for (k = 0; k < pc; k++)
            exp_put(b, la + lbp + k, fbd_data[8*k +: 8], mk_tag(K_RAW, x, lidx));
          line_note(b, x, lidx, la, pc, fbd_sol, fbd_eol);
          if (fbd_eol) m_lines[b][x]++;
        end
        bh_ok[0] = 1;  bh_x[0] = x;  bh_v[0] = cur;  bh_b[0] = b;  bh_tpg[0] = m_tpg[b];
      end
      line_idx[x] = lidx + int'(fbd_eol);
      line_bytes = lb;
    end

    // ---- metadata path: record assembler, payload binding and pointer
    mv = int'(meta_valid && meta_ready);
    if (meta_valid && !meta_ready) meta_low_seen++;
    mx = int'(meta_vc[0]);
    for (y = 0; y < 2; y++) begin
      if (mv != 0 && meta_kind && meta_sof && mx == y) begin
        nbv = (fresh_m[y] != 0) ? pend_v[y] : NO;
        nbb = pend_b[y];
        msof_tk[y] = 1;
        pay_ptr[y] = 'h80;
      end else begin
        nbv = actm_v[y];
        nbb = actm_b[y];
      end
      bm_v[y] = (sr != 0) ? NO : and3(nbv, writing(nbb));
      bm_b[y] = nbb;
    end
    if (mv != 0) begin
      if (!meta_kind) begin
        if (ra_n == 0) ra_vc = mx;
        if (ra_n < 10) begin ra_d[ra_n] = meta_data; ra_be[ra_n] = meta_byte_en; end
        ra_n++;
        if (ra_n == 2) begin
          fid = int'(meta_data[13:0]);
          c0 = (fid == int'(frame_id_buf0)) ? writing(0) : NO;
          c1 = (fid == int'(frame_id_buf1)) ? writing(1) : NO;
          if (c0 != NO) begin ra_rv = c0; ra_rb = 0; end
          else begin ra_rv = c1; ra_rb = 1; end
        end
        if (meta_last) begin
          record_done();
          ra_n = 0;
          ra_rv = NO;
        end
      end else begin
        w = bm_v[mx];
        b = bm_b[mx];
        if (w != NO) begin
          off = pay_ptr[mx];
          if (longint'(off + 4) > m_meta[b]) begin
            add_ovf(cyc, cyc + 3, w == YES);
            if (dbg) $display("dbg %0d: payload past the area, ch %0d offset %0h buffer %0d",
                              cyc, mx, off, b);
            if (w == YES) doom_unc[b] = 1;
          end else begin
            for (k = 0; k < 4; k++)
              if (meta_byte_en[k]) exp_put(b, meta_base(b, mx) + off + k, meta_data[8*k +: 8],
                                           mk_tag(K_PAY, mx, 0));
          end
        end
        pay_ptr[mx] += 4;
      end
    end

    // ---- abort causes and ends, on the state at the start of the cycle
    for (b = 0; b < 2; b++) begin
      bend[b] = 0;
      bdrop[b] = 0;
      for (s = 0; s < 2; s++)
        if (capture_end[s] && slot_b[s] == b && slot_v[s] != NO) begin
          if (slot_v[s] == YES) begin
            bend[b] = 1;
            if (capture_dropped[s]) bdrop[b] = 1;
          end else if (capture_dropped[s]) doom_unc[b] = 1;
        end
      cause[b] = int'(aa != 0 || ne != 0 || wr_exact[b] != 0 || bdrop[b] != 0);
      lvn[b] = NO;
      if (m_st[b] == M_WRITE || ((m_st[b] == M_COMPL || m_st[b] == M_ABORT) && m_wrunc[b] != 0))
        begin
        if (sr != 0 || cause[b] != 0) lvn[b] = YES;
        else if (m_wrunc[b] != 0 || m_st[b] == M_COMPL) lvn[b] = MAY;
      end
      if (lvn[b] == YES && ra_rb == b) ra_rv = NO;
      if (lvn[b] == MAY && ra_rb == b && ra_rv == YES) ra_rv = MAY;
    end
    for (b = 0; b < 2; b++) begin
      if (bend[b] != 0 && m_st[b] == M_WRITE) m_ended[b] = 1;
      if (sr != 0) handle_sr(b);
      else if (cause[b] != 0) begin
        if (m_st[b] == M_WRITE) set_abort(b, 0);
        else if (m_st[b] == M_COMPL) begin m_mayab[b] = 1; m_wrunc[b] = 0; end
        else if (m_st[b] == M_ABORT) m_wrunc[b] = 0;
      end else if (doom_unc[b] != 0) begin
        if (m_st[b] == M_WRITE) set_abort(b, 1);
        else if (m_st[b] == M_COMPL) m_mayab[b] = 1;
      end
    end

    // ---- pending binding update
    clr = int'(aa != 0 || sr != 0);
    lx = q_evc & 1;
    fsv = NO;
    fsb = 0;
    if (q_fs != 0) begin
      if (q_lead == lx) begin
        fsv = (claim_now != 0) ? YES : NO;
        fsb = (claim_now != 0) ? claim_b : 0;
      end else begin
        s = int'(ctx_slot[lx]);
        if (ctx_orphan[lx] || slot_v[s] == NO) fsv = NO;
        else begin
          fsb = slot_b[s];
          if (lvn[fsb] == YES) fsv = NO;
          else fsv = (slot_v[s] == MAY || lvn[fsb] == MAY) ? MAY : YES;
        end
      end
    end
    for (y = 0; y < 2; y++) pl[y] = int'(q_fs != 0 && lx == y && clr == 0);
    for (y = 0; y < 2; y++)
      if (pl[y] != 0 && fresh_p[y] != 0 && pend_v[y] != NO) set_moved(pend_b[y], y, pend_v[y]);
    if (q_fs != 0 && q_lead != lx && fsv != NO) m_join[fsb][lx] = max3(m_join[fsb][lx], fsv);
    for (y = 0; y < 2; y++) begin
      if (clr != 0) pend_v[y] = NO;
      else if (q_fs != 0 && lx == y) begin pend_v[y] = fsv; pend_b[y] = fsb; end
      else if (pend_v[y] != NO) begin
        if (lvn[pend_b[y]] == YES) pend_v[y] = NO;
        else if (lvn[pend_b[y]] == MAY) pend_v[y] = MAY;
      end
      fresh_p[y] = (sr != 0) ? 0 : int'(pl[y] != 0 || (fresh_p[y] != 0 && sof_tk[y] == 0));
      fresh_m[y] = (sr != 0) ? 0 : int'(pl[y] != 0 || (fresh_m[y] != 0 && msof_tk[y] == 0));
      actp_v[y] = bind_v[y];  actp_b[y] = bind_b[y];
      actm_v[y] = bm_v[y];    actm_b[y] = bm_b[y];
    end
    if (sr != 0) begin
      line_idx[0] = 0;  line_idx[1] = 0;  ra_n = 0;  ra_rv = NO;
    end

    // ---- capture slots
    for (s = 0; s < 2; s++) begin
      if (sr != 0) slot_v[s] = NO;
      else if (q_cs[s]) begin
        slot_v[s] = (claim_now != 0) ? YES : NO;
        slot_b[s] = (claim_now != 0) ? claim_b : 0;
      end else if (slot_v[s] != NO) begin
        if (lvn[slot_b[s]] == YES) slot_v[s] = NO;
        else if (lvn[slot_b[s]] == MAY) slot_v[s] = MAY;
      end
    end

    // ---- claim takes effect after the abort causes of this cycle
    if (claim_now != 0) do_claim(claim_b, q_lead);

    // ---- channel completion and line count errors
    for (b = 0; b < 2; b++) if (m_st[b] == M_WRITE) begin
      le = NO;
      dn = YES;
      for (int xx = 0; xx < 2; xx++) begin
        if (m_join[b][xx] == NO) continue;
        if (m_join[b][xx] == MAY) dn = NO;
        ex = m_expl[b][xx];
        if (m_recseen[b][xx] != 0) begin
          if (m_lines[b][xx] > ex || (m_moved[b][xx] == YES && m_lines[b][xx] < ex)) le = YES;
          else if (m_moved[b][xx] == MAY && m_lines[b][xx] < ex && le == NO) le = MAY;
        end
        if (!(m_recseen[b][xx] != 0 && m_lines[b][xx] == ex)) dn = NO;
      end
      if (le == YES) set_abort(b, 1);
      else if (le == MAY) begin set_abort(b, 1); m_mayab[b] = 0; end
      else if (m_ended[b] != 0 && dn == YES) begin
        set_state(b, M_COMPL);
        m_wrunc[b] = 1;
      end
    end

    // ---- dma_buffer_overflow
    if (dma_buffer_overflow) begin
      n_ovf_pulse++;
      matched = 0;
      for (int i = ovf_q.size() - 1; i >= 0; i--)
        if (ovf_q[i].lo <= cyc && cyc <= ovf_q[i].hi) begin
          matched = 1;
          ovf_q.delete(i);
        end
      if (matched == 0) begin
        found = 0;
        for (int h = 0; h < 2; h++)
          if (found == 0 && bh_ok[h] != 0 && bh_v[h] != NO && bh_tpg[h] == 0) begin
            found = 1;
            n_loss++;
            if (m_st[bh_b[h]] == M_WRITE) set_abort(bh_b[h], 0);
            else if (m_st[bh_b[h]] == M_COMPL) m_mayab[bh_b[h]] = 1;
            actp_v[bh_x[h]] = NO;
          end
        if (found == 0 && !ovf_quiet)
          fail($sformatf("dma_buffer_overflow at cycle %0d without a cause", cyc));
      end
    end
    for (int i = ovf_q.size() - 1; i >= 0; i--)
      if (ovf_q[i].hi < cyc) begin
        if (ovf_q[i].must && !ovf_quiet)
          fail($sformatf("dma_buffer_overflow expected in cycles %0d..%0d", ovf_q[i].lo,
                         ovf_q[i].hi));
        ovf_q.delete(i);
      end
    if (dma_axi_write_error) n_aerr_pulse++;

    // ---- buffer ownership, completion events
    so[0] = buf0_state;
    so[1] = buf1_state;
    n01 = 0;
    n10 = 0;
    for (b = 0; b < 2; b++) begin
      if (so[b] == 2'b11) fail($sformatf("buffer %0d ownership 11 at cycle %0d", b, cyc));
      if (so[b] != p_so[b]) begin
        if (p_so[b] == 2'b00 && so[b] == 2'b01) begin
          if (m_st[b] == M_COMPL) check_valid_buffer(b);
          else fail($sformatf("buffer %0d released as valid at cycle %0d; model state %0d",
                              b, cyc, m_st[b]));
          set_state(b, M_SWV);
          m_out[b] = M_SWV;
          n_swv++;
          n01++;
        end else if (p_so[b] == 2'b00 && so[b] == 2'b10) begin
          if (swe_ok(b) == 0)
            fail($sformatf("buffer %0d released as error at cycle %0d; model state %0d, policy %0d",
                           b, cyc, m_st[b], m_policy[b]));
          set_state(b, M_SWE);
          m_out[b] = M_SWE;
          n_swe++;
          n10++;
          n_chk++;
        end else if (so[b] == 2'b00) begin
          if (cyc - m_relcyc[b] > 3)
            fail($sformatf("buffer %0d left software ownership at cycle %0d without release",
                           b, cyc));
        end else fail($sformatf("buffer %0d ownership %0b -> %0b", b, p_so[b], so[b]));
      end
      p_so[b] = so[b];
    end
    // One done pulse per cycle of transitions (two buffers in one cycle give one pulse).
    if (n01 > 0) begin
      if (a_open_ok != 0) a_open_ok = int'(dma_buf_done_ok);
      else if (!dma_buf_done_ok) fail($sformatf("no dma_buf_done_ok with valid release, cycle %0d",
                                                cyc));
    end else begin
      if (a_open_ok != 0)
        fail($sformatf("dma_buf_done_ok without valid release, cycle %0d", cyc - 1));
      a_open_ok = int'(dma_buf_done_ok);
    end
    if (n10 > 0) begin
      if (a_open_err != 0) a_open_err = int'(dma_buf_done_err);
      else if (!dma_buf_done_err)
        fail($sformatf("no dma_buf_done_err with error release, cycle %0d", cyc));
    end else begin
      if (a_open_err != 0) fail($sformatf("dma_buf_done_err without error release, cycle %0d",
                                          cyc - 1));
      a_open_err = int'(dma_buf_done_err);
    end

    // ---- dma_busy
    must_busy = 0;
    settled = 1;
    for (b = 0; b < 2; b++) begin
      if ((m_st[b] == M_WRITE && cyc - m_claimcyc[b] >= 2) ||
          (m_st[b] == M_COMPL && m_mayab[b] == 0 && m_maysr[b] == 0)) must_busy = 1;
      if (!(m_st[b] == M_FREE || m_st[b] == M_SWV || m_st[b] == M_SWE) || cyc - m_stcyc[b] < 3)
        settled = 0;
    end
    if (must_busy != 0 && !dma_busy) fail($sformatf("dma_busy low at cycle %0d", cyc));
    if (settled != 0 && dma_busy) fail($sformatf("dma_busy high at cycle %0d", cyc));
    busy0 = dma_busy ? 0 : busy0 + 1;
    if (busy0 >= 3)
      for (b = 0; b < 2; b++)
        if ((m_st[b] == M_ABORT || m_st[b] == M_COMPL) && so[b] == 2'b00 &&
            cyc - m_stcyc[b] >= 3)
          resolve_free(b);

    // ---- active_buffer, last valid line at the claim
    if (act_cyc >= 0 && cyc - act_cyc >= 2 && int'(active_buffer) != act_exp)
      fail($sformatf("active_buffer %0b at cycle %0d, expected %0d", active_buffer, cyc, act_exp));
    if (act_cyc < 0 && active_buffer)
      fail($sformatf("active_buffer 1 at cycle %0d before any claim", cyc));
    for (b = 0; b < 2; b++)
      if (m_ever[b] != 0 && cyc == m_claimcyc[b] + 2 && m_st[b] == M_WRITE &&
          ((b == 0) ? buf0_last_valid_line : buf1_last_valid_line) != 16'hFFFF)
        fail($sformatf("buffer %0d last valid line not 0xFFFF after the claim", b));

    // ---- software release, registered capture events
    for (b = 0; b < 2; b++)
      if (release_buf[b]) begin
        m_relcyc[b] = cyc;
        if (m_st[b] == M_SWV || m_st[b] == M_SWE) set_state(b, M_FREE);
      end
    q_cs   = (sr != 0) ? 2'b00 : capture_start;
    q_fs   = (sr != 0) ? 0 : int'(ev_fs);
    q_evc  = int'(ev_vc);
    q_lead = int'(lead_vc);
    if (sr != 0) last_buf = 1;
  endtask

  always @(posedge sys_clk) if (sys_run) mon_sys();

  // frame_id_bufN of the claimed buffer is written in the cycle after dma_buf_claim.
  always @(negedge sys_clk) begin
    if (fidw_pend) begin
      if (fidw_idx == 0) frame_id_buf0 = fidw_val;
      else frame_id_buf1 = fidw_val;
      fidw_pend = 1'b0;
    end
  end

  // --------------------------------------------------------------------------------------------
  // AXI4 subordinate with a byte memory (mm_aclk side)
  // --------------------------------------------------------------------------------------------
  typedef struct {
    int          id;
    longint      addr;
    int          nbeat;
    int          got;
    int          nbytes;
    int          b;
    int          stream;
    int          x;
    int          line;
    int          eol;
    int          tready;
    logic [1:0]  resp;
  } aw_t;
  typedef struct {
    logic [DW-1:0] d;
    logic [NB-1:0] s;
    bit            last;
  } wb_t;
  aw_t  aw_q   [$];      // accepted addresses still waiting for data
  wb_t  w_pend [$];      // data beats not yet matched with an address
  aw_t  bdone  [$];      // complete writes waiting for their response
  aw_t  b_cur;
  bit   b_cur_v = 1'b0;
  int   aw_beats_total, w_beats_total;

  // Subordinate behaviour, set by the tests.
  int   aw_p = 100;          // percent of cycles with AWREADY
  int   w_p  = 100;
  int   aw_stall_p = 0;      // percent chance to start a stall
  int   w_stall_p  = 0;
  int   stall_max  = 1;
  int   aw_stall, w_stall;
  bit   aw_block = 1'b0;     // AWREADY held low
  bit   w_block  = 1'b0;
  bit   aw_after_w = 1'b0;   // AWREADY only once the data of that write has started
  int   b_lat_min = 1;
  int   b_lat_max = 4;
  bit   b_reorder = 1'b0;    // responses of different identifiers out of order
  bit   b_hold    = 1'b0;    // no response is given
  bit   err_all   = 1'b0;    // every response an error, SLVERR and DECERR in turn
  bit   err_tog   = 1'b0;
  int   err_n     = 0;       // number of error responses still to give in the window (-1: all)
  longint err_lo  = 0;
  longint err_hi  = 0;
  int   err_id    = -1;      // identifier filter, -1 any
  logic [1:0] err_resp = 2'b10;
  logic aw_rdy_n, w_rdy_n;
  int   aw_log [$];          // stream of each accepted address while aw_log_en is set
  int   last_aw_mcyc = 0;
  bit   aw_log_en = 1'b0;

  // Previous values for the stability checks.
  logic          p_awv, p_awr, p_wv, p_wr;
  logic [IDW-1:0] p_awid;
  logic [AWW-1:0] p_awaddr;
  logic [7:0]    p_awlen;
  logic [3:0]    p_awqos;
  logic [2:0]    p_awprot;
  logic [DW-1:0] p_wdata;
  logic [NB-1:0] p_wstrb;
  logic          p_wlast;

  function automatic void slave_reset();
    aw_q.delete();
    w_pend.delete();
    bdone.delete();
    b_cur_v = 1'b0;
    aw_beats_total = 0;
    w_beats_total = 0;
    aw_stall = 0;
    w_stall = 0;
    p_awv = 1'b0;  p_awr = 1'b0;  p_wv = 1'b0;  p_wr = 1'b0;
    m_nout[0] = 0;  m_nout[1] = 0;
  endfunction

  function automatic logic [1:0] resp_for(input aw_t e);
    if (err_all) begin
      err_tog = ~err_tog;
      return err_tog ? 2'b10 : 2'b11;
    end
    if (err_n != 0 && e.addr >= err_lo && e.addr <= err_hi && (err_id < 0 || err_id == e.id))
      begin
      if (err_n > 0) err_n--;
      return err_resp;
    end
    return 2'b00;
  endfunction

  function automatic void mem_write(input int b, input longint a, input logic [7:0] v);
    int unsigned ua = int'(a);
    if (!exp_has(b, ua))
      fail($sformatf("unexpected write to %08h (buffer %0d) at mm cycle %0d", ua, b, mcyc));
    else if (exp_val(b, ua) !== v)
      fail($sformatf("byte %08h written %02h, expected %02h", ua, v, exp_val(b, ua)));
    if (wm_has(b, ua)) fail($sformatf("byte %08h written twice", ua));
    wm_set(b, ua);
    n_wbytes++;
  endfunction

  function automatic void aw_accept();
    aw_t e;
    int  b64, lim;
    longint lo, hi;
    e.id = int'(m_axi4_awid & 2'b11);
    e.addr = longint'(m_axi4_awaddr);
    e.nbeat = int'(m_axi4_awlen) + 1;
    e.got = 0;
    e.nbytes = 0;
    e.b = e.id & 1;
    e.stream = (e.id >> 1) & 1;
    e.x = 0;
    e.line = 0;
    e.eol = 0;
    e.tready = 0;
    e.resp = 2'b00;
    if (IDW > 2 && (m_axi4_awid >> 2) != 0) fail("AWID upper bits not zero");
    if (m_st[e.b] == M_FREE || m_st[e.b] == M_SWV || m_st[e.b] == M_SWE)
      fail($sformatf("write to %08h for buffer %0d not held by hardware", e.addr, e.b));
    if (int'(m_axi4_awqos) != m_qos[e.b] || int'(m_axi4_awprot) != m_prot[e.b])
      fail($sformatf("AWQOS/AWPROT %0h/%0h, sampled %0h/%0h", m_axi4_awqos, m_axi4_awprot,
                     m_qos[e.b], m_prot[e.b]));
    if (e.addr[2:0] != 3'b000) fail($sformatf("AWADDR %08h not 8-byte aligned", e.addr));
    if (DW == 32 && (e.nbeat % 2) != 0) fail("odd AWLEN + 1 with a 32-bit bus");
    b64 = (DW == 64) ? e.nbeat : e.nbeat / 2;
    if ((e.addr & 4095) + longint'(e.nbeat * NB) > 4096)
      fail($sformatf("burst at %08h of %0d beats crosses 4 KB", e.addr, e.nbeat));
    lim = (e.stream == 0) ? lim64_raw(e.b) : lim64_meta(e.b);
    if (b64 > lim) fail($sformatf("burst of %0d beats above the limit %0d", b64, lim));
    if (e.stream == 0) begin
      e.x = (e.addr >= m_base[e.b] + m_size[e.b]) ? 1 : 0;
      lo = m_base[e.b] + longint'(e.x) * m_size[e.b];
      hi = lo + m_size[e.b];
    end else begin
      e.x = (e.addr >= meta_base(e.b, 0) + m_meta[e.b]) ? 1 : 0;
      lo = meta_base(e.b, e.x);
      hi = lo + m_meta[e.b];
    end
    if (e.addr < lo || e.addr + longint'(b64 * 8) > hi)
      fail($sformatf("burst at %08h outside its region %08h..%08h", e.addr, lo, hi));
    n_aw++;
    m_nout[e.b]++;
    if (n_aw - n_bh > MOUT) fail($sformatf("%0d writes outstanding", n_aw - n_bh));
    if (n_aw - n_bh > max_out) max_out = n_aw - n_bh;
    aw_beats_total += e.nbeat;
    if (aw_log_en) aw_log.push_back(e.stream);
    last_aw_mcyc = mcyc;
    aw_q.push_back(e);
  endfunction

  function automatic void burst_done(input aw_t e);
    int b64, found;
    longint ea;
    if (e.stream == 0) begin
      found = 0;
      if (e.b == 0) begin
        for (int i = 0; i < lq0.size() && found == 0; i++)
          if (lq0[i].x == e.x && lq0[i].start + lq0[i].wr == e.addr && lq0[i].wr < lq0[i].len)
            begin
            found = 1;
            lq0[i].wr += e.nbytes;
            e.line = lq0[i].lidx;
            e.eol = int'(lq0[i].done && lq0[i].wr == lq0[i].len);
          end
      end else begin
        for (int i = 0; i < lq1.size() && found == 0; i++)
          if (lq1[i].x == e.x && lq1[i].start + lq1[i].wr == e.addr && lq1[i].wr < lq1[i].len)
            begin
            found = 1;
            lq1[i].wr += e.nbytes;
            e.line = lq1[i].lidx;
            e.eol = int'(lq1[i].done && lq1[i].wr == lq1[i].len);
          end
      end
      if (found == 0) fail($sformatf("RAW burst at %08h does not continue a line", e.addr));
      else if (e.eol == 0) begin
        b64 = (DW == 64) ? e.nbeat : e.nbeat / 2;
        ea = e.addr + longint'(b64 * 8);
        if (b64 != lim64_raw(e.b) && (ea & 4095) != 0)
          fail($sformatf("RAW burst at %08h closed after %0d beats inside a line", e.addr, b64));
        else n_chk++;
      end
    end
    e.tready = mcyc + $urandom_range(b_lat_max, b_lat_min);
    bdone.push_back(e);
  endfunction

  function automatic void try_match();
    wb_t wb;
    longint a0;
    aw_t e;
    while (aw_q.size() > 0 && w_pend.size() > 0) begin
      wb = w_pend.pop_front();
      a0 = aw_q[0].addr + longint'(aw_q[0].got * NB);
      for (int i = 0; i < NB; i++)
        if (wb.s[i]) begin
          mem_write(aw_q[0].b, a0 + i, wb.d[8*i +: 8]);
          aw_q[0].nbytes++;
        end
      aw_q[0].got++;
      if (wb.last != (aw_q[0].got == aw_q[0].nbeat))
        fail($sformatf("WLAST %0b at beat %0d of %0d", wb.last, aw_q[0].got, aw_q[0].nbeat));
      if (aw_q[0].got == aw_q[0].nbeat) begin
        e = aw_q.pop_front();
        burst_done(e);
      end
    end
  endfunction

  function automatic void b_accept();
    aw_t e = b_cur;
    b_cur_v = 1'b0;
    n_bh++;
    m_nout[e.b]--;
    if (e.resp != 2'b00) m_lvlstop[e.b] = 1;
    else if (m_lvlstop[e.b] == 0 && e.stream == 0 && e.eol != 0) m_lvl[e.b][e.x] = e.line;
    if (e.resp != 2'b00) begin
      er_t r;
      r.t = $realtime;
      r.id = e.id;
      r.addr = e.addr;
      r.resp = int'(e.resp);
      er_q.push_back(r);
      n_err_resp++;
      if (m_st[e.b] == M_WRITE || m_st[e.b] == M_COMPL) set_abort(e.b, 1);
    end
  endfunction

  task automatic mon_mm();
    wb_t wb;
    int  cand [$];
    int  pick, head;
    mcyc++;
    if ($isunknown({m_axi4_awvalid, m_axi4_wvalid, m_axi4_bready}) ||
        (m_axi4_awvalid && $isunknown({m_axi4_awid, m_axi4_awaddr, m_axi4_awlen, m_axi4_awsize,
                                       m_axi4_awburst, m_axi4_awlock, m_axi4_awcache,
                                       m_axi4_awprot, m_axi4_awqos})) ||
        (m_axi4_wvalid && $isunknown({m_axi4_wdata, m_axi4_wstrb, m_axi4_wlast})))
      fail($sformatf("X or Z on an AXI output at mm cycle %0d", mcyc));
    if (p_awv && !p_awr && (!m_axi4_awvalid || m_axi4_awid != p_awid ||
        m_axi4_awaddr != p_awaddr || m_axi4_awlen != p_awlen || m_axi4_awqos != p_awqos ||
        m_axi4_awprot != p_awprot))
      fail($sformatf("AW changed before AWREADY at mm cycle %0d", mcyc));
    if (p_wv && !p_wr && (!m_axi4_wvalid || m_axi4_wdata != p_wdata ||
        m_axi4_wstrb != p_wstrb || m_axi4_wlast != p_wlast))
      fail($sformatf("W changed before WREADY at mm cycle %0d", mcyc));
    if (m_axi4_awvalid) begin
      if (m_axi4_awsize != ((DW == 64) ? 3'b011 : 3'b010)) fail("AWSIZE");
      if (m_axi4_awburst != 2'b01 || m_axi4_awlock != 1'b0 || m_axi4_awcache != 4'b0011)
        fail("AWBURST, AWLOCK or AWCACHE");
    end
    if (!m_axi4_bready) fail("BREADY low");
    if (m_axi4_awvalid && m_axi4_awready) aw_accept();
    if (m_axi4_wvalid && m_axi4_wready) begin
      wb.d = m_axi4_wdata;
      wb.s = m_axi4_wstrb;
      wb.last = m_axi4_wlast;
      w_pend.push_back(wb);
      w_beats_total++;
    end
    try_match();
    if (m_axi4_bvalid && m_axi4_bready && b_cur_v) b_accept();
    p_awv = m_axi4_awvalid;  p_awr = m_axi4_awready;  p_wv = m_axi4_wvalid;
    p_wr = m_axi4_wready;  p_awid = m_axi4_awid;  p_awaddr = m_axi4_awaddr;
    p_awlen = m_axi4_awlen;  p_awqos = m_axi4_awqos;  p_awprot = m_axi4_awprot;
    p_wdata = m_axi4_wdata;  p_wstrb = m_axi4_wstrb;  p_wlast = m_axi4_wlast;

    // Next response: the oldest write of each identifier is its only candidate.
    if (!b_cur_v && !b_hold) begin
      for (int i = 0; i < bdone.size(); i++) begin
        head = 1;
        for (int j = 0; j < i; j++) if (bdone[j].id == bdone[i].id) head = 0;
        if (head != 0 && mcyc >= bdone[i].tready && (b_reorder || i == 0)) cand.push_back(i);
      end
      if (cand.size() > 0) begin
        pick = cand[$urandom_range(cand.size() - 1, 0)];
        b_cur = bdone[pick];
        bdone.delete(pick);
        b_cur.resp = resp_for(b_cur);
        b_cur_v = 1'b1;
      end
    end
    // Next AWREADY and WREADY.
    if (aw_stall > 0) aw_stall--;
    else if ($urandom_range(99, 0) < aw_stall_p) aw_stall = $urandom_range(stall_max, 1);
    if (w_stall > 0) w_stall--;
    else if ($urandom_range(99, 0) < w_stall_p) w_stall = $urandom_range(stall_max, 1);
    aw_rdy_n = !aw_block && aw_stall == 0 && $urandom_range(99, 0) < aw_p;
    if (aw_after_w)
      aw_rdy_n = aw_rdy_n && (w_beats_total > aw_beats_total ||
                              (m_axi4_wvalid && w_beats_total == aw_beats_total));
    w_rdy_n = !w_block && w_stall == 0 && $urandom_range(99, 0) < w_p;
  endtask

  always @(posedge mm_aclk) if (mm_run) mon_mm();

  always @(negedge mm_aclk) begin
    if (!mm_resetn) begin
      m_axi4_awready = 1'b0;
      m_axi4_wready  = 1'b0;
      m_axi4_bvalid  = 1'b0;
      m_axi4_bid     = '0;
      m_axi4_bresp   = 2'b00;
    end else begin
      m_axi4_awready = aw_rdy_n;
      m_axi4_wready  = w_rdy_n;
      m_axi4_bvalid  = b_cur_v;
      m_axi4_bid     = IDW'(b_cur.id);
      m_axi4_bresp   = b_cur.resp;
    end
  end

  // --------------------------------------------------------------------------------------------
  // Resets: the model follows resetn and the subordinate follows mm_resetn. Registered outputs
  // must reach their reset values as soon as the reset is asserted.
  // --------------------------------------------------------------------------------------------
  always @(negedge resetn) begin
    sys_run = 1'b0;
    model_reset();
    #0.1;
    if (dma_buf_claim || dma_drop || dma_busy || active_buffer || buf0_state != 2'b00 ||
        buf1_state != 2'b00 || buf0_last_valid_line != 16'h0 || buf1_last_valid_line != 16'h0 ||
        axi_fault_awid != 8'h0 || axi_fault_resp != 2'b00 || axi_fault_awaddr != 32'h0 ||
        bresp_slverr_count != 16'h0 || bresp_decerr_count != 16'h0 || dma_buf_done_ok ||
        dma_buf_done_err || dma_axi_write_error || dma_buffer_overflow)
      fail("sys_clk outputs not at their reset values while resetn is low");
  end

  always @(negedge mm_resetn) begin
    mm_run = 1'b0;
    slave_reset();
    #0.1;
    if (m_axi4_awvalid !== 1'b0 || m_axi4_wvalid !== 1'b0)
      fail("AWVALID or WVALID not low while mm_resetn is low");
  end

  // --------------------------------------------------------------------------------------------
  // Stimulus: Format Bridge beats and the metadata stream, each from a queue (one word or beat
  // presented at a time, held until taken)
  // --------------------------------------------------------------------------------------------
  typedef struct {
    logic [63:0] d;
    logic [7:0]  be;
    bit          sof;
    bit          sol;
    bit          eol;
    int          x;
    int          gap;
  } rb_t;
  typedef struct {
    logic [31:0] d;
    logic [3:0]  be;
    bit          last;
    bit          kind;
    int          x;
    bit          sof;
    int          gap;
  } mw_t;
  rb_t  raw_q [$];
  rb_t  raw_cur;
  bit   raw_cur_v = 1'b0;
  bit   raw_acc   = 1'b0;
  int   raw_gapc  = 0;
  mw_t  meta_q [$];
  mw_t  meta_cur;
  bit   meta_cur_v = 1'b0;
  bit   meta_acc   = 1'b0;
  int   meta_gapc  = 0;
  bit   meta_drop  = 1'b0;   // drop arriving payload words while a word waits (upstream loss)
  int   n_mdrop    = 0;
  bit   vc_hi      = 1'b0;   // drive bit 1 of the channel fields at random
  bit   err_rand   = 1'b0;   // drive fbd_err at random
  int   src_gap    = 0;
  int   line_gap   = 2;
  int   n_beats    = 0;
  int   n_mwords   = 0;

  always @(posedge sys_clk) begin
    raw_acc  = fbd_valid && fbd_ready;
    meta_acc = meta_valid && meta_ready;
    if (sys_run) begin
      if (fbd_valid && fbd_ready) n_beats++;
      if (meta_valid && meta_ready) n_mwords++;
    end
  end

  always @(negedge sys_clk) begin
    if (raw_cur_v && raw_acc) raw_cur_v = 1'b0;
    raw_acc = 1'b0;
    if (!raw_cur_v) begin
      if (raw_gapc > 0) raw_gapc--;
      else if (raw_q.size() > 0) begin
        if (raw_q[0].gap > 0) begin
          raw_gapc = raw_q[0].gap - 1;
          raw_q[0].gap = 0;
        end else begin
          raw_cur = raw_q.pop_front();
          raw_cur_v = 1'b1;
          fbd_data    = raw_cur.d;
          fbd_byte_en = raw_cur.be;
          fbd_sof     = raw_cur.sof;
          fbd_sol     = raw_cur.sol;
          fbd_eol     = raw_cur.eol;
          fbd_vc_tag  = {vc_hi ? 1'($urandom_range(1, 0)) : 1'b0, 1'(raw_cur.x)};
          fbd_err     = err_rand ? 1'($urandom_range(1, 0)) : 1'b0;
        end
      end
    end
    fbd_valid = raw_cur_v;
    if (!raw_cur_v) begin
      fbd_sof = 1'b0;  fbd_sol = 1'b0;  fbd_eol = 1'b0;
      if (err_rand) fbd_err = 1'($urandom_range(1, 0));
    end
  end

  always @(negedge sys_clk) begin
    int j;
    if (meta_cur_v && meta_acc) meta_cur_v = 1'b0;
    meta_acc = 1'b0;
    if (meta_cur_v && meta_drop && meta_q.size() > 0 && meta_q[0].kind &&
        $urandom_range(1, 0) == 1) begin
      if (meta_q[0].sof) begin
        j = 1;
        while (j < meta_q.size() && !(meta_q[j].kind && meta_q[j].x == meta_q[0].x)) j++;
        if (j < meta_q.size()) meta_q[j].sof = 1'b1;
      end
      void'(meta_q.pop_front());
      n_mdrop++;
    end
    if (!meta_cur_v) begin
      if (meta_gapc > 0) meta_gapc--;
      else if (meta_q.size() > 0) begin
        if (meta_q[0].gap > 0) begin
          meta_gapc = meta_q[0].gap - 1;
          meta_q[0].gap = 0;
        end else begin
          meta_cur = meta_q.pop_front();
          meta_cur_v = 1'b1;
          meta_data    = meta_cur.d;
          meta_byte_en = meta_cur.be;
          meta_last    = meta_cur.last;
          meta_kind    = meta_cur.kind;
          meta_sof     = meta_cur.sof;
          meta_vc      = {vc_hi ? 1'($urandom_range(1, 0)) : 1'b0, 1'(meta_cur.x)};
        end
      end
    end
    meta_valid = meta_cur_v;
    if (!meta_cur_v) begin
      meta_last = 1'b0;  meta_sof = 1'b0;
    end
  end

  // Software: releases a buffer some cycles after it reaches software ownership.
  bit         sw_auto = 1'b0;
  int         sw_min  = 4;
  int         sw_max  = 40;
  int         rel_at [2] = '{-1, -1};
  logic [1:0] rel_req = 2'b00;

  always @(posedge sys_clk) begin
    if (sys_run && sw_auto) begin
      for (int b = 0; b < 2; b++) begin
        if (m_st[b] == M_SWV || m_st[b] == M_SWE) begin
          if (rel_at[b] < 0) rel_at[b] = cyc + $urandom_range(sw_max, sw_min);
          else if (cyc >= rel_at[b]) begin rel_req[b] = 1'b1; rel_at[b] = -1; end
        end else rel_at[b] = -1;
      end
    end
  end

  always @(negedge sys_clk) begin
    release_buf = rel_req;
    rel_req = 2'b00;
  end

  // --------------------------------------------------------------------------------------------
  // Stimulus helpers
  // --------------------------------------------------------------------------------------------
  int fid_ctr = 0;
  int slot_busy [2] = '{0, 0};
  int slot_fid  [2] = '{0, 0};
  int cur_lead  = 0;

  task automatic tick(input int n = 1);
    repeat (n) @(negedge sys_clk);
  endtask

  task automatic csr_defaults();
    dma_enable       = 1'b1;
    dma_start        = 1'b1;
    dma_abort        = 1'b0;
    err_frame_policy = 1'b1;
    dma_buffer0_addr = 32'h1000_0000;
    dma_buffer1_addr = 32'h1010_0000;
    dma_buffer_size  = 32'h0002_0000;
    meta_area_size   = 32'h0000_0400;
    bytes_per_line   = 16'd256;
    line_padding     = 8'd0;
    awqos            = 4'h0;
    max_burst_len    = 8'hFF;
    awprot           = 3'b000;
    active_sel       = 1'b0;
    tpg_height_shadow = 16'd0;
  endtask

  task automatic drive_idle();
    soft_reset = 1'b0;
    fbd_data = '0;  fbd_byte_en = '0;  fbd_valid = 1'b0;  fbd_sof = 1'b0;  fbd_sol = 1'b0;
    fbd_eol = 1'b0;  fbd_vc_tag = 2'b00;  fbd_err = 1'b0;
    capture_start = 2'b00;  capture_end = 2'b00;  capture_dropped = 2'b00;
    ev_fs = 1'b0;  ev_vc = 2'b00;  lead_vc = 1'b0;  ctx_slot = 2'b00;  ctx_orphan = 2'b00;
    frame_id_buf0 = '0;  frame_id_buf1 = '0;
    meta_data = '0;  meta_byte_en = '0;  meta_valid = 1'b0;  meta_last = 1'b0;
    meta_kind = 1'b0;  meta_vc = 2'b00;  meta_sof = 1'b0;
    dma_abort_req = 1'b0;
    release_buf = 2'b00;
    m_axi4_awready = 1'b0;  m_axi4_wready = 1'b0;  m_axi4_bvalid = 1'b0;
    m_axi4_bid = '0;  m_axi4_bresp = 2'b00;
    for (int i = 0; i < NF; i++) begin f_push[i] = 1'b0; f_pop[i] = 1'b0; f_din[i] = '0; end
    csr_defaults();
  endtask

  // Source pacing so that the sensor path never outruns the AXI side (slow: percent factor).
  task automatic set_rates(input int slow);
    real tm, per;
    tm = (PARA_ASYNC_MM_CLK == 32'd0) ? t_sys : t_mm;
    per = tm * ((DW == 64) ? 1.0 : 2.0) / t_sys;
    src_gap = int'($ceil(1.6 * per * slow / 100.0)) - 1;
    if (src_gap < 0) src_gap = 0;
    line_gap = int'($ceil(8.0 * tm / t_sys * slow / 100.0)) + 2;
  endtask

  task automatic slave_defaults();
    aw_p = 100;  w_p = 100;  aw_stall_p = 0;  w_stall_p = 0;  stall_max = 1;
    aw_block = 1'b0;  w_block = 1'b0;  aw_after_w = 1'b0;
    b_lat_min = 1;  b_lat_max = 4;  b_reorder = 1'b0;  b_hold = 1'b0;
    err_all = 1'b0;  err_n = 0;  err_id = -1;
  endtask

  // One line of nbytes on channel x; gap0 idle cycles before it, src_gap between beats.
  task automatic push_line(input int x, input int nbytes, input bit sof, input int gap0);
    rb_t r;
    int  nb = (nbytes + 7) / 8;
    int  rem;
    for (int k = 0; k < nb; k++) begin
      rem = nbytes - 8 * k;
      r.d   = {$urandom, $urandom};
      r.be  = (rem >= 8) ? 8'hFF : 8'((1 << rem) - 1);
      r.sof = sof && (k == 0);
      r.sol = (k == 0);
      r.eol = (k == nb - 1);
      r.x   = x;
      r.gap = (k == 0) ? gap0 : src_gap;
      raw_q.push_back(r);
    end
  endtask

  task automatic push_frame(input int x, input int nlines, input int nbytes, input bit sof);
    for (int l = 0; l < nlines; l++) push_line(x, nbytes, sof && l == 0, line_gap);
  endtask

  function automatic void mw_add(ref mw_t q[$], input logic [31:0] d, input logic [3:0] be,
                                 input bit last, input bit kind, input int x, input bit sof);
    mw_t w;
    w.d = d;  w.be = be;  w.last = last;  w.kind = kind;  w.x = x;  w.sof = sof;  w.gap = 0;
    q.push_back(w);
  endfunction

  // Record words: version and channel, frame_id, timestamp, record id, line_count, fields.
  function automatic void rec_words(ref mw_t q[$], input int x, input int fid, input int lc,
                                    input int nw);
    logic [31:0] d;
    for (int i = 0; i < nw; i++) begin
      case (i)
        0: d = 32'h0000_0001 | (32'(x) << 8);
        1: d = 32'(fid & 16'h3FFF);
        5: d = {16'(lc), 16'($urandom)};
        default: d = $urandom;
      endcase
      mw_add(q, d, 4'hF, i == nw - 1, 1'b0, x, 1'b0);
    end
  endfunction

  function automatic void pay_words(ref mw_t q[$], input int x, input int nw, input bit sof,
                                    input logic [3:0] last_be);
    for (int i = 0; i < nw; i++)
      mw_add(q, $urandom, (i == nw - 1) ? last_be : 4'hF, i == nw - 1, 1'b1, x, sof && i == 0);
  endfunction

  task automatic push_rec(input int x, input int fid, input int lc);
    mw_t q[$];
    rec_words(q, x, fid, lc, 9);
    foreach (q[i]) meta_q.push_back(q[i]);
  endtask

  task automatic push_pay(input int x, input int nw, input bit sof, input logic [3:0] last_be);
    mw_t q[$];
    pay_words(q, x, nw, sof, last_be);
    foreach (q[i]) meta_q.push_back(q[i]);
  endtask

  // A record interleaved at word granularity with one embedded line.
  task automatic push_rec_mix(input int x, input int fid, input int lc, input int px,
                              input int pnw, input bit psof, input logic [3:0] plast_be);
    mw_t r[$];
    mw_t p[$];
    rec_words(r, x, fid, lc, 9);
    pay_words(p, px, pnw, psof, plast_be);
    while (r.size() > 0 || p.size() > 0) begin
      if (p.size() == 0 || (r.size() > 0 && $urandom_range(1, 0) == 1))
        meta_q.push_back(r.pop_front());
      else meta_q.push_back(p.pop_front());
    end
  endtask

  task automatic raw_wait();
    wait (raw_q.size() == 0 && !raw_cur_v);
    tick(1);
  endtask

  task automatic meta_wait();
    wait (meta_q.size() == 0 && !meta_cur_v);
    tick(1);
  endtask

  // Frame Context & Metadata Capture events. ctx_slot and ctx_orphan of the channel change in
  // the cycle after its ev_fs.
  task automatic ev_lead(input int x, output int s);
    int n0 = n_cap_res;
    s = (slot_busy[0] == 0) ? 0 : ((slot_busy[1] == 0) ? 1 : -1);
    if (s < 0) begin
      fail("testbench: no free capture slot");
      return;
    end
    fid_ctr = (fid_ctr % 16383) + 1;
    slot_fid[s] = fid_ctr;
    slot_busy[s] = 1;
    cur_lead = x;
    @(negedge sys_clk);
    last_alloc_fid = 14'(fid_ctr);
    ev_fs = 1'b1;
    ev_vc = {vc_hi ? 1'($urandom_range(1, 0)) : 1'b0, 1'(x)};
    lead_vc = 1'(x);
    capture_start = 2'(1 << s);
    @(negedge sys_clk);
    ev_fs = 1'b0;
    capture_start = 2'b00;
    ctx_slot[x] = 1'(s);
    ctx_orphan[x] = 1'b0;
    if (sys_run) wait (n_cap_res > n0);
  endtask

  task automatic ev_lag(input int x, input int s);
    @(negedge sys_clk);
    ev_fs = 1'b1;
    ev_vc = {vc_hi ? 1'($urandom_range(1, 0)) : 1'b0, 1'(x)};
    lead_vc = 1'(1 - x);
    @(negedge sys_clk);
    ev_fs = 1'b0;
    ctx_slot[x] = 1'(s);
    ctx_orphan[x] = 1'b0;
  endtask

  // ctx_slot of an orphan channel is left naming slot s; the DMA must ignore it.
  task automatic ev_orphan(input int x, input int s);
    @(negedge sys_clk);
    ev_fs = 1'b1;
    ev_vc = {vc_hi ? 1'($urandom_range(1, 0)) : 1'b0, 1'(x)};
    lead_vc = 1'(1 - x);
    @(negedge sys_clk);
    ev_fs = 1'b0;
    ctx_slot[x] = 1'(s);
    ctx_orphan[x] = 1'b1;
  endtask

  task automatic ev_end(input int s, input bit dropped);
    @(negedge sys_clk);
    capture_end = 2'(1 << s);
    capture_dropped = dropped ? 2'(1 << s) : 2'b00;
    @(negedge sys_clk);
    capture_end = 2'b00;
    capture_dropped = 2'b00;
    slot_busy[s] = 0;
  endtask

  task automatic pulse_abort_req();
    @(negedge sys_clk);
    dma_abort_req = 1'b1;
    @(negedge sys_clk);
    dma_abort_req = 1'b0;
  endtask

  // soft_reset for n cycles; upstream blocks are reset by it too, so their streams stop.
  task automatic do_soft_reset(input int n);
    @(negedge sys_clk);
    raw_q.delete();
    meta_q.delete();
    raw_cur_v = 1'b0;   fbd_valid = 1'b0;   fbd_sof = 1'b0;  fbd_sol = 1'b0;  fbd_eol = 1'b0;
    meta_cur_v = 1'b0;  meta_valid = 1'b0;  meta_last = 1'b0;  meta_sof = 1'b0;
    raw_gapc = 0;  meta_gapc = 0;
    slot_busy[0] = 0;  slot_busy[1] = 0;
    soft_reset = 1'b1;
    repeat (n - 1) @(negedge sys_clk);
    @(negedge sys_clk);
    soft_reset = 1'b0;
  endtask

  task automatic sw_release(input logic [1:0] m);
    @(posedge sys_clk);
    rel_req = m;
    tick(2);
  endtask

  // Waits for the outcome of the last claim of buffer b (valid, error, or free after an abort).
  task automatic wait_st(input int b, input int st, input int tmo = 150000);
    int t = 0;
    if (b < 0 || b > 1) begin
      fail("testbench: wait on a capture without a buffer");
      return;
    end
    while (m_out[b] == 0 && t < tmo) begin
      @(posedge sys_clk);
      t++;
    end
    if (m_out[b] != st)
      fail($sformatf("buffer %0d: outcome %0d (state %0d), expected %0d (cycle %0d)", b,
                     m_out[b], m_st[b], st, cyc));
    tick(1);
  endtask

  // A completing buffer has all its commands queued: every expected byte is written, or the
  // writes in flight are at MAX_OUTSTANDING and no address has been taken for a while.
  task automatic wait_written(input int b, input int tmo = 400000);
    int t = 0;
    while (t < tmo && !(m_st[b] == M_COMPL &&
           (((b == 0) ? (wm0.size() == ev0.size()) : (wm1.size() == ev1.size())) ||
            (n_aw - n_bh == MOUT && mcyc - last_aw_mcyc > 200)))) begin
      @(posedge sys_clk);
      t++;
    end
    if (t >= tmo) fail($sformatf("buffer %0d: writes not issued", b));
    tick(20);
  endtask

  function automatic bit slave_idle();
    return aw_q.size() == 0 && w_pend.size() == 0 && bdone.size() == 0 && !b_cur_v &&
           !m_axi4_awvalid && !m_axi4_wvalid;
  endfunction

  function automatic bit idle_now();
    for (int b = 0; b < 2; b++)
      if (!(m_st[b] == M_FREE || m_st[b] == M_SWV || m_st[b] == M_SWE)) return 1'b0;
    return raw_q.size() == 0 && !raw_cur_v && meta_q.size() == 0 && !meta_cur_v &&
           slave_idle() && !dma_busy;
  endfunction

  task automatic wait_idle(input int tmo = 600000);
    int n = 0;
    int t = 0;
    while (n < 64 && t < tmo) begin
      @(posedge sys_clk);
      t++;
      if (idle_now()) n++;
      else n = 0;
    end
    if (n < 64)
      fail($sformatf("not idle after %0d cycles: states %0d %0d, busy %0b, slave idle %0b",
                     tmo, m_st[0], m_st[1], dma_busy, slave_idle()));
    tick(1);
  endtask

  function automatic int lvl_model(input int b);
    int mn = 65536;
    if (m_ever[b] == 0) return 0;
    for (int x = 0; x < 2; x++) begin
      if (m_join[b][x] == MAY) return -1;
      if (m_join[b][x] == YES) begin
        if (m_lvl[b][x] < 0) return 65535;
        if (m_lvl[b][x] < mn) mn = m_lvl[b][x];
      end
    end
    return (mn == 65536) ? 65535 : mn;
  endfunction

  // Error events, counters, fault trace and last valid lines once the DMA is quiet.
  task automatic check_status();
    int lo_s = 0, hi_s = 0, lo_d = 0, hi_d = 0, amb = 0, first = -1, v;
    realtime win;
    win = 12.0 * (t_sys + t_mm) * (SYNC + 2);
    if (n_aerr_pulse != n_err_resp)
      fail($sformatf("%0d dma_axi_write_error pulses for %0d error responses", n_aerr_pulse,
                     n_err_resp));
    foreach (er_q[i]) begin
      if (er_q[i].t > t_sr) begin
        if (first < 0) first = i;
        if (er_q[i].resp == 2) begin lo_s++; hi_s++; end else begin lo_d++; hi_d++; end
      end else if (er_q[i].t > t_sr - win) begin
        amb++;
        if (er_q[i].resp == 2) hi_s++; else hi_d++;
      end
    end
    if (lo_s > 65535) lo_s = 65535;
    if (hi_s > 65535) hi_s = 65535;
    if (lo_d > 65535) lo_d = 65535;
    if (hi_d > 65535) hi_d = 65535;
    if (int'(bresp_slverr_count) < lo_s || int'(bresp_slverr_count) > hi_s)
      fail($sformatf("bresp_slverr_count %0d, expected %0d..%0d", bresp_slverr_count, lo_s, hi_s));
    if (int'(bresp_decerr_count) < lo_d || int'(bresp_decerr_count) > hi_d)
      fail($sformatf("bresp_decerr_count %0d, expected %0d..%0d", bresp_decerr_count, lo_d, hi_d));
    if (amb == 0) begin
      if (first < 0) begin
        if (axi_fault_awid != 8'h0 || axi_fault_resp != 2'b00 || axi_fault_awaddr != 32'h0)
          fail("AXI fault trace not clear without a faulting response");
      end else if (int'(axi_fault_awid) != er_q[first].id ||
                   int'(axi_fault_resp) != er_q[first].resp ||
                   longint'(axi_fault_awaddr) != er_q[first].addr)
        fail($sformatf("fault trace %0h/%0h/%08h, expected %0h/%0h/%08h", axi_fault_awid,
                       axi_fault_resp, axi_fault_awaddr, er_q[first].id, er_q[first].resp,
                       er_q[first].addr));
    end
    for (int b = 0; b < 2; b++) begin
      v = lvl_model(b);
      if (v >= 0 && int'((b == 0) ? buf0_last_valid_line : buf1_last_valid_line) != v)
        fail($sformatf("buffer %0d last valid line %0d, expected %0d", b,
                       (b == 0) ? buf0_last_valid_line : buf1_last_valid_line, v));
    end
    n_chk++;
  endtask

  task automatic quiet_check();
    wait_idle();
    check_status();
  endtask

  // Single-exposure capture: Frame Start, embedded line, pixel lines, record, end of capture.
  // With use_tpg the capture comes from the test pattern path (height nlines).
  bit use_tpg = 1'b0;

  task automatic cap_single(input int x, input int nlines, input int nbytes, input int npay,
                            input int lc, output int b, output int s);
    int fid;
    if (use_tpg) begin
      active_sel = 1'b1;
      tpg_height_shadow = 16'(nlines);
    end
    ev_lead(x, s);
    b = cap_res[$];
    fid = slot_fid[s];
    tick(1);                                // inputs change only after a falling edge
    if (use_tpg) active_sel = 1'b0;
    tick(1);
    if (npay > 0) push_pay(x, npay, 1'b1, 4'hF);
    push_frame(x, nlines, nbytes, 1'b1);
    raw_wait();
    push_rec(x, fid, lc);
    meta_wait();
    ev_end(s, 1'b0);
  endtask

  // Staggered HDR capture: lead channel 0, lag channel 1, lines interleaved.
  task automatic cap_hdr(input int n0, input int n1, input int nb0, input int nb1,
                         input int npay, output int b, output int s);
    int fid;
    ev_lead(0, s);
    b = cap_res[$];
    fid = slot_fid[s];
    tick(2);
    if (npay > 0) push_pay(0, npay, 1'b1, 4'hF);
    if (n0 > 0) push_line(0, nb0, 1'b1, line_gap);
    ev_lag(1, s);
    tick(1);
    if (npay > 0) push_pay(1, npay + 1, 1'b1, 4'h7);
    for (int l = 0; l < n1 || l + 1 < n0; l++) begin
      if (l < n1) push_line(1, nb1, l == 0, line_gap);
      if (l + 1 < n0) push_line(0, nb0, 1'b0, line_gap);
    end
    raw_wait();
    push_rec(0, fid, n0);
    push_rec(1, fid, n1);
    meta_wait();
    ev_end(s, 1'b0);
  endtask

  // --------------------------------------------------------------------------------------------
  // FIFO unit checks: order, first word fall through, full and empty, used count
  // --------------------------------------------------------------------------------------------
  logic [84:0] fq   [NF][$];
  realtime     fq_t [NF][$];
  realtime     f_lastpop [NF];
  bit          f_mon = 1'b0;
  int          f_pp [NF];
  int          f_rp [NF];
  int          f_npush, f_npop, f_ign_push, f_ign_pop;

  function automatic logic [84:0] f_mask(input int i, input logic [84:0] v);
    return v & ((85'd1 << f_width[i]) - 85'd1);
  endfunction

  function automatic void f_wr_check(input int i);
    realtime ww = (f_sync[i] + 3) * f_wper + 2.0 * ((f_async[i] != 0) ? f_rper : f_wper) + 1.0;
    if (!f_full[i] && fq[i].size() >= f_depth[i])
      fail($sformatf("fifo %0d not full with %0d entries", i, fq[i].size()));
    if (int'(f_used[i]) < fq[i].size() || int'(f_used[i]) > f_depth[i])
      fail($sformatf("fifo %0d used %0d with %0d entries", i, f_used[i], fq[i].size()));
    if ($realtime - f_lastpop[i] > ww) begin
      if (int'(f_used[i]) != fq[i].size())
        fail($sformatf("fifo %0d used %0d, %0d entries", i, f_used[i], fq[i].size()));
      if (f_full[i] && fq[i].size() < f_depth[i])
        fail($sformatf("fifo %0d still full with %0d entries", i, fq[i].size()));
    end
    if (f_push[i]) begin
      if (!f_full[i]) begin
        fq[i].push_back(f_mask(i, f_din[i]));
        fq_t[i].push_back($realtime);
        f_npush++;
      end else f_ign_push++;
    end
  endfunction

  function automatic void f_rd_check(input int i);
    realtime wr;
    wr = (f_async[i] != 0) ? (f_sync[i] + 3) * f_rper + 2.0 * f_wper + 1.0 : 3.0 * f_wper + 1.0;
    if (!f_empty[i]) begin
      if (fq[i].size() == 0) fail($sformatf("fifo %0d not empty with no entry", i));
      else if (f_mask(i, f_dout[i]) != fq[i][0])
        fail($sformatf("fifo %0d head %0h, expected %0h", i, f_dout[i], fq[i][0]));
      else n_chk++;
    end else if (fq[i].size() > 0 && $realtime - fq_t[i][0] > wr)
      fail($sformatf("fifo %0d empty with %0d entries", i, fq[i].size()));
    if (f_pop[i]) begin
      if (!f_empty[i] && fq[i].size() > 0) begin
        void'(fq[i].pop_front());
        void'(fq_t[i].pop_front());
        f_lastpop[i] = $realtime;
        f_npop++;
      end else if (f_empty[i]) f_ign_pop++;
    end
  endfunction

  always @(posedge f_wclk) begin
    if (f_mon) begin
      for (int i = 0; i < NF; i++) f_wr_check(i);
      f_rd_check(3);
    end
  end

  always @(posedge f_rclk) begin
    if (f_mon) for (int i = 0; i < 3; i++) f_rd_check(i);
  end

  always @(negedge f_wclk) begin
    for (int i = 0; i < NF; i++) begin
      f_push[i] = f_mon && ($urandom_range(99, 0) < f_pp[i]);
      f_din[i]  = {21'($urandom), $urandom, $urandom};
    end
    f_pop[3] = f_mon && ($urandom_range(99, 0) < f_rp[3]);
  end

  always @(negedge f_rclk) begin
    for (int i = 0; i < 3; i++) f_pop[i] = f_mon && ($urandom_range(99, 0) < f_rp[i]);
  end

  task automatic f_set(input int pp, input int rp);
    for (int i = 0; i < NF; i++) begin f_pp[i] = pp; f_rp[i] = rp; end
  endtask

  // Both sides reset together, released each on its own clock.
  task automatic f_reset(input real wper, input real rper);
    f_mon = 1'b0;
    f_set(0, 0);
    f_wrst_n = 1'b0;
    f_rrst_n = 1'b0;
    f_wper = wper;
    f_rper = rper;
    for (int i = 0; i < NF; i++) begin
      fq[i].delete();
      fq_t[i].delete();
      f_lastpop[i] = 0.0;
    end
    repeat (6) @(posedge f_rclk);
    repeat (6) @(posedge f_wclk);
    @(negedge f_wclk) f_wrst_n = 1'b1;
    @(negedge f_rclk) f_rrst_n = 1'b1;
    repeat (2) @(posedge f_wclk);
    for (int i = 0; i < NF; i++) f_lastpop[i] = $realtime;
    if (f_full[0] || f_full[1] || f_full[2] || f_full[3] || !f_empty[0] || !f_empty[1] ||
        !f_empty[2] || !f_empty[3])
      fail("fifo flags not at reset values");
    f_mon = 1'b1;
  endtask

  task automatic f_phase(input int pp, input int rp, input int ncyc);
    f_set(pp, rp);
    repeat (ncyc) @(posedge f_wclk);
  endtask

  real f_wpers [4] = '{5.0, 10.0, 5.0, 7.0};
  real f_rpers [4] = '{13.0, 6.0, 5.3, 7.0};

  task automatic test_fifo_directed();
    int ip0, ip1;
    f_run = 1'b1;
    for (int c = 0; c < 4; c++) begin
      f_reset(f_wpers[c], f_rpers[c]);
      f_phase(0, 0, 20);
      f_phase(100, 0, 1);            // one entry: head falls through
      f_phase(0, 0, 40);
      ip0 = f_ign_push;
      f_phase(100, 0, 600);          // fill to full, pushes while full are ignored
      if (f_ign_push == ip0) fail("no push while full");
      f_phase(0, 0, 40);
      for (int i = 0; i < NF; i++)
        if (!f_full[i] || int'(f_used[i]) != f_depth[i])
          fail($sformatf("fifo %0d not full after filling", i));
      ip1 = f_ign_pop;
      f_phase(0, 100, 1400);         // drain to empty, pops while empty are ignored
      if (f_ign_pop == ip1) fail("no pop while empty");
      f_phase(0, 0, 40);
      for (int i = 0; i < NF; i++)
        if (!f_empty[i] || f_full[i] || f_used[i] != 0)
          fail($sformatf("fifo %0d not empty after draining", i));
      f_phase(100, 100, 300);        // push and pop in the same cycles
      f_phase(0, 100, 1400);
    end
    f_mon = 1'b0;
    if (f_npop < 1000) fail("too few fifo pops");
  endtask

  task automatic test_fifo_random();
    f_run = 1'b1;
    for (int c = 0; c < 4; c++) begin
      f_reset(f_wpers[c], f_rpers[c]);
      for (int p = 0; p < 12; p++) begin
        for (int i = 0; i < NF; i++) begin
          f_pp[i] = $urandom_range(100, 0);
          f_rp[i] = $urandom_range(100, 0);
        end
        repeat ($urandom_range(400, 50)) @(posedge f_wclk);
        if (p == 6) f_reset(f_wpers[c], f_rpers[c]);
      end
      f_phase(0, 100, 1400);
    end
    f_mon = 1'b0;
    if (f_npop < 2000) fail("too few fifo pops");
  endtask

  // --------------------------------------------------------------------------------------------
  // DMA tests
  // --------------------------------------------------------------------------------------------
  function automatic int rnd(input int lo, input int hi);
    return $urandom_range(hi, lo);
  endfunction

  // Reset values and idle outputs; the first claim takes buffer 0.
  task automatic test_reset_idle();
    int b, s;
    tick(4);
    if (!fbd_ready || !meta_ready || m_axi4_awvalid || m_axi4_wvalid || !m_axi4_bready ||
        dma_busy || buf0_state != 2'b00 || buf1_state != 2'b00 || active_buffer ||
        buf0_last_valid_line != 16'h0 || buf1_last_valid_line != 16'h0 ||
        axi_fault_awid != 8'h0 || axi_fault_resp != 2'b00 || axi_fault_awaddr != 32'h0 ||
        bresp_slverr_count != 16'h0 || bresp_decerr_count != 16'h0)
      fail("outputs not at their idle values after reset");
    n_chk++;
    cap_single(0, 2, 64, 3, 2, b, s);
    if (b != 0) fail($sformatf("first claim took buffer %0d", b));
    wait_st(0, M_SWV);
    sw_release(2'b01);
  endtask

  // Single-exposure captures end to end, alternating buffers, released by software.
  task automatic test_single_capture();
    int b, s;
    int exp_b [4] = '{0, 1, 0, 1};
    for (int i = 0; i < 4; i++) begin
      cap_single((i == 3) ? 1 : 0, 2 + i, 64 + 8 * i + (i % 3), (i % 2 == 0) ? 4 : 5, 2 + i,
                 b, s);
      if (b != exp_b[i]) fail($sformatf("capture %0d claimed buffer %0d", i, b));
      wait_st(b, M_SWV);
      if (i == 1) sw_release(2'b01);
      if (i == 2) sw_release(2'b10);
    end
    sw_release(2'b11);
    // Both free: the picker takes the buffer not claimed last, each way.
    for (int i = 0; i < 2; i++) begin
      cap_single(0, 1, 64, 0, 1, b, s);
      if (b != i) fail($sformatf("both buffers free: claimed %0d, expected %0d", b, i));
      wait_st(b, M_SWV);
      sw_release(2'b11);
    end
    if (n_swv != 6) fail($sformatf("%0d buffers released as valid", n_swv));
  endtask

  // Staggered HDR: interleaved lines, overlap of two captures, lost lag Frame Start, orphan.
  task automatic test_hdr_capture();
    int b, s, bn, sn, bm, sm, fidn, fidm, n0, n1, nb;
    sw_auto = 1'b1;
    n0 = rnd(1, 4);  n1 = rnd(1, 4);  nb = 8 * rnd(4, 20) + rnd(0, 7);
    cap_hdr(n0, n1, nb, nb - 8, rnd(0, 6), b, s);
    // Capture N still writing its lag frame when capture N+1 opens in the other buffer.
    ev_lead(0, sn);  bn = cap_res[$];  fidn = slot_fid[sn];
    tick(2);
    push_frame(0, 2, 96, 1'b1);
    raw_wait();
    ev_lag(1, sn);
    tick(1);
    push_line(1, 72, 1'b1, line_gap);
    raw_wait();
    ev_lead(0, sm);  bm = cap_res[$];  fidm = slot_fid[sm];
    if (bm == bn || bm < 0) fail("next capture did not take the other buffer");
    tick(2);
    push_line(0, 80, 1'b1, line_gap);
    push_line(1, 72, 1'b0, line_gap);
    push_line(0, 80, 1'b0, line_gap);
    push_line(1, 72, 1'b0, line_gap);
    raw_wait();
    push_rec(0, fidn, 2);
    push_rec(1, fidn, 3);
    meta_wait();
    ev_end(sn, 1'b0);
    ev_lag(1, sm);
    tick(1);
    push_frame(1, 2, 40, 1'b1);
    raw_wait();
    push_rec(0, fidm, 2);
    push_rec(1, fidm, 2);
    meta_wait();
    ev_end(sm, 1'b0);
    wait_st(bn, M_SWV);
    wait_st(bm, M_SWV);
    quiet_check();
    // Lag Frame Start lost: lag lines arrive without a Frame Start; the capture completes with
    // its lead channel only.
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_line(0, 64, 1'b1, line_gap);
    push_line(1, 64, 1'b0, line_gap);
    push_line(0, 64, 1'b0, line_gap);
    push_line(1, 64, 1'b0, line_gap);
    raw_wait();
    push_rec(0, slot_fid[s], 2);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    // Orphan lag channel.
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_line(0, 48, 1'b1, line_gap);
    ev_orphan(1, s);
    tick(1);
    push_pay(1, 3, 1'b1, 4'hF);
    push_line(1, 48, 1'b1, line_gap);
    push_line(0, 48, 1'b0, line_gap);
    raw_wait();
    push_rec(0, slot_fid[s], 2);
    push_rec(1, 0, 1);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
  endtask

  // Bursts: limits, 4 KB boundaries, partial last beats.
  task automatic test_burst_split();
    int b, s, fid;
    int mbls [5] = '{0, 1, 7, 255, 254};
    int lens [4] = '{2048, 8 * 9 + 1, 8 * 40 + 5, 8 * 3 + 8};
    sw_auto = 1'b1;
    for (int i = 0; i < 5; i++) begin
      tick(1);
      max_burst_len    = 8'(mbls[i]);
      bytes_per_line   = 16'd2048;
      dma_buffer_size  = 32'h0000_4000;
      dma_buffer0_addr = 32'h1000_0F80 + 32'(i * 'h48);
      dma_buffer1_addr = 32'h1010_0FC8 - 32'(i * 'h10);
      // Short bursts use the test pattern path: one command per beat would overflow the
      // command FIFO on the sensor path.
      active_sel = (mbls[i] < 8 || MBB < 64);
      tpg_height_shadow = 16'd4;
      if (active_sel) src_gap = 0;
      else set_rates(100);
      ev_lead(0, s);  b = cap_res[$];  fid = slot_fid[s];
      tick(1);
      active_sel = 1'b0;
      tick(2);
      push_pay(0, rnd(3, 70), 1'b1, 4'h1);
      for (int l = 0; l < 4; l++) push_line(0, lens[(l + i) % 4] - ((l == 3) ? rnd(0, 7) : 0),
                                            l == 0, line_gap);
      raw_wait();
      push_rec(0, fid, 4);
      meta_wait();
      ev_end(s, 1'b0);
      wait_st(b, M_SWV);
    end
  endtask

  // Line pitch, padding, clipping at bytes_per_line and at the end of the RAW region.
  task automatic test_line_pitch_clip();
    int b, s;
    sw_auto = 1'b1;
    tick(1);  bytes_per_line = 16'd128;  line_padding = 8'd24;
    cap_single(0, 3, 128, 2, 3, b, s);      // equal to bytes_per_line, padding untouched
    wait_st(b, M_SWV);
    tick(1);  bytes_per_line = 16'd120;  line_padding = 8'd8;
    cap_single(1, 3, 96, 0, 3, b, s);       // shorter than bytes_per_line
    wait_st(b, M_SWV);
    tick(1);  bytes_per_line = 16'd64;  line_padding = 8'd0;
    cap_single(0, 2, 72, 0, 2, b, s);       // longer: clipped, aborted
    wait_st(b, M_SWE);
    tick(1);  bytes_per_line = 16'd256;  dma_buffer_size = 32'h100;
    cap_single(0, 2, 200, 0, 2, b, s);      // second line past the end of the region
    wait_st(b, M_SWE);
    tick(1);  dma_buffer_size = 32'h0;
    cap_single(1, 1, 16, 0, 1, b, s);       // region of size 0
    wait_st(b, M_SWE);
    tick(1);  dma_buffer_size = 32'h0002_0000;  bytes_per_line = 16'd0;
    cap_single(0, 1, 16, 0, 1, b, s);       // bytes_per_line 0
    wait_st(b, M_SWE);
    tick(1);  bytes_per_line = 16'd256;
    cap_single(0, 2, 64, 0, 2, b, s);
    wait_st(b, M_SWV);
  endtask

  // Metadata: records and payload interleaved, odd/even lines, strobes, area overflows.
  task automatic test_metadata_layout();
    int b, s, fid;
    sw_auto = 1'b1;
    ev_lead(0, s);  b = cap_res[$];  fid = slot_fid[s];
    tick(2);
    push_pay(0, 5, 1'b1, 4'hF);             // odd word count: next line starts in a high half
    push_pay(0, 6, 1'b0, 4'h3);             // even count, partial last word
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    push_rec_mix(0, fid, 2, 0, 7, 1'b0, 4'h1);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    // Record with a frame_id of no buffer, then the right one.
    ev_lead(1, s);  b = cap_res[$];  fid = slot_fid[s];
    tick(2);
    push_frame(1, 1, 40, 1'b1);
    raw_wait();
    push_rec(1, fid + 100, 1);
    push_rec(1, fid, 1);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    // Payload passing the end of the metadata area.
    tick(1);  meta_area_size = 32'h90;
    cap_single(0, 1, 32, 6, 1, b, s);
    wait_st(b, M_SWE);
    // Record passing the end of the metadata area.
    tick(1);  meta_area_size = 32'h20;
    cap_single(0, 1, 32, 0, 1, b, s);
    wait_st(b, M_SWE);
    tick(1);  meta_area_size = 32'h400;
  endtask

  // Records split at 4 KB and at small burst lengths; record completing inside a payload line.
  task automatic test_record_burst();
    int b, s, fid;
    sw_auto = 1'b1;
    for (int m = 0; m < 4; m++) begin
      tick(1);
      max_burst_len = 8'(m);
      bytes_per_line = 16'd64;
      dma_buffer_size = 32'h7F0;
      dma_buffer0_addr = 32'h1000_0008;
      dma_buffer1_addr = 32'h1010_0010;
      use_tpg = 1'b1;                       // short bursts: one command per beat or two
      cap_single(m % 2, 2, 64, 3 + m, 2, b, s);
      use_tpg = 1'b0;
      wait_st(b, M_SWV);
    end
    tick(1);
    max_burst_len = 8'hFF;
    ev_lead(0, s);  b = cap_res[$];  fid = slot_fid[s];
    tick(2);
    push_pay(0, 3, 1'b1, 4'hF);
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    push_rec(0, fid, 2);
    push_pay(0, 4, 1'b0, 4'hF);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
  endtask

  // Slow AXI: metadata FIFOs fill, meta_ready falls, payload words are dropped upstream.
  task automatic test_meta_backpressure();
    int b, s, fid;
    sw_auto = 1'b1;
    w_p = 25;  aw_p = 50;  b_lat_min = 10;  b_lat_max = 50;
    set_rates(800);
    meta_drop = 1'b1;
    tick(1);
    meta_area_size = 32'h800;               // room for every payload word
    for (int i = 0; i < 2; i++) begin
      ev_lead(i, s);  b = cap_res[$];  fid = slot_fid[s];
      tick(2);
      w_block = 1'b1;                       // W held: the metadata FIFOs fill
      push_pay(i, 150, 1'b1, 4'hF);
      push_pay(i, 121, 1'b0, 4'h7);
      push_frame(i, 1, 64, 1'b1);
      tick(400);
      w_block = 1'b0;
      raw_wait();
      push_rec(i, fid, 1);
      meta_wait();
      ev_end(s, 1'b0);
      wait_st(b, M_SWV);
    end
    meta_drop = 1'b0;
    if (meta_low_seen == 0) fail("meta_ready never fell");
  endtask

  // Claim gating: no free buffer, dma_enable/dma_start/dma_abort, lag of a dropped capture,
  // a second Frame Start of a channel after one load.
  task automatic test_claim_gating();
    int b, s, d0;
    cap_single(0, 1, 64, 0, 1, b, s);
    wait_st(b, M_SWV);
    cap_single(0, 1, 64, 0, 1, b, s);
    wait_st(b, M_SWV);
    d0 = n_drop;
    ev_lead(0, s);
    if (cap_res[$] != -1 || n_drop != d0 + 1) fail("no dma_drop with both buffers held");
    tick(2);
    ev_lag(1, s);
    tick(1);
    push_pay(0, 3, 1'b1, 4'hF);
    push_line(0, 64, 1'b1, line_gap);
    push_line(1, 64, 1'b1, line_gap);
    raw_wait();
    push_rec(0, slot_fid[s], 1);
    push_rec(1, slot_fid[s], 1);
    meta_wait();
    ev_end(s, 1'b0);
    // No buffer free and the DMA gated: neither a claim nor dma_drop.
    for (int g = 0; g < 3; g++) begin
      tick(1);
      dma_enable = (g != 0);  dma_start = (g != 1);  dma_abort = (g == 2);
      ev_lead(0, s);
      if (cap_res[$] != -2) fail($sformatf("claim or dma_drop while gated (%0d), none free", g));
      tick(2);
      ev_end(s, 1'b0);
      tick(1);
      dma_enable = 1'b1;  dma_start = 1'b1;  dma_abort = 1'b0;
    end
    sw_release(2'b11);
    for (int g = 0; g < 3; g++) begin
      tick(1);
      dma_enable = (g != 0);  dma_start = (g != 1);  dma_abort = (g == 2);
      ev_lead(0, s);
      if (cap_res[$] != -2) fail($sformatf("claim or drop while gated (%0d)", g));
      tick(2);
      push_line(0, 64, 1'b1, line_gap);
      raw_wait();
      ev_end(s, 1'b0);
      tick(1);
      dma_enable = 1'b1;  dma_start = 1'b1;  dma_abort = 1'b0;
    end
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_frame(0, 2, 64, 1'b1);
    push_frame(0, 2, 64, 1'b1);             // second Frame Start beat without a new load
    raw_wait();
    push_rec(0, slot_fid[s], 2);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    sw_release(2'b11);
  endtask

  // Release in every state, err_frame_policy 0 and 1, two buffers reaching software together.
  task automatic test_buffer_release();
    int b, s, s2, e0;
    sw_release(2'b11);                      // both free: ignored
    ev_lead(0, s);  b = cap_res[$];
    if (b != 0) fail("release of a free buffer changed the claim");
    tick(2);
    sw_release(2'b01);                      // writing: ignored
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    push_rec(0, slot_fid[s], 2);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    sw_release(2'b01);
    b_hold = 1'b1;                          // draining: ignored
    cap_single(1, 2, 64, 2, 2, b, s);
    wait_written(b);
    sw_release(2'b11);
    b_hold = 1'b0;
    wait_st(b, M_SWV);
    sw_release(2'b10);
    b_hold = 1'b1;                          // aborted with writes in flight: ignored
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    tick(30);
    pulse_abort_req();
    sw_release(2'b11);
    b_hold = 1'b0;
    ev_end(s, 1'b0);
    wait_st(b, M_SWE);
    sw_release(2'b11);
    tick(1);
    err_frame_policy = 1'b0;                // policy 0: aborted buffer returns to free
    e0 = n_swe;
    ev_lead(1, s);  b = cap_res[$];
    tick(2);
    push_frame(1, 1, 64, 1'b1);
    raw_wait();
    pulse_abort_req();
    ev_end(s, 1'b0);
    wait_idle();
    if (n_swe != e0) fail("policy 0 buffer released as error");
    tick(1);
    err_frame_policy = 1'b1;
    ev_lead(0, s);                          // two captures without data, aborted together
    ev_lead(0, s2);
    tick(3);
    e0 = n_swe;
    pulse_abort_req();
    ev_end(s, 1'b1);
    ev_end(s2, 1'b1);
    wait_st(0, M_SWE);
    wait_st(1, M_SWE);
    if (n_swe != e0 + 2) fail("both buffers not released as error");
    sw_release(2'b11);
  endtask

  // dma_abort_req: writing buffers aborted, late lines discarded, draining buffer kept,
  // claim in the cycle of dma_abort_req.
  task automatic test_abort_req();
    int b, s, k;
    sw_auto = 1'b1;
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_frame(0, 4, 128, 1'b1);
    k = rnd(5, 60);
    tick(k);
    pulse_abort_req();
    raw_wait();
    push_rec(0, slot_fid[s], 4);            // record after the abort: discarded
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWE);
    // Capture ended, last lines arrive after the abort.
    ev_lead(1, s);  b = cap_res[$];
    tick(2);
    push_frame(1, 2, 64, 1'b1);
    raw_wait();
    push_rec(1, slot_fid[s], 3);
    meta_wait();
    ev_end(s, 1'b0);
    tick(rnd(1, 10));
    pulse_abort_req();
    push_line(1, 64, 1'b0, line_gap);
    raw_wait();
    wait_st(b, M_SWE);
    // Draining buffer is not aborted.
    b_hold = 1'b1;
    cap_single(0, 2, 64, 2, 2, b, s);
    wait_written(b);
    pulse_abort_req();
    tick(rnd(1, 20));
    b_hold = 1'b0;
    wait_st(b, M_SWV);
    wait_idle();
    // Claim in the cycle of dma_abort_req: lead bound to discard, dropped capture aborts it.
    fork
      ev_lead(0, s);
      begin tick(2); dma_abort_req = 1'b1; tick(1); dma_abort_req = 1'b0; end
    join
    b = cap_res[$];
    if (b < 0) fail("no claim in the cycle of dma_abort_req");
    tick(2);
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    push_rec(0, slot_fid[s], 2);
    meta_wait();
    ev_end(s, 1'b1);
    if (b >= 0) wait_st(b, M_SWE);
  endtask

  // DMA_CONTROL during a capture: dma_abort, dma_enable, dma_start.
  task automatic test_csr_control();
    int b, s;
    sw_auto = 1'b1;
    for (int c = 0; c < 3; c++) begin
      ev_lead(0, s);  b = cap_res[$];
      tick(2);
      push_frame(0, 3, 64, 1'b1);
      tick(rnd(4, 30));
      if (c == 0) dma_abort = 1'b1;
      if (c == 1) dma_enable = 1'b0;
      if (c == 2) dma_start = 1'b0;
      raw_wait();
      push_rec(0, slot_fid[s], 3);
      meta_wait();
      ev_end(s, 1'b0);
      wait_st(b, (c == 2) ? M_SWV : M_SWE);
      ev_lead(1, s);
      if (cap_res[$] != -2) fail("claim while DMA_CONTROL blocks it");
      tick(2);
      ev_end(s, 1'b0);
      tick(1);
      dma_abort = 1'b0;  dma_enable = 1'b1;  dma_start = 1'b1;
      wait_idle();
    end
    cap_single(1, 2, 64, 1, 2, b, s);
    wait_st(b, M_SWV);
  endtask

  // Write responses with errors; counters, fault trace, last valid line, policy.
  task automatic test_axi_error();
    int b, s, b0, s0;
    sw_auto = 1'b1;
    // SLVERR on the second line of a RAW frame (first buffer is 0).
    err_lo = 64'h1000_0100;  err_hi = 64'h1000_01FF;  err_n = 1;  err_resp = 2'b10;  err_id = 0;
    cap_single(0, 3, 128, 2, 3, b, s);
    if (b != 0) fail("first claim not buffer 0");
    wait_st(b, M_SWE);
    quiet_check();
    // DECERR on the record while the buffer drains.
    b_hold = 1'b1;
    cap_single(0, 2, 64, 3, 2, b, s);
    wait_written(b);
    err_lo = meta_base(b, 0);  err_hi = meta_base(b, 0) + 'h27;  err_n = 1;  err_resp = 2'b11;
    err_id = 2 + b;
    b_hold = 1'b0;
    wait_st(b, M_SWE);
    quiet_check();
    // Policy 0: aborted buffer goes back to free.
    tick(1);
    err_frame_policy = 1'b0;
    err_lo = 64'h1000_0000;  err_hi = 64'h1020_0000;  err_n = 1;  err_resp = 2'b10;  err_id = -1;
    cap_single(1, 2, 64, 0, 2, b, s);
    quiet_check();
    tick(1);
    err_frame_policy = 1'b1;
    err_n = 0;
    // HDR without error: last valid line is the smaller of the two channels.
    cap_hdr(3, 2, 64, 64, 2, b, s);
    wait_st(b, M_SWV);
    quiet_check();
    // Error on the first write: no line of the channel returned OKAY.
    ev_lead(0, s0);  b0 = cap_res[$];
    err_lo = line_addr(b0, 0, 0);  err_hi = err_lo + 'h3F;  err_n = 1;  err_resp = 2'b11;
    err_id = b0;
    tick(2);
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    push_rec(0, slot_fid[s0], 2);
    meta_wait();
    ev_end(s0, 1'b0);
    wait_st(b0, M_SWE);
    quiet_check();
    // Error on the embedded payload, written before the lines: the last valid line stays
    // 0xFFFF although every line returns OKAY (needs room for all writes in flight).
    if (MOUT >= 8) begin
      ev_lead(0, s0);  b0 = cap_res[$];
      err_lo = meta_base(b0, 0) + 'h80;  err_hi = err_lo + 'hFF;  err_n = 1;  err_resp = 2'b10;
      err_id = 2 + b0;
      b_hold = 1'b1;                          // responses given in order once all is written
      tick(2);
      push_pay(0, 6, 1'b1, 4'hF);
      meta_wait();
      tick(100);
      push_frame(0, 2, 64, 1'b1);
      raw_wait();
      push_rec(0, slot_fid[s0], 2);
      meta_wait();
      ev_end(s0, 1'b0);
      wait_written(b0);
      b_hold = 1'b0;
      wait_st(b0, M_SWE);
      quiet_check();
      if (((b0 == 0) ? buf0_last_valid_line : buf1_last_valid_line) != 16'hFFFF)
        fail("last valid line advanced after a metadata error");
    end
  endtask

  // More than 65535 SLVERR and DECERR responses: counters saturate; soft_reset clears them.
  task automatic test_error_count_saturation();
    int b, s;
    tick(1);
    err_frame_policy = 1'b0;
    active_sel = 1'b1;
    tpg_height_shadow = 16'd1000;
    bytes_per_line = 16'd8;
    err_all = 1'b1;
    b_lat_min = 0;
    b_lat_max = 1;
    src_gap = 0;
    while (n_err_resp < 2 * 65536 + 40) begin
      ev_lead(0, s);
      tick(2);
      for (int l = 0; l < 48; l++) push_line(0, 8, l == 0, 0);
      raw_wait();
      ev_end(s, 1'b1);
    end
    wait_idle();
    check_status();
    if (bresp_slverr_count != 16'hFFFF || bresp_decerr_count != 16'hFFFF)
      fail("error counters not saturated");
    err_all = 1'b0;
    do_soft_reset(1);
    tick(4);
    if (bresp_slverr_count != 16'h0 || bresp_decerr_count != 16'h0)
      fail("error counters not cleared by soft_reset");
    active_sel = 1'b0;
  endtask

  // Line count: longer, shorter (found at the next Frame Start), TPG height, zero lines.
  task automatic test_line_count();
    int b, s, b2, s2;
    sw_auto = 1'b1;
    cap_single(0, 3, 64, 0, 2, b, s);       // one line more than line_count
    wait_st(b, M_SWE);
    cap_single(0, 2, 64, 0, 4, b, s);       // two lines fewer
    tick(10);
    cap_single(0, 2, 64, 0, 2, b2, s2);     // next Frame Start of the channel
    wait_st(b, M_SWE);
    wait_st(b2, M_SWV);
    tick(1);
    active_sel = 1'b1;
    tpg_height_shadow = 16'd3;
    cap_single(1, 3, 64, 0, 7, b, s);       // test pattern capture: height, not line_count
    tick(1);
    active_sel = 1'b0;
    wait_st(b, M_SWV);
    cap_single(0, 0, 64, 2, 0, b, s);       // zero lines
    wait_st(b, M_SWV);
  endtask

  // A claimed frame none of whose beats reaches the DMA (Frame Start beat removed upstream).
  task automatic test_frame_without_beats();
    int b, s, b2, s2;
    sw_auto = 1'b1;
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_line(0, 64, 1'b0, line_gap);
    push_line(0, 64, 1'b0, line_gap);
    raw_wait();
    push_rec(0, slot_fid[s], 3);
    meta_wait();
    ev_end(s, 1'b0);
    tick(20);
    if (m_st[b] != M_WRITE) fail("buffer left writing before the next Frame Start");
    cap_single(0, 2, 64, 0, 2, b2, s2);
    wait_st(b, M_SWE);
    wait_st(b2, M_SWV);
  endtask

  // Buffer aborted and claimed again while its first capture is still open.
  task automatic test_slot_reuse();
    int bp, sp, ba, sa, bb, sb, order;
    order = rnd(0, 1);
    cap_single(0, 1, 64, 0, 1, bp, sp);     // buffer 0 held by software
    wait_st(bp, M_SWV);
    tick(1);
    err_frame_policy = 1'b0;
    ev_lead(0, sa);  ba = cap_res[$];
    tick(2);
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    pulse_abort_req();
    wait_idle();
    tick(1);
    err_frame_policy = 1'b1;
    if (order == 1) begin ev_lag(1, sa); tick(1); end
    ev_lead(0, sb);  bb = cap_res[$];
    if (bb != ba) fail("aborted buffer not claimed again");
    tick(2);
    if (order == 0) begin ev_lag(1, sa); tick(1); end
    push_line(1, 56, 1'b1, line_gap);      // lag frame of the aborted capture: discarded
    push_line(0, 64, 1'b1, line_gap);
    push_line(1, 56, 1'b0, line_gap);
    raw_wait();
    ev_end(sa, 1'(rnd(0, 1)));              // end of the old capture: no effect
    push_line(0, 64, 1'b0, line_gap);
    raw_wait();
    push_rec(1, slot_fid[sa], 2);
    push_rec(0, slot_fid[sb], 2);
    meta_wait();
    ev_end(sb, 1'b0);
    wait_st(bb, M_SWV);
    sw_release(2'b11);
  endtask

  // Sensor path: full RAW data FIFO, then full command FIFO; beats lost, fbd_ready stays 1.
  task automatic test_sensor_overflow();
    int b, s, l0;
    sw_auto = 1'b1;
    tick(1);
    bytes_per_line = 16'd2048;
    src_gap = 0;
    l0 = n_loss;
    aw_block = 1'b1;
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_frame(0, 4, 2048, 1'b1);
    raw_wait();
    aw_block = 1'b0;
    push_rec(0, slot_fid[s], 4);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWE);
    if (n_loss == l0) fail("no beat lost on a full RAW data FIFO");
    l0 = n_loss;
    aw_block = 1'b1;
    ev_lead(1, s);  b = cap_res[$];
    tick(2);
    for (int l = 0; l < int'(PARA_CMD_FIFO_DEPTH) + 8; l++) push_line(1, 8, l == 0, 0);
    raw_wait();
    aw_block = 1'b0;
    push_rec(1, slot_fid[s], int'(PARA_CMD_FIFO_DEPTH) + 8);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWE);
    if (n_loss == l0) fail("no beat lost on a full RAW command FIFO");
    wait_idle();
    set_rates(100);
    cap_single(0, 2, 256, 0, 2, b, s);
    wait_st(b, M_SWV);
  endtask

  // Test Pattern Generator path: fbd_ready follows the RAW FIFOs, no beat lost; switch of
  // active_sel while the last sensor beats still arrive.
  task automatic test_tpg_backpressure();
    int b, s, b2, s2, f0;
    sw_auto = 1'b1;
    tick(1);
    bytes_per_line = 16'd2048;
    active_sel = 1'b1;
    tpg_height_shadow = 16'd3;
    w_p = 20;
    src_gap = 0;
    f0 = fbd_low_seen;
    cap_single(0, 3, 2048, 2, 3, b, s);
    wait_st(b, M_SWV);
    if (fbd_low_seen == f0) fail("fbd_ready never low on the test pattern path");
    tick(1);
    active_sel = 1'b0;
    w_p = 100;
    set_rates(100);
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_frame(0, 3, 512, 1'b1);
    tick(rnd(20, 80));
    w_block = 1'b1;
    raw_wait();                            // last sensor beats are taken while W is blocked
    active_sel = 1'b1;
    tpg_height_shadow = 16'd2;
    ev_lead(0, s2);  b2 = cap_res[$];
    tick(2);
    src_gap = 0;
    push_frame(0, 2, 512, 1'b1);
    tick(40);
    w_block = 1'b0;
    raw_wait();
    push_rec(0, slot_fid[s], 3);
    push_rec(0, slot_fid[s2], 2);
    meta_wait();
    ev_end(s, 1'b0);
    ev_end(s2, 1'b0);
    wait_st(b2, M_SWV);
    tick(1);
    active_sel = 1'b0;
    wait_idle();
  endtask

  // SKIP: abort or soft_reset in the middle of a RAW burst, a payload burst or a record; the
  // following capture is written correctly.
  task automatic test_skip_release();
    int b, s, k, n0;
    sw_auto = 1'b1;
    tick(1);
    err_frame_policy = 1'(rnd(0, 1));
    bytes_per_line = 16'd2048;
    for (int c = 0; c < 4; c++) begin
      ev_lead(c % 2, s);  b = cap_res[$];
      tick(2);
      if (c == 1) push_pay(c % 2, 90, 1'b1, 4'hF);
      push_frame(c % 2, 2, 2048, 1'b1);
      if (c == 3) push_rec(c % 2, slot_fid[s], 2);
      n0 = (c == 1 || c == 3) ? n_mwords : n_beats;
      k = rnd(10, 200);
      if (c == 1) k = rnd(5, 60);
      if (c == 3) k = rnd(1, 7);
      for (int t = 0; t < 100000 && ((c == 1 || c == 3) ? n_mwords : n_beats) - n0 < k; t++)
        @(posedge sys_clk);
      if (c < 2) pulse_abort_req();
      else do_soft_reset(1);
      raw_wait();
      if (c < 2) begin
        push_rec(c % 2, slot_fid[s], 2);
        meta_wait();
        ev_end(s, 1'b0);
      end
      wait_idle();
      set_rates(100);
      cap_single(c % 2, 2, 512, 4, 2, b, s);
      wait_st(b, M_SWV);
      src_gap = 0;
    end
  endtask

  // soft_reset in every buffer state.
  task automatic test_soft_reset();
    int b, s, e0, b1, n;
    n = rnd(1, 3);
    cap_single(0, 1, 64, 0, 1, b, s);       // buffer 0 owned by software
    wait_st(b, M_SWV);
    err_lo = 64'h1010_0000;  err_hi = 64'h1010_00FF;  err_n = 1;  err_resp = 2'b10;  err_id = 1;
    b_hold = 1'b1;                          // buffer 1 writing with writes in flight
    ev_lead(0, s);  b1 = cap_res[$];
    tick(2);
    push_frame(0, 3, 64, 1'b1);
    tick(60);
    e0 = n_swe;
    do_soft_reset(n);
    b_hold = 1'b0;
    wait_idle();
    if (n_swe != e0 || buf0_state != 2'b01) fail("soft_reset: wrong ownership afterwards");
    check_status();
    do_soft_reset(1);                       // error counted before: cleared now
    tick(4);
    check_status();
    sw_release(2'b01);
    b_hold = 1'b1;                          // draining
    cap_single(1, 2, 64, 0, 2, b, s);
    if (b != 0) fail("first claim after soft_reset not buffer 0");
    wait_written(b);
    do_soft_reset(n);
    b_hold = 1'b0;
    wait_idle();
    b_hold = 1'b1;                          // aborted, writes in flight
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    tick(30);
    pulse_abort_req();
    tick(5);
    do_soft_reset(n);
    b_hold = 1'b0;
    wait_idle();
    // soft_reset in the cycle in which the aborted buffer leaves the abort state.
    ev_lead(0, s);  b = cap_res[$];
    tick(3);
    @(negedge sys_clk);
    dma_abort_req = 1'b1;
    @(negedge sys_clk);
    dma_abort_req = 1'b0;
    soft_reset = 1'b1;
    slot_busy[0] = 0;  slot_busy[1] = 0;
    @(negedge sys_clk);
    soft_reset = 1'b0;
    wait_idle();
    if (n_swe != e0) fail("buffer caught by soft_reset released as error");
    // One-cycle soft_reset one cycle after capture_start: no claim.
    fork
      ev_lead(0, s);
      begin tick(2); soft_reset = 1'b1; tick(1); soft_reset = 1'b0; end
    join
    if (cap_res[$] != -2) fail("claim made in a soft_reset cycle");
    slot_busy[0] = 0;  slot_busy[1] = 0;
    tick(4);
    cap_single(1, 2, 64, 1, 2, b, s);
    if (b != 0) fail("first claim after soft_reset not buffer 0");
    wait_st(b, M_SWV);
    sw_release(2'b11);
  endtask

  // resetn and mm_resetn in the middle of traffic, released at different times.
  task automatic test_reset_traffic();
    int b, s;
    sw_auto = 1'b1;
    for (int r = 0; r < 2; r++) begin
      fork
        begin : traffic
          int bb, ss;
          forever begin
            cap_hdr(rnd(1, 3), rnd(1, 3), 64, 56, rnd(0, 4), bb, ss);
            if ($urandom_range(1, 0) == 1) cap_single(0, rnd(1, 3), 128, 2, rnd(1, 3), bb, ss);
          end
        end
      join_none
      tick(rnd(100, 900));
      #(t_sys * 0.3);
      resetn = 1'b0;
      mm_resetn = 1'b0;
      disable traffic;
      raw_q.delete();  meta_q.delete();
      raw_cur_v = 1'b0;  meta_cur_v = 1'b0;
      tick(1);
      drive_idle();
      slot_busy[0] = 0;  slot_busy[1] = 0;
      tick(rnd(3, 10));
      if (r == 0) begin
        @(negedge sys_clk) resetn = 1'b1;
        repeat (rnd(2, 9)) @(negedge mm_aclk);
        mm_resetn = 1'b1;
      end else begin
        @(negedge mm_aclk) mm_resetn = 1'b1;
        repeat (rnd(2, 9)) @(negedge sys_clk);
        resetn = 1'b1;
      end
      fork
        begin repeat (2) @(posedge sys_clk); t_sr = $realtime; sys_run = 1'b1; end
        begin repeat (2) @(posedge mm_aclk); mm_run = 1'b1; end
      join
      tick(4);
      cap_single(0, 2, 64, 2, 2, b, s);
      if (b != 0) fail("first claim after resetn not buffer 0");
      wait_st(b, M_SWV);
      wait_idle();
    end
  endtask

  // AXI subordinate behaviours: AWREADY after WVALID, W before AW, stalls, reordered
  // responses, MAX_OUTSTANDING reached.
  task automatic test_axi_slave_modes();
    int b, s;
    int wb0 = 0;
    sw_auto = 1'b1;
    aw_after_w = 1'b1;
    set_rates(300);
    cap_hdr(3, 2, 200, 96, 3, b, s);
    wait_st(b, M_SWV);
    aw_after_w = 1'b0;
    aw_block = 1'b1;
    use_tpg = 1'b1;                         // addresses held: the source must wait
    fork
      cap_single(0, 2, 128, 4, 2, b, s);
      begin
        wait (w_beats_total > aw_beats_total);
        wb0 = 1;
      end
    join_any
    use_tpg = 1'b0;
    tick(100);
    aw_block = 1'b0;
    wait fork;
    if (wb0 == 0) fail("write data never ahead of its address");
    wait_st(b, M_SWV);
    aw_p = 60;  w_p = 60;  aw_stall_p = 5;  w_stall_p = 5;  stall_max = 50;
    tick(1);
    active_sel = 1'b1;
    tpg_height_shadow = 16'd4;
    src_gap = 0;
    cap_single(1, 4, 248, 6, 4, b, s);
    wait_st(b, M_SWV);
    tick(1);
    active_sel = 1'b0;
    set_rates(1000);
    cap_single(0, 2, 200, 2, 2, b, s);
    wait_st(b, M_SWV);
    slave_defaults();
    b_reorder = 1'b1;  b_lat_min = 1;  b_lat_max = 40;
    set_rates(200);
    cap_hdr(3, 3, 128, 64, 5, b, s);
    wait_st(b, M_SWV);
    b_reorder = 1'b0;
    b_hold = 1'b1;
    tick(1);
    active_sel = 1'b1;
    tpg_height_shadow = 16'(MOUT + 8);
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    for (int l = 0; l < MOUT + 8; l++) push_line(0, 16, l == 0, 0);
    for (int t = 0; t < 40000 && max_out < MOUT; t++) @(posedge sys_clk);
    tick(50);
    if (max_out != MOUT) fail($sformatf("outstanding writes reached %0d of %0d", max_out, MOUT));
    b_hold = 1'b0;
    raw_wait();
    push_rec(0, slot_fid[s], 1);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    tick(1);
    active_sel = 1'b0;
  endtask

  // A metadata burst ready while RAW commands are queued goes out at the next command.
  task automatic test_meta_priority();
    int b, s, nraw, seen;
    sw_auto = 1'b1;
    tick(1);
    active_sel = 1'b1;
    tpg_height_shadow = 16'd12;
    src_gap = 0;
    aw_block = 1'b1;
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    for (int l = 0; l < 12; l++) push_line(0, 64, l == 0, 0);
    tick(150);
    push_rec(0, slot_fid[s], 1);
    meta_wait();
    tick(200);
    aw_log.delete();
    aw_log_en = 1'b1;
    aw_block = 1'b0;
    raw_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    aw_log_en = 1'b0;
    nraw = 0;
    seen = 0;
    foreach (aw_log[i]) if (seen == 0) begin
      if (aw_log[i] == 1) seen = 1;
      else nraw++;
    end
    if (seen == 0 || nraw > 1)
      fail($sformatf("metadata burst issued after %0d queued RAW bursts", nraw));
    else n_chk++;
    tick(1);
    active_sel = 1'b0;
  endtask

  // Configuration sampled at the claim; later CSR writes and low address bits have no effect.
  task automatic test_config_sampling();
    int b, s;
    sw_auto = 1'b1;
    tick(1);
    dma_buffer0_addr = 32'h1000_0005;
    dma_buffer1_addr = 32'h1010_0003;
    dma_buffer_size  = 32'h0001_0007;
    meta_area_size   = 32'h0000_0403;
    awqos = 4'h5;  awprot = 3'b011;  max_burst_len = 8'hFF;  line_padding = 8'd16;
    ev_lead(0, s);  b = cap_res[$];
    tick(1);
    dma_buffer0_addr = 32'h1200_0000;  dma_buffer1_addr = 32'h1300_0000;
    dma_buffer_size = 32'h100;  meta_area_size = 32'h10;  bytes_per_line = 16'd16;
    line_padding = 8'd0;  awqos = 4'hA;  awprot = 3'b101;  max_burst_len = 8'h00;
    err_frame_policy = 1'b0;  active_sel = 1'b1;  tpg_height_shadow = 16'd9;
    tick(1);
    active_sel = 1'b0;
    push_pay(0, 9, 1'b1, 4'hF);
    push_frame(0, 3, 200, 1'b1);
    raw_wait();
    push_rec(0, slot_fid[s], 3);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    tick(1);
    csr_defaults();
    awqos = 4'h3;  awprot = 3'b110;
    cap_single(1, 2, 64, 1, 2, b, s);
    wait_st(b, M_SWV);
  endtask

  // fbd_err and bit 1 of the channel fields do not change anything.
  task automatic test_unused_inputs();
    int b, s;
    sw_auto = 1'b1;
    vc_hi = 1'b1;
    err_rand = 1'b1;
    cap_hdr(3, 3, 96, 64, 4, b, s);
    wait_st(b, M_SWV);
    cap_single(1, 2, 128, 3, 2, b, s);
    wait_st(b, M_SWV);
    vc_hi = 1'b0;
    err_rand = 1'b0;
  endtask

  // Metadata Writer corners and events before any claim.
  task automatic test_meta_corner();
    int b, s, ba, sa, bp, sp, c0;
    sw_auto = 1'b0;
    for (int r = 0; r < 2; r++) begin
      // Events with no capture: right after resetn, then after soft_reset.
      c0 = n_claim;
      ev_orphan(1, 0);
      tick(1);
      push_pay(1, 3, 1'b1, 4'hF);
      push_pay(0, 2, 1'b0, 4'hF);
      push_line(0, 64, 1'b1, line_gap);
      raw_wait();
      push_rec(0, 1, 1);
      meta_wait();
      @(negedge sys_clk) capture_end = 2'b01;
      @(negedge sys_clk) capture_end = 2'b00;
      tick(20);
      if (n_claim != c0 || n_aw != 0) fail("activity without a capture");
      if (r == 0) do_soft_reset(1);
    end
    // First error after soft_reset on a metadata write of buffer 0.
    err_lo = 64'h1004_0000;  err_hi = 64'h1004_0FFF;  err_n = 1;  err_resp = 2'b11;  err_id = 2;
    cap_single(0, 1, 64, 2, 1, b, s);
    wait_st(b, M_SWE);
    quiet_check();
    sw_release(2'b11);
    // Record completing in the middle of an embedded line; second meta_sof in one capture.
    ev_lead(0, s);  b = cap_res[$];
    tick(2);
    push_pay(0, 3, 1'b1, 4'hF);
    push_frame(0, 2, 64, 1'b1);
    raw_wait();
    push_rec(0, slot_fid[s], 2);
    push_pay(0, 4, 1'b0, 4'h3);
    push_pay(0, 3, 1'b1, 4'hF);
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWV);
    sw_release(2'b11);
    // Abort in the middle of an embedded line, around the end of a record.
    ev_lead(1, s);  b = cap_res[$];
    tick(2);
    push_frame(1, 1, 64, 1'b1);
    raw_wait();
    push_pay(1, 4, 1'b1, 4'hF);
    push_rec(1, slot_fid[s], 1);
    push_pay(1, 30, 1'b0, 4'hF);
    while (meta_q.size() > 25) @(posedge sys_clk);
    pulse_abort_req();
    meta_wait();
    ev_end(s, 1'b0);
    wait_st(b, M_SWE);
    sw_release(2'b11);
    // Rest of an embedded line arriving while its old buffer is written by a later capture.
    cap_single(0, 1, 64, 0, 1, bp, sp);     // the other buffer held by software
    wait_st(bp, M_SWV);
    tick(1);
    err_frame_policy = 1'b0;
    ev_lead(1, sa);  ba = cap_res[$];
    tick(2);
    push_pay(1, 3, 1'b1, 4'hF);
    meta_wait();
    pulse_abort_req();
    wait_idle();
    tick(1);
    err_frame_policy = 1'b1;
    ev_lead(1, s);  b = cap_res[$];
    if (b != ba) fail("aborted buffer not claimed again");
    tick(2);
    push_pay(1, 5, 1'b0, 4'hF);             // rest of the old line: binding not used
    push_frame(1, 1, 64, 1'b1);
    raw_wait();
    push_rec(1, slot_fid[s], 1);
    meta_wait();
    ev_end(s, 1'b0);
    ev_end(sa, 1'b1);
    wait_st(b, M_SWV);
    sw_release(2'b11);
  endtask

  // Random mix of every feature within the input rules.
  task automatic test_random_mix();
    int b, s, kind, k, n0;
    lenient = 1'b1;
    sw_auto = 1'b1;
    for (int ep = 0; ep < 40; ep++) begin
      slave_defaults();
      aw_p = rnd(40, 100);  w_p = rnd(40, 100);  aw_stall_p = rnd(0, 3);  w_stall_p = rnd(0, 3);
      stall_max = rnd(1, 30);  b_lat_min = rnd(0, 3);  b_lat_max = b_lat_min + rnd(0, 30);
      b_reorder = 1'(rnd(0, 1));  aw_after_w = (rnd(0, 7) == 0);
      sw_min = rnd(1, 10);  sw_max = sw_min + rnd(0, 60);
      set_rates(100 * 100 / ((aw_p < w_p) ? aw_p : w_p) + 50 * b_lat_max);
      tick(1);
      err_frame_policy = 1'(rnd(0, 1));
      max_burst_len = 8'(rnd(0, 3) == 0 ? rnd(0, 7) : 255);
      line_padding = 8'(8 * rnd(0, 3));
      kind = rnd(0, 8);
      if (rnd(0, 4) == 0) begin
        err_lo = 64'h1000_0000;  err_hi = 64'h1020_0000;  err_n = 1;
        err_resp = rnd(0, 1) ? 2'b10 : 2'b11;
      end
      if (kind == 7) begin active_sel = 1'b1; tpg_height_shadow = 16'(rnd(1, 4)); src_gap = 0; end
      case (kind)
        0, 7: begin
          cap_single(rnd(0, 1), rnd(0, 4), rnd(1, 300), rnd(0, 20), rnd(0, 4), b, s);
        end
        1: cap_hdr(rnd(1, 4), rnd(1, 4), rnd(8, 200), rnd(8, 200), rnd(0, 10), b, s);
        2, 3, 4: begin
          ev_lead(0, s);  b = cap_res[$];
          tick(2);
          push_pay(0, rnd(1, 40), 1'b1, 4'hF);
          push_frame(0, rnd(1, 4), rnd(8, 400), 1'b1);
          n0 = n_beats;
          k = rnd(1, 40);
          while (n_beats - n0 < k && (raw_q.size() > 0 || raw_cur_v)) @(posedge sys_clk);
          if (kind == 2) pulse_abort_req();
          if (kind == 3) begin tick(1); dma_abort = 1'b1; tick(rnd(1, 5)); dma_abort = 1'b0; end
          if (kind == 4) do_soft_reset(rnd(1, 3));
          raw_wait();
          if (kind != 4) begin
            push_rec(0, slot_fid[s], rnd(1, 4));
            meta_wait();
            ev_end(s, 1'(rnd(0, 1)));
          end
        end
        5: begin
          tick(1);
          dma_enable = 1'b0;
          cap_single(1, 1, 64, 1, 1, b, s);
          tick(1);
          dma_enable = 1'b1;
        end
        default: begin
          ev_lead(0, s);  b = cap_res[$];
          tick(2);
          push_frame(0, rnd(1, 3), rnd(8, 128), 1'b1);
          raw_wait();
          push_rec(0, slot_fid[s], rnd(0, 4));
          meta_wait();
          ev_end(s, 1'b0);
          tick(rnd(1, 50));
          cap_single(0, 1, 64, 0, 1, b, s);
        end
      endcase
      tick(1);
      active_sel = 1'b0;
      if (rnd(0, 3) == 0) begin
        // A frame shorter than its line_count waits for the next Frame Start of its channel,
        // without timeout: software stops such a capture.
        tick(rnd(50, 300));
        if (m_st[0] == M_WRITE || m_st[1] == M_WRITE) pulse_abort_req();
        wait_idle();
        err_n = 0;
        check_status();
      end
    end
    // Captures left writing for lack of lines are stopped by software.
    tick(rnd(1, 300));
    pulse_abort_req();
    slave_defaults();
    wait_idle();
    if (n_swv == 0) fail("no buffer released as valid");
  endtask

  // Forces one wrong expected byte: this test must end in FAIL with errors in the log.
  task automatic test_checker_fault();
    int b, s;
    fault_inject = 1'b1;
    cap_single(0, 2, 64, 0, 2, b, s);
    wait_st(b, M_SWV);
  endtask

  // --------------------------------------------------------------------------------------------
  // Main
  // --------------------------------------------------------------------------------------------
  task automatic apply_reset();
    resetn = 1'b0;
    mm_resetn = 1'b0;
    model_reset();
    slave_reset();
    repeat (6) @(negedge sys_clk);
    resetn = 1'b1;
    if (PARA_ASYNC_MM_CLK != 32'd0) @(negedge mm_aclk);
    mm_resetn = 1'b1;
    fork
      begin repeat (2) @(posedge sys_clk); t_sr = $realtime; sys_run = 1'b1; end
      begin repeat (2) @(posedge mm_aclk); mm_run = 1'b1; end
    join
  endtask

  task automatic final_check();
    wait_idle();
    check_status();
    foreach (ovf_q[i])
      if (ovf_q[i].must) fail("dma_buffer_overflow still expected at the end");
  endtask

  int tcyc = 0;

  always @(posedge sys_clk) begin
    tcyc++;
    if (tcyc > max_cyc) begin
      fail($sformatf("timeout: beats queued %0d, words queued %0d, fbd_ready %0b, meta_ready %0b",
                     raw_q.size(), meta_q.size(), fbd_ready, meta_ready));
      fail($sformatf("timeout: model states %0d %0d, dma_busy %0b, subordinate idle %0b",
                     m_st[0], m_st[1], dma_busy, slave_idle()));
      $display("%0s FAIL (timeout)", test_name);
      $finish;
    end
  end

  initial begin
    bit fifo_test;
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "none";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    dbg = $test$plusargs("DBG");
    void'($urandom(seed));
    clk_sel = (seed > 0) ? (seed - 1) % 3 : 0;
    case (clk_sel)
      0: begin t_sys = 5.0;  t_mm = 13.0; end
      1: begin t_sys = 5.0;  t_mm = 5.3;  end
      default: begin t_sys = 10.0; t_mm = 6.0; end
    endcase
    t_ph = t_mm * $urandom_range(999, 1) / 1000.0;
    if (test_name == "test_error_count_saturation") max_cyc = 8000000;
    $display("tb_top: test %0s seed %0d, sys_clk %.1f ns, mm_aclk %.1f ns, phase %.2f ns",
             test_name, seed, t_sys, (PARA_ASYNC_MM_CLK == 32'd0) ? t_sys : t_mm, t_ph);
    drive_idle();
    slave_defaults();
    clk_go = 1'b1;
    apply_reset();
    set_rates(100);
    tick(2);
    fifo_test = 1'b0;
    case (test_name)
      "test_reset_idle":              test_reset_idle();
      "test_single_capture":          test_single_capture();
      "test_hdr_capture":             test_hdr_capture();
      "test_burst_split":             test_burst_split();
      "test_line_pitch_clip":         test_line_pitch_clip();
      "test_metadata_layout":         test_metadata_layout();
      "test_record_burst":            test_record_burst();
      "test_meta_backpressure":       test_meta_backpressure();
      "test_claim_gating":            test_claim_gating();
      "test_buffer_release":          test_buffer_release();
      "test_abort_req":               test_abort_req();
      "test_csr_control":             test_csr_control();
      "test_axi_error":               test_axi_error();
      "test_error_count_saturation":  test_error_count_saturation();
      "test_line_count":              test_line_count();
      "test_frame_without_beats":     test_frame_without_beats();
      "test_slot_reuse":              test_slot_reuse();
      "test_sensor_overflow":         test_sensor_overflow();
      "test_tpg_backpressure":        test_tpg_backpressure();
      "test_skip_release":            test_skip_release();
      "test_soft_reset":              test_soft_reset();
      "test_reset_traffic":           test_reset_traffic();
      "test_axi_slave_modes":         test_axi_slave_modes();
      "test_meta_priority":           test_meta_priority();
      "test_config_sampling":         test_config_sampling();
      "test_unused_inputs":           test_unused_inputs();
      "test_meta_corner":             test_meta_corner();
      "test_random_mix":              test_random_mix();
      "test_checker_fault":           test_checker_fault();
      "test_fifo_directed":  begin fifo_test = 1'b1; test_fifo_directed(); end
      "test_fifo_random":    begin fifo_test = 1'b1; test_fifo_random(); end
      default: fail($sformatf("unknown test %0s", test_name));
    endcase
    if (!fifo_test) final_check();
    $display("tb_top: claims %0d, drops %0d, valid %0d, error %0d, free after abort %0d",
             n_claim, n_drop, n_swv, n_swe, n_free_ab);
    $display("tb_top: beats lost %0d, overflow pulses %0d, error responses %0d, bytes %0d",
             n_loss, n_ovf_pulse, n_err_resp, n_wbytes);
    if (n_err == 0 && n_chk > 0) $display("%0s PASS", test_name);
    else $display("%0s FAIL (errors=%0d, checks=%0d)", test_name, n_err, n_chk);
    $finish;
  end

endmodule
