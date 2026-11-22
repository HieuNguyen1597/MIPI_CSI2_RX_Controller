`timescale 1ns/1ps
`default_nettype none
// Resets assert together and release on their own clocks; source reset waits for its clock.
// Channel errors use VC0/VC1. Simultaneous new nonzero frame IDs normally agree.
// Lane pulses last one source cycle. Abort spacing is at least 295 ns + 10 UI.
// Reset tail covers two crossing round trips; the abort gap exceeds one round trip.
// Frame ID zero is orphaned. Tests do not accelerate the fixed-width counter limits.
module tb_top #(
  parameter logic [31:0] PARA_MAX_LANES = 4,
  parameter logic [31:0] PARA_SYNC_STAGES = 2,
  parameter logic [31:0] PARA_RST_TAIL = 256,
  parameter int CLK_CASE = 1
);
  localparam int SYS_PS = 4464;
  localparam int BYTE_PS = CLK_CASE == 4 ? 17857 :
                          CLK_CASE == 22 ? 100000 : CLK_CASE == 0 ? 3000 : 4500;
  localparam int PHASE_PS = CLK_CASE == 4 ? 700 :
                           CLK_CASE == 22 ? 2100 : CLK_CASE == 0 ? 400 : 1300;
  localparam int ABORT_GAP = (295000 + (10*BYTE_PS+7)/8 + BYTE_PS-1)/BYTE_PS;
  localparam int CROSS_CYC = (BYTE_PS+SYS_PS-1)/SYS_PS + PARA_SYNC_STAGES + 2;
  localparam logic [23:0] SYNC_MASK = 24'hf7f1ff;
  typedef struct packed {
    logic soft_reset, phy_reset, global_en;
    logic [23:0] mask, clear;
    logic frame_sync, frame_level, crc, ecc1, ecc2, unsupported, wc;
    logic [1:0] vc;
    logic lane_error, packet, sp_overflow, wc_align, select_tpg, full;
    logic lb_overflow, fb_overflow, capture_done, resolution, frame_num;
    logic [13:0] fid0, fid1;
    logic [15:0] line0, line1;
    logic meta_overflow, drop, dma_ok, dma_err, axi_error, dma_overflow;
  } inputs_t;
  typedef struct packed {
    logic [23:0] flags;
    logic [31:0] capture, ecc, crc, drop;
    logic [15:0] abort_count;
    logic [31:0] packets;
    logic lb_sticky, sp_sticky, dma_sticky, irq;
  } outputs_t;
  inputs_t s = '0;
  wire outputs_t q;
  logic sys_clk = 0, byte_clk = 0, raw_resetn = 0;
  logic [1:0] sys_release = 0, byte_release = 0;
  wire resetn = sys_release[1];
  wire source_resetn = byte_release[1];
  logic [PARA_MAX_LANES-1:0] sot = 0, sotsync = 0;
  logic abort_pulse = 0, stop_state = 0;
  bit byte_run = 1, clocks_run = 1, done = 0;
  string test_name = "unset";
  int seed = 1, seed_init, errors = 0, reset_edges = 0;
  longint checks = 0, cycles = 0;
  bit robust = 0, check_ready = 0;

  m_fpt_irq_aggregator #(
    .PARA_MAX_LANES(PARA_MAX_LANES), .PARA_SYNC_STAGES(PARA_SYNC_STAGES),
    .PARA_RST_TAIL(PARA_RST_TAIL)
  ) u_dut (
    .i_sys_clk(sys_clk), .i_rxbyteclkhs(byte_clk), .i_resetn(resetn),
    .i_rxbyteclkhs_resetn(source_resetn),
    .i_soft_reset(s.soft_reset), .i_phy_rst_in_progress(s.phy_reset),
    .i_global_irq_enable(s.global_en), .i_irq_enable_mask(s.mask),
    .i_irq_clear_strobe(s.clear), .o_irq_status(q.flags), .o_error_capture(q.capture),
    .o_ecc_err_cnt(q.ecc), .o_crc_err_cnt(q.crc), .o_frame_drop_cnt(q.drop),
    .o_lane_abort_cnt(q.abort_count), .o_packet_cnt(q.packets),
    .o_linebuf_ovf_sticky(q.lb_sticky), .o_sp_fifo_ovf_sticky(q.sp_sticky),
    .o_dma_ovf_sticky(q.dma_sticky), .o_csi2_irq(q.irq),
    .i_errsoths(sot), .i_errsotsynchs(sotsync), .i_cl_stopstate(stop_state),
    .i_lane_abort_pulse(abort_pulse), .i_err_frame_sync(s.frame_sync),
    .i_err_frame_level(s.frame_level), .i_err_crc(s.crc), .i_err_ecc_1bit(s.ecc1),
    .i_err_ecc_2bit(s.ecc2), .i_err_unsupported_dt(s.unsupported), .i_err_wc(s.wc),
    .i_err_vc(s.vc), .i_err_lane_abort(s.lane_error), .i_packet_done_pulse(s.packet),
    .i_sp_fifo_overflow(s.sp_overflow), .i_err_wc_align(s.wc_align),
    .i_active_sel(s.select_tpg), .i_linebuffer_full(s.full),
    .i_linebuf_overflow(s.lb_overflow), .i_fb_overflow(s.fb_overflow),
    .i_capture_done(s.capture_done), .i_err_resolution(s.resolution),
    .i_err_frame_num(s.frame_num), .i_cur_frame_id_vc0(s.fid0),
    .i_cur_frame_id_vc1(s.fid1), .i_cur_line_vc0(s.line0), .i_cur_line_vc1(s.line1),
    .i_meta_overflow(s.meta_overflow), .i_frame_drop_pulse(s.drop),
    .i_dma_buf_done_ok(s.dma_ok), .i_dma_buf_done_err(s.dma_err),
    .i_dma_axi_write_error(s.axi_error), .i_dma_buffer_overflow(s.dma_overflow)
  );

  initial begin
    #0.001;
    if (test_name != "test_elaboration")
      forever begin #(SYS_PS/2000.0); if (clocks_run) sys_clk = ~sys_clk; end
  end
  initial begin
    #(PHASE_PS/1000.0);
    if (test_name != "test_elaboration")
      forever begin
        #((byte_clk ? (BYTE_PS+1)/2 : BYTE_PS/2)/1000.0);
        if (byte_run && clocks_run) byte_clk = ~byte_clk;
      end
  end
  always @(posedge sys_clk or negedge raw_resetn)
    if (!raw_resetn) sys_release <= 0;
    else sys_release <= {sys_release[0], 1'b1};
  always @(posedge byte_clk or negedge raw_resetn)
    if (!raw_resetn) byte_release <= 0;
    else byte_release <= {byte_release[0], 1'b1};

  task automatic check(input bit ok, input string why);
    checks++;
    if (!ok) begin
      errors++;
      if (errors <= 16) $error("%s cycle=%0d time=%0t: %s", test_name, cycles, $time, why);
    end
  endtask
  task automatic compare_word(input logic [31:0] got, want, input string name);
    if (got !== want)
      check(0, $sformatf("%s got=%08h expected=%08h", name, got, want));
    else checks++;
  endtask
  task automatic finish_test;
    if (!done) begin
      done = 1;
      if (checks == 0) check(0, "no checks executed");
      $display("checks=%0d errors=%0d sys_cycles=%0d seed=%0d",
               checks, errors, cycles, seed);
      if (errors == 0) $display("%s PASS", test_name);
      else $display("%s FAIL errors=%0d", test_name, errors);
      $finish;
    end
  endtask
  task automatic step(input int n = 1);
    repeat (n) begin @(negedge sys_clk); #0.001; end
  endtask
  task automatic source_step(input int n = 1);
    repeat (n) begin @(negedge byte_clk); #0.001; end
  endtask
  task automatic quiet;
    s.frame_sync=0; s.frame_level=0; s.crc=0; s.ecc1=0; s.ecc2=0;
    s.unsupported=0; s.wc=0; s.lane_error=0; s.packet=0; s.sp_overflow=0;
    s.wc_align=0; s.full=0; s.lb_overflow=0; s.fb_overflow=0;
    s.capture_done=0; s.resolution=0; s.frame_num=0; s.meta_overflow=0;
    s.drop=0; s.dma_ok=0; s.dma_err=0; s.axi_error=0; s.dma_overflow=0;
    s.clear=0;
  endtask
  task automatic drain;
    step(4*CROSS_CYC + 8);
  endtask
  task automatic clear_flags(input logic [23:0] bits = '1);
    step(); s.clear = bits;
    step(); s.clear = 0;
    step(5);
  endtask
  task automatic reset_all(input bit resume_source = 1);
    step(); s='0; sot=0; sotsync=0; abort_pulse=0; stop_state=0;
    #0.317; raw_resetn=0;
    step(5); raw_resetn=1;
    if (resume_source) begin
      byte_run=1;
      wait (source_resetn === 1'b1);
    end
    wait (resetn === 1'b1);
    step(5);
  endtask
  task automatic soft_clear;
    step(); s.soft_reset=1;
    step(); s.soft_reset=0;
    step(PARA_RST_TAIL+5);
  endtask
  task automatic drive_event(input int b);
    case (b)
      0,2: begin s.frame_level=1; s.vc=(b==2); end
      1,3: begin s.frame_sync=1; s.vc=(b==3); end
      4: s.crc=1;
      5: s.ecc1=1;
      6: s.ecc2=1;
      7: s.unsupported=1;
      8: s.wc=1;
      12: begin s.full=1; s.select_tpg=0; end
      13: s.lb_overflow=1;
      14: s.drop=1;
      15: s.capture_done=1;
      16: s.dma_ok=1;
      17: s.dma_err=1;
      18: s.axi_error=1;
      20: s.resolution=1;
      21: s.wc_align=1;
      22: s.fb_overflow=1;
      23: s.frame_num=1;
      24: s.packet=1;
      25: s.lane_error=1;
      26: s.sp_overflow=1;
      27: s.meta_overflow=1;
      28: s.dma_overflow=1;
      default: check(0, "invalid synchronous event selector");
    endcase
  endtask
  task automatic pulse_event(input int b);
    step(); quiet(); drive_event(b);
    step(); quiet();
    step(2);
  endtask
  task automatic all_events;
    s.frame_level=1; s.frame_sync=1; s.crc=1; s.ecc1=1; s.ecc2=1;
    s.unsupported=1; s.wc=1; s.full=1; s.select_tpg=0; s.lb_overflow=1;
    s.sp_overflow=1; s.meta_overflow=1; s.dma_overflow=1; s.drop=1;
    s.capture_done=1; s.dma_ok=1; s.dma_err=1; s.axi_error=1;
    s.resolution=1; s.wc_align=1; s.fb_overflow=1; s.frame_num=1;
    s.packet=1; s.lane_error=1;
  endtask
  task automatic source_pulse(input int channel, input int lane = 0);
    source_step();
    case (channel)
      0: sot[lane]=1;
      1: sotsync[lane]=1;
      2: abort_pulse=1;
    endcase
    source_step(); sot=0; sotsync=0; abort_pulse=0;
  endtask

  // Port-driven model of the acknowledged event transfers.
  logic [2:0] sent = 0, pending = 0, accepted = 0;
  logic [2:0] acknowledge [0:PARA_SYNC_STAGES-1];
  logic [2:0] requests [0:PARA_SYNC_STAGES-1];
  logic [PARA_SYNC_STAGES-1:0] stop_pipe = 0;
  logic old_stop = 0;
  wire [2:0] source_events = {abort_pulse, |sotsync, |sot};
  always @(posedge byte_clk or negedge source_resetn) begin : source_model
    if (!source_resetn) begin
      sent <= 0; pending <= 0;
      for (int j=0; j<PARA_SYNC_STAGES; j++) acknowledge[j] <= 0;
    end else begin
      acknowledge[0] <= accepted;
      for (int j=1; j<PARA_SYNC_STAGES; j++) acknowledge[j] <= acknowledge[j-1];
      for (int k=0; k<3; k++) begin
        if ((sent[k] == acknowledge[PARA_SYNC_STAGES-1][k]) &&
            (pending[k] || source_events[k])) begin
          sent[k] <= ~sent[k];
          pending[k] <= 0;
        end else if (source_events[k]) pending[k] <= 1;
      end
    end
  end
  always @(posedge sys_clk or negedge resetn) begin : destination_model
    if (!resetn) begin
      accepted <= 0; stop_pipe <= 0; old_stop <= 0;
      for (int j=0; j<PARA_SYNC_STAGES; j++) requests[j] <= 0;
    end else begin
      requests[0] <= sent;
      for (int j=1; j<PARA_SYNC_STAGES; j++) requests[j] <= requests[j-1];
      accepted <= requests[PARA_SYNC_STAGES-1];
      stop_pipe <= (stop_pipe << 1) | stop_state;
      old_stop <= stop_pipe[PARA_SYNC_STAGES-1];
    end
  end

  outputs_t expected;
  outputs_t expected_history [0:6], actual_history [0:6];
  logic [23:0] uncertain = 0, uncertain_history [0:6];
  int low_abort = 0, high_abort = 0, low_history [0:6], high_history [0:6];
  logic [4:0] level_previous = 0;
  logic [13:0] id_previous [0:1];
  int newest = 0, previous = 0, tail = 0, history_count = 0;
  bit newest_open = 1, previous_open = 0, last_blocked = 0;
  longint last_gate_change = -1000000;
  realtime last_abort = -1.0;
  logic [PARA_MAX_LANES-1:0] old_sot = 0, old_sotsync = 0;
  always @(posedge byte_clk) begin
    if (!source_resetn) begin
      last_abort=-1.0; old_sot=0; old_sotsync=0;
    end else begin
      if (abort_pulse) begin
        if (last_abort >= 0)
          check(($realtime-last_abort)*1000.0 >= 295000.0+10.0*BYTE_PS/8.0-0.01,
                "abort input spacing");
        last_abort=$realtime;
      end
      check(!( |(sot & old_sot)), "SOT input held across source cycles");
      check(!( |(sotsync & old_sotsync)), "SOT sync input held across source cycles");
      old_sot=sot; old_sotsync=sotsync;
    end
  end

  task automatic reset_model;
    expected='0; level_previous=0; id_previous[0]=0; id_previous[1]=0;
    newest=0; previous=0; newest_open=1; previous_open=0; tail=0;
    uncertain=0; low_abort=0; high_abort=0; history_count=0;
    last_blocked=0; last_gate_change=-1000000;
    for (int j=0; j<7; j++) begin
      expected_history[j]='0; actual_history[j]='0; uncertain_history[j]=0;
      low_history[j]=0; high_history[j]=0;
    end
  endtask
  task automatic model_tick;
    logic [23:0] events;
    logic [4:0] levels, rise_level, triggers;
    logic [2:0] transferred;
    bit blocked, change0, change1, known, hit, chosen, irq_next, edge_band;
    int fid, chosen_fid, chosen_vc, chosen_line;
    events=0;
    transferred=requests[PARA_SYNC_STAGES-1]^accepted;
    levels={s.dma_overflow,s.meta_overflow,s.sp_overflow,
            s.lb_overflow,(s.full && !s.select_tpg)};
    rise_level=levels & ~level_previous;
    level_previous=levels;
    blocked=s.soft_reset || s.phy_reset || tail!=0;
    if (blocked != last_blocked) last_gate_change=cycles;
    last_blocked=blocked;
    edge_band=(cycles-last_gate_change <= CROSS_CYC) ||
              (tail>0 && tail<=CROSS_CYC);
    if (s.soft_reset || s.phy_reset) tail=PARA_RST_TAIL;
    else if (tail>0) tail--;
    events[s.vc[0] ? 2 : 0]=s.frame_level;
    events[s.vc[0] ? 3 : 1]=s.frame_sync;
    events[4]=s.crc; events[5]=s.ecc1; events[6]=s.ecc2;
    events[7]=s.unsupported; events[8]=s.wc;
    events[9]=transferred[0]; events[10]=transferred[1];
    events[11]=stop_pipe[PARA_SYNC_STAGES-1] && !old_stop;
    events[12]=rise_level[0]; events[13]=|rise_level[4:1];
    events[14]=s.drop; events[15]=s.capture_done; events[16]=s.dma_ok;
    events[17]=s.dma_err; events[18]=s.axi_error; events[19]=transferred[2];
    events[20]=s.resolution; events[21]=s.wc_align;
    events[22]=s.fb_overflow; events[23]=s.frame_num;
    for (int b=0; b<24; b++) begin
      if (!SYNC_MASK[b]) begin
        if (s.clear[b] && !events[b]) uncertain[b]=0;
        if (events[b] && edge_band) uncertain[b]=1;
        else if (events[b] && !blocked) uncertain[b]=0;
      end
    end
    if (transferred[2]) begin
      if (!blocked && !edge_band && low_abort<65535) low_abort++;
      if ((!blocked || edge_band) && high_abort<65535) high_abort++;
    end
    irq_next=!s.soft_reset && s.global_en && (|(q.flags & s.mask));
    change0=(s.fid0!=id_previous[0] && s.fid0!=0 && s.fid0!=newest);
    change1=(s.fid1!=id_previous[1] && s.fid1!=0 && s.fid1!=newest);
    if (!robust) begin
      check(!(change0 && change1) || s.fid0==s.fid1, "simultaneous frame IDs differ");
      if (s.frame_level || s.frame_sync || s.crc || s.wc || s.unsupported)
        check(!s.vc[1], "channel outside normal environment");
    end
    id_previous[0]=s.fid0; id_previous[1]=s.fid1;
    if (change0 || change1) begin
      previous=newest; previous_open=newest_open;
      newest=change0 ? s.fid0 : s.fid1; newest_open=1;
    end
    if (blocked) events=0;
    expected.flags=(expected.flags & ~s.clear) | events;
    if (events[5] && expected.ecc[15:0]!=16'hffff) expected.ecc[15:0]++;
    if (events[6] && expected.ecc[31:16]!=16'hffff) expected.ecc[31:16]++;
    if (events[4] && expected.crc!=32'hffffffff) expected.crc++;
    if (events[14] && expected.drop!=32'hffffffff) expected.drop++;
    if (events[19] && expected.abort_count!=16'hffff) expected.abort_count++;
    if (s.packet && !blocked) expected.packets++;
    if (!blocked) begin
      expected.lb_sticky |= rise_level[1];
      expected.sp_sticky |= rise_level[2];
      expected.dma_sticky |= rise_level[4];
    end
    triggers={s.lane_error && !blocked,events[8],events[6],events[4],
              events[1] || events[3]};
    chosen=0; chosen_fid=0; chosen_line=0; chosen_vc=0;
    for (int age=0; age<2; age++) begin
      for (int k=0; k<5; k++) begin
        known=(k==0 || k==1 || k==3);
        fid=known ? (s.vc[0] ? s.fid1 : s.fid0) : newest;
        hit=(age==0) ? (newest_open && fid==newest) :
                      (known && previous_open && fid==previous);
        if (!chosen && triggers[k] && fid!=0 && hit) begin
          chosen=1; chosen_fid=fid;
          chosen_vc=known ? int'(s.vc[0]) : 3;
          chosen_line=known ? (s.vc[0] ? s.line1 : s.line0) : 65535;
          if (age==0) newest_open=0;
          previous_open=0;
        end
      end
    end
    if (chosen)
      expected.capture={2'(chosen_vc),14'(chosen_fid),16'(chosen_line)};
    if (s.soft_reset) begin
      expected='0; newest=0; previous=0; newest_open=1; previous_open=0;
      uncertain=0; low_abort=0; high_abort=0;
    end
    expected.irq=irq_next;
  endtask
  task automatic compare_outputs;
    bit match_bit, match_count;
    compare_word({8'd0,q.flags & SYNC_MASK},{8'd0,expected.flags & SYNC_MASK},"status");
    compare_word(q.capture,expected.capture,"capture");
    compare_word(q.ecc,expected.ecc,"ecc");
    compare_word(q.crc,expected.crc,"crc");
    compare_word(q.drop,expected.drop,"drop");
    compare_word(q.packets,expected.packets,"packets");
    check(q.lb_sticky === expected.lb_sticky, "line buffer sticky");
    check(q.sp_sticky === expected.sp_sticky, "short packet sticky");
    check(q.dma_sticky === expected.dma_sticky, "DMA sticky");
    check(q.irq === expected.irq, "registered interrupt");
    check((^q) !== 1'bx, "unknown output");
    for (int j=6; j>0; j--) begin
      expected_history[j]=expected_history[j-1]; actual_history[j]=actual_history[j-1];
      uncertain_history[j]=uncertain_history[j-1];
      low_history[j]=low_history[j-1]; high_history[j]=high_history[j-1];
    end
    expected_history[0]=expected; actual_history[0]=q; uncertain_history[0]=uncertain;
    low_history[0]=low_abort; high_history[0]=high_abort;
    history_count++;
    if (history_count>=7) begin
      for (int b=0; b<24; b++) begin
        if (!SYNC_MASK[b]) begin
          match_bit=0;
          for (int j=0; j<7; j++)
            if (actual_history[3].flags[b] === expected_history[j].flags[b] ||
                uncertain_history[j][b]) match_bit=1;
          check(match_bit, $sformatf("CDC flag %0d outside timing window",b));
        end
      end
      match_count=0;
      for (int j=0; j<7; j++)
        if (actual_history[3].abort_count>=low_history[j] &&
            actual_history[3].abort_count<=high_history[j]) match_count=1;
      check(match_count, $sformatf("abort count=%0d range=%0d..%0d",
            actual_history[3].abort_count,low_history[3],high_history[3]));
    end
  endtask
  always @(negedge resetn) begin
    reset_model();
    #0.1;
    if (check_ready) check(q === '0, "asynchronous reset did not clear outputs");
  end
  always @(posedge sys_clk) begin
    cycles++;
    if (!resetn) begin
      reset_model(); reset_edges++;
      if (reset_edges>=2) check_ready=1;
      #0.1;
      if (check_ready) check(q === '0, "outputs during reset");
    end else begin
      model_tick();
      #0.1;
      if (check_ready) compare_outputs();
    end
  end

  realtime flag_rise [0:23];
  logic [23:0] last_flags = 0;
  always @(q.flags) begin
    for (int b=0; b<24; b++)
      if (q.flags[b] === 1'b1 && last_flags[b] !== 1'b1) flag_rise[b]=$realtime;
    last_flags=q.flags;
  end
  task automatic isolated_source(input int channel, lane = 0);
    realtime started, bound;
    int b;
    drain(); clear_flags();
    source_step();
    started=$realtime;
    case (channel)
      0: begin sot[lane]=1; b=9; end
      1: begin sotsync[lane]=1; b=10; end
      2: begin abort_pulse=1; b=19; end
    endcase
    source_step(); sot=0; sotsync=0; abort_pulse=0;
    drain();
    check(q.flags[b] === 1'b1, "isolated source event missing");
    bound=(BYTE_PS+(PARA_SYNC_STAGES+2)*SYS_PS)/1000.0;
    check(flag_rise[b]>=started &&
          flag_rise[b]-started<=bound+(BYTE_PS+SYS_PS)/2000.0+0.1,
          "isolated source latency");
  endtask

  // Reset all visible state, including while an event is in flight.
  task automatic test_reset_idle;
    step(20); check(q === '0, "idle after reset");
    step(); all_events(); s.fid0=1; s.line0=7;
    source_pulse(0);
    reset_all();
    drain(); check(q === '0, "reset with activity");
  endtask

  // Exercise individual sources and simultaneous compatible sources.
  task automatic test_event_map;
    for (int b=0; b<=28; b++) begin
      if (b!=9 && b!=10 && b!=11 && b!=19) begin
        clear_flags(); pulse_event(b);
        if (b<24) compare_word(q.flags,32'(24'b1<<b),"single event");
        if (b==24 || b==25) compare_word(q.flags,0,"counter/capture only event");
      end
    end
    clear_flags();
    step(); all_events(); s.vc=0;
    step(); quiet();
    step(); all_events(); s.vc=1;
    step(); quiet();
    source_pulse(0); drain(); source_pulse(1); drain(); source_pulse(2);
    #0.317; stop_state=1;
    drain(); compare_word(q.flags,32'hffffff,"all flags accumulated");
  endtask

  // Check set/clear priority, all enables, and the registered interrupt latency.
  task automatic test_irq_control;
    for (int b=0; b<24; b++) begin
      if (SYNC_MASK[b]) begin
        clear_flags();
        step(); s.global_en=1; s.mask=24'b1<<b; drive_event(b);
        step(); quiet(); check(q.flags[b]===1'b1,"flag latency");
        check(q.irq===1'b0,"interrupt early");
        step(); check(q.irq===1'b1,"interrupt late");
        s.clear=24'b1<<b; drive_event(b);
        step(); quiet(); check(q.flags[b]===1'b1,"set must win clear");
        s.clear=24'b1<<b;
        step(); quiet(); check(q.flags[b]===1'b0,"clear latency");
        check(q.irq===1'b1,"clear interrupt early");
        step(); check(q.irq===1'b0,"clear interrupt late");
        clear_flags(24'b1<<b);
      end
    end
    for (int k=0; k<200; k++) begin
      step(); quiet();
      s.mask=$urandom(); s.global_en=$urandom_range(1,0);
      s.clear=$urandom(); s.vc=$urandom_range(1,0);
      s.crc=$urandom_range(1,0); s.ecc1=$urandom_range(1,0);
      s.dma_ok=$urandom_range(1,0); s.frame_level=$urandom_range(1,0);
    end
    step(); quiet(); s.global_en=0; s.mask=0;
    pulse_event(4);
    step(); s.mask=24'h10; s.global_en=1;
    step(); check(q.irq===1'b1,"enable pending flag");
    s.soft_reset=1;
    step(); check(q.irq===1'b0,"soft reset interrupt latency");
    s.soft_reset=0; step(PARA_RST_TAIL+4);
    for (int c=0; c<3; c++) begin
      isolated_source(c);
      clear_flags();
      isolated_source(c);
      drain(); clear_flags();
    end
    #0.317; stop_state=1; drain(); clear_flags();
    step(8); check(q.flags[11]===1'b0,"held stop state after clear");
  endtask

  // Edge detection uses the qualified watermark and each overflow independently.
  task automatic test_edge_sticky;
    step(); s.select_tpg=1; s.full=1; step(4);
    check(q.flags[12]===1'b0,"TPG watermark");
    s.select_tpg=0; step(4); check(q.flags[12]===1'b1,"select falling while full");
    clear_flags(); s.select_tpg=1; step(4);
    check(q.flags[12]===1'b0,"select rising while full");
    s.select_tpg=0; step(3); clear_flags();
    for (int b=0; b<4; b++) begin
      step(); quiet();
      case (b)
        0: s.lb_overflow=1;
        1: s.sp_overflow=1;
        2: s.meta_overflow=1;
        3: s.dma_overflow=1;
      endcase
      step(5); clear_flags(24'h2000); step(5);
      check(q.flags[13]===1'b0,"overflow high after clear");
    end
    step(); quiet(); step();
    s.lb_overflow=1; s.sp_overflow=1; s.meta_overflow=1; s.dma_overflow=1;
    step(); quiet(); step(3);
    check({q.lb_sticky,q.sp_sticky,q.dma_sticky}===3'b111,"all stickies");
    clear_flags(); check({q.lb_sticky,q.sp_sticky,q.dma_sticky}===3'b111,"sticky W1C");
    pulse_event(22); compare_word(q.flags,32'h400000,"format bridge isolation");
  endtask

  // Keep consecutive pulse semantics and reach both 16-bit ECC limits through the ports.
  task automatic test_event_counters;
    step(); s.ecc1=1; s.ecc2=1; s.packet=1; s.drop=1; s.crc=1;
    step(65540); quiet(); step(8);
    compare_word(q.ecc,32'hffffffff,"ECC saturation");
    compare_word(q.packets,65540,"packet consecutive");
    compare_word(q.drop,65540,"drop consecutive");
    compare_word(q.crc,65540,"CRC consecutive");
    soft_clear(); check(q === '0,"soft reset saturated ECC");
  endtask

  // Reach the lane-abort limit with real, legally spaced source pulses.
  task automatic test_lane_abort_saturation;
    for (int n=0; n<65540; n++) begin
      source_pulse(2); source_step(ABORT_GAP-2);
    end
    drain(); compare_word(q.abort_count,65535,"abort saturation");
    check(q.flags[19]===1'b1,"abort saturation flag");
    soft_clear(); compare_word(q.abort_count,0,"soft reset saturated abort");
  endtask

  // Each lane, simultaneous lanes, skew, and a pending burst use the source clock.
  task automatic test_source_events;
    for (int c=0; c<2; c++) begin
      for (int lane=0; lane<PARA_MAX_LANES; lane++) isolated_source(c,lane);
      clear_flags(); source_step();
      if (c==0) sot='1; else sotsync='1;
      source_step(); sot=0; sotsync=0; drain();
      check(q.flags[c+9]===1'b1,"simultaneous lanes");
      for (int skew=1; skew<=2; skew++) begin
        clear_flags();
        source_step();
        if (c==0) sot[0]=1; else sotsync[0]=1;
        source_step(); sot=0; sotsync=0;
        if (skew==2 || PARA_MAX_LANES==1) source_step();
        if (c==0) sot[PARA_MAX_LANES-1]=1; else sotsync[PARA_MAX_LANES-1]=1;
        source_step(); sot=0; sotsync=0;
        drain(); check(q.flags[c+9]===1'b1,"skewed lane event");
      end
      clear_flags();
      repeat (12) source_pulse(c);
      drain(); check(q.flags[c+9]===1'b1,"pending burst");
      clear_flags(); drain(); check(q.flags[c+9]===1'b0,"drained burst");
    end
  endtask

  // Every legally spaced abort is delivered once, independent of the flag state.
  task automatic test_lane_abort_transfer;
    isolated_source(2);
    for (int n=0; n<40; n++) begin
      source_step(ABORT_GAP-2);
      source_pulse(2);
    end
    drain(); compare_word(q.abort_count,41,"abort transfer total");
    clear_flags(); drain(); compare_word(q.abort_count,41,"no extra abort");
  endtask

  // Stop the source after launch, then with a queued event; no hierarchy is inspected.
  task automatic test_byte_clock_stop;
    for (int queued=0; queued<2; queued++) begin
      clear_flags(); source_pulse(0);
      if (queued) source_pulse(0);
      byte_run=0;
      drain(); check(q.flags[9]===1'b1,"delivery while source clock stopped");
      clear_flags(); step(20);
      check(q.flags[9]===1'b0,"no repeat while source clock stopped");
      byte_run=1; source_step(4*PARA_SYNC_STAGES+12); drain();
      if (queued) check(q.flags[9]===1'b1,"pending after resume");
      else check(q.flags[9]===1'b0,"no pending after resume");
      clear_flags();
    end
  endtask

  // A stopped source domain stays reset until its clock returns.
  task automatic test_reset_stopped_clock;
    source_step(); byte_run=0;
    reset_all(0);
    check(source_resetn===1'b0,"source reset released without clock");
    step(30); check(q === '0,"reset with stopped clock");
    byte_run=1; wait(source_resetn===1'b1);
    source_step(3); isolated_source(0);
  endtask

  // Stop-state transitions are asynchronous and only rising edges produce events.
  task automatic test_stop_state;
    realtime started;
    for (int n=0; n<12; n++) begin
      clear_flags();
      #($urandom_range(900,101)/1000.0); started=$realtime; stop_state=1;
      step(PARA_SYNC_STAGES+5);
      check(q.flags[11]===1'b1,"stop rise");
      check(flag_rise[11]-started<=(PARA_SYNC_STAGES+2.5)*SYS_PS/1000.0+0.1,
            "stop rise latency");
      clear_flags(); #0.273; stop_state=0; drain();
      check(q.flags[11]===1'b0,"stop falling edge");
    end
    reset_all();
    step(); raw_resetn=0; #0.3; stop_state=1;
    step(4); raw_resetn=1; drain();
    check(q.flags[11]===1'b1,"stop high at reset release");
    clear_flags(); #0.2; stop_state=0; drain();
    step(); s.phy_reset=1; #0.3; stop_state=1;
    drain(); check(q.flags[11]===1'b0,"stop blocked");
    step(); s.phy_reset=0; step(PARA_RST_TAIL+10);
  endtask

  // Probe both sides of the exact reset-tail boundary for every synchronous source.
  task automatic test_reset_gate;
    for (int kind=0; kind<2; kind++) begin
      for (int b=0; b<=28; b++) begin
        if (b!=9 && b!=10 && b!=11 && b!=19) begin
          for (int boundary=0; boundary<2; boundary++) begin
            soft_clear(); clear_flags();
            step(); s.fid0=14'(b+1); s.fid1=14'(b+1); s.line0=16'(b+7);
            if (kind==0) s.soft_reset=1; else s.phy_reset=1;
            step(); s.soft_reset=0; s.phy_reset=0;
            step(PARA_RST_TAIL-1+boundary); drive_event(b);
            step(); quiet();
            if (boundary==0) check((q.flags & SYNC_MASK)===0,"last blocked cycle");
            else if (b<24) check(q.flags[b]===1'b1,"first open event");
            step(3);
          end
        end
      end
    end
    soft_clear(); pulse_event(4);
    step(); s.phy_reset=1; s.clear='1; s.fid0=91;
    step(5); quiet(); s.fid1=91; s.phy_reset=0;
    step(PARA_RST_TAIL/2); s.soft_reset=1;
    step(3); s.phy_reset=1; s.soft_reset=0;
    step(4); s.phy_reset=0; s.fid0=92;
    step(PARA_RST_TAIL-1); s.crc=1;
    step(); s.crc=0; check(q.flags===0,"retrigger reload");
    step(); s.crc=1; s.vc=0;
    step(); quiet(); step(3);
    check(q.capture[29:16]===14'd92,"capture tracking during gate");
    soft_clear();
    source_step(); sot[0]=1;
    fork
      begin source_step(); sot=0; end
      begin @(posedge byte_clk); step(); s.phy_reset=1; end
    join
    source_pulse(1); step(); s.lb_overflow=1; s.sp_overflow=1; s.dma_overflow=1;
    step(10); s.phy_reset=0; step(PARA_RST_TAIL+20);
    check(q.flags===0,"in-flight and level events blocked");
    check({q.lb_sticky,q.sp_sticky,q.dma_sticky}===0,"blocked stickies");
    step(); quiet();
    for (int c=0; c<3; c++) begin
      for (int side=0; side<3; side++) begin
        reset_all(); step(); s.phy_reset=1;
        step(3); s.phy_reset=0;
        step(PARA_RST_TAIL-2*CROSS_CYC+side*2*CROSS_CYC);
        source_pulse(c); drain();
        if (side==0) check(q.flags[c==2 ? 19 : c+9]===1'b0,"CDC before gate end");
        if (side==2) check(q.flags[c==2 ? 19 : c+9]===1'b1,"CDC after gate end");
        step(PARA_RST_TAIL+4);
      end
    end
  endtask

  // Soft reset clears records and arms while preserving previous input values.
  task automatic test_soft_reset;
    for (int state=0; state<3; state++) begin
      reset_all();
      pulse_event(5); pulse_event(14); pulse_event(24);
      pulse_event(13); pulse_event(26); pulse_event(28);
      step(); s.fid0=10; s.line0=21; step();
      if (state>0) begin s.fid0=11; s.fid1=10; step(); end
      if (state==2) begin s.vc=0; s.crc=1; step(); quiet(); end
      step(); all_events(); s.clear='1; s.fid0=12; s.fid1=12; s.soft_reset=1;
      step(); s.soft_reset=0; s.clear=0;
      step(PARA_RST_TAIL+2); quiet(); step(3);
      compare_word(q.capture,0,"frame history survives soft reset");
      check({q.lb_sticky,q.sp_sticky,q.dma_sticky}===0,"levels survive soft reset");
      step(); s.crc=1; s.vc=0; step(); quiet(); step(3);
      compare_word(q.capture,0,"unchanged frame not reopened");
      step(); s.fid0=13; s.crc=1;
      step(); quiet(); step(3);
      check(q.capture[29:16]===14'd13,"new capture after soft reset");
    end
  endtask

  task automatic capture_trigger(input int k, input int vc);
    s.vc=2'(vc);
    case(k)
      0: s.frame_sync=1;
      1: s.crc=1;
      2: s.ecc2=1;
      3: s.wc=1;
      4: s.lane_error=1;
    endcase
  endtask
  task automatic new_frame(input int a, b);
    step(); quiet(); s.fid0=14'(a); s.fid1=14'(b); step();
  endtask
  task automatic trigger_once(input int k, vc);
    step(); quiet(); capture_trigger(k,vc);
    step(); quiet(); step(2);
  endtask

  // Each trigger field is checked against hand-computed port values.
  task automatic test_capture_fields;
    logic [31:0] want;
    for (int k=0; k<5; k++) begin
      for (int vc=0; vc<2; vc++) begin
        reset_all(); trigger_once(k,vc);
        compare_word(q.capture,0,"trigger before first capture");
        new_frame(10,10); step(); s.line0=16'h1357; s.line1=16'h2468;
        trigger_once(k,vc);
        if (k==2 || k==4) want={2'b11,14'd10,16'hffff};
        else want={2'(vc),14'd10,(vc==0 ? 16'h1357 : 16'h2468)};
        compare_word(q.capture,want,"capture fields");
      end
    end
    reset_all(); new_frame(10,0); trigger_once(1,1);
    compare_word(q.capture,0,"orphan channel");
    for (int b=0; b<29; b++) begin
      if (b!=1 && b!=3 && b!=4 && b!=6 && b!=8 && b!=9 &&
          b!=10 && b!=11 && b!=19 && b!=25) pulse_event(b);
    end
    source_pulse(0); drain(); source_pulse(1); drain(); source_pulse(2);
    #0.3; stop_state=1; drain();
    compare_word(q.capture,0,"non-trigger sources");
  endtask

  // Capture age precedes trigger index; newest first-error records remain immutable.
  task automatic test_capture_order;
    logic [31:0] saved;
    new_frame(10,10); step(); s.line0=8; trigger_once(1,0); saved=q.capture;
    step(); s.line0=9; trigger_once(0,0);
    compare_word(q.capture,saved,"first error held");
    new_frame(11,10); trigger_once(1,1);
    compare_word(q.capture,saved,"previous already held");
    trigger_once(3,0); check(q.capture[29:16]===14'd11,"newest replaces previous");
    saved=q.capture; trigger_once(0,1); compare_word(q.capture,saved,"previous cannot replace");
    reset_all(); new_frame(20,20); new_frame(21,20);
    step(); s.line1=27; trigger_once(1,1);
    compare_word(q.capture,{2'b01,14'd20,16'd27},"previous while both armed");
    step(); s.line0=31; trigger_once(1,0);
    compare_word(q.capture,{2'b00,14'd21,16'd31},"newest after previous");
    saved=q.capture; new_frame(21,21); trigger_once(1,1);
    compare_word(q.capture,saved,"lag join does not rearm");
    new_frame(22,22); trigger_once(1,1);
    check(q.capture[29:16]===14'd22,"slot reuse once");
    new_frame(16383,22); trigger_once(1,0);
    new_frame(1,16383); trigger_once(1,0);
    check(q.capture[29:16]===14'd1,"frame ID wrap");
    new_frame(2,2); trigger_once(0,0); saved=q.capture;
    step(); s.capture_done=1; step(); quiet(); trigger_once(1,0);
    compare_word(q.capture,saved,"between frame end and start");
    new_frame(3,2); trigger_once(0,1);
    compare_word(q.capture,saved,"frame sync still belongs to old frame");
    for (int n=0; n<40; n++) begin
      reset_all(); new_frame(100+n,100+n); new_frame(101+n,100+n);
      step(); s.line1=16'($urandom()); s.vc=1;
      s.frame_sync=1; s.crc=1; s.ecc2=1; s.wc=1; s.lane_error=1;
      step(); quiet(); step(2);
      compare_word(q.capture,{2'b11,14'(101+n),16'hffff},"newest wins trigger priority");
      reset_all();
      step(); s.fid0=14'(201+n); s.fid1=14'(201+n); s.line0=16'($urandom());
      s.frame_sync=1; s.crc=1; s.ecc2=1; s.wc=1; s.lane_error=1;
      saved={2'b00,14'(201+n),s.line0};
      step(); quiet(); step(2);
      compare_word(q.capture,saved,"new capture and lowest trigger");
      reset_all(); new_frame(301+n,301+n);
      step(); s.ecc2=1; s.wc=1; s.vc=0;
      step(); quiet(); step(2);
      compare_word(q.capture,{2'b11,14'(301+n),16'hffff},"unknown before word count");
    end
  endtask

  // Constrained random traffic runs independently in both clock domains.
  task automatic test_random_events;
    bit source_done;
    int fid;
    source_done=0; fid=1;
    fork
      begin
        int gap, source_ticks;
        gap=ABORT_GAP; source_ticks=0;
        while (!source_done) begin
          source_step();
          sot=0; sotsync=0; abort_pulse=0; source_ticks++;
          if ((source_ticks%2)==0 && $urandom_range(3,0)==0)
            sot=$bits(sot)'($urandom());
          if ((source_ticks%2)==0 && $urandom_range(3,0)==0)
            sotsync=$bits(sotsync)'($urandom());
          gap++;
          if (gap>=ABORT_GAP && $urandom_range(15,0)==0) begin
            abort_pulse=1; gap=0;
          end
        end
        source_step(); sot=0; sotsync=0; abort_pulse=0;
      end
      begin
        for (int n=0; n<20000; n++) begin
          step(); quiet();
          s.vc=$urandom_range(1,0); s.global_en=$urandom_range(1,0); s.mask=$urandom();
          s.clear=$urandom(); s.soft_reset=($urandom_range(499,0)==0);
          s.phy_reset=($urandom_range(199,0)==0);
          s.frame_sync=$urandom_range(1,0); s.frame_level=$urandom_range(1,0);
          s.crc=$urandom_range(1,0); s.ecc1=$urandom_range(1,0); s.ecc2=$urandom_range(1,0);
          s.wc=$urandom_range(1,0); s.unsupported=$urandom_range(1,0);
          s.lane_error=$urandom_range(1,0); s.packet=$urandom_range(1,0);
          s.drop=$urandom_range(1,0); s.capture_done=$urandom_range(1,0);
          s.full=$urandom_range(1,0); s.select_tpg=$urandom_range(1,0);
          s.lb_overflow=$urandom_range(1,0); s.sp_overflow=$urandom_range(1,0);
          s.meta_overflow=$urandom_range(1,0); s.dma_overflow=$urandom_range(1,0);
          s.fb_overflow=$urandom_range(1,0); s.wc_align=$urandom_range(1,0);
          s.resolution=$urandom_range(1,0); s.frame_num=$urandom_range(1,0);
          s.dma_ok=$urandom_range(1,0); s.dma_err=$urandom_range(1,0);
          s.axi_error=$urandom_range(1,0); s.line0=16'($urandom()); s.line1=16'($urandom());
          if ($urandom_range(31,0)==0) begin fid++; s.fid0=14'(fid); end
          else if ($urandom_range(7,0)==0) s.fid1=s.fid0;
          if ((n%2000)==1000) begin
            source_step(); byte_run=0; step(25); byte_run=1;
          end
        end
        step(); quiet(); s.soft_reset=0; s.phy_reset=0; source_done=1;
      end
    join
    step(PARA_RST_TAIL+10); drain();
  endtask

  // Only the two explicitly exercised channel assumptions are suspended.
  task automatic test_input_robustness;
    robust=1;
    for (int vc=2; vc<4; vc++) begin
      for (int k=0; k<5; k++) begin
        step(); quiet(); s.vc=2'(vc);
        case(k)
          0: s.frame_level=1;
          1: s.frame_sync=1;
          2: s.crc=1;
          3: s.wc=1;
          4: s.unsupported=1;
        endcase
        step(); quiet(); step(3);
      end
    end
    step(); s.fid0=101; s.fid1=202; s.line0=17; s.line1=29; s.ecc2=1;
    step(); quiet(); step(3);
    compare_word(q.capture,{2'b11,14'd101,16'hffff},"simultaneous IDs use VC0");
    robust=0;
  endtask

  // Held levels clear once, and each level is detected at the first released edge.
  task automatic test_level_clear_reset;
    for (int b=0; b<3; b++) begin
      reset_all(); step();
      case(b)
        0: s.lb_overflow=1;
        1: s.sp_overflow=1;
        2: s.dma_overflow=1;
      endcase
      step(5); clear_flags(24'h2000); step(12);
      check(q.flags[13]===1'b0,"W1C while level remains high");
      step(); quiet(); step();
      case(b)
        0: s.lb_overflow=1;
        1: s.sp_overflow=1;
        2: s.dma_overflow=1;
      endcase
      step(3); check(q.flags[13]===1'b1,"new rise after W1C");
    end
    for (int b=0; b<5; b++) begin
      reset_all(); step(); raw_resetn=0; quiet(); step(4);
      case(b)
        0: s.full=1;
        1: s.lb_overflow=1;
        2: s.sp_overflow=1;
        3: s.meta_overflow=1;
        4: s.dma_overflow=1;
      endcase
      raw_resetn=1; step(8);
      check(q.flags[b==0 ? 12 : 13]===1'b1,"level high at reset release");
      clear_flags(); step(6); check(q.flags===0,"one event from held release level");
    end
  endtask

  // Deliberately corrupt the connected output, including an unknown value.
  task automatic test_checker_fault;
    step(); force q.irq=1'b1;
    step(3); release q.irq;
    step(); force q.flags=24'hxxxxxx;
    step(3); release q.flags;
    step(5);
  endtask

  initial begin : run_test
    void'($value$plusargs("TEST=%s",test_name));
    void'($value$plusargs("SEED=%d",seed));
    seed_init=seed; void'($urandom(seed_init));
    reset_model();
    if (test_name=="test_elaboration") begin
      #0.2;
      check(PARA_MAX_LANES inside {1,2,4},"legal lane count");
      check(PARA_SYNC_STAGES inside {2,3},"legal synchronizer length");
      check(PARA_RST_TAIL>=2*(PARA_SYNC_STAGES+2)+1 &&
            PARA_RST_TAIL<=65535,"legal tail length");
      check($bits(sot)==PARA_MAX_LANES && $bits(q.flags)==24,"interface dimensions");
      finish_test();
    end
    check(PARA_RST_TAIL*SYS_PS>=2*(PARA_SYNC_STAGES+2)*(BYTE_PS+SYS_PS),
          "reset tail too short for clocks");
    check((PARA_SYNC_STAGES+2)*(BYTE_PS+SYS_PS)<295000+10.0*BYTE_PS/8.0,
          "event transfer cannot preserve abort rate");
    check(CLK_CASE inside {0,1,4,22},"unknown clock pair");
    reset_all();
    case(test_name)
      "test_reset_idle": test_reset_idle();
      "test_event_map": test_event_map();
      "test_irq_control": test_irq_control();
      "test_edge_sticky": test_edge_sticky();
      "test_event_counters": test_event_counters();
      "test_lane_abort_saturation": test_lane_abort_saturation();
      "test_source_events": test_source_events();
      "test_lane_abort_transfer": test_lane_abort_transfer();
      "test_byte_clock_stop": test_byte_clock_stop();
      "test_reset_stopped_clock": test_reset_stopped_clock();
      "test_stop_state": test_stop_state();
      "test_reset_gate": test_reset_gate();
      "test_soft_reset": test_soft_reset();
      "test_capture_fields": test_capture_fields();
      "test_capture_order": test_capture_order();
      "test_random_events": test_random_events();
      "test_input_robustness": test_input_robustness();
      "test_level_clear_reset": test_level_clear_reset();
      "test_checker_fault": test_checker_fault();
      default: check(0,"unknown test name");
    endcase
    drain();
    finish_test();
  end
  initial begin
    #0.002;
    if (test_name=="test_lane_abort_saturation") #(SYS_PS*20000000.0/1000.0);
    else #(SYS_PS*1000000.0/1000.0);
    if (!done) begin check(0,"timeout"); finish_test(); end
  end
endmodule
`default_nettype wire
