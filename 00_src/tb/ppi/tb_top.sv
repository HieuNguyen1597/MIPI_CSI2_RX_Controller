// File        : tb_top.sv
// Description : Testbench of m_fpt_ppi (PHY Protocol Interface).
//               The block only wires signals through, so every output must equal its input;
//               lanes at or above PARA_MAX_LANES are not built and drive 0. A monitor checks
//               this after every input or output change, so tests only drive stimulus.
// Run         : +TEST=<name> +SEED=<n>; the last log line is "<name> PASS" or "<name> FAIL".
//
// Design assumptions (agreed with the block owner; the specification does not state them yet):
//   - The block has no clock, reset or storage: no soft reset, no active lane setting, and the
//     outputs follow the inputs with no delay, whether rxbyteclkhs runs or not.
//   - Lanes with an index of PARA_MAX_LANES or above are not built: their inputs are ignored and
//     their outputs, towards the core and towards the D-PHY, are 0.
//   - Signals listed as "declared, not consumed" (data lane ulpsactivenot, Escape mode signals,
//     Low-Power errors) are inputs of this block and end here; no output depends on them.
//   - Enables do not gate anything in this block; the D-PHY handles a disabled lane.
//   - rxactivehs rises when the lane enters High-Speed mode, before rxsynchs and the first byte.
//   - A burst with errsotsynchs has no rxsynchs; rxvalidhs either stays low or the PHY still
//     presents bytes with rxvalidhs high. Both forms are valid input; this block passes them on
//     unchanged and the core discards the burst.
//   - rxskewcalhs stays high after rxactivehs falls and drops when stopstate rises.
//   - The block has no logic, so an X on an input gives the same X on its output.
`timescale 1ns/1ps

module tb_top;

  parameter logic [31:0] PARA_MAX_LANES = 4;

  // Lane x is built when x < PARA_MAX_LANES.
  localparam logic [3:0] BUILT = (PARA_MAX_LANES == 32'd1) ? 4'b0001 :
                                 (PARA_MAX_LANES == 32'd2) ? 4'b0011 : 4'b1111;

  // Byte clock, 224 MHz.
  localparam real T_BYTE_CLK = 4.464;

  // DUT inputs; the vectors carry one bit per lane.
  logic        i_cl_enable, i_cl_stopstate, i_cl_ulpsactivenot, i_rxbyteclkhs, i_cl_rxclkactivehs, i_cl_ulpsclknot, i_phy_core_rst, i_init_done;
  logic [3:0]  i_dl_enable, i_forcerxmode, i_stopstate, i_ulpsactivenot, i_rxvalidhs, i_rxactivehs, i_rxsynchs, i_rxskewcalhs, i_rxclkesc, i_rxlpdtesc, i_rxulpsesc, i_rxvalidesc, i_errsoths, i_errsotsynchs, i_erresc, i_errsyncesc, i_errcontrol;
  logic [7:0]  i_rxdatahs [4];
  logic [3:0]  i_rxtriggeresc [4];
  logic [7:0]  i_rxdataesc [4];

  // DUT outputs.
  logic        o_cl_enable, o_cl_stopstate, o_cl_ulpsactivenot, o_rxbyteclkhs, o_cl_rxclkactivehs, o_cl_ulpsclknot, o_phy_core_rst, o_init_done;
  logic [3:0]  o_dl_enable, o_forcerxmode, o_stopstate, o_rxvalidhs, o_rxactivehs, o_rxsynchs, o_rxskewcalhs, o_errsoths, o_errsotsynchs;
  logic [7:0]  o_rxdatahs [4];

  m_fpt_ppi #(
    .PARA_MAX_LANES(PARA_MAX_LANES)
  ) u_dut (
    .i_cl_enable           (i_cl_enable),
    .i_cl_stopstate        (i_cl_stopstate),
    .i_cl_ulpsactivenot    (i_cl_ulpsactivenot),
    .i_rxbyteclkhs         (i_rxbyteclkhs),
    .i_cl_rxclkactivehs    (i_cl_rxclkactivehs),
    .i_cl_ulpsclknot       (i_cl_ulpsclknot),
    .i_phy_core_rst        (i_phy_core_rst),
    .i_init_done           (i_init_done),
    .o_cl_enable           (o_cl_enable),
    .o_cl_stopstate        (o_cl_stopstate),
    .o_cl_ulpsactivenot    (o_cl_ulpsactivenot),
    .o_rxbyteclkhs         (o_rxbyteclkhs),
    .o_cl_rxclkactivehs    (o_cl_rxclkactivehs),
    .o_cl_ulpsclknot       (o_cl_ulpsclknot),
    .o_phy_core_rst        (o_phy_core_rst),
    .o_init_done           (o_init_done),
    .i_dl_enable_0         (i_dl_enable[0]),
    .i_forcerxmode_0       (i_forcerxmode[0]),
    .i_stopstate_0         (i_stopstate[0]),
    .i_ulpsactivenot_0     (i_ulpsactivenot[0]),
    .i_rxvalidhs_0         (i_rxvalidhs[0]),
    .i_rxactivehs_0        (i_rxactivehs[0]),
    .i_rxsynchs_0          (i_rxsynchs[0]),
    .i_rxskewcalhs_0       (i_rxskewcalhs[0]),
    .i_rxclkesc_0          (i_rxclkesc[0]),
    .i_rxlpdtesc_0         (i_rxlpdtesc[0]),
    .i_rxulpsesc_0         (i_rxulpsesc[0]),
    .i_rxvalidesc_0        (i_rxvalidesc[0]),
    .i_errsoths_0          (i_errsoths[0]),
    .i_errsotsynchs_0      (i_errsotsynchs[0]),
    .i_erresc_0            (i_erresc[0]),
    .i_errsyncesc_0        (i_errsyncesc[0]),
    .i_errcontrol_0        (i_errcontrol[0]),
    .i_rxdatahs_0          (i_rxdatahs[0]),
    .i_rxtriggeresc_0      (i_rxtriggeresc[0]),
    .i_rxdataesc_0         (i_rxdataesc[0]),
    .o_dl_enable_0         (o_dl_enable[0]),
    .o_forcerxmode_0       (o_forcerxmode[0]),
    .o_stopstate_0         (o_stopstate[0]),
    .o_rxvalidhs_0         (o_rxvalidhs[0]),
    .o_rxactivehs_0        (o_rxactivehs[0]),
    .o_rxsynchs_0          (o_rxsynchs[0]),
    .o_rxskewcalhs_0       (o_rxskewcalhs[0]),
    .o_errsoths_0          (o_errsoths[0]),
    .o_errsotsynchs_0      (o_errsotsynchs[0]),
    .o_rxdatahs_0          (o_rxdatahs[0]),
    .i_dl_enable_1         (i_dl_enable[1]),
    .i_forcerxmode_1       (i_forcerxmode[1]),
    .i_stopstate_1         (i_stopstate[1]),
    .i_ulpsactivenot_1     (i_ulpsactivenot[1]),
    .i_rxvalidhs_1         (i_rxvalidhs[1]),
    .i_rxactivehs_1        (i_rxactivehs[1]),
    .i_rxsynchs_1          (i_rxsynchs[1]),
    .i_rxskewcalhs_1       (i_rxskewcalhs[1]),
    .i_rxclkesc_1          (i_rxclkesc[1]),
    .i_rxlpdtesc_1         (i_rxlpdtesc[1]),
    .i_rxulpsesc_1         (i_rxulpsesc[1]),
    .i_rxvalidesc_1        (i_rxvalidesc[1]),
    .i_errsoths_1          (i_errsoths[1]),
    .i_errsotsynchs_1      (i_errsotsynchs[1]),
    .i_erresc_1            (i_erresc[1]),
    .i_errsyncesc_1        (i_errsyncesc[1]),
    .i_errcontrol_1        (i_errcontrol[1]),
    .i_rxdatahs_1          (i_rxdatahs[1]),
    .i_rxtriggeresc_1      (i_rxtriggeresc[1]),
    .i_rxdataesc_1         (i_rxdataesc[1]),
    .o_dl_enable_1         (o_dl_enable[1]),
    .o_forcerxmode_1       (o_forcerxmode[1]),
    .o_stopstate_1         (o_stopstate[1]),
    .o_rxvalidhs_1         (o_rxvalidhs[1]),
    .o_rxactivehs_1        (o_rxactivehs[1]),
    .o_rxsynchs_1          (o_rxsynchs[1]),
    .o_rxskewcalhs_1       (o_rxskewcalhs[1]),
    .o_errsoths_1          (o_errsoths[1]),
    .o_errsotsynchs_1      (o_errsotsynchs[1]),
    .o_rxdatahs_1          (o_rxdatahs[1]),
    .i_dl_enable_2         (i_dl_enable[2]),
    .i_forcerxmode_2       (i_forcerxmode[2]),
    .i_stopstate_2         (i_stopstate[2]),
    .i_ulpsactivenot_2     (i_ulpsactivenot[2]),
    .i_rxvalidhs_2         (i_rxvalidhs[2]),
    .i_rxactivehs_2        (i_rxactivehs[2]),
    .i_rxsynchs_2          (i_rxsynchs[2]),
    .i_rxskewcalhs_2       (i_rxskewcalhs[2]),
    .i_rxclkesc_2          (i_rxclkesc[2]),
    .i_rxlpdtesc_2         (i_rxlpdtesc[2]),
    .i_rxulpsesc_2         (i_rxulpsesc[2]),
    .i_rxvalidesc_2        (i_rxvalidesc[2]),
    .i_errsoths_2          (i_errsoths[2]),
    .i_errsotsynchs_2      (i_errsotsynchs[2]),
    .i_erresc_2            (i_erresc[2]),
    .i_errsyncesc_2        (i_errsyncesc[2]),
    .i_errcontrol_2        (i_errcontrol[2]),
    .i_rxdatahs_2          (i_rxdatahs[2]),
    .i_rxtriggeresc_2      (i_rxtriggeresc[2]),
    .i_rxdataesc_2         (i_rxdataesc[2]),
    .o_dl_enable_2         (o_dl_enable[2]),
    .o_forcerxmode_2       (o_forcerxmode[2]),
    .o_stopstate_2         (o_stopstate[2]),
    .o_rxvalidhs_2         (o_rxvalidhs[2]),
    .o_rxactivehs_2        (o_rxactivehs[2]),
    .o_rxsynchs_2          (o_rxsynchs[2]),
    .o_rxskewcalhs_2       (o_rxskewcalhs[2]),
    .o_errsoths_2          (o_errsoths[2]),
    .o_errsotsynchs_2      (o_errsotsynchs[2]),
    .o_rxdatahs_2          (o_rxdatahs[2]),
    .i_dl_enable_3         (i_dl_enable[3]),
    .i_forcerxmode_3       (i_forcerxmode[3]),
    .i_stopstate_3         (i_stopstate[3]),
    .i_ulpsactivenot_3     (i_ulpsactivenot[3]),
    .i_rxvalidhs_3         (i_rxvalidhs[3]),
    .i_rxactivehs_3        (i_rxactivehs[3]),
    .i_rxsynchs_3          (i_rxsynchs[3]),
    .i_rxskewcalhs_3       (i_rxskewcalhs[3]),
    .i_rxclkesc_3          (i_rxclkesc[3]),
    .i_rxlpdtesc_3         (i_rxlpdtesc[3]),
    .i_rxulpsesc_3         (i_rxulpsesc[3]),
    .i_rxvalidesc_3        (i_rxvalidesc[3]),
    .i_errsoths_3          (i_errsoths[3]),
    .i_errsotsynchs_3      (i_errsotsynchs[3]),
    .i_erresc_3            (i_erresc[3]),
    .i_errsyncesc_3        (i_errsyncesc[3]),
    .i_errcontrol_3        (i_errcontrol[3]),
    .i_rxdatahs_3          (i_rxdatahs[3]),
    .i_rxtriggeresc_3      (i_rxtriggeresc[3]),
    .i_rxdataesc_3         (i_rxdataesc[3]),
    .o_dl_enable_3         (o_dl_enable[3]),
    .o_forcerxmode_3       (o_forcerxmode[3]),
    .o_stopstate_3         (o_stopstate[3]),
    .o_rxvalidhs_3         (o_rxvalidhs[3]),
    .o_rxactivehs_3        (o_rxactivehs[3]),
    .o_rxsynchs_3          (o_rxsynchs[3]),
    .o_rxskewcalhs_3       (o_rxskewcalhs[3]),
    .o_errsoths_3          (o_errsoths[3]),
    .o_errsotsynchs_3      (o_errsotsynchs[3]),
    .o_rxdatahs_3          (o_rxdatahs[3])
  );

  // --------------------------------------------------------------------------------------------
  // Clock and stimulus helpers
  // --------------------------------------------------------------------------------------------
  // The byte clock toggles while clk_run is 1.
  bit clk_run = 1'b1;

  initial begin
    i_rxbyteclkhs = 1'b0;
    forever begin
      #(T_BYTE_CLK / 2.0);
      if (clk_run) i_rxbyteclkhs = ~i_rxbyteclkhs;
    end
  end

  // Set every input except the clock to 0.
  task automatic clear_inputs();
    i_cl_enable = 1'b0;  i_cl_stopstate = 1'b0;  i_cl_ulpsactivenot = 1'b0;
    i_cl_rxclkactivehs = 1'b0;  i_cl_ulpsclknot = 1'b0;  i_phy_core_rst = 1'b0;  i_init_done = 1'b0;
    i_dl_enable = '0;  i_forcerxmode = '0;  i_stopstate = '0;  i_ulpsactivenot = '0;
    i_rxvalidhs = '0;  i_rxactivehs = '0;  i_rxsynchs = '0;  i_rxskewcalhs = '0;
    i_rxclkesc = '0;  i_rxlpdtesc = '0;  i_rxulpsesc = '0;  i_rxvalidesc = '0;
    i_errsoths = '0;  i_errsotsynchs = '0;  i_erresc = '0;  i_errsyncesc = '0;  i_errcontrol = '0;
    for (int l = 0; l < 4; l++) begin
      i_rxdatahs[l] = '0;  i_rxtriggeresc[l] = '0;  i_rxdataesc[l] = '0;
    end
  endtask

  // Randomize every input except the clock; sparse mode keeps most bits at 0.
  task automatic drive_random_all(input bit sparse);
    logic [31:0] r;
    r = $urandom;  i_cl_enable = r[0];  i_cl_stopstate = r[1];  i_cl_ulpsactivenot = r[2];
    i_cl_rxclkactivehs = r[3];  i_cl_ulpsclknot = r[4];  i_phy_core_rst = r[5];  i_init_done = r[6];
    i_dl_enable = 4'($urandom);  i_forcerxmode = 4'($urandom);  i_stopstate = 4'($urandom);
    i_ulpsactivenot = 4'($urandom);  i_rxvalidhs = 4'($urandom);  i_rxactivehs = 4'($urandom);
    i_rxsynchs = 4'($urandom);  i_rxskewcalhs = 4'($urandom);  i_rxclkesc = 4'($urandom);
    i_rxlpdtesc = 4'($urandom);  i_rxulpsesc = 4'($urandom);  i_rxvalidesc = 4'($urandom);
    i_errsoths = 4'($urandom);  i_errsotsynchs = 4'($urandom);  i_erresc = 4'($urandom);
    i_errsyncesc = 4'($urandom);  i_errcontrol = 4'($urandom);
    for (int l = 0; l < 4; l++) begin
      i_rxdatahs[l] = 8'($urandom);  i_rxtriggeresc[l] = 4'($urandom);  i_rxdataesc[l] = 8'($urandom);
    end
    if (sparse) begin
      i_dl_enable &= 4'($urandom);  i_forcerxmode &= 4'($urandom);  i_stopstate &= 4'($urandom);
      i_rxvalidhs &= 4'($urandom);  i_rxactivehs &= 4'($urandom);  i_rxsynchs &= 4'($urandom);
      i_rxskewcalhs &= 4'($urandom);  i_errsoths &= 4'($urandom);  i_errsotsynchs &= 4'($urandom);
    end
  endtask

  // Let the outputs settle.
  task automatic step();
    #6.0;
  endtask

  // Wait n rising clock edges, plus 0.5 ns.
  task automatic cycles(input int n);
    repeat (n) @(posedge i_rxbyteclkhs);
    #0.5;
  endtask

  // Raise an input for one step, then lower it.
  task automatic pulse(ref logic s);
    s = 1'b1;  step();
    s = 1'b0;  step();
  endtask

  // Same for one lane of a vector.
  task automatic pulse_lane(ref logic [3:0] v, input int lane);
    v[lane] = 1'b1;  step();
    v[lane] = 1'b0;  step();
  endtask

  // --------------------------------------------------------------------------------------------
  // Monitor: outputs against inputs
  // --------------------------------------------------------------------------------------------
  int unsigned err_count;
  int unsigned check_count;

  task automatic expect_bit(input string name, input int lane, input logic got, input logic exp);
    if (got !== exp) begin
      err_count++;
      if (err_count <= 20) $error("%s lane %0d: output %b, expected %b", name, lane, got, exp);
    end
  endtask

  task automatic expect_byte(input string name, input int lane, input logic [7:0] got,
                             input logic [7:0] exp);
    if (got !== exp) begin
      err_count++;
      if (err_count <= 20) $error("%s lane %0d: output %b, expected %b", name, lane, got, exp);
    end
  endtask

  task automatic check_outputs();
    check_count++;
    expect_bit("cl_enable", -1, o_cl_enable, i_cl_enable);
    expect_bit("cl_stopstate", -1, o_cl_stopstate, i_cl_stopstate);
    expect_bit("cl_ulpsactivenot", -1, o_cl_ulpsactivenot, i_cl_ulpsactivenot);
    expect_bit("rxbyteclkhs", -1, o_rxbyteclkhs, i_rxbyteclkhs);
    expect_bit("cl_rxclkactivehs", -1, o_cl_rxclkactivehs, i_cl_rxclkactivehs);
    expect_bit("cl_ulpsclknot", -1, o_cl_ulpsclknot, i_cl_ulpsclknot);
    expect_bit("phy_core_rst", -1, o_phy_core_rst, i_phy_core_rst);
    expect_bit("init_done", -1, o_init_done, i_init_done);
    for (int l = 0; l < 4; l++) begin
      expect_bit("dl_enable", l, o_dl_enable[l], BUILT[l] ? i_dl_enable[l] : 1'b0);
      expect_bit("forcerxmode", l, o_forcerxmode[l], BUILT[l] ? i_forcerxmode[l] : 1'b0);
      expect_bit("stopstate", l, o_stopstate[l], BUILT[l] ? i_stopstate[l] : 1'b0);
      expect_bit("rxvalidhs", l, o_rxvalidhs[l], BUILT[l] ? i_rxvalidhs[l] : 1'b0);
      expect_bit("rxactivehs", l, o_rxactivehs[l], BUILT[l] ? i_rxactivehs[l] : 1'b0);
      expect_bit("rxsynchs", l, o_rxsynchs[l], BUILT[l] ? i_rxsynchs[l] : 1'b0);
      expect_bit("rxskewcalhs", l, o_rxskewcalhs[l], BUILT[l] ? i_rxskewcalhs[l] : 1'b0);
      expect_bit("errsoths", l, o_errsoths[l], BUILT[l] ? i_errsoths[l] : 1'b0);
      expect_bit("errsotsynchs", l, o_errsotsynchs[l], BUILT[l] ? i_errsotsynchs[l] : 1'b0);
      expect_byte("rxdatahs", l, o_rxdatahs[l], BUILT[l] ? i_rxdatahs[l] : 8'h00);
    end
  endtask

  localparam int ALL_W = 156;
  logic [ALL_W-1:0] all_in;
  assign all_in = {i_cl_enable,
                     i_cl_stopstate,
                     i_cl_ulpsactivenot,
                     i_rxbyteclkhs,
                     i_cl_rxclkactivehs,
                     i_cl_ulpsclknot,
                     i_phy_core_rst,
                     i_init_done,
                     i_dl_enable,
                     i_forcerxmode,
                     i_stopstate,
                     i_ulpsactivenot,
                     i_rxvalidhs,
                     i_rxactivehs,
                     i_rxsynchs,
                     i_rxskewcalhs,
                     i_rxclkesc,
                     i_rxlpdtesc,
                     i_rxulpsesc,
                     i_rxvalidesc,
                     i_errsoths,
                     i_errsotsynchs,
                     i_erresc,
                     i_errsyncesc,
                     i_errcontrol,
                     i_rxdatahs[0],
                     i_rxdatahs[1],
                     i_rxdatahs[2],
                     i_rxdatahs[3],
                     i_rxtriggeresc[0],
                     i_rxtriggeresc[1],
                     i_rxtriggeresc[2],
                     i_rxtriggeresc[3],
                     i_rxdataesc[0],
                     i_rxdataesc[1],
                     i_rxdataesc[2],
                     i_rxdataesc[3]};

  // Outputs, so that an output that moves on its own is also caught.
  localparam int ALL_W_OUT = 76;
  logic [ALL_W_OUT-1:0] all_out;
  assign all_out = {o_cl_enable, o_cl_stopstate, o_cl_ulpsactivenot, o_rxbyteclkhs,
                    o_cl_rxclkactivehs, o_cl_ulpsclknot, o_phy_core_rst, o_init_done,
                    o_dl_enable, o_forcerxmode, o_stopstate, o_rxvalidhs, o_rxactivehs,
                    o_rxsynchs, o_rxskewcalhs, o_errsoths, o_errsotsynchs,
                    o_rxdatahs[0], o_rxdatahs[1], o_rxdatahs[2], o_rxdatahs[3]};

  always @(all_in or all_out) begin
    #0.05;
    check_outputs();
  end

  // --------------------------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------------------------

  // Enables, force-rx-mode and PHY reset: each alone, then several together.
  task automatic test_control_path();
    clear_inputs();
    pulse(i_cl_enable);
    pulse(i_phy_core_rst);
    for (int l = 0; l < 4; l++) begin
      pulse_lane(i_dl_enable, l);
      pulse_lane(i_forcerxmode, l);
    end
    i_cl_enable = 1'b1;  i_dl_enable = '1;  i_forcerxmode = '1;  i_phy_core_rst = 1'b1;  step();
    i_dl_enable = 4'b1010;  step();
    i_forcerxmode = 4'b0101;  step();
    clear_inputs();
    step();
  endtask

  // Stop State, ULPS, clock lane activity and init_done: each alone.
  task automatic test_state_path();
    clear_inputs();
    pulse(i_cl_stopstate);
    pulse(i_cl_ulpsactivenot);
    pulse(i_cl_rxclkactivehs);
    pulse(i_cl_ulpsclknot);
    pulse(i_init_done);
    for (int l = 0; l < 4; l++) begin
      pulse_lane(i_stopstate, l);
      pulse_lane(i_ulpsactivenot, l);
    end
    i_cl_stopstate = 1'b1;  i_stopstate = '1;  i_cl_rxclkactivehs = 1'b1;  i_init_done = 1'b1;  step();
    clear_inputs();
    step();
  endtask

  // Walking ones and zeros on rxdatahs with the lane qualifiers high, per lane and on all lanes.
  task automatic test_hs_data_walking();
    clear_inputs();
    for (int l = 0; l < 4; l++) begin
      i_rxvalidhs[l] = 1'b1;  i_rxactivehs[l] = 1'b1;  i_rxsynchs[l] = 1'b1;  i_rxskewcalhs[l] = 1'b1;
      for (int b = 0; b < 8; b++) begin
        i_rxdatahs[l] = 8'(1 << b);   step();
        i_rxdatahs[l] = ~8'(1 << b);  step();
      end
      i_rxdatahs[l] = 8'h00;
      i_rxvalidhs[l] = 1'b0;  i_rxactivehs[l] = 1'b0;  i_rxsynchs[l] = 1'b0;  i_rxskewcalhs[l] = 1'b0;
    end
    i_rxvalidhs = '1;  i_rxactivehs = '1;
    for (int b = 0; b < 8; b++) begin
      for (int l = 0; l < 4; l++) i_rxdatahs[l] = 8'(1 << b);
      step();
      for (int l = 0; l < 4; l++) i_rxdatahs[l] = ~8'(1 << b);
      step();
    end
    clear_inputs();
    step();
  endtask

  // One lane differs from the other three, then random values per lane.
  task automatic test_lane_independence();
    logic [3:0] sel;
    clear_inputs();
    for (int l = 0; l < 4; l++) begin
      for (int pol = 0; pol < 2; pol++) begin
        sel = 4'b0001 << l;
        if (pol == 1) sel = ~sel;
        i_dl_enable = sel;  i_forcerxmode = sel;  i_stopstate = sel;  i_rxvalidhs = sel;
        i_rxactivehs = sel;  i_rxsynchs = sel;  i_rxskewcalhs = sel;  i_errsoths = sel;
        i_errsotsynchs = sel;
        for (int k = 0; k < 4; k++) i_rxdatahs[k] = sel[k] ? 8'hA5 : 8'h5A;
        step();
      end
    end
    for (int n = 0; n < 200; n++) begin
      i_dl_enable = 4'($urandom);  i_forcerxmode = 4'($urandom);  i_stopstate = 4'($urandom);
      i_rxvalidhs = 4'($urandom);  i_rxactivehs = 4'($urandom);  i_rxsynchs = 4'($urandom);
      i_rxskewcalhs = 4'($urandom);  i_errsoths = 4'($urandom);  i_errsotsynchs = 4'($urandom);
      for (int k = 0; k < 4; k++) i_rxdatahs[k] = 8'($urandom);
      step();
    end
    clear_inputs();
    step();
  endtask

  // SoT error pulses (one clock cycle) per lane, on all lanes and back to back.
  task automatic test_hs_error_pulses();
    clear_inputs();
    cycles(2);
    for (int l = 0; l < 4; l++) begin
      i_errsoths[l] = 1'b1;     cycles(1);  i_errsoths[l] = 1'b0;     cycles(2);
      i_errsotsynchs[l] = 1'b1; cycles(1);  i_errsotsynchs[l] = 1'b0; cycles(2);
      i_errsoths[l] = 1'b1;  i_errsotsynchs[l] = 1'b1;  cycles(1);
      i_errsoths[l] = 1'b0;  i_errsotsynchs[l] = 1'b0;  cycles(2);
    end
    i_errsoths = '1;  i_errsotsynchs = '1;  cycles(1);
    i_errsoths = '0;  i_errsotsynchs = '0;  cycles(2);
    for (int n = 0; n < 8; n++) begin
      i_errsoths = 4'($urandom);  i_errsotsynchs = 4'($urandom);  cycles(1);
    end
    clear_inputs();
    cycles(2);
  endtask

  // Lanes that are not built stay at 0 whatever their inputs do.
  task automatic test_unbuilt_lanes_driven_low();
    clear_inputs();
    for (int l = 0; l < 4; l++) begin
      i_dl_enable = 4'b0001 << l;  i_forcerxmode = 4'b0001 << l;  i_stopstate = 4'b0001 << l;
      i_rxvalidhs = 4'b0001 << l;  i_rxactivehs = 4'b0001 << l;  i_rxsynchs = 4'b0001 << l;
      i_rxskewcalhs = 4'b0001 << l;  i_errsoths = 4'b0001 << l;  i_errsotsynchs = 4'b0001 << l;
      for (int k = 0; k < 4; k++) i_rxdatahs[k] = (k == l) ? 8'hFF : 8'h00;
      step();
    end
    i_dl_enable = '1;  i_forcerxmode = '1;  i_stopstate = '1;  i_rxvalidhs = '1;  i_rxactivehs = '1;
    i_rxsynchs = '1;  i_rxskewcalhs = '1;  i_errsoths = '1;  i_errsotsynchs = '1;
    for (int k = 0; k < 4; k++) i_rxdatahs[k] = 8'hFF;
    step();
    clear_inputs();
    step();
  endtask

  // Escape mode and low-power error inputs toggle at random; no output may react.
  task automatic test_unused_signals_ignored();
    clear_inputs();
    for (int pattern = 0; pattern < 5; pattern++) begin
      i_dl_enable = 4'($urandom);  i_stopstate = 4'($urandom);  i_rxvalidhs = 4'($urandom);
      i_rxactivehs = 4'($urandom);  i_errsoths = 4'($urandom);
      for (int k = 0; k < 4; k++) i_rxdatahs[k] = 8'($urandom);
      for (int n = 0; n < 100; n++) begin
        i_rxclkesc = 4'($urandom);  i_rxlpdtesc = 4'($urandom);  i_rxulpsesc = 4'($urandom);
        i_rxvalidesc = 4'($urandom);  i_erresc = 4'($urandom);  i_errsyncesc = 4'($urandom);
        i_errcontrol = 4'($urandom);  i_ulpsactivenot = 4'($urandom);
        for (int k = 0; k < 4; k++) begin
          i_rxtriggeresc[k] = 4'($urandom);  i_rxdataesc[k] = 8'($urandom);
        end
        #3.0;
      end
    end
    clear_inputs();
    step();
  endtask

  // No logic in the block: X in gives X out, and known values never give X.
  task automatic test_four_state_inputs();
    for (int base = 0; base < 2; base++) begin
      clear_inputs();
      if (base == 1) begin
        i_cl_enable = 1'b1;  i_cl_stopstate = 1'b1;  i_cl_ulpsactivenot = 1'b1;
        i_cl_rxclkactivehs = 1'b1;  i_cl_ulpsclknot = 1'b1;  i_phy_core_rst = 1'b1;
        i_init_done = 1'b1;  i_dl_enable = '1;  i_forcerxmode = '1;  i_stopstate = '1;
        i_rxvalidhs = '1;  i_rxactivehs = '1;  i_rxsynchs = '1;  i_rxskewcalhs = '1;
        i_errsoths = '1;  i_errsotsynchs = '1;
        for (int k = 0; k < 4; k++) i_rxdatahs[k] = 8'hFF;
      end
      step();
      i_cl_enable = 1'bx;  step();  i_cl_stopstate = 1'bx;  step();  i_cl_ulpsactivenot = 1'bx;  step();
      i_cl_rxclkactivehs = 1'bx;  step();  i_cl_ulpsclknot = 1'bx;  step();  i_phy_core_rst = 1'bx;  step();
      i_init_done = 1'bx;  step();
      for (int l = 0; l < 4; l++) begin
        i_dl_enable[l] = 1'bx;  step();  i_forcerxmode[l] = 1'bx;  step();  i_stopstate[l] = 1'bx;  step();
        i_rxvalidhs[l] = 1'bx;  step();  i_rxactivehs[l] = 1'bx;  step();  i_rxsynchs[l] = 1'bx;  step();
        i_rxskewcalhs[l] = 1'bx;  step();  i_errsoths[l] = 1'bx;  step();  i_errsotsynchs[l] = 1'bx;  step();
        i_rxdatahs[l] = 8'hxx;  step();
        i_rxdatahs[l] = 8'b10x0_x101;  step();
        i_ulpsactivenot[l] = 1'bx;  step();  i_rxclkesc[l] = 1'bx;  step();  i_erresc[l] = 1'bx;  step();
      end
    end
    clear_inputs();
    step();
  endtask

  // Inputs change at random times, at any phase of the byte clock.
  task automatic test_async_changes();
    clear_inputs();
    for (int n = 0; n < 3000; n++) begin
      #(0.1 * $urandom_range(10, 370));
      case ($urandom_range(0, 14))
        0:  i_cl_stopstate = $urandom;
        1:  i_cl_ulpsactivenot = $urandom;
        2:  i_cl_ulpsclknot = $urandom;
        3:  i_cl_rxclkactivehs = $urandom;
        4:  i_init_done = $urandom;
        5:  i_stopstate = 4'($urandom);
        6:  i_dl_enable = 4'($urandom);
        7:  i_forcerxmode = 4'($urandom);
        8:  i_errsoths = 4'($urandom);
        9:  i_errsotsynchs = 4'($urandom);
        10: i_rxvalidhs = 4'($urandom);
        11: i_rxactivehs = 4'($urandom);
        12: i_rxsynchs = 4'($urandom);
        13: i_rxskewcalhs = 4'($urandom);
        default: i_rxdatahs[$urandom_range(0, 3)] = 8'($urandom);
      endcase
    end
    clear_inputs();
    step();
  endtask

  // One transmission on lane l, from Stop State to Stop State, after "lead" cycles.
  //   kind 0: normal burst
  //   kind 1: errsoths pulsed with rxsynchs
  //   kind 2: errsotsynchs pulsed, rxvalidhs stays low
  //   kind 3: errsotsynchs pulsed, no rxsynchs, bytes still presented with rxvalidhs high
  //   kind 4: deskew calibration, rxskewcalhs held past the end of the burst
  // rxvalidhs may pause for a cycle inside a burst.
  task automatic lane_burst(input int l, input int kind, input int nbytes, input int lead);
    cycles(lead + 1);
    i_stopstate[l] = 1'b0;
    cycles(3);
    i_rxactivehs[l] = 1'b1;
    cycles(2);
    case (kind)
      0, 1: begin
        i_rxsynchs[l] = 1'b1;  i_rxvalidhs[l] = 1'b1;  i_rxdatahs[l] = 8'($urandom);
        if (kind == 1) i_errsoths[l] = 1'b1;
        cycles(1);
        i_rxsynchs[l] = 1'b0;  i_errsoths[l] = 1'b0;
        for (int b = 1; b < nbytes; b++) begin
          if ($urandom_range(0, 3) == 0) begin
            i_rxvalidhs[l] = 1'b0;
            cycles(1);
            i_rxvalidhs[l] = 1'b1;
          end
          i_rxdatahs[l] = 8'($urandom);
          cycles(1);
        end
        i_rxvalidhs[l] = 1'b0;
        cycles(1);
        i_rxactivehs[l] = 1'b0;
      end
      2: begin
        i_errsotsynchs[l] = 1'b1;
        cycles(1);
        i_errsotsynchs[l] = 1'b0;
        cycles(nbytes);
        i_rxactivehs[l] = 1'b0;
      end
      3: begin
        i_errsotsynchs[l] = 1'b1;  i_rxvalidhs[l] = 1'b1;  i_rxdatahs[l] = 8'($urandom);
        cycles(1);
        i_errsotsynchs[l] = 1'b0;
        for (int b = 1; b < nbytes; b++) begin
          i_rxdatahs[l] = 8'($urandom);
          cycles(1);
        end
        i_rxvalidhs[l] = 1'b0;
        cycles(1);
        i_rxactivehs[l] = 1'b0;
      end
      default: begin
        i_rxskewcalhs[l] = 1'b1;
        cycles(nbytes);
        i_rxactivehs[l] = 1'b0;
        cycles(2);
      end
    endcase
    cycles(2);
    i_stopstate[l] = 1'b1;
    i_rxskewcalhs[l] = 1'b0;
    cycles(2);
  endtask

  // Every kind of transmission: all lanes with staggered starts, then lane by lane.
  task automatic test_burst_scenarios();
    clear_inputs();
    i_cl_enable = 1'b1;  i_dl_enable = '1;  i_cl_stopstate = 1'b1;  i_stopstate = '1;
    i_cl_rxclkactivehs = 1'b1;
    cycles(4);
    for (int kind = 0; kind < 5; kind++) begin
      for (int skew = 0; skew < 3; skew++) begin
        for (int l = 0; l < 4; l++) begin
          fork
            automatic int ll = l;
            lane_burst(ll, kind, 12, skew * ll);
          join_none
        end
        wait fork;
      end
      for (int l = 0; l < 4; l++) lane_burst(l, kind, 9, 0);
    end
    clear_inputs();
    cycles(2);
  endtask

  // Random values on every input each cycle, in dense and sparse phases.
  task automatic test_random_traffic();
    clear_inputs();
    for (int n = 0; n < 4000; n++) begin
      drive_random_all(((n / 500) % 2) == 1);
      cycles(1);
    end
    clear_inputs();
    cycles(2);
  endtask

  // Byte clock stopped low, then high: outputs still follow the inputs at once.
  task automatic test_no_clock();
    for (int level = 0; level < 2; level++) begin
      clear_inputs();
      if (level == 0) @(negedge i_rxbyteclkhs);
      else            @(posedge i_rxbyteclkhs);
      clk_run = 1'b0;
      for (int n = 0; n < 100; n++) begin
        drive_random_all(n % 2 == 1);
        step();
      end
      clk_run = 1'b1;
      cycles(2);
    end
    clear_inputs();
    step();
  endtask

  // Forces a wrong value on one output; this test must end in FAIL with errors in the log.
  // It confirms that the monitor catches a fault and is not part of the regression.
  task automatic test_checker_fault();
    clear_inputs();
    step();
    force o_cl_enable = ~i_cl_enable;
    pulse(i_cl_enable);
    release o_cl_enable;
    step();
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
    clear_inputs();
    #10.0;
    case (test_name)
      "test_control_path":              test_control_path();
      "test_state_path":                test_state_path();
      "test_hs_data_walking":           test_hs_data_walking();
      "test_lane_independence":         test_lane_independence();
      "test_hs_error_pulses":           test_hs_error_pulses();
      "test_unbuilt_lanes_driven_low":  test_unbuilt_lanes_driven_low();
      "test_unused_signals_ignored":    test_unused_signals_ignored();
      "test_four_state_inputs":         test_four_state_inputs();
      "test_async_changes":             test_async_changes();
      "test_burst_scenarios":           test_burst_scenarios();
      "test_random_traffic":            test_random_traffic();
      "test_no_clock":                  test_no_clock();
      "test_checker_fault":             test_checker_fault();
      default: begin
        err_count++;
        $error("unknown test '%s'", test_name);
      end
    endcase
    #10.0;
    if (err_count == 0 && check_count >= 10) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d, checks=%0d)", test_name, err_count, check_count);
    $finish;
  end

  // Timeout.
  initial begin
    #5_000_000.0;
    $display("%s FAIL (timeout)", test_name);
    $finish;
  end

endmodule
