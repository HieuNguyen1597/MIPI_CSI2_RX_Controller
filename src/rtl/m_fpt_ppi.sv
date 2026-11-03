// Module      : m_fpt_ppi
// Block       : B01 - PHY Protocol Interface (PPI)
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.3
// Date        : 2026-10-27
// Description : Wiring-only boundary between the D-PHY macro and the core, no storage or logic
`timescale 1ns/1ps
`default_nettype none

module m_fpt_ppi #(
  // Parameter Description: number of physical data lanes wired to the D-PHY macro; lanes with an
  // index of PARA_MAX_LANES or above are not built and their outputs are driven to 0.
  // Parameter values: 1, 2 or 4; default is 4.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_LANES = 4
) (
  // Common lane control and state: enables and force-receive-mode from B14 (lite_aclk levels)
  // towards the D-PHY; Stop State and ULPS indications (asynchronous levels) from the D-PHY
  // towards the core. The asynchronous levels are synchronised by their consumers (B11, B14).
  input  wire        i_cl_enable,
  input  wire        i_dl_enable_0,
  input  wire        i_dl_enable_1,
  input  wire        i_dl_enable_2,
  input  wire        i_dl_enable_3,
  input  wire        i_forcerxmode_0,
  input  wire        i_forcerxmode_1,
  input  wire        i_forcerxmode_2,
  input  wire        i_forcerxmode_3,
  output logic       o_cl_enable,
  output logic       o_dl_enable_0,
  output logic       o_dl_enable_1,
  output logic       o_dl_enable_2,
  output logic       o_dl_enable_3,
  output logic       o_forcerxmode_0,
  output logic       o_forcerxmode_1,
  output logic       o_forcerxmode_2,
  output logic       o_forcerxmode_3,
  input  wire        i_cl_stopstate,
  input  wire        i_stopstate_0,
  input  wire        i_stopstate_1,
  input  wire        i_stopstate_2,
  input  wire        i_stopstate_3,
  input  wire        i_cl_ulpsactivenot,
  input  wire        i_ulpsactivenot_0,
  input  wire        i_ulpsactivenot_1,
  input  wire        i_ulpsactivenot_2,
  input  wire        i_ulpsactivenot_3,
  output logic       o_cl_stopstate,
  output logic       o_stopstate_0,
  output logic       o_stopstate_1,
  output logic       o_stopstate_2,
  output logic       o_stopstate_3,
  output logic       o_cl_ulpsactivenot,
  // Clock lane: HS byte clock and clock-lane state from the D-PHY. rxbyteclkhs is only passed on.
  input  wire        i_rxbyteclkhs,
  input  wire        i_cl_rxclkactivehs,
  input  wire        i_cl_ulpsclknot,
  output logic       o_rxbyteclkhs,
  output logic       o_cl_rxclkactivehs,
  output logic       o_cl_ulpsclknot,
  // Data lane High-Speed signals, rxbyteclkhs domain, towards Lane Management; rxskewcalhs also
  // goes to B14.
  input  wire  [7:0] i_rxdatahs_0,
  input  wire  [7:0] i_rxdatahs_1,
  input  wire  [7:0] i_rxdatahs_2,
  input  wire  [7:0] i_rxdatahs_3,
  input  wire        i_rxvalidhs_0,
  input  wire        i_rxvalidhs_1,
  input  wire        i_rxvalidhs_2,
  input  wire        i_rxvalidhs_3,
  input  wire        i_rxactivehs_0,
  input  wire        i_rxactivehs_1,
  input  wire        i_rxactivehs_2,
  input  wire        i_rxactivehs_3,
  input  wire        i_rxsynchs_0,
  input  wire        i_rxsynchs_1,
  input  wire        i_rxsynchs_2,
  input  wire        i_rxsynchs_3,
  input  wire        i_rxskewcalhs_0,
  input  wire        i_rxskewcalhs_1,
  input  wire        i_rxskewcalhs_2,
  input  wire        i_rxskewcalhs_3,
  output logic [7:0] o_rxdatahs_0,
  output logic [7:0] o_rxdatahs_1,
  output logic [7:0] o_rxdatahs_2,
  output logic [7:0] o_rxdatahs_3,
  output logic       o_rxvalidhs_0,
  output logic       o_rxvalidhs_1,
  output logic       o_rxvalidhs_2,
  output logic       o_rxvalidhs_3,
  output logic       o_rxactivehs_0,
  output logic       o_rxactivehs_1,
  output logic       o_rxactivehs_2,
  output logic       o_rxactivehs_3,
  output logic       o_rxsynchs_0,
  output logic       o_rxsynchs_1,
  output logic       o_rxsynchs_2,
  output logic       o_rxsynchs_3,
  output logic       o_rxskewcalhs_0,
  output logic       o_rxskewcalhs_1,
  output logic       o_rxskewcalhs_2,
  output logic       o_rxskewcalhs_3,
  // Data lane Escape mode signals: declared at the PPI, not consumed; they terminate here.
  input  wire        i_rxclkesc_0,
  input  wire        i_rxclkesc_1,
  input  wire        i_rxclkesc_2,
  input  wire        i_rxclkesc_3,
  input  wire        i_rxlpdtesc_0,
  input  wire        i_rxlpdtesc_1,
  input  wire        i_rxlpdtesc_2,
  input  wire        i_rxlpdtesc_3,
  input  wire        i_rxulpsesc_0,
  input  wire        i_rxulpsesc_1,
  input  wire        i_rxulpsesc_2,
  input  wire        i_rxulpsesc_3,
  input  wire  [3:0] i_rxtriggeresc_0,
  input  wire  [3:0] i_rxtriggeresc_1,
  input  wire  [3:0] i_rxtriggeresc_2,
  input  wire  [3:0] i_rxtriggeresc_3,
  input  wire  [7:0] i_rxdataesc_0,
  input  wire  [7:0] i_rxdataesc_1,
  input  wire  [7:0] i_rxdataesc_2,
  input  wire  [7:0] i_rxdataesc_3,
  input  wire        i_rxvalidesc_0,
  input  wire        i_rxvalidesc_1,
  input  wire        i_rxvalidesc_2,
  input  wire        i_rxvalidesc_3,
  // Data lane error signals. errsoths and errsotsynchs are single-cycle rxbyteclkhs pulses towards
  // B02, B11 and B14, which cross them into their own domains. The Low-Power errors (asynchronous
  // levels) are not consumed and terminate in this block.
  input  wire        i_errsoths_0,
  input  wire        i_errsoths_1,
  input  wire        i_errsoths_2,
  input  wire        i_errsoths_3,
  input  wire        i_errsotsynchs_0,
  input  wire        i_errsotsynchs_1,
  input  wire        i_errsotsynchs_2,
  input  wire        i_errsotsynchs_3,
  input  wire        i_erresc_0,
  input  wire        i_erresc_1,
  input  wire        i_erresc_2,
  input  wire        i_erresc_3,
  input  wire        i_errsyncesc_0,
  input  wire        i_errsyncesc_1,
  input  wire        i_errsyncesc_2,
  input  wire        i_errsyncesc_3,
  input  wire        i_errcontrol_0,
  input  wire        i_errcontrol_1,
  input  wire        i_errcontrol_2,
  input  wire        i_errcontrol_3,
  output logic       o_errsoths_0,
  output logic       o_errsoths_1,
  output logic       o_errsoths_2,
  output logic       o_errsoths_3,
  output logic       o_errsotsynchs_0,
  output logic       o_errsotsynchs_1,
  output logic       o_errsotsynchs_2,
  output logic       o_errsotsynchs_3,
  // D-PHY reset request from the PHY Reset Sequencer of B14, and the initialisation status of the
  // D-PHY (asynchronous level, observability only).
  input  wire        i_phy_core_rst,
  input  wire        i_init_done,
  output logic       o_phy_core_rst,
  output logic       o_init_done
);

  // ------------------------------------------------------------------------------------------
  // Constants and internal signals
  // ------------------------------------------------------------------------------------------
  // Lane-built mask: bit x is 1 when lane x is built (x < PARA_MAX_LANES); lane 0 is always built.
  localparam logic [3:0] LPARA_LANE_BUILT = {(PARA_MAX_LANES > 32'd3), (PARA_MAX_LANES > 32'd2),
                                             (PARA_MAX_LANES > 32'd1), 1'b1};

  // Gated copies of the per-lane signals of lanes 1 to 3.
  logic       w_dl_enable_1;
  logic       w_forcerxmode_1;
  logic       w_stopstate_1;
  logic [7:0] w_rxdatahs_1;
  logic       w_rxvalidhs_1;
  logic       w_rxactivehs_1;
  logic       w_rxsynchs_1;
  logic       w_rxskewcalhs_1;
  logic       w_errsoths_1;
  logic       w_errsotsynchs_1;
  logic       w_dl_enable_2;
  logic       w_forcerxmode_2;
  logic       w_stopstate_2;
  logic [7:0] w_rxdatahs_2;
  logic       w_rxvalidhs_2;
  logic       w_rxactivehs_2;
  logic       w_rxsynchs_2;
  logic       w_rxskewcalhs_2;
  logic       w_errsoths_2;
  logic       w_errsotsynchs_2;
  logic       w_dl_enable_3;
  logic       w_forcerxmode_3;
  logic       w_stopstate_3;
  logic [7:0] w_rxdatahs_3;
  logic       w_rxvalidhs_3;
  logic       w_rxactivehs_3;
  logic       w_rxsynchs_3;
  logic       w_rxskewcalhs_3;
  logic       w_errsoths_3;
  logic       w_errsotsynchs_3;

  // ------------------------------------------------------------------------------------------
  // Parameter check
  // ------------------------------------------------------------------------------------------
  // Only 1, 2 or 4 lanes are supported; any other value stops elaboration.
  generate
    if ((PARA_MAX_LANES != 32'd1) && (PARA_MAX_LANES != 32'd2) &&
        (PARA_MAX_LANES != 32'd4)) begin : gen_para_check
      $error("m_fpt_ppi: PARA_MAX_LANES must be 1, 2 or 4");
    end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Lane gate
  // ------------------------------------------------------------------------------------------
  // Lanes 1 to 3 pass through only when built; an unbuilt lane ignores its inputs and drives 0
  // on every output, towards the core and towards the D-PHY.
  always_comb begin : p_com_lane_gate
    w_dl_enable_1      = LPARA_LANE_BUILT[1] ? i_dl_enable_1 : '0;
    w_forcerxmode_1    = LPARA_LANE_BUILT[1] ? i_forcerxmode_1 : '0;
    w_stopstate_1      = LPARA_LANE_BUILT[1] ? i_stopstate_1 : '0;
    w_rxdatahs_1       = LPARA_LANE_BUILT[1] ? i_rxdatahs_1 : '0;
    w_rxvalidhs_1      = LPARA_LANE_BUILT[1] ? i_rxvalidhs_1 : '0;
    w_rxactivehs_1     = LPARA_LANE_BUILT[1] ? i_rxactivehs_1 : '0;
    w_rxsynchs_1       = LPARA_LANE_BUILT[1] ? i_rxsynchs_1 : '0;
    w_rxskewcalhs_1    = LPARA_LANE_BUILT[1] ? i_rxskewcalhs_1 : '0;
    w_errsoths_1       = LPARA_LANE_BUILT[1] ? i_errsoths_1 : '0;
    w_errsotsynchs_1   = LPARA_LANE_BUILT[1] ? i_errsotsynchs_1 : '0;
    w_dl_enable_2      = LPARA_LANE_BUILT[2] ? i_dl_enable_2 : '0;
    w_forcerxmode_2    = LPARA_LANE_BUILT[2] ? i_forcerxmode_2 : '0;
    w_stopstate_2      = LPARA_LANE_BUILT[2] ? i_stopstate_2 : '0;
    w_rxdatahs_2       = LPARA_LANE_BUILT[2] ? i_rxdatahs_2 : '0;
    w_rxvalidhs_2      = LPARA_LANE_BUILT[2] ? i_rxvalidhs_2 : '0;
    w_rxactivehs_2     = LPARA_LANE_BUILT[2] ? i_rxactivehs_2 : '0;
    w_rxsynchs_2       = LPARA_LANE_BUILT[2] ? i_rxsynchs_2 : '0;
    w_rxskewcalhs_2    = LPARA_LANE_BUILT[2] ? i_rxskewcalhs_2 : '0;
    w_errsoths_2       = LPARA_LANE_BUILT[2] ? i_errsoths_2 : '0;
    w_errsotsynchs_2   = LPARA_LANE_BUILT[2] ? i_errsotsynchs_2 : '0;
    w_dl_enable_3      = LPARA_LANE_BUILT[3] ? i_dl_enable_3 : '0;
    w_forcerxmode_3    = LPARA_LANE_BUILT[3] ? i_forcerxmode_3 : '0;
    w_stopstate_3      = LPARA_LANE_BUILT[3] ? i_stopstate_3 : '0;
    w_rxdatahs_3       = LPARA_LANE_BUILT[3] ? i_rxdatahs_3 : '0;
    w_rxvalidhs_3      = LPARA_LANE_BUILT[3] ? i_rxvalidhs_3 : '0;
    w_rxactivehs_3     = LPARA_LANE_BUILT[3] ? i_rxactivehs_3 : '0;
    w_rxsynchs_3       = LPARA_LANE_BUILT[3] ? i_rxsynchs_3 : '0;
    w_rxskewcalhs_3    = LPARA_LANE_BUILT[3] ? i_rxskewcalhs_3 : '0;
    w_errsoths_3       = LPARA_LANE_BUILT[3] ? i_errsoths_3 : '0;
    w_errsotsynchs_3   = LPARA_LANE_BUILT[3] ? i_errsotsynchs_3 : '0;
  end

  // ------------------------------------------------------------------------------------------
  // Common lane control and state
  // ------------------------------------------------------------------------------------------
  // Enables, force-receive-mode and the reset request are lite_aclk levels from B14; the D-PHY
  // treats them as levels, so no synchroniser is needed.
  assign o_cl_enable      = i_cl_enable;
  assign o_dl_enable_0    = i_dl_enable_0;
  assign o_dl_enable_1    = w_dl_enable_1;
  assign o_dl_enable_2    = w_dl_enable_2;
  assign o_dl_enable_3    = w_dl_enable_3;
  assign o_forcerxmode_0  = i_forcerxmode_0;
  assign o_forcerxmode_1  = w_forcerxmode_1;
  assign o_forcerxmode_2  = w_forcerxmode_2;
  assign o_forcerxmode_3  = w_forcerxmode_3;
  assign o_phy_core_rst   = i_phy_core_rst;

  // Stop State, ULPS and initialisation indications are asynchronous levels, synchronised by B11
  // and B14. The data-lane ulpsactivenot_x has no consumer and ends here.
  assign o_cl_stopstate     = i_cl_stopstate;
  assign o_stopstate_0      = i_stopstate_0;
  assign o_stopstate_1      = w_stopstate_1;
  assign o_stopstate_2      = w_stopstate_2;
  assign o_stopstate_3      = w_stopstate_3;
  assign o_cl_ulpsactivenot = i_cl_ulpsactivenot;
  assign o_init_done        = i_init_done;

  // ------------------------------------------------------------------------------------------
  // Clock lane
  // ------------------------------------------------------------------------------------------
  // rxbyteclkhs is a plain connection and is never gated; the clock-lane state signals are
  // asynchronous levels for B14.
  assign o_rxbyteclkhs      = i_rxbyteclkhs;
  assign o_cl_rxclkactivehs = i_cl_rxclkactivehs;
  assign o_cl_ulpsclknot    = i_cl_ulpsclknot;

  // ------------------------------------------------------------------------------------------
  // Data lane High-Speed and errors
  // ------------------------------------------------------------------------------------------
  // Bytes, qualifiers and SoT error pulses reach Lane Management in the cycle they arrive; no
  // burst is qualified or discarded here.
  assign o_rxdatahs_0     = i_rxdatahs_0;
  assign o_rxdatahs_1     = w_rxdatahs_1;
  assign o_rxdatahs_2     = w_rxdatahs_2;
  assign o_rxdatahs_3     = w_rxdatahs_3;
  assign o_rxvalidhs_0    = i_rxvalidhs_0;
  assign o_rxvalidhs_1    = w_rxvalidhs_1;
  assign o_rxvalidhs_2    = w_rxvalidhs_2;
  assign o_rxvalidhs_3    = w_rxvalidhs_3;
  assign o_rxactivehs_0   = i_rxactivehs_0;
  assign o_rxactivehs_1   = w_rxactivehs_1;
  assign o_rxactivehs_2   = w_rxactivehs_2;
  assign o_rxactivehs_3   = w_rxactivehs_3;
  assign o_rxsynchs_0     = i_rxsynchs_0;
  assign o_rxsynchs_1     = w_rxsynchs_1;
  assign o_rxsynchs_2     = w_rxsynchs_2;
  assign o_rxsynchs_3     = w_rxsynchs_3;
  assign o_rxskewcalhs_0  = i_rxskewcalhs_0;
  assign o_rxskewcalhs_1  = w_rxskewcalhs_1;
  assign o_rxskewcalhs_2  = w_rxskewcalhs_2;
  assign o_rxskewcalhs_3  = w_rxskewcalhs_3;
  assign o_errsoths_0     = i_errsoths_0;
  assign o_errsoths_1     = w_errsoths_1;
  assign o_errsoths_2     = w_errsoths_2;
  assign o_errsoths_3     = w_errsoths_3;
  assign o_errsotsynchs_0 = i_errsotsynchs_0;
  assign o_errsotsynchs_1 = w_errsotsynchs_1;
  assign o_errsotsynchs_2 = w_errsotsynchs_2;
  assign o_errsotsynchs_3 = w_errsotsynchs_3;

  // Escape mode signals and Low-Power errors have no consumer in this release and end here.

endmodule

`default_nettype wire
