# Signal mapping — B01 PHY Protocol Interface (PPI)

## Notes

- RTL module: `m_fpt_ppi` (`00_src/rtl/m_fpt_ppi.sv`).
- Reference: FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4.
- The `_x` suffix in a spec name is the lane index; the RTL splits it into one signal per lane, `_0` … `_3`.
- Spec name `-`: RTL-only name, not defined in the specs.

## Mapping

| RTL name | Spec name | Kind |
|---|---|---|
| `m_fpt_ppi` | PHY Protocol Interface – PPI (Block No. 1) | module |
| `PARA_MAX_LANES` | `MAX_LANES` | parameter |
| `LPARA_LANE_BUILT` | - | localparam |
| `gen_para_check` | - | generate block |
| `p_com_lane_gate` | - | always_comb |
| `i_cl_enable` | `cl_enable` | input port |
| `i_dl_enable_0` | `dl_enable_x` | input port |
| `i_dl_enable_1` | `dl_enable_x` | input port |
| `i_dl_enable_2` | `dl_enable_x` | input port |
| `i_dl_enable_3` | `dl_enable_x` | input port |
| `i_forcerxmode_0` | `forcerxmode_x` | input port |
| `i_forcerxmode_1` | `forcerxmode_x` | input port |
| `i_forcerxmode_2` | `forcerxmode_x` | input port |
| `i_forcerxmode_3` | `forcerxmode_x` | input port |
| `o_cl_enable` | `cl_enable` | output port |
| `o_dl_enable_0` | `dl_enable_x` | output port |
| `o_dl_enable_1` | `dl_enable_x` | output port |
| `o_dl_enable_2` | `dl_enable_x` | output port |
| `o_dl_enable_3` | `dl_enable_x` | output port |
| `o_forcerxmode_0` | `forcerxmode_x` | output port |
| `o_forcerxmode_1` | `forcerxmode_x` | output port |
| `o_forcerxmode_2` | `forcerxmode_x` | output port |
| `o_forcerxmode_3` | `forcerxmode_x` | output port |
| `i_cl_stopstate` | `cl_stopstate` | input port |
| `i_stopstate_0` | `stopstate_x` | input port |
| `i_stopstate_1` | `stopstate_x` | input port |
| `i_stopstate_2` | `stopstate_x` | input port |
| `i_stopstate_3` | `stopstate_x` | input port |
| `i_cl_ulpsactivenot` | `cl_ulpsactivenot` | input port |
| `i_ulpsactivenot_0` | `ulpsactivenot_x` | input port |
| `i_ulpsactivenot_1` | `ulpsactivenot_x` | input port |
| `i_ulpsactivenot_2` | `ulpsactivenot_x` | input port |
| `i_ulpsactivenot_3` | `ulpsactivenot_x` | input port |
| `o_cl_stopstate` | `cl_stopstate` | output port |
| `o_stopstate_0` | `stopstate_x` | output port |
| `o_stopstate_1` | `stopstate_x` | output port |
| `o_stopstate_2` | `stopstate_x` | output port |
| `o_stopstate_3` | `stopstate_x` | output port |
| `o_cl_ulpsactivenot` | `cl_ulpsactivenot` | output port |
| `i_rxbyteclkhs` | `rxbyteclkhs` | input port |
| `i_cl_rxclkactivehs` | `cl_rxclkactivehs` | input port |
| `i_cl_ulpsclknot` | `cl_ulpsclknot` | input port |
| `o_rxbyteclkhs` | `rxbyteclkhs` | output port |
| `o_cl_rxclkactivehs` | `cl_rxclkactivehs` | output port |
| `o_cl_ulpsclknot` | `cl_ulpsclknot` | output port |
| `i_rxdatahs_0` | `rxdatahs_x` | input port |
| `i_rxdatahs_1` | `rxdatahs_x` | input port |
| `i_rxdatahs_2` | `rxdatahs_x` | input port |
| `i_rxdatahs_3` | `rxdatahs_x` | input port |
| `i_rxvalidhs_0` | `rxvalidhs_x` | input port |
| `i_rxvalidhs_1` | `rxvalidhs_x` | input port |
| `i_rxvalidhs_2` | `rxvalidhs_x` | input port |
| `i_rxvalidhs_3` | `rxvalidhs_x` | input port |
| `i_rxactivehs_0` | `rxactivehs_x` | input port |
| `i_rxactivehs_1` | `rxactivehs_x` | input port |
| `i_rxactivehs_2` | `rxactivehs_x` | input port |
| `i_rxactivehs_3` | `rxactivehs_x` | input port |
| `i_rxsynchs_0` | `rxsynchs_x` | input port |
| `i_rxsynchs_1` | `rxsynchs_x` | input port |
| `i_rxsynchs_2` | `rxsynchs_x` | input port |
| `i_rxsynchs_3` | `rxsynchs_x` | input port |
| `i_rxskewcalhs_0` | `rxskewcalhs_x` | input port |
| `i_rxskewcalhs_1` | `rxskewcalhs_x` | input port |
| `i_rxskewcalhs_2` | `rxskewcalhs_x` | input port |
| `i_rxskewcalhs_3` | `rxskewcalhs_x` | input port |
| `o_rxdatahs_0` | `rxdatahs_x` | output port |
| `o_rxdatahs_1` | `rxdatahs_x` | output port |
| `o_rxdatahs_2` | `rxdatahs_x` | output port |
| `o_rxdatahs_3` | `rxdatahs_x` | output port |
| `o_rxvalidhs_0` | `rxvalidhs_x` | output port |
| `o_rxvalidhs_1` | `rxvalidhs_x` | output port |
| `o_rxvalidhs_2` | `rxvalidhs_x` | output port |
| `o_rxvalidhs_3` | `rxvalidhs_x` | output port |
| `o_rxactivehs_0` | `rxactivehs_x` | output port |
| `o_rxactivehs_1` | `rxactivehs_x` | output port |
| `o_rxactivehs_2` | `rxactivehs_x` | output port |
| `o_rxactivehs_3` | `rxactivehs_x` | output port |
| `o_rxsynchs_0` | `rxsynchs_x` | output port |
| `o_rxsynchs_1` | `rxsynchs_x` | output port |
| `o_rxsynchs_2` | `rxsynchs_x` | output port |
| `o_rxsynchs_3` | `rxsynchs_x` | output port |
| `o_rxskewcalhs_0` | `rxskewcalhs_x` | output port |
| `o_rxskewcalhs_1` | `rxskewcalhs_x` | output port |
| `o_rxskewcalhs_2` | `rxskewcalhs_x` | output port |
| `o_rxskewcalhs_3` | `rxskewcalhs_x` | output port |
| `i_rxclkesc_0` | `rxclkesc_x` | input port |
| `i_rxclkesc_1` | `rxclkesc_x` | input port |
| `i_rxclkesc_2` | `rxclkesc_x` | input port |
| `i_rxclkesc_3` | `rxclkesc_x` | input port |
| `i_rxlpdtesc_0` | `rxlpdtesc_x` | input port |
| `i_rxlpdtesc_1` | `rxlpdtesc_x` | input port |
| `i_rxlpdtesc_2` | `rxlpdtesc_x` | input port |
| `i_rxlpdtesc_3` | `rxlpdtesc_x` | input port |
| `i_rxulpsesc_0` | `rxulpsesc_x` | input port |
| `i_rxulpsesc_1` | `rxulpsesc_x` | input port |
| `i_rxulpsesc_2` | `rxulpsesc_x` | input port |
| `i_rxulpsesc_3` | `rxulpsesc_x` | input port |
| `i_rxtriggeresc_0` | `rxtriggeresc_x` | input port |
| `i_rxtriggeresc_1` | `rxtriggeresc_x` | input port |
| `i_rxtriggeresc_2` | `rxtriggeresc_x` | input port |
| `i_rxtriggeresc_3` | `rxtriggeresc_x` | input port |
| `i_rxdataesc_0` | `rxdataesc_x` | input port |
| `i_rxdataesc_1` | `rxdataesc_x` | input port |
| `i_rxdataesc_2` | `rxdataesc_x` | input port |
| `i_rxdataesc_3` | `rxdataesc_x` | input port |
| `i_rxvalidesc_0` | `rxvalidesc_x` | input port |
| `i_rxvalidesc_1` | `rxvalidesc_x` | input port |
| `i_rxvalidesc_2` | `rxvalidesc_x` | input port |
| `i_rxvalidesc_3` | `rxvalidesc_x` | input port |
| `i_errsoths_0` | `errsoths_x` | input port |
| `i_errsoths_1` | `errsoths_x` | input port |
| `i_errsoths_2` | `errsoths_x` | input port |
| `i_errsoths_3` | `errsoths_x` | input port |
| `i_errsotsynchs_0` | `errsotsynchs_x` | input port |
| `i_errsotsynchs_1` | `errsotsynchs_x` | input port |
| `i_errsotsynchs_2` | `errsotsynchs_x` | input port |
| `i_errsotsynchs_3` | `errsotsynchs_x` | input port |
| `i_erresc_0` | `erresc_x` | input port |
| `i_erresc_1` | `erresc_x` | input port |
| `i_erresc_2` | `erresc_x` | input port |
| `i_erresc_3` | `erresc_x` | input port |
| `i_errsyncesc_0` | `errsyncesc_x` | input port |
| `i_errsyncesc_1` | `errsyncesc_x` | input port |
| `i_errsyncesc_2` | `errsyncesc_x` | input port |
| `i_errsyncesc_3` | `errsyncesc_x` | input port |
| `i_errcontrol_0` | `errcontrol_x` | input port |
| `i_errcontrol_1` | `errcontrol_x` | input port |
| `i_errcontrol_2` | `errcontrol_x` | input port |
| `i_errcontrol_3` | `errcontrol_x` | input port |
| `o_errsoths_0` | `errsoths_x` | output port |
| `o_errsoths_1` | `errsoths_x` | output port |
| `o_errsoths_2` | `errsoths_x` | output port |
| `o_errsoths_3` | `errsoths_x` | output port |
| `o_errsotsynchs_0` | `errsotsynchs_x` | output port |
| `o_errsotsynchs_1` | `errsotsynchs_x` | output port |
| `o_errsotsynchs_2` | `errsotsynchs_x` | output port |
| `o_errsotsynchs_3` | `errsotsynchs_x` | output port |
| `i_phy_core_rst` | `phy_core_rst` | input port |
| `i_init_done` | `init_done` | input port |
| `o_phy_core_rst` | `phy_core_rst` | output port |
| `o_init_done` | `init_done` | output port |
| `w_dl_enable_1` | `dl_enable_x` | combinational |
| `w_forcerxmode_1` | `forcerxmode_x` | combinational |
| `w_stopstate_1` | `stopstate_x` | combinational |
| `w_rxdatahs_1` | `rxdatahs_x` | combinational |
| `w_rxvalidhs_1` | `rxvalidhs_x` | combinational |
| `w_rxactivehs_1` | `rxactivehs_x` | combinational |
| `w_rxsynchs_1` | `rxsynchs_x` | combinational |
| `w_rxskewcalhs_1` | `rxskewcalhs_x` | combinational |
| `w_errsoths_1` | `errsoths_x` | combinational |
| `w_errsotsynchs_1` | `errsotsynchs_x` | combinational |
| `w_dl_enable_2` | `dl_enable_x` | combinational |
| `w_forcerxmode_2` | `forcerxmode_x` | combinational |
| `w_stopstate_2` | `stopstate_x` | combinational |
| `w_rxdatahs_2` | `rxdatahs_x` | combinational |
| `w_rxvalidhs_2` | `rxvalidhs_x` | combinational |
| `w_rxactivehs_2` | `rxactivehs_x` | combinational |
| `w_rxsynchs_2` | `rxsynchs_x` | combinational |
| `w_rxskewcalhs_2` | `rxskewcalhs_x` | combinational |
| `w_errsoths_2` | `errsoths_x` | combinational |
| `w_errsotsynchs_2` | `errsotsynchs_x` | combinational |
| `w_dl_enable_3` | `dl_enable_x` | combinational |
| `w_forcerxmode_3` | `forcerxmode_x` | combinational |
| `w_stopstate_3` | `stopstate_x` | combinational |
| `w_rxdatahs_3` | `rxdatahs_x` | combinational |
| `w_rxvalidhs_3` | `rxvalidhs_x` | combinational |
| `w_rxactivehs_3` | `rxactivehs_x` | combinational |
| `w_rxsynchs_3` | `rxsynchs_x` | combinational |
| `w_rxskewcalhs_3` | `rxskewcalhs_x` | combinational |
| `w_errsoths_3` | `errsoths_x` | combinational |
| `w_errsotsynchs_3` | `errsotsynchs_x` | combinational |
