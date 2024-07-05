// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// adc_ctrl module


module adc_ctrl
  import adc_ctrl_reg_pkg::*;
#(
  // Param list
  parameter int NumAdcFilter = 8,
  parameter int NumAdcChannel = 2,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 7,
  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}}
) (
  input clk_i,      // regular core clock for SW config interface
  input clk_aon_i,  // always-on slow clock for internal logic
  input rst_ni,     // power-on hardware reset
  input rst_aon_ni, // power-on reset for the 200KHz clock(logic)

  // Regster interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // Inter-module IO, AST interface
  // .pd: Power down ADC (used in deep sleep mode to save power)
  // .channel_sel: channel select for ADC;
  // 2'b0 means stop, 2'b01 means first channel, 2'b10 means second channel, 2'b11 illegal
  output ast_pkg::adc_ast_req_t adc_o,
  // .data: ADC voltage level, each step is 2.148mV(2200mV/1024). It covers 0-2.2V
  // .data_valid: Valid bit(pulse) for adc_d
  input  ast_pkg::adc_ast_rsp_t adc_i,

  // Interrupt indicates a matching or measurement is done
  output logic intr_match_pending_o,

  // Pwrmgr interface
  // Debug cable is detected; wake up the chip in normal sleep and deep sleep mode
  output logic wkup_req_o
);

  adc_ctrl_reg2hw_t reg2hw;
  adc_ctrl_hw2reg_t hw2reg;

  // Alerts
  logic [NumAlerts-1:0] alert_test, alerts;
  assign alert_test = {reg2hw.alert_test.q & reg2hw.alert_test.qe};

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i (alert_test[i]),
      .alert_req_i  (alerts[0]),
      .alert_ack_o  (),
      .alert_state_o(),
      .alert_rx_i   (alert_rx_i[i]),
      .alert_tx_o   (alert_tx_o[i])
    );
  end

  // Register module
  adc_ctrl_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .clk_aon_i,
    .rst_aon_ni,
    .tl_i(tl_i),
    .tl_o(tl_o),
    .reg2hw(reg2hw),
    .hw2reg(hw2reg),
    // SEC_CM: BUS.INTEGRITY
    .intg_err_o(alerts[0])
  );

  // Instantiate DCD core module
  adc_ctrl_core u_adc_ctrl_core (
    .clk_aon_i(clk_aon_i),
    .rst_aon_ni(rst_aon_ni),
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    .reg2hw_i(reg2hw),
    .intr_state_o(hw2reg.intr_state),
    .adc_chn_val_o(hw2reg.adc_chn_val),
    .adc_intr_status_o(hw2reg.adc_intr_status),
    .aon_filter_status_o(hw2reg.filter_status),
    .wkup_req_o,
    .intr_o(intr_match_pending_o),
    .adc_i(adc_i),
    .adc_o(adc_o),
    .aon_fsm_state_o(hw2reg.adc_fsm_state.d)
  );

  // All outputs should be known value after reset
  `ASSERT_KNOWN(IntrKnown, intr_match_pending_o)
  `ASSERT_KNOWN(WakeKnown, wkup_req_o)
  `ASSERT_KNOWN(TlODValidKnown, tl_o.d_valid)
  `ASSERT_KNOWN(TlOAReadyKnown, tl_o.a_ready)
  `ASSERT_KNOWN(AdcKnown_A, adc_o)
  `ASSERT_KNOWN(AlertsKnown_A, alert_tx_o)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// The overall clock manager


  module clkmgr
    import clkmgr_pkg::*;
    import clkmgr_reg_pkg::*;
    import lc_ctrl_pkg::lc_tx_t;
    import prim_mubi_pkg::mubi4_t;
#(
  // Param list
  parameter int NumGroups = 7,
  parameter int NumSwGateableClocks = 4,
  parameter int NumHintableClocks = 4,
  parameter int NumAlerts = 2,

  // Address widths within the block
  parameter int BlockAw = 7,
  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}},
  parameter CLK_NUM = 1
) (
  // Primary module control clocks and resets
  // This drives the register interface
  input clk_i,
  input rst_ni,
  input rst_shadowed_ni,

  // System clocks and resets
  // These are the source clocks for the system
  input clk_main_i,
  input rst_main_ni,
  input clk_io_i,
  input rst_io_ni,
  input clk_usb_i,
  input rst_usb_ni,
  input clk_aon_i,
  input rst_aon_ni,

  // Resets for derived clocks
  // clocks are derived locally
  input rst_io_div2_ni,
  input rst_io_div4_ni,

  // Resets for derived clock generation, root clock gating and related status
  input rst_root_ni,
  input rst_root_main_ni,
  input rst_root_io_ni,
  input rst_root_io_div2_ni,
  input rst_root_io_div4_ni,
  input rst_root_usb_ni,

  // Bus Interface
  input tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // pwrmgr interface
  input pwrmgr_pkg::pwr_clk_req_t pwr_i,
  output pwrmgr_pkg::pwr_clk_rsp_t pwr_o,

  // dft interface
  input prim_mubi_pkg::mubi4_t scanmode_i,

  // idle hints
  // SEC_CM: IDLE.INTERSIG.MUBI
  input prim_mubi_pkg::mubi4_t [3:0] idle_i,

  // life cycle state output
  // SEC_CM: LC_CTRL.INTERSIG.MUBI
  input lc_tx_t lc_hw_debug_en_i,

  // clock bypass control with lc_ctrl
  // SEC_CM: LC_CTRL_CLK_HANDSHAKE.INTERSIG.MUBI
  input lc_tx_t lc_clk_byp_req_i,
  output lc_tx_t lc_clk_byp_ack_o,

  // clock bypass control with ast
  // SEC_CM: CLK_HANDSHAKE.INTERSIG.MUBI
  output mubi4_t io_clk_byp_req_o,
  input mubi4_t io_clk_byp_ack_i,
  output mubi4_t all_clk_byp_req_o,
  input mubi4_t all_clk_byp_ack_i,
  output mubi4_t hi_speed_sel_o,

  // clock calibration has been done.
  // If this is signal is 0, assume clock frequencies to be
  // uncalibrated.
  input prim_mubi_pkg::mubi4_t calib_rdy_i,

  // jittery enable to ast
  output mubi4_t jitter_en_o,

  // external indication for whether dividers should be stepped down
  // SEC_CM: DIV.INTERSIG.MUBI
  input mubi4_t div_step_down_req_i,

  // clock gated indications going to alert handlers
  output clkmgr_cg_en_t cg_en_o,

  // clock output interface
  //output clkmgr_out_t clocks_o
  output clocks_o[CLK_NUM-1:0]

);

  import prim_mubi_pkg::MuBi4False;
  import prim_mubi_pkg::MuBi4True;
  import prim_mubi_pkg::mubi4_test_true_strict;
  import prim_mubi_pkg::mubi4_test_true_loose;
  import prim_mubi_pkg::mubi4_test_false_strict;

  // Hookup point for OCC's on root clocks.
  logic clk_main;
  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_clk_main_buf (
    .clk_i(clk_main_i),
    .clk_o(clk_main)
  );
  logic clk_io;
  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_clk_io_buf (
    .clk_i(clk_io_i),
    .clk_o(clk_io)
  );
  logic clk_usb;
  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_clk_usb_buf (
    .clk_i(clk_usb_i),
    .clk_o(clk_usb)
  );
  logic clk_aon;
  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_clk_aon_buf (
    .clk_i(clk_aon_i),
    .clk_o(clk_aon)
  );

  ////////////////////////////////////////////////////
  // External step down request
  ////////////////////////////////////////////////////
  mubi4_t io_step_down_req;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(1),
    .StabilityCheck(1),
    .ResetValue(MuBi4False)
  ) u_io_step_down_req_sync (
    .clk_i(clk_io),
    .rst_ni(rst_io_ni),
    .mubi_i(div_step_down_req_i),
    .mubi_o({io_step_down_req})
  );


  ////////////////////////////////////////////////////
  // Divided clocks
  // Note divided clocks must use the por version of
  // its related reset to ensure clock division
  // can happen without any dependency
  ////////////////////////////////////////////////////

  logic [1:0] step_down_acks;

  logic clk_io_div2;
  logic clk_io_div4;


  // Declared as size 1 packed array to avoid FPV warning.
  prim_mubi_pkg::mubi4_t [0:0] io_div2_div_scanmode;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(0)
  ) u_io_div2_div_scanmode_sync  (
    .clk_i,
    .rst_ni,
    .mubi_i(scanmode_i),
    .mubi_o({io_div2_div_scanmode})
  );

  prim_clock_div #(
    .Divisor(2)
  ) u_no_scan_io_div2_div (
    // We're using the pre-occ hookup (*_i) version for clock derivation.
    .clk_i(clk_io_i),
    .rst_ni(rst_root_io_ni),
    .step_down_req_i(mubi4_test_true_strict(io_step_down_req)),
    .step_down_ack_o(step_down_acks[0]),
    .test_en_i(mubi4_test_true_strict(io_div2_div_scanmode[0])),
    .clk_o(clk_io_div2)
  );

  // Declared as size 1 packed array to avoid FPV warning.
  prim_mubi_pkg::mubi4_t [0:0] io_div4_div_scanmode;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(0)
  ) u_io_div4_div_scanmode_sync  (
    .clk_i,
    .rst_ni,
    .mubi_i(scanmode_i),
    .mubi_o({io_div4_div_scanmode})
  );

  prim_clock_div #(
    .Divisor(4)
  ) u_no_scan_io_div4_div (
    // We're using the pre-occ hookup (*_i) version for clock derivation.
    .clk_i(clk_io_i),
    .rst_ni(rst_root_io_ni),
    .step_down_req_i(mubi4_test_true_strict(io_step_down_req)),
    .step_down_ack_o(step_down_acks[1]),
    .test_en_i(mubi4_test_true_strict(io_div4_div_scanmode[0])),
    .clk_o(clk_io_div4)
  );

  ////////////////////////////////////////////////////
  // Register Interface
  ////////////////////////////////////////////////////

  logic [NumAlerts-1:0] alert_test, alerts;
  clkmgr_reg_pkg::clkmgr_reg2hw_t reg2hw;
  clkmgr_reg_pkg::clkmgr_hw2reg_t hw2reg;

  // SEC_CM: MEAS.CONFIG.REGWEN
  // SEC_CM: MEAS.CONFIG.SHADOW
  // SEC_CM: CLK_CTRL.CONFIG.REGWEN
  clkmgr_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .rst_shadowed_ni,
    .clk_io_i(clk_io),
    .rst_io_ni,
    .clk_io_div2_i(clk_io_div2),
    .rst_io_div2_ni,
    .clk_io_div4_i(clk_io_div4),
    .rst_io_div4_ni,
    .clk_main_i(clk_main),
    .rst_main_ni,
    .clk_usb_i(clk_usb),
    .rst_usb_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    .shadowed_storage_err_o(hw2reg.fatal_err_code.shadow_storage_err.de),
    .shadowed_update_err_o(hw2reg.recov_err_code.shadow_update_err.de),
    // SEC_CM: BUS.INTEGRITY
    .intg_err_o(hw2reg.fatal_err_code.reg_intg.de)
  );
  assign hw2reg.fatal_err_code.reg_intg.d = 1'b1;
  assign hw2reg.recov_err_code.shadow_update_err.d = 1'b1;
  assign hw2reg.fatal_err_code.shadow_storage_err.d = 1'b1;

  ////////////////////////////////////////////////////
  // Alerts
  ////////////////////////////////////////////////////

  assign alert_test = {
    reg2hw.alert_test.fatal_fault.q & reg2hw.alert_test.fatal_fault.qe,
    reg2hw.alert_test.recov_fault.q & reg2hw.alert_test.recov_fault.qe
  };

  logic recov_alert;
  assign recov_alert =
    hw2reg.recov_err_code.io_measure_err.de |
    hw2reg.recov_err_code.io_timeout_err.de |
    hw2reg.recov_err_code.io_div2_measure_err.de |
    hw2reg.recov_err_code.io_div2_timeout_err.de |
    hw2reg.recov_err_code.io_div4_measure_err.de |
    hw2reg.recov_err_code.io_div4_timeout_err.de |
    hw2reg.recov_err_code.main_measure_err.de |
    hw2reg.recov_err_code.main_timeout_err.de |
    hw2reg.recov_err_code.usb_measure_err.de |
    hw2reg.recov_err_code.usb_timeout_err.de |
    hw2reg.recov_err_code.shadow_update_err.de;

  assign alerts = {
    |reg2hw.fatal_err_code,
    recov_alert
  };

  localparam logic [NumAlerts-1:0] AlertFatal = {1'b1, 1'b0};

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(AlertFatal[i])
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[i]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  ////////////////////////////////////////////////////
  // Clock bypass request
  ////////////////////////////////////////////////////

  mubi4_t extclk_ctrl_sel;
  mubi4_t extclk_ctrl_hi_speed_sel;

  assign extclk_ctrl_sel = mubi4_t'(reg2hw.extclk_ctrl.sel.q);
  assign extclk_ctrl_hi_speed_sel = mubi4_t'(reg2hw.extclk_ctrl.hi_speed_sel.q);

  clkmgr_byp #(
    .NumDivClks(2)
  ) u_clkmgr_byp (
    .clk_i,
    .rst_ni,
    .en_i(lc_hw_debug_en_i),
    .lc_clk_byp_req_i,
    .lc_clk_byp_ack_o,
    .byp_req_i(extclk_ctrl_sel),
    .byp_ack_o(hw2reg.extclk_status.d),
    .hi_speed_sel_i(extclk_ctrl_hi_speed_sel),
    .all_clk_byp_req_o,
    .all_clk_byp_ack_i,
    .io_clk_byp_req_o,
    .io_clk_byp_ack_i,
    .hi_speed_sel_o,

    // divider step down controls
    .step_down_acks_i(step_down_acks)
  );

  ////////////////////////////////////////////////////
  // Feed through clocks
  // Feed through clocks do not actually need to be in clkmgr, as they are
  // completely untouched. The only reason they are here is for easier
  // bundling management purposes through clocks_o
  ////////////////////////////////////////////////////
  prim_clock_buf u_clk_io_div4_powerup_buf (
    .clk_i(clk_io_div4),
    .clk_o(clocks_o.clk_io_div4_powerup)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.io_div4_powerup = MuBi4False;
  prim_clock_buf u_clk_aon_powerup_buf (
    .clk_i(clk_aon),
    .clk_o(clocks_o.clk_aon_powerup)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.aon_powerup = MuBi4False;
  prim_clock_buf u_clk_main_powerup_buf (
    .clk_i(clk_main),
    .clk_o(clocks_o.clk_main_powerup)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.main_powerup = MuBi4False;
  prim_clock_buf u_clk_io_powerup_buf (
    .clk_i(clk_io),
    .clk_o(clocks_o.clk_io_powerup)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.io_powerup = MuBi4False;
  prim_clock_buf u_clk_usb_powerup_buf (
    .clk_i(clk_usb),
    .clk_o(clocks_o.clk_usb_powerup)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.usb_powerup = MuBi4False;
  prim_clock_buf u_clk_io_div2_powerup_buf (
    .clk_i(clk_io_div2),
    .clk_o(clocks_o.clk_io_div2_powerup)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.io_div2_powerup = MuBi4False;
  prim_clock_buf u_clk_aon_secure_buf (
    .clk_i(clk_aon),
    .clk_o(clocks_o.clk_aon_secure)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.aon_secure = MuBi4False;
  prim_clock_buf u_clk_aon_peri_buf (
    .clk_i(clk_aon),
    .clk_o(clocks_o.clk_aon_peri)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.aon_peri = MuBi4False;
  prim_clock_buf u_clk_aon_timers_buf (
    .clk_i(clk_aon),
    .clk_o(clocks_o.clk_aon_timers)
  );

  // clock gated indication for alert handler: these clocks are never gated.
  assign cg_en_o.aon_timers = MuBi4False;

  ////////////////////////////////////////////////////
  // Distribute pwrmgr ip_clk_en requests to each family
  ////////////////////////////////////////////////////
  // clk_main family
  logic pwrmgr_main_en;
  assign pwrmgr_main_en = pwr_i.main_ip_clk_en;
  // clk_io family
  logic pwrmgr_io_en;
  logic pwrmgr_io_div2_en;
  logic pwrmgr_io_div4_en;
  assign pwrmgr_io_en = pwr_i.io_ip_clk_en;
  assign pwrmgr_io_div2_en = pwr_i.io_ip_clk_en;
  assign pwrmgr_io_div4_en = pwr_i.io_ip_clk_en;
  // clk_usb family
  logic pwrmgr_usb_en;
  assign pwrmgr_usb_en = pwr_i.usb_ip_clk_en;

  ////////////////////////////////////////////////////
  // Root gating
  ////////////////////////////////////////////////////

  // clk_main family
  logic [0:0] main_ens;

  logic clk_main_en;
  logic clk_main_root;
  clkmgr_root_ctrl u_main_root_ctrl (
    .clk_i(clk_main),
    .rst_ni(rst_root_main_ni),
    .scanmode_i,
    .async_en_i(pwrmgr_main_en),
    .en_o(clk_main_en),
    .clk_o(clk_main_root)
  );
  assign main_ens[0] = clk_main_en;

  // create synchronized status
  clkmgr_clk_status #(
    .NumClocks(1)
  ) u_main_status (
    .clk_i,
    .rst_ni(rst_root_ni),
    .ens_i(main_ens),
    .status_o(pwr_o.main_status)
  );

  // clk_io family
  logic [2:0] io_ens;

  logic clk_io_en;
  logic clk_io_root;
  clkmgr_root_ctrl u_io_root_ctrl (
    .clk_i(clk_io),
    .rst_ni(rst_root_io_ni),
    .scanmode_i,
    .async_en_i(pwrmgr_io_en),
    .en_o(clk_io_en),
    .clk_o(clk_io_root)
  );
  assign io_ens[0] = clk_io_en;

  logic clk_io_div2_en;
  logic clk_io_div2_root;
  clkmgr_root_ctrl u_io_div2_root_ctrl (
    .clk_i(clk_io_div2),
    .rst_ni(rst_root_io_div2_ni),
    .scanmode_i,
    .async_en_i(pwrmgr_io_div2_en),
    .en_o(clk_io_div2_en),
    .clk_o(clk_io_div2_root)
  );
  assign io_ens[1] = clk_io_div2_en;

  logic clk_io_div4_en;
  logic clk_io_div4_root;
  clkmgr_root_ctrl u_io_div4_root_ctrl (
    .clk_i(clk_io_div4),
    .rst_ni(rst_root_io_div4_ni),
    .scanmode_i,
    .async_en_i(pwrmgr_io_div4_en),
    .en_o(clk_io_div4_en),
    .clk_o(clk_io_div4_root)
  );
  assign io_ens[2] = clk_io_div4_en;

  // create synchronized status
  clkmgr_clk_status #(
    .NumClocks(3)
  ) u_io_status (
    .clk_i,
    .rst_ni(rst_root_ni),
    .ens_i(io_ens),
    .status_o(pwr_o.io_status)
  );

  // clk_usb family
  logic [0:0] usb_ens;

  logic clk_usb_en;
  logic clk_usb_root;
  clkmgr_root_ctrl u_usb_root_ctrl (
    .clk_i(clk_usb),
    .rst_ni(rst_root_usb_ni),
    .scanmode_i,
    .async_en_i(pwrmgr_usb_en),
    .en_o(clk_usb_en),
    .clk_o(clk_usb_root)
  );
  assign usb_ens[0] = clk_usb_en;

  // create synchronized status
  clkmgr_clk_status #(
    .NumClocks(1)
  ) u_usb_status (
    .clk_i,
    .rst_ni(rst_root_ni),
    .ens_i(usb_ens),
    .status_o(pwr_o.usb_status)
  );

  ////////////////////////////////////////////////////
  // Clock Measurement for the roots
  // SEC_CM: TIMEOUT.CLK.BKGN_CHK, MEAS.CLK.BKGN_CHK
  ////////////////////////////////////////////////////

  typedef enum logic [2:0] {
    BaseIdx,
    ClkIoIdx,
    ClkIoDiv2Idx,
    ClkIoDiv4Idx,
    ClkMainIdx,
    ClkUsbIdx,
    CalibRdyLastIdx
  } clkmgr_calib_idx_e;

  // if clocks become uncalibrated, allow the measurement control configurations to change
  mubi4_t [CalibRdyLastIdx-1:0] calib_rdy;
  prim_mubi4_sync #(
    .AsyncOn(1),
    .NumCopies(int'(CalibRdyLastIdx)),
    .ResetValue(MuBi4False)
  ) u_calib_rdy_sync (
    .clk_i,
    .rst_ni,
    .mubi_i(calib_rdy_i),
    .mubi_o({calib_rdy})
  );

  always_comb begin
    hw2reg.measure_ctrl_regwen.de = '0;
    hw2reg.measure_ctrl_regwen.d = reg2hw.measure_ctrl_regwen;

    if (mubi4_test_false_strict(calib_rdy[BaseIdx])) begin
      hw2reg.measure_ctrl_regwen.de = 1'b1;
      hw2reg.measure_ctrl_regwen.d = 1'b1;
    end
  end

  clkmgr_meas_chk #(
    .Cnt(960),
    .RefCnt(1)
  ) u_io_meas (
    .clk_i,
    .rst_ni,
    .clk_src_i(clk_io),
    .rst_src_ni(rst_io_ni),
    .clk_ref_i(clk_aon),
    .rst_ref_ni(rst_aon_ni),
    // signals on source domain
    .src_en_i(clk_io_en & mubi4_test_true_loose(mubi4_t'(reg2hw.io_meas_ctrl_en))),
    .src_max_cnt_i(reg2hw.io_meas_ctrl_shadowed.hi.q),
    .src_min_cnt_i(reg2hw.io_meas_ctrl_shadowed.lo.q),
    .src_cfg_meas_en_i(mubi4_t'(reg2hw.io_meas_ctrl_en.q)),
    .src_cfg_meas_en_valid_o(hw2reg.io_meas_ctrl_en.de),
    .src_cfg_meas_en_o(hw2reg.io_meas_ctrl_en.d),
    // signals on local clock domain
    .calib_rdy_i(calib_rdy[ClkIoIdx]),
    .meas_err_o(hw2reg.recov_err_code.io_measure_err.de),
    .timeout_err_o(hw2reg.recov_err_code.io_timeout_err.de)
  );

  assign hw2reg.recov_err_code.io_measure_err.d = 1'b1;
  assign hw2reg.recov_err_code.io_timeout_err.d = 1'b1;


  clkmgr_meas_chk #(
    .Cnt(480),
    .RefCnt(1)
  ) u_io_div2_meas (
    .clk_i,
    .rst_ni,
    .clk_src_i(clk_io_div2),
    .rst_src_ni(rst_io_div2_ni),
    .clk_ref_i(clk_aon),
    .rst_ref_ni(rst_aon_ni),
    // signals on source domain
    .src_en_i(clk_io_div2_en & mubi4_test_true_loose(mubi4_t'(reg2hw.io_div2_meas_ctrl_en))),
    .src_max_cnt_i(reg2hw.io_div2_meas_ctrl_shadowed.hi.q),
    .src_min_cnt_i(reg2hw.io_div2_meas_ctrl_shadowed.lo.q),
    .src_cfg_meas_en_i(mubi4_t'(reg2hw.io_div2_meas_ctrl_en.q)),
    .src_cfg_meas_en_valid_o(hw2reg.io_div2_meas_ctrl_en.de),
    .src_cfg_meas_en_o(hw2reg.io_div2_meas_ctrl_en.d),
    // signals on local clock domain
    .calib_rdy_i(calib_rdy[ClkIoDiv2Idx]),
    .meas_err_o(hw2reg.recov_err_code.io_div2_measure_err.de),
    .timeout_err_o(hw2reg.recov_err_code.io_div2_timeout_err.de)
  );

  assign hw2reg.recov_err_code.io_div2_measure_err.d = 1'b1;
  assign hw2reg.recov_err_code.io_div2_timeout_err.d = 1'b1;


  clkmgr_meas_chk #(
    .Cnt(240),
    .RefCnt(1)
  ) u_io_div4_meas (
    .clk_i,
    .rst_ni,
    .clk_src_i(clk_io_div4),
    .rst_src_ni(rst_io_div4_ni),
    .clk_ref_i(clk_aon),
    .rst_ref_ni(rst_aon_ni),
    // signals on source domain
    .src_en_i(clk_io_div4_en & mubi4_test_true_loose(mubi4_t'(reg2hw.io_div4_meas_ctrl_en))),
    .src_max_cnt_i(reg2hw.io_div4_meas_ctrl_shadowed.hi.q),
    .src_min_cnt_i(reg2hw.io_div4_meas_ctrl_shadowed.lo.q),
    .src_cfg_meas_en_i(mubi4_t'(reg2hw.io_div4_meas_ctrl_en.q)),
    .src_cfg_meas_en_valid_o(hw2reg.io_div4_meas_ctrl_en.de),
    .src_cfg_meas_en_o(hw2reg.io_div4_meas_ctrl_en.d),
    // signals on local clock domain
    .calib_rdy_i(calib_rdy[ClkIoDiv4Idx]),
    .meas_err_o(hw2reg.recov_err_code.io_div4_measure_err.de),
    .timeout_err_o(hw2reg.recov_err_code.io_div4_timeout_err.de)
  );

  assign hw2reg.recov_err_code.io_div4_measure_err.d = 1'b1;
  assign hw2reg.recov_err_code.io_div4_timeout_err.d = 1'b1;


  clkmgr_meas_chk #(
    .Cnt(1000),
    .RefCnt(1)
  ) u_main_meas (
    .clk_i,
    .rst_ni,
    .clk_src_i(clk_main),
    .rst_src_ni(rst_main_ni),
    .clk_ref_i(clk_aon),
    .rst_ref_ni(rst_aon_ni),
    // signals on source domain
    .src_en_i(clk_main_en & mubi4_test_true_loose(mubi4_t'(reg2hw.main_meas_ctrl_en))),
    .src_max_cnt_i(reg2hw.main_meas_ctrl_shadowed.hi.q),
    .src_min_cnt_i(reg2hw.main_meas_ctrl_shadowed.lo.q),
    .src_cfg_meas_en_i(mubi4_t'(reg2hw.main_meas_ctrl_en.q)),
    .src_cfg_meas_en_valid_o(hw2reg.main_meas_ctrl_en.de),
    .src_cfg_meas_en_o(hw2reg.main_meas_ctrl_en.d),
    // signals on local clock domain
    .calib_rdy_i(calib_rdy[ClkMainIdx]),
    .meas_err_o(hw2reg.recov_err_code.main_measure_err.de),
    .timeout_err_o(hw2reg.recov_err_code.main_timeout_err.de)
  );

  assign hw2reg.recov_err_code.main_measure_err.d = 1'b1;
  assign hw2reg.recov_err_code.main_timeout_err.d = 1'b1;


  clkmgr_meas_chk #(
    .Cnt(480),
    .RefCnt(1)
  ) u_usb_meas (
    .clk_i,
    .rst_ni,
    .clk_src_i(clk_usb),
    .rst_src_ni(rst_usb_ni),
    .clk_ref_i(clk_aon),
    .rst_ref_ni(rst_aon_ni),
    // signals on source domain
    .src_en_i(clk_usb_en & mubi4_test_true_loose(mubi4_t'(reg2hw.usb_meas_ctrl_en))),
    .src_max_cnt_i(reg2hw.usb_meas_ctrl_shadowed.hi.q),
    .src_min_cnt_i(reg2hw.usb_meas_ctrl_shadowed.lo.q),
    .src_cfg_meas_en_i(mubi4_t'(reg2hw.usb_meas_ctrl_en.q)),
    .src_cfg_meas_en_valid_o(hw2reg.usb_meas_ctrl_en.de),
    .src_cfg_meas_en_o(hw2reg.usb_meas_ctrl_en.d),
    // signals on local clock domain
    .calib_rdy_i(calib_rdy[ClkUsbIdx]),
    .meas_err_o(hw2reg.recov_err_code.usb_measure_err.de),
    .timeout_err_o(hw2reg.recov_err_code.usb_timeout_err.de)
  );

  assign hw2reg.recov_err_code.usb_measure_err.d = 1'b1;
  assign hw2reg.recov_err_code.usb_timeout_err.d = 1'b1;


  ////////////////////////////////////////////////////
  // Clocks with only root gate
  ////////////////////////////////////////////////////
  assign clocks_o.clk_io_div4_infra = clk_io_div4_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_div4_infra (
    .clk_i(clk_io_div4),
    .rst_ni(rst_io_div4_ni),
    .mubi_i(((clk_io_div4_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_div4_infra)
  );
  assign clocks_o.clk_main_infra = clk_main_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_main_infra (
    .clk_i(clk_main),
    .rst_ni(rst_main_ni),
    .mubi_i(((clk_main_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.main_infra)
  );
  assign clocks_o.clk_usb_infra = clk_usb_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_usb_infra (
    .clk_i(clk_usb),
    .rst_ni(rst_usb_ni),
    .mubi_i(((clk_usb_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.usb_infra)
  );
  assign clocks_o.clk_io_infra = clk_io_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_infra (
    .clk_i(clk_io),
    .rst_ni(rst_io_ni),
    .mubi_i(((clk_io_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_infra)
  );
  assign clocks_o.clk_io_div2_infra = clk_io_div2_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_div2_infra (
    .clk_i(clk_io_div2),
    .rst_ni(rst_io_div2_ni),
    .mubi_i(((clk_io_div2_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_div2_infra)
  );
  assign clocks_o.clk_io_div4_secure = clk_io_div4_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_div4_secure (
    .clk_i(clk_io_div4),
    .rst_ni(rst_io_div4_ni),
    .mubi_i(((clk_io_div4_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_div4_secure)
  );
  assign clocks_o.clk_main_secure = clk_main_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_main_secure (
    .clk_i(clk_main),
    .rst_ni(rst_main_ni),
    .mubi_i(((clk_main_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.main_secure)
  );
  assign clocks_o.clk_io_div4_timers = clk_io_div4_root;

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_div4_timers (
    .clk_i(clk_io_div4),
    .rst_ni(rst_io_div4_ni),
    .mubi_i(((clk_io_div4_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_div4_timers)
  );

  ////////////////////////////////////////////////////
  // Software direct control group
  ////////////////////////////////////////////////////

  logic clk_io_div4_peri_sw_en;
  logic clk_io_div2_peri_sw_en;
  logic clk_io_peri_sw_en;
  logic clk_usb_peri_sw_en;

  prim_flop_2sync #(
    .Width(1)
  ) u_clk_io_div4_peri_sw_en_sync (
    .clk_i(clk_io_div4),
    .rst_ni(rst_io_div4_ni),
    .d_i(reg2hw.clk_enables.clk_io_div4_peri_en.q),
    .q_o(clk_io_div4_peri_sw_en)
  );

  // Declared as size 1 packed array to avoid FPV warning.
  prim_mubi_pkg::mubi4_t [0:0] clk_io_div4_peri_scanmode;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(0)
  ) u_clk_io_div4_peri_scanmode_sync  (
    .clk_i,
    .rst_ni,
    .mubi_i(scanmode_i),
    .mubi_o(clk_io_div4_peri_scanmode)
  );

  logic clk_io_div4_peri_combined_en;
  assign clk_io_div4_peri_combined_en = clk_io_div4_peri_sw_en & clk_io_div4_en;
  prim_clock_gating #(
    .FpgaBufGlobal(1'b1) // This clock spans across multiple clock regions.
  ) u_clk_io_div4_peri_cg (
    .clk_i(clk_io_div4),
    .en_i(clk_io_div4_peri_combined_en),
    .test_en_i(mubi4_test_true_strict(clk_io_div4_peri_scanmode[0])),
    .clk_o(clocks_o.clk_io_div4_peri)
  );

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_div4_peri (
    .clk_i(clk_io_div4),
    .rst_ni(rst_io_div4_ni),
    .mubi_i(((clk_io_div4_peri_combined_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_div4_peri)
  );

  prim_flop_2sync #(
    .Width(1)
  ) u_clk_io_div2_peri_sw_en_sync (
    .clk_i(clk_io_div2),
    .rst_ni(rst_io_div2_ni),
    .d_i(reg2hw.clk_enables.clk_io_div2_peri_en.q),
    .q_o(clk_io_div2_peri_sw_en)
  );

  // Declared as size 1 packed array to avoid FPV warning.
  prim_mubi_pkg::mubi4_t [0:0] clk_io_div2_peri_scanmode;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(0)
  ) u_clk_io_div2_peri_scanmode_sync  (
    .clk_i,
    .rst_ni,
    .mubi_i(scanmode_i),
    .mubi_o(clk_io_div2_peri_scanmode)
  );

  logic clk_io_div2_peri_combined_en;
  assign clk_io_div2_peri_combined_en = clk_io_div2_peri_sw_en & clk_io_div2_en;
  prim_clock_gating #(
    .FpgaBufGlobal(1'b1) // This clock spans across multiple clock regions.
  ) u_clk_io_div2_peri_cg (
    .clk_i(clk_io_div2),
    .en_i(clk_io_div2_peri_combined_en),
    .test_en_i(mubi4_test_true_strict(clk_io_div2_peri_scanmode[0])),
    .clk_o(clocks_o.clk_io_div2_peri)
  );

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_div2_peri (
    .clk_i(clk_io_div2),
    .rst_ni(rst_io_div2_ni),
    .mubi_i(((clk_io_div2_peri_combined_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_div2_peri)
  );

  prim_flop_2sync #(
    .Width(1)
  ) u_clk_io_peri_sw_en_sync (
    .clk_i(clk_io),
    .rst_ni(rst_io_ni),
    .d_i(reg2hw.clk_enables.clk_io_peri_en.q),
    .q_o(clk_io_peri_sw_en)
  );

  // Declared as size 1 packed array to avoid FPV warning.
  prim_mubi_pkg::mubi4_t [0:0] clk_io_peri_scanmode;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(0)
  ) u_clk_io_peri_scanmode_sync  (
    .clk_i,
    .rst_ni,
    .mubi_i(scanmode_i),
    .mubi_o(clk_io_peri_scanmode)
  );

  logic clk_io_peri_combined_en;
  assign clk_io_peri_combined_en = clk_io_peri_sw_en & clk_io_en;
  prim_clock_gating #(
    .FpgaBufGlobal(1'b1) // This clock spans across multiple clock regions.
  ) u_clk_io_peri_cg (
    .clk_i(clk_io),
    .en_i(clk_io_peri_combined_en),
    .test_en_i(mubi4_test_true_strict(clk_io_peri_scanmode[0])),
    .clk_o(clocks_o.clk_io_peri)
  );

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_io_peri (
    .clk_i(clk_io),
    .rst_ni(rst_io_ni),
    .mubi_i(((clk_io_peri_combined_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.io_peri)
  );

  prim_flop_2sync #(
    .Width(1)
  ) u_clk_usb_peri_sw_en_sync (
    .clk_i(clk_usb),
    .rst_ni(rst_usb_ni),
    .d_i(reg2hw.clk_enables.clk_usb_peri_en.q),
    .q_o(clk_usb_peri_sw_en)
  );

  // Declared as size 1 packed array to avoid FPV warning.
  prim_mubi_pkg::mubi4_t [0:0] clk_usb_peri_scanmode;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(0)
  ) u_clk_usb_peri_scanmode_sync  (
    .clk_i,
    .rst_ni,
    .mubi_i(scanmode_i),
    .mubi_o(clk_usb_peri_scanmode)
  );

  logic clk_usb_peri_combined_en;
  assign clk_usb_peri_combined_en = clk_usb_peri_sw_en & clk_usb_en;
  prim_clock_gating #(
    .FpgaBufGlobal(1'b1) // This clock spans across multiple clock regions.
  ) u_clk_usb_peri_cg (
    .clk_i(clk_usb),
    .en_i(clk_usb_peri_combined_en),
    .test_en_i(mubi4_test_true_strict(clk_usb_peri_scanmode[0])),
    .clk_o(clocks_o.clk_usb_peri)
  );

  // clock gated indication for alert handler
  prim_mubi4_sender #(
    .ResetValue(MuBi4True)
  ) u_prim_mubi4_sender_clk_usb_peri (
    .clk_i(clk_usb),
    .rst_ni(rst_usb_ni),
    .mubi_i(((clk_usb_peri_combined_en) ? MuBi4False : MuBi4True)),
    .mubi_o(cg_en_o.usb_peri)
  );


  ////////////////////////////////////////////////////
  // Software hint group
  // The idle hint feedback is assumed to be synchronous to the
  // clock target
  ////////////////////////////////////////////////////

  logic [3:0] idle_cnt_err;

  clkmgr_trans #(
    .FpgaBufGlobal(1'b0) // This clock is used primarily locally.
  ) u_clk_main_aes_trans (
    .clk_i(clk_main),
    .clk_gated_i(clk_main_root),
    .rst_ni(rst_main_ni),
    .en_i(clk_main_en),
    .idle_i(idle_i[HintMainAes]),
    .sw_hint_i(reg2hw.clk_hints.clk_main_aes_hint.q),
    .scanmode_i,
    .alert_cg_en_o(cg_en_o.main_aes),
    .clk_o(clocks_o.clk_main_aes),
    .clk_reg_i(clk_i),
    .rst_reg_ni(rst_ni),
    .reg_en_o(hw2reg.clk_hints_status.clk_main_aes_val.d),
    .reg_cnt_err_o(idle_cnt_err[HintMainAes])
  );
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(
    ClkMainAesCountCheck_A,
    u_clk_main_aes_trans.u_idle_cnt,
    alert_tx_o[1])

  clkmgr_trans #(
    .FpgaBufGlobal(1'b0) // This clock is used primarily locally.
  ) u_clk_main_hmac_trans (
    .clk_i(clk_main),
    .clk_gated_i(clk_main_root),
    .rst_ni(rst_main_ni),
    .en_i(clk_main_en),
    .idle_i(idle_i[HintMainHmac]),
    .sw_hint_i(reg2hw.clk_hints.clk_main_hmac_hint.q),
    .scanmode_i,
    .alert_cg_en_o(cg_en_o.main_hmac),
    .clk_o(clocks_o.clk_main_hmac),
    .clk_reg_i(clk_i),
    .rst_reg_ni(rst_ni),
    .reg_en_o(hw2reg.clk_hints_status.clk_main_hmac_val.d),
    .reg_cnt_err_o(idle_cnt_err[HintMainHmac])
  );
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(
    ClkMainHmacCountCheck_A,
    u_clk_main_hmac_trans.u_idle_cnt,
    alert_tx_o[1])

  clkmgr_trans #(
    .FpgaBufGlobal(1'b1) // KMAC is getting too big for a single clock region.
  ) u_clk_main_kmac_trans (
    .clk_i(clk_main),
    .clk_gated_i(clk_main_root),
    .rst_ni(rst_main_ni),
    .en_i(clk_main_en),
    .idle_i(idle_i[HintMainKmac]),
    .sw_hint_i(reg2hw.clk_hints.clk_main_kmac_hint.q),
    .scanmode_i,
    .alert_cg_en_o(cg_en_o.main_kmac),
    .clk_o(clocks_o.clk_main_kmac),
    .clk_reg_i(clk_i),
    .rst_reg_ni(rst_ni),
    .reg_en_o(hw2reg.clk_hints_status.clk_main_kmac_val.d),
    .reg_cnt_err_o(idle_cnt_err[HintMainKmac])
  );
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(
    ClkMainKmacCountCheck_A,
    u_clk_main_kmac_trans.u_idle_cnt,
    alert_tx_o[1])

  clkmgr_trans #(
    .FpgaBufGlobal(1'b0) // This clock is used primarily locally.
  ) u_clk_main_otbn_trans (
    .clk_i(clk_main),
    .clk_gated_i(clk_main_root),
    .rst_ni(rst_main_ni),
    .en_i(clk_main_en),
    .idle_i(idle_i[HintMainOtbn]),
    .sw_hint_i(reg2hw.clk_hints.clk_main_otbn_hint.q),
    .scanmode_i,
    .alert_cg_en_o(cg_en_o.main_otbn),
    .clk_o(clocks_o.clk_main_otbn),
    .clk_reg_i(clk_i),
    .rst_reg_ni(rst_ni),
    .reg_en_o(hw2reg.clk_hints_status.clk_main_otbn_val.d),
    .reg_cnt_err_o(idle_cnt_err[HintMainOtbn])
  );
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(
    ClkMainOtbnCountCheck_A,
    u_clk_main_otbn_trans.u_idle_cnt,
    alert_tx_o[1])
  assign hw2reg.fatal_err_code.idle_cnt.d = 1'b1;
  assign hw2reg.fatal_err_code.idle_cnt.de = |idle_cnt_err;

  // state readback
  assign hw2reg.clk_hints_status.clk_main_aes_val.de = 1'b1;
  assign hw2reg.clk_hints_status.clk_main_hmac_val.de = 1'b1;
  assign hw2reg.clk_hints_status.clk_main_kmac_val.de = 1'b1;
  assign hw2reg.clk_hints_status.clk_main_otbn_val.de = 1'b1;

  // SEC_CM: JITTER.CONFIG.MUBI
  assign jitter_en_o = mubi4_t'(reg2hw.jitter_enable.q);

  ////////////////////////////////////////////////////
  // Exported clocks
  ////////////////////////////////////////////////////


  ////////////////////////////////////////////////////
  // Assertions
  ////////////////////////////////////////////////////

  `ASSERT_KNOWN(TlDValidKnownO_A, tl_o.d_valid)
  `ASSERT_KNOWN(TlAReadyKnownO_A, tl_o.a_ready)
  `ASSERT_KNOWN(AlertsKnownO_A,   alert_tx_o)
  `ASSERT_KNOWN(PwrMgrKnownO_A, pwr_o)
  `ASSERT_KNOWN(AllClkBypReqKnownO_A, all_clk_byp_req_o)
  `ASSERT_KNOWN(IoClkBypReqKnownO_A, io_clk_byp_req_o)
  `ASSERT_KNOWN(LcCtrlClkBypAckKnownO_A, lc_clk_byp_ack_o)
  `ASSERT_KNOWN(JitterEnableKnownO_A, jitter_en_o)
  `ASSERT_KNOWN(ClocksKownO_A, clocks_o)
  `ASSERT_KNOWN(CgEnKnownO_A, cg_en_o)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[1])
endmodule // clkmgr

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Flash Controller Module
//
//

module flash_ctrl
  import flash_ctrl_pkg::*;  import flash_ctrl_reg_pkg::*;
#(
  
  // Param list
  parameter int RegNumBanks = 2,
  parameter int RegPagesPerBank = 256,
  parameter int RegBusPgmResBytes = 64,
  parameter int RegPageWidth = 8,
  parameter int RegBankWidth = 1,
  parameter int NumRegions = 8,
  parameter int NumInfoTypes = 3,
  parameter int NumInfos0 = 10,
  parameter int NumInfos1 = 1,
  parameter int NumInfos2 = 2,
  parameter int WordsPerPage = 256,
  parameter int BytesPerWord = 8,
  parameter int BytesPerPage = 2048,
  parameter int BytesPerBank = 524288,
  parameter int unsigned ExecEn = 32'ha26a38f7,
  parameter int MaxFifoDepth = 16,
  parameter int MaxFifoWidth = 5,
  parameter int NumAlerts = 5,

  // Address widths within the block
  parameter int CoreAw = 9,
  parameter int PrimAw = 7,
  parameter int MemAw = 1,
  
  parameter logic [NumAlerts-1:0] AlertAsyncOn    = {NumAlerts{1'b1}},
  parameter flash_key_t           RndCnstAddrKey  = RndCnstAddrKeyDefault,
  parameter flash_key_t           RndCnstDataKey  = RndCnstDataKeyDefault,
  parameter all_seeds_t           RndCnstAllSeeds = RndCnstAllSeedsDefault,
  parameter lfsr_seed_t           RndCnstLfsrSeed = RndCnstLfsrSeedDefault,
  parameter lfsr_perm_t           RndCnstLfsrPerm = RndCnstLfsrPermDefault,
  parameter int                   ProgFifoDepth   = MaxFifoDepth,
  parameter int                   RdFifoDepth     = MaxFifoDepth,
  parameter bit                   SecScrambleEn   = 1'b1
) (
  input        clk_i,
  input        rst_ni,
  input        rst_shadowed_ni,

  input        clk_otp_i,
  input        rst_otp_ni,

  // life cycle interface
  // SEC_CM: LC_CTRL.INTERSIG.MUBI
  input lc_ctrl_pkg::lc_tx_t lc_creator_seed_sw_rw_en_i,
  input lc_ctrl_pkg::lc_tx_t lc_owner_seed_sw_rw_en_i,
  input lc_ctrl_pkg::lc_tx_t lc_iso_part_sw_rd_en_i,
  input lc_ctrl_pkg::lc_tx_t lc_iso_part_sw_wr_en_i,
  input lc_ctrl_pkg::lc_tx_t lc_seed_hw_rd_en_i,
  input lc_ctrl_pkg::lc_tx_t lc_escalate_en_i,
  input lc_ctrl_pkg::lc_tx_t lc_nvm_debug_en_i,

  // Bus Interface
  input        tlul_pkg::tl_h2d_t core_tl_i,
  output       tlul_pkg::tl_d2h_t core_tl_o,
  input        tlul_pkg::tl_h2d_t prim_tl_i,
  output       tlul_pkg::tl_d2h_t prim_tl_o,
  input        tlul_pkg::tl_h2d_t mem_tl_i,
  output       tlul_pkg::tl_d2h_t mem_tl_o,

  // otp/lc/pwrmgr/keymgr Interface
  // SEC_CM: SCRAMBLE.KEY.SIDELOAD
  output       otp_ctrl_pkg::flash_otp_key_req_t otp_o,
  input        otp_ctrl_pkg::flash_otp_key_rsp_t otp_i,
  input        lc_ctrl_pkg::lc_tx_t rma_req_i,
  input        lc_ctrl_pkg::lc_flash_rma_seed_t rma_seed_i,
  output       lc_ctrl_pkg::lc_tx_t rma_ack_o,
  output       pwrmgr_pkg::pwr_flash_t pwrmgr_o,
  output       keymgr_flash_t keymgr_o,

  // IOs
  input cio_tck_i,
  input cio_tms_i,
  input cio_tdi_i,
  output logic cio_tdo_en_o,
  output logic cio_tdo_o,

  // Interrupts
  output logic intr_corr_err_o,   // Correctable errors encountered
  output logic intr_prog_empty_o, // Program fifo is empty
  output logic intr_prog_lvl_o,   // Program fifo is empty
  output logic intr_rd_full_o,    // Read fifo is full
  output logic intr_rd_lvl_o,     // Read fifo is full
  output logic intr_op_done_o,    // Requested flash operation (wr/erase) done

  // Alerts
  input  prim_alert_pkg::alert_rx_t [flash_ctrl_reg_pkg::NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [flash_ctrl_reg_pkg::NumAlerts-1:0] alert_tx_o,

  // Observability
  input ast_pkg::ast_obs_ctrl_t obs_ctrl_i,
  output logic [7:0] fla_obs_o,

  // Flash test interface
  input scan_en_i,
  input prim_mubi_pkg::mubi4_t scanmode_i,
  input scan_rst_ni,
  input prim_mubi_pkg::mubi4_t flash_bist_enable_i,
  input flash_power_down_h_i,
  input flash_power_ready_h_i,
  inout [1:0] flash_test_mode_a_io,
  inout flash_test_voltage_h_io
);

  //////////////////////////////////////////////////////////
  // Double check supplied param is not bigger than allowed
  //////////////////////////////////////////////////////////
  `ASSERT_INIT(FifoDepthCheck_A, (ProgFifoDepth <= MaxFifoDepth) &
                                 (RdFifoDepth <= MaxFifoDepth))

  import prim_mubi_pkg::mubi4_t;

  flash_ctrl_core_reg2hw_t reg2hw;
  flash_ctrl_core_hw2reg_t hw2reg;

  tlul_pkg::tl_h2d_t tl_win_h2d [2];
  tlul_pkg::tl_d2h_t tl_win_d2h [2];

  // Register module
  logic storage_err;
  logic update_err;
  logic intg_err;
  logic eflash_cmd_intg_err;
  logic tl_gate_intg_err;
  logic tl_prog_gate_intg_err;

  // SEC_CM: REG.BUS.INTEGRITY
  // SEC_CM: CTRL.CONFIG.REGWEN
  // SEC_CM: DATA_REGIONS.CONFIG.REGWEN, DATA_REGIONS.CONFIG.SHADOW
  // SEC_CM: INFO_REGIONS.CONFIG.REGWEN, INFO_REGIONS.CONFIG.SHADOW
  // SEC_CM: BANK.CONFIG.REGWEN, BANK.CONFIG.SHADOW
  flash_ctrl_core_reg_top u_reg_core (
    .clk_i,
    .rst_ni,
    .rst_shadowed_ni,

    .tl_i(core_tl_i),
    .tl_o(core_tl_o),

    .tl_win_o (tl_win_h2d),
    .tl_win_i (tl_win_d2h),

    .reg2hw,
    .hw2reg,

    .shadowed_storage_err_o (storage_err),
    .shadowed_update_err_o  (update_err),
    .intg_err_o             (intg_err)
  );

  bank_cfg_t [NumBanks-1:0] bank_cfgs;
  mp_region_cfg_t [MpRegions:0] region_cfgs;
  info_page_cfg_t [NumBanks-1:0][InfoTypes-1:0][InfosPerBank-1:0] info_page_cfgs;

  flash_ctrl_region_cfg u_region_cfg (
    .clk_i,
    .rst_ni,
    .lc_creator_seed_sw_rw_en_i,
    .lc_owner_seed_sw_rw_en_i,
    .lc_iso_part_sw_wr_en_i,
    .lc_iso_part_sw_rd_en_i,
    .bank_cfg_i(reg2hw.mp_bank_cfg_shadowed),
    .region_i(reg2hw.mp_region),
    .region_cfg_i(reg2hw.mp_region_cfg),
    .default_cfg_i(reg2hw.default_region),
    .bank0_info0_cfg_i(reg2hw.bank0_info0_page_cfg),
    .bank0_info1_cfg_i(reg2hw.bank0_info1_page_cfg),
    .bank0_info2_cfg_i(reg2hw.bank0_info2_page_cfg),
    .bank1_info0_cfg_i(reg2hw.bank1_info0_page_cfg),
    .bank1_info1_cfg_i(reg2hw.bank1_info1_page_cfg),
    .bank1_info2_cfg_i(reg2hw.bank1_info2_page_cfg),
    .bank_cfg_o(bank_cfgs),
    .region_cfgs_o(region_cfgs),
    .info_page_cfgs_o(info_page_cfgs)
  );

  // FIFO Connections
  localparam int ProgDepthW = prim_util_pkg::vbits(ProgFifoDepth+1);
  localparam int RdDepthW   = prim_util_pkg::vbits(RdFifoDepth+1);

  logic                    prog_fifo_wvalid;
  logic                    prog_fifo_wready;
  logic                    prog_fifo_rvalid;
  logic                    prog_fifo_ren;
  logic [BusFullWidth-1:0] prog_fifo_wdata;
  logic [BusFullWidth-1:0] prog_fifo_rdata;
  logic [ProgDepthW-1:0]   prog_fifo_depth;

  // Program Control Connections
  logic prog_flash_req;
  logic prog_flash_ovfl;
  logic [BusAddrW-1:0] prog_flash_addr;
  logic prog_op_valid;

  // Read Control Connections
  logic rd_flash_req;
  logic rd_flash_ovfl;
  logic [BusAddrW-1:0] rd_flash_addr;
  logic rd_op_valid;
  logic                    rd_ctrl_wen;
  logic [BusFullWidth-1:0] rd_ctrl_wdata;


  // Erase Control Connections
  logic erase_flash_req;
  logic [BusAddrW-1:0] erase_flash_addr;
  flash_erase_e erase_flash_type;
  logic erase_op_valid;

  // Done / Error signaling from ctrl modules
  logic prog_done, rd_done, erase_done;
  flash_ctrl_err_t prog_err, rd_err, erase_err;
  logic [BusAddrW-1:0] prog_err_addr, rd_err_addr, erase_err_addr;

  // Flash Memory Properties Connections
  logic [BusAddrW-1:0] flash_addr;
  logic flash_req;
  logic flash_rd_done, flash_prog_done, flash_erase_done;
  logic flash_mp_err;
  logic [BusFullWidth-1:0] flash_prog_data;
  logic flash_prog_last;
  flash_prog_e flash_prog_type;
  logic [BusFullWidth-1:0] flash_rd_data;
  logic flash_rd_err;
  logic flash_phy_busy;
  logic rd_op;
  logic prog_op;
  logic erase_op;
  flash_lcmgr_phase_e phase;

  // Flash control arbitration connections to hardware interface
  flash_key_t addr_key;
  flash_key_t rand_addr_key;
  flash_key_t data_key;
  flash_key_t rand_data_key;
  flash_ctrl_reg2hw_control_reg_t hw_ctrl;
  logic hw_req;
  logic [BusAddrByteW-1:0] hw_addr;
  logic hw_done;
  flash_ctrl_err_t hw_err;
  logic hw_wvalid;
  logic [BusFullWidth-1:0] hw_wdata;
  logic hw_wready;
  flash_sel_e if_sel;
  logic sw_sel;
  flash_lcmgr_phase_e hw_phase;
  logic lcmgr_err;
  logic lcmgr_intg_err;
  logic arb_fsm_err;
  logic seed_err;

  // Flash lcmgr interface to direct read fifo
  logic lcmgr_rready;

  // Flash control arbitration connections to software interface
  logic sw_ctrl_done;
  flash_ctrl_err_t sw_ctrl_err;

  // Flash control muxed connections
  flash_ctrl_reg2hw_control_reg_t muxed_ctrl;
  logic [BusAddrByteW-1:0] muxed_addr;
  logic op_start;
  logic [11:0] op_num_words;
  logic [BusAddrW-1:0] op_addr;
  logic [BusAddrW-1:0] ctrl_err_addr;
  flash_op_e op_type;
  flash_part_e op_part;
  logic [InfoTypesWidth-1:0] op_info_sel;
  flash_erase_e op_erase_type;
  flash_prog_e op_prog_type;

  logic ctrl_init_busy;
  logic ctrl_initialized;
  logic fifo_clr;

  // sw read fifo interface
  logic sw_rfifo_wen;
  logic sw_rfifo_wready;
  logic [BusFullWidth-1:0] sw_rfifo_wdata;
  logic sw_rfifo_full;
  logic [RdDepthW-1:0] sw_rfifo_depth;
  logic sw_rfifo_rvalid;
  logic sw_rfifo_rready;
  logic [BusFullWidth-1:0] sw_rfifo_rdata;

  // software tlul interface to read fifo
  logic adapter_req;
  logic adapter_rvalid;
  logic adapter_fifo_err;

  // software tlul interface to prog fifo
  logic sw_wvalid;
  logic [BusFullWidth-1:0] sw_wdata;
  logic sw_wready;

  // lfsr for local entropy usage
  logic [31:0] rand_val;
  logic lfsr_en;
  logic lfsr_seed_en;

  // interface to flash phy
  flash_rsp_t flash_phy_rsp;
  flash_req_t flash_phy_req;

  // import commonly used routines
  import lc_ctrl_pkg::lc_tx_test_true_strict;

  // life cycle connections
  lc_ctrl_pkg::lc_tx_t lc_seed_hw_rd_en;

  lc_ctrl_pkg::lc_tx_t dis_access;

  prim_lc_sync #(
    .NumCopies(1)
  ) u_lc_seed_hw_rd_en_sync (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_seed_hw_rd_en_i),
    .lc_en_o({lc_seed_hw_rd_en})
  );

  prim_lfsr #(
    .EntropyDw(EdnWidth),
    .LfsrDw(LfsrWidth),
    .StateOutDw(LfsrWidth),
    .DefaultSeed(RndCnstLfsrSeed),
    .StatePermEn(1),
    .StatePerm(RndCnstLfsrPerm)
  ) u_lfsr (
    .clk_i,
    .rst_ni,
    .seed_en_i(lfsr_seed_en),
    .seed_i(rma_seed_i),
    .lfsr_en_i(lfsr_en),
    .entropy_i('0),
    .state_o(rand_val)
  );

  // flash disable declaration
  mubi4_t [FlashDisableLast-1:0] flash_disable;

  // flash control arbitration between software and hardware interfaces
  flash_ctrl_arb u_ctrl_arb (
    .clk_i,
    .rst_ni,

    // combined disable
    .disable_i(flash_disable[ArbFsmDisableIdx]),

    // error output shared by both interfaces
    .ctrl_err_addr_o(ctrl_err_addr),

    // software interface to rd_ctrl / erase_ctrl
    .sw_ctrl_i(reg2hw.control),
    .sw_addr_i(reg2hw.addr.q),
    .sw_ack_o(sw_ctrl_done),
    .sw_err_o(sw_ctrl_err),

    // software interface to prog_fifo
    // if prog operation not selected, software interface
    // writes have no meaning
    .sw_wvalid_i(sw_wvalid & prog_op_valid),
    .sw_wdata_i(sw_wdata),
    .sw_wready_o(sw_wready),

    // hardware interface to rd_ctrl / erase_ctrl
    .hw_req_i(hw_req),
    .hw_ctrl_i(hw_ctrl),

    // hardware interface indicating operation phase
    .hw_phase_i(hw_phase),

    // hardware works on word address, however software expects byte address
    .hw_addr_i(hw_addr),
    .hw_ack_o(hw_done),
    .hw_err_o(hw_err),

    // hardware interface to rd_fifo
    .hw_wvalid_i(hw_wvalid),
    .hw_wdata_i(hw_wdata),
    .hw_wready_o(hw_wready),

    // hardware interface does not talk to prog_fifo

    // muxed interface to rd_ctrl / erase_ctrl
    .muxed_ctrl_o(muxed_ctrl),
    .muxed_addr_o(muxed_addr),
    .prog_ack_i(prog_done),
    .prog_err_i(prog_err),
    .prog_err_addr_i(prog_err_addr),
    .rd_ack_i(rd_done),
    .rd_err_i(rd_err),
    .rd_err_addr_i(rd_err_addr),
    .erase_ack_i(erase_done),
    .erase_err_i(erase_err),
    .erase_err_addr_i(erase_err_addr),

    // muxed interface to prog_fifo
    .prog_fifo_wvalid_o(prog_fifo_wvalid),
    .prog_fifo_wdata_o(prog_fifo_wdata),
    .prog_fifo_wready_i(prog_fifo_wready),

    // flash phy initialization ongoing
    .flash_phy_busy_i(flash_phy_busy),

    // clear fifos
    .fifo_clr_o(fifo_clr),

    // phase indication
    .phase_o(phase),

    // indication that sw has been selected
    .sel_o(if_sel),
    .fsm_err_o(arb_fsm_err)
  );

  assign op_start      = muxed_ctrl.start.q;
  assign op_num_words  = muxed_ctrl.num.q;
  assign op_erase_type = flash_erase_e'(muxed_ctrl.erase_sel.q);
  assign op_prog_type  = flash_prog_e'(muxed_ctrl.prog_sel.q);
  assign op_addr       = muxed_addr[BusByteWidth +: BusAddrW];
  assign op_type       = flash_op_e'(muxed_ctrl.op.q);
  assign op_part       = flash_part_e'(muxed_ctrl.partition_sel.q);
  assign op_info_sel   = muxed_ctrl.info_sel.q;
  assign rd_op         = op_type == FlashOpRead;
  assign prog_op       = op_type == FlashOpProgram;
  assign erase_op      = op_type == FlashOpErase;
  assign sw_sel        = if_sel == SwSel;

  // hardware interface
  flash_ctrl_lcmgr #(
    .RndCnstAddrKey(RndCnstAddrKey),
    .RndCnstDataKey(RndCnstDataKey),
    .RndCnstAllSeeds(RndCnstAllSeeds)
  ) u_flash_hw_if (
    .clk_i,
    .rst_ni,
    .clk_otp_i,
    .rst_otp_ni,

    .init_i(reg2hw.init),
    .provision_en_i(lc_tx_test_true_strict(lc_seed_hw_rd_en)),

    // combined disable
    .disable_i(flash_disable[LcMgrDisableIdx]),

    // interface to ctrl arb control ports
    .ctrl_o(hw_ctrl),
    .req_o(hw_req),
    .addr_o(hw_addr),
    .done_i(hw_done),
    .err_i(hw_err),

    // interface to ctrl_arb data ports
    .wready_i(hw_wready),
    .wvalid_o(hw_wvalid),
    .wdata_o(hw_wdata),

    // interface to hw interface read fifo
    .rready_o(lcmgr_rready),
    .rvalid_i(~sw_sel & rd_ctrl_wen),
    .rdata_i(rd_ctrl_wdata),

    // external rma request
    .rma_req_i,
    .rma_ack_o,

    // outgoing seeds
    .seeds_o(keymgr_o.seeds),
    .seed_err_o(seed_err),

    // phase indication
    .phase_o(hw_phase),

    // phy read buffer enable
    .rd_buf_en_o(flash_phy_req.rd_buf_en),

    // connection to otp
    .otp_key_req_o(otp_o),
    .otp_key_rsp_i(otp_i),
    .addr_key_o(addr_key),
    .data_key_o(data_key),
    .rand_addr_key_o(rand_addr_key),
    .rand_data_key_o(rand_data_key),

    // entropy interface
    .edn_req_o(lfsr_seed_en),
    .edn_ack_i(1'b1),
    .lfsr_en_o(lfsr_en),
    .rand_i(rand_val),

    // error indication
    .fatal_err_o(lcmgr_err),
    .intg_err_o(lcmgr_intg_err),

    // disable access to flash storage after rma process
    .dis_access_o(dis_access),

    // init ongoing
    .init_busy_o(ctrl_init_busy),
    .initialized_o(ctrl_initialized),

    .debug_state_o(hw2reg.debug_state.d)
  );




  // Program FIFO
  // Since the program and read FIFOs are never used at the same time, it should really be one
  // FIFO with muxed inputs and outputs.  This should be addressed once the flash integration
  // strategy has been identified
  assign prog_op_valid = op_start & prog_op;

  tlul_pkg::tl_h2d_t prog_tl_h2d;
  tlul_pkg::tl_d2h_t prog_tl_d2h;

  // the program path also needs an lc gate to error back when flash is disabled.
  // This is because tlul_adapter_sram does not actually have a way of signaling
  // write errors, only read errors.
  // SEC_CM: PROG_TL_LC_GATE.FSM.SPARSE
  tlul_lc_gate u_prog_tl_gate (
    .clk_i,
    .rst_ni,
    .tl_h2d_i(tl_win_h2d[0]),
    .tl_d2h_o(tl_win_d2h[0]),
    .tl_h2d_o(prog_tl_h2d),
    .tl_d2h_i(prog_tl_d2h),
    .flush_req_i('0),
    .flush_ack_o(),
    .resp_pending_o(),
    .lc_en_i(lc_ctrl_pkg::mubi4_to_lc_inv(flash_disable[ProgFifoIdx])),
    .err_o(tl_prog_gate_intg_err)
  );

  tlul_adapter_sram #(
    .SramAw(1),          //address unused
    .SramDw(BusWidth),
    .ByteAccess(0),      //flash may not support byte access
    .ErrOnRead(1),       //reads not supported
    .EnableDataIntgPt(1) //passthrough data integrity
  ) u_to_prog_fifo (
    .clk_i,
    .rst_ni,
    .tl_i                       (prog_tl_h2d),
    .tl_o                       (prog_tl_d2h),
    .en_ifetch_i                (prim_mubi_pkg::MuBi4False),
    .req_o                      (sw_wvalid),
    .req_type_o                 (),
    .gnt_i                      (sw_wready),
    .we_o                       (),
    .addr_o                     (),
    .wmask_o                    (),
    .intg_error_o               (),
    .wdata_o                    (sw_wdata),
    .rdata_i                    ('0),
    .rvalid_i                   (1'b0),
    .rerror_i                   (2'b0),
    .compound_txn_in_progress_o (),
    .readback_en_i              (prim_mubi_pkg::MuBi4False),
    .readback_error_o           (),
    .wr_collision_i             (1'b0),
    .write_pending_i            (1'b0)
  );

  prim_fifo_sync #(
    .Width(BusFullWidth),
    .Depth(ProgFifoDepth)
  ) u_prog_fifo (
    .clk_i,
    .rst_ni,
    .clr_i   (reg2hw.fifo_rst.q | fifo_clr | sw_ctrl_done),
    .wvalid_i(prog_fifo_wvalid),
    .wready_o(prog_fifo_wready),
    .wdata_i (prog_fifo_wdata),
    .depth_o (prog_fifo_depth),
    .full_o  (),
    .rvalid_o(prog_fifo_rvalid),
    .rready_i(prog_fifo_ren),
    .rdata_o (prog_fifo_rdata),
    .err_o   ()
  );
  assign hw2reg.curr_fifo_lvl.prog.d = MaxFifoWidth'(prog_fifo_depth);

  // Program handler is consumer of prog_fifo
  logic [1:0] prog_type_en;
  assign prog_type_en[FlashProgNormal] = flash_phy_rsp.prog_type_avail[FlashProgNormal] &
                                         reg2hw.prog_type_en.normal.q;
  assign prog_type_en[FlashProgRepair] = flash_phy_rsp.prog_type_avail[FlashProgRepair] &
                                         reg2hw.prog_type_en.repair.q;

  logic prog_cnt_err;
  flash_ctrl_prog u_flash_ctrl_prog (
    .clk_i,
    .rst_ni,

    // Control interface
    .op_start_i     (prog_op_valid),
    .op_num_words_i (op_num_words),
    .op_done_o      (prog_done),
    .op_err_o       (prog_err),
    .op_addr_i      (op_addr),
    .op_addr_oob_i  ('0),
    .op_type_i      (op_prog_type),
    .type_avail_i   (prog_type_en),
    .op_err_addr_o  (prog_err_addr),
    .cnt_err_o      (prog_cnt_err),

    // FIFO Interface
    .data_i         (prog_fifo_rdata),
    .data_rdy_i     (prog_fifo_rvalid),
    .data_rd_o      (prog_fifo_ren),

    // Flash Macro Interface
    .flash_req_o    (prog_flash_req),
    .flash_addr_o   (prog_flash_addr),
    .flash_ovfl_o   (prog_flash_ovfl),
    .flash_data_o   (flash_prog_data),
    .flash_last_o   (flash_prog_last),
    .flash_type_o   (flash_prog_type),
    .flash_done_i   (flash_prog_done),
    .flash_prog_intg_err_i (flash_phy_rsp.prog_intg_err),
    .flash_mp_err_i (flash_mp_err)
  );



  // a read request is seen from software but a read operation is not enabled
  // AND there are no pending entries to read from the fifo.
  // This indicates software has issued a read when it should not have.
  logic rd_no_op_d, rd_no_op_q;
  logic sw_rd_op;
  assign sw_rd_op = reg2hw.control.start.q & (reg2hw.control.op.q == FlashOpRead);

  // If software ever attempts to read when the FIFO is empty AND if it has never
  // initiated a transaction, OR when flash is disabled, then it is a read that
  // can never complete, error back immediately.
  assign rd_no_op_d = adapter_req & ((~sw_rd_op & ~sw_rfifo_rvalid) |
                      (prim_mubi_pkg::mubi4_test_true_loose(flash_disable[RdFifoIdx])));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      adapter_rvalid <= 1'b0;
      rd_no_op_q <= 1'b0;
    end else begin
      adapter_rvalid <= adapter_req & sw_rfifo_rvalid;
      rd_no_op_q <= rd_no_op_d;
    end
  end

  // tlul adapter represents software's access interface to flash
  tlul_adapter_sram #(
    .SramAw(1),           //address unused
    .SramDw(BusWidth),
    .ByteAccess(0),       //flash may not support byte access
    .ErrOnWrite(1),       //writes not supported
    .EnableDataIntgPt(1),
    .SecFifoPtr(1)        // SEC_CM: FIFO.CTR.REDUN
  ) u_to_rd_fifo (
    .clk_i,
    .rst_ni,
    .tl_i                       (tl_win_h2d[1]),
    .tl_o                       (tl_win_d2h[1]),
    .en_ifetch_i                (prim_mubi_pkg::MuBi4False),
    .req_o                      (adapter_req),
    .req_type_o                 (),
    // if there is no valid read operation, don't hang the
    // bus, just let things normally return
    .gnt_i                      (sw_rfifo_rvalid | rd_no_op_d),
    .we_o                       (),
    .addr_o                     (),
    .wmask_o                    (),
    .wdata_o                    (),
    .intg_error_o               (adapter_fifo_err),
    .rdata_i                    (sw_rfifo_rdata),
    .rvalid_i                   (adapter_rvalid | rd_no_op_q),
    .rerror_i                   ({rd_no_op_q, 1'b0}),
    .compound_txn_in_progress_o (),
    .readback_en_i              (prim_mubi_pkg::MuBi4False),
    .readback_error_o           (),
    .wr_collision_i             (1'b0),
    .write_pending_i            (1'b0)
  );

  assign sw_rfifo_wen = sw_sel & rd_ctrl_wen;
  assign sw_rfifo_wdata = rd_ctrl_wdata;
  assign sw_rfifo_rready = adapter_rvalid;

  // the read fifo below is dedicated to the software read path.
  prim_fifo_sync #(
    .Width(BusFullWidth),
    .Depth(RdFifoDepth)
  ) u_sw_rd_fifo (
    .clk_i,
    .rst_ni,
    .clr_i   (reg2hw.fifo_rst.q),
    .wvalid_i(sw_rfifo_wen),
    .wready_o(sw_rfifo_wready),
    .wdata_i (sw_rfifo_wdata),
    .full_o  (sw_rfifo_full),
    .depth_o (sw_rfifo_depth),
    .rvalid_o(sw_rfifo_rvalid),
    .rready_i(sw_rfifo_rready),
    .rdata_o (sw_rfifo_rdata),
    .err_o   ()
  );
  assign hw2reg.curr_fifo_lvl.rd.d = sw_rfifo_depth;

  logic rd_cnt_err;
  // Read handler is consumer of rd_fifo
  assign rd_op_valid = op_start & rd_op;
  flash_ctrl_rd  u_flash_ctrl_rd (
    .clk_i,
    .rst_ni,

    // To arbiter Interface
    .op_start_i     (rd_op_valid),
    .op_num_words_i (op_num_words),
    .op_done_o      (rd_done),
    .op_err_o       (rd_err),
    .op_err_addr_o  (rd_err_addr),
    .op_addr_i      (op_addr),
    .op_addr_oob_i  ('0),
    .cnt_err_o      (rd_cnt_err),

    // FIFO Interface
    .data_rdy_i     (sw_sel ? sw_rfifo_wready : lcmgr_rready),
    .data_o         (rd_ctrl_wdata),
    .data_wr_o      (rd_ctrl_wen),

    // Flash Macro Interface
    .flash_req_o    (rd_flash_req),
    .flash_addr_o   (rd_flash_addr),
    .flash_ovfl_o   (rd_flash_ovfl),
    .flash_data_i   (flash_rd_data),
    .flash_done_i   (flash_rd_done),
    .flash_mp_err_i (flash_mp_err),
    .flash_rd_err_i (flash_rd_err)
  );

  // Erase handler does not consume fifo
  assign erase_op_valid = op_start & erase_op;
  flash_ctrl_erase u_flash_ctrl_erase (
    // Software Interface
    .op_start_i     (erase_op_valid),
    .op_type_i      (op_erase_type),
    .op_done_o      (erase_done),
    .op_err_o       (erase_err),
    .op_addr_i      (op_addr),
    .op_addr_oob_i  ('0),
    .op_err_addr_o  (erase_err_addr),

    // Flash Macro Interface
    .flash_req_o    (erase_flash_req),
    .flash_addr_o   (erase_flash_addr),
    .flash_op_o     (erase_flash_type),
    .flash_done_i   (flash_erase_done),
    .flash_mp_err_i (flash_mp_err)
  );

  // Final muxing to flash macro module
  always_comb begin
    unique case (op_type)
      FlashOpRead: begin
        flash_req = rd_flash_req;
        flash_addr = rd_flash_addr;
      end
      FlashOpProgram: begin
        flash_req = prog_flash_req;
        flash_addr = prog_flash_addr;
      end
      FlashOpErase: begin
        flash_req = erase_flash_req;
        flash_addr = erase_flash_addr;
      end
      default: begin
        flash_req = 1'b0;
        flash_addr  = '0;
      end
    endcase // unique case (op_type)
  end



  //////////////////////////////////////
  // Info partition properties configuration
  //////////////////////////////////////


  //////////////////////////////////////
  // flash memory properties
  //////////////////////////////////////
  // direct assignment since prog/rd/erase_ctrl do not make use of op_part
  flash_part_e flash_part_sel;
  logic [InfoTypesWidth-1:0] flash_info_sel;
  assign flash_part_sel = op_part;
  assign flash_info_sel = op_info_sel;

  // tie off hardware clear path
  assign hw2reg.erase_suspend.d = 1'b0;

  // Flash memory Properties
  // Memory property is page based and thus should use phy addressing
  // This should move to flash_phy long term
  lc_ctrl_pkg::lc_tx_t lc_escalate_en;
  flash_mp u_flash_mp (
    .clk_i,
    .rst_ni,

    // This is only used in SVAs, hence we do not have to feed in a copy.
    .lc_escalate_en_i(lc_escalate_en),

    // disable flash through memory protection
    .flash_disable_i(flash_disable[MpDisableIdx]),

    // hw info configuration overrides
    .hw_info_scramble_dis_i(mubi4_t'(reg2hw.hw_info_cfg_override.scramble_dis.q)),
    .hw_info_ecc_dis_i(mubi4_t'(reg2hw.hw_info_cfg_override.ecc_dis.q)),

    // arbiter interface selection
    .if_sel_i(if_sel),

    // sw configuration for data partition
    .region_cfgs_i(region_cfgs),
    .bank_cfgs_i(bank_cfgs),

    // sw configuration for info partition
    .info_page_cfgs_i(info_page_cfgs),

    // read / prog / erase controls
    .req_i(flash_req),
    .phase_i(phase),
    .req_addr_i(flash_addr[BusAddrW-1 -: AllPagesW]),
    .req_part_i(flash_part_sel),
    .info_sel_i(flash_info_sel),
    .addr_ovfl_i(rd_flash_ovfl | prog_flash_ovfl),
    .rd_i(rd_op),
    .prog_i(prog_op),
    .pg_erase_i(erase_op & (erase_flash_type == FlashErasePage)),
    .bk_erase_i(erase_op & (erase_flash_type == FlashEraseBank)),
    .erase_suspend_i(reg2hw.erase_suspend),
    .erase_suspend_done_o(hw2reg.erase_suspend.de),
    .rd_done_o(flash_rd_done),
    .prog_done_o(flash_prog_done),
    .erase_done_o(flash_erase_done),
    .error_o(flash_mp_err),

    // flash phy interface
    .req_o(flash_phy_req.req),
    .scramble_en_o(flash_phy_req.scramble_en),
    .ecc_en_o(flash_phy_req.ecc_en),
    .he_en_o(flash_phy_req.he_en),
    .rd_o(flash_phy_req.rd),
    .prog_o(flash_phy_req.prog),
    .pg_erase_o(flash_phy_req.pg_erase),
    .bk_erase_o(flash_phy_req.bk_erase),
    .erase_suspend_o(flash_phy_req.erase_suspend),
    .rd_done_i(flash_phy_rsp.rd_done),
    .prog_done_i(flash_phy_rsp.prog_done),
    .erase_done_i(flash_phy_rsp.erase_done)
  );


  // software interface feedback
  // most values (other than flash_phy_busy) should only update when software operations
  // are actually selected
  assign hw2reg.op_status.done.d     = 1'b1;
  assign hw2reg.op_status.done.de    = sw_ctrl_done;
  assign hw2reg.op_status.err.d      = 1'b1;
  assign hw2reg.op_status.err.de     = |sw_ctrl_err;
  assign hw2reg.status.rd_full.d     = sw_rfifo_full;
  assign hw2reg.status.rd_full.de    = sw_sel;
  assign hw2reg.status.rd_empty.d    = ~sw_rfifo_rvalid;
  assign hw2reg.status.rd_empty.de   = sw_sel;
  assign hw2reg.status.prog_full.d   = ~prog_fifo_wready;
  assign hw2reg.status.prog_full.de  = sw_sel;
  assign hw2reg.status.prog_empty.d  = ~prog_fifo_rvalid;
  assign hw2reg.status.prog_empty.de = sw_sel;
  assign hw2reg.status.init_wip.d    = flash_phy_busy | ctrl_init_busy;
  assign hw2reg.status.init_wip.de   = 1'b1;
  assign hw2reg.status.initialized.d  = ctrl_initialized & ~flash_phy_busy;
  assign hw2reg.status.initialized.de = 1'b1;
  assign hw2reg.control.start.d      = 1'b0;
  assign hw2reg.control.start.de     = sw_ctrl_done;
  // if software operation selected, based on transaction start
  // if software operation not selected, software is free to change contents
  assign hw2reg.ctrl_regwen.d        = sw_sel ? !op_start : 1'b1;

  // phy status
  assign hw2reg.phy_status.init_wip.d  = flash_phy_busy;
  assign hw2reg.phy_status.init_wip.de = 1'b1;
  assign hw2reg.phy_status.prog_normal_avail.d  = flash_phy_rsp.prog_type_avail[FlashProgNormal];
  assign hw2reg.phy_status.prog_normal_avail.de = 1'b1;
  assign hw2reg.phy_status.prog_repair_avail.d  = flash_phy_rsp.prog_type_avail[FlashProgRepair];
  assign hw2reg.phy_status.prog_repair_avail.de = 1'b1;

  // Flash Interface
  assign flash_phy_req.addr = flash_addr;
  assign flash_phy_req.part = flash_part_sel;
  assign flash_phy_req.info_sel = flash_info_sel;
  assign flash_phy_req.prog_type = flash_prog_type;
  assign flash_phy_req.prog_data = flash_prog_data;
  assign flash_phy_req.prog_last = flash_prog_last;
  assign flash_phy_req.region_cfgs = region_cfgs;
  assign flash_phy_req.addr_key = addr_key;
  assign flash_phy_req.data_key = data_key;
  assign flash_phy_req.rand_addr_key = rand_addr_key;
  assign flash_phy_req.rand_data_key = rand_data_key;
  assign flash_phy_req.alert_trig = reg2hw.phy_alert_cfg.alert_trig.q;
  assign flash_phy_req.alert_ack = reg2hw.phy_alert_cfg.alert_ack.q;
  assign flash_phy_req.jtag_req.tck = cio_tck_i;
  assign flash_phy_req.jtag_req.tms = cio_tms_i;
  assign flash_phy_req.jtag_req.tdi = cio_tdi_i;
  assign flash_phy_req.jtag_req.trst_n = '0;
  assign cio_tdo_o = flash_phy_rsp.jtag_rsp.tdo;
  assign cio_tdo_en_o = flash_phy_rsp.jtag_rsp.tdo_oe;
  assign flash_rd_err = flash_phy_rsp.rd_err;
  assign flash_rd_data = flash_phy_rsp.rd_data;
  assign flash_phy_busy = flash_phy_rsp.init_busy;


  // Interface to pwrmgr
  // flash is not idle as long as there is a stateful operation ongoing
  logic flash_idle_d;
  assign flash_idle_d = ~(flash_phy_req.req &
                          (flash_phy_req.prog | flash_phy_req.pg_erase | flash_phy_req.bk_erase));

  prim_flop #(
    .Width(1),
    .ResetValue(1'b1)
  ) u_reg_idle (
    .clk_i,
    .rst_ni,
    .d_i(flash_idle_d),
    .q_o(pwrmgr_o.flash_idle)
  );

  //////////////////////////////////////
  // Alert senders
  //////////////////////////////////////


  logic [NumAlerts-1:0] alert_srcs;
  logic [NumAlerts-1:0] alert_tests;
  logic fatal_prim_flash_alert, recov_prim_flash_alert;

  // An excessive number of recoverable errors may also indicate an attack
  logic recov_err;
  assign recov_err = (sw_ctrl_done & |sw_ctrl_err) |
                     flash_phy_rsp.macro_err |
                     update_err;

  logic fatal_err;
  assign fatal_err = |reg2hw.fault_status;

  logic fatal_std_err;
  assign fatal_std_err = |reg2hw.std_fault_status;

  lc_ctrl_pkg::lc_tx_t local_esc;
  assign local_esc = lc_ctrl_pkg::lc_tx_bool_to_lc_tx(fatal_std_err);

  assign alert_srcs = {
    recov_prim_flash_alert,
    fatal_prim_flash_alert,
    fatal_err,
    fatal_std_err,
    recov_err
  };

  assign alert_tests = {
    reg2hw.alert_test.recov_prim_flash_alert.q & reg2hw.alert_test.recov_prim_flash_alert.qe,
    reg2hw.alert_test.fatal_prim_flash_alert.q & reg2hw.alert_test.fatal_prim_flash_alert.qe,
    reg2hw.alert_test.fatal_err.q & reg2hw.alert_test.fatal_err.qe,
    reg2hw.alert_test.fatal_std_err.q & reg2hw.alert_test.fatal_std_err.qe,
    reg2hw.alert_test.recov_err.q & reg2hw.alert_test.recov_err.qe
  };

  // The alert generated for errors reported in the fault status CSR (fatal_err) is not fatal.
  // This is to enable firmware dealing with multi-bit ECC errors (phy_relbl_err) as well as ICV
  // (phy_storage_err) errors inside the PHY during firmware selection and verification.
  // Once firmware has cleared the corresponding bits in the fault status CSR and the alert
  // handler has acknowledged the alert, the prim_alert_sender will stop triggering the alert.
  // After firmware has passed the firmware selection / verification stage, the alert handler
  // config can be adjusted to still classify the alert as fatal on the receiver side.
  //
  // This doesn't hold for the other errors conditions reported in the fault status CSR. The
  // corresponding bits in the register cannot be unset. The alert thus keeps triggering until
  // reset for these bits.
  //
  // For more details, refer to lowRISC/OpenTitan#21353.
  localparam logic [NumAlerts-1:0] IsFatal = {1'b0, 1'b1, 1'b0, 1'b1, 1'b0};
  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_senders
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(IsFatal[i])
    ) u_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_req_i(alert_srcs[i]),
      .alert_test_i(alert_tests[i]),
      .alert_ack_o(),
      .alert_state_o(),
      .alert_rx_i(alert_rx_i[i]),
      .alert_tx_o(alert_tx_o[i])
    );
  end

  //////////////////////////////////////
  // Flash Disable and execute enable
  //////////////////////////////////////

  prim_lc_sync #(
    .NumCopies(1)
  ) u_lc_escalation_en_sync (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_escalate_en_i),
    .lc_en_o({lc_escalate_en})
  );

  lc_ctrl_pkg::lc_tx_t escalate_en;
  // SEC_CM: MEM.CTRL.LOCAL_ESC
  assign escalate_en = lc_ctrl_pkg::lc_tx_or_hi(dis_access, local_esc);

  // flash functional disable
  lc_ctrl_pkg::lc_tx_t lc_disable;
  assign lc_disable = lc_ctrl_pkg::lc_tx_or_hi(lc_escalate_en, escalate_en);

  // Normally, faults (those registered in fault_status) should also cause flash access
  // to disable.  However, most errors encountered by hardware during flash access
  // are registered as faults (since they functionally never happen).  Out of an abundance
  // of caution for the first iteration, we will not kill flash access based on those
  // faults immediately just in case there are unexpected corner conditions.
  // In other words...cowardice.
  // SEC_CM: MEM.CTRL.GLOBAL_ESC
  // SEC_CM: MEM_DISABLE.CONFIG.MUBI
  mubi4_t lc_conv_disable;
  mubi4_t flash_disable_pre_buf;
  assign lc_conv_disable = lc_ctrl_pkg::lc_to_mubi4(lc_disable);
  assign flash_disable_pre_buf = prim_mubi_pkg::mubi4_or_hi(
      lc_conv_disable,
      mubi4_t'(reg2hw.dis.q));

  prim_mubi4_sync #(
    .NumCopies(int'(FlashDisableLast)),
    .AsyncOn(0)
  ) u_disable_buf (
    .clk_i,
    .rst_ni,
    .mubi_i(flash_disable_pre_buf),
    .mubi_o(flash_disable)
  );

  assign flash_phy_req.flash_disable = flash_disable[PhyDisableIdx];

  logic [prim_mubi_pkg::MuBi4Width-1:0] sw_flash_exec_en;
  mubi4_t flash_exec_en;

  // SEC_CM: EXEC.CONFIG.REDUN
  prim_sec_anchor_buf #(
    .Width(prim_mubi_pkg::MuBi4Width)
  ) u_exec_en_buf (
    .in_i(prim_mubi_pkg::mubi4_bool_to_mubi(reg2hw.exec.q == unsigned'(ExecEn))),
    .out_o(sw_flash_exec_en)
  );

  mubi4_t disable_exec;
  assign disable_exec = mubi4_t'(~flash_disable[IFetchDisableIdx]);
  assign flash_exec_en = prim_mubi_pkg::mubi4_and_hi(
                           disable_exec,
                           mubi4_t'(sw_flash_exec_en)
                         );

  //////////////////////////////////////
  // Errors and Interrupts
  //////////////////////////////////////

  // all software interface errors are treated as synchronous errors
  assign hw2reg.err_code.op_err.d           = 1'b1;
  assign hw2reg.err_code.mp_err.d           = 1'b1;
  assign hw2reg.err_code.rd_err.d           = 1'b1;
  assign hw2reg.err_code.prog_err.d         = 1'b1;
  assign hw2reg.err_code.prog_win_err.d     = 1'b1;
  assign hw2reg.err_code.prog_type_err.d    = 1'b1;
  assign hw2reg.err_code.update_err.d       = 1'b1;
  assign hw2reg.err_code.macro_err.d        = 1'b1;
  assign hw2reg.err_code.op_err.de          = sw_ctrl_err.invalid_op_err;
  assign hw2reg.err_code.mp_err.de          = sw_ctrl_err.mp_err;
  assign hw2reg.err_code.rd_err.de          = sw_ctrl_err.rd_err;
  assign hw2reg.err_code.prog_err.de        = sw_ctrl_err.prog_err;
  assign hw2reg.err_code.prog_win_err.de    = sw_ctrl_err.prog_win_err;
  assign hw2reg.err_code.prog_type_err.de   = sw_ctrl_err.prog_type_err;
  assign hw2reg.err_code.update_err.de      = update_err;
  assign hw2reg.err_code.macro_err.de       = flash_phy_rsp.macro_err;
  assign hw2reg.err_addr.d                  = {ctrl_err_addr, {BusByteWidth{1'h0}}};
  assign hw2reg.err_addr.de                 = sw_ctrl_err.mp_err |
                                              sw_ctrl_err.rd_err |
                                              sw_ctrl_err.prog_err;


  // all hardware interface errors are considered faults
  // There are two types of faults
  // standard faults - things like fsm / counter / tlul integrity
  // custom faults - things like hardware interface not working correctly
  assign hw2reg.fault_status.op_err.d           = 1'b1;
  assign hw2reg.fault_status.mp_err.d           = 1'b1;
  assign hw2reg.fault_status.rd_err.d           = 1'b1;
  assign hw2reg.fault_status.prog_err.d         = 1'b1;
  assign hw2reg.fault_status.prog_win_err.d     = 1'b1;
  assign hw2reg.fault_status.prog_type_err.d    = 1'b1;
  assign hw2reg.fault_status.seed_err.d         = 1'b1;
  assign hw2reg.fault_status.phy_relbl_err.d    = 1'b1;
  assign hw2reg.fault_status.phy_storage_err.d  = 1'b1;
  assign hw2reg.fault_status.spurious_ack.d     = 1'b1;
  assign hw2reg.fault_status.arb_err.d          = 1'b1;
  assign hw2reg.fault_status.host_gnt_err.d     = 1'b1;
  assign hw2reg.fault_status.op_err.de          = hw_err.invalid_op_err;
  assign hw2reg.fault_status.mp_err.de          = hw_err.mp_err;
  assign hw2reg.fault_status.rd_err.de          = hw_err.rd_err;
  assign hw2reg.fault_status.prog_err.de        = hw_err.prog_err;
  assign hw2reg.fault_status.prog_win_err.de    = hw_err.prog_win_err;
  assign hw2reg.fault_status.prog_type_err.de   = hw_err.prog_type_err;
  assign hw2reg.fault_status.seed_err.de        = seed_err;
  assign hw2reg.fault_status.phy_relbl_err.de   = flash_phy_rsp.storage_relbl_err;
  assign hw2reg.fault_status.phy_storage_err.de = flash_phy_rsp.storage_intg_err;
  assign hw2reg.fault_status.spurious_ack.de    = flash_phy_rsp.spurious_ack;
  assign hw2reg.fault_status.arb_err.de         = flash_phy_rsp.arb_err;
  assign hw2reg.fault_status.host_gnt_err.de    = flash_phy_rsp.host_gnt_err;

  // standard faults
  assign hw2reg.std_fault_status.reg_intg_err.d    = 1'b1;
  assign hw2reg.std_fault_status.prog_intg_err.d   = 1'b1;
  assign hw2reg.std_fault_status.lcmgr_err.d       = 1'b1;
  assign hw2reg.std_fault_status.lcmgr_intg_err.d  = 1'b1;
  assign hw2reg.std_fault_status.arb_fsm_err.d     = 1'b1;
  assign hw2reg.std_fault_status.storage_err.d     = 1'b1;
  assign hw2reg.std_fault_status.phy_fsm_err.d     = 1'b1;
  assign hw2reg.std_fault_status.ctrl_cnt_err.d    = 1'b1;
  assign hw2reg.std_fault_status.fifo_err.d        = 1'b1;
  assign hw2reg.std_fault_status.reg_intg_err.de   = intg_err | eflash_cmd_intg_err |
                                                     tl_gate_intg_err | tl_prog_gate_intg_err;
  assign hw2reg.std_fault_status.prog_intg_err.de  = flash_phy_rsp.prog_intg_err;
  assign hw2reg.std_fault_status.lcmgr_err.de      = lcmgr_err;
  assign hw2reg.std_fault_status.lcmgr_intg_err.de = lcmgr_intg_err;
  assign hw2reg.std_fault_status.arb_fsm_err.de    = arb_fsm_err;
  assign hw2reg.std_fault_status.storage_err.de    = storage_err;
  assign hw2reg.std_fault_status.phy_fsm_err.de    = flash_phy_rsp.fsm_err;
  assign hw2reg.std_fault_status.ctrl_cnt_err.de   = rd_cnt_err | prog_cnt_err;
  assign hw2reg.std_fault_status.fifo_err.de       = flash_phy_rsp.fifo_err | adapter_fifo_err;

  // Correctable ECC count / address
  for (genvar i = 0; i < NumBanks; i++) begin : gen_ecc_single_err_reg
    assign hw2reg.ecc_single_err_cnt[i].de = flash_phy_rsp.ecc_single_err[i];
    assign hw2reg.ecc_single_err_cnt[i].d = &reg2hw.ecc_single_err_cnt[i].q ?
                                            reg2hw.ecc_single_err_cnt[i].q :
                                            reg2hw.ecc_single_err_cnt[i].q + 1'b1;

    assign hw2reg.ecc_single_err_addr[i].de = flash_phy_rsp.ecc_single_err[i];
    assign hw2reg.ecc_single_err_addr[i].d = {flash_phy_rsp.ecc_addr[i], {BusByteWidth{1'b0}}};
  end

  logic [LastIntrIdx-1:0] intr_event;
  // Status types
  assign intr_event[ProgEmpty] = !prog_fifo_rvalid;
  // Check whether this FIFO has been drained to a certain level.
  assign intr_event[ProgLvl]   = reg2hw.fifo_lvl.prog.q >= MaxFifoWidth'(prog_fifo_depth);
  assign intr_event[RdFull]    = sw_rfifo_full;
  // Check whether this FIFO has been filled to a certain level.
  assign intr_event[RdLvl]     = reg2hw.fifo_lvl.rd.q <= sw_rfifo_depth;
  // Event types
  assign intr_event[OpDone]    = sw_ctrl_done;
  assign intr_event[CorrErr]   = |flash_phy_rsp.ecc_single_err;

  prim_intr_hw #(
    .Width(1),
    .IntrT ("Status")
  ) u_intr_prog_empty (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_event[ProgEmpty]),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.prog_empty.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.prog_empty.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.prog_empty.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.prog_empty.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.prog_empty.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.prog_empty.d),
    .intr_o                 (intr_prog_empty_o)
  );

  prim_intr_hw #(
    .Width(1),
    .IntrT ("Status")
  ) u_intr_prog_lvl (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_event[ProgLvl]),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.prog_lvl.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.prog_lvl.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.prog_lvl.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.prog_lvl.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.prog_lvl.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.prog_lvl.d),
    .intr_o                 (intr_prog_lvl_o)
  );

  prim_intr_hw #(
    .Width(1),
    .IntrT ("Status")
  ) u_intr_rd_full (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_event[RdFull]),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rd_full.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rd_full.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rd_full.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rd_full.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rd_full.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rd_full.d),
    .intr_o                 (intr_rd_full_o)
  );

  prim_intr_hw #(
    .Width(1),
    .IntrT ("Status")
  ) u_intr_rd_lvl (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_event[RdLvl]),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rd_lvl.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rd_lvl.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rd_lvl.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rd_lvl.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rd_lvl.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rd_lvl.d),
    .intr_o                 (intr_rd_lvl_o)
  );

  prim_intr_hw #(
    .Width(1),
    .IntrT ("Event")
  ) u_intr_op_done (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_event[OpDone]),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.op_done.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.op_done.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.op_done.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.op_done.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.op_done.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.op_done.d),
    .intr_o                 (intr_op_done_o)
  );

  prim_intr_hw #(
    .Width(1),
    .IntrT ("Event")
  ) u_intr_corr_err (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_event[CorrErr]),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.corr_err.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.corr_err.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.corr_err.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.corr_err.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.corr_err.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.corr_err.d),
    .intr_o                 (intr_corr_err_o)
  );

  // Unused bits
  logic [BusByteWidth-1:0] unused_byte_sel;
  logic [top_pkg::TL_AW-1:0] unused_scratch;

  // Unused signals
  assign unused_byte_sel = muxed_addr[BusByteWidth-1:0];
  assign unused_scratch = reg2hw.scratch;


  //////////////////////////////////////
  // flash phy module
  //////////////////////////////////////
  logic flash_host_req;
  logic flash_host_req_rdy;
  logic flash_host_req_done;
  logic flash_host_rderr;
  logic [flash_ctrl_pkg::BusFullWidth-1:0] flash_host_rdata;
  logic [flash_ctrl_pkg::BusAddrW-1:0] flash_host_addr;

  lc_ctrl_pkg::lc_tx_t host_enable;

  // if flash disable is activated, error back from the adapter interface immediately
  assign host_enable = lc_ctrl_pkg::mubi4_to_lc_inv(flash_disable[HostDisableIdx]);

  tlul_pkg::tl_h2d_t gate_tl_h2d;
  tlul_pkg::tl_d2h_t gate_tl_d2h;

  // SEC_CM: MEM_TL_LC_GATE.FSM.SPARSE
  tlul_lc_gate u_tl_gate (
    .clk_i,
    .rst_ni,
    .tl_h2d_i(mem_tl_i),
    .tl_d2h_o(mem_tl_o),
    .tl_h2d_o(gate_tl_h2d),
    .tl_d2h_i(gate_tl_d2h),
    .flush_req_i('0),
    .flush_ack_o(),
    .resp_pending_o(),
    .lc_en_i(host_enable),
    .err_o(tl_gate_intg_err)
  );

  // SEC_CM: HOST.BUS.INTEGRITY
  tlul_adapter_sram #(
    .SramAw(BusAddrW),
    .SramDw(BusWidth),
    .SramBusBankAW(BusBankAddrW),
    .Outstanding(2),
    .ByteAccess(0),
    .ErrOnWrite(1),
    .CmdIntgCheck(1),
    .EnableRspIntgGen(1),
    .EnableDataIntgGen(0),
    .EnableDataIntgPt(1),
    .SecFifoPtr(1),
    .DataXorAddr(1)
  ) u_tl_adapter_eflash (
    .clk_i,
    .rst_ni,
    .tl_i                       (gate_tl_h2d),
    .tl_o                       (gate_tl_d2h),
    .en_ifetch_i                (flash_exec_en),
    .req_o                      (flash_host_req),
    .req_type_o                 (),
    .gnt_i                      (flash_host_req_rdy),
    .we_o                       (),
    .addr_o                     (flash_host_addr),
    .wdata_o                    (),
    .wmask_o                    (),
    .intg_error_o               (eflash_cmd_intg_err),
    .rdata_i                    (flash_host_rdata),
    .rvalid_i                   (flash_host_req_done),
    .rerror_i                   ({flash_host_rderr,1'b0}),
    .compound_txn_in_progress_o (),
    .readback_en_i              (prim_mubi_pkg::MuBi4False),
    .readback_error_o           (),
    .wr_collision_i             (1'b0),
    .write_pending_i            (1'b0)
  );

  flash_phy #(
    .SecScrambleEn(SecScrambleEn)
  ) u_eflash (
    .clk_i,
    .rst_ni,
    .host_req_i        (flash_host_req),
    .host_addr_i       (flash_host_addr),
    .host_req_rdy_o    (flash_host_req_rdy),
    .host_req_done_o   (flash_host_req_done),
    .host_rderr_o      (flash_host_rderr),
    .host_rdata_o      (flash_host_rdata),
    .flash_ctrl_i      (flash_phy_req),
    .flash_ctrl_o      (flash_phy_rsp),
    .tl_i              (prim_tl_i),
    .tl_o              (prim_tl_o),
    .obs_ctrl_i,
    .fla_obs_o,
    .lc_nvm_debug_en_i,
    .flash_bist_enable_i,
    .flash_power_down_h_i,
    .flash_power_ready_h_i,
    .flash_test_mode_a_io,
    .flash_test_voltage_h_io,
    .fatal_prim_flash_alert_o(fatal_prim_flash_alert),
    .recov_prim_flash_alert_o(recov_prim_flash_alert),
    .scanmode_i,
    .scan_en_i,
    .scan_rst_ni
  );

  /////////////////////////////////
  // Assertions
  /////////////////////////////////

  `ASSERT_KNOWN(TlDValidKnownO_A,       core_tl_o.d_valid )
  `ASSERT_KNOWN(TlAReadyKnownO_A,       core_tl_o.a_ready )
  `ASSERT_KNOWN_IF(RspPayLoad_A,        core_tl_o, core_tl_o.d_valid)
  `ASSERT_KNOWN(PrimTlDValidKnownO_A,   prim_tl_o.d_valid )
  `ASSERT_KNOWN(PrimTlAReadyKnownO_A,   prim_tl_o.a_ready )
  `ASSERT_KNOWN_IF(PrimRspPayLoad_A,    prim_tl_o, prim_tl_o.d_valid)
  `ASSERT_KNOWN(MemTlDValidKnownO_A,    mem_tl_o.d_valid )
  `ASSERT_KNOWN(MemTlAReadyKnownO_A,    mem_tl_o.a_ready )
  `ASSERT_KNOWN_IF(MemRspPayLoad_A,     mem_tl_o, mem_tl_o.d_valid)
  `ASSERT_KNOWN(FlashKnownO_A,          {flash_phy_req.req, flash_phy_req.rd,
                                         flash_phy_req.prog, flash_phy_req.pg_erase,
                                         flash_phy_req.bk_erase})
  `ASSERT_KNOWN_IF(FlashAddrKnown_A,    flash_phy_req.addr, flash_phy_req.req)
  `ASSERT_KNOWN_IF(FlashProgKnown_A,    flash_phy_req.prog_data,
    flash_phy_req.prog & flash_phy_req.req)
  `ASSERT_KNOWN(IntrProgEmptyKnownO_A,  intr_prog_empty_o)
  `ASSERT_KNOWN(IntrProgLvlKnownO_A,    intr_prog_lvl_o  )
  `ASSERT_KNOWN(IntrProgRdFullKnownO_A, intr_rd_full_o   )
  `ASSERT_KNOWN(IntrRdLvlKnownO_A,      intr_rd_lvl_o    )
  `ASSERT_KNOWN(IntrOpDoneKnownO_A,     intr_op_done_o   )
  `ASSERT_KNOWN(IntrErrO_A,             intr_corr_err_o  )
  `ASSERT_KNOWN(TdoKnown_A,             cio_tdo_o        )
  `ASSERT(TdoEnIsOne_A,                 cio_tdo_en_o === 1'b1)

  // combined indication that an operation has started
  // This is used only for assertions
  logic unused_op_valid;
  assign unused_op_valid = prog_op_valid | rd_op_valid | erase_op_valid;

  // add more assertions
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(SeedCntAlertCheck_A, u_flash_hw_if.u_seed_cnt,
                                         alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(AddrCntAlertCheck_A, u_flash_hw_if.u_addr_cnt,
                                         alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(PageCntAlertCheck_A, u_flash_hw_if.u_page_cnt,
                                         alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(WordCntAlertCheck_A, u_flash_hw_if.u_word_cnt,
                                         alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(WipeIdx_A, u_flash_hw_if.u_wipe_idx_cnt,
                                         alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(ProgCnt_A, u_flash_ctrl_prog.u_cnt,
                                         alert_tx_o[1])
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RdCnt_A, u_flash_ctrl_rd.u_cnt,
                                         alert_tx_o[1])

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(LcCtrlFsmCheck_A,
    u_flash_hw_if.u_state_regs, alert_tx_o[1])
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(LcCtrlRmaFsmCheck_A,
    u_flash_hw_if.u_rma_state_regs, alert_tx_o[1])
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(ArbFsmCheck_A,
    u_ctrl_arb.u_state_regs, alert_tx_o[1])
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(TlLcGateFsm_A,
    u_tl_gate.u_state_regs, alert_tx_o[1])
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(TlProgLcGateFsm_A,
    u_prog_tl_gate.u_state_regs, alert_tx_o[1])

   for (genvar i=0; i<NumBanks; i++) begin : gen_phy_assertions
     `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(PhyFsmCheck_A,
       u_eflash.gen_flash_cores[i].u_core.u_state_regs, alert_tx_o[1])

     `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(PhyProgFsmCheck_A,
       u_eflash.gen_flash_cores[i].u_core.gen_prog_data.u_prog.u_state_regs, alert_tx_o[1])
   end

   end

  // Alert assertions for redundant counters.
  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RdRspFifoWptrCheck_A,
      u_to_rd_fifo.u_rspfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RdRspFifoRptrCheck_A,
      u_to_rd_fifo.u_rspfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RdSramReqFifoWptrCheck_A,
      u_to_rd_fifo.u_sramreqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RdSramReqFifoRptrCheck_A,
      u_to_rd_fifo.u_sramreqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RdReqFifoWptrCheck_A,
      u_to_rd_fifo.u_reqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(RdReqFifoRptrCheck_A,
      u_to_rd_fifo.u_reqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EflashRspFifoWptrCheck_A,
      u_tl_adapter_eflash.u_rspfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EflashRspFifoRptrCheck_A,
      u_tl_adapter_eflash.u_rspfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EflashSramReqFifoWptrCheck_A,
      u_tl_adapter_eflash.u_sramreqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EflashSramReqFifoRptrCheck_A,
      u_tl_adapter_eflash.u_sramreqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EflashReqFifoWptrCheck_A,
      u_tl_adapter_eflash.u_reqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_wptr,
      alert_tx_o[1])

  `ASSERT_PRIM_COUNT_ERROR_TRIGGER_ALERT(EflashReqFifoRptrCheck_A,
      u_tl_adapter_eflash.u_reqfifo.gen_normal_fifo.u_fifo_cnt.gen_secure_ptrs.u_rptr,
      alert_tx_o[1])

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg_core, alert_tx_o[1])

  // Assertions for countermeasures inside prim_flash
  if ( prim_pkg::ImplGeneric == prim_pkg::ImplGeneric) begin : gen_reg_we_assert_generic
    `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(PrimRegWeOnehotCheck_A,
        u_eflash.u_flash.gen_generic.u_impl_generic.u_reg_top, alert_tx_o[3])
  end

endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Description: I2C top level wrapper file


module i2c
  import i2c_reg_pkg::*;
#(
      // Param list
  parameter int FifoDepth = 64,
  parameter int AcqFifoDepth = 268,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 7,
  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}},
  parameter int unsigned InputDelayCycles = 0
) (
  input                               clk_i,
  input                               rst_ni,
  input prim_ram_1p_pkg::ram_1p_cfg_t ram_cfg_i,

  // Bus Interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // Generic IO
  input                     cio_scl_i,
  output logic              cio_scl_o,
  output logic              cio_scl_en_o,
  input                     cio_sda_i,
  output logic              cio_sda_o,
  output logic              cio_sda_en_o,

  // Interrupts
  output logic              intr_fmt_threshold_o,
  output logic              intr_rx_threshold_o,
  output logic              intr_acq_threshold_o,
  output logic              intr_rx_overflow_o,
  output logic              intr_controller_halt_o,
  output logic              intr_scl_interference_o,
  output logic              intr_sda_interference_o,
  output logic              intr_stretch_timeout_o,
  output logic              intr_sda_unstable_o,
  output logic              intr_cmd_complete_o,
  output logic              intr_tx_stretch_o,
  output logic              intr_tx_threshold_o,
  output logic              intr_acq_stretch_o,
  output logic              intr_unexp_stop_o,
  output logic              intr_host_timeout_o
);

  i2c_reg2hw_t reg2hw;
  i2c_hw2reg_t hw2reg;

  logic [NumAlerts-1:0] alert_test, alerts;

  i2c_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    // SEC_CM: BUS.INTEGRITY
    .intg_err_o(alerts[0])
  );

  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  logic scl_int;
  logic sda_int;

  i2c_core #(
    .InputDelayCycles(InputDelayCycles)
  ) i2c_core (
    .clk_i,
    .rst_ni,
    .ram_cfg_i,

    .reg2hw,
    .hw2reg,

    .scl_i(cio_scl_i),
    .scl_o(scl_int),
    .sda_i(cio_sda_i),
    .sda_o(sda_int),

    .intr_fmt_threshold_o,
    .intr_rx_threshold_o,
    .intr_acq_threshold_o,
    .intr_rx_overflow_o,
    .intr_controller_halt_o,
    .intr_scl_interference_o,
    .intr_sda_interference_o,
    .intr_stretch_timeout_o,
    .intr_sda_unstable_o,
    .intr_cmd_complete_o,
    .intr_tx_stretch_o,
    .intr_tx_threshold_o,
    .intr_acq_stretch_o,
    .intr_unexp_stop_o,
    .intr_host_timeout_o
  );

  // For I2C, in standard, fast and fast-plus modes, outputs simulated as open-drain outputs.
  // Asserting scl or sda high should be equivalent to a tri-state output.
  // The output, when enabled, should only assert low.

  assign cio_scl_o = 1'b0;
  assign cio_sda_o = 1'b0;

  assign cio_scl_en_o = ~scl_int;
  assign cio_sda_en_o = ~sda_int;

  `ASSERT_KNOWN(TlDValidKnownO_A, tl_o.d_valid)
  `ASSERT_KNOWN(TlAReadyKnownO_A, tl_o.a_ready)
  `ASSERT_KNOWN(AlertKnownO_A, alert_tx_o)
  `ASSERT_KNOWN(CioSclKnownO_A, cio_scl_o)
  `ASSERT_KNOWN(CioSclEnKnownO_A, cio_scl_en_o)
  `ASSERT_KNOWN(CioSdaKnownO_A, cio_sda_o)
  `ASSERT_KNOWN(CioSdaEnKnownO_A, cio_sda_en_o)
  `ASSERT_KNOWN(IntrFmtWtmkKnownO_A, intr_fmt_threshold_o)
  `ASSERT_KNOWN(IntrRxWtmkKnownO_A, intr_rx_threshold_o)
  `ASSERT_KNOWN(IntrAcqWtmkKnownO_A, intr_acq_threshold_o)
  `ASSERT_KNOWN(IntrRxOflwKnownO_A, intr_rx_overflow_o)
  `ASSERT_KNOWN(IntrControllerHaltKnownO_A, intr_controller_halt_o)
  `ASSERT_KNOWN(IntrSclInterfKnownO_A, intr_scl_interference_o)
  `ASSERT_KNOWN(IntrSdaInterfKnownO_A, intr_sda_interference_o)
  `ASSERT_KNOWN(IntrStretchTimeoutKnownO_A, intr_stretch_timeout_o)
  `ASSERT_KNOWN(IntrSdaUnstableKnownO_A, intr_sda_unstable_o)
  `ASSERT_KNOWN(IntrCommandCompleteKnownO_A, intr_cmd_complete_o)
  `ASSERT_KNOWN(IntrTxStretchKnownO_A, intr_tx_stretch_o)
  `ASSERT_KNOWN(IntrTxWtmkKnownO_A, intr_tx_threshold_o)
  `ASSERT_KNOWN(IntrAcqStretchKnownO_A, intr_acq_stretch_o)
  `ASSERT_KNOWN(IntrUnexpStopKnownO_A, intr_unexp_stop_o)
  `ASSERT_KNOWN(IntrHostTimeoutKnownO_A, intr_host_timeout_o)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0


module pwm
  import pwm_reg_pkg::*;
#(
  
  // Param list
  parameter int NOutputs = 6,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 7,

  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}},
  parameter int PhaseCntDw = 16,
  parameter int BeatCntDw = 27
) (
  input                       clk_i,
  input                       rst_ni,

  input                       clk_core_i,
  input                       rst_core_ni,

  input                       tlul_pkg::tl_h2d_t tl_i,
  output                      tlul_pkg::tl_d2h_t tl_o,

  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  output logic [NOutputs-1:0] cio_pwm_o,
  output logic [NOutputs-1:0] cio_pwm_en_o
);

  pwm_reg_pkg::pwm_reg2hw_t reg2hw;
  logic [NumAlerts-1:0] alert_test, alerts;

  pwm_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .clk_core_i,
    .rst_core_ni,
    .tl_i       (tl_i),
    .tl_o       (tl_o),
    .reg2hw     (reg2hw),
    // SEC_CM: BUS.INTEGRITY
    .intg_err_o (alerts[0])
  );

  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  assign cio_pwm_en_o = {NOutputs{1'b1}};

  pwm_core #(
    .NOutputs(NOutputs),
    .PhaseCntDw(PhaseCntDw),
    .BeatCntDw(BeatCntDw)
  ) u_pwm_core (
    .clk_core_i,
    .rst_core_ni,
    .reg2hw,
    .pwm_o       (cio_pwm_o)
  );

  `ASSERT_KNOWN(TlDValidKnownO_A, tl_o.d_valid)
  `ASSERT_KNOWN(TlAReadyKnownO_A, tl_o.a_ready)

  `ASSERT_KNOWN(AlertKnownO_A, alert_tx_o)

  `ASSERT_KNOWN(CioPWMKnownO_A, cio_pwm_o)
  `ASSERT(CioPWMEnIsOneO_A, (&cio_pwm_en_o) === 1'b1)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule : pwm

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Power Manager
//


module pwrmgr
  import pwrmgr_pkg::*;
  import pwrmgr_reg_pkg::*;
#(
  // Param list
  parameter int NumWkups = 6,
  parameter int SYSRST_CTRL_AON_WKUP_REQ_IDX = 0,
  parameter int ADC_CTRL_AON_WKUP_REQ_IDX = 1,
  parameter int PINMUX_AON_PIN_WKUP_REQ_IDX = 2,
  parameter int PINMUX_AON_USB_WKUP_REQ_IDX = 3,
  parameter int AON_TIMER_AON_WKUP_REQ_IDX = 4,
  parameter int SENSOR_CTRL_AON_WKUP_REQ_IDX = 5,
  parameter int NumRstReqs = 2,
  parameter int NumIntRstReqs = 2,
  parameter int NumDebugRstReqs = 1,
  parameter int ResetMainPwrIdx = 2,
  parameter int ResetEscIdx = 3,
  parameter int ResetNdmIdx = 4,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 7,

  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}}
) (
  // Clocks and resets
  input clk_slow_i,
  input clk_i,
  input rst_slow_ni,
  input rst_ni,
  input rst_main_ni,
  input clk_lc_i,
  input rst_lc_ni,
  input clk_esc_i,
  input rst_esc_ni,

  // Bus Interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // AST interface
  input  pwr_ast_rsp_t pwr_ast_i,
  output pwr_ast_req_t pwr_ast_o,

  // rstmgr interface
  input  pwr_rst_rsp_t pwr_rst_i,
  output pwr_rst_req_t pwr_rst_o,

  // clkmgr interface
  output pwr_clk_req_t pwr_clk_o,
  input  pwr_clk_rsp_t pwr_clk_i,

  // otp interface
  input  pwr_otp_rsp_t pwr_otp_i,
  output pwr_otp_req_t pwr_otp_o,

  // life cycle interface
  input  pwr_lc_rsp_t pwr_lc_i,
  output pwr_lc_req_t pwr_lc_o,

  // flash interface
  input  pwr_flash_t pwr_flash_i,

  // processor interface
  input  pwr_cpu_t pwr_cpu_i,
  // SEC_CM: LC_CTRL.INTERSIG.MUBI
  output lc_ctrl_pkg::lc_tx_t fetch_en_o,
  input lc_ctrl_pkg::lc_tx_t lc_hw_debug_en_i,
  input lc_ctrl_pkg::lc_tx_t lc_dft_en_i,

  // peripherals wakeup and reset requests
  input  [NumWkups-1:0] wakeups_i,
  input  [NumRstReqs-1:0] rstreqs_i,

  // cpu related inputs
  input  ndmreset_req_i,

  // pinmux and other peripherals
  output logic strap_o,
  output logic low_power_o,

  // rom_ctrl interface
  // SEC_CM: ROM_CTRL.INTERSIG.MUBI
  input rom_ctrl_pkg::pwrmgr_data_t rom_ctrl_i,

  // software issued reset request
  // SEC_CM: RSTMGR.INTERSIG.MUBI
  input prim_mubi_pkg::mubi4_t sw_rst_req_i,

  // escalation interface
  input prim_esc_pkg::esc_tx_t esc_rst_tx_i,
  output prim_esc_pkg::esc_rx_t esc_rst_rx_o,

  output intr_wakeup_o

);
  ////////////////////////////////////////////////////
  // Input handling                                 //
  ////////////////////////////////////////////////////

  logic ndmreset_req_q;
  logic ndm_req_valid;

  prim_flop_2sync #(
    .Width(1),
    .ResetValue('0)
  ) u_ndm_sync (
    .clk_i,
    .rst_ni,
    .d_i(ndmreset_req_i),
    .q_o(ndmreset_req_q)
  );

  assign ndm_req_valid = ndmreset_req_q;

  ////////////////////////////
  ///  escalation detections
  ////////////////////////////

  logic clk_lc;
  logic rst_lc_n;
  assign clk_lc = clk_lc_i;
  assign rst_lc_n = rst_lc_ni;

  logic clk_esc;
  logic rst_esc_n;
  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_esc_clk_buf (
    .clk_i(clk_esc_i),
    .clk_o(clk_esc)
  );

  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_esc_rst_buf (
    .clk_i(rst_esc_ni),
    .clk_o(rst_esc_n)
  );

  logic esc_rst_req_d, esc_rst_req_q;
  prim_esc_receiver #(
    .N_ESC_SEV   (alert_handler_reg_pkg::N_ESC_SEV),
    .PING_CNT_DW (alert_handler_reg_pkg::PING_CNT_DW)
  ) u_esc_rx (
    .clk_i(clk_esc),
    .rst_ni(rst_esc_n),
    .esc_req_o(esc_rst_req_d),
    .esc_rx_o(esc_rst_rx_o),
    .esc_tx_i(esc_rst_tx_i)
  );

  // These assertions use formal or simulation to prove that once esc_rst_req is latched, we expect
  // to see the lc reset requests in pwr_rst_o. The one exception is when escalation requests are
  // cancelled while the CPU fetch is disabled, meaning the fast fsm is inactive.
  `ASSERT(PwrmgrSecCmEscToSlowResetReq_A,
          esc_rst_req_d |-> ##[2:3] (
              (!esc_rst_req_d && lc_ctrl_pkg::lc_tx_test_false_loose(fetch_en_o)) ||
              slow_peri_reqs_masked.rstreqs[ResetEscIdx]
          ), clk_slow_i, !rst_slow_ni)
  `ASSERT(PwrmgrSlowResetReqToFsmResetReq_A,
          slow_peri_reqs_masked.rstreqs[ResetEscIdx] |-> ##1 u_fsm.reset_reqs_i[ResetEscIdx],
          clk_i, !rst_ni)

  `ASSERT(PwrmgrSecCmEscToLCReset_A, u_fsm.reset_reqs_i[ResetEscIdx] &&
          u_fsm.state_q == FastPwrStateActive |-> ##4 pwr_rst_o.rst_lc_req == 2'b11,
          clk_i, !rst_ni)

  always_ff @(posedge clk_lc or negedge rst_lc_n) begin
    if (!rst_lc_n) begin
      esc_rst_req_q <= '0;
    end else if (esc_rst_req_d) begin
      // once latched, do not clear until reset
      esc_rst_req_q <= 1'b1;
    end
  end

  localparam int EscTimeOutCnt = 128;
  logic esc_timeout, esc_timeout_lc_d, esc_timeout_lc_q;
  // SEC_CM: ESC_RX.CLK.BKGN_CHK, ESC_RX.CLK.LOCAL_ESC
  prim_clock_timeout #(
    .TimeOutCnt(EscTimeOutCnt)
  ) u_esc_timeout (
    .clk_chk_i(clk_esc),
    .rst_chk_ni(rst_esc_n),
    .clk_i,
    .rst_ni,
    // if any ip clock enable is turned on, then the escalation
    // clocks are also enabled.
    .en_i(|pwr_clk_o),
    .timeout_o(esc_timeout)
  );

  prim_flop_2sync #(
    .Width(1),
    .ResetValue('0)
  ) u_esc_timeout_sync (
    .clk_i(clk_lc),
    .rst_ni(rst_lc_n),
    .d_i(esc_timeout),
    .q_o(esc_timeout_lc_d)
  );

  always_ff @(posedge clk_lc or negedge rst_lc_n) begin
    if (!rst_lc_n) begin
      esc_timeout_lc_q <= '0;
    end else if (esc_timeout_lc_d) begin
      // once latched, do not clear until reset
      esc_timeout_lc_q <= 1'b1;
    end
  end


  ////////////////////////////
  ///  async declarations
  ////////////////////////////
  pwr_peri_t peri_reqs_raw;
  logic slow_rst_req;

  assign peri_reqs_raw.wakeups = wakeups_i;
  assign peri_reqs_raw.rstreqs[NumRstReqs-1:0] = rstreqs_i;
  assign peri_reqs_raw.rstreqs[ResetMainPwrIdx] = slow_rst_req;
  // SEC_CM: ESC_RX.CLK.LOCAL_ESC, CTRL_FLOW.GLOBAL_ESC
  assign peri_reqs_raw.rstreqs[ResetEscIdx] = esc_rst_req_q | esc_timeout_lc_q;
  assign peri_reqs_raw.rstreqs[ResetNdmIdx] = ndm_req_valid;

  ////////////////////////////
  ///  Software reset request
  ////////////////////////////
  logic sw_rst_req;
  prim_buf #(
    .Width(1)
  ) u_sw_req_buf (
    .in_i(prim_mubi_pkg::mubi4_test_true_strict(sw_rst_req_i)),
    .out_o(sw_rst_req)
  );

  assign peri_reqs_raw.rstreqs[ResetSwReqIdx] = sw_rst_req;

  ////////////////////////////
  ///  clk_i domain declarations
  ////////////////////////////

  pwrmgr_reg2hw_t reg2hw;
  pwrmgr_hw2reg_t hw2reg;
  pwr_peri_t peri_reqs_masked;

  logic req_pwrup;
  logic ack_pwrup;
  logic req_pwrdn;
  logic ack_pwrdn;
  logic fsm_invalid;
  logic clr_slow_req;
  logic usb_ip_clk_en;
  logic usb_ip_clk_status;
  pwrup_cause_e pwrup_cause;

  logic low_power_fall_through;
  logic low_power_abort;

  pwr_flash_t flash_rsp;
  pwr_otp_rsp_t otp_rsp;

  prim_mubi_pkg::mubi4_t rom_ctrl_done;
  prim_mubi_pkg::mubi4_t rom_ctrl_good;

  logic core_sleeping;
  logic low_power_entry;

  ////////////////////////////
  ///  clk_slow_i domain declarations
  ////////////////////////////

  // Captured signals
  // These signals, though on clk_i domain, are safe for clk_slow_i to use
  logic [NumWkups-1:0] slow_wakeup_en;
  logic [NumRstReqs-1:0] slow_reset_en;

  pwr_ast_rsp_t slow_ast;
  pwr_peri_t slow_peri_reqs, slow_peri_reqs_masked;

  pwrup_cause_e slow_pwrup_cause;
  logic slow_pwrup_cause_toggle;
  logic slow_req_pwrup;
  logic slow_ack_pwrup;
  logic slow_req_pwrdn;
  logic slow_ack_pwrdn;
  logic slow_fsm_invalid;
  logic slow_main_pd_n;
  logic slow_io_clk_en;
  logic slow_core_clk_en;
  logic slow_usb_clk_en_lp;
  logic slow_usb_clk_en_active;
  logic slow_clr_req;
  logic slow_usb_ip_clk_en;
  logic slow_usb_ip_clk_status;



  ////////////////////////////
  ///  Register module
  ////////////////////////////
  logic [NumAlerts-1:0] alert_test, alerts;
  logic low_power_hint;
  logic lowpwr_cfg_wen;
  logic clr_hint;
  logic wkup;
  logic clr_cfg_lock;
  logic reg_intg_err;

  // SEC_CM: BUS.INTEGRITY
  // SEC_CM: CTRL.CONFIG.REGWEN, WAKEUP.CONFIG.REGWEN, RESET.CONFIG.REGWEN
  pwrmgr_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .clk_lc_i  (clk_lc  ),
    .rst_lc_ni (rst_lc_n),
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    .intg_err_o (reg_intg_err)
  );

  // whenever low power entry begins, wipe the hint
  assign hw2reg.control.low_power_hint.d = 1'b0;
  assign hw2reg.control.low_power_hint.de = clr_hint;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lowpwr_cfg_wen <= 1'b1;
    end else if (!lowpwr_cfg_wen && (clr_cfg_lock || wkup)) begin
      lowpwr_cfg_wen <= 1'b1;
    end else if (low_power_entry) begin
      lowpwr_cfg_wen <= 1'b0;
    end
  end

  assign hw2reg.ctrl_cfg_regwen.d = lowpwr_cfg_wen;

  assign hw2reg.fault_status.reg_intg_err.de    = reg_intg_err;
  assign hw2reg.fault_status.reg_intg_err.d     = 1'b1;
  assign hw2reg.fault_status.esc_timeout.de     = esc_timeout_lc_q;
  assign hw2reg.fault_status.esc_timeout.d      = 1'b1;

  // The main power domain glitch automatically causes a reset, so regsitering
  // an alert is functionally pointless.  However, if an attacker somehow manages/
  // to silence the reset, this gives us one potential back-up path through alert_handler.
  // Allow capture of main_pd fault status whenever the system is live.
  assign hw2reg.fault_status.main_pd_glitch.de  = pwr_clk_o.main_ip_clk_en;
  assign hw2reg.fault_status.main_pd_glitch.d   = peri_reqs_masked.rstreqs[ResetMainPwrIdx] |
                                                  reg2hw.fault_status.main_pd_glitch.q;

  `ASSERT(GlitchStatusPersist_A, $rose(reg2hw.fault_status.main_pd_glitch.q) |->
          reg2hw.fault_status.main_pd_glitch.q until !rst_lc_ni)

  ////////////////////////////
  ///  alerts
  ////////////////////////////

  // the logic below assumes there is only one alert, so make an
  // explicit assertion check for it.
  `ASSERT_INIT(AlertNumCheck_A, NumAlerts == 1)

  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  assign alerts[0] = reg2hw.fault_status.reg_intg_err.q |
                     reg2hw.fault_status.esc_timeout.q |
                     reg2hw.fault_status.main_pd_glitch.q;

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i         ( clk_lc        ),
      .rst_ni        ( rst_lc_n      ),
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[i]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  ////////////////////////////
  ///  cdc handling
  ////////////////////////////

  pwrmgr_cdc u_cdc (
    .clk_i,
    .rst_ni,
    .clk_slow_i,
    .rst_slow_ni,

    // slow domain signals
    .slow_req_pwrup_i(slow_req_pwrup),
    .slow_ack_pwrdn_i(slow_ack_pwrdn),
    .slow_fsm_invalid_i(slow_fsm_invalid),
    .slow_pwrup_cause_toggle_i(slow_pwrup_cause_toggle),
    .slow_pwrup_cause_i(slow_pwrup_cause),
    .slow_wakeup_en_o(slow_wakeup_en),
    .slow_reset_en_o(slow_reset_en),
    .slow_main_pd_no(slow_main_pd_n),
    .slow_io_clk_en_o(slow_io_clk_en),
    .slow_core_clk_en_o(slow_core_clk_en),
    .slow_usb_clk_en_lp_o(slow_usb_clk_en_lp),
    .slow_usb_clk_en_active_o(slow_usb_clk_en_active),
    .slow_req_pwrdn_o(slow_req_pwrdn),
    .slow_ack_pwrup_o(slow_ack_pwrup),
    .slow_ast_o(slow_ast),
    .slow_peri_reqs_o(slow_peri_reqs),
    .slow_peri_reqs_masked_i(slow_peri_reqs_masked),
    .slow_clr_req_o(slow_clr_req),
    .slow_usb_ip_clk_en_i(slow_usb_ip_clk_en),
    .slow_usb_ip_clk_status_o(slow_usb_ip_clk_status),

    // fast domain signals
    .req_pwrdn_i(req_pwrdn),
    .ack_pwrup_i(ack_pwrup),
    .cfg_cdc_sync_i(reg2hw.cfg_cdc_sync.qe & reg2hw.cfg_cdc_sync.q),
    .cdc_sync_done_o(hw2reg.cfg_cdc_sync.de),
    .wakeup_en_i(reg2hw.wakeup_en),
    .reset_en_i(reg2hw.reset_en),
    .main_pd_ni(reg2hw.control.main_pd_n.q),
    .io_clk_en_i(reg2hw.control.io_clk_en.q),
    .core_clk_en_i(reg2hw.control.core_clk_en.q),
    .usb_clk_en_lp_i(reg2hw.control.usb_clk_en_lp.q),
    .usb_clk_en_active_i(reg2hw.control.usb_clk_en_active.q),
    .ack_pwrdn_o(ack_pwrdn),
    .fsm_invalid_o(fsm_invalid),
    .req_pwrup_o(req_pwrup),
    .pwrup_cause_o(pwrup_cause),
    .peri_reqs_o(peri_reqs_masked),
    .clr_slow_req_i(clr_slow_req),
    .usb_ip_clk_en_o(usb_ip_clk_en),
    .usb_ip_clk_status_i(usb_ip_clk_status),

    // AST signals
    .ast_i(pwr_ast_i),

    // peripheral signals
    .peri_i(peri_reqs_raw),

    // flash handshake
    .flash_i(pwr_flash_i),
    .flash_o(flash_rsp),

    // OTP signals
    .otp_i(pwr_otp_i),
    .otp_o(otp_rsp),

    // rom_ctrl signals
    .rom_ctrl_done_i(rom_ctrl_i.done),
    .rom_ctrl_done_o(rom_ctrl_done),

    // core sleeping
    .core_sleeping_i(pwr_cpu_i.core_sleeping),
    .core_sleeping_o(core_sleeping)

  );
  // rom_ctrl_i.good is not synchronized as it acts as a "payload" signal
  // to "done". Good is only observed if "done" is high.
  assign rom_ctrl_good = rom_ctrl_i.good;
  assign hw2reg.cfg_cdc_sync.d = 1'b0;

  ////////////////////////////
  ///  Wakup and reset capture
  ////////////////////////////

  // reset and wakeup requests are captured into the slow clock domain and then
  // fanned out to other domains as necessary.  This ensures there is not a huge
  // time gap between when the slow clk domain sees the signal vs when the fast
  // clock domains see it.  This creates redundant syncing but keeps the time
  // scale approximately the same across all domains.
  //
  // This also implies that these signals must be at least 1 clk_slow pulse long
  //
  // Since resets are not latched inside pwrmgr, there exists a corner case where
  // non-always-on reset requests may get wiped out by a graceful low power entry
  // It's not clear if this is really an issue at the moment, but something to keep
  // in mind if future changes are needed.
  //
  // Latching the reset requests is not difficult, but the bigger question is who
  // should clear it and when that should happen. If the clearing does not work
  // correctly, it is possible for the device to end up in a permanent reset loop,
  // and that would be very undesirable.

  assign slow_peri_reqs_masked.wakeups = slow_peri_reqs.wakeups & slow_wakeup_en;
  // msb is software request
  // the internal requests include escalation and internal requests
  // the lsbs are the software enabled peripheral requests.
  assign slow_peri_reqs_masked.rstreqs = slow_peri_reqs.rstreqs &
                                         {{NumSwRstReq{1'b1}},
                                          {NumDebugRstReqs{1'b1}},
                                          {NumIntRstReqs{1'b1}},
                                          slow_reset_en};

  for (genvar i = 0; i < NumWkups; i++) begin : gen_wakeup_status
    assign hw2reg.wake_status[i].de = 1'b1;
    assign hw2reg.wake_status[i].d  = peri_reqs_masked.wakeups[i];
  end

  for (genvar i = 0; i < NumRstReqs; i++) begin : gen_reset_status
    assign hw2reg.reset_status[i].de = 1'b1;
    assign hw2reg.reset_status[i].d  = peri_reqs_masked.rstreqs[i];
  end

  assign hw2reg.escalate_reset_status.de = 1'b1;
  assign hw2reg.escalate_reset_status.d = peri_reqs_masked.rstreqs[NumRstReqs];


  ////////////////////////////
  ///  clk_slow FSM
  ////////////////////////////

  pwrmgr_slow_fsm u_slow_fsm (
    .clk_i                (clk_slow_i),
    .rst_ni               (rst_slow_ni),
    .rst_main_ni          (rst_main_ni),
    .wakeup_i             (|slow_peri_reqs_masked.wakeups),
    .reset_req_i          (|slow_peri_reqs_masked.rstreqs),
    .ast_i                (slow_ast),
    .req_pwrup_o          (slow_req_pwrup),
    .pwrup_cause_o        (slow_pwrup_cause),
    .pwrup_cause_toggle_o (slow_pwrup_cause_toggle),
    .ack_pwrup_i          (slow_ack_pwrup),
    .req_pwrdn_i          (slow_req_pwrdn),
    .ack_pwrdn_o          (slow_ack_pwrdn),
    .rst_req_o            (slow_rst_req),
    .fsm_invalid_o        (slow_fsm_invalid),
    .clr_req_i            (slow_clr_req),
    .usb_ip_clk_en_o      (slow_usb_ip_clk_en),
    .usb_ip_clk_status_i  (slow_usb_ip_clk_status),

    .main_pd_ni           (slow_main_pd_n),
    .io_clk_en_i          (slow_io_clk_en),
    .core_clk_en_i        (slow_core_clk_en),
    .usb_clk_en_lp_i      (slow_usb_clk_en_lp),
    .usb_clk_en_active_i  (slow_usb_clk_en_active),

    // outputs to AST - These are on the slow clock domain
    // TBD - need to check this with partners
    .ast_o                (pwr_ast_o)
  );

  lc_ctrl_pkg::lc_tx_t lc_dft_en;
  prim_lc_sync u_prim_lc_sync_dft_en (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_dft_en_i),
    .lc_en_o({lc_dft_en})
  );

  lc_ctrl_pkg::lc_tx_t lc_hw_debug_en;
  prim_lc_sync u_prim_lc_sync_hw_debug_en (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_hw_debug_en_i),
    .lc_en_o({lc_hw_debug_en})
  );

  ////////////////////////////
  ///  clk FSM
  ////////////////////////////

  assign low_power_hint = reg2hw.control.low_power_hint.q == LowPower;
  assign low_power_entry = core_sleeping & low_power_hint;

  pwrmgr_fsm u_fsm (
    .clk_i,
    .rst_ni,
    .clk_slow_i,
    .rst_slow_ni,

    // interface with slow_fsm
    .req_pwrup_i         (req_pwrup),
    .pwrup_cause_i       (pwrup_cause), // por, wake or reset request
    .ack_pwrup_o         (ack_pwrup),
    .req_pwrdn_o         (req_pwrdn),
    .ack_pwrdn_i         (ack_pwrdn),
    .low_power_entry_i   (low_power_entry),
    .reset_reqs_i        (peri_reqs_masked.rstreqs),
    .fsm_invalid_i       (fsm_invalid),
    .clr_slow_req_o      (clr_slow_req),
    .usb_ip_clk_en_i     (usb_ip_clk_en),
    .usb_ip_clk_status_o (usb_ip_clk_status),

    // cfg
    .main_pd_ni        (reg2hw.control.main_pd_n.q),

    // consumed in pwrmgr
    .wkup_o            (wkup),
    .clr_cfg_lock_o    (clr_cfg_lock),
    .fall_through_o    (low_power_fall_through),
    .abort_o           (low_power_abort),
    .clr_hint_o        (clr_hint),

    // rstmgr
    .pwr_rst_o         (pwr_rst_o),
    .pwr_rst_i         (pwr_rst_i),

    // clkmgr
    .ips_clk_en_o      (pwr_clk_o),
    .clk_en_status_i   (pwr_clk_i),

    // otp
    .otp_init_o        (pwr_otp_o.otp_init),
    .otp_done_i        (otp_rsp.otp_done),
    .otp_idle_i        (otp_rsp.otp_idle),

    // lc
    .lc_init_o         (pwr_lc_o.lc_init),
    .lc_done_i         (pwr_lc_i.lc_done),
    .lc_idle_i         (pwr_lc_i.lc_idle),
    .lc_dft_en_i       (lc_dft_en),
    .lc_hw_debug_en_i  (lc_hw_debug_en),

    // flash
    .flash_idle_i      (flash_rsp.flash_idle),

    // rom_ctrl
    .rom_ctrl_done_i   (rom_ctrl_done),
    .rom_ctrl_good_i   (rom_ctrl_good),

    // processing element
    .fetch_en_o,

    // pinmux and other peripherals
    .strap_o,
    .low_power_o
  );

  ////////////////////////////
  ///  Wakeup Info Capture
  ////////////////////////////

  logic wake_info_wen;
  logic [TotalWakeWidth-1:0] wake_info_data;

  assign wake_info_wen = reg2hw.wake_info.abort.qe |
                         reg2hw.wake_info.fall_through.qe |
                         reg2hw.wake_info.reasons.qe;

  assign wake_info_data = {reg2hw.wake_info.abort.q,
                           reg2hw.wake_info.fall_through.q,
                           reg2hw.wake_info.reasons.q};

  pwrmgr_wake_info i_wake_info (
    .clk_i,
    .rst_ni,
    .wr_i            (wake_info_wen),
    .data_i          (wake_info_data),
    .start_capture_i (low_power_o),
    .record_dis_i    (reg2hw.wake_info_capture_dis.q),
    .wakeups_i       (peri_reqs_masked.wakeups),
    .fall_through_i  (low_power_fall_through),
    .abort_i         (low_power_abort),
    .info_o          (hw2reg.wake_info)
  );

  ////////////////////////////
  ///  Interrupts
  ////////////////////////////

  // This interrupt is asserted whenever the fast FSM transitions
  // into active state.  However, it does not assert during POR
  prim_intr_hw #(.Width(1)) intr_wakeup (
    .clk_i,
    .rst_ni,
    .event_intr_i           (wkup),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.d),
    .intr_o                 (intr_wakeup_o)
  );


  ////////////////////////////
  ///  Assertions
  ////////////////////////////

  `ASSERT_KNOWN(TlDValidKnownO_A,  tl_o.d_valid     )
  `ASSERT_KNOWN(TlAReadyKnownO_A,  tl_o.a_ready     )
  `ASSERT_KNOWN(AlertsKnownO_A,    alert_tx_o       )
  `ASSERT_KNOWN(AstKnownO_A,       pwr_ast_o        )
  `ASSERT_KNOWN(RstKnownO_A,       pwr_rst_o        )
  `ASSERT_KNOWN(ClkKnownO_A,       pwr_clk_o        )
  `ASSERT_KNOWN(OtpKnownO_A,       pwr_otp_o        )
  `ASSERT_KNOWN(LcKnownO_A,        pwr_lc_o         )
  `ASSERT_KNOWN(IntrKnownO_A,      intr_wakeup_o    )

  // EscTimeOutCnt also sets the required clock ratios between escalator and local clock
  // Ie, clk_lc cannot be so slow that the timeout count is reached


  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ERR(FsmCheck_A, u_fsm.u_state_regs,
      pwr_rst_o.rst_lc_req && pwr_rst_o.rst_sys_req)
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ERR(SlowFsmCheck_A, u_slow_fsm.u_state_regs,
      pwr_ast_o.pwr_clamp && !pwr_ast_o.main_pd_n, 0, 2,
      clk_slow_i, !rst_slow_ni)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule // pwrmgr

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// This module is the overall reset manager wrapper

// This top level controller is fairly hardcoded right now, but will be switched to a template
module rstmgr
  import rstmgr_pkg::*;
  import rstmgr_reg_pkg::*;
  import prim_mubi_pkg::mubi4_t;
#(
  // Power domain parameters
  parameter int PowerDomains = 2,
  parameter int DomainAonSel = 0,
  parameter int Domain0Sel = 1,

  // Number of non-always-on domains
  parameter int OffDomains = PowerDomains-1,
  // Param list
  parameter int RdWidth = 32,
  parameter int IdxWidth = 4,
  parameter int NumHwResets = 5,
  parameter int NumSwResets = 8,
  parameter int NumTotalResets = 8,
  parameter int NumAlerts = 2,

  // Address widths within the block
  parameter int BlockAw = 7,

  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}},
  parameter bit SecCheck = 1,
  parameter int SecMaxSyncDelay = 2,
  parameter RST_NUM = 1
) (
  // Primary module clocks
  input clk_i,
  input rst_ni,
  input clk_aon_i,
  input clk_io_div4_i,
  input clk_main_i,
  input clk_io_i,
  input clk_io_div2_i,
  input clk_usb_i,
  input clk_por_i,
  input rst_por_ni,

  // POR input
  input [PowerDomains-1:0] por_n_i,

  // Bus Interface
  input tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // pwrmgr interface
  input pwrmgr_pkg::pwr_rst_req_t pwr_i,
  output pwrmgr_pkg::pwr_rst_rsp_t pwr_o,

  // software initiated reset request
  output mubi4_t sw_rst_req_o,

  // Interface to alert handler
  input alert_pkg::alert_crashdump_t alert_dump_i,

  // Interface to cpu crash dump
  input rv_core_ibex_pkg::cpu_crash_dump_t cpu_dump_i,

  // dft bypass
  input scan_rst_ni,
  // SEC_CM: SCAN.INTERSIG.MUBI
  input prim_mubi_pkg::mubi4_t scanmode_i,

  // Reset asserted indications going to alert handler
  output rstmgr_rst_en_t rst_en_o,

  // reset outputs
  //output rstmgr_out_t resets_o
  output  resets_o [RST_NUM-1:0]

);

  import prim_mubi_pkg::MuBi4False;
  import prim_mubi_pkg::MuBi4True;

  // receive POR and stretch
  // The por is at first stretched and synced on clk_aon
  // The rst_ni and pok_i input will be changed once AST is integrated
  logic [PowerDomains-1:0] rst_por_aon_n;

  for (genvar i = 0; i < PowerDomains; i++) begin : gen_rst_por_aon

      // Declared as size 1 packed array to avoid FPV warning.
      prim_mubi_pkg::mubi4_t [0:0] por_scanmode;
      prim_mubi4_sync #(
        .NumCopies(1),
        .AsyncOn(0)
      ) u_por_scanmode_sync (
        .clk_i,
        .rst_ni,
        .mubi_i(scanmode_i),
        .mubi_o(por_scanmode)
      );

    if (i == DomainAonSel) begin : gen_rst_por_aon_normal
      rstmgr_por u_rst_por_aon (
        .clk_i(clk_aon_i),
        .rst_ni(por_n_i[i]),
        .scan_rst_ni,
        .scanmode_i(prim_mubi_pkg::mubi4_test_true_strict(por_scanmode[0])),
        .rst_no(rst_por_aon_n[i])
      );

      // reset asserted indication for alert handler
      prim_mubi4_sender #(
        .ResetValue(MuBi4True)
      ) u_prim_mubi4_sender (
        .clk_i(clk_aon_i),
        .rst_ni(rst_por_aon_n[i]),
        .mubi_i(MuBi4False),
        .mubi_o(rst_en_o.por_aon[i])
      );
    end else begin : gen_rst_por_domain
      logic rst_por_aon_premux;
      prim_flop_2sync #(
        .Width(1),
        .ResetValue('0)
      ) u_por_domain_sync (
        .clk_i(clk_aon_i),
        // do not release from reset if aon has not
        .rst_ni(rst_por_aon_n[DomainAonSel] & por_n_i[i]),
        .d_i(1'b1),
        .q_o(rst_por_aon_premux)
      );

      prim_clock_mux2 #(
        .NoFpgaBufG(1'b1)
      ) u_por_domain_mux (
        .clk0_i(rst_por_aon_premux),
        .clk1_i(scan_rst_ni),
        .sel_i(prim_mubi_pkg::mubi4_test_true_strict(por_scanmode[0])),
        .clk_o(rst_por_aon_n[i])
      );

      // reset asserted indication for alert handler
      prim_mubi4_sender #(
        .ResetValue(MuBi4True)
      ) u_prim_mubi4_sender (
        .clk_i(clk_aon_i),
        .rst_ni(rst_por_aon_n[i]),
        .mubi_i(MuBi4False),
        .mubi_o(rst_en_o.por_aon[i])
      );
    end
  end
  assign resets_o.rst_por_aon_n = rst_por_aon_n;

  logic clk_por;
  logic rst_por_n;
  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_por_clk_buf (
    .clk_i(clk_por_i),
    .clk_o(clk_por)
  );

  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_por_rst_buf (
    .clk_i(rst_por_ni),
    .clk_o(rst_por_n)
  );

  ////////////////////////////////////////////////////
  // Register Interface                             //
  ////////////////////////////////////////////////////

  rstmgr_reg_pkg::rstmgr_reg2hw_t reg2hw;
  rstmgr_reg_pkg::rstmgr_hw2reg_t hw2reg;

  logic reg_intg_err;
  // SEC_CM: BUS.INTEGRITY
  // SEC_CM: SW_RST.CONFIG.REGWEN, DUMP_CTRL.CONFIG.REGWEN
  rstmgr_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .clk_por_i  (clk_por),
    .rst_por_ni (rst_por_n),
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    .intg_err_o(reg_intg_err)
  );


  ////////////////////////////////////////////////////
  // Errors                                         //
  ////////////////////////////////////////////////////

  // consistency check errors
  logic [20:0][PowerDomains-1:0] cnsty_chk_errs;
  logic [20:0][PowerDomains-1:0] shadow_cnsty_chk_errs;

  // consistency sparse fsm errors
  logic [20:0][PowerDomains-1:0] fsm_errs;
  logic [20:0][PowerDomains-1:0] shadow_fsm_errs;

  assign hw2reg.err_code.reg_intg_err.d  = 1'b1;
  assign hw2reg.err_code.reg_intg_err.de = reg_intg_err;
  assign hw2reg.err_code.reset_consistency_err.d  = 1'b1;
  assign hw2reg.err_code.reset_consistency_err.de = |cnsty_chk_errs ||
                                                    |shadow_cnsty_chk_errs;
  assign hw2reg.err_code.fsm_err.d  = 1'b1;
  assign hw2reg.err_code.fsm_err.de = |fsm_errs || |shadow_fsm_errs;
  ////////////////////////////////////////////////////
  // Alerts                                         //
  ////////////////////////////////////////////////////
  logic [NumAlerts-1:0] alert_test, alerts;

  // All of these are fatal alerts
  assign alerts[0] = reg2hw.err_code.reg_intg_err.q |
                     (|reg2hw.err_code.fsm_err.q);

  assign alerts[1] = reg2hw.err_code.reset_consistency_err.q;

  assign alert_test = {
    reg2hw.alert_test.fatal_cnsty_fault.q & reg2hw.alert_test.fatal_cnsty_fault.qe,
    reg2hw.alert_test.fatal_fault.q & reg2hw.alert_test.fatal_fault.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[i]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  ////////////////////////////////////////////////////
  // Source resets in the system                    //
  // These are hardcoded and not directly used.     //
  // Instead they act as async reset roots.         //
  ////////////////////////////////////////////////////

  // The two source reset modules are chained together.  The output of one is fed into the
  // the second.  This ensures that if upstream resets for any reason, the associated downstream
  // reset will also reset.

  logic [PowerDomains-1:0] rst_lc_src_n;
  logic [PowerDomains-1:0] rst_sys_src_n;

  // Declared as size 1 packed array to avoid FPV warning.
  prim_mubi_pkg::mubi4_t [0:0] rst_ctrl_scanmode;
  prim_mubi4_sync #(
    .NumCopies(1),
    .AsyncOn(0)
  ) u_ctrl_scanmode_sync (
    .clk_i (clk_por),
    .rst_ni (rst_por_n),
    .mubi_i(scanmode_i),
    .mubi_o(rst_ctrl_scanmode)
  );

  // lc reset sources
  rstmgr_ctrl u_lc_src (
    .clk_i (clk_por),
    .scanmode_i(prim_mubi_pkg::mubi4_test_true_strict(rst_ctrl_scanmode[0])),
    .scan_rst_ni,
    .rst_req_i(pwr_i.rst_lc_req),
    .rst_parent_ni(rst_por_aon_n),
    .rst_no(rst_lc_src_n)
  );

  // sys reset sources
  rstmgr_ctrl u_sys_src (
    .clk_i (clk_por),
    .scanmode_i(prim_mubi_pkg::mubi4_test_true_strict(rst_ctrl_scanmode[0])),
    .scan_rst_ni,
    .rst_req_i(pwr_i.rst_sys_req),
    .rst_parent_ni(rst_por_aon_n),
    .rst_no(rst_sys_src_n)
  );

  assign pwr_o.rst_lc_src_n = rst_lc_src_n;
  assign pwr_o.rst_sys_src_n = rst_sys_src_n;


  ////////////////////////////////////////////////////
  // leaf reset in the system                       //
  // These should all be generated                  //
  ////////////////////////////////////////////////////
  // To simplify generation, each reset generates all associated power domain outputs.
  // If a reset does not support a particular power domain, that reset is always hard-wired to 0.

  // Generating resets for por
  // Power Domains: ['Aon']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_por (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_main_i),
    .parent_rst_ni(rst_por_aon_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.por[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_por_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[0][DomainAonSel]),
    .fsm_err_o(fsm_errs[0][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_por_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonPorFsmCheck_A,
    u_daon_por.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign resets_o.rst_por_n[Domain0Sel] = '0;
  assign cnsty_chk_errs[0][Domain0Sel] = '0;
  assign fsm_errs[0][Domain0Sel] = '0;
  assign rst_en_o.por[Domain0Sel] = MuBi4True;
  assign shadow_cnsty_chk_errs[0] = '0;
  assign shadow_fsm_errs[0] = '0;

  // Generating resets for por_io
  // Power Domains: ['Aon']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_por_io (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_i),
    .parent_rst_ni(rst_por_aon_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.por_io[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_por_io_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[1][DomainAonSel]),
    .fsm_err_o(fsm_errs[1][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_por_io_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonPorIoFsmCheck_A,
    u_daon_por_io.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign resets_o.rst_por_io_n[Domain0Sel] = '0;
  assign cnsty_chk_errs[1][Domain0Sel] = '0;
  assign fsm_errs[1][Domain0Sel] = '0;
  assign rst_en_o.por_io[Domain0Sel] = MuBi4True;
  assign shadow_cnsty_chk_errs[1] = '0;
  assign shadow_fsm_errs[1] = '0;

  // Generating resets for por_io_div2
  // Power Domains: ['Aon']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_por_io_div2 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div2_i),
    .parent_rst_ni(rst_por_aon_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.por_io_div2[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_por_io_div2_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[2][DomainAonSel]),
    .fsm_err_o(fsm_errs[2][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_por_io_div2_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonPorIoDiv2FsmCheck_A,
    u_daon_por_io_div2.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign resets_o.rst_por_io_div2_n[Domain0Sel] = '0;
  assign cnsty_chk_errs[2][Domain0Sel] = '0;
  assign fsm_errs[2][Domain0Sel] = '0;
  assign rst_en_o.por_io_div2[Domain0Sel] = MuBi4True;
  assign shadow_cnsty_chk_errs[2] = '0;
  assign shadow_fsm_errs[2] = '0;

  // Generating resets for por_io_div4
  // Power Domains: ['Aon']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_por_io_div4 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_por_aon_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.por_io_div4[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_por_io_div4_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[3][DomainAonSel]),
    .fsm_err_o(fsm_errs[3][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_por_io_div4_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonPorIoDiv4FsmCheck_A,
    u_daon_por_io_div4.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign resets_o.rst_por_io_div4_n[Domain0Sel] = '0;
  assign cnsty_chk_errs[3][Domain0Sel] = '0;
  assign fsm_errs[3][Domain0Sel] = '0;
  assign rst_en_o.por_io_div4[Domain0Sel] = MuBi4True;
  assign shadow_cnsty_chk_errs[3] = '0;
  assign shadow_fsm_errs[3] = '0;

  // Generating resets for por_usb
  // Power Domains: ['Aon']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_por_usb (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_usb_i),
    .parent_rst_ni(rst_por_aon_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.por_usb[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_por_usb_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[4][DomainAonSel]),
    .fsm_err_o(fsm_errs[4][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_por_usb_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonPorUsbFsmCheck_A,
    u_daon_por_usb.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign resets_o.rst_por_usb_n[Domain0Sel] = '0;
  assign cnsty_chk_errs[4][Domain0Sel] = '0;
  assign fsm_errs[4][Domain0Sel] = '0;
  assign rst_en_o.por_usb[Domain0Sel] = MuBi4True;
  assign shadow_cnsty_chk_errs[4] = '0;
  assign shadow_fsm_errs[4] = '0;

  // Generating resets for lc
  // Power Domains: ['0', 'Aon']
  // Shadowed: True
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_main_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[5][DomainAonSel]),
    .fsm_err_o(fsm_errs[5][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_lc_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonLcFsmCheck_A,
    u_daon_lc.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_lc (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_main_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_lc_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[5][Domain0Sel]),
    .fsm_err_o(fsm_errs[5][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_lc_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0LcFsmCheck_A,
    u_d0_lc.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc_shadowed (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_main_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_shadowed[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_shadowed_n[DomainAonSel]),
    .err_o(shadow_cnsty_chk_errs[5][DomainAonSel]),
    .fsm_err_o(shadow_fsm_errs[5][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_lc_shadowed_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonLcShadowedFsmCheck_A,
    u_daon_lc_shadowed.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_lc_shadowed (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_main_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_shadowed[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_lc_shadowed_n[Domain0Sel]),
    .err_o(shadow_cnsty_chk_errs[5][Domain0Sel]),
    .fsm_err_o(shadow_fsm_errs[5][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_lc_shadowed_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0LcShadowedFsmCheck_A,
    u_d0_lc_shadowed.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end

  // Generating resets for lc_aon
  // Power Domains: ['Aon']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc_aon (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_aon_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_aon[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_aon_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[6][DomainAonSel]),
    .fsm_err_o(fsm_errs[6][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_lc_aon_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonLcAonFsmCheck_A,
    u_daon_lc_aon.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign resets_o.rst_lc_aon_n[Domain0Sel] = '0;
  assign cnsty_chk_errs[6][Domain0Sel] = '0;
  assign fsm_errs[6][Domain0Sel] = '0;
  assign rst_en_o.lc_aon[Domain0Sel] = MuBi4True;
  assign shadow_cnsty_chk_errs[6] = '0;
  assign shadow_fsm_errs[6] = '0;

  // Generating resets for lc_io
  // Power Domains: ['Aon', '0']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc_io (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_io_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[7][DomainAonSel]),
    .fsm_err_o(fsm_errs[7][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_lc_io_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonLcIoFsmCheck_A,
    u_daon_lc_io.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_lc_io (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_lc_io_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[7][Domain0Sel]),
    .fsm_err_o(fsm_errs[7][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_lc_io_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0LcIoFsmCheck_A,
    u_d0_lc_io.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[7] = '0;
  assign shadow_fsm_errs[7] = '0;

  // Generating resets for lc_io_div2
  // Power Domains: ['Aon', '0']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc_io_div2 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div2_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io_div2[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_io_div2_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[8][DomainAonSel]),
    .fsm_err_o(fsm_errs[8][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_lc_io_div2_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonLcIoDiv2FsmCheck_A,
    u_daon_lc_io_div2.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_lc_io_div2 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div2_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io_div2[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_lc_io_div2_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[8][Domain0Sel]),
    .fsm_err_o(fsm_errs[8][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_lc_io_div2_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0LcIoDiv2FsmCheck_A,
    u_d0_lc_io_div2.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[8] = '0;
  assign shadow_fsm_errs[8] = '0;

  // Generating resets for lc_io_div4
  // Power Domains: ['0', 'Aon']
  // Shadowed: True
  rstmgr_leaf_rst #(
    .SecCheck(0),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc_io_div4 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io_div4[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_io_div4_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[9][DomainAonSel]),
    .fsm_err_o(fsm_errs[9][DomainAonSel])
  );

  rstmgr_leaf_rst #(
    .SecCheck(0),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_lc_io_div4 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io_div4[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_lc_io_div4_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[9][Domain0Sel]),
    .fsm_err_o(fsm_errs[9][Domain0Sel])
  );

  rstmgr_leaf_rst #(
    .SecCheck(0),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc_io_div4_shadowed (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io_div4_shadowed[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_io_div4_shadowed_n[DomainAonSel]),
    .err_o(shadow_cnsty_chk_errs[9][DomainAonSel]),
    .fsm_err_o(shadow_fsm_errs[9][DomainAonSel])
  );

  rstmgr_leaf_rst #(
    .SecCheck(0),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_lc_io_div4_shadowed (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_io_div4_shadowed[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_lc_io_div4_shadowed_n[Domain0Sel]),
    .err_o(shadow_cnsty_chk_errs[9][Domain0Sel]),
    .fsm_err_o(shadow_fsm_errs[9][Domain0Sel])
  );


  // Generating resets for lc_usb
  // Power Domains: ['Aon', '0']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_lc_usb (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_usb_i),
    .parent_rst_ni(rst_lc_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_usb[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_lc_usb_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[10][DomainAonSel]),
    .fsm_err_o(fsm_errs[10][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_lc_usb_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonLcUsbFsmCheck_A,
    u_daon_lc_usb.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_lc_usb (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_usb_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.lc_usb[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_lc_usb_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[10][Domain0Sel]),
    .fsm_err_o(fsm_errs[10][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_lc_usb_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0LcUsbFsmCheck_A,
    u_d0_lc_usb.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[10] = '0;
  assign shadow_fsm_errs[10] = '0;

  // Generating resets for sys
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_sys_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[11][DomainAonSel] = '0;
  assign fsm_errs[11][DomainAonSel] = '0;
  assign rst_en_o.sys[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_d0_sys (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_main_i),
    .parent_rst_ni(rst_sys_src_n[Domain0Sel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.sys[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_sys_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[11][Domain0Sel]),
    .fsm_err_o(fsm_errs[11][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_sys_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0SysFsmCheck_A,
    u_d0_sys.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[11] = '0;
  assign shadow_fsm_errs[11] = '0;

  // Generating resets for sys_io_div4
  // Power Domains: ['Aon']
  // Shadowed: False
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b0)
  ) u_daon_sys_io_div4 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_sys_src_n[DomainAonSel]),
    .sw_rst_req_ni(1'b1),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.sys_io_div4[DomainAonSel]),
    .leaf_rst_o(resets_o.rst_sys_io_div4_n[DomainAonSel]),
    .err_o(cnsty_chk_errs[12][DomainAonSel]),
    .fsm_err_o(fsm_errs[12][DomainAonSel])
  );

  if (SecCheck) begin : gen_daon_sys_io_div4_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    DAonSysIoDiv4FsmCheck_A,
    u_daon_sys_io_div4.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign resets_o.rst_sys_io_div4_n[Domain0Sel] = '0;
  assign cnsty_chk_errs[12][Domain0Sel] = '0;
  assign fsm_errs[12][Domain0Sel] = '0;
  assign rst_en_o.sys_io_div4[Domain0Sel] = MuBi4True;
  assign shadow_cnsty_chk_errs[12] = '0;
  assign shadow_fsm_errs[12] = '0;

  // Generating resets for spi_device
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_spi_device_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[13][DomainAonSel] = '0;
  assign fsm_errs[13][DomainAonSel] = '0;
  assign rst_en_o.spi_device[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_spi_device (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[SPI_DEVICE].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.spi_device[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_spi_device_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[13][Domain0Sel]),
    .fsm_err_o(fsm_errs[13][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_spi_device_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0SpiDeviceFsmCheck_A,
    u_d0_spi_device.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[13] = '0;
  assign shadow_fsm_errs[13] = '0;

  // Generating resets for spi_host0
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_spi_host0_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[14][DomainAonSel] = '0;
  assign fsm_errs[14][DomainAonSel] = '0;
  assign rst_en_o.spi_host0[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_spi_host0 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[SPI_HOST0].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.spi_host0[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_spi_host0_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[14][Domain0Sel]),
    .fsm_err_o(fsm_errs[14][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_spi_host0_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0SpiHost0FsmCheck_A,
    u_d0_spi_host0.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[14] = '0;
  assign shadow_fsm_errs[14] = '0;

  // Generating resets for spi_host1
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_spi_host1_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[15][DomainAonSel] = '0;
  assign fsm_errs[15][DomainAonSel] = '0;
  assign rst_en_o.spi_host1[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_spi_host1 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div2_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[SPI_HOST1].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.spi_host1[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_spi_host1_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[15][Domain0Sel]),
    .fsm_err_o(fsm_errs[15][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_spi_host1_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0SpiHost1FsmCheck_A,
    u_d0_spi_host1.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[15] = '0;
  assign shadow_fsm_errs[15] = '0;

  // Generating resets for usb
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_usb_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[16][DomainAonSel] = '0;
  assign fsm_errs[16][DomainAonSel] = '0;
  assign rst_en_o.usb[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_usb (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_usb_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[USB].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.usb[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_usb_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[16][Domain0Sel]),
    .fsm_err_o(fsm_errs[16][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_usb_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0UsbFsmCheck_A,
    u_d0_usb.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[16] = '0;
  assign shadow_fsm_errs[16] = '0;

  // Generating resets for usb_aon
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_usb_aon_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[17][DomainAonSel] = '0;
  assign fsm_errs[17][DomainAonSel] = '0;
  assign rst_en_o.usb_aon[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_usb_aon (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_aon_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[USB_AON].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.usb_aon[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_usb_aon_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[17][Domain0Sel]),
    .fsm_err_o(fsm_errs[17][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_usb_aon_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0UsbAonFsmCheck_A,
    u_d0_usb_aon.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[17] = '0;
  assign shadow_fsm_errs[17] = '0;

  // Generating resets for i2c0
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_i2c0_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[18][DomainAonSel] = '0;
  assign fsm_errs[18][DomainAonSel] = '0;
  assign rst_en_o.i2c0[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_i2c0 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[I2C0].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.i2c0[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_i2c0_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[18][Domain0Sel]),
    .fsm_err_o(fsm_errs[18][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_i2c0_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0I2c0FsmCheck_A,
    u_d0_i2c0.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[18] = '0;
  assign shadow_fsm_errs[18] = '0;

  // Generating resets for i2c1
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_i2c1_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[19][DomainAonSel] = '0;
  assign fsm_errs[19][DomainAonSel] = '0;
  assign rst_en_o.i2c1[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_i2c1 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[I2C1].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.i2c1[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_i2c1_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[19][Domain0Sel]),
    .fsm_err_o(fsm_errs[19][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_i2c1_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0I2c1FsmCheck_A,
    u_d0_i2c1.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[19] = '0;
  assign shadow_fsm_errs[19] = '0;

  // Generating resets for i2c2
  // Power Domains: ['0']
  // Shadowed: False
  assign resets_o.rst_i2c2_n[DomainAonSel] = '0;
  assign cnsty_chk_errs[20][DomainAonSel] = '0;
  assign fsm_errs[20][DomainAonSel] = '0;
  assign rst_en_o.i2c2[DomainAonSel] = MuBi4True;
  rstmgr_leaf_rst #(
    .SecCheck(SecCheck),
    .SecMaxSyncDelay(SecMaxSyncDelay),
    .SwRstReq(1'b1)
  ) u_d0_i2c2 (
    .clk_i,
    .rst_ni,
    .leaf_clk_i(clk_io_div4_i),
    .parent_rst_ni(rst_lc_src_n[Domain0Sel]),
    .sw_rst_req_ni(reg2hw.sw_rst_ctrl_n[I2C2].q),
    .scan_rst_ni,
    .scanmode_i,
    .rst_en_o(rst_en_o.i2c2[Domain0Sel]),
    .leaf_rst_o(resets_o.rst_i2c2_n[Domain0Sel]),
    .err_o(cnsty_chk_errs[20][Domain0Sel]),
    .fsm_err_o(fsm_errs[20][Domain0Sel])
  );

  if (SecCheck) begin : gen_d0_i2c2_assert
  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(
    D0I2c2FsmCheck_A,
    u_d0_i2c2.gen_rst_chk.u_rst_chk.u_state_regs,
    alert_tx_o[0])
  end
  assign shadow_cnsty_chk_errs[20] = '0;
  assign shadow_fsm_errs[20] = '0;


  ////////////////////////////////////////////////////
  // Reset info construction                        //
  ////////////////////////////////////////////////////

  logic rst_hw_req;
  logic rst_low_power;
  logic pwrmgr_rst_req;

  // there is a valid reset request from pwrmgr
  assign pwrmgr_rst_req = |pwr_i.rst_lc_req || |pwr_i.rst_sys_req;

  // a reset reason is only valid if the related processing element is also reset.
  // In the future, if ever there are multiple processing elements, this code here
  // must be updated to account for each individual core.
  assign rst_hw_req    = pwrmgr_rst_req &
                         (pwr_i.reset_cause == pwrmgr_pkg::HwReq);
  assign rst_low_power = pwrmgr_rst_req &
                         (pwr_i.reset_cause == pwrmgr_pkg::LowPwrEntry);

  // software initiated reset request
  assign sw_rst_req_o = prim_mubi_pkg::mubi4_t'(reg2hw.reset_req.q);

  // when pwrmgr reset request is received (reset is imminent), clear software
  // request so we are not in an infinite reset loop.
  assign hw2reg.reset_req.de = pwrmgr_rst_req;
  assign hw2reg.reset_req.d  = prim_mubi_pkg::MuBi4False;

  // Only sw is allowed to clear a reset reason, hw is only allowed to set it.
  assign hw2reg.reset_info.low_power_exit.d  = 1'b1;
  assign hw2reg.reset_info.low_power_exit.de = rst_low_power;

  // software issued request triggers the same response as hardware, although it is
  // accounted for differently.
  assign hw2reg.reset_info.sw_reset.d = prim_mubi_pkg::mubi4_test_true_strict(sw_rst_req_o) |
                                        reg2hw.reset_info.sw_reset.q;
  assign hw2reg.reset_info.sw_reset.de = rst_hw_req;

  // HW reset requests most likely will be multi-bit, so OR in whatever reasons
  // that are already set.
  assign hw2reg.reset_info.hw_req.d  = pwr_i.rstreqs |
                                       reg2hw.reset_info.hw_req.q;
  assign hw2reg.reset_info.hw_req.de = rst_hw_req;

  ////////////////////////////////////////////////////
  // Crash info capture                             //
  ////////////////////////////////////////////////////

  logic dump_capture;
  assign dump_capture =  rst_hw_req | rst_low_power;

  // halt dump capture once we hit particular conditions
  logic dump_capture_halt;
  assign dump_capture_halt = rst_hw_req;

  rstmgr_crash_info #(
    .CrashDumpWidth($bits(alert_pkg::alert_crashdump_t))
  ) u_alert_info (
    .clk_i(clk_por_i),
    .rst_ni(rst_por_ni),
    .dump_i(alert_dump_i),
    .dump_capture_i(dump_capture & reg2hw.alert_info_ctrl.en.q),
    .slot_sel_i(reg2hw.alert_info_ctrl.index.q),
    .slots_cnt_o(hw2reg.alert_info_attr.d),
    .slot_o(hw2reg.alert_info.d)
  );

  rstmgr_crash_info #(
    .CrashDumpWidth($bits(rv_core_ibex_pkg::cpu_crash_dump_t))
  ) u_cpu_info (
    .clk_i(clk_por_i),
    .rst_ni(rst_por_ni),
    .dump_i(cpu_dump_i),
    .dump_capture_i(dump_capture & reg2hw.cpu_info_ctrl.en.q),
    .slot_sel_i(reg2hw.cpu_info_ctrl.index.q),
    .slots_cnt_o(hw2reg.cpu_info_attr.d),
    .slot_o(hw2reg.cpu_info.d)
  );

  // once dump is captured, no more information is captured until
  // re-enabled by software.
  assign hw2reg.alert_info_ctrl.en.d  = 1'b0;
  assign hw2reg.alert_info_ctrl.en.de = dump_capture_halt;
  assign hw2reg.cpu_info_ctrl.en.d  = 1'b0;
  assign hw2reg.cpu_info_ctrl.en.de = dump_capture_halt;

  ////////////////////////////////////////////////////
  // Exported resets                                //
  ////////////////////////////////////////////////////




  ////////////////////////////////////////////////////
  // Assertions                                     //
  ////////////////////////////////////////////////////

  `ASSERT_INIT(ParameterMatch_A, NumHwResets == pwrmgr_pkg::HwResetWidth)

  // when upstream resets, downstream must also reset

  // output known asserts
  `ASSERT_KNOWN(TlDValidKnownO_A,    tl_o.d_valid  )
  `ASSERT_KNOWN(TlAReadyKnownO_A,    tl_o.a_ready  )
  `ASSERT_KNOWN(AlertsKnownO_A,      alert_tx_o    )
  `ASSERT_KNOWN(PwrKnownO_A,         pwr_o         )
  `ASSERT_KNOWN(ResetsKnownO_A,      resets_o      )
  `ASSERT_KNOWN(RstEnKnownO_A,       rst_en_o      )

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule // rstmgr

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * Ibex RISC-V core
 *
 * 32 bit RISC-V core supporting the RV32I + optionally EMC instruction sets.
 * Instruction and data bus are 32 bit wide TileLink-UL (TL-UL).
 */
module rv_core_ibex
  import rv_core_ibex_pkg::*;
  import rv_core_ibex_reg_pkg::*;
#(
    // Param list
  parameter int NumSwAlerts = 2,
  parameter int NumRegions = 2,
  parameter int NumScratchWords = 8,
  parameter int NumAlerts = 4,

  // Address widths within the block
  parameter int CfgAw = 8,

  parameter logic [NumAlerts-1:0] AlertAsyncOn     = {NumAlerts{1'b1}},
  parameter bit                   PMPEnable        = 1'b1,
  parameter int unsigned          PMPGranularity   = 0,
  parameter int unsigned          PMPNumRegions    = 16,
  parameter int unsigned          MHPMCounterNum   = 10,
  parameter int unsigned          MHPMCounterWidth = 32,
  parameter bit                   RV32E            = 0,
  parameter ibex_pkg::rv32m_e     RV32M            = ibex_pkg::RV32MSingleCycle,
  parameter ibex_pkg::rv32b_e     RV32B            = ibex_pkg::RV32BOTEarlGrey,
  parameter ibex_pkg::regfile_e   RegFile          = ibex_pkg::RegFileFF,
  parameter bit                   BranchTargetALU  = 1'b1,
  parameter bit                   WritebackStage   = 1'b1,
  parameter bit                   ICache           = 1'b1,
  parameter bit                   ICacheECC        = 1'b1,
  parameter bit                   ICacheScramble   = 1'b1,
  parameter bit                   BranchPredictor  = 1'b0,
  parameter bit                   DbgTriggerEn     = 1'b1,
  parameter int unsigned          DbgHwBreakNum    = 4,
  parameter bit                   SecureIbex       = 1'b1,
  parameter ibex_pkg::lfsr_seed_t RndCnstLfsrSeed  = ibex_pkg::RndCnstLfsrSeedDefault,
  parameter ibex_pkg::lfsr_perm_t RndCnstLfsrPerm  = ibex_pkg::RndCnstLfsrPermDefault,
  parameter int unsigned          DmHaltAddr       = 32'h1A110800,
  parameter int unsigned          DmExceptionAddr  = 32'h1A110808,
  parameter bit                   PipeLine         = 1'b0,
  parameter logic [ibex_pkg::SCRAMBLE_KEY_W-1:0] RndCnstIbexKeyDefault =
      ibex_pkg::RndCnstIbexKeyDefault,
  parameter logic [ibex_pkg::SCRAMBLE_NONCE_W-1:0] RndCnstIbexNonceDefault =
      ibex_pkg::RndCnstIbexNonceDefault
) (
  // Clock and Reset
  input  logic        clk_i,
  input  logic        rst_ni,
  // Clock domain for edn
  input  logic        clk_edn_i,
  input  logic        rst_edn_ni,
  // Clock domain for escalation receiver
  input  logic        clk_esc_i,
  input  logic        rst_esc_ni,
  // Reset feedback to rstmgr
  output logic        rst_cpu_n_o,

  input  prim_ram_1p_pkg::ram_1p_cfg_t ram_cfg_i,

  input  logic [31:0] hart_id_i,
  input  logic [31:0] boot_addr_i,

  // Instruction memory interface
  output tlul_pkg::tl_h2d_t     corei_tl_h_o,
  input  tlul_pkg::tl_d2h_t     corei_tl_h_i,

  // Data memory interface
  output tlul_pkg::tl_h2d_t     cored_tl_h_o,
  input  tlul_pkg::tl_d2h_t     cored_tl_h_i,

  // Interrupt inputs
  input  logic        irq_software_i,
  input  logic        irq_timer_i,
  input  logic        irq_external_i,

  // Escalation input for NMI
  input  prim_esc_pkg::esc_tx_t esc_tx_i,
  output prim_esc_pkg::esc_rx_t esc_rx_o,

  // watchdog NMI input
  input logic nmi_wdog_i,

  // Debug Interface
  input  logic        debug_req_i,

  // Crash dump information
  output cpu_crash_dump_t crash_dump_o,

  // CPU Control Signals
  input lc_ctrl_pkg::lc_tx_t lc_cpu_en_i,
  input lc_ctrl_pkg::lc_tx_t pwrmgr_cpu_en_i,
  output pwrmgr_pkg::pwr_cpu_t pwrmgr_o,

  // dft bypass
  input scan_rst_ni,
  input prim_mubi_pkg::mubi4_t scanmode_i,

  // peripheral interface access
  input  tlul_pkg::tl_h2d_t cfg_tl_d_i,
  output tlul_pkg::tl_d2h_t cfg_tl_d_o,

  // connection to edn
  output edn_pkg::edn_req_t edn_o,
  input edn_pkg::edn_rsp_t edn_i,

  // connection to otp scramble interface
  input clk_otp_i,
  input rst_otp_ni,
  output otp_ctrl_pkg::sram_otp_key_req_t icache_otp_key_o,
  input  otp_ctrl_pkg::sram_otp_key_rsp_t icache_otp_key_i,

  // fpga build info
  input [31:0] fpga_info_i,

  // interrupts and alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o

);

  import top_pkg::*;
  import tlul_pkg::*;

  // Register module
  rv_core_ibex_cfg_reg2hw_t reg2hw;
  rv_core_ibex_cfg_hw2reg_t hw2reg;

  // if pipeline=1, do not allow pass through and always break the path
  // if pipeline is 0, passthrough the fifo completely
  localparam bit FifoPass = PipeLine ? 1'b0 : 1'b1;
  localparam int unsigned FifoDepth = PipeLine ? 2 : 0;
  // ICache creates more outstanding transactions
  localparam int NumOutstandingReqs = ICache ? 8 : 2;

  // Instruction interface (internal)
  logic        instr_req;
  logic        instr_gnt;
  logic        instr_rvalid;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;
  logic [6:0]  instr_rdata_intg;
  logic        instr_err;

  // Data interface (internal)
  logic        data_req;
  logic        data_gnt;
  logic        data_rvalid;
  logic        data_we;
  logic [3:0]  data_be;
  logic [31:0] data_addr;
  logic [31:0] data_wdata;
  logic [6:0]  data_wdata_intg;
  logic [31:0] data_rdata;
  logic [6:0]  data_rdata_intg;
  logic        data_err;

  // Pipeline interfaces
  tl_h2d_t tl_i_ibex2fifo;
  tl_d2h_t tl_i_fifo2ibex;
  tl_h2d_t tl_d_ibex2fifo;
  tl_d2h_t tl_d_fifo2ibex;


  // core sleeping
  logic core_sleep;

  // The following intermediate signals are created to aid in simulations.
  //
  // If a parent port is connected directly to a port of sub-modules, the implicit wire connection
  // can have only one procedural driver (assign, force etc). What that means is, it prevents us
  // from forcing the sub-module port without impacting the same port on other sub-modules. The
  // reason for this is, regardless of which hierarchy the port signal is forced, it is a singular
  // wire-connected entity - the effect of the force ends up getting reflected on the same port of
  // sub-modules, as well as the parent port. To achieve the behavior of a force on a sub-module
  // port not impacting (i.e. back-propagating to) the same port on parent / peer sub-modules, we
  // need to add an extra `logic` type variables between the port-port connections.
  logic ibex_top_clk_i;
  logic addr_trans_rst_ni;
  assign ibex_top_clk_i = clk_i;
  assign addr_trans_rst_ni = rst_ni;

  // errors and core alert events
  logic ibus_intg_err, dbus_intg_err;
  logic alert_minor, alert_major_internal, alert_major_bus;
  logic double_fault;
  logic fatal_intg_err, fatal_core_err, recov_core_err;

  // alert events to peripheral module
  logic fatal_intg_event;
  logic fatal_core_event;
  logic recov_core_event;
  // SEC_CM: BUS.INTEGRITY
  assign fatal_intg_event = ibus_intg_err | dbus_intg_err | alert_major_bus;
  assign fatal_core_event = alert_major_internal | double_fault;
  assign recov_core_event = alert_minor;

  // configurations for address translation
  region_cfg_t [NumRegions-1:0] ibus_region_cfg;
  region_cfg_t [NumRegions-1:0] dbus_region_cfg;

  // Reset feedback to clkmgr
  assign rst_cpu_n_o = rst_ni;

  // Escalation receiver that converts differential
  // protocol into single ended signal.
  logic esc_irq_nm;
  prim_esc_receiver #(
    .N_ESC_SEV   (alert_handler_reg_pkg::N_ESC_SEV),
    .PING_CNT_DW (alert_handler_reg_pkg::PING_CNT_DW)
  ) u_prim_esc_receiver (
    .clk_i     ( clk_esc_i  ),
    .rst_ni    ( rst_esc_ni ),
    .esc_req_o ( esc_irq_nm ),
    .esc_rx_o,
    .esc_tx_i
  );

  // Synchronize to fast Ibex clock domain.
  logic alert_irq_nm;
  prim_flop_2sync #(
    .Width(1)
  ) u_alert_nmi_sync (
    .clk_i,
    .rst_ni,
    .d_i(esc_irq_nm),
    .q_o(alert_irq_nm)
  );

  logic wdog_irq_nm;
  prim_flop_2sync #(
    .Width(1)
  ) u_wdog_nmi_sync (
    .clk_i,
    .rst_ni,
    .d_i(nmi_wdog_i),
    .q_o(wdog_irq_nm)
  );

  assign hw2reg.nmi_state.alert.d  = 1'b1;
  assign hw2reg.nmi_state.alert.de = alert_irq_nm;
  assign hw2reg.nmi_state.wdog.d   = 1'b1;
  assign hw2reg.nmi_state.wdog.de  = wdog_irq_nm;

  logic irq_nm;
  assign irq_nm = |(reg2hw.nmi_state & reg2hw.nmi_enable);

  lc_ctrl_pkg::lc_tx_t [0:0] lc_cpu_en;
  prim_lc_sync u_lc_sync (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_cpu_en_i),
    .lc_en_o(lc_cpu_en)
  );

  lc_ctrl_pkg::lc_tx_t [0:0] pwrmgr_cpu_en;
  prim_lc_sync u_pwrmgr_sync (
    .clk_i,
    .rst_ni,
    .lc_en_i(pwrmgr_cpu_en_i),
    .lc_en_o(pwrmgr_cpu_en)
  );

  // timer interrupts do not come from
  // rv_plic and may not be synchronous to the ibex core
  logic irq_timer_sync;
  prim_flop_2sync #(
    .Width(1)
  ) u_intr_timer_sync (
    .clk_i,
    .rst_ni,
    .d_i(irq_timer_i),
    .q_o(irq_timer_sync)
  );


  logic irq_software;
  logic irq_timer;
  logic irq_external;

  prim_sec_anchor_buf #(
    .Width(3)
  ) u_prim_buf_irq (
    .in_i({irq_software_i,
           irq_timer_sync,
           irq_external_i}),
    .out_o({irq_software,
            irq_timer,
            irq_external})
  );


  logic key_req, key_ack;
  logic [ibex_pkg::SCRAMBLE_KEY_W-1:0] key;
  logic [ibex_pkg::SCRAMBLE_NONCE_W-1:0] nonce;
  logic unused_seed_valid;
  localparam int PayLoadW = ibex_pkg::SCRAMBLE_KEY_W + ibex_pkg::SCRAMBLE_NONCE_W + 1;
  prim_sync_reqack_data #(
    .Width(PayLoadW),
    .DataSrc2Dst(1'b0)
  ) u_prim_sync_reqack_data (
    .clk_src_i  ( clk_i                         ),
    .rst_src_ni ( rst_ni                        ),
    .clk_dst_i  ( clk_otp_i                     ),
    .rst_dst_ni ( rst_otp_ni                    ),
    .req_chk_i  ( 1'b1                          ),
    .src_req_i  ( key_req                       ),
    .src_ack_o  ( key_ack                       ),
    .dst_req_o  ( icache_otp_key_o.req          ),
    .dst_ack_i  ( icache_otp_key_i.ack          ),
    .data_i     ( {icache_otp_key_i.key,
                   icache_otp_key_i.nonce[ibex_pkg::SCRAMBLE_NONCE_W-1:0],
                   icache_otp_key_i.seed_valid} ),
    .data_o     ( {key,
                   nonce,
                   unused_seed_valid}           )
  );

  logic unused_nonce;
  assign unused_nonce = |icache_otp_key_i.nonce;

  // Local fetch enable control.
  // Whenever a fatal core error is seen disable local fetch enable.
  lc_ctrl_pkg::lc_tx_t local_fetch_enable_d, local_fetch_enable_q;

  assign local_fetch_enable_d = fatal_core_err ? lc_ctrl_pkg::Off : local_fetch_enable_q;

  prim_lc_sender #(
    .AsyncOn(1), // this instantiates a register
    .ResetValueIsOn(1)
  ) u_prim_lc_sender (
    .clk_i,
    .rst_ni,
    .lc_en_i(local_fetch_enable_d),
    .lc_en_o(local_fetch_enable_q)
  );

  // Multibit AND computation for fetch enable. Fetch is only enabled when local fetch enable,
  // lifecycle CPU enable and power manager CPU enable are all enabled.
  lc_ctrl_pkg::lc_tx_t fetch_enable;
  assign fetch_enable = lc_ctrl_pkg::lc_tx_and_hi(local_fetch_enable_q,
                                                  lc_ctrl_pkg::lc_tx_and_hi(lc_cpu_en[0],
                                                                            pwrmgr_cpu_en[0]));

  ibex_pkg::crash_dump_t crash_dump;
  ibex_top #(
    .PMPEnable                   ( PMPEnable                ),
    .PMPGranularity              ( PMPGranularity           ),
    .PMPNumRegions               ( PMPNumRegions            ),
    .MHPMCounterNum              ( MHPMCounterNum           ),
    .MHPMCounterWidth            ( MHPMCounterWidth         ),
    .RV32E                       ( RV32E                    ),
    .RV32M                       ( RV32M                    ),
    .RV32B                       ( RV32B                    ),
    .RegFile                     ( RegFile                  ),
    .BranchTargetALU             ( BranchTargetALU          ),
    .WritebackStage              ( WritebackStage           ),
    .ICache                      ( ICache                   ),
    // Our automatic SEC_CM label check doesn't look at vendored code so the SEC_CM labels need
    // to be mentioned here. The real locations can be found by grepping the vendored code.
    // TODO(#10071): this should be fixed.
    // SEC_CM: ICACHE.MEM.INTEGRITY
    .ICacheECC                   ( ICacheECC                ),
    // SEC_CM: ICACHE.MEM.SCRAMBLE, SCRAMBLE.KEY.SIDELOAD
    .ICacheScramble              ( ICacheScramble           ),
    // Reduce the number of PRINCE half rounds to 2 (5 effective rounds) to ease timing. This is
    // acceptable for the instruction cache, whereas 3 half rounds (7 effective rounds) are used
    // elsewhere in the design.
    .ICacheScrNumPrinceRoundsHalf( 2                        ),
    .BranchPredictor             ( BranchPredictor          ),
    .DbgTriggerEn                ( DbgTriggerEn             ),
    .DbgHwBreakNum               ( DbgHwBreakNum            ),
    // SEC_CM: LOGIC.SHADOW
    // SEC_CM: PC.CTRL_FLOW.CONSISTENCY, CTRL_FLOW.UNPREDICTABLE, CORE.DATA_REG_SW.SCA
    // SEC_CM: EXCEPTION.CTRL_FLOW.GLOBAL_ESC, EXCEPTION.CTRL_FLOW.LOCAL_ESC
    // SEC_CM: DATA_REG_SW.INTEGRITY, DATA_REG_SW.GLITCH_DETECT
    .SecureIbex                  ( SecureIbex               ),
    .RndCnstLfsrSeed             ( RndCnstLfsrSeed          ),
    .RndCnstLfsrPerm             ( RndCnstLfsrPerm          ),
    .RndCnstIbexKey              ( RndCnstIbexKeyDefault    ),
    .RndCnstIbexNonce            ( RndCnstIbexNonceDefault  ),
    .DmHaltAddr                  ( DmHaltAddr               ),
    .DmExceptionAddr             ( DmExceptionAddr          )
  ) u_core (
    .clk_i              (ibex_top_clk_i),
    .rst_ni,


    .test_en_i          (prim_mubi_pkg::mubi4_test_true_strict(scanmode_i)),
    .scan_rst_ni,

    .ram_cfg_i,

    .hart_id_i,
    .boot_addr_i,

    .instr_req_o        ( instr_req        ),
    .instr_gnt_i        ( instr_gnt        ),
    .instr_rvalid_i     ( instr_rvalid     ),
    .instr_addr_o       ( instr_addr       ),
    .instr_rdata_i      ( instr_rdata      ),
    .instr_rdata_intg_i ( instr_rdata_intg ),
    .instr_err_i        ( instr_err        ),

    .data_req_o         ( data_req         ),
    .data_gnt_i         ( data_gnt         ),
    .data_rvalid_i      ( data_rvalid      ),
    .data_we_o          ( data_we          ),
    .data_be_o          ( data_be          ),
    .data_addr_o        ( data_addr        ),
    .data_wdata_o       ( data_wdata       ),
    .data_wdata_intg_o  ( data_wdata_intg  ),
    .data_rdata_i       ( data_rdata       ),
    .data_rdata_intg_i  ( data_rdata_intg  ),
    .data_err_i         ( data_err         ),

    .irq_software_i     ( irq_software     ),
    .irq_timer_i        ( irq_timer        ),
    .irq_external_i     ( irq_external     ),
    .irq_fast_i         ( '0               ),
    .irq_nm_i           ( irq_nm           ),

    .debug_req_i,
    .crash_dump_o       ( crash_dump       ),

    // icache scramble interface
    .scramble_key_valid_i (key_ack),
    .scramble_key_i       (key),
    .scramble_nonce_i     (nonce),
    .scramble_req_o       (key_req),

    // double fault
    .double_fault_seen_o  (double_fault),

    // SEC_CM: FETCH.CTRL.LC_GATED
    .fetch_enable_i         (fetch_enable),
    .alert_minor_o          (alert_minor),
    .alert_major_internal_o (alert_major_internal),
    .alert_major_bus_o      (alert_major_bus),
    .core_sleep_o           (core_sleep)
  );

  logic core_sleep_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      core_sleep_q <= '0;
    end else begin
      core_sleep_q <= core_sleep;
    end
  end

  prim_buf #(
    .Width(1)
  ) u_core_sleeping_buf (
    .in_i(core_sleep_q),
    .out_o(pwrmgr_o.core_sleeping)
  );



  logic prev_valid;
  logic [31:0] prev_exception_pc;
  logic [31:0] prev_exception_addr;

  assign crash_dump_o.current = crash_dump;
  assign crash_dump_o.prev_valid = prev_valid;
  assign crash_dump_o.prev_exception_pc = prev_exception_pc;
  assign crash_dump_o.prev_exception_addr = prev_exception_addr;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      prev_valid <= '0;
      prev_exception_pc <= '0;
      prev_exception_addr <= '0;
    end else if (double_fault) begin
      prev_valid <= 1'b1;
      prev_exception_pc <= crash_dump.exception_pc;
      prev_exception_addr <= crash_dump.exception_addr;
    end
  end


  //
  // Convert ibex data/instruction bus to TL-UL
  //
  logic [31:0] instr_addr_trans;
  rv_core_addr_trans #(
    .AddrWidth(32),
    .NumRegions(NumRegions)
  ) u_ibus_trans (
    .clk_i,
    .rst_ni(addr_trans_rst_ni),
    .region_cfg_i(ibus_region_cfg),
    .addr_i(instr_addr),
    .addr_o(instr_addr_trans)
  );

  logic [6:0]  instr_wdata_intg;
  logic [top_pkg::TL_DW-1:0] unused_data;
  // tl_adapter_host_i_ibex only reads instruction. a_data is always 0
  assign {instr_wdata_intg, unused_data} = prim_secded_pkg::prim_secded_inv_39_32_enc('0);
  // SEC_CM: BUS.INTEGRITY
  tlul_adapter_host #(
    .MAX_REQS(NumOutstandingReqs),
    // if secure ibex is not set, data integrity is not generated
    // from ibex, therefore generate it in the gasket instead.
    .EnableDataIntgGen(~SecureIbex)
  ) tl_adapter_host_i_ibex (
    .clk_i,
    .rst_ni,
    .req_i        (instr_req),
    .instr_type_i (prim_mubi_pkg::MuBi4True),
    .gnt_o        (instr_gnt),
    .addr_i       (instr_addr_trans),
    .we_i         (1'b0),
    .wdata_i      (32'b0),
    .wdata_intg_i (instr_wdata_intg),
    .be_i         (4'hF),
    .valid_o      (instr_rvalid),
    .rdata_o      (instr_rdata),
    .rdata_intg_o (instr_rdata_intg),
    .err_o        (instr_err),
    .intg_err_o   (ibus_intg_err),
    .tl_o         (tl_i_ibex2fifo),
    .tl_i         (tl_i_fifo2ibex)
  );

  tlul_fifo_sync #(
    .ReqPass(FifoPass),
    .RspPass(FifoPass),
    .ReqDepth(FifoDepth),
    .RspDepth(FifoDepth)
  ) fifo_i (
    .clk_i,
    .rst_ni,
    .tl_h_i      (tl_i_ibex2fifo),
    .tl_h_o      (tl_i_fifo2ibex),
    .tl_d_o      (corei_tl_h_o),
    .tl_d_i      (corei_tl_h_i),
    .spare_req_i (1'b0),
    .spare_req_o (),
    .spare_rsp_i (1'b0),
    .spare_rsp_o ());

  logic [31:0] data_addr_trans;
  rv_core_addr_trans #(
    .AddrWidth(32),
    .NumRegions(NumRegions)
  ) u_dbus_trans (
    .clk_i,
    .rst_ni(addr_trans_rst_ni),
    .region_cfg_i(dbus_region_cfg),
    .addr_i(data_addr),
    .addr_o(data_addr_trans)
  );

  // SEC_CM: BUS.INTEGRITY
  tlul_adapter_host #(
    .MAX_REQS(2),
    .EnableDataIntgGen(~SecureIbex)
  ) tl_adapter_host_d_ibex (
    .clk_i,
    .rst_ni,
    .req_i        (data_req),
    .instr_type_i (prim_mubi_pkg::MuBi4False),
    .gnt_o        (data_gnt),
    .addr_i       (data_addr_trans),
    .we_i         (data_we),
    .wdata_i      (data_wdata),
    .wdata_intg_i (data_wdata_intg),
    .be_i         (data_be),
    .valid_o      (data_rvalid),
    .rdata_o      (data_rdata),
    .rdata_intg_o (data_rdata_intg),
    .err_o        (data_err),
    .intg_err_o   (dbus_intg_err),
    .tl_o         (tl_d_ibex2fifo),
    .tl_i         (tl_d_fifo2ibex)
  );

  tlul_fifo_sync #(
    .ReqPass(FifoPass),
    .RspPass(FifoPass),
    .ReqDepth(FifoDepth),
    .RspDepth(FifoDepth)
  ) fifo_d (
    .clk_i,
    .rst_ni,
    .tl_h_i      (tl_d_ibex2fifo),
    .tl_h_o      (tl_d_fifo2ibex),
    .tl_d_o      (cored_tl_h_o),
    .tl_d_i      (cored_tl_h_i),
    .spare_req_i (1'b0),
    .spare_req_o (),
    .spare_rsp_i (1'b0),
    .spare_rsp_o ());


  //////////////////////////////////
  // Peripheral functions
  //////////////////////////////////

  logic intg_err;
  tlul_pkg::tl_h2d_t tl_win_h2d;
  tlul_pkg::tl_d2h_t tl_win_d2h;
  rv_core_ibex_cfg_reg_top u_reg_cfg (
    .clk_i,
    .rst_ni,
    .tl_i(cfg_tl_d_i),
    .tl_o(cfg_tl_d_o),
    .reg2hw,
    .hw2reg,
    .intg_err_o (intg_err),
    .tl_win_o(tl_win_h2d),
    .tl_win_i(tl_win_d2h)
  );

  ///////////////////////
  // Region assignments
  ///////////////////////

  for(genvar i = 0; i < NumRegions; i++) begin : gen_ibus_region_cfgs
    assign ibus_region_cfg[i].en = reg2hw.ibus_addr_en[i];
    assign ibus_region_cfg[i].matching_region = reg2hw.ibus_addr_matching[i];
    assign ibus_region_cfg[i].remap_addr = reg2hw.ibus_remap_addr[i];
  end

  for(genvar i = 0; i < NumRegions; i++) begin : gen_dbus_region_cfgs
    assign dbus_region_cfg[i].en = reg2hw.dbus_addr_en[i];
    assign dbus_region_cfg[i].matching_region = reg2hw.dbus_addr_matching[i];
    assign dbus_region_cfg[i].remap_addr = reg2hw.dbus_remap_addr[i];
  end

  ///////////////////////
  // Error assignment
  ///////////////////////

  assign fatal_intg_err = fatal_intg_event;
  assign fatal_core_err = fatal_core_event;
  assign recov_core_err = recov_core_event;

  assign hw2reg.err_status.reg_intg_err.d = 1'b1;
  assign hw2reg.err_status.reg_intg_err.de = intg_err;
  assign hw2reg.err_status.fatal_intg_err.d = 1'b1;
  assign hw2reg.err_status.fatal_intg_err.de = fatal_intg_err;
  assign hw2reg.err_status.fatal_core_err.d = 1'b1;
  assign hw2reg.err_status.fatal_core_err.de = fatal_core_err;
  assign hw2reg.err_status.recov_core_err.d = 1'b1;
  assign hw2reg.err_status.recov_core_err.de = recov_core_err;

  ///////////////////////
  // Alert generation
  ///////////////////////

  logic [NumAlerts-1:0] alert_test;
  assign alert_test[0] = reg2hw.alert_test.fatal_sw_err.q &
                         reg2hw.alert_test.fatal_sw_err.qe;
  assign alert_test[1] = reg2hw.alert_test.recov_sw_err.q &
                         reg2hw.alert_test.recov_sw_err.qe;
  assign alert_test[2] = reg2hw.alert_test.fatal_hw_err.q &
                         reg2hw.alert_test.fatal_hw_err.qe;
  assign alert_test[3] = reg2hw.alert_test.recov_hw_err.q &
                         reg2hw.alert_test.recov_hw_err.qe;

  localparam bit [NumAlerts-1:0] AlertFatal = '{1'b0, 1'b1, 1'b0, 1'b1};

  logic [NumAlerts-1:0] alert_events;
  logic [NumAlerts-1:0] alert_acks;

  import prim_mubi_pkg::mubi4_test_true_loose;
  import prim_mubi_pkg::mubi4_t;
  assign alert_events[0] = mubi4_test_true_loose(mubi4_t'(reg2hw.sw_fatal_err.q));
  assign alert_events[1] = mubi4_test_true_loose(mubi4_t'(reg2hw.sw_recov_err.q));
  assign alert_events[2] = intg_err | fatal_intg_err | fatal_core_err;
  assign alert_events[3] = recov_core_err;

  logic unused_alert_acks;
  assign unused_alert_acks = |alert_acks;

  // recoverable alerts are sent once and silenced until activated again.
  assign hw2reg.sw_recov_err.de = alert_acks[1];
  assign hw2reg.sw_recov_err.d = prim_mubi_pkg::MuBi4False;

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_senders
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[0]),
      .IsFatal(AlertFatal[i])
    ) u_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i(alert_test[i]),
      .alert_req_i(alert_events[i]),
      .alert_ack_o(alert_acks[i]),
      .alert_state_o(),
      .alert_rx_i(alert_rx_i[i]),
      .alert_tx_o(alert_tx_o[i])
    );
  end

  //////////////
  // RND Data //
  //////////////

  logic [31:0] rnd_data_q, rnd_data_d;
  logic rnd_valid_q, rnd_valid_d;
  logic rnd_fips_q, rnd_fips_d;
  logic edn_req;
  logic [31:0] edn_data;
  logic edn_ack;
  logic edn_fips;

  always_comb begin
    rnd_valid_d = rnd_valid_q;
    rnd_data_d  = rnd_data_q;
    rnd_fips_d  = rnd_fips_q;

    if (reg2hw.rnd_data.re) begin
      rnd_valid_d = '0;
      rnd_data_d  = '0;
      rnd_fips_d  = '0;
    end else if (edn_req && edn_ack) begin
      rnd_valid_d = 1'b1;
      rnd_data_d  = edn_data;
      rnd_fips_d  = edn_fips;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rnd_valid_q <= '0;
      rnd_data_q  <= '0;
      rnd_fips_q  <= '0;
    end else begin
      rnd_valid_q <= rnd_valid_d;
      rnd_data_q  <= rnd_data_d;
      rnd_fips_q  <= rnd_fips_d;
    end
  end

  assign edn_req = ~rnd_valid_q;

  prim_edn_req #(
    .OutWidth(32)
  ) u_edn_if (
    .clk_i,
    .rst_ni,
    .req_chk_i(1'b1),
    .req_i(edn_req),
    .ack_o(edn_ack),
    .data_o(edn_data),
    .fips_o(edn_fips),
    .err_o(),
    .clk_edn_i,
    .rst_edn_ni,
    .edn_o,
    .edn_i
  );

  assign hw2reg.rnd_data.d                  = rnd_data_q;
  assign hw2reg.rnd_status.rnd_data_valid.d = rnd_valid_q;
  assign hw2reg.rnd_status.rnd_data_fips.d  = rnd_fips_q;

  logic unused_reg2hw;
  assign unused_reg2hw = |reg2hw.rnd_data.q;


  // fpga build info hook-up
  assign hw2reg.fpga_info.d = fpga_info_i;

  /////////////////////////////////////
  // The carved out space is for DV emulation purposes only
  /////////////////////////////////////

  import tlul_pkg::tl_h2d_t;
  import tlul_pkg::tl_d2h_t;
  localparam int TlH2DWidth = $bits(tl_h2d_t);
  localparam int TlD2HWidth = $bits(tl_d2h_t);

  logic [TlH2DWidth-1:0] tl_win_h2d_int;
  logic [TlD2HWidth-1:0] tl_win_d2h_int;
  tl_d2h_t tl_win_d2h_err_rsp;

  prim_buf #(
    .Width(TlH2DWidth)
  ) u_tlul_req_buf (
    .in_i(tl_win_h2d),
    .out_o(tl_win_h2d_int)
  );

  prim_buf #(
    .Width(TlD2HWidth)
  ) u_tlul_rsp_buf (
    .in_i(tl_win_d2h_err_rsp),
    .out_o(tl_win_d2h_int)
  );

  // Interception point for connecting simulation SRAM by disconnecting the tl_d output. The
  // disconnection is done only if `SYNTHESIS is NOT defined AND `RV_CORE_IBEX_SIM_SRAM is
  // defined.
  // This define is used only for verilator as verilator does not support forces.
  assign tl_win_d2h = tl_d2h_t'(tl_win_d2h_int);

  tlul_err_resp u_sim_win_rsp (
    .clk_i,
    .rst_ni,
    .tl_h_i(tl_h2d_t'(tl_win_h2d_int)),
    .tl_h_o(tl_win_d2h_err_rsp)
  );

  // Assertions for CPU enable
  // Allow 2 or 3 cycles for input to enable due to synchronizers
  `ASSERT(FpvSecCmIbexFetchEnable0_A,
      fatal_core_err
      |=>
      lc_ctrl_pkg::lc_tx_test_false_loose(fetch_enable))
  `ASSERT(FpvSecCmIbexFetchEnable1_A,
      lc_ctrl_pkg::lc_tx_test_false_loose(lc_cpu_en_i)
      |->
      ##[2:3] lc_ctrl_pkg::lc_tx_test_false_loose(fetch_enable))
  `ASSERT(FpvSecCmIbexFetchEnable2_A,
      lc_ctrl_pkg::lc_tx_test_false_loose(pwrmgr_cpu_en_i)
      |->
      ##[2:3] lc_ctrl_pkg::lc_tx_test_false_loose(fetch_enable))
  `ASSERT(FpvSecCmIbexFetchEnable3_A,
      lc_ctrl_pkg::lc_tx_test_true_strict(lc_cpu_en_i) &&
      lc_ctrl_pkg::lc_tx_test_true_strict(pwrmgr_cpu_en_i) ##1
      lc_ctrl_pkg::lc_tx_test_true_strict(local_fetch_enable_q) &&
      !fatal_core_err
      |=>
      ##[0:1] lc_ctrl_pkg::lc_tx_test_true_strict(fetch_enable))
  `ASSERT(FpvSecCmIbexFetchEnable3Rev_A,
      ##2 lc_ctrl_pkg::lc_tx_test_true_strict(fetch_enable)
      |->
      ($past(lc_ctrl_pkg::lc_tx_test_true_strict(lc_cpu_en_i), 2) ||
       $past(lc_ctrl_pkg::lc_tx_test_true_strict(lc_cpu_en_i), 3)) &&
      ($past(lc_ctrl_pkg::lc_tx_test_true_strict(pwrmgr_cpu_en_i), 2) ||
       $past(lc_ctrl_pkg::lc_tx_test_true_strict(pwrmgr_cpu_en_i), 3)) &&
      $past(!fatal_core_err))

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg_cfg, alert_tx_o[2])
  `ASSERT_PRIM_ONEHOT_ERROR_TRIGGER_ALERT(RvCoreRegWeOnehotCheck_A,
      u_core.gen_regfile_ff.register_file_i.gen_wren_check.u_prim_onehot_check, alert_tx_o[2])
  `ASSERT_PRIM_ONEHOT_ERROR_TRIGGER_ALERT(RvCoreRegWeOnehotCheckRAddrA_A,
        u_core.gen_regfile_ff.register_file_i.gen_rdata_mux_check.u_prim_onehot_check_raddr_a,
        alert_tx_o[2])
  `ASSERT_PRIM_ONEHOT_ERROR_TRIGGER_ALERT(RvCoreRegWeOnehotCheckRAddrB_A,
        u_core.gen_regfile_ff.register_file_i.gen_rdata_mux_check.u_prim_onehot_check_raddr_b,
        alert_tx_o[2])


endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Top-level debug module (DM)
//
// This module implements the RISC-V debug specification version 0.13,
//
// This toplevel wraps the PULP debug module available from
// https://github.com/pulp-platform/riscv-dbg to match the needs of
// the TL-UL-based lowRISC chip design.


module rv_dm
  import rv_dm_reg_pkg::*;
#(
  
  // Param list
  parameter int NrHarts = 1,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int RegsAw = 4,
  parameter int MemAw = 12,

  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}},
  parameter logic [31:0]          IdcodeValue  = 32'h 0000_0001
) (
  input  logic                clk_i,       // clock
  input  logic                clk_lc_i,    // only declared here so that the topgen
                                           // tooling connects the correct clk/rst domains.
  input  logic                rst_ni,      // asynchronous reset active low, connect PoR
                                           // here, not the system reset
  input  logic                rst_lc_ni,  // asynchronous reset active low, connect the lc
                                           // reset here. this is only used for NDM reset tracking.
  input  logic [31:0]         next_dm_addr_i, // static word address of the next debug module.
  // SEC_CM: LC_HW_DEBUG_EN.INTERSIG.MUBI
  // HW Debug lifecycle enable signal (live version from the life cycle controller)
  input  lc_ctrl_pkg::lc_tx_t lc_hw_debug_en_i,
  // SEC_CM: LC_DFT_EN.INTERSIG.MUBI
  // HW DFT lifecycle enable signal (live version from the life cycle controller)
  input  lc_ctrl_pkg::lc_tx_t lc_dft_en_i,
  // HW Debug lifecycle enable signal (latched version from pinmux, only used for JTAG/TAP gating)
  input  lc_ctrl_pkg::lc_tx_t pinmux_hw_debug_en_i,
  // SEC_CM: OTP_DIS_RV_DM_LATE_DEBUG.INTERSIG.MUBI
  // Late debug enable disablement signal coming from the OTP HW_CFG1 partition.
  input  prim_mubi_pkg::mubi8_t otp_dis_rv_dm_late_debug_i,
  input  prim_mubi_pkg::mubi4_t scanmode_i,
  input                       scan_rst_ni,
  output logic                ndmreset_req_o,  // non-debug module reset
  output logic                dmactive_o,  // debug module is active
  output logic [NrHarts-1:0]  debug_req_o, // async debug request
  input  logic [NrHarts-1:0]  unavailable_i, // communicate whether the hart is unavailable
                                             // (e.g.: power down)

  // bus device for comportable CSR access
  input  tlul_pkg::tl_h2d_t  regs_tl_d_i,
  output tlul_pkg::tl_d2h_t  regs_tl_d_o,

  // bus device with debug memory, for an execution based technique
  input  tlul_pkg::tl_h2d_t  mem_tl_d_i,
  output tlul_pkg::tl_d2h_t  mem_tl_d_o,

  // bus host, for system bus accesses
  output tlul_pkg::tl_h2d_t  sba_tl_h_o,
  input  tlul_pkg::tl_d2h_t  sba_tl_h_i,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  input  jtag_pkg::jtag_req_t jtag_i,
  output jtag_pkg::jtag_rsp_t jtag_o
);

  ///////////////////////////
  // Parameter Definitions //
  ///////////////////////////

  import prim_mubi_pkg::mubi4_bool_to_mubi;
  import prim_mubi_pkg::mubi4_test_true_strict;
  import prim_mubi_pkg::mubi8_test_true_strict;
  import prim_mubi_pkg::mubi32_test_true_strict;
  import lc_ctrl_pkg::lc_tx_test_true_strict;

  `ASSERT_INIT(paramCheckNrHarts, NrHarts > 0)

  // static debug hartinfo
  localparam dm::hartinfo_t DebugHartInfo = '{
    zero1:      '0,
    nscratch:   2, // Debug module needs at least two scratch regs
    zero0:      0,
    dataaccess: 1'b1, // data registers are memory mapped in the debugger
    datasize:   dm::DataCount,
    dataaddr:   dm::DataAddr
  };

  dm::hartinfo_t [NrHarts-1:0]      hartinfo;
  for (genvar i = 0; i < NrHarts; i++) begin : gen_dm_hart_ctrl
    assign hartinfo[i] = DebugHartInfo;
  end

  // Currently only 32 bit busses are supported by our TL-UL IP
  localparam int BusWidth = 32;
  // all harts have contiguous IDs
  localparam logic [NrHarts-1:0] SelectableHarts = {NrHarts{1'b1}};

  ///////////////
  // CSR Nodes //
  ///////////////

  tlul_pkg::tl_h2d_t mem_tl_win_h2d;
  tlul_pkg::tl_d2h_t mem_tl_win_d2h;
  rv_dm_reg_pkg::rv_dm_regs_reg2hw_t regs_reg2hw;
  logic regs_intg_error, rom_intg_error;
  logic sba_gate_intg_error, rom_gate_intg_error;

  rv_dm_regs_reg_top u_reg_regs (
    .clk_i,
    .rst_ni,
    .tl_i      (regs_tl_d_i    ),
    .tl_o      (regs_tl_d_o    ),
    .reg2hw    (regs_reg2hw    ),
    // SEC_CM: BUS.INTEGRITY
    .intg_err_o(regs_intg_error)
  );

  // We are not instantiating the generated rv_dm_mem_reg_top, since the registers are actually
  // implemented inside the vendored-in rv_dm module from the PULP project.
  assign mem_tl_win_h2d = mem_tl_d_i;
  assign mem_tl_d_o = mem_tl_win_d2h;

  // Alerts
  logic [NumAlerts-1:0] alert_test, alerts;

  assign alerts[0] = regs_intg_error | rom_intg_error |
                     sba_gate_intg_error | rom_gate_intg_error;

  assign alert_test = {
    regs_reg2hw.alert_test.q &
    regs_reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  // Decode multibit scanmode enable
  logic testmode;
  assign testmode = mubi4_test_true_strict(scanmode_i);

  ///////////////////////
  // Life Cycle Gating //
  ///////////////////////

  // Debug enable gating.
  localparam int LcEnDebugReqVal = 4 - 1;
  localparam int LcEnResetReqVal = LcEnDebugReqVal + NrHarts;
  // +1 to get number of bits and another +1 because LcEnLastPos is one more than LcEnResetReq.
  localparam int RvDmLcEnSize    = $clog2(LcEnResetReqVal + 2);
  typedef enum logic [RvDmLcEnSize-1:0] {
    LcEnFetch,
    LcEnRom,
    LcEnSba,
    // LcEnDebugReq[NrHarts], <= this unfortunately does not work - SV-LRM mandates the use of
    // integral numbers. Parameters are not allowed in this context.
    LcEnDebugReq,
    // The above literal accommodates NrHarts number of debug requests - so we number the next
    // literal accordingly.
    LcEnResetReq = RvDmLcEnSize'(LcEnResetReqVal),
    // LcEnLastPos must immediately follow LcEnResetReq to calculate RvDmLcEnSize.
    LcEnLastPos
  } rv_dm_lc_en_e;
  // These must be equal so that the difference between LcEnResetReq and LcEnDebugReq is NrHarts.
  `ASSERT(RvDmLcEnDebugVal_A, int'(LcEnDebugReq) == LcEnDebugReqVal)

  // debug enable gating
  typedef enum logic [3:0] {
    PmEnDmiReq,
    PmEnJtagIn,
    PmEnJtagOut,
    PmEnLastPos
  } rv_dm_pm_en_e;

  lc_ctrl_pkg::lc_tx_t lc_hw_debug_en;
  prim_lc_sync #(
    .NumCopies(1)
  ) u_prim_lc_sync_lc_hw_debug_en (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_hw_debug_en_i),
    .lc_en_o({lc_hw_debug_en})
  );

  lc_ctrl_pkg::lc_tx_t lc_dft_en;
  prim_lc_sync #(
    .NumCopies(1)
  ) u_prim_lc_sync_lc_dft_en (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_dft_en_i),
    .lc_en_o({lc_dft_en})
  );

  prim_mubi_pkg::mubi8_t [lc_ctrl_pkg::TxWidth-1:0] otp_dis_rv_dm_late_debug;
  prim_mubi8_sync #(
    .NumCopies (lc_ctrl_pkg::TxWidth)
  ) u_prim_mubi8_sync_otp_dis_rv_dm_late_debug (
    .clk_i,
    .rst_ni,
    .mubi_i(otp_dis_rv_dm_late_debug_i),
    .mubi_o(otp_dis_rv_dm_late_debug)
  );

  prim_mubi_pkg::mubi32_t [lc_ctrl_pkg::TxWidth-1:0] late_debug_enable;
  prim_mubi32_sync #(
    .NumCopies (lc_ctrl_pkg::TxWidth),
    .AsyncOn(0) // No synchronization required since the input signal is already synchronous.
  ) u_prim_mubi32_sync_late_debug_enable (
    .clk_i,
    .rst_ni,
    .mubi_i(prim_mubi_pkg::mubi32_t'(regs_reg2hw.late_debug_enable)),
    .mubi_o(late_debug_enable)
  );

  // SEC_CM: DM_EN.CTRL.LC_GATED
  // This implements a hardened MuBi multiplexor circuit where each output bitlane has its own
  // associated comparators for the enablement condition.
  logic [lc_ctrl_pkg::TxWidth-1:0] lc_hw_debug_en_raw;
  logic [lc_ctrl_pkg::TxWidth-1:0] lc_dft_en_raw;
  logic [lc_ctrl_pkg::TxWidth-1:0] lc_hw_debug_en_gated_raw;
  assign lc_hw_debug_en_raw = lc_hw_debug_en;
  assign lc_dft_en_raw = lc_dft_en;
  for (genvar k = 0; k < lc_ctrl_pkg::TxWidth; k++) begin : gen_mubi_mux
    assign lc_hw_debug_en_gated_raw[k] = (mubi8_test_true_strict(otp_dis_rv_dm_late_debug[k]) ||
                                          mubi32_test_true_strict(late_debug_enable[k])) ?
                                          lc_hw_debug_en_raw[k] :
                                          lc_dft_en_raw[k];
  end

  // The lc_hw_debug_en_gated signal modulates gating logic on the bus-side of the RV_DM.
  // The pinmux_hw_debug_en signal on the other hand modulates the TAP side of the RV_DM.
  // In order for the RV_DM to remain response during a NDM reset request, the TAP side
  // is not further modulated with the LATE_DEBUG_ENABLE CSR.
  lc_ctrl_pkg::lc_tx_t [LcEnLastPos-1:0] lc_hw_debug_en_gated;
  prim_lc_sync #(
    .NumCopies(int'(LcEnLastPos)),
    .AsyncOn(0) // No synchronization required since the input signal is already synchronous.
  ) u_lc_en_sync_copies (
    .clk_i,
    .rst_ni,
    .lc_en_i(lc_ctrl_pkg::lc_tx_t'(lc_hw_debug_en_gated_raw)),
    .lc_en_o(lc_hw_debug_en_gated)
  );

  lc_ctrl_pkg::lc_tx_t [PmEnLastPos-1:0] pinmux_hw_debug_en;
  prim_lc_sync #(
    .NumCopies(int'(PmEnLastPos))
  ) u_pm_en_sync (
    .clk_i,
    .rst_ni,
    .lc_en_i(pinmux_hw_debug_en_i),
    .lc_en_o(pinmux_hw_debug_en)
  );

  dm::dmi_req_t  dmi_req;
  dm::dmi_resp_t dmi_rsp;
  logic dmi_req_valid, dmi_req_ready;
  logic dmi_rsp_valid, dmi_rsp_ready;
  logic dmi_rst_n;

  logic dmi_en;
  // SEC_CM: DM_EN.CTRL.LC_GATED
  assign dmi_en = lc_tx_test_true_strict(pinmux_hw_debug_en[PmEnDmiReq]);

  ////////////////////////
  // NDM Reset Tracking //
  ////////////////////////

  logic reset_req_en;
  logic ndmreset_req, ndmreset_ack;
  logic ndmreset_req_qual;
  // SEC_CM: DM_EN.CTRL.LC_GATED
  assign reset_req_en = lc_tx_test_true_strict(lc_hw_debug_en_gated[LcEnResetReq]);
  assign ndmreset_req_o = ndmreset_req_qual & reset_req_en;

  // Sample the processor reset to detect lc reset assertion.
  logic lc_rst_asserted_async;
  prim_flop_2sync #(
    .Width(1),
    .ResetValue(1) // Resets to 1 to indicate assertion.
  ) u_prim_flop_2sync_lc_rst_assert (
    .clk_i, // Use RV_DM clock
    .rst_ni(rst_lc_ni), // Use LC reset here that resets the entire system except the RV_DM.
    .d_i(1'b0), // Set to 0 to indicate deassertion.
    .q_o(lc_rst_asserted_async)
  );

  // Note that the output of the above flops can be metastable at reset assertion, since the reset
  // signal is coming from a different clock domain and has not been synchronized with clk_i.
  logic lc_rst_asserted;
  prim_flop_2sync #(
    .Width(1)
  ) u_prim_flop_2sync_lc_rst_sync (
    .clk_i,
    .rst_ni,
    .d_i(lc_rst_asserted_async),
    .q_o(lc_rst_asserted)
  );

  // The acknowledgement pulse sets the dmstatus.allhavereset / dmstatus.anyhavereset registers in
  // RV_DM. It should only be asserted once an NDM reset request has been fully completed.
  logic ndmreset_pending_q;
  logic lc_rst_pending_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin : p_ndm_reset
    if (!rst_ni) begin
      ndmreset_pending_q <= 1'b0;
      lc_rst_pending_q <= 1'b0;
    end else begin
      // Only set this if there was no previous pending NDM request.
      if (ndmreset_req && !ndmreset_pending_q) begin
        ndmreset_pending_q <= 1'b1;
      end else if (ndmreset_ack && ndmreset_pending_q) begin
        ndmreset_pending_q <= 1'b0;
      end
      // We only track lc resets that are asserted during an active ndm reset request..
      if (ndmreset_pending_q && lc_rst_asserted) begin
        lc_rst_pending_q <= 1'b1;
      end else if (ndmreset_ack && lc_rst_pending_q) begin
        lc_rst_pending_q <= 1'b0;
      end
    end
  end

  // In order to ACK the following conditions must be met
  // 1) an NDM reset request was asserted and is pending
  // 2) a lc reset was asserted after the NDM reset request
  // 3) the NDM reset request was deasserted
  // 4) the NDM lc request was deasserted
  // 5) the debug module has been ungated for operation (depending on LC state, OTP config and CSR)
  assign ndmreset_ack = ndmreset_pending_q &&
                        lc_rst_pending_q &&
                        !ndmreset_req &&
                        !lc_rst_asserted &&
                        reset_req_en;

  /////////////////////////////////////////
  // System Bus Access Port (TL-UL Host) //
  /////////////////////////////////////////

  logic                   host_req;
  logic   [BusWidth-1:0]  host_add;
  logic                   host_we;
  logic   [BusWidth-1:0]  host_wdata;
  logic [BusWidth/8-1:0]  host_be;
  logic                   host_gnt;
  logic                   host_r_valid;
  logic   [BusWidth-1:0]  host_r_rdata;
  logic                   host_r_err;
  logic                   host_r_other_err;

  // SEC_CM: DM_EN.CTRL.LC_GATED
  // SEC_CM: SBA_TL_LC_GATE.FSM.SPARSE
  tlul_pkg::tl_h2d_t  sba_tl_h_o_int;
  tlul_pkg::tl_d2h_t  sba_tl_h_i_int;
  tlul_lc_gate #(
    .NumGatesPerDirection(2)
  ) u_tlul_lc_gate_sba (
    .clk_i,
    .rst_ni,
    .tl_h2d_i(sba_tl_h_o_int),
    .tl_d2h_o(sba_tl_h_i_int),
    .tl_h2d_o(sba_tl_h_o),
    .tl_d2h_i(sba_tl_h_i),
    .lc_en_i (lc_hw_debug_en_gated[LcEnSba]),
    .err_o   (sba_gate_intg_error),
    .flush_req_i('0),
    .flush_ack_o(),
    .resp_pending_o()
  );

  tlul_adapter_host #(
    .MAX_REQS(1),
    .EnableDataIntgGen(1),
    .EnableRspDataIntgCheck(1)
  ) tl_adapter_host_sba (
    .clk_i,
    .rst_ni,
    .req_i        (host_req),
    .instr_type_i (prim_mubi_pkg::MuBi4False),
    .gnt_o        (host_gnt),
    .addr_i       (host_add),
    .we_i         (host_we),
    .wdata_i      (host_wdata),
    .wdata_intg_i ('0),
    .be_i         (host_be),
    .valid_o      (host_r_valid),
    .rdata_o      (host_r_rdata),
    .rdata_intg_o (),
    .err_o        (host_r_err),
    // Note: This bus integrity error is not connected to the alert due to a few reasons:
    // 1) the SBA module is not active in production life cycle states.
    // 2) there is value in being able to accept incoming transactions with integrity
    //    errors during test / debug life cycle states so that the system can be debugged
    //    without triggering alerts.
    // 3) the error condition is hooked up to an error CSR that can be read out by the debugger
    //    via JTAG so that bus integrity errors can be told appart from regular bus errors.
    .intg_err_o   (host_r_other_err),
    .tl_o         (sba_tl_h_o_int),
    .tl_i         (sba_tl_h_i_int)
  );

  //////////////////////////////////////
  // Debug Memory Port (TL-UL Device) //
  //////////////////////////////////////

  logic                         device_req;
  logic                         device_we;
  logic                         device_re;
  logic [BusWidth/8-1:0]        device_be;
  logic   [BusWidth-1:0]        device_wdata;
  logic   [BusWidth-1:0]        device_rdata;
  logic                         device_err;

  logic [BusWidth-1:0]          device_addr_aligned;
  logic [MemAw-1:0]             device_addr;

  assign device_addr_aligned = BusWidth'(device_addr);

  logic [NrHarts-1:0] debug_req_en;
  logic [NrHarts-1:0] debug_req;
  for (genvar i = 0; i < NrHarts; i++) begin : gen_debug_req_hart
    // SEC_CM: DM_EN.CTRL.LC_GATED
    assign debug_req_en[i] = lc_tx_test_true_strict(lc_hw_debug_en_gated[LcEnDebugReq + i]);
  end
  assign debug_req_o = debug_req & debug_req_en;

  // Gating of JTAG signals
  jtag_pkg::jtag_req_t jtag_in_int;
  jtag_pkg::jtag_rsp_t jtag_out_int;

  assign jtag_in_int = (lc_tx_test_true_strict(pinmux_hw_debug_en[PmEnJtagIn]))  ? jtag_i : '0;
  assign jtag_o = (lc_tx_test_true_strict(pinmux_hw_debug_en[PmEnJtagOut])) ? jtag_out_int : '0;

  // Bound-in DPI module replaces the TAP

  logic tck_muxed;
  logic trst_n_muxed;
  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_prim_clock_mux2 (
    .clk0_i(jtag_in_int.tck),
    .clk1_i(clk_i),
    .sel_i (testmode),
    .clk_o (tck_muxed)
  );

  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_prim_rst_n_mux2 (
    .clk0_i(jtag_in_int.trst_n),
    .clk1_i(scan_rst_ni),
    .sel_i (testmode),
    .clk_o (trst_n_muxed)
  );

  // JTAG TAP
  dmi_jtag #(
    .IdcodeValue    (IdcodeValue),
    .NumDmiWordAbits(7)
  ) dap (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .testmode_i       (testmode),
    .test_rst_ni      (scan_rst_ni),

    .dmi_rst_no       (dmi_rst_n),
    .dmi_req_o        (dmi_req),
    .dmi_req_valid_o  (dmi_req_valid),
    .dmi_req_ready_i  (dmi_req_ready & dmi_en),

    .dmi_resp_i       (dmi_rsp      ),
    .dmi_resp_ready_o (dmi_rsp_ready),
    .dmi_resp_valid_i (dmi_rsp_valid & dmi_en),

    //JTAG
    .tck_i            (tck_muxed),
    .tms_i            (jtag_in_int.tms),
    .trst_ni          (trst_n_muxed),
    .td_i             (jtag_in_int.tdi),
    .td_o             (jtag_out_int.tdo),
    .tdo_oe_o         (jtag_out_int.tdo_oe)
  );

  // SEC_CM: DM_EN.CTRL.LC_GATED
  // SEC_CM: MEM_TL_LC_GATE.FSM.SPARSE
  tlul_pkg::tl_h2d_t mem_tl_win_h2d_gated;
  tlul_pkg::tl_d2h_t mem_tl_win_d2h_gated;
  tlul_lc_gate #(
    .NumGatesPerDirection(2)
  ) u_tlul_lc_gate_rom (
    .clk_i,
    .rst_ni,
    .tl_h2d_i(mem_tl_win_h2d),
    .tl_d2h_o(mem_tl_win_d2h),
    .tl_h2d_o(mem_tl_win_h2d_gated),
    .tl_d2h_i(mem_tl_win_d2h_gated),
    .flush_req_i(ndmreset_req),
    .flush_ack_o(ndmreset_req_qual),
    .resp_pending_o(),
    .lc_en_i (lc_hw_debug_en_gated[LcEnRom]),
    .err_o   (rom_gate_intg_error)
  );

  prim_mubi_pkg::mubi4_t en_ifetch;
  // SEC_CM: DM_EN.CTRL.LC_GATED, EXEC.CTRL.MUBI
  assign en_ifetch = mubi4_bool_to_mubi(lc_tx_test_true_strict(lc_hw_debug_en_gated[LcEnFetch]));

  tlul_adapter_reg #(
    .CmdIntgCheck     (1),
    .EnableRspIntgGen (1),
    .EnableDataIntgGen(1),
    .RegAw            (MemAw),
    .RegDw            (BusWidth),
    .AccessLatency    (1)
  ) i_tlul_adapter_reg (
    .clk_i,
    .rst_ni,
    .tl_i        (mem_tl_win_h2d_gated),
    .tl_o        (mem_tl_win_d2h_gated),
    // SEC_CM: EXEC.CTRL.MUBI
    .en_ifetch_i (en_ifetch),
    // SEC_CM: BUS.INTEGRITY
    .intg_error_o(rom_intg_error),
    .re_o        (device_re),
    .we_o        (device_we),
    .addr_o      (device_addr),
    .wdata_o     (device_wdata),
    .be_o        (device_be),
    .busy_i      (1'b0),
    .rdata_i     (device_rdata),
    .error_i     (device_err)
  );

  assign device_req = device_we || device_re;

  ///////////////////////////
  // Debug Module Instance //
  ///////////////////////////

  dm_top #(
    .NrHarts        (NrHarts),
    .BusWidth       (BusWidth),
    .SelectableHarts(SelectableHarts),
    // The debug module provides a simplified ROM for systems that map the debug ROM to offset 0x0
    // on the system bus. In that case, only one scratch register has to be implemented in the core.
    // However, we require that the DM can be placed at arbitrary offsets in the system, which
    // requires the generalized debug ROM implementation and two scratch registers. We hence set
    // this parameter to a non-zero value (inside dm_mem, this just feeds into a comparison with 0).
    .DmBaseAddress  (1)
  ) u_dm_top (
    .clk_i,
    .rst_ni,
    .next_dm_addr_i,
    .testmode_i            (testmode              ),
    .ndmreset_o            (ndmreset_req          ),
    .ndmreset_ack_i        (ndmreset_ack          ),
    .dmactive_o,
    .debug_req_o           (debug_req             ),
    .unavailable_i,
    .hartinfo_i            (hartinfo              ),
    .slave_req_i           (device_req            ),
    .slave_we_i            (device_we             ),
    .slave_addr_i          (device_addr_aligned   ),
    .slave_be_i            (device_be             ),
    .slave_wdata_i         (device_wdata          ),
    .slave_rdata_o         (device_rdata          ),
    .slave_err_o           (device_err            ),
    .master_req_o          (host_req              ),
    .master_add_o          (host_add              ),
    .master_we_o           (host_we               ),
    .master_wdata_o        (host_wdata            ),
    .master_be_o           (host_be               ),
    .master_gnt_i          (host_gnt              ),
    .master_r_valid_i      (host_r_valid          ),
    .master_r_err_i        (host_r_err            ),
    .master_r_other_err_i  (host_r_other_err      ),
    .master_r_rdata_i      (host_r_rdata          ),
    .dmi_rst_ni            (dmi_rst_n             ),
    .dmi_req_valid_i       (dmi_req_valid & dmi_en),
    .dmi_req_ready_o       (dmi_req_ready         ),
    .dmi_req_i             (dmi_req               ),
    .dmi_resp_valid_o      (dmi_rsp_valid         ),
    .dmi_resp_ready_i      (dmi_rsp_ready & dmi_en),
    .dmi_resp_o            (dmi_rsp               )
  );

  ////////////////
  // Assertions //
  ////////////////

  `ASSERT_KNOWN(TlRegsDValidKnown_A, regs_tl_d_o.d_valid)
  `ASSERT_KNOWN(TlRegsAReadyKnown_A, regs_tl_d_o.a_ready)

  `ASSERT_KNOWN(TlMemDValidKnown_A, mem_tl_d_o.d_valid)
  `ASSERT_KNOWN(TlMemAReadyKnown_A, mem_tl_d_o.a_ready)

  `ASSERT_KNOWN(TlSbaAValidKnown_A, sba_tl_h_o.a_valid)
  `ASSERT_KNOWN(TlSbaDReadyKnown_A, sba_tl_h_o.d_ready)

  `ASSERT_KNOWN(NdmresetOKnown_A, ndmreset_req_o)
  `ASSERT_KNOWN(DmactiveOKnown_A, dmactive_o)
  `ASSERT_KNOWN(DebugReqOKnown_A, debug_req_o)

  // JTAG TDO is driven by an inverted TCK in dmi_jtag_tap.sv
  `ASSERT_KNOWN(JtagRspOTdoKnown_A, jtag_o.tdo, !jtag_i.tck, !jtag_i.trst_n)
  `ASSERT_KNOWN(JtagRspOTdoOeKnown_A, jtag_o.tdo_oe, !jtag_i.tck, !jtag_i.trst_n)

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(SbaTlLcGateFsm_A,
    u_tlul_lc_gate_sba.u_state_regs, alert_tx_o[0])

  `ASSERT_PRIM_FSM_ERROR_TRIGGER_ALERT(RomTlLcGateFsm_A,
    u_tlul_lc_gate_rom.u_state_regs, alert_tx_o[0])

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg_regs, alert_tx_o[0])
endmodule

module rv_plic 
    import rv_plic_reg_pkg::*;
#(
  // Param list
  parameter int NumSrc = 186,
  parameter int NumTarget = 1,
  parameter int PrioWidth = 2,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 27,
  
  parameter logic [NumAlerts-1:0] AlertAsyncOn  = {NumAlerts{1'b1}},
  // OpenTitan IP standardizes on level triggered interrupts,
  // hence LevelEdgeTrig is set to all-zeroes by default.
  // Note that in case of edge-triggered interrupts, CDC handling is not
  // fully implemented yet (this would require instantiating pulse syncs
  // and routing the source clocks / resets to the PLIC).
  parameter logic [NumSrc-1:0]    LevelEdgeTrig = '0, // 0: level, 1: edge
  // derived parameter
  localparam int SRCW    = $clog2(NumSrc)
) (
  input     clk_i,
  input     rst_ni,

  // Bus Interface (device)
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Interrupt Sources
  input  [NumSrc-1:0] intr_src_i,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // Interrupt notification to targets
  output [NumTarget-1:0] irq_o,
  output [SRCW-1:0]      irq_id_o [NumTarget],

  output logic [NumTarget-1:0] msip_o
);
// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//



module rv_timer import rv_timer_reg_pkg::*;
#(
  // Param list
  parameter int N_HARTS = 1,
  parameter int N_TIMERS = 1,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 9,
  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}}
) (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  output logic intr_timer_expired_hart0_timer0_o
);

  rv_timer_reg2hw_t reg2hw;
  rv_timer_hw2reg_t hw2reg;

  logic [N_HARTS-1:0] active;

  logic [11:0] prescaler [N_HARTS];
  logic [7:0]  step      [N_HARTS];

  logic [N_HARTS-1:0] tick;

  logic [63:0] mtime_d  [N_HARTS];
  logic [63:0] mtime    [N_HARTS];
  logic [63:0] mtimecmp [N_HARTS][N_TIMERS]; // Only [harts][0] is connected to mtimecmp CSRs
  logic        mtimecmp_update [N_HARTS][N_TIMERS];

  logic [N_HARTS*N_TIMERS-1:0] intr_timer_set;
  logic [N_HARTS*N_TIMERS-1:0] intr_timer_en;
  logic [N_HARTS*N_TIMERS-1:0] intr_timer_test_q;
  logic [N_HARTS-1:0]          intr_timer_test_qe;
  logic [N_HARTS*N_TIMERS-1:0] intr_timer_state_q;
  logic [N_HARTS-1:0]          intr_timer_state_de;
  logic [N_HARTS*N_TIMERS-1:0] intr_timer_state_d;

  logic [N_HARTS*N_TIMERS-1:0] intr_out;

  /////////////////////////////////////////////////
  // Connecting register interface to the signal //
  /////////////////////////////////////////////////

  // Once reggen supports nested multireg, the following can be automated. For the moment, it must
  // be connected manually.
  assign active[0]  = reg2hw.ctrl[0].q;
  assign prescaler = '{reg2hw.cfg0.prescale.q};
  assign step      = '{reg2hw.cfg0.step.q};

  assign hw2reg.timer_v_upper0.de = tick[0];
  assign hw2reg.timer_v_lower0.de = tick[0];
  assign hw2reg.timer_v_upper0.d = mtime_d[0][63:32];
  assign hw2reg.timer_v_lower0.d = mtime_d[0][31: 0];
  assign mtime[0] = {reg2hw.timer_v_upper0.q, reg2hw.timer_v_lower0.q};
  assign mtimecmp = '{'{{reg2hw.compare_upper0_0.q,reg2hw.compare_lower0_0.q}}};
  assign mtimecmp_update[0][0] = reg2hw.compare_upper0_0.qe | reg2hw.compare_lower0_0.qe;

  assign intr_timer_expired_hart0_timer0_o = intr_out[0];
  assign intr_timer_en            = reg2hw.intr_enable0[0].q;
  assign intr_timer_state_q       = reg2hw.intr_state0[0].q;
  assign intr_timer_test_q        = reg2hw.intr_test0[0].q;
  assign intr_timer_test_qe       = reg2hw.intr_test0[0].qe;
  assign hw2reg.intr_state0[0].de = intr_timer_state_de | mtimecmp_update[0][0];
  assign hw2reg.intr_state0[0].d  = intr_timer_state_d & ~mtimecmp_update[0][0];


  for (genvar h = 0 ; h < N_HARTS ; h++) begin : gen_harts
    prim_intr_hw #(
      .Width(N_TIMERS)
    ) u_intr_hw (
      .clk_i,
      .rst_ni,
      .event_intr_i           (intr_timer_set),

      .reg2hw_intr_enable_q_i (intr_timer_en[h*N_TIMERS+:N_TIMERS]),
      .reg2hw_intr_test_q_i   (intr_timer_test_q[h*N_TIMERS+:N_TIMERS]),
      .reg2hw_intr_test_qe_i  (intr_timer_test_qe[h]),
      .reg2hw_intr_state_q_i  (intr_timer_state_q[h*N_TIMERS+:N_TIMERS]),
      .hw2reg_intr_state_de_o (intr_timer_state_de),
      .hw2reg_intr_state_d_o  (intr_timer_state_d[h*N_TIMERS+:N_TIMERS]),

      .intr_o                 (intr_out[h*N_TIMERS+:N_TIMERS])
    );

    timer_core #(
      .N (N_TIMERS)
    ) u_core (
      .clk_i,
      .rst_ni,

      .active    (active[h]),
      .prescaler (prescaler[h]),
      .step      (step[h]),

      .tick      (tick[h]),

      .mtime_d   (mtime_d[h]),
      .mtime     (mtime[h]),
      .mtimecmp  (mtimecmp[h]),

      .intr      (intr_timer_set[h*N_TIMERS+:N_TIMERS])
    );
  end : gen_harts

  // Register module
  logic [NumAlerts-1:0] alert_test, alerts;
  rv_timer_reg_top u_reg (
    .clk_i,
    .rst_ni,

    .tl_i,
    .tl_o,

    .reg2hw,
    .hw2reg,

    // SEC_CM: BUS.INTEGRITY
    .intg_err_o (alerts[0])
  );

  // Alerts
  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  ////////////////
  // Assertions //
  ////////////////
  `ASSERT_KNOWN(TlODValidKnown, tl_o.d_valid)
  `ASSERT_KNOWN(TlOAReadyKnown, tl_o.a_ready)
  `ASSERT_KNOWN(AlertsKnown_A, alert_tx_o)
  `ASSERT_KNOWN(IntrTimerExpiredHart0Timer0Known, intr_timer_expired_hart0_timer0_o)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Serial Peripheral Interface (SPI) Device module.
//


module spi_device
  import spi_device_reg_pkg::NumAlerts;
  import spi_device_reg_pkg::SPI_DEVICE_EGRESS_BUFFER_IDX;
  import spi_device_reg_pkg::SPI_DEVICE_INGRESS_BUFFER_IDX;
#(
  // Param list
  parameter int unsigned SramDepth = 1024,
  parameter int unsigned SramEgressDepth = 848,
  parameter int unsigned SramIngressDepth = 112,
  parameter int unsigned SramReadBufferOffset = 0,
  parameter int unsigned SramReadBufferDepth = 512,
  parameter int unsigned SramMailboxOffset = 512,
  parameter int unsigned SramMailboxDepth = 256,
  parameter int unsigned SramSfdpOffset = 768,
  parameter int unsigned SramSfdpDepth = 64,
  parameter int unsigned SramTpmRdFifoOffset = 832,
  parameter int unsigned SramTpmRdFifoDepth = 16,
  parameter int unsigned SramPayloadOffset = 0,
  parameter int unsigned SramPayloadDepth = 64,
  parameter int unsigned SramCmdFifoOffset = 64,
  parameter int unsigned SramCmdFifoDepth = 16,
  parameter int unsigned SramAddrFifoOffset = 80,
  parameter int unsigned SramAddrFifoDepth = 16,
  parameter int unsigned SramTpmWrFifoOffset = 96,
  parameter int unsigned SramTpmWrFifoDepth = 16,
  parameter int unsigned NumCmdInfo = 24,
  parameter int unsigned NumLocality = 5,
  parameter int unsigned TpmRdFifoPtrW = 5,
  parameter int unsigned TpmRdFifoWidth = 32,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 13,
  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}},
  parameter spi_device_pkg::sram_type_e SramType = spi_device_pkg::DefaultSramType
) (
  input clk_i,
  input rst_ni,

  // Register interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // SPI Interface
  input              cio_sck_i,
  input              cio_csb_i,
  output logic [3:0] cio_sd_o,
  output logic [3:0] cio_sd_en_o,
  input        [3:0] cio_sd_i,

  input              cio_tpm_csb_i,

  // Passthrough interface
  output spi_device_pkg::passthrough_req_t passthrough_o,
  input  spi_device_pkg::passthrough_rsp_t passthrough_i,

  // Interrupts
  // INTR: Flash mode
  output logic intr_upload_cmdfifo_not_empty_o,
  output logic intr_upload_payload_not_empty_o,
  output logic intr_upload_payload_overflow_o,
  output logic intr_readbuf_watermark_o,
  output logic intr_readbuf_flip_o,

  // INTR: TPM mode
  output logic intr_tpm_header_not_empty_o, // TPM Command/Address buffer
  output logic intr_tpm_rdfifo_cmd_end_o,
  output logic intr_tpm_rdfifo_drop_o,

  // Memory configuration
  input prim_ram_2p_pkg::ram_2p_cfg_t ram_cfg_i,

  // External clock sensor
  output logic sck_monitor_o,

  // DFT related controls
  input mbist_en_i,
  input scan_clk_i,
  input scan_rst_ni,
  input prim_mubi_pkg::mubi4_t scanmode_i
);

  import spi_device_pkg::*;

  localparam int unsigned ReadBufferDepth = spi_device_pkg::SramMsgDepth;
  localparam int unsigned BufferAw        = $clog2(ReadBufferDepth);

  localparam int unsigned TpmRdFifoWidth  = spi_device_reg_pkg::TpmRdFifoWidth;

  // Derived parameters

  logic clk_spi_in, clk_spi_in_muxed, clk_spi_in_buf;   // clock for latch SDI
  logic clk_spi_out, clk_spi_out_muxed, clk_spi_out_buf; // clock for driving SDO
  logic clk_csb, clk_csb_muxed; // CSb as a clock source to latch BUSY

  spi_device_reg_pkg::spi_device_reg2hw_t reg2hw;
  spi_device_reg_pkg::spi_device_hw2reg_t hw2reg;

  tlul_pkg::tl_h2d_t tl_sram_h2d[2];
  tlul_pkg::tl_d2h_t tl_sram_d2h[2];
  tlul_pkg::tl_h2d_t tl_sram_egress_h2d;
  tlul_pkg::tl_d2h_t tl_sram_egress_d2h;
  tlul_pkg::tl_h2d_t tl_sram_ingress_h2d;
  tlul_pkg::tl_d2h_t tl_sram_ingress_d2h;

  // Dual-port SRAM Interface: Refer prim_ram_2p_wrapper.sv
  logic              mem_a_req;
  logic              mem_a_write;
  logic [SramAw-1:0] mem_a_addr;
  logic [SramDw-1:0] mem_a_wdata;
  logic [SramDw-1:0] mem_a_wmask;
  logic              mem_a_rvalid;
  logic [SramDw-1:0] mem_a_rdata;
  logic [1:0]        mem_a_rerror;

  sram_l2m_t mem_b_l2m;
  sram_m2l_t mem_b_m2l;
  logic              mem_b_req;
  logic              mem_b_write;
  logic [SramAw-1:0] mem_b_addr;
  logic [SramDw-1:0] mem_b_wdata;
  logic [SramDw-1:0] mem_b_wmask;
  logic              mem_b_rvalid;
  logic [SramDw-1:0] mem_b_rdata;
  logic [1:0]        mem_b_rerror;


  // Submoule SRAM Requests
  sram_l2m_t flash_sram_l2m;
  sram_m2l_t flash_sram_m2l;
  sram_l2m_t sub_sram_l2m [IoModeEnd];
  sram_m2l_t sub_sram_m2l [IoModeEnd];

  // Host return path mux
  logic [3:0] internal_sd, internal_sd_out, internal_sd_en, internal_sd_en_out;
  logic [3:0] passthrough_sd, passthrough_sd_en;

  // Upload related interfaces (SRAM, FIFOs)
  typedef enum int unsigned {
    SysSramFwEgress  = 0,
    SysSramFwIngress = 1,
    SysSramCmdFifo   = 2,
    SysSramAddrFifo  = 3,
    SysSramTpmRdFifo = 4,
    SysSramEnd       = 5
  } sys_sram_e;

  sram_l2m_t sys_sram_l2m [SysSramEnd]; // FW, CMDFIFO, ADDRFIFO
  sram_m2l_t sys_sram_m2l [SysSramEnd];

  // Arbiter among Upload CmdFifo/AddrFifo & FW access
  logic [SysSramEnd-1:0] sys_sram_req                ;
  logic [SysSramEnd-1:0] sys_sram_gnt                ;
  logic [1:0]            sys_sram_fw_gnt             ;
  logic [SramAw-1:0]     sys_sram_addr   [SysSramEnd];
  logic [SysSramEnd-1:0] sys_sram_write              ;
  logic [SramDw-1:0]     sys_sram_wdata  [SysSramEnd];
  logic [SramDw-1:0]     sys_sram_wmask  [SysSramEnd];
  logic [SysSramEnd-1:0] sys_sram_rvalid             ;
  logic [SramDw-1:0]     sys_sram_rdata  [SysSramEnd];
  logic [1:0]            sys_sram_rerror [SysSramEnd];


  logic        cmdfifo_rvalid, cmdfifo_rready;
  logic [15:0] cmdfifo_rdata;
  logic        cmdfifo_notempty;
  logic        cmdfifo_set_pulse;

  logic        addrfifo_rvalid, addrfifo_rready;
  logic [31:0] addrfifo_rdata;
  logic        addrfifo_notempty;

  logic payload_notempty;
  logic payload_overflow;

  localparam int unsigned CmdFifoPtrW = $clog2(SramCmdFifoDepth+1);
  localparam int unsigned AddrFifoPtrW = $clog2(SramAddrFifoDepth+1);

  localparam int unsigned PayloadByte = SramPayloadDepth * (SramDw/$bits(spi_byte_t));
  localparam int unsigned PayloadDepthW = $clog2(PayloadByte+1);
  localparam int unsigned PayloadIdxW   = $clog2(PayloadByte);

  logic [CmdFifoPtrW-1:0]    cmdfifo_depth;
  logic [AddrFifoPtrW-1:0]   addrfifo_depth;
  logic [PayloadDepthW-1:0]  payload_depth;
  logic [PayloadIdxW-1:0]    payload_start_idx;

  assign payload_notempty = payload_depth != '0;

  /////////////////////
  // Control signals //
  /////////////////////

  logic txorder; // TX bitstream order: 0(bit 7 to 0), 1(bit 0 to 7)
  logic rxorder; // RX bitstream order: 0(bit 7 to 0), 1(bit 0 to 7)

  logic sys_csb_syncd;

  spi_mode_e spi_mode;
  logic cfg_addr_4b_en;
  logic cmd_sync_addr_4b_en;

  // Address 3B/ 4B tracker related signals
  //
  // EN4B/ EX4B change internal status by HW. If SW is involved into the
  // process, the latency is long. As EN4B/ EX4B commands do not assert BUSY
  // bit, the host system issues next read commands without any delays. SW
  // process latency cannot meet the requirement.
  //
  // `spid_addr_4b` submodule processes the broadcasting signal
  // `cfg_addr_4b_en`. The command parser recognizes the commands and triggers
  // the `spid_addr_4b` submodule to change the internal status.
  //
  // The opcodes of the commands SW may configure via CMD_INFO_EN4B,
  // CMD_INFO_EX4B.
  logic cmd_en4b_pulse, cmd_ex4b_pulse;

  // SPI S2P signals
  // io_mode: Determine s2p/p2s behavior.
  // io_mode is changed at the negedge of SPI_CLK (based on the SPI protocol).
  // sub_iomode originates from the clk_spi_in domain, with flop values that
  // may have changed based on the input of SPI. The sub_iomode is selected
  // and sampled on the clk_spi_out domain.
  io_mode_e           io_mode, io_mode_outclk;
  io_mode_e           sub_iomode[IoModeEnd];
  logic               s2p_data_valid;
  spi_byte_t          s2p_data;

  logic        p2s_valid;
  spi_byte_t   p2s_data;
  logic        p2s_sent;

  logic        sub_p2s_valid[IoModeEnd];
  spi_byte_t   sub_p2s_data[IoModeEnd];
  logic        sub_p2s_sent[IoModeEnd];

  // Read commands related signals
  logic [31:0] readbuf_addr_sck;
  logic [31:0] readbuf_addr_busclk;

  // CMD interface
  sel_datapath_e cmd_dp_sel, cmd_only_dp_sel;

  // Mailbox in Passthrough needs to take SPI if readcmd hits mailbox address
  logic intercept_en, intercept_en_out;

  logic cfg_mailbox_en;
  logic [31:0] mailbox_addr;

  // Intercept
  typedef struct packed {
    logic status;
    logic jedec;
    logic sfdp;
    logic mbx;
  } intercept_t;
  intercept_t cfg_intercept_en;
  intercept_t intercept; // Assume signals

  // Threshold value of a buffer in bytes
  logic [BufferAw:0] readbuf_threshold;

  // Synchronous clear of read buffer tracking.
  logic readbuf_clr;

  // Passthrouth config signals
  logic [255:0] cmd_filter;

  logic [31:0] addr_swap_mask;
  logic [31:0] addr_swap_data;

  logic [31:0] payload_swap_mask;
  logic [31:0] payload_swap_data;

  // Additional 2-stage read pipeline configuration
  logic cmd_read_pipeline_sel;

  // Command Info structure
  cmd_info_t [NumTotalCmdInfo-1:0] cmd_info;
  // Broadcasted cmd_info. cmdparse compares the opcode up to CmdInfoReadCmdEnd
  // and latches the cmd_info and broadcast to submodules
  cmd_info_t                  cmd_info_broadcast;
  logic [CmdInfoIdxW-1:0]     cmd_info_idx_broadcast;
  // Combinatorial output of selected cmd_info, to be used with modules that
  // need the values before the 8th posedge of the command.
  cmd_info_t                  cmd_only_info_broadcast;
  logic [CmdInfoIdxW-1:0]     cmd_only_info_idx_broadcast;

  // Synchronization pulse indicating that the 8th bit of the command is
  // arriving. This is used to time the transfer of some data to/from the sys
  // domain.
  logic                       cmd_sync_pulse;

  // CSb edge detector in the system clock and SPI input clock
  // SYS clock assertion can be detected but no usage for the event yet.
  // SPI clock de-assertion cannot be detected as no SCK at the time is given.
  logic sys_csb_deasserted_pulse;

  // Important status bits for tracking in the upload module
  logic cmd_sync_status_busy;
  logic cmd_sync_status_wel;
  logic sck_status_busy;

  // Read Status input and broadcast
  logic sck_status_busy_set;       // set by HW (upload)
  logic csb_status_busy_broadcast; // from spid_status

  // WREN / WRDI HW signal
  logic sck_status_wr_set;
  logic sck_status_wr_clr;

  // Jedec ID
  jedec_cfg_t jedec_cfg;

  // Interrupts in Flash mode
  logic intr_upload_cmdfifo_not_empty, intr_upload_payload_not_empty;
  logic intr_upload_payload_overflow;
  logic intr_readbuf_watermark, intr_readbuf_flip;
  logic flash_sck_readbuf_watermark, flash_sck_readbuf_flip;

  // TPM ===============================================================
  // Interface
  logic tpm_mosi, tpm_miso, tpm_miso_en;
  assign tpm_mosi = cio_sd_i[0];

  // Return-by-HW registers
  logic [8*spi_device_reg_pkg::NumLocality-1:0] tpm_access;
  logic [31:0]                                  tpm_int_enable;
  logic [7:0]                                   tpm_int_vector;
  logic [31:0]                                  tpm_int_status;
  logic [31:0]                                  tpm_intf_capability;
  logic [31:0]                                  tpm_status;
  logic [31:0]                                  tpm_did_vid;
  logic [7:0]                                   tpm_rid;

  // Buffer and FIFO signals
  sram_l2m_t                  tpm_sram_l2m;
  sram_m2l_t                  tpm_sram_m2l;
  logic                       tpm_cmdaddr_rvalid, tpm_cmdaddr_rready;
  logic [31:0]                tpm_cmdaddr_rdata;
  logic                       tpm_rdfifo_wvalid, tpm_rdfifo_wready;
  logic [TpmRdFifoWidth-1:0]  tpm_rdfifo_wdata;
  logic                       tpm_event_rdfifo_cmd_end;
  logic                       tpm_event_rdfifo_drop;

  tpm_cap_t tpm_cap;

  // TPM CFG
  logic cfg_tpm_en, cfg_tpm_mode, cfg_tpm_hw_reg_dis;
  logic cfg_tpm_invalid_locality, cfg_tpm_reg_chk_dis;

  // TPM_STATUS
  logic tpm_status_cmdaddr_notempty;
  logic tpm_status_wrfifo_pending;
  logic tpm_status_wrfifo_release;
  logic tpm_status_rdfifo_aborted;

  // TPM ---------------------------------------------------------------

  /////////////////
  // CSb Buffers //
  /////////////////
  // Split the CSB into multiple explicit buffers. One for reset, two for each
  // clock domains.
  logic clk_csb_buf, rst_csb_buf, sys_csb, sck_csb;
  prim_buf #(
    .Width (4)
  ) u_csb_buf (
    .in_i  ({4{cio_csb_i}}),
    .out_o ({clk_csb_buf, rst_csb_buf, sys_csb, sck_csb})
  );

  // Split TPM CSB into explicit reset and data.
  logic rst_tpm_csb_buf, sys_tpm_csb_buf, sck_tpm_csb_buf;
  logic sys_tpm_csb_syncd; // synchronized prior to be connected to reg
  prim_buf #(
    .Width (3)
  ) u_tpm_csb_buf (
    .in_i  ({3{cio_tpm_csb_i}}),
    .out_o ({rst_tpm_csb_buf, sys_tpm_csb_buf, sck_tpm_csb_buf})
  );

  //////////////////////////////////////////////////////////////////////
  // Connect phase (between control signals above and register module //
  //////////////////////////////////////////////////////////////////////

  assign txorder = reg2hw.cfg.tx_order.q;
  assign rxorder = reg2hw.cfg.rx_order.q;

  // CSb : after 2stage synchronizer
  assign hw2reg.status.csb.d     = sys_csb_syncd;
  assign hw2reg.status.tpm_csb.d = sys_tpm_csb_syncd;

  assign spi_mode = spi_mode_e'(reg2hw.control.mode.q);

  prim_edge_detector #(
    .Width (2),
    .EnSync(1'b 0)
  ) u_intr_upload_edge (
    .clk_i,
    .rst_ni,

    .d_i               ({payload_notempty, payload_overflow}),
    .q_sync_o          (),
    .q_posedge_pulse_o ({intr_upload_payload_not_empty,
                         intr_upload_payload_overflow}),
    .q_negedge_pulse_o ()
  );
  assign intr_upload_cmdfifo_not_empty = cmdfifo_set_pulse;

  prim_intr_hw #(.Width(1)) u_intr_cmdfifo_not_empty (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_upload_cmdfifo_not_empty                ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.upload_cmdfifo_not_empty.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.upload_cmdfifo_not_empty.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.upload_cmdfifo_not_empty.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.upload_cmdfifo_not_empty.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.upload_cmdfifo_not_empty.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.upload_cmdfifo_not_empty.de),
    .intr_o                 (intr_upload_cmdfifo_not_empty_o              )
  );

  prim_intr_hw #(.Width(1)) u_intr_payload_not_empty (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_upload_payload_not_empty                ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.upload_payload_not_empty.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.upload_payload_not_empty.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.upload_payload_not_empty.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.upload_payload_not_empty.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.upload_payload_not_empty.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.upload_payload_not_empty.de),
    .intr_o                 (intr_upload_payload_not_empty_o              )
  );

  prim_intr_hw #(.Width(1)) u_intr_payload_overflow (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_upload_payload_overflow                ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.upload_payload_overflow.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.upload_payload_overflow.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.upload_payload_overflow.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.upload_payload_overflow.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.upload_payload_overflow.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.upload_payload_overflow.de),
    .intr_o                 (intr_upload_payload_overflow_o              )
  );


  prim_pulse_sync u_flash_readbuf_watermark_pulse_sync (
    .clk_src_i   (clk_spi_in_buf             ),
    .rst_src_ni  (rst_ni                     ),
    .src_pulse_i (flash_sck_readbuf_watermark),
    .clk_dst_i   (clk_i                      ),
    .rst_dst_ni  (rst_ni                     ),
    .dst_pulse_o (intr_readbuf_watermark     )
  );
  prim_intr_hw #(.Width(1)) u_intr_readbuf_watermark (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_readbuf_watermark                ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.readbuf_watermark.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.readbuf_watermark.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.readbuf_watermark.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.readbuf_watermark.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.readbuf_watermark.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.readbuf_watermark.de),
    .intr_o                 (intr_readbuf_watermark_o              )
  );

  prim_pulse_sync u_flash_readbuf_flip_pulse_sync (
    .clk_src_i   (clk_spi_in_buf        ),
    .rst_src_ni  (rst_ni                ),
    .src_pulse_i (flash_sck_readbuf_flip),
    .clk_dst_i   (clk_i                 ),
    .rst_dst_ni  (rst_ni                ),
    .dst_pulse_o (intr_readbuf_flip     )
  );
  prim_intr_hw #(.Width(1)) u_intr_readbuf_flip (
    .clk_i,
    .rst_ni,
    .event_intr_i           (intr_readbuf_flip                ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.readbuf_flip.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.readbuf_flip.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.readbuf_flip.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.readbuf_flip.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.readbuf_flip.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.readbuf_flip.de),
    .intr_o                 (intr_readbuf_flip_o              )
  );

  prim_intr_hw #(
    .Width (1       ),
    .IntrT ("Status")
  ) u_intr_tpm_cmdaddr_notempty (
    .clk_i,
    .rst_ni,
    .event_intr_i           (tpm_status_cmdaddr_notempty              ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.tpm_header_not_empty.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.tpm_header_not_empty.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.tpm_header_not_empty.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.tpm_header_not_empty.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.tpm_header_not_empty.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.tpm_header_not_empty.de),
    .intr_o                 (intr_tpm_header_not_empty_o              )
  );

  prim_intr_hw #(
    .Width (1      ),
    .IntrT ("Event")
  ) u_intr_tpm_rdfifo_cmd_end (
    .clk_i,
    .rst_ni,
    .event_intr_i           (tpm_event_rdfifo_cmd_end               ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.tpm_rdfifo_cmd_end.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.tpm_rdfifo_cmd_end.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.tpm_rdfifo_cmd_end.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.tpm_rdfifo_cmd_end.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.tpm_rdfifo_cmd_end.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.tpm_rdfifo_cmd_end.de),
    .intr_o                 (intr_tpm_rdfifo_cmd_end_o              )
  );

  prim_intr_hw #(
    .Width (1      ),
    .IntrT ("Event")
  ) u_intr_tpm_rdfifo_drop (
    .clk_i,
    .rst_ni,
    .event_intr_i           (tpm_event_rdfifo_drop               ),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.tpm_rdfifo_drop.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.tpm_rdfifo_drop.q  ),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.tpm_rdfifo_drop.qe ),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.tpm_rdfifo_drop.q ),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.tpm_rdfifo_drop.d ),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.tpm_rdfifo_drop.de),
    .intr_o                 (intr_tpm_rdfifo_drop_o              )
  );
  // SPI Flash commands registers

  assign cfg_intercept_en = '{
    status:  reg2hw.intercept_en.status.q,
    jedec:   reg2hw.intercept_en.jedec.q,
    sfdp:    reg2hw.intercept_en.sfdp.q,
    mbx:     reg2hw.intercept_en.mbx.q
  };
  logic unused_cfg_intercept_en;
  assign unused_cfg_intercept_en = ^cfg_intercept_en;

  assign hw2reg.last_read_addr.d = readbuf_addr_busclk;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      readbuf_addr_busclk <= '0;
    end else if (sys_csb_deasserted_pulse) begin
      readbuf_addr_busclk <= readbuf_addr_sck;
    end
  end

  // Jedec ID
  assign jedec_cfg = '{ num_cc:    reg2hw.jedec_cc.num_cc.q,
                        cc:        reg2hw.jedec_cc.cc.q,
                        jedec_id:  reg2hw.jedec_id.mf.q,
                        device_id: reg2hw.jedec_id.id.q
                      };

  assign readbuf_threshold = reg2hw.read_threshold.q[BufferAw:0];
  assign readbuf_clr = reg2hw.control.flash_read_buffer_clr.q;
  assign hw2reg.control.flash_read_buffer_clr.d  = '0;
  assign hw2reg.control.flash_read_buffer_clr.de = 1'b1;

  localparam int unsigned MailboxAw = $clog2(SramMailboxDepth*SramDw/8);
  assign cfg_mailbox_en = reg2hw.cfg.mailbox_en.q;
  assign mailbox_addr   = { reg2hw.mailbox_addr.q[31:MailboxAw],
                            {MailboxAw{1'b0}}
                          };
  logic unused_mailbox_addr;
  assign unused_mailbox_addr = ^reg2hw.mailbox_addr.q[MailboxAw-1:0];

  // Passthrough config: value shall be stable while SPI transaction is active
  //assign cmd_filter = reg2hw.cmd_filter.q;
  always_comb begin
    for (int unsigned i = 0 ; i < 256 ; i++) begin
      cmd_filter[i] = reg2hw.cmd_filter[i].q;
    end
  end

  assign addr_swap_mask = reg2hw.addr_swap_mask.q;
  assign addr_swap_data = reg2hw.addr_swap_data.q;

  // payload_swap_mask and _data are big-endian to calculate easily.
  assign payload_swap_mask = {<<8{reg2hw.payload_swap_mask.q}};
  assign payload_swap_data = {<<8{reg2hw.payload_swap_data.q}};

  // Connect command info
  always_comb begin
    for (int unsigned i = 0 ; i < spi_device_reg_pkg::NumCmdInfo ; i++) begin
      cmd_info[i] = '{
        valid:                 reg2hw.cmd_info[i].valid.q,
        opcode:                reg2hw.cmd_info[i].opcode.q,
        addr_mode:             addr_mode_e'(reg2hw.cmd_info[i].addr_mode.q),
        addr_swap_en:          reg2hw.cmd_info[i].addr_swap_en.q,
        mbyte_en:              reg2hw.cmd_info[i].mbyte_en.q,
        dummy_en:              reg2hw.cmd_info[i].dummy_en.q,
        dummy_size:            reg2hw.cmd_info[i].dummy_size.q,
        payload_en:            reg2hw.cmd_info[i].payload_en.q,
        payload_dir:           payload_dir_e'(reg2hw.cmd_info[i].payload_dir.q),
        payload_swap_en:       reg2hw.cmd_info[i].payload_swap_en.q,
        read_pipeline_mode:    read_pipeline_mode_e'(reg2hw.cmd_info[i].read_pipeline_mode.q),
        upload:                reg2hw.cmd_info[i].upload.q,
        busy:                  reg2hw.cmd_info[i].busy.q
      };
    end

    // Manual addition to cmd_info list
    // Default Input mode
    for (int unsigned i = CmdInfoReserveEnd + 1; i < NumTotalCmdInfo; i++) begin
      cmd_info[i] = CmdInfoInput;
    end

    // Hand crafted command information slots
    cmd_info[CmdInfoEn4B].valid  = reg2hw.cmd_info_en4b.valid.q;
    cmd_info[CmdInfoEn4B].opcode = reg2hw.cmd_info_en4b.opcode.q;

    cmd_info[CmdInfoEx4B].valid  = reg2hw.cmd_info_ex4b.valid.q;
    cmd_info[CmdInfoEx4B].opcode = reg2hw.cmd_info_ex4b.opcode.q;

    cmd_info[CmdInfoWrEn].valid  = reg2hw.cmd_info_wren.valid.q;
    cmd_info[CmdInfoWrEn].opcode = reg2hw.cmd_info_wren.opcode.q;

    cmd_info[CmdInfoWrDi].valid  = reg2hw.cmd_info_wrdi.valid.q;
    cmd_info[CmdInfoWrDi].opcode = reg2hw.cmd_info_wrdi.opcode.q;

  end

  ///////////////////////////
  // Clock & reset control //
  ///////////////////////////
  logic sck_n;
  logic rst_spi_n;
  prim_mubi_pkg::mubi4_t [ScanModeUseLast-1:0] scanmode;

  prim_mubi4_sync #(
    .NumCopies(int'(ScanModeUseLast)),
    .AsyncOn(0) // clock/reset below is only used for SVAs.
  ) u_scanmode_sync  (
    .clk_i,
    .rst_ni,
    .mubi_i(scanmode_i),
    .mubi_o(scanmode)
  );

  prim_clock_inv #(
    .NoFpgaBufG(1'b1)
  ) u_clk_spi (
    .clk_i(cio_sck_i),
    .clk_no(sck_n),
    .scanmode_i(prim_mubi_pkg::mubi4_test_true_strict(scanmode[ClkInvSel]))
  );

  assign sck_monitor_o = cio_sck_i;
  assign clk_spi_in    = cio_sck_i;
  assign clk_spi_out   = sck_n;

  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_clk_spi_in_mux (
    .clk0_i(clk_spi_in),
    .clk1_i(scan_clk_i),
    .sel_i(prim_mubi_pkg::mubi4_test_true_strict(scanmode[ClkMuxSel]) | mbist_en_i),
    .clk_o(clk_spi_in_muxed)
  );

  prim_clock_buf #(
    .RegionSel(1'b1)
  ) u_clk_spi_in_buf(
    .clk_i (clk_spi_in_muxed),
    .clk_o (clk_spi_in_buf)
  );

  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_clk_spi_out_mux (
    .clk0_i(clk_spi_out),
    .clk1_i(scan_clk_i),
    .sel_i(prim_mubi_pkg::mubi4_test_true_strict(scanmode[ClkMuxSel])),
    .clk_o(clk_spi_out_muxed)
  );

  prim_clock_buf #(
    .RegionSel(1'b1)
  ) u_clk_spi_out_buf(
    .clk_i (clk_spi_out_muxed),
    .clk_o (clk_spi_out_buf)
  );

  // CSb muxed to scan clock
  prim_clock_mux2 #(
    .NoFpgaBufG(1'b 1)
  ) u_clk_csb_mux (
    .clk0_i (clk_csb_buf),
    .clk1_i (scan_clk_i ),
    .sel_i  (prim_mubi_pkg::mubi4_test_true_strict(scanmode[ClkMuxSel])),
    .clk_o  (clk_csb_muxed)
  );

  prim_clock_buf #(
    .NoFpgaBuf (1'b 1)
  ) u_clk_csb_buf (
    .clk_i (clk_csb_muxed),
    .clk_o (clk_csb      )
  );

  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_csb_rst_scan_mux (
    .clk0_i(rst_ni & ~rst_csb_buf),
    .clk1_i(scan_rst_ni),
    .sel_i(prim_mubi_pkg::mubi4_test_true_strict(scanmode[CsbRstMuxSel])),
    .clk_o(rst_spi_n)
  );

  logic rst_spi_in_n, rst_spi_out_sync_n, rst_spi_out_n;
  assign rst_spi_in_n = rst_spi_n;

  // Synchronizes reset de-assertion to safely occur in the outclk domain.
  prim_flop #(
    .Width      (1),
    .ResetValue (1'b0)
  ) u_rst_spi_out_sync (
    .clk_i (clk_spi_in_buf),
    .rst_ni(rst_spi_n),
    .d_i   (1'b1),
    .q_o   (rst_spi_out_sync_n)
  );

  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_csb_rst_out_scan_mux (
    .clk0_i(rst_spi_out_sync_n),
    .clk1_i(scan_rst_ni),
    .sel_i(prim_mubi_pkg::mubi4_test_true_strict(scanmode[CsbRstMuxSel])),
    .clk_o(rst_spi_out_n)
  );

  logic tpm_rst_in_n, tpm_rst_out_sync_n, tpm_rst_out_n, sys_tpm_rst_n;

  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_tpm_csb_rst_scan_mux (
    .clk0_i (rst_ni & ~rst_tpm_csb_buf),
    .clk1_i (scan_rst_ni),
    .sel_i  (prim_mubi_pkg::mubi4_test_true_strict(scanmode[TpmRstSel])),
    .clk_o  (tpm_rst_in_n)
  );

  // Synchronizes reset de-assertion to safely occur in the outclk domain.
  prim_flop #(
    .Width      (1),
    .ResetValue (1'b0)
  ) u_tpm_rst_out_sync (
    .clk_i (clk_spi_in_buf),
    .rst_ni(tpm_rst_in_n),
    .d_i   (1'b1),
    .q_o   (tpm_rst_out_sync_n)
  );

  prim_clock_mux2 #(
    .NoFpgaBufG(1'b1)
  ) u_tpm_rst_out_scan_mux (
    .clk0_i (tpm_rst_out_sync_n),
    .clk1_i (scan_rst_ni),
    .sel_i  (prim_mubi_pkg::mubi4_test_true_strict(scanmode[TpmRstSel])),
    .clk_o  (tpm_rst_out_n)
  );

  // TPM Read FIFO uses TPM CSb as a reset.
  // The write port is clocked at SYS_CLK. Metastability may occur as CSb may
  // be asserted, de-asserted independent of SYS_CLK. This reset synchronizer
  // (sync to SYS_CLK), may delay the reset signal by 2 SYS_CLK when TPM_CSb
  // is de-asserted.
  prim_rst_sync #(
    .ActiveHigh (1'b 0),
    .SkipScan   (1'b 0)
  ) u_tpm_csb_rst_sync (
    .clk_i,
    .d_i   (tpm_rst_in_n),
    .q_o   (sys_tpm_rst_n),

    .scan_rst_ni,
    .scanmode_i (scanmode[TpmRstSel])
  );

  // CSb edge on the system clock
  spid_csb_sync u_spid_csb_sync (
    .clk_i,
    .rst_ni,
    .sck_i                     (clk_spi_in_buf),
    .sck_pulse_en_i            (1'b1),
    .csb_i                     (clk_csb),
    .csb_deasserted_pulse_o    (sys_csb_deasserted_pulse)
  );

  prim_flop_2sync #(
    .Width       (1)
  ) u_sys_csb_syncd (
    .clk_i,
    .rst_ni,
    .d_i       (sys_csb),
    .q_o       (sys_csb_syncd)
  );

  // TPM CSb 2FF sync to SYS_CLK
  prim_flop_2sync #(
    .Width      (1    ),
    .ResetValue (1'b 1)
  ) u_sys_tpm_csb_sync (
    .clk_i,
    .rst_ni,

    .d_i (sys_tpm_csb_buf),
    .q_o (sys_tpm_csb_syncd)
  );

  //////////////////////////////
  // SPI_DEVICE mode selector //
  //////////////////////////////
  // This logic chooses appropriate signals based on input SPI_DEVICE mode.
  // Assume spi_mode does not change dynamically

  // io_mode to spi_s2p io_mode should be affected at the negedge of SPI_CLK
  // based on SPI protocol. the internal io_mode signal is generated by SPI
  // input signals. So, the io_mode should be latched at clk_spi_out to not
  // introduce the timing loop.
  //
  // example: cmdparse triggers sel_dp at 8th beat of CMD bit.
  //       -> readcmd activates, it also changes IoMode if opcode is DualIO
  //          or QuadIO commands
  //       -> changed io_mode affects spi_s2p module, which again affects
  //          cmdparse module.
  always_ff @(posedge clk_spi_out_buf or negedge rst_spi_out_n) begin
    if (!rst_spi_out_n) io_mode_outclk <= SingleIO;
    else                io_mode_outclk <= io_mode;
  end

  // SCK clock domain MUX for SRAM access for Flash and Passthrough
  always_comb begin
    flash_sram_l2m = '{ default: '0 };

    for (int unsigned i = IoModeCmdParse ; i < IoModeEnd ; i++) begin
      sub_sram_m2l[i] = '{
        rvalid: 1'b 0,
        rdata: '0,
        rerror: '{uncorr: 1'b 0, corr: 1'b 0}
      };
    end

    unique case (cmd_dp_sel)
      DpReadCmd, DpReadSFDP: begin
        // SRAM:: Remember this has glitch
        // switch should happen only when clock gate is disabled.
        flash_sram_l2m = sub_sram_l2m[IoModeReadCmd];
        sub_sram_m2l[IoModeReadCmd] = flash_sram_m2l;
      end

      DpUpload: begin
        flash_sram_l2m = sub_sram_l2m[IoModeUpload];
        sub_sram_m2l[IoModeUpload] = flash_sram_m2l;
      end

      default: begin
        if (cmd_only_dp_sel == DpUpload) begin
          // Be ready to upload commands on the 8th command bit, when directed
          flash_sram_l2m = sub_sram_l2m[IoModeUpload];
          sub_sram_m2l[IoModeUpload] = flash_sram_m2l;
        end else begin
          // DpNone, DpReadStatus, DpReadJEDEC
          flash_sram_l2m = '{default: '0 };
        end
      end
    endcase
  end

  always_comb begin
    // SRAM comb logic is in SCK clock domain
    mem_b_l2m = '{ default: '0 };

    flash_sram_m2l = '{
      rvalid: 1'b 0,
      rdata: '0,
      rerror: '{uncorr: 1'b 0, corr: 1'b 0}
    };
    tpm_sram_m2l = '{
      rvalid: 1'b 0,
      rdata: '0,
      rerror: '{uncorr: 1'b 0, corr: 1'b 0}
    };

    if (!sck_csb && ((spi_mode == FlashMode) || (spi_mode == PassThrough))) begin
      mem_b_l2m = flash_sram_l2m;
      flash_sram_m2l = mem_b_m2l;
    end else if (cfg_tpm_en) begin
      mem_b_l2m = tpm_sram_l2m;
      tpm_sram_m2l = mem_b_m2l;
    end
  end

  // inverted SCK clock domain MUX for IO Mode and P2S
  always_comb begin
    io_mode = SingleIO;
    p2s_valid = 1'b 0;
    p2s_data  = 8'h 0;
    sub_p2s_sent = '{default: 1'b 0};

    unique case (spi_mode)
      FlashMode, PassThrough: begin
        unique case (cmd_dp_sel)
          DpNone: begin
            io_mode = sub_iomode[IoModeCmdParse];

            sub_p2s_sent[IoModeCmdParse] = p2s_sent;

          end
          DpReadCmd, DpReadSFDP: begin
            io_mode = sub_iomode[IoModeReadCmd];

            p2s_valid = sub_p2s_valid[IoModeReadCmd];
            p2s_data  = sub_p2s_data[IoModeReadCmd];
            sub_p2s_sent[IoModeReadCmd] = p2s_sent;
          end
          DpReadStatus: begin
            io_mode = sub_iomode[IoModeStatus];

            p2s_valid = sub_p2s_valid[IoModeStatus];
            p2s_data  = sub_p2s_data[IoModeStatus];
            sub_p2s_sent[IoModeStatus] = p2s_sent;

          end

          DpReadJEDEC: begin
            io_mode = sub_iomode[IoModeJedec];

            p2s_valid = sub_p2s_valid[IoModeJedec];
            p2s_data  = sub_p2s_data[IoModeJedec];
            sub_p2s_sent[IoModeJedec] = p2s_sent;
          end

          DpUpload: begin
            io_mode = sub_iomode[IoModeUpload];

            p2s_valid = sub_p2s_valid[IoModeUpload];
            p2s_data  = sub_p2s_data[IoModeUpload];
            sub_p2s_sent[IoModeUpload] = p2s_sent;
          end
          // DpUnknown:
          default: begin
            io_mode = sub_iomode[IoModeCmdParse];

            sub_p2s_sent[IoModeCmdParse] = p2s_sent;
          end
        endcase
      end

      default: begin
        io_mode = SingleIO;
      end
    endcase
  end
  `ASSERT_KNOWN(SpiModeKnown_A, spi_mode)

  // Add 2-cycle delay to flash read data when requested.
  // This mechanism should only be deployed on read commands with dummy cycles,
  // so omit delaying the output enable.
  logic [3:0] internal_sd_stg1_d, internal_sd_stg1_q;
  logic [3:0] internal_sd_stg2_d, internal_sd_stg2_q;
  logic [3:0] internal_sd_en_stg1, internal_sd_en_stg2;
  logic intercept_en_stg1, intercept_en_stg2;
  assign internal_sd_stg1_d = internal_sd;
  assign internal_sd_stg2_d = internal_sd_stg1_q;

  prim_flop #(
    .Width         ($bits(internal_sd_stg1_d)),
    .ResetValue    ('0)
  ) u_read_pipe_stg1 (
    .clk_i     (clk_spi_out_buf),
    .rst_ni    (rst_spi_out_n),
    .d_i       (internal_sd_stg1_d),
    .q_o       (internal_sd_stg1_q)
  );

  prim_flop #(
    .Width         ($bits(internal_sd_stg2_d)),
    .ResetValue    ('0)
  ) u_read_pipe_stg2 (
    .clk_i     (clk_spi_out_buf),
    .rst_ni    (rst_spi_out_n),
    .d_i       (internal_sd_stg2_d),
    .q_o       (internal_sd_stg2_q)
  );

  prim_flop #(
    .Width         ($bits(internal_sd_en)),
    .ResetValue    ('0)
  ) u_read_en_pipe_stg1 (
    .clk_i     (clk_spi_out_buf),
    .rst_ni    (rst_spi_out_n),
    .d_i       (internal_sd_en),
    .q_o       (internal_sd_en_stg1)
  );

  prim_flop #(
    .Width         ($bits(internal_sd_en_stg1)),
    .ResetValue    ('0)
  ) u_read_en_pipe_stg2 (
    .clk_i     (clk_spi_out_buf),
    .rst_ni    (rst_spi_out_n),
    .d_i       (internal_sd_en_stg1),
    .q_o       (internal_sd_en_stg2)
  );

  prim_flop #(
    .Width         (1),
    .ResetValue    ('0)
  ) u_read_intercept_pipe_stg1 (
    .clk_i     (clk_spi_out_buf),
    .rst_ni    (rst_spi_out_n),
    .d_i       (intercept_en),
    .q_o       (intercept_en_stg1)
  );

  prim_flop #(
    .Width         (1),
    .ResetValue    ('0)
  ) u_read_intercept_pipe_stg2 (
    .clk_i     (clk_spi_out_buf),
    .rst_ni    (rst_spi_out_n),
    .d_i       (intercept_en_stg1),
    .q_o       (intercept_en_stg2)
  );

  always_comb begin
    if (cmd_read_pipeline_sel) begin
      internal_sd_out = internal_sd_stg2_q;
      internal_sd_en_out = internal_sd_en_stg2;
      intercept_en_out = intercept_en_stg2;
    end else begin
      internal_sd_out = internal_sd;
      internal_sd_en_out = internal_sd_en;
      intercept_en_out = intercept_en;
    end
  end


  always_comb begin
    cio_sd_o    = internal_sd_out;
    cio_sd_en_o = internal_sd_en_out;

    if (cfg_tpm_en && !sck_tpm_csb_buf) begin : miso_tpm
      // TPM transaction is on-going. MOSI, MISO is being used by TPM
      cio_sd_o    = {2'b 00, tpm_miso,    1'b 0};
      cio_sd_en_o = {2'b 00, tpm_miso_en, 1'b 0};

    end else begin : spi_out_flash_passthrough
      // SPI Flash, Passthrough modes
      unique case (spi_mode)
        FlashMode: begin
          cio_sd_o    = internal_sd_out;
          cio_sd_en_o = internal_sd_en_out;
        end

        PassThrough: begin
          if (intercept_en_out) begin
            cio_sd_o    = internal_sd_out;
            cio_sd_en_o = internal_sd_en_out;
          end else begin
            cio_sd_o    = passthrough_sd;
            cio_sd_en_o = passthrough_sd_en;
          end
        end

        default: begin
          cio_sd_o    = internal_sd;
          cio_sd_en_o = internal_sd_en;
        end
      endcase
    end
  end

  // Assume `intercept` is registered (SPI_IN).
  // passthrough assumed signal shall be registered in (SPI_OUT)
  always_ff @(posedge clk_spi_out_buf or negedge rst_spi_out_n) begin
    if (!rst_spi_out_n) intercept_en <= 1'b 0;
    else                intercept_en <= |intercept;
  end
  // intercept_en shall not be de-asserted except mailbox
  `ASSUME(InterceptLevel_M,
    $rose(|{intercept.status, intercept.jedec, intercept.sfdp}) |=>
      ##1 $stable(intercept_en) until !rst_spi_out_n,
    clk_spi_out_buf, !rst_spi_out_n)

  ////////////////////////////
  // SPI Serial to Parallel //
  ////////////////////////////
  spi_s2p u_s2p (
    .clk_i        (clk_spi_in_buf),
    .rst_ni       (rst_spi_in_n),

    // SPI interface
    .s_i          (cio_sd_i),

    .data_valid_o (s2p_data_valid),
    .data_o       (s2p_data      ),

    // Config (changed dynamically)
    .order_i      (rxorder),
    .io_mode_i    (io_mode_outclk)
  );

  spi_p2s u_p2s (
    .clk_i        (clk_spi_out_buf),
    .rst_ni       (rst_spi_out_n),

    .data_valid_i (p2s_valid),
    .data_i       (p2s_data),
    .data_sent_o  (p2s_sent),

    .csb_i        (sck_csb),
    .s_en_o       (internal_sd_en),
    .s_o          (internal_sd),

    .order_i      (txorder),
    .io_mode_i    (io_mode_outclk)
  );

  ////////////////////
  // SPI Flash Mode //
  ////////////////////

  spi_cmdparse u_cmdparse (
    .clk_i  (clk_spi_in_buf),
    .rst_ni (rst_spi_in_n),

    .data_valid_i (s2p_data_valid),
    .data_i       (s2p_data),

    .spi_mode_i   (spi_mode),

    .cmd_info_i   (cmd_info),

    .sck_status_busy_i(sck_status_busy),

    .io_mode_o    (sub_iomode[IoModeCmdParse]),

    .sel_dp_o          (cmd_dp_sel),
    .cmd_only_sel_dp_o (cmd_only_dp_sel),
    .cmd_info_o        (cmd_info_broadcast),
    .cmd_info_idx_o    (cmd_info_idx_broadcast),
    .cmd_only_info_o     (cmd_only_info_broadcast),
    .cmd_only_info_idx_o (cmd_only_info_idx_broadcast),
    .cmd_sync_pulse_o  (cmd_sync_pulse),

    .cmd_read_pipeline_sel_o (cmd_read_pipeline_sel),

    .cfg_intercept_en_status_i (cfg_intercept_en.status),
    .cfg_intercept_en_jedec_i  (cfg_intercept_en.jedec),
    .cfg_intercept_en_sfdp_i   (cfg_intercept_en.sfdp),

    .intercept_status_o (intercept.status),
    .intercept_jedec_o  (intercept.jedec),
    .intercept_sfdp_o   (intercept.sfdp),

    // Not used for now
    .cmd_config_req_o (),
    .cmd_config_idx_o ()
  );

  spi_readcmd u_readcmd (
    .clk_i  (clk_spi_in_buf),
    .rst_ni (rst_spi_in_n),

    .clk_out_i  (clk_spi_out_buf),
    .rst_out_ni (rst_spi_out_n),

    .sys_clk_i  (clk_i),
    .sys_rst_ni (rst_ni),

    .sel_dp_i   (cmd_dp_sel),

    // SRAM interface
    .sram_l2m_o (sub_sram_l2m[IoModeReadCmd]),
    .sram_m2l_i (sub_sram_m2l[IoModeReadCmd]),

    // S2P
    .s2p_valid_i   (s2p_data_valid),
    .s2p_byte_i    (s2p_data),

    // P2S
    .p2s_valid_o   (sub_p2s_valid [IoModeReadCmd]),
    .p2s_byte_o    (sub_p2s_data  [IoModeReadCmd]),
    .p2s_sent_i    (sub_p2s_sent  [IoModeReadCmd]),

    .spi_mode_i       (spi_mode),

    .cmd_info_i     (cmd_info_broadcast),
    .cmd_info_idx_i (cmd_info_idx_broadcast),

    .readbuf_threshold_i (readbuf_threshold),
    .sys_readbuf_clr_i   (readbuf_clr),

    .addr_4b_en_i (cfg_addr_4b_en),

    .mailbox_en_i           (cfg_mailbox_en ),
    .cfg_intercept_en_mbx_i (cfg_intercept_en.mbx),

    .mailbox_addr_i    (mailbox_addr   ),
    .mailbox_assumed_o (intercept.mbx  ),

    .readbuf_address_o (readbuf_addr_sck),

    .io_mode_o (sub_iomode [IoModeReadCmd]),

    .sck_read_watermark_o (flash_sck_readbuf_watermark),
    .sck_read_flip_o      (flash_sck_readbuf_flip)
  );

  // Begin: Read Status ==============================================
  logic readstatus_qe;
  logic [23:0] readstatus_q;
  logic [23:0] readstatus_d;

  assign readstatus_qe =  reg2hw.flash_status.busy.qe &&
                          reg2hw.flash_status.wel.qe &&
                          reg2hw.flash_status.status.qe;
  assign readstatus_q = { reg2hw.flash_status.status.q,
                          reg2hw.flash_status.wel.q,
                          reg2hw.flash_status.busy.q
                        };
  assign hw2reg.flash_status.busy.d   = readstatus_d[0];
  assign hw2reg.flash_status.wel.d    = readstatus_d[1];
  assign hw2reg.flash_status.status.d = readstatus_d[23:2];

  assign sck_status_wr_set = (cmd_only_dp_sel == DpWrEn);
  assign sck_status_wr_clr = (cmd_only_dp_sel == DpWrDi);

  logic flash_status_sync_fifo_clr;
  assign flash_status_sync_fifo_clr = reg2hw.control.flash_status_fifo_clr.q;
  assign hw2reg.control.flash_status_fifo_clr.d  = '0;
  assign hw2reg.control.flash_status_fifo_clr.de = 1'b1;

  spid_status u_spid_status (
    .clk_i  (clk_spi_in_buf),
    .rst_ni (rst_spi_in_n),

    .clk_out_i  (clk_spi_out_buf),
    .rst_out_ni (rst_spi_out_n),

    .clk_csb_i (clk_csb),

    .sys_clk_i  (clk_i),
    .sys_rst_ni (rst_ni),

    .sys_csb_deasserted_pulse_i (sys_csb_deasserted_pulse),

    .sys_update_clr_i(flash_status_sync_fifo_clr),

    .sys_status_we_i (readstatus_qe),
    .sys_status_i    (readstatus_q),
    .sys_status_o    (readstatus_d),

    .sel_dp_i       (cmd_dp_sel),
    .cmd_info_i     (cmd_info_broadcast),
    .cmd_info_idx_i (cmd_info_idx_broadcast),

    .outclk_p2s_valid_o (sub_p2s_valid[IoModeStatus]),
    .outclk_p2s_byte_o  (sub_p2s_data[IoModeStatus]),
    .outclk_p2s_sent_i  (sub_p2s_sent[IoModeStatus]),

    .io_mode_o   (sub_iomode[IoModeStatus]),

    .inclk_busy_set_i  (sck_status_busy_set), // SCK domain

    .inclk_we_set_i (sck_status_wr_set),
    .inclk_we_clr_i (sck_status_wr_clr),

    .inclk_status_commit_i  (s2p_data_valid),
    .cmd_sync_status_busy_o (cmd_sync_status_busy),
    .cmd_sync_status_wel_o  (cmd_sync_status_wel),
    .sck_status_busy_o      (sck_status_busy),
    .csb_busy_broadcast_o   (csb_status_busy_broadcast), // SCK domain
    .scan_rst_ni,
    .scanmode_i             (scanmode[StatusFifoRstSel])
  );

  // Tie unused
  logic unused_sub_sram_status;
  assign unused_sub_sram_status = ^{
    sub_sram_l2m[IoModeStatus],
    sub_sram_m2l[IoModeStatus]
  };
  assign sub_sram_l2m[IoModeStatus] = '0;

  // End: Read Status ------------------------------------------------

  spid_jedec u_jedec (
    .clk_i  (clk_spi_in_buf),
    .rst_ni (rst_spi_in_n),

    .clk_out_i  (clk_spi_out_buf),
    .rst_out_ni (rst_spi_out_n),

    .cmd_sync_pulse_i (cmd_sync_pulse),

    .sys_jedec_i (jedec_cfg),

    .io_mode_o (sub_iomode[IoModeJedec]),

    .sel_dp_i       (cmd_dp_sel),
    .cmd_info_i     (cmd_info_broadcast),
    .cmd_info_idx_i (cmd_info_idx_broadcast),

    .outclk_p2s_valid_o (sub_p2s_valid[IoModeJedec]),
    .outclk_p2s_byte_o  (sub_p2s_data[IoModeJedec]),
    .outclk_p2s_sent_i  (sub_p2s_sent[IoModeJedec])
  );
  // Tie unused
  logic unused_sub_sram_jedec;
  assign unused_sub_sram_jedec = ^{
    sub_sram_l2m[IoModeJedec],
    sub_sram_m2l[IoModeJedec]
  };
  assign sub_sram_l2m[IoModeJedec] = '0;

  // Begin: Upload ===================================================
  spid_upload #(
    .CmdFifoBaseAddr  (SramCmdFifoIdx),
    .CmdFifoDepth     (SramCmdFifoDepth),
    .AddrFifoBaseAddr (SramAddrFifoIdx),
    .AddrFifoDepth    (SramAddrFifoDepth),
    .PayloadBaseAddr  (SramPayloadIdx),
    .PayloadDepth     (SramPayloadDepth),

    .SpiByte ($bits(spi_byte_t))
  ) u_upload (
    .clk_i  (clk_spi_in_buf),
    .rst_ni (rst_spi_in_n),

    .sys_clk_i  (clk_i),
    .sys_rst_ni (rst_ni),

    .clk_csb_i (clk_csb),

    .sel_dp_i          (cmd_dp_sel),
    .cmd_only_sel_dp_i (cmd_only_dp_sel),

    .sck_sram_o (sub_sram_l2m[IoModeUpload]),
    .sck_sram_i (sub_sram_m2l[IoModeUpload]),

    .sys_cmdfifo_sram_o (sys_sram_l2m[SysSramCmdFifo]),
    .sys_cmdfifo_sram_i (sys_sram_m2l[SysSramCmdFifo]),
    .sys_cmdfifo_gnt_i  (sys_sram_gnt[SysSramCmdFifo]),

    .sys_addrfifo_sram_o (sys_sram_l2m[SysSramAddrFifo]),
    .sys_addrfifo_sram_i (sys_sram_m2l[SysSramAddrFifo]),
    .sys_addrfifo_gnt_i  (sys_sram_gnt[SysSramAddrFifo]),

    // SYS clock FIFO interface
    .sys_cmdfifo_rvalid_o (cmdfifo_rvalid),
    .sys_cmdfifo_rready_i (cmdfifo_rready),
    .sys_cmdfifo_rdata_o  (cmdfifo_rdata),

    .sys_addrfifo_rvalid_o (addrfifo_rvalid),
    .sys_addrfifo_rready_i (addrfifo_rready),
    .sys_addrfifo_rdata_o  (addrfifo_rdata),

    // Interface: SPI to Parallel
    .s2p_valid_i  (s2p_data_valid),
    .s2p_byte_i   (s2p_data),

    // Interface: Parallel to SPI
    .p2s_valid_o (sub_p2s_valid[IoModeUpload]),
    .p2s_data_o  (sub_p2s_data [IoModeUpload]),
    .p2s_sent_i  (sub_p2s_sent [IoModeUpload]),

    .spi_mode_i (spi_mode),

    .cmd_sync_cfg_addr_4b_en_i (cmd_sync_addr_4b_en),
    .cmd_sync_status_wel_i     (cmd_sync_status_wel),
    .cmd_sync_status_busy_i    (cmd_sync_status_busy),

    .cmd_only_info_i     (cmd_only_info_broadcast),
    .cmd_only_info_idx_i (cmd_only_info_idx_broadcast),

    .io_mode_o (sub_iomode[IoModeUpload]),

    .set_busy_o (sck_status_busy_set),

    .sys_cmdfifo_set_o       (cmdfifo_set_pulse),
    .sys_cmdfifo_notempty_o  (cmdfifo_notempty),
    .sys_cmdfifo_full_o      (), // not used
    .sys_addrfifo_notempty_o (addrfifo_notempty),
    .sys_addrfifo_full_o     (), // not used
    .sys_payload_overflow_o  (payload_overflow),

    .sys_cmdfifo_depth_o     (cmdfifo_depth),
    .sys_addrfifo_depth_o    (addrfifo_depth),
    .sys_payload_depth_o     (payload_depth),
    .sys_payload_start_idx_o (payload_start_idx)
  );
  // FIFO connect
  assign cmdfifo_rready = reg2hw.upload_cmdfifo.data.re;
  assign hw2reg.upload_cmdfifo.data.d = cmdfifo_rdata[7:0];
  assign hw2reg.upload_cmdfifo.busy.d = cmdfifo_rdata[13];
  assign hw2reg.upload_cmdfifo.wel.d = cmdfifo_rdata[14];
  assign hw2reg.upload_cmdfifo.addr4b_mode.d = cmdfifo_rdata[15];
  logic unused_cmdfifo_re;
  assign unused_cmdfifo_re = ^{reg2hw.upload_cmdfifo.busy.re,
                               reg2hw.upload_cmdfifo.wel.re,
                               reg2hw.upload_cmdfifo.addr4b_mode.re};
  logic unused_cmdfifo_q;
  assign unused_cmdfifo_q = ^{reg2hw.upload_cmdfifo.data.q,
                              reg2hw.upload_cmdfifo.busy.q,
                              reg2hw.upload_cmdfifo.wel.q,
                              reg2hw.upload_cmdfifo.addr4b_mode.q,
                              cmdfifo_rdata[12:8],
                              cmdfifo_rvalid};

  assign addrfifo_rready = reg2hw.upload_addrfifo.re;
  assign hw2reg.upload_addrfifo.d = addrfifo_rdata;
  logic unused_addrfifo_q;
  assign unused_addrfifo_q = ^{reg2hw.upload_addrfifo.q, addrfifo_rvalid};

  // Connect UPLOAD_STATUS
  assign hw2reg.upload_status.cmdfifo_depth.de = 1'b1;
  assign hw2reg.upload_status.cmdfifo_depth.d  = cmdfifo_depth;

  assign hw2reg.upload_status.cmdfifo_notempty.de = 1'b1;
  assign hw2reg.upload_status.cmdfifo_notempty.d  = cmdfifo_notempty;

  assign hw2reg.upload_status.addrfifo_depth.de = 1'b 1;
  assign hw2reg.upload_status.addrfifo_depth.d  = addrfifo_depth;

  assign hw2reg.upload_status.addrfifo_notempty.de = 1'b 1;
  assign hw2reg.upload_status.addrfifo_notempty.d  = addrfifo_notempty;

  assign hw2reg.upload_status2.payload_depth.de = 1'b 1;
  assign hw2reg.upload_status2.payload_depth.d  = payload_depth;

  assign hw2reg.upload_status2.payload_start_idx.de = 1'b 1;
  assign hw2reg.upload_status2.payload_start_idx.d = payload_start_idx;
  `ASSERT_INIT(PayloadStartIdxWidthMatch_A,
    $bits(hw2reg.upload_status2.payload_start_idx.d) == PayloadIdxW)

  // End:   Upload ---------------------------------------------------

  // Begin: Address 3B/4B Tracker ====================================
  assign cmd_en4b_pulse = cmd_only_dp_sel == DpEn4B;
  assign cmd_ex4b_pulse = cmd_only_dp_sel == DpEx4B;
  spid_addr_4b u_spid_addr_4b (
    .sys_clk_i  (clk_i ),
    .sys_rst_ni (rst_ni),

    .spi_clk_i  (clk_spi_in_buf),

    .cmd_sync_pulse_i (cmd_sync_pulse),

    .reg2hw_addr_mode_addr_4b_en_q_i   (reg2hw.addr_mode.addr_4b_en.q),
    .reg2hw_addr_mode_addr_4b_en_qe_i  (reg2hw.addr_mode.addr_4b_en.qe),
    .hw2reg_addr_mode_pending_d_o      (hw2reg.addr_mode.pending.d),
    .hw2reg_addr_mode_addr_4b_en_d_o   (hw2reg.addr_mode.addr_4b_en.d),

    .spi_cfg_addr_4b_en_o      (cfg_addr_4b_en), // broadcast
    .cmd_sync_cfg_addr_4b_en_o (cmd_sync_addr_4b_en), // early output for upload

    .spi_addr_4b_set_i (cmd_en4b_pulse), // EN4B command
    .spi_addr_4b_clr_i (cmd_ex4b_pulse)  // EX4B command
  );
  // End:   Address 3B/4B Tracker ------------------------------------

  /////////////////////
  // SPI Passthrough //
  /////////////////////

  // Passthrough block
  // signal: sys_csb_syncd -> sysclock 2FF CSb
  // signal: sys_busy  -> output of u_status readstatus_d[0]
  //              set by CSb deassertion pulse & BUSY(SCK)
  //              clr by CSb = 1 & SW writing 0
  //
  // NOTE: there will be a gap between the actual assertion of CSb and the CSb
  //   syncd event visible in the u_status BUSY logic (2FF @ SYS_CLK). So,
  //   there's chance that the SW may clear the BUSY right at the CSb
  //   assertion event. If that happens, passthrough block may set during SPI
  //   transaction. The behavior of the SPI_DEVICE in this scenario is
  //   undeterminstic.
  logic  passthrough_block;
  assign passthrough_block = csb_status_busy_broadcast;

  spi_passthrough u_passthrough (
    .clk_i     (clk_spi_in_buf),
    .rst_ni    (rst_spi_in_n),
    .clk_out_i (clk_spi_out_buf),
    .rst_out_ni(rst_spi_out_n),

    .cfg_cmd_filter_i (cmd_filter),

    .cfg_addr_mask_i  (addr_swap_mask),
    .cfg_addr_value_i (addr_swap_data),

    .cfg_payload_mask_i (payload_swap_mask),
    .cfg_payload_data_i (payload_swap_data),

    .cfg_addr_4b_en_i (cfg_addr_4b_en),

    .cmd_info_i (cmd_info),

    .spi_mode_i       (spi_mode),

    // Control: BUSY block
    .passthrough_block_i (passthrough_block),

    // Host SPI
    .host_sck_i  (cio_sck_i),
    .host_csb_i  (cio_csb_i),
    .host_s_i    (cio_sd_i),
    .host_s_o    (passthrough_sd),
    .host_s_en_o (passthrough_sd_en),

    // Passthrough to SPI_HOST HWIP
    .passthrough_o,
    .passthrough_i,

    .event_cmd_filtered_o ()
  );

  //////////////////
  // TPM over SPI //
  //////////////////
  // Instance of spi_tpm
  spi_tpm #(
    // CmdAddrFifoDepth
    .EnLocality  (1)
  ) u_spi_tpm (
    .clk_in_i  (clk_spi_in_buf ),
    .clk_out_i (clk_spi_out_buf),
    .rst_ni    (tpm_rst_in_n ),
    .rst_out_ni(tpm_rst_out_n),

    .sys_clk_i (clk_i),
    .sys_rst_ni(rst_ni       ),

    .sys_tpm_rst_ni(sys_tpm_rst_n),

    .csb_i     (sck_tpm_csb_buf), // used as data only
    .mosi_i    (tpm_mosi       ),
    .miso_o    (tpm_miso       ),
    .miso_en_o (tpm_miso_en    ),

    .tpm_cap_o (tpm_cap),
    .cfg_tpm_en_i               (cfg_tpm_en              ),
    .cfg_tpm_mode_i             (cfg_tpm_mode            ),
    .cfg_tpm_hw_reg_dis_i       (cfg_tpm_hw_reg_dis      ),
    .cfg_tpm_reg_chk_dis_i      (cfg_tpm_reg_chk_dis     ),
    .cfg_tpm_invalid_locality_i (cfg_tpm_invalid_locality),

    .sys_access_reg_i          (tpm_access         ),
    .sys_int_enable_reg_i      (tpm_int_enable     ),
    .sys_int_vector_reg_i      (tpm_int_vector     ),
    .sys_int_status_reg_i      (tpm_int_status     ),
    .sys_intf_capability_reg_i (tpm_intf_capability),
    .sys_status_reg_i          (tpm_status         ),
    .sys_id_reg_i              (tpm_did_vid        ),
    .sys_rid_reg_i             (tpm_rid            ),

    .sck_sram_o                (tpm_sram_l2m),
    .sck_sram_i                (tpm_sram_m2l),
    .sys_sram_o                (sys_sram_l2m[SysSramTpmRdFifo]),
    .sys_sram_i                (sys_sram_m2l[SysSramTpmRdFifo]),
    .sys_sram_gnt_i            (sys_sram_gnt[SysSramTpmRdFifo]),

    .sys_cmdaddr_rvalid_o (tpm_cmdaddr_rvalid),
    .sys_cmdaddr_rdata_o  (tpm_cmdaddr_rdata ),
    .sys_cmdaddr_rready_i (tpm_cmdaddr_rready),

    .sys_rdfifo_wvalid_i  (tpm_rdfifo_wvalid       ),
    .sys_rdfifo_wdata_i   (tpm_rdfifo_wdata        ),
    .sys_rdfifo_wready_o  (tpm_rdfifo_wready       ),
    .sys_rdfifo_cmd_end_o (tpm_event_rdfifo_cmd_end),
    .sys_tpm_rdfifo_drop_o(tpm_event_rdfifo_drop   ),

    .sys_wrfifo_release_i(tpm_status_wrfifo_release),

    .sys_cmdaddr_notempty_o (tpm_status_cmdaddr_notempty),
    .sys_wrfifo_pending_o   (tpm_status_wrfifo_pending),
    .sys_rdfifo_aborted_o   (tpm_status_rdfifo_aborted)
  );

  // Register connection
  //  TPM_CAP:
  assign hw2reg.tpm_cap = '{
    rev:         '{ de: 1'b 1, d: tpm_cap.rev         },
    locality:    '{ de: 1'b 1, d: tpm_cap.locality    },
    max_wr_size: '{ de: 1'b 1, d: tpm_cap.max_wr_size },
    max_rd_size: '{ de: 1'b 1, d: tpm_cap.max_rd_size }
  };

  //  CFG:
  assign cfg_tpm_en               = reg2hw.tpm_cfg.en.q;
  assign cfg_tpm_mode             = reg2hw.tpm_cfg.tpm_mode.q;
  assign cfg_tpm_hw_reg_dis       = reg2hw.tpm_cfg.hw_reg_dis.q;
  assign cfg_tpm_reg_chk_dis      = reg2hw.tpm_cfg.tpm_reg_chk_dis.q;
  assign cfg_tpm_invalid_locality = reg2hw.tpm_cfg.invalid_locality.q;

  //  STATUS:
  assign hw2reg.tpm_status = '{
    rdfifo_aborted:   '{ d: tpm_status_rdfifo_aborted },
    wrfifo_pending:   '{ d: tpm_status_wrfifo_pending },
    cmdaddr_notempty: '{ d: tpm_status_cmdaddr_notempty }
  };

  // wrfifo_release is RW0C
  assign tpm_status_wrfifo_release = reg2hw.tpm_status.wrfifo_pending.qe &
                                     ~reg2hw.tpm_status.wrfifo_pending.q;

  //  Return-by-HW registers:
  //    TPM_ACCESS_x, TPM_STS_x, TPM_INT_ENABLE, TPM_INT_VECTOR,
  //    TPM_INT_STATUS, TPM_INTF_CAPABILITY, TPM_DID_VID, TPM_RID
  for (genvar i = 0 ; i < spi_device_reg_pkg::NumLocality ; i++) begin : g_tpm_access
    assign tpm_access[8*i+:8] = reg2hw.tpm_access[i].q;
  end : g_tpm_access

  assign tpm_int_enable      = reg2hw.tpm_int_enable.q;
  assign tpm_int_vector      = reg2hw.tpm_int_vector.q;
  assign tpm_int_status      = reg2hw.tpm_int_status.q;
  assign tpm_intf_capability = reg2hw.tpm_intf_capability.q;
  assign tpm_status          = reg2hw.tpm_sts.q;
  assign tpm_did_vid         = { reg2hw.tpm_did_vid.did.q ,
                                 reg2hw.tpm_did_vid.vid.q };
  assign tpm_rid             = reg2hw.tpm_rid.q;

  // Command / Address Buffer
  logic  unused_tpm_cmdaddr;
  assign unused_tpm_cmdaddr = ^{tpm_cmdaddr_rvalid, reg2hw.tpm_cmd_addr};

  assign tpm_cmdaddr_rready  = reg2hw.tpm_cmd_addr.cmd.re;
  assign hw2reg.tpm_cmd_addr = '{
    addr: tpm_cmdaddr_rdata[23: 0],
    cmd:  tpm_cmdaddr_rdata[31:24]
  };

  // Read FIFO (write by SW)
  logic  unused_tpm_rdfifo;
  assign unused_tpm_rdfifo= tpm_rdfifo_wready;

  assign tpm_rdfifo_wvalid = reg2hw.tpm_read_fifo.qe;
  assign tpm_rdfifo_wdata  = reg2hw.tpm_read_fifo.q;

  // END: TPM over SPI --------------------------------------------------------

  ////////////////////
  // Common modules //
  ////////////////////

  logic [SramDw-1:0] sys_sram_l2m_fw_wmask[2];
  assign tl_sram_egress_h2d = tl_sram_h2d[SPI_DEVICE_EGRESS_BUFFER_IDX];
  assign tl_sram_d2h[SPI_DEVICE_EGRESS_BUFFER_IDX] = tl_sram_egress_d2h;
  assign tl_sram_ingress_h2d = tl_sram_h2d[SPI_DEVICE_INGRESS_BUFFER_IDX];
  assign tl_sram_d2h[SPI_DEVICE_INGRESS_BUFFER_IDX] = tl_sram_ingress_d2h;

  tlul_adapter_sram #(
    .SramAw      (SramAw),
    .SramDw      (SramDw),
    .Outstanding (1),
    .ErrOnRead   (1), // write-only memory window
    .ByteAccess  (0)
  ) u_tlul2sram_egress (
    .clk_i,
    .rst_ni,

    .tl_i                       (tl_sram_egress_h2d),
    .tl_o                       (tl_sram_egress_d2h),
    .en_ifetch_i                (prim_mubi_pkg::MuBi4False),
    .req_o                      (sys_sram_l2m[SysSramFwEgress].req),
    .req_type_o                 (),
    .gnt_i                      (sys_sram_fw_gnt[SPI_DEVICE_EGRESS_BUFFER_IDX]),
    .we_o                       (sys_sram_l2m[SysSramFwEgress].we),
    .addr_o                     (sys_sram_l2m[SysSramFwEgress].addr),
    .wdata_o                    (sys_sram_l2m[SysSramFwEgress].wdata),
    .wmask_o                    (sys_sram_l2m_fw_wmask[SPI_DEVICE_EGRESS_BUFFER_IDX]),  // Not used
    .intg_error_o               (),
    .rdata_i                    (sys_sram_m2l[SysSramFwEgress].rdata),
    .rvalid_i                   (sys_sram_m2l[SysSramFwEgress].rvalid),
    .rerror_i                   (sys_sram_m2l[SysSramFwEgress].rerror),
    .compound_txn_in_progress_o (),
    .readback_en_i              (prim_mubi_pkg::MuBi4False),
    .readback_error_o           (),
    .wr_collision_i             (1'b0),
    .write_pending_i            (1'b0)
  );

  tlul_adapter_sram #(
    .SramAw      (SramAw),
    .SramDw      (SramDw),
    .Outstanding (1),
    .ErrOnWrite  (1), // read-only memory window
    .ByteAccess  (0)
  ) u_tlul2sram_ingress (
    .clk_i,
    .rst_ni,

    .tl_i                       (tl_sram_ingress_h2d),
    .tl_o                       (tl_sram_ingress_d2h),
    .en_ifetch_i                (prim_mubi_pkg::MuBi4False),
    .req_o                      (sys_sram_l2m[SysSramFwIngress].req),
    .req_type_o                 (),
    .gnt_i                      (sys_sram_fw_gnt[SPI_DEVICE_INGRESS_BUFFER_IDX]),
    .we_o                       (sys_sram_l2m[SysSramFwIngress].we),
    .addr_o                     (sys_sram_l2m[SysSramFwIngress].addr),
    .wdata_o                    (sys_sram_l2m[SysSramFwIngress].wdata),
    .wmask_o                    (sys_sram_l2m_fw_wmask[SPI_DEVICE_INGRESS_BUFFER_IDX]),  // Not used
    .intg_error_o               (),
    .rdata_i                    (sys_sram_m2l[SysSramFwIngress].rdata),
    .rvalid_i                   (sys_sram_m2l[SysSramFwIngress].rvalid),
    .rerror_i                   (sys_sram_m2l[SysSramFwIngress].rerror),
    .compound_txn_in_progress_o (),
    .readback_en_i              (prim_mubi_pkg::MuBi4False),
    .readback_error_o           (),
    .wr_collision_i             (1'b0),
    .write_pending_i            (1'b0)
  );
  assign sys_sram_l2m[SysSramFwEgress].wstrb =
    sram_mask2strb(sys_sram_l2m_fw_wmask[SPI_DEVICE_EGRESS_BUFFER_IDX]);
  assign sys_sram_l2m[SysSramFwIngress].wstrb =
    sram_mask2strb(sys_sram_l2m_fw_wmask[SPI_DEVICE_INGRESS_BUFFER_IDX]);

  logic sys_sram_hw_req;
  always_comb begin
    sys_sram_hw_req = 1'b0;
    for (int unsigned i = 0; i < SysSramEnd; i++) begin
      if ((i != SysSramFwEgress) && (i != SysSramFwIngress)) begin
        sys_sram_hw_req |= sys_sram_l2m[i].req;
      end
    end
  end

  always_comb begin
    for (int unsigned i = 0; i < SysSramEnd; i++) begin
      sys_sram_req[i] = sys_sram_l2m[i].req;
    end
    if (sys_sram_hw_req) begin
      // Fixed low priority. (Discussed in #10065)
      // When HW requests the SRAM access, lower the SW requests (and grant)
      sys_sram_req[SysSramFwEgress] = 1'b0;
      sys_sram_fw_gnt[SPI_DEVICE_EGRESS_BUFFER_IDX] = 1'b0;
      sys_sram_req[SysSramFwIngress] = 1'b0;
      sys_sram_fw_gnt[SPI_DEVICE_INGRESS_BUFFER_IDX] = 1'b0;
    end else begin
      sys_sram_fw_gnt[SPI_DEVICE_EGRESS_BUFFER_IDX] = sys_sram_gnt[SysSramFwEgress];
      sys_sram_fw_gnt[SPI_DEVICE_INGRESS_BUFFER_IDX] = sys_sram_gnt[SysSramFwIngress];
    end
  end

  for (genvar i = 0 ; i < SysSramEnd ; i++) begin : g_sram_connect
    assign sys_sram_addr  [i] = sys_sram_l2m[i].addr;
    assign sys_sram_write [i] = sys_sram_l2m[i].we;
    assign sys_sram_wdata [i] = sys_sram_l2m[i].wdata;
    assign sys_sram_wmask [i] = sram_strb2mask(sys_sram_l2m[i].wstrb);

    assign sys_sram_m2l[i].rvalid = sys_sram_rvalid[i];
    assign sys_sram_m2l[i].rdata  = sys_sram_rdata[i];
    assign sys_sram_m2l[i].rerror = sys_sram_rerror[i];

    // There is only ever a single source of requests on the SYS port (SW), so
    // requests should always be granted.
    `ASSERT(ReqAlwaysAccepted_A, sys_sram_req[i] |-> sys_sram_gnt[i])
  end : g_sram_connect

  prim_sram_arbiter #(
    .N      (SysSramEnd),
    .SramDw (SramDw),
    .SramAw (SramAw),

    .EnMask (1'b 1)
  ) u_sys_sram_arbiter (
    .clk_i,
    .rst_ni,

    .req_i       (sys_sram_req),
    .req_addr_i  (sys_sram_addr),
    .req_write_i (sys_sram_write),
    .req_wdata_i (sys_sram_wdata),
    .req_wmask_i (sys_sram_wmask),
    .gnt_o       (sys_sram_gnt),

    .rsp_rvalid_o (sys_sram_rvalid),
    .rsp_rdata_o  (sys_sram_rdata),
    .rsp_error_o  (sys_sram_rerror),

    .sram_req_o    (mem_a_req),
    .sram_addr_o   (mem_a_addr),
    .sram_write_o  (mem_a_write),
    .sram_wdata_o  (mem_a_wdata),
    .sram_wmask_o  (mem_a_wmask),
    .sram_rvalid_i (mem_a_rvalid),
    .sram_rdata_i  (mem_a_rdata),
    .sram_rerror_i (mem_a_rerror)
  );

  // SRAM Wrapper
  // The SRAM should only be reset if both modes are inactive.
  logic spi_dpram_rst_n;
  assign spi_dpram_rst_n = tpm_rst_in_n | rst_spi_in_n;

  assign mem_b_req   = mem_b_l2m.req;
  assign mem_b_write = mem_b_l2m.we;
  assign mem_b_addr  = mem_b_l2m.addr;
  assign mem_b_wdata = mem_b_l2m.wdata;
  assign mem_b_wmask = sram_strb2mask(mem_b_l2m.wstrb);

  assign mem_b_m2l.rvalid = mem_b_rvalid;
  assign mem_b_m2l.rdata  = mem_b_rdata;
  assign mem_b_m2l.rerror = mem_b_rerror;

  spid_dpram #(
    .SramType            (SramType),
    .EnableECC           (0),
    .EnableParity        (1),
    .EnableInputPipeline (0),
    .EnableOutputPipeline(0)
  ) u_spid_dpram (
    .clk_sys_i      (clk_i),
    .rst_sys_ni     (rst_ni),

    .clk_spi_i      (clk_spi_in_buf),
    .rst_spi_ni     (spi_dpram_rst_n),

    .sys_req_i      (mem_a_req),
    .sys_write_i    (mem_a_write),
    .sys_addr_i     (mem_a_addr),
    .sys_wdata_i    (mem_a_wdata),
    .sys_wmask_i    (mem_a_wmask),
    .sys_rvalid_o   (mem_a_rvalid),
    .sys_rdata_o    (mem_a_rdata),
    .sys_rerror_o   (mem_a_rerror),

    .spi_req_i      (mem_b_req),
    .spi_write_i    (mem_b_write),
    .spi_addr_i     (mem_b_addr),
    .spi_wdata_i    (mem_b_wdata),
    .spi_wmask_i    (mem_b_wmask),
    .spi_rvalid_o   (mem_b_rvalid),
    .spi_rdata_o    (mem_b_rdata),
    .spi_rerror_o   (mem_b_rerror),

    .cfg_i          (ram_cfg_i)
  );

  // Register module
  logic [NumAlerts-1:0] alert_test, alerts;
  spi_device_reg_top u_reg (
    .clk_i,
    .rst_ni,

    .tl_i (tl_i),
    .tl_o (tl_o),

    .tl_win_o (tl_sram_h2d),
    .tl_win_i (tl_sram_d2h),

    .reg2hw,
    .hw2reg,

    // SEC_CM: BUS.INTEGRITY
    .intg_err_o (alerts[0])
  );

  // Alerts
  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  // make sure scanmode_i is never X (including during reset)
  `ASSERT_KNOWN(scanmodeKnown, scanmode_i, clk_i, 0)
  `ASSERT_KNOWN(CioSdoEnOKnown, cio_sd_en_o)

  `ASSERT_KNOWN(IntrUploadCmdfifoNotEmptyOKnown,
                intr_upload_cmdfifo_not_empty_o)
  `ASSERT_KNOWN(IntrUploadPayloadNotEmptyOKnown,
                intr_upload_payload_not_empty_o)
  `ASSERT_KNOWN(IntrUploadPayloadOverflowOKnown,
                intr_upload_payload_overflow_o)
  `ASSERT_KNOWN(IntrReadbufWatermarkOKnown,  intr_readbuf_watermark_o)
  `ASSERT_KNOWN(IntrReadbufFlipOKnown,       intr_readbuf_flip_o)
  `ASSERT_KNOWN(IntrTpmHeaderNotEmptyOKnown, intr_tpm_header_not_empty_o)
  `ASSERT_KNOWN(IntrTpmRdfifoCmdEndOKnown, intr_tpm_rdfifo_cmd_end_o)
  `ASSERT_KNOWN(IntrTpmRdfifoDropOKnown, intr_tpm_rdfifo_drop_o)

  `ASSERT_KNOWN(AlertKnownO_A,         alert_tx_o)

  // Assume the tpm_en is set when TPM transaction is idle.
  `ASSUME(TpmEnableWhenTpmCsbIdle_M, $rose(cfg_tpm_en) |-> cio_tpm_csb_i)

  // When CSBs are inactive, spi_device shouldn't drive the CIO
  `ASSERT(CioSdoEnOffWhenInactive, cio_csb_i && cio_tpm_csb_i -> cio_sd_en_o === 0)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Serial Peripheral Interface (SPI) Host module.
//
//


module spi_host
  import spi_host_reg_pkg::*;
#(
  
  // Param list
  parameter logic ByteOrder = 1,
  parameter int NumCS = 1,
  parameter int TxDepth = 72,
  parameter int RxDepth = 64,
  parameter int CmdDepth = 4,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 6,

  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}}
) (
  input              clk_i,
  input              rst_ni,

  // Register interface
  input              tlul_pkg::tl_h2d_t tl_i,
  output             tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // SPI Interface
  output logic             cio_sck_o,
  output logic             cio_sck_en_o,
  output logic [NumCS-1:0] cio_csb_o,
  output logic [NumCS-1:0] cio_csb_en_o,
  output logic [3:0]       cio_sd_o,
  output logic [3:0]       cio_sd_en_o,
  input        [3:0]       cio_sd_i,

  // Passthrough interface
  input  spi_device_pkg::passthrough_req_t passthrough_i,
  output spi_device_pkg::passthrough_rsp_t passthrough_o,

  output logic             intr_error_o,
  output logic             intr_spi_event_o
);

  import spi_host_cmd_pkg::*;

  spi_host_reg2hw_t reg2hw;
  spi_host_hw2reg_t hw2reg;

  tlul_pkg::tl_h2d_t fifo_win_h2d [2];
  tlul_pkg::tl_d2h_t fifo_win_d2h [2];

  // Register module
  logic [NumAlerts-1:0] alert_test, alerts;
  spi_host_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .tl_i       (tl_i),
    .tl_o       (tl_o),
    .tl_win_o   (fifo_win_h2d),
    .tl_win_i   (fifo_win_d2h),
    .reg2hw,
    .hw2reg,
    // SEC_CM: BUS.INTEGRITY
    .intg_err_o (alerts[0])
  );

  // Alerts
  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  logic             sck;
  logic [NumCS-1:0] csb;
  logic [3:0]       sd_out;
  logic [3:0]       sd_en, sd_en_core;
  logic [3:0]       sd_i;
  logic             output_en;

  assign output_en = reg2hw.control.output_en;

  assign sd_en     = output_en ? sd_en_core : 4'h0;

  if (NumCS == 1) begin : gen_passthrough_implementation
    logic passthrough_en;
    assign passthrough_en  = passthrough_i.passthrough_en;

    logic        pt_sck;
    logic        pt_sck_en;
    logic [0:0]  pt_csb;
    logic [0:0]  pt_csb_en;
    logic [3:0]  pt_sd_out;
    logic [3:0]  pt_sd_en;

    assign pt_sck       = passthrough_i.sck;
    assign pt_sck_en    = passthrough_i.sck_en;
    assign pt_csb[0]    = passthrough_i.csb;
    assign pt_csb_en[0] = passthrough_i.csb_en;
    assign pt_sd_out    = passthrough_i.s;
    assign pt_sd_en     = passthrough_i.s_en;

    assign cio_sck_o    = passthrough_en ? pt_sck    : sck;
    assign cio_sck_en_o = passthrough_en ? pt_sck_en : output_en;
    assign cio_csb_o    = passthrough_en ? pt_csb    : csb;
    assign cio_csb_en_o = passthrough_en ? pt_csb_en : output_en;
    assign cio_sd_o     = passthrough_en ? pt_sd_out : sd_out;
    assign cio_sd_en_o  = passthrough_en ? pt_sd_en  : sd_en;

  end                   : gen_passthrough_implementation
  else begin            : gen_passthrough_ignore
     // Passthrough only supported for instances with one CSb line
    `ASSERT(PassthroughNumCSCompat_A, !passthrough_i.passthrough_en, clk_i, rst_ni)

    assign cio_sck_o    = sck;
    assign cio_sck_en_o = output_en;
    assign cio_csb_o    = csb;
    assign cio_csb_en_o = {NumCS{output_en}};
    assign cio_sd_o     = sd_out;
    assign cio_sd_en_o  = sd_en;

    logic       unused_pt_en;
    logic       unused_pt_sck;
    logic       unused_pt_sck_en;
    logic       unused_pt_csb;
    logic       unused_pt_csb_en;
    logic [3:0] unused_pt_sd_out;
    logic [3:0] unused_pt_sd_en;

    assign unused_pt_en     = passthrough_i.passthrough_en;
    assign unused_pt_sck    = passthrough_i.sck;
    assign unused_pt_sck_en = passthrough_i.sck_en;
    assign unused_pt_csb    = passthrough_i.csb;
    assign unused_pt_csb_en = passthrough_i.csb_en;
    assign unused_pt_sd_out = passthrough_i.s;
    assign unused_pt_sd_en  = passthrough_i.s_en;

  end                   : gen_passthrough_ignore

  assign passthrough_o.s = cio_sd_i;
  assign sd_i            = cio_sd_i;

  assign hw2reg.status.byteorder.d  = ByteOrder;
  assign hw2reg.status.byteorder.de = 1'b1;

  logic command_valid;
  logic core_command_valid;
  logic command_busy;
  logic core_command_ready;

  command_t core_command, command;
  logic error_csid_inval;
  logic error_cmd_inval;
  logic error_busy;
  logic test_csid_inval;
  logic test_dir_inval;
  logic test_speed_inval;

  assign test_csid_inval  = (reg2hw.csid.q >= NumCS);

  always_comb begin
    test_speed_inval           = 1'b1;
    test_dir_inval             = 1'b1;
    unique case (reg2hw.command.speed.q)
      Standard: begin
        test_dir_inval   = 1'b0;
        test_speed_inval = 1'b0;
      end
      Dual, Quad: begin
        test_dir_inval   = (reg2hw.command.direction.q == Bidir);
        test_speed_inval = 1'b0;
      end
      default: begin
      end
    endcase
  end

  always_comb begin
    command.segment.cmd_rd_en = 1'b0;
    command.segment.cmd_wr_en = 1'b0;
    unique case (reg2hw.command.direction.q)
      RdOnly: begin
        command.segment.cmd_rd_en = 1'b1;
      end
      WrOnly: begin
        command.segment.cmd_wr_en = 1'b1;
      end
      Bidir: begin
        command.segment.cmd_rd_en = 1'b1;
        command.segment.cmd_wr_en = 1'b1;
      end
      default: begin
      end
    endcase
  end

  assign error_csid_inval = command_valid & ~command_busy &
                            test_csid_inval;
  assign error_cmd_inval  = command_valid & ~command_busy &
                            (test_speed_inval | test_dir_inval);

  spi_host_reg_pkg::spi_host_reg2hw_configopts_mreg_t configopts;

  if (NumCS == 1) begin : gen_single_device
    assign configopts   = reg2hw.configopts[0];
    assign command.csid = '0;
  end else begin : gen_multiple_devices
    logic [CSW-1:0] csid;
    assign csid         = (test_csid_inval) ? '0 : reg2hw.csid.q[CSW-1:0];
    assign configopts   = reg2hw.configopts[csid];
    assign command.csid = csid;
  end : gen_multiple_devices

  assign command.configopts.clkdiv   = configopts.clkdiv.q;
  assign command.configopts.csnidle  = configopts.csnidle.q;
  assign command.configopts.csnlead  = configopts.csnlead.q;
  assign command.configopts.csntrail = configopts.csntrail.q;
  assign command.configopts.full_cyc = configopts.fullcyc.q;
  assign command.configopts.cpha     = configopts.cpha.q;
  assign command.configopts.cpol     = configopts.cpol.q;

  assign command.segment.len         = reg2hw.command.len.q;
  assign command.segment.csaat       = reg2hw.command.csaat.q;
  assign command.segment.speed       = reg2hw.command.speed.q;


  logic [3:0] cmd_qes;

  assign cmd_qes = {
      reg2hw.command.len.qe,
      reg2hw.command.speed.qe,
      reg2hw.command.direction.qe,
      reg2hw.command.csaat.qe
  };

  // Any qe pin from COMMAND will suffice.
  assign command_valid = |cmd_qes;

  logic active;
  logic rx_stall;
  logic tx_stall;

  assign hw2reg.status.ready.d    = ~command_busy;
  assign hw2reg.status.active.d   = active;
  assign hw2reg.status.rxstall.d  = rx_stall;
  assign hw2reg.status.txstall.d  = tx_stall;

  assign hw2reg.status.ready.de   = 1'b1;
  assign hw2reg.status.active.de  = 1'b1;
  assign hw2reg.status.rxstall.de = 1'b1;
  assign hw2reg.status.txstall.de = 1'b1;

  logic sw_rst;

  logic [3:0]  cmd_qd;

  spi_host_command_queue #(
    .CmdDepth(CmdDepth)
  ) u_cmd_queue (
    .clk_i,
    .rst_ni,
    .command_i            (command),
    .command_valid_i      (command_valid),
    .command_busy_o       (command_busy),
    .core_command_o       (core_command),
    .core_command_valid_o (core_command_valid),
    .core_command_ready_i (core_command_ready),
    .error_busy_o         (error_busy),
    .qd_o                 (cmd_qd),
    .sw_rst_i             (sw_rst)
  );

  logic [31:0] tx_data;
  logic [3:0]  tx_be;
  logic        tx_valid;
  logic        tx_ready;

  logic [31:0] rx_data;
  logic        rx_valid;
  logic        rx_ready;

  spi_host_window u_window (
    .clk_i,
    .rst_ni,
    .rx_win_i   (fifo_win_h2d[0]),
    .rx_win_o   (fifo_win_d2h[0]),
    .tx_win_i   (fifo_win_h2d[1]),
    .tx_win_o   (fifo_win_d2h[1]),
    .tx_data_o  (tx_data),
    .tx_be_o    (tx_be),
    .tx_valid_o (tx_valid),
    .rx_data_i  (rx_data),
    .rx_ready_o (rx_ready)
  );

  logic [31:0] core_tx_data;
  logic [3:0]  core_tx_be;
  logic        core_tx_valid;
  logic        core_tx_ready;
  logic        core_tx_byte_select_full;

  logic [31:0] core_rx_data;
  logic        core_rx_valid;
  logic        core_rx_ready;

  logic [7:0]  rx_watermark;
  logic [7:0]  tx_watermark;
  logic [7:0]  rx_qd;
  logic [7:0]  tx_qd;

  logic        tx_empty, tx_full, tx_wm;
  logic        rx_empty, rx_full, rx_wm;

  assign rx_watermark = reg2hw.control.rx_watermark.q;
  assign tx_watermark = reg2hw.control.tx_watermark.q;

  assign hw2reg.status.txqd.d    = tx_qd;
  assign hw2reg.status.rxqd.d    = rx_qd;
  assign hw2reg.status.cmdqd.d   = cmd_qd;
  assign hw2reg.status.txwm.d    = tx_wm;
  assign hw2reg.status.rxwm.d    = rx_wm;
  assign hw2reg.status.rxempty.d = rx_empty;
  assign hw2reg.status.txempty.d = tx_empty;
  assign hw2reg.status.rxfull.d  = rx_full;
  assign hw2reg.status.txfull.d  = tx_full;

  assign hw2reg.status.txqd.de    = 1'b1;
  assign hw2reg.status.rxqd.de    = 1'b1;
  assign hw2reg.status.cmdqd.de   = 1'b1;
  assign hw2reg.status.txwm.de    = 1'b1;
  assign hw2reg.status.rxwm.de    = 1'b1;
  assign hw2reg.status.rxempty.de = 1'b1;
  assign hw2reg.status.txempty.de = 1'b1;
  assign hw2reg.status.rxfull.de  = 1'b1;
  assign hw2reg.status.txfull.de  = 1'b1;

  logic error_overflow, error_underflow;
  logic error_access_inval;

  // Since the DATA FIFOs are essentially directly connected to SW registers, it is an error if
  // there is ever a need for flow control.
  assign error_overflow    = tx_valid & ~tx_ready;
  assign error_underflow   = rx_ready & ~rx_valid;
  logic access_valid;
  assign error_access_inval = tx_valid & ~access_valid;

  always_comb begin
    unique case (tx_be)
      4'b1000,
      4'b0100,
      4'b0010,
      4'b0001,
      4'b1100,
      4'b0110,
      4'b0011,
      4'b1111: begin
        access_valid = 1'b1;
      end
      default: begin
        access_valid = 1'b0;
      end
    endcase
  end

  logic tx_valid_checked;
  assign tx_valid_checked = tx_valid & ~error_overflow & ~error_access_inval;

  // Note on ByteOrder and ByteSwapping.
  // ByteOrder == 1 is for Little-Endian transmission (i.e. LSB first), which is acheived by
  // default with the prim_packer_fifo implementation.  Thus we have to swap if Big-Endian
  // transmission is required (i.e. if ByteOrder == 0).
  spi_host_data_fifos #(
    .TxDepth(TxDepth),
    .RxDepth(RxDepth),
    .SwapBytes(~ByteOrder)
  ) u_data_fifos (
    .clk_i,
    .rst_ni,

    .tx_data_i                  (tx_data),
    .tx_be_i                    (tx_be),
    .tx_valid_i                 (tx_valid_checked),
    .tx_ready_o                 (tx_ready),
    .tx_watermark_i             (tx_watermark),

    .core_tx_data_o             (core_tx_data),
    .core_tx_be_o               (core_tx_be),
    .core_tx_valid_o            (core_tx_valid),
    .core_tx_ready_i            (core_tx_ready),
    .core_tx_byte_select_full_i (core_tx_byte_select_full),

    .core_rx_data_i             (core_rx_data),
    .core_rx_valid_i            (core_rx_valid),
    .core_rx_ready_o            (core_rx_ready),

    .rx_data_o                  (rx_data),
    .rx_valid_o                 (rx_valid),
    .rx_ready_i                 (rx_ready),
    .rx_watermark_i             (rx_watermark),

    .tx_empty_o                 (tx_empty),
    .tx_full_o                  (tx_full),
    .tx_qd_o                    (tx_qd),
    .tx_wm_o                    (tx_wm),
    .rx_empty_o                 (rx_empty),
    .rx_full_o                  (rx_full),
    .rx_qd_o                    (rx_qd),
    .rx_wm_o                    (rx_wm),

    .sw_rst_i                   (sw_rst)
);

  logic en_sw;
  logic enb_error;
  logic en;

  assign en     = en_sw & ~enb_error;
  assign sw_rst = reg2hw.control.sw_rst.q;
  assign en_sw  = reg2hw.control.spien.q;

  spi_host_core #(
    .NumCS(NumCS)
  ) u_spi_core (
    .clk_i,
    .rst_ni,

    .command_i             (core_command),
    .command_valid_i       (core_command_valid),
    .command_ready_o       (core_command_ready),
    .en_i                  (en),
    .tx_data_i             (core_tx_data),
    .tx_be_i               (core_tx_be),
    .tx_valid_i            (core_tx_valid),
    .tx_ready_o            (core_tx_ready),
    .tx_byte_select_full_o (core_tx_byte_select_full),
    .rx_data_o             (core_rx_data),
    .rx_valid_o            (core_rx_valid),
    .rx_ready_i            (core_rx_ready),
    .sck_o                 (sck),
    .csb_o                 (csb),
    .sd_o                  (sd_out),
    .sd_en_o               (sd_en_core),
    .sd_i,
    .rx_stall_o            (rx_stall),
    .tx_stall_o            (tx_stall),
    .sw_rst_i              (sw_rst),
    .active_o              (active)
  );

  logic event_error;

  logic [5:0] error_vec;
  logic [5:0] error_mask;
  logic [5:0] sw_error_status;

  assign error_vec  = {
      error_access_inval,
      error_csid_inval,
      error_cmd_inval,
      error_underflow,
      error_overflow,
      error_busy
  };

  // This mask dictates what classes of error events are to be escalated to error interrupts.
  // (Assume here that error interrupts will be enabled)
  // Software generated errors can be configured to _not_ generate error events.
  // Bus errors (such as invalid write access) always generate error events and therefore
  // interrupts.
  assign error_mask = {
      1'b1, // invalid access: always an error event
      reg2hw.error_enable.csidinval.q,
      reg2hw.error_enable.cmdinval.q,
      reg2hw.error_enable.underflow.q,
      reg2hw.error_enable.overflow.q,
      reg2hw.error_enable.cmdbusy.q
  };

  assign hw2reg.error_status.accessinval.d  = error_access_inval;
  assign hw2reg.error_status.csidinval.d    = error_csid_inval;
  assign hw2reg.error_status.cmdinval.d     = error_cmd_inval;
  assign hw2reg.error_status.underflow.d    = error_underflow;
  assign hw2reg.error_status.overflow.d     = error_overflow;
  assign hw2reg.error_status.cmdbusy.d      = error_busy;

  // Write the status register whenever the corresponding event occurs.
  // Only clear them from software.
  assign hw2reg.error_status.accessinval.de = error_access_inval;
  assign hw2reg.error_status.csidinval.de   = error_csid_inval;
  assign hw2reg.error_status.cmdinval.de    = error_cmd_inval;
  assign hw2reg.error_status.underflow.de   = error_underflow;
  assign hw2reg.error_status.overflow.de    = error_overflow;
  assign hw2reg.error_status.cmdbusy.de     = error_busy;

  assign sw_error_status[5] = reg2hw.error_status.accessinval.q;
  assign sw_error_status[4] = reg2hw.error_status.csidinval.q;
  assign sw_error_status[3] = reg2hw.error_status.cmdinval.q;
  assign sw_error_status[2] = reg2hw.error_status.underflow.q;
  assign sw_error_status[1] = reg2hw.error_status.overflow.q;
  assign sw_error_status[0] = reg2hw.error_status.cmdbusy.q;

  assign event_error   = |(error_vec & error_mask);
  assign enb_error     = |sw_error_status;

  prim_intr_hw #(.Width(1)) intr_hw_error (
    .clk_i,
    .rst_ni,
    .event_intr_i           (event_error),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.error.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.error.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.error.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.error.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.error.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.error.d),
    .intr_o                 (intr_error_o)
  );

  logic status_spi_event;
  logic status_idle, status_ready, status_tx_wm, status_rx_wm, status_tx_empty, status_rx_full;
  logic [5:0] event_vector;
  logic [5:0] event_mask;

  assign status_idle     = ~active;
  assign status_ready    = ~command_busy;
  assign status_tx_wm    = tx_wm;
  assign status_rx_wm    = rx_wm;
  assign status_tx_empty = tx_empty;
  assign status_rx_full  = rx_full;

  assign event_vector = {
    status_idle,
    status_ready,
    status_tx_wm,
    status_rx_wm,
    status_tx_empty,
    status_rx_full
  };

  assign event_mask = {
    reg2hw.event_enable.idle.q,
    reg2hw.event_enable.ready.q,
    reg2hw.event_enable.txwm.q,
    reg2hw.event_enable.rxwm.q,
    reg2hw.event_enable.txempty.q,
    reg2hw.event_enable.rxfull.q
  };

  // Qualify interrupt sources individually with dedicated mask register CSR.EVENT_ENABLE
  assign status_spi_event = |(event_vector & event_mask);

  prim_intr_hw #(
    .Width(1),
    .IntrT("Status")
  ) intr_hw_spi_event (
    .clk_i,
    .rst_ni,
    .event_intr_i           (status_spi_event),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.spi_event.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.spi_event.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.spi_event.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.spi_event.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.spi_event.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.spi_event.d),
    .intr_o                 (intr_spi_event_o)
  );


  `ASSERT_KNOWN(TlDValidKnownO_A, tl_o.d_valid)
  `ASSERT_KNOWN(TlAReadyKnownO_A, tl_o.a_ready)
  `ASSERT_KNOWN(AlertKnownO_A, alert_tx_o)
  `ASSERT_KNOWN(CioSckKnownO_A, cio_sck_o)
  `ASSERT_KNOWN(CioSckEnKnownO_A, cio_sck_en_o)
  `ASSERT_KNOWN(CioCsbKnownO_A, cio_csb_o)
  `ASSERT_KNOWN(CioCsbEnKnownO_A, cio_csb_en_o)
  `ASSERT_KNOWN_IF(CioSdKnownO_A, cio_sd_o, !passthrough_i.passthrough_en |
    (passthrough_i.passthrough_en && passthrough_i.csb_en && !passthrough_i.csb),
    passthrough_i.sck_en & passthrough_i.sck)
  `ASSERT_KNOWN(CioSdEnKnownO_A, cio_sd_en_o)
  `ASSERT_KNOWN(IntrSpiEventKnownO_A, intr_spi_event_o)
  `ASSERT_KNOWN(IntrErrorKnownO_A, intr_error_o)

  // passthrough_o.s is passed through to spi_device, it may contain unknown data,
  // but the unknown data won't be used based on the SPI protocol.
  // Hence, instead of checking known data, here does a connectivity check.
  `ASSERT(PassthroughConn_A, passthrough_o.s === cio_sd_i)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule : spi_host

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// ------------------- W A R N I N G: A U T O - G E N E R A T E D   C O D E !! -------------------//
// PLEASE DO NOT HAND-EDIT THIS FILE. IT HAS BEEN AUTO-GENERATED WITH THE FOLLOWING COMMAND:
//
// util/topgen.py -t hw/top_earlgrey/data/top_earlgrey.hjson \
//                -o hw/top_earlgrey/ \
//                --rnd_cnst_seed \
//                1017106219537032642877583828875051302543807092889754935647094601236425074047

module top_earlgrey #(
  // Manually defined parameters

  // Auto-inferred parameters
  // parameters for uart0
  // parameters for uart1
  // parameters for uart2
  // parameters for uart3
  // parameters for gpio
  parameter bit GpioGpioAsyncOn = 1,
  // parameters for spi_device
  parameter spi_device_pkg::sram_type_e SpiDeviceSramType = spi_device_pkg::DefaultSramType,
  // parameters for i2c0
  parameter int I2c0InputDelayCycles = 0,
  // parameters for i2c1
  parameter int I2c1InputDelayCycles = 0,
  // parameters for i2c2
  parameter int I2c2InputDelayCycles = 0,
  // parameters for pattgen
  // parameters for rv_timer
  // parameters for otp_ctrl
  parameter OtpCtrlMemInitFile = "",
  // parameters for lc_ctrl
  parameter bit SecLcCtrlVolatileRawUnlockEn = top_pkg::SecVolatileRawUnlockEn,
  parameter logic [15:0] LcCtrlSiliconCreatorId = 16'h 4001,
  parameter logic [15:0] LcCtrlProductId = 16'h 0002,
  parameter logic [7:0] LcCtrlRevisionId = 8'h 01,
  parameter logic [31:0] LcCtrlIdcodeValue = jtag_id_pkg::LC_CTRL_JTAG_IDCODE,
  // parameters for alert_handler
  // parameters for spi_host0
  // parameters for spi_host1
  // parameters for usbdev
  parameter bit UsbdevStub = 0,
  parameter int UsbdevRcvrWakeTimeUs = 100,
  // parameters for pwrmgr_aon
  // parameters for rstmgr_aon
  parameter bit SecRstmgrAonCheck = 1'b1,
  parameter int SecRstmgrAonMaxSyncDelay = 2,
  // parameters for clkmgr_aon
  // parameters for sysrst_ctrl_aon
  // parameters for adc_ctrl_aon
  // parameters for pwm_aon
  // parameters for pinmux_aon
  parameter bit SecPinmuxAonVolatileRawUnlockEn = top_pkg::SecVolatileRawUnlockEn,
  parameter pinmux_pkg::target_cfg_t PinmuxAonTargetCfg = pinmux_pkg::DefaultTargetCfg,
  // parameters for aon_timer_aon
  // parameters for sensor_ctrl_aon
  // parameters for sram_ctrl_ret_aon
  parameter bit SramCtrlRetAonInstrExec = 0,
  // parameters for flash_ctrl
  parameter bit SecFlashCtrlScrambleEn = 1,
  parameter int FlashCtrlProgFifoDepth = 4,
  parameter int FlashCtrlRdFifoDepth = 16,
  // parameters for rv_dm
  parameter logic [31:0] RvDmIdcodeValue = jtag_id_pkg::RV_DM_JTAG_IDCODE,
  // parameters for rv_plic
  // parameters for aes
  parameter bit SecAesMasking = 1,
  parameter aes_pkg::sbox_impl_e SecAesSBoxImpl = aes_pkg::SBoxImplDom,
  parameter int unsigned SecAesStartTriggerDelay = 0,
  parameter bit SecAesAllowForcingMasks = 1'b0,
  parameter bit SecAesSkipPRNGReseeding = 1'b0,
  // parameters for hmac
  // parameters for kmac
  parameter bit KmacEnMasking = 1,
  parameter bit KmacSwKeyMasked = 0,
  parameter int SecKmacCmdDelay = 0,
  parameter bit SecKmacIdleAcceptSwMsg = 0,
  // parameters for otbn
  parameter bit OtbnStub = 0,
  parameter otbn_pkg::regfile_e OtbnRegFile = otbn_pkg::RegFileFF,
  parameter bit SecOtbnMuteUrnd = 0,
  parameter bit SecOtbnSkipUrndReseedAtStart = 0,
  // parameters for keymgr
  parameter bit KeymgrUseOtpSeedsInsteadOfFlash = 0,
  parameter bit KeymgrKmacEnMasking = 1,
  // parameters for csrng
  parameter aes_pkg::sbox_impl_e CsrngSBoxImpl = aes_pkg::SBoxImplCanright,
  // parameters for entropy_src
  parameter int EntropySrcEsFifoDepth = 3,
  parameter int unsigned EntropySrcDistrFifoDepth = 2,
  parameter bit EntropySrcStub = 0,
  // parameters for edn0
  // parameters for edn1
  // parameters for sram_ctrl_main
  parameter bit SramCtrlMainInstrExec = 1,
  // parameters for rom_ctrl
  parameter RomCtrlBootRomInitFile = "",
  parameter bit SecRomCtrlDisableScrambling = 1'b0,
  // parameters for rv_core_ibex
  parameter bit RvCoreIbexPMPEnable = 1,
  parameter int unsigned RvCoreIbexPMPGranularity = 0,
  parameter int unsigned RvCoreIbexPMPNumRegions = 16,
  parameter int unsigned RvCoreIbexMHPMCounterNum = 2,
  parameter int unsigned RvCoreIbexMHPMCounterWidth = 32,
  parameter bit RvCoreIbexRV32E = 0,
  parameter ibex_pkg::rv32m_e RvCoreIbexRV32M = ibex_pkg::RV32MSingleCycle,
  parameter ibex_pkg::rv32b_e RvCoreIbexRV32B = ibex_pkg::RV32BOTEarlGrey,
  parameter ibex_pkg::regfile_e RvCoreIbexRegFile = ibex_pkg::RegFileFF,
  parameter bit RvCoreIbexBranchTargetALU = 1,
  parameter bit RvCoreIbexWritebackStage = 1,
  parameter bit RvCoreIbexICache = 1,
  parameter bit RvCoreIbexICacheECC = 1,
  parameter bit RvCoreIbexICacheScramble = 1,
  parameter bit RvCoreIbexBranchPredictor = 0,
  parameter bit RvCoreIbexDbgTriggerEn = 1,
  parameter int RvCoreIbexDbgHwBreakNum = 4,
  parameter bit RvCoreIbexSecureIbex = 1,
  parameter int unsigned RvCoreIbexDmHaltAddr =
      tl_main_pkg::ADDR_SPACE_RV_DM__MEM + dm::HaltAddress[31:0],
  parameter int unsigned RvCoreIbexDmExceptionAddr =
      tl_main_pkg::ADDR_SPACE_RV_DM__MEM + dm::ExceptionAddress[31:0],
  parameter bit RvCoreIbexPipeLine = 0
) (
  // Multiplexed I/O
  input        [46:0] mio_in_i,
  output logic [46:0] mio_out_o,
  output logic [46:0] mio_oe_o,
  // Dedicated I/O
  input        [15:0] dio_in_i,
  output logic [15:0] dio_out_o,
  output logic [15:0] dio_oe_o,

  // pad attributes to padring
  output prim_pad_wrapper_pkg::pad_attr_t [pinmux_reg_pkg::NMioPads-1:0] mio_attr_o,
  output prim_pad_wrapper_pkg::pad_attr_t [pinmux_reg_pkg::NDioPads-1:0] dio_attr_o,


  // Inter-module Signal External type
  output ast_pkg::adc_ast_req_t       adc_req_o,
  input  ast_pkg::adc_ast_rsp_t       adc_rsp_i,
  input  edn_pkg::edn_req_t       ast_edn_req_i,
  output edn_pkg::edn_rsp_t       ast_edn_rsp_o,
  output lc_ctrl_pkg::lc_tx_t       ast_lc_dft_en_o,
  input  ast_pkg::ast_obs_ctrl_t       obs_ctrl_i,
  input  prim_ram_1p_pkg::ram_1p_cfg_t       ram_1p_cfg_i,
  input  prim_ram_2p_pkg::ram_2p_cfg_t       spi_ram_2p_cfg_i,
  input  prim_ram_1p_pkg::ram_1p_cfg_t       usb_ram_1p_cfg_i,
  input  prim_rom_pkg::rom_cfg_t       rom_cfg_i,
  output prim_mubi_pkg::mubi4_t       clk_main_jitter_en_o,
  output prim_mubi_pkg::mubi4_t       io_clk_byp_req_o,
  input  prim_mubi_pkg::mubi4_t       io_clk_byp_ack_i,
  output prim_mubi_pkg::mubi4_t       all_clk_byp_req_o,
  input  prim_mubi_pkg::mubi4_t       all_clk_byp_ack_i,
  output prim_mubi_pkg::mubi4_t       hi_speed_sel_o,
  input  prim_mubi_pkg::mubi4_t       div_step_down_req_i,
  input  prim_mubi_pkg::mubi4_t       calib_rdy_i,
  input  prim_mubi_pkg::mubi4_t       flash_bist_enable_i,
  input  logic       flash_power_down_h_i,
  input  logic       flash_power_ready_h_i,
  inout   [1:0] flash_test_mode_a_io,
  inout         flash_test_voltage_h_io,
  output logic [7:0] flash_obs_o,
  output entropy_src_pkg::entropy_src_rng_req_t       es_rng_req_o,
  input  entropy_src_pkg::entropy_src_rng_rsp_t       es_rng_rsp_i,
  output logic       es_rng_fips_o,
  output tlul_pkg::tl_h2d_t       ast_tl_req_o,
  input  tlul_pkg::tl_d2h_t       ast_tl_rsp_i,
  output pinmux_pkg::dft_strap_test_req_t       dft_strap_test_o,
  input  logic       dft_hold_tap_sel_i,
  output logic       usb_dp_pullup_en_o,
  output logic       usb_dn_pullup_en_o,
  output pwrmgr_pkg::pwr_ast_req_t       pwrmgr_ast_req_o,
  input  pwrmgr_pkg::pwr_ast_rsp_t       pwrmgr_ast_rsp_i,
  output otp_ctrl_pkg::otp_ast_req_t       otp_ctrl_otp_ast_pwr_seq_o,
  input  otp_ctrl_pkg::otp_ast_rsp_t       otp_ctrl_otp_ast_pwr_seq_h_i,
  inout         otp_ext_voltage_h_io,
  output logic [7:0] otp_obs_o,
  input  logic [1:0] por_n_i,
  input  logic [31:0] fpga_info_i,
  input  ast_pkg::ast_alert_req_t       sensor_ctrl_ast_alert_req_i,
  output ast_pkg::ast_alert_rsp_t       sensor_ctrl_ast_alert_rsp_o,
  input  ast_pkg::ast_status_t       sensor_ctrl_ast_status_i,
  input  logic [8:0] ast2pinmux_i,
  input  prim_mubi_pkg::mubi4_t       ast_init_done_i,
  output prim_pad_wrapper_pkg::pad_attr_t [3:0] sensor_ctrl_manual_pad_attr_o,
  output logic       sck_monitor_o,
  input  logic       usbdev_usb_rx_d_i,
  output logic       usbdev_usb_tx_d_o,
  output logic       usbdev_usb_tx_se0_o,
  output logic       usbdev_usb_tx_use_d_se0_o,
  output logic       usbdev_usb_rx_enable_o,
  output logic       usbdev_usb_ref_val_o,
  output logic       usbdev_usb_ref_pulse_o,


  // All externally supplied clocks
  input clk_main_i,
  input clk_io_i,
  input clk_usb_i,
  input clk_aon_i,

  // All clocks forwarded to ast
  output clkmgr_pkg::clkmgr_out_t clks_ast_o,
  output rstmgr_pkg::rstmgr_out_t rsts_ast_o,

  input                      scan_rst_ni, // reset used for test mode
  input                      scan_en_i,
  input prim_mubi_pkg::mubi4_t scanmode_i   // lc_ctrl_pkg::On for Scan
);

  import tlul_pkg::*;
  import top_pkg::*;
  import tl_main_pkg::*;
  import top_earlgrey_pkg::*;
  // Compile-time random constants
  import top_earlgrey_rnd_cnst_pkg::*;

  // Signals
  logic [56:0] mio_p2d;
  logic [74:0] mio_d2p;
  logic [74:0] mio_en_d2p;
  logic [15:0] dio_p2d;
  logic [15:0] dio_d2p;
  logic [15:0] dio_en_d2p;
  // uart0
  logic        cio_uart0_rx_p2d;
  logic        cio_uart0_tx_d2p;
  logic        cio_uart0_tx_en_d2p;
  // uart1
  logic        cio_uart1_rx_p2d;
  logic        cio_uart1_tx_d2p;
  logic        cio_uart1_tx_en_d2p;
  // uart2
  logic        cio_uart2_rx_p2d;
  logic        cio_uart2_tx_d2p;
  logic        cio_uart2_tx_en_d2p;
  // uart3
  logic        cio_uart3_rx_p2d;
  logic        cio_uart3_tx_d2p;
  logic        cio_uart3_tx_en_d2p;
  // gpio
  logic [31:0] cio_gpio_gpio_p2d;
  logic [31:0] cio_gpio_gpio_d2p;
  logic [31:0] cio_gpio_gpio_en_d2p;
  // spi_device
  logic        cio_spi_device_sck_p2d;
  logic        cio_spi_device_csb_p2d;
  logic        cio_spi_device_tpm_csb_p2d;
  logic [3:0]  cio_spi_device_sd_p2d;
  logic [3:0]  cio_spi_device_sd_d2p;
  logic [3:0]  cio_spi_device_sd_en_d2p;
  // i2c0
  logic        cio_i2c0_sda_p2d;
  logic        cio_i2c0_scl_p2d;
  logic        cio_i2c0_sda_d2p;
  logic        cio_i2c0_sda_en_d2p;
  logic        cio_i2c0_scl_d2p;
  logic        cio_i2c0_scl_en_d2p;
  // i2c1
  logic        cio_i2c1_sda_p2d;
  logic        cio_i2c1_scl_p2d;
  logic        cio_i2c1_sda_d2p;
  logic        cio_i2c1_sda_en_d2p;
  logic        cio_i2c1_scl_d2p;
  logic        cio_i2c1_scl_en_d2p;
  // i2c2
  logic        cio_i2c2_sda_p2d;
  logic        cio_i2c2_scl_p2d;
  logic        cio_i2c2_sda_d2p;
  logic        cio_i2c2_sda_en_d2p;
  logic        cio_i2c2_scl_d2p;
  logic        cio_i2c2_scl_en_d2p;
  // pattgen
  logic        cio_pattgen_pda0_tx_d2p;
  logic        cio_pattgen_pda0_tx_en_d2p;
  logic        cio_pattgen_pcl0_tx_d2p;
  logic        cio_pattgen_pcl0_tx_en_d2p;
  logic        cio_pattgen_pda1_tx_d2p;
  logic        cio_pattgen_pda1_tx_en_d2p;
  logic        cio_pattgen_pcl1_tx_d2p;
  logic        cio_pattgen_pcl1_tx_en_d2p;
  // rv_timer
  // otp_ctrl
  logic [7:0]  cio_otp_ctrl_test_d2p;
  logic [7:0]  cio_otp_ctrl_test_en_d2p;
  // lc_ctrl
  // alert_handler
  // spi_host0
  logic [3:0]  cio_spi_host0_sd_p2d;
  logic        cio_spi_host0_sck_d2p;
  logic        cio_spi_host0_sck_en_d2p;
  logic        cio_spi_host0_csb_d2p;
  logic        cio_spi_host0_csb_en_d2p;
  logic [3:0]  cio_spi_host0_sd_d2p;
  logic [3:0]  cio_spi_host0_sd_en_d2p;
  // spi_host1
  logic [3:0]  cio_spi_host1_sd_p2d;
  logic        cio_spi_host1_sck_d2p;
  logic        cio_spi_host1_sck_en_d2p;
  logic        cio_spi_host1_csb_d2p;
  logic        cio_spi_host1_csb_en_d2p;
  logic [3:0]  cio_spi_host1_sd_d2p;
  logic [3:0]  cio_spi_host1_sd_en_d2p;
  // usbdev
  logic        cio_usbdev_sense_p2d;
  logic        cio_usbdev_usb_dp_p2d;
  logic        cio_usbdev_usb_dn_p2d;
  logic        cio_usbdev_usb_dp_d2p;
  logic        cio_usbdev_usb_dp_en_d2p;
  logic        cio_usbdev_usb_dn_d2p;
  logic        cio_usbdev_usb_dn_en_d2p;
  // pwrmgr_aon
  // rstmgr_aon
  // clkmgr_aon
  // sysrst_ctrl_aon
  logic        cio_sysrst_ctrl_aon_ac_present_p2d;
  logic        cio_sysrst_ctrl_aon_key0_in_p2d;
  logic        cio_sysrst_ctrl_aon_key1_in_p2d;
  logic        cio_sysrst_ctrl_aon_key2_in_p2d;
  logic        cio_sysrst_ctrl_aon_pwrb_in_p2d;
  logic        cio_sysrst_ctrl_aon_lid_open_p2d;
  logic        cio_sysrst_ctrl_aon_ec_rst_l_p2d;
  logic        cio_sysrst_ctrl_aon_flash_wp_l_p2d;
  logic        cio_sysrst_ctrl_aon_bat_disable_d2p;
  logic        cio_sysrst_ctrl_aon_bat_disable_en_d2p;
  logic        cio_sysrst_ctrl_aon_key0_out_d2p;
  logic        cio_sysrst_ctrl_aon_key0_out_en_d2p;
  logic        cio_sysrst_ctrl_aon_key1_out_d2p;
  logic        cio_sysrst_ctrl_aon_key1_out_en_d2p;
  logic        cio_sysrst_ctrl_aon_key2_out_d2p;
  logic        cio_sysrst_ctrl_aon_key2_out_en_d2p;
  logic        cio_sysrst_ctrl_aon_pwrb_out_d2p;
  logic        cio_sysrst_ctrl_aon_pwrb_out_en_d2p;
  logic        cio_sysrst_ctrl_aon_z3_wakeup_d2p;
  logic        cio_sysrst_ctrl_aon_z3_wakeup_en_d2p;
  logic        cio_sysrst_ctrl_aon_ec_rst_l_d2p;
  logic        cio_sysrst_ctrl_aon_ec_rst_l_en_d2p;
  logic        cio_sysrst_ctrl_aon_flash_wp_l_d2p;
  logic        cio_sysrst_ctrl_aon_flash_wp_l_en_d2p;
  // adc_ctrl_aon
  // pwm_aon
  logic [5:0]  cio_pwm_aon_pwm_d2p;
  logic [5:0]  cio_pwm_aon_pwm_en_d2p;
  // pinmux_aon
  // aon_timer_aon
  // sensor_ctrl_aon
  logic [8:0]  cio_sensor_ctrl_aon_ast_debug_out_d2p;
  logic [8:0]  cio_sensor_ctrl_aon_ast_debug_out_en_d2p;
  // sram_ctrl_ret_aon
  // flash_ctrl
  logic        cio_flash_ctrl_tck_p2d;
  logic        cio_flash_ctrl_tms_p2d;
  logic        cio_flash_ctrl_tdi_p2d;
  logic        cio_flash_ctrl_tdo_d2p;
  logic        cio_flash_ctrl_tdo_en_d2p;
  // rv_dm
  // rv_plic
  // aes
  // hmac
  // kmac
  // otbn
  // keymgr
  // csrng
  // entropy_src
  // edn0
  // edn1
  // sram_ctrl_main
  // rom_ctrl
  // rv_core_ibex


  logic [185:0]  intr_vector;
  // Interrupt source list
  logic intr_uart0_tx_watermark;
  logic intr_uart0_rx_watermark;
  logic intr_uart0_tx_done;
  logic intr_uart0_rx_overflow;
  logic intr_uart0_rx_frame_err;
  logic intr_uart0_rx_break_err;
  logic intr_uart0_rx_timeout;
  logic intr_uart0_rx_parity_err;
  logic intr_uart0_tx_empty;
  logic intr_uart1_tx_watermark;
  logic intr_uart1_rx_watermark;
  logic intr_uart1_tx_done;
  logic intr_uart1_rx_overflow;
  logic intr_uart1_rx_frame_err;
  logic intr_uart1_rx_break_err;
  logic intr_uart1_rx_timeout;
  logic intr_uart1_rx_parity_err;
  logic intr_uart1_tx_empty;
  logic intr_uart2_tx_watermark;
  logic intr_uart2_rx_watermark;
  logic intr_uart2_tx_done;
  logic intr_uart2_rx_overflow;
  logic intr_uart2_rx_frame_err;
  logic intr_uart2_rx_break_err;
  logic intr_uart2_rx_timeout;
  logic intr_uart2_rx_parity_err;
  logic intr_uart2_tx_empty;
  logic intr_uart3_tx_watermark;
  logic intr_uart3_rx_watermark;
  logic intr_uart3_tx_done;
  logic intr_uart3_rx_overflow;
  logic intr_uart3_rx_frame_err;
  logic intr_uart3_rx_break_err;
  logic intr_uart3_rx_timeout;
  logic intr_uart3_rx_parity_err;
  logic intr_uart3_tx_empty;
  logic [31:0] intr_gpio_gpio;
  logic intr_spi_device_upload_cmdfifo_not_empty;
  logic intr_spi_device_upload_payload_not_empty;
  logic intr_spi_device_upload_payload_overflow;
  logic intr_spi_device_readbuf_watermark;
  logic intr_spi_device_readbuf_flip;
  logic intr_spi_device_tpm_header_not_empty;
  logic intr_spi_device_tpm_rdfifo_cmd_end;
  logic intr_spi_device_tpm_rdfifo_drop;
  logic intr_i2c0_fmt_threshold;
  logic intr_i2c0_rx_threshold;
  logic intr_i2c0_acq_threshold;
  logic intr_i2c0_rx_overflow;
  logic intr_i2c0_controller_halt;
  logic intr_i2c0_scl_interference;
  logic intr_i2c0_sda_interference;
  logic intr_i2c0_stretch_timeout;
  logic intr_i2c0_sda_unstable;
  logic intr_i2c0_cmd_complete;
  logic intr_i2c0_tx_stretch;
  logic intr_i2c0_tx_threshold;
  logic intr_i2c0_acq_stretch;
  logic intr_i2c0_unexp_stop;
  logic intr_i2c0_host_timeout;
  logic intr_i2c1_fmt_threshold;
  logic intr_i2c1_rx_threshold;
  logic intr_i2c1_acq_threshold;
  logic intr_i2c1_rx_overflow;
  logic intr_i2c1_controller_halt;
  logic intr_i2c1_scl_interference;
  logic intr_i2c1_sda_interference;
  logic intr_i2c1_stretch_timeout;
  logic intr_i2c1_sda_unstable;
  logic intr_i2c1_cmd_complete;
  logic intr_i2c1_tx_stretch;
  logic intr_i2c1_tx_threshold;
  logic intr_i2c1_acq_stretch;
  logic intr_i2c1_unexp_stop;
  logic intr_i2c1_host_timeout;
  logic intr_i2c2_fmt_threshold;
  logic intr_i2c2_rx_threshold;
  logic intr_i2c2_acq_threshold;
  logic intr_i2c2_rx_overflow;
  logic intr_i2c2_controller_halt;
  logic intr_i2c2_scl_interference;
  logic intr_i2c2_sda_interference;
  logic intr_i2c2_stretch_timeout;
  logic intr_i2c2_sda_unstable;
  logic intr_i2c2_cmd_complete;
  logic intr_i2c2_tx_stretch;
  logic intr_i2c2_tx_threshold;
  logic intr_i2c2_acq_stretch;
  logic intr_i2c2_unexp_stop;
  logic intr_i2c2_host_timeout;
  logic intr_pattgen_done_ch0;
  logic intr_pattgen_done_ch1;
  logic intr_rv_timer_timer_expired_hart0_timer0;
  logic intr_otp_ctrl_otp_operation_done;
  logic intr_otp_ctrl_otp_error;
  logic intr_alert_handler_classa;
  logic intr_alert_handler_classb;
  logic intr_alert_handler_classc;
  logic intr_alert_handler_classd;
  logic intr_spi_host0_error;
  logic intr_spi_host0_spi_event;
  logic intr_spi_host1_error;
  logic intr_spi_host1_spi_event;
  logic intr_usbdev_pkt_received;
  logic intr_usbdev_pkt_sent;
  logic intr_usbdev_disconnected;
  logic intr_usbdev_host_lost;
  logic intr_usbdev_link_reset;
  logic intr_usbdev_link_suspend;
  logic intr_usbdev_link_resume;
  logic intr_usbdev_av_out_empty;
  logic intr_usbdev_rx_full;
  logic intr_usbdev_av_overflow;
  logic intr_usbdev_link_in_err;
  logic intr_usbdev_rx_crc_err;
  logic intr_usbdev_rx_pid_err;
  logic intr_usbdev_rx_bitstuff_err;
  logic intr_usbdev_frame;
  logic intr_usbdev_powered;
  logic intr_usbdev_link_out_err;
  logic intr_usbdev_av_setup_empty;
  logic intr_pwrmgr_aon_wakeup;
  logic intr_sysrst_ctrl_aon_event_detected;
  logic intr_adc_ctrl_aon_match_pending;
  logic intr_aon_timer_aon_wkup_timer_expired;
  logic intr_aon_timer_aon_wdog_timer_bark;
  logic intr_sensor_ctrl_aon_io_status_change;
  logic intr_sensor_ctrl_aon_init_status_change;
  logic intr_flash_ctrl_prog_empty;
  logic intr_flash_ctrl_prog_lvl;
  logic intr_flash_ctrl_rd_full;
  logic intr_flash_ctrl_rd_lvl;
  logic intr_flash_ctrl_op_done;
  logic intr_flash_ctrl_corr_err;
  logic intr_hmac_hmac_done;
  logic intr_hmac_fifo_empty;
  logic intr_hmac_hmac_err;
  logic intr_kmac_kmac_done;
  logic intr_kmac_fifo_empty;
  logic intr_kmac_kmac_err;
  logic intr_otbn_done;
  logic intr_keymgr_op_done;
  logic intr_csrng_cs_cmd_req_done;
  logic intr_csrng_cs_entropy_req;
  logic intr_csrng_cs_hw_inst_exc;
  logic intr_csrng_cs_fatal_err;
  logic intr_entropy_src_es_entropy_valid;
  logic intr_entropy_src_es_health_test_failed;
  logic intr_entropy_src_es_observe_fifo_ready;
  logic intr_entropy_src_es_fatal_err;
  logic intr_edn0_edn_cmd_req_done;
  logic intr_edn0_edn_fatal_err;
  logic intr_edn1_edn_cmd_req_done;
  logic intr_edn1_edn_fatal_err;

  // Alert list
  prim_alert_pkg::alert_tx_t [alert_pkg::NAlerts-1:0]  alert_tx;
  prim_alert_pkg::alert_rx_t [alert_pkg::NAlerts-1:0]  alert_rx;


  // define inter-module signals
  ast_pkg::ast_obs_ctrl_t       ast_obs_ctrl;
  prim_ram_1p_pkg::ram_1p_cfg_t       ast_ram_1p_cfg;
  prim_ram_2p_pkg::ram_2p_cfg_t       ast_spi_ram_2p_cfg;
  prim_ram_1p_pkg::ram_1p_cfg_t       ast_usb_ram_1p_cfg;
  prim_rom_pkg::rom_cfg_t       ast_rom_cfg;
  alert_pkg::alert_crashdump_t       alert_handler_crashdump;
  prim_esc_pkg::esc_rx_t [3:0] alert_handler_esc_rx;
  prim_esc_pkg::esc_tx_t [3:0] alert_handler_esc_tx;
  logic       aon_timer_aon_nmi_wdog_timer_bark;
  csrng_pkg::csrng_req_t [1:0] csrng_csrng_cmd_req;
  csrng_pkg::csrng_rsp_t [1:0] csrng_csrng_cmd_rsp;
  entropy_src_pkg::entropy_src_hw_if_req_t       csrng_entropy_src_hw_if_req;
  entropy_src_pkg::entropy_src_hw_if_rsp_t       csrng_entropy_src_hw_if_rsp;
  entropy_src_pkg::cs_aes_halt_req_t       csrng_cs_aes_halt_req;
  entropy_src_pkg::cs_aes_halt_rsp_t       csrng_cs_aes_halt_rsp;
  flash_ctrl_pkg::keymgr_flash_t       flash_ctrl_keymgr;
  otp_ctrl_pkg::flash_otp_key_req_t       flash_ctrl_otp_req;
  otp_ctrl_pkg::flash_otp_key_rsp_t       flash_ctrl_otp_rsp;
  lc_ctrl_pkg::lc_flash_rma_seed_t       flash_ctrl_rma_seed;
  otp_ctrl_pkg::sram_otp_key_req_t [3:0] otp_ctrl_sram_otp_key_req;
  otp_ctrl_pkg::sram_otp_key_rsp_t [3:0] otp_ctrl_sram_otp_key_rsp;
  pwrmgr_pkg::pwr_flash_t       pwrmgr_aon_pwr_flash;
  pwrmgr_pkg::pwr_rst_req_t       pwrmgr_aon_pwr_rst_req;
  pwrmgr_pkg::pwr_rst_rsp_t       pwrmgr_aon_pwr_rst_rsp;
  pwrmgr_pkg::pwr_clk_req_t       pwrmgr_aon_pwr_clk_req;
  pwrmgr_pkg::pwr_clk_rsp_t       pwrmgr_aon_pwr_clk_rsp;
  pwrmgr_pkg::pwr_otp_req_t       pwrmgr_aon_pwr_otp_req;
  pwrmgr_pkg::pwr_otp_rsp_t       pwrmgr_aon_pwr_otp_rsp;
  pwrmgr_pkg::pwr_lc_req_t       pwrmgr_aon_pwr_lc_req;
  pwrmgr_pkg::pwr_lc_rsp_t       pwrmgr_aon_pwr_lc_rsp;
  logic       pwrmgr_aon_strap;
  logic       pwrmgr_aon_low_power;
  lc_ctrl_pkg::lc_tx_t       pwrmgr_aon_fetch_en;
  rom_ctrl_pkg::pwrmgr_data_t       rom_ctrl_pwrmgr_data;
  rom_ctrl_pkg::keymgr_data_t       rom_ctrl_keymgr_data;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_flash_rma_req;
  lc_ctrl_pkg::lc_tx_t [1:0] lc_ctrl_lc_flash_rma_ack;
  logic       usbdev_usb_dp_pullup;
  logic       usbdev_usb_dn_pullup;
  logic       usbdev_usb_aon_suspend_req;
  logic       usbdev_usb_aon_wake_ack;
  logic       usbdev_usb_aon_bus_not_idle;
  logic       usbdev_usb_aon_bus_reset;
  logic       usbdev_usb_aon_sense_lost;
  logic       pinmux_aon_usbdev_wake_detect_active;
  edn_pkg::edn_req_t [7:0] edn0_edn_req;
  edn_pkg::edn_rsp_t [7:0] edn0_edn_rsp;
  edn_pkg::edn_req_t [7:0] edn1_edn_req;
  edn_pkg::edn_rsp_t [7:0] edn1_edn_rsp;
  otp_ctrl_pkg::otbn_otp_key_req_t       otp_ctrl_otbn_otp_key_req;
  otp_ctrl_pkg::otbn_otp_key_rsp_t       otp_ctrl_otbn_otp_key_rsp;
  otp_ctrl_pkg::otp_keymgr_key_t       otp_ctrl_otp_keymgr_key;
  keymgr_pkg::hw_key_req_t       keymgr_aes_key;
  keymgr_pkg::hw_key_req_t       keymgr_kmac_key;
  keymgr_pkg::otbn_key_req_t       keymgr_otbn_key;
  kmac_pkg::app_req_t [2:0] kmac_app_req;
  kmac_pkg::app_rsp_t [2:0] kmac_app_rsp;
  logic       kmac_en_masking;
  prim_mubi_pkg::mubi4_t [3:0] clkmgr_aon_idle;
  jtag_pkg::jtag_req_t       pinmux_aon_lc_jtag_req;
  jtag_pkg::jtag_rsp_t       pinmux_aon_lc_jtag_rsp;
  jtag_pkg::jtag_req_t       pinmux_aon_rv_jtag_req;
  jtag_pkg::jtag_rsp_t       pinmux_aon_rv_jtag_rsp;
  lc_ctrl_pkg::lc_tx_t       pinmux_aon_pinmux_hw_debug_en;
  otp_ctrl_pkg::otp_lc_data_t       otp_ctrl_otp_lc_data;
  otp_ctrl_pkg::lc_otp_program_req_t       lc_ctrl_lc_otp_program_req;
  otp_ctrl_pkg::lc_otp_program_rsp_t       lc_ctrl_lc_otp_program_rsp;
  otp_ctrl_pkg::lc_otp_vendor_test_req_t       lc_ctrl_lc_otp_vendor_test_req;
  otp_ctrl_pkg::lc_otp_vendor_test_rsp_t       lc_ctrl_lc_otp_vendor_test_rsp;
  lc_ctrl_pkg::lc_keymgr_div_t       lc_ctrl_lc_keymgr_div;
  logic       lc_ctrl_strap_en_override;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_dft_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_nvm_debug_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_hw_debug_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_cpu_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_keymgr_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_escalate_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_check_byp_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_clk_byp_req;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_clk_byp_ack;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_creator_seed_sw_rw_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_owner_seed_sw_rw_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_iso_part_sw_rd_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_iso_part_sw_wr_en;
  lc_ctrl_pkg::lc_tx_t       lc_ctrl_lc_seed_hw_rd_en;
  logic       rv_plic_msip;
  logic       rv_plic_irq;
  logic       rv_dm_debug_req;
  rv_core_ibex_pkg::cpu_crash_dump_t       rv_core_ibex_crash_dump;
  pwrmgr_pkg::pwr_cpu_t       rv_core_ibex_pwrmgr;
  spi_device_pkg::passthrough_req_t       spi_device_passthrough_req;
  spi_device_pkg::passthrough_rsp_t       spi_device_passthrough_rsp;
  logic       rv_dm_ndmreset_req;
  prim_mubi_pkg::mubi4_t       rstmgr_aon_sw_rst_req;
  logic [5:0] pwrmgr_aon_wakeups;
  logic [1:0] pwrmgr_aon_rstreqs;
  tlul_pkg::tl_h2d_t       main_tl_rv_core_ibex__corei_req;
  tlul_pkg::tl_d2h_t       main_tl_rv_core_ibex__corei_rsp;
  tlul_pkg::tl_h2d_t       main_tl_rv_core_ibex__cored_req;
  tlul_pkg::tl_d2h_t       main_tl_rv_core_ibex__cored_rsp;
  tlul_pkg::tl_h2d_t       main_tl_rv_dm__sba_req;
  tlul_pkg::tl_d2h_t       main_tl_rv_dm__sba_rsp;
  tlul_pkg::tl_h2d_t       rv_dm_regs_tl_d_req;
  tlul_pkg::tl_d2h_t       rv_dm_regs_tl_d_rsp;
  tlul_pkg::tl_h2d_t       rv_dm_mem_tl_d_req;
  tlul_pkg::tl_d2h_t       rv_dm_mem_tl_d_rsp;
  tlul_pkg::tl_h2d_t       rom_ctrl_rom_tl_req;
  tlul_pkg::tl_d2h_t       rom_ctrl_rom_tl_rsp;
  tlul_pkg::tl_h2d_t       rom_ctrl_regs_tl_req;
  tlul_pkg::tl_d2h_t       rom_ctrl_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       main_tl_peri_req;
  tlul_pkg::tl_d2h_t       main_tl_peri_rsp;
  tlul_pkg::tl_h2d_t       spi_host0_tl_req;
  tlul_pkg::tl_d2h_t       spi_host0_tl_rsp;
  tlul_pkg::tl_h2d_t       spi_host1_tl_req;
  tlul_pkg::tl_d2h_t       spi_host1_tl_rsp;
  tlul_pkg::tl_h2d_t       usbdev_tl_req;
  tlul_pkg::tl_d2h_t       usbdev_tl_rsp;
  tlul_pkg::tl_h2d_t       flash_ctrl_core_tl_req;
  tlul_pkg::tl_d2h_t       flash_ctrl_core_tl_rsp;
  tlul_pkg::tl_h2d_t       flash_ctrl_prim_tl_req;
  tlul_pkg::tl_d2h_t       flash_ctrl_prim_tl_rsp;
  tlul_pkg::tl_h2d_t       flash_ctrl_mem_tl_req;
  tlul_pkg::tl_d2h_t       flash_ctrl_mem_tl_rsp;
  tlul_pkg::tl_h2d_t       hmac_tl_req;
  tlul_pkg::tl_d2h_t       hmac_tl_rsp;
  tlul_pkg::tl_h2d_t       kmac_tl_req;
  tlul_pkg::tl_d2h_t       kmac_tl_rsp;
  tlul_pkg::tl_h2d_t       aes_tl_req;
  tlul_pkg::tl_d2h_t       aes_tl_rsp;
  tlul_pkg::tl_h2d_t       entropy_src_tl_req;
  tlul_pkg::tl_d2h_t       entropy_src_tl_rsp;
  tlul_pkg::tl_h2d_t       csrng_tl_req;
  tlul_pkg::tl_d2h_t       csrng_tl_rsp;
  tlul_pkg::tl_h2d_t       edn0_tl_req;
  tlul_pkg::tl_d2h_t       edn0_tl_rsp;
  tlul_pkg::tl_h2d_t       edn1_tl_req;
  tlul_pkg::tl_d2h_t       edn1_tl_rsp;
  tlul_pkg::tl_h2d_t       rv_plic_tl_req;
  tlul_pkg::tl_d2h_t       rv_plic_tl_rsp;
  tlul_pkg::tl_h2d_t       otbn_tl_req;
  tlul_pkg::tl_d2h_t       otbn_tl_rsp;
  tlul_pkg::tl_h2d_t       keymgr_tl_req;
  tlul_pkg::tl_d2h_t       keymgr_tl_rsp;
  tlul_pkg::tl_h2d_t       rv_core_ibex_cfg_tl_d_req;
  tlul_pkg::tl_d2h_t       rv_core_ibex_cfg_tl_d_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_main_regs_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_main_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_main_ram_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_main_ram_tl_rsp;
  tlul_pkg::tl_h2d_t       uart0_tl_req;
  tlul_pkg::tl_d2h_t       uart0_tl_rsp;
  tlul_pkg::tl_h2d_t       uart1_tl_req;
  tlul_pkg::tl_d2h_t       uart1_tl_rsp;
  tlul_pkg::tl_h2d_t       uart2_tl_req;
  tlul_pkg::tl_d2h_t       uart2_tl_rsp;
  tlul_pkg::tl_h2d_t       uart3_tl_req;
  tlul_pkg::tl_d2h_t       uart3_tl_rsp;
  tlul_pkg::tl_h2d_t       i2c0_tl_req;
  tlul_pkg::tl_d2h_t       i2c0_tl_rsp;
  tlul_pkg::tl_h2d_t       i2c1_tl_req;
  tlul_pkg::tl_d2h_t       i2c1_tl_rsp;
  tlul_pkg::tl_h2d_t       i2c2_tl_req;
  tlul_pkg::tl_d2h_t       i2c2_tl_rsp;
  tlul_pkg::tl_h2d_t       pattgen_tl_req;
  tlul_pkg::tl_d2h_t       pattgen_tl_rsp;
  tlul_pkg::tl_h2d_t       pwm_aon_tl_req;
  tlul_pkg::tl_d2h_t       pwm_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       gpio_tl_req;
  tlul_pkg::tl_d2h_t       gpio_tl_rsp;
  tlul_pkg::tl_h2d_t       spi_device_tl_req;
  tlul_pkg::tl_d2h_t       spi_device_tl_rsp;
  tlul_pkg::tl_h2d_t       rv_timer_tl_req;
  tlul_pkg::tl_d2h_t       rv_timer_tl_rsp;
  tlul_pkg::tl_h2d_t       pwrmgr_aon_tl_req;
  tlul_pkg::tl_d2h_t       pwrmgr_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       rstmgr_aon_tl_req;
  tlul_pkg::tl_d2h_t       rstmgr_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       clkmgr_aon_tl_req;
  tlul_pkg::tl_d2h_t       clkmgr_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       pinmux_aon_tl_req;
  tlul_pkg::tl_d2h_t       pinmux_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       otp_ctrl_core_tl_req;
  tlul_pkg::tl_d2h_t       otp_ctrl_core_tl_rsp;
  tlul_pkg::tl_h2d_t       otp_ctrl_prim_tl_req;
  tlul_pkg::tl_d2h_t       otp_ctrl_prim_tl_rsp;
  tlul_pkg::tl_h2d_t       lc_ctrl_tl_req;
  tlul_pkg::tl_d2h_t       lc_ctrl_tl_rsp;
  tlul_pkg::tl_h2d_t       sensor_ctrl_aon_tl_req;
  tlul_pkg::tl_d2h_t       sensor_ctrl_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       alert_handler_tl_req;
  tlul_pkg::tl_d2h_t       alert_handler_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_ret_aon_regs_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_ret_aon_regs_tl_rsp;
  tlul_pkg::tl_h2d_t       sram_ctrl_ret_aon_ram_tl_req;
  tlul_pkg::tl_d2h_t       sram_ctrl_ret_aon_ram_tl_rsp;
  tlul_pkg::tl_h2d_t       aon_timer_aon_tl_req;
  tlul_pkg::tl_d2h_t       aon_timer_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       sysrst_ctrl_aon_tl_req;
  tlul_pkg::tl_d2h_t       sysrst_ctrl_aon_tl_rsp;
  tlul_pkg::tl_h2d_t       adc_ctrl_aon_tl_req;
  tlul_pkg::tl_d2h_t       adc_ctrl_aon_tl_rsp;
  clkmgr_pkg::clkmgr_out_t       clkmgr_aon_clocks;
  clkmgr_pkg::clkmgr_cg_en_t       clkmgr_aon_cg_en;
  rstmgr_pkg::rstmgr_out_t       rstmgr_aon_resets;
  rstmgr_pkg::rstmgr_rst_en_t       rstmgr_aon_rst_en;
  logic       rv_core_ibex_irq_timer;
  logic [31:0] rv_core_ibex_hart_id;
  logic [31:0] rv_core_ibex_boot_addr;
  jtag_pkg::jtag_req_t       pinmux_aon_dft_jtag_req;
  jtag_pkg::jtag_rsp_t       pinmux_aon_dft_jtag_rsp;
  otp_ctrl_part_pkg::otp_broadcast_t       otp_ctrl_otp_broadcast;
  prim_mubi_pkg::mubi8_t       csrng_otp_en_csrng_sw_app_read;
  otp_ctrl_pkg::otp_device_id_t       lc_ctrl_otp_device_id;
  otp_ctrl_pkg::otp_manuf_state_t       lc_ctrl_otp_manuf_state;
  otp_ctrl_pkg::otp_device_id_t       keymgr_otp_device_id;
  prim_mubi_pkg::mubi8_t       sram_ctrl_main_otp_en_sram_ifetch;
  prim_mubi_pkg::mubi8_t       rv_dm_otp_dis_rv_dm_late_debug;

  // define mixed connection to port
  assign edn0_edn_req[2] = ast_edn_req_i;
  assign ast_edn_rsp_o = edn0_edn_rsp[2];
  assign ast_lc_dft_en_o = lc_ctrl_lc_dft_en;
  assign ast_obs_ctrl = obs_ctrl_i;
  assign ast_ram_1p_cfg = ram_1p_cfg_i;
  assign ast_spi_ram_2p_cfg = spi_ram_2p_cfg_i;
  assign ast_usb_ram_1p_cfg = usb_ram_1p_cfg_i;
  assign ast_rom_cfg = rom_cfg_i;

  // define partial inter-module tie-off
  otp_ctrl_pkg::sram_otp_key_rsp_t unused_otp_ctrl_sram_otp_key_rsp3;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp1;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp2;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp3;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp4;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp5;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp6;
  edn_pkg::edn_rsp_t unused_edn1_edn_rsp7;

  // assign partial inter-module tie-off
  assign unused_otp_ctrl_sram_otp_key_rsp3 = otp_ctrl_sram_otp_key_rsp[3];
  assign unused_edn1_edn_rsp1 = edn1_edn_rsp[1];
  assign unused_edn1_edn_rsp2 = edn1_edn_rsp[2];
  assign unused_edn1_edn_rsp3 = edn1_edn_rsp[3];
  assign unused_edn1_edn_rsp4 = edn1_edn_rsp[4];
  assign unused_edn1_edn_rsp5 = edn1_edn_rsp[5];
  assign unused_edn1_edn_rsp6 = edn1_edn_rsp[6];
  assign unused_edn1_edn_rsp7 = edn1_edn_rsp[7];
  assign otp_ctrl_sram_otp_key_req[3] = '0;
  assign edn1_edn_req[1] = '0;
  assign edn1_edn_req[2] = '0;
  assign edn1_edn_req[3] = '0;
  assign edn1_edn_req[4] = '0;
  assign edn1_edn_req[5] = '0;
  assign edn1_edn_req[6] = '0;
  assign edn1_edn_req[7] = '0;


  // OTP HW_CFG* Broadcast signals.
  // TODO(#6713): The actual struct breakout and mapping currently needs to
  // be performed by hand.
  assign csrng_otp_en_csrng_sw_app_read =
      otp_ctrl_otp_broadcast.hw_cfg1_data.en_csrng_sw_app_read;
  assign sram_ctrl_main_otp_en_sram_ifetch =
      otp_ctrl_otp_broadcast.hw_cfg1_data.en_sram_ifetch;
  assign rv_dm_otp_dis_rv_dm_late_debug =
      otp_ctrl_otp_broadcast.hw_cfg1_data.dis_rv_dm_late_debug;
  assign lc_ctrl_otp_device_id =
      otp_ctrl_otp_broadcast.hw_cfg0_data.device_id;
  assign lc_ctrl_otp_manuf_state =
      otp_ctrl_otp_broadcast.hw_cfg0_data.manuf_state;
  assign keymgr_otp_device_id =
      otp_ctrl_otp_broadcast.hw_cfg0_data.device_id;

  logic unused_otp_broadcast_bits;
  assign unused_otp_broadcast_bits = ^{
    otp_ctrl_otp_broadcast.valid,
    otp_ctrl_otp_broadcast.hw_cfg0_data.hw_cfg0_digest,
    otp_ctrl_otp_broadcast.hw_cfg1_data.hw_cfg1_digest,
    otp_ctrl_otp_broadcast.hw_cfg1_data.unallocated
  };

  // See #7978 This below is a hack.
  // This is because ast is a comportable-like module that sits outside
  // of top_earlgrey's boundary.
  assign clks_ast_o = clkmgr_aon_clocks;
  assign rsts_ast_o = rstmgr_aon_resets;

  // ibex specific assignments
  // TODO: This should be further automated in the future.
  assign rv_core_ibex_irq_timer = intr_rv_timer_timer_expired_hart0_timer0;
  assign rv_core_ibex_hart_id = '0;

  assign rv_core_ibex_boot_addr = ADDR_SPACE_ROM_CTRL__ROM;


  // Struct breakout module tool-inserted DFT TAP signals
  pinmux_jtag_breakout u_dft_tap_breakout (
    .req_i    (pinmux_aon_dft_jtag_req),
    .rsp_o    (pinmux_aon_dft_jtag_rsp),
    .tck_o    (),
    .trst_no  (),
    .tms_o    (),
    .tdi_o    (),
    .tdo_i    (1'b0),
    .tdo_oe_i (1'b0)
  );

  // Wire up alert handler LPGs
  prim_mubi_pkg::mubi4_t [alert_pkg::NLpg-1:0] lpg_cg_en;
  prim_mubi_pkg::mubi4_t [alert_pkg::NLpg-1:0] lpg_rst_en;


  // peri_lc_io_div4_0
  assign lpg_cg_en[0] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[0] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // peri_spi_device_0
  assign lpg_cg_en[1] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[1] = rstmgr_aon_rst_en.spi_device[rstmgr_pkg::Domain0Sel];
  // peri_i2c0_0
  assign lpg_cg_en[2] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[2] = rstmgr_aon_rst_en.i2c0[rstmgr_pkg::Domain0Sel];
  // peri_i2c1_0
  assign lpg_cg_en[3] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[3] = rstmgr_aon_rst_en.i2c1[rstmgr_pkg::Domain0Sel];
  // peri_i2c2_0
  assign lpg_cg_en[4] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[4] = rstmgr_aon_rst_en.i2c2[rstmgr_pkg::Domain0Sel];
  // timers_lc_io_div4_0
  assign lpg_cg_en[5] = clkmgr_aon_cg_en.io_div4_timers;
  assign lpg_rst_en[5] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // secure_lc_io_div4_0
  assign lpg_cg_en[6] = clkmgr_aon_cg_en.io_div4_secure;
  assign lpg_rst_en[6] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // peri_spi_host0_0
  assign lpg_cg_en[7] = clkmgr_aon_cg_en.io_peri;
  assign lpg_rst_en[7] = rstmgr_aon_rst_en.spi_host0[rstmgr_pkg::Domain0Sel];
  // peri_spi_host1_0
  assign lpg_cg_en[8] = clkmgr_aon_cg_en.io_div2_peri;
  assign lpg_rst_en[8] = rstmgr_aon_rst_en.spi_host1[rstmgr_pkg::Domain0Sel];
  // peri_usb_0
  assign lpg_cg_en[9] = clkmgr_aon_cg_en.usb_peri;
  assign lpg_rst_en[9] = rstmgr_aon_rst_en.usb[rstmgr_pkg::Domain0Sel];
  // powerup_por_io_div4_Aon
  assign lpg_cg_en[10] = clkmgr_aon_cg_en.io_div4_powerup;
  assign lpg_rst_en[10] = rstmgr_aon_rst_en.por_io_div4[rstmgr_pkg::DomainAonSel];
  // powerup_lc_io_div4_Aon
  assign lpg_cg_en[11] = clkmgr_aon_cg_en.io_div4_powerup;
  assign lpg_rst_en[11] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // secure_lc_io_div4_Aon
  assign lpg_cg_en[12] = clkmgr_aon_cg_en.io_div4_secure;
  assign lpg_rst_en[12] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // peri_lc_io_div4_Aon
  assign lpg_cg_en[13] = clkmgr_aon_cg_en.io_div4_peri;
  assign lpg_rst_en[13] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // timers_lc_io_div4_Aon
  assign lpg_cg_en[14] = clkmgr_aon_cg_en.io_div4_timers;
  assign lpg_rst_en[14] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // infra_lc_io_div4_0
  assign lpg_cg_en[15] = clkmgr_aon_cg_en.io_div4_infra;
  assign lpg_rst_en[15] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::Domain0Sel];
  // infra_lc_io_div4_Aon
  assign lpg_cg_en[16] = clkmgr_aon_cg_en.io_div4_infra;
  assign lpg_rst_en[16] = rstmgr_aon_rst_en.lc_io_div4[rstmgr_pkg::DomainAonSel];
  // infra_lc_0
  assign lpg_cg_en[17] = clkmgr_aon_cg_en.main_infra;
  assign lpg_rst_en[17] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // infra_sys_0
  assign lpg_cg_en[18] = clkmgr_aon_cg_en.main_infra;
  assign lpg_rst_en[18] = rstmgr_aon_rst_en.sys[rstmgr_pkg::Domain0Sel];
  // secure_lc_0
  assign lpg_cg_en[19] = clkmgr_aon_cg_en.main_secure;
  assign lpg_rst_en[19] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // aes_trans_lc_0
  assign lpg_cg_en[20] = clkmgr_aon_cg_en.main_aes;
  assign lpg_rst_en[20] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // hmac_trans_lc_0
  assign lpg_cg_en[21] = clkmgr_aon_cg_en.main_hmac;
  assign lpg_rst_en[21] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // kmac_trans_lc_0
  assign lpg_cg_en[22] = clkmgr_aon_cg_en.main_kmac;
  assign lpg_rst_en[22] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];
  // otbn_trans_lc_0
  assign lpg_cg_en[23] = clkmgr_aon_cg_en.main_otbn;
  assign lpg_rst_en[23] = rstmgr_aon_rst_en.lc[rstmgr_pkg::Domain0Sel];

// tie-off unused connections
//VCS coverage off
// pragma coverage off
    prim_mubi_pkg::mubi4_t unused_cg_en_0;
    assign unused_cg_en_0 = clkmgr_aon_cg_en.aon_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_1;
    assign unused_cg_en_1 = clkmgr_aon_cg_en.main_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_2;
    assign unused_cg_en_2 = clkmgr_aon_cg_en.io_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_3;
    assign unused_cg_en_3 = clkmgr_aon_cg_en.usb_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_4;
    assign unused_cg_en_4 = clkmgr_aon_cg_en.io_div2_powerup;
    prim_mubi_pkg::mubi4_t unused_cg_en_5;
    assign unused_cg_en_5 = clkmgr_aon_cg_en.aon_secure;
    prim_mubi_pkg::mubi4_t unused_cg_en_6;
    assign unused_cg_en_6 = clkmgr_aon_cg_en.aon_peri;
    prim_mubi_pkg::mubi4_t unused_cg_en_7;
    assign unused_cg_en_7 = clkmgr_aon_cg_en.aon_timers;
    prim_mubi_pkg::mubi4_t unused_cg_en_8;
    assign unused_cg_en_8 = clkmgr_aon_cg_en.usb_infra;
    prim_mubi_pkg::mubi4_t unused_cg_en_9;
    assign unused_cg_en_9 = clkmgr_aon_cg_en.io_infra;
    prim_mubi_pkg::mubi4_t unused_cg_en_10;
    assign unused_cg_en_10 = clkmgr_aon_cg_en.io_div2_infra;
    prim_mubi_pkg::mubi4_t unused_rst_en_0;
    assign unused_rst_en_0 = rstmgr_aon_rst_en.por_aon[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_1;
    assign unused_rst_en_1 = rstmgr_aon_rst_en.por_aon[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_2;
    assign unused_rst_en_2 = rstmgr_aon_rst_en.por[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_3;
    assign unused_rst_en_3 = rstmgr_aon_rst_en.por[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_4;
    assign unused_rst_en_4 = rstmgr_aon_rst_en.por_io[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_5;
    assign unused_rst_en_5 = rstmgr_aon_rst_en.por_io[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_6;
    assign unused_rst_en_6 = rstmgr_aon_rst_en.por_io_div2[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_7;
    assign unused_rst_en_7 = rstmgr_aon_rst_en.por_io_div2[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_8;
    assign unused_rst_en_8 = rstmgr_aon_rst_en.por_io_div4[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_9;
    assign unused_rst_en_9 = rstmgr_aon_rst_en.por_usb[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_10;
    assign unused_rst_en_10 = rstmgr_aon_rst_en.por_usb[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_11;
    assign unused_rst_en_11 = rstmgr_aon_rst_en.lc_shadowed[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_12;
    assign unused_rst_en_12 = rstmgr_aon_rst_en.lc[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_13;
    assign unused_rst_en_13 = rstmgr_aon_rst_en.lc_shadowed[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_14;
    assign unused_rst_en_14 = rstmgr_aon_rst_en.lc_aon[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_15;
    assign unused_rst_en_15 = rstmgr_aon_rst_en.lc_aon[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_16;
    assign unused_rst_en_16 = rstmgr_aon_rst_en.lc_io[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_17;
    assign unused_rst_en_17 = rstmgr_aon_rst_en.lc_io[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_18;
    assign unused_rst_en_18 = rstmgr_aon_rst_en.lc_io_div2[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_19;
    assign unused_rst_en_19 = rstmgr_aon_rst_en.lc_io_div2[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_20;
    assign unused_rst_en_20 = rstmgr_aon_rst_en.lc_io_div4_shadowed[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_21;
    assign unused_rst_en_21 = rstmgr_aon_rst_en.lc_io_div4_shadowed[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_22;
    assign unused_rst_en_22 = rstmgr_aon_rst_en.lc_usb[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_23;
    assign unused_rst_en_23 = rstmgr_aon_rst_en.lc_usb[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_24;
    assign unused_rst_en_24 = rstmgr_aon_rst_en.sys[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_25;
    assign unused_rst_en_25 = rstmgr_aon_rst_en.sys_io_div4[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_26;
    assign unused_rst_en_26 = rstmgr_aon_rst_en.sys_io_div4[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_27;
    assign unused_rst_en_27 = rstmgr_aon_rst_en.spi_device[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_28;
    assign unused_rst_en_28 = rstmgr_aon_rst_en.spi_host0[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_29;
    assign unused_rst_en_29 = rstmgr_aon_rst_en.spi_host1[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_30;
    assign unused_rst_en_30 = rstmgr_aon_rst_en.usb[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_31;
    assign unused_rst_en_31 = rstmgr_aon_rst_en.usb_aon[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_32;
    assign unused_rst_en_32 = rstmgr_aon_rst_en.usb_aon[rstmgr_pkg::Domain0Sel];
    prim_mubi_pkg::mubi4_t unused_rst_en_33;
    assign unused_rst_en_33 = rstmgr_aon_rst_en.i2c0[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_34;
    assign unused_rst_en_34 = rstmgr_aon_rst_en.i2c1[rstmgr_pkg::DomainAonSel];
    prim_mubi_pkg::mubi4_t unused_rst_en_35;
    assign unused_rst_en_35 = rstmgr_aon_rst_en.i2c2[rstmgr_pkg::DomainAonSel];
//VCS coverage on
// pragma coverage on

  // Peripheral Instantiation


  uart #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[0:0])
  ) u_uart0 (

      // Input
      .cio_rx_i    (cio_uart0_rx_p2d),

      // Output
      .cio_tx_o    (cio_uart0_tx_d2p),
      .cio_tx_en_o (cio_uart0_tx_en_d2p),

      // Interrupt
      .intr_tx_watermark_o  (intr_uart0_tx_watermark),
      .intr_rx_watermark_o  (intr_uart0_rx_watermark),
      .intr_tx_done_o       (intr_uart0_tx_done),
      .intr_rx_overflow_o   (intr_uart0_rx_overflow),
      .intr_rx_frame_err_o  (intr_uart0_rx_frame_err),
      .intr_rx_break_err_o  (intr_uart0_rx_break_err),
      .intr_rx_timeout_o    (intr_uart0_rx_timeout),
      .intr_rx_parity_err_o (intr_uart0_rx_parity_err),
      .intr_tx_empty_o      (intr_uart0_tx_empty),
      // [0]: fatal_fault
      .alert_tx_o  ( alert_tx[0:0] ),
      .alert_rx_i  ( alert_rx[0:0] ),

      // Inter-module signals
      .tl_i(uart0_tl_req),
      .tl_o(uart0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  uart #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[1:1])
  ) u_uart1 (

      // Input
      .cio_rx_i    (cio_uart1_rx_p2d),

      // Output
      .cio_tx_o    (cio_uart1_tx_d2p),
      .cio_tx_en_o (cio_uart1_tx_en_d2p),

      // Interrupt
      .intr_tx_watermark_o  (intr_uart1_tx_watermark),
      .intr_rx_watermark_o  (intr_uart1_rx_watermark),
      .intr_tx_done_o       (intr_uart1_tx_done),
      .intr_rx_overflow_o   (intr_uart1_rx_overflow),
      .intr_rx_frame_err_o  (intr_uart1_rx_frame_err),
      .intr_rx_break_err_o  (intr_uart1_rx_break_err),
      .intr_rx_timeout_o    (intr_uart1_rx_timeout),
      .intr_rx_parity_err_o (intr_uart1_rx_parity_err),
      .intr_tx_empty_o      (intr_uart1_tx_empty),
      // [1]: fatal_fault
      .alert_tx_o  ( alert_tx[1:1] ),
      .alert_rx_i  ( alert_rx[1:1] ),

      // Inter-module signals
      .tl_i(uart1_tl_req),
      .tl_o(uart1_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  uart #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[2:2])
  ) u_uart2 (

      // Input
      .cio_rx_i    (cio_uart2_rx_p2d),

      // Output
      .cio_tx_o    (cio_uart2_tx_d2p),
      .cio_tx_en_o (cio_uart2_tx_en_d2p),

      // Interrupt
      .intr_tx_watermark_o  (intr_uart2_tx_watermark),
      .intr_rx_watermark_o  (intr_uart2_rx_watermark),
      .intr_tx_done_o       (intr_uart2_tx_done),
      .intr_rx_overflow_o   (intr_uart2_rx_overflow),
      .intr_rx_frame_err_o  (intr_uart2_rx_frame_err),
      .intr_rx_break_err_o  (intr_uart2_rx_break_err),
      .intr_rx_timeout_o    (intr_uart2_rx_timeout),
      .intr_rx_parity_err_o (intr_uart2_rx_parity_err),
      .intr_tx_empty_o      (intr_uart2_tx_empty),
      // [2]: fatal_fault
      .alert_tx_o  ( alert_tx[2:2] ),
      .alert_rx_i  ( alert_rx[2:2] ),

      // Inter-module signals
      .tl_i(uart2_tl_req),
      .tl_o(uart2_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  uart #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[3:3])
  ) u_uart3 (

      // Input
      .cio_rx_i    (cio_uart3_rx_p2d),

      // Output
      .cio_tx_o    (cio_uart3_tx_d2p),
      .cio_tx_en_o (cio_uart3_tx_en_d2p),

      // Interrupt
      .intr_tx_watermark_o  (intr_uart3_tx_watermark),
      .intr_rx_watermark_o  (intr_uart3_rx_watermark),
      .intr_tx_done_o       (intr_uart3_tx_done),
      .intr_rx_overflow_o   (intr_uart3_rx_overflow),
      .intr_rx_frame_err_o  (intr_uart3_rx_frame_err),
      .intr_rx_break_err_o  (intr_uart3_rx_break_err),
      .intr_rx_timeout_o    (intr_uart3_rx_timeout),
      .intr_rx_parity_err_o (intr_uart3_rx_parity_err),
      .intr_tx_empty_o      (intr_uart3_tx_empty),
      // [3]: fatal_fault
      .alert_tx_o  ( alert_tx[3:3] ),
      .alert_rx_i  ( alert_rx[3:3] ),

      // Inter-module signals
      .tl_i(uart3_tl_req),
      .tl_o(uart3_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  gpio #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[4:4]),
    .GpioAsyncOn(GpioGpioAsyncOn)
  ) u_gpio (

      // Input
      .cio_gpio_i    (cio_gpio_gpio_p2d),

      // Output
      .cio_gpio_o    (cio_gpio_gpio_d2p),
      .cio_gpio_en_o (cio_gpio_gpio_en_d2p),

      // Interrupt
      .intr_gpio_o (intr_gpio_gpio),
      // [4]: fatal_fault
      .alert_tx_o  ( alert_tx[4:4] ),
      .alert_rx_i  ( alert_rx[4:4] ),

      // Inter-module signals
      .tl_i(gpio_tl_req),
      .tl_o(gpio_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  spi_device #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[5:5]),
    .SramType(SpiDeviceSramType)
  ) u_spi_device (

      // Input
      .cio_sck_i        (cio_spi_device_sck_p2d),
      .cio_csb_i        (cio_spi_device_csb_p2d),
      .cio_tpm_csb_i    (cio_spi_device_tpm_csb_p2d),
      .cio_sd_i         (cio_spi_device_sd_p2d),

      // Output
      .cio_sd_o         (cio_spi_device_sd_d2p),
      .cio_sd_en_o      (cio_spi_device_sd_en_d2p),

      // Interrupt
      .intr_upload_cmdfifo_not_empty_o (intr_spi_device_upload_cmdfifo_not_empty),
      .intr_upload_payload_not_empty_o (intr_spi_device_upload_payload_not_empty),
      .intr_upload_payload_overflow_o  (intr_spi_device_upload_payload_overflow),
      .intr_readbuf_watermark_o        (intr_spi_device_readbuf_watermark),
      .intr_readbuf_flip_o             (intr_spi_device_readbuf_flip),
      .intr_tpm_header_not_empty_o     (intr_spi_device_tpm_header_not_empty),
      .intr_tpm_rdfifo_cmd_end_o       (intr_spi_device_tpm_rdfifo_cmd_end),
      .intr_tpm_rdfifo_drop_o          (intr_spi_device_tpm_rdfifo_drop),
      // [5]: fatal_fault
      .alert_tx_o  ( alert_tx[5:5] ),
      .alert_rx_i  ( alert_rx[5:5] ),

      // Inter-module signals
      .ram_cfg_i(ast_spi_ram_2p_cfg),
      .passthrough_o(spi_device_passthrough_req),
      .passthrough_i(spi_device_passthrough_rsp),
      .mbist_en_i('0),
      .sck_monitor_o(sck_monitor_o),
      .tl_i(spi_device_tl_req),
      .tl_o(spi_device_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .scan_clk_i (clkmgr_aon_clocks.clk_io_div2_peri),
      .rst_ni (rstmgr_aon_resets.rst_spi_device_n[rstmgr_pkg::Domain0Sel])
  );
  i2c #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[6:6]),
    .InputDelayCycles(I2c0InputDelayCycles)
  ) u_i2c0 (

      // Input
      .cio_sda_i    (cio_i2c0_sda_p2d),
      .cio_scl_i    (cio_i2c0_scl_p2d),

      // Output
      .cio_sda_o    (cio_i2c0_sda_d2p),
      .cio_sda_en_o (cio_i2c0_sda_en_d2p),
      .cio_scl_o    (cio_i2c0_scl_d2p),
      .cio_scl_en_o (cio_i2c0_scl_en_d2p),

      // Interrupt
      .intr_fmt_threshold_o    (intr_i2c0_fmt_threshold),
      .intr_rx_threshold_o     (intr_i2c0_rx_threshold),
      .intr_acq_threshold_o    (intr_i2c0_acq_threshold),
      .intr_rx_overflow_o      (intr_i2c0_rx_overflow),
      .intr_controller_halt_o  (intr_i2c0_controller_halt),
      .intr_scl_interference_o (intr_i2c0_scl_interference),
      .intr_sda_interference_o (intr_i2c0_sda_interference),
      .intr_stretch_timeout_o  (intr_i2c0_stretch_timeout),
      .intr_sda_unstable_o     (intr_i2c0_sda_unstable),
      .intr_cmd_complete_o     (intr_i2c0_cmd_complete),
      .intr_tx_stretch_o       (intr_i2c0_tx_stretch),
      .intr_tx_threshold_o     (intr_i2c0_tx_threshold),
      .intr_acq_stretch_o      (intr_i2c0_acq_stretch),
      .intr_unexp_stop_o       (intr_i2c0_unexp_stop),
      .intr_host_timeout_o     (intr_i2c0_host_timeout),
      // [6]: fatal_fault
      .alert_tx_o  ( alert_tx[6:6] ),
      .alert_rx_i  ( alert_rx[6:6] ),

      // Inter-module signals
      .ram_cfg_i(ast_ram_1p_cfg),
      .tl_i(i2c0_tl_req),
      .tl_o(i2c0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_i2c0_n[rstmgr_pkg::Domain0Sel])
  );
  i2c #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[7:7]),
    .InputDelayCycles(I2c1InputDelayCycles)
  ) u_i2c1 (

      // Input
      .cio_sda_i    (cio_i2c1_sda_p2d),
      .cio_scl_i    (cio_i2c1_scl_p2d),

      // Output
      .cio_sda_o    (cio_i2c1_sda_d2p),
      .cio_sda_en_o (cio_i2c1_sda_en_d2p),
      .cio_scl_o    (cio_i2c1_scl_d2p),
      .cio_scl_en_o (cio_i2c1_scl_en_d2p),

      // Interrupt
      .intr_fmt_threshold_o    (intr_i2c1_fmt_threshold),
      .intr_rx_threshold_o     (intr_i2c1_rx_threshold),
      .intr_acq_threshold_o    (intr_i2c1_acq_threshold),
      .intr_rx_overflow_o      (intr_i2c1_rx_overflow),
      .intr_controller_halt_o  (intr_i2c1_controller_halt),
      .intr_scl_interference_o (intr_i2c1_scl_interference),
      .intr_sda_interference_o (intr_i2c1_sda_interference),
      .intr_stretch_timeout_o  (intr_i2c1_stretch_timeout),
      .intr_sda_unstable_o     (intr_i2c1_sda_unstable),
      .intr_cmd_complete_o     (intr_i2c1_cmd_complete),
      .intr_tx_stretch_o       (intr_i2c1_tx_stretch),
      .intr_tx_threshold_o     (intr_i2c1_tx_threshold),
      .intr_acq_stretch_o      (intr_i2c1_acq_stretch),
      .intr_unexp_stop_o       (intr_i2c1_unexp_stop),
      .intr_host_timeout_o     (intr_i2c1_host_timeout),
      // [7]: fatal_fault
      .alert_tx_o  ( alert_tx[7:7] ),
      .alert_rx_i  ( alert_rx[7:7] ),

      // Inter-module signals
      .ram_cfg_i(ast_ram_1p_cfg),
      .tl_i(i2c1_tl_req),
      .tl_o(i2c1_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_i2c1_n[rstmgr_pkg::Domain0Sel])
  );
  i2c #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[8:8]),
    .InputDelayCycles(I2c2InputDelayCycles)
  ) u_i2c2 (

      // Input
      .cio_sda_i    (cio_i2c2_sda_p2d),
      .cio_scl_i    (cio_i2c2_scl_p2d),

      // Output
      .cio_sda_o    (cio_i2c2_sda_d2p),
      .cio_sda_en_o (cio_i2c2_sda_en_d2p),
      .cio_scl_o    (cio_i2c2_scl_d2p),
      .cio_scl_en_o (cio_i2c2_scl_en_d2p),

      // Interrupt
      .intr_fmt_threshold_o    (intr_i2c2_fmt_threshold),
      .intr_rx_threshold_o     (intr_i2c2_rx_threshold),
      .intr_acq_threshold_o    (intr_i2c2_acq_threshold),
      .intr_rx_overflow_o      (intr_i2c2_rx_overflow),
      .intr_controller_halt_o  (intr_i2c2_controller_halt),
      .intr_scl_interference_o (intr_i2c2_scl_interference),
      .intr_sda_interference_o (intr_i2c2_sda_interference),
      .intr_stretch_timeout_o  (intr_i2c2_stretch_timeout),
      .intr_sda_unstable_o     (intr_i2c2_sda_unstable),
      .intr_cmd_complete_o     (intr_i2c2_cmd_complete),
      .intr_tx_stretch_o       (intr_i2c2_tx_stretch),
      .intr_tx_threshold_o     (intr_i2c2_tx_threshold),
      .intr_acq_stretch_o      (intr_i2c2_acq_stretch),
      .intr_unexp_stop_o       (intr_i2c2_unexp_stop),
      .intr_host_timeout_o     (intr_i2c2_host_timeout),
      // [8]: fatal_fault
      .alert_tx_o  ( alert_tx[8:8] ),
      .alert_rx_i  ( alert_rx[8:8] ),

      // Inter-module signals
      .ram_cfg_i(ast_ram_1p_cfg),
      .tl_i(i2c2_tl_req),
      .tl_o(i2c2_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_i2c2_n[rstmgr_pkg::Domain0Sel])
  );
  pattgen #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[9:9])
  ) u_pattgen (

      // Output
      .cio_pda0_tx_o    (cio_pattgen_pda0_tx_d2p),
      .cio_pda0_tx_en_o (cio_pattgen_pda0_tx_en_d2p),
      .cio_pcl0_tx_o    (cio_pattgen_pcl0_tx_d2p),
      .cio_pcl0_tx_en_o (cio_pattgen_pcl0_tx_en_d2p),
      .cio_pda1_tx_o    (cio_pattgen_pda1_tx_d2p),
      .cio_pda1_tx_en_o (cio_pattgen_pda1_tx_en_d2p),
      .cio_pcl1_tx_o    (cio_pattgen_pcl1_tx_d2p),
      .cio_pcl1_tx_en_o (cio_pattgen_pcl1_tx_en_d2p),

      // Interrupt
      .intr_done_ch0_o (intr_pattgen_done_ch0),
      .intr_done_ch1_o (intr_pattgen_done_ch1),
      // [9]: fatal_fault
      .alert_tx_o  ( alert_tx[9:9] ),
      .alert_rx_i  ( alert_rx[9:9] ),

      // Inter-module signals
      .tl_i(pattgen_tl_req),
      .tl_o(pattgen_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  rv_timer #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[10:10])
  ) u_rv_timer (

      // Interrupt
      .intr_timer_expired_hart0_timer0_o (intr_rv_timer_timer_expired_hart0_timer0),
      // [10]: fatal_fault
      .alert_tx_o  ( alert_tx[10:10] ),
      .alert_rx_i  ( alert_rx[10:10] ),

      // Inter-module signals
      .tl_i(rv_timer_tl_req),
      .tl_o(rv_timer_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_timers),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  otp_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[15:11]),
    .MemInitFile(OtpCtrlMemInitFile),
    .RndCnstLfsrSeed(RndCnstOtpCtrlLfsrSeed),
    .RndCnstLfsrPerm(RndCnstOtpCtrlLfsrPerm),
    .RndCnstScrmblKeyInit(RndCnstOtpCtrlScrmblKeyInit)
  ) u_otp_ctrl (

      // Output
      .cio_test_o    (cio_otp_ctrl_test_d2p),
      .cio_test_en_o (cio_otp_ctrl_test_en_d2p),

      // Interrupt
      .intr_otp_operation_done_o (intr_otp_ctrl_otp_operation_done),
      .intr_otp_error_o          (intr_otp_ctrl_otp_error),
      // [11]: fatal_macro_error
      // [12]: fatal_check_error
      // [13]: fatal_bus_integ_error
      // [14]: fatal_prim_otp_alert
      // [15]: recov_prim_otp_alert
      .alert_tx_o  ( alert_tx[15:11] ),
      .alert_rx_i  ( alert_rx[15:11] ),

      // Inter-module signals
      .otp_ext_voltage_h_io(otp_ext_voltage_h_io),
      .otp_ast_pwr_seq_o(otp_ctrl_otp_ast_pwr_seq_o),
      .otp_ast_pwr_seq_h_i(otp_ctrl_otp_ast_pwr_seq_h_i),
      .edn_o(edn0_edn_req[1]),
      .edn_i(edn0_edn_rsp[1]),
      .pwr_otp_i(pwrmgr_aon_pwr_otp_req),
      .pwr_otp_o(pwrmgr_aon_pwr_otp_rsp),
      .lc_otp_vendor_test_i(lc_ctrl_lc_otp_vendor_test_req),
      .lc_otp_vendor_test_o(lc_ctrl_lc_otp_vendor_test_rsp),
      .lc_otp_program_i(lc_ctrl_lc_otp_program_req),
      .lc_otp_program_o(lc_ctrl_lc_otp_program_rsp),
      .otp_lc_data_o(otp_ctrl_otp_lc_data),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_creator_seed_sw_rw_en_i(lc_ctrl_lc_creator_seed_sw_rw_en),
      .lc_owner_seed_sw_rw_en_i(lc_ctrl_pkg::Off),
      .lc_seed_hw_rd_en_i(lc_ctrl_lc_seed_hw_rd_en),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .lc_check_byp_en_i(lc_ctrl_lc_check_byp_en),
      .otp_keymgr_key_o(otp_ctrl_otp_keymgr_key),
      .flash_otp_key_i(flash_ctrl_otp_req),
      .flash_otp_key_o(flash_ctrl_otp_rsp),
      .sram_otp_key_i(otp_ctrl_sram_otp_key_req),
      .sram_otp_key_o(otp_ctrl_sram_otp_key_rsp),
      .otbn_otp_key_i(otp_ctrl_otbn_otp_key_req),
      .otbn_otp_key_o(otp_ctrl_otbn_otp_key_rsp),
      .otp_broadcast_o(otp_ctrl_otp_broadcast),
      .obs_ctrl_i(ast_obs_ctrl),
      .otp_obs_o(otp_obs_o),
      .core_tl_i(otp_ctrl_core_tl_req),
      .core_tl_o(otp_ctrl_core_tl_rsp),
      .prim_tl_i(otp_ctrl_prim_tl_req),
      .prim_tl_o(otp_ctrl_prim_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,
      .scan_en_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  lc_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[18:16]),
    .SecVolatileRawUnlockEn(SecLcCtrlVolatileRawUnlockEn),
    .RndCnstLcKeymgrDivInvalid(RndCnstLcCtrlLcKeymgrDivInvalid),
    .RndCnstLcKeymgrDivTestUnlocked(RndCnstLcCtrlLcKeymgrDivTestUnlocked),
    .RndCnstLcKeymgrDivDev(RndCnstLcCtrlLcKeymgrDivDev),
    .RndCnstLcKeymgrDivProduction(RndCnstLcCtrlLcKeymgrDivProduction),
    .RndCnstLcKeymgrDivRma(RndCnstLcCtrlLcKeymgrDivRma),
    .RndCnstInvalidTokens(RndCnstLcCtrlInvalidTokens),
    .SiliconCreatorId(LcCtrlSiliconCreatorId),
    .ProductId(LcCtrlProductId),
    .RevisionId(LcCtrlRevisionId),
    .IdcodeValue(LcCtrlIdcodeValue)
  ) u_lc_ctrl (
      // [16]: fatal_prog_error
      // [17]: fatal_state_error
      // [18]: fatal_bus_integ_error
      .alert_tx_o  ( alert_tx[18:16] ),
      .alert_rx_i  ( alert_rx[18:16] ),

      // Inter-module signals
      .jtag_i(pinmux_aon_lc_jtag_req),
      .jtag_o(pinmux_aon_lc_jtag_rsp),
      .esc_scrap_state0_tx_i(alert_handler_esc_tx[1]),
      .esc_scrap_state0_rx_o(alert_handler_esc_rx[1]),
      .esc_scrap_state1_tx_i(alert_handler_esc_tx[2]),
      .esc_scrap_state1_rx_o(alert_handler_esc_rx[2]),
      .pwr_lc_i(pwrmgr_aon_pwr_lc_req),
      .pwr_lc_o(pwrmgr_aon_pwr_lc_rsp),
      .lc_otp_vendor_test_o(lc_ctrl_lc_otp_vendor_test_req),
      .lc_otp_vendor_test_i(lc_ctrl_lc_otp_vendor_test_rsp),
      .otp_lc_data_i(otp_ctrl_otp_lc_data),
      .lc_otp_program_o(lc_ctrl_lc_otp_program_req),
      .lc_otp_program_i(lc_ctrl_lc_otp_program_rsp),
      .kmac_data_o(kmac_app_req[1]),
      .kmac_data_i(kmac_app_rsp[1]),
      .lc_dft_en_o(lc_ctrl_lc_dft_en),
      .lc_nvm_debug_en_o(lc_ctrl_lc_nvm_debug_en),
      .lc_hw_debug_en_o(lc_ctrl_lc_hw_debug_en),
      .lc_cpu_en_o(lc_ctrl_lc_cpu_en),
      .lc_keymgr_en_o(lc_ctrl_lc_keymgr_en),
      .lc_escalate_en_o(lc_ctrl_lc_escalate_en),
      .lc_clk_byp_req_o(lc_ctrl_lc_clk_byp_req),
      .lc_clk_byp_ack_i(lc_ctrl_lc_clk_byp_ack),
      .lc_flash_rma_req_o(lc_ctrl_lc_flash_rma_req),
      .lc_flash_rma_ack_i(lc_ctrl_lc_flash_rma_ack),
      .lc_flash_rma_seed_o(flash_ctrl_rma_seed),
      .lc_check_byp_en_o(lc_ctrl_lc_check_byp_en),
      .lc_creator_seed_sw_rw_en_o(lc_ctrl_lc_creator_seed_sw_rw_en),
      .lc_owner_seed_sw_rw_en_o(lc_ctrl_lc_owner_seed_sw_rw_en),
      .lc_iso_part_sw_rd_en_o(lc_ctrl_lc_iso_part_sw_rd_en),
      .lc_iso_part_sw_wr_en_o(lc_ctrl_lc_iso_part_sw_wr_en),
      .lc_seed_hw_rd_en_o(lc_ctrl_lc_seed_hw_rd_en),
      .lc_keymgr_div_o(lc_ctrl_lc_keymgr_div),
      .otp_device_id_i(lc_ctrl_otp_device_id),
      .otp_manuf_state_i(lc_ctrl_otp_manuf_state),
      .hw_rev_o(),
      .strap_en_override_o(lc_ctrl_strap_en_override),
      .tl_i(lc_ctrl_tl_req),
      .tl_o(lc_ctrl_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_kmac_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_kmac_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  alert_handler #(
    .RndCnstLfsrSeed(RndCnstAlertHandlerLfsrSeed),
    .RndCnstLfsrPerm(RndCnstAlertHandlerLfsrPerm)
  ) u_alert_handler (

      // Interrupt
      .intr_classa_o (intr_alert_handler_classa),
      .intr_classb_o (intr_alert_handler_classb),
      .intr_classc_o (intr_alert_handler_classc),
      .intr_classd_o (intr_alert_handler_classd),

      // Inter-module signals
      .crashdump_o(alert_handler_crashdump),
      .edn_o(edn0_edn_req[4]),
      .edn_i(edn0_edn_rsp[4]),
      .esc_rx_i(alert_handler_esc_rx),
      .esc_tx_o(alert_handler_esc_tx),
      .tl_i(alert_handler_tl_req),
      .tl_o(alert_handler_tl_rsp),
      // alert signals
      .alert_rx_o  ( alert_rx ),
      .alert_tx_i  ( alert_tx ),
      // synchronized clock gated / reset asserted
      // indications for each alert
      .lpg_cg_en_i  ( lpg_cg_en  ),
      .lpg_rst_en_i ( lpg_rst_en ),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_io_div4_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  spi_host #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[19:19])
  ) u_spi_host0 (

      // Input
      .cio_sd_i     (cio_spi_host0_sd_p2d),

      // Output
      .cio_sck_o    (cio_spi_host0_sck_d2p),
      .cio_sck_en_o (cio_spi_host0_sck_en_d2p),
      .cio_csb_o    (cio_spi_host0_csb_d2p),
      .cio_csb_en_o (cio_spi_host0_csb_en_d2p),
      .cio_sd_o     (cio_spi_host0_sd_d2p),
      .cio_sd_en_o  (cio_spi_host0_sd_en_d2p),

      // Interrupt
      .intr_error_o     (intr_spi_host0_error),
      .intr_spi_event_o (intr_spi_host0_spi_event),
      // [19]: fatal_fault
      .alert_tx_o  ( alert_tx[19:19] ),
      .alert_rx_i  ( alert_rx[19:19] ),

      // Inter-module signals
      .passthrough_i(spi_device_passthrough_req),
      .passthrough_o(spi_device_passthrough_rsp),
      .tl_i(spi_host0_tl_req),
      .tl_o(spi_host0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_peri),
      .rst_ni (rstmgr_aon_resets.rst_spi_host0_n[rstmgr_pkg::Domain0Sel])
  );
  spi_host #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[20:20])
  ) u_spi_host1 (

      // Input
      .cio_sd_i     (cio_spi_host1_sd_p2d),

      // Output
      .cio_sck_o    (cio_spi_host1_sck_d2p),
      .cio_sck_en_o (cio_spi_host1_sck_en_d2p),
      .cio_csb_o    (cio_spi_host1_csb_d2p),
      .cio_csb_en_o (cio_spi_host1_csb_en_d2p),
      .cio_sd_o     (cio_spi_host1_sd_d2p),
      .cio_sd_en_o  (cio_spi_host1_sd_en_d2p),

      // Interrupt
      .intr_error_o     (intr_spi_host1_error),
      .intr_spi_event_o (intr_spi_host1_spi_event),
      // [20]: fatal_fault
      .alert_tx_o  ( alert_tx[20:20] ),
      .alert_rx_i  ( alert_rx[20:20] ),

      // Inter-module signals
      .passthrough_i(spi_device_pkg::PASSTHROUGH_REQ_DEFAULT),
      .passthrough_o(),
      .tl_i(spi_host1_tl_req),
      .tl_o(spi_host1_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div2_peri),
      .rst_ni (rstmgr_aon_resets.rst_spi_host1_n[rstmgr_pkg::Domain0Sel])
  );
  usbdev #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[21:21]),
    .Stub(UsbdevStub),
    .RcvrWakeTimeUs(UsbdevRcvrWakeTimeUs)
  ) u_usbdev (

      // Input
      .cio_sense_i     (cio_usbdev_sense_p2d),
      .cio_usb_dp_i    (cio_usbdev_usb_dp_p2d),
      .cio_usb_dn_i    (cio_usbdev_usb_dn_p2d),

      // Output
      .cio_usb_dp_o    (cio_usbdev_usb_dp_d2p),
      .cio_usb_dp_en_o (cio_usbdev_usb_dp_en_d2p),
      .cio_usb_dn_o    (cio_usbdev_usb_dn_d2p),
      .cio_usb_dn_en_o (cio_usbdev_usb_dn_en_d2p),

      // Interrupt
      .intr_pkt_received_o    (intr_usbdev_pkt_received),
      .intr_pkt_sent_o        (intr_usbdev_pkt_sent),
      .intr_disconnected_o    (intr_usbdev_disconnected),
      .intr_host_lost_o       (intr_usbdev_host_lost),
      .intr_link_reset_o      (intr_usbdev_link_reset),
      .intr_link_suspend_o    (intr_usbdev_link_suspend),
      .intr_link_resume_o     (intr_usbdev_link_resume),
      .intr_av_out_empty_o    (intr_usbdev_av_out_empty),
      .intr_rx_full_o         (intr_usbdev_rx_full),
      .intr_av_overflow_o     (intr_usbdev_av_overflow),
      .intr_link_in_err_o     (intr_usbdev_link_in_err),
      .intr_rx_crc_err_o      (intr_usbdev_rx_crc_err),
      .intr_rx_pid_err_o      (intr_usbdev_rx_pid_err),
      .intr_rx_bitstuff_err_o (intr_usbdev_rx_bitstuff_err),
      .intr_frame_o           (intr_usbdev_frame),
      .intr_powered_o         (intr_usbdev_powered),
      .intr_link_out_err_o    (intr_usbdev_link_out_err),
      .intr_av_setup_empty_o  (intr_usbdev_av_setup_empty),
      // [21]: fatal_fault
      .alert_tx_o  ( alert_tx[21:21] ),
      .alert_rx_i  ( alert_rx[21:21] ),

      // Inter-module signals
      .usb_rx_d_i(usbdev_usb_rx_d_i),
      .usb_tx_d_o(usbdev_usb_tx_d_o),
      .usb_tx_se0_o(usbdev_usb_tx_se0_o),
      .usb_tx_use_d_se0_o(usbdev_usb_tx_use_d_se0_o),
      .usb_dp_pullup_o(usbdev_usb_dp_pullup),
      .usb_dn_pullup_o(usbdev_usb_dn_pullup),
      .usb_rx_enable_o(usbdev_usb_rx_enable_o),
      .usb_ref_val_o(usbdev_usb_ref_val_o),
      .usb_ref_pulse_o(usbdev_usb_ref_pulse_o),
      .usb_aon_suspend_req_o(usbdev_usb_aon_suspend_req),
      .usb_aon_wake_ack_o(usbdev_usb_aon_wake_ack),
      .usb_aon_bus_reset_i(usbdev_usb_aon_bus_reset),
      .usb_aon_sense_lost_i(usbdev_usb_aon_sense_lost),
      .usb_aon_bus_not_idle_i(usbdev_usb_aon_bus_not_idle),
      .usb_aon_wake_detect_active_i(pinmux_aon_usbdev_wake_detect_active),
      .ram_cfg_i(ast_usb_ram_1p_cfg),
      .tl_i(usbdev_tl_req),
      .tl_o(usbdev_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_usb_peri),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_peri),
      .rst_ni (rstmgr_aon_resets.rst_usb_n[rstmgr_pkg::Domain0Sel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_usb_aon_n[rstmgr_pkg::Domain0Sel])
  );
  pwrmgr #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[22:22])
  ) u_pwrmgr_aon (

      // Interrupt
      .intr_wakeup_o (intr_pwrmgr_aon_wakeup),
      // [22]: fatal_fault
      .alert_tx_o  ( alert_tx[22:22] ),
      .alert_rx_i  ( alert_rx[22:22] ),

      // Inter-module signals
      .pwr_ast_o(pwrmgr_ast_req_o),
      .pwr_ast_i(pwrmgr_ast_rsp_i),
      .pwr_rst_o(pwrmgr_aon_pwr_rst_req),
      .pwr_rst_i(pwrmgr_aon_pwr_rst_rsp),
      .pwr_clk_o(pwrmgr_aon_pwr_clk_req),
      .pwr_clk_i(pwrmgr_aon_pwr_clk_rsp),
      .pwr_otp_o(pwrmgr_aon_pwr_otp_req),
      .pwr_otp_i(pwrmgr_aon_pwr_otp_rsp),
      .pwr_lc_o(pwrmgr_aon_pwr_lc_req),
      .pwr_lc_i(pwrmgr_aon_pwr_lc_rsp),
      .pwr_flash_i(pwrmgr_aon_pwr_flash),
      .esc_rst_tx_i(alert_handler_esc_tx[3]),
      .esc_rst_rx_o(alert_handler_esc_rx[3]),
      .pwr_cpu_i(rv_core_ibex_pwrmgr),
      .wakeups_i(pwrmgr_aon_wakeups),
      .rstreqs_i(pwrmgr_aon_rstreqs),
      .ndmreset_req_i(rv_dm_ndmreset_req),
      .strap_o(pwrmgr_aon_strap),
      .low_power_o(pwrmgr_aon_low_power),
      .rom_ctrl_i(rom_ctrl_pwrmgr_data),
      .fetch_en_o(pwrmgr_aon_fetch_en),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .sw_rst_req_i(rstmgr_aon_sw_rst_req),
      .tl_i(pwrmgr_aon_tl_req),
      .tl_o(pwrmgr_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_slow_i (clkmgr_aon_clocks.clk_aon_powerup),
      .clk_lc_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_esc_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_main_ni (rstmgr_aon_resets.rst_por_aon_n[rstmgr_pkg::Domain0Sel]),
      .rst_lc_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_esc_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_slow_ni (rstmgr_aon_resets.rst_por_aon_n[rstmgr_pkg::DomainAonSel])
  );
  rstmgr #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[24:23]),
    .SecCheck(SecRstmgrAonCheck),
    .SecMaxSyncDelay(SecRstmgrAonMaxSyncDelay)
  ) u_rstmgr_aon (
      // [23]: fatal_fault
      // [24]: fatal_cnsty_fault
      .alert_tx_o  ( alert_tx[24:23] ),
      .alert_rx_i  ( alert_rx[24:23] ),

      // Inter-module signals
      .por_n_i(por_n_i),
      .pwr_i(pwrmgr_aon_pwr_rst_req),
      .pwr_o(pwrmgr_aon_pwr_rst_rsp),
      .resets_o(rstmgr_aon_resets),
      .rst_en_o(rstmgr_aon_rst_en),
      .alert_dump_i(alert_handler_crashdump),
      .cpu_dump_i(rv_core_ibex_crash_dump),
      .sw_rst_req_o(rstmgr_aon_sw_rst_req),
      .tl_i(rstmgr_aon_tl_req),
      .tl_o(rstmgr_aon_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_por_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_powerup),
      .clk_main_i (clkmgr_aon_clocks.clk_main_powerup),
      .clk_io_i (clkmgr_aon_clocks.clk_io_powerup),
      .clk_usb_i (clkmgr_aon_clocks.clk_usb_powerup),
      .clk_io_div2_i (clkmgr_aon_clocks.clk_io_div2_powerup),
      .clk_io_div4_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_por_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel])
  );
  clkmgr #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[26:25])
  ) u_clkmgr_aon (
      // [25]: recov_fault
      // [26]: fatal_fault
      .alert_tx_o  ( alert_tx[26:25] ),
      .alert_rx_i  ( alert_rx[26:25] ),

      // Inter-module signals
      .clocks_o(clkmgr_aon_clocks),
      .cg_en_o(clkmgr_aon_cg_en),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .io_clk_byp_req_o(io_clk_byp_req_o),
      .io_clk_byp_ack_i(io_clk_byp_ack_i),
      .all_clk_byp_req_o(all_clk_byp_req_o),
      .all_clk_byp_ack_i(all_clk_byp_ack_i),
      .hi_speed_sel_o(hi_speed_sel_o),
      .div_step_down_req_i(div_step_down_req_i),
      .lc_clk_byp_req_i(lc_ctrl_lc_clk_byp_req),
      .lc_clk_byp_ack_o(lc_ctrl_lc_clk_byp_ack),
      .jitter_en_o(clk_main_jitter_en_o),
      .pwr_i(pwrmgr_aon_pwr_clk_req),
      .pwr_o(pwrmgr_aon_pwr_clk_rsp),
      .idle_i(clkmgr_aon_idle),
      .calib_rdy_i(calib_rdy_i),
      .tl_i(clkmgr_aon_tl_req),
      .tl_o(clkmgr_aon_tl_rsp),
      .scanmode_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_main_i (clk_main_i),
      .clk_io_i (clk_io_i),
      .clk_usb_i (clk_usb_i),
      .clk_aon_i (clk_aon_i),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_io_div4_shadowed_n[rstmgr_pkg::DomainAonSel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel]),
      .rst_io_ni (rstmgr_aon_resets.rst_lc_io_n[rstmgr_pkg::DomainAonSel]),
      .rst_io_div2_ni (rstmgr_aon_resets.rst_lc_io_div2_n[rstmgr_pkg::DomainAonSel]),
      .rst_io_div4_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_main_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::DomainAonSel]),
      .rst_usb_ni (rstmgr_aon_resets.rst_lc_usb_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_io_ni (rstmgr_aon_resets.rst_por_io_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_io_div2_ni (rstmgr_aon_resets.rst_por_io_div2_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_io_div4_ni (rstmgr_aon_resets.rst_por_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_main_ni (rstmgr_aon_resets.rst_por_n[rstmgr_pkg::DomainAonSel]),
      .rst_root_usb_ni (rstmgr_aon_resets.rst_por_usb_n[rstmgr_pkg::DomainAonSel])
  );
  sysrst_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[27:27])
  ) u_sysrst_ctrl_aon (

      // Input
      .cio_ac_present_i     (cio_sysrst_ctrl_aon_ac_present_p2d),
      .cio_key0_in_i        (cio_sysrst_ctrl_aon_key0_in_p2d),
      .cio_key1_in_i        (cio_sysrst_ctrl_aon_key1_in_p2d),
      .cio_key2_in_i        (cio_sysrst_ctrl_aon_key2_in_p2d),
      .cio_pwrb_in_i        (cio_sysrst_ctrl_aon_pwrb_in_p2d),
      .cio_lid_open_i       (cio_sysrst_ctrl_aon_lid_open_p2d),
      .cio_ec_rst_l_i       (cio_sysrst_ctrl_aon_ec_rst_l_p2d),
      .cio_flash_wp_l_i     (cio_sysrst_ctrl_aon_flash_wp_l_p2d),

      // Output
      .cio_bat_disable_o    (cio_sysrst_ctrl_aon_bat_disable_d2p),
      .cio_bat_disable_en_o (cio_sysrst_ctrl_aon_bat_disable_en_d2p),
      .cio_key0_out_o       (cio_sysrst_ctrl_aon_key0_out_d2p),
      .cio_key0_out_en_o    (cio_sysrst_ctrl_aon_key0_out_en_d2p),
      .cio_key1_out_o       (cio_sysrst_ctrl_aon_key1_out_d2p),
      .cio_key1_out_en_o    (cio_sysrst_ctrl_aon_key1_out_en_d2p),
      .cio_key2_out_o       (cio_sysrst_ctrl_aon_key2_out_d2p),
      .cio_key2_out_en_o    (cio_sysrst_ctrl_aon_key2_out_en_d2p),
      .cio_pwrb_out_o       (cio_sysrst_ctrl_aon_pwrb_out_d2p),
      .cio_pwrb_out_en_o    (cio_sysrst_ctrl_aon_pwrb_out_en_d2p),
      .cio_z3_wakeup_o      (cio_sysrst_ctrl_aon_z3_wakeup_d2p),
      .cio_z3_wakeup_en_o   (cio_sysrst_ctrl_aon_z3_wakeup_en_d2p),
      .cio_ec_rst_l_o       (cio_sysrst_ctrl_aon_ec_rst_l_d2p),
      .cio_ec_rst_l_en_o    (cio_sysrst_ctrl_aon_ec_rst_l_en_d2p),
      .cio_flash_wp_l_o     (cio_sysrst_ctrl_aon_flash_wp_l_d2p),
      .cio_flash_wp_l_en_o  (cio_sysrst_ctrl_aon_flash_wp_l_en_d2p),

      // Interrupt
      .intr_event_detected_o (intr_sysrst_ctrl_aon_event_detected),
      // [27]: fatal_fault
      .alert_tx_o  ( alert_tx[27:27] ),
      .alert_rx_i  ( alert_rx[27:27] ),

      // Inter-module signals
      .wkup_req_o(pwrmgr_aon_wakeups[0]),
      .rst_req_o(pwrmgr_aon_rstreqs[0]),
      .tl_i(sysrst_ctrl_aon_tl_req),
      .tl_o(sysrst_ctrl_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel])
  );
  adc_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[28:28])
  ) u_adc_ctrl_aon (

      // Interrupt
      .intr_match_pending_o (intr_adc_ctrl_aon_match_pending),
      // [28]: fatal_fault
      .alert_tx_o  ( alert_tx[28:28] ),
      .alert_rx_i  ( alert_rx[28:28] ),

      // Inter-module signals
      .adc_o(adc_req_o),
      .adc_i(adc_rsp_i),
      .wkup_req_o(pwrmgr_aon_wakeups[1]),
      .tl_i(adc_ctrl_aon_tl_req),
      .tl_o(adc_ctrl_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel])
  );
  pwm #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[29:29])
  ) u_pwm_aon (

      // Output
      .cio_pwm_o    (cio_pwm_aon_pwm_d2p),
      .cio_pwm_en_o (cio_pwm_aon_pwm_en_d2p),
      // [29]: fatal_fault
      .alert_tx_o  ( alert_tx[29:29] ),
      .alert_rx_i  ( alert_rx[29:29] ),

      // Inter-module signals
      .tl_i(pwm_aon_tl_req),
      .tl_o(pwm_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_peri),
      .clk_core_i (clkmgr_aon_clocks.clk_aon_peri),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_core_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel])
  );
  pinmux #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[30:30]),
    .SecVolatileRawUnlockEn(SecPinmuxAonVolatileRawUnlockEn),
    .TargetCfg(PinmuxAonTargetCfg)
  ) u_pinmux_aon (
      // [30]: fatal_fault
      .alert_tx_o  ( alert_tx[30:30] ),
      .alert_rx_i  ( alert_rx[30:30] ),

      // Inter-module signals
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_check_byp_en_i(lc_ctrl_lc_check_byp_en),
      .pinmux_hw_debug_en_o(pinmux_aon_pinmux_hw_debug_en),
      .lc_jtag_o(pinmux_aon_lc_jtag_req),
      .lc_jtag_i(pinmux_aon_lc_jtag_rsp),
      .rv_jtag_o(pinmux_aon_rv_jtag_req),
      .rv_jtag_i(pinmux_aon_rv_jtag_rsp),
      .dft_jtag_o(pinmux_aon_dft_jtag_req),
      .dft_jtag_i(pinmux_aon_dft_jtag_rsp),
      .dft_strap_test_o(dft_strap_test_o),
      .dft_hold_tap_sel_i(dft_hold_tap_sel_i),
      .sleep_en_i(pwrmgr_aon_low_power),
      .strap_en_i(pwrmgr_aon_strap),
      .strap_en_override_i(lc_ctrl_strap_en_override),
      .pin_wkup_req_o(pwrmgr_aon_wakeups[2]),
      .usbdev_dppullup_en_i(usbdev_usb_dp_pullup),
      .usbdev_dnpullup_en_i(usbdev_usb_dn_pullup),
      .usb_dppullup_en_o(usb_dp_pullup_en_o),
      .usb_dnpullup_en_o(usb_dn_pullup_en_o),
      .usb_wkup_req_o(pwrmgr_aon_wakeups[3]),
      .usbdev_suspend_req_i(usbdev_usb_aon_suspend_req),
      .usbdev_wake_ack_i(usbdev_usb_aon_wake_ack),
      .usbdev_bus_not_idle_o(usbdev_usb_aon_bus_not_idle),
      .usbdev_bus_reset_o(usbdev_usb_aon_bus_reset),
      .usbdev_sense_lost_o(usbdev_usb_aon_sense_lost),
      .usbdev_wake_detect_active_o(pinmux_aon_usbdev_wake_detect_active),
      .tl_i(pinmux_aon_tl_req),
      .tl_o(pinmux_aon_tl_rsp),

      .periph_to_mio_i      (mio_d2p    ),
      .periph_to_mio_oe_i   (mio_en_d2p ),
      .mio_to_periph_o      (mio_p2d    ),

      .mio_attr_o,
      .mio_out_o,
      .mio_oe_o,
      .mio_in_i,

      .periph_to_dio_i      (dio_d2p    ),
      .periph_to_dio_oe_i   (dio_en_d2p ),
      .dio_to_periph_o      (dio_p2d    ),

      .dio_attr_o,
      .dio_out_o,
      .dio_oe_o,
      .dio_in_i,

      .scanmode_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_powerup),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_powerup),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel]),
      .rst_sys_ni (rstmgr_aon_resets.rst_sys_io_div4_n[rstmgr_pkg::DomainAonSel])
  );
  aon_timer #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[31:31])
  ) u_aon_timer_aon (

      // Interrupt
      .intr_wkup_timer_expired_o (intr_aon_timer_aon_wkup_timer_expired),
      .intr_wdog_timer_bark_o    (intr_aon_timer_aon_wdog_timer_bark),
      // [31]: fatal_fault
      .alert_tx_o  ( alert_tx[31:31] ),
      .alert_rx_i  ( alert_rx[31:31] ),

      // Inter-module signals
      .nmi_wdog_timer_bark_o(aon_timer_aon_nmi_wdog_timer_bark),
      .wkup_req_o(pwrmgr_aon_wakeups[4]),
      .aon_timer_rst_req_o(pwrmgr_aon_rstreqs[1]),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .sleep_mode_i(pwrmgr_aon_low_power),
      .tl_i(aon_timer_aon_tl_req),
      .tl_o(aon_timer_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_timers),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_timers),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel])
  );
  sensor_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[33:32])
  ) u_sensor_ctrl_aon (

      // Output
      .cio_ast_debug_out_o    (cio_sensor_ctrl_aon_ast_debug_out_d2p),
      .cio_ast_debug_out_en_o (cio_sensor_ctrl_aon_ast_debug_out_en_d2p),

      // Interrupt
      .intr_io_status_change_o   (intr_sensor_ctrl_aon_io_status_change),
      .intr_init_status_change_o (intr_sensor_ctrl_aon_init_status_change),
      // [32]: recov_alert
      // [33]: fatal_alert
      .alert_tx_o  ( alert_tx[33:32] ),
      .alert_rx_i  ( alert_rx[33:32] ),

      // Inter-module signals
      .ast_alert_i(sensor_ctrl_ast_alert_req_i),
      .ast_alert_o(sensor_ctrl_ast_alert_rsp_o),
      .ast_status_i(sensor_ctrl_ast_status_i),
      .ast_init_done_i(ast_init_done_i),
      .ast2pinmux_i(ast2pinmux_i),
      .wkup_req_o(pwrmgr_aon_wakeups[5]),
      .manual_pad_attr_o(sensor_ctrl_manual_pad_attr_o),
      .tl_i(sensor_ctrl_aon_tl_req),
      .tl_o(sensor_ctrl_aon_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_aon_i (clkmgr_aon_clocks.clk_aon_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_aon_ni (rstmgr_aon_resets.rst_lc_aon_n[rstmgr_pkg::DomainAonSel])
  );
  sram_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[34:34]),
    .RndCnstSramKey(RndCnstSramCtrlRetAonSramKey),
    .RndCnstSramNonce(RndCnstSramCtrlRetAonSramNonce),
    .RndCnstLfsrSeed(RndCnstSramCtrlRetAonLfsrSeed),
    .RndCnstLfsrPerm(RndCnstSramCtrlRetAonLfsrPerm),
    .MemSizeRam(4096),
    .InstrExec(SramCtrlRetAonInstrExec)
  ) u_sram_ctrl_ret_aon (
      // [34]: fatal_error
      .alert_tx_o  ( alert_tx[34:34] ),
      .alert_rx_i  ( alert_rx[34:34] ),

      // Inter-module signals
      .sram_otp_key_o(otp_ctrl_sram_otp_key_req[1]),
      .sram_otp_key_i(otp_ctrl_sram_otp_key_rsp[1]),
      .cfg_i(ast_ram_1p_cfg),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_hw_debug_en_i(lc_ctrl_pkg::Off),
      .otp_en_sram_ifetch_i(prim_mubi_pkg::MuBi8False),
      .regs_tl_i(sram_ctrl_ret_aon_regs_tl_req),
      .regs_tl_o(sram_ctrl_ret_aon_regs_tl_rsp),
      .ram_tl_i(sram_ctrl_ret_aon_ram_tl_req),
      .ram_tl_o(sram_ctrl_ret_aon_ram_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::DomainAonSel])
  );
  flash_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[39:35]),
    .RndCnstAddrKey(RndCnstFlashCtrlAddrKey),
    .RndCnstDataKey(RndCnstFlashCtrlDataKey),
    .RndCnstAllSeeds(RndCnstFlashCtrlAllSeeds),
    .RndCnstLfsrSeed(RndCnstFlashCtrlLfsrSeed),
    .RndCnstLfsrPerm(RndCnstFlashCtrlLfsrPerm),
    .SecScrambleEn(SecFlashCtrlScrambleEn),
    .ProgFifoDepth(FlashCtrlProgFifoDepth),
    .RdFifoDepth(FlashCtrlRdFifoDepth)
  ) u_flash_ctrl (

      // Input
      .cio_tck_i    (cio_flash_ctrl_tck_p2d),
      .cio_tms_i    (cio_flash_ctrl_tms_p2d),
      .cio_tdi_i    (cio_flash_ctrl_tdi_p2d),

      // Output
      .cio_tdo_o    (cio_flash_ctrl_tdo_d2p),
      .cio_tdo_en_o (cio_flash_ctrl_tdo_en_d2p),

      // Interrupt
      .intr_prog_empty_o (intr_flash_ctrl_prog_empty),
      .intr_prog_lvl_o   (intr_flash_ctrl_prog_lvl),
      .intr_rd_full_o    (intr_flash_ctrl_rd_full),
      .intr_rd_lvl_o     (intr_flash_ctrl_rd_lvl),
      .intr_op_done_o    (intr_flash_ctrl_op_done),
      .intr_corr_err_o   (intr_flash_ctrl_corr_err),
      // [35]: recov_err
      // [36]: fatal_std_err
      // [37]: fatal_err
      // [38]: fatal_prim_flash_alert
      // [39]: recov_prim_flash_alert
      .alert_tx_o  ( alert_tx[39:35] ),
      .alert_rx_i  ( alert_rx[39:35] ),

      // Inter-module signals
      .otp_o(flash_ctrl_otp_req),
      .otp_i(flash_ctrl_otp_rsp),
      .lc_nvm_debug_en_i(lc_ctrl_lc_nvm_debug_en),
      .flash_bist_enable_i(flash_bist_enable_i),
      .flash_power_down_h_i(flash_power_down_h_i),
      .flash_power_ready_h_i(flash_power_ready_h_i),
      .flash_test_mode_a_io(flash_test_mode_a_io),
      .flash_test_voltage_h_io(flash_test_voltage_h_io),
      .lc_creator_seed_sw_rw_en_i(lc_ctrl_lc_creator_seed_sw_rw_en),
      .lc_owner_seed_sw_rw_en_i(lc_ctrl_lc_owner_seed_sw_rw_en),
      .lc_iso_part_sw_rd_en_i(lc_ctrl_lc_iso_part_sw_rd_en),
      .lc_iso_part_sw_wr_en_i(lc_ctrl_lc_iso_part_sw_wr_en),
      .lc_seed_hw_rd_en_i(lc_ctrl_lc_seed_hw_rd_en),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .rma_req_i(lc_ctrl_lc_flash_rma_req),
      .rma_ack_o(lc_ctrl_lc_flash_rma_ack[0]),
      .rma_seed_i(flash_ctrl_rma_seed),
      .pwrmgr_o(pwrmgr_aon_pwr_flash),
      .keymgr_o(flash_ctrl_keymgr),
      .obs_ctrl_i(ast_obs_ctrl),
      .fla_obs_o(flash_obs_o),
      .core_tl_i(flash_ctrl_core_tl_req),
      .core_tl_o(flash_ctrl_core_tl_rsp),
      .prim_tl_i(flash_ctrl_prim_tl_req),
      .prim_tl_o(flash_ctrl_prim_tl_rsp),
      .mem_tl_i(flash_ctrl_mem_tl_req),
      .mem_tl_o(flash_ctrl_mem_tl_rsp),
      .scanmode_i,
      .scan_rst_ni,
      .scan_en_i,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  rv_dm #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[40:40]),
    .IdcodeValue(RvDmIdcodeValue)
  ) u_rv_dm (
      // [40]: fatal_fault
      .alert_tx_o  ( alert_tx[40:40] ),
      .alert_rx_i  ( alert_rx[40:40] ),

      // Inter-module signals
      .next_dm_addr_i('0),
      .jtag_i(pinmux_aon_rv_jtag_req),
      .jtag_o(pinmux_aon_rv_jtag_rsp),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .lc_dft_en_i(lc_ctrl_lc_dft_en),
      .pinmux_hw_debug_en_i(pinmux_aon_pinmux_hw_debug_en),
      .otp_dis_rv_dm_late_debug_i(rv_dm_otp_dis_rv_dm_late_debug),
      .unavailable_i(1'b0),
      .ndmreset_req_o(rv_dm_ndmreset_req),
      .dmactive_o(),
      .debug_req_o(rv_dm_debug_req),
      .sba_tl_h_o(main_tl_rv_dm__sba_req),
      .sba_tl_h_i(main_tl_rv_dm__sba_rsp),
      .regs_tl_d_i(rv_dm_regs_tl_d_req),
      .regs_tl_d_o(rv_dm_regs_tl_d_rsp),
      .mem_tl_d_i(rv_dm_mem_tl_d_req),
      .mem_tl_d_o(rv_dm_mem_tl_d_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_lc_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_sys_n[rstmgr_pkg::Domain0Sel]),
      .rst_lc_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  rv_plic #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[41:41])
  ) u_rv_plic (
      // [41]: fatal_fault
      .alert_tx_o  ( alert_tx[41:41] ),
      .alert_rx_i  ( alert_rx[41:41] ),

      // Inter-module signals
      .irq_o(rv_plic_irq),
      .irq_id_o(),
      .msip_o(rv_plic_msip),
      .tl_i(rv_plic_tl_req),
      .tl_o(rv_plic_tl_rsp),
      .intr_src_i (intr_vector),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  aes #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[43:42]),
    .AES192Enable(1'b1),
    .SecMasking(SecAesMasking),
    .SecSBoxImpl(SecAesSBoxImpl),
    .SecStartTriggerDelay(SecAesStartTriggerDelay),
    .SecAllowForcingMasks(SecAesAllowForcingMasks),
    .SecSkipPRNGReseeding(SecAesSkipPRNGReseeding),
    .RndCnstClearingLfsrSeed(RndCnstAesClearingLfsrSeed),
    .RndCnstClearingLfsrPerm(RndCnstAesClearingLfsrPerm),
    .RndCnstClearingSharePerm(RndCnstAesClearingSharePerm),
    .RndCnstMaskingLfsrSeed(RndCnstAesMaskingLfsrSeed),
    .RndCnstMaskingLfsrPerm(RndCnstAesMaskingLfsrPerm)
  ) u_aes (
      // [42]: recov_ctrl_update_err
      // [43]: fatal_fault
      .alert_tx_o  ( alert_tx[43:42] ),
      .alert_rx_i  ( alert_rx[43:42] ),

      // Inter-module signals
      .idle_o(clkmgr_aon_idle[0]),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .edn_o(edn0_edn_req[5]),
      .edn_i(edn0_edn_rsp[5]),
      .keymgr_key_i(keymgr_aes_key),
      .tl_i(aes_tl_req),
      .tl_o(aes_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_aes),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_aes),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  hmac #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[44:44])
  ) u_hmac (

      // Interrupt
      .intr_hmac_done_o  (intr_hmac_hmac_done),
      .intr_fifo_empty_o (intr_hmac_fifo_empty),
      .intr_hmac_err_o   (intr_hmac_hmac_err),
      // [44]: fatal_fault
      .alert_tx_o  ( alert_tx[44:44] ),
      .alert_rx_i  ( alert_rx[44:44] ),

      // Inter-module signals
      .idle_o(clkmgr_aon_idle[1]),
      .tl_i(hmac_tl_req),
      .tl_o(hmac_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_hmac),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  kmac #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[46:45]),
    .EnMasking(KmacEnMasking),
    .SwKeyMasked(KmacSwKeyMasked),
    .SecCmdDelay(SecKmacCmdDelay),
    .SecIdleAcceptSwMsg(SecKmacIdleAcceptSwMsg),
    .RndCnstLfsrSeed(RndCnstKmacLfsrSeed),
    .RndCnstLfsrPerm(RndCnstKmacLfsrPerm),
    .RndCnstBufferLfsrSeed(RndCnstKmacBufferLfsrSeed),
    .RndCnstMsgPerm(RndCnstKmacMsgPerm)
  ) u_kmac (

      // Interrupt
      .intr_kmac_done_o  (intr_kmac_kmac_done),
      .intr_fifo_empty_o (intr_kmac_fifo_empty),
      .intr_kmac_err_o   (intr_kmac_kmac_err),
      // [45]: recov_operation_err
      // [46]: fatal_fault_err
      .alert_tx_o  ( alert_tx[46:45] ),
      .alert_rx_i  ( alert_rx[46:45] ),

      // Inter-module signals
      .keymgr_key_i(keymgr_kmac_key),
      .app_i(kmac_app_req),
      .app_o(kmac_app_rsp),
      .entropy_o(edn0_edn_req[3]),
      .entropy_i(edn0_edn_rsp[3]),
      .idle_o(clkmgr_aon_idle[2]),
      .en_masking_o(kmac_en_masking),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .tl_i(kmac_tl_req),
      .tl_o(kmac_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_kmac),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_kmac),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  otbn #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[48:47]),
    .Stub(OtbnStub),
    .RegFile(OtbnRegFile),
    .RndCnstUrndPrngSeed(RndCnstOtbnUrndPrngSeed),
    .SecMuteUrnd(SecOtbnMuteUrnd),
    .SecSkipUrndReseedAtStart(SecOtbnSkipUrndReseedAtStart),
    .RndCnstOtbnKey(RndCnstOtbnOtbnKey),
    .RndCnstOtbnNonce(RndCnstOtbnOtbnNonce)
  ) u_otbn (

      // Interrupt
      .intr_done_o (intr_otbn_done),
      // [47]: fatal
      // [48]: recov
      .alert_tx_o  ( alert_tx[48:47] ),
      .alert_rx_i  ( alert_rx[48:47] ),

      // Inter-module signals
      .otbn_otp_key_o(otp_ctrl_otbn_otp_key_req),
      .otbn_otp_key_i(otp_ctrl_otbn_otp_key_rsp),
      .edn_rnd_o(edn1_edn_req[0]),
      .edn_rnd_i(edn1_edn_rsp[0]),
      .edn_urnd_o(edn0_edn_req[6]),
      .edn_urnd_i(edn0_edn_rsp[6]),
      .idle_o(clkmgr_aon_idle[3]),
      .ram_cfg_i(ast_ram_1p_cfg),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_rma_req_i(lc_ctrl_lc_flash_rma_req),
      .lc_rma_ack_o(lc_ctrl_lc_flash_rma_ack[1]),
      .keymgr_key_i(keymgr_otbn_key),
      .tl_i(otbn_tl_req),
      .tl_o(otbn_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_otbn),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  keymgr #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[50:49]),
    .UseOtpSeedsInsteadOfFlash(KeymgrUseOtpSeedsInsteadOfFlash),
    .KmacEnMasking(KeymgrKmacEnMasking),
    .RndCnstLfsrSeed(RndCnstKeymgrLfsrSeed),
    .RndCnstLfsrPerm(RndCnstKeymgrLfsrPerm),
    .RndCnstRandPerm(RndCnstKeymgrRandPerm),
    .RndCnstRevisionSeed(RndCnstKeymgrRevisionSeed),
    .RndCnstCreatorIdentitySeed(RndCnstKeymgrCreatorIdentitySeed),
    .RndCnstOwnerIntIdentitySeed(RndCnstKeymgrOwnerIntIdentitySeed),
    .RndCnstOwnerIdentitySeed(RndCnstKeymgrOwnerIdentitySeed),
    .RndCnstSoftOutputSeed(RndCnstKeymgrSoftOutputSeed),
    .RndCnstHardOutputSeed(RndCnstKeymgrHardOutputSeed),
    .RndCnstAesSeed(RndCnstKeymgrAesSeed),
    .RndCnstKmacSeed(RndCnstKeymgrKmacSeed),
    .RndCnstOtbnSeed(RndCnstKeymgrOtbnSeed),
    .RndCnstCdi(RndCnstKeymgrCdi),
    .RndCnstNoneSeed(RndCnstKeymgrNoneSeed)
  ) u_keymgr (

      // Interrupt
      .intr_op_done_o (intr_keymgr_op_done),
      // [49]: recov_operation_err
      // [50]: fatal_fault_err
      .alert_tx_o  ( alert_tx[50:49] ),
      .alert_rx_i  ( alert_rx[50:49] ),

      // Inter-module signals
      .edn_o(edn0_edn_req[0]),
      .edn_i(edn0_edn_rsp[0]),
      .aes_key_o(keymgr_aes_key),
      .kmac_key_o(keymgr_kmac_key),
      .otbn_key_o(keymgr_otbn_key),
      .kmac_data_o(kmac_app_req[0]),
      .kmac_data_i(kmac_app_rsp[0]),
      .otp_key_i(otp_ctrl_otp_keymgr_key),
      .otp_device_id_i(keymgr_otp_device_id),
      .flash_i(flash_ctrl_keymgr),
      .lc_keymgr_en_i(lc_ctrl_lc_keymgr_en),
      .lc_keymgr_div_i(lc_ctrl_lc_keymgr_div),
      .rom_digest_i(rom_ctrl_keymgr_data),
      .kmac_en_masking_i(kmac_en_masking),
      .tl_i(keymgr_tl_req),
      .tl_o(keymgr_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_shadowed_ni (rstmgr_aon_resets.rst_lc_shadowed_n[rstmgr_pkg::Domain0Sel]),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  csrng #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[52:51]),
    .RndCnstCsKeymgrDivNonProduction(RndCnstCsrngCsKeymgrDivNonProduction),
    .RndCnstCsKeymgrDivProduction(RndCnstCsrngCsKeymgrDivProduction),
    .SBoxImpl(CsrngSBoxImpl)
  ) u_csrng (

      // Interrupt
      .intr_cs_cmd_req_done_o (intr_csrng_cs_cmd_req_done),
      .intr_cs_entropy_req_o  (intr_csrng_cs_entropy_req),
      .intr_cs_hw_inst_exc_o  (intr_csrng_cs_hw_inst_exc),
      .intr_cs_fatal_err_o    (intr_csrng_cs_fatal_err),
      // [51]: recov_alert
      // [52]: fatal_alert
      .alert_tx_o  ( alert_tx[52:51] ),
      .alert_rx_i  ( alert_rx[52:51] ),

      // Inter-module signals
      .csrng_cmd_i(csrng_csrng_cmd_req),
      .csrng_cmd_o(csrng_csrng_cmd_rsp),
      .entropy_src_hw_if_o(csrng_entropy_src_hw_if_req),
      .entropy_src_hw_if_i(csrng_entropy_src_hw_if_rsp),
      .cs_aes_halt_i(csrng_cs_aes_halt_req),
      .cs_aes_halt_o(csrng_cs_aes_halt_rsp),
      .otp_en_csrng_sw_app_read_i(csrng_otp_en_csrng_sw_app_read),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .tl_i(csrng_tl_req),
      .tl_o(csrng_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  entropy_src #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[54:53]),
    .EsFifoDepth(EntropySrcEsFifoDepth),
    .DistrFifoDepth(EntropySrcDistrFifoDepth),
    .Stub(EntropySrcStub)
  ) u_entropy_src (

      // Interrupt
      .intr_es_entropy_valid_o      (intr_entropy_src_es_entropy_valid),
      .intr_es_health_test_failed_o (intr_entropy_src_es_health_test_failed),
      .intr_es_observe_fifo_ready_o (intr_entropy_src_es_observe_fifo_ready),
      .intr_es_fatal_err_o          (intr_entropy_src_es_fatal_err),
      // [53]: recov_alert
      // [54]: fatal_alert
      .alert_tx_o  ( alert_tx[54:53] ),
      .alert_rx_i  ( alert_rx[54:53] ),

      // Inter-module signals
      .entropy_src_hw_if_i(csrng_entropy_src_hw_if_req),
      .entropy_src_hw_if_o(csrng_entropy_src_hw_if_rsp),
      .cs_aes_halt_o(csrng_cs_aes_halt_req),
      .cs_aes_halt_i(csrng_cs_aes_halt_rsp),
      .entropy_src_rng_o(es_rng_req_o),
      .entropy_src_rng_i(es_rng_rsp_i),
      .entropy_src_xht_o(),
      .entropy_src_xht_i(entropy_src_pkg::ENTROPY_SRC_XHT_RSP_DEFAULT),
      .otp_en_entropy_src_fw_read_i(prim_mubi_pkg::MuBi8True),
      .otp_en_entropy_src_fw_over_i(prim_mubi_pkg::MuBi8True),
      .rng_fips_o(es_rng_fips_o),
      .tl_i(entropy_src_tl_req),
      .tl_o(entropy_src_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  edn #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[56:55])
  ) u_edn0 (

      // Interrupt
      .intr_edn_cmd_req_done_o (intr_edn0_edn_cmd_req_done),
      .intr_edn_fatal_err_o    (intr_edn0_edn_fatal_err),
      // [55]: recov_alert
      // [56]: fatal_alert
      .alert_tx_o  ( alert_tx[56:55] ),
      .alert_rx_i  ( alert_rx[56:55] ),

      // Inter-module signals
      .csrng_cmd_o(csrng_csrng_cmd_req[0]),
      .csrng_cmd_i(csrng_csrng_cmd_rsp[0]),
      .edn_i(edn0_edn_req),
      .edn_o(edn0_edn_rsp),
      .tl_i(edn0_tl_req),
      .tl_o(edn0_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  edn #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[58:57])
  ) u_edn1 (

      // Interrupt
      .intr_edn_cmd_req_done_o (intr_edn1_edn_cmd_req_done),
      .intr_edn_fatal_err_o    (intr_edn1_edn_fatal_err),
      // [57]: recov_alert
      // [58]: fatal_alert
      .alert_tx_o  ( alert_tx[58:57] ),
      .alert_rx_i  ( alert_rx[58:57] ),

      // Inter-module signals
      .csrng_cmd_o(csrng_csrng_cmd_req[1]),
      .csrng_cmd_i(csrng_csrng_cmd_rsp[1]),
      .edn_i(edn1_edn_req),
      .edn_o(edn1_edn_rsp),
      .tl_i(edn1_tl_req),
      .tl_o(edn1_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  sram_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[59:59]),
    .RndCnstSramKey(RndCnstSramCtrlMainSramKey),
    .RndCnstSramNonce(RndCnstSramCtrlMainSramNonce),
    .RndCnstLfsrSeed(RndCnstSramCtrlMainLfsrSeed),
    .RndCnstLfsrPerm(RndCnstSramCtrlMainLfsrPerm),
    .MemSizeRam(131072),
    .InstrExec(SramCtrlMainInstrExec)
  ) u_sram_ctrl_main (
      // [59]: fatal_error
      .alert_tx_o  ( alert_tx[59:59] ),
      .alert_rx_i  ( alert_rx[59:59] ),

      // Inter-module signals
      .sram_otp_key_o(otp_ctrl_sram_otp_key_req[0]),
      .sram_otp_key_i(otp_ctrl_sram_otp_key_rsp[0]),
      .cfg_i(ast_ram_1p_cfg),
      .lc_escalate_en_i(lc_ctrl_lc_escalate_en),
      .lc_hw_debug_en_i(lc_ctrl_lc_hw_debug_en),
      .otp_en_sram_ifetch_i(sram_ctrl_main_otp_en_sram_ifetch),
      .regs_tl_i(sram_ctrl_main_regs_tl_req),
      .regs_tl_o(sram_ctrl_main_regs_tl_rsp),
      .ram_tl_i(sram_ctrl_main_ram_tl_req),
      .ram_tl_o(sram_ctrl_main_ram_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  rom_ctrl #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[60:60]),
    .BootRomInitFile(RomCtrlBootRomInitFile),
    .RndCnstScrNonce(RndCnstRomCtrlScrNonce),
    .RndCnstScrKey(RndCnstRomCtrlScrKey),
    .SecDisableScrambling(SecRomCtrlDisableScrambling),
    .MemSizeRom(32768)
  ) u_rom_ctrl (
      // [60]: fatal
      .alert_tx_o  ( alert_tx[60:60] ),
      .alert_rx_i  ( alert_rx[60:60] ),

      // Inter-module signals
      .rom_cfg_i(ast_rom_cfg),
      .pwrmgr_data_o(rom_ctrl_pwrmgr_data),
      .keymgr_data_o(rom_ctrl_keymgr_data),
      .kmac_data_o(kmac_app_req[2]),
      .kmac_data_i(kmac_app_rsp[2]),
      .regs_tl_i(rom_ctrl_regs_tl_req),
      .regs_tl_o(rom_ctrl_regs_tl_rsp),
      .rom_tl_i(rom_ctrl_rom_tl_req),
      .rom_tl_o(rom_ctrl_rom_tl_rsp),

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel])
  );
  rv_core_ibex #(
    .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[64:61]),
    .RndCnstLfsrSeed(RndCnstRvCoreIbexLfsrSeed),
    .RndCnstLfsrPerm(RndCnstRvCoreIbexLfsrPerm),
    .RndCnstIbexKeyDefault(RndCnstRvCoreIbexIbexKeyDefault),
    .RndCnstIbexNonceDefault(RndCnstRvCoreIbexIbexNonceDefault),
    .PMPEnable(RvCoreIbexPMPEnable),
    .PMPGranularity(RvCoreIbexPMPGranularity),
    .PMPNumRegions(RvCoreIbexPMPNumRegions),
    .MHPMCounterNum(RvCoreIbexMHPMCounterNum),
    .MHPMCounterWidth(RvCoreIbexMHPMCounterWidth),
    .RV32E(RvCoreIbexRV32E),
    .RV32M(RvCoreIbexRV32M),
    .RV32B(RvCoreIbexRV32B),
    .RegFile(RvCoreIbexRegFile),
    .BranchTargetALU(RvCoreIbexBranchTargetALU),
    .WritebackStage(RvCoreIbexWritebackStage),
    .ICache(RvCoreIbexICache),
    .ICacheECC(RvCoreIbexICacheECC),
    .ICacheScramble(RvCoreIbexICacheScramble),
    .BranchPredictor(RvCoreIbexBranchPredictor),
    .DbgTriggerEn(RvCoreIbexDbgTriggerEn),
    .DbgHwBreakNum(RvCoreIbexDbgHwBreakNum),
    .SecureIbex(RvCoreIbexSecureIbex),
    .DmHaltAddr(RvCoreIbexDmHaltAddr),
    .DmExceptionAddr(RvCoreIbexDmExceptionAddr),
    .PipeLine(RvCoreIbexPipeLine)
  ) u_rv_core_ibex (
      // [61]: fatal_sw_err
      // [62]: recov_sw_err
      // [63]: fatal_hw_err
      // [64]: recov_hw_err
      .alert_tx_o  ( alert_tx[64:61] ),
      .alert_rx_i  ( alert_rx[64:61] ),

      // Inter-module signals
      .rst_cpu_n_o(),
      .ram_cfg_i(ast_ram_1p_cfg),
      .hart_id_i(rv_core_ibex_hart_id),
      .boot_addr_i(rv_core_ibex_boot_addr),
      .irq_software_i(rv_plic_msip),
      .irq_timer_i(rv_core_ibex_irq_timer),
      .irq_external_i(rv_plic_irq),
      .esc_tx_i(alert_handler_esc_tx[0]),
      .esc_rx_o(alert_handler_esc_rx[0]),
      .debug_req_i(rv_dm_debug_req),
      .crash_dump_o(rv_core_ibex_crash_dump),
      .lc_cpu_en_i(lc_ctrl_lc_cpu_en),
      .pwrmgr_cpu_en_i(pwrmgr_aon_fetch_en),
      .pwrmgr_o(rv_core_ibex_pwrmgr),
      .nmi_wdog_i(aon_timer_aon_nmi_wdog_timer_bark),
      .edn_o(edn0_edn_req[7]),
      .edn_i(edn0_edn_rsp[7]),
      .icache_otp_key_o(otp_ctrl_sram_otp_key_req[2]),
      .icache_otp_key_i(otp_ctrl_sram_otp_key_rsp[2]),
      .fpga_info_i(fpga_info_i),
      .corei_tl_h_o(main_tl_rv_core_ibex__corei_req),
      .corei_tl_h_i(main_tl_rv_core_ibex__corei_rsp),
      .cored_tl_h_o(main_tl_rv_core_ibex__cored_req),
      .cored_tl_h_i(main_tl_rv_core_ibex__cored_rsp),
      .cfg_tl_d_i(rv_core_ibex_cfg_tl_d_req),
      .cfg_tl_d_o(rv_core_ibex_cfg_tl_d_rsp),
      .scanmode_i,
      .scan_rst_ni,

      // Clock and reset connections
      .clk_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_edn_i (clkmgr_aon_clocks.clk_main_infra),
      .clk_esc_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_secure),
      .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
      .rst_esc_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
      .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
  );
  // interrupt assignments
  assign intr_vector = {
      intr_edn1_edn_fatal_err, // IDs [185 +: 1]
      intr_edn1_edn_cmd_req_done, // IDs [184 +: 1]
      intr_edn0_edn_fatal_err, // IDs [183 +: 1]
      intr_edn0_edn_cmd_req_done, // IDs [182 +: 1]
      intr_entropy_src_es_fatal_err, // IDs [181 +: 1]
      intr_entropy_src_es_observe_fifo_ready, // IDs [180 +: 1]
      intr_entropy_src_es_health_test_failed, // IDs [179 +: 1]
      intr_entropy_src_es_entropy_valid, // IDs [178 +: 1]
      intr_csrng_cs_fatal_err, // IDs [177 +: 1]
      intr_csrng_cs_hw_inst_exc, // IDs [176 +: 1]
      intr_csrng_cs_entropy_req, // IDs [175 +: 1]
      intr_csrng_cs_cmd_req_done, // IDs [174 +: 1]
      intr_keymgr_op_done, // IDs [173 +: 1]
      intr_otbn_done, // IDs [172 +: 1]
      intr_kmac_kmac_err, // IDs [171 +: 1]
      intr_kmac_fifo_empty, // IDs [170 +: 1]
      intr_kmac_kmac_done, // IDs [169 +: 1]
      intr_hmac_hmac_err, // IDs [168 +: 1]
      intr_hmac_fifo_empty, // IDs [167 +: 1]
      intr_hmac_hmac_done, // IDs [166 +: 1]
      intr_flash_ctrl_corr_err, // IDs [165 +: 1]
      intr_flash_ctrl_op_done, // IDs [164 +: 1]
      intr_flash_ctrl_rd_lvl, // IDs [163 +: 1]
      intr_flash_ctrl_rd_full, // IDs [162 +: 1]
      intr_flash_ctrl_prog_lvl, // IDs [161 +: 1]
      intr_flash_ctrl_prog_empty, // IDs [160 +: 1]
      intr_sensor_ctrl_aon_init_status_change, // IDs [159 +: 1]
      intr_sensor_ctrl_aon_io_status_change, // IDs [158 +: 1]
      intr_aon_timer_aon_wdog_timer_bark, // IDs [157 +: 1]
      intr_aon_timer_aon_wkup_timer_expired, // IDs [156 +: 1]
      intr_adc_ctrl_aon_match_pending, // IDs [155 +: 1]
      intr_sysrst_ctrl_aon_event_detected, // IDs [154 +: 1]
      intr_pwrmgr_aon_wakeup, // IDs [153 +: 1]
      intr_usbdev_av_setup_empty, // IDs [152 +: 1]
      intr_usbdev_link_out_err, // IDs [151 +: 1]
      intr_usbdev_powered, // IDs [150 +: 1]
      intr_usbdev_frame, // IDs [149 +: 1]
      intr_usbdev_rx_bitstuff_err, // IDs [148 +: 1]
      intr_usbdev_rx_pid_err, // IDs [147 +: 1]
      intr_usbdev_rx_crc_err, // IDs [146 +: 1]
      intr_usbdev_link_in_err, // IDs [145 +: 1]
      intr_usbdev_av_overflow, // IDs [144 +: 1]
      intr_usbdev_rx_full, // IDs [143 +: 1]
      intr_usbdev_av_out_empty, // IDs [142 +: 1]
      intr_usbdev_link_resume, // IDs [141 +: 1]
      intr_usbdev_link_suspend, // IDs [140 +: 1]
      intr_usbdev_link_reset, // IDs [139 +: 1]
      intr_usbdev_host_lost, // IDs [138 +: 1]
      intr_usbdev_disconnected, // IDs [137 +: 1]
      intr_usbdev_pkt_sent, // IDs [136 +: 1]
      intr_usbdev_pkt_received, // IDs [135 +: 1]
      intr_spi_host1_spi_event, // IDs [134 +: 1]
      intr_spi_host1_error, // IDs [133 +: 1]
      intr_spi_host0_spi_event, // IDs [132 +: 1]
      intr_spi_host0_error, // IDs [131 +: 1]
      intr_alert_handler_classd, // IDs [130 +: 1]
      intr_alert_handler_classc, // IDs [129 +: 1]
      intr_alert_handler_classb, // IDs [128 +: 1]
      intr_alert_handler_classa, // IDs [127 +: 1]
      intr_otp_ctrl_otp_error, // IDs [126 +: 1]
      intr_otp_ctrl_otp_operation_done, // IDs [125 +: 1]
      intr_rv_timer_timer_expired_hart0_timer0, // IDs [124 +: 1]
      intr_pattgen_done_ch1, // IDs [123 +: 1]
      intr_pattgen_done_ch0, // IDs [122 +: 1]
      intr_i2c2_host_timeout, // IDs [121 +: 1]
      intr_i2c2_unexp_stop, // IDs [120 +: 1]
      intr_i2c2_acq_stretch, // IDs [119 +: 1]
      intr_i2c2_tx_threshold, // IDs [118 +: 1]
      intr_i2c2_tx_stretch, // IDs [117 +: 1]
      intr_i2c2_cmd_complete, // IDs [116 +: 1]
      intr_i2c2_sda_unstable, // IDs [115 +: 1]
      intr_i2c2_stretch_timeout, // IDs [114 +: 1]
      intr_i2c2_sda_interference, // IDs [113 +: 1]
      intr_i2c2_scl_interference, // IDs [112 +: 1]
      intr_i2c2_controller_halt, // IDs [111 +: 1]
      intr_i2c2_rx_overflow, // IDs [110 +: 1]
      intr_i2c2_acq_threshold, // IDs [109 +: 1]
      intr_i2c2_rx_threshold, // IDs [108 +: 1]
      intr_i2c2_fmt_threshold, // IDs [107 +: 1]
      intr_i2c1_host_timeout, // IDs [106 +: 1]
      intr_i2c1_unexp_stop, // IDs [105 +: 1]
      intr_i2c1_acq_stretch, // IDs [104 +: 1]
      intr_i2c1_tx_threshold, // IDs [103 +: 1]
      intr_i2c1_tx_stretch, // IDs [102 +: 1]
      intr_i2c1_cmd_complete, // IDs [101 +: 1]
      intr_i2c1_sda_unstable, // IDs [100 +: 1]
      intr_i2c1_stretch_timeout, // IDs [99 +: 1]
      intr_i2c1_sda_interference, // IDs [98 +: 1]
      intr_i2c1_scl_interference, // IDs [97 +: 1]
      intr_i2c1_controller_halt, // IDs [96 +: 1]
      intr_i2c1_rx_overflow, // IDs [95 +: 1]
      intr_i2c1_acq_threshold, // IDs [94 +: 1]
      intr_i2c1_rx_threshold, // IDs [93 +: 1]
      intr_i2c1_fmt_threshold, // IDs [92 +: 1]
      intr_i2c0_host_timeout, // IDs [91 +: 1]
      intr_i2c0_unexp_stop, // IDs [90 +: 1]
      intr_i2c0_acq_stretch, // IDs [89 +: 1]
      intr_i2c0_tx_threshold, // IDs [88 +: 1]
      intr_i2c0_tx_stretch, // IDs [87 +: 1]
      intr_i2c0_cmd_complete, // IDs [86 +: 1]
      intr_i2c0_sda_unstable, // IDs [85 +: 1]
      intr_i2c0_stretch_timeout, // IDs [84 +: 1]
      intr_i2c0_sda_interference, // IDs [83 +: 1]
      intr_i2c0_scl_interference, // IDs [82 +: 1]
      intr_i2c0_controller_halt, // IDs [81 +: 1]
      intr_i2c0_rx_overflow, // IDs [80 +: 1]
      intr_i2c0_acq_threshold, // IDs [79 +: 1]
      intr_i2c0_rx_threshold, // IDs [78 +: 1]
      intr_i2c0_fmt_threshold, // IDs [77 +: 1]
      intr_spi_device_tpm_rdfifo_drop, // IDs [76 +: 1]
      intr_spi_device_tpm_rdfifo_cmd_end, // IDs [75 +: 1]
      intr_spi_device_tpm_header_not_empty, // IDs [74 +: 1]
      intr_spi_device_readbuf_flip, // IDs [73 +: 1]
      intr_spi_device_readbuf_watermark, // IDs [72 +: 1]
      intr_spi_device_upload_payload_overflow, // IDs [71 +: 1]
      intr_spi_device_upload_payload_not_empty, // IDs [70 +: 1]
      intr_spi_device_upload_cmdfifo_not_empty, // IDs [69 +: 1]
      intr_gpio_gpio, // IDs [37 +: 32]
      intr_uart3_tx_empty, // IDs [36 +: 1]
      intr_uart3_rx_parity_err, // IDs [35 +: 1]
      intr_uart3_rx_timeout, // IDs [34 +: 1]
      intr_uart3_rx_break_err, // IDs [33 +: 1]
      intr_uart3_rx_frame_err, // IDs [32 +: 1]
      intr_uart3_rx_overflow, // IDs [31 +: 1]
      intr_uart3_tx_done, // IDs [30 +: 1]
      intr_uart3_rx_watermark, // IDs [29 +: 1]
      intr_uart3_tx_watermark, // IDs [28 +: 1]
      intr_uart2_tx_empty, // IDs [27 +: 1]
      intr_uart2_rx_parity_err, // IDs [26 +: 1]
      intr_uart2_rx_timeout, // IDs [25 +: 1]
      intr_uart2_rx_break_err, // IDs [24 +: 1]
      intr_uart2_rx_frame_err, // IDs [23 +: 1]
      intr_uart2_rx_overflow, // IDs [22 +: 1]
      intr_uart2_tx_done, // IDs [21 +: 1]
      intr_uart2_rx_watermark, // IDs [20 +: 1]
      intr_uart2_tx_watermark, // IDs [19 +: 1]
      intr_uart1_tx_empty, // IDs [18 +: 1]
      intr_uart1_rx_parity_err, // IDs [17 +: 1]
      intr_uart1_rx_timeout, // IDs [16 +: 1]
      intr_uart1_rx_break_err, // IDs [15 +: 1]
      intr_uart1_rx_frame_err, // IDs [14 +: 1]
      intr_uart1_rx_overflow, // IDs [13 +: 1]
      intr_uart1_tx_done, // IDs [12 +: 1]
      intr_uart1_rx_watermark, // IDs [11 +: 1]
      intr_uart1_tx_watermark, // IDs [10 +: 1]
      intr_uart0_tx_empty, // IDs [9 +: 1]
      intr_uart0_rx_parity_err, // IDs [8 +: 1]
      intr_uart0_rx_timeout, // IDs [7 +: 1]
      intr_uart0_rx_break_err, // IDs [6 +: 1]
      intr_uart0_rx_frame_err, // IDs [5 +: 1]
      intr_uart0_rx_overflow, // IDs [4 +: 1]
      intr_uart0_tx_done, // IDs [3 +: 1]
      intr_uart0_rx_watermark, // IDs [2 +: 1]
      intr_uart0_tx_watermark, // IDs [1 +: 1]
      1'b 0 // ID [0 +: 1] is a special case and tied to zero.
  };

  // TL-UL Crossbar
  xbar_main u_xbar_main (
    .clk_main_i (clkmgr_aon_clocks.clk_main_infra),
    .clk_fixed_i (clkmgr_aon_clocks.clk_io_div4_infra),
    .clk_usb_i (clkmgr_aon_clocks.clk_usb_infra),
    .clk_spi_host0_i (clkmgr_aon_clocks.clk_io_infra),
    .clk_spi_host1_i (clkmgr_aon_clocks.clk_io_div2_infra),
    .rst_main_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
    .rst_fixed_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
    .rst_usb_ni (rstmgr_aon_resets.rst_lc_usb_n[rstmgr_pkg::Domain0Sel]),
    .rst_spi_host0_ni (rstmgr_aon_resets.rst_lc_io_n[rstmgr_pkg::Domain0Sel]),
    .rst_spi_host1_ni (rstmgr_aon_resets.rst_lc_io_div2_n[rstmgr_pkg::Domain0Sel]),

    // port: tl_rv_core_ibex__corei
    .tl_rv_core_ibex__corei_i(main_tl_rv_core_ibex__corei_req),
    .tl_rv_core_ibex__corei_o(main_tl_rv_core_ibex__corei_rsp),

    // port: tl_rv_core_ibex__cored
    .tl_rv_core_ibex__cored_i(main_tl_rv_core_ibex__cored_req),
    .tl_rv_core_ibex__cored_o(main_tl_rv_core_ibex__cored_rsp),

    // port: tl_rv_dm__sba
    .tl_rv_dm__sba_i(main_tl_rv_dm__sba_req),
    .tl_rv_dm__sba_o(main_tl_rv_dm__sba_rsp),

    // port: tl_rv_dm__regs
    .tl_rv_dm__regs_o(rv_dm_regs_tl_d_req),
    .tl_rv_dm__regs_i(rv_dm_regs_tl_d_rsp),

    // port: tl_rv_dm__mem
    .tl_rv_dm__mem_o(rv_dm_mem_tl_d_req),
    .tl_rv_dm__mem_i(rv_dm_mem_tl_d_rsp),

    // port: tl_rom_ctrl__rom
    .tl_rom_ctrl__rom_o(rom_ctrl_rom_tl_req),
    .tl_rom_ctrl__rom_i(rom_ctrl_rom_tl_rsp),

    // port: tl_rom_ctrl__regs
    .tl_rom_ctrl__regs_o(rom_ctrl_regs_tl_req),
    .tl_rom_ctrl__regs_i(rom_ctrl_regs_tl_rsp),

    // port: tl_peri
    .tl_peri_o(main_tl_peri_req),
    .tl_peri_i(main_tl_peri_rsp),

    // port: tl_spi_host0
    .tl_spi_host0_o(spi_host0_tl_req),
    .tl_spi_host0_i(spi_host0_tl_rsp),

    // port: tl_spi_host1
    .tl_spi_host1_o(spi_host1_tl_req),
    .tl_spi_host1_i(spi_host1_tl_rsp),

    // port: tl_usbdev
    .tl_usbdev_o(usbdev_tl_req),
    .tl_usbdev_i(usbdev_tl_rsp),

    // port: tl_flash_ctrl__core
    .tl_flash_ctrl__core_o(flash_ctrl_core_tl_req),
    .tl_flash_ctrl__core_i(flash_ctrl_core_tl_rsp),

    // port: tl_flash_ctrl__prim
    .tl_flash_ctrl__prim_o(flash_ctrl_prim_tl_req),
    .tl_flash_ctrl__prim_i(flash_ctrl_prim_tl_rsp),

    // port: tl_flash_ctrl__mem
    .tl_flash_ctrl__mem_o(flash_ctrl_mem_tl_req),
    .tl_flash_ctrl__mem_i(flash_ctrl_mem_tl_rsp),

    // port: tl_hmac
    .tl_hmac_o(hmac_tl_req),
    .tl_hmac_i(hmac_tl_rsp),

    // port: tl_kmac
    .tl_kmac_o(kmac_tl_req),
    .tl_kmac_i(kmac_tl_rsp),

    // port: tl_aes
    .tl_aes_o(aes_tl_req),
    .tl_aes_i(aes_tl_rsp),

    // port: tl_entropy_src
    .tl_entropy_src_o(entropy_src_tl_req),
    .tl_entropy_src_i(entropy_src_tl_rsp),

    // port: tl_csrng
    .tl_csrng_o(csrng_tl_req),
    .tl_csrng_i(csrng_tl_rsp),

    // port: tl_edn0
    .tl_edn0_o(edn0_tl_req),
    .tl_edn0_i(edn0_tl_rsp),

    // port: tl_edn1
    .tl_edn1_o(edn1_tl_req),
    .tl_edn1_i(edn1_tl_rsp),

    // port: tl_rv_plic
    .tl_rv_plic_o(rv_plic_tl_req),
    .tl_rv_plic_i(rv_plic_tl_rsp),

    // port: tl_otbn
    .tl_otbn_o(otbn_tl_req),
    .tl_otbn_i(otbn_tl_rsp),

    // port: tl_keymgr
    .tl_keymgr_o(keymgr_tl_req),
    .tl_keymgr_i(keymgr_tl_rsp),

    // port: tl_rv_core_ibex__cfg
    .tl_rv_core_ibex__cfg_o(rv_core_ibex_cfg_tl_d_req),
    .tl_rv_core_ibex__cfg_i(rv_core_ibex_cfg_tl_d_rsp),

    // port: tl_sram_ctrl_main__regs
    .tl_sram_ctrl_main__regs_o(sram_ctrl_main_regs_tl_req),
    .tl_sram_ctrl_main__regs_i(sram_ctrl_main_regs_tl_rsp),

    // port: tl_sram_ctrl_main__ram
    .tl_sram_ctrl_main__ram_o(sram_ctrl_main_ram_tl_req),
    .tl_sram_ctrl_main__ram_i(sram_ctrl_main_ram_tl_rsp),


    .scanmode_i
  );
  xbar_peri u_xbar_peri (
    .clk_peri_i (clkmgr_aon_clocks.clk_io_div4_infra),
    .rst_peri_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),

    // port: tl_main
    .tl_main_i(main_tl_peri_req),
    .tl_main_o(main_tl_peri_rsp),

    // port: tl_uart0
    .tl_uart0_o(uart0_tl_req),
    .tl_uart0_i(uart0_tl_rsp),

    // port: tl_uart1
    .tl_uart1_o(uart1_tl_req),
    .tl_uart1_i(uart1_tl_rsp),

    // port: tl_uart2
    .tl_uart2_o(uart2_tl_req),
    .tl_uart2_i(uart2_tl_rsp),

    // port: tl_uart3
    .tl_uart3_o(uart3_tl_req),
    .tl_uart3_i(uart3_tl_rsp),

    // port: tl_i2c0
    .tl_i2c0_o(i2c0_tl_req),
    .tl_i2c0_i(i2c0_tl_rsp),

    // port: tl_i2c1
    .tl_i2c1_o(i2c1_tl_req),
    .tl_i2c1_i(i2c1_tl_rsp),

    // port: tl_i2c2
    .tl_i2c2_o(i2c2_tl_req),
    .tl_i2c2_i(i2c2_tl_rsp),

    // port: tl_pattgen
    .tl_pattgen_o(pattgen_tl_req),
    .tl_pattgen_i(pattgen_tl_rsp),

    // port: tl_pwm_aon
    .tl_pwm_aon_o(pwm_aon_tl_req),
    .tl_pwm_aon_i(pwm_aon_tl_rsp),

    // port: tl_gpio
    .tl_gpio_o(gpio_tl_req),
    .tl_gpio_i(gpio_tl_rsp),

    // port: tl_spi_device
    .tl_spi_device_o(spi_device_tl_req),
    .tl_spi_device_i(spi_device_tl_rsp),

    // port: tl_rv_timer
    .tl_rv_timer_o(rv_timer_tl_req),
    .tl_rv_timer_i(rv_timer_tl_rsp),

    // port: tl_pwrmgr_aon
    .tl_pwrmgr_aon_o(pwrmgr_aon_tl_req),
    .tl_pwrmgr_aon_i(pwrmgr_aon_tl_rsp),

    // port: tl_rstmgr_aon
    .tl_rstmgr_aon_o(rstmgr_aon_tl_req),
    .tl_rstmgr_aon_i(rstmgr_aon_tl_rsp),

    // port: tl_clkmgr_aon
    .tl_clkmgr_aon_o(clkmgr_aon_tl_req),
    .tl_clkmgr_aon_i(clkmgr_aon_tl_rsp),

    // port: tl_pinmux_aon
    .tl_pinmux_aon_o(pinmux_aon_tl_req),
    .tl_pinmux_aon_i(pinmux_aon_tl_rsp),

    // port: tl_otp_ctrl__core
    .tl_otp_ctrl__core_o(otp_ctrl_core_tl_req),
    .tl_otp_ctrl__core_i(otp_ctrl_core_tl_rsp),

    // port: tl_otp_ctrl__prim
    .tl_otp_ctrl__prim_o(otp_ctrl_prim_tl_req),
    .tl_otp_ctrl__prim_i(otp_ctrl_prim_tl_rsp),

    // port: tl_lc_ctrl
    .tl_lc_ctrl_o(lc_ctrl_tl_req),
    .tl_lc_ctrl_i(lc_ctrl_tl_rsp),

    // port: tl_sensor_ctrl_aon
    .tl_sensor_ctrl_aon_o(sensor_ctrl_aon_tl_req),
    .tl_sensor_ctrl_aon_i(sensor_ctrl_aon_tl_rsp),

    // port: tl_alert_handler
    .tl_alert_handler_o(alert_handler_tl_req),
    .tl_alert_handler_i(alert_handler_tl_rsp),

    // port: tl_sram_ctrl_ret_aon__regs
    .tl_sram_ctrl_ret_aon__regs_o(sram_ctrl_ret_aon_regs_tl_req),
    .tl_sram_ctrl_ret_aon__regs_i(sram_ctrl_ret_aon_regs_tl_rsp),

    // port: tl_sram_ctrl_ret_aon__ram
    .tl_sram_ctrl_ret_aon__ram_o(sram_ctrl_ret_aon_ram_tl_req),
    .tl_sram_ctrl_ret_aon__ram_i(sram_ctrl_ret_aon_ram_tl_rsp),

    // port: tl_aon_timer_aon
    .tl_aon_timer_aon_o(aon_timer_aon_tl_req),
    .tl_aon_timer_aon_i(aon_timer_aon_tl_rsp),

    // port: tl_sysrst_ctrl_aon
    .tl_sysrst_ctrl_aon_o(sysrst_ctrl_aon_tl_req),
    .tl_sysrst_ctrl_aon_i(sysrst_ctrl_aon_tl_rsp),

    // port: tl_adc_ctrl_aon
    .tl_adc_ctrl_aon_o(adc_ctrl_aon_tl_req),
    .tl_adc_ctrl_aon_i(adc_ctrl_aon_tl_rsp),

    // port: tl_ast
    .tl_ast_o(ast_tl_req_o),
    .tl_ast_i(ast_tl_rsp_i),


    .scanmode_i
  );

  // Pinmux connections
  // All muxed inputs
  assign cio_gpio_gpio_p2d[0] = mio_p2d[MioInGpioGpio0];
  assign cio_gpio_gpio_p2d[1] = mio_p2d[MioInGpioGpio1];
  assign cio_gpio_gpio_p2d[2] = mio_p2d[MioInGpioGpio2];
  assign cio_gpio_gpio_p2d[3] = mio_p2d[MioInGpioGpio3];
  assign cio_gpio_gpio_p2d[4] = mio_p2d[MioInGpioGpio4];
  assign cio_gpio_gpio_p2d[5] = mio_p2d[MioInGpioGpio5];
  assign cio_gpio_gpio_p2d[6] = mio_p2d[MioInGpioGpio6];
  assign cio_gpio_gpio_p2d[7] = mio_p2d[MioInGpioGpio7];
  assign cio_gpio_gpio_p2d[8] = mio_p2d[MioInGpioGpio8];
  assign cio_gpio_gpio_p2d[9] = mio_p2d[MioInGpioGpio9];
  assign cio_gpio_gpio_p2d[10] = mio_p2d[MioInGpioGpio10];
  assign cio_gpio_gpio_p2d[11] = mio_p2d[MioInGpioGpio11];
  assign cio_gpio_gpio_p2d[12] = mio_p2d[MioInGpioGpio12];
  assign cio_gpio_gpio_p2d[13] = mio_p2d[MioInGpioGpio13];
  assign cio_gpio_gpio_p2d[14] = mio_p2d[MioInGpioGpio14];
  assign cio_gpio_gpio_p2d[15] = mio_p2d[MioInGpioGpio15];
  assign cio_gpio_gpio_p2d[16] = mio_p2d[MioInGpioGpio16];
  assign cio_gpio_gpio_p2d[17] = mio_p2d[MioInGpioGpio17];
  assign cio_gpio_gpio_p2d[18] = mio_p2d[MioInGpioGpio18];
  assign cio_gpio_gpio_p2d[19] = mio_p2d[MioInGpioGpio19];
  assign cio_gpio_gpio_p2d[20] = mio_p2d[MioInGpioGpio20];
  assign cio_gpio_gpio_p2d[21] = mio_p2d[MioInGpioGpio21];
  assign cio_gpio_gpio_p2d[22] = mio_p2d[MioInGpioGpio22];
  assign cio_gpio_gpio_p2d[23] = mio_p2d[MioInGpioGpio23];
  assign cio_gpio_gpio_p2d[24] = mio_p2d[MioInGpioGpio24];
  assign cio_gpio_gpio_p2d[25] = mio_p2d[MioInGpioGpio25];
  assign cio_gpio_gpio_p2d[26] = mio_p2d[MioInGpioGpio26];
  assign cio_gpio_gpio_p2d[27] = mio_p2d[MioInGpioGpio27];
  assign cio_gpio_gpio_p2d[28] = mio_p2d[MioInGpioGpio28];
  assign cio_gpio_gpio_p2d[29] = mio_p2d[MioInGpioGpio29];
  assign cio_gpio_gpio_p2d[30] = mio_p2d[MioInGpioGpio30];
  assign cio_gpio_gpio_p2d[31] = mio_p2d[MioInGpioGpio31];
  assign cio_i2c0_sda_p2d = mio_p2d[MioInI2c0Sda];
  assign cio_i2c0_scl_p2d = mio_p2d[MioInI2c0Scl];
  assign cio_i2c1_sda_p2d = mio_p2d[MioInI2c1Sda];
  assign cio_i2c1_scl_p2d = mio_p2d[MioInI2c1Scl];
  assign cio_i2c2_sda_p2d = mio_p2d[MioInI2c2Sda];
  assign cio_i2c2_scl_p2d = mio_p2d[MioInI2c2Scl];
  assign cio_spi_host1_sd_p2d[0] = mio_p2d[MioInSpiHost1Sd0];
  assign cio_spi_host1_sd_p2d[1] = mio_p2d[MioInSpiHost1Sd1];
  assign cio_spi_host1_sd_p2d[2] = mio_p2d[MioInSpiHost1Sd2];
  assign cio_spi_host1_sd_p2d[3] = mio_p2d[MioInSpiHost1Sd3];
  assign cio_uart0_rx_p2d = mio_p2d[MioInUart0Rx];
  assign cio_uart1_rx_p2d = mio_p2d[MioInUart1Rx];
  assign cio_uart2_rx_p2d = mio_p2d[MioInUart2Rx];
  assign cio_uart3_rx_p2d = mio_p2d[MioInUart3Rx];
  assign cio_spi_device_tpm_csb_p2d = mio_p2d[MioInSpiDeviceTpmCsb];
  assign cio_flash_ctrl_tck_p2d = mio_p2d[MioInFlashCtrlTck];
  assign cio_flash_ctrl_tms_p2d = mio_p2d[MioInFlashCtrlTms];
  assign cio_flash_ctrl_tdi_p2d = mio_p2d[MioInFlashCtrlTdi];
  assign cio_sysrst_ctrl_aon_ac_present_p2d = mio_p2d[MioInSysrstCtrlAonAcPresent];
  assign cio_sysrst_ctrl_aon_key0_in_p2d = mio_p2d[MioInSysrstCtrlAonKey0In];
  assign cio_sysrst_ctrl_aon_key1_in_p2d = mio_p2d[MioInSysrstCtrlAonKey1In];
  assign cio_sysrst_ctrl_aon_key2_in_p2d = mio_p2d[MioInSysrstCtrlAonKey2In];
  assign cio_sysrst_ctrl_aon_pwrb_in_p2d = mio_p2d[MioInSysrstCtrlAonPwrbIn];
  assign cio_sysrst_ctrl_aon_lid_open_p2d = mio_p2d[MioInSysrstCtrlAonLidOpen];
  assign cio_usbdev_sense_p2d = mio_p2d[MioInUsbdevSense];

  // All muxed outputs
  assign mio_d2p[MioOutGpioGpio0] = cio_gpio_gpio_d2p[0];
  assign mio_d2p[MioOutGpioGpio1] = cio_gpio_gpio_d2p[1];
  assign mio_d2p[MioOutGpioGpio2] = cio_gpio_gpio_d2p[2];
  assign mio_d2p[MioOutGpioGpio3] = cio_gpio_gpio_d2p[3];
  assign mio_d2p[MioOutGpioGpio4] = cio_gpio_gpio_d2p[4];
  assign mio_d2p[MioOutGpioGpio5] = cio_gpio_gpio_d2p[5];
  assign mio_d2p[MioOutGpioGpio6] = cio_gpio_gpio_d2p[6];
  assign mio_d2p[MioOutGpioGpio7] = cio_gpio_gpio_d2p[7];
  assign mio_d2p[MioOutGpioGpio8] = cio_gpio_gpio_d2p[8];
  assign mio_d2p[MioOutGpioGpio9] = cio_gpio_gpio_d2p[9];
  assign mio_d2p[MioOutGpioGpio10] = cio_gpio_gpio_d2p[10];
  assign mio_d2p[MioOutGpioGpio11] = cio_gpio_gpio_d2p[11];
  assign mio_d2p[MioOutGpioGpio12] = cio_gpio_gpio_d2p[12];
  assign mio_d2p[MioOutGpioGpio13] = cio_gpio_gpio_d2p[13];
  assign mio_d2p[MioOutGpioGpio14] = cio_gpio_gpio_d2p[14];
  assign mio_d2p[MioOutGpioGpio15] = cio_gpio_gpio_d2p[15];
  assign mio_d2p[MioOutGpioGpio16] = cio_gpio_gpio_d2p[16];
  assign mio_d2p[MioOutGpioGpio17] = cio_gpio_gpio_d2p[17];
  assign mio_d2p[MioOutGpioGpio18] = cio_gpio_gpio_d2p[18];
  assign mio_d2p[MioOutGpioGpio19] = cio_gpio_gpio_d2p[19];
  assign mio_d2p[MioOutGpioGpio20] = cio_gpio_gpio_d2p[20];
  assign mio_d2p[MioOutGpioGpio21] = cio_gpio_gpio_d2p[21];
  assign mio_d2p[MioOutGpioGpio22] = cio_gpio_gpio_d2p[22];
  assign mio_d2p[MioOutGpioGpio23] = cio_gpio_gpio_d2p[23];
  assign mio_d2p[MioOutGpioGpio24] = cio_gpio_gpio_d2p[24];
  assign mio_d2p[MioOutGpioGpio25] = cio_gpio_gpio_d2p[25];
  assign mio_d2p[MioOutGpioGpio26] = cio_gpio_gpio_d2p[26];
  assign mio_d2p[MioOutGpioGpio27] = cio_gpio_gpio_d2p[27];
  assign mio_d2p[MioOutGpioGpio28] = cio_gpio_gpio_d2p[28];
  assign mio_d2p[MioOutGpioGpio29] = cio_gpio_gpio_d2p[29];
  assign mio_d2p[MioOutGpioGpio30] = cio_gpio_gpio_d2p[30];
  assign mio_d2p[MioOutGpioGpio31] = cio_gpio_gpio_d2p[31];
  assign mio_d2p[MioOutI2c0Sda] = cio_i2c0_sda_d2p;
  assign mio_d2p[MioOutI2c0Scl] = cio_i2c0_scl_d2p;
  assign mio_d2p[MioOutI2c1Sda] = cio_i2c1_sda_d2p;
  assign mio_d2p[MioOutI2c1Scl] = cio_i2c1_scl_d2p;
  assign mio_d2p[MioOutI2c2Sda] = cio_i2c2_sda_d2p;
  assign mio_d2p[MioOutI2c2Scl] = cio_i2c2_scl_d2p;
  assign mio_d2p[MioOutSpiHost1Sd0] = cio_spi_host1_sd_d2p[0];
  assign mio_d2p[MioOutSpiHost1Sd1] = cio_spi_host1_sd_d2p[1];
  assign mio_d2p[MioOutSpiHost1Sd2] = cio_spi_host1_sd_d2p[2];
  assign mio_d2p[MioOutSpiHost1Sd3] = cio_spi_host1_sd_d2p[3];
  assign mio_d2p[MioOutUart0Tx] = cio_uart0_tx_d2p;
  assign mio_d2p[MioOutUart1Tx] = cio_uart1_tx_d2p;
  assign mio_d2p[MioOutUart2Tx] = cio_uart2_tx_d2p;
  assign mio_d2p[MioOutUart3Tx] = cio_uart3_tx_d2p;
  assign mio_d2p[MioOutPattgenPda0Tx] = cio_pattgen_pda0_tx_d2p;
  assign mio_d2p[MioOutPattgenPcl0Tx] = cio_pattgen_pcl0_tx_d2p;
  assign mio_d2p[MioOutPattgenPda1Tx] = cio_pattgen_pda1_tx_d2p;
  assign mio_d2p[MioOutPattgenPcl1Tx] = cio_pattgen_pcl1_tx_d2p;
  assign mio_d2p[MioOutSpiHost1Sck] = cio_spi_host1_sck_d2p;
  assign mio_d2p[MioOutSpiHost1Csb] = cio_spi_host1_csb_d2p;
  assign mio_d2p[MioOutFlashCtrlTdo] = cio_flash_ctrl_tdo_d2p;
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut0] = cio_sensor_ctrl_aon_ast_debug_out_d2p[0];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut1] = cio_sensor_ctrl_aon_ast_debug_out_d2p[1];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut2] = cio_sensor_ctrl_aon_ast_debug_out_d2p[2];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut3] = cio_sensor_ctrl_aon_ast_debug_out_d2p[3];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut4] = cio_sensor_ctrl_aon_ast_debug_out_d2p[4];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut5] = cio_sensor_ctrl_aon_ast_debug_out_d2p[5];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut6] = cio_sensor_ctrl_aon_ast_debug_out_d2p[6];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut7] = cio_sensor_ctrl_aon_ast_debug_out_d2p[7];
  assign mio_d2p[MioOutSensorCtrlAonAstDebugOut8] = cio_sensor_ctrl_aon_ast_debug_out_d2p[8];
  assign mio_d2p[MioOutPwmAonPwm0] = cio_pwm_aon_pwm_d2p[0];
  assign mio_d2p[MioOutPwmAonPwm1] = cio_pwm_aon_pwm_d2p[1];
  assign mio_d2p[MioOutPwmAonPwm2] = cio_pwm_aon_pwm_d2p[2];
  assign mio_d2p[MioOutPwmAonPwm3] = cio_pwm_aon_pwm_d2p[3];
  assign mio_d2p[MioOutPwmAonPwm4] = cio_pwm_aon_pwm_d2p[4];
  assign mio_d2p[MioOutPwmAonPwm5] = cio_pwm_aon_pwm_d2p[5];
  assign mio_d2p[MioOutOtpCtrlTest0] = cio_otp_ctrl_test_d2p[0];
  assign mio_d2p[MioOutSysrstCtrlAonBatDisable] = cio_sysrst_ctrl_aon_bat_disable_d2p;
  assign mio_d2p[MioOutSysrstCtrlAonKey0Out] = cio_sysrst_ctrl_aon_key0_out_d2p;
  assign mio_d2p[MioOutSysrstCtrlAonKey1Out] = cio_sysrst_ctrl_aon_key1_out_d2p;
  assign mio_d2p[MioOutSysrstCtrlAonKey2Out] = cio_sysrst_ctrl_aon_key2_out_d2p;
  assign mio_d2p[MioOutSysrstCtrlAonPwrbOut] = cio_sysrst_ctrl_aon_pwrb_out_d2p;
  assign mio_d2p[MioOutSysrstCtrlAonZ3Wakeup] = cio_sysrst_ctrl_aon_z3_wakeup_d2p;

  // All muxed output enables
  assign mio_en_d2p[MioOutGpioGpio0] = cio_gpio_gpio_en_d2p[0];
  assign mio_en_d2p[MioOutGpioGpio1] = cio_gpio_gpio_en_d2p[1];
  assign mio_en_d2p[MioOutGpioGpio2] = cio_gpio_gpio_en_d2p[2];
  assign mio_en_d2p[MioOutGpioGpio3] = cio_gpio_gpio_en_d2p[3];
  assign mio_en_d2p[MioOutGpioGpio4] = cio_gpio_gpio_en_d2p[4];
  assign mio_en_d2p[MioOutGpioGpio5] = cio_gpio_gpio_en_d2p[5];
  assign mio_en_d2p[MioOutGpioGpio6] = cio_gpio_gpio_en_d2p[6];
  assign mio_en_d2p[MioOutGpioGpio7] = cio_gpio_gpio_en_d2p[7];
  assign mio_en_d2p[MioOutGpioGpio8] = cio_gpio_gpio_en_d2p[8];
  assign mio_en_d2p[MioOutGpioGpio9] = cio_gpio_gpio_en_d2p[9];
  assign mio_en_d2p[MioOutGpioGpio10] = cio_gpio_gpio_en_d2p[10];
  assign mio_en_d2p[MioOutGpioGpio11] = cio_gpio_gpio_en_d2p[11];
  assign mio_en_d2p[MioOutGpioGpio12] = cio_gpio_gpio_en_d2p[12];
  assign mio_en_d2p[MioOutGpioGpio13] = cio_gpio_gpio_en_d2p[13];
  assign mio_en_d2p[MioOutGpioGpio14] = cio_gpio_gpio_en_d2p[14];
  assign mio_en_d2p[MioOutGpioGpio15] = cio_gpio_gpio_en_d2p[15];
  assign mio_en_d2p[MioOutGpioGpio16] = cio_gpio_gpio_en_d2p[16];
  assign mio_en_d2p[MioOutGpioGpio17] = cio_gpio_gpio_en_d2p[17];
  assign mio_en_d2p[MioOutGpioGpio18] = cio_gpio_gpio_en_d2p[18];
  assign mio_en_d2p[MioOutGpioGpio19] = cio_gpio_gpio_en_d2p[19];
  assign mio_en_d2p[MioOutGpioGpio20] = cio_gpio_gpio_en_d2p[20];
  assign mio_en_d2p[MioOutGpioGpio21] = cio_gpio_gpio_en_d2p[21];
  assign mio_en_d2p[MioOutGpioGpio22] = cio_gpio_gpio_en_d2p[22];
  assign mio_en_d2p[MioOutGpioGpio23] = cio_gpio_gpio_en_d2p[23];
  assign mio_en_d2p[MioOutGpioGpio24] = cio_gpio_gpio_en_d2p[24];
  assign mio_en_d2p[MioOutGpioGpio25] = cio_gpio_gpio_en_d2p[25];
  assign mio_en_d2p[MioOutGpioGpio26] = cio_gpio_gpio_en_d2p[26];
  assign mio_en_d2p[MioOutGpioGpio27] = cio_gpio_gpio_en_d2p[27];
  assign mio_en_d2p[MioOutGpioGpio28] = cio_gpio_gpio_en_d2p[28];
  assign mio_en_d2p[MioOutGpioGpio29] = cio_gpio_gpio_en_d2p[29];
  assign mio_en_d2p[MioOutGpioGpio30] = cio_gpio_gpio_en_d2p[30];
  assign mio_en_d2p[MioOutGpioGpio31] = cio_gpio_gpio_en_d2p[31];
  assign mio_en_d2p[MioOutI2c0Sda] = cio_i2c0_sda_en_d2p;
  assign mio_en_d2p[MioOutI2c0Scl] = cio_i2c0_scl_en_d2p;
  assign mio_en_d2p[MioOutI2c1Sda] = cio_i2c1_sda_en_d2p;
  assign mio_en_d2p[MioOutI2c1Scl] = cio_i2c1_scl_en_d2p;
  assign mio_en_d2p[MioOutI2c2Sda] = cio_i2c2_sda_en_d2p;
  assign mio_en_d2p[MioOutI2c2Scl] = cio_i2c2_scl_en_d2p;
  assign mio_en_d2p[MioOutSpiHost1Sd0] = cio_spi_host1_sd_en_d2p[0];
  assign mio_en_d2p[MioOutSpiHost1Sd1] = cio_spi_host1_sd_en_d2p[1];
  assign mio_en_d2p[MioOutSpiHost1Sd2] = cio_spi_host1_sd_en_d2p[2];
  assign mio_en_d2p[MioOutSpiHost1Sd3] = cio_spi_host1_sd_en_d2p[3];
  assign mio_en_d2p[MioOutUart0Tx] = cio_uart0_tx_en_d2p;
  assign mio_en_d2p[MioOutUart1Tx] = cio_uart1_tx_en_d2p;
  assign mio_en_d2p[MioOutUart2Tx] = cio_uart2_tx_en_d2p;
  assign mio_en_d2p[MioOutUart3Tx] = cio_uart3_tx_en_d2p;
  assign mio_en_d2p[MioOutPattgenPda0Tx] = cio_pattgen_pda0_tx_en_d2p;
  assign mio_en_d2p[MioOutPattgenPcl0Tx] = cio_pattgen_pcl0_tx_en_d2p;
  assign mio_en_d2p[MioOutPattgenPda1Tx] = cio_pattgen_pda1_tx_en_d2p;
  assign mio_en_d2p[MioOutPattgenPcl1Tx] = cio_pattgen_pcl1_tx_en_d2p;
  assign mio_en_d2p[MioOutSpiHost1Sck] = cio_spi_host1_sck_en_d2p;
  assign mio_en_d2p[MioOutSpiHost1Csb] = cio_spi_host1_csb_en_d2p;
  assign mio_en_d2p[MioOutFlashCtrlTdo] = cio_flash_ctrl_tdo_en_d2p;
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut0] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[0];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut1] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[1];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut2] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[2];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut3] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[3];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut4] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[4];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut5] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[5];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut6] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[6];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut7] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[7];
  assign mio_en_d2p[MioOutSensorCtrlAonAstDebugOut8] = cio_sensor_ctrl_aon_ast_debug_out_en_d2p[8];
  assign mio_en_d2p[MioOutPwmAonPwm0] = cio_pwm_aon_pwm_en_d2p[0];
  assign mio_en_d2p[MioOutPwmAonPwm1] = cio_pwm_aon_pwm_en_d2p[1];
  assign mio_en_d2p[MioOutPwmAonPwm2] = cio_pwm_aon_pwm_en_d2p[2];
  assign mio_en_d2p[MioOutPwmAonPwm3] = cio_pwm_aon_pwm_en_d2p[3];
  assign mio_en_d2p[MioOutPwmAonPwm4] = cio_pwm_aon_pwm_en_d2p[4];
  assign mio_en_d2p[MioOutPwmAonPwm5] = cio_pwm_aon_pwm_en_d2p[5];
  assign mio_en_d2p[MioOutOtpCtrlTest0] = cio_otp_ctrl_test_en_d2p[0];
  assign mio_en_d2p[MioOutSysrstCtrlAonBatDisable] = cio_sysrst_ctrl_aon_bat_disable_en_d2p;
  assign mio_en_d2p[MioOutSysrstCtrlAonKey0Out] = cio_sysrst_ctrl_aon_key0_out_en_d2p;
  assign mio_en_d2p[MioOutSysrstCtrlAonKey1Out] = cio_sysrst_ctrl_aon_key1_out_en_d2p;
  assign mio_en_d2p[MioOutSysrstCtrlAonKey2Out] = cio_sysrst_ctrl_aon_key2_out_en_d2p;
  assign mio_en_d2p[MioOutSysrstCtrlAonPwrbOut] = cio_sysrst_ctrl_aon_pwrb_out_en_d2p;
  assign mio_en_d2p[MioOutSysrstCtrlAonZ3Wakeup] = cio_sysrst_ctrl_aon_z3_wakeup_en_d2p;

  // All dedicated inputs
  logic [15:0] unused_dio_p2d;
  assign unused_dio_p2d = dio_p2d;
  assign cio_usbdev_usb_dp_p2d = dio_p2d[DioUsbdevUsbDp];
  assign cio_usbdev_usb_dn_p2d = dio_p2d[DioUsbdevUsbDn];
  assign cio_spi_host0_sd_p2d[0] = dio_p2d[DioSpiHost0Sd0];
  assign cio_spi_host0_sd_p2d[1] = dio_p2d[DioSpiHost0Sd1];
  assign cio_spi_host0_sd_p2d[2] = dio_p2d[DioSpiHost0Sd2];
  assign cio_spi_host0_sd_p2d[3] = dio_p2d[DioSpiHost0Sd3];
  assign cio_spi_device_sd_p2d[0] = dio_p2d[DioSpiDeviceSd0];
  assign cio_spi_device_sd_p2d[1] = dio_p2d[DioSpiDeviceSd1];
  assign cio_spi_device_sd_p2d[2] = dio_p2d[DioSpiDeviceSd2];
  assign cio_spi_device_sd_p2d[3] = dio_p2d[DioSpiDeviceSd3];
  assign cio_sysrst_ctrl_aon_ec_rst_l_p2d = dio_p2d[DioSysrstCtrlAonEcRstL];
  assign cio_sysrst_ctrl_aon_flash_wp_l_p2d = dio_p2d[DioSysrstCtrlAonFlashWpL];
  assign cio_spi_device_sck_p2d = dio_p2d[DioSpiDeviceSck];
  assign cio_spi_device_csb_p2d = dio_p2d[DioSpiDeviceCsb];

    // All dedicated outputs
  assign dio_d2p[DioUsbdevUsbDp] = cio_usbdev_usb_dp_d2p;
  assign dio_d2p[DioUsbdevUsbDn] = cio_usbdev_usb_dn_d2p;
  assign dio_d2p[DioSpiHost0Sd0] = cio_spi_host0_sd_d2p[0];
  assign dio_d2p[DioSpiHost0Sd1] = cio_spi_host0_sd_d2p[1];
  assign dio_d2p[DioSpiHost0Sd2] = cio_spi_host0_sd_d2p[2];
  assign dio_d2p[DioSpiHost0Sd3] = cio_spi_host0_sd_d2p[3];
  assign dio_d2p[DioSpiDeviceSd0] = cio_spi_device_sd_d2p[0];
  assign dio_d2p[DioSpiDeviceSd1] = cio_spi_device_sd_d2p[1];
  assign dio_d2p[DioSpiDeviceSd2] = cio_spi_device_sd_d2p[2];
  assign dio_d2p[DioSpiDeviceSd3] = cio_spi_device_sd_d2p[3];
  assign dio_d2p[DioSysrstCtrlAonEcRstL] = cio_sysrst_ctrl_aon_ec_rst_l_d2p;
  assign dio_d2p[DioSysrstCtrlAonFlashWpL] = cio_sysrst_ctrl_aon_flash_wp_l_d2p;
  assign dio_d2p[DioSpiDeviceSck] = 1'b0;
  assign dio_d2p[DioSpiDeviceCsb] = 1'b0;
  assign dio_d2p[DioSpiHost0Sck] = cio_spi_host0_sck_d2p;
  assign dio_d2p[DioSpiHost0Csb] = cio_spi_host0_csb_d2p;

  // All dedicated output enables
  assign dio_en_d2p[DioUsbdevUsbDp] = cio_usbdev_usb_dp_en_d2p;
  assign dio_en_d2p[DioUsbdevUsbDn] = cio_usbdev_usb_dn_en_d2p;
  assign dio_en_d2p[DioSpiHost0Sd0] = cio_spi_host0_sd_en_d2p[0];
  assign dio_en_d2p[DioSpiHost0Sd1] = cio_spi_host0_sd_en_d2p[1];
  assign dio_en_d2p[DioSpiHost0Sd2] = cio_spi_host0_sd_en_d2p[2];
  assign dio_en_d2p[DioSpiHost0Sd3] = cio_spi_host0_sd_en_d2p[3];
  assign dio_en_d2p[DioSpiDeviceSd0] = cio_spi_device_sd_en_d2p[0];
  assign dio_en_d2p[DioSpiDeviceSd1] = cio_spi_device_sd_en_d2p[1];
  assign dio_en_d2p[DioSpiDeviceSd2] = cio_spi_device_sd_en_d2p[2];
  assign dio_en_d2p[DioSpiDeviceSd3] = cio_spi_device_sd_en_d2p[3];
  assign dio_en_d2p[DioSysrstCtrlAonEcRstL] = cio_sysrst_ctrl_aon_ec_rst_l_en_d2p;
  assign dio_en_d2p[DioSysrstCtrlAonFlashWpL] = cio_sysrst_ctrl_aon_flash_wp_l_en_d2p;
  assign dio_en_d2p[DioSpiDeviceSck] = 1'b0;
  assign dio_en_d2p[DioSpiDeviceCsb] = 1'b0;
  assign dio_en_d2p[DioSpiHost0Sck] = cio_spi_host0_sck_en_d2p;
  assign dio_en_d2p[DioSpiHost0Csb] = cio_spi_host0_csb_en_d2p;


  // make sure scanmode_i is never X (including during reset)
  `ASSERT_KNOWN(scanmodeKnown, scanmode_i, clk_main_i, 0)

endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Description: UART top level wrapper file

module uart
    import uart_reg_pkg::*;
#(
  
  // Param list
  parameter int RxFifoDepth = 64,
  parameter int TxFifoDepth = 32,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 6,

  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}}
) (
  input           clk_i,
  input           rst_ni,

  // Bus Interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // Generic IO
  input           cio_rx_i,
  output logic    cio_tx_o,
  output logic    cio_tx_en_o,

  // Interrupts
  output logic    intr_tx_watermark_o ,
  output logic    intr_rx_watermark_o ,
  output logic    intr_tx_empty_o  ,
  output logic    intr_rx_overflow_o  ,
  output logic    intr_rx_frame_err_o ,
  output logic    intr_rx_break_err_o ,
  output logic    intr_rx_timeout_o   ,
  output logic    intr_rx_parity_err_o
);

  logic [NumAlerts-1:0] alert_test, alerts;
  uart_reg2hw_t reg2hw;
  uart_hw2reg_t hw2reg;

  uart_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    // SEC_CM: BUS.INTEGRITY
    .intg_err_o (alerts[0])
  );

  uart_core uart_core (
    .clk_i,
    .rst_ni,
    .reg2hw,
    .hw2reg,

    .rx    (cio_rx_i   ),
    .tx    (cio_tx_o   ),

    .intr_tx_watermark_o,
    .intr_rx_watermark_o,
    .intr_tx_empty_o,
    .intr_rx_overflow_o,
    .intr_rx_frame_err_o,
    .intr_rx_break_err_o,
    .intr_rx_timeout_o,
    .intr_rx_parity_err_o
  );

  // Alerts
  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(1'b1)
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni,
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  // always enable the driving out of TX
  assign cio_tx_en_o = 1'b1;

  // Assert Known for outputs
  `ASSERT(TxEnIsOne_A, cio_tx_en_o === 1'b1)
  `ASSERT_KNOWN(TxKnown_A, cio_tx_o, clk_i, !rst_ni || !cio_tx_en_o)

  // Assert Known for alerts
  `ASSERT_KNOWN(AlertsKnown_A, alert_tx_o)

  // Assert Known for interrupts
  `ASSERT_KNOWN(TxWatermarkKnown_A, intr_tx_watermark_o)
  `ASSERT_KNOWN(RxWatermarkKnown_A, intr_rx_watermark_o)
  `ASSERT_KNOWN(TxEmptyKnown_A, intr_tx_empty_o)
  `ASSERT_KNOWN(RxOverflowKnown_A, intr_rx_overflow_o)
  `ASSERT_KNOWN(RxFrameErrKnown_A, intr_rx_frame_err_o)
  `ASSERT_KNOWN(RxBreakErrKnown_A, intr_rx_break_err_o)
  `ASSERT_KNOWN(RxTimeoutKnown_A, intr_rx_timeout_o)
  `ASSERT_KNOWN(RxParityErrKnown_A, intr_rx_parity_err_o)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule

// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// USB Full-Speed Device Interface (usbdev).
//
//


module usbdev
  import usbdev_pkg::*;
  import usbdev_reg_pkg::*;
  import prim_util_pkg::vbits;
#(
  
  // Param list
  parameter int NEndpoints = 12,
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int BlockAw = 12,
  parameter bit Stub = 1'b0,
  parameter logic [NumAlerts-1:0] AlertAsyncOn = {NumAlerts{1'b1}},
  // Max time (in microseconds) from rx_enable_o high to the
  // external differential receiver outputting valid data (when
  // configured to use one).
  parameter int unsigned RcvrWakeTimeUs = 1
) (
  input  logic       clk_i,
  input  logic       rst_ni,
  input  logic       clk_aon_i,
  input  logic       rst_aon_ni,

  // Register interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Alerts
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0] alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0] alert_tx_o,

  // Data inputs
  input  logic       cio_usb_dp_i, // differential P, can be used in single-ended mode to detect SE0
  input  logic       cio_usb_dn_i, // differential N, can be used in single-ended mode to detect SE0
  input  logic       usb_rx_d_i, // single-ended input from the differential receiver

  // Data outputs
  output logic       cio_usb_dp_o,
  output logic       cio_usb_dp_en_o,
  output logic       cio_usb_dn_o,
  output logic       cio_usb_dn_en_o,
  output logic       usb_tx_se0_o, // single-ended zero output
  output logic       usb_tx_d_o,

  // Non-data I/O
  input  logic       cio_sense_i,
  output logic       usb_dp_pullup_o,
  output logic       usb_dn_pullup_o,
  output logic       usb_rx_enable_o,
  output logic       usb_tx_use_d_se0_o,

  // Direct pinmux aon detect connections
  output logic       usb_aon_suspend_req_o,
  output logic       usb_aon_wake_ack_o,

  // Events and state from wakeup module
  input  logic       usb_aon_bus_reset_i,
  input  logic       usb_aon_sense_lost_i,
  input  logic       usb_aon_bus_not_idle_i,
  input  logic       usb_aon_wake_detect_active_i,

  // SOF reference for clock calibration
  output logic       usb_ref_val_o,
  output logic       usb_ref_pulse_o,

  // memory configuration
  input prim_ram_1p_pkg::ram_1p_cfg_t ram_cfg_i,

  // Interrupts
  output logic       intr_pkt_received_o, // Packet received
  output logic       intr_pkt_sent_o, // Packet sent
  output logic       intr_powered_o,
  output logic       intr_disconnected_o,
  output logic       intr_host_lost_o,
  output logic       intr_link_reset_o,
  output logic       intr_link_suspend_o,
  output logic       intr_link_resume_o,
  output logic       intr_av_out_empty_o,
  output logic       intr_rx_full_o,
  output logic       intr_av_overflow_o,
  output logic       intr_link_in_err_o,
  output logic       intr_link_out_err_o,
  output logic       intr_rx_crc_err_o,
  output logic       intr_rx_pid_err_o,
  output logic       intr_rx_bitstuff_err_o,
  output logic       intr_frame_o,
  output logic       intr_av_setup_empty_o
);

  // Could make SramDepth, MaxPktSizeByte, AVSetupFifoDepth, AVOutFifoDepth and RXFifoDepth
  // module parameters but may need to fix register def for the first two
  localparam int SramDw = 32; // Places packing bytes to SRAM assume this
  localparam int SramDepth = 512; // 2kB, SRAM Width is DW
  localparam int MaxPktSizeByte = 64;

  localparam int SramAw = $clog2(SramDepth);
  localparam int SizeWidth = $clog2(MaxPktSizeByte);
  localparam int NBuf = (SramDepth * SramDw) / (MaxPktSizeByte * 8);
  localparam int NBufWidth = $clog2(NBuf);

  // AV SETUP and AV OUT fifos just store buffer numbers
  localparam int AVFifoWidth = NBufWidth;
  localparam int AVSetupFifoDepth = 4;
  localparam int AVOutFifoDepth = 8;

  // RX fifo stores              buf# +  size(0-MaxPktSizeByte)  + EP# + Type
  localparam int RXFifoWidth = NBufWidth + (1+SizeWidth)         +  4  + 1;
  localparam int RXFifoDepth = 8;
  // derived parameter
  localparam int RXFifoDepthW = prim_util_pkg::vbits(RXFifoDepth+1);

  usbdev_reg2hw_t reg2hw, reg2hw_regtop;
  usbdev_hw2reg_t hw2reg, hw2reg_regtop;

  logic rst_n;
  if (Stub) begin : gen_stubbed_reset
    assign rst_n = '0;
  end else begin : gen_no_stubbed_reset
    assign rst_n = rst_ni;
  end

  tlul_pkg::tl_h2d_t tl_sram_h2d;
  tlul_pkg::tl_d2h_t tl_sram_d2h;

  // Software access to the Packet Buffer RAM
  logic              sw_mem_a_req;
  logic              sw_mem_a_gnt;
  logic              sw_mem_a_write;
  logic [SramAw-1:0] sw_mem_a_addr;
  logic [SramDw-1:0] sw_mem_a_wdata;
  logic              sw_mem_a_rvalid;
  logic [SramDw-1:0] sw_mem_a_rdata;
  logic [1:0]        sw_mem_a_rerror;

  // usbdev hardware access to the Packet Buffer RAM
  logic              usb_mem_b_req;
  logic              usb_mem_b_write;
  logic [SramAw-1:0] usb_mem_b_addr;
  logic [SramDw-1:0] usb_mem_b_wdata;
  logic [SramDw-1:0] usb_mem_b_rdata;

  logic              clr_devaddr;
  logic              event_av_setup_empty, event_av_out_empty, event_av_overflow, event_rx_full;
  logic              link_reset, link_suspend;
  logic              host_lost, link_disconnect, link_powered;
  logic              event_link_reset, event_link_suspend, event_link_resume;
  logic              event_host_lost, event_disconnect, event_powered;
  logic              event_rx_crc5_err, event_rx_crc16_err;
  logic              event_rx_crc_err, event_rx_pid_err;
  logic              event_rx_bitstuff_err;
  logic              event_in_err;
  logic              event_out_err;
  logic              event_frame, event_sof;
  logic              link_active;

  // Diagnostic visibility of OUT-side exceptional events
  logic              event_ign_avsetup, event_drop_avout, event_drop_rx, event_datatog_out;
  // Diagnostic visibility of IN-side exceptional events
  logic              event_timeout_in, event_nak_in, event_nodata_in;

  // Interrupt to software reports both types of CRC error; they are separated only for the
  // purpose of diagnostic event counting.
  assign event_rx_crc_err = event_rx_crc5_err | event_rx_crc16_err;

  logic [10:0]       frame;
  logic [2:0]        link_state;
  logic              connect_en;
  logic              resume_link_active;

  // Current state of OUT data toggles
  logic [NEndpoints-1:0] out_data_toggle;
  // Write strobe from register interface
  logic                  out_datatog_we;
  // Write data from register interface
  logic [NEndpoints-1:0] out_datatog_status;
  logic [NEndpoints-1:0] out_datatog_mask;

  // Current state of IN data toggles
  logic [NEndpoints-1:0] in_data_toggle;
  // Write strobe from register interface
  logic                  in_datatog_we;
  // Write data from register interface
  logic [NEndpoints-1:0] in_datatog_status;
  logic [NEndpoints-1:0] in_datatog_mask;

  /////////////////////////////////
  // USB RX after CDC & muxing   //
  /////////////////////////////////
  logic usb_rx_d;
  logic usb_rx_dp;
  logic usb_rx_dn;
  /////////////////////////////////
  // USB TX after CDC & muxing   //
  /////////////////////////////////
  logic usb_tx_d;
  logic usb_tx_se0;
  logic usb_tx_dp;
  logic usb_tx_dn;
  logic usb_tx_oe;
  /////////////////////////////////
  // USB contol pins after CDC   //
  /////////////////////////////////
  logic usb_pwr_sense;
  logic usb_pullup_en;
  logic usb_dp_pullup_en;
  logic usb_dn_pullup_en;

  //////////////////////////////////
  // Microsecond timing reference //
  //////////////////////////////////
  // us_tick ticks for one cycle every us, and it is based off a free-running
  // counter.
  logic [5:0]   ns_cnt;
  logic         us_tick;

  assign us_tick = (ns_cnt == 6'd47);
  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      ns_cnt <= '0;
    end else begin
      if (us_tick) begin
        ns_cnt <= '0;
      end else begin
        ns_cnt <= ns_cnt + 1'b1;
      end
    end
  end

  /////////////////////////////
  // Receive interface fifos //
  /////////////////////////////

  logic              avsetup_fifo_rst;
  logic              avsetup_fifo_wready;
  logic              avout_fifo_rst;
  logic              avout_fifo_wready;
  logic              event_pkt_received;
  logic              avsetup_rvalid, avsetup_rready;
  logic              avout_rvalid, avout_rready;
  logic              rx_fifo_rst;
  logic              rx_wvalid, rx_wready;
  logic              rx_wready_setup, rx_wready_out;
  logic              rx_fifo_rvalid;
  logic              rx_fifo_re;

  logic [AVFifoWidth - 1:0] avsetup_rdata;
  logic [AVFifoWidth - 1:0] avout_rdata;
  logic [RXFifoWidth - 1:0] rx_wdata, rx_rdata;

  logic [NEndpoints-1:0] clear_rxenable_out;

  // Software reset signals
  assign avsetup_fifo_rst = reg2hw.fifo_ctrl.avsetup_rst.qe & reg2hw.fifo_ctrl.avsetup_rst.q;
  assign avout_fifo_rst = reg2hw.fifo_ctrl.avout_rst.qe & reg2hw.fifo_ctrl.avout_rst.q;
  assign rx_fifo_rst = reg2hw.fifo_ctrl.rx_rst.qe & reg2hw.fifo_ctrl.rx_rst.q;

  // Separate 'FIFO empty' interrupts for the OUT and SETUP FIFOs because each interrupt cannot be
  // cleared without writing a buffer into the FIFO
  assign event_av_setup_empty = connect_en & ~avsetup_rvalid;
  assign event_av_out_empty = connect_en & ~avout_rvalid;
  // A single 'overflow' interrupt suffices since this indicates a programming error
  assign event_av_overflow = (reg2hw.avsetupbuffer.qe & (~avsetup_fifo_wready))
                           | (reg2hw.avoutbuffer.qe   & (~avout_fifo_wready));
  assign hw2reg.usbstat.rx_empty.d = connect_en & ~rx_fifo_rvalid;

  // Available SETUP Buffer FIFO
  prim_fifo_sync #(
    .Width(AVFifoWidth),
    .Pass(1'b0),
    .Depth(AVSetupFifoDepth),
    .OutputZeroIfEmpty(1'b0)
  ) usbdev_avsetupfifo (
    .clk_i,
    .rst_ni    (rst_n),
    .clr_i     (avsetup_fifo_rst),

    .wvalid_i  (reg2hw.avsetupbuffer.qe),
    .wready_o  (avsetup_fifo_wready),
    .wdata_i   (reg2hw.avsetupbuffer.q),

    .rvalid_o  (avsetup_rvalid),
    .rready_i  (avsetup_rready),
    .rdata_o   (avsetup_rdata),
    .full_o    (hw2reg.usbstat.av_setup_full.d),
    .depth_o   (hw2reg.usbstat.av_setup_depth.d),
    .err_o     ()
  );

  // Available OUT Buffer FIFO
  prim_fifo_sync #(
    .Width(AVFifoWidth),
    .Pass(1'b0),
    .Depth(AVOutFifoDepth),
    .OutputZeroIfEmpty(1'b0)
  ) usbdev_avoutfifo (
    .clk_i,
    .rst_ni    (rst_n),
    .clr_i     (avout_fifo_rst),

    .wvalid_i  (reg2hw.avoutbuffer.qe),
    .wready_o  (avout_fifo_wready),
    .wdata_i   (reg2hw.avoutbuffer.q),

    .rvalid_o  (avout_rvalid),
    .rready_i  (avout_rready),
    .rdata_o   (avout_rdata),
    .full_o    (hw2reg.usbstat.av_out_full.d),
    .depth_o   (hw2reg.usbstat.av_out_depth.d),
    .err_o     ()
  );

  assign rx_fifo_re = reg2hw.rxfifo.ep.re | reg2hw.rxfifo.setup.re |
                      reg2hw.rxfifo.size.re | reg2hw.rxfifo.buffer.re;

  // The number of used entries in the Received Buffer FIFO is presented to the software
  logic [RXFifoDepthW-1:0] rx_depth;
  assign hw2reg.usbstat.rx_depth.d = rx_depth;

  // We can always accept a SETUP packet if the Received Buffer FIFO is not full...
  assign rx_wready_setup = rx_wready;
  // ... but regular OUT packets are not permitted to use the final entry; still qualified
  // with 'rx_wready' for when reset is asserted.
  assign rx_wready_out   = rx_wready & (rx_depth < RXFifoDepthW'(RXFifoDepth - 1));

  // Received Buffer FIFO
  prim_fifo_sync #(
    .Width(RXFifoWidth),
    .Pass(1'b0),
    .Depth(RXFifoDepth),
    .OutputZeroIfEmpty(1'b1)
  ) usbdev_rxfifo (
    .clk_i,
    .rst_ni    (rst_n),
    .clr_i     (rx_fifo_rst),

    .wvalid_i  (rx_wvalid),
    .wready_o  (rx_wready),
    .wdata_i   (rx_wdata),

    .rvalid_o  (rx_fifo_rvalid),
    .rready_i  (rx_fifo_re),
    .rdata_o   (rx_rdata),
    .full_o    (event_rx_full),
    .depth_o   (rx_depth),
    .err_o     ()
  );

  assign hw2reg.rxfifo.ep.d = rx_rdata[16:13];
  assign hw2reg.rxfifo.setup.d = rx_rdata[12];
  assign hw2reg.rxfifo.size.d = rx_rdata[11:5];
  assign hw2reg.rxfifo.buffer.d = rx_rdata[4:0];
  assign event_pkt_received = rx_fifo_rvalid;

  // The rxfifo register is hrw, but we just need the read enables.
  logic [16:0] unused_rxfifo_q;
  assign unused_rxfifo_q = {reg2hw.rxfifo.ep.q, reg2hw.rxfifo.setup.q,
                            reg2hw.rxfifo.size.q, reg2hw.rxfifo.buffer.q};

  ////////////////////////////////////
  // IN (Transmit) interface config //
  ////////////////////////////////////
  logic [NBufWidth-1:0]  in_buf [NEndpoints];
  logic [SizeWidth:0]    in_size [NEndpoints];
  logic [3:0]            in_endpoint;
  logic                  in_endpoint_val;
  logic [NEndpoints-1:0] in_rdy;
  logic [NEndpoints-1:0] clear_rdybit, set_sentbit, update_pend, set_sending;
  logic                  setup_received, in_ep_xact_end;
  logic [NEndpoints-1:0] ep_out_iso, ep_in_iso;
  logic [NEndpoints-1:0] enable_out, enable_setup, in_ep_stall, out_ep_stall;
  logic [NEndpoints-1:0] ep_set_nak_on_out;
  logic [NEndpoints-1:0] ep_in_enable, ep_out_enable;
  logic [3:0]            out_endpoint;
  logic                  out_endpoint_val;
  logic                  use_diff_rcvr, usb_diff_rx_ok;

  // Endpoint enables
  always_comb begin : proc_map_ep_enable
    for (int i = 0; i < NEndpoints; i++) begin
      ep_in_enable[i] = reg2hw.ep_in_enable[i].q;
      ep_out_enable[i] = reg2hw.ep_out_enable[i].q;
    end
  end

  // RX enables
  always_comb begin : proc_map_rxenable
    for (int i = 0; i < NEndpoints; i++) begin
      enable_setup[i] = reg2hw.rxenable_setup[i].q;
      enable_out[i]   = reg2hw.rxenable_out[i].q;
      ep_set_nak_on_out[i] = reg2hw.set_nak_out[i].q;
    end
  end

  // STALL for both directions
  always_comb begin : proc_map_stall
    for (int i = 0; i < NEndpoints; i++) begin
      in_ep_stall[i] = reg2hw.in_stall[i];
      out_ep_stall[i] = reg2hw.out_stall[i];
    end
  end

  always_comb begin : proc_map_iso
    for (int i = 0; i < NEndpoints; i++) begin
      ep_out_iso[i] = reg2hw.out_iso[i].q;
      ep_in_iso[i] = reg2hw.in_iso[i].q;
    end
  end

  always_comb begin : proc_map_buf_size
    for (int i = 0; i < NEndpoints; i++) begin
      in_buf[i]  = reg2hw.configin[i].buffer.q;
      in_size[i] = reg2hw.configin[i].size.q;
    end
  end

  always_comb begin : proc_map_rdy_reg2hw
    for (int i = 0; i < NEndpoints; i++) begin
      in_rdy[i] = reg2hw.configin[i].rdy.q;
    end
  end

  // Captured properties of current IN buffer, maintained throughout packet collection as
  // protection against change during packet retraction by FW.
  logic [NBufWidth-1:0] in_buf_q, in_buf_d;
  logic [SizeWidth:0] in_size_q, in_size_d;
  logic       in_xact_starting;
  logic [3:0] in_xact_start_ep;

  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      in_buf_q  <= '0;
      in_size_q <= '0;
    end else begin
      in_buf_q  <= in_buf_d;
      in_size_q <= in_size_d;
    end
  end
  assign in_buf_d  = in_xact_starting ? in_buf[in_xact_start_ep]  : in_buf_q;
  assign in_size_d = in_xact_starting ? in_size[in_xact_start_ep] : in_size_q;

  // OUT data toggles are maintained with the packet engine but may be set or
  // cleared by software
  assign out_datatog_we = reg2hw.out_data_toggle.status.qe & reg2hw.out_data_toggle.mask.qe;
  assign out_datatog_status = reg2hw.out_data_toggle.status.q;
  assign out_datatog_mask = reg2hw.out_data_toggle.mask.q;
  // Software may read tham at any time
  assign hw2reg.out_data_toggle.status.d = out_data_toggle;
  assign hw2reg.out_data_toggle.mask.d = {NEndpoints{1'b0}};

  // IN data toggles are maintained with the packet engine but may be set or
  // cleared by software
  assign in_datatog_we = reg2hw.in_data_toggle.status.qe & reg2hw.in_data_toggle.mask.qe;
  assign in_datatog_status = reg2hw.in_data_toggle.status.q;
  assign in_datatog_mask = reg2hw.in_data_toggle.mask.q;
  // Software may read them at any time
  assign hw2reg.in_data_toggle.status.d = in_data_toggle;
  assign hw2reg.in_data_toggle.mask.d = {NEndpoints{1'b0}};

  always_comb begin
    set_sentbit = '0;
    if (in_ep_xact_end && in_endpoint_val) begin
      set_sentbit[in_endpoint] = 1'b1;
    end
  end

  always_comb begin : proc_map_sent
    for (int i = 0; i < NEndpoints; i++) begin
      hw2reg.in_sent[i].de = set_sentbit[i];
      hw2reg.in_sent[i].d  = 1'b1;
    end
  end

  // This must be held level for the interrupt, so no sent packets are missed.
  logic sent_event_pending;
  always_comb begin
    sent_event_pending = 1'b0;
    for (int i = 0; i < NEndpoints; i++) begin
      sent_event_pending |= reg2hw.in_sent[i].q;
    end
  end

  // Clear of rxenable_out bit
  // If so configured, for every received transaction on a given endpoint, clear
  // the rxenable_out bit. In this configuration, hardware defaults to NAKing
  // any subsequent transaction, so software has time to decide the next
  // response.
  always_comb begin
    clear_rxenable_out = '0;
    if (rx_wvalid && out_endpoint_val) begin
      clear_rxenable_out[out_endpoint] = ep_set_nak_on_out[out_endpoint];
    end
  end

  always_comb begin
    for (int i = 0; i < NEndpoints; i++) begin
      hw2reg.rxenable_out[i].d = 1'b0;
      hw2reg.rxenable_out[i].de = clear_rxenable_out[i];
    end
  end

  always_comb begin
    clear_rdybit = '0;
    update_pend  = '0;
    if (event_link_reset) begin
      clear_rdybit = {NEndpoints{1'b1}};
      update_pend  = {NEndpoints{1'b1}};
    end else begin
      if (setup_received & out_endpoint_val) begin
        // Clear pending when a SETUP is received
        clear_rdybit[out_endpoint]   = 1'b1;
        update_pend[out_endpoint]    = 1'b1;
      end else if (in_ep_xact_end & in_endpoint_val) begin
        // Clear rdy and sending when an IN transmission was successful
        clear_rdybit[in_endpoint]   = 1'b1;
      end
    end
  end

  // IN transaction starting on any endpoint?
  always_comb begin
    set_sending = '0;
    if (in_xact_starting) set_sending[in_xact_start_ep] = 1'b1;
  end

  // Clearing of rdy bit in response to successful IN packet transmission or packet cancellation
  // through link reset or SETUP packet reception.
  always_comb begin : proc_map_rdy_hw2reg
    for (int i = 0; i < NEndpoints; i++) begin
      hw2reg.configin[i].rdy.de = clear_rdybit[i];
      hw2reg.configin[i].rdy.d  = 1'b0;
    end
  end

  // Update the pending bit by copying the ready bit that is about to clear
  always_comb begin : proc_map_pend
    for (int i = 0; i < NEndpoints; i++) begin
      hw2reg.configin[i].pend.de = update_pend[i];
      hw2reg.configin[i].pend.d  = reg2hw.configin[i].rdy.q | reg2hw.configin[i].pend.q;
    end
  end

  // Update the sending bit to mark that collection of the packet by the USB host has been
  // attempted and FW shall not attempt retraction of the packet.
  always_comb begin : proc_map_sending
    for (int i = 0; i < NEndpoints; i++) begin
      hw2reg.configin[i].sending.de = set_sending[i] | set_sentbit[i] | update_pend[i];
      hw2reg.configin[i].sending.d  = ~set_sentbit[i] & ~update_pend[i];
    end
  end

  ////////////////////////////////////////////////////////
  // USB interface -- everything is in USB clock domain //
  ////////////////////////////////////////////////////////
  logic cfg_pinflip;
  assign cfg_pinflip = reg2hw.phy_config.pinflip.q;
  assign usb_dp_pullup_en = cfg_pinflip ? 1'b0 : usb_pullup_en;
  assign usb_dn_pullup_en = !cfg_pinflip ? 1'b0 : usb_pullup_en;


  usbdev_usbif #(
    .NEndpoints     (NEndpoints),
    .AVFifoWidth    (AVFifoWidth),
    .RXFifoWidth    (RXFifoWidth),
    .MaxPktSizeByte (MaxPktSizeByte),
    .NBuf           (NBuf),
    .SramAw         (SramAw)
  ) usbdev_impl (
    .clk_48mhz_i          (clk_i),
    .rst_ni               (rst_n),

    // Pins
    .usb_d_i              (usb_rx_d),
    .usb_dp_i             (usb_rx_dp),
    .usb_dn_i             (usb_rx_dn),
    .usb_oe_o             (usb_tx_oe),
    .usb_d_o              (usb_tx_d),
    .usb_se0_o            (usb_tx_se0),
    .usb_dp_o             (usb_tx_dp),
    .usb_dn_o             (usb_tx_dn),
    .usb_sense_i          (usb_pwr_sense),
    .usb_pullup_en_o      (usb_pullup_en),

    // receive side
    .rx_setup_i           (enable_setup),
    .rx_out_i             (enable_out),
    .rx_stall_i           (out_ep_stall),
    .avsetup_rvalid_i     (avsetup_rvalid),
    .avsetup_rready_o     (avsetup_rready),
    .avsetup_rdata_i      (avsetup_rdata),
    .avout_rvalid_i       (avout_rvalid),
    .avout_rready_o       (avout_rready),
    .avout_rdata_i        (avout_rdata),

    .rx_wvalid_o          (rx_wvalid),
    .rx_wready_setup_i    (rx_wready_setup),
    .rx_wready_out_i      (rx_wready_out),
    .rx_wdata_o           (rx_wdata),
    .setup_received_o     (setup_received),
    .out_endpoint_o       (out_endpoint),  // will be stable for several cycles
    .out_endpoint_val_o   (out_endpoint_val),

    // transmit side
    .in_xact_starting_o   (in_xact_starting),
    .in_xact_start_ep_o   (in_xact_start_ep),
    .in_buf_i             (in_buf_q),
    .in_size_i            (in_size_q),
    .in_stall_i           (in_ep_stall),
    .in_rdy_i             (in_rdy),
    .in_ep_xact_end_o     (in_ep_xact_end),
    .in_endpoint_o        (in_endpoint),
    .in_endpoint_val_o    (in_endpoint_val),

    // memory
    .mem_req_o            (usb_mem_b_req),
    .mem_write_o          (usb_mem_b_write),
    .mem_addr_o           (usb_mem_b_addr),
    .mem_wdata_o          (usb_mem_b_wdata),
    .mem_rdata_i          (usb_mem_b_rdata),

    // time reference
    .us_tick_i            (us_tick),

    // control
    .connect_en_i         (connect_en),
    .devaddr_i            (reg2hw.usbctrl.device_address.q),
    .clr_devaddr_o        (clr_devaddr),
    .in_ep_enabled_i      (ep_in_enable),
    .out_ep_enabled_i     (ep_out_enable),
    .out_ep_iso_i         (ep_out_iso),
    .in_ep_iso_i          (ep_in_iso),
    .diff_rx_ok_i         (usb_diff_rx_ok),
    .cfg_eop_single_bit_i (reg2hw.phy_config.eop_single_bit.q), // cdc ok: quasi-static
    .tx_osc_test_mode_i   (reg2hw.phy_config.tx_osc_test_mode.q), // cdc ok: quasi-static
    .cfg_use_diff_rcvr_i  (usb_rx_enable_o),
    .cfg_pinflip_i        (cfg_pinflip),
    .out_data_toggle_o    (out_data_toggle),
    .out_datatog_we_i     (out_datatog_we),
    .out_datatog_status_i (out_datatog_status),
    .out_datatog_mask_i   (out_datatog_mask),
    .in_data_toggle_o     (in_data_toggle),
    .in_datatog_we_i      (in_datatog_we),
    .in_datatog_status_i  (in_datatog_status),
    .in_datatog_mask_i    (in_datatog_mask),

    .resume_link_active_i (resume_link_active),

    // status
    .frame_o              (frame),
    .frame_start_o        (event_frame),
    .sof_detected_o       (event_sof),
    .link_state_o         (link_state),
    .link_disconnect_o    (link_disconnect),
    .link_powered_o       (link_powered),
    .link_reset_o         (link_reset),
    .link_active_o        (link_active),
    .link_suspend_o       (link_suspend),
    .link_resume_o        (event_link_resume),
    .host_lost_o          (host_lost),
    .link_in_err_o        (event_in_err),
    .link_out_err_o       (event_out_err),
    .rx_crc5_err_o        (event_rx_crc5_err),
    .rx_crc16_err_o       (event_rx_crc16_err),
    .rx_pid_err_o         (event_rx_pid_err),
    .rx_bitstuff_err_o    (event_rx_bitstuff_err),

    // event counters
    .event_ign_avsetup_o  (event_ign_avsetup),
    .event_drop_avout_o   (event_drop_avout),
    .event_drop_rx_o      (event_drop_rx),
    .event_datatog_out_o  (event_datatog_out),
    .event_timeout_in_o   (event_timeout_in),
    .event_nak_in_o       (event_nak_in),
    .event_nodata_in_o    (event_nodata_in)
  );

  /////////////////////////////////
  // Control signal / status CDC //
  /////////////////////////////////

  assign hw2reg.usbstat.link_state.d = link_state;
  assign hw2reg.usbstat.frame.d = frame;

  assign connect_en = reg2hw.usbctrl.enable.q;
  assign resume_link_active = reg2hw.usbctrl.resume_link_active.qe &
                              reg2hw.usbctrl.resume_link_active.q;

  // Just want a pulse to ensure only one interrupt for an event
  prim_edge_detector #(
    .Width(5),
    .EnSync(1'b0)
  ) gen_event (
    .clk_i,
    .rst_ni           (rst_n),
    .d_i              ({link_disconnect, link_reset, link_suspend,
                        host_lost, link_powered}),
    .q_sync_o         (),
    .q_posedge_pulse_o({event_disconnect, event_link_reset, event_link_suspend,
                        event_host_lost, event_powered}),
    .q_negedge_pulse_o()
  );

  assign hw2reg.usbstat.host_lost.d = host_lost;

  // resets etc cause the device address to clear
  assign hw2reg.usbctrl.device_address.de = clr_devaddr;
  assign hw2reg.usbctrl.device_address.d = '0;

  // Clear the stall flag when a SETUP is received

  always_comb begin : proc_stall_tieoff
    for (int i = 0; i < NEndpoints; i++) begin
      hw2reg.in_stall[i].d  = 1'b0;
      hw2reg.out_stall[i].d  = 1'b0;
      if (setup_received && out_endpoint_val && out_endpoint == 4'(unsigned'(i))) begin
        hw2reg.out_stall[i].de = 1'b1;
        hw2reg.in_stall[i].de = 1'b1;
      end else begin
        hw2reg.out_stall[i].de = 1'b0;
        hw2reg.in_stall[i].de = 1'b0;
      end
    end
  end

  if (Stub) begin : gen_stubbed_memory
    // Stub this window off with an error responder if stubbed.
    tlul_err_resp u_tlul_err_resp (
      .clk_i,
      .rst_ni,
      .tl_h_i(tl_sram_h2d),
      .tl_h_o(tl_sram_d2h)
    );

    // Tie off unused signals
    assign sw_mem_a_req    = '0;
    assign sw_mem_a_gnt    = '0;
    assign sw_mem_a_write  = '0;
    assign sw_mem_a_addr   = '0;
    assign sw_mem_a_wdata  = '0;
    assign sw_mem_a_rvalid = '0;
    assign sw_mem_a_rdata  = '0;
    assign sw_mem_a_rerror = '0;

    assign usb_mem_b_rdata = '0;

    logic unused_usb_mem_b_sigs;
    assign unused_usb_mem_b_sigs = ^{
      ram_cfg_i,
      usb_mem_b_req,
      usb_mem_b_write,
      usb_mem_b_addr,
      usb_mem_b_wdata,
      usb_mem_b_rdata
    };
  end else begin : gen_no_stubbed_memory
    // TL-UL to SRAM adapter
    tlul_adapter_sram #(
      .SramAw(SramAw),
      .ByteAccess(0)
    ) u_tlul2sram (
      .clk_i,
      .rst_ni,

      .tl_i                       (tl_sram_h2d),
      .tl_o                       (tl_sram_d2h),
      .en_ifetch_i                (prim_mubi_pkg::MuBi4False),
      .req_o                      (sw_mem_a_req),
      .req_type_o                 (),
      .gnt_i                      (sw_mem_a_gnt),
      .we_o                       (sw_mem_a_write),
      .addr_o                     (sw_mem_a_addr),
      .wdata_o                    (sw_mem_a_wdata),
      .wmask_o                    (),           // Not used
      .intg_error_o               (),
      .rdata_i                    (sw_mem_a_rdata),
      .rvalid_i                   (sw_mem_a_rvalid),
      .rerror_i                   (sw_mem_a_rerror),
      .compound_txn_in_progress_o (),
      .readback_en_i              (prim_mubi_pkg::MuBi4False),
      .readback_error_o           (),
      .wr_collision_i             (1'b0),
      .write_pending_i            (1'b0)
    );

    // Single Port RAM implementation, which will award the `usb` port absolute priority and
    // delay `sw` access by in the event of a collision.
    //
    // In practice the `usb` access to memory is sporadic (4x oversampling of bits, and read/write
    // operations transfer 32 bits so on average the probability of collision is just 1/128 even
    // during active USB traffic, if the TL-UL interface were active on every cycle).

    // usb access has absolute priority, followed by any deferred write, and then any sw access.
    logic               mem_req;
    logic               mem_write;
    logic  [SramAw-1:0] mem_addr;
    logic  [SramDw-1:0] mem_wdata;
    assign mem_req   = usb_mem_b_req | sw_mem_a_req;
    assign mem_write = usb_mem_b_req ? usb_mem_b_write : sw_mem_a_write;
    assign mem_addr  = usb_mem_b_req ? usb_mem_b_addr  : sw_mem_a_addr;
    assign mem_wdata = usb_mem_b_req ? usb_mem_b_wdata : sw_mem_a_wdata;

    logic              mem_rvalid;
    logic [SramDw-1:0] mem_rdata;
    logic [1:0]        mem_rerror;
    logic              mem_rsteering;

    // Always grant when no `usb` request.
    assign sw_mem_a_gnt = !usb_mem_b_req;

    // `usb` relies upon its read data remaining static after read.
    logic              mem_b_read_q;
    logic [SramDw-1:0] mem_b_rdata_q;

    // Remember granted read accesses.
    // NOTE: No pipelining within the RAM model.
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        mem_rsteering <= 1'b0;
        mem_b_read_q  <= 1'b0;
        mem_b_rdata_q <= {SramDw{1'b0}};
      end else begin
        mem_rsteering <= usb_mem_b_req;
        mem_b_read_q  <= usb_mem_b_req & !usb_mem_b_write;
        // Capture the `usb` read data.
        if (mem_b_read_q)
          mem_b_rdata_q <= mem_rdata;
      end
    end

    // Read responses.
    assign sw_mem_a_rvalid  = mem_rvalid & !mem_rsteering;
    assign sw_mem_a_rerror  = {2{sw_mem_a_rvalid}} & mem_rerror;
    // We may safely return the read data to both (no security implications), but `usb` rdata
    // must be held static after the read, and be unaffected by `sw` reads.
    assign sw_mem_a_rdata  = mem_rdata;
    assign usb_mem_b_rdata = mem_b_read_q ? mem_rdata : mem_b_rdata_q;

    // SRAM Wrapper
    prim_ram_1p_adv #(
      .Depth (SramDepth),
      .Width (SramDw),    // 32 x 512 --> 2kB
      .DataBitsPerMask(8),

      .EnableECC           (0), // No Protection
      .EnableParity        (0),
      .EnableInputPipeline (0),
      .EnableOutputPipeline(0)
    ) u_memory_1p (
      .clk_i,
      .rst_ni,

      .req_i      (mem_req),
      .write_i    (mem_write),
      .addr_i     (mem_addr),
      .wdata_i    (mem_wdata),
      .wmask_i    ({SramDw{1'b1}}),
      .rdata_o    (mem_rdata),
      .rvalid_o   (mem_rvalid),
      .rerror_o   (mem_rerror),
      .cfg_i      (ram_cfg_i),
      .alert_o    ()
    );
  end : gen_no_stubbed_memory

  logic [NumAlerts-1:0] alert_test, alerts;

  // Register module
  usbdev_reg_top u_reg (
    .clk_i,
    .rst_ni, // this reset is not stubbed off so that the registers are still accessible.
    .clk_aon_i,
    .rst_aon_ni, // this reset is not stubbed off so that the registers are still accessible.

    .tl_i (tl_i),
    .tl_o (tl_o),

    .tl_win_o (tl_sram_h2d),
    .tl_win_i (tl_sram_d2h),

    .reg2hw(reg2hw_regtop),
    .hw2reg(hw2reg_regtop),

    // SEC_CM: BUS.INTEGRITY
    .intg_err_o (alerts[0])
  );

  // Stub off all register connections to reg_top.
  if (Stub) begin : gen_stubbed
    logic unused_sigs;
    assign reg2hw = '0;
    assign hw2reg_regtop = '0;
    assign unused_sigs = ^{reg2hw_regtop, hw2reg};
  end else begin : gen_not_stubbed
    assign reg2hw = reg2hw_regtop;
    assign hw2reg_regtop = hw2reg;
  end

  // Alerts
  assign alert_test = {
    reg2hw.alert_test.q &
    reg2hw.alert_test.qe
  };

  // Alerts not stubbed off because registers and T-L access still present.
  localparam logic [NumAlerts-1:0] AlertIsFatal = {NumAlerts{1'b1}};
  for (genvar i = 0; i < NumAlerts; i++) begin : gen_alert_tx
    prim_alert_sender #(
      .AsyncOn(AlertAsyncOn[i]),
      .IsFatal(AlertIsFatal[i])
    ) u_prim_alert_sender (
      .clk_i,
      .rst_ni, // this reset is not stubbed off so that the pings still work.
      .alert_test_i  ( alert_test[i] ),
      .alert_req_i   ( alerts[0]     ),
      .alert_ack_o   (               ),
      .alert_state_o (               ),
      .alert_rx_i    ( alert_rx_i[i] ),
      .alert_tx_o    ( alert_tx_o[i] )
    );
  end

  // Interrupts
  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_hw_pkt_received (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_pkt_received),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.pkt_received.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.pkt_received.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.pkt_received.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.pkt_received.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.pkt_received.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.pkt_received.d),
    .intr_o                 (intr_pkt_received_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_hw_pkt_sent (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (sent_event_pending),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.pkt_sent.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.pkt_sent.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.pkt_sent.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.pkt_sent.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.pkt_sent.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.pkt_sent.d),
    .intr_o                 (intr_pkt_sent_o)
  );

  prim_intr_hw #(.Width(1)) intr_disconnected (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_disconnect),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.disconnected.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.disconnected.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.disconnected.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.disconnected.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.disconnected.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.disconnected.d),
    .intr_o                 (intr_disconnected_o)
  );

  prim_intr_hw #(.Width(1)) intr_powered (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_powered),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.powered.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.powered.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.powered.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.powered.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.powered.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.powered.d),
    .intr_o                 (intr_powered_o)
  );

  prim_intr_hw #(.Width(1)) intr_host_lost (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_host_lost),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.host_lost.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.host_lost.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.host_lost.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.host_lost.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.host_lost.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.host_lost.d),
    .intr_o                 (intr_host_lost_o)
  );

  prim_intr_hw #(.Width(1)) intr_link_reset (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_link_reset),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.link_reset.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.link_reset.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.link_reset.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.link_reset.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.link_reset.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.link_reset.d),
    .intr_o                 (intr_link_reset_o)
  );

  prim_intr_hw #(.Width(1)) intr_link_suspend (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_link_suspend),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.link_suspend.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.link_suspend.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.link_suspend.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.link_suspend.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.link_suspend.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.link_suspend.d),
    .intr_o                 (intr_link_suspend_o)
  );

  prim_intr_hw #(.Width(1)) intr_link_resume (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_link_resume),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.link_resume.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.link_resume.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.link_resume.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.link_resume.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.link_resume.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.link_resume.d),
    .intr_o                 (intr_link_resume_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_av_out_empty (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_av_out_empty),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.av_out_empty.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.av_out_empty.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.av_out_empty.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.av_out_empty.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.av_out_empty.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.av_out_empty.d),
    .intr_o                 (intr_av_out_empty_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_rx_full (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_rx_full),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rx_full.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rx_full.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rx_full.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rx_full.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rx_full.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rx_full.d),
    .intr_o                 (intr_rx_full_o)
  );

  prim_intr_hw #(.Width(1)) intr_av_overflow (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_av_overflow),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.av_overflow.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.av_overflow.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.av_overflow.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.av_overflow.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.av_overflow.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.av_overflow.d),
    .intr_o                 (intr_av_overflow_o)
  );

  prim_intr_hw #(.Width(1)) intr_link_in_err (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_in_err),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.link_in_err.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.link_in_err.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.link_in_err.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.link_in_err.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.link_in_err.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.link_in_err.d),
    .intr_o                 (intr_link_in_err_o)
  );

  prim_intr_hw #(.Width(1)) intr_link_out_err (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_out_err),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.link_out_err.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.link_out_err.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.link_out_err.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.link_out_err.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.link_out_err.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.link_out_err.d),
    .intr_o                 (intr_link_out_err_o)
  );

  prim_intr_hw #(.Width(1)) intr_rx_crc_err (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_rx_crc_err),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rx_crc_err.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rx_crc_err.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rx_crc_err.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rx_crc_err.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rx_crc_err.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rx_crc_err.d),
    .intr_o                 (intr_rx_crc_err_o)
  );

  prim_intr_hw #(.Width(1)) intr_rx_pid_err (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_rx_pid_err),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rx_pid_err.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rx_pid_err.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rx_pid_err.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rx_pid_err.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rx_pid_err.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rx_pid_err.d),
    .intr_o                 (intr_rx_pid_err_o)
  );

  prim_intr_hw #(.Width(1)) intr_rx_bitstuff_err (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_rx_bitstuff_err),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rx_bitstuff_err.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rx_bitstuff_err.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rx_bitstuff_err.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rx_bitstuff_err.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rx_bitstuff_err.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rx_bitstuff_err.d),
    .intr_o                 (intr_rx_bitstuff_err_o)
  );

  prim_intr_hw #(.Width(1)) intr_frame (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_frame),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.frame.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.frame.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.frame.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.frame.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.frame.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.frame.d),
    .intr_o                 (intr_frame_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_av_setup_empty (
    .clk_i,
    .rst_ni, // not stubbed off so that the interrupt regs still work.
    .event_intr_i           (event_av_setup_empty),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.av_setup_empty.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.av_setup_empty.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.av_setup_empty.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.av_setup_empty.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.av_setup_empty.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.av_setup_empty.d),
    .intr_o                 (intr_av_setup_empty_o)
  );

  /////////////////////////////////
  // USB IO Muxing               //
  /////////////////////////////////
  logic cio_usb_oe;
  logic usb_rx_enable;
  assign cio_usb_dp_en_o = cio_usb_oe;
  assign cio_usb_dn_en_o = cio_usb_oe;
  assign usb_tx_use_d_se0_o = reg2hw.phy_config.tx_use_d_se0.q; // cdc ok: quasi-static
  assign hw2reg.usbstat.sense.d = usb_pwr_sense;

  usbdev_iomux i_usbdev_iomux (
    .clk_i,
    .rst_ni             (rst_n),

    // Register interface
    .hw2reg_sense_o     (hw2reg.phy_pins_sense),
    .reg2hw_drive_i     (reg2hw.phy_pins_drive),

    // Chip IO
    .usb_rx_d_i             (usb_rx_d_i),
    .usb_rx_dp_i            (cio_usb_dp_i),
    .usb_rx_dn_i            (cio_usb_dn_i),
    .cio_usb_sense_i        (cio_sense_i),
    .usb_tx_d_o             (usb_tx_d_o),
    .usb_tx_se0_o           (usb_tx_se0_o),
    .usb_tx_dp_o            (cio_usb_dp_o),
    .usb_tx_dn_o            (cio_usb_dn_o),
    .usb_tx_oe_o            (cio_usb_oe),
    .usb_dp_pullup_en_o     (usb_dp_pullup_o),
    .usb_dn_pullup_en_o     (usb_dn_pullup_o),
    .usb_rx_enable_o        (usb_rx_enable_o),

    // Internal interface
    .usb_rx_d_o             (usb_rx_d),
    .usb_rx_dp_o            (usb_rx_dp),
    .usb_rx_dn_o            (usb_rx_dn),
    .usb_tx_d_i             (usb_tx_d),
    .usb_tx_se0_i           (usb_tx_se0),
    .usb_tx_dp_i            (usb_tx_dp),
    .usb_tx_dn_i            (usb_tx_dn),
    .usb_tx_oe_i            (usb_tx_oe),
    .usb_pwr_sense_o        (usb_pwr_sense),
    .usb_dp_pullup_en_i     (usb_dp_pullup_en),
    .usb_dn_pullup_en_i     (usb_dn_pullup_en),
    .usb_rx_enable_i        (usb_rx_enable)
  );

  // Differential receiver enable
  assign use_diff_rcvr = reg2hw.phy_config.use_diff_rcvr.q;
  // enable rx only when the single-ended input is enabled and the device is
  // not suspended (unless it is forced on in the I/O mux).
  assign usb_rx_enable = use_diff_rcvr & ~link_suspend;

  // Symbols from the differential receiver are invalid until it has finished
  // waking up / powering on
  // Add 1 to the specified time to account for uncertainty in the
  // free-running counter for us_tick.
  localparam int RcvrWakeTimeWidth = vbits(RcvrWakeTimeUs + 2);
  logic [RcvrWakeTimeWidth-1:0] usb_rcvr_ok_counter_d, usb_rcvr_ok_counter_q;

  assign usb_diff_rx_ok = (usb_rcvr_ok_counter_q == '0);
  always_comb begin
    // When don't need to use a differential receiver, RX is always ready
    usb_rcvr_ok_counter_d = '0;
    if (use_diff_rcvr & !usb_rx_enable_o) begin
      usb_rcvr_ok_counter_d = RcvrWakeTimeWidth'(RcvrWakeTimeUs + 1);
    end else if (us_tick && (usb_rcvr_ok_counter_q > '0)) begin
      usb_rcvr_ok_counter_d = usb_rcvr_ok_counter_q - 1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      usb_rcvr_ok_counter_q <= RcvrWakeTimeWidth'(RcvrWakeTimeUs + 1);
    end else begin
      usb_rcvr_ok_counter_q <= usb_rcvr_ok_counter_d;
    end
  end

  /////////////////////////////////////////
  // SOF Reference for Clock Calibration //
  /////////////////////////////////////////

  logic usb_ref_val_d, usb_ref_val_q;
  logic usb_ref_disable;
  assign usb_ref_disable = reg2hw.phy_config.usb_ref_disable.q;

  // Directly forward the pulse unless disabled.
  assign usb_ref_pulse_o = usb_ref_disable ? 1'b0 : event_sof;

  // The first pulse is always ignored, but causes the valid to be asserted.
  // The valid signal is deasserted when:
  // - The link is no longer active.
  // - The host is lost (no SOF for 4ms).
  // - The reference generation is disabled.
  assign usb_ref_val_d = usb_ref_pulse_o                           ? 1'b1 :
      (!link_active || host_lost || usb_ref_disable) ? 1'b0 : usb_ref_val_q;

  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      usb_ref_val_q <= 1'b0;
    end else begin
      usb_ref_val_q <= usb_ref_val_d;
    end
  end

  assign usb_ref_val_o = usb_ref_val_q;

  /////////////////////////////////////////
  // USB aon detector signaling          //
  /////////////////////////////////////////
  assign usb_aon_suspend_req_o = reg2hw.wake_control.suspend_req.qe &
                                 reg2hw.wake_control.suspend_req.q;
  assign usb_aon_wake_ack_o = reg2hw.wake_control.wake_ack.qe &
                              reg2hw.wake_control.wake_ack.q;

  /////////////////////////////////////////
  // capture async event and debug info  //
  /////////////////////////////////////////

  assign hw2reg.wake_events.module_active.de = 1'b1;
  assign hw2reg.wake_events.module_active.d = usb_aon_wake_detect_active_i;
  assign hw2reg.wake_events.bus_not_idle.de = 1'b1;
  assign hw2reg.wake_events.bus_not_idle.d = usb_aon_bus_not_idle_i;
  assign hw2reg.wake_events.disconnected.de = 1'b1;
  assign hw2reg.wake_events.disconnected.d = usb_aon_sense_lost_i;
  assign hw2reg.wake_events.bus_reset.de = 1'b1;
  assign hw2reg.wake_events.bus_reset.d = usb_aon_bus_reset_i;

  /////////////////////////////////////
  // Diagnostic/performance counters //
  /////////////////////////////////////

  // SW write strobes for the event enables of the counters.
  logic ctr_out_ev_qe;
  logic ctr_in_ev_qe;
  logic ctr_errors_ev_qe;
  assign ctr_out_ev_qe = &{reg2hw.count_out.ign_avsetup.qe,
                           reg2hw.count_out.drop_avout.qe,
                           reg2hw.count_out.drop_rx.qe,
                           reg2hw.count_out.datatog_out.qe};
  assign ctr_in_ev_qe = &{reg2hw.count_in.timeout.qe,
                          reg2hw.count_in.nak.qe,
                          reg2hw.count_in.nodata.qe};
  assign ctr_errors_ev_qe = &{reg2hw.count_errors.crc5.qe,
                              reg2hw.count_errors.crc16.qe,
                              reg2hw.count_errors.bitstuff.qe,
                              reg2hw.count_errors.pid_invalid.qe};

  // Counters use 'rst_n' and remain at zero in Stubbed implementation
  usbdev_counter #(.NEndpoints(NEndpoints), .NEvents(4)) u_ctr_out(
    .clk_i        (clk_i),
    .rst_ni       (rst_n),
    .reset_i      (reg2hw.count_out.rst.qe & reg2hw.count_out.rst.q),
    .event_i      ({event_ign_avsetup,
                    event_drop_avout,
                    event_drop_rx,
                    event_datatog_out}),
    .ep_i         (out_endpoint),
    // Set of events being counted.
    .ev_qe_i      (ctr_out_ev_qe),
    .ev_i         ({reg2hw.count_out.ign_avsetup.q,
                    reg2hw.count_out.drop_avout.q,
                    reg2hw.count_out.drop_rx.q,
                    reg2hw.count_out.datatog_out.q}),
    .ev_o         ({hw2reg.count_out.ign_avsetup.d,
                    hw2reg.count_out.drop_avout.d,
                    hw2reg.count_out.drop_rx.d,
                    hw2reg.count_out.datatog_out.d}),
    // Endpoints being monitored.
    .endp_qe_i    (reg2hw.count_out.endpoints.qe),
    .endpoints_i  (reg2hw.count_out.endpoints.q),
    .endpoints_o  (hw2reg.count_out.endpoints.d),
    .count_o      (hw2reg.count_out.count.d)
  );

  usbdev_counter #(.NEndpoints(NEndpoints), .NEvents(3)) u_ctr_in(
    .clk_i        (clk_i),
    .rst_ni       (rst_n),
    .reset_i      (reg2hw.count_in.rst.qe & reg2hw.count_in.rst.q),
    .event_i      ({event_timeout_in,
                    event_nak_in,
                    event_nodata_in}),
    .ep_i         (in_endpoint),
    // Set of events being counted.
    .ev_qe_i      (ctr_in_ev_qe),
    .ev_i         ({reg2hw.count_in.timeout.q,
                    reg2hw.count_in.nak.q,
                    reg2hw.count_in.nodata.q}),
    .ev_o         ({hw2reg.count_in.timeout.d,
                    hw2reg.count_in.nak.d,
                    hw2reg.count_in.nodata.d}),
    // Endpoints being monitored.
    .endp_qe_i    (reg2hw.count_in.endpoints.qe),
    .endpoints_i  (reg2hw.count_in.endpoints.q),
    .endpoints_o  (hw2reg.count_in.endpoints.d),
    .count_o      (hw2reg.count_in.count.d)
  );

  usbdev_counter #(.NEndpoints(NEndpoints), .NEvents(1)) u_ctr_nodata_in(
    .clk_i        (clk_i),
    .rst_ni       (rst_n),
    .reset_i      (reg2hw.count_nodata_in.rst.qe & reg2hw.count_nodata_in.rst.q),
    .event_i      (event_nodata_in),
    .ep_i         (in_endpoint),
    // Single event for this counter, so enables not required.
    .ev_qe_i      (1'b1),
    .ev_i         (1'b1),
    .ev_o         (), // not used
    // Endpoints being monitored.
    .endp_qe_i    (reg2hw.count_nodata_in.endpoints.qe),
    .endpoints_i  (reg2hw.count_nodata_in.endpoints.q),
    .endpoints_o  (hw2reg.count_nodata_in.endpoints.d),
    .count_o      (hw2reg.count_nodata_in.count.d)
  );

  usbdev_counter #(.NEndpoints(1), .NEvents(4)) u_ctr_errors(
    .clk_i        (clk_i),
    .rst_ni       (rst_n),
    .reset_i      (reg2hw.count_errors.rst.qe & reg2hw.count_errors.rst.q),
    .event_i      ({event_rx_crc5_err,
                    event_rx_crc16_err,
                    event_rx_bitstuff_err,
                    event_rx_pid_err}),
    .ep_i         (1'b0),
    // Set of events being counted.
    .ev_qe_i      (ctr_errors_ev_qe),
    .ev_i         ({reg2hw.count_errors.crc5.q,
                    reg2hw.count_errors.crc16.q,
                    reg2hw.count_errors.bitstuff.q,
                    reg2hw.count_errors.pid_invalid.q}),
    .ev_o         ({hw2reg.count_errors.crc5.d,
                    hw2reg.count_errors.crc16.d,
                    hw2reg.count_errors.bitstuff.d,
                    hw2reg.count_errors.pid_invalid.d}),
    // This events are not reliably associated with specific endpoints, so no endpoint enables.
    .endp_qe_i    (1'b1),
    .endpoints_i  (1'b1),
    .endpoints_o  (), // not used
    .count_o      (hw2reg.count_errors.count.d)
  );

  /////////////////////////////////
  // Xprop assertions on outputs //
  /////////////////////////////////

  `ASSERT_KNOWN(TlODValidKnown_A, tl_o.d_valid)
  `ASSERT_KNOWN(TlOAReadyKnown_A, tl_o.a_ready)
  // These pins are not necessarily associated with any clock but it probably makes most sense to
  // check them on the fastest clock.
  `ASSERT_KNOWN(USBTxDKnown_A, usb_tx_d_o)
  `ASSERT_KNOWN(CIODpKnown_A, cio_usb_dp_o)
  `ASSERT_KNOWN(CIODpEnKnown_A, cio_usb_dp_en_o)
  `ASSERT_KNOWN(CIODnKnown_A, cio_usb_dn_o)
  `ASSERT_KNOWN(CIODnEnKnown_A, cio_usb_dn_en_o)
  `ASSERT_KNOWN(USBTxSe0Known_A, usb_tx_se0_o)
  `ASSERT_KNOWN(USBDpPUKnown_A, usb_dp_pullup_o)
  `ASSERT_KNOWN(USBDnPUKnown_A, usb_dn_pullup_o)
  `ASSERT_KNOWN(USBRxEnableKnown_A, usb_rx_enable_o)
  `ASSERT_KNOWN(USBAonSuspendReqKnown_A, usb_aon_suspend_req_o)
  `ASSERT_KNOWN(USBAonWakeAckKnown_A, usb_aon_wake_ack_o)
  `ASSERT_KNOWN(USBRefValKnown_A, usb_ref_val_o, clk_i, !rst_ni)
  `ASSERT_KNOWN(USBRefPulseKnown_A, usb_ref_pulse_o, clk_i, !rst_ni)
  // Assert Known for alerts
  `ASSERT_KNOWN(AlertsKnown_A, alert_tx_o)
  //Interrupt signals
  `ASSERT_KNOWN(USBIntrPktRcvdKnown_A, intr_pkt_received_o)
  `ASSERT_KNOWN(USBIntrPktSentKnown_A, intr_pkt_sent_o)
  `ASSERT_KNOWN(USBIntrPwrdKnown_A, intr_powered_o)
  `ASSERT_KNOWN(USBIntrDisConKnown_A, intr_disconnected_o)
  `ASSERT_KNOWN(USBIntrHostLostKnown_A, intr_host_lost_o)
  `ASSERT_KNOWN(USBIntrLinkRstKnown_A, intr_link_reset_o)
  `ASSERT_KNOWN(USBIntrLinkSusKnown_A, intr_link_suspend_o)
  `ASSERT_KNOWN(USBIntrLinkResKnown_A, intr_link_resume_o)
  `ASSERT_KNOWN(USBIntrAvOutEmptyKnown_A, intr_av_out_empty_o)
  `ASSERT_KNOWN(USBIntrRxFullKnown_A, intr_rx_full_o)
  `ASSERT_KNOWN(USBIntrAvOverKnown_A, intr_av_overflow_o)
  `ASSERT_KNOWN(USBIntrLinkInErrKnown_A, intr_link_in_err_o)
  `ASSERT_KNOWN(USBIntrLinkOutErrKnown_A, intr_link_out_err_o)
  `ASSERT_KNOWN(USBIntrRxCrCErrKnown_A, intr_rx_crc_err_o)
  `ASSERT_KNOWN(USBIntrRxPidErrKnown_A, intr_rx_pid_err_o)
  `ASSERT_KNOWN(USBIntrRxBitstuffErrKnown_A, intr_rx_bitstuff_err_o)
  `ASSERT_KNOWN(USBIntrFrameKnown_A, intr_frame_o)
  `ASSERT_KNOWN(USBIntrAvSetupEmptyKnown_A, intr_av_setup_empty_o)

  // Alert assertions for reg_we onehot check
  `ASSERT_PRIM_REG_WE_ONEHOT_ERROR_TRIGGER_ALERT(RegWeOnehotCheck_A, u_reg, alert_tx_o[0])
endmodule

// This is a tilelink crossbar that supports multiple masters and multiple slaves
// Since opentitan generates this module using templates, it is not convenient to integrate it into the environment, so we manually write a parameterized empty shell
module xbar_main #(
    parameter CLK_NUM = 1,
    parameter RST_NUM = 1,
    parameter HOST_NUM = 1,
    parameter DEVICE_NUM = 1
) (
  input clk [CLK_NUM-1:0],
  input rst [RST_NUM-1:0],

  // Host interfaces
  input  tlul_pkg::tl_h2d_t tl_host_i[HOST_NUM-1:0],
  output tlul_pkg::tl_d2h_t tl_host_o[HOST_NUM-1:0],


  // Device interfaces
  output tlul_pkg::tl_h2d_t tl_device_o[DEVICE_NUM-1:0],
  input  tlul_pkg::tl_d2h_t tl_device_i[DEVICE_NUM-1:0],

  input prim_mubi_pkg::mubi4_t scanmode_i
);

// This is a tilelink peripheral crossbar that supports single masters and multiple slaves
// Since opentitan generates this module using templates, it is not convenient to integrate it into the environment, so we manually write a parameterized empty shell
module xbar_peri #(
    parameter DEVICE_NUM = 1
) (
  input clk_peri_i,
  input rst_peri_ni,

  // Host interfaces
  input  tlul_pkg::tl_h2d_t tl_main_i,
  output tlul_pkg::tl_d2h_t tl_main_o,

  // Device interfaces
  output tlul_pkg::tl_h2d_t tl_device_o [DEVICE_NUM-1:0],
  input  tlul_pkg::tl_d2h_t tl_device_i [DEVICE_NUM-1:0],

  input prim_mubi_pkg::mubi4_t scanmode_i
);

endmodule
