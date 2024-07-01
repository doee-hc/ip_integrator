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
  parameter int NEndpoints = 12;
  parameter int NumAlerts = 1;

  // Address widths within the block
  parameter int BlockAw = 12;
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

