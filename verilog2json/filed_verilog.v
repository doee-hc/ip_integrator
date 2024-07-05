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
 endmodule
