// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// SRAM controller.
//

module sram_ctrl
  import sram_ctrl_pkg::*;
  import sram_ctrl_reg_pkg::*;
#(
  
  // The width of this RAM is currently restricted to 39 (32bit data + 7bit integrity).
  parameter int DataWidth = 32 + tlul_pkg::DataIntgWidth,
  parameter int NonceWidth = 64,

    // Param list
  parameter int NumAlerts = 1,

  // Address widths within the block
  parameter int RegsAw = 6,
  parameter int RamAw = 1,
  // Number of words stored in the SRAM.
  parameter int MemSizeRam = 32'h1000,
  // Enable asynchronous transitions on alerts.
  parameter logic [NumAlerts-1:0] AlertAsyncOn          = {NumAlerts{1'b1}},
  // Enables the execute from SRAM feature.
  parameter bit InstrExec                               = 1,
  // Random netlist constants
  parameter otp_ctrl_pkg::sram_key_t   RndCnstSramKey   = RndCnstSramKeyDefault,
  parameter otp_ctrl_pkg::sram_nonce_t RndCnstSramNonce = RndCnstSramNonceDefault,
  parameter lfsr_seed_t                RndCnstLfsrSeed  = RndCnstLfsrSeedDefault,
  parameter lfsr_perm_t                RndCnstLfsrPerm  = RndCnstLfsrPermDefault
) (
  // SRAM Clock
  input  logic                                       clk_i,
  input  logic                                       rst_ni,
  // OTP Clock (for key interface)
  input  logic                                       clk_otp_i,
  input  logic                                       rst_otp_ni,
  // Bus Interface (device) for SRAM
  input  tlul_pkg::tl_h2d_t                          ram_tl_i,
  output tlul_pkg::tl_d2h_t                          ram_tl_o,
  // Bus Interface (device) for CSRs
  input  tlul_pkg::tl_h2d_t                          regs_tl_i,
  output tlul_pkg::tl_d2h_t                          regs_tl_o,
  // Alert outputs.
  input  prim_alert_pkg::alert_rx_t [NumAlerts-1:0]  alert_rx_i,
  output prim_alert_pkg::alert_tx_t [NumAlerts-1:0]  alert_tx_o,
  // Life-cycle escalation input (scraps the scrambling keys)
  // SEC_CM: LC_ESCALATE_EN.INTERSIG.MUBI
  input  lc_ctrl_pkg::lc_tx_t                        lc_escalate_en_i,
  // SEC_CM: LC_HW_DEBUG_EN.INTERSIG.MUBI
  input  lc_ctrl_pkg::lc_tx_t                        lc_hw_debug_en_i,
  // Otp configuration for sram execution
  // SEC_CM: EXEC.INTERSIG.MUBI
  input  prim_mubi_pkg::mubi8_t                      otp_en_sram_ifetch_i,
  // Key request to OTP (running on clk_fixed)
  // SEC_CM: SCRAMBLE.KEY.SIDELOAD
  output otp_ctrl_pkg::sram_otp_key_req_t            sram_otp_key_o,
  input  otp_ctrl_pkg::sram_otp_key_rsp_t            sram_otp_key_i,
  // config
  input  prim_ram_1p_pkg::ram_1p_cfg_t               cfg_i
); 
 endmodule

