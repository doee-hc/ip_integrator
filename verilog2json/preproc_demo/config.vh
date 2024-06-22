`include "arbiter_config_0.vh"

`define  ARBITER_WIDTH 5
`define  PORTS `ARBITER_WIDTH

`define  ARB_BLOCK_ACK 1

`define  ARB_LSB_HIGH_PRIORITY

// `define add_pseudo_instr(instr_n, instr_format, instr_category, instr_group)  \
//   constraint riscv_``instr_group``_``instr_n``_c { \
//     if (pseudo_instr_name  == ``instr_n) { \
//         format             == ``instr_format; \
//         category           == ``instr_category; \
//         group              == ``instr_group; \
//     } \
//   }
  
// `define ADD(a,b) a  \
//                 +   \
//                 b
