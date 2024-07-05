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