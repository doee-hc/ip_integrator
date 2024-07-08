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