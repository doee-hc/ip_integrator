module top_module();

    // Parameters for instances0
    parameter instances0_param0 = 0;
    parameter instances0_param1 = 1;
    parameter instances0_param2 = 2;
    parameter instances0_param3 = 3;

    // Parameters for instances1
    parameter instances1_param0 = 0;
    parameter instances1_param1 = 1;
    parameter instances1_param2 = 2;
    parameter instances1_param3 = 3;

    // Ports for instances0
    a::b [31:0] instances0_s0 [3:0];
    a::b [31:0] instances0_s1 [3:0];
    logic [31:0] instances0_s4;

    // Ports for instances1
    a::b [31:0] instances1_s2 [3:0];
    a::b [31:0] instances1_s3 [3:0];
    logic [31:0] instances1_s5;

    ip0 #(
        .instances0(instances0_instances0),
        .instances1(instances0_instances1)
    ) instances0 (
      .s0(instances0_s0),
      .s1(instances0_s1),
      .s4(instances0_s4)
    );

    ip1 #(
        .instances0(instances1_instances0),
        .instances1(instances1_instances1)
    ) instances1 (
      .s2(instances1_s2),
      .s3(instances1_s3),
      .s5(instances1_s5)
    );

    // Connection instance0 & inst1
    assign instance1_s2[7:4] = instance0_s0[3:0];
    assign instance1_s2[15:12] = instance0_s0[11:8];
    assign instance1_s2[23:20] = instance0_s0[19:16];
    assign instance1_s2[31:28] = instance0_s0[27:24];

endmodule