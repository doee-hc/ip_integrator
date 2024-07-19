module top_module(
    input  logic [7:0] input_instance0_s4,
    output logic [7:0] output_instance0_s4,
    inout  logic [16:0] inout_instance0_s4,
    input  logic [7:0] input_instance1_s5,
    output logic [7:0] output_instance1_s5,
    inout  logic [16:0] inout_instance1_s5
);

    // Parameters for instance0
    parameter instance0_param0 = 0;
    parameter instance0_param1 = 1;
    parameter instance0_param2 = 2;
    parameter instance0_param3 = 3;

    // Parameters for instance1
    parameter instance1_param0 = 0;
    parameter instance1_param1 = 1;
    parameter instance1_param2 = 2;
    parameter instance1_param3 = 3;

    // Ports for instance0
    a::b [31:0] instance0_s0 [3:0];
    a::b [31:0] instance0_s1 [3:0];
    logic [31:0] instance0_s4;

    // Ports for instance1
    a::b [31:0] instance1_s2 [3:0];
    a::b [31:0] instance1_s3 [3:0];
    logic [31:0] instance1_s5;

    ip0 #(
        .param0(instance0_param0),
        .param1(instance0_param1),
        .param2(instance0_param2),
        .param3(instance0_param3)
    ) instance0 (
      .s0(instance0_s0),
      .s1(instance0_s1),
      .s4(instance0_s4)
    );

    ip1 #(
        .param0(instance1_param0),
        .param1(instance1_param1),
        .param2(instance1_param2),
        .param3(instance1_param3)
    ) instance1 (
      .s2(instance1_s2),
      .s3(instance1_s3),
      .s5(instance1_s5)
    );

    // Connect instance0 & instance1
    assign instance1_s2[7:4] = instance0_s0[3:0];
    assign instance1_s2[15:12] = instance0_s0[11:8];
    assign instance1_s2[23:20] = instance0_s0[19:16];
    assign instance1_s2[31:28] = instance0_s0[27:24];
    assign instance0_s1[7:4] = instance1_s3[3:0];
    assign instance0_s1[15:12] = instance1_s3[11:8];
    assign instance0_s1[23:20] = instance1_s3[19:16];
    assign instance0_s1[31:28] = instance1_s3[27:24];
    // instance0 Tie
    assign instance0_s2[3:0] = '0;
    assign instance0_s1[3:0] = '1;
    // instance0 IO
    assign instance0_s4[7:0] = input_instance0_s4;
    assign output_instance0_s4 = instance0_s4[15:8];
    assign inout_instance0_s4 = instance0_s4[31:15];
    // instance1 Tie
    assign instance1_s2[3:0] = '0;
    assign instance1_s1[3:0] = '1;
    // instance1 IO
    assign instance1_s5[7:0] = input_instance1_s5;
    assign output_instance1_s5 = instance1_s5[15:8];
    assign inout_instance1_s5 = instance1_s5[31:15];

endmodule