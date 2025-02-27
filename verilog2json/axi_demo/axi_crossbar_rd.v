module axi_crossbar_rd #
(
    parameter M_REGIONS= 1,
    parameter S_IF_COUNT = 4,
    parameter M_IF_COUNT = 4,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    parameter S_IF_ID_WIDTH = 8,
    parameter M_IF_ID_WIDTH = S_IF_ID_WIDTH+$clog2(S_IF_COUNT),
    parameter M_BASE_ADDR = 0,
    parameter M_ADDR_WIDTH = {M_IF_COUNT{{M_REGIONS{32'd24}}}},
    parameter M_CONNECT = {M_IF_COUNT{{S_IF_COUNT{1'b1}}}},
    parameter S_AR_FIFO = {S_IF_COUNT{2'd0}},
    parameter S_R_FIFO = {S_IF_COUNT{2'd2}},
    parameter M_AR_FIFO = {M_IF_COUNT{2'd1}},
    parameter M_R_FIFO = {M_IF_COUNT{2'd0}}
)
(
    input                                 clk,
    input                                 rst,
    input  [S_IF_COUNT*S_IF_ID_WIDTH-1:0] s_axi_arid,
    input  [S_IF_COUNT*ADDR_WIDTH-1:0]    s_axi_araddr,
    input  [S_IF_COUNT*8-1:0]             s_axi_arlen,
    input  [S_IF_COUNT*3-1:0]             s_axi_arsize,
    input  [S_IF_COUNT*2-1:0]             s_axi_arburst,
    input  [S_IF_COUNT-1:0]               s_axi_arlock,
    input  [S_IF_COUNT*4-1:0]             s_axi_arqos,
    input  [S_IF_COUNT-1:0]               s_axi_arvalid,
    output [S_IF_COUNT-1:0]               s_axi_arready,
    output [S_IF_COUNT*S_IF_ID_WIDTH-1:0] s_axi_rid,
    output [S_IF_COUNT*DATA_WIDTH-1:0]    s_axi_rdata,
    output [S_IF_COUNT*2-1:0]             s_axi_rresp,
    output [S_IF_COUNT-1:0]               s_axi_rlast,
    output [S_IF_COUNT-1:0]               s_axi_rvalid,
    input  [S_IF_COUNT-1:0]               s_axi_rready,
    output [M_IF_COUNT*M_IF_ID_WIDTH-1:0] m_axi_arid,
    output [M_IF_COUNT*ADDR_WIDTH-1:0]    m_axi_araddr,
    output [M_IF_COUNT*8-1:0]             m_axi_arlen,
    output [M_IF_COUNT*3-1:0]             m_axi_arsize,
    output [M_IF_COUNT*2-1:0]             m_axi_arburst,
    output [M_IF_COUNT-1:0]               m_axi_arlock,
    output [M_IF_COUNT*4-1:0]             m_axi_arqos,
    output [M_IF_COUNT-1:0]               m_axi_arvalid,
    input  [M_IF_COUNT-1:0]               m_axi_arready,
    input  [M_IF_COUNT*M_IF_ID_WIDTH-1:0] m_axi_rid,
    input  [M_IF_COUNT*DATA_WIDTH-1:0]    m_axi_rdata,
    input  [M_IF_COUNT*2-1:0]             m_axi_rresp,
    input  [M_IF_COUNT-1:0]               m_axi_rlast,
    input  [M_IF_COUNT-1:0]               m_axi_rvalid,
    output [M_IF_COUNT-1:0]               m_axi_rready,

    output [S_IF_COUNT*ADDR_WIDTH-1:0]    rd_error_addr,
    output [S_IF_COUNT-1:0]               rd_error_valid,
    input  [S_IF_COUNT-1:0]               rd_error_ready
);


    logic a;
    logic a;
    logic a;
    logic a;
    logic a;

    cpu cpu0();
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;
    logic a;

endmodule