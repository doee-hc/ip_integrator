module axi_fifo_rd #
(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter STRB_WIDTH = (DATA_WIDTH/8),
    parameter ID_WIDTH = 8,
    parameter AR_FIFO = 1,
    parameter R_FIFO = 2
)
(
    input                      clk,
    input                      rst,
    input  [ID_WIDTH-1:0]      s_axi_arid,
    input  [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  [7:0]               s_axi_arlen,
    input  [2:0]               s_axi_arsize,
    input  [1:0]               s_axi_arburst,
    input                      s_axi_arlock,
    input  [3:0]               s_axi_arqos,
    input                      s_axi_arvalid,
    output                     s_axi_arready,
    output [ID_WIDTH-1:0]      s_axi_rid,
    output [DATA_WIDTH-1:0]    s_axi_rdata,
    output [1:0]               s_axi_rresp,
    output                     s_axi_rlast,
    output                     s_axi_rvalid,
    input                      s_axi_rready,
    output [ID_WIDTH-1:0]      m_axi_arid,
    output [ADDR_WIDTH-1:0]    m_axi_araddr,
    output [7:0]               m_axi_arlen,
    output [2:0]               m_axi_arsize,
    output [1:0]               m_axi_arburst,
    output                     m_axi_arlock,
    output [3:0]               m_axi_arqos,
    output                     m_axi_arvalid,
    input                      m_axi_arready,
    input  [ID_WIDTH-1:0]      m_axi_rid,
    input  [DATA_WIDTH-1:0]    m_axi_rdata,
    input  [1:0]               m_axi_rresp,
    input                      m_axi_rlast,
    input                      m_axi_rvalid,
    output                     m_axi_rready
);
endmodule