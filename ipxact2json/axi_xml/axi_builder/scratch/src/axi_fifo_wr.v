// reuse-pragma startSub [InsertComponentPrefix %subText 7]
module axi_fifo_wr #(


    // reuse-pragma startSub [ReplaceParameter -design axi_fifo_wr -lib work -format systemverilog DATA_WIDTH -endTok , -indent "    "]
    parameter DATA_WIDTH = 32,

    // reuse-pragma startSub [ReplaceParameter -design axi_fifo_wr -lib work -format systemverilog ADDR_WIDTH -endTok , -indent "    "]
    parameter ADDR_WIDTH = 32,

    // reuse-pragma startSub [ReplaceParameter -design axi_fifo_wr -lib work -format systemverilog STRB_WIDTH -endTok , -indent "    "]
    parameter STRB_WIDTH = (DATA_WIDTH/8),

    // reuse-pragma startSub [ReplaceParameter -design axi_fifo_wr -lib work -format systemverilog ID_WIDTH -endTok , -indent "    "]
    parameter ID_WIDTH = 8,

    // reuse-pragma startSub [ReplaceParameter -design axi_fifo_wr -lib work -format systemverilog AW_FIFO -endTok , -indent "    "]
    parameter AW_FIFO = 2,

    // reuse-pragma startSub [ReplaceParameter -design axi_fifo_wr -lib work -format systemverilog W_FIFO -endTok , -indent "    "]
    parameter W_FIFO = 0,

    // reuse-pragma startSub B_FIFO [ReplaceParameter -design axi_fifo_wr -lib work -format systemverilog B_FIFO -endTok "" -indent "    "]
    parameter B_FIFO = 0

    // reuse-pragma endSub B_FIFO
    )
(
    input                      clk,
    input                      rst,
    input  [ID_WIDTH-1:0]      s_axi_awid,
    input  [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  [7:0]               s_axi_awlen,
    input  [2:0]               s_axi_awsize,
    input  [1:0]               s_axi_awburst,
    input                      s_axi_awlock,
    input  [3:0]               s_axi_awqos,
    input                      s_axi_awvalid,
    output                     s_axi_awready,
    input  [ID_WIDTH-1:0]      s_axi_wid,  
    input  [DATA_WIDTH-1:0]    s_axi_wdata,
    input  [STRB_WIDTH-1:0]    s_axi_wstrb,
    input                      s_axi_wlast,
    input                      s_axi_wvalid,
    output                     s_axi_wready,
    output [ID_WIDTH-1:0]      s_axi_bid,
    output [1:0]               s_axi_bresp,
    output                     s_axi_bvalid,
    input                      s_axi_bready,
    output [ID_WIDTH-1:0]      m_axi_awid,
    output [ADDR_WIDTH-1:0]    m_axi_awaddr,
    output [7:0]               m_axi_awlen,
    output [2:0]               m_axi_awsize,
    output [1:0]               m_axi_awburst,
    output                     m_axi_awlock,
    output [3:0]               m_axi_awqos,
    output                     m_axi_awvalid,
    input                      m_axi_awready,
    output [ID_WIDTH-1:0]      m_axi_wid, 
    output [DATA_WIDTH-1:0]    m_axi_wdata,
    output [STRB_WIDTH-1:0]    m_axi_wstrb,
    output                     m_axi_wlast,
    output                     m_axi_wvalid,
    input                      m_axi_wready,
    input  [ID_WIDTH-1:0]      m_axi_bid,
    input  [1:0]               m_axi_bresp,
    input                      m_axi_bvalid,
    output                     m_axi_bready
);
endmodule