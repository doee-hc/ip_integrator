// -----------------------------------------------------------------------------
// Auto-Generated by:        __   _ __      _  __
//                          / /  (_) /____ | |/_/
//                         / /__/ / __/ -_)>  <
//                        /____/_/\__/\__/_/|_|
//                     Build your hardware, easily!
//                   https://github.com/enjoy-digital/litex
//
// Filename   : litepcie_core.v
// Device     : xc7a
// LiteX sha1 : 81b70d1
// Date       : 2024-06-15 18:32:50
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

//------------------------------------------------------------------------------
// Module
//------------------------------------------------------------------------------

module litepcie_core (
    output wire          clk,
    output wire  [127:0] dma0_reader_axi_tdata,
    output wire          dma0_reader_axi_tlast,
    input  wire          dma0_reader_axi_tready,
    output wire          dma0_reader_axi_tuser,
    output wire          dma0_reader_axi_tvalid,
    output wire          dma0_status_reader_enable,
    output wire          dma0_status_writer_enable,
    input  wire  [127:0] dma0_writer_axi_tdata,
    input  wire          dma0_writer_axi_tlast,
    output wire          dma0_writer_axi_tready,
    input  wire          dma0_writer_axi_tuser,
    input  wire          dma0_writer_axi_tvalid,
    output wire  [127:0] dma1_reader_axi_tdata,
    output wire          dma1_reader_axi_tlast,
    input  wire          dma1_reader_axi_tready,
    output wire          dma1_reader_axi_tuser,
    output wire          dma1_reader_axi_tvalid,
    output wire          dma1_status_reader_enable,
    output wire          dma1_status_writer_enable,
    input  wire  [127:0] dma1_writer_axi_tdata,
    input  wire          dma1_writer_axi_tlast,
    output wire          dma1_writer_axi_tready,
    input  wire          dma1_writer_axi_tuser,
    input  wire          dma1_writer_axi_tvalid,
    output wire  [127:0] dma2_reader_axi_tdata,
    output wire          dma2_reader_axi_tlast,
    input  wire          dma2_reader_axi_tready,
    output wire          dma2_reader_axi_tuser,
    output wire          dma2_reader_axi_tvalid,
    output wire          dma2_status_reader_enable,
    output wire          dma2_status_writer_enable,
    input  wire  [127:0] dma2_writer_axi_tdata,
    input  wire          dma2_writer_axi_tlast,
    output wire          dma2_writer_axi_tready,
    input  wire          dma2_writer_axi_tuser,
    input  wire          dma2_writer_axi_tvalid,
    output wire  [127:0] dma3_reader_axi_tdata,
    output wire          dma3_reader_axi_tlast,
    input  wire          dma3_reader_axi_tready,
    output wire          dma3_reader_axi_tuser,
    output wire          dma3_reader_axi_tvalid,
    output wire          dma3_status_reader_enable,
    output wire          dma3_status_writer_enable,
    input  wire  [127:0] dma3_writer_axi_tdata,
    input  wire          dma3_writer_axi_tlast,
    output wire          dma3_writer_axi_tready,
    input  wire          dma3_writer_axi_tuser,
    input  wire          dma3_writer_axi_tvalid,
    output wire   [31:0] mmap_axi_lite_araddr,
    output wire    [2:0] mmap_axi_lite_arprot,
    input  wire          mmap_axi_lite_arready,
    output wire          mmap_axi_lite_arvalid,
    output wire   [31:0] mmap_axi_lite_awaddr,
    output wire    [2:0] mmap_axi_lite_awprot,
    input  wire          mmap_axi_lite_awready,
    output wire          mmap_axi_lite_awvalid,
    output wire          mmap_axi_lite_bready,
    input  wire    [1:0] mmap_axi_lite_bresp,
    input  wire          mmap_axi_lite_bvalid,
    input  wire   [31:0] mmap_axi_lite_rdata,
    output wire          mmap_axi_lite_rready,
    input  wire    [1:0] mmap_axi_lite_rresp,
    input  wire          mmap_axi_lite_rvalid,
    output wire   [31:0] mmap_axi_lite_wdata,
    input  wire          mmap_axi_lite_wready,
    output wire    [3:0] mmap_axi_lite_wstrb,
    output wire          mmap_axi_lite_wvalid,
    input  wire   [31:0] mmap_slave_axi_araddr,
    input  wire    [1:0] mmap_slave_axi_arburst,
    input  wire    [3:0] mmap_slave_axi_arcache,
    input  wire          mmap_slave_axi_arid,
    input  wire    [7:0] mmap_slave_axi_arlen,
    input  wire          mmap_slave_axi_arlock,
    input  wire    [2:0] mmap_slave_axi_arprot,
    input  wire    [3:0] mmap_slave_axi_arqos,
    output wire          mmap_slave_axi_arready,
    input  wire    [3:0] mmap_slave_axi_arregion,
    input  wire    [2:0] mmap_slave_axi_arsize,
    input  wire          mmap_slave_axi_aruser,
    input  wire          mmap_slave_axi_arvalid,
    input  wire   [31:0] mmap_slave_axi_awaddr,
    input  wire    [1:0] mmap_slave_axi_awburst,
    input  wire    [3:0] mmap_slave_axi_awcache,
    input  wire          mmap_slave_axi_awid,
    input  wire    [7:0] mmap_slave_axi_awlen,
    input  wire          mmap_slave_axi_awlock,
    input  wire    [2:0] mmap_slave_axi_awprot,
    input  wire    [3:0] mmap_slave_axi_awqos,
    output wire          mmap_slave_axi_awready,
    input  wire    [3:0] mmap_slave_axi_awregion,
    input  wire    [2:0] mmap_slave_axi_awsize,
    input  wire          mmap_slave_axi_awuser,
    input  wire          mmap_slave_axi_awvalid,
    output wire          mmap_slave_axi_bid,
    input  wire          mmap_slave_axi_bready,
    output wire    [1:0] mmap_slave_axi_bresp,
    output wire          mmap_slave_axi_buser,
    output wire          mmap_slave_axi_bvalid,
    output wire  [127:0] mmap_slave_axi_rdata,
    output wire          mmap_slave_axi_rid,
    output wire          mmap_slave_axi_rlast,
    input  wire          mmap_slave_axi_rready,
    output wire    [1:0] mmap_slave_axi_rresp,
    output wire          mmap_slave_axi_ruser,
    output wire          mmap_slave_axi_rvalid,
    input  wire  [127:0] mmap_slave_axi_wdata,
    input  wire          mmap_slave_axi_wlast,
    output wire          mmap_slave_axi_wready,
    input  wire   [15:0] mmap_slave_axi_wstrb,
    input  wire          mmap_slave_axi_wuser,
    input  wire          mmap_slave_axi_wvalid,
    input  wire   [15:0] msi_irqs,
    input  wire          pcie_clk_n,
    (* dont_touch = "true" *)
    input  wire          pcie_clk_p,
    input  wire          pcie_rst_n,
    input  wire    [3:0] pcie_rx_n,
    input  wire    [3:0] pcie_rx_p,
    output wire    [3:0] pcie_tx_n,
    output wire    [3:0] pcie_tx_p,
    output wire          rst
);

endmodule

// -----------------------------------------------------------------------------
//  Auto-Generated by LiteX on 2024-06-15 18:32:51.
//------------------------------------------------------------------------------
