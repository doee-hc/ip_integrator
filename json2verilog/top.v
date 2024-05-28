module top_module();

      localparam m_axi_fifo_rd_instance_ID_WIDTH = 5;
      localparam m_axi_fifo_rd_instance_ADDR_WIDTH = 32;
      localparam m_axi_fifo_rd_instance_DATA_WIDTH = 32;

      localparam s_axi_fifo_rd_instance_ID_WIDTH = 5;
      localparam s_axi_fifo_rd_instance_ADDR_WIDTH = 32;
      localparam s_axi_fifo_rd_instance_DATA_WIDTH = 32;

      wire [4:0] m_axi_arid;
      wire [4:0] s_axi_arid;

      assign s_axi_arid = m_axi_arid;

      wire [31:0] m_axi_araddr;
      wire [31:0] s_axi_araddr;

      assign s_axi_araddr = m_axi_araddr;

      wire s_axi_arready;
      wire m_axi_arready;

      assign m_axi_arready = s_axi_arready;

      wire [4:0] m_axi_rid;
      wire [4:0] s_axi_rid;

      assign s_axi_rid = m_axi_rid;

      wire [31:0] m_axi_rdata;
      wire [31:0] s_axi_rdata;

      assign s_axi_rdata = m_axi_rdata;

      wire s_axi_rready;
      wire m_axi_rready;

      assign m_axi_rready = s_axi_rready;

    // Instance of m_axi_fifo_rd_instance
    axi_fifo_rd #(
        .ID_WIDTH(m_axi_fifo_rd_instance_ID_WIDTH),
        .ADDR_WIDTH(m_axi_fifo_rd_instance_ADDR_WIDTH),
        .DATA_WIDTH(m_axi_fifo_rd_instance_DATA_WIDTH)
    ) m_axi_fifo_rd_instance (
        .m_id_width(m_id_width),
        .m_addr_width(m_addr_width),
        .m_data_width(m_data_width)
    );

    // Instance of s_axi_fifo_rd_instance
    axi_fifo_rd #(
        .ID_WIDTH(s_axi_fifo_rd_instance_ID_WIDTH),
        .ADDR_WIDTH(s_axi_fifo_rd_instance_ADDR_WIDTH),
        .DATA_WIDTH(s_axi_fifo_rd_instance_DATA_WIDTH)
    ) s_axi_fifo_rd_instance (
        .s_id_width(s_id_width),
        .s_addr_width(s_addr_width),
        .s_data_width(s_data_width)
    );


endmodule