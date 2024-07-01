module ahb3lite_interconnect
import ahb3lite_pkg::*;
#(
	parameter	HADDR_SIZE  = 32,
	parameter	HDATA_SIZE  = 32,
	parameter	MASTERS  = 3,
	parameter	SLAVES  = 8,
	
	parameter bit [SLAVES-1:0] SLAVE_MASK [MASTERS] = '{MASTERS{ {SLAVES{1'b1}} }},
    parameter bit [SLAVES-1:0] ERROR_ON_SLAVE_NASK[MASTERS] = invert_slave_mask(),
    parameter bit [SLAVES-1:0] ERROR_ON_NO_SLAVE [MASTERS] = '{MASTER{1'b0}},
	parameter	MASTER_BITS = MASTERS==1 ? 1: $clog2(MASTERS)

)
(
	input 	HRESETN,
	        HCLK,
	input	[MASTER_BITS-1:0] mst_priority  [MASTERS],
	input	                  mst_HSEL      [MASTERS],
	input	[HADDR_SIZE-1:0]  mst_HADDR     [MASTERS],
	input	[HADDR_SIZE-1:0]  mst_HWDATA    [MASTERS],
	output	[HADDR_SIZE-1:0]  mst_HRDATA    [MASTERS],
	input	                  mst_HWRITE    [MASTERS],
    input   [2:0]             mst_HSIZE     [MASTERS],
    input   [2:0]             mst_HBURST    [MASTERS]

	
);