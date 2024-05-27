module transmitter_is_initiator;

wire u_initiator_transmitter_sck;
wire u_target_receiver_sck;
wire u_initiator_transmitter_ws;
wire u_target_receiver_ws;
wire [2:0] u_initiator_transmitter_sd;
wire [2:0] u_target_receiver_sd;

initiator_transmitter u_initiator_transmitter (
.sck( u_initiator_transmitter_sck_u_target_receiver_sck ),
.ws( u_initiator_transmitter_ws_u_target_receiver_ws ),
.sd( u_initiator_transmitter_sd_u_target_receiver_sd )
);
target_receiver u_target_receiver (
.sck( u_initiator_transmitter_sck_u_target_receiver_sck ),
.ws( u_initiator_transmitter_ws_u_target_receiver_ws ),
.sd( u_initiator_transmitter_sd_u_target_receiver_sd )
);
endmodule