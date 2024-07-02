module uart_double_buffer(
    output [31:0] rxbuffer,
    input [31:0] txbuffer,
    
    output logic rx_ready_flag,
    input logic clear_rx_flag,

    output logic tx_ready_flag,
    input logic clear_tx_flag,

    input logic clock,
    input logic reset
);



endmodule