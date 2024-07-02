module spi_controller(
    input logic [31:0] clkdiv,

    input [15:0] data_in,
    input logic din_ready, 

    output [15:0] data_out,
    output logic dout_ready,
    
    //SPI signals
    output logic sclk,
    output logic miso,
    output logic mosi,
    //CS will be taken care of by GPIO module(s)

    input logic clock,
    input logic reset
);

reg [31:0] counter;
reg div_clock;

always_ff @(posedge clock, posedge reset) begin
    if(reset) begin
        counter = 0;
        div_clock = 0;
    end
    else begin
        if(counter == clkdiv) begin
            counter = 0;
            div_clock = ~div_clock;
        end
        else
            counter = counter + 1;
    end
end

endmodule