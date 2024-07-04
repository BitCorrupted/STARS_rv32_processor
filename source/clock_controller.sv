module clock_controller(
    input logic halt,
    output logic cpu_clock,
    input logic clock,
    input logic reset
);

reg enable_clock;

always_ff @(negedge clock, posedge reset, negedge halt) begin
    if(reset)
        enable_clock = 0;
    else if(halt)
        enable_clock = 0;
    else if(~clock)
        enable_clock = 1;
end

assign cpu_clock = clock && enable_clock;

endmodule