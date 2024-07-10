
// module top (
//   // I/O ports
//   input  logic serclk, reset,
//   input  logic [20:0] pb,
//   output logic [7:0] left, right,
//          ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
//   output logic red, green, blue,

//   // UART ports
//   output logic [7:0] txdata,
//   input  logic [7:0] rxdata,
//   output logic txclk, rxclk,
//   input  logic txready, rxready
// );
// logic [31:0] duty;
// coil launch(.trig(pb[1]), .rst(reset), .clk(serclk), .charge_out(right[0]), .duty(duty));
// pwm p_time(.duty(duty), .clk(serclk), .pwm_signal(right[1]));


// endmodule

module coil (
    input logic trig, rst, clk,
    output logic charge_out,
    output logic [31:0] duty
);
logic trig_store, charge;
assign charge_out = ~charge;

always_ff @(posedge clk, posedge rst) begin
    if (rst) begin
        trig_store <= 1'b0;
    end
    else begin 
        trig_store <= (trig_store ^ trig);
    end
end

logic [31:0] counter;
always @ (posedge clk, posedge rst) begin
if (rst) begin 
    counter <= 32'd0;
end
else if ((counter < 100000000) && trig_store) begin 
    counter <= counter + 1;
    charge <= 1'b1;
end
else if (counter == 100000000) begin 
    duty <= 32'd10000;
    charge <= 1'b0;

end
else begin
    charge <= 1'b0;
    duty <= 32'd20000;

end
end

endmodule

module pwm (
  input [31:0] duty,
  input clk,
  output pwm_signal
);

  reg [31:0] counter = 0;
  
  always @ (posedge clk) begin
    if (counter < 588000) counter <= counter + 1;
    else counter <= 0;
  end

  assign pwm_signal = (counter < duty) ? 1:0;


endmodule