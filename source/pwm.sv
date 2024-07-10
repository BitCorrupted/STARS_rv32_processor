module pwm (
  input logic signed [31:0] duty,
  input logic clk, rst
  output logic pwm_signal
);
logic [31:0] duty1;
logic [31:0] counter;

always_comb begin
  if (duty < 0)
  duty1 = -duty;
  else if (duty > 0) begin
  duty1 = duty;
  end
  else begin
  duty1 = 32'd0;
  end
end

  assign counter = '0;
  
  always_ff@(posedge clk, posedge rst) begin
    if (counter < 588000) counter <= counter + 1;
    else counter <= 0;
  end

  assign pwm_signal = (counter < duty1) ? 1:0;


endmodule