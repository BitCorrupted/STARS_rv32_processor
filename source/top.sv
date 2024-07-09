module clock_psc(
  input logic clk, 
  input logic rst,
  input logic [7:0] lim,
  output logic hzX);
  
  logic [7:0] ctr;
  logic [7:0] ctr_next;
  logic next_hzX;
  logic hzX1;

  always_ff @(posedge clk, posedge rst) begin
    if(rst) begin
      ctr <= 8'b0;
      hzX1 <= 1'b0;
    end else begin
      ctr <= ctr_next;
      hzX1 <= next_hzX;
    end
  end
  always_comb begin
    if(ctr == lim) begin
      ctr_next = 8'b0;
      next_hzX = hzX1 ? 1'b0 : 1'b1; 
    end else begin
      ctr_next = ctr + 1;
      next_hzX = hzX1;
    end
  end
  
  assign hzX = (lim == 8'b0) ? clk : hzX1;
  
endmodule
