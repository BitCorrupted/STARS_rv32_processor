module pc(
    output [31:0] pc_out,
    output [31:0] pc_add_4,
    input [31:0] generated_immediate,
    input logic branch_decision,
    input [31:0] pc_write_value,
    input logic pc_immediate_jump,
    input logic in_en,
    input logic clock,
    input logic reset
);

reg [31:0] current_pc;
logic [31:0] next_pc;
logic [31:0] pc_4;
logic [31:0] pc_add_immediate;

always_comb begin
    pc_4 = current_pc + 4;
    pc_add_immediate = pc_immediate_jump ? pc_write_value + {generated_immediate[30:0],1'b0}: current_pc + {generated_immediate[30:0],1'b0};

    next_pc = branch_decision ? pc_add_immediate : pc_4;
end
assign pc_add_4 = pc_4;

always_ff @(posedge clock, negedge reset) begin
    if(~reset) begin
        current_pc = 0; //placeholder constant for initialization
    end
    else begin
        if(in_en)
            current_pc = next_pc;
        else
            current_pc = current_pc;
    end

end
assign pc_out = current_pc;

endmodule