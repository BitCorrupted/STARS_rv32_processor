module pc(
    output [31:0] pc_out,
    output [31:0] pc_add_4,
    input [31:0] generated_immediate,
    input logic branch_decision,
    input [31:0] pc_write_value,
    input logic pc_immediate_jump,
    input logic in_en,
    input logic auipc_in,
    input logic clock,
    input logic reset
);

reg [31:0] current_pc;
logic [31:0] next_pc;
logic [31:0] pc_4;
logic [31:0] pc_add_immediate;

always_comb begin
    pc_add_immediate = pc_immediate_jump ? pc_write_value + generated_immediate + 4 : current_pc + generated_immediate + 4;
    pc_4 = current_pc + 4;

    next_pc = current_pc;
    if(in_en) begin
        next_pc = branch_decision ? pc_add_immediate : pc_4;
    end
end
assign pc_add_4 = auipc_in ? pc_add_immediate : pc_4;

always_ff @(posedge clock, posedge reset) begin
    if(reset) begin
        current_pc <= '0; //placeholder constant for initialization
    end
    else begin
        current_pc <= next_pc;
    end

end
assign pc_out = current_pc;

endmodule