module pc(
    output [31:0] pc_out,
    output [31:0] pc_add_4,
    input [31:0] generated_immediate,
    input logic branch_decision,
    input [31:0] pc_write_value,
    input logic pc_immediate_jump,
    input logic in_en,
    logic clock,
    logic reset
);

reg [31:0] current_pc;
logic [31:0] next_pc;

always_comb begin
    logic [31:0] pc_4;
    logic [31:0] pc_add_immediate;

    assign pc_4 = current_pc + 4;
    assign pc_add_4 = pc_4;
    assign pc_add_immediate = pc_immediate_jump ? pc_write_value : current_pc + generated_immediate;

    assign next_pc = branch_decision ? pc_add_immediate : pc_4;
end

always_ff @(posedge clock, negedge reset) begin
    if(~reset) begin
        current_pc = 0;
    end
    else begin
        if(in_en)
            current_pc = next_pc;
        else
            current_pc = current_pc;
    end

end

endmodule