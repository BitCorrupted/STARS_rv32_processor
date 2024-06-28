module pc(
    output logic [31:0] pc_out,
    output logic [31:0] pc_add_out,
    input logic [31:0] generated_immediate,
    input logic branch_decision,
    input logic [31:0] pc_write_value,
    input logic pc_add_write_value,
    input logic in_en,
    input logic auipc_in,
    input logic clock,
    input logic reset
);

reg [31:0] current_pc;
logic [31:0] next_pc;
logic [31:0] pc_add_4;
logic [31:0] pc_add_immediate;

always_comb begin
    pc_add_immediate = pc_add_write_value ? (pc_write_value + generated_immediate) : (current_pc + {generated_immediate[30:0], 1'b0}); // program counter stuff
    pc_add_4 = (current_pc + 4);
end

assign pc_add_out = auipc_in ? pc_add_immediate : pc_add_4;


always_comb begin
    next_pc = current_pc;
    if(in_en) begin
        next_pc = (branch_decision || pc_add_write_value) ? pc_add_immediate : pc_add_4;
    end
end

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
