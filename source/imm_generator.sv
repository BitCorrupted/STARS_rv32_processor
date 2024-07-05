module imm_generator (
    input logic [31:0] inst,
    input logic [2:0] type_i,
    output logic [31:0] imm_gen
);

    always_comb begin
        case (type_i)
            3'd1 : imm_gen = {{20{inst[31]}}, inst[31:20]}; //I
            3'd2 : imm_gen = {{20{inst[31]}}, inst[31:25], inst[11:7]}; //S
            3'd3 : imm_gen = {{20{inst[31]}}, inst[7], inst[30:25], inst [11:8], 1'b0}; //SB
            3'd5 : imm_gen = {inst[31:12], 12'd0}; //U
            3'd4 : imm_gen = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21],1'b0}; //UJ
            default : imm_gen = '0;
        endcase
    end
endmodule