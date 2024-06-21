typedef enum logic [2:0] {I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_t;
  inst_t [31:0] inst_type;

module imm_generator (
    input logic [31:0] inst,
    input logic [2:0] type_i,
    output logic [31:0] imm_gen
);

    always_comb begin
        case (type_i)
            I : imm_gen = {inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], 
            inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31:20]};

            S : imm_gen = {inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], 
            inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31:25], inst[11:7]};

            SB : imm_gen = {inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31],
            inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[7], inst[30:25], inst [11:8]};
            U : imm_gen = {inst[31:12], 12'd0};
            
            UJ : imm_gen = {inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31], inst[31],
            inst[19:12], inst[20], inst[31:21]};
            default : imm_gen = '0;
        endcase
    end
endmodule