module alu_mux (
input logic [31:0] imm_gen, reg_b,
input logic alu_mux_en,
output logic [31:0] rdb
);
    always_comb
        if (alu_mux_en)
            rdb = reg_b;
        else
            rdb = imm_gen;
endmodule

typedef enum logic [4:0] {
    FOP_ADD = 0,  FOP_ADDI = 1,
    FOP_SUB = 2,  FOP_SUBI = 3,
    FOP_SLL = 4,  FOP_SLLI = 5,
    FOP_SRL = 6,  FOP_SRLI = 7, 
    FOP_SRA = 8,  FOP_SRAI = 9, 
    FOP_AND = 10, FOP_ANDI = 11, 
    FOP_OR = 12,  FOP_ORI = 13, 
    FOP_XOR = 14, FOP_XORI = 15,
    FOP_IMM = 16
    } fop_t;

module ALU (
input logic [31:0] rda, rdb,
input logic [4:0] fop,
output logic [31:0] result,
output logic Z, N
);

    always_comb begin
        case (fop)
            FOP_ADD : result = rda + rdb;
            FOP_ADDI : result = rda + rdb;
            FOP_SUB : result = rda - rdb;
            FOP_SUBI : result = rda - rdb;
            FOP_SLL : result = rda << rdb;
            FOP_SLLI : result = rda << rdb;
            FOP_SRL : result = rda >> rdb;
            FOP_SRLI : result = rda >> rdb;
            FOP_SRA : result = rda >>> rdb;
            FOP_SRAI : result = rda >>> rdb;
            FOP_AND : result = rda & rdb;
            FOP_ANDI : result = rda & rdb;
            FOP_OR : result = rda | rdb;
            FOP_ORI : result = rda | rdb;
            FOP_XOR : result = rda ^ rdb;
            FOP_XORI : result = rda ^ rdb;
            FOP_IMM : result = rdb;
        endcase
    end

    assign Z = (result == 0) ? 1'b1 : 1'b0;
    assign N = (result < 0) ? 1'b1 : 1'b0;

endmodule