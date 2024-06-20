module decoder (
    input logic [6:0] opcode,
    input logic [24:0] instruction,
    output logic [16:0] control_logic,
    output logic [34:0] imm_gen,
    output logic [5:0] regA, regB, RegD
);
  typedef enum logic [2:0] {R = 0, I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_type;

always_comb begin

    if (opcode == 00000011 || opcode == 0010011 || opcode == 0011011) begin
        inst_type = I;
    end
    if (opcode == 0110011 || opcode == 0111011) begin
        inst_type = R;
    end
    if (opcode == 0100011) begin
        inst_type = S;
    end
    if (opcode == 1100011) begin
        inst_type = SB;
    end
    if (opcode == 1101111) begin
        inst_type = UJ;
    end
    if (opcode == 0110111) begin
        inst_type = U;
    end
end
//change
endmodule

