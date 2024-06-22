module decoder (
    input logic [31:0] inst,
    output logic [34:0] imm_gen,
    output logic [4:0] rs1, rs2, rd,
    output logic [2:0] type_out,
    output logic [16:0] control_out
);
  typedef enum logic [2:0] {R = 0, I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_t;
  inst_t inst_type;
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;

always_comb begin
    opcode = inst[6:0];
    case (opcode)
        7'b0000011, 7'b0010011, 7'b0011011: inst_type = I;
        7'b0110011, 7'b0111011: inst_type = R;
        7'b0100011: inst_type = S;
        7'b1100011: inst_type = SB;
        7'b1101111: inst_type = UJ;
        7'b0110111: inst_type = U;
        default: inst_type = R;
    endcase
    type_out = inst_type;
end

always_comb begin
    case (inst_type) 
        R: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end
        I: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = 5'b0; rd = inst[11:7]; end
        S: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = 5'b0; end
        SB: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = 5'b0; end
        U: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end    
        UJ: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end    

        default: begin funct7 = 7'b0; funct3 = 3'b0; rs1 = 5'b0; rs2 = 5'b0; rd = 5'b0; end

    endcase 
    control_out = {inst[6:0], funct3, funct7};
    end
    
endmodule
