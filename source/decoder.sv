module decoder (
    input logic [31:0] inst,
    output logic [34:0] imm_gen,
    output logic [4:0] rs1, rs2, rd,
    output logic [2:0] type_out,
    output logic [17:0] control_out
);
  typedef enum logic [2:0] {R = 0, I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_t;
  inst_t inst_type;
  logic [6:0] opcode = inst[6:0];
  logic [2:0] funct3;
  logic [6:0] funct7;

always_comb begin

    if ((opcode == 7'b0000011) || (opcode == 7'b0010011) || (opcode == 7'b0011011)) begin
        inst_type = I;
    end
    if (opcode == 7'b0110011 || opcode == 7'b0111011) begin
        inst_type = R;
    end
    if (opcode == 7'b0100011) begin
        inst_type = S;
    end
    if (opcode == 7'b1100011) begin
        inst_type = SB;
    end
    if (opcode == 7'b1101111) begin
        inst_type = UJ;
    end
    if (opcode == 7'b0110111) begin
        inst_type = U;
    end

    case (inst_type) 
        R: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end
        I: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = 5'b0; rd = inst[11:7]; end
        S: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = 5'b0; end
        SB: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = 5'b0; end
        U: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end    
        UJ: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end    

        default: begin funct7 = 7'b0; funct3 = 3'b0; rs1 = 5'b0; rs2 = 5'b0; rd = inst[11:7]; end

    endcase    
    end
    
endmodule

