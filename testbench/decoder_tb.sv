`timescale 1ms/10ps
  typedef enum logic [2:0] {R = 0, I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_t;
  inst_t inst_type;
module tb;

    logic [31:0] inst;
    logic [34:0] imm_gen;
    logic [4:0] rs1, rs2, rd;
    logic [2:0] type_out;
    logic [16:0] control_out;

    decoder dec1(.inst(inst), .imm_gen(imm_gen), .rs1(rs1), .rs2(rs2), .rd(rd), .type_out(type_out), .control_out(control_out));

    task r_type;
    input logic [6:0] funct7;
    input logic [4:0] rs2, rs1;
    input logic [2:0] funct3;
    input logic [4:0] rd;
    input logic [6:0] opcode;

    begin
        inst = {funct7, rs2, rs1, funct3, rd, opcode};
    end
    endtask

    task i_type;
    input logic [11:0] imm;
    input logic [4:0] rs1;
    input logic [2:0] funct3;
    input logic [4:0] rd;
    input logic [6:0] funct7;

    begin
        inst = {imm, rs1, funct3, rd, funct7};
    end
    endtask

    task s_type;
    input logic [6:0] imm2;
    input logic [4:0] rs2, rs1;
    input logic [2:0] funct3;
    input logic [4:0] imm1;
    input logic [6:0] opcode;

    begin
        inst = {imm2, rs2, rs1, funct3, imm1, opcode};
    end
    endtask

    task sb_type;
    input logic [6:0] imm2;
    input logic [4:0] rs2, rs1;
    input logic [2:0] funct3;
    input logic [4:0] imm1;
    input logic [6:0] opcode;

    begin
        inst = {imm2, rs2, rs1, funct3, imm1, opcode};
    end
    endtask

    task u_type;
    input logic [19:0] imm;
    input logic [4:0] rd;
    input logic [6:0] opcode;

    begin
        inst = {imm, rd, opcode};
    end
    endtask

    task uj_type;
    input logic [19:0] imm;
    input logic [4:0] rd;
    input logic [6:0] opcode;

    begin
        inst = {imm, rd, opcode};
    end
    endtask


initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);
    inst = 32'b0;
    #10
    for (int i = 0; i < 31; i++) begin
        #10;
        r_type(7'b0, i, i, 3'b0, i, 7'b0110011);
        #10;
    end

    for (int i = 0; i < 31; i++) begin
        #10;
        i_type(11'b0, i, 3'b0, i, 7'b0000011);
        #10;
    end

    for (int i = 0; i < 31; i++) begin
        #10;
        s_type(7'b0, i, i, 3'b0, i, 7'b0100011);
        #10;
    end

    for (int i = 0; i < 31; i++) begin
        #10;
        sb_type(7'b0, i, i, 3'b0, i, 7'b1100011);
        #10;
    end

    for (int i = 0; i < 31; i++) begin
        #10;
        u_type(20'b0, i, 7'b0110111);
        #10;
    end 

    for (int i = 0; i < 31; i++) begin
        #10;
        uj_type(20'b0, i, 7'b1101111);
        #10;
    end

    #10 $finish;
end

endmodule



module decoder (
    input logic [31:0] inst,
    output logic [34:0] imm_gen,
    output logic [4:0] rs1, rs2, rd,
    output logic [2:0] type_out,
    output logic [16:0] control_out
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
    else begin
        inst_type = R;
    end
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
