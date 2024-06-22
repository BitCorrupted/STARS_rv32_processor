
`timescale 1ms/10ps

typedef enum logic [2:0] {R = 0, I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_type;
typedef enum logic [3:0] {
    FOP_ADD = 0,
    FOP_SUB = 1,
    FOP_SLL = 2,
    FOP_SRL = 3,
    FOP_SRA = 4, 
    FOP_AND = 5,
    FOP_OR = 6, 
    FOP_XOR = 7,
    FOP_IMM = 8
} fop_t;
typedef enum logic [2:0] {BEQ = 1, BNE = 2, BLT = 3, BGE = 4, BLTU = 5, BGEU = 6, NONE = 0} b_t;

module control_logic_unit_tb;

    logic [2:0] i_type;
    logic [16:0] instruction;

    logic [3:0] alu_op;
    logic [2:0] branch_type;
    logic reg_write_en;
    logic alu_mux_en;
    logic store_byte;
    logic mem_to_reg;
    logic pc_absolute_jump_vec;
    logic load_byte;
    logic read_next_pc;
    logic write_mem;
    logic read_mem;
    wire tb;

    control_logic_unit uut (
        .i_type(i_type),
        .instruction(instruction),
        .alu_op(alu_op),
        .branch_type(branch_type),
        .reg_write_en(reg_write_en),
        .alu_mux_en(alu_mux_en),
        .store_byte(store_byte),
        .mem_to_reg(mem_to_reg),
        .pc_absolute_jump_vec(pc_absolute_jump_vec),
        .load_byte(load_byte),
        .read_next_pc(read_next_pc),
        .write_mem(write_mem),
        .read_mem(read_mem)
    );

    task check_output;
        input logic [3:0] exp_alu_op; 
        input logic [2:0] exp_branch_type;
        input logic exp_reg_write_en, exp_alu_mux_en, 
                      exp_store_byte, exp_mem_to_reg, 
                      exp_pc_absolute_jump_vec, exp_load_byte, 
                      exp_read_next_pc, exp_write_mem, 
                      exp_read_mem;
        begin
        $display("check done");
        if (alu_op !== exp_alu_op) $display("Error: ALU_OP mismatch");
        if (branch_type !== exp_branch_type) $display("Error: BRANCH_TYPE mismatch");
        if (reg_write_en !== exp_reg_write_en) $display("Error: REG_WRITE_EN mismatch");
        if (alu_mux_en !== exp_alu_mux_en) $display("Error: ALU_MUX_EN mismatch");
        if (store_byte !== exp_store_byte) $display("Error: STORE_BYTE mismatch");
        if (mem_to_reg !== exp_mem_to_reg) $display("Error: MEM_TO_REG mismatch");
        if (pc_absolute_jump_vec !== exp_pc_absolute_jump_vec) $display("Error: PC_ABSOLUTE_JUMP_VEC mismatch");
        if (load_byte !== exp_load_byte) $display("Error: LOAD_BYTE mismatch");
        if (read_next_pc !== exp_read_next_pc) $display("Error: READ_NEXT_PC mismatch");
        if (write_mem !== exp_write_mem) $display("Error: WRITE_MEM mismatch");
        if (read_mem !== exp_read_mem) $display("Error: READ_MEM mismatch");
        end
    endtask

    initial begin
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);
    i_type = 0;
    instruction = 0;

    #10;

    i_type = R;
    // ADD
    instruction = 17'b00000000000110011;
    #10;
    check_output(FOP_ADD, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SUB
    instruction = 17'b01000000000110011;
    #10;
    check_output(FOP_SUB, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // XOR
    instruction = 17'b00000001000110011;
    #10;
    check_output(FOP_XOR, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // OR
    instruction = 17'b00000001100110011;
    #10;
    check_output(FOP_OR, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // AND
    instruction = 17'b00000001110110011;
    #10;
    check_output(FOP_AND, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SLL
    instruction = 17'b00000000010110011;
    #10;
    check_output(FOP_SLL, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SRL
    instruction = 17'b00000001010110011;
    #10;
    check_output(FOP_SRL, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SRA
    instruction = 17'b01000001010110011;
    #10;
    check_output(FOP_SRA, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SLT
    instruction = 17'b00000000100110011;
    #10;
    check_output(FOP_SUB, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SLTU
    instruction = 17'b00000000110110011;
    #10;
    check_output(FOP_SUB, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // Test I-type instructions
    i_type = I;

    // LB
    instruction = 17'b00000000000000011;
    #10;
    check_output(FOP_ADD, 3'd0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1);

    // LW
    instruction = 17'b00000000100000011;
    #10;
    check_output(FOP_ADD, 3'd0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1);

    // ADDI
    instruction = 17'b00000000000010011;
    #10;
    check_output(FOP_ADD, 3'd0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SLLI
    instruction = 17'b00000000010010011;
    #10;
    check_output(FOP_SLL, 3'd0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // SLTI
    instruction = 17'b00000000100010011;
    #10;
    check_output(FOP_SUB, 3'd0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

    // Test U-type instructions
i_type = U;

// LUI
instruction = 17'b00000000000110111;
#10;
check_output(FOP_IMM, 3'd0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

// Test UJ-type instructions
i_type = UJ;

// JAL
instruction = 17'b00000000001101111;
#10;
check_output(FOP_ADD, 3'd0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0);

// Test S-type instructions
i_type = S;

// SB
instruction = 17'b00000000000100011;
#10;
check_output(FOP_ADD, 3'd0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0);

// SW
instruction = 17'b00000000100100011;
#10;
check_output(FOP_ADD, 3'd0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0);

// Test SB-type instructions
i_type = SB;

// BEQ
instruction = 17'b00000000011100011;
#10;
check_output(FOP_SUB, BEQ, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

// BNE
instruction = 17'b00000001001100011;
#10;
check_output(FOP_SUB, BNE, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

// BLT
instruction = 17'b00000001011100011;
#10;
check_output(FOP_SUB, BLT, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

// BGE
instruction = 17'b00000001101100011;
#10;
check_output(FOP_SUB, BGE, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

// BLTU
instruction = 17'b00000001111100011;
#10;
check_output(FOP_SUB, BLT, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);

// BGEU
instruction = 17'b00000000001100011;
#10;
check_output(FOP_SUB, BGE, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0);


#3 $finish; 
    end

endmodule
 
module control_logic_unit(
    input logic [2:0] i_type,
    input logic [16:0] instruction,
    output logic [3:0] alu_op,
    output logic [2:0] branch_type,
    output logic reg_write_en, alu_mux_en, store_byte, 
    mem_to_reg, pc_absolute_jump_vec, load_byte, read_next_pc,
    write_mem, read_mem
);

always_comb begin
    if (i_type == R) begin
        branch_type = 3'd0;
        read_mem = 1'b0;
        mem_to_reg = 1'b0;
        write_mem = 1'b0;
        alu_mux_en = 1'b0;
        reg_write_en = 1'b1;
        store_byte = 1'b0;
        load_byte = 1'b0;
        pc_absolute_jump_vec = 1'b0;
        read_next_pc = 1'b0;
    end

        case (instruction)
        // R-type
        17'b00000000000110011: begin alu_op = FOP_ADD; end
        17'b01000000000110011: begin alu_op = FOP_SUB; end
        17'b00000001000110011: begin alu_op = FOP_XOR; end
        17'b00000001100110011: begin alu_op = FOP_OR; end
        17'b00000001110110011: begin alu_op = FOP_AND; end
        17'b00000000010110011: begin alu_op = FOP_SLL; end
        17'b00000001010110011: begin alu_op = FOP_SRL; end
        17'b01000001010110011: begin alu_op = FOP_SRA; end
        17'b00000000100110011: begin alu_op = FOP_SUB; end //slt
        17'b00000000110110011: begin alu_op = FOP_SUB; end //sltu

        17'b00000000000000011: begin branch_type = 3'd0; read_mem = 1'b1; mem_to_reg = 1'b1; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b1; end //lb

        17'b00000000100000011: begin branch_type = 3'd0; read_mem = 1'b1; mem_to_reg = 1'b1; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //lw

        17'b00000000000010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //addi

        17'b00000000010010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SLL; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //slli

        17'b00000000100010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //slti

        17'b00000000110010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //sltiu

        17'b00000001000010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_XOR; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //xori

        17'b00000001010010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SRL; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //srli

        17'b01000001010010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SRA; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //srai

        17'b00000001100010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_OR; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //ori

        17'b00000001110010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_AND; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //andi

        17'b00000000000100011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b1; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b1; load_byte = 1'b0; end //sb

        17'b00000000100100011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b1; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //sw

        17'b00000000000110111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_IMM; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //lui

        17'b00000000011100011: begin branch_type = BEQ; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //beq

        17'b00000001001100011: begin branch_type = BNE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bne

        17'b00000001011100011: begin branch_type = BLT; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //blt

        17'b00000001101100011: begin branch_type = BGE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bge

        17'b00000001111100011: begin branch_type = BLT; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bltu

        17'b00000000001100011: begin branch_type = BGE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bgeu

        17'b00000000001100111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_absolute_jump_vec = 1'b1; store_byte = 1'b0; load_byte = 1'b0; end //jalr

        17'b00000000001101111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_absolute_jump_vec = 1'b1; store_byte = 1'b0; load_byte = 1'b0; end //jal

        endcase

    end

endmodule