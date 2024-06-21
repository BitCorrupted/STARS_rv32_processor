typedef enum logic [2:0] {R = 0, I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_type;
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

module control_logic_unit(
    input logic [2:0] i_type,
    input logic [16:0] instruction,
    output logic [4:0] alu_op,
    output logic [2:0] branch_type,
    output logic reg_write_en, alu_mux_en, store_byte, 
    mem_to_reg, pc_absolute_jump_vec, load_byte, read_next_pc,
    write_mem, read_mem
);
inst_type i_t = i_type;
fop_t alu_op;

always_comb begin

    if (i_t == R) begin
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

        case (instruction)
        17'b00000000000110011: begin alu_op = FOP_ADD; end
        17'b01000000000110011: begin alu_op = FOP_SUB; end
        17'b00000001000110011: begin alu_op = FOP_XOR; end
        17'b00000001100110011: begin alu_op = FOP_OR; end
        17'b00000001110110011: begin alu_op = FOP_AND; end
        17'b00000000010110011: begin alu_op = FOP_SLL; end
        17'b00000001010110011: begin alu_op = FOP_SRL; end
        17'b01000001010110011: begin alu_op = FOP_SRA; end
        // 17'b00000000100110011: begin alu_op = FOP_SLT; end
        // 17'b00000000110110011: begin alu_op = FOP_SLTU; end



        endcase

    end

    else begin

        case (instruction)
        18'


        endcase
    end


end





endmodule