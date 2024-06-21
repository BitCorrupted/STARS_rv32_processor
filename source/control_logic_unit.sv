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
        17'b0000000000001001: begin 
            branch_type = 3'b0;
            read_mem = 1'b0;
            mem_to_reg = 1'b0;
            write_mem = 1'b0;
            alu_mux_en = 1'b1;
            store_byte = 1'b0;
            load_byte = 1'b0;
            pc_absolute_jump_vec = 1'b0;
            read_next_pc = 1'b0;

            alu_op = FOP_ADD;
        end
        endcase
    end


end





endmodule