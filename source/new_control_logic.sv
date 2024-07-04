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

typedef enum logic [2:0] {BEQ = 1, BNE = 2, BLT = 3, BGE = 4, BLTU = 5, BGEU = 6, JMP = 7, NONE = 0} b_t;
 
module control_logic_unit(
    input logic [2:0] i_type,
    input logic [16:0] instruction,
    output logic [3:0] alu_op,
    output logic [2:0] branch_type,
    output logic reg_write_en, alu_mux_en, store_byte, 
    mem_to_reg, pc_add_write_value, load_byte, read_next_pc,
    write_mem, read_mem, slt, u
);

always_comb begin
    u = 1'b0;
    slt = 1'b0;
    if (i_type == R) begin
        branch_type = 3'd0;
        read_mem = 1'b0;
        mem_to_reg = 1'b0;
        write_mem = 1'b0;
        alu_mux_en = 1'b0;
        reg_write_en = 1'b1;
        store_byte = 1'b0;
        load_byte = 1'b0;
        pc_add_write_value = 1'b0;
        read_next_pc = 1'b0;
    end

        case (instruction[6:0])

        7'b0110011: 
        begin
            case(instruction[9:7])

            3'b000: begin if(instruction[15]) begin
                // sub
                alu_op = FOP_SUB;
            end
            else begin 
                // add
                alu_op = FOP_ADD;
            end
            end
            3'b001: begin //sll 
            alu_op = FOP_SLL;
            end
            3'b010: begin //slt
            alu_op = FOP_SUB;
            slt = 1'b1;
            end
            3'b011: begin //sltu
            alu_op = FOP_SUB;
            slt = 1'b1;
            end
            3'b100: begin //xor
            alu_op = FOP_XOR;
            end 
            3'b101: begin if(instruction[15]) begin
                // sra
                alu_op = FOP_SRA;
            end
            else begin 
                // srl
                alu_op = FOP_SRL;
            end
            end
            3'b110: begin //or
            alu_op = FOP_OR;
            end
            3'b111: begin //and
            alu_op = FOP_AND;
            end 

            endcase
        end

        7'b0110111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_IMM; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //lui

        //7'b0110111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        //reg_write_en = 1'b1; read_next_pc = 1'b1; pc_add_write_value = 1'b1; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //auipc

        7'b1101111: begin branch_type = JMP; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //jal

        7'b1100011: begin
            case(instruction[9:7])

            3'b000: begin branch_type = BEQ; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //beq

            3'b001: begin branch_type = BNE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //bne

            3'b100: begin branch_type = BLT; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //blt

            3'b101: begin branch_type = BGE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //bge

            3'b110: begin branch_type = BLTU; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0; u = 1'b1; end //bltu

            3'b111: begin branch_type = BGEU; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; u = 1'b1; end //bgeu

            default: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end
            endcase
        end

        7'b0000011: begin
            case(instruction[9:7])

            3'b000: begin branch_type = 3'd0; read_mem = 1'b1; mem_to_reg = 1'b1; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b1; slt = 1'b0;end //lb

            3'b010: begin branch_type = 3'd0; read_mem = 1'b1; mem_to_reg = 1'b1; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //lw

            default: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end
            endcase
        end

        7'b0010011: begin
            case(instruction[9:7])

            3'b000: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //addi

            3'b001: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SLL; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //slli

            3'b010: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b1;end //slti

            3'b011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b1; u = 1'b1; end //sltiu
            
            3'b100: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_XOR; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //xori
              
            3'b101: begin if(instruction[15]) 
            begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SRA; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //srai
            else 
            begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SRL; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //srli
            end

            3'b110: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_OR; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //ori

            3'b111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_AND; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //andi

            endcase
        end

        7'b0100011: begin 
            case(instruction[9:7])
            3'b000: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b1; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b1; load_byte = 1'b0; slt = 1'b0;end //sb

            3'b010: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b1; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //sw

            default: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end
            endcase
        end
        default: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end
        endcase

    end

endmodule