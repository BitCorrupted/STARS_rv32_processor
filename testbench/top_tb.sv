`timescale 1ms/10ps
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
module tb;
 // Your code goes here...
 logic hz100, reset;
    logic [2:0] i_type; // instruction type (r, i, s, etc)
    logic [16:0] instruction; // shortened instruction from decoder to control logic
    logic [3:0] alu_op; // alu operation
    logic [2:0] branch_type; // branch command
    logic reg_write_en, alu_mux_en, store_byte, 
    mem_to_reg, pc_absolute_jump_vec, load_byte, read_next_pc,
    write_mem, read_mem;

    logic [31:0] inst; // full 32 bit instruction

    logic [31:0] imm_gen; // imm_gen output from control logic
    logic [4:0] regA, regB, rd; // for register file

    
   
    logic [31:0] register_write_data;
    logic [31:0] regA_data, regB_data;

    logic [31:0] program_counter;
    logic [31:0] program_counter_out;
    logic branch_choice;
  

    logic [31:0] result;
    logic Z, N, C, V;

    logic b_out;
    logic [31:0] data_to_write, data_read;
    
    //logic [31:0] b_out_connect;
   // assign right = data_to_write[7:0];

  
  //this is a test
//   ram ram(.clk(hz100), .rst(reset), .data_address(result), .instruction_address(program_counter), .dm_read_en(read_mem), .dm_write_en(write_mem),
//     .data_to_write(data_to_write), .instruction_read(inst), .data_read(data_read));
  
  decoder decoder(.inst(inst), .rs1(regA), .rs2(regB), .rd(rd), .type_out(i_type), .control_out(instruction));

   control_logic_unit control_logic(.i_type(i_type), .instruction(instruction), .alu_op(alu_op), .branch_type(branch_type), .reg_write_en(reg_write_en), .alu_mux_en(alu_mux_en), .store_byte(store_byte),
  .mem_to_reg(mem_to_reg), .pc_absolute_jump_vec(pc_absolute_jump_vec), .load_byte(load_byte), .read_next_pc(read_next_pc), .write_mem(write_mem), .read_mem(read_mem));

  branch_logic branch_logic(.branch_type(branch_type), .ALU_neg_flag(N), .ALU_overflow_flag(V), .ALU_zero_flag(Z), .b_out(branch_choice));

   pc pc(.pc_out(program_counter), .pc_add_4(program_counter_out), .generated_immediate(imm_gen), .branch_decision(branch_choice), .pc_write_value(regA_data), .pc_immediate_jump(pc_absolute_jump_vec), .in_en(1'b1), .auipc_in(alu_mux_en), .clock(hz100), .reset(reset));

  register_file register_file(.clk(hz100), .rst(reset), .regA_address(regA), .regB_address(regB), .rd_address(rd), .register_write_en(reg_write_en), .register_write_data(register_write_data), .regA_data(regA_data), .regB_data(regB_data));

   writeback writeback(.memory_value(data_read), .ALU_value(result), .pc_4_value(program_counter_out), .mem_to_reg(mem_to_reg), .load_byte(load_byte), .read_pc_4(1'b0), .register_write(register_write_data));



   byte_demux byte_demux(.reg_b(regB_data), .store_byte_en(store_byte), .b_out(data_to_write));

   //byte_imm_gen byte_immediate_generator (.b_out(b_out_connect), .imm_gen_byte(data_to_write));

   ALU ALU(.rda(regA_data), .fop(alu_op), .result(result), .Z(Z), .N(N), .C(C), .V(V), .imm_gen(imm_gen), .reg_b(regB_data), .alu_mux_en(alu_mux_en));

   imm_generator imm_generator(.inst(inst), .type_i(i_type), .imm_gen(imm_gen));
  
task reset_module;
       reset = 1;
    #3; 
    reset = 0;
    #3;
endtask


always begin
    hz100 = 0;
    #3;
    hz100 = 1;
    #3;
end

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    reset_module;

//     inst = 32'b00000000010100000000000100010011;

// #6;


//     inst = 32'b00000000000000000000010001100011;

//     #6;
//     inst = 32'b00000000000000000100010001100011;
       inst = 32'h00000393;
    #6 inst = 32'h00700863;
    #6 inst = 32'h00500413;
    #6 inst = 32'h00600413;
    #6 inst = 32'h00700413;
    #6 inst = 32'h00100413;
    #6 inst = 32'h00d00413;
    #6 inst = 32'h01600413;
    #6 inst = 32'h01c00413; 

    #5 $finish;
end

endmodule


module register_file(
    input logic clk, rst,
    input logic [4:0] regA_address, regB_address, rd_address,
    input logic register_write_en,
    input logic [31:0] register_write_data,
    output logic [31:0] regA_data, regB_data

);

logic [31:0][31:0] registers_state;
logic [31:0][31:0] next_registers_state;

assign regA_data = registers_state[regA_address];
assign regB_data = registers_state[regB_address];

always_comb begin
    next_registers_state = registers_state;


    if (register_write_en && rd_address != 5'b0) begin
        next_registers_state[rd_address] = register_write_data;
    end
end


always_ff @(posedge clk, posedge rst) begin
    if (rst) begin
        //for (integer i = 0; i < 32; i++) begin
        //    registers_state[i] <= 32'b0;
        //end
        //registers_state <= '{default:'0};
        registers_state <= '0;
    end

    else begin
        registers_state <= next_registers_state;
    end


end

endmodule

module ALU (
    input logic signed [31:0] rda, imm_gen, reg_b,
    input logic [3:0] fop,
    input logic alu_mux_en,
    output logic signed [31:0] result,
    output logic Z, N, C, V
);

  logic [31:0] rdb;
  assign rdb = (alu_mux_en) ? imm_gen : reg_b;


    always_comb begin
        case (fop)
            FOP_ADD : result = rda + rdb;
            FOP_SUB : result = rda - rdb;
            FOP_SLL : result = rda << rdb;
            FOP_SRL : result = rda >> rdb;
            FOP_SRA : result = rda >>> rdb;
            FOP_AND : result = rda & rdb;
            FOP_OR : result = rda | rdb;
            FOP_XOR : result = rda ^ rdb;
            FOP_IMM : result = imm_gen;
            default : result = '0;
        endcase
    end

    assign Z = (result == 0) ? 1'b1 : 1'b0;
    assign N = result[31];

    always_comb begin
        if (fop == FOP_ADD) begin
            if ((rda[31] && rdb[31] && !result[31]) || (!rda[31] && !rdb[31] && result[31]))
                V = 1'b1;
            else
                V = '0;
        end else if (fop == FOP_SUB) begin
            if ((rda[31] && !rdb[31] && !result[31]) || (!rda[31] && rdb[31] && result[31]))
                V = 1'b1;
            else
                V = '0;
        end
        else V = '0;
    end
endmodule

module branch_logic(
    input logic [2:0] branch_type,
    input logic ALU_neg_flag, ALU_zero_flag, ALU_overflow_flag,
    output logic b_out
);

always_comb begin
    if ((branch_type == BEQ)&&(ALU_zero_flag) && (!ALU_overflow_flag)) begin
        b_out = 1'b1;

    end

    else if ((branch_type == BNE) && (!ALU_zero_flag)&& (!ALU_overflow_flag)) begin
        b_out = 1'b1;

    end

    else if ((branch_type == BLT) && (ALU_neg_flag)&& (!ALU_overflow_flag)) begin
        b_out = 1'b1;
    end
    
    else if((branch_type == BGE) && (!ALU_neg_flag) && (!ALU_zero_flag)&& (!ALU_overflow_flag)) begin
      b_out = 1'b1;
    end

    else if ((branch_type == BLTU) && (ALU_neg_flag)&& (!ALU_overflow_flag)) begin
        b_out = 1'b1;
    end
    
    else if((branch_type == BGEU) && (!ALU_neg_flag) && (!ALU_zero_flag)&& (!ALU_overflow_flag)) begin
        b_out = 1'b1;
    end

    else if ((branch_type == NONE)) begin
        b_out = 1'b0;

    end

    else begin
        b_out = 1'b0;

    end

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

        case (instruction)
        // R-type
        17'b00000000000110011: begin alu_op = FOP_ADD; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b01000000000110011: begin alu_op = FOP_SUB; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b00000001000110011: begin alu_op = FOP_XOR; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b00000001100110011: begin alu_op = FOP_OR; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b00000001110110011: begin alu_op = FOP_AND; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b00000000010110011: begin alu_op = FOP_SLL; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b00000001010110011: begin alu_op = FOP_SRL; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b01000001010110011: begin alu_op = FOP_SRA; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end
        17'b00000000100110011: begin alu_op = FOP_SUB; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end //slt
        17'b00000000110110011: begin alu_op = FOP_SUB; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_absolute_jump_vec = 1'b0; read_next_pc = 1'b0;end //sltu

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

        17'b00000000000010111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //auipc

        17'b00000000001100011: begin branch_type = BEQ; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //beq

        17'b00000000011100011: begin branch_type = BNE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bne

        17'b00000001001100011: begin branch_type = BLT; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //blt

        17'b00000001011100011: begin branch_type = BGE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bge

        17'b00000001101100011: begin branch_type = BLTU; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bltu

        17'b00000001111100011: begin branch_type = BGEU; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end //bgeu

        17'b00000000001100111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_absolute_jump_vec = 1'b1; store_byte = 1'b0; load_byte = 1'b0; end //jalr

        17'b00000000001101111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_absolute_jump_vec = 1'b1; store_byte = 1'b0; load_byte = 1'b0; end //jal

        default: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_absolute_jump_vec = 1'b0; store_byte = 1'b0; load_byte = 1'b0; end
        

        endcase

    end

endmodule

module decoder (
    input logic [31:0] inst,
    //output logic [31:0] imm_gen,
    output logic [4:0] rs1, rs2, rd,
    output logic [2:0] type_out,
    output logic [16:0] control_out
);

 // typedef enum logic [2:0] {R = 0, I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_t;
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;
  logic [2:0] inst_t;

always_comb begin
    opcode = inst[6:0];
    case (opcode)
        7'b0000011, 7'b0010011, 7'b0011011: inst_t = 3'd1;
        7'b0110011, 7'b0111011: inst_t = 3'b0;
        7'b0100011: inst_t = 3'd2;
        7'b1100011: inst_t = 3'd3;
        7'b1101111: inst_t = 3'd4;
        7'b0110111: inst_t = 3'd5;
        default: inst_t = 3'b0;
    endcase
    type_out = inst_t;
end

always_comb begin
    case (inst_t) 
        3'b0: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end
        3'd1: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = 5'b0; rd = inst[11:7]; end
        3'b010: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = 5'b0; end
        3'd3: begin funct7 = 7'b0; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = 5'b0; end
        3'd5: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end    
        3'd4: begin funct7 = inst[31:25]; funct3 = inst[14:12]; rs1 = inst[19:15]; rs2 = inst[24:20]; rd = inst[11:7]; end    

        default: begin funct7 = 7'b0; funct3 = 3'b0; rs1 = 5'b0; rs2 = 5'b0; rd = 5'b0; end

    endcase 
    control_out = {funct7, funct3, inst[6:0]};
    end
    
endmodule

module byte_demux (
    input logic [31:0] reg_b,
    input logic store_byte_en,
    output logic [31:0]  b_out
);

    always_comb begin
      if(store_byte_en) begin
        b_out = {24'd0, reg_b[7:0]};
      end else begin
        b_out = reg_b;
      end
    
    end
endmodule

// module byte_imm_gen (
//     input logic [31:0] b_out,
//     output logic [31:0] imm_gen_byte
// );
//     assign imm_gen_byte = {24'd0, b_out[7:0]};
// endmodule

module imm_generator (
    input logic [31:0] inst,
    input logic [2:0] type_i,
    output logic [31:0] imm_gen
);

    always_comb begin
        case (type_i)
            3'd1 : imm_gen = {{20{inst[31]}}, inst[31:20]};
            3'd2 : imm_gen = {{20{inst[31]}}, inst[31:25], inst[11:7]};
            3'd3 : imm_gen = {{21{inst[31]}}, inst[7], inst[30:25], inst [11:8]};
            3'd5 : imm_gen = {inst[31:12], 12'd0};
            3'd4 : imm_gen = {{12{inst[31]}}, inst[19:12], inst[20], inst[31:21]};
            default : imm_gen = '0;
        endcase
    end
endmodule


module pc(
    output logic [31:0] pc_out,
    output logic [31:0] pc_add_4,
    input logic [31:0] generated_immediate,
    input logic branch_decision,
    input logic [31:0] pc_write_value,
    input logic pc_immediate_jump,
    input logic in_en,
    input logic auipc_in,
    input logic clock,
    input logic reset
);

reg [31:0] current_pc;
logic [31:0] next_pc;
logic [31:0] pc_4;
logic [31:0] pc_add_immediate;

assign pc_add_immediate = pc_immediate_jump ? (pc_write_value + {generated_immediate[30:0],1'b0}) : (current_pc + {generated_immediate[30:0],1'b0}); // program counter stuff
//assign pc_add_4 = auipc_in ? pc_add_immediate : (current_pc + 4);


always_comb begin
    next_pc = current_pc;
    if(in_en) begin
        next_pc = branch_decision ? pc_add_immediate : (current_pc + 4);
    end
end

always_ff @(posedge clock, posedge reset) begin
    if(reset) begin
        current_pc <= '0; //placeholder constant for initialization
    end
    else begin
        current_pc <= next_pc;
    end

end
assign pc_out = current_pc;

endmodule

module writeback(
    input logic [31:0] memory_value,
    input logic [31:0] ALU_value,
    input logic [31:0] pc_4_value,
    input logic mem_to_reg,
    input logic load_byte,
    input logic read_pc_4,
    output logic [31:0] register_write
);

logic [31:0] register_value;

always_comb begin
    if(read_pc_4)
        register_value = pc_4_value;
    else if(~mem_to_reg)
        register_value = ALU_value;
    else if(load_byte)
        register_value = {24'b0,memory_value[7:0]};
    else
        register_value = memory_value;
end
assign register_write = register_value;

endmodule


module ram (
    input logic clk, rst,
    input logic [31:0] data_address, // alu result to be read or written
    input logic [31:0] instruction_address, // no brainer, it is the insturction address
    input logic dm_read_en, dm_write_en, // enable ports for the read and enable
    input logic [31:0] data_to_write, // data to be written into memory
    output logic [31:0] instruction_read, data_read // things we got from memory dude
);

logic [31:0] memory [4095:0];

initial begin
        $readmemh("cpu.mem", memory);
end

always @(posedge clk) begin
    if (dm_write_en) begin
        memory[{4'b0, data_address[7:0]}] <= data_to_write;
    end
    data_read <= memory[{4'b0, data_address[7:0]}];
    instruction_read <= memory[{4'b0, instruction_address[9:2]}];
end



endmodule

module synckey(
  input logic [19:0] in,
  output logic [4:0] out,
  output logic strobe,
  input logic clk, rst
);

assign out = in[19] ? 5'd19:
in[18] ? 5'd18: 
in[17] ? 5'd17: 
in[16] ? 5'd16: 
in[15] ? 5'd15: 
in[14] ? 5'd14: 
in[13] ? 5'd13: 
in[12] ? 5'd12: 
in[11] ? 5'd11: 
in[10] ? 5'd10: 
in[9] ? 5'd9: 
in[8] ? 5'd8: 
in[7] ? 5'd7: 
in[6] ? 5'd6: 
in[5] ? 5'd5: 
in[4] ? 5'd4: 
in[3] ? 5'd3: 
in[2] ? 5'd2: 
in[1] ? 5'd1:  
in[0] ? 5'd0: 5'd0;

//assign strobe = |in;

logic Q;
always_ff @(posedge clk, posedge rst) begin

  if (rst) begin
    Q <= 1'b0;
  end
  else begin
    Q <= |in;
  end

end

always_ff @(posedge clk, posedge rst) begin
  if (rst) begin
    strobe <= 1'b0;
  end
  else begin
    strobe <= Q;
  end
end 


endmodule










