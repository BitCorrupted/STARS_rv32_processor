`default_nettype none
// Empty top module
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

    //typedef enum logic [2:0] { I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_type;
    //inst_type [2:0] i_type;




module top (
  // I/O ports
  input  logic hz100, reset,
  input  logic [20:0] pb,
  output logic [7:0] left, right,
         ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
  output logic red, green, blue,

  // UART ports
  output logic [7:0] txdata,
  input  logic [7:0] rxdata,
  output logic txclk, rxclk,
  input  logic txready, rxready
);

wire [31:0] ssdata;
core core(.hz100(hz100), .reset(reset || pb[19]), .left(left), .right(right), .ssdata(ssdata), .pb(pb[18:0]));
ssdec ssd0(ssdata[3:0], 1'b1, ss0[6:0]);
ssdec ssd1(ssdata[7:4], 1'b1, ss1[6:0]);
ssdec ssd2(ssdata[11:8], 1'b1, ss2[6:0]);
ssdec ssd3(ssdata[15:12], 1'b1, ss3[6:0]);
ssdec ssd4(ssdata[19:16], 1'b1, ss4[6:0]);
ssdec ssd5(ssdata[23:20], 1'b1, ss5[6:0]);
ssdec ssd6(ssdata[27:24], 1'b1, ss6[6:0]);
ssdec ssd7(ssdata[31:28], 1'b1, ss7[6:0]);



endmodule

module core(
  input logic hz100, reset,
  input logic [18:0] pb, 
  output logic [7:0] right,
  output logic [7:0] left,
  output logic [31:0] ssdata

);

  logic [2:0] i_type; // instruction type (r, i, s, etc)
    logic [16:0] instruction; // shortened instruction from decoder to control logic
    logic [3:0] alu_op; // alu operation
    logic [2:0] branch_type; // branch command
    logic reg_write_en, alu_mux_en, store_byte, 
    mem_to_reg, pc_add_write_value, load_byte, read_next_pc,
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
    logic pc_en;
    logic slt;
    logic u;
    //logic [31:0] b_out_connect;
  // assign right = result[7:0];
  assign left = program_counter[7:0];
   wire [31:0]reg8_data;
   wire [31:0] reg7_data;

assign right = result[7:0];
  //assign right = reg8_data[7:0];
//    assign reset = pb[20];

  
  //this is a test
 // request unit 




logic keyclk;
logic keyclk1;

  logic i_ready, d_ready, busy_o, read_i, write_i;
  logic [3:0] sel_i;
  logic [31:0] cpu_dat_i, cpu_dat_o, adr_i;
  synckey s1(.in({1'b0, pb[18:0]}), .out(), .strobe(keyclk), .clk(hz100), .rst(reset));

  clock_controller clock_controller(.halt(1'b0), .cpu_clock(keyclk1), .clock(keyclk), .reset(reset));

    request_unit_j ru1(.clk(keyclk1), .rst(reset), .memread(1'b1), .memwrite(write_mem), .data_address(result), .instruction_address(program_counter), .alu_result(result), .busy_o(busy_o), .cpu_dat_o(cpu_dat_o), .read_i(read_i), .write_i(write_i), .cpu_dat_i(cpu_dat_i), .instruction(inst), .adr_i(adr_i), .data_read(data_read), .sel_i(sel_i), .i_hit(i_ready));

//   request_unit ru(.clk(keyclk1), .nRST(reset), .D_fetch(read_mem), .D_write(write_mem), 
//   .I_fetch(1'b1), .data_adr(result), .instr_adr(program_counter), .writedata(data_to_write), 
//   .i_done(i_ready), .d_done(d_ready), .instr(inst), .data(data_read), .busy_o(busy_o), 
//   .cpu_dat_o(cpu_dat_o), .write_i(write_i), .read_i(read_i), .adr_i(adr_i), 
//   .cpu_dat_i(cpu_dat_i), .sel_i(sel_i));

  ram ram(.clk(keyclk1), .nRST(reset), .read_i(read_i), .write_i(write_i), 
  .cpu_dat_i(cpu_dat_i), .adr_i(adr_i[11:0]), .sel_i(sel_i), 
  .cpu_dat_o(cpu_dat_o), .busy_o(busy_o));

//   dram ram(.clk(keyclk1), .rst(reset), .data_address(result), 
//   .instruction_address(program_counter), .dm_read_en(read_mem), .dm_write_en(write_mem),
//    .data_to_write(data_to_write), .instruction_read(inst), .data_read(data_read), 
//    .pc_enable(pc_en));
  
  
  decoder decoder(.inst(inst), .rs1(regA), .rs2(regB), .rd(rd), .type_out(i_type), .control_out(instruction));

   control_logic_unit control_logic(.i_type(i_type), .instruction(instruction), .alu_op(alu_op), .branch_type(branch_type), .reg_write_en(reg_write_en), .alu_mux_en(alu_mux_en), .store_byte(store_byte),
  .mem_to_reg(mem_to_reg), .pc_add_write_value(pc_add_write_value), .load_byte(load_byte), .read_next_pc(read_next_pc), .write_mem(write_mem), .read_mem(read_mem), .slt(slt), .u(u));

  branch_logic branch_logic(.branch_type(branch_type), .ALU_neg_flag(N), .ALU_overflow_flag(V), .ALU_zero_flag(Z), .b_out(branch_choice));

   pc pc(.pc_out(program_counter), .pc_add_out(program_counter_out), .generated_immediate(imm_gen), .branch_decision(branch_choice), .pc_write_value(regA_data), .pc_add_write_value(pc_add_write_value), .in_en(i_ready), .auipc_in(alu_mux_en), .clock(keyclk1), .reset(reset));

  register_file register_file(.clk(keyclk1), .rst(reset), .regA_address(regA), .regB_address(regB), .rd_address(rd), .register_write_en(reg_write_en), .register_write_data(register_write_data), .regA_data(regA_data), .regB_data(regB_data), .reg8(reg8_data), .reg7(reg7_data));

   writeback writeback(.memory_value(data_read), .ALU_value(result), .pc_4_value(program_counter_out), .mem_to_reg(mem_to_reg), .load_byte(load_byte), .read_pc_4(1'b0), .register_write(register_write_data), .slt(slt), .ALU_neg_flag(N), .ALU_overflow_flag(V));

   byte_demux byte_demux(.reg_b(regB_data), .store_byte_en(store_byte), .b_out(data_to_write));


   ALU ALU(.srda(regA_data), .fop(alu_op), .result(result), .Z(Z), .N(N), .V(V), .imm_gen(imm_gen), .srdb(regB_data), .alu_mux_en(alu_mux_en), .rda_u(regA_data), .rdb_u(regB_data), .u(u));

   imm_generator imm_generator(.inst(inst), .type_i(i_type), .imm_gen(imm_gen));
  
endmodule

module register_file(
    input logic clk, rst,
    input logic [4:0] regA_address, regB_address, rd_address,
    input logic register_write_en,
    input logic [31:0] register_write_data,
    output logic [31:0] regA_data, regB_data,
    output logic [31:0] reg8, reg7

);

logic [31:0][31:0] registers_state;
logic [31:0][31:0] next_registers_state;

assign regA_data = registers_state[regA_address];
assign regB_data = registers_state[regB_address];
assign reg8 = registers_state[8];
assign reg7 = registers_state[7];

always_comb begin
    next_registers_state = registers_state;


    if (register_write_en && rd_address != 5'b0) begin
        next_registers_state[rd_address] = register_write_data;
    end
end


always_ff @(posedge clk, posedge rst) begin
    if (rst) begin
        registers_state <= '0;
    end

    else begin
        registers_state <= next_registers_state;
    end

end

endmodule

module ALU (
    input logic signed [31:0] srda, imm_gen, srdb,
    input logic unsigned [31:0] rda_u, rdb_u,
    input logic [3:0] fop,
    input logic alu_mux_en, u,
    output logic [31:0] result,
    output logic Z, N, V
);
  logic [31:0] rda, rdb;
  logic [31:0] rdb_mux;

  always_comb begin
    if (!u) begin
      rda = srda;
      rdb_mux = srdb;
    end
  else begin
      rda = rda_u;
      rdb_mux = rdb_u;
    end
  end

  // logic [31:0] rdb;
  assign rdb = (alu_mux_en) ? imm_gen : rdb_mux;

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
    
    else if((branch_type == BGE) && (!ALU_neg_flag) && (!ALU_overflow_flag)) begin
      b_out = 1'b1;
    end

    else if ((branch_type == BLTU) && (ALU_neg_flag)&& (!ALU_overflow_flag)) begin
        b_out = 1'b1;
    end
    
    else if((branch_type == BGEU) && (!ALU_neg_flag) && (!ALU_overflow_flag)) begin
        b_out = 1'b1;
    end

    else if(branch_type == JMP) begin
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
    mem_to_reg, pc_add_write_value, load_byte, read_next_pc,
    write_mem, read_mem, slt, u
);

always_comb begin
        u = 1'b0;
        case (instruction)
        // R-type
        17'b00000000000110011: begin alu_op = FOP_ADD; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0; slt = 1'b0; end
        17'b01000000000110011: begin alu_op = FOP_SUB; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b0;end
        17'b00000001000110011: begin alu_op = FOP_XOR; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b0;end
        17'b00000001100110011: begin alu_op = FOP_OR; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b0;end
        17'b00000001110110011: begin alu_op = FOP_AND; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b0;end
        17'b00000000010110011: begin alu_op = FOP_SLL; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b0;end
        17'b00000001010110011: begin alu_op = FOP_SRL; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b0;end
        17'b01000001010110011: begin alu_op = FOP_SRA; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b0;end
        17'b00000000100110011: begin alu_op = FOP_SUB; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b1;end //slt
        17'b00000000110110011: begin alu_op = FOP_SUB; branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; write_mem = 1'b0; alu_mux_en = 1'b0; reg_write_en = 1'b1; store_byte = 1'b0;
        load_byte = 1'b0; pc_add_write_value = 1'b0; read_next_pc = 1'b0;slt = 1'b1; u = 1'b1; end //sltu

        17'b00000000000000011: begin branch_type = 3'd0; read_mem = 1'b1; mem_to_reg = 1'b1; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b1; slt = 1'b0;end //lb

        17'b00000000100000011: begin branch_type = 3'd0; read_mem = 1'b1; mem_to_reg = 1'b1; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //lw

        17'b00000000000010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //addi

        17'b00000000010010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SLL; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //slli

        17'b00000000100010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b1;end //slti

        17'b00000000110010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b1; u = 1'b1; end //sltiu

        17'b00000001000010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_XOR; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //xori

        17'b00000001010010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SRL; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //srli

        17'b01000001010010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SRA; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //srai

        17'b00000001100010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_OR; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //ori

        17'b00000001110010011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_AND; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //andi

        17'b00000000000100011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b1; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b1; load_byte = 1'b0; slt = 1'b0;end //sb

        17'b00000000100100011: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b1; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //sw

        17'b00000000000110111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_IMM; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //lui

        17'b00000000000010111: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b1; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_add_write_value = 1'b1; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //auipc

        17'b00000000001100011: begin branch_type = BEQ; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //beq

        17'b00000000011100011: begin branch_type = BNE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //bne

        17'b00000001001100011: begin branch_type = BLT; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //blt

        17'b00000001011100011: begin branch_type = BGE; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //bge

        17'b00000001101100011: begin branch_type = BLTU; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0; u = 1'b1; end //bltu

        17'b00000001111100011: begin branch_type = BGEU; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_SUB; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; u = 1'b1; end //bgeu

        17'b00000000001100111: begin branch_type = JMP; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_add_write_value = 1'b1; store_byte = 1'b0; load_byte = 1'b0; slt = 1'b0;end //jalr

        17'b00000000001101111: begin branch_type = JMP; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b1; read_next_pc = 1'b1; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end //jal

        default: begin branch_type = 3'd0; read_mem = 1'b0; mem_to_reg = 1'b0; alu_op = FOP_ADD; write_mem = 1'b0; alu_mux_en = 1'b0; 
        reg_write_en = 1'b0; read_next_pc = 1'b0; pc_add_write_value = 1'b0; store_byte = 1'b0; load_byte = 1'b0;slt = 1'b0; end
        

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
        3'd5: begin funct7 = 7'b0; funct3 = 3'b0; rs1 = 5'b0; rs2 = 5'b0; rd = inst[11:7]; end    
        3'd4: begin funct7 = 7'b0; funct3 = 3'b0; rs1 = 5'b0; rs2 = 5'b0; rd = inst[11:7]; end  

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

module imm_generator (
    input logic [31:0] inst,
    input logic [2:0] type_i,
    output logic [31:0] imm_gen
);

    always_comb begin
        case (type_i)
            3'd1 : imm_gen = {{20{inst[31]}}, inst[31:20]}; //I
            3'd2 : imm_gen = {{20{inst[31]}}, inst[31:25], inst[11:7]}; //S
            3'd3 : imm_gen = {{20{inst[31]}}, inst[7], inst[30:25], inst [11:8], 1'b0}; //SB
            3'd5 : imm_gen = {inst[31:12], 12'd0}; //U
            3'd4 : imm_gen = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21],1'b0}; //UJ
            default : imm_gen = '0;
        endcase
    end
endmodule

module pc(
    output logic [31:0] pc_out,
    output logic [31:0] pc_add_out,
    input logic [31:0] generated_immediate,
    input logic branch_decision,
    input logic [31:0] pc_write_value,
    input logic pc_add_write_value,
    input logic in_en,
    input logic auipc_in,
    input logic clock,
    input logic reset
);

reg [31:0] current_pc;
logic [31:0] next_pc;
logic [31:0] pc_add_4;
logic [31:0] pc_add_immediate;

always_comb begin
    pc_add_immediate = (pc_add_write_value ? pc_write_value : current_pc) + generated_immediate; //pc_add_immediate is true for AUIPC and JALR
    pc_add_4 = (current_pc + 4); //next instruction location
end

assign pc_add_out = auipc_in ? pc_add_immediate : pc_add_4; //AUIPC decision


always_comb begin
    next_pc = current_pc;
    if(in_en) begin //if PC is enabled
        next_pc = branch_decision ? pc_add_immediate : pc_add_4; //if branch or jump, then branch, else next instruction
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
    input logic slt, ALU_neg_flag, ALU_overflow_flag,
    output logic [31:0] register_write
);

logic [31:0] register_value;

always_comb begin
    if(read_pc_4)
        register_value = pc_4_value;
    
    else if(load_byte)
        register_value = {24'b0,memory_value[7:0]};

    else if(slt) begin
        if ((ALU_neg_flag) && (!ALU_overflow_flag)) begin
            register_value = 32'd1;
        end
        else begin
            register_value = 32'b0;
        end
    end
    else if(~mem_to_reg)
        register_value = ALU_value;
    else
        register_value = memory_value;
end
assign register_write = register_value;

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

module ssdec(
  input logic [3:0] in,
  input logic enable,
  output logic [6:0] out
);

always_comb begin
  case({enable,in})
    5'b10000: begin out = 7'b0111111; end
    5'b10001: begin out = 7'b0000110; end
    5'b10010: begin out = 7'b1011011; end
    5'b10011: begin out = 7'b1001111; end
    5'b10100: begin out = 7'b1100110; end
    5'b10101: begin out = 7'b1101101; end
    5'b10110: begin out = 7'b1111101; end
    5'b10111: begin out = 7'b0000111; end
    5'b11000: begin out = 7'b1111111; end
    5'b11001: begin out = 7'b1100111; end
    5'b11010: begin out = 7'b1110111; end
    5'b11011: begin out = 7'b1111100; end
    5'b11100: begin out = 7'b0111001; end
    5'b11101: begin out = 7'b1011110; end
    5'b11110: begin out = 7'b1111001; end
    5'b11111: begin out = 7'b1110001; end
    default: begin out = 7'b0000000; end
  endcase
end

endmodule

module clock_controller(
    input logic halt,
    output logic cpu_clock,
    input logic clock,
    input logic reset
);

reg enable_clock;

always_ff @(negedge clock, posedge reset) begin
    if(reset)
      enable_clock = 0;
    else 
      enable_clock = ~halt;
end

assign cpu_clock = clock && enable_clock;

endmodule


// module request_unit (
//     // From CPU
//     input logic clk, nRST,
//     input logic D_fetch, D_write, I_fetch,  
//     input logic [31:0] data_adr, instr_adr, 
//     input logic [31:0] writedata,           
//     // To CPU
//     output logic i_done, d_done,            
//     output logic [31:0] instr, data,        
//     // From Wishbone
//     input logic busy_o,                     
//     input logic [31:0] cpu_dat_o,          
//     output logic write_i, read_i,
//     output logic [31:0] adr_i,              
//     output logic [31:0] cpu_dat_i,          
//     output logic [3:0] sel_i                
// );

//     assign sel_i = 4'b1111;

//     logic [31:0] next_instr, next_data, next_adr, next_data_w;

//     typedef enum logic [1:0] {IDLE, READ_D, READ_I, WRITE_D} StateType;

//     StateType state, next_state;

//     always_ff @(posedge clk, posedge nRST) begin
//         if (nRST) begin
//             state       <= IDLE;
//             instr       <= 32'b0;
//             data        <= 32'b0;
//             adr_i       <= 32'b0;
//             cpu_dat_i   <= 32'b0;
//         end else begin
//             state       <= next_state;
//             instr       <= next_instr;
//             data        <= next_data;
//             adr_i       <= next_adr;
//             cpu_dat_i   <= next_data_w;
//         end
//     end

//     always_comb begin
//         // To Wishbone
//         read_i      = 1'b0; // read request
//         write_i     = 1'b0; // write request
//         // To CPU
//         i_done      = 1'b0; 
//         d_done      = 1'b0;
//         ///////////////////
//         next_state  = state;
//         next_data   = data;
//         next_instr  = instr;
//         next_adr    = adr_i;
//         next_data_w = cpu_dat_i;
//         case (state)
//             IDLE: begin
//                 if (D_fetch) begin
//                     read_i      = 1'b1;
//                     write_i     = 1'b0;
//                     next_adr    = data_adr;
//                     next_state  = READ_D;
//                 end else if (D_write) begin
//                     read_i      = 1'b0;
//                     write_i     = 1'b1;
//                     next_adr    = data_adr;
//                     next_data_w = writedata;
//                     next_state  = WRITE_D;
//                 end else if (I_fetch) begin
//                     read_i      = 1'b1;
//                     write_i     = 1'b0;
//                     next_adr    = instr_adr;
//                     next_state  = READ_I;
//                 end else begin
//                     read_i      = 1'b0;
//                     write_i     = 1'b0;
//                     next_adr    = 32'b0;
//                     next_data_w = 32'b0;
//                     next_state  = IDLE;
//                 end
//             end
//             READ_I: begin
//                 read_i          = 1'b0;
//                 if (busy_o) begin
//                     next_instr  = cpu_dat_o;
//                 end else begin
//                     next_adr    = 32'b0;
//                     i_done      = 1'b1;
//                     next_instr  = 32'b0;
//                     next_state  = IDLE;
//                 end
//             end
//             READ_D: begin
//                 read_i          = 1'b0;
//                 if (busy_o) begin
//                     next_data   = cpu_dat_o;
//                 end else begin
//                     next_adr    = 32'b0;
//                     d_done      = 1'b1;
//                     next_data   = 32'b0;
//                     next_state  = IDLE;
//                 end
//             end
//             WRITE_D: begin
//                 write_i         = 1'b0;
//                 if (!busy_o) begin
//                     next_adr    = 32'b0;
//                     next_data_w = 32'b0;
//                     d_done      = 1'b1;
//                     next_state  = IDLE;
//                 end
//             end
//             default:; 
//         endcase
//     end
// endmodule

module request_unit_j(
    input logic clk, rst, memread, memwrite, // CPU side signals
    input logic [31:0] data_address, 
    input logic [31:0] instruction_address, alu_result, 
    input logic busy_o, // wish bone
    input logic [31:0] cpu_dat_o, // wishbone side signals as inputs
    output logic read_i, write_i, 
    output logic [31:0] cpu_dat_i,
    output logic [31:0] instruction,
    output logic [31:0] adr_i,
    output logic [31:0] data_read,
    output logic [3:0] sel_i,
    output logic i_hit // pc enable signal 
);

typedef enum logic [1:0] {
    IDLE, BUSY
} state_t;

state_t state, next_state;
logic request_type, next_request_type, next_i_hit, i_request, d_hit, next_read, next_write, next_d_hit, next_i_request; // if 1, we are requesting data from memory (like IF or DF so on and so forth so yeah)
logic [31:0] next_cpu_dat, next_instruction, next_adr;

always_ff @(posedge clk, posedge rst) begin
    if(rst) begin
        read_i <= 1'b0; // read request
        write_i <= 1'b0; // write request
        adr_i <= '0; // address
        cpu_dat_i <= '0; // cpu data
        sel_i <= '0;
        instruction <= '0;
        i_hit <= 1'b0;
        state <= IDLE;
        i_request <= 1'b0;
        d_hit <= 1'b0;
    end else begin
        read_i <= next_read;
        write_i <= next_write;
        adr_i <= next_adr;
        cpu_dat_i <= next_cpu_dat;
        sel_i <= 4'd15;
        instruction <= next_instruction;
        d_hit <= next_d_hit;
        i_hit <= next_i_hit;
        state <= next_state;
        i_request <= next_i_request;
    end
end

    always_comb begin
        // if we are busy, we want to retain our current values below
        next_adr = adr_i;
        next_read = read_i;
        next_write = write_i;
        next_cpu_dat = cpu_dat_i;
        next_instruction = instruction;
        next_i_hit = i_hit;
        next_d_hit = d_hit;
        next_state = state;
        next_i_request = i_request;
        data_read = '0;
        case(state)
            IDLE : begin
                next_state = (!busy_o) ? BUSY : IDLE; 
                if(memread && !d_hit) begin // reading data from memory to write back

                    // settings to the wishbone interface bus

                    next_read = 1'b1; // enabling read
                    next_write = 1'b0; // disabling write
                    next_adr = alu_result; // setting the address to the address computed by the alu
                    next_i_request = 1'b0; 

                end else if (memwrite &&  !d_hit) begin // writing data into memory

                    // settings to the wishbone interface bus

                    next_read = 1'b0; // disabling read
                    next_write = 1'b1; // enabling write
                    next_adr = data_address; // address of the data we want to write into
                    next_cpu_dat = alu_result; // data we are writing into
                    next_i_request = 1'b0; // we are not requesting data

                end else begin // instruction fetching from the memory

                    // settings to the wishbone interface bus
                    next_read = 1'b1; // enabling read
                    next_write = 1'b0; // disabling write
                    next_adr = data_address; // setting the address of the instruction we are trying to fetch currently
                    next_i_request = 1'b1; // requesitng from memory
                end
            end
            
            BUSY : begin
                if(!busy_o) begin
                    next_d_hit = !i_request;
                    next_i_hit = i_request; // We only want pc counter to increment when we get an instruction fetch hit
                    // Note, we might not need to register the i_hit at all now, might need to double check the timing of this
                    next_state = IDLE;

                    if(memread) begin
                        data_read = cpu_dat_o; // data to be written back into the register                        
                    end else begin
                        next_instruction = cpu_dat_o; // The next instruction
                    end
                end
            end
            default:;
        endcase
   end

endmodule

module ram (
    input logic clk, nRST,
    // From Request Unit
    input logic         read_i, write_i,
    input logic [31:0]  cpu_dat_i,
    input logic [11:0]  adr_i,
    input logic [3:0]   sel_i,
    // To Request Unit
    output logic [31:0] cpu_dat_o,
    output logic        busy_o
);
    logic [31:0] memory [4095:0];

    logic next_busy_o, wen, next_wen, ren, next_ren, read_done;
    reg write_done;

    ///////////////////////////////////////////////

    initial begin
        $readmemh("cpu.mem", memory);
    end

    always @(negedge clk) begin
        if (wen) begin
            memory[adr_i]   <= cpu_dat_i; // we are setting the memory at the address location equal to the cpu_dat_i
            cpu_dat_o       <= 32'b0; // we aren't sending any value back
            write_done      <= 1'b1;
            read_done       <= 1'b0;
        end else if (ren) begin
            cpu_dat_o       <= memory[adr_i]; // data we are reading and sending back subsequently 
            write_done      <= 1'b0;
            read_done       <= 1'b1;
        end else begin
            cpu_dat_o       <= 32'b0;
            write_done      <= 1'b0;
            read_done       <= 1'b0; 
        end 
    end

    typedef enum logic [1:0] {IDLE, READ, WRITE} StateType;

    StateType state, next_state;

    always_ff @(posedge clk, posedge nRST) begin
        if (nRST) begin
            state       <= IDLE;
            busy_o      <= 1'b0;
            wen         <= 1'b0;
            ren <= 1'b0;
        end else begin
            state       <= next_state;
            busy_o      <= next_busy_o;
            wen         <= next_wen;
            ren <= next_ren;
        end
    end

    always_comb begin
        next_state  = state;
        next_busy_o = busy_o;
        next_wen    = wen;
        next_ren    = ren;
        case (state)
            IDLE: begin // RU in IDLE or READ_I or READ_D or WRITE_D
                if (read_i && !write_i) begin
                    next_state  = READ;
                    next_busy_o = 1'b1;
                    next_ren    = 1'b1;
                    next_wen = 1'b0;
                end else if (write_i && !read_i) begin
                    next_ren = 1'b0;
                    next_wen    = 1'b1;
                    next_state  = WRITE;
                    next_busy_o = 1'b1;
                end
            end 
            READ: begin // RU in READ_I or READ_D
                if (read_done) begin
                    next_state  = IDLE;
                    next_busy_o = 1'b0;
                    next_ren    = 1'b0;
                    next_wen = 1'b0;
                end
            end
            WRITE: begin // RU in WRITE_D
                if (write_done) begin
                    next_state  = IDLE;
                    next_busy_o = 1'b0;
                    next_wen    = 1'b0;
                    next_ren = 1'b0;
                end 
            end
            default:; 
        endcase
    end
    
endmodule





module dram (
    input logic clk, rst,
    input logic [31:0] data_address, // alu result to be read or written
    input logic [31:0] instruction_address, // no brainer, it is the insturction address
    input logic dm_read_en, dm_write_en, // enable ports for the read and enable
    input logic [31:0] data_to_write, // data to be written into memory
    output logic [31:0] instruction_read, data_read, // things we got from memory dude
    output logic pc_enable
);

logic [31:0] memory [4095:0];

initial begin
        $readmemh("cpu.mem", memory);
end


typedef enum logic {IDLE, WAIT} StateType;

StateType state, next_state;


always_ff @(posedge clk, posedge rst) begin

  if (rst) begin

    state <= IDLE;

  end else begin
    state <= next_state;
  end

end


// assign data_out = memory[address_DM];

// assign instr_out = memory[address_IM];


always_comb begin

  pc_enable = 1'b1;

  next_state = state;

  case (state)

  IDLE: begin

    if (dm_read_en | dm_write_en) begin

      pc_enable = 1'b0;

      next_state = WAIT;

    end

  end

  WAIT: begin

  // pc_enable = 1'b1;

    next_state = IDLE;

  end

  endcase

end

always @(negedge clk) begin
    if (dm_write_en) begin
        memory[{4'b0, data_address[7:0]}] <= data_to_write;
    end
    data_read <= memory[{4'b0, data_address[7:0]}];
    instruction_read <= memory[{4'b0, instruction_address[9:2]}];
end



endmodule
