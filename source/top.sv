`default_nettype none
// Empty top module

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

  // Your code goes here...
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

    logic [31:0] rdb;
    logic b_out;
    logic [31:0] data_to_write, data_read;

  
  //this is a test
  ram ram(.clk(hz100), .rst(reset), .data_address(result), .instruction_address(program_counter), .dm_read_en(read_mem), .dm_write_en(write_mem),
   .data_to_write(data_to_write), .instruction_read(inst), .data_read(data_read));
  
  decoder decoder(.inst(inst), .imm_gen(imm_gen), .rs1(regA), .rs2(regB), .rd(rd), .type_out(i_type), .control_out(instruction));

  control_logic_unit control_logic(.i_type(i_type), .instruction(instruction), .alu_op(alu_op), .branch_type(branch_type), .reg_write_en(reg_write_en), .alu_mux_en(alu_mux_en), .store_byte(store_byte),
  .mem_to_reg(mem_to_reg), .pc_absolute_jump_vec(pc_absolute_jump_vec), .load_byte(load_byte), .read_next_pc(read_next_pc), .write_mem(write_mem), .read_mem(read_mem));

  branch_logic branch_logic(.branch_type(branch_type), .ALU_neg_flag(N), .ALU_overflow_flag(V), .ALU_zero_flag(Z), .b_out(branch_choice));

  pc pc(.pc_out(program_counter), .pc_add_4(program_counter_out), .generated_immediate(imm_gen), .branch_decision(branch_choice), .pc_write_value(regA_data), .pc_immediate_jump(pc_absolute_jump_vec), .in_en(), .auipc_in(alu_mux_en), .clock(hz100), .reset(reset));

  register_file register_file(.clk(hz100), .rst(reset), .regA_address(regA), .regB_address(regB), .rd_address(rd), .register_write_en(reg_write_en), .register_write_data(register_write_data), .regA_data(regA_data), .regB_data(regB_data));

  writeback writeback(.memory_value(data_read), .ALU_value(result), .pc_4_value(program_counter_out), .mem_to_reg(mem_to_reg), .load_byte(load_byte), .read_pc_4(read_next_pc), .register_write(register_write_data));



  byte_demux byte_demux(.reg_b(regB_data), .store_byte_en(store_byte), .reg_b_out(data_to_write), .b_out(b_out));

  byte_imm_gen byte_immediate_generator (.b_out(b_out), .imm_gen_byte(data_to_write));

  ALU ALU(.rda(regA_data), .rdb(regB_data), .fop(alu_op), .result(result), .Z(Z), .N(N), .C(C), .V(V));

  imm_generator imm_generator(.inst(inst), .type_i(i_type), .imm_gen(imm_gen));

  alu_mux alu_mux(.imm_gen(imm_gen), .reg_b(regB_data), .alu_mux_en(alu_mux_en), .rdb(rdb));








endmodule

// Add more modules down here...
