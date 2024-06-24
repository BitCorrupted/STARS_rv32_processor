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

    logic [34:0] imm_gen; // imm_gen output from control logic
    logic [4:0] regA, regB, rd; // for register file
  
  //this is a test
 ram ram(.clk(hz100), .rst(reset), .data_address(), .instruction_address(), .dm_read_en(), .dm_write_en(), .data_to_write(), .instruction_read(), .data_read());
 
 decoder decoder(.inst(inst), .imm_gen(imm_gen), .rs1(regA), .rs2(regB), .rd(rd), .type_out(i_type), .control_out(instruction));

 control_logic_unit control_logic(.i_type(i_type), .instruction(instruction), .alu_op(alu_op), .branch_type(branch_type), .reg_write_en(reg_write_en), .alu_mux_en(alu_mux_en), .store_byte(store_byte),
 .mem_to_reg(mem_to_reg), .pc_absolute_jump_vec(pc_absolute_jump_vec), .load_byte(load_byte), .read_next_pc(read_next_pc), .write_mem(write_mem), .read_mem(read_mem));

 imm_generator imm_generator(.inst(inst), .type_i(i_type), .imm_gen());

alu_mux alu_mux(.imm_gen(), .reg_b(), .alu_mux(), .rdb());

branch_logic branch_logic(.branch_type(), .ALU_neg_flag(), .ALU_overflow_flag(), .ALU_zero_flag(), .b_out());

pc pc(.pc_out(), .pc_add_4(), .generated_immediate(), .branch_decision(), .pc_write_value(), .pc_immediate_jump(), .in_en(), .auipc_in(), .clock(hz100), .reset(reset));

ALU ALU(.rda(), .rdb(), .fop(), .result(), .Z(), .N(), .C(), .V());

register_file register_file(.clk(hz100), .rst(reset), .regA_address(), .regB_address(), .rd_address(), .register_write_en(), .register_write_data(), .regA_data(), .regB_data());

writeback writeback(.memory_value(), .ALU_value(), .pc_4_value(), .mem_to_reg(), .load_byte(), .read_pc_4(), .register_write());

byte_demux byte_demux(.reg_b(), .store_byte_en(), .reg_b_out(), .b_out());











endmodule

// Add more modules down here...
