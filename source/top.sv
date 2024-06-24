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
  
  //this is a test
 ram ram(.clk(hz100), .rst(reset), .data_address(), .instruction_address(), .dm_read_en(), .dm_write_en(), .data_to_write(), .instruction_read(), .data_read());
 decoder decoder(.inst(), .imm_gen(), .rs1(), .rs2(), .rd(), .type_out(), .control_out());

 control_logic_unit control_logic(.i_type(), .instruction(), .alu_op(), .branch_type(), .register_write_en(), .alu_mux_en(), .store_byte(),
 .mem_to_reg(), .pc_absolute_jump_vec(), .load_byte(), .read_next_pc(), .write_mem(), .read_mem());

 imm_generator imm_generator(.inst(), .type_i(), .imm_gen());

alu_mux alu_mux(.imm_gen(), .reg_b(), .alu_mux(), .rdb());

branch_logic branch_logic(.branch_type(), .ALU_neg_flag(), .ALU_overflow_flag(), .ALU_zero_flag(), .b_out());

pc pc(.pc_out(), .pc_add_4(), .generated_immediate(), .branch_decision(), .pc_write_value(), .pc_immediate_jump(), .in_en(), .auipc_in(), .clock(hz100), .reset(reset));

ALU ALU(.rda(), .rdb(), .fop(), .result(), .Z(), .N(), .C(), .V());

register_file register_file(.clk(hz100), .rst(reset), .regA_address(), .regB_address(), .rd_address(), .register_write_en(), .register_write_data(), .regA_data(), .regB_data());

writeback writeback(.memory_value(), .ALU_value(), .pc_4_value(), .mem_to_reg(), .load_byte(), .read_pc_4(), .register_write());

byte_demux byte_demux(.reg_b(), .store_byte_en(), .reg_b_out(), .b_out());











endmodule

// Add more modules down here...
