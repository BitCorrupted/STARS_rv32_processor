module control_logic_unit(
    input logic [2:0] i_type,
    input logic [16:0] instruction,
    output logic [3:0] alu_op,
    output logic [2:0] branch_type,
    output logic reg_write_en, alu_mux_en, store_byte, 
    mem_to_reg, pc_absolute_jump_vec, load_byte, read_next_pc,
    write_mem, read_mem
);
`timescale 1ms/10ps
module tb;


initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    #3 $finish;
end

endmodule
