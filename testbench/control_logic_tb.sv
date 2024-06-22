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

`timescale 1ms/10ps
module tb;

logic [2:0] i_type;
logic [16:0] instruction;
logic [3:0] alu_op;
logic [2:0] branch_type;
logic reg_write_en, alu_mux_en, store_byte, 
mem_to_reg, pc_absolute_jump_vec, load_byte, read_next_pc,
write_mem, read_mem;

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    #3 $finish;
end

endmodule
