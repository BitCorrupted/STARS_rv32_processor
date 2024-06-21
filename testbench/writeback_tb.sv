`timescale 1ms/10ps
module tb;

logic [3:0] a;
logic [31:0] out;

writeback wb(32'hFFFFFFFF, 32'hEEEEEEEE, 32'hDDDDDDDD, a[0], a[1], a[2], out);
initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);
    for(int i = 0; i < 8; i++)
        #10 a = i;
    #3 $finish;
end

endmodule

module writeback(
    input [31:0] memory_value,
    input [31:0] ALU_value,
    input [31:0] pc_4_value,
    input mem_to_reg,
    input load_byte,
    input read_pc_4,
    output register_write
);

logic [31:0] register_value;

always_comb begin
    // if(read_pc_4)
    //     register_value = pc_4_value;
    // else if(~mem_to_reg)
    //     register_value = ALU_value;
    // else if(load_byte)
    //     register_value = {24'b0,memory_value[7:0]};
    // else
    //     register_value = memory_value;
end

//assign register_write = register_value;
assign register_write = memory_value;

endmodule
