`timescale 1ms/10ps
module tb;

logic [3:0] a;
logic [31:0] out;
integer total_tests = 0;
integer passed_tests = 0;

writeback wb(32'hFFFFFFFF, 32'hEEEEEEEE, 32'hDDDDDDDD, a[0], a[1], a[2], out);
initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);
    for(int i = 0; i < 8; i++) begin
        a = i[3:0];
        #2;
        if(a[2])
            test_value(out, 32'hDDDDDDDD, "Improper PC+4 writeback");
        else if(~a[0])
            test_value(out, 32'hEEEEEEEE, "Improper ALU writeback");
        else if(a[1])
            test_value(out, 32'h000000FF, "Improper Byte writeback");
        else
            test_value(out, 32'hFFFFFFFF, "Improper Memory writeback");
    end
    $display("Total Tests: %d Tests Passed: %d",total_tests, passed_tests);
    #3 $finish;
end

task test_value(input int test_variable, test_value, string fail_message);
    total_tests++;
    if(test_variable == test_value)
        passed_tests++;
    else
        $display(fail_message);
endtask

endmodule

module writeback(
    input logic [31:0] memory_value,
    input logic [31:0] ALU_value,
    input logic [31:0] pc_4_value,
    input logic mem_to_reg,
    input logic load_byte,
    input logic read_pc_4,
    input logic slt, ALU_neg_flag, ALU_overflow_flag
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

    else if(slt) begin
        if ((ALU_neg_flag) && (!ALU_overflow_flag)) begin
            register_value = 32'd1;
        end
        else begin
            register_value = '0;
        end
    end
    else
        register_value = memory_value;
end
assign register_write = register_value;

endmodule