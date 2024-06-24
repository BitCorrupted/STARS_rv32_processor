`timescale 1ms/10ps
module tb;
logic clock = 0;
logic reset = 1;
logic [31:0] test_gen_i = 0;
logic test_branch_decision = 0;
logic [31:0] test_pc_write_value = 0;
logic test_pc_immediate_jump = 0;
logic test_in_en = 0;
logic test_auipc_in = 0;
logic [31:0] test_pc;
logic [31:0] last_pc;
logic [31:0] test_pc_4;
pc testpc(test_pc, test_pc_4, test_gen_i, test_branch_decision, test_pc_write_value, test_pc_immediate_jump, test_in_en, test_auipc_in, clock, reset);
integer total_tests = 0;
integer passed_tests = 0;

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    reset_module;
    test_value(test_pc, 0, "PC reset value error");

    // test_in_en = 1;
    // test_gen_i = 32;
    // test_pc_write_value = 256;
    // pulse_clock;
    // test_value(test_pc, 4, "PC clock increment error");
    // pulse_clock;
    // test_value(test_pc, 8, "PC clock increment error");
    // test_branch_decision = 1;
    // pulse_clock;
    // test_value(test_pc, 72, "PC Branch Error");
    // pulse_clock;
    // test_value(test_pc, 136, "PC Branch Error");
    // test_gen_i = 32'hFFFFFFF0;
    // pulse_clock;
    // test_value(test_pc, 104, "PC Negative Branch Error");
    // test_pc_immediate_jump = 1;
    // pulse_clock;
    // test_value(test_pc, 224, "PC Absolute Branch Error");

    // test_branch_decision = 0;
    // test_pc_immediate_jump = 0;
    // test_in_en = 0;
    // test_gen_i = 32;
    // test_pc_write_value = 16;
    // pulse_clock;
    // test_value(test_pc, 224, "PC Write Disable Error");
    // pulse_clock;
    // test_value(test_pc, 224, "PC Write Disable Error");
    // test_branch_decision = 1;
    // pulse_clock;
    // test_value(test_pc, 224, "PC Write Disable Error");
    // pulse_clock;
    // test_value(test_pc, 224, "PC Write Disable Error");
    // test_gen_i = 32'hFFFFFFF0;
    // pulse_clock;
    // test_value(test_pc, 224, "PC Write Disable Error");
    // test_pc_immediate_jump = 1;
    // pulse_clock;
    // test_value(test_pc, 224, "PC Write Disable Error");

    for(int i = 0; i < 500; i++) begin
        if($random > 32'hEEEEEEEE) begin
            reset_module;
            test_value(test_pc, 0, "PC Reset Error");
        end
        last_pc = test_pc;
        test_branch_decision = $random > 80000000 ? 1 : 0;
        test_pc_immediate_jump = $random > 80000000 ? 1 : 0;
        test_auipc_in = test_pc_immediate_jump || test_branch_decision ? 0 : $random > 80000000 ? 1 : 0;
        test_in_en = $random > 80000000 ? 1 : 0;

        test_gen_i = $random;
        test_gen_i = test_gen_i / 4;
        test_pc_write_value = $random;
        #1;
        if(test_auipc_in)
            test_value(test_pc_4, last_pc + test_gen_i, "auipc error");
        else
            test_value(test_pc_4, last_pc + 4, "PC_4 Add 4 Error");

        pulse_clock;
        if(~test_in_en)
            test_value(test_pc, last_pc, "PC Write Disable Error");
        else begin
            if(test_branch_decision) begin
                if(test_pc_immediate_jump)
                    test_value(test_pc, test_pc_write_value + test_gen_i, "PC Abs. Jump Error");
                else
                    test_value(test_pc, last_pc + test_gen_i, "PC Rel. Jump Error");
            end
            else
                test_value(test_pc, last_pc + 4, "PC Non-Branch Error");
        end 
        //$display("%d %d %d %d %d %d %d %D", test_pc, last_pc, test_pc_4, test_branch_decision, test_pc_immediate_jump, test_in_en, test_gen_i, test_pc_write_value);
    end


    $display("Total Tests: %d Tests Passed: %d",total_tests, passed_tests);
    #3 $finish;

end

task reset_module;
       reset = 0;
    #3 reset = 1;
    #3;
endtask

task pulse_clock;
        clock = 1;
    #3  clock = 0;
    #3;
endtask

task test_value(input int test_variable, test_value, string fail_message);
    total_tests++;
    if(test_variable == test_value)
        passed_tests++;
    else
        $display(fail_message);
endtask

endmodule

module pc(
    output [31:0] pc_out,
    output [31:0] pc_add_4,
    input [31:0] generated_immediate,
    input logic branch_decision,
    input [31:0] pc_write_value,
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

always_comb begin
    pc_add_immediate = pc_immediate_jump ? pc_write_value + generated_immediate: current_pc + generated_immediate;
    pc_4 = current_pc + 4;

    next_pc = branch_decision ? pc_add_immediate : pc_4;
end
assign pc_add_4 = auipc_in ? pc_add_immediate : pc_4;

always_ff @(posedge clock, negedge reset) begin
    if(~reset) begin
        current_pc = 0; //placeholder constant for initialization
    end
    else begin
        if(in_en)
            current_pc = next_pc;
        else
            current_pc = current_pc;
    end

end
assign pc_out = current_pc;

endmodule