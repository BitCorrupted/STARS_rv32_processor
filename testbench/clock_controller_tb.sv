`timescale 1ms/10ps
module tb;
logic test_halt, test_cpu_clock, clock, reset;
integer total_tests, passed_tests = 0;
clock_controller test_cc(test_halt, test_cpu_clock, clock, reset);

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    test_halt = 0;
    clock = 0;
    reset = 0;
    reset_module;
    pulse_clock;
    pulse_clock;
    test_halt = 1;
    pulse_clock;
    pulse_clock;
    clock = 1;
    #1 test_halt = 0;
    #2 clock = 0;
    #3 pulse_clock;
    clock = 1;
    #2 test_halt = 1;
    #1 clock = 0;
    #3 pulse_clock;
    clock = 1;
    #1 test_halt = 0;
    #2 clock = 0;
    #3 pulse_clock;
    clock = 1;
    #3 test_halt = 1;
    clock = 0;
    #3 pulse_clock;
    test_halt = 0;
    pulse_clock;
    pulse_clock;
    

    #3 $finish;
    
end

task reset_module;
       reset = 1;
    #3 reset = 0;
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

module clock_controller(
    input logic halt,
    output logic cpu_clock,
    input logic clock,
    input logic reset
);

reg enable_clock;

always_ff @(negedge clock, posedge reset, negedge halt) begin
    if(reset)
        enable_clock = 0;
    else if(halt)
        enable_clock = 0;
    else if(~clock)
        enable_clock = 1;
end

assign cpu_clock = clock && enable_clock;

endmodule