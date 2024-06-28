`timescale 1ms/10ps
module tb;
logic clock = 0;
logic reset = 1;
logic [31:0] test_gen_i = 0;
logic test_branch_decision = 0;
logic [31:0] test_pc_write_value = 0;
logic test_pc_add_write_value = 0;
logic test_in_en = 0;
logic test_auipc_in = 0;
logic [31:0] test_pc;
logic [31:0] last_pc;
logic [31:0] test_pc_add_out;
pc testpc(test_pc, test_pc_add_out, test_gen_i, test_branch_decision, test_pc_write_value, test_pc_add_write_value, test_in_en, test_auipc_in, clock, reset);
integer total_tests = 0;
integer passed_tests = 0;
integer instruction;

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    reset_module;
    test_value(test_pc, 0, "PC reset value error");

    for(int i = 0; i < 500; i++) begin
        if($random > 32'hEEEEEEEE) begin
            reset_module;
            test_value(test_pc, 0, "PC Reset Error");
        end
        last_pc = test_pc;
        instruction = $random % 6;
        case(instruction)
            0: begin//non-branching instruction
                test_gen_i = $random; //dc
                test_branch_decision = 0;
                test_pc_write_value = $random; //dc
                test_pc_add_write_value = 0;
                test_auipc_in = 0;
                test_in_en = 1;
                #1;
                pulse_clock;
                test_value(test_pc, last_pc + 4, "PC Non-Branch Error");
            end
            1: begin //branching instruction
                test_gen_i = $random;
                test_branch_decision = 1;
                test_pc_write_value = $random; //dc
                test_pc_add_write_value = 0;
                test_auipc_in = 0;
                test_in_en = 1;
                #1;
                pulse_clock;
                test_value(test_pc, last_pc + test_gen_i, "PC Branch Error");
            end
            2: begin //jal
                test_gen_i = $random;
                test_branch_decision = 1;
                test_pc_write_value = $random; //dc
                test_pc_add_write_value = 0; //dc
                test_auipc_in = 0;
                test_in_en = 1;
                #1;
                test_value(test_pc_add_out, last_pc + 4, "PC link (JAL) error");
                pulse_clock;
                test_value(test_pc, last_pc + test_gen_i, "PC jump (JAL) error");
            end
            3: begin //jalr
                test_gen_i = $random;
                test_branch_decision = 1;
                test_pc_write_value = $random;
                test_pc_add_write_value = 1; 
                test_auipc_in = 0;
                test_in_en = 1;
                #1;
                test_value(test_pc_add_out, last_pc + 4, "PC link (JALR) error");
                pulse_clock;
                test_value(test_pc, test_pc_write_value + test_gen_i, "PC jump (JALR) error");
            end
            4: begin //auipc
                test_gen_i = $random; //dc
                test_branch_decision = 0;
                test_pc_write_value = $random; //dc
                test_pc_add_write_value = 1; //dc
                test_auipc_in = 1;
                test_in_en = 1;
                #1;
                test_value(test_pc_add_out, test_pc_write_value + test_gen_i, "AUIPC add error");
                pulse_clock;
                test_value(test_pc, last_pc + 4, "AUIPC next instruction error");
            end
            5: begin //pc disable
                test_gen_i = $random; //dc
                test_branch_decision = 0;
                test_pc_write_value = $random; //dc
                test_pc_add_write_value = 0; //dc
                test_auipc_in = 0;
                test_in_en = 0;
                #1;
                pulse_clock;
                test_value(test_pc, last_pc, "PC Disable Error");
            end

        endcase
    end


    $display("Total Tests: %d Tests Passed: %d",total_tests, passed_tests);
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
    pc_add_immediate = (pc_add_write_value ? pc_write_value : current_pc) + generated_immediate; // program counter stuff
    pc_add_4 = (current_pc + 4);
end

assign pc_add_out = auipc_in ? pc_add_immediate : pc_add_4;


always_comb begin
    next_pc = current_pc;
    if(in_en) begin
        next_pc = branch_decision ? pc_add_immediate : pc_add_4;
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
