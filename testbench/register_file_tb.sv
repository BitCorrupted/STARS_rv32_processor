`timescale 1ns/10ps

module tb;


string tb_test_name;
logic clock = 0;
logic reset = 1;
logic [4:0] regA_address, regB_address, rd_address;
logic register_write_en;
logic [31:0] register_write_data, regA_data, regB_data;

register_file reg_f(.clk(clock), .rst(reset), .regA_address(regA_address), 
.regB_address(regB_address), .rd_address(rd_address),
.register_write_en(register_write_en), .register_write_data(register_write_data), 
.regA_data(regA_data), .regB_data(regB_data));


task reset_module;
       reset = 0;
    #3; 
    reset = 1;
    #3;
endtask

integer exp_register_valA;
integer exp_register_valB;
always begin
    clock = 0;
    #3;
    clock = 1;
    #3;
end

task check_outputs;
        input logic exp_register_valA, exp_register_valB;  
        input string check_num;
    begin
        if (exp_register_valA == regA_data) begin
            $info("Correct regA output. test: %s", check_num);  

        end

        else begin
            $error("Incorrect regA output. Actual: %0d, Expected: %0d. test: %s", regA_data, exp_register_valA, check_num);
        end

        if (exp_register_valB == regB_data) begin
            $info("Correct regB output. test: %s", check_num);  

        end

        else begin
            $error("Incorrect regB output. Actual: %0d, Expected: %0d. test: %s", regB_data, exp_register_valB, check_num);
        end
    end
    endtask 

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);
    regA_address = '0;
    regB_address = 5'd1;
    reset = 1'b1;
    register_write_en = 1'b0;
    rd_address = 5'd0;
    register_write_data = '0;

    //Test 0: on reset
    tb_test_name = "on reset";
    reset_module;
    #3;
    for (integer i = 0; i < 32; i++) begin
        regA_address = i;
        regB_address = i;
        exp_register_valA = 32'b0;
        exp_register_valB = 32'b0;
        check_outputs(exp_register_valA, exp_register_valB, tb_test_name);
        #6;
    end

    //Test 1: write to each reg and read
    tb_test_name = "write to rd";
    #3;
    register_write_en = 1'b1;
    register_write_data = 32'd25;

    for (integer i = 0; i < 32; i++) begin
        rd_address = i;

        //check_outputs(exp_register_valA, exp_register_valB, tb_test_name);
        #6;
    end
    #3;
    register_write_en = 1'b0;
    for (integer i = 0; i < 32; i++) begin
        regA_address = i;
        regB_address = i;
        exp_register_valA = 32'd25;
        exp_register_valB = 32'd25;
        check_outputs(exp_register_valA, exp_register_valB, tb_test_name);
        #6;
    end

    



     $finish;
end

endmodule

module register_file(
    input logic clk, rst,
    input logic [4:0] regA_address, regB_address, rd_address,
    input logic register_write_en,
    input logic [31:0] register_write_data,
    output logic [31:0] regA_data, regB_data

);

logic [31:0][31:0] registers_state;
logic [31:0][31:0] next_registers_state;

always_comb begin
    next_registers_state = registers_state;
    regA_data = next_registers_state[regA_address];
    regB_data = next_registers_state[regB_address];

    if (register_write_en) begin
        next_registers_state[rd_address] = register_write_data;
    end
end


always_ff @(posedge clk, negedge rst) begin
    if (!rst) begin
        //for (integer i = 0; i < 32; i++) begin
        //    registers_state[i] <= 32'b0;
        //end
        //registers_state <= '{default:'0};
        registers_state <= '0;
    end

    else begin
        registers_state <= next_registers_state;
    end


end

endmodule