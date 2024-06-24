`timescale 1ms/10ps
module tb;

string tb_test_name;
logic clock = 0;
logic reset = 1;
logic dm_read_en, dm_write_en;
logic [15:0] data_address, instruction_address;
logic [31:0] data_to_write, instruction_read, data_read;

memory_module memory_t(.clk(clock), .rst(reset), .dm_read_en(dm_read_en), .dm_write_en(dm_write_en),
.data_address(data_address), .instruction_address(instruction_address), .data_to_write(data_to_write),
.instruction_read(instruction_read), .data_read(data_read));


always begin
    clock = 0;
    #3;
    clock = 1;
    #3;
end

task reset_module;
       reset = 0;
    #3; 
    reset = 1;
    #3;
endtask

integer exp_register_val;

task check_outputs;
        input logic [31:0] exp_register_val;  
        input string check_num;

    begin
        $display("exp val = %d", exp_register_val);
        if (exp_register_val == data_read) begin
            $info("Correct reg output. test: %s", check_num);  

        end

        else begin
            $error("Incorrect regA output. Actual: %0d, Expected: %0d. address %d test: %s", data_read, exp_register_val, data_address, check_num);
        end


    end
    endtask 

    task check_instruction;
        input logic [31:0] exp_register_val;  
        input string check_num;

    begin
        $display("exp val = %d", exp_register_val);
        if (exp_register_val == instruction_read) begin
            $info("Correct reg output. test: %s", check_num);  

        end

        else begin
            $error("Incorrect regA output. Actual: %0d, Expected: %0d. address %d test: %s", instruction_read, exp_register_val, instruction_address, check_num);
        end


    end
    endtask 

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    //Test 0: On reset
    tb_test_name = "on reset";

    reset_module;
    dm_read_en = 1;
    #3;

    for (integer i = 0; i < 32; i++) begin
        data_address = i;
         exp_register_val = 32'b0;
        #3;
        check_outputs( exp_register_val, tb_test_name);
        #6;

    end


   // Test 1: write to mem
    tb_test_name = "write to mem";
    dm_read_en = 0;
    dm_write_en = 1;

    #3;

    for (integer i = 0; i < 32; i++) begin
        data_address = i;
        data_to_write = i;
        
        #6;

    end

    


    


    //Test 2: read data

    tb_test_name = "read mem";

    dm_read_en = 1;
    #3;

    for (integer i = 0; i < 32; i++) begin
        data_address = i;
         exp_register_val = i;
        #3;
        check_outputs( exp_register_val, tb_test_name);
        #6;

    end



    //Test 3: read instruction

    tb_test_name = "read instruction";

    dm_read_en = 0;
    #3;

    for (integer i = 0; i < 32; i++) begin
        instruction_address = i;
         exp_register_val = i;
        #3;
        check_instruction(exp_register_val, tb_test_name);
        #6;

    end

    //Test 4: 



    #3 $finish;
end

endmodule

module memory_module (
    input logic clk, rst,
    input logic [15:0] data_address, instruction_address,
    input logic dm_read_en, dm_write_en,
    input logic [31:0] data_to_write,
    output logic [31:0] instruction_read, data_read
);

logic [15:0] mem [8191:0] ;
initial $readmemh("cpu.mem", mem, 0, 4095);

always_ff @(posedge clk) begin

    if (dm_write_en && data_address) begin
        mem[data_address] <= data_to_write;
    end

end

always_ff @(posedge clk, negedge rst) begin
    if (!rst) begin
        data_read <= '0;
        instruction_read <= mem[16'b0];
    end

    else if (dm_read_en) begin
        data_read <= mem[data_address];

    end
    
    else if (!dm_read_en) begin
        instruction_read <= mem[instruction_address];

    end

    else begin
        instruction_read <= 32'b00000000000000000000000000010011;
        data_read <= '0;

    end
end



endmodule
