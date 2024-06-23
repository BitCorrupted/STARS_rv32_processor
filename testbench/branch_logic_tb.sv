`timescale 1ms/10ps
typedef enum logic [2:0] {BEQ = 1, BNE = 2, BLT = 3, BGE = 4, BLTU = 5, BGEU = 6, NONE = 0} b_t;
module tb;

logic [2:0] branch_type;
logic ALU_neg_flag, ALU_zero_flag, ALU_overflow_flag, b_out;

string tb_test_name;
integer exp_branch;

branch_logic b(.branch_type(branch_type), .ALU_neg_flag(ALU_neg_flag), 
.ALU_zero_flag(ALU_zero_flag), .b_out(b_out), .ALU_overflow_flag(ALU_overflow_flag));

task check_outputs;
        input logic exp_branch;  
        input string check_num;
    begin
        if (exp_branch == b_out) begin
            $info("Correct branch value. test: %s", check_num);  

        end

        else begin
            $error("Incorrect branch value. Actual: %0d, Expected: %0d. test: %s", b_out, exp_branch, check_num);
        end
    end
    endtask 

initial begin
    // make sure to dump the signals so we can see them in the waveform
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);
    //Test 1 beq
    ALU_overflow_flag = 0;
    tb_test_name = "beq branch";
    ALU_overflow_flag = 0;

    ALU_neg_flag = 0;
    ALU_zero_flag = 1;
    branch_type = BEQ;

    exp_branch = 1;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 2 no branch
    tb_test_name = "no beq branch";

    ALU_neg_flag = 0;
    ALU_zero_flag = 1;
    branch_type = BNE;

    exp_branch = 0;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 3 bne branch
    tb_test_name = "bne branch";

    ALU_neg_flag = 1;
    ALU_zero_flag = 0;
    branch_type = 3'd2;

    exp_branch = 1;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 4 no bne branch
    tb_test_name = "no bne branch";

    ALU_neg_flag = 1;
    ALU_zero_flag = 0;
    branch_type = 3'd0;

    exp_branch = 0;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 5 blt branch
    tb_test_name = "blt branch";

    ALU_neg_flag = 1;
    ALU_zero_flag = 0;
    branch_type = 3'd3;

    exp_branch = 1;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 6 no blt branch
    tb_test_name = "no blt branch";

    ALU_neg_flag = 1;
    ALU_zero_flag = 0;
    branch_type = 3'd4;

    exp_branch = 0;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 7 bge branch
    tb_test_name = "bge branch";

    ALU_neg_flag = 0;
    ALU_zero_flag = 0;
    branch_type = 3'd4;

    exp_branch = 1;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 8 no bge branch
    tb_test_name = "no bge branch";

    ALU_neg_flag = 0;
    ALU_zero_flag = 0;
    branch_type = 3'd5;

    exp_branch = 0;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 9 bltu branch
    tb_test_name = "bltu branch";

    ALU_neg_flag = 1;
    ALU_zero_flag = 0;
    branch_type = 3'd5;

    exp_branch = 1;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 10 no bltu branch
    tb_test_name = "no bltu branch";

    ALU_neg_flag = 1;
    ALU_zero_flag = 0;
    branch_type = 3'd6;

    exp_branch = 0;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 11 bgeu branch
    tb_test_name = "bgeu branch";

    ALU_neg_flag = 0;
    ALU_zero_flag = 0;
    branch_type = 3'd6;

    exp_branch = 1;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 12 no bgeu branch
    tb_test_name = "no bgeu branch";

    ALU_neg_flag = 0;
    ALU_zero_flag = 0;
    branch_type = 3'd0;

    exp_branch = 0;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4

    //Test 13 overflow
    tb_test_name = "overflow test";

    ALU_overflow_flag = 1
    ALU_neg_flag = 1;
    ALU_zero_flag = 0;
    branch_type = 3'd1;

    exp_branch = 0;
    #1
    check_outputs(exp_branch, tb_test_name);

    #4



     $finish;
end

endmodule


module branch_logic(
    input logic [2:0] branch_type,
    input logic ALU_neg_flag, ALU_zero_flag,
    output logic b_out
);

always_comb begin
    if ((branch_type == BEQ)&&(ALU_zero_flag)) begin
        b_out = 1'b1;

    end

    else if ((branch_type == BNE) && (!ALU_zero_flag)) begin
        b_out = 1'b1;

    end

    else if ((branch_type == BLT) && (ALU_neg_flag)) begin
        b_out = 1'b1;
    end
    
    else if((branch_type == BGE) && (!ALU_neg_flag) && (!ALU_zero_flag)) begin
        b_out = 1'b1;
    end

    else if ((branch_type == BLTU) && (ALU_neg_flag)) begin
        b_out = 1'b1;
    end
    
    else if((branch_type == BGEU) && (!ALU_neg_flag) && (!ALU_zero_flag)) begin
        b_out = 1'b1;
    end

    else if ((branch_type == NONE)) begin
        b_out = 1'b0;

    end

    else begin
        b_out = 1'b0;

    end

end



endmodule