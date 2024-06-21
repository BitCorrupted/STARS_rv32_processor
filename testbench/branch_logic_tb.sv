`timescale 1ms/10ps
module tb;

logic [2:0] branch_type;
logic ALU_neg_flag, ALU_zero_flag, b_out;

string tb_test_name;

branch_logic b(.branch_type(branch_type), .ALU_neg_flag(ALU_neg_flag), 
.ALU_zero_flag(ALU_zero_flag), .b_out(b_out));

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
    tb_test_name = "beq branch"

    ALU_neg_flag = 0;
    ALU_zero_flag = 1;
    branch_type = 3'd1;

    exp_branch = 1;
    check_outputs(exp_branch, tb_test_name);

     $finish;
end

endmodule

module branch_logic(
    input logic [2:0] branch_type,
    input logic ALU_neg_flag, ALU_zero_flag,
    output logic b_out
);

always_comb begin
    if ((branch_type == 3'd1)&&(ALU_zero_flag)) begin
        b_out = 1'b1;

    end

    else if ((branch_type == 3'd2) && (!ALU_zero_flag)) begin
        b_out = 1'b1;

    end

    else if ((branch_type == 3'd3) && (ALU_neg_flag)) begin
        b_out = 1'b1;
    end
    
    else if((branch_type == 3'd4) && (!ALU_neg_flag) && (!ALU_zero_flag)) begin
        b_out = 1'b1;
    end

    else if ((branch_type == 3'b0)) begin
        b_out = 1'b0;

    end

    else begin
        b_out = 1'b0;

    end

end



endmodule