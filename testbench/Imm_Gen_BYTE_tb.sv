module byte_imm_gen (
    input logic [31:0] b_out,
    output logic [31:0] imm_gen_byte
);
    assign imm_gen_byte = {24'd0, b_out[7:0]};
endmodule

module tb;
    logic [31:0] tb_b_out, tb_imm_gen_byte; 

    byte_imm_gen byte_immediate_gen (.b_out(tb_b_out), .imm_gen_byte(tb_imm_gen_byte));

    logic total_imm_gen_byte_tests = 0;
    logic passed_imm_gen_byte_tests = 0;
    logic [31:0] exp_imm_gen_byte;

    task check_imm_gen_byte;
        total_imm_gen_byte_tests++;
            if (exp_imm_gen_byte != tb_imm_gen_byte) begin
                $display("Error: tb_imm_gen_byte = %1d, expected %1d", tb_imm_gen_byte, exp_imm_gen_byte);
            end
            else
                passed_imm_gen_byte_tests++;
    endtask

    task imm_gen_byte;
        repeat (5) begin
            tb_b_out = $random;
            exp_imm_gen_byte = {24'd0, tb_b_out[7:0]};
            #1;
            check_imm_gen_byte();
            #1;
        end
    endtask

    initial begin
        $dumpfile("sim.vcd");
        $dumpvars(0, tb);

        tb_b_out = 0;
        #1;

        imm_gen_byte;
        #1;

        $finish;
    end
endmodule