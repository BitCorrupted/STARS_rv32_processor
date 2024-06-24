typedef enum logic [2:0] {I = 1, S = 2, SB = 3, UJ = 4, U = 5} inst_t;
  inst_t [31:0] inst_type;

module imm_generator (
    input logic [31:0] inst,
    input logic [2:0] type_i,
    output logic [31:0] imm_gen
);

    always_comb begin
        case (type_i)
            I : imm_gen = {{20{inst[31]}}, inst[31:20]};
            S : imm_gen = {{20{inst[31]}}, inst[31:25], inst[11:7]};
            SB : imm_gen = {{21{inst[31]}}, inst[7], inst[30:25], inst [11:8]};
            U : imm_gen = {inst[31:12], 12'd0};
            UJ : imm_gen = {{12{inst[31]}}, inst[19:12], inst[20], inst[31:21]};
            default : imm_gen = '0;
        endcase
    end
endmodule



module tb;
    logic [31:0] tb_inst, tb_imm_gen;
    logic [2:0] tb_type_i;

    imm_generator tb_imm_generator (.inst(tb_inst), .imm_gen(tb_imm_gen), .type_i(tb_type_i));

    logic total_imm_gen_tests = 0;
    logic passed_imm_gen_tests = 0;
    logic [31:0] exp_imm_gen;

    task check_imm_gen;
        total_imm_gen_tests++;
            if (exp_imm_gen != tb_imm_gen) begin
                $display("Error: tb_imm_gen = %1d, expected %1d", tb_imm_gen, exp_imm_gen);
            end
            else
                passed_imm_gen_tests++;
    endtask

    task TYPE_I;
        repeat (5) begin
            tb_type_i = I;
            tb_inst = $random;
            exp_imm_gen = {{20{tb_inst[31]}}, tb_inst[31:20]};
            #1;
            check_imm_gen();
            #1;
        end
    endtask

    task TYPE_S;
        repeat (5) begin
            tb_type_i = S;
            tb_inst = $random;
            exp_imm_gen = {{20{tb_inst[31]}}, tb_inst[31:25], tb_inst[11:7]};
            #1;
            check_imm_gen();
            #1;
        end
    endtask

    task TYPE_SB;
        repeat (5) begin
            tb_type_i = SB;
            tb_inst = $random;
            exp_imm_gen = {{21{tb_inst[31]}}, tb_inst[7], tb_inst[30:25], tb_inst [11:8]};
            #1;
            check_imm_gen();
            #1;
        end
    endtask

    task TYPE_UJ;
        repeat (5) begin
            tb_type_i = UJ;
            tb_inst = $random;
            exp_imm_gen = {{12{tb_inst[31]}}, tb_inst[19:12], tb_inst[20], tb_inst[31:21]};
            #1;
            check_imm_gen();
            #1;
        end
    endtask

    task TYPE_U;
        repeat (5) begin
            tb_type_i = U;
            tb_inst = $random;
            exp_imm_gen = {tb_inst[31:12], 12'd0};
            #1;
            check_imm_gen();
            #1;
        end
    endtask

    initial begin
        $dumpfile("sim.vcd");
        $dumpvars(0, tb);

        tb_inst = 0;
        tb_type_i = 0;
        #1;

        TYPE_I;
        #1;

        TYPE_S;
        #1;

        TYPE_SB;
        #1;

        TYPE_UJ;
        #1;

        TYPE_U;
        #1;

        $finish;
    end

endmodule