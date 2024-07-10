`timescale 1ms/10ps

typedef enum logic [3:0] {
    FOP_ADD = 0,
    FOP_SUB = 1,
    FOP_SLL = 2,
    FOP_SRL = 3,
    FOP_SRA = 4, 
    FOP_AND = 5,
    FOP_OR = 6, 
    FOP_XOR = 7,
    FOP_IMM = 8
    } fop_t;

module ALU (
    input logic signed [31:0] srda, imm_gen, srdb,
    input logic unsigned [31:0] rda_u, rdb_u,
    input logic [3:0] fop,
    input logic alu_mux_en, u,
    output logic [31:0] result,
    // output logic signed [31:0] sresult,
    // output logic unsigned [31:0] result_u,
    output logic Z, N, V
);
  logic [31:0] rda, rdb;
  logic [31:0] rdb_mux;
//   logic [31:0] result;

  always_comb begin
    if (!u) begin
      rda = srda;
      rdb_mux = srdb;
    //   result = sresult;
    end
    else begin
      rda = rda_u;
      rdb_mux = rdb_u;
    //   result = result_u;
    end
  end

  assign rdb = (alu_mux_en) ? imm_gen : rdb_mux;

    always_comb begin
        case (fop)
            FOP_ADD : result = rda + rdb;
            FOP_SUB : result = rda - rdb;
            FOP_SLL : result = rda << rdb;
            FOP_SRL : result = rda >> rdb;
            FOP_SRA : result = rda >>> rdb;
            FOP_AND : result = rda & rdb;
            FOP_OR  : result = rda | rdb;
            FOP_XOR : result = rda ^ rdb;
            FOP_IMM : result = imm_gen;
            default : result = '0;
        endcase
    end

    assign Z = (result == 0) ? 1'b1 : 1'b0;
    assign N = result[31];

    always_comb begin
        if (fop == FOP_ADD) begin
            if ((rda[31] && rdb[31] && !result[31]) || (!rda[31] && !rdb[31] && result[31]))
                V = 1'b1;
            else
                V = '0;
        end else if (fop == FOP_SUB) begin
            if ((rda[31] && !rdb[31] && !result[31]) || (!rda[31] && rdb[31] && result[31]))
                V = 1'b1;
            else
                V = '0;
        end
        else V = '0;
    end
endmodule

module tb;
    logic signed [31:0] tb_srda, tb_imm_gen, tb_srdb;
    logic unsigned [31:0] tb_rda_u, tb_rdb_u;
    logic [3:0] tb_fop;
    logic tb_alu_mux_en, tb_u;
    logic  [31:0] tb_result;
    logic tb_Z, tb_N, tb_V;

    ALU tb_alu (
        .srda(tb_srda),
        .imm_gen(tb_imm_gen),
        .srdb(tb_srdb),
        .rda_u(tb_rda_u),
        .rdb_u(tb_rdb_u),
        .fop(tb_fop),
        .alu_mux_en(tb_alu_mux_en),
        .u(tb_u),
        .result(tb_result),
        .Z(tb_Z),
        .N(tb_N),
        .V(tb_V));

    logic total_result_tests = 0;
    logic passed_result_tests = 0;
    logic [31:0] exp_result;

    task check_result;
        total_result_tests++;
            if (exp_result != tb_result) begin
                $display("Error: tb_result = %1d, expected %1d", tb_result, exp_result);
            end
            else
                passed_result_tests++;
    endtask

    logic total_flag_tests = 0;
    logic passed_flag_tests = 0;
    logic exp_Z, exp_N, exp_V, exp_C;

    task check_flag;
        total_flag_tests++;
            if (exp_Z != tb_Z) begin
                $display("Error: tb_Z = %1d, expected %1d", tb_Z, exp_Z);
            end
            if (exp_N != tb_N) begin 
                $display("Error: tb_N = %1d, expected %1d", tb_N, exp_N);
            end
            if (exp_V != tb_V) begin
                $display("Error: tb_V = %1d, expected %1d", tb_V, exp_V);
            end
            else 
                passed_flag_tests++;
    endtask

    task SHIFT_RIGHT_ARITHMETIC_IMM;

        // srda >>> imm_gen
        tb_srda = 32'b10000000000000000000000000000000;
        tb_srdb = 2;
        tb_rda_u = 32'b11000000000000000000000000000000;
        tb_rdb_u = 4;
        tb_imm_gen = 6;
        tb_fop = FOP_SRA;
        tb_alu_mux_en = 1'b1;
        tb_u = 0;
        exp_N = exp_result[31];
        exp_V = ((tb_srda[31] && !tb_srdb[31] && !exp_result[31]) || (!tb_srda[31] && tb_srdb[31] && exp_result[31]) || 
                 (tb_rda_u[31] && !tb_rdb_u[31] && !exp_result[31]) || (!tb_rda_u[31] && tb_rdb_u[31] && exp_result[31]));
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_result = tb_srda >>> tb_imm_gen;
        #1;
        check_result();        
        #1;
        check_flag();
        #1;

        // srda >>> srdb
        tb_srda = 32'b10000000000000000000000000000000;
        tb_srdb = 2;
        tb_rda_u = 32'b11000000000000000000000000000000;
        tb_rdb_u = 4;
        tb_imm_gen = 6;
        tb_fop = FOP_SRA;
        tb_alu_mux_en = 1'b0;
        tb_u = 0;
        exp_N = exp_result[31];
        exp_V = ((tb_srda[31] && !tb_srdb[31] && !exp_result[31]) || (!tb_srda[31] && tb_srdb[31] && exp_result[31]) || 
                 (tb_rda_u[31] && !tb_rdb_u[31] && !exp_result[31]) || (!tb_rda_u[31] && tb_rdb_u[31] && exp_result[31]));
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_result = tb_srda >>> tb_srdb;
        #1;
        check_result();        
        #1;
        check_flag();
        #1;

        // rda_u >>> imm_gen
        tb_srda = 32'b10000000000000000000000000000000;
        tb_srdb = 2;
        tb_rda_u = 32'b11000000000000000000000000000000;
        tb_rdb_u = 4;
        tb_imm_gen = 6;
        tb_fop = FOP_SRA;
        tb_alu_mux_en = 1'b1;
        tb_u = 1'b1;
        exp_N = exp_result[31];
        exp_V = ((tb_srda[31] && !tb_srdb[31] && !exp_result[31]) || (!tb_srda[31] && tb_srdb[31] && exp_result[31]) || 
                 (tb_rda_u[31] && !tb_rdb_u[31] && !exp_result[31]) || (!tb_rda_u[31] && tb_rdb_u[31] && exp_result[31]));
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_result = tb_rda_u >>> tb_imm_gen;
        #1;
        check_result();        
        #1;
        check_flag();
        #1;

        // rda_u >>> rdb_u
        tb_srda = 32'b10000000000000000000000000000000;
        tb_srdb = 2;
        tb_rda_u = 32'b11000000000000000000000000000000;
        tb_rdb_u = 4;
        tb_imm_gen = 6;
        tb_fop = FOP_SRA;
        tb_alu_mux_en = 1'b0;
        tb_u = 1'b1;
        exp_N = exp_result[31];
        exp_V = ((tb_srda[31] && !tb_srdb[31] && !exp_result[31]) || (!tb_srda[31] && tb_srdb[31] && exp_result[31]) || 
                 (tb_rda_u[31] && !tb_rdb_u[31] && !exp_result[31]) || (!tb_rda_u[31] && tb_rdb_u[31] && exp_result[31]));
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_result = tb_rda_u >>> tb_rdb_u;
        #1;
        check_result();        
        #1;
        check_flag();
        #1;
    endtask

    

    initial begin
        $dumpfile("sim.vcd");
        $dumpvars(0, tb);

        SHIFT_RIGHT_ARITHMETIC_IMM;
        #1;

        $finish;
    end
endmodule