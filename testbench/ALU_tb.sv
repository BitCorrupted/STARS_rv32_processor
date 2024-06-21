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

module tb (
    input logic [31:0] tb_rda, tb_rdb,
    input logic [3:0] tb_fop,
    output logic [31:0] tb_result,
    output logic [31:0] tb_Z, tb_N, tb_V, tb_C
);

    ALU tb_alu (.rda(tb_rda), .rdb(tb_rdb), .fop(tb_fop), .resut(tb_result), .Z(tb_Z), .N(tb_N), .V(tb_V), .C(tb_C));

    logic total_result_tests = 0;
    logic passed_result_tests = 0;
    logic [31:0] exp_result;

    task check_result;
    total_result_tests++;
    if (exp_result != tb_result)
        $display("Error: tb_result = %1d, expected %1d", tb_result, exp_result);
    else
        passed_result_tests++;
    endtask

    logic total_Z_tests = 0;
    logic passed_Z_tests = 0;
    logic [31:0] exp_Z;

    task check_Z;
    total_Z_tests++;
    if (exp_Z != tb_Z)
        $display("Error: tb_Z = %1d, expected %1d", tb_Z, exp_Z);
    else
        passed_Z_tests++;
    endtask

    logic total_N_tests = 0;
    logic passed_N_tests = 0;
    logic [31:0] exp_N;

    task check_N;
    total_N_tests++;
    if (exp_N != tb_N)
        $display("Error: tb_N = %1d, expected %1d", tb_N, exp_N);
    else
        passed_N_tests++;
    endtask

    logic total_V_tests = 0;
    logic passed_V_tests = 0;
    logic [31:0] exp_V;

    task check_V;
    total_V_tests++;
    if (exp_V != tb_V)
        $display("Error: tb_V = %1d, expected %1d", tb_V, exp_V);
    else
        passed_V_tests++;
    endtask

    logic total_C_tests = 0;
    logic passed_C_tests = 0;
    logic [31:0] exp_C;

    task check_C;
    total_C_tests++;
    if (exp_C != tb_C)
        $display("Error: tb_C = %1d, expected %1d", tb_C, exp_C);
    else
        passed_C_tests++;
    endtask

    

endmodule