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
    input logic [31:0] rda, rdb,
    input logic [3:0] fop,
    output logic [31:0] result,
    output logic Z, N, C, V
);

    always_comb begin
        case (fop)
            FOP_ADD : result = rda + rdb;
            FOP_SUB : result = rda - rdb;
            FOP_SLL : result = rda << rdb;
            FOP_SRL : result = rda >> rdb;
            FOP_SRA : result = rda >>> rdb;
            FOP_AND : result = rda & rdb;
            FOP_OR : result = rda | rdb;
            FOP_XOR : result = rda ^ rdb;
            FOP_IMM : result = rdb;
            default : result = '0;
        endcase
    end

    assign Z = (result == 0) ? 1'b1 : 1'b0;
    assign N = result[31];

    //carry out 
    always_comb begin
        if (fop == FOP_ADD || fop == FOP_SUB) begin
            if ((fop == FOP_ADD) && rda[31] && rdb[31]) begin
                C = 1'b1;
            end if ((fop == FOP_SUB) && rda[31] && !rda[31]) begin
                C = 1'b1;
            end else
                C = '0;
            
        end
        else C = '0;
    end

    //overflow
    always_comb begin
        if (fop == FOP_ADD) begin
            if (rda[31] && rdb[31] && !result[31])
                V = 1'b1;
            if (!rda[31] && !rdb[31] && result[31])
                V = 1'b1;
            else
                V = '0;
        end
            else if (fop == FOP_SUB) begin
            if ((rda[31] && !rdb[31] && !result[31]) || (!rda[31] && rdb[31] && result[31]))
                V = 1'b1;
            else
                V = '0;
        end
        else V = '0;
    end
endmodule

module tb;
    logic [31:0] tb_rda, tb_rdb, tb_result;
    logic [3:0] tb_fop;
    logic tb_Z, tb_N, tb_V, tb_C;

    ALU tb_alu (.rda(tb_rda), .rdb(tb_rdb), .fop(tb_fop), .result(tb_result), .Z(tb_Z), .N(tb_N), .V(tb_V), .C(tb_C));

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
    logic exp_Z;

    task check_Z;
    total_Z_tests++;
    if (exp_Z != tb_Z)
        $display("Error: tb_Z = %1d, expected %1d", tb_Z, exp_Z);
    else
        passed_Z_tests++;
    endtask

    logic total_N_tests = 0;
    logic passed_N_tests = 0;
    logic exp_N;

    task check_N;
    total_N_tests++;
    if (exp_N != tb_N)
        $display("Error: tb_N = %1d, expected %1d", tb_N, exp_N);
    else
        passed_N_tests++;
    endtask

    logic total_V_tests = 0;
    logic passed_V_tests = 0;
    logic exp_V;

    task check_V;
    total_V_tests++;
    if (exp_V != tb_V)
        $display("Error: tb_V = %1d, expected %1d", tb_V, exp_V);
    else
        passed_V_tests++;
    endtask

    logic total_C_tests = 0;
    logic passed_C_tests = 0;
    logic exp_C;

    task check_C;
    total_C_tests++;
    if (exp_C != tb_C)
        $display("Error: tb_C = %1d, expected %1d", tb_C, exp_C);
    else
        passed_C_tests++;
    endtask

    task ADD;
        
    //14+2=16
        tb_rda = 32'd14;
        tb_rdb = 32'd2;
        tb_fop = FOP_ADD;
        #1;
        exp_result = 32'd16;
        exp_Z = 0;
        exp_N = 0;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
        
    //4294967296 + 1 = 0 (carry out should be on)
        tb_rda = 32'b011111111111111111111111111111111;
        tb_rdb = 32'b1;
        tb_fop = FOP_ADD;
        #1;
        exp_result = 32'd0;
        exp_Z = 0;
        exp_N = 0;
        exp_V = 0;
        exp_C = 1;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task SUB;
        tb_rda = 32'd68;
        tb_rdb = 32'd29;
        tb_fop = FOP_SUB;
        #1;
        exp_result = 32'd39;
        exp_Z = 0;
        exp_N = 0;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task SLL;
        tb_rda = 32'd7;
        tb_rdb = 32'd2;
        tb_fop = FOP_SLL;
        #1;
        exp_result = 32'd28;
        exp_Z = 0;
        exp_N = 0;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task SRL;
        tb_rda = 32'd470;
        tb_rdb = 32'd1;
        tb_fop = FOP_SRL;
        #1;
        exp_result = 235;
        exp_Z = 0;
        exp_N = 0;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task SRA;
        tb_rda = 32'b10000000000000000000000000000001;
        tb_rdb = 5;
        tb_fop = FOP_SRA;
        #1;
        exp_result = 32'b11111100000000000000000000000000;
        exp_Z = 0;
        exp_N = 1;
        exp_V = 0;                                                              //WRONG
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task AND;
        tb_rda = 32'b10101000010111101010101011100001;
        tb_rdb = 32'b10101000010111101010101011100001;
        tb_fop = FOP_AND;
        #1;
        exp_result = 32'b10101000010111101010101011100001;
        exp_Z = 0;
        exp_N = 1;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task OR;
        tb_rda = 32'b11010011010100101010101011010001;
        tb_rdb = 32'b0;
        tb_fop = FOP_OR;
        #1;
        exp_result = 32'b11010011010100101010101011010001;
        exp_Z = 0;
        exp_N = 1;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task XOR;
        tb_rda = 32'b010001010101101111011100011100000;
        tb_rdb = 32'b010001010101101111011100011100000;
        tb_fop = FOP_XOR;
        #1;
        exp_result = 32'b0;
        exp_Z = 1;
        exp_N = 0;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task IMM;
        tb_rda = 32'b10101010101010010000011100010001;
        tb_rdb = 32'd981247;
        tb_fop = FOP_IMM;
        #1;
        exp_result = 32'd981247;
        exp_Z = 0;
        exp_N = 0;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task Z;
        tb_rda = 32'b10010101010100000110101011010100;
        tb_rdb = 32'b10010101010100000110101011010100;
        tb_fop = FOP_SUB;
        #1;
        exp_result = 32'b0;
        exp_Z = 1'b1;
        exp_N = 0;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    task N;
        tb_rda = 32'd762;
        tb_rdb = 32'd1000;
        tb_fop = FOP_SUB;
        #1;
        exp_result = -238;
        exp_Z = 0;
        exp_N = 1'b1;
        exp_V = 0;
        exp_C = 0;
        #1;
        check_result ();
        check_Z ();
        check_N ();
        check_V ();
        check_C ();
        #1;
    endtask

    // task C;
    //     tb_rda = ;
    //     tb_rdb = ;
    //     tb_fop = FOP_;
    //     #1;
    //     exp_result = ;
    //     exp_Z = ;
    //     exp_N = ;
    //     exp_V = ;
    //     exp_C = ;
    //     #1;
    //     check_result ();
    //     check_Z ();
    //     check_N ();
    //     check_V ();
    //     check_C ();
    //     #1;
    // endtask

    // task V;
    //     tb_rda = ;
    //     tb_rdb = ;
    //     tb_fop = FOP_;
    //     #1;
    //     exp_result = ;
    //     exp_Z = ;
    //     exp_N = ;
    //     exp_V = ;
    //     exp_C = ;
    //     #1;
    //     check_result ();
    //     check_Z ();
    //     check_N ();
    //     check_V ();
    //     check_C ();
    //     #1;
    // endtask


initial begin
    $dumpfile("sim.vcd");
    $dumpvars(0, tb);

    ADD;
    #1;
    SUB;
    #1;
    SLL;
    #1;
    SRL;
    #1;
    SRA;
    #1;
    AND;
    #1;
    OR;
    #1;
    XOR;
    #1;
    IMM;
    #1;
    Z;
    #1;
    N;
    #1;
    // C;
    // #1;
    // V;
    // #1;
    
    $finish;
end

endmodule