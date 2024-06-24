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
    input logic signed [31:0] rda, rdb,
    input logic [3:0] fop,
    output logic signed [31:0] result,
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
            if (exp_C != tb_C) begin
                $display("Error: tb_C = %1d, expected %1d", tb_C, exp_C);
            end
            else 
                passed_flag_tests++;
    endtask

    task ADD; //error with overflow on random checks
        
        // Basic functional tests
        tb_rda = 10;
        tb_rdb = 5;
        tb_fop = FOP_ADD;
        exp_result = tb_rda + tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && !tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;
    
        // Negative test
        tb_rda = -10;
        tb_rdb = 5;
        tb_fop = FOP_ADD;
        exp_result = tb_rda + tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && !tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Overflow test
        tb_rda = 2147483647; // max positive number
        tb_rdb = 1;
        tb_fop = FOP_ADD;
        exp_result = tb_rda + tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && !tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Zero test
        tb_rda = 4294967295; // max number 
        tb_rdb = 1;
        tb_fop = FOP_ADD;
        exp_result = tb_rda + tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && !tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // // Carry test
        // tb_rda = 2147483647;
        // tb_rdb = 2147483647;
        // tb_fop = FOP_ADD;
        // exp_result = tb_rda + tb_rdb;
        // exp_Z = 0;
        // exp_N = 0;
        // exp_V = 0;
        // exp_C = 0;
        // #1;
        // check_result();
        // #1;
        // check_flag();
        // #1;

        // Random tests
        repeat (10) begin
            tb_rda = $random;
            tb_rdb = $random;
            tb_fop = FOP_ADD;
            exp_result = tb_rda + tb_rdb;
            exp_Z = (exp_result == 0) ? 1 : 0;
            exp_N = exp_result[31];
            exp_V = (tb_rda[31] && tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && !tb_rdb[31] && exp_result[31]);
            exp_C = (tb_rda[31] && tb_rdb[31]);
            #1;
            check_result();
            #1;
            check_flag();
            #1;
        end
    endtask

    task SUB;
        
        // Basic functional tests
        tb_rda = 10;
        tb_rdb = 5;
        tb_fop = FOP_SUB;
        exp_result = tb_rda - tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && !tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && !tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Negative test
        tb_rda = -10;
        tb_rdb = 5;
        tb_fop = FOP_SUB;
        exp_result = tb_rda - tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && !tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && !tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Overflow test
        tb_rda = -1234567890;
        tb_rdb = 2134567890;
        tb_fop = FOP_SUB;
        exp_result = tb_rda - tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && !tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && !tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Zero test
        tb_rda = 123456789;
        tb_rdb = 123456789;
        tb_fop = FOP_SUB;
        exp_result = tb_rda - tb_rdb;
        exp_Z = (exp_result == 0) ? 1 : 0;
        exp_N = exp_result[31];
        exp_V = (tb_rda[31] && !tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && tb_rdb[31] && exp_result[31]);
        exp_C = (tb_rda[31] && !tb_rdb[31]);
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // // Carry test
        // tb_rda = 3696709287;
        // tb_rdb = 505290208;
        // tb_fop = FOP_SUB;
        // exp_result = tb_rda - tb_rdb;
        // exp_Z = 0;
        // exp_N = 0;
        // exp_V = 0;
        // exp_C = 1;
        // #1;
        // check_result();
        // #1;
        // check_flag();
        // #1;

        // Random tests
        repeat (10) begin
            tb_rda = $random;
            tb_rdb = $random;
            tb_fop = FOP_SUB;
            exp_result = tb_rda - tb_rdb;
            exp_Z = (exp_result == 0) ? 1 : 0;
            exp_N = exp_result[31];
            exp_V = (tb_rda[31] && !tb_rdb[31] && !exp_result[31]) || (!tb_rda[31] && tb_rdb[31] && exp_result[31]);
            exp_C = (tb_rda[31] && !tb_rdb[31]);
            #1;
            check_result();
            #1;
            check_flag();
            #1;

        end
    endtask

    task SHIFT_LEFT_LOGICAL;
        
        // Basic functional test
        tb_rda = 32'h0000_0001;
        tb_rdb = 1;
        tb_fop = FOP_SLL;
        exp_result = tb_rda << tb_rdb;
        exp_Z = 0;
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Shift left by 0
        tb_rda = 32'h8000_0000; // Large negative number
        tb_rdb = 0;
        tb_fop = FOP_SLL;
        exp_result = tb_rda << tb_rdb;
        exp_Z = 0;
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Edge case: Shift left by 31
        tb_rda = 32'h0000_0001; // 1
        tb_rdb = 31;            // Shift left by 31
        tb_fop = FOP_SLL;
        exp_result = tb_rda << tb_rdb;
        exp_Z = 0;
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Shift resulting in zero
        tb_rda = 32'h0000_0001; // 1
        tb_rdb = 32;            // Shift left by 32
        tb_fop = FOP_SLL;
        exp_result = 0;
        exp_Z = 1;
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Random tests
        repeat (10) begin
            tb_rda = $random;
            tb_rdb = $random % 32; // Limiting shift amount to 31
            tb_fop = FOP_SLL;
            exp_result = tb_rda << tb_rdb;
            exp_Z = (exp_result == 0) ? 1 : 0;
            exp_N = exp_result[31];
            #1;
            check_result();
            #1;
            check_flag();
            #1;
        end
    endtask

    task SHIFT_RIGHT_LOGICAL;
        
        // Basic functional test
        tb_rda = 32'h0000_0010;
        tb_rdb = 1;
        tb_fop = FOP_SRL;
        exp_result = tb_rda >> tb_rdb;
        exp_Z = 0;
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Shift right by 0
        tb_rda = 32'h8000_0000; // Large negative number
        tb_rdb = 0;
        tb_fop = FOP_SRL;
        exp_result = tb_rda >> tb_rdb;
        exp_Z = 0;
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Edge case: Shift right by 31
        tb_rda = 32'h0000_0001; // 1
        tb_rdb = 31;            // Shift right by 31
        tb_fop = FOP_SRL;
        exp_result = tb_rda >> tb_rdb;
        exp_Z = 1;
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Shift resulting in zero
        tb_rda = 32'h8000_0000; // Large negative number
        tb_rdb = 32;            // Shift right by 32
        tb_fop = FOP_SRL;
        exp_result = 0;
        exp_Z = 1;
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Random tests
        repeat (10) begin
            tb_rda = $random;
            tb_rdb = $random % 32; // Limiting shift amount to 31
            tb_fop = FOP_SRL;
            exp_result = tb_rda >> tb_rdb;
            exp_Z = (exp_result == 0) ? 1 : 0;
            exp_N = exp_result[31];
            #1;
            check_result();
            #1;
            check_flag();
            #1;
        end
    endtask

    task SHIFT_RIGHT_ARITHMETIC; //needs fixing, >>> is not the correct syntax for shift right arithmetic
    
        // Basic functional test
        tb_rda = 32'h0000_0010;
        tb_rdb = 1;
        tb_fop = FOP_SRA;
        exp_result = tb_rda >>> tb_rdb;
        exp_Z = 0;
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Shift right by 0
        tb_rda = 32'h8000_0000; // Large negative number
        tb_rdb = 0;
        tb_fop = FOP_SRA;
        exp_result = 32'h8000_0000;
        exp_Z = 0;
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Edge case: Shift right by 31
        tb_rda = 32'h0000_0001; // 1
        tb_rdb = 31;            // Shift right by 31
        tb_fop = FOP_SRA;
        exp_result = 0;
        exp_Z = 1;
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Shift resulting in negative number
        tb_rda = 32'h8000_0000; // Large negative number
        tb_rdb = 32;            // Shift right by 32
        tb_fop = FOP_SRA;
        exp_result = 0;
        exp_Z = 0;
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Random tests
        repeat (10) begin
            tb_rda = $random;
            tb_rdb = $random % 32; // Limiting shift amount to 31
            tb_fop = FOP_SRA;
            exp_result = tb_rda >>> tb_rdb;
            exp_Z = (exp_result == 0) ? 1 : 0;
            exp_N = exp_result[31];
            #1;
            check_result();
            #1;
            check_flag();
            #1;
        end
    endtask

    task AND;

        // Basic low signal test #1
        tb_rda = 32'h0234_5678;
        tb_rdb = 1;
        tb_fop = FOP_AND;
        exp_result = 0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic low signal test #2
        tb_rda = 1;
        tb_rdb = 32'h0234_5678;
        tb_fop = FOP_AND;
        exp_result = 0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic high signal test #1
        tb_rda = 32'h0234_5678;
        tb_rdb = 32'h0234_5678;
        tb_fop = FOP_AND;
        exp_result = 32'h0234_5678;
        exp_Z = 0; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic high signal test #2
        tb_rda = 32'h8765_4321;
        tb_rdb = 32'h8765_4321;
        tb_fop = FOP_AND;
        exp_result = 32'h8765_4321;
        exp_Z = 0; 
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Zero zero zero test
        tb_rda = 0;
        tb_rdb = 0;
        tb_fop = FOP_AND;
        exp_result = 0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;
    endtask

    task OR;

        // Basic test #1
        tb_rda = 32'b111010101010101000001011111110;
        tb_rdb = 0;
        tb_fop = FOP_OR;
        exp_result = 32'b111010101010101000001011111110;
        exp_Z = 0; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic test #2
        tb_rda = 0;
        tb_rdb = 32'b111010101010101000001011111110;
        tb_fop = FOP_OR;
        exp_result = 32'b111010101010101000001011111110;
        exp_Z = 0; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic test #3 (negative)
        tb_rda = 32'b11111111111111111111110000000000;
        tb_rdb = 32'b00000000000000000000001111111111;
        tb_fop = FOP_OR;
        exp_result = 32'b11111111111111111111111111111111;
        exp_Z = 0; 
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic test #4 (negative)
        tb_rda = 32'b10101010101010101010101010101010;
        tb_rdb = 32'b1010101010101010101010101010101;
        tb_fop = FOP_OR;
        exp_result = 32'b11111111111111111111111111111111;
        exp_Z = 0; 
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Zero zero zero test
        tb_rda = 0;
        tb_rdb = 0;
        tb_fop = FOP_OR;
        exp_result = 0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;
    endtask

    task XOR;

        // Basic test #1             same as or test
        tb_rda = 32'b111010101010101000001011111110;
        tb_rdb = 0;
        tb_fop = FOP_XOR;
        exp_result = 32'b111010101010101000001011111110;
        exp_Z = 0; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic test #2             same as or test
        tb_rda = 0;
        tb_rdb = 32'b111010101010101000001011111110;
        tb_fop = FOP_XOR;
        exp_result = 32'b111010101010101000001011111110;
        exp_Z = 0; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic test #3 (negative)             same as or test
        tb_rda = 32'b11111111111111111111110000000000;
        tb_rdb = 32'b00000000000000000000001111111111;
        tb_fop = FOP_XOR;
        exp_result = 32'b11111111111111111111111111111111;
        exp_Z = 0; 
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Basic test #4 (negative)             same as or test
        tb_rda = 32'b10101010101010101010101010101010;
        tb_rdb = 32'b1010101010101010101010101010101;
        tb_fop = FOP_XOR;
        exp_result = 32'b11111111111111111111111111111111;
        exp_Z = 0; 
        exp_N = 1;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Zero zero zero test
        tb_rda = 0;
        tb_rdb = 0;
        tb_fop = FOP_XOR;
        exp_result = 0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        // Xor property test
        tb_rda = 32'b10101010101010101010101010101010;
        tb_rdb = 32'b10101010101010101010101010101010;
        tb_fop = FOP_XOR;
        exp_result = 32'b0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;
    endtask

    task IMMEDIATE;
        
        tb_rda = $random;
        tb_rdb = 0;
        tb_fop = FOP_IMM;
        exp_result = 0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        tb_rda = $random;
        tb_rdb = 12345678;
        tb_fop = FOP_IMM;
        exp_result = 12345678;
        exp_Z = 0; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        tb_rda = 0;
        tb_rdb = 80085;
        tb_fop = FOP_IMM;
        exp_result = 80085;
        exp_Z = 0; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        tb_rda = $random;
        tb_rdb = 0;
        tb_fop = FOP_IMM;
        exp_result = 0;
        exp_Z = 1; 
        exp_N = 0;
        #1;
        check_result();
        #1;
        check_flag();
        #1;

        repeat (10) begin
            tb_rda = $random;
            tb_rdb = $random;
            tb_fop = FOP_IMM;
            exp_result = tb_rdb;
            exp_Z = (exp_result == 0) ? 1 : 0;
            exp_N = exp_result[31];
            #1;
            check_result();
            #1;
            check_flag();
            #1;
        end
    endtask     

    initial begin
        $dumpfile("sim.vcd");
        $dumpvars(0, tb);

        tb_rda = 0;
        tb_rdb = 0;
        tb_fop = 0;
        #1;

        ADD;
        #1;
        
        SUB;
        #1;
        
        SHIFT_LEFT_LOGICAL;
        #1;
        
        SHIFT_RIGHT_LOGICAL;
        #1;
        
        SHIFT_RIGHT_ARITHMETIC;
        #1;

        AND;
        #1;

        OR;
        #1;

        XOR;
        #1;

        IMMEDIATE;
        #1;

        $finish;
    end
endmodule