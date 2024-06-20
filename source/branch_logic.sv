module branch_logic(
    input logic [3:0] branch_type,
    input logic ALU_neg_flag, ALU_zero_flag,
    output logic b_out
);

always_comb begin
    if ((branch_type == 3'd1)&&(ALU_zero_flag)) begin
        b_out = 1'b1;

    end

    else if ((branch_type == 3'd2) && (!ALU_zero_flag)) begin
        b_out = 1'b1

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
