module byte_demux (
input logic [31:0] reg_b,
input logic store_byte_en,
output logic [31:0] reg_b_out, b_out
);

    always_comb begin
        if (store_byte_en) begin
            b_out = reg_b;
            reg_b_out = '0;
        end else begin
            reg_b_out = reg_b;
            b_out = '0;
        end
    end
endmodule

module byte_imm_gen (
input logic [31:0] b_out,
output logic [31:0] imm_gen_byte
);

assign imm_gen_byte = {24'd0, b_out[7:0]};

endmodule