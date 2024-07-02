module IO_mod_robot(
     input logic clk, rst,
    input logic write_mem, read_mem,
    input logic [31:0] data_from_mem,
    input logic [31:0] data_address, data_to_write,
    output logic [31:0] data_read,
    output logic [31:0] IO_out, IO_pwm
    input logic [31:0] IO_in
);
 logic [31:0] output_reg, input_reg, pwm_reg;
 logic [31:0] next_output_reg, next_input_reg, next_pwm_reg;


 always_comb begin

    assign input_reg = IO_in;

    next_output_reg = output_reg;
    next_input_reg = input_reg;
    next_pwm_reg = pwm_reg;

    if (write_mem) begin
        if (data_address == 32'hFFFFFFFF) begin
            next_output_reg = data_to_write; 

        end

        else if(data_address == 32'hFFFFFFFD) begin 
            next_pwm_reg = data_to_write;

        end

        else begin
            next_output_reg = output_reg;
            next_pwm_reg = pwm_reg;
        end

    end

    else if(read_mem) begin
        if (data_address == 32'hFFFFFFFC) begin
            data_read = input_reg;

        end
        
        else begin
            data_read = data_from_mem;
            next_input_reg = input_reg; 

        end
    end

    else begin
        next_output_reg = output_reg;
        next_input_reg = input_reg;
        next_pwm_reg = pwm_reg;


    end

 end

 assign IO_out = output_reg;
 assign IO_pwm = pwm_reg;

always_ff @(posedge clk, posedge rst) begin
    if (rst) begin
        output_reg <= '0;
        input_reg <= '0;
        pwm_reg <= '0;
    end

    else begin
        output_reg <= next_output_reg;
        input_reg <= next_input_reg;
        pwm_reg <= next_pwm_reg

    end

end

endmodule