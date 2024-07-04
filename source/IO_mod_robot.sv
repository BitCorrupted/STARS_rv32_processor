module IO_mod_robot(
     input logic clk, rst,
    input logic write_mem, read_mem,
    input logic [31:0] data_from_mem,
    input logic [31:0] data_address, data_to_write,
    output logic [31:0] data_read,
    output logic [31:0] IO_out, IO_pwm,
    input logic [31:0] IO_in
);
 logic [31:0] output_reg, input_reg, pwm_reg;
 logic [31:0] next_output_reg, next_input_reg, next_pwm_reg;


 always_comb begin

    

    next_output_reg = output_reg;
    next_input_reg = IO_in;
    next_pwm_reg = pwm_reg;

    if (write_mem) begin
        case(data_address)
            32'hFFFFFFFF: begin //GPIO output register
                next_output_reg = data_to_write; 
            end
            32'hFFFFFFFD: begin //PWM register
                next_pwm_reg = data_to_write;
            end
            default: begin //Other addresses
                next_output_reg = output_reg;
                next_pwm_reg = pwm_reg;
            end
        endcase
        data_read = data_from_mem;
    end

    else if(read_mem) begin
        case(data_address)
            32'hFFFFFFFC: begin //GPIO input register(?)
                data_read = input_reg;
            end
            default: begin
                data_read = data_from_mem;
                next_input_reg = IO_in;
            end
        endcase
    end

    else begin
        next_output_reg = output_reg;
        next_input_reg = IO_in;
        next_pwm_reg = pwm_reg;
        data_read = data_from_mem;
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
        pwm_reg <= next_pwm_reg;

    end

end

endmodule