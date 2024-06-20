/*
write_i = enabling the write port for the bus
read_i = enabling the read port for the bus
adr_i = addres we are trying to access from memory
cpu_dat_i = the data we are trying to write into memory based on the address of the instruction
sel_i = just permanently set to 15 since we are always trying to get 32 bits of data
cpu_dat_o = data that was read from memory (could be instruction fetch or could be the value loaded from the memory in sram)
busy_o = the bus is busy
*/


always_ff @(posedge clk, negedge rst) begin
    if(!rst) begin
        read_i <= 1'b0;
        write_i <= 1'b0;
        adr_i <= '0;
        cpu_dat_i <= '0;
        sel_i <= '0;
        instruction <= '0;
    end else begin
        read_i <= next_read;
        write_i <= next_write;
        adr_i <= next_adr;
        cpu_dat_i <= next_cpu_dat;
        sel_i <= 4'd15;
        instruction <= next_instruction;
    end
end


    always_comb begin
        // if we are busy, we want to retain our current values below
        next_adr = adr_i;
        next_read = read_i;
        next_write = write_i;
        next_cpu_dat = cpu_dat_i;
        next_instruction = instruction;
        if(memread = 1'b1) begin
            next_read = 1'b1; // indicating that we want to read data from the memory in sram
            if(!busy_o) begin // if it is not busy, we can start indicating our next address
                next_adr = data_address;
                reg_d = cpu_dat_o; // write back
            end 
        end else if (memwrite = 1'b1) begin
            next_write = 1'b1;
            if(!busy_o) begin
                next_adr = data_address; 
                next_cpu_dat = result; // alu result
            end
        end else begin
            next_read = 1'b1;
            if(!busy_o)  begin
                next_adr = instruction_address; 
                next_instruction = cpu_dat_o; 
            end
    end
end