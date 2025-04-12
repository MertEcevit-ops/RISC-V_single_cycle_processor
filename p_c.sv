module p_c(
	input 	logic [31:0] pc_next
	input 	logic clk, rst;
	
	output 	logic[31:0] pc);
	
 always@(posedge clk)
	begin
	
	if (rst == 1'b0)
	 begin
	 pc <= 32'h00000000;
	 end
	else
	 begin
		pc <= pc_next;
	end
 end
endmodule
