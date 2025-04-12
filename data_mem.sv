module data_mem(
	input 	logic [31:0] a, wd;
	input 	logic clk, rst, we;
	output	logic [31:0] rd);
	
	reg [31:0] data_mem [1023:0];
	
	assign rd = (we == 1'b0) ? data_mem[a] : 32'h00000000;
	
	always@(posedge clk) begin
	if (we)
		begin
		data_mem <= wd;
	 end
	end
endmodule