module alu(;
//Declaring inputs
	input 	logic	[31:0]	a,b;
	input 	logic	[3:0]	alu_control;
	output 	logic	[31:0	]alu_result;
	output	logic	zero);
	
  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation
  logic	[31:0] clz_result;
  logic [31:0] ctz_result;
  logic [31:0] cpop_result;

  assign condinvb = alu_control[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alu_control[2] & ~alu_control[1] |
                    ~alu_control[1] & alu_control[0];
					
  always_comb begin
    clz_result = 32;  // clz loop
    for (int i = 31; i >= 0; i--) begin
      if (a[i]) begin
        clz_result = 31 - i;
        break;
      end
    end
  end
  
  always_comb begin
    ctz_result = 32;  // Tüm bitler 0 ise sonuç 32 olacak
    for (int i = 0; i < 32; i++) begin
      if (a[i]) begin
        ctz_result = i;
        break;
      end
    end
  end

  always_comb begin
    cpop_result = '0;
    for (int i = 0; i < 32; i++) begin
      if (a[i]) 
        cpop_result = cpop_result + 1;
    end
  end

  always_comb
    case (alu_control)
      4'b0000:  alu_result = sum;         // add
      4'b0001:  alu_result = sum;         // subtract
      4'b0010:  alu_result = a & b;       // and
      4'b0011:  alu_result = a | b;       // or
      4'b0100:  alu_result = a ^ b;       // xor
      4'b0101:  alu_result = sum[31] ^ v; // slt
      4'b0110:  alu_result = a << b[4:0]; // sll
      4'b0111:  alu_result = a >> b[4:0]; // srl
	  4'b1000:  alu_result = clz_result;  // clz
      4'b1001:  alu_result = ctz_result;  // ctz
      4'b1010:  alu_result = cpop_result; // cpop
      default:  alu_result = 32'bx;
    endcase

  assign zero = (alu_result == 32'b0);
  assign v = ~(alu_control[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule
	
endmodule