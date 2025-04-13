module riscv_single
  import riscv_pkg::*;
(
    parameter DMemInitFile  = “dmem.mem”;       // data memory initialization file
    parameter IMemInitFile  = “imem.mem”;       // instruction memory initialization file
)   (
    input  logic             clk_i,       // system clock
    input  logic             rstn_i,      // system reset
    input  logic  [XLEN-1:0] addr_i,      // memory adddres input for reading
    output logic  [XLEN-1:0] data_o,      // memory data output for reading
    output logic             update_o,    // retire signal
    output logic  [XLEN-1:0] pc_o,        // retired program counter
    output logic  [XLEN-1:0] instr_o,     // retired instruction
    output logic  [     4:0] reg_addr_o,  // retired register address
    output logic  [XLEN-1:0] reg_data_o,  // retired register data
    output logic  [XLEN-1:0] mem_addr_o,  // retired memory address
    output logic  [XLEN-1:0] mem_data_o,  // retired memory data
    output logic             mem_wrt_o   // retired memory write enable signal
);
  // module body
  // use other modules according to the need.

endmodule



module control_unit(input  logic [6:0] op,
					input  logic [2:0] funct3,
					input  logic       funct7b5,
					input  logic       zero,
					output logic [1:0] result_src,
					output logic       mem_write,
					output logic       pc_src, alu_src,
					output logic       reg_write, jump,
					output logic [1:0] imm_src,
					output logic [2:0] alu_control);

  logic [1:0] alu_op;
  logic       branch;

  main_decoder md(op, result_src, mem_write, branch,
             alu_src, reg_write, Jump, imm_src, alu_op);
  alu_decoder  ad(op[5], funct3, funct7b5, alu_op, alu_control);

  assign pc_rc = branch & zero | jump;
endmodule

module main_decoder	(input  logic [6:0] op,
					output logic [1:0] result_src,
					output logic       mem_write,
					output logic       branch, alu_src,
					output logic       reg_write, jump,
					output logic [1:0] imm_src,
					output logic [1:0] alu_op);

  logic [10:0] controls;

  assign {reg_write, imm_src, alu_src, mem_write,
          result_src, branch, alu_op, jump} = controls;

  always_comb
    case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module alu_decoder(input  logic  opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] alu_op,
              output logic [3:0] alu_control);

  logic  r_type_sub;
  assign r_type_sub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(alu_op)
      2'b00:                alu_control = 4'b0000; // addition
      2'b01:                alu_control = 4'b0001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 4'b0000:  if (r_type_sub) 
                            alu_control = 4'b0001; // sub
                          else          
                            alu_control = 4'b0000; // add, addi
                 4'b0010:    alu_control = 4'b0101; // slt, slti
                 4'b0110:    alu_control = 4'b0011; // or, ori
                 4'b0111:    alu_control = 4'b0010; // and, andi
				 4'b1000:	 alu_control = 4'b1000; // clz
				 4'b1001:    alu_control = 4'b1001; // ctz
				 4'b1010:    alu_control = 4'b1010; // cpop
                 default:   alu_control = 4'bxxx; // ???
               endcase
    endcase
endmodule

module datapath(
    input  logic        clk, reset,
    input  logic [1:0]  result_src, 
    input  logic        pc_src, alu_src,
    input  logic        reg_write,
    input  logic [1:0]  imm_src,
    input  logic [2:0]  alu_control,
    output logic        zero,
    output logic [31:0] pc,
    input  logic [31:0] instr,
    output logic [31:0] alu_result, write_data,
    input  logic [31:0] read_data
);

  logic [31:0] pc_next, pc_plus_4, pc_target;
  logic [31:0] imm_ext;
  logic [31:0] src_a, src_b;
  logic [31:0] result;

  // next pc logic
  flopr #(32) pc_reg(clk, reset, pc_next, pc); 
  adder       pc_add_4(pc, 32'd4, pc_plus_4);
  adder       pc_add_branch(pc, imm_ext, pc_target);
  mux2 #(32)  pc_mux(pc_plus_4, pc_target, pc_src, pc_next);
 
  // register file logic
  regfile     reg_file(clk, reg_write, instr[19:15], instr[24:20], 
                       instr[11:7], result, src_a, write_data);
  extend      extender(instr[31:7], imm_src, imm_ext);

  // alu logic
  mux2 #(32)  src_b_mux(write_data, imm_ext, alu_src, src_b);
  alu         alu_unit(src_a, src_b, alu_control, alu_result, zero);
  mux3 #(32)  result_mux(alu_result, read_data, pc_plus_4, result_src, result);

endmodule

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


module reg_files(input  logic        clk, 
				 input  logic        we3, 
				 input  logic [ 4:0] a1, a2, a3, 
				 input  logic [31:0] wd3, 
				 output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  imm_src,
              output logic [31:0] imm_ext);
 
  always_comb
    case(imm_src) 
               // I-type 
      2'b00:   imm_ext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   imm_ext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   imm_ext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   imm_ext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
      default: imm_ext = 32'bx; // undefined
    endcase             
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, rst,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module instr_memory(
	input  logic [31:0] a,
    output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule


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