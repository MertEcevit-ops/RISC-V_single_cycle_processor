'include "alu_decoder.sv"
'include "main_decoder.sv"


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