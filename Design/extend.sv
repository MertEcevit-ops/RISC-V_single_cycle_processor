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