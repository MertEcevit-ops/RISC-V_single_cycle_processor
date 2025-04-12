# RISC-V Single Cycle Processor

## Overview

The **RISC-V Single Cycle Processor** project implements a simple processor based on the RISC-V instruction set architecture (ISA), where each instruction is executed in a single clock cycle. The design focuses on a minimalistic and efficient architecture, suitable for understanding the fundamentals of processor design and RISC-V architecture.

## Key Features

- **RISC-V Architecture**: Implements the basic RISC-V instructions (e.g., arithmetic, logic, load, store, and control).
- **Single-Cycle Design**: Each instruction is executed in a single clock cycle, simplifying the pipeline and control logic.
- **Basic Components**:
  - **ALU (Arithmetic Logic Unit)**: Performs arithmetic and logical operations.
  - **Registers**: A small set of general-purpose registers (e.g., 32 registers).
  - **Control Unit**: Decodes instructions and generates control signals for the processor.
  - **Memory**: Basic data memory (RAM) and instruction memory.
  - **Instruction Fetch-Decode-Execute**: A straightforward cycle where instructions are fetched, decoded, and executed in one cycle.

## Components

- **CPU Core**: Handles instruction decoding, execution, and result storing.
- **ALU**: Performs basic operations like addition, subtraction, and bitwise operations.
- **Register File**: Stores the 32 general-purpose registers for data manipulation.
- **Control Unit**: Controls the operation flow by issuing control signals for various components.
- **Memory**: Separate instruction and data memories for storing instructions and data values.

## Instructions Supported

- **Arithmetic Operations**: ADD, SUB, AND, OR, etc.
- **Logic Operations**: AND, OR, NOT.
- **Load/Store**: LW, SW (Load Word, Store Word).
- **Branching**: BEQ (Branch Equal), BNE (Branch Not Equal).
- **Control Instructions**: JAL (Jump and Link), JALR (Jump and Link Register).
- **Compressed Instructions** : CLZ, CTZ, CPOP
