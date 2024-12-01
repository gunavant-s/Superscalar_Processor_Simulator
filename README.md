# Dynamic Instruction Scheduling Simulator

A cycle-accurate simulator for an out-of-order superscalar processor that models dynamic instruction scheduling with perfect caches and branch prediction[1].

## Features

- Models a complete out-of-order pipeline including:
  - Fetch, Decode, Rename, Register Read, Dispatch, Issue, Execute, Writeback and Retire stages
  - Reorder Buffer (ROB) for in-order retirement
  - Issue Queue (IQ) for out-of-order execution
  - Register renaming with Rename Map Table (RMT)
  - Universal pipelined Function Units

## Architecture Details

- 67 architectural registers (r0-r66)[1]
- Three instruction types with different execution latencies:
  - Type 0: 1 cycle
  - Type 1: 2 cycles  
  - Type 2: 5 cycles

## Usage

```bash
./sim <ROB_SIZE> <IQ_SIZE> <WIDTH> <tracefile>
```

## Input Format

The simulator accepts trace files in the following format:
```
<PC> <operation_type> <dest_reg> <src1_reg> <src2_reg>
```

Where:
- PC: Program counter (hex)
- operation_type: 0, 1, or 2 
- dest_reg: Destination register (-1 if none)
- src1_reg: First source register (-1 if none)
- src2_reg: Second source register (-1 if none)

## Output Format

The simulator outputs timing information for each instruction in the format[1]:
```
<seq_no> fu{<op_type>} src{<src1>,<src2>} dst{<dst>} FE{<begin-cycle>,<duration>} DE{...} RN{...} RR{...} DI{...} IS{...} EX{...} WB{...} RT{...}
```

## Implementation Details

- Written in C/C++
- Optimized with -O3 compiler flag
- Models data dependencies and structural hazards
- Perfect caches and branch prediction assumed.

## Building

```bash
make
```

## Requirements

- C/C++ compiler
- Make build system
