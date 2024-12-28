# Tec-TACIT
A software design philosophy where functions operate without explicitly naming their arguments


# TACIT Programming Language - Core Language Reference

## Language Architecture

### Core Design
- Stack-based operation model
- Zero-operand (tacit) programming
- Immediate execution model
- Z80 assembly implementation

### Memory Architecture

#### Stack Structure
```
Data Stack (DSTACK):
- Location: 0x0A00
- Size: 128 bytes (DSIZE)
- Operations: Push, Pop, Peek
- Stack pointer in SP register

Return Stack (RSTACK):
- Location: 0x0900
- Size: 128 bytes (RSIZE)
- Used for: Subroutine calls, loop control
- Managed through IX register
```

#### Memory Map
```
ROM (TEC-1):
0x0000-0x07FF: System ROM
  0x0000: Reset vector
  0x0008-0x0038: RST vectors
  0x0040: Seven-segment lookup
  0x0180: TACIT core code

RAM:
0x0800: Text Input Buffer (256 bytes)
0x0900: Return Stack (128 bytes)
0x0980: RSTACK Top
0x0A00: Data Stack (128 bytes)
0x0B00: Opcode Tables
0x0C00: Variables and Definitions
0x0CA0: Heap Start
```

### System Variables

#### Core System Variables (0x0A00)
```
TBPTR:    Text buffer pointer
VTEMP1:   Temporary variable 1
VTEMP2:   Temporary variable 2
RST08-30: Interrupt vectors
BAUD:     Baud rate storage
INTVEC:   Interrupt vector
NMIVEC:   Non-maskable interrupt
GETCVEC:  Character input vector
PUTCVEC:  Character output vector
```

#### User Variables (0x0C00)
Special purpose assignments:
```
h: Heap pointer
k: Text input buffer pointer
r: Division remainder
s: Stack start address
v: Interrupt ID
z: Last defined function
```

### Hardware Interface

#### I/O Ports (TEC-1)
```
00H (KEYBUF): MM74C923N Keyboard Encoder
01H (SCAN):   Display Scan Latch
02H (DISPLY): Display Latch
03H (PORT3):  ST3/Strobe
04H (PORT4):  ST4/LCD
05H-07H:      Auxiliary Ports
```

#### Serial Interface (6850 ACIA)
Registers:
```
Control (0x80): Configuration
Status (0x80):  Status flags
TDR (0x81):    Transmit Data
RDR (0x81):    Receive Data
```

### Interrupt System

#### Vector Table
```
RST 08H (0x0008): General purpose
RST 10H (0x0010): General purpose
RST 18H (0x0018): General purpose
RST 20H (0x0020): General purpose
RST 28H (0x0028): General purpose
RST 30H (0x0030): General purpose
RST 38H (0x0038): Serial interrupt
RST 66H (0x0066): Non-maskable interrupt
```

### Constants
```
TRUE:      -1 (0xFFFF)
FALSE:      0 (0x0000)
UNLIMITED: -1 (0xFFFF)
CTRL_C:     3 (0x0003)
CTRL_H:     8 (0x0008)
BSLASH:    92 (0x005C)
```

### Character Set

#### Control Characters
```
0x00: NUL (String terminator)
0x03: ETX (End of text)
0x08: BS  (Backspace)
0x0A: LF  (Line feed)
0x0D: CR  (Carriage return)
0x1B: ESC (Escape)
```

#### Special Characters
```
0x20: Space (Command separator)
0x21-0x7E: Printable characters
0x5C: Backslash (Escape character)
0x60: Backtick (String delimiter)
```

### System Protection
- Stack underflow detection
- Stack overflow protection
- Memory bounds checking
- Invalid operation detection
- Error recovery procedures

### Error Handling
Error codes and their meanings:
```
0x01: Stack underflow
0x02: Stack overflow
0x03: Invalid memory access
0x04: Division by zero
0x05: Invalid opcode
0x06: Memory protection violation
0x07: Invalid nesting level
0x08: Buffer overflow
```

### Development Guidelines
1. Stack balance
   - Track stack effects
   - Document stack changes
   - Check for underflow

2. Memory usage
   - Use temporary storage efficiently
   - Clean up after operations
   - Monitor heap usage

3. Optimization
   - Use bit operations when possible
   - Minimize memory access
   - Keep critical paths short

---

# TACIT Programming Language - Command Documentation

## Command Reference

### Stack Effect Notation
Stack effects are shown as: ( before -- after )
Example: DUP ( n -- n n )

## Basic Operations

### Arithmetic Operators

#### + (Addition)
```
Stack Effect: ( n1 n2 -- sum )
Description: Add n1 and n2
Example: 5 3 +  \ Leaves 8 on stack
```

#### - (Subtraction)
```
Stack Effect: ( n1 n2 -- difference )
Description: Subtract n2 from n1
Example: 10 4 -  \ Leaves 6 on stack
```

#### * (Multiplication)
```
Stack Effect: ( n1 n2 -- product )
Description: Multiply n1 by n2
Example: 6 7 *  \ Leaves 42 on stack
```

#### / (Division)
```
Stack Effect: ( n1 n2 -- quotient )
Description: Divide n1 by n2
Example: 20 5 /  \ Leaves 4 on stack
```

#### % (Modulo)
```
Stack Effect: ( n1 n2 -- remainder )
Description: Remainder of n1/n2
Example: 17 5 %  \ Leaves 2 on stack
```

### Memory Operations

#### @ (Fetch)
```
Stack Effect: ( addr -- value )
Description: Fetch value from memory
Example: 1000 @  \ Fetch from address 1000
```

#### ! (Store)
```
Stack Effect: ( value addr -- )
Description: Store value to memory
Example: 42 1000 !  \ Store 42 at address 1000
```

### Stack Operations

#### DUP
```
Stack Effect: ( n -- n n )
Description: Duplicate top stack item
Example: 5 DUP  \ Leaves 5 5 on stack
```

#### DROP
```
Stack Effect: ( n -- )
Description: Remove top stack item
Example: 5 DROP  \ Removes 5 from stack
```

#### SWAP
```
Stack Effect: ( n1 n2 -- n2 n1 )
Description: Exchange top two stack items
Example: 1 2 SWAP  \ Leaves 2 1 on stack
```

#### ROT
```
Stack Effect: ( n1 n2 n3 -- n2 n3 n1 )
Description: Rotate top three stack items
Example: 1 2 3 ROT  \ Leaves 2 3 1 on stack
```

#### OVER
```
Stack Effect: ( n1 n2 -- n1 n2 n1 )
Description: Copy second item to top
Example: 1 2 OVER  \ Leaves 1 2 1 on stack
```

### Logic Operations

#### AND
```
Stack Effect: ( n1 n2 -- n3 )
Description: Bitwise AND
Example: 255 240 AND  \ Leaves 240 on stack
```

#### OR
```
Stack Effect: ( n1 n2 -- n3 )
Description: Bitwise OR
Example: 129 66 OR  \ Bitwise OR result
```

#### XOR
```
Stack Effect: ( n1 n2 -- n3 )
Description: Bitwise XOR
Example: 255 128 XOR  \ Bitwise XOR result
```

#### NOT
```
Stack Effect: ( n -- n )
Description: Bitwise NOT
Example: 0 NOT  \ Leaves all bits set
```

### Control Flow

#### : (Colon Definition)
```
Stack Effect: ( -- )
Description: Begin a word definition
Example: : SQUARE DUP * ;
```

#### ; (Semicolon)
```
Stack Effect: ( -- )
Description: End a word definition
Example: : DOUBLE 2 * ;
```

#### [ (Begin Quotation)
```
Stack Effect: ( -- )
Description: Begin quotation block
Example: [1 2 +] 
```

#### ] (End Quotation)
```
Stack Effect: ( -- )
Description: End quotation block
Example: [DUP *] i
```

### I/O Operations

#### . (Print Number)
```
Stack Effect: ( n -- )
Description: Print number and space
Example: 42 .  \ Prints "42 "
```

#### EMIT
```
Stack Effect: ( char -- )
Description: Print character
Example: 65 EMIT  \ Prints "A"
```

#### KEY
```
Stack Effect: ( -- char )
Description: Get character from input
Example: KEY  \ Waits for keypress
```

### String Operations

#### " (String Literal)
```
Stack Effect: ( -- addr )
Description: Create string literal
Example: "Hello"  \ Puts address on stack
```

#### ` (String Output)
```
Stack Effect: ( -- )
Description: Immediate string output
Example: `Hello`  \ Prints immediately
```

### Alternative Commands

#### ANOP (No Operation)
```
Stack Effect: ( -- )
Description: Do nothing
Example: ANOP  \ Just continues
```

#### TRUEX
```
Stack Effect: ( -- -1 )
Description: Push true value
Example: TRUEX  \ Pushes -1
```

#### FALSEX
```
Stack Effect: ( -- 0 )
Description: Push false value
Example: FALSEX  \ Pushes 0
```

### System Operations

#### RESET
```
Stack Effect: ( -- )
Description: System reset
Implementation: Reinitialize system
```

#### COLD
```
Stack Effect: ( -- )
Description: Cold start
Implementation: Full system initialization
```

#### WARM
```
Stack Effect: ( -- )
Description: Warm start
Implementation: Partial reset
```

### Special Purpose Commands

#### ALLOT
```
Stack Effect: ( n -- )
Description: Allocate n bytes
Example: 100 ALLOT  \ Allocate 100 bytes
```

#### VARIABLE
```
Stack Effect: ( -- )
Description: Create variable
Example: VARIABLE X  \ Create variable X
```

#### CONSTANT
```
Stack Effect: ( n -- )
Description: Create constant
Example: 42 CONSTANT ANSWER
```

## Error Handling Commands

#### ABORT
```
Stack Effect: ( -- )
Description: Abort execution
Implementation: Reset system state
```

#### ?STACK
```
Stack Effect: ( -- )
Description: Check stack underflow
Implementation: Abort if stack empty
```

---

# TACIT Programming Language - Implementation Details

## System Architecture

### Z80 Register Usage
```
AF: Accumulator and Flags
    - A: Primary arithmetic
    - F: Status flags

BC: Base Counter
    - Used for Text Input Buffer pointer
    - Loop counter in string operations

DE: Data Extra
    - Temporary storage
    - String operations

HL: Main work register
    - Memory operations
    - Arithmetic calculations

IX: Return Stack pointer
    - Manages RSTACK
    - Protected during interrupts

IY: Next routine pointer
    - Fast dispatch to NEXT
    - Loop optimization

SP: Data Stack pointer
    - Manages DSTACK
    - Standard Z80 stack operations
```

### Memory Management

#### Memory Layout Details
```
ROM Areas:
0x0000: Reset vector (JP RESET)
0x0008-0x0038: RST vectors
0x0040: Seven-segment lookup table
0x0180: TACIT core code start

RAM Areas:
0x0800: TIB (Text Input Buffer)
0x0900: Return Stack area
0x0A00: Data Stack area
0x0B00: Opcode tables
0x0C00: Variables
0x0CA0: Heap start
```

#### Buffer Management
```assembly
; TIB Buffer Handling
TIB:      DS   TIBSIZE    ; 256 bytes buffer
vTIBPtr:  DS   2          ; Current TIB pointer

; Buffer Operations
interpret2:
    LD  E,0              ; Reset nesting
    PUSH BC              ; Save TIB offset
    LD  HL,TIB          ; Point to buffer start
```

### Interrupt System Implementation

#### Interrupt Vector Table
```assembly
; Reset Vector
RSTVEC:  JP   RESET     ; 0x0000

; RST 08H - General Purpose
RST1:    PUSH af
         LD   a,1
         JP   ISR

; RST 38H - Serial
         RET            ; If not using ACIA

; NMI Handler
         PUSH af
         LD   a,8
         JP   ISR
```

#### Interrupt Service Routine
```assembly
ISR:     
    PUSH bc            ; Save registers
    PUSH de
    PUSH hl
    LD   h,0
    LD   l,a
    LD   (vIntID),hl  ; Store interrupt ID
    CALL enter
    .CSTR "Z"         ; Signal interrupt
    POP  hl
    POP  de
    POP  bc
    POP  af
    RET
```

### Serial Communication

#### ACIA Implementation
```assembly
; ACIA Constants
CONTROL:  EQU   $80    ; Control register
STATUS:   EQU   $80    ; Status register
TDR:      EQU   $81    ; Transmit Data
RDR:      EQU   $81    ; Receive Data

; Initialize ACIA
    LD   a,MRESET
    OUT  (CONTROL),a   ; Reset ACIA
    LD   a,RTSLID+F8N2+DIV_64
    OUT  (CONTROL),a   ; 8N2 format, /64 baud
```

#### Character I/O
```assembly
; Transmit Character
TXCHAR:  
    PUSH bc
    LD   b,a           ; Save character
TXCHAR1:
    IN   a,(STATUS)
    BIT  1,a          ; Test TDRE
    JR   z,TXCHAR1
    LD   a,b
    OUT  (TDR),a
    POP  bc
    RET

; Receive Character
RXCHAR:
    IN   a,(STATUS)
    BIT  0,a          ; Test RDRF
    JR   z,RXCHAR
    IN   a,(RDR)
    RET
```

### Stack Implementation

#### Stack Operations
```assembly
; Push to Return Stack
RPUSH:   
    DI               ; Disable interrupts
    DEC  IX
    LD   (IX+0),H
    DEC  IX
    LD   (IX+0),L
    EI               ; Enable interrupts
    RET

; Pop from Return Stack
RPOP:    
    DI               ; Disable interrupts
    LD   L,(IX+0)
    INC  IX
    LD   H,(IX+0)
    INC  IX
    EI               ; Enable interrupts
    RET
```

### Input Processing

#### Text Input Buffer
```assembly
; Process Input Character
WAITCHAR:
    CALL getchar     ; Get input
    CP   $20         ; Compare to space
    JR   NC,WAITCHAR1
    CP   $0          ; Check for null
    JR   Z,WAITCHAR4
    CP   "\r"        ; Check for return
    JR   Z,WAITCHAR3
    CP   CTRL_H      ; Check for backspace
    JR   z,backSpace
```

#### Command Processing
```assembly
; Command Interpreter
INTERPRET:
    CALL prompt
    LD   bc,0        ; Reset TIB pointer
    LD   (vTIBPtr),bc

; Process Commands
INTERPRET2:
    LD   E,0         ; Reset nesting
    PUSH bc          ; Save TIB pointer
    LD   hl,TIB      ; Point to buffer
```

### Number Handling

#### Number Conversion
```assembly
; Convert ASCII to Number
NUM:     
    LD   hl,$0000    ; Clear accumulator
    LD   a,(bc)      ; Get character
    CP   "-"         ; Check for negative
    JR   nz,num0
    INC  bc          ; Skip minus sign
NUM0:
    EX   af,af'      ; Save sign flag
NUM1:
    LD   a,(bc)      ; Get digit
    SUB  "0"         ; Convert to binary
    JR   c,num2      ; Exit if not digit
    CP   10          ; Check if valid
    JR   nc,num2
```

### Display Interface

#### Seven-Segment Display
```assembly
; Display Lookup Table
SEVENSEGMENT:
    DB   0EBH,28H,0CDH,0ADH  ; 0,1,2,3
    DB   2EH,0A7H,0E7H,29H   ; 4,5,6,7
    DB   0EFH,2FH,6FH,0E6H   ; 8,9,A,B
    DB   0C3H,0ECH,0C7H,47H  ; C,D,E,F
```

### Error Handling

#### Stack Protection
```assembly
; Check Stack Underflow
ETX:     
    LD   hl,-DSTACK  ; Check stack depth
    ADD  hl,SP
    JR   NC,etx1     ; If underflow
    LD   SP,DSTACK   ; Reset stack
```

### Initialization

#### System Startup
```assembly
; Initialize System
INIT:    
    LD   IX,RSTACK   ; Setup return stack
    LD   IY,NEXT     ; Setup NEXT pointer
    LD   hl,vars     ; Clear variables
    LD   de,hl
    INC  de
    LD   (hl),0
    LD   bc,VARS_SIZE * 3
    LDIR
```

## Performance Optimization

### Critical Path Optimization
1. Register Usage
   - Minimize memory access
   - Keep frequently used values in registers
   - Efficient register allocation

2. Loop Optimization
   - Use IY for NEXT dispatch
   - Minimize stack operations
   - Efficient counter handling

3. Memory Access
   - Batch operations where possible
   - Use block moves (LDIR)
   - Minimize bank switching

### Code Size Optimization
1. Subroutine Sharing
2. Table-Driven Dispatch
3. Macro Expansion Control

---

# TACIT Programming Language - Code Examples

## Basic Examples

### 1. Stack Manipulation
```tacit
\ Basic stack operations
42 DUP .        \ Prints: 42 42
1 2 SWAP .      \ Prints: 1 2 -> 2 1
1 2 3 ROT       \ Stack: 2 3 1

\ Complex stack manipulation
: 2DUP OVER OVER ;
: 2DROP DROP DROP ;
: 2SWAP >R >R 2SWAP R> R> 2SWAP ;
10 20 2DUP      \ Stack: 10 20 10 20
```

### 2. Arithmetic Operations
```tacit
\ Basic math
: SQUARE DUP * ;
: CUBE DUP DUP * * ;
5 SQUARE .      \ Prints: 25
3 CUBE .        \ Prints: 27

\ Complex calculations
: AVERAGE + 2 / ;
10 20 AVERAGE . \ Prints: 15

: FACTORIAL     \ Calculate factorial
  DUP 0= IF
    DROP 1
  ELSE
    DUP 1- FACTORIAL *
  THEN ;
```

### 3. Memory Operations
```tacit
\ Variable operations
VARIABLE X
42 X !          \ Store 42 in X
X @ .           \ Print X's value

\ Array operations
CREATE ARRAY 10 CELLS ALLOT  \ Create 10-cell array
: ARRAY@ CELLS ARRAY + @ ;   \ Array fetch
: ARRAY! CELLS ARRAY + ! ;   \ Array store

5 0 ARRAY!     \ Store 5 at index 0
3 1 ARRAY!     \ Store 3 at index 1
0 ARRAY@ .     \ Print first element
```

## Intermediate Examples

### 1. String Processing
```tacit
\ String reversal
: REVERSE
  DUP STRING-LENGTH  \ Get string length
  BEGIN
    DUP 0 >
  WHILE
    1- SWAP DUP ROT + C@
    EMIT
  REPEAT DROP ;

\ String comparison
: COMPARE-STRINGS
  BEGIN
    DUP C@ SWAP DUP C@ ROT
    2DUP = IF
      0= IF DROP DROP TRUE EXIT THEN
    ELSE
      DROP DROP FALSE EXIT
    THEN
    1+ SWAP 1+
  AGAIN ;
```

### 2. Control Structures
```tacit
\ If-Then-Else
: POSITIVE? 
  DUP 0> IF
    ." Positive"
  ELSE
    ." Non-positive"
  THEN ;

\ Counted loop
: COUNTDOWN
  BEGIN
    DUP 0> WHILE
    DUP . 1-
  REPEAT DROP ;

\ Case statement
: CASE? 
  DUP 1 = IF DROP ." One" EXIT THEN
  DUP 2 = IF DROP ." Two" EXIT THEN
  DROP ." Other" ;
```

## Advanced Examples

### 1. Data Structures

#### Linked List Implementation
```tacit
\ Node structure
: NODE CREATE 2 CELLS ALLOT ;
: VALUE@ @ ;           \ Get node value
: NEXT@ CELL+ @ ;     \ Get next pointer
: VALUE! ! ;          \ Set node value
: NEXT! CELL+ ! ;     \ Set next pointer

\ List operations
: MAKE-NODE           \ Create new node
  HERE >R            \ Save address
  , 0 ,             \ Value and null pointer
  R> ;              \ Return node address

: INSERT-FRONT       \ Insert at front
  SWAP OVER NEXT@ SWAP NEXT!  \ Link new to old
  SWAP OVER NEXT! ;          \ Update head
```

### 2. Sorting Algorithms

#### Bubble Sort
```tacit
: BUBBLE-SORT        \ Sort array of n elements
  DUP 1- 0 DO
    DUP 1- I DO
      DUP I CELLS + DUP CELL+
      @ OVER @ >    \ Compare adjacent
      IF
        DUP @ >R    \ Swap if needed
        OVER @ OVER !
        R> ROT !
      ELSE
        DROP DROP
      THEN
    LOOP
  LOOP DROP ;
```

### 3. Mathematical Functions

#### Scientific Calculator Functions
```tacit
: PI 355 113 */ ;    \ Approximate PI
: SQRT               \ Square root
  DUP 0= IF EXIT THEN
  DUP 2/ DUP
  BEGIN
    2DUP
    OVER DUP * ROT -
    ABS 1 >
  WHILE
    >R DUP R@ /
    R> + 2/
  REPEAT NIP ;

: POWER              \ x^n power function
  1 SWAP 0
  ?DO OVER * LOOP
  NIP ;
```

### 4. I/O Examples

#### Terminal Interface
```tacit
: PROMPT
  CR ." > " ;

: GET-LINE
  TIB DUP MAX-LINE
  ACCEPT SPACE ;

: PROCESS-COMMAND
  BL WORD FIND
  IF
    EXECUTE
  ELSE
    ." Unknown command" CR
  THEN ;

: TERMINAL
  BEGIN
    PROMPT
    GET-LINE
    PROCESS-COMMAND
    DEPTH 0< IF
      ." Stack underflow" CR
      QUIT
    THEN
  AGAIN ;
```

### 5. System Utilities

#### Memory Dump
```tacit
: .BYTE        \ Print byte in hex
  BASE @ >R
  HEX
  0 <# # # #> TYPE SPACE
  R> BASE ! ;

: DUMP         \ Memory dump utility
  CR
  BEGIN
    DUP .      \ Print address
    16 0 DO
      DUP I + C@
      .BYTE    \ Print each byte
    LOOP
    CR
    16 +       \ Next line
    DUP
    ?TERMINAL
  UNTIL
  DROP ;
```

### 6. Hardware Interface Examples

#### Display Control
```tacit
: DIGIT>SEG    \ Convert digit to segments
  SEVENSEGMENT + C@ ;

: SHOW-DIGIT   \ Display single digit
  DIGIT>SEG DISPLAY C! ;

: SHOW-NUMBER  \ Display multi-digit number
  0 100 UM/MOD    \ Split into digits
  ROT SHOW-DIGIT
  ROT SHOW-DIGIT
  SHOW-DIGIT ;
```

#### Serial Communication
```tacit
: INIT-SERIAL
  MRESET CONTROL C!      \ Reset ACIA
  F8N2+DIV_64 CONTROL C! \ 8N2 format
  RIE CONTROL C! ;       \ Enable Rx interrupt

: SEND-STRING
  BEGIN
    DUP C@ ?DUP
  WHILE
    EMIT
    1+
  REPEAT DROP ;
```

### 7. Game Examples

#### Number Guessing Game
```tacit
VARIABLE SECRET
VARIABLE TRIES

: RANDOM       \ Simple random number
  TIME @ 100 MOD 1+ ;

: INIT-GAME
  RANDOM SECRET !
  0 TRIES ! ;

: GUESS
  1 TRIES +!
  DUP SECRET @ =
  IF
    ." Correct! In "
    TRIES @ . ." tries" CR
    TRUE
  ELSE
    DUP SECRET @ <
    IF ." Too low" ELSE ." Too high" THEN
    CR FALSE
  THEN ;

: PLAY
  INIT-GAME
  BEGIN
    ." Enter guess: "
    KEY DUP EMIT
    GUESS
  UNTIL ;
```

### 8. Debug Utilities

#### Stack Inspector
```tacit
: .S          \ Print stack contents
  DEPTH
  BEGIN
    DUP 0>
  WHILE
    DUP PICK .
    1-
  REPEAT DROP ;

: TRACE       \ Simple trace utility
  DEPTH >R
  ." Before: " .S CR
  EXECUTE
  ." After:  " .S CR
  DEPTH R> - . ." items" CR ;
```

---


Copyright © 2024 John Hardy
GNU GENERAL PUBLIC LICENSE Version 3

