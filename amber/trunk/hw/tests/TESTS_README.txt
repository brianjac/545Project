Description of tests found in amber

adc.S
    Addition test -- checks for overflow

addr_ex.S
    Interrupt (exception) test -- Jumps to an exception address in order to trigger jump to supervisory mode, and checks that jump + register banking + masking is successful

add.S
    Simple additon test -- not comprehensive

and.S
    Tests and with flag set -- good basic format for other flag set operator testing


barrel_shift_rs.S
    Tests Rs barrel shifter -- tests 0 shift and overflow shifts


barrel_shift.S
    Tests inline barrel-shifter (modifying arguments before they hit the execution unit), does 0, 1, 31, and 8.

bcc.S (branch on carry clear)
    moves with carry clear, and then branches on carry clear to either pass or fail (not very comprehensive)

bic_bug.S
    tests specific bug with bic during Linux kernel_init.
The following instruction stored the result    // 
//  in r3, instead of r2                                        //
//  c00b229c:	e1120310 	tst	r2, r0, lsl r3          //
//  c00b22a0:	11c22310 	bicne	r2, r2, r0, lsl r3      //
Tests this error and then compares values in registers to make sure value was stored to the right register.

bl.S
    branch and link -- checks that 1 branch with link gets the correct return address, can supplement this with comprehensive branch tests from 447 (translated to ARM ISA)

cache1.S
    Basic cache operation test; reads + writes with summation check at the end

cache2.S
    Tests operation between cached data and uncached instruction accesses (basic, not comprehensive)

cache3.S
    Tests large writes and reads-- writes to and reads multiple times from 2k words (> size of the entire cache)
256 lines x 4 words x 1 way = 1024 words

cacheable_area.S
    Cacheable_area co-processor function
    -Writes to a block of memory that straddles cache region boundary (first 16 bytes cacheable, 2nd 16 are not), loops a few times to enable evictions, reads and performs summing check

cache_flush.S
    Performs a flush in the middle of a sequence of data reads, checks read correctness.
Only tests 1 flush total, should do multiple (in a row, in different running conditions)

cache_swap_bug.S
/*
Bug is caused by very subtle timing interactions in the Cache

It does not detect a hit, which it should, on the read phase of the swap
operation. It doesnt detect the hit because it is still completing
a fill started by the memory request to load an instructuion
at swap instruction + 8.
This only occurs when the swap target address is in the cache
but the cache instruction address is not, and the instruction
address the third of a group of 4 instruction words.

Test copies sequence to another bit of memory and runs it.
Repeats this a few times moving the sequence to slightly
different memory locations each time
*/

cache_swap.S
    Fills the cache, then does a swap using data in the cache. That data is checked to make sure it is invalidated (correct action). Check is done by another read (interpreted as a cache miss).

change_mode.S
    teq, tst, cmp, cmn with the p-flag set
    starts in supervisor mode, changes to interrupt mode, then fiq mode, then supervisor again, finally user mode

change_sbits.S
    movs where the destination register is r15
    depending on the processor mode and whether the s bit is set or not, some or none of the status bits will change.

In supervisor mode,switch to user mode, then set and  check condition flags

In user mode, only the condition status bit and the pc can be changed. In user mode, change to supervisor mode (this will fail)

conflict_rd.S
    tests that a register conflict between a ldr and a regop that changes the value of the same register is handled correctly (data dependency?)

ddr31.S
    Word accesses to random addresses in DDR3. Creates a list of addresses in an area of boot_mem. Then it writes to all addresses with data value equal to address. Finally it reads back all locations checking that the read value is correct.

ddr32.S
    byte read and writes to DDR3

ddr33.S
    back to back write-read accesses to DDR3

ethmac_mem.S
    Wishbone access to the internal memory in the ethernet MAC module (we are not using this part of the core)

ethmac_reg.S
    Wishbone access to registers in the Ethernet MAC module (we are not using this module)

ethmac_tx.S
    Ethernet MAC frame transmit and recieve functions and Ethmac DMA access to hiboot mem. Ethmac is put in loopback mode and a packet is transmitted and recieved (we are not using the Ethmac module for our project)

firq.S
    tests fiq's (fast interrupts)
    executes 20 FIRQs randomly while executing a small loop of code.
    Test checks the full set of FIRQ registers (r8-r14) and will only pass if all interrupts are handled correctly. (It doesn't look like it tests SPSR_fiq or changes to the CPSR though....)

flow1.S
    instruction and data flow (THIS IS SUPER IMPORTANT FOR OoO)
    tests that a stm writing to cached memory also writes all data through to main memory

flow2.S
    instruction and data flow (IMPORTANT FOR OoO)
    tests that a stream of str instructions writing to cached memory works correctly

flow3.S
    instruction and data flow (IMPORTANT FOR OoO)
    tests ldm where the pc is loaded which causes a jump. At the same time the mode is changed, this is repeated with the cache enabled.

flow_bug.S
    Bug: processor illegally skipped an instruction after a sequence of 3 conditional not-execute instructions and 1 conditional execute instruction

hiboot_mem.S
    Wishbone read and write access to hi (non-cacheable) boot SRAM

inflate_bug.S
    Tests a specific load-store sequence that was not executing correctly

irq_disable.S
    runs a sequence that adds a bunch of numbers 80 times. during this execution , IRQ interrupts are triggered using a random timer (possible nested interrupts, so irq nesting disable is important for this test).

irq.S
    Same (or very similar) to irq_disable.S test above

irq_stm.S
    Loop of stm instructions interrupted by random IRQ interrupts. Check that stm instruction is not executed twice in a row (once before and once after the interrupt)

ldm1.S
    Tests the standard form of ldm

ldm2.S
    Tests ldm where the user mode registers are loaded while in a priviledged mode (must save those register values!!)

ldm3.S
    Tests ldm where the status bits are loaded

ldm4.S
    Tests ldm in user mode where the status bits are loaded. (the s bit should be ignored in user mode)

ldm5.S
    Loads user mode registers that are banked in supervisor mode, and then checks to make sure that the supervisor mode registers are unchanged

ldm_stm_onetwo.S
    tests ldm and stm of single registers with cache enabled, then tests ldm and stm of two registers with the cache enabled

ldr.S
    Tests ldr and ldrb (yay bytes yay)

ldr_str_pc.S
    Tests ldr and str of r15 (pc, for jumps)

ldrt.S
    More (similar/same tests from ldr and ldrb?) or ldr and ldrb

mla.S
    Test for mla (multiply and accumulate) instruction
    uses a test set of 16 numbers and multiplies them with each other in every combination.
    Each result checked against an expected result. (good range of values for robust multiplication testing)

mlas_bug.S
    Bug: The flags were getting set 1 cycle early. So in an operation with (0 x N) + 1, the z flag was set because it didn't take into account the addition done right at the end

mov_rrx.S
    Tests mov with RRX (rrx moves the bits in register rm to the right by 1 -- rotate right with extend)
    Checks that the carry flag value is rotated into the target register

movs_bug.S
    Bug in linux memcpy

mul.S
    Test for mul (multiply) instruction
    Uses a testset of 16 numbers and multiplies them with eachother in every combination. Checks each result against expected result

sbc.S
    64-bit subtractions using subtract with carry

stm1.S
    Basic tests for data storage

stm2.S
    Test jumps into user mode, loads some values into registers 8-14, then jumps to FIRQ and saves the suer mode registers to memory (!!! impt)

stm_stream.S
    Generates dense stream of writes as possible to check that the memory controller/caches/hierarchy can cope with the worst-case request load (not sure if this deals with locality when generating stream -- might want to write sub-tests that generate stream with high locality vs. with very low locality (higher latency))

strb.S
    Tests str and strb with different indexing modes

sub.S
    Corner case tests for sub and subs

swi.S
    Software interrupt testing

swp_lock_bug.S
    Bug: An instruction read broke immediately after a swp instruction

swp.S
    Tests for swp and swpb

teq.S (test equivalent)
    // Test "Strange issue with r12 after TEQLSP"
// Tests for bug where testlsp command would switch the core 
// from supervisor into FIRQ mode if it were executed. However the condition is not
// met so it is not executed. The bug is the next instrument works in FIRQ mode anyway.
// This was caused because the mode bits used in the ececute stage were not conditional on the
// teq instrumention being executed.

// Also tests correct setting of carry flag when second operand is a constant

tst.S
    Uses other tests and uses tst at the end to check flags (test bits)

uart_reg.S
    Wishbone read and write access to the Amber UART registers (Important for our preliminary implementation!!)

uart_rxint.S
    UART receive interrupt function. Some text is sent from the test_uart to the uart and an interrupt is generated. (Important for our preliminary implementation!!)

uart_rx.S
    UART recieve function tests (Important for preliminary implementation!!)

uart_tx.S
    uses the tb_uart in loopback mode to verify the transmitted data (Important for preliminary implementation!!)

undefined_ins.S
    Tests the undefined instruction interrupt.
    Fires a few FP instructions into the core. These cause undefined instruction interrupts with executed. (We can use this undefined interrupt in order to switch modes possibly)
