coretest
========

Test platform for the Cryptech Open HSM project.

## Description ##
This platform and hardware design is used to functionally verfiy cores
developed in the Cryptech Open HSM project. The test core itself
contains just enough functionality to be able to verify that the SW in
the host computer can talk to the core in the FPGA by reading and
writing 32 bit data words to given addresses.

This project includes cores in Verilog, a testbench as well as host SW
to talk to the core.

## Architecture ##
The basic architecture contains a state machine that awaits read and
write commands from an external interface. These commands are sent to
the core being tested and the response is then returned to the host
using the same extarnal interface.

The commands accepted are:
  - RESET the core being tested. In total 1 byte.
    - 0x01 opcode
    
  - READ a 32-bit data word from a given address. In total 5 bytes.
    - 0x02 opcode
    - 32-bit address in MSB format


  - WRITE a given 32-bit data word to a given address. In total 9 bytes. 
    - 0x03 opcode
    - 32-bit address in MSB format.
    - 32-bit data in MSB format.


The possible responses are:
  - OK. Sent after successful write and reset. In total 1 byte.
    - 0xff response code.

  - ERROR. Sent after unsuccessful read or write operations. In total 1
    byte.
    - 0xfe response code.
    
  - DATA and 32-bit data. Sent after successful read operation. In total
    5 bytes.
    - 0xfd response code
    - 32-bit data in MSB format.
    

## Status ##
***(2014-02-10):***

Initial version of the project. Based on previous cttest project but
renamed and with new (ideas) about the test architecture.

