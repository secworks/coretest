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

The communication interface is a byte wide data interface with
handshake. The interface to the core to be tested is a memory like
interface with chip select, write enable and 16-bit address and 32-bit
data widths.


## Protocol ##
Coretest uses a simple command-response protocol to allow a host to
control the test functionality.

The commands are sent as a sequence of bytes with a command byte
followed by zero or more arguments. The response consists of a response
code byte followed by zero or more data fields.

The start of a command is signalled by a Start of Command (SOC)
byte. The end of a command is signalled by a End of Command (EOC)
byte. These bytes are:
  - SOC: 0x55
  - EOC: 0xaa

The start of a response is signalled by a Start of Response (SOR)
byte. The end of a response is signalled by a End of Respons (EOC)
byte. These bytes are:
 - SOR: 0xaa
 - EOR: 0x55

The commands accepted are:
  - RESET the core being tested. In total 3 byte with SOC and EOC.
    - 0x01 opcode


  - READ a 32-bit data word from a given address. In total 7 bytes with
    SOC and EOC.
    - 0x10 opcode
    - 32-bit address in MSB format


  - WRITE a given data word to a given address. In total 8 bytes
    with SOC and EOC. 
    - 0x11 opcode
    - 16-bit address in MSB format.
    - 32-bit data in MSB format.


The possible responses are:
  - OK. Sent after successful write and reset. In total 3 bytes with SOR
    and EOR.
    - 0xff response code.


  - UNKNOWN. Unknown command received, In tota 3 bytes with SOR and EOR.
    - 0xfe response code.


  - CMD_ERROR. Unsuccessful command such as write to read only register. In total 3
    bytes with SOR and EOR.
    - 0xfd response code.


  - DATA and 32-bit data. Sent after successful read operation. In total
    7 bytes with SOR and EOR.
    - 0x7f response code
    - 32-bit data in MSB format.
    

## Status ##
***(2014-02-10):***

Initial version of the project. Based on previous cttest project but
renamed and with new (ideas) about the test architecture.

Speciefied command and response protocol.

