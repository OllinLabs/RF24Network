[![Bitdeli Badge](https://d2weczhvl823v0.cloudfront.net/svarmo/rf24network/trend.png)](https://bitdeli.com/free "Bitdeli Badge")

[![svarmo](https://img.shields.io/badge/status-stable-brightgreen.svg)]()

[![svarmo](https://img.shields.io/badge/platform-arduino-lightgrey.svg)]() [![svarmo](https://img.shields.io/badge/mcu-ATmega328P-lightgrey.svg)]()


# DYNAMIC ADDRESSING - SELF BALANCING Network Layer for nRF24L01(+) radios. (FOR ARDUINO ATMega)

This library has been adapted from TMRh20 and Manicbug’s code.

It was modified to work for ARDUINO ATMEGA. I removed all the code for RPi, ATTiny etc etc. The reason for this is that it was cluttered with pre-compiler conditionals for each MCU which made understanding the logic impossible and making changes a real challenge (Also it looked ugly as shit)

## Change overview
- It is designed (and tested) to work TMRh20/RF24 library (https://github.com/TMRh20/RF24) it is really good. I think it also works with the original manicbug version but I don't think I tested it further.
- Cleaned up formatting. (I would like to replace all underscores from variable names eventually)
- Removed dependencies on print\_f and made IF_SERIAL_DEBUG standard debug logging
- It is assumed that the listening pipe for all nodes is pipe 0
- Added broadcast messaging
- Changed input message queue to FIFO from LIFO. The FIFO queue uses a bi-dimensional list instead of list with pointers
- Added capacity to send/receive message larger than the maxCapacity. The messages are sent in shards.
- **Added dynamic messaging capabilities**, now new nodes can be added to the network without a predefined address and will request (via broadcast and channel scanning) an address to join the network.

I will update the Svarmo documentation wiki with more info on the changes

I will continue to actively support and update this library, file any issues and I will address them
in due time. Anyone interested in contributing is welcome to submit pull requests. If it becomes too
annoying  I'll just stop



## Reference reading:

Extra documentation:

1. Manicbug changes (original): Please see the full documentation at http://maniacbug.github.com/RF24Network/index.html
2. TMRh20 changes: http://tmrh20.github.io/RF24Network/
3. Svarmo changes: https://github.com/svarmo/RF24Network/wiki
