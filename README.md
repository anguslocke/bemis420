# bemis420
Shitty arduino code I threw together to control some addressable LED strips.

## spi
The strip is written to using the hardware SPI on the atmega328.
A SRAM buffer holds the pixel data to be pushed to the strip.
The first byte is written manually; afterwards, the transfer complete ISR writes
successive bytes from the buffer until the whole strip has been updated.

## main loop
### frame update
The start of the main loop initiates the strip push.
Afterwards, the buffer is updated with the next frame.  The frame-generating code
can start before the whole strip is pushed, as long as it only writes pixels that
have already been pushed.  (there are methods for safely checking push progress)
### input handling
maybe we poll inputs or something. not implemented yet.
