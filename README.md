
zephyr for LPC1768 Board

This are changes needed to get zephyr running on my lpc1768 based targetboard (see my other repository).

It's similar to LPCexpresso - has the same clock and uses a lot of similar pin configurations.

Currently only UART3 on pin 0.25 and 0.26 is supported.

The soc part still makes heavy use of the openlpc based code, this should be removed in the long run.
