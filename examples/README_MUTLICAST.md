Note on wiring

1. Put all the wiring crap here
2. Put a note on the CHIP_SELECT, CHIP_ENABLE pins here
3. Change the sketehces to have that as input params

 a) Node00 = MulticastListerNode00.ino, will write to serial the messages
      that are intended for it AND the multicast messages received
 b) Node01 = MulticastListerNode01.ino, will write to serial the messages
      that are intended for it AND the multicast messages received
 c) Node02-5 = TxMulticastNode.ino (See documentation for that class)
      basically it will transmit to both 00, and multicast.
