Note on wiring

1. Put all the wiring crap here
2. Put a note on the CHIP_SELECT, CHIP_ENABLE pins here
3. Change the sketehces to have that as input params

This sketch is the same as the multicast test but uses LEDs to indicate the
type of message being broadcasted.

1. Add wiring for Led pins

TX - Node
GreenLED - Sending to 00, should see GreenLED on (00) light up
BlueLED  - Sending Multicast message, should see the BlueLED on (00)
            and (01) light up
RedLED   - Sending message failed. Will light up with either GreenLED or
            BlueLED to indicate who was the intended recipient.
            ie. RedLED + GreenLED = Failed to send to 00
                RedLED + BlueLED  = Failed to send multicast (Mx) message
