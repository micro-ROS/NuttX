To use this properly, please do the next steps:
We will take two differents devices coordinator(c) and end point(e). You can have all the end points that you want but only one coordinator.
This config is to setup a start topology network over 802.15.4 protocol.

C: i8 /dev/ieee0 startpan cd:ab
C: i8 set chan 11
C: i8 set saddr 42:01
C: i8 acceptassoc
e: i8 /dev/ieee0
e: i8 set chan 11
e: i8 set panid cd:ab
e: i8 set saddr 42:02
e: i8 set ep_saddr 42:01
e: i8 assoc

If the association is correct, you can send messages(In hexadecimal) from the e to c with this command
e: i8 tx abcdef
