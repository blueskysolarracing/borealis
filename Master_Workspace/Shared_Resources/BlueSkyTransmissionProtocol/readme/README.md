**Changes from GEN10 version:** 

- Sender ID must now be passed in as an argument to B_tcpStart(), rather than be defined in “main.h” which is difficult to find.
- Enums are now provided in “protocol_ids.h” to give Sender IDs and Data IDs actual names. It is recommended to use these Enums to send and receive data, to increase code readability.
- New variable (senderID) and buffer (data[]) are added to the B_tcpPacket_t that is received. Using these new elements**,** parsing data in serialParse will be clearer and more consistent. However, legacy support is still maintained for parsing data in the old way.