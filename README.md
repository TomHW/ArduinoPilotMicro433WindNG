# ArduinoPilotMicro433WindNG
Seatalk Autopilot Remote Control extended

This software is based on the Seatalk Autopilot Remote Control poject from [AK Homberger](https://github.com/AK-Homberger/Seatalk-Autopilot-Remote-Control). The original Software is focussed on the Autopilot, but I found that the hardware can also be used as an interface from Seatalk1 to NMEA0183. My boat has an old instrument cluster (Autohelm ST30) for measuring depth and speed through the water and I implemented parsers for depth and speed. The NMEA0183 records are sent via USB to a connected Raspberry Pi on which Signal K is running and distributed from there.

The remote control has nothing to do when the autopilot is disengaged and the Pro Micro Board has lots of free output pins. Therefore I added a relay board with 4 relays and connected them to free pins. The program evaluates the operating status of the Autopilot and when it is disengaged, the remote control switches the relays on and off. For example, a relay switches on the deck lighting. If I come to the boat in the dark, I can switch on the deck lighting from the dock.
