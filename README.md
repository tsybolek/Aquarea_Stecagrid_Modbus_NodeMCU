# Aquarea_Stecagrid_Modbus_NodeMCU

In this project, NodeMCU:

a) reads current performance data (especially energy consumtion and generation) from a Panasonic Aquarea heat pump (over the CN-CNT + logic level shifter)

b) reads current AC power generation from a Stecagrid inverter (over RS-485 interface module)

c) averages the obtained values over the specified interval (e.g. 10 sec; duration can be set separately for aquarea and for Steca)

d) exposes these average values over modbus TCP (as slave) into the local network.

These data can be used to optimize the utilization of the self-produced electric energy (by over-heating the buffer tank during excessive 
current production by solar panels).

Modbus implementation is based on a tutorial from http://trialcommand.com

Ad a):

Many thanks to Egyras for aquarea protocol decrypting, see the repo here: https://github.com/Egyras/Panasonic-H-Aquarea
Specs of the CN-CNT connector and serial port as well as wiring scheme are also given there.
As for wiring, the important point is that aquarea CN-CNT uses 5V logic levels, whereas NodeMCU uses 3.3V logic levels, so they are not directly compatible.
Although the guys on Egyras' repo seem to use some resistors or voltage dividers, I do not, but instead just a logic level shifter like this: https://www.amazon.de/dp/B07LG6RK7L/ref=pe_3044161_185740101_TE_item
Having tested the hardware for more than 2 week, I can report absolutely no problems.

Ad b):

It was an unpleasant experience for me to discover that Stecagrid inverters use a *proprietary* protocol on their RS-485 bus, and upon my request, the vendor
provided only sparse documentation for the "services" and not the protocol itself. Well, since I only was interested in just one parameter (current AC power), it was not very
difficult to find out the exact HEX sequence for the request. The decryption of the answer was then possible by means of the provided ducumentation.
The RS-485 adapter module I'm using is this one: https://www.makershop.de/module/kommunikation-module/ttl-rs485-adapter/  It allows using the
RS-485 bus as a normal serial port (38400 8N1). Until now, no problems with the hardware either.
