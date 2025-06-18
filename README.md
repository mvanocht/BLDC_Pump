# BLDC_Pump
Brushless DC Motor Water Pump

This Demo will be based on the ROHM BD16851 EVK.  A three-phase brushless motor (water pump) will be attached to the EVK.
The system will be powered using a Mean Well HLG-240H-12A Power Supply which can produce 12 V @ 16 A.  It is rated for IP65. An IP65 rating indicates that a device or enclosure is protected against dust and low-pressure water jets from any direction. It's a common rating for outdoor electronics, lighting, and other equipment exposed to the elements. The "6" in IP65 signifies complete dust protection, while the "5" means it can withstand water projected by a nozzle from any angle. 

An Arduino Uno will be connected to the ROHM BD16851 EVK and it will generate a 500 Hz PWM signal which will be used to control the speed of the pump.  A flow transducer will be connected to the Arduino to monitor the flow of water. The system will also include a resivour, a radiator, and a 1500 Watt Heater. Temperature sensors will monitor the Inlet and Outlet temperature of the water entering and leaving the radiator.

The Ardunio will be connected by USB to a laptop, running software to present a display for the user to view parameters and control the motor's speed, and to enable or disable a radiator fan. Consideration will be given to enable or disable the heater as well.

