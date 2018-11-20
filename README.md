# Vision
Mernok Vision anti-collision system firmware 

The MErnok vision system is used to provide a anti-collision avoidance to mine vehicles. It uses a magnetic field for close range warning, as well as periodic RF messages for long range presence detection.

The folders are split into libraries for the system and the actual projects with associated BSPs. The BPSs support all generations of the vision system boards. 

The Vision firmware flow is based on a modified RTOS flow. The main function constantly loops through a number of sub tasks which comete for process time multiple times per millisecond.

The main tasks, currently, are as follows:
* Master interface  
This deals checks for and deals with master communication on all ports. USB, CAN and UART messages are processed and responded to here. When an inbound message arives it signals the aprpriate interrupt, and this process uses the captured data. once the request is processed, a response is sent back to the master at the same port (USB, CAN, UART). This will, in theory, allow multiple masters with various communication media.
* CC1101   
this task handles RF interface. if a message needs to be sent, it sends it. Otherwise if the device is set up to receive inbound RF messages, it will set up the receiver and check for messages here.
* Refresh settings    
This process will check the RFID eeprom every few seconds to check if device settings have changed. 
* LF RX   
This process handles incoming LF magnetic messages. if one is recieved, it will signal to the RF task to send a response.
* LF TX   
If the device is set up to transmit a magnetic field, this will initiate the PWM loop at the set period.
* GPIO    
This thread checks all GPIO to see if they need to be reset (timed out). 
* Status   
This process handels the status of the device, currently it checks the voltage of the device, the charging status, the antenna and the CAN termination
* uBlox   
This task handles the GPS module communication.

something
zcvfd


