# nrf24-BayangSW
nRF24L01 multi-protocol RC transmitter - modified for use with the STM32F103C8 microcontroller.
Based on Goebish's nrf24-multipro

![Screenshot](https://i.imgur.com/PkeiJ6Y.jpg)  
![Screenshot](https://i.imgur.com/E3ntuna.jpg)  

Optimised for use with the BetaFPV Lite Radio 2, which has no screen or menu to configure models.

## Binding Procedure
- Use the default model setup on the transmitter - OpenTX companion can be used to confirm switch/channel assignments (I reversed the switches on the top)
- Set Bayang channel mapping to individual switch positions in nrf-multipro.ino
- While holding the appropriate stick pattern listed below, power up the Lite Radio.
- Arm quad based on it's firmware's specification. 


#### Protocol is selected with stick position at startup:

- all sticks neutral = Bayang protocol with Silverware telemetry for OpenTX
- Rudder right = EAchine E010, NiHui NH-010, JJRC H36 mini  
- Aileron left = EAchine H8(C) mini, BayangToys X6/X7/X9, JJRC JJ850, Floureon H101, BWhoop B03 ...  
- Rudder left = renew TX ID

#### Extra features (if available on aircraft):

- Channel 5: led light, 3 pos. rate on CX-10 and FQ777-124, H7, inverted flight on H101  
- Channel 6: flip control  
- Channel 7: still camera  
- Channel 8: video camera, pitch trim (FQ777-124)  
- Channel 9: headless  
- Channel 10: calibrate Y (V2x2), pitch trim (H7), RTH (H8 mini/H20, FQ777-124), 360deg flip mode (H8 mini 3D/H22)  
- Channel 11: calibrate X (V2x2), roll trim (H7,FQ777-124), emergency stop (BWhoop B03)  
- Channel 12: Reset / Rebind  

#### Rebind
- Hold Rudder left, throttle down and aileron right, pitch up for 2 seconds or so to rebind without power cycling the transmitter