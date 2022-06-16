# motorblinds_esp32

### Parts
- Stepper motor
- Stepper driver
- ESP32 node mcu
- Coupling
- 100uF Capacitor
- Voltage step down convertor
- Breadboard
- Wires/solder
- Mounting bracket and screws

### Resources
#### TMC2209 Info
TMC2209 datasheet: https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf
BigTreeTech TMC2209 schematic: https://github.com/bigtreetech/BIGTREETECH-TMC2209-V1.2/blob/master/Schematic/TMC2209-V1.2.pdf
BigTreeTech TMC2209 manual: https://github.com/bigtreetech/BIGTREETECH-TMC2209-V1.2/blob/master/manual/TMC2209-V1.2-manual.pdf
ESP32 with TMC2209 stallguard example: https://github.com/edwardocano/Esp32-TMC2209/blob/master/stallguard/stallguard.ino
How to connect TMC2209 for UART (stallguard): https://forum.arduino.cc/t/using-a-tmc2209-silent-stepper-motor-driver-with-an-arduino/666992/14
Explaning TMC2209 settings: https://forum.arduino.cc/t/using-a-tmc2209-silent-stepper-motor-driver-with-an-arduino/666992/25
TMCStepper library documentation: https://teemuatlut.github.io/TMCStepper/class_t_m_c2209_stepper.html
AccelStepper library documentation: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html

More explanation on Stallguard: https://gist.github.com/metalinspired/dcfe07ed0b9f42870eb54dcf8e29c126

How to connect UART for stallguard: https://forum.arduino.cc/t/tmcstepper-arduino-tmc2209/956036/9

ESP-now scanning devices: https://circuitcellar.com/research-design-hub/design-solutions/using-esp-now-protocol-part-1/

https://www.home-assistant.io/integrations/cover.mqtt/