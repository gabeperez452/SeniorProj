# SeniorProj

**gabemain.c:**
A main STM32 c file that allows the device to run off of the push of a button. First button push sets servo to 0 degrees (up), then every button push afterwards activates the device.

**gabeswitch.c:** 
A test file for testing switch inputs on the STM32.

**main.c:** 
OLD deprecated main file.

**FSM_main.c**
STM32 c file that communicates with the raspberry pi. Waits for PRINT:COMPLETE signal, to start the finite state machine. After running FSM removes print from Octoprint queue and starts a new print.

**signal_manager.py**
Python script that bridges connection between STM signalling and Octoprint API.

**signal_manager.Service**
Makes sure signal_manager.py runs at startup as a background application.


![STM32PINOUT](https://os.mbed.com/media/uploads/jeromecoutant/nucleo_l552ze_q_zio_right_2020_2_11.png)


