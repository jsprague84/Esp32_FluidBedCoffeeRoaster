This is a work in progress.  The project is an esp32 based controller operating a modified popcorn popper.  
-The ESP32 uses 2 6675Max modules and K thermocouples for bean temp and env temp. 
-The heater uses a SSR with PWM. 
-The DC fan uses L289N module with PWM.

Utilizes Modbus TCP for communication with Artisan software for roaster control.
This code also provides a webserver for monitoring, debug, and manual controls

The project is for learning some different control protocols. I am expanding the capabilities to possibly roast without Artisan.  
In the future I will create an MQTT branch as well for integration with low code software like NodeRed.
