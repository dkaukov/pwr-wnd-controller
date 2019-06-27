# Overview
This is firmware for the custom-built 3ch power window controller, using Arduino as MCU. I'm currently using it in Outlander 3rd gen.

# BOM
* 1x [Pololu 5V, 600mA Step-Down Voltage Regulator D24V6F5](https://www.pololu.com/product/2107)
* 1x [DFRduino Pro Mini V1.3(16M5V328)](https://www.dfrobot.com/product-696.html#.UnCo2lCjiBY)
* 3x [ACS711 Current Sensor Carrier -25 to +25A](https://www.pololu.com/product/2198)
* 1x [8 Channels 5V Relay Module](https://www.itead.cc/prototyping/basic-module/8-channels-5v-relay-module.html)

Approximate cost: $60US

# Features
* Auto-close windows after locking the car.
* Auto close/open on short press
* Auto learning window(s) end-points.
* Various safety features (Maximum run time, overcurrent protection, etc..)
* Supports "Lock" button
* Sleep mode, to reduce battery drainage.
* Can leave windows open - double click on remote.
