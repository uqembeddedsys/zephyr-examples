# Prac 3 - T Wilfred MK- S44280422 : Aaron Helmore - S44821526 <br />

## BASE NODE

## Overview <br />

Aim of this application is implement a base node that would actively search and establish a connection to a mobile node using BLE, then periodically read data from the mobile node and transmit to serial console using a custom serialization protocol (CSP).

<br />

## Functionality <br />

Expected funtionality has been succesfully implemented for the base device.

Status:

> 1.  Purple LED whne searching, blinking fast<br />
> 1.  Blue steady blink when connected and transmitting data to seriel<br />

<br />

## Folder Structure <br />

The folder structure for the following project is as seen below.

```
csse4011-s4428042
|
└───apps
│   └───P3
|       └───base
|
└───myoslib
│   └───cli
|       └───src
|       └───inc
│   └───hal
|       └───src
|       └───inc
│   └───os_hci
|       └───src
|       └───inc
│   └───scu
|       └───src
|       └───inc

```

## App Build Instructions - Linux/WSL <br />

Building for the Base:

> ~$ cd /apps/p3/base <br />
> ~$ west build -p <br />

Flashing:

> Put device into bootloader mode (Side Button) <br /> <br />
> Then run:<br /><br />
> ~$ nrfutil pkg generate --hw-version 52 --sd-req=0x00 \<br />
> --application build/zephyr/zephyr.hex \<br />
> --application-version 1 base.zip && sudo chmod a+rw /dev/ttyACM0 && nrfutil dfu >usb-serial ><br /> -pkg base.zip -p /dev/ttyACM0<br />
