# Prac 3 - T Wilfred MK- S44280422 : Aaron Helmore - S44821526 <br />

## MOBILE NODE <br />

## Overview <br />

Aim of this application is implement a mobile node that would scan for various data from four specific static node, and transmit this data to the base module using BLE with GATT characteristics once connected.

<br />

## Functionality <br />

Expected funtionality has been succesfully implemented for the base device.

Status:

> 1.  Green LED when searching, blinking FAST<br />
> 1.  Green LED when connected, blinking SLOW<br />

<br />

## Folder Structure <br />

The folder structure for the following project is as seen below.

```
csse4011-s4428042
|
└───apps
│   └───P3
|       └───mobile
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

> ~$ cd /apps/p3/mobile <br />
> ~$ west build -p <br />

Flashing:

> ~$ west flash<br />
