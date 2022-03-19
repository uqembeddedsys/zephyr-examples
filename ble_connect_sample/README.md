## **Sample BLE Connect Apps**

This is partial code from a previous years practical of CSSE4011,
it demonstrates how to filter and connect to a ble device. In this case,
the `base` app should be flashed on the dongle, where it will scan and
attempt to connect to ONLY the thingy52 with a specific ID (`mobile`).

Once connected, it will read GATT characteristics for various data
streams from the mobile and print over usb-console (/dev/ttyACMX).

The provided code is only partial from a much larger project. However,
you should still be able to compile both of these on Zephyr V3.0.

For compilation, you will only need to use `west build` in the app dir, as the build-sys has been pre-configured to build specifically for the required
board.

 * `base` -> nrf dongle.

 * `mobile` -> thingy52.

Base device (will be similar to your AHU)
```SHELL
$ cd base/
$ west build

$ nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
        --application build/zephyr/zephyr.hex \
        --application-version 1 plagarism_is_bad.zip

$ nrfutil dfu usb-serial -pkg plagarism_is_bad.zip -p /dev/ttyACM0

$ sudo screen /dev/ttyACM0  #ttyACMX, X maybe different for you...
```

Mobile device (will be similar to your SCU)
```SHELL
$ cd mobile/
$ west build
$ west flash

#Segger RTT
$ JLinkRTTLogger -Device NRF52840_XXAA -RTTChannel 1 -if SWD -Speed 4000 ~/rtt.log

#In another terminal
nc localhost 19021
```

### **IMPORTANT**
This implementation can *likely be improved* and you might notice dodgy
code/implementation of certain aspects. So use it only as a reference to
get you started.

### **APP LOGIC**
Once both boards have been flashed, the base device will scan for the
mobile device and it will blink purple. Once connected, the base device
will blink blue. Similarly, while not connected, the mobile device will
blink red rapidly, and once connected, it will blink slowly.

The mobile is setup with segger-rtt, so if you would like to debug, feel
free to see the print outputs. The base device has usb-console, can
be opened with screen to see debug output.