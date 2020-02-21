#!/bin/bash

OLIMEX_TINY_H_FOUND=$(lsusb -d 15ba:0002b >/dev/null && echo "interface/ftdi/olimex-arm-usb-ocd-h.cfg")
STLINK_V2_FOUND=$(lsusb -d 0483:3748 >/dev/null && echo "interface/stlink-v2.cfg")

openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000" -c "reset" -c "exit"