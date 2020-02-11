#!/bin/bash


OLIMEX_TINY_H_FOUND=$(lsusb -d 15ba:0002b >/dev/null && echo "interface/ftdi/olimex-arm-usb-ocd-h.cfg")
STLINK_V2_FOUND=$(lsusb -d 0483:3748 >/dev/null && echo "interface/stlink-v2.cfg")

INTERFACE="$OLIMEX_TINY_H_FOUND $STLINK_V2_FOUND"

echo Interface: $INTERFACE

if [ $1 = "olimex-stm32-e407" ]; then
  openocd -f $INTERFACE -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
elif [ $1 = "stm32l1" ]; then
  openocd -f $INTERFACE -f target/stm32l1.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
else
  echo "Error. Try: ./flash.sh olimex-stm32-e407 or ./flash.sh stm32l1"
fi

