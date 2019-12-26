#!/bin/bash

# Source tools
currdir="${0%/*}"
. $currdir/tools.sh

# lsusb dependency installation
install_dep usbutils

if [ $1 = "olimex-stm32-e407" ];then
  if lsusb -d 15BA:002a; then
    openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
  elif lsusb -d 15BA:0003;then
    openocd -f interface/ftdi/olimex-arm-usb-ocd.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
  elif lsusb -d 15BA:002b;then
    openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
  else
    echo "Error. Unsuported olimex-stm32-e407 flashing device. Try openocd"
  fi
elif [ $1 = "stm32l1" ];then
  openocd -f interface/stlink-v2.cfg -f target/stm32l1.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
else
  echo "Error. Try: ./flash.sh olimex-stm32-e407 or ./flash.sh stm32l1"
fi
