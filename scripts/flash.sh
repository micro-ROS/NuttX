#!/bin/bash

if [ $1 = "olimex-stm32-e407" ]; then
  openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
elif [ $1 = "stm32l1" ]; then
  openocd -f interface/stlink-v2.cfg -f target/stm32l1.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
else
  echo "Error. Try: ./flash.sh olimex-stm32-e407 or ./flash.sh stm32l1"
fi

