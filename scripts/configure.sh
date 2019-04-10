#!/bin/bash
make distclean

if [ $1 = "olimex-stm32-e407" ]; then
#Configurations for Olimex Board
  if [ $2 = "nsh" ]; then
    ./tools/configure.sh olimex-stm32-e407/nsh
  elif [ $2 = "aux_serial" ]; then
    ./tools/configure.sh olimex-stm32-e407/aux_serial
  elif [ $2 = "nsh_uart" ]; then
    ./tools/configure.sh olimex-stm32-e407/nsh_uart
  elif [ $2 = "bmp180" ];then
    ./tools/configure.sh olimex-stm32-e407/bmp180
  elif [ $2 = "hih6130" ];then
    ./tools/configure.sh olimex-stm32-e407/hih6130
  elif [ $2 = "adc" ];then
    ./tools/configure.sh olimex-stm32-e407/adc
  elif [ $2 = "microxrcedds" ];then
    ./tools/configure.sh olimex-stm32-e407/microxrcedds
  elif [ $2 = "mrf24j40-6lowpan" ];then
    ./tools/configure.sh olimex-stm32-e407/mrf24j40-6lowpan
  elif [ $2 = "pm" ];then
    ./tools/configure.sh olimex-stm32-e407/pm
  elif [ $2 = "tcpecho" ];then
    ./tools/configure.sh olimex-stm32-e407/tcpecho
  elif [ $2 = "udpecho" ];then
    ./tools/configure.sh olimex-stm32-e407/udpecho
  elif [ $2 = "sd" ];then
    ./tools/configure.sh olimex-stm32-e407/sd
  elif [ $2 = "timer" ];then
    ./tools/configure.sh olimex-stm32-e407/timer
  elif [ $2 = "dac" ];then
    ./tools/configure.sh olimex-stm32-e407/dac
  elif [ $2 = "uros" ];then
    ./tools/configure.sh olimex-stm32-e407/uros
  else
    echo "Error"
  fi
#--------------------------------
#--------------------------------
elif [ $1 = "stm32l1" ]; then
#Configurations for STM32LDiscovery Board
  if [ $2 = "nsh" ]; then
    ./tools/configure.sh stm32ldiscovery/nsh
  elif [ $2 = "bmp180" ];then
    ./tools/configure.sh stm32ldiscovery/bmp180
  elif [ $2 = "hih6130" ];then
    ./tools/configure.sh stm32ldiscovery/hih6130
  elif [ $2 = "mrf24j40-mac" ];then
    ./tools/configure.sh stm32ldiscovery/mrf24j40-mac
  elif [ $2 = "microxrcedds" ];then
    ./tools/configure.sh stm32ldiscovery/microxrcedds
  elif [ $2 = "pm" ];then
    ./tools/configure.sh stm32ldiscovery/pm
  else
    echo "Error"
  fi
#--------------------------------
#--------------------------------
else
    echo "Error"
fi

