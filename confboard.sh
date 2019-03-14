#!/bin/bash
echo make distclean
make distclean
echo ./tools/configure.sh ./configs/olimex-stm32-e407/uros
./tools/configure.sh ./configs/olimex-stm32-e407/uros

# local setup
echo cd /root
cd /root
echo . ./uros_build_ws/install/local_setup.sh
. ./uros_build_ws/install/local_setup.sh 
echo cd nuttx
cd nuttx
