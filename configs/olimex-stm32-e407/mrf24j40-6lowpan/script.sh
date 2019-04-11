#!/bin/bash

echo "Installing Device tree Compiler\n"
sudo apt install device-tree-compiler

echo "Creating device tree for MRF24J40 radio and compiling it"
cat <<EOF >mrf24j40ma-overlay.dts
/dts-v1/;
/plugin/;

/ {
        compatible = "bcrm,bcm2835", "bcrm,bcm2836", "bcrm,bcm2708", "bcrm,bcm2709";

        fragment@0 {
                target = <&spi0>;
                __overlay__ {
                        #address-cells = <1>;
                        #size-cells = <0>;
                        status = "okay";

                        mrf24j40@0 {
                                compatible = "mrf24j40";
                                reg = <0>;
                                interrupts = <23 8>;
                                interrupt-parent = <&gpio>;
                                spi-max-frequency = <5000000>;
                        };

                        spidev@0 {
                                status = "disabled";
                        };

                        spidev@1 {
                                status = "disabled";
                        };
                };
        };
};
EOF

dtc -@ -O dtb -o mrf24j40ma.dtbo mrf24j40ma-overlay.dts
sudo cp mrf24j40ma.dtbo /boot/overlays/.

echo "Adding MRF24J40ma to system initialization"
sudo echo "dtparam=spi=on" > /boot/config.txt
sudo echo "dtoverlay=mrf24j40ma" > /boot/config.txt


echo "Installing WPAN tools"
git clone https://github.com/linux-wpan/wpan-tools 
sudo apt install dh-autoreconf libnl-3-dev libnl-genl-3-dev
cd wpan-tools
./autogen.sh
./configure CFLAGS='-g -O0' --prefix=/usr --sysconfdir=/etc --libdir=/usr/lib
make
sudo make install
sudo reboot
