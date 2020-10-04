#!/bin/bash

## setup pigpio
# tool needed for installing pigpio
sudo apt install python-setuptools python3-setuptools

# make a Downloads directory
if [[ ! -d ~/Downloads ]]; then
    mkdir ~/Downloads
fi

cd ~/Downloads

wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install

## setup spi bus permissions via udev
UDEV_RULES_FILE=/etc/udev/rules.d/99-spi.rules
if [[ ! -f $UDEV_RULES_FILE ]]; then
    sudo touch $UDEV_RULES_FILE
fi
sudo echo "ACTION==\"add\",KERNEL==\"spidev0.0\",MODE=\"0660\",GROUP=\"spi\"\n" | sudo tee $UDEV_RULES_FILE
sudo echo "ACTION==\"add\",KERNEL==\"spidev0.1\",MODE=\"0660\",GROUP=\"spi\"\n" | sudo tee -a $UDEV_RULES_FILE
