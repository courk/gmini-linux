#!/bin/sh

# Little script to automate arch/arm/confis/*

readonly CONFIGS_PATH="arch/arm/configs"

function usage(){
    echo "Usage: $0 [anchovy] [salami] [pepperoni] [chorizo] etc."
}

if [[ $# < 1 ]]; then
    usage
    exit 2
fi

if [[ `basename "$PWD"` != "kernel" ]]; then
  echo "script must be run from \$SOURCE_TOP/kernel"
  exit 2
fi

CONFIG_FILE=$CONFIGS_PATH/$1_defconfig
if [ ! -e $CONFIG_FILE ]; then
  echo "Could not find $CONFIGS_PATH/$1"
  exit 2
fi

export ARCH=arm
cp $CONFIG_FILE .config
make menuconfig
cp .config $CONFIG_FILE

