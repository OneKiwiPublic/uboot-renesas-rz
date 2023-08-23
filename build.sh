#!/bin/bash

ROOTDIR=`pwd`
export PATH=/home/vanson/working/avnet/gcc-arm-10.2-2020.11-x86_64-aarch64-none-elf/bin:$PATH
export CROSS_COMPILE=aarch64-none-elf-
export ARCH=arm64
export KBUILD_OUTPUT=./build

#make renesas-rzg2l-evk_defconfig
make solidrun-rzg2lc_defconfig
#make menuconfig
#make savedefconfig

make DEVICE_TREE=renesas-rzg2l-evk -j8
make DEVICE_TREE=solidrun-rzg2lc -j8