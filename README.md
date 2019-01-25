Mainline linux kernel for Orange Pi PC2
---------------------------------------------------

Megous' fork of the Linux kernel was meant for the Orange Pi PC2 among others. This fork of Megous'
fork is mainly out of my interest in running it on the Orange Pi PC2.

Features in addition to mainline added by Megous:

- CPU frequency and voltage scaling (cpufreq)
- Thermal regulation (if CPU heats above certain temperature, it will try to cool itself down by reducing CPU frequency)
- Re-enable ethernet support added and reverted during 4.13 development cycle
- HDMI output support (patches from [jernejsk](https://github.com/jernejsk/linux-1/tree/h3_hdmi_audio_v1) and icenowy, with DTS changes for H5 by me)
- Configure on-board micro-switches to perform system power off function
- Wireguard (https://www.wireguard.com/)

Further Modifications added to this fork:

- Node for H5's Mali-450 added to device tree.
- Set CONFIG_DRM_FBDEV_OVERALLOC to 200 in config

How to Compile for Orange Pi PC2
--------------------------------

A standard Linux desktop environment is needed with a recent aarch64 linaro toolchain is setup for cross-compiling.
I have succesfully compiled and booted this kernel using the gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu toolchain.

Assuming Orange Pi's SD card is mounted to /media/thekeyman/target_sd_card, type the following commands 
(adjusting paths to match your setup):

    export ARCH="arm64"
    export CROSS_COMPILE="aarch64-linux-gnu-"
    export PATH=/home/thekeyman/toolchains/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin:$PATH
    cp linux-4.14-64 .config
    export INSTALL_MOD_PATH=/media/thekeyman/target_sd_card
    make -j4 Image dtbs modules
    make modules_install

Booting the kernel
--------------------

Copy the following files to your SD card's /boot directory:

arch/arm64/boot/Image

arch/arm64/boot/dts/allwinner/sun50i-h5-orangepi-pc2.dtb

Adjust your u-boot boot script to use them. e.g.:

    load ${devtype} ${devnum} ${fdt_addr_r} /boot/sun50i-h5-orangepi-pc2.dtb
    load ${devtype} ${devnum} ${ramdisk_addr_r} /boot/uInitrd
    load ${devtype} ${devnum} ${kernel_addr_r} /boot/Image
    booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}

Recent mainline u-boots seem to properly support the Orange Pi PC2.
