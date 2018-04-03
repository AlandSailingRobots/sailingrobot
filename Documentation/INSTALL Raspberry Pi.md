Raspberry Pi system installation guide
======================================
**Draft:** 2018-03-29 /KH

Resources:
  * https://archlinuxarm.org/platforms/armv8/broadcom/raspberry-pi-3
  * `lsblk` – list block devices. Check device name of SD card (sdf on this machine)

This document should follow the syntax and formatting conventions of upstream Arch Linux documentation referenced above.

## Backing up SD cards using a workstation

1. Create a directory where you would like to save your image (which will be included in the compressed archive in a later step)

    ```console
    $ mkdir sdcard-backup
    $ cd sdcard-backup
    ```

2. Dump the whole SD card to a file

    ```console
    # dd if=/dev/sdx of=sdcard.dd
    ```

    You can include other data here as well as all files in this directory will be included in the archive. For example `fdisk -l /dev/sdx > fdisk-l.txt` or you could edit a note in a textfile.


3. Generate a checksum for verifying integrity

    ```console
    $ sha256sum sdcard.dd > sdcard.dd.sha256
    ```

    **Note:** If you want to output progress and create checksum on-the-fly using `pv` and `tee` (depending on that you have them installed)

    ```console
    # pv /dev/sdx | tee sdcard.dd | sha256sum > sdcard.dd.sha256
    # sed -i 's/-$/sdcard.dd/' sdcard.dd.sha256
    ```

4. Compress the archive

    1. Exit the working directory where the raw backup output resides

        ```console
        # cd ..
        ```

    2. Compress the whole directory along with content created in previous steps

        **Example 1**: as a mountable [SquashFS-container](https://en.wikipedia.org/wiki/SquashFS) (mksquashfs can be installed on Arch Linux workstations using `pacman -S squashfs-tools`)

        ```console
        # mksquashfs sdcard-backup sdcard-backup-$(date +%F).sqf -comp xz
        ```

        **Example 2**: as a bzip2-compressed tar-file
    
        ```console
        # tar cvjf sdcard-backup-$(date +%F).tar.bz2 sdcard-backup 
        ```

5. Clean up by deleting the backup directory

    ```console
    # rm -rf sdcard-backup
    ```


## Cloning a previously made backup to SD cards using a workstation

(Create mountpoint, mount sqf-image or uncompress tar archive, dd or use pv from file to device)

To be continued...


## Setting up the SD card boot media using a workstation
**Note:** Replace sdx in the following instructions with the device name for the SD card as it appears on your computer.

1. Start fdisk to partition the SD card:

    ```console
    # fdisk /dev/sdx
    ```

2. At the fdisk prompt, delete old partitions and create new:

   1. Type **o**. This will clear out any partitions on the drive.
   2. Type **p** to list partitions. There should be no partitions left.
   3. Type **n**, then **p** for primary, **1** for the first partition on the drive, press ENTER to accept the default first sector, then type **+100M** for the last sector.
   4. Type **t**, then **c** to set the first partition **:w** to type W95 FAT32 (LBA).
   5. Type **n**, then **p** for primary, 2 for the second partition on the drive, and then press ENTER twice to accept the default first and last sector.
   6. Write the partition table and exit by typing **w**.

3. Create and mount the FAT filesystem:

    ```console
    # mkfs.vfat /dev/sdx1
    # mkdir boot
    # mount /dev/sdx1 boot
    ```

4. Create and mount the ext4 filesystem:

    ```console
    # mkfs.ext4 /dev/sdx2
    # mkdir root
    # mount /dev/sdx2 root
    ```

5. Download and extract the root filesystem (as root, not via sudo): (<kbd>Ctrl</kbd>+<kbd>Alt</kbd>+<kbd>F5</kbd> login as root/root)

    ```console
    # cd /home/sailbot
    # wget http://archlinuxarm.org/os/ArchLinuxARM-rpi-2-latest.tar.gz
    # bsdtar -xpf ArchLinuxARM-rpi-2-latest.tar.gz -C root
    # sync
    ```

6. Move boot files to the first partition:

    ```console
    # mv root/boot/* boot
    ```

7. Unmount the two partitions:

    ```console
    # umount boot root
    ```


## Configuring the system on the Raspberry Pi

8. Insert the SD card into the Raspberry Pi, connect ethernet, and apply 5V power


9. Log in on the console after it has started up

    * username: **root**
    * default password: **root**


16. Change keyboard layout to scandinavian, check date and change if needed

    ```console
    # localectl set-keymap sv-latin1
    # localectl set-x11-keymap se
    # date
    # date –-set “2018-03-29 15:33 EET”
    ```
    To change keyboard layout on-the-fly:
    ```console
    # loadkeys sv
    ```

10. Edit the file *boot/cmdline.txt*

  * **See also:** [Issues regarding UART on the Rpi3 workaround](https://www.raspberrypi.org/forums/viewtopic.php?p=947968#p947968). Save a copy of *cmdline.txt* just in case.
    ```console
    # cp cmdline.txt cmdline2.txt
    # nano /boot/cmdline.txt
    ```

* Add the following content to the line (while still keeping all content on a single line):

    ```
    dwc_otg.lpm_enable=0 fsck.repair=yes
    ```

    **Edit**: Previously the following were (perhaps wrongly) stated here: `console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rw rootwait fsck.repair=yesfsck.repair=yes`


11. Edit the file *boot/config.txt*

    Uncomment `#core_freq=250`, `device_tree_param=i2c_arm=on`, `device_tree_param=spi=on`, `device_tree_param=i2s=on` (search in nano with <kbd>Ctrl</kbd>+<kbd>w</kbd>) and add `enable_uart=1` at the end
        
    ```console
    # nano /boot/config.txt
    ```
    The file should beyond possible default values also include the following:
    ```
    core_freq=250
    enable_uart=1
    device_tree_param=i2c_arm=on
    device_tree_param=spi=on
    device_tree_param=i2s=on
    ```
        

12. Edit the file /etc/modules-load.d/raspberrypi.conf and add the lines i2c-bcm2708 and i2c-dev


    ```console
    # nano /etc/modules-load.d/raspberrypi.conf
    ```
    Add the following modules:
    ```
    i2c-bcm2708
    i2c-dev
    ```


13. To change hostname (standard name is alarmpi)

    ```console
    # echo sailbot > /etc/hostname
    ```

14. Stop and disable ttyS0 and gpsd

    **Edit:** This can not even be done at this point and is part of a later UART fix, also the services will probably already be disabled by default. Ignore this step?

    ```console
    # systemctl stop serial-getty@ttyS0.service
    # systemctl disable serial-getty@ttyS0.service
    # systemctl stop gpsd.socket
    # systemctl disable gpsd.socket
    ```


15. Reboot the system for settings to take effect

    ```console
    # reboot
    ```
    **Edit:** There is really no specific reason to reboot the system yet


## Installing software on the Raspberry Pi

17. Synchronize system package database and install updates using pacman


    ```console
    # pacman -Syu
    ```

    (if you get timeouts, edit the file */etc/resolv.conf* using `nano /etc/resolv.conf` and add `options timeout:1`)


18. Install basic packages.
        
    ```console
    # pacman -S gcc make git sudo wget
    ```

19. Install i²c (intra-board communication)


    ```console
    # pacman -S i2c-tools
    ```
   

20. Install gpsd (GPS daemon and library)


    ```console
    # pacman -S gpsd
    ```

21. install wiringPi (**Note:** Use a custom build from our gitrepo instead?)
        
    ```console
    # git clone git://git.drogon.net/wiringPi
    # cd wiringPi
    # ./build
    # cd ..
    ```


22. Install the boost libraries


    ```console
    # pacman -S boost boost-libs
    ```


23. Install the sailingrobot repositories in */root*
        
    ```console
    # cd /root
    # git clone https://github.com/pophaax/raspi
    ```



24. Enter the *raspi/Pishell* directory and type `./pishell.sh` to start Pishell and select install

    ```console
    # cd /raspi/Pishell
    # ./pishell.sh
    ```


25. Extras:
        
  * if your makefiles fail: `export SAILINGROBOTS_HOME=~/sailingrobot`
  * if you get timeouts, edit the file */etc/resolv.conf* and add options
`timeout:1`, to make it permanent, add it to */etc/resolv.conf.head* aswell
  * change date with date --set “26 Apr 2016 15:33 EET”

