Raspberry Pi system installation guide
======================================

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

2. Dump the whole SD card to a file together with SHA-256 integrity checksum using *pv* and *tee*

    (If you need to install the commands on your workstation, use `sudo pacman -S pv tee`)

    ```console
    # pv /dev/sdx | tee sdcard.dd | sha256sum > sdcard.dd.sha256
    # sed -i 's/-$/sdcard.dd/' sdcard.dd.sha256
    ```
    OR

      * If you can not install *pv* and *tee* the same result can be acheived using *dd* but without any indication of progress)


        ```console
        # dd if=/dev/sdx of=sdcard.dd
        # sha256sum sdcard.dd > sdcard.dd.sha256
        ```

    You can include other data besides the image in the backup archive by adding files in this directory, they will then be included in the archive. For example `fdisk -l /dev/sdx > fdisk-l.txt` or you could edit a note in a textfile.


4. Compress the archive

    1. Exit the working directory where the raw backup output resides

        ```console
        # cd ..
        ```

    2. Compress the whole directory along with content created in previous steps

        This will create a mountable [SquashFS-container](https://en.wikipedia.org/wiki/SquashFS) (mksquashfs can be installed on Arch Linux workstations using `pacman -S squashfs-tools`)

        ```console
        # mksquashfs sdcard-backup sdcard-backup-$(date +%F).sqf -comp xz
        ```

        OR

          * create a bzip2-compressed tar-file

          ```console
          # tar cvjf sdcard-backup-$(date +%F).tar.bz2 sdcard-backup 
          ```

5. Clean up by deleting the backup directory

    ```console
    # rm -rf sdcard-backup
    ```


## Cloning a previously made backup to SD cards using a workstation

1. Create a mountpoint for accessing the contents of the backup archive and mount the SquashFS-archive

    ```console
    $ mkdir backupmnt
    ```

2. Mount archive contents in the directory, enter directory and list contents

    ```console
    # mount sdcard-backup-1234-56-87.sqf backupmnt
    # cd backupmnt
    # ls -lah
    ```

3. Insert SD card in card reader and make sure you know its device name

    * **NOTE:** If you use the wrong devicename below you might try to overwrite the harddrive in your workstation so be careful and check the devicename!

    ```console
    # lsblk
    # dmesg | tail -n 50
    ```

    * Below, instead of *sdx*, use the real devicename you got in the previous step

4. Clone the raw dd-image from the archive to the physcial SD card using *pv* (pipeviewer)

    ```console
    # pv sdcard.dd > /dev/sdx
    ```

5. Sync and eject SD card to ensure everything is written to it before removal

    ```console
    # sync
    # eject /dev/sdx
    ```

6. Clean up

    ```console
    # cd ..
    # umount backupmount
    # rmdir backupmount
    ```
    

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

10. Edit the file */boot/cmdline.txt*

  * Save a copy of *cmdline.txt* just in case
    ```console
    # cp /boot/cmdline.txt /boot/cmdline.bak
    # nano /boot/cmdline.txt
    ```

* Add the following content to the line (while still keeping all content on a single line):

    ```
    dwc_otg.lpm_enable=0 fsck.repair=yes
    ```

    **Edit**: Previously the following were (perhaps wrongly) stated here: `console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rw rootwait fsck.repair=yesfsck.repair=yes`


11. Edit the file *boot/config.txt*

    ```console
    # nano /boot/config.txt
    ```
    The file should beyond possible default values also include the following:
    ```
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
    # git clone https://github.com/AlandSailingRobots/sailingrobot.git
    ```

25. Extras:
        
  * if your makefiles fail: `export SAILINGROBOTS_HOME=$HOME/sailingrobot`
  * if you get timeouts, edit the file */etc/resolv.conf* and add options
5. Download and extract the root filesystem (as root, not via sudo): (<kbd>Ctrl</kbd>+<kbd>Alt</kbd>+<kbd>F5</kbd> login as root/root)

    ```console
    # cd /home/sailbot
