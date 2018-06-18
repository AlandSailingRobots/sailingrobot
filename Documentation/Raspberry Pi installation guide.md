Raspberry Pi system installation guide
======================================

Resources:
  * https://archlinuxarm.org/platforms/armv8/broadcom/raspberry-pi-3
  * `lsblk` – list block devices. Check device name of SD card (sdf on this machine)

This document should follow the syntax and formatting conventions of upstream Arch Linux documentation referenced above.

**Before formatting or overwriting SD-cards it is highly recommended to make backups.**

  * See *SD-card backup and restore guide* for detailed instructions on backing up data

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

5. Download and extract the root filesystem (as root, not via sudo, password is "root"):

    ```console
    # cd /home/sailbot
    # wget http://archlinuxarm.org/os/ArchLinuxARM-rpi-2-latest.tar.gz
	# su
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

    *Note:* It is also possible to do theese steps remotely by connecting to the Raspberry over [ssh](https://en.wikipedia.org/wiki/Secure_Shell) by obtaining the IP-adress of the Raspberry (for example by using `ifconfig`)

      If the Raspberry has the adress 192.168.4.112 and you wish to connect from a workstation:

      ```console
      $ ssh alarm@192.168.4.112
      alarm@192.168.4.112's password: # "alarm"
      Welcome to Arch Linux ARM
      
           Website: http://archlinuxarm.org
             Forum: http://archlinuxarm.org/forum
               IRC: #archlinux-arm on irc.Freenode.net
      Last login: Tue Mar 27 12:38:10 2018 from 192.168.4.191
      [alarm@sailbot ~]$ su
      Password: # "root"
      [root@sailbot alarm]# 
      ```

10. Generate locale and configure default as by https://wiki.archlinux.org/index.php/locale

    1. Edit the file */etc/locale.gen* and uncomment desired locale(s) (for example en_US@UTF-8)
    2. Run `locale-gen`
	3. Check that desired locale(s) exists in output of `locale -a`
	4. Edit */etc/locale.conf* and change `LANG=C` to:

        ```sh
        LANG=en_US.UTF-8
        ```

11. Change keyboard layout to scandinavian, check date and change if needed

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

12. Edit the file */boot/cmdline.txt*

    1. Save a copy of *cmdline.txt* just in case
    ```console
    # cp /boot/cmdline.txt /boot/cmdline.bak
    # nano /boot/cmdline.txt
    ```

    2. Add the following content to the line (while still keeping all content on a single line):

    ```
    dwc_otg.lpm_enable=0 fsck.repair=yes
    ```

    **Edit**: Previously the following were (perhaps wrongly) stated here: `console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rw rootwait fsck.repair=yesfsck.repair=yes`


13. Edit the file *boot/config.txt*

    ```console
    # nano /boot/config.txt
    ```
    The file should beyond possible default values also include the following:
    ```
    device_tree_param=i2c_arm=on
    device_tree_param=spi=on
    device_tree_param=i2s=on
    ```
        

14. Edit the file /etc/modules-load.d/raspberrypi.conf and add the lines `i2c-bcm2708` and `i2c-dev`


    ```console
    # nano /etc/modules-load.d/raspberrypi.conf
    ```
    Add the following modules:
    ```
    i2c-bcm2708
    i2c-dev
    ```


15. To change hostname (standard name is alarmpi)

    ```console
    # echo sailbot > /etc/hostname
    # hostname sailbot
    ```


## Install software on the Raspberry Pi

16. Synchronize system package database and install updates using pacman


    ```console
    # pacman -Syu
    ```

    (if you get timeouts, edit the file */etc/resolv.conf* using `nano /etc/resolv.conf` and add `options timeout:1`)


17. Install basic packages.
        
    ```console
    # pacman -S gcc make git sudo wget
    ```

18. Install i²c (intra-board communication)


    ```console
    # pacman -S i2c-tools
    ```
   

19. Install and configure gpsd (GPS daemon and library)


    1. Install
    ```console
    # pacman -S gpsd
    ```
	2. Edit */etc/gpsd* and change `DEVICES=""`to:
    ```sh
    DEVICES="/dev/gps0"
    ```
    3. (Re)start gpsd
    ```console
    # systemctl restart gpsd
    ```
    *Note:* If you have the GPS connected you can test it using `cgps -s`

20. install wiringPi (**Note:** Use a custom build from our gitrepo instead?)
        
    ```console
    # git clone git://git.drogon.net/wiringPi
    # cd wiringPi
    # ./build
    # cd ..
    ```

21. Install the boost libraries

    ```console
    # pacman -S boost boost-libs
    ```

22. Install the sailingrobot repositories in */root*
        
    ```console
    # cd /root
    # git clone --recursive https://github.com/AlandSailingRobots/sailingrobot.git
    ```

23. Configure automatic time sync using GPS and [NTP](https://en.wikipedia.org/wiki/Network_Time_Protocol)

    1. Install the ntp client and daemon
    ```console
    # pacman -S ntp
    # ntpdate -v -u pool.ntp.org
    ```
    * **Note:** ntp seems to be blocked in current network

    2. Configure gpsd as a timesource by editing */etc/ntp.conf* (from the guide [Using ntpd with GPS](https://wiki.archlinux.org/index.php/Network_Time_Protocol_daemon#Using_ntpd_with_GPS))
    ```sh
    #=========================================================
    #  GPSD native ntpd driver
    #=========================================================
    # This driver exists from at least ntp version 4.2.8
    # Details at
    #   https://www.eecis.udel.edu/~mills/ntp/html/drivers/driver46.html
    server 127.127.46.0 
    fudge 127.127.46.0 time1 0.0 time2 0.0 refid GPS
    ```

    3. Enable autostart of the ntpd service
    ```console
    # systemctl enable ntpd
    # systemctl start ntpd
    ```

    4. List ntp peer status by running `ntpq -p`

24. Follow the *SD-card backup and restore guide* on how to create a backup of the newly installed and configured system


*Extras:*
        
  * if your makefiles fail: `export SAILINGROBOTS_HOME=$HOME/sailingrobot`
  * if you get timeouts, edit the file */etc/resolv.conf* and add options
  * (console login as root: <kbd>Ctrl</kbd>+<kbd>Alt</kbd>+<kbd>F5</kbd> login as root/root)
