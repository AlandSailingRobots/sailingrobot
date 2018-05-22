Backups are currently stored as raw disk images inside compressed [SquashFS-containers](https://en.wikipedia.org/wiki/SquashFS) uploaded to [Google Drive](https://drive.google.com).

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

1. Insert SD card in card reader and make sure you know its device name

    * **NOTE:** If you use the wrong devicename below you might try to overwrite the harddrive in your workstation so be careful and check the devicename!

    ```console
    # lsblk
    # dmesg | tail -n 50
    ```

    * Below, instead of *sdx*, use the real devicename you got in the previous step

2. Create a mountpoint for accessing the contents of the backup archive and mount the SquashFS-archive

    ```console
    $ mkdir backupmnt
    ```

3. Mount archive contents in the directory, enter directory and list contents

    ```console
    # mount sdcard-backup-1234-56-78.sqf backupmnt
    # cd backupmnt
    # ls -lah
    ```

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
