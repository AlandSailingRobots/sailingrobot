Install Arduino devtools
========================
(this is a draft)

Assuming you use Arch Linux

    ```console
    # pacman -S arduino
    ```

## Makefile (commandline build and upload)

Install [arduino-mk from AUR](https://aur.archlinux.org/packages/arduino-mk/). This could at a later stage be included as a [https://github.com/sudar/Arduino-Makefile](GitHub submodule).

As root, create the following symlink

```console
# ln -s /usr/share/arduino/hardware/archlinux-arduino /usr/share/arduino/hardware/arduino
```

## References

  * https://wiki.archlinux.org/index.php/arduino
