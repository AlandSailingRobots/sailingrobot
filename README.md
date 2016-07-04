# sailingrobot

## Required Packages

* Wiring PI (Can be found in the libs directory)
* sqlite3-dev
* libgps-dev
* libboast-all-dev
* libcurl-openssl-dev
* arm-linux-gnueabihf (For cross compiling to the PI)

## Getting Starting

After installing the required packages you need to bring in the git submodules, this can be done using the commands below:

```
git submodule init
git submodule update
```

To build on the PI run the following make command:

```
make clean all
```

To cross compile for the PI run the following make command:

```
make clean all TOOLCHAIN=raspi_cc
```

A database is also needed to run the control system, a script has been provided to set this up for you. just run installdb.sh. It will ask you for a server address and password, this is for our web tracking tool, if you are not using this you may just leave these blank.

MAIN LOOP

To get the right path to the db file when not on raspberry pi:
	run sr with "./sr ."
