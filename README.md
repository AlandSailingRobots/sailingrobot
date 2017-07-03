# Aland Sailing Robots

Project to make an autonomous sailing boat initiated by Åland University of Applied Sciences

[Website](www.sailingrobots.ax)

## NavigationSystem

### Build

To build the system type make (target) (ext variable 1) (ext variable 2) ...
(In the NavigationSystem folder)

##### Example:

```
make ASPire USE_SIM=1
```

#### Targets

```
all: Default, same as make ASPire

ASPire: Builds the configuration for ASPire

Janet: Builds the configuration for Janet

tests: Builds the unit tests

integration_test_ASPire: Build the integration test for ASPire
```

#### External Variables

```
USE_SIM: 0 if you want to run on the hardware, 1 if you want to use the sailing_simulator (default 0)

USE_LNM: Chooses which navigation algorithm you want to use! 0 for the line-follow algorithm, 1 for the Local Navigation Module (Voter System) (default 0)
```

### Run

To run the program use (no parameters needed)

```
./sr
```

### Updating the database

To update the configuration tables in the database, you run the update_config.py script with one of the configuration json files as a parameter

```
python update_config.py config_ASPire.json
```

To change the waypoints you use the waypoints.py together with one of the json in the Mission Folder

```
python waypoints.py Mission/ASS.json
```

## sailing_simulator

The python simulator exists as a submodule in this project
