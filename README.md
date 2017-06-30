# NavigationSystem

---

## Build

To build the system type make (target) (ext variable 1) (ext variable 2) ...

Example:
'''
make ASPire USE_SIM=1
'''

### Available Targets

all: Default, same as make ASPire

ASPire: Builds the configuration for ASPire

Janet: Builds the configuration for Janet

tests: Builds the unit tests

integration_test_ASPire: Build the integration test for ASPire

### External Variables

USE_SIM: 0 if you want to run on the hardware, 1 if you want to use the sailing_simulator (default 0)

USE_LNM: Chooses which navigation algorithm you want to use! 0 for the line-follow algorithm, 1 for the Local Navigation Module (Voter System)

---

## Updating the database

Two python scripts, update_config.py and waypoints.py updates the database (asr.db) from a json file
