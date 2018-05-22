Tests
=======

## CANAIS Test

Integration test for the AIS, tests if we can receive and read the messages from the AIS.

**To use:** 

```shell
$ cd ../sailingrobot/NavigationSystem
$ make integration_test_ASPire
$ ./ais-integration-test.run
```

## CV7Integration Test

## HTTP Sync Test

Test for HTTPSync, test if it's able to connect to the server and database.

**To use:** 

```shell
$ cd ../sailingrobot/NavigationSystem
$ make integration_test_ASPire
$ ./HTTPSync-test.run
```

## Integration Test

Global Integrationtest for the ASPire.
Monitor the values from the CAN-bus and able to send commands to the actuators.

Currently monitoring windsensor, actuator feedback and if the radio controller is in manual mode. It is a interface between the messagebus and the CAN-bus that can be monitored.

**To use:** 

```shell
$ cd ../sailingrobot/NavigationSystem
$ make integration_test_ASPire
$ ./integration-tests-ASPire.run
```

## Logger Integration Test

## Marine Data integration test

## XBee Integration test

## XBee Sync Integration Test
