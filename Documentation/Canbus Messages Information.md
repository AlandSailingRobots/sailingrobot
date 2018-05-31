 
# Canbus Messages Info #

* The standard CanMsg has 8 bytes of storage in total. If used with the CanbusMessageHandler which is the easiest way of manipulating and handling this data, the last byte is used for an error code.
(Further documentation exist in the CanbusMessageHandler.h file)

* The CANFrameReciever class does now have an automatic logging system, so if the last byte of a CanMsg is not zero, it will automatically log this bytes error before sending it further.

* Further documentation can be found in the [CanBusCommon Library](https://github.com/AlandSailingRobots/CanBusCommon).
