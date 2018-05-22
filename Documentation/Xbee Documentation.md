XBee Documentation
==================

xBee sync is like a “backup” for the httpsync. 

The xBee class is an interface against the xBee antenna.
The xBeeSync is the code that uses the xBee class to send data over the antenna.
And the last component is the xBeeRelay which is a completely separate program with its own main-function. 
The xBeeRelay is meant to be run on a computer, with a xBee antenna, on the following boat to get all the data that the xBeeSync(the sailing robot) sends out.

![Xbee](Media/Xbee.png)


The xBeeSync can also be configured to send xml-encoded data to a computer running LabView so that you can follow what is happening from there.

The xBeeSync can be configured in a couple of different ways. To start off, the database table xbee_config contains all the configuration options.


|id   |send(bool) |recive(bool) |send_logs(bool) |loop_time(double) |                                   |
|-----|-----------|-------------|----------------|------------------|
|1    |0          |0            |0               |400.0             |


If the recieve field is set to 1, the xBee will listen for rudder and sail commands from LabView (you can use that program as a remote if you want to).

If the send field is set to 1, you can do one of two things depending on the send_logs flag. If send_logs is 0 the xBeeSync will send xml-encoded data to LabView. Else it will gather all the datalogs saved by the program and send them over the xBee antenna.

The loop_time field controls how fast the xBeeSync tries to send/recieve data.conny
