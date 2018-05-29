 # Readme #
 
 * The CANFrameReciever is now set to intercept the last byte of the CanMsg and logging the value as an error if the value is larger than 0, before sending it further. This is due to the way the CanMessageHandler uses the last byte for errors. NOTE: The message will still be processed if an error has occured, but the error will be logged first.
