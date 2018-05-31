DBHandler Documentation
=======================

The **getDataAsJson()** function makes a select towards a table and attempts to parse them into json.
There is an example of how it works in the getLogs() function.
The function selects data from a table using a Id to fetch specific fields and then parses it to json and returns it as a std::string. 
The keys for the values are mapped to the name of the table value fields. Example : the field “pressure” in “arduino_datalogs” will be mapped like : “pressure” : “500”;
The id can be left empty if you want to do select statements without an id. 
The table and key in parameters could probably be the same when the getWaypoints() function is changed. The parameter useArray makes the json parser use an array instead of json objects when it parses data. 

**getTableNames()** fetches a vector of table names depending of what the in parameter is. This function searches for tablenames and the in parameter makes it know what to look for. Example is found in getLogs().

**insertLog()** inserts a log entry into the database and returns the id it just inserted.

**insertDataLog()** is called from sailingrobots.cpp and this is where all the datalogs are being inserted into the database. This data comes from the whole program and includes things from other threads, like the windsensor. This function looks pretty ugly and should probably be changed somehow. At the end of the function the system_datalogs table saves foreign keys to the other tables to keep them ordered.

**updateConfigs()** reads data in json format and attempts to parse them and insert them into the database. This data is fetched from the server when configs are changed. It updates tables one after another using the json data.

**getLogs()** returns all data currently in the database from all tables ending with “datalogs” as json array. If no logs is removed it will send multiple logs, which shouldn’t happen.

**removeLogs()** removes logs using the response of the server. Some kind of check needs to be made that only real json data is being parsed, right now there is a check but it could be made better. 
A function called isJson(std::string data) would probably do. If the json class attempts to parse data that’s not in json format it will crash throwing a what() exception so parsing can be dangerous.

**getIdFromTable()** returns either max or min id from a table depending on the “max” in parameter.

**getWaypoints()** and **getConfigs()** returns data as json.