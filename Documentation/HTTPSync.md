HTTPSYNC DOCS 
=============

The sync reads boat id and password from the database table “server”. Also server URL is read from there. 

![HTTP Sync](Media/HTTPSYNC.png)

URL to push towards hostgator (live page) : http://www.sailingrobots.com/testdata/sync/

URL to push locally towards a computer in the webteam : 10.168.4.103/Remote-sailing-robots/sync/ (Mostly for testing)

(Replace IP with whatever computer you want to push towards)

Server database table : 

|id   |boat_id |boat_pwd |srv_addr                                    |
|-----|--------|---------|--------------------------------------------|
|1    |???     |****     |http://www.sailingrobots.com/testdata/sync/ |

The table should look something like this to work properly, this setup pushes data towards the live page. The end slash is important due to the 

The **setupHTTPSync()** function just initializes CURL connection to the server, if the URL is correct this shouldn’t be a problem. It kills the thread if this function fails.

The **pushWaypoints()** function pushes all the current waypoints present in the database up to the server. If no waypoints are added it should work regardless. Waypoints are parsed like Waypoint_1 ,Waypoint_2 at the moment and should probably be changed to how getLogs() parse the json data.

**updateConfigs()** reads configs from the server and adds all of them to the database. Configs are only read if they are changed on the webpage. Configs are not read directly into the program, you need to restart the program in order to get the new configs. TODO : Configs should be read into the program when they are updated , using the “state” datatable.

**pushConfigs()** pushes configs to the server. The config functionality is not used at the moment but it should be working.
Currently you can only have one config on the boat at the time. Should probably implement some kind of function to read different configs from server.

**syncServer()** is the main function. It gets the logs from the dbHandler already parsed as json and pushes them to the server. If the httpsync config remove_logs is enabled it will attempt to remove logs using the response from the server. The server response should look like following : [{“table” : “system_datalogs”, “id” : ]} to be able to work in remove logs. 

All tables ending with "datalogs" are getting pushed through the syncServer() function. The “system_datalogs” is the biggest one and manages the ids of the rest tables.

The datatable “state” manages if any new waypoints or configs has been added to the database. The sync writes to these tables to make the rest of the program aware of the config/route change.

The boat_id part needs to be able to check for a password on the server.

The config table for httpsync : 

|id   |delay |remove_logs |
|-----|------|------------|
|1    |0     |1           |


Delay adds a delay to the thread when it’s pushing logs. It should probably be changed to handle some kind of sending frequency rather than a delay.

The remove_logs flag decides if logs will be removed after they’re pushed. This flag is currently not implemented anywhere due to the fact that syncServer() pushes all logs all the time, so if no logs is removed it will push multiple entries of the same logs and the data size will increase after each insertDataLog() call.



