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

