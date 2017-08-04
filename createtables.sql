PRAGMA foreign_keys = ON;
BEGIN TRANSACTION;

DROP TABLE IF EXISTS "config_buffer";
CREATE TABLE config_buffer (id INTEGER PRIMARY KEY AUTOINCREMENT,
	compass INTEGER,
);
INSERT INTO "config_buffer" VALUES(1,1,100,1);

DROP TABLE IF EXISTS "current_Mission";
CREATE TABLE current_Mission (id INTEGER PRIMARY KEY AUTOINCREMENT, -- no autoincrement to ensure a correct order
	isCheckpoint BOOLEAN,
	latitude DOUBLE,
	longitude DOUBLE,
	declination INTEGER,
	radius INTEGER,
 	stay_time INTEGER,
	harvested BOOLEAN
);


-- -----------------------------------------------------
-- Table dataLogs_gps
-- -----------------------------------------------------

DROP TABLE IF EXISTS "dataLogs_gps";
CREATE TABLE dataLogs_gps (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  time TIME,
  latitude DOUBLE,
  longitude DOUBLE,
  speed DOUBLE,
  course DOUBLE,
  satellites_used INTEGER,
  route_started BOOLEAN
);



-- -----------------------------------------------------
-- Table dataLogs_course_calculation
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_course_calculation";
CREATE TABLE dataLogs_course_calculation (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  distance_to_waypoint DOUBLE,
  bearing_to_waypoint DOUBLE,
  course_to_steer DOUBLE,
  tack INTEGER,
  going_starboard INTEGER
);



-- -----------------------------------------------------
-- Table GPSD_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "gpsd_config";
CREATE TABLE gpsd_config (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	loop_time DOUBLE
);



-- -----------------------------------------------------
-- Table arduino_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "arduino_config";
CREATE TABLE arduino_config (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	loop_time DOUBLE
);



-- -----------------------------------------------------
-- Table vesselState_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "vesselState_config";
CREATE TABLE vesselState_config (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	loop_time DOUBLE,
	speedLimit DOUBLE
);



-- -----------------------------------------------------
-- Table windState_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "windState_config";
CREATE TABLE windState_config (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	time_filter_ms INTEGER
);



-- -----------------------------------------------------
-- Table windsensor_datalogs
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_windsensor";
CREATE TABLE dataLogs_windsensor (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  direction INTEGER,
  speed DOUBLE,
  temperature DOUBLE
);



-- -----------------------------------------------------
-- Table dataLogs_compass
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_compass";
CREATE TABLE dataLogs_compass (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  heading INTEGER,
  pitch INTEGER,
  roll INTEGER
);



-- -----------------------------------------------------
-- Table dataLogs_actuator_feedback
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_actuator_feedback";
CREATE TABLE dataLogs_actuator_feedback (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  rudder_position DOUBLE,
  wingsail_position DOUBLE,
  rc_on BOOLEAN,
  wind_vane_angle DOUBLE
);



-- -----------------------------------------------------
-- Table dataLogs_actuator_feedback
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_current_sensors";
CREATE TABLE dataLogs_actuator_sensors (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  actuator_unit DOUBLE,
  navigation_unit DOUBLE
);



-- -----------------------------------------------------
-- Table dataLogs_system
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_system";
CREATE TABLE dataLogs_system (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  gps_id INTEGER,
  --course_calculation_id INTEGER,
  -- arduino_id INTEGER,
  windsensor_id INTEGER,
  compass_id INTEGER,
  --sail_command_sail_state INTEGER,
  --rudder_command_rudder_state INTEGER,
  --sail_servo_position INTEGER,
  --rudder_servo_position INTEGER,
  waypoint_id INTEGER, -- Rename it ?
  true_wind_direction_calc DOUBLE,
  CONSTRAINT gps_id
    FOREIGN KEY (gps_id)
    REFERENCES dataLogs_gps (id),
  CONSTRAINT course_calculation_id
    FOREIGN KEY (course_calculation_id)
    REFERENCES dataLogs_course_calculation (id),
  CONSTRAINT windsensor_id
    FOREIGN KEY (windsensor_id)
    REFERENCES dataLogs_windsensor (id),
  CONSTRAINT compass_id
    FOREIGN KEY (compass_id)
    REFERENCES dataLogs_compass (id),
  );


CREATE TRIGGER remove_logs AFTER DELETE ON "dataLogs_system"
BEGIN

DELETE FROM "dataLogs_gps" WHERE ID = OLD.gps_id;
DELETE FROM "dataLogs_course_calculation" WHERE ID = OLD.course_calculation_id;
DELETE FROM "dataLogs_compass" WHERE ID = OLD.compass_id;
DELETE FROM "dataLogs_windsensor" WHERE ID = OLD.windsensor_id;

END;



-- -----------------------------------------------------
-- Table config_xbee
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_xbee";
CREATE TABLE config_xbee (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  send INTEGER,
  receive INTEGER,
  send_logs INTEGER,
  loop_time DOUBLE,
  push_only_latest_logs BOOLEAN
);



-- -----------------------------------------------------
-- Table config_httpsync
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_httpsync";
CREATE TABLE config_httpsync (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  delay INTEGER,
  remove_logs BOOLEAN,
  push_only_latest_logs BOOLEAN
);



-- -----------------------------------------------------
-- Table config_wind_vane
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_wind_vane";
CREATE TABLE config_wind_vane (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  use_self_steering BOOLEAN,
  wind_sensor_self_steering BOOLEAN,
  self_steering_interval DOUBLE
);

-- -----------------------------------------------------
-- Table config_i2c
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_i2c";
CREATE TABLE config_i2c (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time DOUBLE
);

/*data for configs*/
INSERT INTO "config_xbee" VALUES(1,1,1,0,0.1,1);
INSERT INTO "config_httpsync" VALUES(1,0,0,1);
INSERT INTO "config_wind_vane" VALUES(1,1,0,1.5);
INSERT INTO "config_i2c" VALUES(1, 0.1);

-- HTTPSync
DROP TABLE IF EXISTS "config_HTTPSyncNode";
CREATE TABLE config_HTTPSyncNode (id INTEGER PRIMARY KEY AUTOINCREMENT,
	boat_id VARCHAR,	-- ex: boat01
	boat_pwd VARCHAR,
	srv_addr VARCHAR,
	configs_updated VARCHAR,
	route_updated VARCHAR
);

DROP TABLE IF EXISTS "scanning_measurements";
CREATE TABLE scanning_measurements (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	waypoint_id INTEGER,
	i INTEGER,
	j INTEGER,
	time_UTC TIMESTAMP, -- DEFAULT CURRENT_TIMESTAMP, -- this won't work, the pi does not know the time
	latitude DOUBLE,							  -- use script to insert time from gps to pi, at startup,
	longitude DOUBLE,							  -- or insert time with gpsmodel.time, as done with dataLogs
	air_temperature DOUBLE,

	-- not enforced: foreign_keys off (line 1)
	FOREIGN KEY(waypoint_id) REFERENCES waypoints(id) -- See the modification of waypoints
);

DELETE FROM sqlite_sequence;
INSERT INTO "sqlite_sequence" VALUES('configs',1);
COMMIT;
