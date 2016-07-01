PRAGMA foreign_keys = ON;
BEGIN TRANSACTION;

DROP TABLE IF EXISTS "buffer_config";
CREATE TABLE buffer_config (id INTEGER PRIMARY KEY AUTOINCREMENT,
	compass INTEGER,
	true_wind INTEGER,
	windsensor INTEGER
);
INSERT INTO "buffer_config" VALUES(1,1,100,1);

DROP TABLE IF EXISTS "waypoints";
CREATE TABLE waypoints (id INTEGER PRIMARY KEY AUTOINCREMENT, -- no autoincrement to ensure a correct order
	latitude DOUBLE,
	longitude DOUBLE,
	declination INTEGER,
	radius INTEGER,
	harvested BOOLEAN
);

DROP TABLE IF EXISTS "waypoint_index";
CREATE TABLE waypoint_index (
	id INTEGER PRIMARY KEY,
	i INTEGER,
	j INTEGER,

	-- not enforced: foreign_keys off (line 1)
	FOREIGN KEY(id) REFERENCES waypoints(id)
);

DROP TABLE IF EXISTS "waypoint_stationary";
CREATE TABLE waypoint_stationary (
	id INTEGER PRIMARY KEY,
	time INTEGER,

	-- not enforced: foreign_keys off (line 1)
	FOREIGN KEY(id) REFERENCES waypoints(id)
);


-- -----------------------------------------------------
-- Table gps_datalogs
-- -----------------------------------------------------

DROP TABLE IF EXISTS "gps_datalogs";
CREATE TABLE gps_datalogs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  time TIME,
  latitude DOUBLE,
  longitude DOUBLE,
  speed DOUBLE,
  heading DOUBLE,
  satellites_used INTEGER,
  route_started BOOLEAN
);


-- -----------------------------------------------------
-- Table arduino_datalogs
-- -----------------------------------------------------

DROP TABLE IF EXISTS "arduino_datalogs";
CREATE TABLE arduino_datalogs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  pressure INTEGER,
  rudder INTEGER,
  sheet INTEGER,
  current INTEGER
);


-- -----------------------------------------------------
-- Table course_calculation_datalogs
-- -----------------------------------------------------
DROP TABLE IF EXISTS "course_calculation_datalogs";
CREATE TABLE course_calculation_datalogs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  distance_to_waypoint DOUBLE,
  bearing_to_waypoint DOUBLE,
  course_to_steer DOUBLE,
  tack INTEGER,
  going_starboard INTEGER
);



-- -----------------------------------------------------
-- Table windsensor_datalogs
-- -----------------------------------------------------
DROP TABLE IF EXISTS "windsensor_datalogs";
CREATE TABLE windsensor_datalogs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  direction INTEGER,
  speed DOUBLE,
  temperature DOUBLE
);



-- -----------------------------------------------------
-- Table compass_datalogs
-- -----------------------------------------------------
DROP TABLE IF EXISTS "compass_datalogs";
CREATE TABLE compass_datalogs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  heading INTEGER,
  pitch INTEGER,
  roll INTEGER
);



-- -----------------------------------------------------
-- Table system_datalogs
-- -----------------------------------------------------
DROP TABLE IF EXISTS "system_datalogs";
CREATE TABLE system_datalogs (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  gps_id INTEGER,
  course_calculation_id INTEGER,
  arduino_id INTEGER,
  windsensor_id INTEGER,
  compass_id INTEGER,
  sail_command_sail_state INTEGER,
  rudder_command_rudder_state INTEGER,
  sail_servo_position INTEGER,
  rudder_servo_position INTEGER,
  waypoint_id INTEGER,
  true_wind_direction_calc DOUBLE,
  CONSTRAINT gps_id
    FOREIGN KEY (gps_id)
    REFERENCES gps_datalogs (id),
  CONSTRAINT course_calculation_id
    FOREIGN KEY (course_calculation_id)
    REFERENCES course_calculation_datalogs (id),
  CONSTRAINT windsensor_id
    FOREIGN KEY (windsensor_id)
    REFERENCES windsensor_datalogs (id),
  CONSTRAINT compass_id
    FOREIGN KEY (compass_id)
    REFERENCES compass_datalogs (id),
  CONSTRAINT arduino_id
    FOREIGN KEY (arduino_id)
    REFERENCES arduino_datalogs (id)
  );


CREATE TRIGGER remove_logs AFTER DELETE ON "system_datalogs"
BEGIN

DELETE FROM "gps_datalogs" WHERE ID = OLD.gps_id;
DELETE FROM "course_calculation_datalogs" WHERE ID = OLD.course_calculation_id;
DELETE FROM "compass_datalogs" WHERE ID = OLD.compass_id;
DELETE FROM "windsensor_datalogs" WHERE ID = OLD.windsensor_id;
DELETE FROM "arduino_datalogs" WHERE ID = OLD.arduino_id;

END;

-- -----------------------------------------------------
-- Table sail_command_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "sail_command_config";
CREATE TABLE sail_command_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  close_reach_command INTEGER,
  run_command INTEGER
);



-- -----------------------------------------------------
-- Table rudder_command_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "rudder_command_config";
CREATE TABLE rudder_command_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  extreme_command INTEGER,
  midship_command INTEGER
);



-- -----------------------------------------------------
-- Table course_calculation_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "course_calculation_config";
CREATE TABLE course_calculation_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  tack_angle DOUBLE,
  tack_max_angle DOUBLE,
  tack_min_speed DOUBLE,
  sector_angle DOUBLE
);



-- -----------------------------------------------------
-- Table windsensor_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "windsensor_config";
CREATE TABLE windsensor_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  port VARCHAR(45),
  baud_rate INTEGER
);



-- -----------------------------------------------------
-- Table rudder_servo_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "rudder_servo_config";
CREATE TABLE rudder_servo_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  channel INTEGER,
  speed INTEGER,
  acceleration INTEGER
);



-- -----------------------------------------------------
-- Table sail_servo_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "sail_servo_config";
CREATE TABLE sail_servo_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  channel INTEGER,
  speed INTEGER,
  acceleration INTEGER
);



-- -----------------------------------------------------
-- Table maestro_controller_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "maestro_controller_config";
CREATE TABLE maestro_controller_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  port VARCHAR(45)
);



-- -----------------------------------------------------
-- Table xbee_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "xbee_config";
CREATE TABLE xbee_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  send INTEGER,
  recieve INTEGER,
  send_logs INTEGER,
  loop_time DOUBLE
);



-- -----------------------------------------------------
-- Table httpsync_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "httpsync_config";
CREATE TABLE httpsync_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  delay INTEGER,
  remove_logs BOOLEAN,
  push_only_latest_logs BOOLEAN
);



-- -----------------------------------------------------
-- Table sailing_robot_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "sailing_robot_config";
CREATE TABLE sailing_robot_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  flag_heading_compass INTEGER,
  loop_time DOUBLE,
  scanning BOOLEAN,
  line_follow BOOLEAN
);



-- -----------------------------------------------------
-- Table waypoint_routing_configrudder_servo";
DROP TABLE IF EXISTS "waypoint_routing_config";
CREATE TABLE waypoint_routing_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  radius_ratio DOUBLE,
  sail_adjust_time DOUBLE,
  adjust_degree_limit DOUBLE,
  max_command_angle DOUBLE,
  rudder_speed_min DOUBLE
);



-- -----------------------------------------------------
-- Table wind_vane_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "wind_vane_config";
CREATE TABLE wind_vane_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  use_self_steering BOOLEAN,
  wind_sensor_self_steering BOOLEAN,
  self_steering_interval DOUBLE
);

-- -----------------------------------------------------
-- Table i2c_config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "i2c_config";
CREATE TABLE i2c_config (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time DOUBLE
);

/*data for configs*/
INSERT INTO "sail_command_config" VALUES(1, 4400,5300);
INSERT INTO "rudder_command_config" VALUES(1,7000,5520);
INSERT INTO "course_calculation_config" VALUES(1,45.0,60.0,1.0,5.0);
INSERT INTO "windsensor_config" VALUES(1,'/dev/ttyS0',4800);
INSERT INTO "maestro_controller_config" VALUES(1,'/dev/ttyACM0');
INSERT INTO "rudder_servo_config" VALUES(1,4,0,0);
INSERT INTO "sail_servo_config" VALUES(1,3,0,0);
INSERT INTO "xbee_config" VALUES(1,1,1,0,0.1);
INSERT INTO "httpsync_config" VALUES(1,0,0,1);
INSERT INTO "sailing_robot_config" VALUES(1,1,0.5,0,0);
INSERT INTO "waypoint_routing_config" VALUES(1,0.5,5,25,90.0,1.0);
INSERT INTO "wind_vane_config" VALUES(1,1,0,1.5);
INSERT INTO "i2c_config" VALUES(1, 0.1);


-- only used in DBHandler::getLogs commented code, remove?
DROP TABLE IF EXISTS "messages";
CREATE TABLE messages (id INTEGER PRIMARY KEY AUTOINCREMENT, -- remove log after sync to minimize db size
	gps_time TIMESTAMP,
	type VARCHAR,
	msg VARCHAR,
	log_id INTEGER				-- FK
);

-- HTTPSync
DROP TABLE IF EXISTS "server";
CREATE TABLE server (id INTEGER PRIMARY KEY AUTOINCREMENT,
	boat_id VARCHAR,	-- ex: boat01
	boat_pwd VARCHAR,
	srv_addr VARCHAR
);

-- HTTPSync
DROP TABLE IF EXISTS "state";
CREATE TABLE state (id INTEGER PRIMARY KEY AUTOINCREMENT,
	configs_updated VARCHAR,
	route_updated VARCHAR
);

DROP TABLE IF EXISTS "mock";
CREATE TABLE mock (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	gps BOOLEAN,
	windsensor BOOLEAN,
	analog_arduino BOOLEAN,
	compass BOOLEAN,
	position BOOLEAN,
	maestro BOOLEAN
);
INSERT INTO "mock" VALUES(1,1,1,1,1,1,1);

DROP TABLE IF EXISTS "scanning_measurements";
CREATE TABLE scanning_measurements (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	waypoint_id INTEGER,
	i INTEGER,
	j INTEGER,
	time_UTC TIMESTAMP, -- DEFAULT CURRENT_TIMESTAMP, -- this won't work, the pi does not know the time
	latitude DOUBLE,							  -- use script to insert time from gps to pi, at startup,
	longitude DOUBLE,							  -- or insert time with gpsmodel.time, as done with datalogs
	air_temperature DOUBLE,

	-- not enforced: foreign_keys off (line 1)
	FOREIGN KEY(waypoint_id) REFERENCES waypoints(id)
);

DELETE FROM sqlite_sequence;
INSERT INTO "sqlite_sequence" VALUES('configs',1);
COMMIT;
