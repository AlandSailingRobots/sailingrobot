PRAGMA foreign_keys = ON;
BEGIN TRANSACTION;

DROP TABLE IF EXISTS "current_Mission";
CREATE TABLE current_Mission (id INTEGER PRIMARY KEY AUTOINCREMENT, -- no autoincrement to ensure a correct order
	is_checkpoint BOOLEAN,
	latitude      DOUBLE,
	longitude     DOUBLE,
	declination   INTEGER,
	radius        INTEGER,
 	stay_time     INTEGER,
	harvested     BOOLEAN,
    id_mission    INTEGER,
    rankInMission INTEGER,
    name          VARCHAR(200)
);


-- -----------------------------------------------------
-- Table dataLogs_actuator_feedback
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_actuator_feedback";
CREATE TABLE dataLogs_actuator_feedback (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  rudder_position 	DOUBLE,
  wingsail_position DOUBLE,
  rc_on 			BOOLEAN,
  wind_vane_angle 	DOUBLE,
  t_timestamp		TIMESTAMP
);

-- -----------------------------------------------------
-- Table dataLogs_compass
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_compass";
CREATE TABLE dataLogs_compass (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  heading 		DOUBLE,
  pitch 		DOUBLE,
  roll 			DOUBLE,
  t_timestamp 	TIMESTAMP
);

-- -----------------------------------------------------
-- Table dataLogs_course_calculation
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_course_calculation";
CREATE TABLE dataLogs_course_calculation (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  distance_to_waypoint DOUBLE,
  bearing_to_waypoint  DOUBLE,
  course_to_steer 	   DOUBLE,
  tack 				   BOOLEAN,
  going_starboard 	   BOOLEAN,
  t_timestamp		   TIMESTAMP
);

-- -----------------------------------------------------
-- Table dataLogs_actuator_feedback
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_current_sensors";
CREATE TABLE dataLogs_current_sensors (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  actuator_unit   	DOUBLE,
  navigation_unit 	DOUBLE,
  wind_vane_angle 	DOUBLE,
  wind_vane_clutch 	DOUBLE,
  sailboat_drive 	DOUBLE,
  t_timestamp		TIMESTAMP
);

-- -----------------------------------------------------
-- Table dataLogs_gps
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_gps";
CREATE TABLE dataLogs_gps (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  has_fix		  BOOLEAN,
  online		  BOOLEAN,
  time 			  TIME,
  latitude 		  DOUBLE,
  longitude 	  DOUBLE,
  speed 		  DOUBLE,
  course 	      DOUBLE,
  satellites_used INTEGER,
  route_started   BOOLEAN,
  t_timestamp	  TIMESTAMP
);

-- -----------------------------------------------------
-- Table marine_sensors_datalogs
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_marine_sensors";
CREATE TABLE dataLogs_marine_sensors (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  temperature 	 DOUBLE,
  conductivity	 DOUBLE,
  ph			 DOUBLE,
  t_timestamp	 TIMESTAMP
);

-- -----------------------------------------------------
-- Table dataLogs_gps
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_vessel_state";
CREATE TABLE dataLogs_vessel_state (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  heading 	      DOUBLE,
  latitude 		  DOUBLE,
  longitude 	  DOUBLE,
  speed 		  DOUBLE,
  course 		  DOUBLE,
  t_timestamp	  TIMESTAMP
);

-- -----------------------------------------------------
-- Table dataLogs_gps
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_wind_state";
CREATE TABLE dataLogs_wind_state (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  true_wind_speed 			DOUBLE,
  true_wind_direction 		DOUBLE,
  apparent_wind_speed 		DOUBLE,
  apparent_wind_direction 	DOUBLE,
  t_timestamp	  			TIMESTAMP
);

-- -----------------------------------------------------
-- Table windsensor_datalogs
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_windsensor";
CREATE TABLE dataLogs_windsensor (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  direction   DOUBLE,
  speed 	  DOUBLE,
  temperature DOUBLE,
  t_timestamp  TIMESTAMP
);



-- -----------------------------------------------------
-- Table dataLogs_system
-- -----------------------------------------------------
DROP TABLE IF EXISTS "dataLogs_system";
CREATE TABLE dataLogs_system (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  actuator_feedback_id 	INTEGER,
  compass_id 			INTEGER,
  course_calculation_id INTEGER,
  current_sensors_id 	INTEGER,
  gps_id 				INTEGER,
  marine_sensors_id 	INTEGER,
  vessel_state_id	 	INTEGER,
  wind_state_id	 		INTEGER,
  windsensor_id 		INTEGER,
  current_mission_id 	INTEGER,
  CONSTRAINT actuator_feedback_id
  	FOREIGN KEY (actuator_feedback_id)
  	REFERENCES dataLogs_actuator_feedback (id),
  CONSTRAINT compass_id
    FOREIGN KEY (compass_id)
    REFERENCES dataLogs_compass (id),
  CONSTRAINT course_calculation_id
    FOREIGN KEY (course_calculation_id)
    REFERENCES dataLogs_course_calculation (id),
  CONSTRAINT current_sensors_id
  	FOREIGN KEY (current_sensors_id)
  	REFERENCES dataLogs_current_sensors (id),
  CONSTRAINT gps_id
    FOREIGN KEY (gps_id)
    REFERENCES dataLogs_gps (id),
  CONSTRAINT marine_sensors_id
    FOREIGN KEY (marine_sensors_id)
    REFERENCES dataLogs_marine_sensors (id),
  CONSTRAINT vessel_state_id
    FOREIGN KEY (vessel_state_id)
    REFERENCES dataLogs_vessel_state (id),
  CONSTRAINT wind_state_id
    FOREIGN KEY (wind_state_id)
    REFERENCES dataLogs_wind_state (id),
  CONSTRAINT windsensor_id
    FOREIGN KEY (windsensor_id)
    REFERENCES dataLogs_windsensor (id)
  );


CREATE TRIGGER remove_logs AFTER DELETE ON "dataLogs_system"
BEGIN

DELETE FROM "dataLogs_actuator_feedback" WHERE ID = OLD.actuator_feedback_id;
DELETE FROM "dataLogs_compass" WHERE ID = OLD.compass_id;
DELETE FROM "dataLogs_course_calculation" WHERE ID = OLD.course_calculation_id;
DELETE FROM "dataLogs_current_sensors" WHERE ID = OLD.current_sensors_id;
DELETE FROM "dataLogs_gps" WHERE ID = OLD.gps_id;
DELETE FROM "dataLogs_marine_sensors" WHERE ID = OLD.marine_sensors_id;
DELETE FROM "dataLogs_vessel_state" WHERE ID = OLD.vessel_state_id;
DELETE FROM "dataLogs_wind_state" WHERE ID = OLD.wind_state_id;
DELETE FROM "dataLogs_windsensor" WHERE ID = OLD.windsensor_id;

END;

-- -----------------------------------------------------
-- Table communication CAN AIS config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_ais";
CREATE TABLE config_ais (id INTEGER PRIMARY KEY AUTOINCREMENT,
	loop_time DOUBLE
);

-- -----------------------------------------------------
-- Table communication AIS processing config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_ais_processing";
CREATE TABLE config_ais_processing (id INTEGER PRIMARY KEY AUTOINCREMENT,
	loop_time 	DOUBLE,
	radius 		INTEGER,
	mmsi_aspire INTEGER
);

-- -----------------------------------------------------
-- Table communication CAN arduino config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_can_arduino";
CREATE TABLE config_can_arduino (id INTEGER PRIMARY KEY AUTOINCREMENT,
	loop_time DOUBLE
);

-- -----------------------------------------------------
-- Table HMC6343Node config (compass)
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_compass";
CREATE TABLE config_compass (id INTEGER PRIMARY KEY AUTOINCREMENT,
	loop_time 			DOUBLE,
	heading_buffer_size INTEGER
);

-- -----------------------------------------------------
-- Table CourseRegulatorNode config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_course_regulator";
CREATE TABLE config_course_regulator (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time 	   DOUBLE,
  max_rudder_angle INTEGER,
  p_gain 		   DOUBLE,
  i_gain 		   DOUBLE,
  d_gain 		   DOUBLE
);

-- -----------------------------------------------------
-- Table DBLogger Node config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_dblogger";
CREATE TABLE config_dblogger (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time DOUBLE
);

-- -----------------------------------------------------
-- Table GPS config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_gps";
CREATE TABLE config_gps (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time DOUBLE
);

-- -----------------------------------------------------
-- Table config_httpsync
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_httpsync";
CREATE TABLE config_httpsync (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time 			DOUBLE,
  remove_logs 			BOOLEAN,
  push_only_latest_logs BOOLEAN,
  boat_id 				VARCHAR,	-- ex: boat01
  boat_pwd 				VARCHAR,
  srv_addr 				VARCHAR,
  configs_updated		VARCHAR,
  route_updated 		VARCHAR
);

-- -----------------------------------------------------
-- Table SailControlNode config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_sail_control";
CREATE TABLE config_sail_control (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time 		DOUBLE,
  max_sail_angle 	INTEGER,
  min_sail_angle 	INTEGER
);

-- -----------------------------------------------------
-- Table Simulator config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_simulator";
CREATE TABLE config_simulator (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time DOUBLE
);

-- -----------------------------------------------------
-- Table Solar Tracker config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_solar_tracker";
CREATE TABLE config_solar_tracker (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time DOUBLE
);

-- -----------------------------------------------------
-- Table StateEstimationNode config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_vessel_state";
CREATE TABLE config_vessel_state (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time 			DOUBLE,
  course_config_speed_1 INTEGER,
  course_config_speed_2 INTEGER
);

-- -----------------------------------------------------
-- Table LocalNavigationModule config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_voter_system";
CREATE TABLE config_voter_system (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time 				DOUBLE,
  max_vote 					INTEGER,
  waypoint_voter_weight 	DOUBLE,
  wind_voter_weight 		DOUBLE,
  channel_voter_weight 		DOUBLE,
  midrange_voter_weight 	DOUBLE,
  proximity_voter_weight 	DOUBLE
);

-- -----------------------------------------------------
-- Table CANWindSensorNode config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_wind_sensor";
CREATE TABLE config_wind_sensor (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time DOUBLE
);

-- -----------------------------------------------------
-- Table WingsailControlNode config
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_wingsail_control";
CREATE TABLE config_wingsail_control (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time      DOUBLE,
  max_cmd_angle  INTEGER
);

-- -----------------------------------------------------
-- Table config_xbee
-- -----------------------------------------------------
DROP TABLE IF EXISTS "config_xbee";
CREATE TABLE config_xbee (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  loop_time 			DOUBLE,
  send 					INTEGER,
  receive 				INTEGER,
  send_logs 			INTEGER,
  push_only_latest_logs BOOLEAN
);

-- -----------------------------------------------------
-- Table sailing zone
-- -----------------------------------------------------
DROP TABLE IF EXISTS "sailing_zone";
CREATE TABLE sailing_zone (id INTEGER PRIMARY KEY AUTOINCREMENT
);

/*data for configs*/
INSERT INTO "config_ais" VALUES(1,0.5);
INSERT INTO "config_ais_processing" VALUES(1,0.5,0,0);
INSERT INTO "config_can_arduino" VALUES(1,0.5);
INSERT INTO "config_compass" VALUES(1,0.5,1);
INSERT INTO "config_course_regulator" VALUES(1,0.5,30,1,1,1);
INSERT INTO "config_dblogger" VALUES(1,0.5);
INSERT INTO "config_gps" VALUES(1,0.5);
INSERT INTO "config_sail_control" VALUES(1,0.5,70,15);
INSERT INTO "config_simulator" VALUES(1,0.5);
INSERT INTO "config_vessel_state" VALUES(1, 0.5, 1, 2); -- NOTE: Marc: See the values of the course_config_speed
INSERT INTO "config_voter_system" VALUES(1,0.5,25,1,1,1,1,2);
INSERT INTO "config_wind_sensor" VALUES(1,0.5);
INSERT INTO "config_wingsail_control" VALUES(1,0.5,15);
INSERT INTO "config_xbee" VALUES(1,1,1,0,0.1,1);

DELETE FROM sqlite_sequence;
INSERT INTO "sqlite_sequence" VALUES('configs',1);
COMMIT;
