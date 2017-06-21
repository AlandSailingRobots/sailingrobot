#!/bin/bash
# Prepares db and runs tests

START_DIR=/home/sailbot/sailingrobot/tests/db/
INSTALL_DIR=/home/sailbot/installation/
TEST_DIR=/home/sailbot/sailingrobot/tests/db/
DB_FILE=createtables.sql
TEST_DB_FILE=testdb.db

if [ -f "$TEST_DB_FILE" ]; then
	rm "$TEST_DB_FILE" 
fi

cd $INSTALL_DIR

if [ -f "$DB_FILE" ]; then

	echo "Creating db..."
	sqlite3 "$TEST_DIR$TEST_DB_FILE" < "$DB_FILE"
	
	sqlite3 "$TEST_DIR$TEST_DB_FILE" "INSERT INTO waypoints VALUES(1,2.2,3.3,500,0,0);"
	sqlite3 "$TEST_DIR$TEST_DB_FILE" "INSERT INTO waypoint_stationary VALUES (1,1);"

	cd $START_DIR

	make

	if [ -f db_tests ]; then
		echo "Running db tests:"
		./db_tests
	else 
		echo "Missing db_tests."
	fi

else 
	echo "Missing createtables.sql"
fi