#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

print_result()
{
	if $1
	then 
		printf "$CLR_OK"
		printf "Success\n"
	else 
		printf "$CLR_ERR"
		printf "Failed\n"
	fi
}

printf "$CLR_INFO\nCreating database in $CLR_DIR$INSTALLATION_PATH$REPO_MAIN/$CLR_INFO\n"
	if sqlite3 asr.db < $DIR/createtables.sql;
	then print_result true; else print_result false; break; fi
	printf "$CLR_ASK\nServer settings:\n$CLR_OPT"
	read -p "Boat name: " BOATID
	read -p "Boat password: " BOATPWD
	read -p "Server address: " SRVADDR
	printf "$CLR_INFO"
	if sqlite3 asr.db "INSERT INTO server(id, boat_id, boat_pwd, srv_addr) VALUES('1', '$BOATID', '$BOATPWD', '$SRVADDR')";
		then print_result true;
		cp asr.db test_asr.db
	else print_result false; fi