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
	if [ -f "./asr.db" ]
	then
		rm asr.db;
	fi;

	if [ $1 == ASPire ]
	then
		printf "$CLR_INFO\nCreating database ASPire in $CLR_DIR$INSTALLATION_PATH$REPO_MAIN/$CLR_INFO\n"
		if sqlite3 asr.db < $DIR/createtablesASPire.sql;
		then print_result true; else print_result false; break; fi
	elif [ $1 == Janet ]
	then
		printf "$CLR_INFO\nCreating database Janet in $CLR_DIR$INSTALLATION_PATH$REPO_MAIN/$CLR_INFO\n"
		if sqlite3 asr.db < $DIR/createtablesJanet.sql;
		then print_result true; else print_result false; break; fi
	else
		printf "No arguments to create the DataBase: \nPlease insert 'ASPire' or 'Janet' as arguments to create the DB.\n"
	fi;

	#
	# read -p "Boat password: " BOATPWD
	# read -p "Server address: " SRVADDR
	# printf "$CLR_INFO"
	# if sqlite3 asr.db "INSERT INTO config_httpsync(id, loop_time, remove_logs, push_only_latest_logs, boat_id, boat_pwd, srv_addr) VALUES('1', '0.5', '0', '0', '$BOATID', '$BOATPWD', '$SRVADDR')";
	# 	then print_result true;
	# 	cp asr.db test_asr.db
	# else print_result false; fi
