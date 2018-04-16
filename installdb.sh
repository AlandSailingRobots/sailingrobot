#!/bin/sh
set -e

DIR=$(cd "$(dirname "$0")" && pwd)
DBFILE="$DIR/asr.db"
BOATID=$1

print_result()
{
	if $1; then
		printf 'Success\n'
	else
		printf 'Failed\n'
	fi
}

if [ -f "$DBFILE" ]; then
	rm -v "$DBFILE";
fi

case "$BOATID" in
	ASPire)
		#printf "$CLR_INFO\nCreating database ASPire in $CLR_DIR$INSTALLATION_PATH$REPO_MAIN/$CLR_INFO\n"
		printf 'Creating database for ASPire in %s\n' "$DBFILE"
		if sqlite3 "$DBFILE" < "$DIR/createtablesASPire.sql"; then
			print_result true
		else
			print_result false
		fi
		;;
	Janet)
		#printf "$CLR_INFO\nCreating database Janet in $CLR_DIR$INSTALLATION_PATH$REPO_MAIN/$CLR_INFO\n"
		printf 'Creating database for Janet in %s\n' "$DBFILE"
		if sqlite3 "$DBFILE" < "$DIR/createtablesJanet.sql"; then
			print_result true
		else
			print_result false
		fi
		;;
	*)
		printf "No arguments to create the DataBase: \\nPlease insert 'ASPire' or 'Janet' as arguments to create the DB.\\n"
		exit 1
esac

printf 'Server connection HTTPsync configuration\n'
printf 'Example server adresses are https://sailingrobots.ax/aspire/sync/ or http://localhost/aspire/sync\n'
printf 'Enter server address (URL): '
read -r SRVADDR
printf 'Example boat password is something like aspirepassword123\n'
printf 'Enter boat password: '
read -r BOATPWD

#"config_httpsync": {
#  "loop_time": 0.5,
#  "remove_logs": 0,
#  "push_only_latest_logs": 1,
#  "boat_id": "ASPire",
#  "boat_pwd": "aspirepassword123",
#  "srv_addr": "https://sailingrobots.ax/aspire/sync/",
#  "configs_updated": "0",
#  "route_updated":"0"
#},

printf 'Storing configuration into %s\n' "$DBFILE"
if sqlite3 "$DBFILE" "INSERT INTO config_httpsync(id, loop_time, remove_logs, push_only_latest_logs, boat_id, boat_pwd, srv_addr) VALUES('1', '0.5', '0', '0', '$BOATID', '$BOATPWD', '$SRVADDR')"; then
	print_result true
else
	print_result false
fi
# printf "$CLR_INFO"
# if sqlite3 asr.db "INSERT INTO config_httpsync(id, loop_time, remove_logs, push_only_latest_logs, boat_id, boat_pwd, srv_addr) VALUES('1', '0.5', '0', '0', '$BOATID', '$BOATPWD', '$SRVADDR')";
# 	then print_result true;
# 	cp asr.db test_asr.db
# else print_result false; fi
