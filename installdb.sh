#!/bin/sh
set -e

DIR=$(cd "$(dirname "$0")" && pwd)
DBFILE="$DIR/asr.db"
BOATID=$1

print_result() {
	if $1; then
		printf 'Success\n'
	else
		printf 'Failed\n'
	fi
}

if [ -f "$DBFILE" ]; then
	rm -v "$DBFILE"
fi

case "$BOATID" in
ASPire)
	printf 'Creating database for ASPire in %s\n' "$DBFILE"
	if sqlite3 "$DBFILE" <"$DIR/createtablesASPire.sql"; then
		print_result true
	else
		print_result false
	fi
	;;
Janet)
	printf 'Creating database for Janet in %s\n' "$DBFILE"
	if sqlite3 "$DBFILE" <"$DIR/createtablesJanet.sql"; then
		print_result true
	else
		print_result false
	fi
	;;
Velvet)
	printf 'Creating database for Velvet in %s\n' "$DBFILE"
	if sqlite3 "$DBFILE" <"$DIR/createtablesVelvet.sql"; then
		print_result true
	else
		print_result false
	fi
	;;
*)
	printf "ERROR: No arguments to create the DataBase!\\n"
	printf "Please insert 'ASPire', 'Janet' or 'Velvet' as arguments to create the DB.\\n"
	exit 1
	;;
esac

printf "Server connection HTTPsync configuration\\n"
printf "Example server adresses are https://sailingrobots.ax/aspire/sync/"
printf " or http://localhost/sync/\\n"
printf 'Enter server address (URL): '
read -r SRVADDR
printf 'Enter boat password: '
read -r BOATPWD

printf 'Storing configuration into %s\n' "$DBFILE"
if sqlite3 "$DBFILE" "INSERT INTO config_httpsync(id, loop_time, remove_logs, push_only_latest_logs, boat_id, boat_pwd, srv_addr) VALUES('1', '0.5', '0', '1', '$BOATID', '$BOATPWD', '$SRVADDR')"
then
	print_result true
else
	print_result false
fi
