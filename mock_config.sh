printf "\n"
read -p "Enter mock settings (1/0): " MOCK

printf "Enter xBee settings (1/0)\n"
read -p "	send: " SEND
read -p "	receive: " RECV

sqlite3 asr.db "UPDATE mock SET 
	GPS = $MOCK,
	Windsensor = $MOCK, 
	Compass = $MOCK, 
	Position = $MOCK, 
	Maestro = $MOCK;

	UPDATE configs SET xb_send = $SEND,	xb_recv = $RECV;"

printf "DONE\n\n"