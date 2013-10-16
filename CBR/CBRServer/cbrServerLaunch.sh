if [ $# -lt 1 ]; then
	echo "USAGE: $0 cbr_database"
	exit 1
else
	PWDDIR=$(pwd)
	CONSOLE="xterm -bg BLACK -fg WHITE -e "
	COMMAND="java cbr_server $PWDDIR/$1"
	echo "$CONSOLE$COMMAND &"
	$CONSOLE$COMMAND &
fi
