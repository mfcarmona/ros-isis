#!/bin/bash
#

xterm -e java "$pwd"\server &
sleep 1
libCbrClient_Test
echo "-"
echo "Press enter to continue"
read
java client
echo "-"
echo "Press enter to continue"
read
libCbrClient_Test
echo "-"


