#!/bin/bash
# simple script to obtain data over usart from stm32 board (for Linux)

DEV="/dev/ttyACM0"   # update to where your board mounted
BAUDRATE=115200

mydev=`ls $DEV 2>&1 >/dev/null`
if [[ $? != 0 ]]; then
  printf "\n\tSpecified device [$DEV] does not exist, possible options are:  "
  devs=`ls /dev/*ACM*`
  printf "[$devs]\n\n"
  exit
fi

printf "\n-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- \n"
stty -F $DEV ispeed $BAUDRATE cs8 -cstopb -parenb cread clocal -icanon -echo raw
while true
do
  while read -r line
  do
    echo $line
  done
done < $DEV
