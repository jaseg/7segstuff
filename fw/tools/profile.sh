#!/bin/sh
NLOOPS=1000
SLEEP=0.1

[ $# -lt 1 ] && echo "ERROR: Not enough arguments" && exit 2
elffile="$1"

trap "exit" SIGINT

logfile=$(mktemp)

arm-none-eabi-gdb -x profile.gdb "$elffile" > "$logfile" 2>/dev/null&
gdbpid=$!
trap "kill -TERM $gdbpid; rm $logfile" EXIT
echo "Gathering..."
for i in $(seq 1 $NLOOPS); do
    echo "$i/$NLOOPS"
    kill -INT $gdbpid
    sleep $SLEEP
done
kill -TERM $gdbpid
trap "rm '$logfile'" EXIT

egrep -o '\w+ \(.*\) at .*' "$logfile" |cut -d' ' -f1|sort|uniq -c|sort -n
echo 'Total:' $(egrep -c '\w+ \(.*\) at .*' "$logfile")

