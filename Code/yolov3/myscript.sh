#!/bin/bash

while [ TRUE ]
do
ts=$(date +%s%N) ; python detect.py --image ./inputimage/camera.jpg ; tt=$((($(date +%s%N) - $ts)/1000000)) ; echo "Time taken: $tt milliseconds"
done

