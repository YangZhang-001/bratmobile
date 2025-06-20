#!/bin/bash
export TIME="time result\ncmd:%C\nreal %es\nuser %Us \nsys  %Ss \nmemory:%MKB \ncpu %P"
temp=$(head -n 1 /sys/class/thermal/thermal_zone0/temp)

# if [ $temp -lt 80000 ]
# then
# 	echo "temperature of $temp ok, building"
	sudo rm target targetless test/closed_loop_task CMakeCache.txt Makefile
	sudo rm /tmp/graph*
	cmake .
	cd src/
	sudo make install
	cd ..
	make

# else
# 	echo "too hot! temp = $temp , not building"
# fi
