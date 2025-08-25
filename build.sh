#!/bin/bash
sudo rm target targetless test/closed_loop_task CMakeCache.txt Makefile
sudo rm /tmp/graph*
cmake .
cd src/
sudo make install
cd ..
make
