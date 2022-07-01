# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
 
# ****************************************************
# Targets needed to bring the executable up to date
 
build: kine_test.o kinematics.o
    $(CC) $(CFLAGS) -o run kine_test.o kinematics.o
 
# The main.o target can be written more simply
 
kine_test.o: kine_test.cpp 
    $(CC) $(CFLAGS) -c kine_test.cpp

kinematics.o: kinematics.cpp
	$(CC) $(CFLAGS) -c .\walk_test\kinematics.cpp
