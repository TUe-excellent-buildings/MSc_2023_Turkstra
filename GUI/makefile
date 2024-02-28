# Makefile for GUI_Janneke
#libraries
BOOST   = /usr/include/boost
EIGEN   = /usr/include/eigen
BSO     = /home/kknopp/Documents/MSc_2023_Turkstra

ALL_LIB = -I$(BOOST) -I$(EIGEN) -I$(BSO)

# Compiler settings
CPP   = g++ -std=c++17
FLAGS  = -O3 -lglut -lGL -lGLU

MAINFILE = GUI_Jildert.cpp

.PHONY: all cls clean

#make arguments
cls:
	# this argument clears the screen, so that it is easier to find the beginning of the compiler output
	@clear
	@clear
all:
	$(CPP) -o GUI_Jildert $(ALL_LIB) $(MAINFILE) $(FLAGS)
clean:
	@rm -f get_results
