#libraries
BOOST   = /usr/include/boost
EIGEN   = /usr/include/eigen
BSO     = /home/tessa/MSc_2023_Turkstra/experiments_code
GUI     = MSc_2023_Turkstra/experiments_code/freeglut/include/GL
ALL_LIB = -I$(BOOST) -I$(EIGEN) -I$(BSO) -I$(GUI)

#compiler settings
CPP   = g++ -g -std=c++17
FLAGS  = -march=native -lglut -lGL -lGLU -lpthread #-lfade2d
#D_LIB  = -I${FADE}/include_fade2d

#test suites
MAINFILE = human_approach.cpp

.PHONY: all cls clean

#make arguments
cls:
	# this argument clears the screen, so that it is easier to find the beginning of the compiler output
	@clear
	@clear
all:
	$(CPP) $(D_LIB) -o human_approach $(ALL_LIB) $(MAINFILE) $(FLAGS)
clean:
	@rm -f human_approach