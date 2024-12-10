INC=-I./include

all: planner
planner: src/planner.cpp
	g++ -o planner $(INC) -O3 src/planner.cpp src/planner_best_anytime.cpp
debug:
	g++ -o planner $(INC) -g src/planner.cpp src/planner_best_anytime.cpp

clean:
	rm planner
