INC=-I./include

all: planner
planner: src/planner.cpp
	g++ -o planner $(INC) -O3 src/planner.cpp src/runtest_ARA.cpp src/planner_ARA.cpp src/planner_ADstar.cpp src/runtest_ADstar.cpp
debug:
	g++ -o planner $(INC) -g src/planner.cpp src/runtest_ARA.cpp src/planner_ARA.cpp src/planner_ADstar.cpp src/runtest_ADstar.cpp

clean:
	rm planner
