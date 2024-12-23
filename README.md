# Anytime and Incremental Algorithms for Unknown Environments
Authors: Evan Wassmann, Trevor Stack, Jeremy Kilbride

This Repository is for our final project in CMU 16-782 Robot Planning and Decision Making. For our project we implemented Anytime D*, Anytime Repairing A*, and D* lite for efficiently re-planning in unknown environments. References for these algorithms are below  
  \
  \
[1] M. Likhachev, G. J. Gordon, and S. Thrun, “ARA* : Anytime A* with Provable Bounds on Sub-Optimality,” in Advances in Neural Information Processing Systems, MIT Press, 2003. Accessed: Oct. 30, 2024. [Online]. Available: https://proceedings.neurips.cc/paper_files/paper/2003/hash/ee8fe9093fbbb687bef15a38facc44d2-Abstract.html

[2] Koenig, Sven & Likhachev, Maxim. (2002). D*Lite.. Proceedings of the National Conference on Artificial Intelligence. 476-483. 

[3] Likhachev, Maxim & Ferguson, David & Gordon, Geoffrey & Stentz, Anthony & Thrun, Sebastian. (2005). Anytime Dynamic A*: An Anytime, Replanning Algorithm.. Proceedings of the International Conference on Automated Planning and Scheduling (ICAPS). 262-271. 

## Prequisites
Your system needs to have the following to build and run our code 
### Windows
- Cmake
- Visual Studio Build tools- can be downloaded here: https://visualstudio.microsoft.com/downloads/?q=build+tools#build-tools-for-visual-studio-2022
### Linux
- make
- Cmake
- gcc

install with `sudo apt install make cmake gcc`
## Building
To build our code do the following: 

### Windows (Method 1: CMake + Build Script)
All you need to do to build is open the developer command prompt and navigate to the top directory for this project. Then just enter the command `build.bat`.
### Linux/WSL (Method 1: CMake + Build Script)
In the terminal, navigate to the top directory for this project and build with `./build.sh`.

Both of these scripts will build the code, move the executable to the top level directory, and then delete the build directory.

### Linux/WSL (Method 2: Minimal MakeFile)
In the top level directory simply give the command `make` to compile the program. To compile in debug mode give the command `make debug`. This will compile without any build artifacts.

### Windows (Method 3: Compiling Manually)
Open the developer command prompt, navigate to the top level directory, and enter the command `cl /O2 src\planner.cpp /I include src\runtest_ARA.cpp src\planner_ARA.cpp`
### Linux/WSL (Method 3: Compiling Manually)
In the top level directory enter the command `g++ -o planner -I./include -O3 src/planner.cpp src/planner_ARA.cpp`

##
Maps were created by the authors or adapted from homework 1 of CMU course 16782. 

### Sensor Code Explained - Trevor

link:
https://www.dropbox.com/scl/fi/jyzpkod0nezagdnlnw4kx/Sensor_Code_explained-Made-with-Clipchamp_1732822912775.mp4?rlkey=lxnvx3aj93zlnquhwwq1oep9o&st=cg3eo9l0&dl=0
