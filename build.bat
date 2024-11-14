mkdir build
cd build
cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
move planner.exe .. 
cd ..
rmdir build /s /q