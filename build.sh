mkdir build_2
cd build_2
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4
mv planner ..
cd ..
rm -r build_2/