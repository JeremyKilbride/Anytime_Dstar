mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4
mv planner ..
cd ..
rm -r build/