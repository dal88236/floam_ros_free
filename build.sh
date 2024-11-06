echo "Configuring and building F-LOAM..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
cd ..
