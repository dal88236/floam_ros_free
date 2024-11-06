echo "Building ROS2 nodes"

cd ros_wrapper
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
cd ../..