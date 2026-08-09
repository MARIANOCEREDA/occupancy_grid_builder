cd /home/dev/ws/src/external/gtsam
cmake -S . -B build
cmake --build build --target check
cmake --build build --target install