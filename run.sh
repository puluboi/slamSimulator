
/usr/bin/cmake --build ./build --config Debug --target all -j 18 --

export DISPLAY=:0

./build/slamSimulator

