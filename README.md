# AKD Ethercat Master

## Building
```
mkdir build
cd build
cmake ..
make
```
Debug mode will print more data to stdout. If you would like to build in debug mode, replace the cmake command with : 
```
cmake -DDEBUG_MODE=ON ..
```

# Using for ROS

If you'd like to use this in a ROS enviroment without having to install the library on the system, I wrote a [ROS wrapper](https://github.com/AJBuilder/AKD-Ethercat-Master) for the library.

Also, a [ROS example](https://github.com/AJBuilder/joystick_ecat_demo).