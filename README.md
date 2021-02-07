# Vehicle Dynamics Observer

Vehicle Dynamics Observer is a Library which implements a weighted difference observer 
using the dynamical bicycle model.


## Release Notes
* Completed:
    * Vehicle Dynamics Observer Base Class
* TODO:
    * Testing

## Getting Started
These instructions will help you generate the necessary documentation for using this package, and list the required dependencies.

### Documentation

The documentation for this project is Doxygen based. To generate, execute the following commands:

```
cd <path>/vehicle_dynamics
doxygen Doxyfile
```

### Dependencies

The follwing dependencies are required, and can be installed accordingly.

```
sudo apt install doxygen
sudo apt install libgtest-dev
sudo apt install build-essential
sudo apt install python-catkin-tools
sudo apt install ros-noetic-desktop-full (Includes required Eigen3 library)

```
## Running the tests

To compile unit and pipeline tests, use the following command:
```
catkin build vehicle_dynamics --no-deps --catkin-make-args run_tests
```

### Break down into end to end tests

The Vehicle Dynamics Observer test verifies basic functionality of the Vehicle Dynamics Observer base class.   

```
veh_dyn_observer_test.cpp
```

## Built With

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - Build tool used for compiling this project
* [Google Test](https://github.com/google/googletest) - Unit testing framework
* [ros_noetic](http://wiki.ros.org/noetic) - Open source meta-operating system


## Authors

* **Ryan Shedlock**

## License

This project is licensed under the MIT License - see the LICENSE file for details
