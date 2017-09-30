# nao_whole_body_ik
A whole body IK (Inverse Kinematics) solver for the Nao robot. It allows the user to specify pose constraints for multiple end effectors simultaneously (e.g. both hands and feet simultaneously). This isn't a 'from-scratch' implementation, but rather a wrapper around KDL's Tree Velocity IK Solver along with a trick to bypass the need for seed states.

## Functionality Overview
Numerical IK solvers using Jacobian-based optimization require a starting point or seed state, and the choice of this seed state heavily influences the solver's ability to arrive at a solution. In this impementation, a good seed state is chosen automatically from a database containing hundreds of thousands of valid joint configurations by using K-nearest neighbor search to find joint configurations that result in end effector poses close to what the IK solver is being asked to solve for. This drastically improves the success rate of the IK solver, and eliminates the need for the user to provide any seed state. The only drawback is that the operation becomes more time consuming because of the additional computation required to perform the nearest neighbor search.

## Setup
This repository is a ROS package, so you need ROS to be able to use it. It was developed on ROS Indigo, but should work with Jade and Kinetic as well (untested). In addition to the standard ROS library, you also need `moveit`, which you can install by:

`sudo apt-get install ros-indigo-moveit`

And you'll also need the Nao robot's meshes:

`sudo apt-get install ros-indigo-nao-meshes`

Aside from these ROS dependencies, you'll also need the following packages:

* Flann (Fast Library for Nearest Neighbors Approximation)
* HDF5

To build this package, simply clone it into an existing ROS workspace and build as usual using `catkin_make`. Once the package is built, you'll first have to construct the databases before you can actually use the IK solver. You can do this by:

`roslaunch nao_whole_body_ik_solver generate_multifoot_db.launch`

This will take a while, approximately 20-40min. But don't worry, it only has to be done once. After the databases are generated, you can check if everything is in order by running the rostests. First build the tests by running `make tests` from inside the workspace's build directory, and then run the tests by:

`rostest nao_whole_body_ik run.test`

All the tests won't always succeed (the tests run the IK solver for randomly generated queries that sometimes fail, especially when database search is turned off), but the first few tests should always succeed.

Once you have everything up and running, you can start using the IK solver in your own code. Refer to the code in `example_main.cpp` to get an idea on how to use it.
