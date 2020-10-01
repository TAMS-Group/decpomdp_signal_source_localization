# Signal source localization Application
This application is designed to locate a Wi-Fi signal source using dec-POMDP and a non-linear Policy Graph improvement algorithm.
It was designed as part of the Bachelor thesis: "Locating the source of a WLAN signal using multiple robots and Dec-POMDP planning" by Tobias Krüger

## Acknowledgements 
We use the [NPGI](https://github.com/laurimi/npgi/) algorithm by Mikko Lauri in the dec_pomdp_algorithm package.
We use the [tams_turtlebot](https://github.com/TAMS-Group/tams_turtlebot) package in order to use TurtleBot2s in physical experiments with this system
We use use the [move_base_stub](https://github.com/TAMS-Group/move_base_stub) package to simulate robots in all simulated experiments

## Compilation
The application can be build using the [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) tool.

## Get Started
This system consists out of 5 packages. The easiest way to try out its capabilities is to call 
```
roslaunch dec_pomdp_simulation simulation.launch
```

This command will start two client instances, one server instance, and one planning algorithm instance on the current machine.
With the command 
```
rosservice call /record_experiments [number]
```
we can instruct the running system to conduct [number] experiments in simulation.


To conduct physical experiments we have to deploy the system to a robot. This can be done by modifying and executing the 'deploy.sh' script.
Afterwards we need to start a client node on each robot through:
```
roslaunch dec_pomdp_client client.launch
```
or
```
roslaunch dec_pomdp_client random_movement.launch
```
This node assumes that there is already some process running that enables movement via the [move_base](http://wiki.ros.org/move_base) package. For the TurtleBot2s we achieve this by running:
```
roslaunch tams_turtlebot_bringup tams_turtlebot.launch
```

Additionally, we need to start the server-side components:
```
roslaunch dec_pomdp_server server.launch
```
and 
```
roslaunch dec_pomdp_algorithm dec_pomdp.launch
```
To ensure all of these processes can run correctly, we need to check the launch files 'server.launch' and 'client.launch' to ensure the robot hosts are configured correctly.

If all of these processes are running, we can call: 
```
rosservice call /start_experiment [?random_movement?] [?simulate_measurements?]
```
This command will start a full experiment with the two robots.