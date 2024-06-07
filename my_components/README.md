# Checkpoint 10


## Task1 compose your node

Inside the repository checkpoint9, create a new branch named composition.
In this new branch, add a new ROS2 package named my_components.
Create a new component named PreApproach that replicates the behavior of the pre_approach.cpp node you created in Checkpoint 9. This is, it moves the robot in front of the shelf, facing it (see image below).
For this component, you can remove the use of the parameters obstacle and degrees and use hardcoded values

how to run
Terminal1
```
ros2 run rclcpp_components component_container
```
Terminal2
```
ros2 component load /ComponentManager my_components my_components::PreApproach
```

## Task 2   Final Approach 

In this 2nd Task of the project, you will create 2 new components, a server and a client, that will allow the robot to attach to the shelf.

- Create a new component named AttachServer. This component will contain the service server /approach_shelf that you created for Checkpoint 9. So, when called, the server will start the final approach behavior to make the robot attach to the shelf.
- For the AttachServer component, use the manual-composition approach.
- Create a new component named AttachClient. This component will contain a service client that will call to the /approach_shelf service so that the robot starts the final approach.
- For the AttachClient component, use the run-time composition approach.
- The service will use the same interface GoToLoading.srv that you created for Checkpoint 9.
- For these components, you can remove the use of the final_approach parameter.
- Create a launch file named attach_to_shelf.launch.py. This launch file will start a container named my_container and will load the following 2 components:
    - PreApproach with name pre_approach
    - AttachServer with name attach_server

how to run
```
ros2 launch my_components attach_to_shelf.launch.py
```
to load the cart,
```
ros2 component load /my_container my_components my_components::AttachClient
```