# gazebo_sfm_plugin
A plugin for simulation of human pedestrians in ROS2 and Gazebo.

**Tested in ROS2 Galactic and Gazebo Garden**

The persons are affected by the obstacles and other persons using the [Social Force Model](https://github.com/robotics-upo/lightsfm)


![](media/images/capture3.jpg)


## Plugin configuration

The plugin can be applied to each Gazebo Actor indicated in the Gazebo world file.

An example snippet is shown next:

```html
<include>
    <pose>-1 2 1.25 0 0 0</pose>
    <name>actor1</name>
    <uri>https://fuel.gazebosim.org/1.0/Mingfei/models/actor_walk</uri>

    <plugin name="gz::sim::systems::PedestrianSFMPlugin"
        filename="libPedestrianSFMPlugin.so">
        <velocity>0.9</velocity>
        <radius>0.4</radius>
        <animation_factor>6.0</animation_factor>
        <people_distance>6.0</people_distance>
        <!-- weights -->
        <goal_weight>2.0</goal_weight>
        <obstacle_weight>80.0</obstacle_weight>
        <social_weight>15</social_weight>
        <group_gaze_weight>3.0</group_gaze_weight>
        <group_coh_weight>2.0</group_coh_weight>
        <group_rep_weight>1.0</group_rep_weight>
        <ignore_obstacles>
            <model>cafe</model>
            <model>ground_plane</model>
        </ignore_obstacles>
        <trajectory>
            <cyclic>true</cyclic>
            <waypoint>-1 2 1.25</waypoint>
            <waypoint>-1 -8 1.25</waypoint>
        </trajectory>
    </plugin>
</include>
```
The parameters that can be configured for each pedestrian are:

### General params

* ```<velocity>```. Maximum velocity (*m/s*) of the pedestrian.
* ```<radius>```. Approximate radius of the pedestrian's body (m).
* ```<animation_factor>```. Factor employed to coordinate the animation with the walking velocity.
* ```<people_distance>```.  Maximum detection distance of the surrounding pedestrians.

### SFM Weights

*  The weight factors that modify the navigation behavior. See the [Social Force Model](https://github.com/robotics-upo/lightsfm) for further information.

### Obstacle params

* ```<ignore_obstacles>```.  All the models that must be ignored as obstacles, must be indicated here. The other actors in the world are included automatically.

### Trajectory params

* ```<trajectory>```. The list of waypoints that the actor must reach must be indicated here.

	- ```<waypoint>```. Each waypoint must be indicated by its coordinates X, Y, Z in the world frame.
	- ```<cyclic>```. If true, the actor will start the waypoint sequence when the last waypoint is reached.

## Dependencies

* Yo must download and install the Social Force Model library, lightsfm https://github.com/robotics-upo/lightsfm

## Compilation

* This is a ROS2 package so it must be placed inside a ROS2 workspace and compiled through the regular colcon compiler.
```sh
colcon build --packages-select gazebo_sfm_plugin
```

## Example

An example Gazebo world can be launched through:
```sh
ros2 launch gazebo_sfm_plugin cafe_ros2.launch.py
```

