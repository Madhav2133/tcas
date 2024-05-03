# Traffic Management System

## Task-1:

To implement a node at the junctions/marked regions in a traffic system that monitors them and defines the movements of the robots in the environment.

[![Video](https://drive.google.com/file/d/1bNlPTUruVNoyypPp91DsQRQzbbuhFBAs/preview)](https://drive.google.com/file/d/1bNlPTUruVNoyypPp91DsQRQzbbuhFBAs/view)

### To use:

Firstly, create your ros workspace and clone the package into it's `src/` directory

```
git clone https://github.com/ReyeTech/TrafficManagementSystem.git
```

Now use `colcon build` and build both `tcas` and `nav2` packages.

```
colcon build --packages-select tcas
```

### Let's run the junction monitor node:

First source the workspace, if you haven't:

```
source ~/<your_workspace_dir>/install/setup.bash
```

Now run the launch file to start the monitoring node:

```
ros2 launch tcas tcas.launch.py
ros2 launch tcas lane.launch.py
```

## Note:

If you would like to define the parameters of the junction, go to the `config/junction_details.yaml`and `config/robot_details.yaml` and give the coordinates of the marked regions and the no of robots deployed along with their namespaces.
