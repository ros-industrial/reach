# Robot Reach Study
## Installation
Before building the Robot Reach Study package, install the following packages:
- PCL 1.8
- VTK 7.1
- vtk_viewer
  - `git clone https://raesgit.datasys.swri.edu/agoins/10.R8706.git`
- tool_path_planner
  - `git clone https://raesgit.datasys.swri.edu/agoins/10.R8706.git`

## Running Robot Reach Study
### Setup Steps
1. Create a valid MoveIt package for your robot
2. Create an STL mesh of the object you wish to study
3. Edit `robot.launch` to include the `planning_context.launch` file from your robot's MoveIt package
4. Edit `reach_study_object.yaml` to specify information about your reach study object

Run the setup launch file
- Specify the `robot` argument to be the name of the launch file you edited above
  - Defaults to `robot`
- Specify the `see_urdf` argument to toggle launching of RViz
  - Defaults to true

`roslaunch robot_reach_study setup.launch robot:=NAME_OF_YOUR_ROBOT_LAUNCH_FILE see_urdf:=true`

### Run Steps
Run the start launch file
- Specify the `reach_object` argument to be the name of the .yaml file you edited above
  - Defaults to `reach_study_object`
- Specify the `config_name` argument to be the name of the folder where the reach study results are saved
  - Creates a folder with that name within `robot_reach_study/output` to save reach study data
  - Must be unique between different reach studies
  - Defaults to `reach_study_object`

`roslaunch robot_reach_study start.launch reach_object:=NAME_OF_YOUR_YAML_FILE config_name:=NAME_OF_YOUR_OUTPUT_FOLDER`

### Example
`my_custom_robot.launch`

`my_custom_reach_object.yaml`

`roslaunch robot_reach_study setup.launch robot:=my_custom_robot`

`roslaunch robot_reach_study start.launch reach_object:=my_custom_reach_object config_name:=test_1`
