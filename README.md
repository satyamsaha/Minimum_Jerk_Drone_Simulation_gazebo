# PX4 Offboard Control with ROS

This repository provides instructions on setting up a ROS workspace for PX4 offboard control and includes example Python scripts for path generation and data saving.

## Setting Up the Environment

1. **Create Catkin Workspace:** Create a catkin workspace and clone the PX4 source code.

    ```bash
    mkdir -p ~/catkin_ws/src
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    ```

2. **Restart Your Computer:** Restart your computer after the installation completes.

## Creating ROS Package

3. **Navigate to Source Directory:** Navigate to the source directory of your catkin workspace.

    ```bash
    cd ~/catkin_ws/src
    ```

4. **Create ROS Package:** Create a new ROS package named `offboard_py` with `rospy` dependency.

    ```bash
    catkin_create_pkg offboard_py rospy
    ```

5. **Build the Package:** Build the new package.

    ```bash
    cd ..
    catkin build
    source devel/setup.bash
    ```

## Storing Python Files

6. **Create Scripts Folder:** Create a `scripts` folder in the package to store Python files.

    ```bash
    roscd offboard_py
    mkdir scripts
    cd scripts
    ```

7. **Copy Python Files:** Copy the following Python files to the `scripts` folder: `random_path4.py`, `data_saver_1.py`, and `path.py`.

## Creating Launch File

8. **Create Launch File:** Create a launch file named `start_offb.launch` for offboard control.

    ```bash
    ~/catkin_ws/src/offboard_py/src
    mkdir launch
    cd launch
    touch start_offb.launch
    ```

9. **Edit `start_offb.launch`:** Open `start_offb.launch` and copy the content from the provided `start_offb.launch` file.

## Running the System

10. **Launch Offboard Control:** Launch the offboard control.

    ```bash
    roslaunch offboard_py start_offb.launch
    ```

11. **Run Python Scripts:** In separate terminals, navigate to the `scripts` directory and run the Python scripts.

    ```bash
    cd ~/catkin_ws/src/offboard_py/scripts
    python3 path.py
    python3 data_saver_1.py
    ```

12. **Visualize the Path:** Open RViz and add the `path` topic to visualize the path.

---

**Note:** Make sure to modify the file paths and commands according to your system setup. For PX4 you can clone latest version.Also if requires then edit the package.xml file of the workspace.
