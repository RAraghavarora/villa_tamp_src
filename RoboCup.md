# Creating maps

[Resource 1 (somewhat outdated)](https://github.com/AustinVillaatHome/Robocup_Integrated_Systems/blob/master/Robocup_Competition_Doc.md#1-before-the-competitionsetup-days)

 - (Optional) If you wish to see the camera feed before/during map creation, run this command: `rqt_image_view /hsrbb/head_rgbd_sensor/rgb/image_raw` **On the backpack**
 - **On the backpack** 
  ```
  cd ~/villa_tools/scripts
  ./make_map
  ```
   - Use the keyboard to teleoperate the robot. Basic controls: `i` for forward, `,` for backward, `j` and `l` to rotate left and right, respectively
   - Press ctrl+c when done
   - > Make sure to stay away from the front of the robot. Good mapping procedure is only scanning an area once, and do not do any back tracking. Once finished Ctrl+C. It will ask for the name of the folder where the map will be placed. Try to be descriptive with name (Don't use "new", "final", "final_final." use the date, "v1", "v2" instead).
   - New map created in ~/maps/
 - Copy and paste this map in your workspace `~/workspaces/<your_ws>/src/villa_common/villa_maps/maps/real/<map_name>/<map_name>` (there are 2 folders with map_name)
 - Now, we need to annotate the map for creating a knowledge base. Follow steps in: [Good resource](https://github.com/utexas-bwi/knowledge_representation/blob/master/doc/annotating_maps.md)
 - Moving the map (non-annotated) to the robot:
   - ```
     cd ~/maps/
     rsync -havz ./<your_map_name> austin@austin-hsrb.local:/home/austin/maps/<your_map_name>
     ```
 - Choosing the new map on the robot:
   - Make sure the robot is turned on and connected to the laptop (terminal should show `aus up`). Then do `ssh_robot_admin` to SSH into the robot.
   - **On the HSRB** Run `configure_map ~/maps/<your_map_name>`. This will ask for the sudo password. Find it at: [Robot logins](https://github.com/AustinVillaatHome/documentation/wiki/Accounts)
 - **On the backpack** Create the knowledge database using: `prepare_knowledge_villa <knowledge_folder> <maps_folder>`
 - rosrun knowledge_representation populate_with_map \<dir\>

# Running a task

- After robot is running, and connected to the backpack, go to villa/villa_launch/config and run
`rviz -d villa/villa_launch/config villa_display.rviz`
- Localize the robot in rviz using '2d pose estimate' button
- Start necessary servers: [Check the documentation for villa_state_machines](https://github.com/AustinVillaatHome/villa_state_machines)
  - Dynamic Scene Graph
    ```
    conda activate ovir3d
    roscd dynamic_scene_graph
    python scripts/dynamic_scene_graph_server.py
    ```
  - Anygrasp
    ```
    conda activate anygrasp
    roscd anygrasp_ros
    python scripts/anygrasp_server.py
    ```
  - Text to speech
    ```
    roscd villa_speech
    python text_to_speech_server.py
    ```
- Run the necessary task: `rosrun pick_and_place storing_groceries`
