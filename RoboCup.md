# Creating maps

[Resource 1 (somewhat outdated)](https://github.com/AustinVillaatHome/Robocup_Integrated_Systems/blob/master/Robocup_Competition_Doc.md#1-before-the-competitionsetup-days)

 - (Optional) If you wish to see the camera feed before/during map creation run this command: `rqt_image_view /hsrbb/head_rgbd_sensor/rgb/image_raw` **On the backpack**
 - **On the backpack** 
  ```
  cd ~/villa_tools/scripts
  ./make_map
  ```
   - Use keyboard to teleoperate the robot. Basic controls: `i` for forward, `,` for backward, `j` and `l` to rotate left and right respectively
   - Press ctrl+c when done
   - > Make sure to stay away from the front of the robot. Good mapping procedure is only scanning an area once, and do not do any back tracking. Once finished Ctrl+C. It will ask for the name of the folder where the map will be placed. Try to be descriptive with name (Don't use "new", "final", "final_final." use the date, "v1", "v2" instead).
   - New map created in ~/maps/
 - Copy and paste this map in your workspace `~/workspaces/<your_ws>/src/villa_common/villa_maps/maps/real/<map_name>`
 - Now, we need to annotate the map for creating knowledge base. Follow steps in: [Good resource](https://github.com/utexas-bwi/knowledge_representation/blob/master/doc/annotating_maps.md)
 - Moving the map (non-annotated) to the robot:
   - ```
     cd ~/maps/
     rsync -havz ./<your_map_name> austin@austin-hsrb:/home/austin/maps/<your_map_name>
     ```
 - Choosing the new map on the robot:
   - Make sure the robot is turned on, and connected to the laptop (terminal should show `aus up`). Then do `ssh_robot_admin` to SSH into the robot.
   - **On the HSRB** Run `configure_map ~/maps/<your_map_name>`. This will ask for sudo password. Find it at: [Robot logins](https://github.com/AustinVillaatHome/documentation/wiki/Accounts)
 - **On the backpack** Create the knowledge database using: `prepare_knowledge_villa <knowledge_folder> <maps_folder>`
