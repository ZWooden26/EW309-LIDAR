The following are each group members script for the Occupancy Grid Project:

1) Hernandez_LIDAR.py
2) mumaw_script.py
3) wooden_LIDAR.py
4) wooden_LIDAR.py is the final code created for submission.

The following is supplementary code to help with the final script:

reset_odom.py
The following are all items needed in order to create the Readme and Docsify page:

README.md
docsify-readme page
index.html
Hernandez Contributions: Was able to program the color sequence of area unexplored, free space, object detection, and the origin of the robot's reset of odometry. Created README. Created reset odom function. Organized Github.

Mumaw Contributions: Programmed the initial odometry subscription and Occupancy Grid publishing functionality. Helped create content with the Docsify page. Glue Guy

Wooden Contributions: Integrated color sequencing, time alignment, odometry subscription, and Occupancy Grid publishing.

Calendar of Objectives:

April 14th 
a) learned the basics of the occupancy grid: localization, code structure, width/height/resolution 
b) created new topic to publish occupancy grid 
c) learned how to structure the callback for the Laser Scan subscription 
d) project update: walked through code to make random detection (0-100) appear on the occupancy grid

April 15th 
a) created a basic working map, with LASER scan that can detect objects 
b) began to experiment with color coding for unknown area, free area, and objects detected 
c) began to work on integrating odometry with LIDAR to not rotate map

April 18th a) learned the basics of time alignment between the Scan message and the odom message. (Allows for correctly updated Grid) b) created map that does not rotate with its yaw axis movement c) working on integrating Grid colors with correct odom/scan alignment

April 21st a) finished time alignment b) refining the integration of grid colors

April 22nd a) final demonstration of full functionality. Learned the art of docsify and requirements for submission

April 25th a) Still working to get time alignment. Started to work more on organizing on the Docsify page

April 28th a) Still working on code. We realized the issue is with publishing free space from the robot to the object detected. Fixing that error and finishing Docsify page.

April 29th a) Completed Code. Organizing Github and the rest of the README. Making final Commits with updated files.
