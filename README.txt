Project for Ubuntu 20.04 LTS using ROS noetic distribution.

A common problem in robotics is locating objects and arranging them in some manner. In most cases, this is accomplished using an overhead camera that provides a “God’s eye” view, i.e., a view of an entire area. In this project, we are attempting a variation of this problem by removing the overhead camera and relying solely on a camera attached to the mobile robot that only provides a partial view of the area. In order to test our approach, our team’s goal is to use a TurtleBot3 Burger robot to rearrange several blocks into a specified order. To accomplish this task, we will assemble the mobile robot and modify it to suit the problem. The overall approach has the robot searching a designated area for blocks in a back and forth search pattern. As each block is found, the block will be moved to a staging area for further processing. After scanning the entirety of the area, the robot will identify each block in the staging area, then move the blocks to the desired order.

Repo's Structure:
Documentation - stores the various image, docx, txt, and pptx files pertaining to the details of this project.

Robot - contains the ros structure for the TurtleBot3 Burger's Raspberry Pi 4.

computer - contains the ros structure for the user's own computer. It also stores the launchable files in order to move the TurtleBot and the training images used in some of the computer vision software.
