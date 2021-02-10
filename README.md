# **Graph-SLAM implementation**

The following is a documented presentation of a Graph-SLAM implementation based on the course ["Mobile Sensing and Robotics 2"](https://www.ipb.uni-bonn.de/msr2-2020/) given by Cyrill Stachniss at the University of Bonn. The dataset used for in this example has been provided in the same course. A formal theoretical explanation can be found in the [relative paper](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf). This aims to be a more informal approach to the same problem. 

## **Why SLAM?**

SLAM stands for **S**imultaneous **L**ocalization **A**nd **M**apping. Once a robot is placed in a new environment it needs to localize itself and create a map of the surrounding (useful for performing future activities such as path planning). 

Usually SLAM algorithms are used in scenarios where the pose and the map of the robot is not known. The only information available are the odometry measurements $$u$$ (for example an encoder attached to the motor axis)

