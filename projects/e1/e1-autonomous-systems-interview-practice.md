[Home](../../README.md) | Previous: [Home Service Robot](../p5/p5-home-service-robot.md)

# Elective 1: Autonomous Systems Interview Practice

- Path: __Perception/Sensor Engineer__

## Required Question

_Explain a recent project you've worked on. Why did you choose this project? What difficulties did you run into this project that you did not expect, and how did you solve them?_

As a requirement of Udacity's Robotics Software Engineer Nanodegree, I recently deployed a skid-steer robot that can autonomously fetch and deliver items in a Gazebo environment using ROS (Robot Operating System) and C++. The project combined SLAM (Simultaneous Localization and Mapping), navigation, and programming of custom ROS nodes, and was developed in Ubuntu Linux.

While the project itself was not as hard as originally expected thanks to the experience I had gained from working on similar tasks, it still posed some challenges.

The first one was designing a Gazebo world feature-rich enough for robust mapping and localization. In this regard, I decided to reproduce my real home, but this meant I could not rely on stock Gazebo models to populate the environment. All CADs (computer-aided designs) had to be created from scratch, using the basic components Gazebo provides (cube, cylinder, sphere) and experimenting with colour as well as translation and rotation properties. For more complex models (wavy curtains and an oval coffee table) I relied on Blender. The whole process was very time-consuming, but the outcome was realistic and very pleasing.

Another issue occurred while mapping the environment via SLAM. Even after fine-tuning the parameter values to reduce distortion at loop closure, sketching table and chair legs in the 2D occupancy grid proved surprisingly difficult to do in one go, since additional passes in the same spots tended to wipe out at least some of the marks left by previous scans. For this reason, for a small portion of the final map I had to take snapshots from various angles through repeated attempts, and then collate the frames in post-processing.

Third, 

 while testing the navigation stack after configuration.

Finally, communication between publisher and subscriber (via odometry distance).

#### Follow-up

_You mentioned your robot had a skid-steer design. Can you explain what this means, and how this setup differs from other available options?_

Sure!

## Perception/Sensor Engineer Questions

### Question 1

_What are some of the advantages & disadvantages of cameras, lidar and radar? What combination of these (and other sensors) would you use to ensure appropriate and accurate perception of the environment?_

### Question 2

_How do features from algorithms like SIFT, SURF and HOG differ? Explain how these algorithms work, and how you would use them within a perception pipeline._

### Question 3

__[Code]__ _Describe how a particle filter works, where it is useful, and how it performs against similar algorithms. Code an example of how you update the weights of the particles between steps._

[Home](../../README.md) | Previous: [Home Service Robot](../p5/p5-home-service-robot.md)
