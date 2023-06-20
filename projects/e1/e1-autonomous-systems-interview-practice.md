[Home](../../README.md) | Previous: [Home Service Robot](../p5/p5-home-service-robot.md)

# Elective 1: Autonomous Systems Interview Practice

- Path: __Perception/Sensor Engineer__

## Required Question

_Explain a recent project you've worked on. Why did you choose this project? What difficulties did you run into this project that you did not expect, and how did you solve them?_

As a requirement of Udacity's Robotics Software Engineer Nanodegree, I recently deployed a skid-steer robot that can autonomously fetch and deliver items in a Gazebo environment using ROS (Robot Operating System) and C++. The project combined SLAM (Simultaneous Localization and Mapping), navigation, and programming of custom ROS nodes, and was developed in Ubuntu Linux.

The project was not as hard as I initially foresaw, as it built on the experience I acquired from working on similar tasks previously. Nevertheless, it still posed some challenges.

The first one was designing a Gazebo world feature-rich enough for robust mapping and localization. In this regard, I decided to reproduce my real home, but this meant I could not rely on stock Gazebo models to populate the environment. All model CADs (computer-aided designs) had to be created from scratch, using the basic components Gazebo provides (cube, cylinder, sphere) and experimenting with colour as well as translation and rotation properties. I also relied on Blender for more complex models, an oval coffee table and wavy curtains. The whole process took around three days, but the outcome was realistic and very pleasing.

Another issue occurred during the mapping phase when loop closure in the 2d occupancy grid produced some distortion which meant several passes had to be made and for small details, a collage of multiple images had to be made.

_Follow-up: You mentioned your robot had a skid-steer design. Can you explain what this means, and how this setup differs from other available options?_

Sure!

## Perception/Sensor Engineer Questions

### Question 1

_What are some of the advantages & disadvantages of cameras, lidar and radar? What combination of these (and other sensors) would you use to ensure appropriate and accurate perception of the environment?_

### Question 2

_How do features from algorithms like SIFT, SURF and HOG differ? Explain how these algorithms work, and how you would use them within a perception pipeline._

### Question 3

__[Code]__ _Describe how a particle filter works, where it is useful, and how it performs against similar algorithms. Code an example of how you update the weights of the particles between steps._

[Home](../../README.md) | Previous: [Home Service Robot](../p5/p5-home-service-robot.md)