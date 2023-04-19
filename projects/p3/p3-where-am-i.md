[Home](../../README.md) | Previous: [Go Chase It!](../p2/p2-go-chase-it.md)

# Project 3: Where Am I?

__Figure 1: The Rearranged Environment__
!['Where Am I?' Animated GIF](./img/img2.png)

## Parameter Configuration

### AMCL

<table>
    <thead>
        <tr>
            <th>Type</th>
            <th>Parameter</th>
            <th>Value</th>
            <th>Explanation</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td rowspan=4>Overall Filter Parameters</td>
            <td><code>min_particles</code></td>
            <td><code>500</code></td>
            <td>To reduce computational burden once the robot is localised. [1]</td>
        </tr>
        <tr>
            <td><code>max_particles</code></td>
            <td><code>5000</code></td>
            <td>To give some weight to (random) alternate routes when robot is stuck.</td>
        </tr>
        <tr>
            <td><code>kld_err</code></td>
            <td><code>0.01</code></td>
            <td rowspan=2 style="text-align:left;vertical-align:top">
                Assume kernel density estimation of odometry data is fairly reliable. [1]
            </td>
        </tr>
            <td><code>kld_z</code></td>
            <td><code>0.99</code></td>
        </tr>
        <tr>
            <td rowspan=1>Laser Model Parameters</td>
        </tr>
        <tr>
            <td rowspan=1>Odometry Model Parameters</td>
            <td><code>odom_model_type</code></td>
            <td><code>diff</code></td>
            <td>Adopted for skid-steer robots [2].</td>
        </tr>
    </tbody>
</table>

### Move Base

__Figure 2: Global and Local Costmaps__
![](./img/mov6.gif)

## Resources

[1] Zheng, Kaiyu: "ROS Navigation Tuning Guide" (2019 revision)

[2] McLeod, Haidyn: "ROS Localization and Navigation using Gazebo" (2018)
