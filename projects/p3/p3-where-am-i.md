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
            <td rowspan=9>Overall Filter Parameters</td>
            <td><code>min_particles</code></td>
            <td><code>500</code></td>
            <td>To reduce computational burden once the robot is localised [1].</td>
        </tr>
        <tr>
            <td><code>max_particles</code></td>
            <td><code>5000</code></td>
            <td>To give some weight to (random) alternate routes when robot is stuck.</td>
        </tr>
        <tr>
            <td><code>kld_err</code></td>
            <td><code>0.01</code></td>
            <td rowspan=2>Assume kernel density estimation of odometry data is fairly reliable [1].</td>
        </tr>
        <tr>
            <td><code>kld_z</code></td>
            <td><code>0.99</code></td>
        </tr>
        <tr>
            <td><code>update_min_d</code></td>
            <td><code>0.05</code></td>
            <td rowspan=2>Minimum translational (5 cm) and rotational (π/24 radians) distances to travel before triggering a parameter update.</td>
        </tr>
        <tr>
            <td><code>update_min_a</code></td>
            <td><code>0.1309</code></td>
        </tr>
        <tr>
            <td><code>recovery_alpha_slow</code></td>
            <td><code>0.001</code></td>
            <td rowspan=2>Suggested values [3] to enable recovery mode.</td>
        </tr>
        <tr>
            <td><code>recovery_alpha_fast</code></td>
            <td><code>0.1</code></td>
        </tr>
        <tr>
            <td><code>initial_pose_a</code></td>
            <td><code>-1.570796</code></td>
            <td>Rotate initial pose mean (yaw) 90° clockwise (-π/2 radians) to align with map.</td>
        </tr>
        <tr>
            <td rowspan=5>Laser Model Parameters</td>
        </tr>
        <tr>
            <td><code>laser_max_beams</code></td>
            <td><code>120</code></td>
            <td>Empirically, this values gives the most stable results in terms of aligning laser scan with map landmark edges.</td>
        </tr>
        <tr>
            <td><code>laser_z_hit</code></td>
            <td><code>0.95</code></td>
            <td rowspan=2>Keep randomness small to assume low measurement noise [1].</td>
        </tr>
        <tr>
            <td><code>laser_z_rand</code></td>
            <td><code>0.05</code></td>
        </tr>
        <tr>
            <td><code>laser_likelihood_max_dist</code></td>
            <td><code>4.0</code></td>
            <td>Increase distance for obstacle inflation [1].</td>
        </tr>
        <tr>
            <td rowspan=5>Odometry Model Parameters</td>
            <td><code>odom_model_type</code></td>
            <td><code>diff</code></td>
            <td>Adopted for skid-steer robots [2].</td>
        </tr>
        <tr>
            <td><code>odom_alpha1</code></td>
            <td><code>0.005</code></td>
            <td rowspan=4>Reduce default <code>odom_alpha*</code> parameters to assume low noise in odometry data [1].</td>
        </tr>
        <tr>
            <td><code>odom_alpha2</code></td>
            <td><code>0.005</code></td>
        </tr>
        <tr>
            <td><code>odom_alpha3</code></td>
            <td><code>0.005</code></td>
        </tr>
        <tr>
            <td><code>odom_alpha4</code></td>
            <td><code>0.005</code></td>
        </tr>
    </tbody>
</table>

### Move Base

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
            <td rowspan=3>Costmap Common Parameters</td>
            <td><code>robot_radius</code></td>
            <td><code>0.3</code></td>
            <td>Enough to prevent bumping into corners while allowing for some degree of movement.</td>
        </tr>
        <tr>
            <td><code>inflation_radius</code></td>
            <td><code>1.75</code></td>
            <td rowspan=2>With this combination the robot tends to move in the middle of obstacles [1].</td>
        </tr>
        <tr>
            <td><code>cost_scaling_factor</code></td>
            <td><code>2.58</code></td>
        </tr>
        <tr>
            <td rowspan=2>Base Local Planner Parameters</td>
        </tr>
    </tbody>
</table>

__Figure 2: Global and Local Costmaps__
![](./img/mov6.gif)

## Resources

[1] Zheng, Kaiyu: "ROS Navigation Tuning Guide" (2019 revision)

[2] McLeod, Haidyn: "ROS Localization and Navigation using Gazebo" (2018)

[3] [AMCL (Noetic) Official Documentation](http://wiki.ros.org/amcl?distro=noetic)
