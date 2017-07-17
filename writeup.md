## Project: Kinematics Pick & Place

---
This is the write up for the Kinematics project. 
The jupyter notebook consists of the working and calculations
https://github.com/lisaljl/Udacity-RoboND-Kinematics/blob/master/code/Kinematics_Project_Test_Notebook.ipynb

Video for the working arm
https://youtu.be/B2PavVfN0Tg

#### DH Parameters for Kuka arm

The parameters is obtained using the following convention


![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/DH.png "DH annotation")



| i        | a(i-1)  | a(i-1)  | d(i)    | θ(i)    |
| :------- |:-------:|:-------:|:-------:|:-------:|
| 1        | 0       | 0       | d1      |θ1       |
| 2        | -90     | a1      | 0       |θ2 - 90  |
| 3        | 0       | a2      | 0       |θ3       |
| 4        | -90     | a3      | d4      |θ4       |
| 5        | 90      | 0       | 0       |θ5       |
| 6        | -90     | 0       | 0       |θ6       |
| G        | 0       | 0       | dG      |0        |


| i        | a(i-1)  | a(i-1)  | d(i)    | θ(i)    |
| :------- |:-------:|:-------:|:-------:|:-------:|
| 1        | 0       | 0       | 0.75    |θ1       |
| 2        | -90     | 0.35    | 0       |θ2 - 90  |
| 3        | 0       | 1.25    | 0       |θ3       |
| 4        | -90     | -0.054  | 1.5     |θ4       |
| 5        | 90      | 0       | 0       |θ5       |
| 6        | -90     | 0       | 0       |θ6       |
| G        | 0       | 0       | 0.303   |0        |


### Forward Kinematics

#### Transformation matrix for Kuka arm
More details on how the transformation matrices are obtained, please refer to the jupyter notebook
```
            T0_1 =  Matrix([
            [cos(q1), -sin(q1), 0,  0],
            [sin(q1),  cos(q1), 0,  0],
            [      0,        0, 1, d1],
            [      0,        0, 0,  1]])

            T1_2 =  Matrix([
            [sin(q2),  cos(q2), 0, a1],
            [      0,        0, 1,  0],
            [cos(q2), -sin(q2), 0,  0],
            [      0,        0, 0,  1]])

            T2_3 =  Matrix([
            [cos(q3), -sin(q3), 0, a2],
            [sin(q3),  cos(q3), 0,  0],
            [      0,        0, 1,  0],
            [      0,        0, 0,  1]])

            T3_4 =  Matrix([
            [ cos(q4), -sin(q4), 0, a3],
            [       0,        0, 1, d4],
            [-sin(q4), -cos(q4), 0,  0],
            [       0,        0, 0,  1]])

            T4_5 =  Matrix([
            [cos(q5), -sin(q5),  0, 0],
            [      0,        0, -1, 0],
            [sin(q5),  cos(q5),  0, 0],
            [      0,        0,  0, 1]])

            T5_6 =  Matrix([
            [ cos(q6), -sin(q6), 0, 0],
            [       0,        0, 1, 0],
            [-sin(q6), -cos(q6), 0, 0],
            [       0,        0, 0, 1]])

            T6_G =  Matrix([
            [0,  0, 1,  0],
            [0, -1, 0,  0],
            [1,  0, 0, dG],
            [0,  0, 0,  1]])

            
```
#### Transformation matrix for base_link to all links

```
     T0_2 = T0_1*T1_2 #base_link to link_2
     T0_3 = T0_2*T2_3 #link_2 to link_3
     T0_4 = T0_3*T3_4 #link_3 to link_4
     T0_5 = T0_4*T4_5 #link_4 to link_5
     T0_6 = T0_5*T5_6 #link_5 to link_6
     T0_G = simplify(T0_6*T6_G) #link_6 to link_G
```

#### Homogeneous transform matrix from base_link to gripper_link
In this case, alpha has been replaced with the following
```
s = {
     alpha0: 0, 
     alpha1: -pi/2,  
     alpha2: 0, 
     alpha3: -pi/2, 
     alpha4: pi/2, 
     alpha5: -pi/2}

TG_0 =  Matrix([
[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), a1*cos(q1) + a2*sin(q2)*cos(q1) + a3*sin(q2 + q3)*cos(q1) + d4*cos(q1)*cos(q2 + q3) - d7*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2 + q3))],
[-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), a1*sin(q1) + a2*sin(q1)*sin(q2) + a3*sin(q1)*sin(q2 + q3) + d4*sin(q1)*cos(q2 + q3) - d7*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2 + q3))],
[                                    -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                     a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) - d7*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5))],
[                                                                                       0,                                                                                                                                                            0,                                                                                                                                                            0,                                                                                                                                                                                  1]])
```

### Inverse Kinematics
Ik is obtained by splitting into two parts, this is to reduce the complexity of the calculations. One part is a RRR revolute arm, another a spherical arm. Solving the RRR revolute arm would return the joint angles for joint 1,2 and 3. While the spherical arm would give joint angles for joint 4,5 and 6

#### RRR joint

![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/ik_q2q3.png "q2q3")
We would ignore joint 4,5,6. Effectively imagine an arm with just joint 1,2,3

##### Angle for joint 1
This is the easiest angle to determine as it just atan2(y, x)

##### Angle for joint 2 and 3
We discard joint 1 for now. To calculate the angles for these two joints, it is about applying the cosine rule to obtain the angle first for angle 3, then calculating angle 2. The figure below explains how theta is obtained

![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/ik_q2q3_triangle.png "q2q3")

Forming this equation
```
l2 = a2
l3 = sqrt(a3^2 + d4^2)
β = 180 - 𝚹3
xy = sqrt(Wc_x^2 + Wc_y^2) - a0
z = Wc_z - d1
```
Using the cosine rule
```
D^2 = xy^2 + z^2 = l2^2 + l3 ^2 - 2(l2)(l3)cos(β)
cos(𝚹3) = (xy^2 + z^2 - l3*l3 - l2*l2)/(2*l3*l2) = r
𝚹3 = atan2(sqrt(1-r*r), r), where sqrt(1-r*r) = sin(𝚹3)
𝚹3 = atan2(-sqrt(1-r*r), r)
𝚹2 = γ - ⍺ = atan2(xy, z) - atan2(l3*sin(𝚹3), l2+l3*cos(𝚹3))
𝚹3 = 𝚹3 - pi/2
```
𝚹3 has to be be minus with 90 degrees, as the start point is not vertical up, but horizontal right

𝚹3 is checked to ensure it is between -pi and pi/2, as it starts horizontally
𝚹2 is checked to be -pi to pi

##### Angle for joint 4,5,6
The overall roll, pitch, yaw for the end effector relative to the base link is as follows:
![alt text](https://raw.githubusercontent.com/lisaljl/Udacity-RoboND-Kinematics/master/code/rot_spherical.png "rotation")

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:
```
    R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
 ```  
Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:

    R0_6 * Rcorr = Rrpy
    where Rrpy = Homogeneous RPY rotation between base_link and gripper_link
    ajd Rcorr is the difference in axis between the DH convention frame to the URDF frame

As we calculated joints 1-3, we can substitute those values in their respective individual rotation matrices and pre-multiply both sides of eq1 by inv(R0_3) which leads to:
```
    R0_3 * R3_6 * Rcorr = Rrpy
    R3_6 = inv(R0_3) * Rrpy * Rcorr.T
    Rcorr = rot_z()*rot_y()
 ```   
Note for Rrpy we are using extrinsic rotation for X-Y-Z, as by default the method tf.transformation_matrix("","rxyz"), by default returns roll, pitch, yaw for an extrinsic rotation of  X-Y-Z. As such the inverse rotation matrix is 

```
Rxyz_ext = Rx(roll) * Ry(pitch) * Rz(yaw)
inv(Rxyz_ext) = Rz(yaw) * Ry(pitch) * Rx (roll)

    Rrpy = Matrix([[cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma)], 
               [sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma)],
               [          -sin(beta),                                    cos(beta)*sin(gamma), cos(beta)*cos(gamma)]]);

𝚹4 = atan(sin(q4)*sin(q5), -sin(q5)*cos(q4))
where both sin(q5) cancels out

𝚹6 = atan(-sin(q5)*sin(q6), sin(q5)*cos(q6))
where both sin(q5) cancels out

𝚹5 = atan(sqrt(sin2(q5)*cos2(q6) + sin2(q5)*sin2(q6)), cos(q5)) = atan(sin(q5), cos(q5))
where sin2 + cos2 = 1

```



