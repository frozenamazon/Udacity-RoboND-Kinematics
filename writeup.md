## Project: Kinematics Pick & Place

---
This is the write up for the Kinematics project. 
The jupyter notebook consists of the working and calculations
https://github.com/frozenamazon/Udacity-RoboND-Kinematics/blob/master/code/Kinematics_Project_Test_Notebook.ipynb
Video for the working arm
https://youtu.be/B2PavVfN0Tg

#### DH Parameters for Kuka arm

The parameters is obtained using the following convention
![alt text](https://raw.githubusercontent.com/frozenamazon/Udacity-RoboND-Kinematics/master/code/DH.png "DH annotation")


| i        | a(i-1)  | a(i-1)  | d(i-1)  | θ(i-1)  |
| :------- |:-------:|:-------:|:-------:|:-------:|
| 1        | 0       | 0       | d1      |θ1       |
| 2        | -90     | a1      | 0       |θ2 - 90  |
| 3        | 0       | a2      | 0       |θ3       |
| 4        | -90     | a3      | d4      |θ4       |
| 5        | 90      | 0       | 0       |θ5       |
| 6        | -90     | 0       | 0       |θ6       |
| G        | 0       | 0       | dG      |0        |


| i        | a(i-1)  | a(i-1)  | d(i-1)  | θ(i-1)  |
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
We would ignore joint 4,5,6. Effectively imagine an arm with just joint 1,2,3
##### Angle for joint 1
This is the easiest angle to determine as it just atan2(y, x)

##### Angle for joint 2 and 3
We discard joint 1 for now. To calculate the angles for these two joints, it is about applying the cosine rule to obtain the angle first for angle 3, then calculating angle 2. The figure below explains how theta is obtained

