## Project: Kinematics Pick & Place

---
This is the write up for the Kinematics project. Alternatively, you may also look at the python notebook that has the working for the following matrices
https://github.com/frozenamazon/RoboND-Kinematics-Project/blob/master/code/Kinematics_Project_Test_Notebook.ipynb

#### DH Parameters for Kuka arm

| i        | a(i-1)  | a(i-1)  | d(i-1)  | θ(i-1)  |
| :------- |:-------:|:-------:|:-------:|:-------:|
| 1        | 0       | 0       | d1      |θ1       |
| 2        | -90     | a1      | 0       |θ2 - 90  |
| 3        | 0       | a2      | 0       |θ3       |
| 4        | -90     | a3      | d4      |θ4       |
| 5        | 90      | 0       | 0       |θ5       |
| 6        | -90     | 0       | 0       |θ6       |
| G        | 0       | 0       | dG      |0        |

#### Transformation matrix for Kuka arm
```
T1_0 =  Matrix([
[            cos(q1),            -sin(q1),            0,               0],
[sin(q1)*cos(alpha0), cos(alpha0)*cos(q1), -sin(alpha0), -d1*sin(alpha0)],
[sin(alpha0)*sin(q1), sin(alpha0)*cos(q1),  cos(alpha0),  d1*cos(alpha0)],
[                  0,                   0,            0,               1]]) 

T2_1 =  Matrix([
[             sin(q2),             cos(q2),            0, a1],
[-cos(alpha1)*cos(q2), sin(q2)*cos(alpha1), -sin(alpha1),  0],
[-sin(alpha1)*cos(q2), sin(alpha1)*sin(q2),  cos(alpha1),  0],
[                   0,                   0,            0,  1]]) 

T3_2 =  Matrix([
[            cos(q3),            -sin(q3),            0, a2],
[sin(q3)*cos(alpha2), cos(alpha2)*cos(q3), -sin(alpha2),  0],
[sin(alpha2)*sin(q3), sin(alpha2)*cos(q3),  cos(alpha2),  0],
[                  0,                   0,            0,  1]]) 

T4_3 =  Matrix([
[            cos(q4),            -sin(q4),            0,              a3],
[sin(q4)*cos(alpha3), cos(alpha3)*cos(q4), -sin(alpha3), -d4*sin(alpha3)],
[sin(alpha3)*sin(q4), sin(alpha3)*cos(q4),  cos(alpha3),  d4*cos(alpha3)],
[                  0,                   0,            0,               1]]) 

T5_4 =  Matrix([
[            cos(q5),            -sin(q5),            0, 0],
[sin(q5)*cos(alpha4), cos(alpha4)*cos(q5), -sin(alpha4), 0],
[sin(alpha4)*sin(q5), sin(alpha4)*cos(q5),  cos(alpha4), 0],
[                  0,                   0,            0, 1]]) 

T6_5 =  Matrix([
[            cos(q6),            -sin(q6),            0, 0],
[sin(q6)*cos(alpha5), cos(alpha5)*cos(q6), -sin(alpha5), 0],
[sin(alpha5)*sin(q6), sin(alpha5)*cos(q6),  cos(alpha5), 0],
[                  0,                   0,            0, 1]]) 

TG_6 =  Matrix([
[0,  0, 1,  0],
[0, -1, 0,  0],
[1,  0, 0, d7],
[0,  0, 0,  1]]) 
```
#### Homogeneous transform matrix from base_link to gripper_link
In this case, alpha has been replaced with the following
s = {
     alpha0: 0, 
     alpha1: -pi/2,  
     alpha2: 0, 
     alpha3: -pi/2, 
     alpha4: pi/2, 
     alpha5: -pi/2}

TG_0 =  Matrix([

[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), 
((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), 
((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), 
a1*cos(q1) + a2*sin(q2)*cos(q1) + a3*sin(q2 + q3)*cos(q1) + d4*cos(q1)*cos(q2 + q3) - d7*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2 + q3))],

[-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  
((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6),  
((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), 
a1*sin(q1) + a2*sin(q1)*sin(q2) + a3*sin(q1)*sin(q2 + q3) + d4*sin(q1)*cos(q2 + q3) - d7*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2 + q3))],

[-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), 
-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3),    
-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), 
a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) - d7*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5))],

[ 0, 0,  0, 1]]) 
