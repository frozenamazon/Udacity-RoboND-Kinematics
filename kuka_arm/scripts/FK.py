from sympy import symbols, cos, sin, pi, sqrt, simplify
from sympy.matrices import Matrix
import numpy as np

class Transformation:

	def rot_x(self, q):
	    R_x = Matrix([[ 1,              0,        0],
	              [ 0,        cos(q), -sin(q)],
	              [ 0,        sin(q),  cos(q)]])
	    
	    return R_x
	    
	def rot_y(self, q):              
	    R_y = Matrix([[ cos(q),        0,  sin(q)],
	              [       0,        1,        0],
	              [-sin(q),        0,  cos(q)]])
	    
	    return R_y

	def rot_z(self, q):    
	    R_z = Matrix([[ cos(q), -sin(q),        0],
	              [ sin(q),  cos(q),        0],
	              [ 0,              0,        1]])
	    
	    return R_z

	def transformation_matrix(self, alpha,a,d,q):    
	    return Matrix([[        cos(q),        -sin(q),       0,           a],
	                   [ sin(q)*cos(alpha),  cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
	                   [ sin(q)*sin(alpha),  cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
	                   [             0,              0,        0,        1]])

	def initiate(self):

		a0, a1, a2, a3, a4, a5 = symbols('a0:6')
		d1, d2, d3, d4, d5, d6, dG = symbols('d1:8')
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha0:6')
		q1, q2, q3, q4, q5, q6 = symbols('q1:7') 

        # Define Modified DHfrom Transformation matrix
		T0_1 = self.transformation_matrix(0, 0, d1, q1)
		T1_2 = self.transformation_matrix(-pi/2, a1, 0, q2 - pi/2) #remember to offset q2 = -pi/2
		T2_3 = self.transformation_matrix(0, a2, 0, q3)
		T3_4 = self.transformation_matrix(-pi/2, a3, d4, q4)
		T4_5 = self.transformation_matrix(pi/2, 0, 0, q5)
		T5_6 = self.transformation_matrix(-pi/2, 0, 0, q6)

		r6_G = self.rot_z(pi) * self.rot_y(-pi/2)
		t6_G = Matrix([[0],[0],[dG]])
		T6_G = r6_G.row_join(t6_G)
		T6_G = T6_G.col_join(Matrix([[0,0,0,1]]))

		T0_2 = T0_1*T1_2 #base_link to link_2
		T0_3 = T0_2*T2_3 #link_2 to link_3
		T0_4 = T0_3*T3_4 #link_3 to link_4
		T0_5 = T0_4*T4_5 #link_4 to link_5
		T0_6 = T5_0*T5_6 #link_5 to link_6
		T0_G = T6_0*T6_G #link_6 to link_G

		self.T0_3 = T0_3
		self.Rcorr = r6_G
	


