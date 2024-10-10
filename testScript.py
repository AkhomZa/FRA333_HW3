# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.
2.
3.
'''

import HW3_utils as FK
import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from spatialmath.base import tr2rpy
from spatialmath.base import *
from math import pi

q = [0,-pi/2,0]
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
R,P,R_e,p_e = FK.FKHW3(q)
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a=0,alpha=0,d=0.0892,offset=pi), #frame1
        rtb.RevoluteMDH(a=0,alpha = pi/2,d=0), #frame2
        rtb.RevoluteMDH(a=-0.425,alpha=0,d=0), #frame3
    ],tool = SE3(-(0.39243+0.082),-0.093,0.109) * SE3.RPY(0,-pi/2,0,order='zyx')
)
#robot.plot(q,block=True)
sol = robot.jacobe(q)
sol[abs(sol) < 0.000001] = 0
print("Jacobain at e = ")
print(sol)

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
J_e = robot.jacobe(q)[:3,:3]
e_val = 0.001
if abs(np.linalg.det(J_e)) < e_val:
    flag = 1
else:flag = 0
print(flag)
print(abs(np.linalg.det(J_e)))
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here

#==============================================================================================================#