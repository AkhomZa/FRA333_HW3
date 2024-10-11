# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.นายคณพล__6505
2.นายอาคม_6577
3.
'''
import HW3_utils as FK
import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from spatialmath.base import tr2rpy
from spatialmath.base import *
from math import pi

q = [0,0,0]

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]: #รับค่า q เข้ามา
    R,P,R_e,p_e = FK.FKHW3(q)

    num_joints = len(q) #จำนวนข้อต่อ 3
    J_0 = np.zeros((6, num_joints)) #สร้าง matrix jacobian 6*3 ตามกำหนด (สำหรับ J3)
    # R0_i = [[0,0,1],[-np.sin(q[0]),np.cos(q[0]),0],[-np.sin(q[0]),np.cos(q[0]),0]] #กำหนดค่าแกน Zi เทียบ Z0
    z_i = [[0,0,1],[-np.sin(q[0]),np.cos(q[0]),0],[-np.sin(q[0]),np.cos(q[0]),0]] #กำหนดค่าแกน Zi เทียบ Z0
    for i in range(num_joints): #i -> [0,1,2]

        P_i = P[:,i] #P01 P02 P03

        Jv_i = np.cross(z_i[i] , (p_e - P_i)) # R0_1*[0,0,1] x (P0e-P01) / R0_2*[0,0,1] x (P0e-P02) / R0_3*[0,0,1] x (P0e-P03)
        # คำนวณ Angular Velocity Jacobian
        Jw_i = z_i[i]
        
        # เพิ่มเข้าไปใน Jacobian
        J_0[:3, i] = Jv_i  # Linear part
        J_0[3:, i] = Jw_i   # Angular part 
    J_0[abs(J_0) < 0.0001] = 0
    return J_0


#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J_e = (endEffectorJacobianHW3(q))[:3,:3]
    e_val = 0.001
    if abs(np.linalg.det(J_e)) < e_val:
        flag = 1 #closed singularity
    else:flag = 0 #normal
    return flag
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#

print(endEffectorJacobianHW3(q))
print(checkSingularityHW3(q))