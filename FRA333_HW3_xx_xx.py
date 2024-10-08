# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.นายคณพล__6505
2.นายอาคม_6577
3.
'''
import HW3_utils as FK
import numpy as np
from math import pi

q = [0,-pi,0] 

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]: #รับค่า q เข้ามา
    num_joints = len(q) #จำนวนข้อต่อ
    J_e = np.zeros((6, num_joints)) #สร้าง matrix jacobian 6*3 ตามกำหนด
    z_i = [[0,0,1],[np.sin(q[0]),np.cos(q[0]),0],[np.sin(q[0]),np.cos(q[0]),0]] #กำหนดค่าแกน Zi เทียบ Z0
    for i in range(num_joints): #i -> [0,1,2]

        P_i = ((FK.FKHW3(q))[1])[:,i+1] #P01 P02 P03
        
        # คำนวณ Linear Velocity Jacobian
        Jv_i = np.cross(z_i[i], ((FK.FKHW3(q))[3] - P_i)) # Z1 x (P0e-P01) / Z2 x (P0e-P02) / Z3 x (P0e-P03)
        
        # คำนวณ Angular Velocity Jacobian
        Jw_i = z_i[i]
        
        # เพิ่มเข้าไปใน Jacobian
        J_e[:3, i] = Jv_i  # Linear part
        J_e[3:, i] = Jw_i  # Angular part

    return J_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J_e = (endEffectorJacobianHW3(q))[:3,:3]
    e_val = 0.001
    if abs(np.linalg.det(J_e)) < e_val:
        flag = 0
    else:flag = 1
    print(abs(np.linalg.det(J_e)))
    return flag
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#


print(endEffectorJacobianHW3(q))
print(checkSingularityHW3(q))
#print(np.linalg.det(endEffectorJacobianHW3(q)[:3,:3]))