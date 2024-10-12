# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.นายคณพล__6505
2.นายอาคม_6577
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
def endEffectorJacobianHW3(q:list[float])->list[float]:#รับค่า q เข้ามา
    # เรียกใช้ FKHW3 จากไฟล์ HW3_utils
    R,P,R_e,p_e = FK.FKHW3(q)
    num_joints = len(q) # จำนวนข้อต่อ 3
    J_e = np.zeros((6, num_joints)) # สร้าง matrix jacobian 6*3 ตามกำหนด (สำหรับ J3)

    # กำหนดแกน Zi ในแต่ละเฟรม
    z_i = [[0,0,1],[-np.sin(q[0]),np.cos(q[0]),0],[-np.sin(q[0]),np.cos(q[0]),0]] #กำหนดค่าแกน Zi เทียบ Z0
    
    for i in range(num_joints): #i -> [0,1,2]
        P_i = P[:,i] #P01 P02 P03
        Jv_i = np.cross(z_i[i] , (p_e - P_i)) # R0_1*[0,0,1] x (P0e-P01) / R0_2*[0,0,1] x (P0e-P02) / R0_3*[0,0,1] x (P0e-P03) 
        Jw_i = z_i[i] # คำนวณ Angular Velocity Jacobian
        
        # ใส่ค่าใน Jacobian
        J_e[:3, i] = Jv_i  # Linear part
        J_e[3:, i] = Jw_i   # Angular part 
        
    J_e[abs(J_e) < 0.0001] = 0 # กำจัดค่าที่มีขนาดเล็กเกินไป
    return J_e


#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J_e = (endEffectorJacobianHW3(q))[:3,:3] # ส่วนความเร็วเชิงเส้นของ Jacobian
    e_val = 0.001
    if abs(np.linalg.det(J_e)) < e_val:
        flag = 1 # อยู่ในสภาวะ singularity
        print("อยู่ในสภาวะ singularity")
    else:
        flag = 0 # อยู่ในสภาวะปกติ 
        print("อยู่ในสภาวะปกติ")
    return flag


        
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
# แปลงเวกเตอร์ wrench w (โมเมนต์และแรง) ให้เป็น numpy array เพื่อใช้ในการคำนวณ
    w = np.array(w)

    # เรียกใช้ฟังก์ชัน endEffectorJacobianHW3 เพื่อคำนวณ Jacobian J_e ที่ตำแหน่ง q
    J_e = endEffectorJacobianHW3(q)

    # คำนวณแรงบิดที่ข้อต่อ (tau) โดยใช้สมการ τ = J^T * w
    # ทรานสโพส Jacobian (J^T) คูณกับ wrench (w) จะได้แรงบิดที่ข้อต่อ
    tau = np.dot(J_e.T, w)

    # แปลงผลลัพธ์จาก numpy array ให้เป็น list เพื่อให้สอดคล้องกับรูปแบบ output
    return tau.tolist()

#==============================================================================================================#

# ทดสอบการทำงานของฟังก์ชัน
# กำหนดค่า w (เวกเตอร์ขนาด 6)
w = [1, 1, 5, 1, 2, 1]  # แทนด้วยค่าจริงที่ต้องการ ประกอบด้วย โมเมนต์ (3 องค์ประกอบแรก) และแรง (3 องค์ประกอบหลัง)

print("Jacobian:")
print(endEffectorJacobianHW3(q))
print("Singularity Check:")
print(checkSingularityHW3(q))
print("Effort at Joints:")
print(computeEffortHW3(q, w))