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

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    num_joints = len(q)
    J_e = np.zeros((6, num_joints))
    z_i = np.array([0,0,1],[np.sin(q[0]),np.cos(q[0]),0],[np.sin(q[0]),np.cos(q[0]),0])
    for i in range(num_joints):

        #z_i = np.array([0, 0, 1])  # แกน Z ของข้อต่อ i
        
        # ตำแหน่งของจุดปลายหุ่นยนต์
        P_i = ((FK.FKHW3(q))[2])[i]  # ตัวอย่าง (ตำแหน่งจุดปลาย)
        
        # คำนวณ Linear Velocity Jacobian
        Jv_i = np.cross(z_i[i], (FK.FKHW3(q)[3] - P_i)) # oe-o1 , oe-o2 , oe-o3
        
        # คำนวณ Angular Velocity Jacobian
        Jw_i = z_i
        
        # เพิ่มเข้าไปใน Jacobian
        J_e[:3, i] = Jv_i  # Linear part
        J_e[3:, i] = Jw_i  # Angular part
    pass
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    pass
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#
q = [0,0,0]
print(endEffectorJacobianHW3(q))