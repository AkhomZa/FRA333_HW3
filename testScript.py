# file สำหรับตรวจคำตอบ
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
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
# สร้างโมเดลหุ่นยนต์โดยใช้ DH-parameters
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a=0,alpha=0,d=0.0892,offset=pi), #frame1
        rtb.RevoluteMDH(a=0,alpha = pi/2,d=0), #frame2
        rtb.RevoluteMDH(a=-0.425,alpha=0,d=0), #frame3
    ],tool = SE3(-(0.39243+0.082),-0.093,0.109) * SE3.RPY(0,-pi/2,0,order='zyx') # กำหนดตำแหน่งเครื่องมือ (tool)
)

# คำนวณ Jacobian ที่ตำแหน่งที่กำหนด (q)
sol = robot.jacob0(q)


sol[abs(sol) < 0.000001] = 0 # กำจัดค่าที่มีขนาดเล็กเกินไป
print("Jacobain at e = ")
print(sol)

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
# คำนวณส่วนของ Jacobian สำหรับการวิเคราะห์ความผิดปกติ
sol_linear = robot.jacob0(q)[:3,:3]
e_val = 0.001

# ตรวจสอบการเกิด singularity โดยใช้ Determinant
if abs(np.linalg.det(sol_linear)) < e_val:
    flag = 1 # อยู่ในสภาวะ singularity
    print("อยู่ในสภาวะ singularity")
else:
    flag = 0 # สภาวะปกติ
    print("อยู่ในสภาวะปกติ")

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
# กำหนดค่า wrench w (เวกเตอร์ขนาด 6)
w = [1, 1, 5, 1, 2, 1]  # แทนด้วยค่าจริงที่ต้องการ ประกอบด้วย โมเมนต์ (3 องค์ประกอบแรก) และแรง (3 องค์ประกอบหลัง)

# คำนวณ torque โดยใช้ roboticstoolbox
J_e = robot.jacob0(q)

# คำนวณแรงบิดที่ข้อต่อ (tau) โดยใช้สมการ τ = J^T * w
# ทรานสโพส Jacobian (J^T) คูณกับ wrench (w) จะได้แรงบิดที่ข้อต่อ
tau_rt = np.dot(J_e.T, w)

# แสดงผลแรงบิดที่คำนวณได้
print("effort for roboticstoolbox: ", tau_rt.tolist())

#==============================================================================================================#