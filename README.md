
# FRA333_HW3
จัดทำโดย นายคณพล กาจธัญกิจ 65340500005 และ นายอาคม สนธิขันธ์ 65340500077

โปรแกรมนี้เป็นโปรแกรมสำหรับหา Jacobian, Singularity และ Effort ของหุ่นยนต์ RRR ที่มีลักษณะโครงสร้างดังรูปที่ 1 

![Example image](pic1.png)



## ติดตั้งก่อนการใช้งาน

โปรแกรมนี้ต้องการ `numpy` และ `roboticstoolbox` ในการทำงาน คุณสามารถติดตั้งได้โดยใช้คำสั่งต่อไปนี้:

```bash
pip install numpy<2
pip3 install roboticstoolbox-python
```
โปรเจคนี้ประกอบไปด้วย 3 ไฟล์หลักสำหรับการทำงาน:

1. `FRA333_HW3_6505_6577.py` – ไฟล์ที่เก็บฟังก์ชันต่างๆ ที่พัฒนาสำหรับการแก้หา Jacobian, Singularity และ Effort ของหุ่นยนต์
2. `testScript.py` – สคริปต์สำหรับทดสอบฟังก์ชันที่พัฒนาในไฟล์ `FRA333_HW3_6505_6577.py` และเปรียบเทียบผลลัพธ์กับ Robotic Toolbox
3. `HW3_utils.py` เป็นไฟล์สำหรับสมการ Forward Kinematics ที่ใช้ในการหา Jacobian, Singularity และ Effort ของหุ่นยนต์

## วิธีการ run โปรแกรม

1. run ไฟล์ `FRA333_HW3_6505_6577.py` เพื่อเรียกใช้ฟังก์ชันและแสดงผลลัพธ์ของแต่ละปัญหา
2. ใช้ไฟล์ `testScript.py` เพื่อตรวจสอบความถูกต้องของฟังก์ชันเทียบกับผลลัพธ์จาก Robotic Toolbox

## Files and Functions

## `FRA333_HW3_6505_6577.py`
ไฟล์นี้ประกอบด้วยฟังก์ชันหลัก 3 ฟังก์ชัน:

### 1.**`endEffectorJacobianHW3(q: list[float]) -> list[float]`**  
  ฟังก์ชันนี้คำนวณเมทริกซ์ Jacobian สำหรับตำแหน่ง end-effector โดยรับค่ามุมข้อต่อ `q`
  - **Input**:  
    - `q`: รายการมุมข้อต่อ (จำนวน 3 ข้อสำหรับหุ่นยนต์ 3-DOF)

  - **Output**:  
    - เมทริกซ์ Jacobian ขนาด 6x3 ซึ่งแสดงความสัมพันธ์ระหว่างความเร็วเชิงเส้นและเชิงมุม

  - **อธิบายโค้ดโดยละเอียด**

    สร้าง DH_parameter โดยใช้ฟังก์ชั่นจาก Robotic toolbox
    ```
    robot = rtb.DHRobot(
      [
        rtb.RevoluteMDH(a=0,alpha=0,d=0.0892,offset=pi), #frame1
        rtb.RevoluteMDH(a=0,alpha = pi/2,d=0), #frame2
        rtb.RevoluteMDH(a=-0.425,alpha=0,d=0), #frame3
      ],tool = SE3(-(0.39243+0.082),-0.093,0.109) * SE3.RPY(0,-pi/2,0,order='zyx') # กำหนดตำแหน่งเครื่องมือ (tool)
    )
    ```

    เมื่อสร้าง DH_parameter เสร็จเพิ่ม end_effector ชื่อ tool ซึ่งใส่ Transformation matrix เกิดจาก Translation + Rotation ซึ่งค่า Position ของ {e} เทียบ {3} ได้มาจากไฟล์ HW3_utils.py ที่ให้มา ในชื่อตัวแปร d4 d5 d6 และ Rotation [0,pi/2,0] ตามที่โจทย์กำหนดไว้
    ```
    tool = SE3(-(0.39243+0.082),-0.093,0.109) * SE3.RPY(0,-pi/2,0,order='zyx')
    ```

    ใช้ฟังก์ชั่นใน Robotic toolbox เพื่อหา Jacobian โดยป้อนค่า q เป็น Input
    ```
    sol = robot.jacob0(q)
    ```

    เพื่อให้สามารถดูคำตอบได้อย่างง่าย จึงทำการตัดเลขในคำตอบที่ค่าน้อยมากๆ 
    ```
    sol[abs(sol) < 0.000001] = 0
    ```

    ค่าที่ใช้เป็น threshold สำหรับการปัดค่าเป็น 0 กำหนดโดยสังเกตจากผลลัพธ์ ซึ่งส่วนใหญ่มีค่าไม่น้อยกว่าที่ threshold นี้แล้วจะได้คำตอบ เมื่อ q = [0,0,0]
    ### Output
    ```
    [[-0.109   -0.093   -0.093  ]
     [ 0.89943  0.       0.     ]
     [ 0.      -0.89943 -0.47443]
     [ 0.       0.       0.     ]
     [ 0.       1.       1.     ]
     [ 1.       0.       0.     ]]
    ```

### 2.**`checkSingularityHW3(q: list[float]) -> bool`**  
  ฟังก์ชันตรวจสอบว่าหุ่นยนต์อยู่ในสภาวะ singularity หรือไม่ โดยการคำนวณค่า determinant ของส่วนเชิงเส้นของเมทริกซ์ Jacobian
  - **Input**:  
    - `q`: รายการมุมข้อต่อ (จำนวน 3 ข้อ)
  - **Output**:  
    - ค่าบูลีน (`True` หากหุ่นยนต์อยู่ในสภาวะ singularity, `False` หากไม่อยู่ในสภาวะดังกล่าว)

  - **อธิบายโค้ดโดยละเอียด**
    เรียกใช้ฟังก์ชัน endEffectorJacobianHW3(q) เพื่อคำนวณ Jacobian ของ end effector จากนั้นเลือกเฉพาะส่วนย่อยของ Jacobian ที่เกี่ยวข้องกับความเร็วเชิงเส้น (3 แถวแรก และ 3 คอลัมน์แรก)
    ```
    J_e = (endEffectorJacobianHW3(q))[:3,:3]
    ```
  
    การตั้งค่าเกณฑ์ความผิดพลาด (threshold)
    ```
    e_val = 0.001
    ```
  
    กำหนดค่าความผิดพลาด (threshold) เป็น 0.001 เพื่อใช้ในการตรวจสอบว่า Jacobian มีค่า determinant ใกล้เคียงกับศูนย์หรือไม่ ซึ่งเป็นตัวบ่งชี้ถึง singularity
    - ใช้ np.linalg.det(J_e) เพื่อคำนวณค่า determinant ของเมทริกซ์ Jacobian ส่วนย่อยที่เกี่ยวกับความเร็วเชิงเส้น
    - ถ้า determinant มีค่าน้อยกว่าค่าเกณฑ์ (e_val), จะถือว่า Jacobian เกือบเป็น singular matrix หรือใกล้จุดเอกฐาน ซึ่งหมายความว่าระบบจะไม่สามารถขยับในบางทิศทางได้ (เกิด singularity) และ flag จะถูกตั้งเป็น 1
    - หากค่า determinant มากกว่าเกณฑ์, ระบบถือว่าอยู่ในสถานะปกติ (ไม่อยู่ใน singularity) และ flag จะเป็น 0
    ```
      if abs(np.linalg.det(J_e)) < e_val:
          flag = 1 # อยู่ในสภาวะ singularity
          print("อยู่ในสภาวะ singularity")
      else:
          flag = 0 # อยู่ในสภาวะปกติ 
          print("อยู่ในสภาวะปกติ")
      return flag
    ```
    ### Output
    ```
    Singularity Check:
    อยู่ในสภาวะปกติ
    0
    ```

### 3.**`computeEffortHW3(q: list[float], w: list[float]) -> list[float]`**  
  ฟังก์ชันคำนวณแรงบิดที่ต้องการสำหรับข้อต่อโดยรับมุมข้อต่อ `q` และเวกเตอร์ wrench ภายนอก `w`
  - **Input**:  
    - `q`: รายการมุมข้อต่อ (จำนวน 3 ข้อ)
    - `w`: เวกเตอร์ wrench (ขนาด 6) ที่ประกอบด้วยโมเมนต์และแรง
  - **Output**:  
    - รายการแรงบิดที่จำเป็นสำหรับแต่ละข้อต่อ
     
    - **อธิบายโค้ดโดยละเอียด**
    แปลงเวกเตอร์แรงและโมเมนต์ w ให้เป็น numpy array เพื่อใช้ในการคำนวณแบบ matrix.
    ```
    w = np.array(w)
    ```
  
    ใช้สมการ τ = J^T * w เพื่อคำนวณแรงบิด τ ที่แต่ละข้อต่อ โดยใช้ Jacobian ของหุ่นยนต์และแรงที่กระทำบน end effector.
    ```
    tau = np.dot(J_e.T, w)
    ```
    ### Output
    ```
    Effort at Joints:
    [1.7904300000000002, -2.5901499999999995, -0.46514999999999995]
    ```

## `testScript.py`
สคริปต์นี้ใช้ตรวจสอบความถูกต้องของฟังก์ชันที่พัฒนาในไฟล์ `FRA333_HW3_6505_6577.py` โดยเปรียบเทียบกับผลลัพธ์ที่ได้จาก Robotic Toolbox (`rtb`)

#### Test Descriptions:

- **ทดสอบ Jacobian ของ end-effector**:  
  เมทริกซ์ Jacobian ที่คำนวณโดยใช้ฟังก์ชัน `endEffectorJacobianHW3` จะถูกเปรียบเทียบกับผลลัพธ์จากโมเดลของหุ่นยนต์ใน Robotic Toolbox

  ใช้ฟังก์ชั่นใน Robotic toolbox เพื่อหา Jacobian โดยป้อนค่า q เป็น Input
  ```
  sol = robot.jacob0(q)
  ```

  เพื่อให้สามารถดูคำตอบได้อย่างง่าย จึงทำการตัดเลขในคำตอบที่ค่าน้อยมากๆ
  ```
  sol[abs(sol) < 0.000001] = 0
  ```
  ค่าที่ใช้เป็น threshold สำหรับการปัดค่าเป็น 0 กำหนดโดยสังเกตจากผลลัพธ์ ซึ่งส่วนใหญ่มีค่าไม่น้อยกว่าที่ threshold นี้แล้วคำตอบ เมื่อ q = [0,0,0]
  **Output**
  ```
  [[-0.109   -0.093   -0.093  ]
   [ 0.89943  0.       0.     ]
   [ 0.      -0.89943 -0.47443]
   [ 0.       0.       0.     ]
   [ 0.       1.       1.     ]
   [ 1.       0.       0.     ]]
  ```

- **ทดสอบ singularity**:  
  ฟังก์ชันตรวจสอบ singularity จะถูกทดสอบโดยการตรวจสอบค่า determinant ของส่วนเชิงเส้นของ Jacobian ทั้งจากการคำนวณแบบกำหนดเองและ Robotic Toolbox
  
  เรียกใช้ฟังก์ชัน robot.jacob0(q)[:3,:3] เพื่อคำนวณ Jacobian ของ end effector จาก roboticstoolbox 
  ```
  sol_linear = robot.jacob0(q)[:3,:3]
  ```
  การตั้งค่าเกณฑ์ความผิดพลาด (threshold)
  ```
  e_val = 0.001
  ```
  ใช้โค้ดชุดเดี่ยวกันกับในไฟล์ FRA333_HW3_6505_6577.py แต่เปลี่ยรนจาก np.linalg.det(J_e) เป็น np.linalg.det(sol_linear) เพื่อหา singularity ที่อิงจาก roboticstoolbox
  ```
  if abs(np.linalg.det(sol_linear)) < e_val:
    flag = 1 # อยู่ในสภาวะ singularity
    print("อยู่ในสภาวะ singularity")
  else:
    flag = 0 # สภาวะปกติ
    print("อยู่ในสภาวะปกติ")
  ```
  **Output**
  ```
  Singularity Check:
  อยู่ในสภาวะปกติ
  0
  ```
  
- **ทดสอบแรงบิดที่ข้อต่อ**:  
  แรงบิดที่ข้อต่อที่คำนวณได้จากฟังก์ชัน `computeEffortHW3` จะถูกเปรียบเทียบกับค่าแรงบิดที่ได้จาก Robotic Toolbox โดยใช้ wrench เดียวกัน

  คำนวณ torque โดยใช้ roboticstoolbox
  ```
  J_e = robot.jacob0(q)
  ```
  คำนวณแรงบิดที่ข้อต่อ (tau) โดยใช้สมการ τ = J^T * w
  ทรานสโพส Jacobian (J^T) คูณกับ wrench (w) จะได้แรงบิดที่ข้อต่อ
  ```
  tau_rt = np.dot(J_e.T, w)
  ```
  **Output**
  ```
  Effort at Joints:
  [1.7904300000000002, -2.5901499999999995, -0.46514999999999995]
  ```

