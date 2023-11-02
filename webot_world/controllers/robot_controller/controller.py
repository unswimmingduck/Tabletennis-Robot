"""变量命名规范如下："""
"""
要求：
1.使用英文首字母缩写、下滑线、数字等组合
2.不重名
3.简单易懂
4.常量和变量都要做好简要的文档说明

"""
"""
@brief:简要描述变量或常量
@param:此模块内使用的变量和常量（数值附上）

"""

"""[![Powered by Python Robotics]
(https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/pr_powered.min.svg)]
(https://github.com/petercorke/robotics-toolbox-python)"""

from controller import Supervisor
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
# from roboticstoolbox.tools.trajectory import mstraj
from roboticstoolbox.tools.trajectory import jtraj
import random
import math


"""此处创建实例化对象"""
robot = Supervisor()

# 检索子节点树
root_node = robot.getRoot()
children_field = root_node.getField('children')

# 创建乒乓球
children_field.importMFNode(-1, "pingpongball.wbo")
 
# 获取乒乓球和puma机械臂字段
ball_node = robot.getFromDef("pingpongball")
arm_node = robot.getFromDef("PUMA")
ball_tfield = ball_node.getField("translation")


"""此处声明需要用到的变量"""
vmax = 4.5
vmin = 4.3
thetam = 0.13


# 随机初始化小球初速度
v0 = random.uniform(vmin,vmax)
theta0 = random.uniform(0.11,thetam)
vx = (-1) * v0 * math.cos(theta0)
vy = v0 * math.sin(theta0)
vz = 0
ball_node.setVelocity([vx,vy,vz,0,0,0])



# 设定仿真步长
timestep = 16

"""在此处初始化机械臂关节、传感器、电机等组件"""
# 初始化机械臂关节
# 用于存储motors字段
motors = []
ps = []
# 创建motors'name元组
motors_name = ("joint1","joint2","joint3","joint4","joint5","joint6")
for mn in motors_name:
    motor = robot.getDevice(mn)
    motor.setVelocity(0.) # 初始关节速度设置为0
    position_sensor = motor.getPositionSensor()
    position_sensor.enable(timestep)
    ps.append(position_sensor)
    motors.append(motor)

# print(motors)




"""在此处利用numpy、urdf文件等来对puma机械臂进行正逆运动学、动力学建模"""

"""
@brief:此处用于申明坐标变换相关矩阵
@param:T_A_B    puma基坐标系关于世界坐标系下的位姿变换
       R_A_B    基坐标系相对于世界坐标系下的旋转变换
       T_B_A    位姿逆变换
       R_B_A    旋转逆变换

"""

T_A_B = np.array([[0,-1,0,-1.5],
                  [1,0,0,-0.6],
                  [0,0,1,0.4],
                  [0,0,0,1]])

T_B_A = np.array([[0,1,0,0.6],
                  [-1,0,0,-1.5],
                  [0,0,1,-0.4],
                  [0,0,0,1]])

R_A_B = np.array([[0,-1,0],
                  [1,0,0],
                  [0,0,1]])

R_B_A = np.array([[0,1,0],
                  [-1,0,0],
                  [0,0,1]])

"""根据官方提供的标准DH参数自定义创建兼容 rtb的puma DHRobot Class 并将其放入相应目录下"""
puma = rtb.models.DH.Puma560() # 实例化对象





"""在此处定义自定义函数"""
"""
要求：
1.函数体内必须要有简要描述功能的文档字符长，传参类型说明可写可不写
2.参数类型和返回值类型要与webots api提供的参数类型一致
3.尽量不使用复杂的定义，如：一等对象编程技巧
"""

a = 0     # 摩擦系数
b = 0.902 # 反弹系数



def transformb(lis_t:list) -> np.ndarray:
    """
    此函数用于将小球在世界坐标系下的坐标其次变换为puma基坐标系下坐标，
    并返回np.ndarray类型的齐次列向量
    """
    lis_t.append(1)
    ls = np.array(lis_t)[:,None]
    
    return np.dot(T_B_A,ls)


def v_after_strike(p:np.ndarray) -> np.ndarray:
    """
    此函数用于根据碰撞处小球坐标与目标点的坐标，求算小球被击打后的速度(相对于世界坐标系)，返回类型为np.ndarray的列向量
    """
    tp = [-0.685,0,0.776] #此处输入目标点的坐标
    h = p[2] - 0.776811
    g = 9.81
    t = pow((2 * h) / g,0.5)
    return np.array([(tp[0]-p[0]) / t,(tp[1]-p[1]) / t,0.],dtype='float')


def strike_position(V:np.ndarray, P:np.ndarray) -> np.ndarray: 
    """
    此函数用于计算击打位置，传入碰撞后的速度和位置
    """
    g = 9.81  # 重力加速度
    t = V[2]/g  # 乒乓球竖直速度降为0所需时间
    P0 = np.array([P[0]+V[0]*t, P[1]+V[1]*t, P[2]+0.5*g*t**2],dtype='float')  # 乒乓球竖直速度降为0的位置，即击打位置
    return P0


def strike_velocity(V):
    """
    此函数用于计算小球到达击打点时的速度
    """
    return np.array([V[0],V[1],0.],dtype='float')


def Conversion(V0, Vf, P0):
    #  定义函数用来求解球拍姿态矩阵，V0为乒乓球入射速度，Vf为出射速度
    Vn = Vf - (1-a)*V0  #  出射速度的法线方向（球拍X方向）的速度
    Vnx = Vn[0]
    Vny = Vn[1]
    R11 = Vnx/pow(Vnx**2 + Vny**2, 1/2)
    R12 = Vny/pow(Vnx**2 + Vny**2, 1/2)
    R21 = R12
    R22 = -R11
    R = np.array([[R11, R12, 0], [R21, R22, 0], [0, 0, 1]])                                      # 球拍相对于世界坐标系的姿态矩阵
    T = np.array([[R11, R12, 0, P0[0]], [R21, R22, 0, P0[1]], [0, 0, 1, P0[2]], [0, 0, 0, 1]])   # 球拍相对于世界坐标系的位姿矩阵
    return R,T

def Velocity(V0, Vf,R):

    #  定义函数用来求解球拍速度，V0为乒乓球入射速度，Vf为出射速度,R为球拍相对于基坐标系的旋转矩阵
    
    v1x = Vf[0] - (1-a)*V0[0]               # 法线方向速度在x方向上投影
    v1y = Vf[1] - (1 - a) * V0[1]           # 法线方向在y方向上投影
    v2y = Vf[1] + b*V0[1]                   # 切线方向速度在x方向上投影
    Vh1x = -v1y*v2y/v1x + Vf[0] + b*V0[0]   # 求解乒乓球拍相对于球拍坐标系在x方向速度
    Vh1 = np.array([Vh1x, 0, 0])            # 乒乓球拍在其他两个方向速度设置为0
    # Vh2 = np.dot(R,Vh1)                   # 乒乓球拍的线速度
    # Vh2 = np.dot(R_B_A,Vh2)
    Vh2 = np.append(Vh1,[0,0,0])            # 乒乓球拍的角速度设置为零，相对于球拍坐标系
    E = np.zeros((3,3))
    r = np.block([
            [R,E],
            [E,R]
        ])
    Vh = r * np.mat(Vh2).T

    return Vh.A                             # 返回Vh,即乒乓球拍关于基坐标系的速度






flag1 = 0 # 碰撞标记位
flag2 = 0 # 轨迹规划完成标记位
flag3 = 0 # 机械臂恢复位姿标志位
flag4 = 0

ap = []   # 存储机械臂各关节角位移量

# 主循环:
while robot.step(timestep) != -1:
    
    # 测试逆运动学函数
    # targetPosition = [-1.37,0,1.2]

    # # sol = puma.ikine_LMS(transformb(targetPosition))
  
    # we = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

    # q = puma.qz
 
    # ilimit = 30

    # slimit = 100

    # tol = 1e-6

    # Tep = np.array([[0.2,0,0,-0.83],
    #                 [0,0.5,0,0],
    #                 [0,0,0.6,0.8],
    #                 [0,0,0,1]])

    # sol = puma.ik_gn(
    #             Tep = Tep,
    #             q0 = q,
    #             ilimit=ilimit,
    #             slimit=slimit,
    #             tol=tol,
    #             we=we,
    #             use_pinv=False,
    #             reject_jl=False,)

    # # qr = [3.14,0,0,0,1.57,0]
    # for i in range(len(motors)):
    #     # motors[i].setPosition(sol.q[i])
    #     motors[i].setPosition(sol[0][i])
    #     # print(puma.sol[0][i])



    # 每timestep获取小球的位置以及速度信息
    ball_position = ball_node.getPosition()
    ball_velocity = ball_node.getVelocity()

    # if -1.37 < ball_position[0] < 1.37 and -0.7625 < ball_position[1] < 0.7625 and ball_position[2] < 0.797:   # 小球在此发生碰撞（与乒乓球桌）
    if ball_velocity[2] > 0 and flag4 == 0 and -1.37 < ball_position[0] < 1.37 and -0.7625 < ball_position[1] < 0.7625:

        flag1 = 1 
        # flag3 = 0
   
 
    if flag1 == 1:    # 乒乓球与桌面相碰
        # print(ball_position)
        # print(ball_velocity)
        
        # 将坐标和速度转换成 ndarray 类型
        bp = np.array(ball_position)
        bv = np.array(ball_velocity)

    
        # 确定最佳击球点位置和所需时间以及小球在击球点时的速度

        p = strike_position(bv,bp)                  # 求解小球击打位置
        
        v0 = strike_velocity(bv)                    # 小球到达击球点的速度

        vf = v_after_strike(p)                      # 击打后小球球速

        Re0,Te0 = Conversion(v0,vf,p)               # 计算到达击球点时，机械臂末端执行器（乒乓球拍）在世界坐标系的位姿矩阵

        Reb = np.dot(R_B_A,Re0)

        Teb = np.dot(T_B_A,Te0)                     # 计算...,乒乓球拍相对于基坐标系的位姿矩阵
        # print(type(Teb))

        Veb = Velocity(v0,vf,Reb)                   # 计算乒乓球拍相对于基坐标系的速度
        

        # Veb = np.dot(r_B_A,Ve0.reshape(-1,1))     # 计算球拍相对于基坐标系速度

        g = 9.81                                    # 重力加速度

        t0 = ball_velocity[2] / g                              # 击球时间   




        # 根据击球位置和时间以及目标落点位置，确定机械臂的轨迹曲线


        # 求逆
        """LM求逆算法"""
        Teb = sm.SE3(Teb)
        # print(Teb)

        sol1 = puma.ikine_LMS(Teb)
        qf = sol1.q
        print(sol1)
        # print(qf)




        """NR求逆算法"""
       
        # # Cartesion DoF priority matrix
        # we = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        # # Our desired end-effector poses
        # Tep = Teb

        # # Maximum iterations allowed in a search
        # ilimit = 30

        # # Maximum searches allowed per problem
        # slimit = 100

        # # Solution tolerance
        # tol = 1e-6            

        # sol1 = puma.ik_gn(
        # Tep,
        # ilimit=ilimit,
        # slimit=slimit,
        # tol=tol,
        # we=we,
        # use_pinv=False,
        # reject_jl=False,
        # )


        # 利用雅可比矩阵求关节末速度
        jaco = np.mat(puma.jacob0(qf))
        qdf = jaco.I * np.mat(Veb.reshape(-1,1))
        qdf = qdf.T.A
        







        #轨迹规划

        """ Multi-segment multi-axis trajectory """
        # q0 = puma.qr

        # qf = sol1[0]

        # jaco = np.mat(puma.jacob0(qf))

        # qd0 = np.array([0,0,0,0,0,0])

        # qdf = jaco.I * np.mat(Veb.reshape(-1,1))

        # qdf = qdf.A
        # # print(qdf)

        # # tval = np.linspace(0,int(t * 1000),16) / 1000

        # viapoints = np.vstack([q0,qf])
        # print(viapoints)

        # qdmax = np.array([3.14,3.14,3.14,6.28,6.28,6.28])

        # sol2 = mstraj(viapoints=viapoints,dt=0.016,tacc=t,qdmax=qdmax,qd0=qd0,qdf=qdf)

        # # print(sol2)



        """ joint-space trajectory """

        q0 = puma.qr
        # qf = sol1[0][0]
       
        
        # num = t0 // 0.016
        # jaco = np.mat(puma.jacob0(qf))
        tval = np.arange(0,int((t0+0.375) * 1000),16) / 1000
        qd0 = np.array([0,0,0,0,0,0])
        qd1 = qdf
        

        sol2 = jtraj(q0=q0,qf=qf,t=tval,qd0=qd0,qd1=qd1)








        # 结束后flag自增，防止多次进行不必要的计算
        flag1 = 0    # 计算完成后重置标记位
        flag2 = 1    # 标记计算轨迹完成
        j = 0
        flag4 = 1
        
    if flag2 == 1:
        """机械臂运动到击球点"""
        i = 0
        for i in range(6):

            motors[i].setPosition(sol2.q[j][i])
            motors[i].setVelocity(math.fabs(sol2.qd[j][i]))
            # motors[i].setAcceleration(abs(sol2.qdd[j][i]))

        j += 1

        if j == sol2.q.shape[0] - 1:
            flag2 = 0
            flag3 = 1
            j = 0

    if flag3 == 1:
        """机械臂恢复姿态"""
        for i in range(6):
            motors[i].setPosition(puma.qr[i])  
            motors[i].setVelocity(3.0)


    # 将计算结果传入机械臂控制API控制其运行

    # if -1.37 < ball_position[0] < 1.37 and -0.7625 < ball_position[1] < 0.7625 and ball_position[2] < 0.787:   # 小球在此发生碰撞（与乒乓球桌）
    #     flag1 = 1 
    #     # flag3 = 0


    for i in range(len(ps)):
        # position_sensor = motors[i].getPositionSensor()
        ap.append(ps[i].getValue())

    """检测乒乓球是否与地面发生碰撞并且机械臂恢复初始击球姿态，若完成，则重置小球坐标且赋于随机初速度"""
    if ball_position[2] <= 0.021 and math.fabs(ap[0]-puma.qr[0]) <= 0.01 and math.fabs(ap[1]-puma.qr[1]) <= 0.01 and math.fabs(ap[2]-puma.qr[2]) <= 0.01 and math.fabs(ap[3]-puma.qr[3]) <= 0.01 and math.fabs(ap[4]-puma.qr[4]) <= 0.01 and math.fabs(ap[5]-puma.qr[5]) <= 0.01:

        flag3 = 0
        flag4 = 0
        #重置小球坐标
        ball_tfield.setSFVec3f([1.37,0.1,1.45])
        # #随机初始化小球初速度
        v0 = random.uniform(vmin,vmax)
        theta0 = random.uniform(0.11,thetam)
        vx = (-1) * v0 * math.cos(theta0)
        vy = v0 * math.sin(theta0)
        vz = 0
        ball_node.setVelocity([vx,vy,vz,0,0,0])  

    # print(f"flag1:{flag1}")
    # print(f"flag2:{flag2}")
    # print(f"flag3:{flag3}")


  

    pass


