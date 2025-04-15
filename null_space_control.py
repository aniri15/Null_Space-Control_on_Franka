from numpy import float64
import pybullet as p
import pybullet_data
from pybullet_sim_panda.utils import *
import time
from pybullet_sim_panda.dynamics import PandaDynamics
import spatialmath as sm
from eec.eec import EEC
from eec.subfunctions import *
import copy
import matplotlib.pyplot as plt
import os



def reverseTwist(twist):
    a, b = twist[0:3], twist[3:]
    return np.concatenate([b, a], axis=None)

def makeNullProjector(J, W):
    W_inv = np.linalg.inv(W)
    J_pseudoinv = W_inv @ J.T @ np.linalg.inv(J @ W_inv @ J.T)
    return np.eye(J.shape[1], dtype=np.float64) - J.T @ J_pseudoinv.T







RATE = 240. # default: 240Hz
REALTIME = 0
DURATION = 20
STEPSIZE = 1/RATE

t = 0.

uid = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.5])
p.resetDebugVisualizerCamera(cameraDistance= 0, cameraYaw=-50, cameraPitch=0,  cameraTargetPosition=[-0.3,0.8,0.8])
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # for loading plane

p.resetSimulation() #init
p.setRealTimeSimulation(REALTIME)
p.setGravity(0, 0, -9.81) #set gravity

plane_id = p.loadURDF("plane.urdf", useFixedBase=True) # load plane
p.changeDynamics(plane_id,-1,restitution=.95)


cube_pose =[0, 0.7, 0.3]
orn = [0, 0, 0, 1]
full_path_cube = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf/cube.urdf")
cube = p.loadURDF(full_path_cube, cube_pose, orn, useFixedBase=True)

cylinder_center = [0.0, 0.5, 0.63]
full_path_cylinder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf/cylinder.urdf")
cylinder = p.loadURDF(full_path_cylinder, basePosition=cylinder_center, baseOrientation=orn, 
                                                    useMaximalCoordinates=True, useFixedBase=True)


panda = PandaDynamics(p, uid) # load robot
panda.set_arm_positions([0,0,0,-np.pi/2,0,-np.pi/2,-2.96])
panda.setControlMode("torque")



""" Target position and orientation is needed
"""
#target_pos = np.array([ 6.12636866e-01, -3.04817487e-12,  5.54489818e-01], np.float64)
#target_pos = np.array([ 6.12636866e-01, -3.04817487e-12,  0.6], np.float64)
# target_pos = np.array([0.6, 0, 0.6], np.float64)
target_pos = np.array(cylinder_center,np.float64)
target_ori = np.array([2.77158854, 1.14802956, 0.41420822], np.float64)
target_R = sm.base.exp2r(target_ori)
K_p = 30 # propotional(position) gain
K_r = 2 # propotional(rotation) gain
K_dp = 15 # damping(position) gain
K_dr = 0.6 # damping(rotation) gain
D_null = 1. # null-space damping 



""" Data initializing
"""
R = sm.base.exp2r(panda.get_ee_pose(exp_flag=True)[1])
R_past = copy.deepcopy(R)
R_dot = (R-R_past)/STEPSIZE
pos_past = panda.get_ee_pose(exp_flag=True)[0]
R_e = target_R.T @ R
R_e_past = copy.deepcopy(R_e)
eec_panda = EEC(dt=STEPSIZE, R_init=R_e, k=3)



time_list = []
theta_bar_list = []
pos_error_list = []
rot_error_list = []
joint_vel_list = []

pos_list = []
ball = 0
load_ball = False
# for i in range(1000):
#     pos, _ = p.getBasePositionAndOrientation(cylinder)
#     p.submitProfileTiming("Step")
#     jointsPose = p.calculateInverseKinematics(panda._robot, 11, pos, maxNumIterations=50)
#     for i in range(7):
#         panda._client.setJointMotorControl2(panda._robot, i, p.POSITION_CONTROL, jointsPose[i], force=500)
#     p.stepSimulation()
#     time.sleep(1./240.)



print("Null space control")

#for i in range(7):
#    panda._client.setJointMotorControl2(panda._robot, i, p.VELOCITY_CONTROL, 0, force=0)
#p.stepSimulation()


""" Control loop
"""
for i in range(int(DURATION/STEPSIZE)):
    if i%RATE == 0:
        print("Simulation time: {:.3f}".format(t))
    

    pos, ori = panda.get_ee_pose(exp_flag=True)
    # print(pos)
    # print(ori)
    pos_error = pos - target_pos
    vel = (pos-pos_past)/STEPSIZE

    R = sm.base.exp2r(ori)
    R_dot = (R-R_past)/STEPSIZE
    R_e = target_R.T @ R
    R_e_dot = (R_e-R_e_past)/STEPSIZE

    vel_b = R.T @ vel
    w_e_b = vee(R_e.T @ R_e_dot)
    w_b = vee(R.T @ R_dot)
    eec_panda.update(R_e, w_e_b)

    d_term = np.concatenate((vel_b * K_dp, w_b * K_dr), axis=None)

    conv_ori = B(eec_panda.get_unit_vector()*eec_panda._theta) @ ((R_e.T @ sm.base.exp2r(eec_panda._eec)).T)
    Convert = np.concatenate((np.concatenate((R, np.zeros((3,3), np.float64)), axis=1),
                              np.concatenate((np.zeros((3,3), np.float64), conv_ori), axis=1)),
                             axis=0)
    p_term = np.concatenate((pos_error*K_p, eec_panda._eec*K_r), axis=None)
    p_term = Convert.T @ p_term

    # (pos, ori) to (ori, pos)
    p_term = reverseTwist(p_term)
    d_term = reverseTwist(d_term)
    Fb = -d_term - p_term
    Jb = panda.get_body_jacobian()
    tau = Jb.T @ Fb

    tau_grav = panda.inverseDynamics(panda.get_states("all")["position"], [0.]*9, [0.]*9)[:-2]
    #print(len(tau_grav))
    joint_pos = np.array(panda.get_states("all")["position"], np.float64)
    M = panda.getMassMatrix(joint_pos)[:-2, :-2]
    N = makeNullProjector(Jb, M)
    joint_vel = np.array(panda.get_states()["velocity"], np.float64)
    tau_null = N @ (-D_null * joint_vel)

    
    target_torque = tau + tau_grav + tau_null
    #print(target_torque)
    # key step for torque control
    # implement torque control
    panda.setTargetTorques(target_torque, saturate=True)


    joint_pos = panda.get_states("all")["position"]
    error = np.linalg.norm(pos_error)

    #print("==========Error==========")
    #print(error)
    contact_points = p.getContactPoints(panda._robot, cube)
    if len(contact_points) > 0:
        print("Contact!")
    else:
        if (error > 0.05):
            pos_list.append(joint_pos)

    if (error < 0.05) & (load_ball == False):
            print("Goal reached!")
            # random position for the ball
            for i in range(7):
                ball_position = [np.random.uniform(-0.3, 0.3), 0.2, 0.55]
            # ball_position = [-0.3, 0.2, 0.55]
                full_path_ball = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf/sphere_small.urdf")
                ball = p.loadURDF(full_path_ball, 
                                            ball_position, useFixedBase=True)
                load_ball = True

    #null space control
    if ball != 0:
        collision = p.getContactPoints(panda._robot, ball)
        collision2 = p.getContactPoints(panda._robot, cube)
        #if (len(collision) > 0 or len(collision2) > 0):
        if (len(collision) > 0):
            force = [0.1]*7
            tau_null = N @ force + tau_null
            #target_torque = tau + tau_grav + tau_null
            target_torque = tau_null
            print("apply force")
            # key step for torque control
            # implement torque control
            panda.setTargetTorques(target_torque, saturate=True)
        else:
            pos_list.append(joint_pos)
            #print(" null space control finished ")

                
                # delete the ball
                # p.removeBody(ball)
    # # user interface parameters
    # rollId = p.addUserDebugParameter("roll", -1.5, 1.5, 0)
    # pitchId = p.addUserDebugParameter("pitch", -1.5, 1.5, 0)
    # yawId = p.addUserDebugParameter("yaw", -1.5, 1.5, 0)
    # fwdxId = p.addUserDebugParameter("fwd_x", -1, 1, 0)
    # fwdyId = p.addUserDebugParameter("fwd_y", -1, 1, 0)
    # fwdzId = p.addUserDebugParameter("fwd_z", -1, 1, 0)

    # while True:
    #     roll = p.readUserDebugParameter(rollId)
    #     pitch = p.readUserDebugParameter(pitchId)
    #     yaw = p.readUserDebugParameter(yawId)
    #     x = p.readUserDebugParameter(fwdxId)
    #     y = p.readUserDebugParameter(fwdyId)
    #     z = p.readUserDebugParameter(fwdzId)

    #     orn = p.getQuaternionFromEuler([roll, pitch, yaw])
    #     p.resetBasePositionAndOrientation(ball, [x, y, z], orn)
        
    # print("==========Torque==========")
    # print(panda._target_torque)
    # print("============EEC============")
    # print(eec_panda._eec)



    t += STEPSIZE
    p.stepSimulation()
    time.sleep(STEPSIZE)

    R_past = copy.deepcopy(R)
    pos_past = copy.deepcopy(pos)
    R_e_past = copy.deepcopy(R_e)





    """ For data plotting
    """
    theta_bar = np.linalg.norm(eec_panda._eec)
    time_list.append(t)
    theta_bar_list.append(theta_bar)
    pos_error_list.append(np.linalg.norm(pos_error))
    joint_vel_list.append(np.linalg.norm(joint_vel))





plt.figure(1)
plt.plot(time_list, theta_bar_list)
plt.xlabel("Time(s)")
plt.ylabel("EEC error(rad)")
plt.title("EEC Error(theta_bar)")

plt.figure(2)
plt.plot(time_list, pos_error_list)
plt.xlabel("Time(s)")
plt.ylabel("Positional error(m)")
plt.title("Positional Error")

plt.figure(3)
plt.plot(time_list, joint_vel_list)
plt.xlabel("Time(s)")
plt.ylabel("Joint space velocity norm(/s)")
plt.title("Joint velocity norm")

plt.show()

