import os
import pybullet as p
import pybullet_data
import numpy as np

# 物理シミュレーションとの接続
physicsClient = p.connect(p.GUI)

# pybullet_dataをパスに追加: モデルファイル読み込みのため
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 重力加速度の設定
p.setGravity(0, 0, -9.81)

# シミュレーションの刻み幅の設定
sampling_time = 0.01
p.setTimeStep(sampling_time)

# シミュレーションのリアルタイム描画設定
p.setRealTimeSimulation(1)

# 床モデルの読み込み
planeId = p.loadURDF("plane.urdf")

# ロボットモデルの読み込み
#botId = p.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf")
botId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])

# ロボット情報の取得
joint_num = p.getNumJoints(botId)
print(joint_num)
for i in range (joint_num):
    print(p.getJointInfo(botId, i))

# Step 1: 初期目標姿勢まで移動させる
is_initialization_end = False
joint_positions_initial = [-90.0, 30.0, 0.0, -50.0, 30.0, 30.0, 30.0]                           # degree
joint_positions_initial = [joint_positions_initial[i]*np.pi/180.0 for i in range(joint_num)]    # rad
print("Initialization start")
while is_initialization_end == False:

    # 初期姿勢までPTP
    joint_positions = [j[0] for j in p.getJointStates(botId, range(p.getNumJoints(botId)))]
    for i in range(p.getNumJoints(botId)):
        error = joint_positions_initial[i]-joint_positions[i]
        velocity_reference = 5.0 * error
        p.setJointMotorControl2(bodyUniqueId=botId,
                                jointIndex=i,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=velocity_reference)

    # シミュレーションの更新
    p.stepSimulation()

    if velocity_reference <= 0.01:
        is_initialization_end = True
        print("Initialization end")

# Step 2: シミュレーション開始
print("Simulation start")
for step in range(1000):

    # シミュレーション内の時間を計算
    time = step * sampling_time

    # ベース位置・姿勢の現在情報の取得
    basePos, baseOrn = p.getBasePositionAndOrientation(botId) # ベース位置・姿勢(クォータニオン)の計算
    baseEuler = p.getEulerFromQuaternion(baseOrn)             # クォータニオンからベースオイラー角を計算
    baseOrnMatrix = p.getMatrixFromQuaternion(baseOrn)        # クォータニオンから姿勢行列を計算
    baseVel, baseAngularVel = p.getBaseVelocity(botId)        # ベース速度・角速度を取得

    # 関節角度の取得
    joint_positions = [j[0] for j in p.getJointStates(botId, range(p.getNumJoints(botId)))]
    joint_velocities = [j[1] for j in p.getJointStates(botId, range(p.getNumJoints(botId)))]

    # エンドエフェクタの位置・姿勢を取得 (最終リンクの原点位置)
    kukaEndEffectorIndex = 6
    ee_states = p.getLinkState(botId, kukaEndEffectorIndex)
    eePos = ee_states[4]                            # エンドエフェクタ位置の計算
    eeOrn = ee_states[5]                            # エンドエフェクタのクォータニオンを取得
    eeEuler = p.getEulerFromQuaternion(eeOrn)       # クォータニオンからオイラー角を計算
    eeOrnMatrix = p.getMatrixFromQuaternion(eeOrn)  # クォータニオンから姿勢行列を計算

    if step == 0:
        eePosIni = eePos
        eeOrnIni = eeOrn
        eePosPrev = eePos

    # 手先目標位置を計算
    eePosCmd = list(eePosIni)
    eeOrnCmd = eeOrnIni

    eePosCmd[0] = eePosIni[0] + 0.1 * np.sin(2.0 * np.pi * 1.0 * time)
    #eePosCmd[1] = eePosIni[1] + 0.2 * np.cos(2.0 * np.pi * 1.0 * time)
    #eePosCmd[2] = eePosIni[2] #+ 0.2 * np.cos(2.0 * np.pi * 1.0 * time)

    # 逆運動学を計算

    joint_positions_ref = p.calculateInverseKinematics(botId, kukaEndEffectorIndex, eePosCmd, eeOrnCmd);
    #joint_positions_ref = p.calculateInverseKinematics(botId, kukaEndEffectorIndex, eePosRef, eeOrnRef, lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp)

    # 制御入力の決定 (位置制御)
    for i in range(p.getNumJoints(botId)):
        p.setJointMotorControl2(bodyUniqueId=botId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_positions_ref[i])

    """
    # 制御入力の決定 (速度制御)
    for i in range(p.getNumJoints(botId)):
        p.setJointMotorControl2(bodyUniqueId=botId,
                                jointIndex=i,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=10.0*(joint_positions_ref[i]-joint_positions[i]))
    """

    """
    # 逆動力学計算
    joint_accel_ref = [0.0 for i in range(joint_num)]
    for i in range(joint_num):
        joint_accel_ref[i] = 100.0*(0.3-joint_positions[i]) + 20.0 * (0.0 - joint_velocities[i])
    torque = p.calculateInverseDynamics(botId, joint_positions, joint_velocities, joint_accel_ref)
    # 制御入力の決定 (力制御)
    for i in range(p.getNumJoints(botId)):
        p.setJointMotorControl2(bodyUniqueId=botId,
                                jointIndex=i,
                                controlMode=p.TORQUE_CONTROL,
                                force=torque[i])
    """

    # キーボード入力の読み込み
    keys = p.getKeyboardEvents()
    if keys == "q":
        break

    # 手先の移動軌跡を描画 (デバッグ用)
    p.addUserDebugLine(lineFromXYZ=eePosPrev, lineToXYZ=eePos, lineColorRGB=[0.5, 0, 0], lineWidth=10, lifeTime=3)

    # シミュレーションの更新
    p.stepSimulation()

    eePosPrev = eePos

print("Simulation end")

# Step 3: 物理シミュレーションとの切断
p.disconnect()
