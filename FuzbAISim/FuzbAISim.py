import pybullet as p
import pybullet_data
import time, datetime
import threading
import math
from FuzbAIAgent_Example import *
import random

class FuzbAISim:
    def __init__(self):
        print(" ______         _             _____ ")
        print("|  ____|       | |      /\   |_   _|")
        print("| |__ _   _ ___| |__   /  \    | |  ")
        print("|  __| | | |_  / '_ \ / /\ \   | |  ")
        print("| |  | |_| |/ /| |_) / ____ \ _| |_ ")
        print("|_|   \__,_/___|_.__/_/    \_\_____|")                                     
        print("")
        print("LAK FuzbAI simulator v1 - 2024")

        self.ballPos = None
        self.ballVel = None

        self.scoreDisp = None # Text used for score display

        # Text used for player status display
        self.playerStatusDisp1 = None
        self.playerStatusDisp2 = None

        self.rodPositions = [0]*8
        self.rodAngles = [0]*8
        
        self.score = [0,0]

        self.ballPosNoise = 5
        self.ballVelNoise = 0.01

        self.defaultBallPos = [0.718,0.71,0.3]

        # Player object indices in the URDF tree model
        self.redPlayers = [ 2, 5, 6, 14, 15, 16, 17, 18, 28, 29, 30 ]
        self.bluePlayers = [ 9, 10, 11, 21, 22, 23, 24, 25, 33, 34, 37 ]

        # Joint indices in the URDF tree model
        self.revJoints = [0, 3, 7, 12, 19, 26, 31, 35]
        self.slideJoints = [1, 4, 8, 13, 20, 27, 32, 36]

        self.travels = [190, 356, 180, 116, 116, 180, 356, 190]
        self.redIndices = [0, 1, 3, 5]

        self.p1 = PlayerAgent()
        self.p2 = PlayerAgent()

        # Camera delay settings
        self.simulatedDelay = 0.08
        self.delayedMemory = []
        self.maxMemory = 0.5 # Maximum delay time

        # Deadband settings
        self.prevRefPositions = [0]*8
        self.motionDirection = [1]*8
        self.motorDeadband = 0.005

        self.loadSimulator(False)

        self.isRunning = False

        self.t = 0

        self.status_player1 = 0
        self.status_player2 = 0

        self.motorCommandsExternal1 = []
        self.motorCommandsExternal2 = []

    def getCameraDict(self, player = 1):
        ball_x, ball_y = 1000*self.ballPos[0] - 115, 730 - 1000*self.ballPos[1]
        ball_vx, ball_vy = self.ballVel[0][0] + (random.random() - 0.5) * self.ballVelNoise, -self.ballVel[0][1] + (random.random() - 0.5) * self.ballVelNoise

        # Simple camera model
        camPos = [ [ 100, 350 ],  [ 1100, 350 ]] # Camera position
        z0 = 0.191 # z-position of the ball on the table

        camCorr = []
        ballSize = []

        for ci in range(2):
            dPos = [ ball_x - camPos[ci][0], ball_y - camPos[ci][1] ]
            d = math.sqrt(dPos[0]**2 + dPos[1]**2)    
            
            # Simulate shift in ball position based on ball's z-axis position
            camCorr.append([dPos[0] / d * (self.ballPos[2] - z0) * 100,   dPos[1] / d * (self.ballPos[2] - z0) * 100 ])
            ballSize.append(35 * 1e3/d)

        rp = self.rodPositions
        ra = self.rodAngles

        if player == 2:
            # Reverse the field
            ball_x = 1210 - ball_x
            ball_y = 700 - ball_y        

            ball_vx = -ball_vx
            ball_vy = -ball_vy
            
            rp = [1-rp[7-i] for i in range(8)]   
            ra = [-ra[7-i] for i in range(8)]     
        
        cam1 = { "cameraID": 0, 
                "ball_x": ball_x + camCorr[0][0] + (random.random() - 0.5) * self.ballPosNoise, "ball_y": ball_y + camCorr[0][1] + (random.random() - 0.5) * self.ballPosNoise, 
                "ball_vx": ball_vx, "ball_vy": ball_vy, "ball_size": ballSize[0], 
                "rod_position_calib": rp, "rod_angle": ra }

        cam2 = { "cameraID": 1, 
                "ball_x": ball_x + camCorr[1][0] + (random.random() - 0.5) * self.ballPosNoise, "ball_y": ball_y + camCorr[1][1] + (random.random() - 0.5) * self.ballPosNoise, 
                "ball_vx": ball_vx, "ball_vy": ball_vy, "ball_size": ballSize[1], 
                "rod_position_calib": rp, "rod_angle": ra }

        if player == 1:
            score = self.score.copy()  # Copy otherwise all sampled data will contain the same reference to an array which will update itself.
        else:
            score = self.score[::-1]

        return {"camData": [cam1, cam2], "camDataOK": [True, True], "score": score}

    def showScore(self):
        if self.scoreDisp is not None:
            p.removeUserDebugItem(self.scoreDisp)

        self.scoreDisp = p.addUserDebugText(f"Score {self.score[0]}:{self.score[1]}", [-0.1, -0.5, 0.1],
                                            textColorRGB=[0, 0, 0],
                                            textSize=2,
                                            parentObjectUniqueId=self.mizaId)

    def showPlayerStatus(self):
        if self.playerStatusDisp1 is not None:
            p.removeUserDebugItem(self.playerStatusDisp1)

        self.playerStatusDisp1 = p.addUserDebugText(f"Player 1: {'Demo' if self.status_player1 == 0 else 'External'}", [-0.8, -0.5, 0.1],
                                            textColorRGB=[1, 0, 0],
                                            textSize=1.5,
                                            parentObjectUniqueId=self.mizaId)

        if self.playerStatusDisp2 is not None:
            p.removeUserDebugItem(self.playerStatusDisp2)

        self.playerStatusDisp2 = p.addUserDebugText(f"Player 2: {'Demo' if self.status_player2 == 0 else 'External'}", [0.4, -0.5, 0.1],
                                            textColorRGB=[0, 0, 1],
                                            textSize=1.5,
                                            parentObjectUniqueId=self.mizaId)


    def sampleCameras(self, t):
        self.delayedMemory.append((t, self.getCameraDict(1), self.getCameraDict(2)))

        while len(self.delayedMemory) > 0 and t - self.delayedMemory[0][0] > self.maxMemory:
            self.delayedMemory.pop(0)

    def getDelayedCamera(self, player, t):
        if len(self.delayedMemory) == 0:
            return None

        for i in range(len(self.delayedMemory)):
            if self.delayedMemory[i][0] >= t:
                return self.delayedMemory[i][player]
    
        # Return first (the oldest) by default
        return self.delayedMemory[0][player]

    def nudgeBall(self):
        velocityNoise = 0.1
        p.resetBaseVelocity(self.ball, linearVelocity=[random.random()*velocityNoise,random.random()*velocityNoise,0])

    def applyMotorDeadband(self, i, newPos):    
        motionDiff = newPos - self.prevRefPositions[i]

        if motionDiff > self.motorDeadband and self.motionDirection[i] <= 0:
            # Changed direction - forward
            self.motionDirection[i] = 1
        elif motionDiff < -self.motorDeadband and self.motionDirection[i] >= 0:
            # Changed direction - backward
            self.motionDirection[i] = -1
        elif (self.motionDirection[i] >= 0 and motionDiff > 0) or (self.motionDirection[i] <= 0 and motionDiff < 0):
            # Change nothing - motion in the same direction
            pass    
        else:
            # Inside the deadband - ignore motion
            return self.prevRefPositions[i]

        self.prevRefPositions[i] = newPos
        return newPos

    def loadSimulator(self, printJointInfo = False):
        print("Loading simulator...")
        physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version

        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME,0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS,1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING,1)
        p.setPhysicsEngineParameter(enableFileCaching=0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
        p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0,cameraPitch=-80, cameraTargetPosition=[0.72,0.375,0])

        p.setGravity(0,0,-9.8)

        # Load basic plane
        planeId = p.loadURDF("plane.urdf")

        shift = [0,0,0]
        meshScale = [0.001, 0.001, 0.001]

        mizaStartPos = [0,0,0]
        mizaStartOrientation = p.getQuaternionFromEuler([0,0,0])

        # Import collision model - miza
        print("Loading FuzbAI table model...")
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                            fileName="meshes/Miza.obj",
                                            rgbaColor=[1, 1, 1, 1],
                                            specularColor=[0.4, .4, 0],
                                            visualFramePosition=[0,0,0],
                                            meshScale=[1e-5, 1e-5, 1e-5]) # Very small visual model
                                        
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                            fileName="meshes/Miza.obj",
                                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                                            collisionFramePosition=shift,
                                            meshScale=meshScale)

        mizaCollisionId = p.createMultiBody(baseMass=0,
                                            baseInertialFramePosition=[0, 0, 0],
                                            baseOrientation=p.getQuaternionFromEuler([math.pi/2,0,math.pi/2]),
                                            baseCollisionShapeIndex=collisionShapeId,
                                            baseVisualShapeIndex=visualShapeId,
                                            basePosition=[0,0,0],
                                            useMaximalCoordinates=True)
        
        # Import main URDF model
        self.mizaId = p.loadURDF("urdf/miza_garlando.urdf",mizaStartPos, mizaStartOrientation, useFixedBase=1)

        if printJointInfo:
            jointsNum = p.getNumJoints(self.mizaId)
            for i in range(jointsNum):
                jInfo = p.getJointInfo(self.mizaId, i)
                print(jInfo)

        # Load the ball
        print("Loading ball model...")
        self.ball = p.loadURDF("sphere_small.urdf", self.defaultBallPos, p.getQuaternionFromEuler([0,0,0]), globalScaling = 0.035 / (2*0.03)) # sphere_small has radius of 3 cm
        p.changeVisualShape(self.ball, -1, rgbaColor=[1,1,0,1])

        # Adjust dynamics
        p.changeDynamics(self.ball, -1, mass=0.0287, lateralFriction=0.2, rollingFriction=0.00005, spinningFriction=0.01, restitution=0.7, linearDamping=0)
        p.changeDynamics(mizaCollisionId, -1, restitution=0.8)

        # Set player colors
        for rp in self.redPlayers:
            p.changeVisualShape(self.mizaId, rp, rgbaColor=[1,0,0,1])

        for bp in self.bluePlayers:
            p.changeVisualShape(self.mizaId, bp, rgbaColor=[0,0,1,1])

        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,cameraPitch=-80, cameraTargetPosition=[0.72,0.375,0])

        # Enable realtime simulation
        p.setRealTimeSimulation(1)
        #p.setTimeStep(1/200) # Not working with realtime simulation

    def run(self):
        #self.simThread = threading.Thread(target=lambda: self.__run(), daemon=True)        
        #self.simThread.start()
        threading.Thread(target=self.__run).start()

    def stop(self):
        self.isRunning = False

    def __run(self):
        t0 = time.time()
        self.isRunning = True

        refPos = 0
        prev_t = 0
        ball_moving = 0
            
        print(f'\n*********************************\nStarting main loop\n*********************************\n')

        self.showScore()
        self.nudgeBall()
        self.showPlayerStatus()

        prev_key_t = 0

        try:    
            while self.isRunning:    
                self.ballPos, ballOrn = p.getBasePositionAndOrientation(self.ball)        
                self.ballVel = p.getBaseVelocity(self.ball)

                if self.ballPos[2] < 0.1:
                    #print(ballPos)
                    # Is the ball under the table?
                    if (self.ballPos[0] > 0 and self.ballPos[0] < 1.4 and self.ballPos[1] > 0 and self.ballPos[1] < 0.7):
                        # On which side?
                        if self.ballPos[0] < 0.72:
                            # Blue scored a goal
                            self.score[1] += 1
                            print(f'Blue scored goal ({self.score[0]}:{self.score[1]})')
                        else:
                            # Red scored a goal
                            self.score[0] += 1
                            print(f'Red scored goal ({self.score[0]}:{self.score[1]})')

                        self.showScore()

                    # Reset the ball  
                    print("Dropping ball at start location")   
                    p.resetBasePositionAndOrientation(self.ball, self.defaultBallPos, p.getQuaternionFromEuler([0,0,0]))          
                    self.nudgeBall()
                
                self.t = time.time() - t0

                if math.sqrt(self.ballVel[0][0]**2 + self.ballVel[0][1]**2) > 0.05:
                    ball_moving = self.t

                if self.t - ball_moving > 3:
                    # Ball is not moving - move it to a random location
                    print("Ball stationary, dropping to a random location")    
                    ball_moving = self.t
                    
                    p.resetBasePositionAndOrientation(self.ball, [0.718 + 0.6*random.random(), 0.71 - random.random()*0.7, 0.3], p.getQuaternionFromEuler([0,0,0]))
                    self.nudgeBall()

                angles = []
                rodPoses = []     
                
                for ji in range(8):        
                    angles.append(32*p.getJointState(self.mizaId, self.revJoints[ji])[0] / math.pi)

                # Linear...
                for ji in range(8):                         
                    rodPoses.append(1-1000*p.getJointState(self.mizaId, self.slideJoints[ji])[0] / self.travels[ji])
                
                self.rodPositions = rodPoses
                self.rodAngles = angles

                linVel = 1.5910861528058136
                rotVel = 174.74649915501303

                # Process the agents...
                if self.t - prev_t > 0.02:  
                    try:     
                        if self.status_player1 == 0:             
                            motors1 = self.p1.process_data(self.getDelayedCamera(1, self.t - self.simulatedDelay))
                        else:
                            # Use the external motor data...
                            motors1 = self.motorCommandsExternal1
                            self.motorCommandsExternal1 = []        

                        driveMap = [0, 1, 3, 5]
                        for m in motors1:
                            axisID = driveMap[m["driveID"]-1]
                            jId_rot = self.revJoints[axisID]
                            jId_lin = self.slideJoints[axisID]

                            refAngle = m["rotationTargetPosition"]*2*math.pi                
                            p.setJointMotorControl2(self.mizaId, jId_rot, controlMode=p.POSITION_CONTROL, targetPosition=refAngle, force=2.0943448919793832, maxVelocity=rotVel*m["rotationVelocity"], positionGain=2.817867199313025, velocityGain=7.574019729635704)

                            refPos = self.applyMotorDeadband(axisID, self.travels[axisID]*(1-m["translationTargetPosition"])/1000)
                            p.setJointMotorControl2(self.mizaId, jId_lin, controlMode=p.POSITION_CONTROL, targetPosition=refPos, force=13.303989530423438, maxVelocity=linVel*m["translationVelocity"], positionGain=0.19343157707177333, velocityGain=3.9227062400839023)
                    except:
                        print("Exception in agent 1")

                    try:                                               
                        if self.status_player2 == 0:             
                            motors2 = self.p2.process_data(self.getDelayedCamera(2, self.t - self.simulatedDelay))
                        else:
                            # Use the external motor data...
                            motors2 = self.motorCommandsExternal2
                            self.motorCommandsExternal2 = []        

                        driveMap = [7, 6, 4, 2]                        
                        for m in motors2:
                            axisID = driveMap[m["driveID"]-1]
                            jId_rot = self.revJoints[axisID]
                            jId_lin = self.slideJoints[axisID]

                            refAngle = -m["rotationTargetPosition"]*2*math.pi                
                            p.setJointMotorControl2(self.mizaId, jId_rot, controlMode=p.POSITION_CONTROL, targetPosition=refAngle, force=2.0943448919793832, maxVelocity=rotVel*m["rotationVelocity"], positionGain=2.817867199313025, velocityGain=7.574019729635704)

                            refPos = self.applyMotorDeadband(axisID, self.travels[axisID]*(m["translationTargetPosition"])/1000)
                            p.setJointMotorControl2(self.mizaId, jId_lin, controlMode=p.POSITION_CONTROL, targetPosition=refPos, force=13.303989530423438, maxVelocity=linVel*m["translationVelocity"], positionGain=0.19343157707177333, velocityGain=3.9227062400839023)
                    except:
                        print("Exception in agent 2")

                    prev_t = self.t        

                self.sampleCameras(self.t)
                #print("States: ", rodPositions, rodAngles)
                #print(p.getLinkState(mizaId, 3))

                keys = p.getKeyboardEvents()
                if self.t - prev_key_t > 0.1:
                    for k, v in keys.items():        
                        if (k == 65309 and (v & p.KEY_WAS_TRIGGERED)): # 65309 == enter
                            # Move the ball over the table
                            p.resetBasePositionAndOrientation(self.ball, self.defaultBallPos, p.getQuaternionFromEuler([0,0,0]))
                        if (k == 32): # Esc
                            running = False 
                            break            

                        # Enable/disable player 1
                        if (k == 49): # 1
                            self.status_player1 = (self.status_player1 + 1) % 2
                            self.showPlayerStatus()
                            pass    

                        # Enable/disable player 2
                        if (k == 50): # 2
                            self.status_player2 = (self.status_player2 + 1) % 2
                            self.showPlayerStatus()
                            pass    

                if len(keys) > 0:
                    prev_key_t = self.t

            print("Stopping simulation...")
            p.disconnect()
            print("Stopping server...")
            print("Done")

        except KeyboardInterrupt:
            print("Stopping simulation on keyboard interrupt...")
            p.disconnect()
            print("Stopping server...")

        except:
            pass

        print(f'Main loop stopped')

        self.isRunning = False

if __name__ == "__main__":
    sim = FuzbAISim()
    sim.run()

    while sim.isRunning:
        pass
