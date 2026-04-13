import json
import time
import math
import random

class PlayerAgent():
    def __init__(self):

        # Load geometry json
        f = open('www/geometry.json')
        self.geometry = json.load(f)
        f.close()

        self.reset()
        pass

    def setState(self, i, state):
        #print(f"Player {i}: {self.player_state[i]} -> {state}")
        self.player_state[i] = state
        self.player_timer[i] = time.time()

    def reset(self):
        self.player_state = [0, 0, 0, 0, 0, 0, 0, 0]
        t = time.time()
        self.player_timer = [t, t, t, t, t, t, t, t]
        self.player_refangle = [0,0,0,0,0,0,0,0]
        self.player_velangle = [0,0,0,0,0,0,0,0]        
        

    def process_data(self, camera): # Return dict
        playerMapping = [1, 2, -1, 3, -1, 4, -1, -1]        

        CD0 = camera["camData"][0]
        CD1 = camera["camData"][1]        

        bx = CD0["ball_x"]
        by = CD0["ball_y"]

        vx = CD0["ball_vx"]
        vy = CD0["ball_vy"]

        #print(bx, by, vx, vy)

        power = 0.8
        fullpower = 0.2

        commands = []

        for i in range(8):
            rg = self.geometry["rods"][i]

            HPy = by
            HPx = bx

            #if i != 3:
            #    continue

            # Is ball moving into the right direction?
            dx = (rg["position"] - bx)
            t = 0
            if abs(vx) > 0.1:
                t = dx / vx
            
            # The opponent?
            if playerMapping[i] < 0:
                continue

            kickTheBall = False
            pullUpTheLegsForward = False
            pullUpTheLegs = False
            releaseLegs = False
            softlyKickBackwards = False

            if i > 0:
                if (dx < 300):
                    if (dx > 100 + vx * 0.5 and vx > 0.2):
                        pullUpTheLegs = True
                    elif (dx > 50 and vx > 0.2):
                        pullUpTheLegsForward = True                    
                
                if (dx < -50 or abs(vx) < 0.1):
                    releaseLegs = True                

                #print(i, dx)
                if (dx > 0 and dx < 50 and abs(vx) < 0.15):
                    softlyKickBackwards = True                            

            # Find the point where the ball with intersect with the player's line            
            HPy = by + vy * (t + 25) # Advance the position for 1 second
            if (HPy < 0):
                HPy = 0
            if (HPy > self.geometry["field"]["dimension_y"]):
                HPy = self.geometry["field"]["dimension_y"]

            HPx = bx + vx *(t + 25) # Advance the position for 1 second
            if (HPx < 0):
                HPx = 0
            if (HPx > self.geometry["field"]["dimension_x"]):
                HPx = self.geometry["field"]["dimension_x"]            

            if (HPy > 0 and HPy < self.geometry["field"]["dimension_y"]):            
                # Find which player is the closest one to the ball
                minD = 1000
                minPlayer = -1
                
                for ip in range(rg["players"]):                
                    # Distance from the current player's position
                    pos = rg["travel"] * CD0["rod_position_calib"][i]                    
                    p_pos = rg["first_offset"] + pos + ip * rg["spacing"]

                    d = HPy - p_pos

                    # Now determine if the player can move to the position...
                    pos2 = pos + d
                    rod_pos2 = pos2 / rg["travel"]

                    if (rod_pos2 < 0):
                        if (ip < rg["players"] - 1):
                            d = abs(d) + abs(rod_pos2 * rg["travel"])
                        else:
                            if (minPlayer >= 0):
                                continue               
                    elif (rod_pos2 > 1):
                        if (ip > 0):
                            d = abs(d) + abs((rod_pos2 - 1) * rg["travel"])
                        else:
                            if (minPlayer >= 0):
                                continue

                    if abs(d) < minD:
                        minD = abs(d)
                        minPlayer = ip

                if (minPlayer < 0):
                    # No player can cover this one...
                    pass
                

                if (minPlayer >= 0):
                    pos = rg["travel"] * CD0["rod_position_calib"][i]
                    p_pos = rg["first_offset"] + pos + minPlayer * rg["spacing"]

                    #if i == 2:
                    #    print(i, minD, bx, by, rg["position"])

                    if (minD < 10) and (abs(bx - rg["position"] - 25) < 30): # and (i != 5)):
                        kickTheBall = True

                    if (self.player_state[i] == 20 or self.player_state[i] == 21):
                        if (minD < 10 and abs(bx - rg["position"]) < 35):
                            kickTheBall = True
                    
                    rodPos = (HPy - rg["first_offset"] - minPlayer * rg["spacing"]) / rg["travel"]
                    if (rodPos > 1):
                        rodPos = 1
                    elif (rodPos < 0):
                        rodPos = 0                
                    
                    #print(f"Player {minPlayer} can cover with mind={minD} -> rodPos={rodPos}")

                    # Attack goal center
                    if (i == 5):
                        ready_attack_on_goal = False
                        kick_ball_goal_center = False

                        ball_center = [0,0]

                        # When ball is close to player use the actual ball position.                        
                        if (((bx - rg["position"]) > 50*0) and (abs(1000*vx)>20) and (1000*vx < 0)): #-1 to avoid devide by 0
                            #print('Attack center intersection prediction')
                            ball_center = [rg["position"], by + vy/(vx+0.002)*(rg["position"]- bx)]
                        else:
                            ball_center = [bx, by]                                          

                        rod_optimal = [ball_center[0], ball_center[1]]

                        rodPos = (rod_optimal[1] - rg["first_offset"] - minPlayer * rg["spacing"]) / rg["travel"]
                        # This player can not move that low! Change to lower player
                        if ((rodPos < 0) and (minPlayer != 0)):
                            tmp_rodPos = (rod_optimal[1] - rg["first_offset"] - (minPlayer - 1) * rg["spacing"]) / rg["travel"]
                            if (tmp_rodPos > 0):
                                minPlayer = minPlayer - 1
                                rodPos = tmp_rodPos

                        # This player can not move that high! Change to higher player!
                        if ((rodPos > 1) and (minPlayer != 2)):
                            tmp_rodPos = (rod_optimal[1] - rg["first_offset"] - (minPlayer + 1) * rg["spacing"]) / rg["travel"]
                            if (tmp_rodPos < 1):
                                minPlayer = minPlayer + 1
                                rodPos = tmp_rodPos

                        if (rodPos > 1):
                            rodPos = 1
                        elif (rodPos < 0):
                            rodPos = 0
                                
                    curPower = power

                    if random.random() < fullpower:
                        curPower = 1.0

                    if curPower < 0.05:
                        curPower = 0.05

                    if self.player_state[i] == 0:
                        if (kickTheBall): # and (i!=5)):
                            # Wait a bit
                            self.setState(i, 1)
                            #print("Kick!!!")

                            # Inform the other players to pull-up the legs...
                            for j in range(i + 1, 8):
                                if (playerMapping[j] < 0):
                                    continue

                                self.player_timer[j] = time.time() + 0.5
                                self.player_state[j] = 20
                                self.player_velangle[j] = 0.2
                                self.player_refangle[j] = 0.4                                
                        elif ((i == 5) and ready_attack_on_goal):
                            self.setState(i, 41)
                        
                        elif (pullUpTheLegsForward):
                            # Pull up the legs (forward)
                            self.setState(i, 21)
                            self.player_velangle[i] = 0.2
                            self.player_refangle[i] = -0.4
                        elif (pullUpTheLegs):
                            # Pull up the legs (backwards)
                            self.setState(i, 20)
                            self.player_velangle[i] = 0.2
                            self.player_refangle[i] = 0.4
                        elif (softlyKickBackwards):
                            # Slowly kick backwards
                            self.setState(i, 3)
                            self.player_velangle[i] = 0.2
                            self.player_refangle[i] = 0.2                            
                    elif self.player_state[i] == 1:
                        if ((time.time() - self.player_timer[i]) > 0.025):
                            # Arm the player
                            self.setState(i, 2)
                            self.player_velangle[i] = 1.0 * curPower
                            self.player_refangle[i] = 0.5

                    elif self.player_state[i] == 2:
                        if ((time.time() - self.player_timer[i]) > 0.050):
                            # Kick!
                            self.setState(i, 3)
                            self.player_velangle[i] = 1.0 * curPower
                            self.player_refangle[i] = -0.5

                    elif self.player_state[i] == 3:
                        if ((time.time() - self.player_timer[i]) > 0.2):
                            # Return to normal
                            self.setState(i, 10)
                            self.player_velangle[i] = 0.5 * curPower
                            self.player_refangle[i] = 0

                    elif self.player_state[i] == 10:
                        if ((time.time() - self.player_timer[i]) > 0.15):
                            # Wait for it to return
                            self.setState(i, 0)

                    elif self.player_state[i] == 20:
                        if (kickTheBall):
                            # Kick!
                            self.setState(i, 3)
                            self.player_velangle[i] = 1.0 * curPower
                            self.player_refangle[i] = -0.5

                    elif self.player_state[i] == 21:
                        if (releaseLegs and (time.time() - self.player_timer[i]) > 0.25):
                            # Return to normal
                            self.setState(i, 10)
                            self.player_velangle[i] = 0.5 * curPower
                            self.player_refangle[i] = 0

                    elif self.player_state[i] == 41:
                            # Rod 6 attack goal -- arm the player ahead of time.
                            if (time.time() - self.player_timer[i]) > 0.05:
                                # Arm the player
                                print('Ready attacker for kick')
                                self.setState(i, 42)
                                self.player_velangle[i] = 1.0 * curPower
                                self.player_refangle[i] = 0.5

                    elif self.player_state[i] == 42:
                        # Wait for the ball to be in position!
                        if (kick_ball_goal_center and ((time.time() - self.player_timer[i]) > 0)):
                            print('Attacker kick')
                            self.setState(i, 3)
                            self.player_velangle[i] = 1.0 * curPower
                            self.player_refangle[i] = -0.5
                        elif ((not ready_attack_on_goal) and ((time.time() - self.player_timer[i]) > 0.1)):
                            # Abort kick! The ball got away!
                            # Return to normal
                            print('Return attacker to normal')
                            self.setState(i, 3)

                    cmd = {
                        "driveID": playerMapping[i],
                        "rotationTargetPosition": self.player_refangle[i],
                        "rotationVelocity": self.player_velangle[i],
                        "translationTargetPosition": rodPos,
                        "translationVelocity": 1.0 }

                    #print(cmd)

                    # Request the motion
                    commands.append(cmd)

        return commands