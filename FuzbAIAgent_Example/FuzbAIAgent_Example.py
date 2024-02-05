import requests
import json
import time
import math
import random

HOST_ADDRESS = '127.0.0.1:23336'

def get_camera_state():
    cam_url = f"http://{HOST_ADDRESS}/Camera/State"
    response = requests.get(cam_url)    
    return response.json()

def send_motor_commands(cmds):
    motors_url = f"http://{HOST_ADDRESS}/Motors/SendCommand?blue=False"
    response = requests.post(motors_url, json=cmds)

# Main player agent class
class PlayerAgent():
    def __init__(self):
        # Load field geometry from json
        # origin in top-left corner, x towards right, y towards down (as seen in the simulator)   
        f = open('geometry.json')
        self.geometry = json.load(f)
        f.close()
        
        # Geometry contents:
        # - field: field dimensions (in mm)
        # - rods: array of player rod information
        #         * id: 1 to 8 (1 is the red goalie)
        #         * team: red or blue
        #         * position: x-axis position of the rod
        #         * travel: travel range (in mm) of the rod
        #         * players: number of players on the rod
        #         * first_offset: y-axis position of the first player center
        #         * spacing: spacing between players on the rod

        self.demo_state = 0
        self.demo_t = 0

    def process_data(self, camera):
        # Rod 0 = drive 1
        # Rod 1 = drive 2
        # Rod 3 = drive 3
        # Rod 5 = drive 4
        playerMapping = [1, 2, -1, 3, -1, 4, -1, -1]        

        # Process camera data
        CD0 = camera["camData"][0]
        CD1 = camera["camData"][1]        
        # Input data in each of the CD0/CD1:
        #  * ball_x, ball_y: position of the ball (in mm)
        #  * ball_vx, ball_vy: velocity of the ball (in m/s)
        #  * ball_size: area of the ball in the captured image (in pixels)
        #  * rod_position_calib: calibrated position of the rod (in interval [0,1])
        #  * rod_angle: calibrated angle of the rod (in interval [-32,+32])

        # ToDo: combine data from CD0 and CD1
        # In case the ball is obstructed by the player or the rod, the quality of the ball position/velocity information will be reduced
        # System reports the visible area of the ball via the parameters CD0["ball_size"] and CD1["ball_size"] - the value of those can be used to 
        # combine information

        # Ball position
        bx = CD0["ball_x"]
        by = CD0["ball_y"]

        # Ball velocity
        vx = CD0["ball_vx"]
        vy = CD0["ball_vy"]
        #print(bx, by, vx, vy)

        commands = []
        for i in range(8): # index i indicates the rod number (0 = red goalie...)
            # Rod i geometry
            rg = self.geometry["rods"][i]

            # The opponent?
            if playerMapping[i] < 0:
                continue

            # If ball inside the field...
            if (by > 0 and by < self.geometry["field"]["dimension_y"]):            
                
                for ip in range(rg["players"]):                
                    # Determine the position of all player figures
                    pos = rg["travel"] * CD0["rod_position_calib"][i]       # Current rod offset in mm            
                    p_pos = rg["first_offset"] + pos + ip * rg["spacing"]   # y-position of the ip-th player on the rod

                
            if i == 0: # The logic below for the goalie only
                if self.demo_state == 0:
                    if time.time() - self.demo_t < 5:
                        # Translational move                    
                        cmd = {
                                "driveID": playerMapping[i],
                                "rotationTargetPosition": 0.0,      # Normal position
                                "rotationVelocity": 0.2,            # Reduced rotational speed
                                "translationTargetPosition": 0.5 + math.sin(time.time() - self.demo_t) * 0.5,
                                "translationVelocity": 1.0 }        # Max translational speed
                        # Request the motion
                        commands.append(cmd)

                    else:
                        self.demo_t = time.time()
                        self.demo_state = 1

                        # Start the ball kick - pull the legs back
                        cmd = {
                                "driveID": playerMapping[i],
                                "rotationTargetPosition": 0.5,      # Legs back...
                                "rotationVelocity": 1,              # Max rotation speed
                                "translationTargetPosition": 0.5,   # Mid motion
                                "translationVelocity": 1.0 }        # Max translational speed
                        # Request the motion
                        commands.append(cmd)

                elif self.demo_state == 1:
                    if time.time() - self.demo_t > 0.05:
                        self.demo_t = time.time()                    
                        self.demo_state = 2

                        # Kick!
                        cmd = {
                                "driveID": playerMapping[i],
                                "rotationTargetPosition": -0.5,     # Forward kick
                                "rotationVelocity": 1,              # Max rotation speed
                                "translationTargetPosition": 0.5,   # Mid motion
                                "translationVelocity": 1.0 }        # Max translational speed
                        # Request the motion
                        commands.append(cmd)
                    
                elif self.demo_state == 2:
                    if time.time() - self.demo_t > 0.2:
                        self.demo_t = time.time()                    
                        self.demo_state = 0

                        # Back to normal position!
                        cmd = {
                                "driveID": playerMapping[i],
                                "rotationTargetPosition": 0,        # Normal position
                                "rotationVelocity": 0.2,            # Reduced speed
                                "translationTargetPosition": 0.5,   # Mid motion
                                "translationVelocity": 1.0 }        # Max translational speed
                        # Request the motion
                        commands.append(cmd)
        
        return commands

# Instantiate the agent
agent = PlayerAgent()

while True:
    time.sleep(0.02)

    # Get camera data
    camData = get_camera_state()    

    CD0 = camData["camData"][0]
    CD1 = camData["camData"][1]        

    bx = CD0["ball_x"]
    by = CD0["ball_y"]

    print(f"Ball: {bx} / {by}")

    # Process the camera data and return motor commands
    motorData = agent.process_data(camData)

    # Send the motor commands
    send_motor_commands({'commands': motorData })