import socket
import FuzbAISim
import time
import json

# installer: pyinstaller --hidden-import numpy  FuzbAISimWrapper.py

UDP_IP = "127.0.0.1"
UDP_PORT_TX = 15006
UDP_PORT_RX1 = 15005
UDP_PORT_RX2 = 15004

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock1.setblocking(0)
sock1.bind((UDP_IP, UDP_PORT_RX1))

sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2.setblocking(0)
sock2.bind((UDP_IP, UDP_PORT_RX2))

# Start simulator
sim = FuzbAISim.FuzbAISim()
sim.run()

time.sleep(0.1)

# Star the update thread
while sim.isRunning:    
    #time.sleep(0.02)

    # Send camera status
    try:
        camStat = sim.getDelayedCamera(1, sim.t - sim.simulatedDelay)
        sock.sendto(bytes(json.dumps(camStat), 'utf-8'), (UDP_IP, UDP_PORT_TX))        
    except:
        pass

    try:
        while True:
            data, addr = sock1.recvfrom(5000)
            motorData = json.loads(data.decode('utf-8'))
            
            # Parse motor data
            sim.motorCommandsExternal1 = motorData["commands"]
    except socket.error:
        pass    

    try:
        while True:
            data, addr = sock2.recvfrom(5000)
            motorData = json.loads(data.decode('utf-8'))
            
            # Parse motor data
            sim.motorCommandsExternal2 = motorData["commands"]
    except socket.error:
        pass    

print("###########################################\nSim stopped")