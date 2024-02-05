import threading
import signal
import time
import sys
import json
import uvicorn
import argparse
from fastapi import FastAPI, Response, Request
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import List

import socket
from mimetypes import guess_type
from os.path import isfile

host_name = "127.0.0.1"
port = 23336
UDP_PORT_TX = 15006
UDP_PORT_RX = 15005
UDP_PORT_RX_blue = 15004

app = FastAPI()

cameraStat = ""

def serverThread():
    global cameraStat
    while True:
        try:
            data, addr = sock.recvfrom(4000)
            cameraStat = data.decode('utf-8')            
        except socket.error:
            pass

if __name__ != "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(0)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((host_name, UDP_PORT_TX))
    threading.Thread(target=serverThread, daemon=True).start()

class MotorCommand(BaseModel):
    driveID : int
    rotationTargetPosition : float
    rotationVelocity : float
    translationTargetPosition : float
    translationVelocity : float

class MotorCommands(BaseModel):
    commands : List[MotorCommand]

@app.get('/Camera/State')
async def camera_state():     
    #print(cameraStat)
    return Response(content=cameraStat, media_type="text/json")       

@app.post('/Motors/SendCommand')
async def send_command(request : Request, blue : bool | None = None): #MotorCommands): 
    if blue:        
        sock.sendto(await request.body(), (host_name, UDP_PORT_RX_blue)) 
    else:
        sock.sendto(await request.body(), (host_name, UDP_PORT_RX)) 
    return { "Result": "Setting reference..." }    

@app.get('/Competition')
async def competion_state():
   return { "state": 2, "time": 0, "playerName": 'SimBlue', "scorePlayer": 0, "scoreFuzbAI": 0, "level": 0, "results": [] }

@app.get('/{filename}')
async def serve_static(filename):     
    filename = './www/' + filename

    if not isfile(filename):
        return Response(status_code=404)

    with open(filename) as f:
        content = f.read()

    content_type, _ = guess_type(filename)
    return Response(content, media_type=content_type)


if __name__ == "__main__":        
    parser = argparse.ArgumentParser(prog='FuzbAI server')
    parser.add_argument('--port', type=int, default=port, help='HTTP REST port number')
    args = parser.parse_args()

    print(f"Starting server\n http://{host_name}:{args.port}/Render.html")
    uvicorn.run("FuzbAISimServer:app", port=args.port, log_level="warning")    
