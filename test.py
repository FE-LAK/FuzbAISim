import threading
import signal
import time
import sys
import json
from flask import Flask, jsonify, Response
from waitress import serve
import FuzbAISim
import uvicorn
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

# installer: pyinstaller --hidden-import numpy  test.py

#app = Flask(__name__, static_url_path='', static_folder='www')

host_name = "127.0.0.1"
port = 23336


'''
@app.route('/Camera/State')
def camera_state():    
    #return jsonify(getCameraDict())    
    return Response(response=json.dumps(getCameraDict()), status=200, mimetype="application/json")

@app.route('/Competition')
def competion_state():
   return jsonify({ "state": 2, "time": 0, "playerName": 'SimBlue', "scorePlayer": score[1], "scoreFuzbAI": score[0], "level": 0, "results": [] })
'''

#sys.exit(1)
#flaskThread = threading.Thread(target=lambda: app.run(host=host_name, port=port, debug=False, use_reloader=False, threaded=True), daemon=True)
#flaskThread = threading.Thread(target=lambda: serve(app, host=host_name, port=port,threads= 8), daemon=True)
#flaskThread.start()

'''
async def app(scope, receive, send):
    """
    Echo the method and path back in an HTTP response.
    """
    assert scope['type'] == 'http'

    body = f'Received {scope["method"]} request to {scope["path"]}'
    await send({
        'type': 'http.response.start',
        'status': 200,
        'headers': [
            [b'content-type', b'text/plain'],
        ]
    })
    await send({
        'type': 'http.response.body',
        'body': body.encode('utf-8'),
    })
'''

app = FastAPI()
#app.mount("/", StaticFiles(directory="www"), name="static")

print("#################################################", __name__)

if __name__ != "__main__":        
    sim = FuzbAISim.FuzbAISim()
    sim.run()


@app.get('/Camera/State')
async def camera_state(t):    
    return sim.getCameraDict()
    #return jsonify(getCameraDict())    
    #return Response(response=json.dumps(getCameraDict()), status=200, mimetype="application/json")

@app.get('/Competition')
async def competion_state(noResults, t):
   return { "state": 2, "time": 0, "playerName": 'SimBlue', "scorePlayer": sim.score[1], "scoreFuzbAI": sim.score[0], "level": 0, "results": [] }

if __name__ == "__main__":        
    print("Starting server...")
    uvicorn.run("test:app", port=5000, log_level="info")    
