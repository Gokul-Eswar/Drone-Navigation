import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import asyncio
import threading
import uvicorn
import json
from typing import List

from fastapi import FastAPI, WebSocket, HTTPException, Request
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse

from ..drone_interfaces.mavros_interface import MAVROSInterface
from ..msg import DroneState

# --- Shared State & FastAPI App ---
# We create a simple class to hold state that needs to be shared
# between the ROS2 node thread and the FastAPI server thread.
class SharedState:
    def __init__(self):
        self.mavros_interface: MAVROSInterface = None
        self.active_connections: List[WebSocket] = []
        self.main_event_loop = None

shared_state = SharedState()
app = FastAPI(title="Indoor Drone Navigation Control")
# Note: The paths are relative to where the server is run from,
# which will be the workspace root (e.g., `ros2 run ...`).
# `setup.py` installs these files to `share/`, so we need to adjust this later if needed.
templates = Jinja2Templates(directory="gui_interface/web_gui/templates")

# --- FastAPI Endpoints ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    shared_state.active_connections.append(websocket)
    try:
        while True:
            await websocket.receive_text() # Just keep connection open
    except Exception:
        pass
    finally:
        if websocket in shared_state.active_connections:
            shared_state.active_connections.remove(websocket)

@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):
    return templates.TemplateResponse("dashboard.html", {"request": request})

# Define API endpoints for each drone action
@app.post("/api/arm")
async def arm():
    if shared_state.mavros_interface:
        await shared_state.mavros_interface.arm()
        return {"status": "arm goal sent"}
    raise HTTPException(status_code=503, detail="Drone interface not available.")

@app.post("/api/takeoff")
async def takeoff(request: Request):
    data = await request.json()
    altitude = data.get("altitude", 1.5)
    if shared_state.mavros_interface:
        await shared_state.mavros_interface.takeoff(altitude)
        return {"status": "takeoff goal sent"}
    raise HTTPException(status_code=503, detail="Drone interface not available.")

@app.post("/api/land")
async def land():
    if shared_state.mavros_interface:
        await shared_state.mavros_interface.land()
        return {"status": "land goal sent"}
    raise HTTPException(status_code=503, detail="Drone interface not available.")

@app.post("/api/emergency_stop")
async def emergency_stop():
    if shared_state.mavros_interface:
        await shared_state.mavros_interface.emergency_stop()
        return {"status": "emergency stop goal sent"}
    raise HTTPException(status_code=503, detail="Drone interface not available.")

# --- ROS2 Node ---
class GUIServerNode(Node):
    def __init__(self):
        super().__init__('gui_server_node')
        self.get_logger().info("GUI Server Node starting up...")

        shared_state.mavros_interface = MAVROSInterface(self)

        self.create_subscription(DroneState, '/indoor_drone/state', self._state_broadcast_callback, 10)

        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()

    def _run_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        shared_state.main_event_loop = loop
        uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")

    def _state_broadcast_callback(self, msg: DroneState):
        if not shared_state.main_event_loop or not shared_state.active_connections:
            return

        message = {
            'type': 'drone_status',
            'data': {
                'armed': msg.armed,
                'flight_mode': msg.flight_mode,
                'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                'battery_percentage': msg.battery_percentage,
            }
        }
        json_message = json.dumps(message)

        for ws in shared_state.active_connections:
            # Use run_coroutine_threadsafe to schedule the async send in the other thread's event loop
            asyncio.run_coroutine_threadsafe(ws.send_text(json_message), shared_state.main_event_loop)

def main(args=None):
    rclpy.init(args=args)
    node = GUIServerNode()
    # Use a MultiThreadedExecutor to handle the node's callbacks separately from the main spin
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
