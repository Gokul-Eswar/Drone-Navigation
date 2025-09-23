from fastapi import FastAPI, WebSocket, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
import asyncio
import json
from typing import Dict, List, Any

# Assuming these are in the correct path relative to src
from ..mission_planning.intelligent_mission_planner import IntelligentMissionPlanner, MissionPlan
from ..drone_interfaces.universal_interface import MAVROSInterface, UniversalDroneInterface
from ..safety_systems.advanced_safety_monitor import AdvancedSafetyMonitor

# --- Placeholder/Mock Implementations ---

class MissionExecutor:
    def __init__(self, drone_interface: UniversalDroneInterface, mission_planner: IntelligentMissionPlanner):
        self.drone_interface = drone_interface
        self.mission_planner = mission_planner
        self.current_mission_plan: Optional[MissionPlan] = None
        self.is_running = False

    async def start_mission(self, mission_id: str, mission_plan: MissionPlan) -> bool:
        if self.is_running:
            print("A mission is already running.")
            return False

        print(f"Starting mission {mission_id}...")
        self.current_mission_plan = mission_plan
        self.is_running = True

        # Simulate mission execution
        for i, waypoint in enumerate(self.current_mission_plan.waypoints):
            if not self.is_running:
                print("Mission aborted.")
                break
            print(f"Executing waypoint {i+1}/{len(self.current_mission_plan.waypoints)}: moving to {waypoint.position}")
            await self.drone_interface.goto_position(*waypoint.position)
            await asyncio.sleep(waypoint.dwell_time)

        print(f"Mission {mission_id} completed.")
        self.is_running = False
        return True

    def stop_mission(self):
        self.is_running = False

class MapManager:
    async def get_available_maps(self) -> List[str]:
        print("Fetching available maps...")
        return ["map1.yaml", "map2.pgm", "office_level_1.json"]

async def process_gui_command(command: Dict) -> Dict:
    print(f"Processing GUI command: {command}")
    action = command.get("action")
    if action == "takeoff":
        await drone_interface.takeoff()
        return {"status": "success", "message": "Takeoff initiated"}
    elif action == "land":
        await drone_interface.land()
        return {"status": "success", "message": "Landing initiated"}
    elif action == "emergency_stop":
        await emergency_stop()
        return {"status": "success", "message": "Emergency stop initiated"}
    return {"status": "unknown_command"}


# --- FastAPI Application Setup ---

app = FastAPI(title="Indoor Drone Navigation Control")

# Note: The paths are relative to the project root where the server is expected to be run
app.mount("/static", StaticFiles(directory="gui_interface/web_gui/static"), name="static")
templates = Jinja2Templates(directory="gui_interface/web_gui/templates")

# --- Global instances of core components ---
config = {}
drone_interface = MAVROSInterface(config)
mission_planner = IntelligentMissionPlanner(config)
safety_monitor = AdvancedSafetyMonitor(config, drone_interface)
mission_executor = MissionExecutor(drone_interface, mission_planner)
map_manager = MapManager()
active_connections: List[WebSocket] = []
mission_plan_storage: Dict[str, MissionPlan] = {}


# --- WebSocket Communication ---

async def broadcast_drone_status():
    """Periodically broadcasts drone status to all connected clients."""
    while True:
        await asyncio.sleep(1)

        message = {
            'type': 'drone_status',
            'data': drone_interface.get_state().__dict__
        }

        for connection in active_connections[:]:
            try:
                await connection.send_text(json.dumps(message, default=str))
            except Exception:
                active_connections.remove(connection)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(broadcast_drone_status())
    await drone_interface.connect()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time communication"""
    await websocket.accept()
    active_connections.append(websocket)

    try:
        while True:
            data = await websocket.receive_text()
            command = json.loads(data)
            response = await process_gui_command(command)
            await websocket.send_text(json.dumps(response))
    except Exception:
        pass
    finally:
        if websocket in active_connections:
            active_connections.remove(websocket)


# --- REST API Endpoints ---

@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):
    """Main dashboard page"""
    return templates.TemplateResponse("dashboard.html", {"request": request})

@app.post("/api/missions")
async def create_mission(mission_data: Dict):
    """Create new mission via REST API"""
    try:
        plan = await mission_planner.plan_mission(mission_data)
        mission_plan_storage[plan.mission_id] = plan
        return {"status": "success", "mission_id": plan.mission_id}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/api/missions/{mission_id}/start")
async def start_mission(mission_id: str):
    """Start execution of planned mission"""
    plan = mission_plan_storage.get(mission_id)
    if not plan:
        raise HTTPException(status_code=404, detail="Mission plan not found.")

    try:
        asyncio.create_task(mission_executor.start_mission(mission_id, plan))
        return {"status": "success", "message": f"Mission {mission_id} started."}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.post("/api/emergency_stop")
async def emergency_stop():
    """Emergency stop all operations"""
    try:
        mission_executor.stop_mission()
        await drone_interface.emergency_stop()
        return {"status": "emergency_stop_executed"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/maps")
async def get_available_maps():
    """Get list of available maps"""
    try:
        maps = await map_manager.get_available_maps()
        return {"maps": maps}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
