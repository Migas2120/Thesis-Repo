import asyncio, json, time, math, heapq
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional

# add after stdlib imports at the top:
try:
    import numpy as np
    from shapely.geometry import Point as ShapelyPoint, Polygon as ShapelyPolygon
    import trimesh
    _GEOM_OK = True
except Exception:
    _GEOM_OK = False

# ----------------- math helpers -----------------
def v_add(a,b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def v_sub(a,b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def v_mul(a,s): return (a[0]*s, a[1]*s, a[2]*s)
def v_len(a):   return math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
def v_norm(a):  return (0,0,0) if (l:=v_len(a))==0 else (a[0]/l, a[1]/l, a[2]/l)

# ----- Unity payload parsing (axis note!) -----
# SerializableWaypoint comes as: {"coords":[x, z, y], "hold": float}
def parse_serializable_waypoints(swps):
    wps = []
    for wp in swps:
        coords = wp.get("coords")
        if not coords or len(coords) < 3:
            continue
        x = float(coords[0])
        z = float(coords[2])  # Unity sent "z" in slot 1
        y = float(coords[1])  # Unity sent "y" in slot 2
        wps.append((x, y, z))  # server uses (x, y, z)
    return wps

# OutgoingMapAreaMessage points[] is Unity Vector3 serialized as {x, y, z}
# BUT you constructed Vector3(wp.x, wp.z, wp.y) on the Unity side,
# so to recover world (x,y,z) we must swap back: (x, z, y)
def parse_unity_vector3_points(points):
    pts = []
    for p in points:
        x = float(p.get("x", 0.0))
        y_ = float(p.get("y", 0.0))  # actually original Z
        z_ = float(p.get("z", 0.0))  # actually original Y
        pts.append((x, y_, z_))  # swap back to (x, y, z)
    return pts

def generate_map_area_waypoints(points_xyz, grid_spacing: float, vertical_step: float):
    """
    points_xyz: list of (x,y,z) corners (any 3D shape).
    Returns a flat list of (x,y,z) waypoints following boustrophedon rows per Z layer.
    Uses trimesh/shapely if available; falls back to bbox grid if not.
    """
    if not points_xyz:
        return []

    if _GEOM_OK:
        pts = np.array(points_xyz, dtype=float)
        try:
            hull = trimesh.convex.convex_hull(pts)  # robust hull
        except Exception:
            hull = trimesh.Trimesh(pts).convex_hull

        z_min = float(np.min(pts[:,2]))
        z_max = float(np.max(pts[:,2]))
        z_layers = np.arange(z_min, z_max + vertical_step, vertical_step)

        waypoints = []
        for z in z_layers:
            slice = hull.section(plane_origin=[0,0,z], plane_normal=[0,0,1])
            if slice is None:
                continue
            path2D, T = slice.to_2D()
            polys = path2D.polygons_full
            if not polys:
                continue

            # take outer loop only
            poly = polys[0]
            shapely_poly = ShapelyPolygon(list(poly.exterior.coords))

            minx, miny, maxx, maxy = shapely_poly.bounds
            y_values = np.arange(miny, maxy + grid_spacing, grid_spacing)
            for i, y in enumerate(y_values):
                x_values = np.arange(minx, maxx + grid_spacing, grid_spacing)
                row = []
                for x in x_values:
                    if shapely_poly.contains(ShapelyPoint(x, y)):
                        point_local = np.array([x, y, 0, 1])
                        point_world = T @ point_local
                        row.append((float(point_world[0]), float(point_world[1]), float(point_world[2])))
                if i % 2 == 1:
                    row.reverse()
                waypoints.extend(row)
        return waypoints

    # --------- Fallback: bbox grid over layers (no deps) ----------
    xs = [p[0] for p in points_xyz]
    ys = [p[1] for p in points_xyz]
    zs = [p[2] for p in points_xyz]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    minz, maxz = min(zs), max(zs)

    waypoints = []
    # simple layers
    z = minz
    while z <= maxz + 1e-6:
        # boustrophedon rows in XY bbox
        y = miny
        row_idx = 0
        while y <= maxy + 1e-6:
            x_vals = []
            x = minx
            while x <= maxx + 1e-6:
                x_vals.append((x, y, z))
                x += grid_spacing
            if row_idx % 2 == 1:
                x_vals.reverse()
            waypoints.extend(x_vals)
            y += grid_spacing
            row_idx += 1
        z += vertical_step

    return waypoints

# ----------------- data models -----------------
@dataclass(order=True)
class PQItem:
    priority: int
    order: int
    mission_id: str = field(compare=False)
    waypoints: List[Tuple[float,float,float]] = field(compare=False)
    mode: str = field(compare=False, default="once")
    patrol_loops: int = field(compare=False, default=1)
    preempt: bool = field(compare=False, default=False)

@dataclass
class MissionRuntime:
    mission_id: str
    waypoints: List[Tuple[float,float,float]]
    mode: str = "once"
    patrol_loops: int = 1
    current_idx: int = 0
    loops_done: int = 0

@dataclass
class DroneSim:
    drone_id: int
    radius: float = 0.35
    speed: float = 20             # m/s along path
    pos: Tuple[float,float,float] = (0.0,0.0,0.0)
    vel: Tuple[float,float,float] = (0.0,0.0,0.0)
    active: Optional[MissionRuntime] = None
    paused_stack: List[MissionRuntime] = field(default_factory=list)
    queue: List[PQItem] = field(default_factory=list)
    _pq_order: int = 0
    history: List[Tuple[float,float,float,float]] = field(default_factory=list)  # (t,x,y,z)

    def tick(self, dt: float, now: float):
        # move toward current waypoint if any
        if self.active and self.active.waypoints:
            target = self.active.waypoints[self.active.current_idx]
            to_target = v_sub(target, self.pos)
            dist = v_len(to_target)

            if dist < 1e-3:
                # Arrived at waypoint → advance
                self.active.current_idx += 1
                if self.active.current_idx >= len(self.active.waypoints):
                    # finished a pass
                    self.active.loops_done += 1
                    if self.active.mode == "loop" and self.active.loops_done < max(1, self.active.patrol_loops):
                        self.active.current_idx = 0
                    else:
                        # mission complete
                        self.active = None
                self.vel = (0.0,0.0,0.0)
            else:
                dirn = v_norm(to_target)
                step = min(self.speed * dt, dist)
                self.pos = v_add(self.pos, v_mul(dirn, step))
                self.vel = v_mul(dirn, self.speed)
        else:
            self.vel = (0.0,0.0,0.0)
            # pull from queue if idle
            if self.queue:
                item = heapq.heappop(self.queue)
                self.active = MissionRuntime(
                    mission_id=item.mission_id,
                    waypoints=item.waypoints,
                    mode=item.mode,
                    patrol_loops=item.patrol_loops,
                    current_idx=0,
                    loops_done=0
                )
        # record history
        self.history.append((now, self.pos[0], self.pos[1], self.pos[2]))

    def add_mission(self, mission_id: str, waypoints, mode: str, patrol_loops: int, priority: int, preempt: bool):
        # normalize waypoints: accept [[x,y,z], {...}, (x,y,z)]
        norm_wps = []
        for wp in waypoints:
            if isinstance(wp, dict):
                norm_wps.append((float(wp["x"]), float(wp["y"]), float(wp["z"])))
            else:
                # list/tuple [x,y,z]
                norm_wps.append((float(wp[0]), float(wp[1]), float(wp[2])))

        if preempt and self.active:
            # push current to paused stack and start new right away
            self.paused_stack.append(self.active)
            self.active = MissionRuntime(
                mission_id=mission_id,
                waypoints=norm_wps,
                mode=mode,
                patrol_loops=patrol_loops,
                current_idx=0,
                loops_done=0
            )
        else:
            self._pq_order += 1
            heapq.heappush(self.queue, PQItem(
                priority=int(priority),
                order=self._pq_order,
                mission_id=mission_id,
                waypoints=norm_wps,
                mode=mode,
                patrol_loops=int(patrol_loops),
                preempt=bool(preempt)
            ))

    def set_params(self, speed: Optional[float]=None, radius: Optional[float]=None):
        if speed is not None:  self.speed = float(speed)
        if radius is not None: self.radius = float(radius)

    def predict(self, t_future: float) -> Tuple[float,float,float]:
        # simple constant-velocity prediction
        return v_add(self.pos, v_mul(self.vel, t_future))


class CentralSim:
    def __init__(self, host="0.0.0.0", port=65432,
                 tick_hz=30.0, broadcast_hz=10.0,
                 horizon=3.0, buffer=0.1):
        self.host, self.port = host, port
        self.dt = 1.0/tick_hz
        self.broadcast_dt = 1.0/broadcast_hz
        self.horizon = horizon
        self.buffer = buffer
        self.clients: Dict[int, asyncio.StreamWriter] = {}  # drone_id -> writer
        self.client_ids: Dict[asyncio.StreamWriter, int] = {}
        self.next_drone_id = 1
        self.drones: Dict[int, DroneSim] = {}
        self._lock = asyncio.Lock()
        self._last_broadcast = 0.0

        self._last_alert_log = 0.0
        self._alert_log_cooldown = 0.5  # seconds

    async def start(self):
        server = await asyncio.start_server(self._handle_client, self.host, self.port)
        print(f"[CDS] Listening on {self.host}:{self.port}")
        async with server:
            asyncio.create_task(self._sim_loop())
            await server.serve_forever()

    # ------------- simulation & collisions -------------
    async def _sim_loop(self):
        while True:
            try:
                await asyncio.sleep(self.dt)
                now = time.time()
                async with self._lock:
                    for d in self.drones.values():
                        d.tick(self.dt, now)

                    # broadcast world_state periodically
                    if now - self._last_broadcast >= self.broadcast_dt:
                        await self._broadcast_world()
                        self._last_broadcast = now

                    # collision check each tick; send alert only when any pair violates threshold within horizon
                    alerts = self._compute_collisions(self.horizon)
                    if alerts:
                        now2 = time.time()
                        if now2 - self._last_alert_log >= self._alert_log_cooldown:
                            pairs_str = ", ".join([f"{p['a']}-{p['b']}@{p['t_min']}s(d={p['d_min']})" for p in alerts[:3]])
                            more = "" if len(alerts) <= 3 else f" +{len(alerts)-3} more"
                            print(f"[CDS][collision] {pairs_str}{more}")
                            self._last_alert_log = now2
                        await self._broadcast({"type":"collision_alert","horizon":self.horizon,"pairs":alerts})
            except Exception as e:
                print("[CDS] sim loop error:", e)

    def _compute_collisions(self, horizon: float):
        ids = sorted(self.drones.keys())
        pairs = []
        samples = 30
        for i in range(len(ids)):
            for j in range(i+1, len(ids)):
                a = self.drones[ids[i]]
                b = self.drones[ids[j]]
                thresh = a.radius + b.radius + self.buffer
                best = None
                for k in range(samples+1):
                    t = horizon * k / samples
                    pa = a.predict(t)
                    pb = b.predict(t)
                    d = v_len(v_sub(pa, pb))
                    if best is None or d < best[0]:
                        best = (d, t, pa, pb)
                d_min, t_min, pa, pb = best
                if d_min <= thresh:
                    pairs.append({
                        "a": a.drone_id, "b": b.drone_id,
                        "t_min": round(t_min,3),
                        "d_min": round(d_min,3),
                        "threshold": round(thresh,3),
                        "pa": {"x":pa[0],"y":pa[1],"z":pa[2]},
                        "pb": {"x":pb[0],"y":pb[1],"z":pb[2]},
                    })
        return pairs

    async def _broadcast_world(self):
        state = {
            "type":"world_state",
            "drones":[
                {
                    "drone_id": d.drone_id,
                    "pos": {"x":d.pos[0],"y":d.pos[1],"z":d.pos[2]},
                    "vel": {"x":d.vel[0],"y":d.vel[1],"z":d.vel[2]},
                    "radius": d.radius,
                    "active_mission": (d.active.mission_id if d.active else None)
                } for d in self.drones.values()
            ]
        }
        await self._broadcast(state)

    # ------------- networking -------------
    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        async with self._lock:
            drone_id = self.next_drone_id
            self.next_drone_id += 1
            self.clients[drone_id] = writer
            self.client_ids[writer] = drone_id
            self.drones[drone_id] = DroneSim(drone_id=drone_id)

        addr = writer.get_extra_info('peername')
        print(f"[CDS] Client {addr} connected → drone_id={drone_id}")
        await self._send(writer, {"type": "hello", "drone_id": drone_id})

        buffer = ""
        try:
            while True:
                data = await reader.read(4096)   # read whatever is available
                if not data:
                    break
                chunk = data.decode(errors="replace")
                buffer += chunk

                # split on newlines if present
                parts = buffer.split("\n")
                buffer = parts.pop()  # keep last partial (could be incomplete JSON)

                for raw in parts:
                    text = raw.strip()
                    if not text:
                        # show empty lines too (so you know if client sent \n\n)
                        print(f"[CDS][recv-empty] drone {drone_id}")
                        continue

                    print(f"[CDS][recv-raw] drone {drone_id}: {text!r}")
                    try:
                        msg = json.loads(text)
                        await self._handle_msg(drone_id, msg, writer)
                    except json.JSONDecodeError as e:
                        print(f"[CDS][recv-bad-json] {e} text={text!r}")
                        # optional: echo back to client
                        await self._send(writer, {"type": "error", "error": f"bad_json:{e}", "raw": text})

        except asyncio.CancelledError:
            pass
        finally:
            print(f"[CDS] Client drone_id={drone_id} disconnected")
            async with self._lock:
                self.clients.pop(drone_id, None)
                self.client_ids.pop(writer, None)
            writer.close()
            await writer.wait_closed()

    async def _handle_msg(self, drone_id: int, msg: dict, writer: asyncio.StreamWriter):
        mtype = msg.get("type","")
        async with self._lock:
            d = self.drones[drone_id]

            if mtype == "set_params":
                d.set_params(speed=msg.get("speed"), radius=msg.get("radius"))
                await self._send(writer, {"type":"ack","req":"set_params","drone_id":drone_id}) 

            elif mtype == "set_pose":  # optional: if you want to hard-set pos (e.g., spawn/start)
                x,y,z = float(msg["x"]), float(msg["y"]), float(msg["z"])
                d.pos = (x,y,z)
                d.vel = (0.0,0.0,0.0)
                await self._send(writer, {"type":"ack","req":"set_pose","drone_id":drone_id})

            elif mtype == "add_mission":
                # Unity sends SerializableWaypoint objects: {"coords":[x,z,y], "hold":float}
                swps = msg.get("waypoints", [])
                norm_wps = parse_serializable_waypoints(swps)  # <-- helper you added
                if not norm_wps:
                    await self._send(writer, {"type":"error","error":"add_mission: empty_or_bad_waypoints"})
                    return

                d.add_mission(
                    mission_id = msg["mission_id"],
                    waypoints  = norm_wps,
                    mode       = msg.get("mode","once"),
                    patrol_loops = int(msg.get("patrol_loops",1)),
                    priority   = int(msg.get("priority",5)),
                    preempt    = bool(msg.get("preempt", False))
                )
                await self._send(writer, {"type":"ack","req":"add_mission","drone_id":drone_id,"mission_id":msg["mission_id"]})

            elif mtype == "map_area":
                # Unity sends Vector3 points as {x,y,z} but you built them from (x, z, y)
                raw_pts = msg.get("points", [])
                points_xyz = parse_unity_vector3_points(raw_pts)  # <-- helper you added
                grid_spacing  = float(msg.get("grid_spacing", 2.0))
                vertical_step = float(msg.get("vertical_step", 1.0))
                priority = int(msg.get("priority", 5))
                preempt  = bool(msg.get("preempt", False))

                wps = generate_map_area_waypoints(points_xyz, grid_spacing, vertical_step)  # <-- slicer
                if not wps:
                    await self._send(writer, {"type":"error","error":"map_area: no_waypoints_generated"})
                    return

                d.add_mission(
                    mission_id   = msg.get("mission_id", "map_area_auto"),
                    waypoints    = wps,
                    mode         = "once",
                    patrol_loops = 1,
                    priority     = priority,
                    preempt      = preempt
                )
                await self._send(writer, {
                    "type":"ack","req":"map_area","drone_id":drone_id,
                    "mission_id": msg.get("mission_id", "map_area_auto"),
                    "generated_waypoints": len(wps),
                    "geom_backend": ("trimesh" if _GEOM_OK else "bbox-fallback")
                })

            elif mtype == "reset":
                self.drones[drone_id] = DroneSim(drone_id=drone_id)
                await self._send(writer, {"type":"ack","req":"reset","drone_id":drone_id})

            elif mtype == "dump_history_csv":
                # Optional: target another drone
                target_id = int(msg.get("drone_id", drone_id))
                if target_id not in self.drones:
                    await self._send(writer, {"type":"error","error":f"dump_history_csv: unknown drone_id={target_id}"})
                    return
                path = msg.get("path", f"history_drone_{target_id}.csv")
                hist = self.drones[target_id].history
                try:
                    with open(path, "w", encoding="utf-8") as f:
                        f.write("t,x,y,z\n")
                        for t,x,y,z in hist:
                            f.write(f"{t},{x},{y},{z}\n")
                    await self._send(writer, {
                        "type":"ack","req":"dump_history_csv",
                        "drone_id": target_id, "path": path, "rows": len(hist)
                    })
                except Exception as e:
                    await self._send(writer, {"type":"error","error":f"dump_history_csv_failed:{e}"})

            elif mtype == "dump_history_all_csv":
                """
                Write ALL drones' histories as separate CSVs on the SERVER machine.
                Optional prefix: {"type":"dump_history_all_csv","prefix":"run_2025-09-11_"}
                """
                prefix = msg.get("prefix", "history_")
                results = []
                try:
                    for did, ds in self.drones.items():
                        path = f"{prefix}drone_{did}.csv"
                        with open(path, "w", encoding="utf-8") as f:
                            f.write("t,x,y,z\n")
                            for t,x,y,z in ds.history:
                                f.write(f"{t},{x},{y},{z}\n")
                        results.append({"drone_id": did, "path": path, "rows": len(ds.history)})
                    await self._send(writer, {
                        "type":"ack","req":"dump_history_all_csv","results":results
                    })
                except Exception as e:
                    await self._send(writer, {"type":"error","error":f"dump_history_all_csv_failed:{e}"})

            elif mtype == "get_history":
                # Optional: target another drone's history
                target_id = int(msg.get("drone_id", drone_id))
                if target_id not in self.drones:
                    await self._send(writer, {"type":"error","error":f"get_history: unknown drone_id={target_id}"})
                    return
                N = int(msg.get("n", 1000))
                hist = self.drones[target_id].history[-N:]
                await self._send(writer, {
                    "type":"history",
                    "drone_id": target_id,
                    "samples": [{"t":t,"x":x,"y":y,"z":z} for (t,x,y,z) in hist]
                })


            elif mtype == "world_state":
                await self._broadcast_world()

            else:
                await self._send(writer, {"type":"error","error":f"unknown_type:{mtype}"})

    async def _send(self, writer: asyncio.StreamWriter, obj: dict):
        writer.write((json.dumps(obj) + "\n").encode())
        await writer.drain()

    async def _broadcast(self, obj: dict):
        dead = []
        for did, w in self.clients.items():
            try:
                await self._send(w, obj)
            except Exception:
                dead.append(did)
        for did in dead:
            self.clients.pop(did, None)

if __name__ == "__main__":
    sim = CentralSim(port=65432, tick_hz=30.0, broadcast_hz=10.0, horizon=3.0, buffer=0.1)
    try:
        asyncio.run(sim.start())
    except KeyboardInterrupt:
        print("\n[CDS] Shutting down.")
