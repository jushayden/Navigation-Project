"""
sim_base44_integration.py
Integrates the room-scale A* + Pure Pursuit simulator with the Base44 app API.

Flow:
- GET /api/apps/{APP_ID}/entities/Order   (headers: {api_key, Content-Type})
- Randomize positions for 3 restaurant names (from your app or hardcoded)
- Map each order.user_location (lat/lon) -> grid cell (drop-off)
- Plan & execute: Home -> R_for_order1 -> D_user1 -> R_for_order2 -> D_user2 -> ... -> Home
- On STOP at restaurant:    PUT Order status -> "in_transit"
- On STOP at drop-off:      PUT Order status -> "delivered"
- After delivering an order: POST RobotTrajectory (order_id, trajectory_data, total_distance, travel_time, status)

Requirements:
  pip install requests matplotlib numpy
  (optional) pip install pyserial   # if you want BluetoothLink

Set APP_ID and API_KEY below (from your Base44 app).
"""

import os, math, heapq, time, sys, json, requests
from typing import Tuple, List, Optional, Dict
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============== CONFIG ==============
BASE_URL = "https://app.base44.com/api/apps"
APP_ID   = os.environ.get("BASE44_APP_ID", "<YOUR_APP_ID_HERE>")
API_KEY  = os.environ.get("BASE44_API_KEY", "<YOUR_API_KEY_HERE>")

ORDER_STATUSES_TO_PULL = {"pending", "in_transit"}
RESTAURANT_NAMES = ["Sushi Zen", "Pasta House", "Burger Box"]

ORIGIN_LAT = 36.144200
ORIGIN_LON = -86.802900

# ============== SIM TUNING ==============
SEED            = None
H, W            = 30, 40
RES_M_PER_CELL  = 0.10
MIN_SEP_CELLS   = 4.0
DT              = 0.06
V_LINEAR        = 0.85
V_NEAR          = 0.45
LOOKAHEAD       = 2.0
LOOKAHEAD_NEAR  = 0.80
DENSIFY_STEP    = 0.22
APPROACH_RADIUS = 2.0
STOP_TOL        = 0.28
DWELL_SECONDS   = 2.0
ENTER_RADIUS    = 0.60
LEAVE_RADIUS    = 1.20
ANIMATE         = True

ENABLE_BLUETOOTH = False
BT_PORT          = "COM8"
BT_BAUD          = 115200

# --------- Utils ----------
def rng_seed(seed):
    return np.random.default_rng(seed)

def cell_to_xy(cell):
    r, c = cell
    return float(c) + 0.5, float(r) + 0.5

def angle_wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def progress_bar(prefix, frac, width=24):
    frac = max(0.0, min(1.0, frac))
    filled = int(round(frac*width))
    return f"{prefix} [{chr(9608)*filled}{chr(9617)*(width-filled)}] {int(frac*100):3d}%"

# --------- Base44 API Client ----------
class Base44API:
    def __init__(self, base_url, app_id, api_key):
        self.base = f"{base_url}/{app_id}/entities"
        self.headers = {"api_key": api_key, "Content-Type": "application/json"}

    def _get(self, entity):
        r = requests.get(f"{self.base}/{entity}", headers=self.headers, timeout=10)
        r.raise_for_status()
        return r.json()

    def _put(self, entity, entity_id, payload):
        r = requests.put(f"{self.base}/{entity}/{entity_id}", headers=self.headers, data=json.dumps(payload), timeout=10)
        r.raise_for_status()
        return r.json()

    def _post(self, entity, payload):
        r = requests.post(f"{self.base}/{entity}", headers=self.headers, data=json.dumps(payload), timeout=10)
        r.raise_for_status()
        return r.json()

    def fetch_orders(self):
        try:
            items = self._get("Order")
        except Exception as e:
            print(f"[API] GET Order failed: {e}"); return []
        return [o for o in items if (o.get("status") or "").lower() in ORDER_STATUSES_TO_PULL]

    def set_order_status(self, entity_id, new_status):
        try:
            self._put("Order", entity_id, {"status": new_status})
            print(f"[API] Order {entity_id} -> {new_status}")
        except Exception as e:
            print(f"[API] PUT Order failed: {e}")

    def post_robot_trajectory(self, order_id, traj_points, total_distance_m, travel_time_s, status="completed"):
        payload = {"order_id": order_id, "trajectory_data": traj_points, "total_distance": total_distance_m, "travel_time": travel_time_s, "status": status}
        try:
            self._post("RobotTrajectory", payload)
            print(f"[API] Posted RobotTrajectory for order {order_id}, points={len(traj_points)}")
        except Exception as e:
            print(f"[API] POST RobotTrajectory failed: {e}")

# --------- Geo -> Grid ----------
class GeoProjector:
    def __init__(self, lat0, lon0):
        self.lat0 = lat0; self.lon0 = lon0
        self.k_lat = 111_320.0
        self.k_lon = 111_320.0 * math.cos(math.radians(lat0))

    def gps_to_grid(self, lat, lon):
        dx = (lon - self.lon0) * self.k_lon
        dy = (lat - self.lat0) * self.k_lat
        cx = max(0, min(W-1, int(round(dx / RES_M_PER_CELL))))
        cy = max(0, min(H-1, int(round(dy / RES_M_PER_CELL))))
        return (cy, cx)

# --------- Restaurants ----------
def randomize_restaurants(names, H, W, min_sep, seed=None):
    rng = np.random.default_rng(seed)
    cells = {}; tries = 0
    while len(cells) < len(names) and tries < 10000:
        r = int(rng.integers(0, H)); c = int(rng.integers(0, W))
        ok = all(math.hypot(rr-r, cc-c) >= min_sep for (rr, cc) in cells.values())
        if ok: cells[names[len(cells)]] = (r, c)
        tries += 1
    return cells

# --------- A* ----------
def astar(start, goal):
    g = {start: 0.0}; parent = {start: None}
    pq = [(math.hypot(start[0]-goal[0], start[1]-goal[1]), 0, start)]; counter = 0
    moves = [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(1,1,math.sqrt(2))]
    while pq:
        _, _, cur = heapq.heappop(pq)
        if cur == goal:
            path=[]; n=cur
            while n is not None: path.append(n); n = parent[n]
            return path[::-1]
        cr, cc = cur
        for dr, dc, cost in moves:
            nr, nc = cr+dr, cc+dc
            if not (0 <= nr < H and 0 <= nc < W): continue
            ng = g[cur] + cost
            if (nr,nc) not in g or ng < g[(nr,nc)]:
                g[(nr,nc)] = ng; parent[(nr,nc)] = cur; counter += 1
                heapq.heappush(pq, (ng + math.hypot(nr-goal[0], nc-goal[1]), counter, (nr,nc)))
    return [start, goal]

def densify(poly, step):
    out=[poly[0]]
    for i in range(len(poly)-1):
        p,q = poly[i], poly[i+1]
        L = float(np.hypot(*(q-p))); n = max(1, int(L/step))
        for k in range(1, n+1): out.append(p + (q-p)*(k/n))
    return np.array(out)

def nearest_index(px,py,pts,start_idx,window=25):
    i0=start_idx; i1=min(len(pts), start_idx+window)
    d2 = np.sum((pts[i0:i1]-np.array([px,py]))**2, axis=1)
    return i0 + int(np.argmin(d2))

def lookahead_simple(px,py,pts,L,start_idx):
    for i in range(start_idx, len(pts)):
        if math.hypot(pts[i][0]-px, pts[i][1]-py) >= L: return pts[i], i
    return pts[-1], len(pts)-1

# --------- Bluetooth ----------
class BluetoothLink:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)
    def send_twist(self, v, omega):
        self.ser.write((json.dumps({"v": v, "omega": omega}) + "\n").encode("utf-8"))
    def send_event(self, event, zone):
        self.ser.write((json.dumps({"event": event, "zone": zone, "ts": time.time()}) + "\n").encode("utf-8"))

# ===================== MAIN =====================
def main():
    rng = rng_seed(SEED)
    api = Base44API(BASE_URL, APP_ID, API_KEY)
    projector = GeoProjector(ORIGIN_LAT, ORIGIN_LON)
    bt = None
    if ENABLE_BLUETOOTH:
        try: bt = BluetoothLink(BT_PORT, BT_BAUD)
        except Exception as e: print(f"[BT] Could not open port: {e}")

    rest_cells = randomize_restaurants(RESTAURANT_NAMES, H, W, MIN_SEP_CELLS, seed=SEED)
    orders = api.fetch_orders()
    if not orders:
        print("[Sim] No orders with allowed statuses; generating dummy 2 orders for demo.")
        orders = [
            {"id": "demo1", "restaurant": RESTAURANT_NAMES[0], "user_location": {"latitude": ORIGIN_LAT + 0.0007, "longitude": ORIGIN_LON + 0.0015}, "status": "pending"},
            {"id": "demo2", "restaurant": RESTAURANT_NAMES[1], "user_location": {"latitude": ORIGIN_LAT - 0.0009, "longitude": ORIGIN_LON + 0.0010}, "status": "pending"},
        ]

    home = (int(H*0.2), int(W*0.2))
    stops_cells = [home]; stop_names = ["Home"]
    for o in orders:
        oid = str(o.get("id")); rname = str(o.get("restaurant"))
        rcell = rest_cells.get(rname, home)
        user_loc = o.get("user_location") or {}
        dcell = projector.gps_to_grid(user_loc.get("latitude", ORIGIN_LAT), user_loc.get("longitude", ORIGIN_LON))
        stops_cells += [rcell, dcell]; stop_names += [f"R:{rname}:{oid}", f"D:{oid}"]
    stops_cells += [home]; stop_names += ["Home"]

    legs = [astar(stops_cells[i], stops_cells[i+1]) for i in range(len(stops_cells)-1)]
    all_cells = []
    for i,seg in enumerate(legs): all_cells.extend(seg if i==0 else seg[1:])
    full_way = densify(np.array([cell_to_xy(c) for c in all_cells]), DENSIFY_STEP)

    leg_idx = 0
    leg_way = densify(np.array([cell_to_xy(c) for c in legs[0]]), DENSIFY_STEP)
    leg_progress = 0
    x,y = cell_to_xy(stops_cells[0]); theta=0.0
    traj_x, traj_y = [], []
    dwell_timer = 0.0; t_sim = 0.0; total_dist = 0.0; travel_time = 0.0
    dwell_time_accum = {name: 0.0 for name in stop_names}
    last_inside = [math.hypot(x-cell_to_xy(c)[0], y-cell_to_xy(c)[1]) <= ENTER_RADIUS for c in stops_cells]
    active_order_id = None; this_order_points = []; this_order_dist_m = 0.0; this_order_travel_s = 0.0

    def log(msg, newline=True):
        s=f"[t={t_sim:6.2f}s] {msg}"
        if newline: print(s, flush=True)
        else: sys.stdout.write("\r"+s); sys.stdout.flush()

    log("Starting at Home (inside radius).")

    fig, ax = plt.subplots(figsize=(11, 6)); fig.patch.set_facecolor('black'); ax.set_facecolor('black')
    ax.set_xticks(np.arange(0, W+1, 1)); ax.set_yticks(np.arange(0, H+1, 1))
    ax.grid(which='major', color=(1,1,1,0.20), linestyle='-', linewidth=0.6)
    sx = ax.secondary_xaxis('top', functions=(lambda u: u*RES_M_PER_CELL, lambda u: u/RES_M_PER_CELL))
    sy = ax.secondary_yaxis('right', functions=(lambda u: u*RES_M_PER_CELL, lambda u: u/RES_M_PER_CELL))
    sx.set_xlabel('meters', color='white'); sy.set_ylabel('meters', color='white')
    for a in (sx, sy, ax): a.tick_params(colors='white')

    (bg_line,) = ax.plot(full_way[:,0], full_way[:,1], 'b-', lw=2, alpha=0.35, label='Planned path (A*)')
    (traj_line,) = ax.plot([], [], 'r-', lw=2, label='Robot trajectory')
    (dot,) = ax.plot([x],[y], 'ro', ms=6, zorder=7)
    arrow_len=1.0
    arrow = ax.quiver([x],[y],[arrow_len*math.cos(theta)],[arrow_len*math.sin(theta)], angles='xy', scale_units='xy', scale=1, width=0.008, color='cyan', zorder=8)
    label = ax.text(x, y+0.7, "Jerry", ha='center', va='bottom', fontsize=9, color='white', bbox=dict(boxstyle="round,pad=0.2", fc='black', ec='white', lw=0.6, alpha=0.9), zorder=9)
    status_txt = ax.text(-0.22, 1.0, "", transform=ax.transAxes, ha='left', va='top', fontsize=9, color='white', family='monospace', bbox=dict(boxstyle="round,pad=0.3", fc=(0,0,0,0.6), ec='white', lw=0.6))

    cx,cy = cell_to_xy(stops_cells[0]); ax.scatter([cx],[cy], s=70, color='lime', marker='s', edgecolors='white', linewidths=0.7, zorder=6, label='Home')
    for name, rc in zip(stop_names[1:-1], stops_cells[1:-1]):
        cx, cy = cell_to_xy(rc)
        if name.startswith("R:"):
            ax.scatter([cx],[cy], s=70, color='#FF8C00', marker='^', edgecolors='white', linewidths=0.7, zorder=6)
            ax.annotate(name.split(":")[1], (cx, cy), textcoords="offset points", xytext=(0,7), ha='center', fontsize=8, color='white', weight='bold', zorder=6)
        elif name.startswith("D:"):
            ax.scatter([cx],[cy], s=70, color='#1E90FF', marker='*', edgecolors='white', linewidths=0.7, zorder=6)
            ax.annotate(name, (cx, cy), textcoords="offset points", xytext=(0,7), ha='center', fontsize=8, color='white', weight='bold', zorder=6)

    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_aspect('equal', adjustable='box')
    ax.set_title('Sim x Base44 - Home -> R(order) -> D(user) -> ... -> Home', color='white')
    leg_obj = ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), framealpha=0.2, title='Key')
    plt.setp(leg_obj.get_texts(), color='white'); plt.setp(leg_obj.get_title(), color='white')
    plt.subplots_adjust(right=0.80); fig.tight_layout()

    def rebuild_leg(i_from, i_to):
        nonlocal leg_way, leg_progress
        leg_way = densify(np.array([cell_to_xy(c) for c in astar(stops_cells[i_from], stops_cells[i_to])]), DENSIFY_STEP)
        leg_progress = 0

    def mark_enter_leave():
        for i,c in enumerate(stops_cells):
            cx,cy = cell_to_xy(c); d = math.hypot(x-cx, y-cy); inside = d <= ENTER_RADIUS
            if not last_inside[i] and inside:
                log(f"ENTER {stop_names[i]} radius")
                if bt: bt.send_event("enter", stop_names[i])
            if last_inside[i] and d >= LEAVE_RADIUS:
                log(f"LEAVE {stop_names[i]} radius"); last_inside[i] = False
                if bt: bt.send_event("leave", stop_names[i])
            else: last_inside[i] = inside

    def init():
        traj_line.set_data([], []); dot.set_data([x],[y])
        arrow.set_offsets([x,y]); arrow.set_UVC(arrow_len*math.cos(theta), arrow_len*math.sin(theta))
        label.set_position((x, y+0.7)); status_txt.set_text("")
        return traj_line, dot, label, arrow, status_txt

    def step(_):
        nonlocal x,y,theta,leg_idx,leg_progress,dwell_timer,t_sim,total_dist,travel_time
        nonlocal active_order_id,this_order_points,this_order_travel_s,this_order_dist_m
        t_sim += DT; mark_enter_leave()
        if leg_idx >= len(stops_cells)-1: return traj_line, dot, label, arrow, status_txt
        target_cell = stops_cells[leg_idx+1]; tx,ty = cell_to_xy(target_cell)
        dist = math.hypot(tx-x, ty-y); target_name = stop_names[leg_idx+1]
        if active_order_id is None and target_name.startswith("D:"):
            active_order_id = target_name.split(":")[1]; this_order_points = []; this_order_travel_s = 0.0; this_order_dist_m = 0.0
        if dist <= STOP_TOL:
            if dwell_timer <= 0:
                log(f"STOPPED at {target_name}")
                if target_name.startswith("R:"): api.set_order_status(target_name.split(":")[2], "in_transit")
                elif target_name.startswith("D:"): api.set_order_status(target_name.split(":")[1], "delivered")
                dwell_timer = DWELL_SECONDS
            dwell_timer -= DT; dwell_time_accum[target_name] += DT
            if dwell_timer <= 0:
                log(f"DWELL COMPLETE at {target_name}"); dwell_timer = 0.0
                if target_name.startswith("D:") and active_order_id:
                    api.post_robot_trajectory(active_order_id, this_order_points, this_order_dist_m*RES_M_PER_CELL, this_order_travel_s); active_order_id = None
                leg_idx += 1
                if leg_idx < len(stops_cells)-1: rebuild_leg(leg_idx, leg_idx+1)
            dot.set_data([x],[y]); label.set_position((x, y+0.7))
            arrow.set_offsets([x,y]); arrow.set_UVC(arrow_len*math.cos(theta), arrow_len*math.sin(theta))
            traj_line.set_data(traj_x, traj_y)
            status_txt.set_text(f"STATE: DWELL\nTarget: {target_name}\nRemain: {dist:4.2f}\nTimer: {max(0.0,dwell_timer):4.2f}s")
            return traj_line, dot, label, arrow, status_txt
        lsx,lsy = cell_to_xy(stops_cells[leg_idx]); maxd = max(1e-6, math.hypot(tx-lsx, ty-lsy))
        frac = 1.0 - min(1.0, dist/maxd)
        leg_progress = nearest_index(x, y, leg_way, leg_progress, window=25)
        if dist <= APPROACH_RADIUS or leg_progress >= len(leg_way)-3:
            L=LOOKAHEAD_NEAR; v=V_NEAR; target_pt=np.array([tx,ty]); mode="APPROACH"
        else:
            L=LOOKAHEAD; v=V_LINEAR; target_pt,_=lookahead_simple(x,y,leg_way,L,leg_progress); mode="TRAVEL"
        alpha = angle_wrap(math.atan2(target_pt[1]-y, target_pt[0]-x) - theta)
        omega = (2.0 * math.sin(alpha) / max(L,1e-6)) * v
        x_prev,y_prev = x,y
        x += v*math.cos(theta)*DT; y += v*math.sin(theta)*DT; theta = angle_wrap(theta + omega*DT)
        sd = math.hypot(x-x_prev, y-y_prev); total_dist += sd; travel_time += DT
        if bt: bt.send_twist(v, omega)
        if active_order_id:
            this_order_travel_s += DT; this_order_dist_m += sd
            this_order_points.append({"x": x, "y": y, "theta": theta, "timestamp": t_sim})
        traj_x.append(x); traj_y.append(y); traj_line.set_data(traj_x, traj_y)
        dot.set_data([x],[y]); label.set_position((x, y+0.7))
        arrow.set_offsets([x,y]); arrow.set_UVC(arrow_len*math.cos(theta), arrow_len*math.sin(theta))
        status_txt.set_text(f"STATE: {mode}\nTarget: {target_name}\nSpeed: {v:4.2f}\nRemain: {dist:4.2f}\n{progress_bar('', frac, 16)}")
        return traj_line, dot, label, arrow, status_txt

    rebuild_leg(0,1)
    if ANIMATE:
        anim = FuncAnimation(fig, step, init_func=init, frames=8000, interval=int(DT*1000), blit=False, repeat=False)
        fig._anim = anim; plt.show()
    else:
        init()
        for _ in range(8000): step(_)
        plt.show()

if __name__ == "__main__":
    main()
