# Navigation Project

A room-scale delivery-robot navigation system: an A\* + Pure Pursuit motion simulator wired to a live order feed and a WebSocket control server.

## What's here

- **`sim_base44_integration.py`** — the simulator. It plans routes with A\* on a grid (configurable resolution, default 0.10 m per cell) and follows them with a Pure Pursuit controller. It pulls delivery orders from a [Base44](https://base44.com) app, maps each customer location onto a grid cell, and runs a full Home → restaurant → drop-off → Home loop — updating each order's status (`in_transit`, `delivered`) and posting the driven trajectory back to the app.
- **`websocket_server.py`** — a WebSocket bridge that reports robot status (battery, location, speed) and accepts navigation commands (`NAVIGATE`, `QUICK_ROUTE`, `GET_STATUS`) from a client app.

## Run it

```bash
pip install requests matplotlib numpy websockets
# optional: pip install pyserial   # for a Bluetooth serial link

export BASE44_APP_ID=your_app_id
export BASE44_API_KEY=your_api_key

python sim_base44_integration.py   # the route simulator
python websocket_server.py         # the control server
```

## Stack

Python · A\* pathfinding · Pure Pursuit control · Base44 API · WebSockets · Matplotlib (trajectory animation)
