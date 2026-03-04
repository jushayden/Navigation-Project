import asyncio
import websockets
import json
import subprocess

connected_clients = set()

async def handle_client(websocket, path):
    connected_clients.add(websocket)
    print(f"Client connected. Total clients: {len(connected_clients)}")
    try:
        await websocket.send(json.dumps({
            "battery": 92, "status": "idle", "piId": "UCF-JERRY-001",
            "location": "Student Union", "speed": 0
        }))
        async for message in websocket:
            print(f"Received: {message}")
            data = json.loads(message)
            if data.get('type') == 'NAVIGATE':
                start_lat = data.get('start_lat')
                start_lon = data.get('start_lon')
                dest_lat = data.get('dest_lat')
                dest_lon = data.get('dest_lon')
                print(f"Navigate from ({start_lat}, {start_lon}) to ({dest_lat}, {dest_lon})")
                await websocket.send(json.dumps({
                    "status": "navigating", "message": "Route received, starting navigation"
                }))
            elif data.get('type') == 'GET_STATUS':
                await websocket.send(json.dumps({
                    "battery": 92, "status": "idle", "piId": "UCF-JERRY-001",
                    "location": "Student Union", "speed": 0
                }))
            elif data.get('type') == 'QUICK_ROUTE':
                from_loc = data.get('from')
                to_loc = data.get('to')
                print(f"Quick route: {from_loc} -> {to_loc}")
                await websocket.send(json.dumps({
                    "status": "received",
                    "message": f"Route from {from_loc} to {to_loc} received"
                }))
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        connected_clients.remove(websocket)
        print(f"Client removed. Total clients: {len(connected_clients)}")

async def main():
    print("Starting Jerry WebSocket Server on port 8765...")
    server = await websockets.serve(handle_client, "0.0.0.0", 8765)
    print("Server ready! Waiting for connections...")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
