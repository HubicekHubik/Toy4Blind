import asyncio
import struct
from bleak import BleakScanner, BleakClient
import serial
import time

SERIAL_PORT = "COM21"  # virtuální COM port pro EI
BAUDRATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUDRATE)

# NUS UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# počítání frekvence
packet_count = 0
last_time = time.time()

async def notification_handler(sender, data):
    global packet_count, last_time

    # měření datové frekvence
    packet_count += 1
    now = time.time()
    if now - last_time >= 1.0:
        print(f"IMU rate: {packet_count} Hz")
        packet_count = 0
        last_time = now

    # rozbal floaty z binárního paketu
    if len(data) == 12:  # 3 floaty x 4B
        ax, ay, az = struct.unpack('<fff', data)
        # poslat do EI data forwarderu
        line = f"{ax}\t{ay}\t{az}\r\n"
        ser.write(line.encode('utf-8'))

async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()

    target = None
    for d in devices:
        if "Zephyr" in (d.name or ""):
            target = d
            break

    if not target:
        print("Device not found.")
        return

    print(f"Connecting to {target.address}")
    async with BleakClient(target.address) as client:
        print("Connected.")

        # enable notifications
        await client.start_notify(NUS_TX_UUID, notification_handler)

        print("Notifications enabled. Waiting for data...\n")

        while True:
            await asyncio.sleep(1)

asyncio.run(main())