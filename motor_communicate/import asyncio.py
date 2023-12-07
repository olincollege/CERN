import asyncio
import websockets
import bluetooth

# Define the ESP32's Bluetooth MAC address (replace with your ESP32's MAC address)
esp32_mac_address = "00:00:00:00:00:00"


async def handle_connection(websocket, path):
    async for message in websocket:
        try:
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            sock.connect((esp32_mac_address, 1))
            sock.send(message)
            sock.close()
            print(f"Command '{message}' sent successfully to ESP32 via Bluetooth")
        except Exception as e:
            print(f"Error: {str(e)}")


if __name__ == "__main__":
    start_server = websockets.serve(handle_connection, "localhost", 8080)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
