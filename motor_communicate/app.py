"""
Handle HTML/Javascript requests from user
"""
from flask import Flask, request
import serial

app = Flask(__name__)
ser = serial.Serial("/dev/rfcomm1", 115200)


@app.route("/send_data", methods=["POST"])
def send_data():
    data = request.json.get("data")

    # Ensure the data is within the range -255 to 255
    data = [max(-255, min(255, d)) for d in data]

    ser.write(",".join(map(str, data)).encode())
    return {"success": True}


if __name__ == "__main__":
    app.run(debug=True)
