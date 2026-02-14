from flask import Flask, request
import os, time

app = Flask(__name__)

SAVE_DIR = "received_audio"
os.makedirs(SAVE_DIR, exist_ok=True)

@app.route("/upload_raw", methods=["POST"])
def upload_raw():
    device_id = request.args.get("device_id", "UNKNOWN")
    filename  = request.args.get("filename", "audio.wav")

    folder = os.path.join(SAVE_DIR, device_id)
    os.makedirs(folder, exist_ok=True)

    ts = time.strftime("%Y%m%d_%H%M%S")
    safe_name = filename.replace("/", "_")

    path = os.path.join(folder, f"{ts}_{safe_name}")

    data = request.get_data()

    with open(path, "wb") as f:
        f.write(data)

    print("Saved:", path, "bytes:", len(data))
    return "OK", 200

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)