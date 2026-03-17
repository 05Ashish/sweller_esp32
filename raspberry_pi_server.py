import os
import time
import random
import threading
import requests
from flask import Flask, request, jsonify, send_from_directory

app = Flask(__name__)

# ============================================================================
# CONFIGURATION - CHANGE THESE!
# ============================================================================
AUDIO_DIR = '/home/pi/audio_archive'
FIRMWARE_DIR = '/home/pi/firmware'

# MUST be the "RAW" content URLs from GitHub
GITHUB_VERSION_URL = 'https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO/main/version.txt'
GITHUB_BIN_URL = 'https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO/main/firmware.bin'

# Ensure directories exist
os.makedirs(AUDIO_DIR, exist_ok=True)
os.makedirs(FIRMWARE_DIR, exist_ok=True)

# ============================================================================
# GITHUB AUTO-SYNC WORKER
# ============================================================================
def github_sync_worker():
    """Background thread that pulls updates from GitHub to the Pi."""
    local_version_file = os.path.join(FIRMWARE_DIR, 'version.txt')
    local_bin_file = os.path.join(FIRMWARE_DIR, 'firmware.bin')

    print("[SYNC] GitHub Sync Worker started.")

    while True:
        try:
            # 1. Determine Local Version
            local_version = 0
            if os.path.exists(local_version_file):
                with open(local_version_file, 'r') as f:
                    content = f.read().strip()
                    if content.isdigit():
                        local_version = int(content)

            # 2. Fetch Remote Version with Cache-Busting
            # Adding a random query string ensures we don't get a cached file
            cache_buster = f"?cb={random.randint(100000, 999999)}"
            response = requests.get(GITHUB_VERSION_URL + cache_buster, timeout=15)
            
            if response.status_code == 200:
                remote_version_str = response.text.strip()
                if remote_version_str.isdigit():
                    remote_version = int(remote_version_str)
                    
                    # 3. Trigger Download if GitHub is newer
                    if remote_version > local_version:
                        print(f"[SYNC] Update detected! Pi ({local_version}) -> GitHub ({remote_version})")
                        
                        bin_res = requests.get(GITHUB_BIN_URL + cache_buster, stream=True, timeout=60)
                        if bin_res.status_code == 200:
                            with open(local_bin_file, 'wb') as f:
                                for chunk in bin_res.iter_content(chunk_size=8192):
                                    f.write(chunk)
                            
                            # Finalize version update
                            with open(local_version_file, 'w') as f:
                                f.write(str(remote_version))
                                
                            print(f"[SYNC] SUCCESS: Updated to Version {remote_version}")
                    else:
                        # Log this only for deep debugging
                        # print(f"[SYNC] Up to date (v{local_version})")
                        pass
            else:
                print(f"[SYNC] GitHub Error {response.status_code}")

        except Exception as e:
            print(f"[SYNC] Worker Exception: {e}")

        # Wait 5 minutes between checks
        time.sleep(300)

# Start the background sync thread
threading.Thread(target=github_sync_worker, daemon=True).start()

# ============================================================================
# FLASK ROUTES
# ============================================================================

@app.route('/status', methods=['GET'])
def get_status():
    """Check the Pi's status from a browser."""
    local_version = "Unknown"
    v_file = os.path.join(FIRMWARE_DIR, 'version.txt')
    if os.path.exists(v_file):
        with open(v_file, 'r') as f:
            local_version = f.read().strip()
            
    return jsonify({
        "status": "online",
        "pi_firmware_version": local_version,
        "audio_files_archived": len(os.listdir(AUDIO_DIR))
    }), 200

@app.route('/firmware/<filename>', methods=['GET'])
def download_firmware(filename):
    """Serve firmware files to ESP32."""
    return send_from_directory(FIRMWARE_DIR, filename)

@app.route('/upload', methods=['POST'])
def upload_file():
    """Receive audio from ESP32."""
    filename = request.headers.get('X-Filename', f"rec_{int(time.time())}.wav")
    file_path = os.path.join(AUDIO_DIR, filename)
    
    try:
        with open(file_path, 'wb') as f:
            f.write(request.data)
        print(f"[UPLOAD] Saved: {filename}")
        return jsonify({"status": "success"}), 200
    except Exception as e:
        print(f"[UPLOAD] Error: {e}")
        return jsonify({"status": "error"}), 500

if __name__ == '__main__':
    # Use threaded=True to handle multiple ESP32s uploading at once
    app.run(host='0.0.0.0', port=5000, threaded=True)