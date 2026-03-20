from flask import Flask, request, jsonify, send_from_directory
import os
import requests
import threading

app = Flask(__name__)

# ============================================================================
# CONFIGURATION
# ============================================================================
AUDIO_DIR = '/home/pi/audio_archive' 
FIRMWARE_DIR = '/home/pi/firmware'

# Your DigitalOcean Backend Endpoints
PRESIGN_URL = "http://159.89.167.230:3002/api/uploads/presign"
SUCCESS_URL = "http://159.89.167.230:3002/api/uploads/success"

os.makedirs(AUDIO_DIR, exist_ok=True)
os.makedirs(FIRMWARE_DIR, exist_ok=True)

# ============================================================================
# BACKGROUND WORKER: R2 UPLOAD & DB SYNC
# ============================================================================
def process_cloud_upload(filepath, filename):
    print(f"\n[CLOUD] 1. Requesting secure upload ticket for {filename}...")
    
    try:
        # Step 1: Ask backend for the Presigned URL using the exact casing required
        ticket_response = requests.post(PRESIGN_URL, json={"fileName": filename})
        
        if ticket_response.status_code not in [200, 201]:
            print(f"[CLOUD ERROR] Backend refused presign. Code: {ticket_response.status_code}")
            return

        # Extract the exact keys from your backend's JSON response
        response_data = ticket_response.json()
        upload_url = response_data.get("uploadUrl")
        storage_key = response_data.get("storageKey")

        if not upload_url:
            print("[CLOUD ERROR] Could not find 'uploadUrl' in the backend response.")
            return

        # Step 2: Stream the file directly to Cloudflare R2 using the URL
        print(f"[CLOUD] 2. Ticket received! Uploading {storage_key} to R2 bucket...")
        with open(filepath, 'rb') as f:
            # R2 requires a PUT request for presigned URLs
            upload_response = requests.put(upload_url, data=f)

        # Step 3: Tell the backend it was a success so it updates the Database
        if upload_response.status_code == 200:
            print(f"[CLOUD SUCCESS] {filename} is in the bucket!")
            print(f"[CLOUD] 3. Notifying backend of success...")
            
            # Sending back the storageKey so the backend knows exactly which file finished
            payload = {
                "fileName": filename, 
                "storageKey": storage_key, 
                "status": "uploaded"
            }
            success_response = requests.post(SUCCESS_URL, json=payload)
            
            if success_response.status_code in [200, 201]:
                print("[CLOUD SUCCESS] Backend database updated!")
                
                # OPTIONAL: Uncomment the line below to automatically delete the file 
                # from the Pi's SSD once it is safely in the cloud.
                # os.remove(filepath) 
            else:
                print(f"[CLOUD WARNING] Uploaded to R2, but /success endpoint failed: {success_response.text}")
                
        else:
            print(f"[CLOUD ERROR] R2 Upload failed. Code: {upload_response.status_code}")
            print(f"R2 Response: {upload_response.text}")

    except Exception as e:
        print(f"[CLOUD ERROR] Background process crashed: {e}")

# ============================================================================
# ENDPOINT: RECEIVE AUDIO FROM ESP32
# ============================================================================
@app.route('/upload', methods=['POST'])
def upload_audio():
    # Grab the filename sent by the ESP32 header
    filename = request.headers.get('X-Filename')
    if not filename: 
        return jsonify({"error": "Missing X-Filename"}), 400

    safe_filename = os.path.basename(filename)
    filepath = os.path.join(AUDIO_DIR, safe_filename)

    try:
        # Save locally so we never lose the file
        with open(filepath, 'wb') as f:
            f.write(request.get_data())
        
        # Fire the cloud upload in the background so the ESP32 doesn't timeout
        threading.Thread(target=process_cloud_upload, args=(filepath, safe_filename)).start()
        
        # Instantly tell the ESP32 it's safe to delete the SD card copy
        return jsonify({"message": "Saved locally, triggering cloud sync"}), 200

    except Exception as e:
        print(f"[LOCAL ERROR] Failed to save {safe_filename} to SSD: {e}")
        return jsonify({"error": "Internal Server Error"}), 500

# ============================================================================
# ENDPOINT: SERVE FIRMWARE (Not currently used since ESP32 uses GitHub)
# ============================================================================
@app.route('/firmware/<path:filename>', methods=['GET'])
def serve_firmware(filename):
    return send_from_directory(FIRMWARE_DIR, filename)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)