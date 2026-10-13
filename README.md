# Vision-Guided Quadcopter for Warehouse Inventory Scanning

A low-cost quadcopter system for scanning warehouse inventory at several shelf heights without GPS.
An ESP32-CAM on the drone reads QR labels, a laptop receives and stores the scans, and the
camera-based flight controller is validated in simulation.

This repository supports the paper *"Vision-Guided Quadcopter for GPS-Free Warehouse Inventory
Scanning: Design, prototype status and simulation-based validation"* (IJRTI format).

## Project status

| Part | Status |
| --- | --- |
| F450 quadcopter (flight controller, ESP32-CAM, 3S LiPo) | Built; flown manually with an RC transmitter |
| ESP32-CAM QR scanning, Wi-Fi to laptop | Implemented (`esp code/`) |
| Flask server saving scans to CSV | Implemented (`app.py`) |
| Camera-based position control (ArUco + PID) | Simulation only |
| Autonomous flight | Not attempted |

No hardware flight measurements are reported. The numbers below come from simulation.

## Repository layout

| Path | Purpose |
| --- | --- |
| `esp code/` | Arduino sketch for the AI-Thinker ESP32-CAM QR scanner |
| `app.py` | Flask server that receives decoded QR text and appends it to `results.csv` |
| `sim_pose.py` | Test of ArUco marker pose accuracy on rendered images |
| `sim_qr.py` | Test of QR decoding on rendered ESP32-CAM-like frames |
| `sim_flight.py` | Closed-loop quadcopter simulation (plant, Kalman filter, PID, mission) |
| `run_experiments.py` | Monte Carlo runs of the flight simulation |
| `make_figs.py` | Regenerates the plots used in the paper |
| `pose_results.json`, `qr_results.json`, `flight_results.json` | Saved results of the simulations |

## 1. QR scanner (ESP32-CAM + Flask)

The ESP32-CAM decodes the QR label on the board and sends the decoded text to the laptop as JSON.
The laptop saves each scan with a timestamp.

**ESP32-CAM**

1. Install the Arduino libraries `ESP32QRCodeReader` and `ArduinoJson`.
2. In the sketch, set `WIFI_NAME`, `WIFI_PASSWORD` and `LAPTOP_IP` (the laptop's IP on the same network).
3. Select the AI-Thinker ESP32-CAM board and upload.

**Laptop**

```bash
pip install flask
python app.py          # listens on port 5000
```

**Endpoints**

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Health check |
| POST | `/qrdata` | Receives `{"data": "<QR text>"}` and appends a row to `results.csv` |

**QR label format** (one field per line):

```
Item ID: WH001
Product: DC Motor Assembly
Quantity: 120
Location: Rack A-01
```

**Test without hardware**

```bash
curl -X POST http://localhost:5000/qrdata -H "Content-Type: application/json" \
     -d '{"data": "Item ID: WH001\nProduct: DC Motor Assembly\nQuantity: 120\nLocation: Rack A-01"}'
```

`results.csv` columns: `timestamp, item_id, product, quantity, location, raw_payload, status`.

**Security note:** do not commit real Wi-Fi names or passwords. Keep the placeholders in the code.

## 2. Simulation

Python 3.9+ with OpenCV (with `cv2.aruco`), NumPy, SciPy and Matplotlib:

```bash
pip install opencv-python numpy scipy matplotlib
python sim_pose.py 300          # marker pose accuracy (about 15 min)
python sim_qr.py 100            # QR read rate (about 5 min)
python run_experiments.py 100   # flight Monte Carlo (about 10 min)
python make_figs.py             # plots
```

Run the scripts in this order, from the repository folder. Seeds are fixed, so results should
reproduce closely; a different OpenCV version can shift them slightly.

### What the simulation does

- **Pose test:** renders ArUco markers (10 and 15 cm) at known poses (1280x720 camera, f = 900 px)
  with blur and noise, then measures the OpenCV pose error.
- **QR test:** renders QR labels (5, 8, 10 cm) on 640x480 frames (f = 600 px, fixed focus at 0.5 m)
  with defocus, motion blur, noise and JPEG compression, then counts successful decodes.
- **Flight simulation:** a quadcopter model with attitude lag, drag, gusts and a throttle bias.
  The camera runs at 30 Hz with delay, noise and dropouts. A Kalman filter and PID controller fly
  a six-waypoint, three-level scan mission. 100 random runs per setting.

All blur, noise, delay and dynamics parameters are assumptions and are listed in the code.

### Main results (simulated)

| Result | Value |
| --- | --- |
| 15 cm marker pose error at 2.5 m | 2.2 mm lateral, 1.3 mm vertical, 17.8 mm depth |
| 10 cm QR label per-frame read rate (0 degrees) | 98 % at 0.2 m, 93 % at 0.4 m, 96 % at 0.5 m, 75 % at 0.6 m, 0 % at 0.8 m |
| Mission completion, arrival tolerance 10 cm | 100 % in 50.5 s on average |
| Mission completion, arrival tolerance 5 cm / 3 cm | 71 % / 0 % |
| Without the Kalman filter (10 cm tolerance) | 100 % completion, but 11 % of runs leave the geofence |
| Total loop delay | 100 % up to 120 ms, 50 % at 140 ms, 0 % at 160 ms |

### Limitations

Rendered images and a simplified flight model are not a substitute for flight tests. Real lighting,
lens distortion, Wi-Fi jitter and prop wash are not modelled. Hardware validation is future work.

## Planned work

1. Forward commands from the ESP8266 to the flight controller over MSP.
2. Store scans in Firebase.
3. Staged hardware tests: calibration, command-link latency, tethered and free hover, three-level scan.

## AI assistance

The simulation code and the paper draft were developed with the help of Claude (Anthropic).
The author ran the experiments and verified the results.
