from flask import Flask, request, jsonify
from datetime import datetime
import csv
import os
import threading

app = Flask(__name__)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(BASE_DIR, "results.csv")
CSV_LOCK = threading.Lock()

FIELDS = [
    "timestamp", "item_id", "product", "quantity",
    "location", "raw_payload", "status"
]


def parse_inventory(text):
    parsed = {}
    for line in text.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            parsed[key.strip().lower()] = value.strip()
    return {
        "item_id": parsed.get("item id", ""),
        "product": parsed.get("product", ""),
        "quantity": parsed.get("quantity", ""),
        "location": parsed.get("location", ""),
    }


def save_scan(row):
    with CSV_LOCK:
        needs_header = (
            not os.path.exists(CSV_FILE)
            or os.path.getsize(CSV_FILE) == 0
        )
        with open(CSV_FILE, "a", newline="", encoding="utf-8-sig") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDS)
            if needs_header:
                writer.writeheader()
            writer.writerow(row)


@app.route("/", methods=["GET"])
def home():
    return jsonify({
        "status": "running",
        "message": "ESP32-CAM Warehouse QR Scanner Server",
        "endpoint": "POST /qrdata",
    })


@app.route("/qrdata", methods=["POST"])
def receive_qr():
    content = request.get_json(silent=True) or {}
    qr_text = content.get("data", "")

    if not isinstance(qr_text, str) or not qr_text.strip():
        return jsonify({
            "status": "error",
            "message": "Missing or empty string field: data",
        }), 400

    qr_text = qr_text.strip()
    inventory = parse_inventory(qr_text)
    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        **inventory,
        "raw_payload": qr_text,
        "status": "decoded",
    }

    try:
        save_scan(row)
    except OSError:
        app.logger.exception("Could not append scan to CSV")
        return jsonify({
            "status": "error",
            "message": "Could not save scan to CSV",
        }), 500

    print("\nQR code received")
    print(f"Item ID:  {inventory['item_id']}")
    print(f"Product:  {inventory['product']}")
    print(f"Quantity: {inventory['quantity']}")
    print(f"Location: {inventory['location']}")
    print(f"CSV file: {CSV_FILE}")

    return jsonify({
        "status": "success",
        "message": "QR payload received and saved",
        "inventory": inventory,
    }), 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
