"""
╔══════════════════════════════════════════════════════════════════╗
║   SmartSort AI — SO-ARM101 Robot Controller                      ║
║   Connects to Hiwonder SO-ARM101 via LeRobot + Feetech SDK       ║
║                                                                  ║
║   SETUP (run once before anything):                              ║
║   1. pip install -e ".[feetech]"   (inside lerobot folder)       ║
║   2. lerobot-find-port             (find your COM port)          ║
║   3. lerobot-setup-motors ...      (set motor IDs)               ║
║   4. lerobot-calibrate ...         (calibrate arm)               ║
║   5. Set PORT below to your COM port                             ║
║                                                                  ║
║   USAGE:                                                         ║
║       python robot_controller.py                                 ║
║       → Starts a local server on port 5050                       ║
║       → Simulation sends HTTP requests to trigger arm            ║
╚══════════════════════════════════════════════════════════════════╝
"""

import time
import threading
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# ── CONFIG — CHANGE THESE TO MATCH YOUR SETUP ──────────────────────
PORT         = "COM3"          # ← Change to your port (Windows: COM3, COM4 etc.)
                                #   Run: lerobot-find-port   to find it
ROBOT_ID     = "smartsort_follower"
SERVER_PORT  = 5050             # Local HTTP server port (simulation connects here)
MOVE_SPEED   = 150              # Motor speed (0-1000). Lower = slower, safer
MOVE_DELAY   = 0.6              # Seconds to wait between each joint move

# ── JOINT POSITIONS (tune these after calibration) ──────────────────
# Each position is: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
# Values are in degrees (-180 to +180 range after calibration)

POSITIONS = {

    # Home / standby position — arm resting upright
    "home": {
        "shoulder_pan":  0.0,
        "shoulder_lift": -30.0,
        "elbow_flex":     60.0,
        "wrist_flex":    -30.0,
        "wrist_roll":     0.0,
        "gripper":        0.0,   # open
    },

    # Ready position — arm hovering above conveyor
    "ready": {
        "shoulder_pan":  0.0,
        "shoulder_lift":  10.0,
        "elbow_flex":     45.0,
        "wrist_flex":    -20.0,
        "wrist_roll":     0.0,
        "gripper":        0.0,   # open
    },

    # DOWN — arm lowered to card level on conveyor belt
    "down": {
        "shoulder_pan":  0.0,
        "shoulder_lift":  55.0,   # ← increase to go lower
        "elbow_flex":     80.0,
        "wrist_flex":     10.0,
        "wrist_roll":     0.0,
        "gripper":       -20.0,   # slightly open for grab
    },

    # GRAB — gripper closes around card
    "grab": {
        "shoulder_pan":  0.0,
        "shoulder_lift":  55.0,
        "elbow_flex":     80.0,
        "wrist_flex":     10.0,
        "wrist_roll":     0.0,
        "gripper":        40.0,   # closed
    },

    # LIFT — raise card up after grabbing
    "lift": {
        "shoulder_pan":  0.0,
        "shoulder_lift":  10.0,
        "elbow_flex":     45.0,
        "wrist_flex":    -20.0,
        "wrist_roll":     0.0,
        "gripper":        40.0,   # still closed
    },

    # DROP — move over reject bin and drop
    "drop": {
        "shoulder_pan":  60.0,    # ← rotate toward reject bin
        "shoulder_lift":  20.0,
        "elbow_flex":     50.0,
        "wrist_flex":    -10.0,
        "wrist_roll":     0.0,
        "gripper":       -20.0,   # open = drop card
    },
}

# ── SORTED SEQUENCE ──────────────────────────────────────────────────
# When an INVALID card is detected, arm executes these steps in order:
REJECT_SEQUENCE = ["ready", "down", "grab", "lift", "drop", "home"]

# For demo mode (simple down-up without grabbing):
DEMO_SEQUENCE   = ["ready", "down", "home"]


# ══════════════════════════════════════════════════════════════════════
#   ROBOT DRIVER
# ══════════════════════════════════════════════════════════════════════
class SO101Controller:
    def __init__(self):
        self.robot     = None
        self.connected = False
        self.busy      = False
        self.last_action = "none"
        self._connect()

    def _connect(self):
        """Connect to the SO-ARM101 follower arm via LeRobot."""
        print(f"\n[ARM] Connecting to SO-ARM101 on {PORT}...")
        try:
            # Import LeRobot components
            from lerobot.common.robot_devices.robots.factory import make_robot
            from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError

            self.robot = make_robot(
                robot_type="so101_follower",
                port=PORT,
                id=ROBOT_ID,
            )
            self.robot.connect()
            self.connected = True
            print(f"[ARM] ✓ Connected successfully!")
            print(f"[ARM] Robot ID: {ROBOT_ID}")
            self._go_home()

        except ImportError:
            print("[ARM] ⚠ LeRobot not installed or not found in path.")
            print("[ARM]   Install: pip install -e '.[feetech]'  inside lerobot folder")
            print("[ARM]   Running in SIMULATION MODE (no physical movement)")
            self.connected = False

        except Exception as e:
            print(f"[ARM] ⚠ Connection failed: {e}")
            print(f"[ARM]   Check: Is {PORT} the correct port?")
            print(f"[ARM]   Run: lerobot-find-port  to find the right port")
            print(f"[ARM]   Running in SIMULATION MODE")
            self.connected = False

    def _send_position(self, position_name: str):
        """Send the arm to a named position."""
        if position_name not in POSITIONS:
            print(f"[ARM] Unknown position: {position_name}")
            return

        pos = POSITIONS[position_name]
        print(f"[ARM]   → Moving to: {position_name.upper()}")

        if not self.connected:
            # Simulation mode — just log it
            print(f"[ARM]   [SIM] Joints: {json.dumps(pos, indent=6)}")
            time.sleep(MOVE_DELAY)
            return

        try:
            # Send all joint positions to the robot
            action = {}
            for joint_name, angle in pos.items():
                action[joint_name] = angle

            self.robot.send_action(action)
            time.sleep(MOVE_DELAY)

        except Exception as e:
            print(f"[ARM] Error moving to {position_name}: {e}")

    def execute_sequence(self, sequence: list, label: str = ""):
        """Execute a sequence of positions."""
        if self.busy:
            print("[ARM] ⚠ Arm is busy — ignoring trigger")
            return False

        self.busy = True
        self.last_action = label

        print(f"\n[ARM] ══ EXECUTING: {label} ══")
        print(f"[ARM]   Steps: {' → '.join(s.upper() for s in sequence)}")

        try:
            for step in sequence:
                self._send_position(step)
            print(f"[ARM] ✓ Sequence complete\n")
        except Exception as e:
            print(f"[ARM] ✗ Sequence error: {e}")
        finally:
            self.busy = False

        return True

    def _go_home(self):
        """Move arm to home position on startup."""
        print("[ARM] Moving to HOME position...")
        self._send_position("home")

    def trigger_reject(self, card_info: dict = None):
        """Called when an INVALID card is detected — full reject sequence."""
        label = "REJECT"
        if card_info:
            missing = card_info.get("missing_field", "unknown")
            name    = card_info.get("name", "unknown")
            label   = f"REJECT [{missing}] — {name}"

        print(f"\n[ARM] ⚡ INVALID CARD DETECTED!")
        if card_info:
            print(f"[ARM]   Card  : {card_info.get('name', '?')}")
            print(f"[ARM]   Issue : {card_info.get('missing_field', '?')}")

        # Run in a thread so HTTP server stays responsive
        t = threading.Thread(
            target=self.execute_sequence,
            args=(REJECT_SEQUENCE, label),
            daemon=True
        )
        t.start()

    def trigger_demo(self):
        """Simple down-up demo — good for showing judges."""
        t = threading.Thread(
            target=self.execute_sequence,
            args=(DEMO_SEQUENCE, "DEMO"),
            daemon=True
        )
        t.start()

    def get_status(self) -> dict:
        return {
            "connected":   self.connected,
            "busy":        self.busy,
            "last_action": self.last_action,
            "port":        PORT,
            "robot_id":    ROBOT_ID,
            "mode":        "LIVE" if self.connected else "SIMULATION",
        }

    def disconnect(self):
        if self.connected and self.robot:
            try:
                self._go_home()
                self.robot.disconnect()
                print("[ARM] Disconnected.")
            except Exception as e:
                print(f"[ARM] Disconnect error: {e}")


# ══════════════════════════════════════════════════════════════════════
#   HTTP SERVER  (simulation sends requests here)
# ══════════════════════════════════════════════════════════════════════
arm = None  # global arm instance

class RobotHTTPHandler(BaseHTTPRequestHandler):
    """
    Endpoints:
        GET  /status          → returns arm status as JSON
        POST /reject          → triggers full reject sequence
        POST /demo            → triggers simple demo down-up
        POST /home            → moves arm to home
    """

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/status":
            self._json(arm.get_status())

        elif path == "/health":
            self._json({"ok": True})

        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        path = urlparse(self.path).path

        # Read body if any
        length   = int(self.headers.get("Content-Length", 0))
        raw_body = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw_body)
        except Exception:
            body = {}

        if path == "/reject":
            card_info = body.get("card", {})
            arm.trigger_reject(card_info)
            self._json({"triggered": True, "mode": "reject", "card": card_info})

        elif path == "/demo":
            arm.trigger_demo()
            self._json({"triggered": True, "mode": "demo"})

        elif path == "/home":
            threading.Thread(
                target=arm.execute_sequence,
                args=(["home"], "HOME"),
                daemon=True
            ).start()
            self._json({"triggered": True, "mode": "home"})

        else:
            self._json({"error": "unknown endpoint"}, 404)

    def _json(self, data: dict, code: int = 200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")  # allow browser fetch
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        # Custom log format
        print(f"[HTTP] {self.address_string()} — {fmt % args}")

    def do_OPTIONS(self):
        # CORS preflight
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()


# ══════════════════════════════════════════════════════════════════════
#   MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    global arm

    print("=" * 60)
    print("  SmartSort AI — SO-ARM101 Robot Controller")
    print("=" * 60)

    # Init arm
    arm = SO101Controller()

    # Start HTTP server
    server = HTTPServer(("0.0.0.0", SERVER_PORT), RobotHTTPHandler)

    print(f"\n[SERVER] ✓ Listening on http://localhost:{SERVER_PORT}")
    print(f"[SERVER]   /status  → arm status")
    print(f"[SERVER]   /reject  → trigger reject sequence")
    print(f"[SERVER]   /demo    → trigger demo movement")
    print(f"[SERVER]   /home    → return to home")
    print(f"\n[SERVER] Keep this running while using the simulation!")
    print(f"[SERVER] Press Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[SERVER] Shutting down...")
        arm.disconnect()
        server.shutdown()


if __name__ == "__main__":
    main()
