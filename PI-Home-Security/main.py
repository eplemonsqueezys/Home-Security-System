from flask import Flask, render_template, jsonify, request
import threading
import time
import json
import os
import atexit
from collections import defaultdict # For schedules by ID

# --- Mock RPi.GPIO for non-Raspberry Pi environments ---
try:
    import RPi.GPIO as GPIO
    print("RPi.GPIO imported successfully. Running on Raspberry Pi.")
except ImportError:
    # Mock RPi.GPIO for non-Raspberry Pi environments
    class MockGPIO:
        BCM = 11
        IN = 1
        OUT = 0
        HIGH = 1
        LOW = 0
        PUD_UP = 20
        def setmode(self, mode): print("Mock GPIO: setmode called")
        def setwarnings(self, state): print("Mock GPIO: setwarnings called")
        def setup(self, pin, mode, pull_up_down=None):
            print(f"Mock GPIO: Setup pin {pin} as {'IN' if mode == self.IN else 'OUT'}")
        def input(self, pin):
            return self.LOW # Default to not tripped for stability
        def output(self, pin, value):
            state = "HIGH" if value == self.HIGH else "LOW"
            print(f"Mock GPIO: Pin {pin} set to {state}")
        def cleanup(self, pin=None):
            print(f"Mock GPIO: Cleaning up pin {pin if pin else 'all'}")
    GPIO = MockGPIO()
    print("RPi.GPIO not found, using mock GPIO. Physical GPIO operations will be simulated.")

app = Flask(__name__)
CONFIG_FILE = "config.json"
DEFAULT_ZONE_PINS = [4, 17, 18, 27, 22, 23, 24, 25] # Default GPIO pins for zones
RELAY_PINS = [5, 6, 13, 19, 26, 20, 21, 16] # Default GPIO pins for relays (e.g., siren)

# Global state variables
armed = False # Master arm/disarm state
zone_state = [] # Current physical state of each zone (True if tripped/open, False if closed)
zone_armed = [] # Whether each zone is currently armed for detection (can be manually toggled)
zone_labels = [] # User-defined labels for each zone (e.g., "Front Door", "Living Room Window")
pin_layout = {} # Mapping of zone index to GPIO pin number
schedules = [] # List of scheduled arm/disarm events
alerted_zones = set() # Zones that have already triggered an alert in the current alarm state
zone_ding_unarmed = [] # Zones that "ding" (make a sound) when tripped while the system is unarmed
home_mode_zones = [] # Zones configured to be armed when system is in 'home' mode
away_mode_zones = [] # Zones configured to be armed when system is in 'away' mode
mode = "away" # Current system mode ("home" or "away")

schedule_id_counter = 0 # Counter for unique schedule IDs
current_zone_pins = [] # List of currently active GPIO pins for zones
lock = threading.Lock() # Lock for thread-safe access to shared global variables

# Keep track of one-time schedules executed today to prevent re-execution
executed_schedules_today = set()
# Flag to control the schedule runner thread
schedule_runner_active = True

# --- GPIO Initialization ---
GPIO.setmode(GPIO.BCM) # Use Broadcom SOC channel numbers
GPIO.setwarnings(False) # Disable GPIO warnings

def free_pins(pins):
    """Cleans up GPIO pins to release them."""
    for pin in pins:
        try:
            GPIO.cleanup(pin)
            print(f"GPIO: Cleaned up pin {pin}")
        except Exception as e:
            print(f"GPIO: Error cleaning up pin {pin}: {e}")

def setup_zone_pins(new_pins):
    """Sets up new GPIO pins for zones and reinitializes zone states."""
    global current_zone_pins, zone_state, zone_armed
    with lock:
        # Clean up previously used pins
        free_pins(current_zone_pins)
        current_zone_pins[:] = new_pins # Update global list of current pins

        # Initialize zone states and arming status for new pins
        num_zones = len(current_zone_pins)
        zone_state[:] = [False] * num_zones # All zones initially not tripped
        zone_armed[:] = [True] * num_zones

        for p in current_zone_pins:
            if p == 0: # Skip placeholder for unassigned pins
                print(f"GPIO: Skipping pin setup for unassigned pin (0).")
                continue
            try:
                # Set up pin as input with pull-up resistor (assuming normally closed sensors)
                GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                print(f"GPIO: Setup input pin {p} with PUD_UP")
            except Exception as e:
                print(f"[GPIO INPUT ERROR] Pin {p}: {e}")

def setup_relay_pins():
    """Sets up GPIO pins for relays as outputs and ensures they are off."""
    with lock:
        free_pins(RELAY_PINS) # Clean up any prior relay pin setups
        for pin in RELAY_PINS:
            try:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW) # Ensure relay is off (e.g., siren is silent)
                print(f"GPIO: Setup output pin {pin} to LOW")
            except Exception as e:
                print(f"[GPIO OUTPUT ERROR] Relay pin {pin}: {e}")

# --- Configuration Management ---
def save_config():
    """Saves the current application configuration to a JSON file."""
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump({
                "pin_layout": pin_layout,
                "zone_labels": zone_labels,
                "zone_ding_unarmed": zone_ding_unarmed,
                "home_mode_zones": home_mode_zones,
                "away_mode_zones": away_mode_zones,
                "schedules": schedules, # Include schedules in saved config
                "mode": mode # Save current mode
            }, f, indent=4) # Use indent for readability
        print("[CONFIG] Configuration saved successfully.")
    except Exception as e:
        print(f"[CONFIG] Save error: {e}")

def load_config():
    """Loads application configuration from a JSON file.
    Initializes default values if the file doesn't exist."""
    global pin_layout, zone_labels, zone_ding_unarmed, home_mode_zones, \
           away_mode_zones, schedules, schedule_id_counter, mode, armed

    default_num_zones = len(DEFAULT_ZONE_PINS)
    # Default initial states for a fresh configuration
    default_zone_labels = [f"Zone {i+1}" for i in range(default_num_zones)]
    default_pin_layout = {str(i): DEFAULT_ZONE_PINS[i] for i in range(default_num_zones)}
    default_home_mode_zones = list(range(default_num_zones))
    default_away_mode_zones = list(range(default_num_zones))

    if not os.path.exists(CONFIG_FILE):
        print(f"[CONFIG] {CONFIG_FILE} not found. Creating default configuration.")
        pin_layout.update(default_pin_layout)
        zone_labels[:] = default_zone_labels
        zone_ding_unarmed[:] = []
        home_mode_zones[:] = default_home_mode_zones
        away_mode_zones[:] = default_away_mode_zones
        schedules[:] = []
        schedule_id_counter = 0
        mode = "away"
        armed = False # Start disarmed by default
        pins = [pin_layout[str(i)] for i in range(len(zone_labels))]
        setup_zone_pins(pins)
        save_config()
        return

    try:
        with open(CONFIG_FILE, "r") as f:
            data = json.load(f)
            pin_layout.update(data.get("pin_layout", default_pin_layout))
            loaded_zone_labels = data.get("zone_labels", default_zone_labels)
            zone_labels[:] = loaded_zone_labels
            num_loaded_zones = len(zone_labels)

            zone_ding_unarmed[:] = data.get("zone_ding_unarmed", [])
            home_mode_zones[:] = data.get("home_mode_zones", [])
            away_mode_zones[:] = data.get("away_mode_zones", [])
            schedules[:] = data.get("schedules", [])
            mode = data.get("mode", "away")

            schedule_id_counter = max([s['id'] for s in schedules]) + 1 if schedules else 0

            pins_to_setup = []
            for i in range(num_loaded_zones):
                pin = pin_layout.get(str(i))
                if pin is not None:
                    pins_to_setup.append(int(pin))
                else:
                    print(f"Warning: No pin defined in config for zone {i} ('{zone_labels[i]}'). Using default if available or 0.")
                    if i < len(DEFAULT_ZONE_PINS):
                        pins_to_setup.append(DEFAULT_ZONE_PINS[i])
                        pin_layout[str(i)] = DEFAULT_ZONE_PINS[i]
                    else:
                        pins_to_setup.append(0)
                        pin_layout[str(i)] = 0

            setup_zone_pins(pins_to_setup)
            with lock:
                if mode == "home":
                    zone_armed[:] = [True if i in home_mode_zones else False for i in range(len(zone_state))]
                else:
                    zone_armed[:] = [True if i in away_mode_zones else False for i in range(len(zone_state))]
                armed = False

            print("[CONFIG] Configuration loaded successfully.")
    except Exception as e:
        print(f"[CONFIG] Load error: {e}. Reverting to default configuration.")
        pin_layout.clear()
        for i in range(default_num_zones):
            pin_layout[str(i)] = DEFAULT_ZONE_PINS[i]
        zone_labels[:] = default_zone_labels
        zone_ding_unarmed[:] = []
        home_mode_zones[:] = default_home_mode_zones
        away_mode_zones[:] = default_away_mode_zones
        schedules[:] = []
        schedule_id_counter = 0
        mode = "away"
        armed = False
        pins = [pin_layout[str(i)] for i in range(len(zone_labels))]
        setup_zone_pins(pins)
        save_config()

# --- Initialize GPIO and Load Configuration on Startup ---
setup_relay_pins()
load_config()

# --- Background Threads ---
def poll_zones():
    """Continuously polls the state of alarm zones via GPIO inputs."""
    while True:
        with lock:
            num_current_zones = len(current_zone_pins)
            for i in range(num_current_zones):
                p = current_zone_pins[i]
                if p == 0:
                    zone_state[i] = False
                    continue
                try:
                    val = GPIO.input(p)
                    zone_state[i] = (val == GPIO.HIGH)
                except Exception as e:
                    print(f"Error reading GPIO pin {p} for zone {i}: {e}. Setting zone state to False.")
                    zone_state[i] = False
        time.sleep(0.1)

def alarm_watcher():
    """Monitors zone states and triggers alarm and ding."""
    global alerted_zones
    last_relay_state = None

    while True:
        try:
            current_triggered_by_armed_zone = False
            current_relay_output_state = GPIO.LOW

            with lock:
                for i, tripped in enumerate(zone_state):
                    if i < len(zone_armed) and zone_armed[i] and tripped:
                        current_triggered_by_armed_zone = True
                        break

                current_relay_output_state = GPIO.HIGH if (armed and current_triggered_by_armed_zone) else GPIO.LOW

                if current_relay_output_state != last_relay_state:
                    action = "ACTIVATING" if current_relay_output_state == GPIO.HIGH else "DEACTIVATING"
                    print(f"Alarm Relays: {action} (System Armed: {armed}, Armed Zone Tripped: {current_triggered_by_armed_zone})")
                    for p in RELAY_PINS:
                        try:
                            GPIO.output(p, current_relay_output_state)
                            print(f"  GPIO Pin {p} set to {current_relay_output_state}")
                        except Exception as e:
                            print(f"ERROR: Could not control relay pin {p}: {e}")
                    last_relay_state = current_relay_output_state

                for i, tripped in enumerate(zone_state):
                    if i >= len(zone_labels) or i >= len(zone_armed) or i >= len(zone_ding_unarmed):
                        print(f"WARNING: Zone index {i} out of bounds for some configuration lists. Skipping alerts for this zone.")
                        continue

                    current_zone_label = zone_labels[i]
                    current_zone_is_armed_for_detection = zone_armed[i]
                    current_zone_dings_unarmed = (i in zone_ding_unarmed)

                    if tripped:
                        if not current_zone_is_armed_for_detection and current_zone_dings_unarmed:
                            try:
                                with open("static/ding.flag", "w") as f: f.write("1")
                                print(f"Ding flag set for unarmed zone '{current_zone_label}'.")
                            except IOError as e:
                                print(f"[DING FLAG ERROR] Could not write static/ding.flag: {e}")

                        if current_zone_is_armed_for_detection and armed and i not in alerted_zones:
                            print(f"Zone '{current_zone_label}' tripped and armed.")
                            alerted_zones.add(i)
                    else:
                        if i in alerted_zones:
                            print(f"Zone '{current_zone_label}' is no longer tripped. Removing from alerted_zones.")
                            alerted_zones.remove(i)

            time.sleep(0.2)
        except Exception as e:
            print(f"ERROR in alarm_watcher loop: {e}")
            time.sleep(1)

def schedule_runner():
    """Periodically checks and executes scheduled arm/disarm events."""
    global armed
    last_checked_day = time.localtime().tm_mday

    while schedule_runner_active:
        now = time.localtime()
        current_time_str = time.strftime("%H:%M", now)
        current_day = now.tm_mday

        if current_day != last_checked_day:
            with lock:
                executed_schedules_today.clear()
                print(f"[SCHEDULE] Cleared executed schedules for the new day (Day {current_day}).")
            last_checked_day = current_day

        with lock:
            for sched in schedules:
                if sched['time'] == current_time_str:
                    if sched['repeat'] or sched['id'] not in executed_schedules_today:
                        if sched['action'] == 'arm' and not armed:
                            armed = True
                            print(f"[SCHEDULE] System armed by schedule at {current_time_str}")
                        elif sched['action'] == 'disarm' and armed:
                            armed = False
                            print(f"[SCHEDULE] System disarmed by schedule at {current_time_str}")

                        if not sched['repeat']:
                            executed_schedules_today.add(sched['id'])
        time.sleep(30)

# Start background threads
threading.Thread(target=poll_zones, daemon=True).start()
threading.Thread(target=alarm_watcher, daemon=True).start()
threading.Thread(target=schedule_runner, daemon=True).start()

# --- Flask Routes ---
@app.route('/')
def index():
    """Renders the main alarm dashboard HTML page."""
    return render_template('index.html')

@app.route('/api/zone_status')
def zone_status():
    """Returns the current status of the alarm system and zones."""
    with lock:
        serializable_pin_layout = {int(k): v for k, v in pin_layout.items()}
        return jsonify({
            'armed': armed,
            'zones': zone_state,
            'zone_armed': zone_armed,
            'zone_labels': zone_labels,
            'pin_layout': serializable_pin_layout,
            'mode': mode,
            'home_mode_zones': home_mode_zones,
            'away_mode_zones': away_mode_zones,
            'zone_ding_unarmed': zone_ding_unarmed
        })

@app.route('/api/arm', methods=['POST'])
def api_arm():
    """API to arm or disarm the entire alarm system."""
    global armed
    data = request.json or {}
    new_armed_state = data.get('armed')

    if not isinstance(new_armed_state, bool):
        return jsonify({'error': 'Invalid request. "armed" (boolean) is required.'}), 400

    with lock:
        armed = new_armed_state
        print(f"System {'ARMED' if armed else 'DISARMED'} via API.")
    return ('', 204)

@app.route('/api/zone_arm', methods=['POST'])
def api_zone_arm():
    """API to arm or disarm a specific zone for detection."""
    data = request.json or {}
    idx = data.get('index')
    status = data.get('armed')

    if not isinstance(idx, int) or not isinstance(status, bool):
        return jsonify({'error': 'Invalid request. "index" (int) and "armed" (boolean) are required.'}), 400

    with lock:
        if 0 <= idx < len(zone_armed):
            zone_armed[idx] = status
            print(f"Zone {idx} ('{zone_labels[idx]}') set to {'ARMED' if status else 'DISARMED'} for detection.")
            return ('', 204)
        else:
            return jsonify({'error': f'Zone index {idx} out of bounds (0-{len(zone_armed)-1}).'}), 400

@app.route('/api/set_labels', methods=['POST'])
def api_set_labels():
    """API to set user-defined labels for zones."""
    data = request.get_json() or {}
    labs = data.get('labels', [])

    if not isinstance(labs, list):
        return jsonify({'error': 'Invalid labels format (expected list of strings).'}), 400

    with lock:
        current_num_zones = len(zone_labels)
        new_num_zones = len(labs)

        zone_labels[:] = labs

        if new_num_zones > current_num_zones:
            extension_length = new_num_zones - current_num_zones
            zone_state.extend([False] * extension_length)
            zone_armed.extend([True] * extension_length)
            for i in range(current_num_zones, new_num_zones):
                if i < len(DEFAULT_ZONE_PINS):
                    pin_layout[str(i)] = DEFAULT_ZONE_PINS[i]
                else:
                    pin_layout[str(i)] = 0
                home_mode_zones.append(i)
                away_mode_zones.append(i)
        elif new_num_zones < current_num_zones:
            zone_state[:] = zone_state[:new_num_zones]
            zone_armed[:] = zone_armed[:new_num_zones]
            zone_ding_unarmed[:] = [idx for idx in zone_ding_unarmed if idx < new_num_zones]
            home_mode_zones[:] = [idx for idx in home_mode_zones if idx < new_num_zones]
            away_mode_zones[:] = [idx for idx in away_mode_zones if idx < new_num_zones]

            new_pin_layout_dict = {}
            for i in range(new_num_zones):
                if str(i) in pin_layout:
                    new_pin_layout_dict[str(i)] = pin_layout[str(i)]
                elif i < len(DEFAULT_ZONE_PINS):
                    new_pin_layout_dict[str(i)] = DEFAULT_ZONE_PINS[i]
                else:
                    new_pin_layout_dict[str(i)] = 0
            pin_layout.clear()
            pin_layout.update(new_pin_layout_dict)

        current_pins_for_setup = []
        for i in range(new_num_zones):
            pin = pin_layout.get(str(i))
            if pin is not None:
                current_pins_for_setup.append(int(pin))
            else:
                if i < len(DEFAULT_ZONE_PINS):
                    current_pins_for_setup.append(DEFAULT_ZONE_PINS[i])
                else:
                    current_pins_for_setup.append(0)

        setup_zone_pins(current_pins_for_setup)
        save_config()
        print(f"Zone labels and system size updated. Current zones: {len(zone_labels)}")
        return ('', 204)

@app.route('/api/set_ding_zones', methods=['POST'])
def api_set_ding_zones():
    """API to set which zones 'ding' when tripped while the system is unarmed."""
    data = request.json or {}
    ding_zones = data.get("ding_zones", [])
    if not isinstance(ding_zones, list):
        return jsonify({'error': 'Invalid ding_zones format (expected list of integers)'}), 400

    with lock:
        valid_ding_zones = [idx for idx in ding_zones if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        zone_ding_unarmed[:] = valid_ding_zones
        save_config()
        print(f"Ding zones updated: {zone_ding_unarmed}")
    return ('', 204)

@app.route('/api/set_mode', methods=['POST'])
def api_set_mode():
    """API to set the system mode ('home' or 'away') and update zone arming accordingly."""
    global mode, zone_armed
    data = request.json or {}
    new_mode = data.get("mode")

    if new_mode not in ["home", "away"]:
        return jsonify({'error': 'Invalid mode. Must be "home" or "away"'}), 400

    with lock:
        mode = new_mode
        if mode == "home":
            zone_armed[:] = [True if i in home_mode_zones else False for i in range(len(zone_state))]
            print(f"Mode set to HOME. Zones armed for detection: {[zone_labels[i] for i in home_mode_zones if i < len(zone_labels)]}")
        else:
            zone_armed[:] = [True if i in away_mode_zones else False for i in range(len(zone_state))]
            print(f"Mode set to AWAY. Zones armed for detection: {[zone_labels[i] for i in away_mode_zones if i < len(zone_labels)]}")
        save_config()
    return ('', 204)

@app.route('/api/set_mode_zones', methods=['POST'])
def api_set_mode_zones():
    """API to configure which zones are armed in 'home' and 'away' modes."""
    data = request.json or {}
    home_config = data.get("home", [])
    away_config = data.get("away", [])

    if not isinstance(home_config, list) or not isinstance(away_config, list):
        return jsonify({'error': 'Invalid mode zone configuration format (expected lists of integers)'}), 400

    with lock:
        home_mode_zones[:] = [idx for idx in home_config if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        away_mode_zones[:] = [idx for idx in away_config if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        save_config()
        print(f"Home mode zones config: {home_mode_zones}")
        print(f"Away mode zones config: {away_mode_zones}")
    return ('', 204)

@app.route('/api/clear_ding')
def clear_ding():
    """API to clear the 'ding' flag file (used by frontend to stop ding sound)."""
    try:
        if os.path.exists("static/ding.flag"):
            os.remove("static/ding.flag")
            print("Ding flag cleared.")
    except Exception as e:
        print(f"Error clearing ding flag: {e}")
    return ('', 204)

@app.route('/api/schedules')
def api_schedules():
    """Returns the list of configured schedules."""
    with lock:
        return jsonify(schedules)

@app.route('/api/add_schedule', methods=['POST'])
def api_add_schedule():
    """API to add a new schedule."""
    global schedule_id_counter
    data = request.json or {}
    zone = data.get('zone')
    action = data.get('action')
    time_str = data.get('time')
    repeat = data.get('repeat', False)

    if action not in ['arm', 'disarm'] or not time_str or not isinstance(time_str, str) or not isinstance(repeat, bool):
        return jsonify({'error': 'Invalid schedule data. Requires "action" (arm/disarm), "time" (HH:MM), and "repeat" (boolean).'}), 400
    if not (isinstance(zone, int) and 0 <= zone < len(zone_labels) or zone is None):
         return jsonify({'error': 'Invalid zone index provided for schedule context.'}), 400

    with lock:
        new_schedule = {
            'id': schedule_id_counter,
            'zone': zone,
            'action': action,
            'time': time_str,
            'repeat': repeat
        }
        schedules.append(new_schedule)
        schedule_id_counter += 1
        save_config()
        print(f"Schedule added: {new_schedule}")
    return jsonify(new_schedule), 201

@app.route('/api/delete_schedule', methods=['POST'])
def api_delete_schedule():
    """API to delete a schedule by its ID."""
    data = request.json or {}
    sched_id = data.get('id')

    if not isinstance(sched_id, int):
        return jsonify({'error': 'Schedule ID (integer) is required.'}), 400

    with lock:
        global schedules
        initial_len = len(schedules)
        schedules = [s for s in schedules if s['id'] != sched_id]
        if len(schedules) < initial_len:
            save_config()
            print(f"Schedule {sched_id} deleted.")
            return ('', 204)
        else:
            return jsonify({'error': f'Schedule with ID {sched_id} not found.'}), 404

@app.route('/api/set_pin_layout', methods=['POST'])
def api_set_pin_layout():
    """API to update the GPIO pin assignments for zones."""
    data = request.json or {}
    
    if not isinstance(data, dict):
        return jsonify({'error': 'Invalid pin layout format (expected dictionary of zone_idx: pin_number).'}), 400

    new_pin_layout_received = {}
    for k, v in data.items():
        try:
            zone_idx = int(k)
            pin_num = int(v)
            if 0 <= zone_idx < len(zone_labels):
                new_pin_layout_received[str(zone_idx)] = pin_num
            else:
                print(f"Warning: Attempted to set pin for non-existent zone index {zone_idx}. Skipping.")
        except ValueError:
            return jsonify({'error': f'Invalid pin data for key "{k}": value "{v}". Expected integers.'}), 400

    with lock:
        pin_layout.update(new_pin_layout_received)

        current_pins_for_setup = []
        for i in range(len(zone_labels)):
            pin = pin_layout.get(str(i))
            if pin is not None:
                current_pins_for_setup.append(int(pin))
            else:
                if i < len(DEFAULT_ZONE_PINS):
                    current_pins_for_setup.append(DEFAULT_ZONE_PINS[i])
                    pin_layout[str(i)] = DEFAULT_ZONE_PINS[i]
                else:
                    current_pins_for_setup.append(0)
                    pin_layout[str(i)] = 0

        setup_zone_pins(current_pins_for_setup)
        save_config()
        print(f"Pin layout updated: {pin_layout}")
    return ('', 204)

# --- Cleanup on Exit ---
def cleanup():
    """Cleans up all GPIO pins when the application exits."""
    global schedule_runner_active
    schedule_runner_active = False
    time.sleep(0.5)
    free_pins(current_zone_pins)
    free_pins(RELAY_PINS)
    GPIO.cleanup()
    print("Application exiting. GPIO cleanup complete.")

atexit.register(cleanup)    

if __name__ == '__main__':
    if not os.path.exists('static'):
        os.makedirs('static')
    if not os.path.exists('static/ding.flag'):
        with open('static/ding.flag', 'w') as f:
            f.write('0')

    app.run(host='0.0.0.0', port=5000, debug=False)