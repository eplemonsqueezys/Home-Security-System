from flask import Flask, render_template, jsonify, request
import threading
import time
import json
import os
import atexit
import logging
from collections import defaultdict
import datetime
import subprocess

# --- Update system packages ---
try:
    updateres = subprocess.run(["sudo", "apt-get", "update", "-y"], check=True)
    print(f"Update result: {updateres.returncode}")
    updateres = subprocess.run(["sudo", "apt-get", "upgrade", "-y"], check=True)
    print(f"Update result: {updateres.returncode}")
    updateres = subprocess.run(["git", "pull"], check=True)
    print(f"Update result: {updateres.returncode}")
    os.system("@reboot sudo python3 /home/pi/PI-Home-Security/main.py")
    
except subprocess.CalledProcessError as e:
    print.error(f"Update requires restart....: {e}. Restarting system!.")
# --- End of system update ---


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("AlarmSystem")

# --- Mock RPi.GPIO for non-Raspberry Pi environments ---
try:
    import RPi.GPIO as GPIO
    logger.info("RPi.GPIO imported successfully. Running on Raspberry Pi.")
except ImportError:
    logger.warning("RPi.GPIO not found, using mock GPIO. Physical GPIO operations will be simulated.")
    class MockGPIO:
        BCM = 11
        IN = 1
        OUT = 0
        HIGH = 1
        LOW = 0
        PUD_UP = 20
        
        def setmode(self, mode): 
            logger.info("Mock GPIO: setmode called")
            
        def setwarnings(self, state): 
            logger.info("Mock GPIO: setwarnings called")
            
        def setup(self, pin, mode, pull_up_down=None):
            logger.info(f"Mock GPIO: Setup pin {pin} as {'IN' if mode == self.IN else 'OUT'}")
            
        def input(self, pin):
            return self.LOW
            
        def output(self, pin, value):
            state = "HIGH" if value == self.HIGH else "LOW"
            logger.info(f"Mock GPIO: Pin {pin} set to {state}")
            
        def cleanup(self, pin=None):
            logger.info(f"Mock GPIO: Cleaning up pin {pin if pin else 'all'}")
            
    GPIO = MockGPIO()

app = Flask(__name__)
CONFIG_FILE = "config.json"
DEFAULT_ZONE_PINS = [4, 17, 18, 27, 22, 16, 24, 25]
RELAY_PINS = [23]

# Global state variables
armed = False
zone_state = []
zone_armed = []
zone_labels = []
pin_layout = {}
schedules = []
alerted_zones = set()
zone_ding_unarmed = []
home_mode_zones = []
away_mode_zones = []
mode = "away"
schedule_id_counter = 0
current_zone_pins = []
lock = threading.RLock()  
executed_schedules_today = set()
schedule_runner_active = True
alarm_triggered = False
ding_triggered = False
last_ding_time = 0
DING_COOLDOWN = 5  # seconds between dings 

# --- GPIO Initialization ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def free_pins(pins):
    """Must be called while holding the lock"""
    for pin in pins:
        if pin == 0:  # Skip unassigned pins
            continue
        try:
            GPIO.cleanup(pin)
            logger.info(f"GPIO: Cleaned up pin {pin}")
        except Exception as e:
            logger.error(f"GPIO: Error cleaning up pin {pin}: {e}")

def setup_zone_pins(new_pins):
    global current_zone_pins, zone_state, zone_armed
    with lock:
        free_pins(current_zone_pins)
        current_zone_pins[:] = new_pins
        num_zones = len(current_zone_pins)
        zone_state[:] = [False] * num_zones
        zone_armed[:] = [True] * num_zones

        for p in current_zone_pins:
            if p == 0:
                logger.info(f"GPIO: Skipping pin setup for unassigned pin (0).")
                continue
            try:
                GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                logger.info(f"GPIO: Setup input pin {p} with PUD_UP")
            except Exception as e:
                logger.error(f"[GPIO INPUT ERROR] Pin {p}: {e}")

def setup_relay_pins():
    with lock:
        for pin in RELAY_PINS:
            try:
                GPIO.cleanup(pin)  # Ensure the pin is cleaned up before setup
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                logger.info(f"GPIO: Setup output pin {pin} to LOW")
            except Exception as e:
                logger.error(f"[GPIO OUTPUT ERROR] Relay pin {pin}: {e}")

# --- Configuration Management ---
def save_config():
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump({
                "pin_layout": pin_layout,
                "zone_labels": zone_labels,
                "zone_ding_unarmed": zone_ding_unarmed,
                "home_mode_zones": home_mode_zones,
                "away_mode_zones": away_mode_zones,
                "schedules": schedules,
                "mode": mode,
                "armed": armed
            }, f, indent=4)
        logger.info("[CONFIG] Configuration saved successfully.")
    except Exception as e:
        logger.error(f"[CONFIG] Save error: {e}")

def load_config():
    global pin_layout, zone_labels, zone_ding_unarmed, home_mode_zones, \
        away_mode_zones, schedules, schedule_id_counter, mode, armed

    default_num_zones = len(DEFAULT_ZONE_PINS)
    default_zone_labels = [f"Zone {i+1}" for i in range(default_num_zones)]
    default_pin_layout = {str(i): DEFAULT_ZONE_PINS[i] for i in range(default_num_zones)}
    default_home_mode_zones = list(range(default_num_zones))
    default_away_mode_zones = list(range(default_num_zones))

    if not os.path.exists(CONFIG_FILE):
        logger.info(f"[CONFIG] {CONFIG_FILE} not found. Creating default configuration.")
        pin_layout.update(default_pin_layout)
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
        armed = data.get("armed", False)

        schedule_id_counter = max([s['id'] for s in schedules]) + 1 if schedules else 0

        pins_to_setup = []
        for i in range(num_loaded_zones):
            pin = pin_layout.get(str(i))
            if pin is not None:
                pins_to_setup.append(int(pin))
            else:
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

        logger.info("[CONFIG] Configuration loaded successfully.")
    except Exception as e:
        logger.error(f"[CONFIG] Load error: {e}. Reverting to default configuration.")
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

# Initialize GPIO under lock protection
with lock:
    setup_relay_pins()
    load_config()

# --- Background Threads ---
def poll_zones():
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
                    logger.error(f"Error reading GPIO pin {p} for zone {i}: {e}. Setting zone state to False.")
                    zone_state[i] = False
        time.sleep(0.1)

def alarm_watcher():
    global alerted_zones, alarm_triggered, ding_triggered, last_ding_time
    last_relay_state = None

    while True:
        try:
            current_triggered_by_armed_zone = False
            current_relay_output_state = GPIO.LOW

            with lock:
                # Check if any armed zone is tripped
                for i, tripped in enumerate(zone_state):
                    if i < len(zone_armed) and zone_armed[i] and tripped:
                        current_triggered_by_armed_zone = True
                        break

                # Set relay state based on armed status and zone trips
                current_relay_output_state = GPIO.HIGH if (armed and current_triggered_by_armed_zone) else GPIO.LOW

                if current_relay_output_state != last_relay_state:
                    action = "ACTIVATING" if current_relay_output_state == GPIO.HIGH else "DEACTIVATING"
                    logger.info(f"Alarm Relays: {action} (System Armed: {armed}, Armed Zone Tripped: {current_triggered_by_armed_zone})")
                    for p in RELAY_PINS:
                        try:
                            GPIO.output(p, current_relay_output_state)
                            logger.info(f" GPIO Pin {p} set to {current_relay_output_state}")
                        except Exception as e:
                            logger.error(f"ERROR: Could not control relay pin {p}: {e}")
                    last_relay_state = current_relay_output_state

                # Handle alarm flag - only set if system is armed and zone is tripped
                if armed and current_triggered_by_armed_zone:
                    if not alarm_triggered:
                        alarm_triggered = True
                        try:
                            with open("static/alarm.flag", "w") as f: 
                                f.write("1")
                            logger.info("Alarm flag set for armed zone tripped.")
                        except IOError as e:
                            logger.error(f"[ALARM FLAG ERROR] Could not write static/alarm.flag: {e}")
                elif alarm_triggered:
                    alarm_triggered = False
                    try:
                        if os.path.exists("static/alarm.flag"):
                            os.remove("static/alarm.flag")
                    except Exception as e:
                        logger.error(f"Error clearing alarm flag: {e}")

                # Handle ding flag
                current_time = time.time()
                for i, tripped in enumerate(zone_state):
                    if i >= len(zone_labels) or i >= len(zone_armed) or i >= len(zone_ding_unarmed):
                        continue

                    current_zone_label = zone_labels[i]
                    current_zone_is_armed_for_detection = zone_armed[i]
                    current_zone_dings_unarmed = (i in zone_ding_unarmed)

                    if tripped:
                        if not current_zone_is_armed_for_detection and current_zone_dings_unarmed:
                            if current_time - last_ding_time > DING_COOLDOWN:
                                ding_triggered = True
                                last_ding_time = current_time
                                try:
                                    with open("static/ding.flag", "w") as f: 
                                        f.write("1")
                                    logger.info(f"Ding flag set for unarmed zone '{current_zone_label}'.")
                                except IOError as e:
                                    logger.error(f"[DING FLAG ERROR] Could not write static/ding.flag: {e}")

                        if current_zone_is_armed_for_detection and armed and i not in alerted_zones:
                            logger.info(f"Zone '{current_zone_label}' tripped and armed.")
                            alerted_zones.add(i)
                    else:
                        if i in alerted_zones:
                            logger.info(f"Zone '{current_zone_label}' is no longer tripped. Removing from alerted_zones.")
                            alerted_zones.remove(i)

                # Clear ding flag when no zones are tripped
                if not any(zone_state) and ding_triggered:
                    ding_triggered = False
                    try:
                        if os.path.exists("static/ding.flag"):
                            os.remove("static/ding.flag")
                    except Exception as e:
                        logger.error(f"Error clearing ding flag: {e}")

            time.sleep(0.2)
        except Exception as e:
            logger.error(f"ERROR in alarm_watcher loop: {e}")
            time.sleep(1)

def schedule_runner():
    global armed
    last_checked_day = time.localtime().tm_mday

    while schedule_runner_active:
        now = time.localtime()
        current_time_str = time.strftime("%H:%M", now)
        current_day = now.tm_mday

        if current_day != last_checked_day:
            with lock:
                executed_schedules_today.clear()
                logger.info(f"[SCHEDULE] Cleared executed schedules for the new day (Day {current_day}).")
            last_checked_day = current_day

        with lock:
            for sched in schedules:
                if sched['time'] == current_time_str:
                    if sched['repeat'] or sched['id'] not in executed_schedules_today:
                        if sched['action'] == 'arm' and not armed:
                            armed = True
                            save_config()
                            logger.info(f"[SCHEDULE] System armed by schedule at {current_time_str}")
                        elif sched['action'] == 'disarm' and armed:
                            armed = False
                            save_config()
                            logger.info(f"[SCHEDULE] System disarmed by schedule at {current_time_str}")

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
    return render_template('index.html')

@app.route('/api/zone_status')
def zone_status():
    with lock:
        serializable_pin_layout = {int(k): v for k, v in pin_layout.items()}
        return jsonify({
            'armed': armed,
            'zones': zone_state,
            'zone_armed': zone_armed,
            'zone_labels': zone_labels,
            'pin_layout': serializable_pin_layout,
            'schedules': schedules,  # Added schedules to response
            'mode': mode,
            'home_mode_zones': home_mode_zones,
            'away_mode_zones': away_mode_zones,
            'zone_ding_unarmed': zone_ding_unarmed,
            'system_time': datetime.datetime.now().isoformat()
        })

@app.route('/api/arm', methods=['POST'])
def api_arm():
    global armed
    data = request.json or {}
    new_armed_state = data.get('armed')

    if not isinstance(new_armed_state, bool):
        return jsonify({'error': 'Invalid request. "armed" (boolean) is required.'}), 400

    with lock:
        armed = new_armed_state
        save_config()
        logger.info(f"System {'ARMED' if armed else 'DISARMED'} via API.")
    return ('', 204)

@app.route('/api/zone_arm', methods=['POST'])
def api_zone_arm():
    data = request.json or {}
    idx = data.get('index')
    status = data.get('armed')

    if not isinstance(idx, int) or not isinstance(status, bool):
        return jsonify({'error': 'Invalid request. "index" (int) and "armed" (boolean) are required.'}), 400

    with lock:
        if 0 <= idx < len(zone_armed):
            zone_armed[idx] = status
            logger.info(f"Zone {idx} ('{zone_labels[idx]}') set to {'ARMED' if status else 'DISARMED'} for detection.")
            return ('', 204)
        else:
            return jsonify({'error': f'Zone index {idx} out of bounds (0-{len(zone_armed)-1}).'}), 400

@app.route('/api/set_labels', methods=['POST'])
def api_set_labels():
    data = request.get_json() or {}
    labs = data.get('labels', [])

    if not isinstance(labs, list):
        return jsonify({'error': 'Invalid labels format (expected list of strings).'}), 400

    with lock:
        current_num_zones = len(zone_labels)
        new_num_zones = len(labs)

        zone_labels[:] = labs

        # Only reconfigure GPIO if number of zones changes
        if new_num_zones != current_num_zones:
            if new_num_zones > current_num_zones:
                extension_length = new_num_zones - current_num_zones
                zone_state.extend([False] * extension_length)
                zone_armed.extend([True] * extension_length)
                for i in range(current_num_zones, new_num_zones):
                    if i < len(DEFAULT_ZONE_PINS):
                        pin_layout[str(i)] = DEFAULT_ZONE_PINS[i]
                    else:
                        pin_layout[str(i)] = 0
                home_mode_zones.extend(range(current_num_zones, new_num_zones))
                away_mode_zones.extend(range(current_num_zones, new_num_zones))
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
        logger.info(f"Zone labels updated. Current zones: {len(zone_labels)}")
        return jsonify({
            'new_num_zones': new_num_zones,
            'pin_layout': pin_layout,
            'home_mode_zones': home_mode_zones,
            'away_mode_zones': away_mode_zones
        })

@app.route('/api/set_ding_zones', methods=['POST'])
def api_set_ding_zones():
    data = request.json or {}
    ding_zones = data.get("ding_zones", [])
    if not isinstance(ding_zones, list):
        return jsonify({'error': 'Invalid ding_zones format (expected list of integers)'}), 400

    with lock:
        valid_ding_zones = [idx for idx in ding_zones if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        zone_ding_unarmed[:] = valid_ding_zones
        save_config()
        logger.info(f"Ding zones updated: {zone_ding_unarmed}")
        return ('', 204)

@app.route('/api/set_mode', methods=['POST'])
def api_set_mode():
    global mode, zone_armed
    data = request.json or {}
    new_mode = data.get("mode")

    if new_mode not in ["home", "away"]:
        return jsonify({'error': 'Invalid mode. Must be "home" or "away"'}), 400

    with lock:
        mode = new_mode
        if mode == "home":
            zone_armed[:] = [True if i in home_mode_zones else False for i in range(len(zone_state))]
            logger.info(f"Mode set to HOME. Zones armed for detection: {[zone_labels[i] for i in home_mode_zones if i < len(zone_labels)]}")
        else:
            zone_armed[:] = [True if i in away_mode_zones else False for i in range(len(zone_state))]
            logger.info(f"Mode set to AWAY. Zones armed for detection: {[zone_labels[i] for i in away_mode_zones if i < len(zone_labels)]}")
        save_config()
        return ('', 204)

@app.route('/api/set_mode_zones', methods=['POST'])
def api_set_mode_zones():
    data = request.json or {}
    home_config = data.get("home", [])
    away_config = data.get("away", [])

    if not isinstance(home_config, list) or not isinstance(away_config, list):
        return jsonify({'error': 'Invalid mode zone configuration format (expected lists of integers)'}), 400

    with lock:
        home_mode_zones[:] = [idx for idx in home_config if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        away_mode_zones[:] = [idx for idx in away_config if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        save_config()
        logger.info(f"Home mode zones config: {home_mode_zones}")
        logger.info(f"Away mode zones config: {away_mode_zones}")
        return ('', 204)

@app.route('/api/clear_ding')
def clear_ding():
    global ding_triggered
    ding_triggered = False
    try:
        if os.path.exists("static/ding.flag"):
            os.remove("static/ding.flag")
        logger.info("Ding flag cleared.")
    except Exception as e:
        logger.error(f"Error clearing ding flag: {e}")
    return ('', 204)

@app.route('/api/clear_alarm')
def clear_alarm():
    global alarm_triggered
    alarm_triggered = False
    try:
        if os.path.exists("static/alarm.flag"):
            os.remove("static/alarm.flag")
        logger.info("Alarm flag cleared.")
    except Exception as e:
        logger.error(f"Error clearing alarm flag: {e}")
    return ('', 204)

@app.route('/api/schedules')
def api_schedules():
    with lock:
        return jsonify(schedules)

@app.route('/api/add_schedule', methods=['POST'])
def api_add_schedule():
    global schedule_id_counter
    data = request.json or {}
    action = data.get('action')
    time_str = data.get('time')
    repeat = data.get('repeat', False)

    if action not in ['arm', 'disarm'] or not time_str or not isinstance(time_str, str) or not isinstance(repeat, bool):
        return jsonify({'error': 'Invalid schedule data. Requires "action" (arm/disarm), "time" (HH:MM), and "repeat" (boolean).'}), 400

    with lock:
        new_schedule = {
            'id': schedule_id_counter,
            'action': action,
            'time': time_str,
            'repeat': repeat
        }
        schedules.append(new_schedule)
        schedule_id_counter += 1
        save_config()
        logger.info(f"Schedule added: {new_schedule}")
        return jsonify(new_schedule), 201

@app.route('/api/delete_schedule', methods=['POST'])
def api_delete_schedule():
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
            logger.info(f"Schedule {sched_id} deleted.")
            return ('', 204)
        else:
            return jsonify({'error': f'Schedule with ID {sched_id} not found.'}), 404

@app.route('/api/set_pin_layout', methods=['POST'])
def api_set_pin_layout():
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
                logger.warning(f"Attempted to set pin for non-existent zone index {zone_idx}. Skipping.")
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
        logger.info(f"Pin layout updated: {pin_layout}")
        return ('', 204)

@app.route('/api/system_info')
def system_info():
    return jsonify({
        'system_time': datetime.datetime.now().isoformat(),
        'zones_count': len(zone_labels),
        'version': '1.3.1'
    })

# --- Cleanup on Exit ---
def cleanup():
    global schedule_runner_active
    schedule_runner_active = False
    time.sleep(0.5)
    with lock:
        free_pins(current_zone_pins)
        free_pins(RELAY_PINS)
        GPIO.cleanup()
    logger.info("Application exiting. GPIO cleanup complete.")

atexit.register(cleanup) 

if __name__ == '__main__':
    if not os.path.exists('static'):
        os.makedirs('static')

    # Create flag files if they don't exist
    for flag_file in ['ding.flag', 'alarm.flag']:
        if not os.path.exists(f'static/{flag_file}'):
            with open(f'static/{flag_file}', 'w') as f:
                f.write('0')

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)