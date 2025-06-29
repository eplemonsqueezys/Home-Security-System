from flask import Flask, render_template, jsonify, request
import threading
import time
import json
import os
import atexit

# --- Debug Print Control ---
debug_prints = True  # Set False to suppress debug/info logs, show errors only

def log(msg, error=False):
    if debug_prints or error:
        print(msg)

# --- Mock RPi.GPIO for non-Raspberry Pi environments ---
try:
    import RPi.GPIO as GPIO
    log("RPi.GPIO imported successfully. Running on Raspberry Pi.")
except ImportError:
    class MockGPIO:
        BCM = 11
        IN = 1
        OUT = 0
        HIGH = 1
        LOW = 0
        PUD_UP = 20
        def setmode(self, mode): log("Mock GPIO: setmode called")
        def setwarnings(self, state): log("Mock GPIO: setwarnings called")
        def setup(self, pin, mode, pull_up_down=None):
            log(f"Mock GPIO: Setup pin {pin} as {'IN' if mode == self.IN else 'OUT'}")
        def input(self, pin): return self.LOW
        def output(self, pin, value):
            state = "HIGH" if value == self.HIGH else "LOW"
            log(f"Mock GPIO: Pin {pin} set to {state}")
        def cleanup(self, pin=None):
            log(f"Mock GPIO: Cleaning up pin {pin if pin else 'all'}")
    GPIO = MockGPIO()
    log("RPi.GPIO not found, using mock GPIO. Physical GPIO operations will be simulated.", error=True)

app = Flask(__name__)

CONFIG_FILE = "config.json"
DEFAULT_ZONE_PINS = [4, 17, 18, 27, 22, 23, 24, 25]
RELAY_PINS = [5, 6, 13, 19, 26, 20, 21, 16]

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
lock = threading.Lock()
executed_schedules_today = set()
schedule_runner_active = True

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def free_pins(pins):
    for pin in pins:
        try:
            GPIO.cleanup(pin)
            log(f"GPIO: Cleaned up pin {pin}")
        except Exception as e:
            log(f"GPIO: Error cleaning up pin {pin}: {e}", error=True)

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
                log("GPIO: Skipping pin setup for unassigned pin (0).")
                continue
            try:
                GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                log(f"GPIO: Setup input pin {p} with PUD_UP")
            except Exception as e:
                log(f"[GPIO INPUT ERROR] Pin {p}: {e}", error=True)

def setup_relay_pins():
    with lock:
        free_pins(RELAY_PINS)
        for pin in RELAY_PINS:
            try:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                log(f"GPIO: Setup output pin {pin} to LOW")
            except Exception as e:
                log(f"[GPIO OUTPUT ERROR] Relay pin {pin}: {e}", error=True)

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
                "mode": mode
            }, f, indent=4)
        log("[CONFIG] Configuration saved successfully.")
    except Exception as e:
        log(f"[CONFIG] Save error: {e}", error=True)

def load_config():
    global pin_layout, zone_labels, zone_ding_unarmed, home_mode_zones, away_mode_zones, schedules, schedule_id_counter, mode, armed

    default_num_zones = len(DEFAULT_ZONE_PINS)
    default_zone_labels = [f"Zone {i+1}" for i in range(default_num_zones)]
    default_pin_layout = {str(i): DEFAULT_ZONE_PINS[i] for i in range(default_num_zones)}
    default_home_mode_zones = list(range(default_num_zones))
    default_away_mode_zones = list(range(default_num_zones))

    if not os.path.exists(CONFIG_FILE):
        log(f"[CONFIG] {CONFIG_FILE} not found. Creating default configuration.")
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
            zone_labels[:] = data.get("zone_labels", default_zone_labels)
            zone_ding_unarmed[:] = data.get("zone_ding_unarmed", [])
            home_mode_zones[:] = data.get("home_mode_zones", [])
            away_mode_zones[:] = data.get("away_mode_zones", [])
            schedules[:] = data.get("schedules", [])
            mode = data.get("mode", "away")
            schedule_id_counter = max([s['id'] for s in schedules]) + 1 if schedules else 0

            pins_to_setup = []
            for i in range(len(zone_labels)):
                pin = pin_layout.get(str(i))
                if pin is not None:
                    pins_to_setup.append(int(pin))
                else:
                    pins_to_setup.append(DEFAULT_ZONE_PINS[i] if i < len(DEFAULT_ZONE_PINS) else 0)
            setup_zone_pins(pins_to_setup)

            with lock:
                zone_armed[:] = [i in (home_mode_zones if mode == "home" else away_mode_zones) for i in range(len(zone_state))]
                armed = False

        log("[CONFIG] Configuration loaded successfully.")
    except Exception as e:
        log(f"[CONFIG] Load error: {e}. Reverting to default configuration.", error=True)
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
        armed = True
        pins = [pin_layout[str(i)] for i in range(len(zone_labels))]
        setup_zone_pins(pins)
        save_config()

setup_relay_pins()
load_config()

# Background thread to poll GPIO zones
def poll_zones():
    while True:
        with lock:
            for i, p in enumerate(current_zone_pins):
                if p == 0:
                    zone_state[i] = False
                    continue
                try:
                    val = GPIO.input(p)
                    zone_state[i] = (val == GPIO.HIGH)
                except Exception as e:
                    log(f"Error reading GPIO pin {p} for zone {i}: {e}. Setting zone state to False.", error=True)
                    zone_state[i] = False
        time.sleep(0.1)

# Background thread to watch for alarms
def alarm_watcher():
    global alerted_zones
    last_relay_state = None

    while True:
        try:
            triggered = any(tripped and zone_armed[i] for i, tripped in enumerate(zone_state))
            new_state = GPIO.HIGH if armed and triggered else GPIO.LOW

            if new_state != last_relay_state:
                log(f"Alarm Relays: {'ACTIVATING' if new_state == GPIO.HIGH else 'DEACTIVATING'} (Armed: {armed}, Triggered: {triggered})")
                for p in RELAY_PINS:
                    try:
                        GPIO.output(p, new_state)
                        log(f"  GPIO Pin {p} set to {new_state}")
                    except Exception as e:
                        log(f"ERROR: Could not control relay pin {p}: {e}", error=True)
                last_relay_state = new_state

            try:
                if armed and triggered:
                    with open("static/alarm.flag", "w") as f:
                        f.write("1")
                    log("Alarm flag set.")
                elif os.path.exists("static/alarm.flag"):
                    os.remove("static/alarm.flag")
            except IOError as e:
                log(f"[ALARM FLAG ERROR] {e}", error=True)

            for i, tripped in enumerate(zone_state):
                if i >= len(zone_labels) or i >= len(zone_armed):
                    continue
                label = zone_labels[i]

                if tripped:
                    if not zone_armed[i] and i in zone_ding_unarmed:
                        try:
                            with open("static/ding.flag", "w") as f:
                                f.write("1")
                            log(f"Ding flag set for unarmed zone '{label}'.")
                        except IOError as e:
                            log(f"[DING FLAG ERROR] {e}", error=True)
                    if zone_armed[i] and armed and i not in alerted_zones:
                        log(f"Zone '{label}' tripped and armed.")
                        alerted_zones.add(i)
                else:
                    if i in alerted_zones:
                        log(f"Zone '{label}' no longer tripped. Removing from alerted.")
                        alerted_zones.remove(i)

            time.sleep(0.2)
        except Exception as e:
            log(f"ERROR in alarm_watcher loop: {e}", error=True)
            time.sleep(1)

# Background thread to handle schedules
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
                log(f"[SCHEDULE] New day {current_day}. Cleared executed schedules.")
            last_checked_day = current_day

        with lock:
            for sched in schedules:
                if sched['time'] == current_time_str:
                    if sched['repeat'] or sched['id'] not in executed_schedules_today:
                        if sched['action'] == 'arm' and not armed:
                            armed = True
                            log(f"[SCHEDULE] System armed by schedule at {current_time_str}")
                        elif sched['action'] == 'disarm' and armed:
                            armed = False
                            log(f"[SCHEDULE] System disarmed by schedule at {current_time_str}")
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
            'mode': mode,
            'home_mode_zones': home_mode_zones,
            'away_mode_zones': away_mode_zones,
            'zone_ding_unarmed': zone_ding_unarmed
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
        log(f"System {'ARMED' if armed else 'DISARMED'} via API.")
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
            log(f"Zone {idx} ('{zone_labels[idx]}') set to {'ARMED' if status else 'DISARMED'} for detection.")
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

        pins_to_setup = [pin_layout.get(str(i), 0) for i in range(new_num_zones)]
        setup_zone_pins(pins_to_setup)
        save_config()

        log(f"Zone labels updated: {zone_labels}")
    return ('', 204)

@app.route('/api/set_pin_layout', methods=['POST'])
def api_set_pin_layout():
    data = request.get_json() or {}
    layout = data.get('pin_layout', {})

    if not isinstance(layout, dict):
        return jsonify({'error': 'Invalid pin_layout format (expected dict).'}), 400

    with lock:
        pin_layout.clear()
        for k, v in layout.items():
            try:
                pin_layout[str(int(k))] = int(v)
            except Exception:
                return jsonify({'error': 'Pin layout keys and values must be integers.'}), 400
        pins_to_setup = [pin_layout.get(str(i), 0) for i in range(len(zone_labels))]
        setup_zone_pins(pins_to_setup)
        save_config()
        log(f"Pin layout updated: {pin_layout}")
    return ('', 204)

@app.route('/api/set_ding_unarmed', methods=['POST'])
def api_set_ding_unarmed():
    data = request.get_json() or {}
    zones = data.get('zones', [])

    if not isinstance(zones, list) or not all(isinstance(i, int) for i in zones):
        return jsonify({'error': 'Invalid zones format (expected list of integers).'}), 400

    with lock:
        zone_ding_unarmed[:] = zones
        save_config()
        log(f"Zones with ding on unarmed trigger updated: {zone_ding_unarmed}")
    return ('', 204)

@app.route('/api/set_mode', methods=['POST'])
def api_set_mode():
    global mode
    data = request.get_json() or {}
    new_mode = data.get('mode')

    if new_mode not in ('home', 'away'):
        return jsonify({'error': 'Invalid mode, must be "home" or "away".'}), 400

    with lock:
        mode = new_mode
        armed_zones = home_mode_zones if mode == "home" else away_mode_zones
        for i in range(len(zone_armed)):
            zone_armed[i] = (i in armed_zones)
        save_config()
        log(f"Mode set to {mode}. Zones armed accordingly.")
    return ('', 204)

@app.route('/api/schedules', methods=['GET', 'POST', 'DELETE'])
def api_schedules():
    global schedule_id_counter
    if request.method == 'GET':
        with lock:
            return jsonify(schedules)
    elif request.method == 'POST':
        data = request.get_json() or {}
        time_str = data.get('time')
        action = data.get('action')
        repeat = data.get('repeat', False)

        if not time_str or action not in ('arm', 'disarm'):
            return jsonify({'error': 'Missing or invalid time/action.'}), 400
        try:
            time.strptime(time_str, '%H:%M')
        except ValueError:
            return jsonify({'error': 'Invalid time format, expected HH:MM.'}), 400

        with lock:
            new_sched = {
                'id': schedule_id_counter,
                'time': time_str,
                'action': action,
                'repeat': repeat
            }
            schedules.append(new_sched)
            schedule_id_counter += 1
            save_config()
            log(f"Schedule added: {new_sched}")
            return jsonify(new_sched)
    elif request.method == 'DELETE':
        data = request.get_json() or {}
        sched_id = data.get('id')
        if sched_id is None:
            return jsonify({'error': 'Schedule id required.'}), 400

        with lock:
            schedules[:] = [s for s in schedules if s['id'] != sched_id]
            save_config()
            log(f"Schedule with id {sched_id} deleted.")
            return ('', 204)

@app.route('/api/trigger_ding', methods=['POST'])
def api_trigger_ding():
    try:
        with open("static/ding.flag", "w") as f:
            f.write("1")
        log("Ding flag set via API.")
        return ('', 204)
    except IOError as e:
        log(f"[DING FLAG ERROR] {e}", error=True)
        return jsonify({'error': 'Failed to set ding flag.'}), 500

# Cleanup on exit
def cleanup():
    global schedule_runner_active
    schedule_runner_active = False
    GPIO.cleanup()
    log("GPIO cleaned up on exit.")

atexit.register(cleanup)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
