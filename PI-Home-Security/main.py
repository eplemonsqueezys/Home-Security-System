from flask import Flask, render_template, jsonify, request
import threading
import time
import json
import os
import atexit
import smtplib
from email.mime.text import MIMEText
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
            # Simulate a normally closed circuit (HIGH for open/tripped)
            # You can customize this for testing tripped states if needed
            # For demonstration, let's randomly trip a zone occasionally
            # if random.random() < 0.05:
            #     print(f"Mock GPIO: Pin {pin} returning HIGH (tripped)")
            #     return self.HIGH
            # else:
            #     return self.LOW
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
alerted_zones = set() # Zones that have already triggered an SMS alert in the current alarm state
zone_ding_unarmed = [] # Zones that "ding" (make a sound) when tripped while the system is unarmed
home_mode_zones = [] # Zones configured to be armed when system is in 'home' mode
away_mode_zones = [] # Zones configured to be armed when system is in 'away' mode
mode = "away" # Current system mode ("home" or "away")
sms_numbers = [] # List of phone numbers (email-to-SMS gateways) for alerts

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
            # This can happen if the pin was not set up as input/output
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
        # When setting up new pins, default all zones to armed for detection
        # This default can be overridden by mode setting upon config load
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
                # Mark zone as problematic if setup fails
                # (Optional: consider how to represent this in zone_state/UI)

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
                "sms_numbers": sms_numbers,
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
           away_mode_zones, sms_numbers, schedules, schedule_id_counter, mode, armed

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
        sms_numbers[:] = []
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
            # Ensure zone_labels match the number of pins if config is old/inconsistent
            # Or assume zone_labels dictates the number of zones
            loaded_zone_labels = data.get("zone_labels", default_zone_labels)
            zone_labels[:] = loaded_zone_labels
            num_loaded_zones = len(zone_labels)

            zone_ding_unarmed[:] = data.get("zone_ding_unarmed", [])
            home_mode_zones[:] = data.get("home_mode_zones", [])
            away_mode_zones[:] = data.get("away_mode_zones", [])
            sms_numbers[:] = data.get("sms_numbers", [])
            schedules[:] = data.get("schedules", [])
            mode = data.get("mode", "away")
            # armed state is not loaded from config as it's dynamic, it will be set by mode.

            # Ensure schedule_id_counter is higher than any existing schedule ID
            schedule_id_counter = max([s['id'] for s in schedules]) + 1 if schedules else 0

            # Reconstruct current_zone_pins based on loaded pin_layout and zone_labels
            pins_to_setup = []
            for i in range(num_loaded_zones):
                pin = pin_layout.get(str(i))
                if pin is not None:
                    pins_to_setup.append(int(pin))
                else:
                    # If a pin is not defined in pin_layout for an existing zone, use default or placeholder (0)
                    print(f"Warning: No pin defined in config for zone {i} ('{zone_labels[i]}'). Using default if available or 0.")
                    if i < len(DEFAULT_ZONE_PINS):
                        pins_to_setup.append(DEFAULT_ZONE_PINS[i])
                        pin_layout[str(i)] = DEFAULT_ZONE_PINS[i] # Update pin_layout with default
                    else:
                        pins_to_setup.append(0) # Assign 0 as placeholder for unassigned pin
                        pin_layout[str(i)] = 0 # Update pin_layout with 0

            setup_zone_pins(pins_to_setup)
            # When loading config, apply the loaded mode's arming state to zone_armed
            with lock: # Ensure zone_armed update is thread-safe
                if mode == "home":
                    zone_armed[:] = [True if i in home_mode_zones else False for i in range(len(zone_state))]
                else: # mode == "away"
                    zone_armed[:] = [True if i in away_mode_zones else False for i in range(len(zone_state))]
                # Set global 'armed' state based on mode
                # For simplicity, we assume if a mode is set, the system is 'ready to arm' or 'armed'
                # If you want to persist global 'armed' state, you'd save/load it.
                # For now, let's assume setting a mode implies the system is generally considered "armed" if any zones in that mode are armed.
                # Or, more simply, just keep it consistent with the "current" mode's effect.
                # Let's set the global 'armed' to false by default on boot unless explicitly armed by schedule or API
                armed = False # System starts disarmed and can be armed by UI/schedule

            print("[CONFIG] Configuration loaded successfully.")
    except Exception as e:
        print(f"[CONFIG] Load error: {e}. Reverting to default configuration.")
        # If loading fails, re-initialize defaults to ensure a runnable state
        pin_layout.clear() # Clear existing
        for i in range(default_num_zones):
            pin_layout[str(i)] = DEFAULT_ZONE_PINS[i]
        zone_labels[:] = default_zone_labels
        zone_ding_unarmed[:] = []
        home_mode_zones[:] = default_home_mode_zones
        away_mode_zones[:] = default_away_mode_zones
        sms_numbers[:] = []
        schedules[:] = []
        schedule_id_counter = 0
        mode = "away"
        armed = False
        pins = [pin_layout[str(i)] for i in range(len(zone_labels))]
        setup_zone_pins(pins)
        save_config() # Save the newly created default config


# --- SMS Alert Function ---
def send_sms_alert(zone_name):
    """Sends an SMS alert to configured numbers via a local SMTP server.
    Note: This requires a properly configured local SMTP server (e.g., Postfix)."""
    # Using 'localhost' and port 25 assumes a local SMTP server is running and configured
    # to relay messages to SMS gateways (e.g., number@carrier-gateway.com).
    # For common carriers, examples are:
    # AT&T: number@txt.att.net
    # Verizon: number@vtext.com
    # T-Mobile: number@tmomail.net
    # Sprint: number@messaging.sprintpcs.com
    # Google Fi: number@msg.fi.google.com
    # These often require sender email to be whitelisted by the carrier.
    sender_email = "your_email@example.com" # !!! IMPORTANT: Replace with a valid sender email address
    if sender_email == "your_email@example.com":
        print("[SMS ERROR] Sender email not configured. Please update main.py: sender_email.")
        return

    for number in sms_numbers:
        to_address = f"{number}"
        try:
            msg = MIMEText(f"Security Alert: Zone '{zone_name}' has been triggered!")
            msg['Subject'] = "Security System Alert"
            msg['From'] = sender_email
            msg['To'] = to_address

            # Connect to local SMTP server (often Postfix/Sendmail for Linux)
            with smtplib.SMTP('localhost', 25) as server:
                server.send_message(msg)
            print(f"[SMS] Alert sent to {to_address} for zone '{zone_name}'")
        except Exception as e:
            print(f"[SMS ERROR] Could not send alert to {to_address}: {e}")

# --- Initialize GPIO and Load Configuration on Startup ---
setup_relay_pins() # Set up relays first
load_config() # Load saved configuration or set up defaults

# --- Background Threads ---
def poll_zones():
    """Continuously polls the state of alarm zones via GPIO inputs."""
    while True:
        with lock:
            # Recheck length within loop in case zone config changes
            num_current_zones = len(current_zone_pins)
            for i in range(num_current_zones):
                p = current_zone_pins[i]
                if p == 0: # Skip unassigned pins
                    zone_state[i] = False # Ensure it's not marked as tripped
                    continue
                try:
                    val = GPIO.input(p)
                    # Assume HIGH means the circuit is open/tripped (e.g., door opened)
                    zone_state[i] = (val == GPIO.HIGH)
                except Exception as e:
                    print(f"Error reading GPIO pin {p} for zone {i}: {e}. Setting zone state to False.")
                    zone_state[i] = False # Default to not tripped on error
        time.sleep(0.1) # Poll every 100ms

def alarm_watcher():
    """Monitors zone states and triggers alarm, ding, and SMS alerts."""
    global alerted_zones # Declare global to modify the set

    # Ensure GPIO is set up (should ideally be done once at application start)
    try:
        # This setup should only run ONCE for the entire application.
        # If your app restarts this thread, ensure GPIO.cleanup() is called appropriately
        # or handle re-initialization. For a persistent daemon, this is fine.
        GPIO.setmode(GPIO.BCM) # Use Broadcom GPIO numbers
        for p in RELAY_PINS:
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW) # Initialize relays to OFF
        print("GPIO initialized for alarm watcher.")
    except Exception as e:
        print(f"CRITICAL ERROR: Could not initialize GPIO for alarm watcher: {e}")
        # Depending on criticality, you might want to exit or disable alarm functionality
        return # Exit the watcher if GPIO fails to initialize


    last_relay_state = None # To detect changes in relay state for logging

    while True:
        try:
            current_triggered_by_armed_zone = False
            current_relay_output_state = GPIO.LOW # Default to off

            with lock:
                # Determine if the main alarm should be triggered
                # Alarm triggers if the system is globally 'armed' AND any 'zone_armed' is tripped
                for i, tripped in enumerate(zone_state):
                    # Robustly check index bounds before accessing lists
                    if i < len(zone_armed) and zone_armed[i] and tripped:
                        current_triggered_by_armed_zone = True
                        break # Found at least one armed and tripped zone

                # Calculate the desired relay state
                current_relay_output_state = GPIO.HIGH if (armed and current_triggered_by_armed_zone) else GPIO.LOW

                # Control relays (e.g., siren)
                if current_relay_output_state != last_relay_state:
                    action = "ACTIVATING" if current_relay_output_state == GPIO.HIGH else "DEACTIVATING"
                    print(f"Alarm Relays: {action} (System Armed: {armed}, Armed Zone Tripped: {current_triggered_by_armed_zone})")
                    for p in RELAY_PINS:
                        try:
                            GPIO.output(p, current_relay_output_state)
                            print(f"  GPIO Pin {p} set to {current_relay_output_state}")
                        except Exception as e:
                            print(f"ERROR: Could not control relay pin {p}: {e}")
                    last_relay_state = current_relay_output_state # Update last state after attempting output


                # Handle ding and SMS alerts for individual zones
                for i, tripped in enumerate(zone_state):
                    # Ensure all related lists have this index for consistent behavior
                    if i >= len(zone_labels) or i >= len(zone_armed) or i >= len(zone_ding_unarmed):
                        # This should ideally not happen if data loading is consistent, but acts as a safeguard.
                        print(f"WARNING: Zone index {i} out of bounds for some configuration lists. Skipping alerts for this zone.")
                        continue # Skip to the next zone

                    current_zone_label = zone_labels[i]
                    current_zone_is_armed_for_detection = zone_armed[i]
                    current_zone_dings_unarmed = (i in zone_ding_unarmed)

                    if tripped:
                        # Ding sound for unarmed zones configured to ding
                        if not current_zone_is_armed_for_detection and current_zone_dings_unarmed:
                            try:
                                # Create a flag file for the frontend to play a ding sound
                                with open("static/ding.flag", "w") as f: f.write("1")
                                # Frontend will clear this flag after playing.
                                print(f"Ding flag set for unarmed zone '{current_zone_label}'.")
                            except IOError as e:
                                print(f"[DING FLAG ERROR] Could not write static/ding.flag: {e}")

                        # SMS alert for armed and tripped zones
                        # Only send if the zone is armed, the system is globally armed, AND it hasn't alerted yet
                        if current_zone_is_armed_for_detection and armed and i not in alerted_zones:
                            print(f"Zone '{current_zone_label}' tripped and armed. Sending SMS alert.")
                            send_sms_alert(current_zone_label)
                            alerted_zones.add(i) # Add to set to prevent duplicate alerts
                    else:
                        # If a zone is no longer tripped, remove it from the alerted_zones set
                        if i in alerted_zones:
                            print(f"Zone '{current_zone_label}' is no longer tripped. Removing from alerted_zones.")
                            alerted_zones.remove(i)

            time.sleep(0.2) # Check every 200ms
        except Exception as e:
            print(f"ERROR in alarm_watcher loop: {e}")
            # Consider a longer sleep or error handling strategy here to prevent rapid error looping
            time.sleep(1) # Sleep longer on error to prevent CPU hogging

def schedule_runner():
    """Periodically checks and executes scheduled arm/disarm events."""
    global armed
    # Track the day to reset executed_schedules_today
    last_checked_day = time.localtime().tm_mday

    while schedule_runner_active:
        now = time.localtime()
        current_time_str = time.strftime("%H:%M", now)
        current_day = now.tm_mday

        # Reset executed_schedules_today at the beginning of a new day
        if current_day != last_checked_day:
            with lock:
                executed_schedules_today.clear()
                print(f"[SCHEDULE] Cleared executed schedules for the new day (Day {current_day}).")
            last_checked_day = current_day

        with lock:
            for sched in schedules:
                # Check if the schedule matches current time
                if sched['time'] == current_time_str:
                    # If it's a repeating schedule OR it's a one-time schedule not yet executed today
                    if sched['repeat'] or sched['id'] not in executed_schedules_today:
                        if sched['action'] == 'arm' and not armed:
                            armed = True
                            print(f"[SCHEDULE] System armed by schedule at {current_time_str}")
                        elif sched['action'] == 'disarm' and armed:
                            armed = False
                            print(f"[SCHEDULE] System disarmed by schedule at {current_time_str}")

                        # If it's a one-time schedule, mark it as executed for today
                        if not sched['repeat']:
                            executed_schedules_today.add(sched['id'])
        time.sleep(30) # Check schedules every 30 seconds

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
        # Return pin_layout with integer keys for cleaner JS consumption
        serializable_pin_layout = {int(k): v for k, v in pin_layout.items()}
        return jsonify({
            'armed': armed, # Global alarm system armed state
            'zones': zone_state, # Current physical state (tripped/ok)
            'zone_armed': zone_armed, # Whether each zone is armed for detection
            'zone_labels': zone_labels, # User-defined labels
            'pin_layout': serializable_pin_layout, # GPIO pin assignments
            'mode': mode, # Current system mode ("home" or "away")
            'home_mode_zones': home_mode_zones, # Zones armed in home mode config
            'away_mode_zones': away_mode_zones, # Zones armed in away mode config
            'sms_numbers': sms_numbers, # Configured SMS alert numbers
            'zone_ding_unarmed': zone_ding_unarmed # Zones that ding when tripped and disarmed
        })

@app.route('/api/arm', methods=['POST'])
def api_arm():
    """API to arm or disarm the entire alarm system."""
    global armed
    data = request.json or {}
    # Expect 'armed' boolean in request body
    new_armed_state = data.get('armed')

    if not isinstance(new_armed_state, bool):
        return jsonify({'error': 'Invalid request. "armed" (boolean) is required.'}), 400

    with lock:
        armed = new_armed_state # Set global armed state
        print(f"System {'ARMED' if armed else 'DISARMED'} via API.")
    return ('', 204) # No Content, successful

@app.route('/api/zone_arm', methods=['POST'])
def api_zone_arm():
    """API to arm or disarm a specific zone for detection."""
    data = request.json or {}
    idx = data.get('index')
    status = data.get('armed') # True to arm, False to disarm

    if not isinstance(idx, int) or not isinstance(status, bool):
        return jsonify({'error': 'Invalid request. "index" (int) and "armed" (boolean) are required.'}), 400

    with lock:
        if 0 <= idx < len(zone_armed):
            zone_armed[idx] = status # Update individual zone arming status
            print(f"Zone {idx} ('{zone_labels[idx]}') set to {'ARMED' if status else 'DISARMED'} for detection.")
            # Note: This manual arming can be overridden if mode changes and recalculates zone_armed
            # The UI should ideally reflect whether this manual state is overridden by mode config.
            # For simplicity, this directly updates zone_armed.
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

        zone_labels[:] = labs # Update all labels

        if new_num_zones > current_num_zones:
            # New zones added, extend other relevant lists
            extension_length = new_num_zones - current_num_zones
            zone_state.extend([False] * extension_length) # New zones initially not tripped
            zone_armed.extend([True] * extension_length) # New zones default to armed for detection
            # Initialize new pin_layout entries and add to home/away modes
            for i in range(current_num_zones, new_num_zones):
                if i < len(DEFAULT_ZONE_PINS):
                    pin_layout[str(i)] = DEFAULT_ZONE_PINS[i]
                else:
                    pin_layout[str(i)] = 0 # Placeholder for unassigned pin
                home_mode_zones.append(i) # New zones armed by default in home mode
                away_mode_zones.append(i) # New zones armed by default in away mode
        elif new_num_zones < current_num_zones:
            # Zones removed, truncate lists
            zone_state[:] = zone_state[:new_num_zones]
            zone_armed[:] = zone_armed[:new_num_zones]
            zone_ding_unarmed[:] = [idx for idx in zone_ding_unarmed if idx < new_num_zones]
            home_mode_zones[:] = [idx for idx in home_mode_zones if idx < new_num_zones]
            away_mode_zones[:] = [idx for idx in away_mode_zones if idx < new_num_zones]

            # Rebuild pin_layout to only include relevant zones
            new_pin_layout_dict = {}
            for i in range(new_num_zones):
                if str(i) in pin_layout:
                    new_pin_layout_dict[str(i)] = pin_layout[str(i)]
                elif i < len(DEFAULT_ZONE_PINS): # Fallback to default if somehow missing
                    new_pin_layout_dict[str(i)] = DEFAULT_ZONE_PINS[i]
                else:
                    new_pin_layout_dict[str(i)] = 0 # Placeholder
            pin_layout.clear()
            pin_layout.update(new_pin_layout_dict)

        # Re-setup pins based on the potentially changed number of zones and their pins
        current_pins_for_setup = []
        for i in range(new_num_zones):
            pin = pin_layout.get(str(i))
            if pin is not None:
                current_pins_for_setup.append(int(pin))
            else:
                # Should not happen if pin_layout was updated, but for robustness:
                if i < len(DEFAULT_ZONE_PINS):
                    current_pins_for_setup.append(DEFAULT_ZONE_PINS[i])
                else:
                    current_pins_for_setup.append(0) # Assign 0 or handle as unassigned

        setup_zone_pins(current_pins_for_setup)
        save_config()
        print(f"Zone labels and system size updated. Current zones: {len(zone_labels)}")
        return ('', 204)

@app.route('/api/set_ding_zones', methods=['POST'])
def api_set_ding_zones():
    """API to set which zones 'ding' when tripped while the system is unarmed."""
    data = request.json or {}
    ding_zones = data.get("ding_zones", []) # Expects a list of zone indices
    if not isinstance(ding_zones, list):
        return jsonify({'error': 'Invalid ding_zones format (expected list of integers)'}), 400

    with lock:
        # Validate indices to ensure they refer to existing zones
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
        # Update individual zone_armed status based on the selected mode's configuration
        if mode == "home":
            zone_armed[:] = [True if i in home_mode_zones else False for i in range(len(zone_state))]
            print(f"Mode set to HOME. Zones armed for detection: {[zone_labels[i] for i in home_mode_zones if i < len(zone_labels)]}")
        else: # mode == "away"
            zone_armed[:] = [True if i in away_mode_zones else False for i in range(len(zone_state))]
            print(f"Mode set to AWAY. Zones armed for detection: {[zone_labels[i] for i in away_mode_zones if i < len(zone_labels)]}")
        save_config() # Save configuration to persist the current mode
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
        # Validate indices and update
        home_mode_zones[:] = [idx for idx in home_config if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        away_mode_zones[:] = [idx for idx in away_config if isinstance(idx, int) and 0 <= idx < len(zone_labels)]
        save_config()
        print(f"Home mode zones config: {home_mode_zones}")
        print(f"Away mode zones config: {away_mode_zones}")
    return ('', 204)

@app.route('/api/set_sms', methods=['POST'])
def api_set_sms():
    """API to set the list of SMS recipient numbers."""
    data = request.json or {}
    sms_list = data.get("sms", [])
    if not isinstance(sms_list, list):
        return jsonify({'error': 'Invalid SMS numbers format (expected list of strings)'}), 400

    with lock:
        # Basic validation for email-like format, or just accept strings
        valid_sms_numbers = [str(num).strip() for num in sms_list if isinstance(num, str) and '@' in str(num)]
        sms_numbers[:] = valid_sms_numbers # Update SMS numbers
        save_config()
        print(f"SMS numbers updated: {sms_numbers}")
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
    zone = data.get('zone') # Zone is mostly for context, actual arm/disarm is global
    action = data.get('action') # 'arm' or 'disarm'
    time_str = data.get('time') # HH:MM format
    repeat = data.get('repeat', False) # True for repeating daily, False for one-time

    # Basic validation
    if action not in ['arm', 'disarm'] or not time_str or not isinstance(time_str, str) or not isinstance(repeat, bool):
        return jsonify({'error': 'Invalid schedule data. Requires "action" (arm/disarm), "time" (HH:MM), and "repeat" (boolean).'}), 400
    if not (isinstance(zone, int) and 0 <= zone < len(zone_labels) or zone is None): # Allow None for global context
         return jsonify({'error': 'Invalid zone index provided for schedule context.'}), 400


    with lock:
        new_schedule = {
            'id': schedule_id_counter,
            'zone': zone, # Store zone even if not directly used for global arm/disarm
            'action': action,
            'time': time_str,
            'repeat': repeat
        }
        schedules.append(new_schedule)
        schedule_id_counter += 1
        save_config()
        print(f"Schedule added: {new_schedule}")
    return jsonify(new_schedule), 201 # Return the newly created schedule with its ID

@app.route('/api/delete_schedule', methods=['POST'])
def api_delete_schedule():
    """API to delete a schedule by its ID."""
    data = request.json or {}
    sched_id = data.get('id')

    if not isinstance(sched_id, int):
        return jsonify({'error': 'Schedule ID (integer) is required.'}), 400

    with lock:
        global schedules
        # Filter out the schedule with the given ID
        initial_len = len(schedules)
        schedules = [s for s in schedules if s['id'] != sched_id]
        if len(schedules) < initial_len:
            save_config()
            print(f"Schedule {sched_id} deleted.")
            return ('', 204)
        else:
            return jsonify({'error': f'Schedule with ID {sched_id} not found.'}), 404

@app.route('/api/set_pin_layout', methods=['POST']) # Corrected endpoint name
def api_set_pin_layout():
    """API to update the GPIO pin assignments for zones."""
    data = request.json or {} # Expected format: {"0": 4, "1": 17, ...}
    
    if not isinstance(data, dict):
        return jsonify({'error': 'Invalid pin layout format (expected dictionary of zone_idx: pin_number).'}), 400

    new_pin_layout_received = {}
    for k, v in data.items():
        try:
            zone_idx = int(k)
            pin_num = int(v)
            if 0 <= zone_idx < len(zone_labels): # Only update for existing zones
                new_pin_layout_received[str(zone_idx)] = pin_num
            else:
                print(f"Warning: Attempted to set pin for non-existent zone index {zone_idx}. Skipping.")
        except ValueError:
            return jsonify({'error': f'Invalid pin data for key "{k}": value "{v}". Expected integers.'}), 400

    with lock:
        # Update only the pins that were provided in the request
        pin_layout.update(new_pin_layout_received) # Update existing entries and add new ones

        # Re-setup GPIO pins based on the updated pin_layout for all current zones
        current_pins_for_setup = []
        for i in range(len(zone_labels)):
            pin = pin_layout.get(str(i))
            if pin is not None:
                current_pins_for_setup.append(int(pin))
            else:
                # If a pin is not defined in pin_layout for an existing zone, use default or placeholder
                print(f"Warning: Pin not defined in current layout for zone {i} ('{zone_labels[i]}'). Using default if available or 0.")
                if i < len(DEFAULT_ZONE_PINS):
                    current_pins_for_setup.append(DEFAULT_ZONE_PINS[i])
                    pin_layout[str(i)] = DEFAULT_ZONE_PINS[i] # Update pin_layout for consistency
                else:
                    current_pins_for_setup.append(0) # Assign 0 or handle as unassigned
                    pin_layout[str(i)] = 0 # Update pin_layout for consistency


        setup_zone_pins(current_pins_for_setup)
        save_config()
        print(f"Pin layout updated: {pin_layout}")
    return ('', 204)

# --- Cleanup on Exit ---
def cleanup():
    """Cleans up all GPIO pins when the application exits."""
    global schedule_runner_active
    schedule_runner_active = False # Stop the schedule runner thread
    time.sleep(0.5) # Give threads a moment to finish
    free_pins(current_zone_pins)
    free_pins(RELAY_PINS)
    GPIO.cleanup()
    print("Application exiting. GPIO cleanup complete.")

atexit.register(cleanup) # Register cleanup function to run on exit

if __name__ == '__main__':
    # Create the static directory if it doesn't exist for ding.flag
    if not os.path.exists('static'):
        os.makedirs('static')
    # Create a dummy ding.flag to ensure it exists if app starts and nothing creates it
    if not os.path.exists('static/ding.flag'):
        with open('static/ding.flag', 'w') as f:
            f.write('0') # Default to not dinging (0)

    # Flask app will run on all available network interfaces on port 5000
    app.run(host='0.0.0.0', port=5000, debug=False) # Set debug=True for development