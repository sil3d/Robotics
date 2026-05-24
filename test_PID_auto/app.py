import json
import os
import time
import threading
import logging
import numpy as np
from collections import deque
from flask import Flask, render_template
from flask_socketio import SocketIO
# Fix: websocket-client (not websockets) provides WebSocketApp
try:
    import websocket as wsclient
except ImportError as exc:
    print("[ERROR] websocket-client is not installed.")
    print("  Run: pip uninstall websockets -y && pip install websocket-client")
    raise exc

WebSocketApp = wsclient.WebSocketApp
create_connection = wsclient.create_connection

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False
from industrial_ai import UnifiedRLAgent

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger()

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

ESP32_IP    = "192.168.4.1"
WS_URL      = f"ws://{ESP32_IP}/ws"
SERIAL_BAUD = 115200

# ── Config globale partagée ───────────────────────────────
_CFG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "data", "robot_config.json"
)

def _load_global_config():
    """Charge robot_config.json. Retourne {} si absent ou invalide."""
    try:
        with open(_CFG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def _save_global_config(cfg: dict):
    """Écrit robot_config.json de façon atomique."""
    try:
        tmp = _CFG_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        os.replace(tmp, _CFG_PATH)
    except Exception as e:
        logger.warning("[CFG] Sauvegarde globale échouée: %s", e)

robot_ws         = None
serial_conn      = None
connection_mode  = 'wifi'

_ws_lock      = threading.Lock()
_serial_lock  = threading.Lock()
_history_lock = threading.Lock()

# ── État ──────────────────────────────────────────────────
auto_pilot_active = False  # IA DÉSACTIVÉE par défaut

_gcfg = _load_global_config()
yaw_kp       = float(_gcfg.get("pid",    {}).get("kp",  4.0))
yaw_ki       = float(_gcfg.get("pid",    {}).get("ki",  0.02))
yaw_kd       = float(_gcfg.get("pid",    {}).get("kd",  0.7))
motor_a_trim  = float(_gcfg.get("trims",  {}).get("a",   0.0))
motor_b_trim  = float(_gcfg.get("trims",  {}).get("b",   0.0))
motor_a_minpwm= float(_gcfg.get("minpwm", {}).get("a",  55.0))
motor_b_minpwm= float(_gcfg.get("minpwm", {}).get("b",  55.0))

current_dir_x = 0.0
current_dir_y = 0.0
locked_yaw = 0.0
is_yaw_locked = False

yaw_history = deque(maxlen=100)
telemetry_history = deque(maxlen=50)

# ── IA ────────────────────────────────────────────────────
ai_agent = UnifiedRLAgent()

# ── Serial / USB ──────────────────────────────────────────
def list_serial_ports():
    if serial is None:
        return []
    return [{'device': p.device, 'desc': p.description}
            for p in serial.tools.list_ports.comports()]

def connect_serial(port=None):
    global serial_conn, connection_mode
    if serial is None:
        logger.warning("[USB] pyserial not installed")
        return False
    if not port:
        ports = serial.tools.list_ports.comports()
        if len(ports) == 1:
            port = ports[0].device
        elif len(ports) == 0:
            logger.warning("[USB] No serial port found")
            return False
        else:
            logger.warning("[USB] Multiple ports found")
            return False
    try:
        with _serial_lock:
            if serial_conn and serial_conn.is_open:
                serial_conn.close()
            serial_conn = serial.Serial(port, SERIAL_BAUD, timeout=1)
        connection_mode = 'usb'
        logger.info(f"[USB] Connected on {port}")
        socketio.emit('connection_status', {'mode': 'usb', 'port': port})
        return True
    except Exception as e:
        logger.warning(f"[USB] Failed: {e}")
        return False

def switch_to_wifi():
    global connection_mode, serial_conn
    with _serial_lock:
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.close()
            except Exception:
                pass
        serial_conn = None
    connection_mode = 'wifi'
    logger.info("[WIFI] Switched to WiFi")
    socketio.emit('connection_status', {'mode': 'wifi'})

# ── Telemetry ────────────────────────────────────────────
def _push_telemetry(data: dict):
    with _history_lock:
        yaw_history.append(float(data.get('y', 0.0)))
        telemetry_history.append(data)
        global is_yaw_locked, locked_yaw
        is_yaw_locked = int(data.get('lk', 0)) == 1
        if is_yaw_locked and len(yaw_history) >= 2:
            locked_yaw = yaw_history[-2]  # Approximation
    socketio.emit('telemetry', data)

# ── WiFi listener ─────────────────────────────────────────
def on_message(ws, message):
    if connection_mode != 'wifi':
        return
    try:
        # websocket-client returns bytes for raw frames; decode UTF-8
        if isinstance(message, bytes):
            message = message.decode('utf-8')
        data = json.loads(message)
        _push_telemetry(data)
    except Exception as e:
        logger.warning("WiFi msg error: %s", e)

def esp32_listener():
    global robot_ws
    while True:
        try:
            ws = WebSocketApp(WS_URL, on_message=on_message,
                on_error=lambda _ws, err: logger.warning("WS: %s", err))
            with _ws_lock:
                robot_ws = ws
            ws.run_forever()
        except Exception as e:
            logger.warning("WiFi listener: %s", e)
        with _ws_lock:
            robot_ws = None
        time.sleep(1)

threading.Thread(target=esp32_listener, daemon=True).start()

# ── USB listener ──────────────────────────────────────────
def serial_listener():
    global serial_conn, connection_mode
    while True:
        # Read Serial even in WiFi mode (ESP32 can be connected via USB)
        with _serial_lock:
            conn = serial_conn
        if not conn or not conn.is_open:
            time.sleep(0.1); continue
        try:
            line = conn.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('IMU,'):
                # Format: IMU,yaw,yaw_rate,ax,ay,az
                parts = line.split(',')
                if len(parts) >= 6:
                    data = {
                        'y': float(parts[1]),
                        'yr': float(parts[2]),
                        'ax': int(parts[3]),
                        'ay': int(parts[4]),
                        'az': int(parts[5])
                    }
                    _push_telemetry(data)
            elif line.startswith('{'):
                data = json.loads(line)
                if 'y' in data:
                    _push_telemetry(data)
        except Exception as e:
            logger.warning("USB error: %s", e)

threading.Thread(target=serial_listener, daemon=True).start()

# Auto-detect (déplacé dans une fonction pour éviter l'émission avant démarrage)
def auto_detect_connection():
    # Try to connect Serial for IMU data (even if WiFi is used for commands)
    if SERIAL_AVAILABLE:
        ports = serial.tools.list_ports.comports()
        if len(ports) == 1:
            try:
                with _serial_lock:
                    if serial_conn and serial_conn.is_open:
                        serial_conn.close()
                    serial_conn = serial.Serial(ports[0].device, SERIAL_BAUD, timeout=1)
                logger.info(f"[USB] Serial connected for IMU: {ports[0].device}")
            except Exception as e:
                logger.warning(f"[USB] Serial connect failed: {e}")
    # Default to WiFi for commands
    switch_to_wifi()
    logger.info("[WIFI] Startup default: WiFi mode enabled (USB available via button)")

# ── IA Loop ───────────────────────────────────────────────
def ia_loop():
    while True:
        if auto_pilot_active:
            with _history_lock:
                if len(telemetry_history) == 0:
                    time.sleep(0.05); continue
                latest = telemetry_history[-1].copy()
            
            latest['dy'] = current_dir_y
            latest['dx'] = current_dir_x
            latest['locked_yaw'] = locked_yaw
            
            try:
                trim_l, trim_r, ramp_boost, log_msg = ai_agent.get_action_and_learn(latest)
                
                send_to_esp32({
                    "t": "ia",
                    "tl": round(trim_l, 2),
                    "tr": round(trim_r, 2),
                    "rbst": round(ramp_boost, 3)
                })
                
                stats = ai_agent.get_training_stats()
                socketio.emit('ia_update', {
                    'active': True,
                    'trim_l': round(trim_l, 2),
                    'trim_r': round(trim_r, 2),
                    'ramp_boost': round(ramp_boost, 3),
                    'log': log_msg,
                    'stats': stats
                })
            except Exception as e:
                logger.error("IA error: %s", e)
        time.sleep(0.05)  # 20Hz pour plus de réactivité

threading.Thread(target=ia_loop, daemon=True).start()

# ── Send ──────────────────────────────────────────────────
def send_to_esp32(data):
    payload = json.dumps(data, separators=(',', ':'))
    logger.info(f"→ ESP32 [{connection_mode}]: {payload}")
    try:
        if connection_mode == 'usb':
            with _serial_lock:
                conn = serial_conn
            if conn and conn.is_open:
                conn.write((payload + '\n').encode('utf-8'))
                return
        with _ws_lock:
            if robot_ws and robot_ws.sock and robot_ws.sock.connected:
                robot_ws.send(payload)
            else:
                ws = create_connection(WS_URL, timeout=2.0)
                ws.send(payload)
                ws.close()
    except Exception as e:
        logger.warning("Send failed: %s", e)

# ── Routes ────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('command')
def handle_command(data):
    global current_dir_x, current_dir_y
    send_to_esp32(data)
    current_dir_x = data.get('x', 0.0)
    current_dir_y = data.get('y', 0.0)

@socketio.on('toggle_auto')
def handle_toggle_auto(data):
    global auto_pilot_active
    auto_pilot_active = data.get('active', False)
    if auto_pilot_active:
        logger.info("[IA] Drive Assist ACTIVE")
        ai_agent.reset_episode()
    else:
        logger.info("[IA] Drive Assist DÉSACTIVÉE")
        vals = ai_agent.get_learned_values()
        send_to_esp32({
            "t": "ia", "active": 0,
            "tl": round(vals[0], 2), "tr": round(vals[1], 2), "rbst": round(vals[2], 3)
        })
    socketio.emit('ia_status', {'active': auto_pilot_active})

@socketio.on('manual_pid')
def handle_manual_pid(data):
    global yaw_kp, yaw_ki, yaw_kd, motor_a_trim, motor_b_trim, motor_a_minpwm, motor_b_minpwm
    yaw_kp = data.get('ykp', data.get('kp', yaw_kp))
    yaw_ki = data.get('yki', data.get('ki', yaw_ki))
    yaw_kd = data.get('ykd', data.get('kd', yaw_kd))
    motor_a_trim   = data.get('ta', motor_a_trim)
    motor_b_trim   = data.get('tb', motor_b_trim)
    motor_a_minpwm = data.get('ma', motor_a_minpwm)
    motor_b_minpwm = data.get('mb', motor_b_minpwm)
    send_to_esp32({"t": "cfg", "ykp": yaw_kp, "yki": yaw_ki, "ykd": yaw_kd,
                   "ta": motor_a_trim, "tb": motor_b_trim,
                   "ma": motor_a_minpwm, "mb": motor_b_minpwm})

def _export_int8_background():
    """Lance export_rpi.py en arrière-plan et notifie le front quand c'est fini."""
    try:
        import subprocess, sys
        export_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "export_rpi.py")
        logger.info("[EXPORT] Démarrage export INT8...")
        socketio.emit('save_status', {'step': 'export_int8', 'status': 'running'})
        result = subprocess.run(
            [sys.executable, export_script],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            capture_output=True, text=True, timeout=60
        )
        if result.returncode == 0:
            logger.info("[EXPORT] INT8 OK → drive_assist_rpi_int8.pt")
            socketio.emit('save_status', {'step': 'export_int8', 'status': 'ok',
                                          'msg': 'drive_assist_rpi_int8.pt prêt'})
        else:
            logger.warning("[EXPORT] Erreur export: %s", result.stderr[-300:])
            socketio.emit('save_status', {'step': 'export_int8', 'status': 'error',
                                          'msg': result.stderr[-200:]})
    except Exception as e:
        logger.warning("[EXPORT] Exception: %s", e)
        socketio.emit('save_status', {'step': 'export_int8', 'status': 'error', 'msg': str(e)})

@socketio.on('save_config')
def handle_save_config(_):
    # 1. EEPROM ESP32
    send_to_esp32({"t": "save"})
    socketio.emit('save_status', {'step': 'eeprom', 'status': 'ok'})

    # 2. robot_config.json (source de vérité globale)
    cfg = _load_global_config()
    cfg.setdefault("_comment", "Source de vérité globale.")
    cfg["pid"]    = {"kp": yaw_kp, "ki": yaw_ki, "kd": yaw_kd}
    cfg["trims"]  = {"a": motor_a_trim, "b": motor_b_trim}
    cfg["minpwm"] = {"a": motor_a_minpwm, "b": motor_b_minpwm}
    ia = ai_agent.get_learned_values()
    cfg["ia_trims"] = {"L": round(ia[0], 3), "R": round(ia[1], 3), "boost": round(ia[2], 4)}
    _save_global_config(cfg)
    logger.info("[CFG] robot_config.json mis à jour")
    socketio.emit('save_status', {'step': 'robot_config_json', 'status': 'ok'})

    # 3. Modèle IA .pt (force save)
    try:
        if hasattr(ai_agent, 'agent'):
            ai_agent.agent._save(force=True)
        socketio.emit('save_status', {'step': 'model_pt', 'status': 'ok',
                                      'msg': 'drive_assist_model.pt sauvegardé'})
    except Exception as e:
        socketio.emit('save_status', {'step': 'model_pt', 'status': 'error', 'msg': str(e)})

    # 4. Export INT8 pour Raspberry Pi (en arrière-plan)
    threading.Thread(target=_export_int8_background, daemon=True).start()

@socketio.on('get_ia_stats')
def handle_get_ia_stats(_):
    socketio.emit('ia_stats', ai_agent.get_training_stats())

@socketio.on('reset_ia')
def handle_reset_ia(_):
    ai_agent.reset_episode()
    socketio.emit('ia_status', {'active': auto_pilot_active, 'reset': True})

@socketio.on('set_connection')
def handle_set_connection(data):
    mode = data.get('mode', 'wifi')
    if mode == 'usb':
        if not connect_serial(data.get('port')):
            socketio.emit('connection_status', {'mode': connection_mode, 'error': 'USB not found'})
    else:
        switch_to_wifi()

@socketio.on('scan_ports')
def handle_scan_ports(_):
    socketio.emit('ports_list', {'ports': list_serial_ports()})

@socketio.on('toggle_us')
def handle_toggle_us(data):
    enabled = data.get('enabled', True)
    send_to_esp32({"t": "us", "en": 1 if enabled else 0})
    socketio.emit('us_status', {'enabled': enabled})

if __name__ == '__main__':
    # Démarrer l'auto-détection après un court délai pour laisser le serveur démarrer
    def delayed_auto_detect():
        time.sleep(0.5)
        auto_detect_connection()
    threading.Thread(target=delayed_auto_detect, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
