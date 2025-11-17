# server.py
from fastapi import FastAPI, HTTPException, Depends, Header, status
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List
import uuid
import hashlib
import json
import paho.mqtt.client as mqtt

from sqlalchemy import (
    create_engine, Column, Integer, String, Float,
    DateTime, ForeignKey
)
from sqlalchemy.orm import sessionmaker, declarative_base, Session, relationship

# =========================
# Configuration
# =========================

ONLINE_THRESHOLD_SECONDS = 20  # Device is considered online if seen within this many seconds

IMPORTANT_EVENTS = {
    "Gerakan Terdeteksi",   # movement
    "Posisi Diminta",       # request position
    "System Armed",
    "System Disarmed",
    "Theft Warning",
}

MQTT_BROKER = "localhost"          # or your broker IP, e.g. "127.0.0.1"
MQTT_PORT = 1883
MQTT_BASE_TOPIC = "motor"

mqtt_client = mqtt.Client()

# =========================
# Database setup
# =========================

DATABASE_URL = "sqlite:///./antitheft.db"

engine = create_engine(
    DATABASE_URL, connect_args={"check_same_thread": False}
)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, nullable=False)
    email = Column(String, unique=True, index=True, nullable=False)
    password_hash = Column(String, nullable=False)
    auth_token = Column(String, unique=True, index=True, nullable=True)

    devices = relationship("Device", back_populates="owner")


class Device(Base):
    __tablename__ = "devices"

    id = Column(String, primary_key=True, index=True)  # device_id, e.g. MOTOR-ABC123
    name = Column(String, nullable=False)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=True)
    last_seen = Column(DateTime, nullable=True)
    last_status = Column(String, nullable=True)
    last_lat = Column(Float, nullable=True)
    last_lon = Column(Float, nullable=True)

    owner = relationship("User", back_populates="devices")
    alerts = relationship("Alert", back_populates="device")


class Alert(Base):
    __tablename__ = "alerts"

    id = Column(Integer, primary_key=True, index=True)
    device_id = Column(String, ForeignKey("devices.id"), index=True)
    status = Column(String, nullable=False)
    lat = Column(Float, nullable=True)
    lon = Column(Float, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)

    device = relationship("Device", back_populates="alerts")


Base.metadata.create_all(bind=engine)


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# =========================
# Simple auth helpers (no JWT)
# =========================

def hash_password(password: str) -> str:
    return hashlib.sha256(password.encode("utf-8")).hexdigest()


def verify_password(password: str, password_hash: str) -> bool:
    return hash_password(password) == password_hash


def generate_token() -> str:
    return uuid.uuid4().hex


def get_current_user(
    db: Session = Depends(get_db),
    x_auth_token: Optional[str] = Header(default=None),
):
    """
    Simple auth using 'X-Auth-Token' header.
    """
    if not x_auth_token:
        raise HTTPException(status_code=401, detail="Missing X-Auth-Token header")

    user = db.query(User).filter(User.auth_token == x_auth_token).first()
    if not user:
        raise HTTPException(status_code=401, detail="Invalid auth token")
    return user


# =========================
# Pydantic models
# =========================

class RegisterIn(BaseModel):
    name: str
    email: str
    password: str


class LoginIn(BaseModel):
    email: str
    password: str


class AuthOut(BaseModel):
    user_id: int
    name: str
    email: str
    auth_token: str


class DeviceRegisterIn(BaseModel):
    device_id: str
    name: str


class DeviceOut(BaseModel):
    id: str
    name: str
    user_id: Optional[int]


class AlertIn(BaseModel):
    device_id: str
    status: str
    lat: Optional[float] = None
    lon: Optional[float] = None


class AlertOut(BaseModel):
    id: int
    device_id: str
    status: str
    lat: Optional[float]
    lon: Optional[float]
    created_at: datetime


class CommandIn(BaseModel):
    command: str
    value: Optional[str] = None


# In-memory pending commands per device
PENDING_COMMANDS: dict[str, Optional[dict]] = {}


# =========================
# FastAPI app
# =========================

app = FastAPI(
    title="Motor Anti-Theft Backend",
    description="Prototype backend with DB, simple auth, device registration, and debug dashboard",
)


@app.on_event("startup")
def on_startup():
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()  # Start background thread for MQTT
        print("[MQTT] Connected to broker")
    except Exception as e:
        print("[MQTT] Failed to connect at startup:", e)


@app.on_event("shutdown")
def on_shutdown():
    try:
        mqtt_client.loop_stop()  # Stop background thread
        mqtt_client.disconnect()
        print("[MQTT] Disconnected from broker")
    except Exception as e:
        print("[MQTT] Failed to disconnect:", e)


# =========================
# Auth endpoints (for mobile + dashboard)
# =========================

@app.post("/register", response_model=AuthOut)
def register(payload: RegisterIn, db: Session = Depends(get_db)):
    existing = db.query(User).filter(User.email == payload.email).first()
    if existing:
        raise HTTPException(status_code=400, detail="Email already registered")

    user = User(
        name=payload.name,
        email=payload.email,
        password_hash=hash_password(payload.password),
        auth_token=generate_token(),
    )
    db.add(user)
    db.commit()
    db.refresh(user)

    return AuthOut(
        user_id=user.id,
        name=user.name,
        email=user.email,
        auth_token=user.auth_token,
    )


@app.post("/login", response_model=AuthOut)
def login(payload: LoginIn, db: Session = Depends(get_db)):
    user = db.query(User).filter(User.email == payload.email).first()
    if not user or not verify_password(payload.password, user.password_hash):
        raise HTTPException(status_code=400, detail="Invalid email or password")

    if not user.auth_token:
        user.auth_token = generate_token()
        db.commit()
        db.refresh(user)

    return AuthOut(
        user_id=user.id,
        name=user.name,
        email=user.email,
        auth_token=user.auth_token,
    )


# =========================
# User-device endpoints
# =========================

@app.get("/me/devices", response_model=List[DeviceOut])
def list_my_devices(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    devices = (
        db.query(Device)
        .filter(Device.user_id == current_user.id)
        .order_by(Device.id)
        .all()
    )
    return [
        DeviceOut(id=d.id, name=d.name, user_id=d.user_id)
        for d in devices
    ]


@app.post("/devices/register", response_model=DeviceOut)
def register_device(
    payload: DeviceRegisterIn,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    device = db.query(Device).filter(Device.id == payload.device_id).first()

    if device:
        if device.user_id is None:
            device.user_id = current_user.id
            device.name = payload.name
            db.commit()
            db.refresh(device)
        elif device.user_id != current_user.id:
            raise HTTPException(status_code=409, detail="Device already owned by another user")
        else:
            device.name = payload.name
            db.commit()
            db.refresh(device)
    else:
        device = Device(
            id=payload.device_id,
            name=payload.name,
            user_id=current_user.id,
        )
        db.add(device)
        db.commit()
        db.refresh(device)

    if device.id not in PENDING_COMMANDS:
        PENDING_COMMANDS[device.id] = None

    return DeviceOut(id=device.id, name=device.name, user_id=device.user_id)


@app.delete("/devices/{device_id}", response_model=DeviceOut)
def remove_device(
    device_id: str,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    device = db.query(Device).filter(Device.id == device_id).first()
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    # Only the owner can remove/unlink it
    if device.user_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="You are not the owner of this device",
        )

    # "Remove" = unlink from user, keep alerts + device record
    device.user_id = None
    db.commit()
    db.refresh(device)

    # Also clear pending commands, if any
    if device.id in PENDING_COMMANDS:
        PENDING_COMMANDS[device.id] = None

    return DeviceOut(id=device.id, name=device.name, user_id=device.user_id)


# =========================
# Device / ESP-facing endpoints
# =========================

@app.post("/api/alert")
def receive_alert(payload: AlertIn, db: Session = Depends(get_db)):
    device = db.query(Device).filter(Device.id == payload.device_id).first()

    if not device:
        device = Device(
            id=payload.device_id,
            name=f"Unregistered device {payload.device_id}",
            user_id=None,
        )
        db.add(device)

    # Always update current state
    device.last_seen = datetime.utcnow()
    device.last_status = payload.status
    device.last_lat = payload.lat
    device.last_lon = payload.lon

    # Only store meaningful events
    alert_id = None
    if payload.status in IMPORTANT_EVENTS:
        alert = Alert(
            device_id=device.id,
            status=payload.status,
            lat=payload.lat,
            lon=payload.lon,
            created_at=datetime.utcnow(),
        )
        db.add(alert)
        db.commit()
        db.refresh(alert)
        alert_id = alert.id

        print("IMPORTANT EVENT:", {
            "device_id": alert.device_id,
            "status": alert.status,
            "lat": alert.lat,
            "lon": alert.lon,
            "created_at": alert.created_at.isoformat(),
        })
    else:
        db.commit()

    if device.id not in PENDING_COMMANDS:
        PENDING_COMMANDS[device.id] = None

    return {"ok": True, "alert_id": alert_id}


@app.get("/api/devices/{device_id}/alerts", response_model=List[AlertOut])
def get_device_alerts(device_id: str, db: Session = Depends(get_db)):
    device = db.query(Device).filter(Device.id == device_id).first()
    if not device:
        raise HTTPException(status_code=404, detail="Unknown device")

    alerts = (
        db.query(Alert)
        .filter(Alert.device_id == device_id)
        .order_by(Alert.created_at.asc())
        .all()
    )

    return [
        AlertOut(
            id=a.id,
            device_id=a.device_id,
            status=a.status,
            lat=a.lat,
            lon=a.lon,
            created_at=a.created_at,
        )
        for a in alerts
    ]


@app.post("/api/send/{device_id}")
def send_command(device_id: str, cmd: CommandIn, db: Session = Depends(get_db)):
    device = db.query(Device).filter(Device.id == device_id).first()
    if not device:
        raise HTTPException(status_code=404, detail="Unknown device")

    topic = f"{MQTT_BASE_TOPIC}/{device_id}/cmd"
    payload = {
        "command": cmd.command,
        "value": cmd.value,
    }
    try:
        result = mqtt_client.publish(topic, json.dumps(payload), qos=1)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            raise HTTPException(
                status_code=500,
                detail=f"MQTT publish failed with code {result.rc}",
            )
    except Exception as e:
        print("[MQTT] Publish error:", e)
        raise HTTPException(status_code=500, detail="Failed to publish MQTT command")

    return {
        "ok": True,
        "device_id": device_id,
        "topic": topic,
        "payload": payload,
    }


@app.get("/api/commands/{device_id}")
def get_commands(device_id: str, db: Session = Depends(get_db)):
    device = db.query(Device).filter(Device.id == device_id).first()
    if not device:
        raise HTTPException(status_code=404, detail="Unknown device")

    cmd = PENDING_COMMANDS.get(device_id) or {"command": None, "value": None}
    PENDING_COMMANDS[device_id] = None
    return cmd


@app.get("/devices/{device_id}/status")
def device_status(device_id: str, db: Session = Depends(get_db)):
    device = db.query(Device).filter(Device.id == device_id).first()
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    if not device.last_seen:
        return {"device_id": device_id, "online": False, "last_seen": None}

    diff = (datetime.utcnow() - device.last_seen).total_seconds()
    online = diff <= ONLINE_THRESHOLD_SECONDS

    return {
        "device_id": device_id,
        "online": online,
        "seconds_since_seen": diff,
        "last_seen": device.last_seen.isoformat(),
    }


@app.get("/devices/{device_id}/current")
def current_status(device_id: str, db: Session = Depends(get_db)):
    device = db.query(Device).filter(Device.id == device_id).first()
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    if not device.last_seen:
        return {
            "device_id": device.id,
            "online": False,
            "seconds_since_seen": None,
            "last_status": None,
            "lat": None,
            "lon": None,
        }

    diff = (datetime.utcnow() - device.last_seen).total_seconds()
    online = diff < ONLINE_THRESHOLD_SECONDS

    return {
        "device_id": device.id,
        "online": online,
        "seconds_since_seen": diff,
        "last_status": device.last_status,
        "lat": device.last_lat,
        "lon": device.last_lon,
    }


# =========================
# Debug dashboard (with user + device flows)
# =========================

@app.get("/", response_class=HTMLResponse)
def dashboard():
    html = """
    <!doctype html>
    <html>
    <head>
        <meta charset="utf-8" />
        <title>Motor Anti-Theft Debug Dashboard</title>
        <style>
            body {
                font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
                max-width: 1000px;
                margin: 2rem auto;
                padding: 0 1rem;
            }
            h1, h2, h3 {
                margin-bottom: 0.5rem;
            }
            button {
                padding: 0.4rem 0.8rem;
                margin: 0.1rem;
                cursor: pointer;
            }
            input, select {
                padding: 0.25rem 0.4rem;
                margin: 0.15rem 0;
            }
            fieldset {
                border: 1px solid #ccc;
                border-radius: 6px;
                padding: 0.75rem 1rem;
                margin-bottom: 1rem;
            }
            legend {
                font-weight: 600;
                padding: 0 0.25rem;
            }
            #log {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 0.5rem;
                height: 220px;
                overflow-y: auto;
                white-space: pre-wrap;
                font-size: 0.85rem;
                background: #fafafa;
            }
            #alerts {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 0.5rem;
                max-height: 250px;
                overflow-y: auto;
                font-size: 0.9rem;
            }
            .alert-item {
                margin: 0.25rem 0;
                border-bottom: 1px dashed #ddd;
                padding-bottom: 0.25rem;
            }
            .alert-status {
                font-weight: 600;
            }
            .pill {
                display: inline-block;
                padding: 0.1rem 0.4rem;
                font-size: 0.75rem;
                border-radius: 999px;
                background: #eee;
                margin-left: 0.25rem;
            }
            .row {
                display: flex;
                gap: 1rem;
                flex-wrap: wrap;
            }
            .col {
                flex: 1 1 280px;
            }
            .small {
                font-size: 0.8rem;
                color: #666;
            }
            .token-box {
                font-family: monospace;
                font-size: 0.8rem;
                padding: 0.25rem 0.4rem;
                border-radius: 4px;
                background: #f3f3f3;
                word-break: break-all;
            }
        </style>
    </head>
    <body>
        <h1>Motor Anti-Theft Debug Dashboard</h1>
        <p class="small">
            This page lets you: register users, log in, register devices to a user, send commands,
            and view alerts in real time. Perfect for testing your ESP32 + GPS prototype.
        </p>

        <div class="row">
            <div class="col">
                <fieldset>
                    <legend>User Register</legend>
                    <label>Name<br>
                        <input id="regName" type="text" placeholder="Your name">
                    </label><br>
                    <label>Email<br>
                        <input id="regEmail" type="email" placeholder="you@example.com">
                    </label><br>
                    <label>Password<br>
                        <input id="regPassword" type="password" placeholder="password">
                    </label><br>
                    <button onclick="doRegister()">Register</button>
                </fieldset>

                <fieldset>
                    <legend>User Login</legend>
                    <label>Email<br>
                        <input id="loginEmail" type="email" placeholder="you@example.com">
                    </label><br>
                    <label>Password<br>
                        <input id="loginPassword" type="password" placeholder="password">
                    </label><br>
                    <button onclick="doLogin()">Login</button>
                    <p class="small">After login, your auth token will be used for /me/devices and /devices/register.</p>
                </fieldset>

                <fieldset>
                    <legend>Current User</legend>
                    <div id="currentUser">Not logged in</div>
                    <div class="small">Auth token (for debugging):</div>
                    <div id="authTokenBox" class="token-box">(none)</div>
                    <button onclick="logout()">Logout</button>
                </fieldset>
            </div>

            <div class="col">
                <fieldset>
                    <legend>Register / Rename Device</legend>
                    <label>Device ID<br>
                        <input id="devId" type="text" placeholder="e.g. MOTOR-ABC123">
                    </label><br>
                    <label>Device Name<br>
                        <input id="devName" type="text" placeholder="Beat putih depan kos">
                    </label><br>
                    <button onclick="registerDevice()">Register / Claim Device</button>
                    <p class="small">Requires login. Device will be linked to the current user.</p>
                </fieldset>

                <fieldset>
                    <legend>My Devices</legend>
                    <button onclick="loadMyDevices()">Refresh My Devices</button><br>
                    <label>Selected device<br>
                        <select id="deviceSelect">
                            <option value="">(none)</option>
                        </select>
                    </label>
                    <br>
                    <div id="deviceStatus">Status: (none)</div>
                    <br>
                    <button onclick="removeSelectedDevice()">Remove from my account</button>
                    <p class="small">
                        This only unlinks the device from your account. Alerts and device ID stay in the system.
                    </p>
                </fieldset>
            </div>
        </div>

        <fieldset>
            <legend>Commands to Device</legend>
            <div>
                <button onclick="sendCommand('BUZZ')">Buzz alarm</button>
                <button onclick="sendCommand('REQUEST_POSITION')">Request position</button>
                <button onclick="sendCommand('ARM')">Arm</button>
                <button onclick="sendCommand('DISARM')">Disarm</button>
                <button onclick="refreshAlerts()">Refresh alerts</button>
            </div>
            <div style="margin-top: 0.5rem;">
                <input id="customCmd" type="text" placeholder="Custom command (e.g. TEST)">
                <button onclick="sendCustomCommand()">Send custom</button>
            </div>
        </fieldset>

        <div class="row">
            <div class="col">
                <h2>Alerts</h2>
                <div id="alerts">No alerts yet</div>
            </div>
            <div class="col">
                <h2>Server Log</h2>
                <div id="log"></div>
            </div>
        </div>

        <script>
        let authToken = null;
        let currentUser = null;

        function log(msg) {
            const logDiv = document.getElementById('log');
            const time = new Date().toLocaleTimeString();
            logDiv.textContent += `[${time}] ${msg}\\n`;
            logDiv.scrollTop = logDiv.scrollHeight;
        }

        function updateUserUI() {
            const userDiv = document.getElementById('currentUser');
            const tokenDiv = document.getElementById('authTokenBox');
            if (currentUser && authToken) {
                userDiv.textContent = `#${currentUser.user_id} - ${currentUser.name} (${currentUser.email})`;
                tokenDiv.textContent = authToken;
            } else {
                userDiv.textContent = "Not logged in";
                tokenDiv.textContent = "(none)";
            }
        }

        async function doRegister() {
            const name = document.getElementById('regName').value.trim();
            const email = document.getElementById('regEmail').value.trim();
            const password = document.getElementById('regPassword').value;

            if (!name || !email || !password) {
                log("Register: please fill all fields");
                return;
            }

            try {
                const res = await fetch('/register', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ name, email, password })
                });
                const data = await res.json();
                if (!res.ok) {
                    log(`Register failed: ${res.status} ${data.detail || ''}`);
                    return;
                }
                authToken = data.auth_token;
                currentUser = data;
                updateUserUI();
                log(`Registered & logged in as ${data.email}`);
            } catch (e) {
                log('Register error: ' + e);
            }
        }

        async function doLogin() {
            const email = document.getElementById('loginEmail').value.trim();
            const password = document.getElementById('loginPassword').value;

            if (!email || !password) {
                log("Login: please fill all fields");
                return;
            }

            try {
                const res = await fetch('/login', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ email, password })
                });
                const data = await res.json();
                if (!res.ok) {
                    log(`Login failed: ${res.status} ${data.detail || ''}`);
                    return;
                }
                authToken = data.auth_token;
                currentUser = data;
                updateUserUI();
                log(`Logged in as ${data.email}`);
                loadMyDevices();
            } catch (e) {
                log('Login error: ' + e);
            }
        }

        function logout() {
            authToken = null;
            currentUser = null;
            updateUserUI();
            const sel = document.getElementById('deviceSelect');
            sel.innerHTML = '<option value="">(none)</option>';
            log("Logged out");
        }

        async function registerDevice() {
            if (!authToken) {
                log("Device register: please login first");
                return;
            }
            const device_id = document.getElementById('devId').value.trim();
            const name = document.getElementById('devName').value.trim();
            if (!device_id || !name) {
                log("Device register: fill device ID and name");
                return;
            }
            try {
                const res = await fetch('/devices/register', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                        'X-Auth-Token': authToken
                    },
                    body: JSON.stringify({ device_id, name })
                });
                const data = await res.json();
                if (!res.ok) {
                    log(`Device register failed: ${res.status} ${data.detail || ''}`);
                    return;
                }
                log(`Device registered/claimed: ${data.id} (${data.name})`);
                loadMyDevices();
            } catch (e) {
                log('Device register error: ' + e);
            }
        }

        async function updateDeviceStatus(deviceId) {
            if (!deviceId) {
                document.getElementById('deviceStatus').textContent = "Status: (none)";
                return;
            }
            try {
                const res = await fetch(`/devices/${deviceId}/status`);
                const data = await res.json();
                if (!res.ok) {
                    log("Error loading status");
                    return;
                }
                const online = data.online ? "🟢 Online" : "🔴 Offline";
                document.getElementById('deviceStatus').textContent =
                    `Status: ${online} (last seen ${data.seconds_since_seen.toFixed(0)}s ago)`;
            } catch (e) {
                log("Status error: " + e);
            }
        }

        async function loadMyDevices() {
            if (!authToken) {
                log("Load my devices: please login first");
                return;
            }
            try {
                const res = await fetch('/me/devices', {
                    headers: { 'X-Auth-Token': authToken }
                });
                const data = await res.json();
                if (!res.ok) {
                    log(`Load my devices failed: ${res.status} ${data.detail || ''}`);
                    return;
                }
                const sel = document.getElementById('deviceSelect');
                sel.innerHTML = '';
                if (!data.length) {
                    sel.innerHTML = '<option value="">(no devices)</option>';
                    log("No devices yet for this user");
                    return;
                }
                for (const d of data) {
                    const opt = document.createElement('option');
                    opt.value = d.id;
                    opt.textContent = `${d.id} - ${d.name}`;
                    sel.appendChild(opt);
                }
                log(`Loaded ${data.length} device(s) for current user`);
            } catch (e) {
                log('Load my devices error: ' + e);
            }
        }

        async function sendCommand(cmd) {
            const deviceId = document.getElementById('deviceSelect').value;
            if (!deviceId) {
                log("Send command: no device selected");
                return;
            }
            try {
                const res = await fetch(`/api/send/${deviceId}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ command: cmd })
                });
                const data = await res.json();
                if (!res.ok) {
                    log(`Error sending command: ${res.status} ${data.detail || ''}`);
                } else {
                    log(`Command '${cmd}' queued for ${deviceId}`);
                }
            } catch (e) {
                log('Error sending command: ' + e);
            }
        }

        function sendCustomCommand() {
            const cmd = document.getElementById('customCmd').value.trim();
            if (!cmd) {
                log("Custom command is empty");
                return;
            }
            sendCommand(cmd);
        }

        async function refreshAlerts() {
            const deviceId = document.getElementById('deviceSelect').value;
            if (!deviceId) {
                document.getElementById('alerts').textContent = 'No device selected';
                return;
            }
            try {
                const res = await fetch(`/api/devices/${deviceId}/alerts`);
                const data = await res.json();
                const alertsDiv = document.getElementById('alerts');
                alertsDiv.innerHTML = '';
                if (!Array.isArray(data) || !data.length) {
                    alertsDiv.textContent = 'No alerts yet';
                    return;
                }
                for (const a of data.slice().reverse()) {
                    const div = document.createElement('div');
                    div.className = 'alert-item';
                    const status = document.createElement('span');
                    status.className = 'alert-status';
                    status.textContent = a.status;
                    const time = document.createElement('span');
                    time.className = 'pill';
                    time.textContent = a.created_at;
                    const coords = document.createElement('div');
                    coords.textContent = `lat: ${a.lat}, lon: ${a.lon}`;
                    div.appendChild(status);
                    div.appendChild(time);
                    div.appendChild(coords);
                    alertsDiv.appendChild(div);
                }
                log(`Alerts refreshed for ${deviceId}`);
            } catch (e) {
                document.getElementById('alerts').textContent = 'Error loading alerts';
                log('Error refreshing alerts: ' + e);
            }
        }

        async function removeSelectedDevice() {
            if (!authToken) {
                log("Remove device: please login first");
                return;
            }
            const deviceId = document.getElementById('deviceSelect').value;
            if (!deviceId) {
                log("Remove device: no device selected");
                return;
            }

            if (!confirm(`Remove device ${deviceId} from your account?`)) {
                return;
            }

            try {
                const res = await fetch(`/devices/${deviceId}`, {
                    method: 'DELETE',
                    headers: {
                        'X-Auth-Token': authToken
                    }
                });
                const data = await res.json();
                if (!res.ok) {
                    log(`Remove device failed: ${res.status} ${data.detail || ''}`);
                    return;
                }
                log(`Device unlinked from account: ${data.id}`);
                // reload list so it disappears
                loadMyDevices();
                // clear alerts list too
                document.getElementById('alerts').textContent = 'No device selected';
            } catch (e) {
                log('Remove device error: ' + e);
            }
        }

        document.getElementById('deviceSelect').onchange = () => {
            updateDeviceStatus(document.getElementById('deviceSelect').value);
            refreshAlerts();
        };

        setInterval(() => {
            const id = document.getElementById('deviceSelect').value;
            updateDeviceStatus(id);
        }, 5000);

        setInterval(refreshAlerts, 5000);
        </script>
    </body>
    </html>
    """
    return HTMLResponse(html)
