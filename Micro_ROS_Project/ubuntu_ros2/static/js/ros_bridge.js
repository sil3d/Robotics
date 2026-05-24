// ── Telemetry update ────────────────────────────────────────────────────────
let usMaskState = [true, true, true, true];

async function updateTelemetry() {
    try {
        const resp = await fetch('/state');
        const data = await resp.json();

        const usMask = Array.isArray(data.us_mask) && data.us_mask.length >= 4
            ? data.us_mask.map(v => Boolean(v))
            : [true, true, true, true];
        usMaskState = usMask.slice();

        // Robot mode with color
        const modeEl = document.getElementById('robotMode');
        modeEl.textContent = data.robot_mode || 'UNKNOWN';
        modeEl.className = 'status-' + (data.robot_mode || 'unknown').toLowerCase();

        // Sensor health
        const sh = data.sensor_health;
        if (sh && sh.sensors) {
            const s = sh.sensors;
            document.getElementById('imuHealth').textContent = s.imu ? (s.imu.healthy ? 'OK' : 'FAIL') : '--';
            document.getElementById('tagHealth').textContent = s.camera_apriltag ? (s.camera_apriltag.healthy ? 'OK' : 'FAIL') : '--';
            const usF = s.ultrasonic_front;
            document.getElementById('usHealth').textContent = usF ? (usF.healthy ? 'OK' : 'FAIL') : '--';
        } else if (sh && sh.us_front !== undefined) {
            const usF = sh.us_front >= 0 ? sh.us_front.toFixed(0) + 'cm' : '--';
            const usB = sh.us_back >= 0 ? sh.us_back.toFixed(0) + 'cm' : '--';
            const usL = sh.us_left >= 0 ? sh.us_left.toFixed(0) + 'cm' : '--';
            const usR = sh.us_right >= 0 ? sh.us_right.toFixed(0) + 'cm' : '--';
            document.getElementById('imuHealth').textContent = sh.imu_healthy ? 'OK' : 'FAIL';
            document.getElementById('usHealth').textContent = `F:${usF} B:${usB} L:${usL} R:${usR}`;
            document.getElementById('tagHealth').textContent = '--';
        }

        // Confidence bar
        const conf = (data.localization_confidence || 0) * 100;
        document.getElementById('confFill').style.width = conf + '%';
        document.getElementById('confValue').textContent = conf.toFixed(0) + '%';

        // Box info
        if (data.box_info) {
            document.getElementById('boxColor').textContent = data.box_info.color || '--';
            document.getElementById('boxOrient').textContent = data.box_info.orientation || '--';
            document.getElementById('boxDist').textContent = data.box_info.distance ? data.box_info.distance.toFixed(2) : '--';
            document.getElementById('boxWidth').textContent = data.box_info.width_mm ? data.box_info.width_mm.toFixed(1) : '--';
            document.getElementById('boxHeight').textContent = data.box_info.height_mm ? data.box_info.height_mm.toFixed(1) : '--';
        }

        // Gripper
        if (data.gripper_status) {
            document.getElementById('gripperState').textContent = data.gripper_status.state || '--';
            document.getElementById('gripperHasBox').textContent = data.gripper_status.has_box ? 'YES' : 'NO';
        }

        // Mission state
        const missionState = data.mission_state || {};
        document.getElementById('taskState').textContent = missionState.state || data.task_status || 'IDLE';
        if (data.mission_state) {
            document.getElementById('missionState').textContent = missionState.state || 'IDLE';
            document.getElementById('missionColor').textContent = missionState.target_color || '--';
            document.getElementById('missionTag').textContent = (missionState.target_tag !== undefined && missionState.target_tag !== null) ? missionState.target_tag : '--';
            if (missionState.lstm) {
                document.getElementById('lstmEnabled').textContent = missionState.lstm.enabled ? 'ON' : 'OFF';
                document.getElementById('lstmRecording').textContent = missionState.lstm.recording_enabled ? 'ON' : 'OFF';
                document.getElementById('lstmConfidence').textContent = missionState.lstm.last_prediction ? (Math.round((missionState.lstm.last_prediction.confidence || 0) * 100) + '%') : '--';
                document.getElementById('lstmFallback').textContent = missionState.lstm.last_fallback_reason || 'OK';
                const toggleBtn = document.getElementById('lstmToggleBtn');
                if (toggleBtn) toggleBtn.textContent = missionState.lstm.enabled ? 'Disable LSTM' : 'Enable LSTM';
            }
        }
        document.getElementById('scanState').textContent = data.scan_status || 'IDLE';

        // Mission counter
        if (data.mission_count) {
            const mc = data.mission_count;
            const el = document.getElementById('missionProgress');
            el.textContent = mc.completed + '/' + mc.total;
            el.style.color = mc.remaining > 0 ? '#00ff00' : '#00ffaa';
        }

        // IMU telemetry
        if (data.imu) {
            document.getElementById('imuYaw').textContent = data.imu.yaw_deg.toFixed(1) + ' deg';
            document.getElementById('imuOmega').textContent = data.imu.omega_z.toFixed(1) + ' deg/s';
            document.getElementById('imuAx').textContent = data.imu.ax.toFixed(2);
            document.getElementById('imuAy').textContent = data.imu.ay.toFixed(2);
            document.getElementById('imuAz').textContent = data.imu.az.toFixed(2);

            const us = data.imu.us || [];
            us.forEach((val, i) => {
                const el = document.getElementById('us' + i);
                if (el) {
                    if (val < 0) { el.textContent = '-- cm'; el.className = 'sensor-value'; }
                    else { el.textContent = val.toFixed(1) + ' cm'; el.className = val < 20 ? 'sensor-value danger' : val < 50 ? 'sensor-value warning' : 'sensor-value'; }
                }
            });

            const connIMU = document.getElementById('connIMU');
            if (connIMU) connIMU.className = 'status-dot connected';
        }

        const usButtons = document.querySelectorAll('.us-toggle-btn');
        usButtons.forEach((btn) => {
            const idx = parseInt(btn.dataset.usIdx, 10);
            const enabled = usMaskState[idx];
            btn.classList.toggle('active', enabled);
            btn.classList.toggle('off', !enabled);
            btn.textContent = `US${idx + 1} ${enabled ? 'ON' : 'OFF'}`;
        });

        const usValues = [
            document.getElementById('us0'),
            document.getElementById('us1'),
            document.getElementById('us2'),
            document.getElementById('us3'),
        ];
        usValues.forEach((el, idx) => {
            if (!el) return;
            if (!usMaskState[idx]) {
                el.textContent = 'OFF';
                el.className = 'sensor-value';
            }
        });

        if (data.cmd_result) document.getElementById('lastCmd').textContent = data.cmd_result;
    } catch (e) { console.error('Telemetry error:', e); }
}

// ── Action functions ─────────────────────────────────────────────────────────
function calibrateGripper() { fetch('/service/calibrate_gripper', {method: 'POST'}); }
function startMission()      { fetch('/service/start_task',        {method: 'POST'}); }
function stopMission()       { fetch('/service/cancel_task',       {method: 'POST'}); }
function resetOdom()         { fetch('/service/reset_odom',        {method: 'POST'}); }
function startScan()         { fetch('/service/start_scan',        {method: 'POST'}); }
function resetRecovery()     { fetch('/service/reset_recovery',    {method: 'POST'}); }

function toggleLstm() {
    const enabled = document.getElementById('lstmEnabled').textContent !== 'ON';
    fetch('/api/mission_ctrl', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({enabled: enabled, recording_enabled: true})
    });
}

function emergencyStop() {
    fetch('/service/emergency_stop', {method: 'POST'});
    stopRobot();
}

function sendGripper(cmd) {
    fetch('/api/gripper', {method: 'POST', body: JSON.stringify({value: cmd})});
}

function sendUltrasonicMask(mask) {
    fetch('/api/config', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            us_mask: mask.map(v => v ? 1 : 0),
            save: 1,
        })
    });
}

// ── Velocity control ─────────────────────────────────────────────────────────
function moveForward()  { fetch('/velocity', {method: 'POST', body: JSON.stringify({linear:  0.2, angular:  0})}); }
function moveBackward() { fetch('/velocity', {method: 'POST', body: JSON.stringify({linear: -0.2, angular:  0})}); }
function turnLeft()     { fetch('/velocity', {method: 'POST', body: JSON.stringify({linear:  0,   angular:  0.5})}); }
function turnRight()    { fetch('/velocity', {method: 'POST', body: JSON.stringify({linear:  0,   angular: -0.5})}); }
function stopMove()     { fetch('/velocity', {method: 'POST', body: JSON.stringify({linear:  0,   angular:  0})}); }
function stopTurn()     { fetch('/velocity', {method: 'POST', body: JSON.stringify({linear:  0,   angular:  0})}); }
function stopRobot()    { fetch('/velocity', {method: 'POST', body: JSON.stringify({linear:  0,   angular:  0})}); }

// ── Mission editor ───────────────────────────────────────────────────────────
let missionRows = [];

function loadMissions() {
    fetch('/api/missions').then(r => r.json()).then(data => {
        const missions = data.missions || [];
        document.getElementById('missionHomeTag').value = data.home_tag || 12;
        document.getElementById('missionRepeat').checked = data.repeat !== false;
        missionRows = missions.map(m => ({
            pickup_tag: m.pickup_tag, drop_tag: m.drop_tag,
            color: m.color || 'blue', label: m.label || ''
        }));
        renderMissionRows();
        document.getElementById('missionList').textContent = missions.length + ' mission(s) configured';
    }).catch(() => {
        document.getElementById('missionList').textContent = 'Error loading missions';
    });
}

function renderMissionRows() {
    const editor = document.getElementById('missionEditor');
    let html = '';
    missionRows.forEach((m, i) => {
        html += '<div style="margin-bottom:4px; display:flex; gap:4px; align-items:center;">' +
            '<span style="color:#888; min-width:18px;">#' + (i + 1) + '</span>' +
            '<input type="number" placeholder="Pickup" value="' + (m.pickup_tag || '') + '" style="width:50px; background:#222; color:#fff; border:1px solid #444;" onchange="missionRows[' + i + '].pickup_tag=+this.value">' +
            '<span style="color:#666;">→</span>' +
            '<input type="number" placeholder="Drop" value="' + (m.drop_tag || '') + '" style="width:50px; background:#222; color:#fff; border:1px solid #444;" onchange="missionRows[' + i + '].drop_tag=+this.value">' +
            '<select style="background:#222; color:#fff; border:1px solid #444;" onchange="missionRows[' + i + '].color=this.value">' +
                '<option value="blue"' + (m.color === 'blue' ? ' selected' : '') + '>Blue</option>' +
                '<option value="green"' + (m.color === 'green' ? ' selected' : '') + '>Green</option>' +
            '</select>' +
            '<input type="text" placeholder="Label" value="' + (m.label || '') + '" style="flex:1; background:#222; color:#fff; border:1px solid #444;" onchange="missionRows[' + i + '].label=this.value">' +
            '<button onclick="missionRows.splice(' + i + ',1); renderMissionRows();" style="background:#600; color:#fff; border:none; cursor:pointer;">✕</button>' +
            '</div>';
    });
    editor.innerHTML = html;
}

function addMissionRow() {
    missionRows.push({pickup_tag: 3, drop_tag: 6, color: 'blue', label: ''});
    renderMissionRows();
}

function saveMissions() {
    const payload = {
        missions: missionRows,
        home_tag: parseInt(document.getElementById('missionHomeTag').value) || 12,
        repeat: document.getElementById('missionRepeat').checked
    };
    fetch('/api/missions', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
    }).then(r => r.json()).then(data => {
        document.getElementById('missionList').textContent =
            data.status === 'ok' ? data.count + ' mission(s) saved!' : 'Error: ' + (data.error || 'unknown');
    });
}

// ── Init ─────────────────────────────────────────────────────────────────────
document.querySelectorAll('.us-toggle-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
        const idx = parseInt(btn.dataset.usIdx, 10);
        usMaskState[idx] = !usMaskState[idx];
        sendUltrasonicMask(usMaskState);
    });
});
loadMissions();
setInterval(updateTelemetry, 100);
