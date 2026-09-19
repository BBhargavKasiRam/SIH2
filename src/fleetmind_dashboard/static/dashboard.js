/**
 * FleetMind Dashboard — Real-time SSE Client
 * Connects to the dashboard_node's SSE endpoint and renders fleet state.
 */

(function () {
    'use strict';

    const SSE_URL = '/api/sse';
    const POLL_URL = '/api/state';
    const STALE_THRESHOLD_MS = 10000;

    let evtSource = null;
    let pollInterval = null;
    let lastState = {};

    // ── Initialization ──────────────────────────────────────────────────
    function init() {
        connectSSE();
        // Fallback poll every 3 seconds in case SSE is not available
        pollInterval = setInterval(pollState, 3000);
        // Initial fetch
        pollState();
    }

    // ── SSE Connection ──────────────────────────────────────────────────
    function connectSSE() {
        if (evtSource) {
            evtSource.close();
        }

        evtSource = new EventSource(SSE_URL);

        evtSource.addEventListener('state_update', function (e) {
            try {
                const state = JSON.parse(e.data);
                updateDashboard(state);
                setConnectionStatus(true);
            } catch (err) {
                console.warn('SSE parse error:', err);
            }
        });

        evtSource.onopen = function () {
            setConnectionStatus(true);
        };

        evtSource.onerror = function () {
            setConnectionStatus(false);
            // Reconnect after 5 seconds
            setTimeout(connectSSE, 5000);
        };
    }

    // ── Polling Fallback ────────────────────────────────────────────────
    function pollState() {
        fetch(POLL_URL)
            .then(function (r) { return r.json(); })
            .then(function (state) {
                updateDashboard(state);
                setConnectionStatus(true);
            })
            .catch(function () {
                setConnectionStatus(false);
            });
    }

    // ── Connection Status ───────────────────────────────────────────────
    function setConnectionStatus(connected) {
        var el = document.getElementById('connectionStatus');
        var textEl = el.querySelector('.status-text');
        if (connected) {
            el.className = 'connection-status connected';
            textEl.textContent = 'Connected';
        } else {
            el.className = 'connection-status disconnected';
            textEl.textContent = 'Disconnected';
        }
    }

    // ── Main Render ─────────────────────────────────────────────────────
    function updateDashboard(state) {
        lastState = state;
        renderRobots(state.robots || {});
        renderSafetyDecisions(state.safety_decisions || {});
        renderDiagnostics(state.diagnostics || '');
        updateSummary(state);
        updateAlerts(state);
    }

    // ── Summary Bar ─────────────────────────────────────────────────────
    function updateSummary(state) {
        var robots = state.robots || {};
        var fleetNames = {};
        var alertCount = 0;

        Object.values(robots).forEach(function (r) {
            if (r.fleet_name) fleetNames[r.fleet_name] = true;
        });

        var decisions = state.safety_decisions || {};
        Object.values(decisions).forEach(function (d) {
            if (d.risk_level === 'CRITICAL' || d.risk_level === 'WARNING') {
                alertCount++;
            }
        });

        document.getElementById('totalRobots').textContent = Object.keys(robots).length;
        document.getElementById('totalFleets').textContent = Object.keys(fleetNames).length;
        document.getElementById('activeAlerts').textContent = alertCount;
    }

    // ── Alert Banner ────────────────────────────────────────────────────
    function updateAlerts(state) {
        var banner = document.getElementById('alertBanner');
        var content = document.getElementById('alertContent');
        var criticals = [];

        var decisions = state.safety_decisions || {};
        Object.values(decisions).forEach(function (d) {
            if (d.risk_level === 'CRITICAL') {
                criticals.push('⚠ ' + d.robot_id + ': ' + d.reason);
            }
        });

        if (criticals.length > 0) {
            banner.style.display = 'block';
            content.textContent = criticals.join(' │ ');
        } else {
            banner.style.display = 'none';
        }
    }

    // ── Robot Cards ─────────────────────────────────────────────────────
    function renderRobots(robots) {
        var grid = document.getElementById('robotGrid');
        var robotIds = Object.keys(robots).sort();

        if (robotIds.length === 0) {
            grid.innerHTML = '<div class="empty-state"><p>Waiting for robot telemetry...</p><div class="pulse-ring"></div></div>';
            return;
        }

        var html = '';
        robotIds.forEach(function (id) {
            var r = robots[id];
            var status = (r.status || 'IDLE').toUpperCase();
            var cardClass = 'robot-card';
            if (status === 'YIELDING') cardClass += ' yielding';
            if (status === 'CRITICAL') cardClass += ' critical';

            var badgeClass = 'robot-status-badge ';
            if (status === 'MOVING') badgeClass += 'badge-moving';
            else if (status === 'YIELDING') badgeClass += 'badge-yielding';
            else if (status === 'CAUTION') badgeClass += 'badge-caution';
            else badgeClass += 'badge-idle';

            var battery = r.battery_percentage != null ? r.battery_percentage : 100;
            var batteryClass = battery > 50 ? 'battery-high' : (battery > 20 ? 'battery-mid' : 'battery-low');

            var x = r.x != null ? r.x.toFixed(2) : '—';
            var y = r.y != null ? r.y.toFixed(2) : '—';
            var vel = r.velocity != null ? r.velocity.toFixed(2) : '—';
            var task = r.current_task_id || 'None';
            var waitingFor = r.waiting_for_id || '';
            var fleet = r.fleet_name || '—';

            html += '<div class="' + cardClass + '">';
            html += '  <div class="robot-header">';
            html += '    <div><div class="robot-name">' + escapeHtml(id) + '</div>';
            html += '    <div class="robot-fleet">Fleet: ' + escapeHtml(fleet) + '</div></div>';
            html += '    <span class="' + badgeClass + '">' + status + '</span>';
            html += '  </div>';
            html += '  <div class="robot-metrics">';
            html += '    <div class="metric"><span class="metric-label">Position</span><span class="metric-value">(' + x + ', ' + y + ')</span></div>';
            html += '    <div class="metric"><span class="metric-label">Velocity</span><span class="metric-value">' + vel + ' m/s</span></div>';
            html += '    <div class="metric"><span class="metric-label">Battery</span><span class="metric-value">' + battery.toFixed(1) + '%</span></div>';
            html += '    <div class="metric"><span class="metric-label">Task</span><span class="metric-value">' + escapeHtml(task) + '</span></div>';

            if (waitingFor) {
                html += '    <div class="metric"><span class="metric-label">Waiting For</span><span class="metric-value" style="color: var(--accent-amber);">' + escapeHtml(waitingFor) + '</span></div>';
            }

            html += '  </div>';
            html += '  <div class="battery-bar"><div class="battery-fill ' + batteryClass + '" style="width: ' + Math.max(battery, 2) + '%;"></div></div>';
            html += '</div>';
        });

        grid.innerHTML = html;
    }

    // ── Safety Decisions ────────────────────────────────────────────────
    function renderSafetyDecisions(decisions) {
        var list = document.getElementById('safetyList');
        var ids = Object.keys(decisions).sort();

        if (ids.length === 0) {
            list.innerHTML = '<div class="empty-state-small"><p>No active safety decisions</p></div>';
            return;
        }

        var html = '';
        ids.forEach(function (id) {
            var d = decisions[id];
            var riskClass = 'safety-item risk-' + (d.risk_level || 'safe').toLowerCase();

            html += '<div class="' + riskClass + '">';
            html += '  <div class="safety-robot">' + escapeHtml(d.robot_id || id) + '</div>';
            html += '  <div class="safety-decision">' + escapeHtml(d.decision || '—') + ' → ' + escapeHtml(d.target_peer || 'none') + ' [' + (d.risk_level || '—') + ']</div>';
            if (d.reason) {
                html += '  <div class="safety-reason">' + escapeHtml(d.reason) + '</div>';
            }
            html += '</div>';
        });

        list.innerHTML = html;
    }

    // ── Diagnostics ─────────────────────────────────────────────────────
    function renderDiagnostics(text) {
        var el = document.getElementById('diagnosticsOutput');
        if (text) {
            el.textContent = text;
        }
    }

    // ── Utility ─────────────────────────────────────────────────────────
    function escapeHtml(str) {
        var div = document.createElement('div');
        div.appendChild(document.createTextNode(str));
        return div.innerHTML;
    }

    // ── Boot ────────────────────────────────────────────────────────────
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
