// Placeholder for dashboard JavaScript
document.addEventListener("DOMContentLoaded", function() {
    console.log("Dashboard loaded.");

    const takeoffBtn = document.getElementById("takeoff-btn");
    const landBtn = document.getElementById("land-btn");
    const emergencyBtn = document.getElementById("emergency-btn");
    const connectionStatus = document.getElementById("connection-status");
    const batteryText = document.getElementById("battery-text");
    const batteryBar = document.getElementById("battery-bar");
    const altitude = document.getElementById("altitude");
    const speed = document.getElementById("speed");
    const flightMode = document.getElementById("flight-mode");
    const dronePosition = document.getElementById("drone-position");

    let ws;

    function connect() {
        ws = new WebSocket("ws://" + window.location.host + "/ws");

        ws.onopen = function(event) {
            console.log("WebSocket connected.");
            connectionStatus.innerHTML = '<span class="status-indicator online"></span><span>Connected</span>';
        };

        ws.onmessage = function(event) {
            const message = JSON.parse(event.data);
            if (message.type === 'drone_status') {
                update_telemetry(message.data);
            } else {
                console.log("Received message:", message);
            }
        };

        ws.onclose = function(event) {
            console.log("WebSocket disconnected. Reconnecting in 3 seconds...");
            connectionStatus.innerHTML = '<span class="status-indicator offline"></span><span>Disconnected</span>';
            setTimeout(connect, 3000);
        };

        ws.onerror = function(error) {
            console.error("WebSocket error:", error);
            ws.close();
        };
    }

    function send_command(action, payload = {}) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            const command = { action, ...payload };
            ws.send(JSON.stringify(command));
        } else {
            console.error("WebSocket is not connected.");
        }
    }

    function update_telemetry(data) {
        batteryText.textContent = data.battery_level.toFixed(1) + "%";
        batteryBar.style.width = data.battery_level + "%";
        altitude.textContent = data.position[2].toFixed(2) + " m";

        const vel = Math.sqrt(data.velocity[0]**2 + data.velocity[1]**2 + data.velocity[2]**2);
        speed.textContent = vel.toFixed(2) + " m/s";

        flightMode.textContent = data.flight_mode;
        dronePosition.textContent = `Position: (${data.position[0].toFixed(2)}, ${data.position[1].toFixed(2)}, ${data.position[2].toFixed(2)})`;
    }

    takeoffBtn.addEventListener("click", () => send_command("takeoff"));
    landBtn.addEventListener("click", () => send_command("land"));
    emergencyBtn.addEventListener("click", () => send_command("emergency_stop"));

    connect();
});
