
// ==========================================
// MULTI-TARGET RADAR TRACKING SYSTEM
// KALMAN FILTER + NOISE PERFORMANCE CHART
// ==========================================

// 1. INITIAL TARGET DATA

const targets = [
    {
        id: 101,
        x: 14, y: 24,
        vx: 2, vy: 2,
        speed: 2.83,
        kalman_x: 14, kalman_y: 24,
        kalman_vx: 2, kalman_vy: 2,
        active: true,
        prediction_error: 0,
        average_prediction_error: 0,
        prediction_count: 0,
        average_raw_measurement_error: 0,
        average_kalman_estimation_error: 0,
        error_reduction_percent: 0,
        noise_comparison_count: 0,
        history: [{ x: 14, y: 24 }]
    },
    {
        id: 202,
        x: 48, y: 19,
        vx: -1, vy: 2,
        speed: 2.24,
        kalman_x: 48, kalman_y: 19,
        kalman_vx: -1, kalman_vy: 2,
        active: true,
        prediction_error: 0,
        average_prediction_error: 0,
        prediction_count: 0,
        average_raw_measurement_error: 0,
        average_kalman_estimation_error: 0,
        error_reduction_percent: 0,
        noise_comparison_count: 0,
        history: [{ x: 48, y: 19 }]
    },
    {
        id: 303,
        x: 29, y: 58,
        vx: 2, vy: -1,
        speed: 2.24,
        kalman_x: 29, kalman_y: 58,
        kalman_vx: 2, kalman_vy: -1,
        active: true,
        prediction_error: 0,
        average_prediction_error: 0,
        prediction_count: 0,
        average_raw_measurement_error: 0,
        average_kalman_estimation_error: 0,
        error_reduction_percent: 0,
        noise_comparison_count: 0,
        history: [{ x: 29, y: 58 }]
    }
];

// 2. RADAR CONFIGURATION

const canvas = document.getElementById("radarCanvas");

if (!canvas) {
    throw new Error("Radar canvas not found.");
}

const ctx = canvas.getContext("2d");

const CENTER_X = canvas.width / 2;
const CENTER_Y = canvas.height / 2;
const RADAR_RADIUS = 260;

const MAX_SPEED = 5;
const RADAR_MIN = 0;
const RADAR_MAX = 100;

let scanAngle = 0;
let radarRunning = true;
let animationId = null;
let selectedTarget = "all";

// 3. GET VISIBLE TARGETS

function getVisibleTargets() {
    if (selectedTarget === "all") {
        return targets.filter(target => target.active);
    }

    return targets.filter(target =>
        target.id === Number(selectedTarget) && target.active
    );
}

// 4. TARGET FILTERING

function filterTargets() {
    const dropdown = document.getElementById("targetFilter");

    if (!dropdown) return;

    selectedTarget = dropdown.value;

    const info = document.getElementById("selectedTargetInfo");

    if (info) {
        info.textContent = selectedTarget === "all"
            ? "Showing all available targets."
            : "Currently viewing Target " + selectedTarget;
    }

    updateDashboard();
}

// 5. UPDATE FILTER OPTIONS

function updateFilterOptions() {
    const dropdown = document.getElementById("targetFilter");

    if (!dropdown) return;

    const previousValue = selectedTarget;

    dropdown.innerHTML =
        '<option value="all">Show All Targets</option>';

    targets.forEach(target => {
        const option = document.createElement("option");
        option.value = target.id;
        option.textContent = "Target " + target.id;
        dropdown.appendChild(option);
    });

    const exists = targets.some(
        target => String(target.id) === previousValue
    );

    if (previousValue === "all" || exists) {
        dropdown.value = previousValue;
    } else {
        selectedTarget = "all";
        dropdown.value = "all";
    }

    const info = document.getElementById("selectedTargetInfo");

    if (info) {
        info.textContent = selectedTarget === "all"
            ? "Showing all available targets."
            : "Currently viewing Target " + selectedTarget;
    }
}

// 6. COORDINATE CONVERSION

function radarX(x) {
    return CENTER_X + (x - 50) * 4.5;
}

function radarY(y) {
    return CENTER_Y - (y - 50) * 4.5;
}

// 7. DISTANCE CALCULATION

function calculateDistance(target) {
    return Math.sqrt(
        Math.pow(target.kalman_x - 50, 2) +
        Math.pow(target.kalman_y - 50, 2)
    );
}

// 8. DRAW RADAR BACKGROUND

function drawRadarBackground() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.fillStyle = "#071321";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    ctx.strokeStyle = "#1c8068";
    ctx.lineWidth = 1;

    for (let radius = 65; radius <= RADAR_RADIUS; radius += 65) {
        ctx.beginPath();
        ctx.arc(CENTER_X, CENTER_Y, radius, 0, Math.PI * 2);
        ctx.stroke();
    }

    ctx.beginPath();

    ctx.moveTo(CENTER_X, CENTER_Y - RADAR_RADIUS);
    ctx.lineTo(CENTER_X, CENTER_Y + RADAR_RADIUS);

    ctx.moveTo(CENTER_X - RADAR_RADIUS, CENTER_Y);
    ctx.lineTo(CENTER_X + RADAR_RADIUS, CENTER_Y);

    ctx.stroke();

    ctx.fillStyle = "#42d6a4";
    ctx.beginPath();
    ctx.arc(CENTER_X, CENTER_Y, 4, 0, Math.PI * 2);
    ctx.fill();
}

// 9. RADAR SWEEP

function drawRadarSweep() {
    const sweepX =
        CENTER_X + RADAR_RADIUS * Math.cos(scanAngle);

    const sweepY =
        CENTER_Y + RADAR_RADIUS * Math.sin(scanAngle);

    const gradient = ctx.createLinearGradient(
        CENTER_X, CENTER_Y, sweepX, sweepY
    );

    gradient.addColorStop(0, "rgba(66,214,164,0.5)");
    gradient.addColorStop(1, "rgba(66,214,164,0.05)");

    ctx.strokeStyle = "#42d6a4";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(CENTER_X, CENTER_Y);
    ctx.lineTo(sweepX, sweepY);
    ctx.stroke();

    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.moveTo(CENTER_X, CENTER_Y);

    ctx.arc(
        CENTER_X,
        CENTER_Y,
        RADAR_RADIUS,
        scanAngle - 0.15,
        scanAngle
    );

    ctx.closePath();
    ctx.fill();
}

// 10. TARGET HISTORY

function drawTargetHistory(target) {
    if (!target.history || target.history.length < 2) return;

    ctx.strokeStyle = "rgba(66,214,164,0.65)";
    ctx.lineWidth = 2;
    ctx.beginPath();

    target.history.forEach((point, index) => {
        const x = radarX(point.x);
        const y = radarY(point.y);

        if (index === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    });

    ctx.stroke();
}

// 11. PREDICTED POSITIONS

function drawPredictedPositions() {
    const input = document.getElementById("predictionTime");

    if (!input) return;

    const time = Number(input.value);

    if (!Number.isFinite(time) || time < 0) return;

    getVisibleTargets().forEach(target => {
        const futureX =
            target.kalman_x + target.kalman_vx * time;

        const futureY =
            target.kalman_y + target.kalman_vy * time;

        ctx.strokeStyle = "#ff8c42";
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);

        ctx.beginPath();

        ctx.moveTo(
            radarX(target.kalman_x),
            radarY(target.kalman_y)
        );

        ctx.lineTo(radarX(futureX), radarY(futureY));
        ctx.stroke();

        ctx.setLineDash([]);

        ctx.fillStyle = "#ff8c42";
        ctx.beginPath();
        ctx.arc(radarX(futureX), radarY(futureY), 5, 0, Math.PI * 2);
        ctx.fill();
    });
}

// 12. DRAW TARGETS

function drawTargets() {
    getVisibleTargets().forEach(target => {
        drawTargetHistory(target);

        // Measured position
        ctx.fillStyle = "#ffcf55";
        ctx.beginPath();
        ctx.arc(radarX(target.x), radarY(target.y), 4, 0, Math.PI * 2);
        ctx.fill();

        // Kalman position
        ctx.fillStyle = "#42d6a4";
        ctx.beginPath();
        ctx.arc(
            radarX(target.kalman_x),
            radarY(target.kalman_y),
            6,
            0,
            Math.PI * 2
        );
        ctx.fill();

        ctx.fillStyle = "#ffffff";
        ctx.font = "13px Arial";

        ctx.fillText(
            "ID " + target.id,
            radarX(target.kalman_x) + 10,
            radarY(target.kalman_y) - 10
        );
    });
}

// 13. MAIN RADAR DRAWING

function drawRadar() {
    drawRadarBackground();
    drawRadarSweep();
    drawPredictedPositions();
    drawTargets();

    if (radarRunning) {
        scanAngle += 0.015;
    }

    scanAngle %= Math.PI * 2;
}

// 14. ANIMATION

function animateRadar() {
    drawRadar();

    if (radarRunning) {
        animationId = requestAnimationFrame(animateRadar);
    }
}

// 15. RADAR CONTROLS

function startRadar() {
    if (radarRunning) return;

    radarRunning = true;
    animateRadar();
}

function stopRadar() {
    radarRunning = false;

    if (animationId !== null) {
        cancelAnimationFrame(animationId);
        animationId = null;
    }

    drawRadar();
}

function resetRadar() {
    scanAngle = 0;
    drawRadar();
}

// 16. TARGET TRACKING TABLE

function updateTargetTable() {
    const table = document.getElementById("targetTable");

    if (!table) return;

    table.innerHTML = "";

    getVisibleTargets().forEach(target => {
        const row = document.createElement("tr");

        row.innerHTML = `
            <td>${target.id}</td>
            <td>${target.x.toFixed(2)}</td>
            <td>${target.y.toFixed(2)}</td>
            <td>${target.kalman_x.toFixed(2)}</td>
            <td>${target.kalman_y.toFixed(2)}</td>
            <td>${target.speed.toFixed(2)}</td>
            <td>${calculateDistance(target).toFixed(2)}</td>
            <td>${target.active ? "Active" : "Inactive"}</td>
        `;

        table.appendChild(row);
    });

    document.getElementById("totalTargets").textContent = targets.length;

    document.getElementById("activeTargets").textContent =
        targets.filter(target => target.active).length;
}

// 17. KALMAN FILTER TABLE

function updateKalmanTable() {
    const table = document.getElementById("kalmanTable");

    if (!table) return;

    table.innerHTML = "";

    getVisibleTargets().forEach(target => {
        const row = document.createElement("tr");

        row.innerHTML = `
            <td>${target.id}</td>
            <td>${target.kalman_vx.toFixed(2)}</td>
            <td>${target.kalman_vy.toFixed(2)}</td>
            <td>${target.kalman_x.toFixed(2)}</td>
            <td>${target.kalman_y.toFixed(2)}</td>
        `;

        table.appendChild(row);
    });
}

// 18. NOISE PERFORMANCE TABLE

function updateNoiseComparisonTable() {
    const table = document.getElementById("noiseComparisonTable");

    if (!table) return;

    table.innerHTML = "";

    const visibleTargets = getVisibleTargets();

    if (visibleTargets.length === 0) {
        table.innerHTML =
            '<tr><td colspan="5">No active targets available.</td></tr>';
        return;
    }

    visibleTargets.forEach(target => {
        const row = document.createElement("tr");

        row.innerHTML = `
            <td>${target.id}</td>
            <td>${target.average_raw_measurement_error.toFixed(2)}</td>
            <td>${target.average_kalman_estimation_error.toFixed(2)}</td>
            <td>${target.error_reduction_percent.toFixed(2)}%</td>
            <td>${target.noise_comparison_count}</td>
        `;

        table.appendChild(row);
    });
}

// 19. LIVE PERFORMANCE COMPARISON CHART

function drawNoiseComparisonChart() {
    const chart = document.getElementById("noiseComparisonChart");

    if (!chart) return;

    const chartCtx = chart.getContext("2d");
    const width = chart.width;
    const height = chart.height;

    chartCtx.clearRect(0, 0, width, height);

    chartCtx.fillStyle = "#ffffff";
    chartCtx.fillRect(0, 0, width, height);

    const visibleTargets = getVisibleTargets().filter(
        target => target.noise_comparison_count > 0
    );

    if (visibleTargets.length === 0) {
        chartCtx.fillStyle = "#555555";
        chartCtx.font = "18px Arial";
        chartCtx.textAlign = "center";
        chartCtx.fillText(
            "Run measurement noise simulation to display chart",
            width / 2,
            height / 2
        );
        return;
    }

    const left = 75;
    const right = 35;
    const top = 40;
    const bottom = 75;

    const plotWidth = width - left - right;
    const plotHeight = height - top - bottom;

    const maxError = Math.max(
        1,
        ...visibleTargets.map(target =>
            Math.max(
                target.average_raw_measurement_error,
                target.average_kalman_estimation_error
            )
        )
    );

    const chartMax = Math.ceil(maxError * 1.2 * 2) / 2;
    const step = chartMax / 5;

    // Horizontal grid lines and Y-axis labels
    chartCtx.strokeStyle = "#e3e8ee";
    chartCtx.lineWidth = 1;

    chartCtx.font = "13px Arial";
    chartCtx.fillStyle = "#344054";
    chartCtx.textAlign = "right";

    for (let i = 0; i <= 5; i++) {
        const value = i * step;
        const y = top + plotHeight - (value / chartMax) * plotHeight;

        chartCtx.beginPath();
        chartCtx.moveTo(left, y);
        chartCtx.lineTo(width - right, y);
        chartCtx.stroke();

        chartCtx.fillText(value.toFixed(1), left - 10, y + 4);
    }

    // Axes
    chartCtx.strokeStyle = "#344054";
    chartCtx.lineWidth = 2;

    chartCtx.beginPath();
    chartCtx.moveTo(left, top);
    chartCtx.lineTo(left, top + plotHeight);
    chartCtx.lineTo(width - right, top + plotHeight);
    chartCtx.stroke();

    const groupWidth = plotWidth / visibleTargets.length;
    const barWidth = Math.min(45, groupWidth * 0.27);

    visibleTargets.forEach((target, index) => {
        const centerX = left + groupWidth * (index + 0.5);

        const rawHeight =
            (target.average_raw_measurement_error / chartMax) *
            plotHeight;

        const kalmanHeight =
            (target.average_kalman_estimation_error / chartMax) *
            plotHeight;

        const rawX = centerX - barWidth - 4;
        const kalmanX = centerX + 4;

        const baseY = top + plotHeight;

        // Raw measurement error bar
        chartCtx.fillStyle = "#e5a33d";

        chartCtx.fillRect(
            rawX,
            baseY - rawHeight,
            barWidth,
            rawHeight
        );

        // Kalman estimation error bar
        chartCtx.fillStyle = "#269b79";

        chartCtx.fillRect(
            kalmanX,
            baseY - kalmanHeight,
            barWidth,
            kalmanHeight
        );

        // Values above bars
        chartCtx.font = "12px Arial";
        chartCtx.fillStyle = "#263238";
        chartCtx.textAlign = "center";

        chartCtx.fillText(
            target.average_raw_measurement_error.toFixed(2),
            rawX + barWidth / 2,
            baseY - rawHeight - 8
        );

        chartCtx.fillText(
            target.average_kalman_estimation_error.toFixed(2),
            kalmanX + barWidth / 2,
            baseY - kalmanHeight - 8
        );

        // Target labels
        chartCtx.font = "14px Arial";
        chartCtx.fillText(
            "Target " + target.id,
            centerX,
            baseY + 25
        );
    });

    // Axis titles
    chartCtx.save();

    chartCtx.translate(20, top + plotHeight / 2);
    chartCtx.rotate(-Math.PI / 2);

    chartCtx.font = "14px Arial";
    chartCtx.textAlign = "center";
    chartCtx.fillStyle = "#344054";

    chartCtx.fillText("Average Position Error (units)", 0, 0);

    chartCtx.restore();

    chartCtx.font = "14px Arial";
    chartCtx.textAlign = "center";
    chartCtx.fillStyle = "#344054";

    chartCtx.fillText(
        "Targets",
        left + plotWidth / 2,
        height - 12
    );
}

// 20. TRACKING ACCURACY TABLE

function updateAccuracyTable() {
    const table = document.getElementById("accuracyTable");

    if (!table) return;

    table.innerHTML = "";

    getVisibleTargets().forEach(target => {
        const row = document.createElement("tr");

        row.innerHTML = `
            <td>${target.id}</td>
            <td>${target.prediction_error.toFixed(2)}</td>
            <td>${target.average_prediction_error.toFixed(2)}</td>
            <td>${target.prediction_count}</td>
        `;

        table.appendChild(row);
    });
}

// 21. FUTURE POSITION PREDICTION

function predictTargets() {
    const input = document.getElementById("predictionTime");
    const results = document.getElementById("predictionResults");

    if (!input || !results) return;

    const time = Number(input.value);

    if (!Number.isFinite(time) || time < 0) {
        results.textContent = "Enter a valid non-negative time.";
        return;
    }

    const visibleTargets = getVisibleTargets();

    results.innerHTML =
        "<strong>Kalman-Based Predicted Positions</strong><br>";

    if (visibleTargets.length === 0) {
        results.innerHTML += "No active targets available.";
        return;
    }

    visibleTargets.forEach(target => {
        const futureX =
            target.kalman_x + target.kalman_vx * time;

        const futureY =
            target.kalman_y + target.kalman_vy * time;

        results.innerHTML += `
            Target ${target.id}:
            X = ${futureX.toFixed(2)},
            Y = ${futureY.toFixed(2)}<br>
        `;
    });
}

// 22. TARGET MONITORING ALERTS

function detectTargetAlerts() {
    const alertSummary = document.getElementById("alertSummary");
    const alertList = document.getElementById("alertList");

    if (!alertSummary || !alertList) return;

    const alerts = [];

    targets.forEach(target => {
        const speed = Math.sqrt(
            target.kalman_vx ** 2 +
            target.kalman_vy ** 2
        );

        if (target.active && speed > MAX_SPEED) {
            alerts.push({
                type: "warning",
                message:
                    "HIGH-SPEED ALERT: Target " +
                    target.id + " is moving at " +
                    speed.toFixed(2) + " units/second."
            });
        }

        if (
            target.active &&
            (
                target.kalman_x < RADAR_MIN ||
                target.kalman_x > RADAR_MAX ||
                target.kalman_y < RADAR_MIN ||
                target.kalman_y > RADAR_MAX
            )
        ) {
            alerts.push({
                type: "danger",
                message:
                    "BOUNDARY ALERT: Target " +
                    target.id + " is outside the radar area."
            });
        }

        if (!target.active) {
            alerts.push({
                type: "danger",
                message:
                    "INACTIVE ALERT: Target " +
                    target.id + " is inactive."
            });
        }
    });

    if (alerts.length === 0) {
        alertSummary.innerHTML =
            "<p>System monitoring is active. No critical alerts.</p>";

        alertList.innerHTML =
            '<div class="alert-item alert-success">' +
            "No alerts detected. All targets are within normal limits." +
            "</div>";

        return;
    }

    alertSummary.innerHTML =
        "<p>" + alerts.length + " alert(s) detected.</p>";

    alertList.innerHTML = "";

    alerts.forEach(alert => {
        const element = document.createElement("div");

        element.className = "alert-item alert-" + alert.type;
        element.textContent = alert.message;

        alertList.appendChild(element);
    });
}

// 23. REFRESH ALL DASHBOARD SECTIONS

function updateDashboard() {
    updateTargetTable();
    updateKalmanTable();
    updateNoiseComparisonTable();
    updateAccuracyTable();
    drawNoiseComparisonChart();
    predictTargets();
    detectTargetAlerts();
    drawRadar();
}

// 24. LOAD C BACKEND JSON

async function loadCTargetData() {
    try {
        const response = await fetch(
            "./targets.json?t=" + Date.now(),
            { cache: "no-store" }
        );

        if (!response.ok) {
            throw new Error("Unable to load dashboard/targets.json");
        }

        const data = await response.json();

        if (!Array.isArray(data.targets)) {
            throw new Error("Invalid JSON target data.");
        }

        data.targets.forEach(newTarget => {
            const id = Number(newTarget.id);

            const existingTarget = targets.find(
                target => target.id === id
            );

            if (existingTarget) {

                if (
                    existingTarget.x !== Number(newTarget.x) ||
                    existingTarget.y !== Number(newTarget.y)
                ) {
                    existingTarget.history.push({
                        x: Number(newTarget.x),
                        y: Number(newTarget.y)
                    });

                    if (existingTarget.history.length > 30) {
                        existingTarget.history.shift();
                    }
                }

                existingTarget.x = Number(newTarget.x);
                existingTarget.y = Number(newTarget.y);

                existingTarget.vx = Number(newTarget.vx);
                existingTarget.vy = Number(newTarget.vy);
                existingTarget.speed = Number(newTarget.speed);

                existingTarget.kalman_x =
                    Number(newTarget.kalman_x ?? newTarget.x);

                existingTarget.kalman_y =
                    Number(newTarget.kalman_y ?? newTarget.y);

                existingTarget.kalman_vx =
                    Number(newTarget.kalman_vx ?? newTarget.vx);

                existingTarget.kalman_vy =
                    Number(newTarget.kalman_vy ?? newTarget.vy);

                existingTarget.active = Boolean(newTarget.active);

                existingTarget.prediction_error =
                    Number(newTarget.prediction_error || 0);

                existingTarget.average_prediction_error =
                    Number(newTarget.average_prediction_error || 0);

                existingTarget.prediction_count =
                    Number(newTarget.prediction_count || 0);

                existingTarget.average_raw_measurement_error =
                    Number(newTarget.average_raw_measurement_error || 0);

                existingTarget.average_kalman_estimation_error =
                    Number(newTarget.average_kalman_estimation_error || 0);

                existingTarget.error_reduction_percent =
                    Number(newTarget.error_reduction_percent || 0);

                existingTarget.noise_comparison_count =
                    Number(newTarget.noise_comparison_count || 0);

            } else {

                targets.push({
                    id: id,

                    x: Number(newTarget.x),
                    y: Number(newTarget.y),

                    vx: Number(newTarget.vx),
                    vy: Number(newTarget.vy),

                    speed: Number(newTarget.speed),

                    kalman_x:
                        Number(newTarget.kalman_x ?? newTarget.x),

                    kalman_y:
                        Number(newTarget.kalman_y ?? newTarget.y),

                    kalman_vx:
                        Number(newTarget.kalman_vx ?? newTarget.vx),

                    kalman_vy:
                        Number(newTarget.kalman_vy ?? newTarget.vy),

                    active: Boolean(newTarget.active),

                    prediction_error:
                        Number(newTarget.prediction_error || 0),

                    average_prediction_error:
                        Number(newTarget.average_prediction_error || 0),

                    prediction_count:
                        Number(newTarget.prediction_count || 0),

                    average_raw_measurement_error:
                        Number(newTarget.average_raw_measurement_error || 0),

                    average_kalman_estimation_error:
                        Number(newTarget.average_kalman_estimation_error || 0),

                    error_reduction_percent:
                        Number(newTarget.error_reduction_percent || 0),

                    noise_comparison_count:
                        Number(newTarget.noise_comparison_count || 0),

                    history: [{
                        x: Number(newTarget.x),
                        y: Number(newTarget.y)
                    }]
                });
            }
        });

        // Remove targets no longer present in JSON
        const currentIDs = new Set(
            data.targets.map(target => Number(target.id))
        );

        for (let i = targets.length - 1; i >= 0; i--) {
            if (!currentIDs.has(targets[i].id)) {
                targets.splice(i, 1);
            }
        }

        updateFilterOptions();
        updateDashboard();

        console.log(
            "C tracking, Kalman and performance chart data loaded successfully."
        );

    } catch (error) {
        console.error("JSON loading error:", error);
    }
}

// 25. INITIALIZE DASHBOARD

updateFilterOptions();
updateDashboard();
animateRadar();

loadCTargetData();

// 26. AUTOMATIC JSON SYNCHRONIZATION

setInterval(loadCTargetData, 1000);

// 27. PREDICTION INPUT

const predictionInput =
    document.getElementById("predictionTime");

if (predictionInput) {
    predictionInput.addEventListener("input", () => {
        predictTargets();
        drawRadar();
    });
}
