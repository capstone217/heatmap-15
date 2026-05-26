
    const RSSI_MIN = -90;
    const RSSI_MAX = -30;
    const API_BASE = window.location.protocol === "file:"
        ? "http://localhost:8000"
        : `${window.location.protocol}//${window.location.host}`;
    const WS_BASE = window.location.protocol === "file:"
        ? "ws://localhost:8000/ws"
        : `${window.location.protocol === "https:" ? "wss:" : "ws:"}//${window.location.host}/ws`;
    const DEMO_SPEED_TEXT = "22";
    const DEFAULT_GRID_SPACING_M = 0.3;

    let currentBand = "2.4";
    let selectedSsid = "";
    let selectedBssid = "";
    let manualApPosition = null;
    let lastRenderState = {};
    let allDataPoints = [];
    let robotX = 0;
    let robotY = 0;
    let isMoving = false;
    let ws = null;

    function setConnectionState(state, text) {
        const badge = document.getElementById("ws-status");
        const label = document.getElementById("ws-status-text");
        badge.classList.remove("connected", "disconnected", "error");
        badge.classList.add(state);
        label.innerText = text;
    }

    function setCameraOffline() {
        document.getElementById("camera-card").classList.add("offline");
    }

    function setCameraOnline() {
        document.getElementById("camera-card").classList.remove("offline");
    }

    function getAreaSize() {
        const width = parseFloat(document.getElementById("area-x").value);
        const height = parseFloat(document.getElementById("area-y").value);
        return {
            width: Number.isFinite(width) && width > 0 ? width : 3,
            height: Number.isFinite(height) && height > 0 ? height : 3
        };
    }

    function getGridStep(area) {
        if (Math.max(area.width, area.height) <= 3.5) {
            return 0.3;
        }
        if (Math.max(area.width, area.height) <= 6) {
            return 1;
        }
        return 2;
    }

    function getHeatCellPx(transform, area) {
        const targetMeters = Math.min(getGridStep(area) * 0.35, 0.2);
        return Math.max(5, Math.round(targetMeters * transform.scale));
    }

    function canonicalBssid(value) {
        return String(value ?? "").trim().toLowerCase();
    }

    function apMatchesBand(ap) {
        return ap.band && String(ap.band).includes(currentBand);
    }

    function getAvailableSsids() {
        const values = new Set();
        allDataPoints.forEach(point => {
            (point.aps || []).forEach(ap => {
                if (apMatchesBand(ap) && ap.ssid) {
                    values.add(ap.ssid);
                }
            });
        });
        return Array.from(values).sort((a, b) => a.localeCompare(b));
    }

    function getAvailableBssids() {
        const labels = new Map();
        allDataPoints.forEach(point => {
            (point.aps || []).forEach(ap => {
                if (!apMatchesBand(ap) || !ap.bssid) {
                    return;
                }
                const key = canonicalBssid(ap.bssid);
                if (!labels.has(key)) {
                    labels.set(key, `${ap.bssid}${ap.ssid ? ` (${ap.ssid})` : ""}`);
                }
            });
        });
        return Array.from(labels.entries())
            .sort((a, b) => a[1].localeCompare(b[1]))
            .map(([value, label]) => ({ value, label }));
    }

    function setSelectOptions(selectId, options, selectedValue, emptyLabel) {
        const select = document.getElementById(selectId);
        if (!select) {
            return "";
        }

        select.innerHTML = "";
        if (options.length === 0) {
            select.appendChild(new Option(emptyLabel, ""));
            return "";
        }

        options.forEach(option => {
            const value = typeof option === "string" ? option : option.value;
            const label = typeof option === "string" ? option : option.label;
            select.appendChild(new Option(label, value));
        });

        const values = options.map(option => typeof option === "string" ? option : option.value);
        const nextValue = values.includes(selectedValue) ? selectedValue : values[0];
        select.value = nextValue;
        return nextValue;
    }

    function refreshApSelectors() {
        selectedSsid = setSelectOptions("ssid-select", getAvailableSsids(), selectedSsid, "SSID 대기 중");
        selectedBssid = setSelectOptions("bssid-select", getAvailableBssids(), selectedBssid, "MAC 대기 중");
    }

    function connectWebSocket() {
        ws = new WebSocket(WS_BASE);

        ws.onopen = () => setConnectionState("connected", "WebSocket 연결됨");
        ws.onclose = () => setConnectionState("disconnected", "WebSocket 끊김");
        ws.onerror = () => setConnectionState("error", "WebSocket 오류");
        ws.onmessage = handleSocketMessage;
    }

    function handleSocketMessage(event) {
        let data;
        try {
            data = JSON.parse(event.data);
        } catch (error) {
            console.warn("Invalid WebSocket JSON:", event.data);
            return;
        }

        if (!data || data.type !== "state") {
            return;
        }

        const nextX = Number(data.x);
        const nextY = Number(data.y);
        const statusEl = document.getElementById("robot-status");
        const wasMoving = isMoving;
        isMoving = data.status === "running";

        statusEl.innerText = isMoving ? "주행 중" : "대기 중";

        if (Number.isFinite(nextX)) {
            robotX = nextX;
            document.getElementById("pos-x").innerText = robotX.toFixed(2);
        }

        if (Number.isFinite(nextY)) {
            robotY = nextY;
            document.getElementById("pos-y").innerText = robotY.toFixed(2);
        }

        updateUwbDebug(data.uwb);

        if (wasMoving && !isMoving) {
            alert("탐사가 완료되었습니다.");
        }

        if (Array.isArray(data.points)) {
            allDataPoints = data.points
                .map(normalizePoint)
                .filter(Boolean);
            document.getElementById("point-count").innerText = allDataPoints.length;
        } else if (Array.isArray(data.new_points)) {
            data.new_points
                .map(normalizePoint)
                .filter(Boolean)
                .forEach(point => allDataPoints.push(point));
            document.getElementById("point-count").innerText = allDataPoints.length;
        }

        renderCanvas();
    }

    function updateUwbDebug(uwb) {
        const debugEl = document.getElementById("uwb-debug");
        if (!debugEl) {
            return;
        }

        if (!uwb || Object.keys(uwb).length === 0) {
            debugEl.innerText = "UWB 객체 없음";
            setUwbStatus(false, "UWB 객체 없음");
            return;
        }

        const connected = uwb.uwb_connected ? "연결됨" : "연결 안 됨";
        const parsed = uwb.uwb_parse_ok ? "파싱 OK" : "파싱 실패/대기";
        const valid = uwb.uwb_valid ? "좌표 유효" : "좌표 없음";
        const port = uwb.uwb_port || "-";
        const baud = uwb.uwb_baud || "-";
        const anchors = uwb.uwb_anchor_count ?? 0;
        const lastSeen = uwb.uwb_last_seen || "-";
        setUwbStatus(Boolean(uwb.uwb_connected), uwb.uwb_connected ? `UWB 연결됨 (${port})` : "UWB 연결 안 됨");

        debugEl.innerText = `상태: ${connected} / ${parsed} / ${valid} | 포트: ${port} | baud: ${baud} | 앵커: ${anchors} | 마지막: ${lastSeen}`;
    }

    function setUwbStatus(isConnected, text) {
        const badge = document.getElementById("uwb-status");
        const label = document.getElementById("uwb-status-text");
        if (!badge || !label) {
            return;
        }

        badge.classList.remove("connected", "disconnected", "error");
        badge.classList.add(isConnected ? "connected" : "disconnected");
        label.innerText = text;
    }

    function normalizePoint(point) {
        if (!point) {
            return null;
        }

        const x = Number(point.x);
        const y = Number(point.y);

        if (!Number.isFinite(x) || !Number.isFinite(y)) {
            return null;
        }

        const aps = Array.isArray(point.aps)
            ? point.aps
                .map(ap => ({
                    ssid: String(ap.ssid ?? ""),
                    bssid: String(ap.bssid ?? ""),
                    band: String(ap.band ?? ""),
                    rssi: Number(ap.rssi)
                }))
                .filter(ap => Number.isFinite(ap.rssi))
            : [];

        if (Number.isFinite(Number(point.rssi_2g))) {
            aps.push({ ssid: "2.4GHz RSSI", bssid: "", band: "2.4", rssi: Number(point.rssi_2g) });
        }

        if (Number.isFinite(Number(point.rssi_5g))) {
            aps.push({ ssid: "5GHz RSSI", bssid: "", band: "5", rssi: Number(point.rssi_5g) });
        }

        if (Number.isFinite(Number(point.rssi)) && aps.length === 0) {
            aps.push({ ssid: "RSSI", bssid: "", band: currentBand, rssi: Number(point.rssi) });
        }

        return {
            timestamp: point.timestamp ?? Date.now(),
            x,
            y,
            aps
        };
    }

    async function postJson(url, body) {
        const response = await fetch(`${API_BASE}${url}`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: body ? JSON.stringify(body) : undefined
        });

        if (!response.ok) {
            throw new Error(`${response.status} ${response.statusText}`);
        }

        return response.json();
    }

    async function exportCsvServer() {
        try {
            const response = await fetch(`${API_BASE}/api/export_csv`);
            if (!response.ok) {
                throw new Error(`${response.status} ${response.statusText}`);
            }

            const blob = await response.blob();
            const filename = response.headers.get("content-disposition")?.split("filename=")[1] || "wifi_fingerprint.csv";
            const url = URL.createObjectURL(blob);
            const link = document.createElement("a");
            link.href = url;
            link.download = filename.replace(/\"/g, "");
            document.body.appendChild(link);
            link.click();
            link.remove();
            URL.revokeObjectURL(url);
            fetchCsvFiles();
        } catch (error) {
            alert(`서버 CSV 저장 실패: ${error.message}`);
        }
    }

    async function fetchCsvFiles() {
        try {
            const response = await fetch(`${API_BASE}/api/csv_files`);
            if (!response.ok) {
                return;
            }
            const data = await response.json();
            const list = document.getElementById("csv-files");
            if (!Array.isArray(data.files) || data.files.length === 0) {
                list.innerHTML = "저장된 CSV가 없습니다.";
                return;
            }
            list.innerHTML = data.files
                .map(file => `<div><a href="${API_BASE}/saved_csv/${file}" target="_blank">${file}</a></div>`)
                .join("");
        } catch (error) {
            console.warn("CSV 파일 목록 로드 실패:", error);
        }
    }

    async function fetchHeatmapImages() {
        try {
            const response = await fetch(`${API_BASE}/api/heatmap_images`);
            if (!response.ok) {
                return;
            }
            const data = await response.json();
            const list = document.getElementById("heatmap-images");
            if (!Array.isArray(data.files) || data.files.length === 0) {
                list.innerHTML = "저장된 히트맵 이미지가 없습니다.";
                return;
            }
            list.innerHTML = data.files
                .map(file => `<div><a href="${API_BASE}/heatmap_images/${file}" target="_blank">${file}</a></div>`)
                .join("");
        } catch (error) {
            console.warn("히트맵 이미지 목록 로드 실패:", error);
        }
    }

    async function stopAutoMapping() {
        try {
            const result = await postJson("/api/stop_auto", {});
            if (result.status !== "success") {
                throw new Error(result.message || "system error");
            }
        } catch (error) {
            alert(`탐사 중지 실패: ${error.message}`);
        }
    }

    async function saveHeatmapImage() {
        const canvas = document.getElementById("ssidHeatmapCanvas");
        const imageData = canvas.toDataURL("image/png");

        try {
            const response = await fetch(`${API_BASE}/api/save_heatmap_image`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ image_data: imageData })
            });
            const result = await response.json();
            if (result.status !== "success") {
                throw new Error(result.message || "이미지 저장 실패");
            }
            alert(`히트맵 이미지가 저장되었습니다: ${result.filename}`);
            fetchHeatmapImages();
        } catch (error) {
            alert(`히트맵 저장 실패: ${error.message}`);
        }
    }

    async function startAutoMapping() {
        const area = getAreaSize();
        try {
            const result = await postJson("/api/start_auto", { width: area.width, height: area.height });
            if (result.status !== "success") {
                throw new Error(result.message || "system error");
            }
        } catch (error) {
            alert(`탐사 시작 실패: ${error.message}`);
        }
    }

    async function triggerScan() {
        try {
            const result = await postJson("/api/scan");
            if (result.data) {
                const point = normalizePoint(result.data);
                if (point) {
                    allDataPoints.push(point);
                    document.getElementById("point-count").innerText = allDataPoints.length;
                    renderCanvas();
                }
            }
        } catch (error) {
            alert(`수동 스캔 실패: ${error.message}`);
        }
    }

    async function resetCollectedData() {
        if (!confirm("수집된 데이터를 모두 초기화할까요?")) {
            return;
        }

        try {
            const result = await postJson("/api/reset_data", {});
            if (result.status !== "success") {
                throw new Error(result.message || "초기화 실패");
            }
            allDataPoints = [];
            document.getElementById("point-count").innerText = "0";
            document.getElementById("best-spot-info").innerText = "약전계 후보 위치: -";
            document.getElementById("fingerprint-info").innerText = "측정 지점을 클릭하면 해당 좌표의 MAC/RSSI 목록이 표시됩니다.";
            selectedSsid = "";
            selectedBssid = "";
            lastRenderState = {};
            renderCanvas();
        } catch (error) {
            alert(`초기화 실패: ${error.message}`);
        }
    }

    function switchBand(band) {
        currentBand = band;
        document.getElementById("btn-24").classList.toggle("active", band === "2.4");
        document.getElementById("btn-5").classList.toggle("active", band === "5");
        selectedSsid = "";
        selectedBssid = "";
        renderCanvas();
    }

    function setSelectedSsid(value) {
        selectedSsid = value;
        renderCanvas();
    }

    function setSelectedBssid(value) {
        selectedBssid = value;
        renderCanvas();
    }

    function setManualApPosition() {
        const area = getAreaSize();
        const x = Number(document.getElementById("ap-x").value);
        const y = Number(document.getElementById("ap-y").value);
        if (!Number.isFinite(x) || !Number.isFinite(y) || x < 0 || y < 0 || x > area.width || y > area.height) {
            alert("AP 위치가 탐사 구역 안에 오도록 X/Y 값을 입력하세요.");
            return;
        }
        manualApPosition = { x, y };
        renderCanvas();
    }

    function clearManualApPosition() {
        manualApPosition = null;
        document.getElementById("ap-x").value = "";
        document.getElementById("ap-y").value = "";
        renderCanvas();
    }

    function colorForRssi(rssi, alpha = 0.9) {
        const norm = Math.max(0, Math.min(1, (rssi - RSSI_MIN) / (RSSI_MAX - RSSI_MIN)));
        const hue = 240 - norm * 240;
        return `hsla(${hue}, 95%, 48%, ${alpha})`;
    }

    function getSsidSamples() {
        return allDataPoints
            .map(point => {
                const aps = Array.isArray(point.aps)
                    ? point.aps.filter(ap =>
                        apMatchesBand(ap)
                        && ap.ssid === selectedSsid
                        && Number.isFinite(ap.rssi)
                    )
                    : [];

                if (aps.length === 0) {
                    return null;
                }

                const strongest = Math.max(...aps.map(ap => ap.rssi));
                return {
                    x: point.x,
                    y: point.y,
                    rssi: strongest,
                    apCount: aps.length,
                    point
                };
            })
            .filter(Boolean);
    }

    function getBssidSamples() {
        return allDataPoints
            .map(point => {
                const aps = Array.isArray(point.aps)
                    ? point.aps.filter(ap =>
                        apMatchesBand(ap)
                        && canonicalBssid(ap.bssid) === selectedBssid
                        && Number.isFinite(ap.rssi)
                    )
                    : [];

                if (aps.length === 0) {
                    return null;
                }

                return {
                    x: point.x,
                    y: point.y,
                    rssi: aps[0].rssi,
                    apCount: 1,
                    point
                };
            })
            .filter(Boolean);
    }

    function computeWeakLocation(area, samples) {
        if (samples.length < 3) {
            return null;
        }

        let weakest = null;
        const step = Math.min(0.3, Math.max(0.15, Math.max(area.width, area.height) / 20));
        for (let y = 0; y <= area.height; y += step) {
            for (let x = 0; x <= area.width; x += step) {
                const rssi = estimateRssiAt(x, y, samples, area);
                if (rssi === null) {
                    continue;
                }

                if (!weakest || rssi < weakest.rssi) {
                    weakest = { x, y, rssi };
                }
            }
        }

        return weakest;
    }

    function estimateRssiAt(x, y, samples, area) {
        let weightedSum = 0;
        let weightTotal = 0;
        let nearest = Infinity;
        const radius = Math.max(area.width, area.height) * 0.8;

        for (const sample of samples) {
            const dx = x - sample.x;
            const dy = y - sample.y;
            const distance = Math.hypot(dx, dy);
            nearest = Math.min(nearest, distance);

            if (distance < 0.03) {
                return sample.rssi;
            }

            if (distance <= radius) {
                const weight = 1 / Math.pow(distance, 2);
                weightedSum += sample.rssi * weight;
                weightTotal += weight;
            }
        }

        if (weightTotal === 0) {
            return null;
        }

        const confidence = Math.max(0.25, 1 - nearest / radius);
        return weightedSum / weightTotal * confidence + RSSI_MIN * (1 - confidence);
    }

    function buildTransform(canvas, area) {
        const pad = 46;
        const plotWidth = canvas.width - pad * 2;
        const plotHeight = canvas.height - pad * 2;
        const scale = Math.min(plotWidth / area.width, plotHeight / area.height);
        const offsetX = (canvas.width - area.width * scale) / 2;
        const offsetY = (canvas.height - area.height * scale) / 2;

        return {
            pad,
            scale,
            left: offsetX,
            right: offsetX + area.width * scale,
            top: offsetY,
            bottom: offsetY + area.height * scale,
            toX: x => offsetX + x * scale,
            toY: y => offsetY + (area.height - y) * scale,
            toMeterX: px => (px - offsetX) / scale,
            toMeterY: py => area.height - (py - offsetY) / scale
        };
    }

    function drawGrid(ctx, transform, area) {
        ctx.save();
        ctx.strokeStyle = "#e9ecef";
        ctx.lineWidth = 1;
        ctx.font = "11px Consolas, monospace";
        ctx.fillStyle = "#868e96";
        ctx.textAlign = "center";
        ctx.textBaseline = "top";

        const step = getGridStep(area);
        for (let x = 0; x <= area.width + 0.0001; x += step) {
            const px = transform.toX(x);
            ctx.beginPath();
            ctx.moveTo(px, transform.top);
            ctx.lineTo(px, transform.bottom);
            ctx.stroke();
            ctx.fillText(x.toFixed(x % 1 === 0 ? 0 : 1), px, transform.bottom + 8);
        }

        ctx.textAlign = "right";
        ctx.textBaseline = "middle";
        for (let y = 0; y <= area.height + 0.0001; y += step) {
            const py = transform.toY(y);
            ctx.beginPath();
            ctx.moveTo(transform.left, py);
            ctx.lineTo(transform.right, py);
            ctx.stroke();
            ctx.fillText(y.toFixed(y % 1 === 0 ? 0 : 1), transform.left - 8, py);
        }

        ctx.strokeStyle = "#495057";
        ctx.lineWidth = 1.5;
        ctx.strokeRect(transform.left, transform.top, area.width * transform.scale, area.height * transform.scale);
        ctx.restore();
    }

    function drawHeatmap(ctx, transform, area, samples) {
        if (samples.length === 0) {
            ctx.save();
            ctx.fillStyle = "#868e96";
            ctx.font = "bold 15px sans-serif";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText("RSSI 데이터 대기 중", (transform.left + transform.right) / 2, (transform.top + transform.bottom) / 2);
            ctx.restore();
            return { grid: [], cellPx: 0 };
        }

        const cellPx = getHeatCellPx(transform, area);
        const cols = Math.ceil((transform.right - transform.left) / cellPx);
        const rows = Math.ceil((transform.bottom - transform.top) / cellPx);
        const grid = Array.from({ length: rows }, () => Array(cols).fill(null));

        for (let row = 0; row < rows; row++) {
            for (let col = 0; col < cols; col++) {
                const px = transform.left + col * cellPx + cellPx / 2;
                const py = transform.top + row * cellPx + cellPx / 2;
                const mx = transform.toMeterX(px);
                const my = transform.toMeterY(py);

                if (mx < 0 || mx > area.width || my < 0 || my > area.height) {
                    continue;
                }

                const rssi = estimateRssiAt(mx, my, samples, area);
                if (rssi === null) {
                    continue;
                }

                grid[row][col] = rssi;
                ctx.fillStyle = colorForRssi(rssi, 0.68);
                ctx.fillRect(transform.left + col * cellPx, transform.top + row * cellPx, cellPx + 1, cellPx + 1);
            }
        }

        return { grid, cellPx };
    }

    function drawWeakZones(ctx, transform, grid, cellPx) {
        ctx.save();
        ctx.fillStyle = "rgba(0, 0, 0, 0.18)";
        ctx.strokeStyle = "rgba(33, 37, 41, 0.45)";
        ctx.lineWidth = 1;

        for (let row = 0; row < grid.length; row++) {
            for (let col = 0; col < grid[row].length; col++) {
                const rssi = grid[row][col];
                if (rssi === null || rssi > -75) {
                    continue;
                }
                const x = transform.left + col * cellPx;
                const y = transform.top + row * cellPx;
                ctx.fillRect(x, y, cellPx + 1, cellPx + 1);
                ctx.beginPath();
                ctx.moveTo(x, y + cellPx);
                ctx.lineTo(x + cellPx, y);
                ctx.stroke();
            }
        }

        ctx.restore();
    }

    function drawContourLines(ctx, transform, grid, cellPx) {
        const levels = [-80, -70, -60, -50, -40];
        ctx.save();
        ctx.lineWidth = 1.2;
        ctx.globalAlpha = 0.55;

        for (const level of levels) {
            ctx.strokeStyle = colorForRssi(level, 1);
            for (let row = 0; row < grid.length - 1; row++) {
                for (let col = 0; col < grid[row].length - 1; col++) {
                    const here = grid[row][col];
                    const right = grid[row][col + 1];
                    const down = grid[row + 1][col];

                    if (here === null) {
                        continue;
                    }

                    const px = transform.left + col * cellPx;
                    const py = transform.top + row * cellPx;

                    if (right !== null && (here - level) * (right - level) < 0) {
                        ctx.beginPath();
                        ctx.moveTo(px + cellPx, py);
                        ctx.lineTo(px + cellPx, py + cellPx);
                        ctx.stroke();
                    }

                    if (down !== null && (here - level) * (down - level) < 0) {
                        ctx.beginPath();
                        ctx.moveTo(px, py + cellPx);
                        ctx.lineTo(px + cellPx, py + cellPx);
                        ctx.stroke();
                    }
                }
            }
        }

        ctx.restore();
    }

    function drawMeasurementPoints(ctx, transform, samples) {
        ctx.save();
        ctx.font = "bold 9px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";

        samples.forEach(sample => {
            const px = transform.toX(sample.x);
            const py = transform.toY(sample.y);
            ctx.beginPath();
            ctx.arc(px, py, 5.5, 0, Math.PI * 2);
            ctx.fillStyle = "#fff";
            ctx.fill();
            ctx.lineWidth = 2;
            ctx.strokeStyle = colorForRssi(sample.rssi, 1);
            ctx.stroke();

            if (sample.apCount > 0) {
                ctx.fillStyle = "#212529";
                ctx.fillText(sample.apCount, px, py + 0.5);
            }
        });

        ctx.restore();
    }

    function drawManualAp(ctx, transform) {
        if (!manualApPosition) {
            return;
        }

        const px = transform.toX(manualApPosition.x);
        const py = transform.toY(manualApPosition.y);
        ctx.save();
        ctx.fillStyle = "#212529";
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(px, py - 12);
        ctx.lineTo(px + 10, py + 8);
        ctx.lineTo(px - 10, py + 8);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = "#fff";
        ctx.font = "bold 10px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("AP", px, py + 2);
        ctx.restore();
    }

    function drawRobot(ctx, transform) {
        const area = getAreaSize();
        const rx = Math.max(0, Math.min(area.width, robotX));
        const ry = Math.max(0, Math.min(area.height, robotY));
        const px = transform.toX(rx);
        const py = transform.toY(ry);

        ctx.save();
        ctx.beginPath();
        ctx.arc(px, py, 10, 0, Math.PI * 2);
        ctx.fillStyle = "#fd7e14";
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 3;
        ctx.stroke();
        ctx.restore();
    }

    function drawBestSpot(ctx, transform, spot) {
        if (!spot) {
            return;
        }

        const px = transform.toX(spot.x);
        const py = transform.toY(spot.y);
        ctx.save();
        ctx.strokeStyle = "#ffc107";
        ctx.fillStyle = "rgba(255, 193, 7, 0.8)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(px, py, 12, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = "#212529";
        ctx.font = "bold 12px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText("BEST", px, py);
        ctx.restore();
    }

    function renderHeatmap(canvasId, samples, emptyText) {
        const canvas = document.getElementById(canvasId);
        const ctx = canvas.getContext("2d");
        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.parentElement.getBoundingClientRect();

        canvas.width = Math.max(1, Math.floor(rect.width * dpr));
        canvas.height = Math.max(1, Math.floor(rect.height * dpr));
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        const viewCanvas = {
            width: rect.width,
            height: rect.height
        };
        const area = getAreaSize();
        const transform = buildTransform(viewCanvas, area);

        ctx.clearRect(0, 0, rect.width, rect.height);
        const { grid, cellPx } = drawHeatmap(ctx, transform, area, samples);
        if (grid.length > 0) {
            drawWeakZones(ctx, transform, grid, cellPx);
            drawContourLines(ctx, transform, grid, cellPx);
        }
        drawGrid(ctx, transform, area);
        drawMeasurementPoints(ctx, transform, samples);
        drawManualAp(ctx, transform);
        const weakSpot = computeWeakLocation(area, samples);
        if (weakSpot) {
            drawBestSpot(ctx, transform, weakSpot);
        }
        drawRobot(ctx, transform);

        if (samples.length === 0 && emptyText) {
            ctx.save();
            ctx.fillStyle = "#495057";
            ctx.font = "bold 13px sans-serif";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText(emptyText, (transform.left + transform.right) / 2, (transform.top + transform.bottom) / 2 + 22);
            ctx.restore();
        }

        lastRenderState[canvasId] = { transform, samples };
        return { area, grid, cellPx };
    }

    function renderCanvas() {
        refreshApSelectors();
        const area = getAreaSize();
        const ssidSamples = selectedSsid ? getSsidSamples() : [];
        const bssidSamples = selectedBssid ? getBssidSamples() : [];

        document.getElementById("spacing-badge").innerText = `${getGridStep(area).toFixed(getGridStep(area) % 1 === 0 ? 0 : 1)} m`;
        document.getElementById("speed-badge").innerText = DEMO_SPEED_TEXT;

        renderHeatmap("ssidHeatmapCanvas", ssidSamples, "SSID를 선택하세요");
        renderHeatmap("bssidHeatmapCanvas", bssidSamples, "BSSID/MAC을 선택하세요");

        const ssidWeakSpot = computeWeakLocation(area, ssidSamples);
        const bssidWeakSpot = computeWeakLocation(area, bssidSamples);
        const bestSpotLines = [];
        if (ssidWeakSpot) {
            bestSpotLines.push(
                `SSID BEST(${selectedSsid}): X:${ssidWeakSpot.x.toFixed(2)} Y:${ssidWeakSpot.y.toFixed(2)} (예상 RSSI ${ssidWeakSpot.rssi.toFixed(1)} dBm)`
            );
        }
        if (bssidWeakSpot) {
            const bssidLabel = selectedBssid.toUpperCase();
            bestSpotLines.push(
                `BSSID BEST(${bssidLabel}): X:${bssidWeakSpot.x.toFixed(2)} Y:${bssidWeakSpot.y.toFixed(2)} (예상 RSSI ${bssidWeakSpot.rssi.toFixed(1)} dBm)`
            );
        }
        if (bestSpotLines.length > 0) {
            document.getElementById("best-spot-info").innerText = bestSpotLines.join(" / ");
        } else {
            document.getElementById("best-spot-info").innerText = "약전계 후보 위치: -";
        }
    }

    function showFingerprint(point, sourceLabel) {
        const panel = document.getElementById("fingerprint-info");
        const aps = Array.isArray(point.aps)
            ? [...point.aps].sort((a, b) => Number(b.rssi) - Number(a.rssi))
            : [];

        const lines = [
            `${sourceLabel}`,
            `좌표: (${point.x.toFixed(2)}m, ${point.y.toFixed(2)}m)`,
            "",
            "수신 AP 목록:"
        ];

        if (aps.length === 0) {
            lines.push("수신된 AP가 없습니다.");
        } else {
            aps.forEach(ap => {
                const mac = ap.bssid || "(MAC 없음)";
                const ssid = ap.ssid ? ` / ${ap.ssid}` : "";
                const band = ap.band ? ` / ${ap.band}` : "";
                lines.push(`${mac}${ssid}${band} / ${Number(ap.rssi).toFixed(0)} dBm`);
            });
        }

        panel.innerText = lines.join("\n");
    }

    function handleHeatmapClick(canvasId, sourceLabel, event) {
        const state = lastRenderState[canvasId];
        if (!state || !state.transform || !Array.isArray(state.samples)) {
            return;
        }

        const rect = event.currentTarget.getBoundingClientRect();
        const clickX = event.clientX - rect.left;
        const clickY = event.clientY - rect.top;
        let nearest = null;

        state.samples.forEach(sample => {
            const px = state.transform.toX(sample.x);
            const py = state.transform.toY(sample.y);
            const distance = Math.hypot(clickX - px, clickY - py);
            if (!nearest || distance < nearest.distance) {
                nearest = { distance, sample };
            }
        });

        if (!nearest || nearest.distance > 16 || !nearest.sample.point) {
            document.getElementById("fingerprint-info").innerText = "측정 지점을 클릭하면 해당 좌표의 MAC/RSSI 목록이 표시됩니다.";
            return;
        }

        showFingerprint(nearest.sample.point, sourceLabel);
    }

    function csvEscape(value) {
        const text = String(value ?? "");
        return `"${text.replace(/"/g, '""')}"`;
    }

    function downloadCSV() {
        if (allDataPoints.length === 0) {
            alert("저장할 데이터가 없습니다.");
            return;
        }

        const rows = [["Timestamp", "X", "Y", "SSID", "BSSID", "RSSI", "Band", "Interface", "Wi-Fi Type"]];
        allDataPoints.forEach(point => {
            if (!Array.isArray(point.aps)) {
                return;
            }

            point.aps.forEach(ap => {
                const wifiType = ap.interface && (ap.interface.startsWith("wlan") || ap.interface.startsWith("wlx"))
                    ? "Dongle"
                    : ap.band
                        ? "Current AP"
                        : "Unknown";

                rows.push([
                    point.timestamp,
                    point.x,
                    point.y,
                    ap.ssid,
                    ap.bssid,
                    ap.rssi,
                    ap.band,
                    ap.interface || "",
                    wifiType
                ]);
            });
        });

        const csv = rows
            .map(row => row.map(csvEscape).join(","))
            .join("\n");
        const blob = new Blob([`\ufeff${csv}`], { type: "text/csv;charset=utf-8;" });
        const link = document.createElement("a");

        link.href = URL.createObjectURL(blob);
        link.download = "heatmap_data.csv";
        document.body.appendChild(link);
        link.click();
        link.remove();
        URL.revokeObjectURL(link.href);
    }

    function loadDemoData() {
        const area = getAreaSize();
        const points = [];
        const apX = area.width * 0.25;
        const apY = area.height * 0.75;

        for (let y = 0.25; y <= area.height; y += 0.35) {
            for (let x = 0.25; x <= area.width; x += 0.35) {
                const distance = Math.hypot(x - apX, y - apY);
                const rssi24 = Math.max(RSSI_MIN, -38 - distance * 13 + (Math.random() - 0.5) * 6);
                const rssi5 = Math.max(RSSI_MIN, -44 - distance * 18 + (Math.random() - 0.5) * 7);
                points.push(normalizePoint({
                    timestamp: Date.now(),
                    x,
                    y,
                    aps: [
                        { ssid: "Demo-WiFi", bssid: "AA:BB:CC:11:22:33", band: "2.4GHz", rssi: rssi24 },
                        { ssid: "Demo-WiFi", bssid: "AA:BB:CC:44:55:66", band: "2.4GHz", rssi: Math.max(RSSI_MIN, rssi24 - 12 + (Math.random() - 0.5) * 4) },
                        { ssid: "Demo-WiFi-5G", bssid: "12:34:56:78:90:AA", band: "5GHz", rssi: rssi5 }
                    ]
                }));
            }
        }

        allDataPoints = points.filter(Boolean);
        robotX = area.width * 0.5;
        robotY = area.height * 0.5;
        document.getElementById("pos-x").innerText = robotX.toFixed(2);
        document.getElementById("pos-y").innerText = robotY.toFixed(2);
        document.getElementById("point-count").innerText = allDataPoints.length;
        renderCanvas();
        fetchCsvFiles();
    }

    document.getElementById("area-x").addEventListener("input", renderCanvas);
    document.getElementById("area-y").addEventListener("input", renderCanvas);
    document.getElementById("ssidHeatmapCanvas").addEventListener("click", event => {
        handleHeatmapClick("ssidHeatmapCanvas", "SSID 통합 커버리지", event);
    });
    document.getElementById("bssidHeatmapCanvas").addEventListener("click", event => {
        handleHeatmapClick("bssidHeatmapCanvas", "BSSID/MAC RSSI", event);
    });
    window.addEventListener("resize", renderCanvas);
    window.addEventListener("load", () => {
        const cameraStream = document.getElementById("camera-stream");
        setCameraOnline();
        cameraStream.src = `${API_BASE}/api/video_feed?ts=${Date.now()}`;
        connectWebSocket();
        renderCanvas();
        fetchCsvFiles();
        fetchHeatmapImages();
        window.setInterval(renderCanvas, 800);
    });
