// ================= 1. KHỞI TẠO BẢN ĐỒ LEAFLET (ĐA LỚP) =================
// Đặt tâm mặc định ngay giữa sân trường ĐH Bách Khoa (Khu B - Khu F)
const defaultCoords = [16.0748, 108.1510];

// Lớp 1: Google Maps Vệ tinh + Tên đường/tòa nhà
const googleHybrid = L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', {
  maxZoom: 21,
  attribution: '© Google Satellite'
});

// Lớp 2: Google Maps Đường phố tiêu chuẩn
const googleStreets = L.tileLayer('https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}', {
  maxZoom: 21,
  attribution: '© Google Maps'
});

// Lớp 3: OpenStreetMap
const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 20,
  attribution: '© OpenStreetMap'
});

// Khởi tạo bản đồ (Mặc định bật Google Đường phố)
const map = L.map('map', {
  center: defaultCoords,
  zoom: 18,
  layers: [googleStreets] 
});

// Thêm nút chọn loại bản đồ ở góc trên bên phải bản đồ
const baseMaps = {
  "🗺️ Google Đường phố": googleStreets,
  "🛰️ Google Vệ tinh (Sân trường)": googleHybrid,
  "📐 OpenStreetMap": osmLayer
};
L.control.layers(baseMaps).addTo(map);

// Icon Xe Giao Hàng
const roverIcon = L.divIcon({
  html: `<div class="w-8 h-8 rounded-full bg-emerald-500 border-2 border-white shadow-lg flex items-center justify-center text-slate-950 font-bold">
           <i class="fa-solid fa-truck-fast"></i>
         </div>`,
  className: '',
  iconSize: [32, 32],
  iconAnchor: [16, 16]
});

let roverMarker = L.marker(defaultCoords, { icon: roverIcon }).addTo(map)
                   .bindPopup("<b>Xe giao hàng #01</b><br>Đang chờ lệnh");

// ================= 2. QUẢN LÝ WAYPOINTS =================
let waypoints = [];
let wpMarkers = [];
let polyline = L.polyline([], { color: '#38bdf8', weight: 4, dashArray: '6, 6' }).addTo(map);

// Sự kiện Click trên bản đồ để thêm Waypoint
map.on('click', function(e) {
  const lat = parseFloat(e.latlng.lat.toFixed(8));
  const lng = parseFloat(e.latlng.lng.toFixed(8));
  addWaypoint(lat, lng, 30); // Mặc định dừng 30 giây
});

function addWaypoint(lat, lng, stopTime) {
  const index = waypoints.length + 1;
  const wp = { id: index, lat, lng, stop: stopTime };
  waypoints.push(wp);

  const markerIcon = L.divIcon({
    html: `<div class="w-6 h-6 rounded-full bg-sky-500 border-2 border-white text-white text-xs font-bold flex items-center justify-center shadow">${index}</div>`,
    className: '',
    iconSize: [24, 24],
    iconAnchor: [12, 12]
  });

  const marker = L.marker([lat, lng], { icon: markerIcon }).addTo(map);
  wpMarkers.push(marker);

  updateMapRoute();
  renderWaypointList();
}

function updateMapRoute() {
  const latlngs = waypoints.map(wp => [wp.lat, wp.lng]);
  polyline.setLatLngs(latlngs);
}

function updateStopTime(index, value) {
  waypoints[index].stop = parseInt(value) || 0;
}

function removeWaypoint(index) {
  waypoints.splice(index, 1);
  map.removeLayer(wpMarkers[index]);
  wpMarkers.splice(index, 1);

  waypoints.forEach((wp, i) => wp.id = i + 1);
  wpMarkers.forEach((m, i) => {
    m.setIcon(L.divIcon({
      html: `<div class="w-6 h-6 rounded-full bg-sky-500 border-2 border-white text-white text-xs font-bold flex items-center justify-center shadow">${i + 1}</div>`,
      className: '',
      iconSize: [24, 24],
      iconAnchor: [12, 12]
    }));
  });

  updateMapRoute();
  renderWaypointList();
}

function clearWaypoints() {
  waypoints = [];
  wpMarkers.forEach(m => map.removeLayer(m));
  wpMarkers = [];
  updateMapRoute();
  renderWaypointList();
}

function renderWaypointList() {
  const container = document.getElementById('wp-list');
  document.getElementById('wp-count').innerText = waypoints.length;

  if (waypoints.length === 0) {
    container.innerHTML = `<p class="text-xs text-slate-500 text-center mt-10">Chưa có điểm giao hàng nào.<br>Hãy click lên bản đồ để thêm điểm.</p>`;
    return;
  }

  container.innerHTML = waypoints.map((wp, idx) => `
    <div class="bg-slate-800/80 border border-slate-700 rounded-lg p-2.5 text-xs flex items-center justify-between">
      <div>
        <span class="inline-block w-5 h-5 rounded-full bg-sky-500/20 text-sky-400 font-bold text-center leading-5 mr-1">#${wp.id}</span>
        <span class="font-mono text-slate-300">${wp.lat}, ${wp.lng}</span>
      </div>
      <div class="flex items-center space-x-2">
        <div class="flex items-center bg-slate-900 border border-slate-700 rounded px-1.5 py-0.5">
          <i class="fa-regular fa-clock text-slate-400 mr-1"></i>
          <input type="number" value="${wp.stop}" min="0" max="3600"
                 onchange="updateStopTime(${idx}, this.value)"
                 class="w-12 bg-transparent text-right font-semibold text-amber-400 focus:outline-none">
          <span class="text-slate-400 ml-1">s</span>
        </div>
        <button onclick="removeWaypoint(${idx})" class="text-slate-500 hover:text-rose-400">
          <i class="fa-solid fa-xmark"></i>
        </button>
      </div>
    </div>
  `).join('');
}

// ================= 3. KẾT NỐI MQTT (HIVEMQ PUBLIC BROKER) =================
let client = null;
const TOPIC_TELEMETRY = "lct_pbl3_rover/telemetry"; // Đã đổi Topic để tránh trùng lặp
const TOPIC_COMMAND   = "lct_pbl3_rover/command";

function toggleMqttConfig() {
  const el = document.getElementById('mqtt-config');
  el.classList.toggle('hidden');
}

function connectMQTT() {
  const host = document.getElementById('mqtt-host').value.trim();

  const options = {
    clientId: 'web_gcs_' + Math.random().toString(16).substring(2, 8),
    clean: true,
    connectTimeout: 4000,
    // Không cần truyền username/password cho Public Broker
  };

  if (client) client.end();

  console.log("Đang kết nối tới HiveMQ Public...", host);
  client = mqtt.connect(host, options);

  client.on('connect', () => {
    const badge = document.getElementById('mqtt-status');
    badge.className = "px-2.5 py-0.5 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400 border border-emerald-500/30";
    badge.innerHTML = `<i class="fa-solid fa-circle text-[8px] mr-1"></i> MQTT Connected`;
    
    console.log('✅ Đã kết nối thành công tới HiveMQ Public!');
    client.subscribe(TOPIC_TELEMETRY);
  });

  client.on('message', (topic, message) => {
    if (topic === TOPIC_TELEMETRY) {
      try {
        const data = JSON.parse(message.toString());
        updateTelemetryUI(data);
      } catch (e) {
        console.error("Invalid JSON from STM32:", e);
      }
    }
  });

  client.on('error', (err) => {
    console.error('MQTT Error:', err);
    alert('Lỗi kết nối MQTT. Vui lòng kiểm tra lại mạng hoặc Host URL.');
  });
}

// function updateTelemetryUI(data) {
//   if (data.lat && data.lng) {
//     roverMarker.setLatLng([data.lat, data.lng]);
//     document.getElementById('val-gps').innerText = `${data.lat.toFixed(8)}, ${data.lng.toFixed(8)}`;
//   }
//   if (data.bat !== undefined) {
//     document.getElementById('val-bat').innerText = `${data.bat}%`;
//     document.getElementById('bar-bat').style.width = `${data.bat}%`;
//   }
//   if (data.spd !== undefined) {
//     document.getElementById('val-speed').innerText = data.spd.toFixed(1);
//   }
//   if (data.status) {
//     document.getElementById('val-status').innerText = data.status;
//   }
// }

// ================= 4. GỬI LỆNH XUỐNG STM32 =================
function publishCommand(payloadObj) {
  if (!client || !client.connected) {
    alert("Chưa kết nối tới MQTT Broker!");
    return;
  }
  const msg = JSON.stringify(payloadObj);
  client.publish(TOPIC_COMMAND, msg);
  console.log("Published to STM32:", msg);
}

function uploadMission() {
  if (waypoints.length === 0) {
    alert("Vui lòng chọn ít nhất 1 điểm giao hàng trên bản đồ!");
    return;
  }
  
  const targetSpeed = parseFloat(document.getElementById('rng-speed').value);
  
  // 1. Gửi lệnh xóa lộ trình cũ và set tốc độ trước
  publishCommand({ cmd: "CLEAR_MISSION", speed: targetSpeed });
  
  // 2. Gửi từng điểm waypoint cách nhau 500ms để STM32 kịp xử lý
  waypoints.forEach((wp, index) => {
      setTimeout(() => {
          publishCommand({
              cmd: "ADD_WP",
              lat: wp.lat,
              lng: wp.lng,
              stop: wp.stop
          });
      }, (index + 1) * 500); 
  });
  
  alert("Đang nạp hành trình xuống xe. Vui lòng đợi vài giây!");
}

// Bổ sung thêm tính năng trượt camera theo xe vào hàm updateTelemetryUI
function updateTelemetryUI(data) {
  if (data.lat && data.lng) {
    const newLatLng = [data.lat, data.lng];
    roverMarker.setLatLng(newLatLng);
    
    // Tự động dịch chuyển camera bản đồ đi theo xe
    map.panTo(newLatLng, { animate: true, duration: 1.0 });

    document.getElementById('val-gps').innerText = `${data.lat.toFixed(8)}, ${data.lng.toFixed(8)}`;
  }
  if (data.bat !== undefined) {
    document.getElementById('val-bat').innerText = `${data.bat}%`;
    document.getElementById('bar-bat').style.width = `${data.bat}%`;
  }
  if (data.spd !== undefined) {
    document.getElementById('val-speed').innerText = data.spd.toFixed(1);
  }
  if (data.status) {
    document.getElementById('val-status').innerText = data.status;
  }
}

function sendSpeedCommand() {
  const targetSpeed = parseFloat(document.getElementById('rng-speed').value);
  publishCommand({
    cmd: "SET_SPEED",
    speed: targetSpeed
  });
}

function sendQuickCmd(action) {
  publishCommand({ cmd: action });
}

window.onload = () => {
  connectMQTT();
};