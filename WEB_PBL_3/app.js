// ================= CẤU HÌNH BÙ TRỪ TỌA ĐỘ (OFFSET) =================
// Offset = Tọa độ trên Web - Tọa độ thực tế (của mạch M10).
// Nếu bạn dùng bản đồ Esri mà thấy đã khớp 100%, hãy đổi 2 số này thành 0.
const OFFSET_LAT = 0.00000023; 
const OFFSET_LNG = 0.00064278; 

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

// THÊM LỚP 4: Esri World Imagery (Ảnh vệ tinh cực nét và chuẩn tọa độ WGS-84 nhất)
// THÊM LỚP 4: Esri World Imagery
const esriImagery = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
  maxZoom: 22,         // Cho phép người dùng lăn chuột zoom tối đa lên mức 22
  maxNativeZoom: 19,   // Tới mức 19 là dừng tải, tự động nội suy phóng to ảnh cũ
  attribution: '© Esri World Imagery'
});

// Khởi tạo bản đồ (Mặc định bật Esri để chuẩn tọa độ nhất)
const map = L.map('map', {
  center: defaultCoords,
  zoom: 18,
  layers: [esriImagery] 
});

// Thêm nút chọn loại bản đồ ở góc trên bên phải bản đồ
const baseMaps = {
  "🌍 Esri Vệ tinh (Chuẩn tọa độ)": esriImagery,
  "🗺️ Google Đường phố": googleStreets,
  "🛰️ Google Vệ tinh": googleHybrid,
  "📐 OpenStreetMap": osmLayer
};
L.control.layers(baseMaps).addTo(map);

// Icon Xe Giao Hàng
const roverIcon = L.divIcon({
  html: `<div id="rover-marker-ui" class="w-8 h-8 rounded-full bg-emerald-500 border-2 border-white shadow-lg flex items-center justify-center text-slate-950 font-bold" style="transition: transform 0.3s ease;">
           <i class="fa-solid fa-location-arrow"></i>
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
  // Tọa độ HÌNH ẢNH (Bị lệch trên Web) để vẽ icon cho đúng chỗ click chuột
  const display_lat = parseFloat(e.latlng.lat.toFixed(8));
  const display_lng = parseFloat(e.latlng.lng.toFixed(8));

  // Tọa độ THỰC TẾ (Đã trừ Offset) để gửi xuống cho xe STM32
  const true_lat = parseFloat((display_lat - OFFSET_LAT).toFixed(8));
  const true_lng = parseFloat((display_lng - OFFSET_LNG).toFixed(8));

  addWaypoint(true_lat, true_lng, display_lat, display_lng, 30); // Mặc định dừng 30 giây
});

function addWaypoint(true_lat, true_lng, display_lat, display_lng, stopTime) {
  const index = waypoints.length + 1;
  // Lưu cả tọa độ thực tế (true) để gửi MQTT và tọa độ hiển thị (display) để vẽ đồ họa
  const wp = { id: index, true_lat, true_lng, display_lat, display_lng, stop: stopTime };
  waypoints.push(wp);

  const markerIcon = L.divIcon({
    html: `<div class="w-6 h-6 rounded-full bg-sky-500 border-2 border-white text-white text-xs font-bold flex items-center justify-center shadow">${index}</div>`,
    className: '',
    iconSize: [24, 24],
    iconAnchor: [12, 12]
  });

  // Vẽ marker dựa trên tọa độ hình ảnh
  const marker = L.marker([display_lat, display_lng], { icon: markerIcon }).addTo(map);
  wpMarkers.push(marker);

  updateMapRoute();
  renderWaypointList();
}

function updateMapRoute() {
  // Vẽ đường Polyline nối các điểm dựa trên tọa độ hình ảnh
  const latlngs = waypoints.map(wp => [wp.display_lat, wp.display_lng]);
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

// Render danh sách Waypoint ở cột bên trái
function renderWaypointList() {
  const container = document.getElementById('wp-list');
  document.getElementById('wp-count').innerText = waypoints.length;

  if (waypoints.length === 0) {
    container.innerHTML = `<p class="text-xs text-slate-500 text-center mt-10">Chưa có điểm giao hàng nào.<br>Hãy click lên bản đồ để thêm điểm.</p>`;
    return;
  }

  container.innerHTML = waypoints.map((wp, idx) => `
    <div class="bg-white border border-slate-200 shadow-sm rounded-lg p-2.5 text-xs flex items-center justify-between mb-2">
      <div>
        <span class="inline-block w-5 h-5 rounded-full bg-sky-100 text-sky-600 font-bold text-center leading-5 mr-1">#${wp.id}</span>
        <!-- Hiển thị tọa độ chuẩn (true) để theo dõi -->
        <span class="font-mono text-slate-600">${wp.true_lat}, ${wp.true_lng}</span>
      </div>
      <div class="flex items-center space-x-2">
        <div class="flex items-center bg-slate-50 border border-slate-200 rounded px-1.5 py-0.5">
          <i class="fa-regular fa-clock text-slate-400 mr-1"></i>
          <input type="number" value="${wp.stop}" min="0" max="3600"
                 onchange="updateStopTime(${idx}, this.value)"
                 class="w-12 bg-transparent text-right font-semibold text-amber-600 focus:outline-none">
          <span class="text-slate-500 ml-1">s</span>
        </div>
        <button onclick="removeWaypoint(${idx})" class="text-slate-400 hover:text-rose-500">
          <i class="fa-solid fa-xmark"></i>
        </button>
      </div>
    </div>
  `).join('');
}


// ================= 3. KẾT NỐI MQTT (HIVEMQ PUBLIC BROKER) =================
let client = null;
const TOPIC_TELEMETRY = "lct_pbl3_rover/telemetry"; 
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


// CẬP NHẬT TELEMETRY TỪ XE GỬI LÊN (ĐÃ ÁP DỤNG OFFSET)
function updateTelemetryUI(data) {
  if (data.lat && data.lng) {
    const stm32_lat = data.lat;
    const stm32_lng = data.lng;

    // Cộng thêm Offset để icon xe không bị trôi xuyên tường trên bản đồ lệch
    const display_lat = stm32_lat + OFFSET_LAT;
    const display_lng = stm32_lng + OFFSET_LNG;

    const newLatLng = [display_lat, display_lng];
    roverMarker.setLatLng(newLatLng);
    map.panTo(newLatLng, {animate: true, duration: 1.0}); 
    
    // UI chữ số vẫn hiển thị tọa độ chuẩn của xe
    document.getElementById('val-gps').innerText = `${stm32_lat.toFixed(8)}, ${stm32_lng.toFixed(8)}`;
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
  
  // Xử lý góc xoay Yaw
  if (data.yaw !== undefined) {
    document.getElementById('val-yaw').innerText = data.yaw.toFixed(1);
    const markerUI = document.getElementById('rover-marker-ui');
    if (markerUI) {
      const yawOffset = -45; 
      const finalAngle = data.yaw + yawOffset; 
      markerUI.style.transform = `rotate(${finalAngle}deg)`;
    }
  }
}

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
  
  // 2. Gửi từng điểm waypoint (GỬI TỌA ĐỘ TRUE_LAT / TRUE_LNG)
  waypoints.forEach((wp, index) => {
      setTimeout(() => {
          publishCommand({
              cmd: "ADD_WP",
              lat: wp.true_lat, // STM32 nhận tọa độ thực đã bù trừ
              lng: wp.true_lng,
              stop: wp.stop
          });
      }, (index + 1) * 500); 
  });
  
  console.log("Đang nạp hành trình xuống xe...");
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



// // ================= 1. KHỞI TẠO BẢN ĐỒ LEAFLET (ĐA LỚP) =================
// // Đặt tâm mặc định ngay giữa sân trường ĐH Bách Khoa (Khu B - Khu F)
// const defaultCoords = [16.0748, 108.1510];

// // Lớp 1: Google Maps Vệ tinh + Tên đường/tòa nhà
// const googleHybrid = L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', {
//   maxZoom: 21,
//   attribution: '© Google Satellite'
// });

// // Lớp 2: Google Maps Đường phố tiêu chuẩn
// const googleStreets = L.tileLayer('https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}', {
//   maxZoom: 21,
//   attribution: '© Google Maps'
// });

// // Lớp 3: OpenStreetMap
// const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
//   maxZoom: 20,
//   attribution: '© OpenStreetMap'
// });

// // Khởi tạo bản đồ (Mặc định bật Google Đường phố)
// const map = L.map('map', {
//   center: defaultCoords,
//   zoom: 18,
//   layers: [googleStreets] 
// });

// // Thêm nút chọn loại bản đồ ở góc trên bên phải bản đồ
// const baseMaps = {
//   "🗺️ Google Đường phố": googleStreets,
//   "🛰️ Google Vệ tinh (Sân trường)": googleHybrid,
//   "📐 OpenStreetMap": osmLayer
// };
// L.control.layers(baseMaps).addTo(map);

// // Icon Xe Giao Hàng
// const roverIcon = L.divIcon({
//   html: `<div id="rover-marker-ui" class="w-8 h-8 rounded-full bg-emerald-500 border-2 border-white shadow-lg flex items-center justify-center text-slate-950 font-bold" style="transition: transform 0.3s ease;">
//            <i class="fa-solid fa-location-arrow"></i>
//          </div>`,
//   className: '',
//   iconSize: [32, 32],
//   iconAnchor: [16, 16]
// });

// let roverMarker = L.marker(defaultCoords, { icon: roverIcon }).addTo(map)
//                    .bindPopup("<b>Xe giao hàng #01</b><br>Đang chờ lệnh");

// // ================= 2. QUẢN LÝ WAYPOINTS =================
// let waypoints = [];
// let wpMarkers = [];
// let polyline = L.polyline([], { color: '#38bdf8', weight: 4, dashArray: '6, 6' }).addTo(map);

// // Sự kiện Click trên bản đồ để thêm Waypoint
// map.on('click', function(e) {
//   const lat = parseFloat(e.latlng.lat.toFixed(8));
//   const lng = parseFloat(e.latlng.lng.toFixed(8));
//   addWaypoint(lat, lng, 30); // Mặc định dừng 30 giây
// });

// function addWaypoint(lat, lng, stopTime) {
//   const index = waypoints.length + 1;
//   const wp = { id: index, lat, lng, stop: stopTime };
//   waypoints.push(wp);

//   const markerIcon = L.divIcon({
//     html: `<div class="w-6 h-6 rounded-full bg-sky-500 border-2 border-white text-white text-xs font-bold flex items-center justify-center shadow">${index}</div>`,
//     className: '',
//     iconSize: [24, 24],
//     iconAnchor: [12, 12]
//   });

//   const marker = L.marker([lat, lng], { icon: markerIcon }).addTo(map);
//   wpMarkers.push(marker);

//   updateMapRoute();
//   renderWaypointList();
// }

// function updateMapRoute() {
//   const latlngs = waypoints.map(wp => [wp.lat, wp.lng]);
//   polyline.setLatLngs(latlngs);
// }

// function updateStopTime(index, value) {
//   waypoints[index].stop = parseInt(value) || 0;
// }

// function removeWaypoint(index) {
//   waypoints.splice(index, 1);
//   map.removeLayer(wpMarkers[index]);
//   wpMarkers.splice(index, 1);

//   waypoints.forEach((wp, i) => wp.id = i + 1);
//   wpMarkers.forEach((m, i) => {
//     m.setIcon(L.divIcon({
//       html: `<div class="w-6 h-6 rounded-full bg-sky-500 border-2 border-white text-white text-xs font-bold flex items-center justify-center shadow">${i + 1}</div>`,
//       className: '',
//       iconSize: [24, 24],
//       iconAnchor: [12, 12]
//     }));
//   });

//   updateMapRoute();
//   renderWaypointList();
// }

// function clearWaypoints() {
//   waypoints = [];
//   wpMarkers.forEach(m => map.removeLayer(m));
//   wpMarkers = [];
//   updateMapRoute();
//   renderWaypointList();
// }

// function renderWaypointList() {
//   const container = document.getElementById('wp-list');
//   document.getElementById('wp-count').innerText = waypoints.length;

//   if (waypoints.length === 0) {
//     container.innerHTML = `<p class="text-xs text-slate-500 text-center mt-10">Chưa có điểm giao hàng nào.<br>Hãy click lên bản đồ để thêm điểm.</p>`;
//     return;
//   }

//   container.innerHTML = waypoints.map((wp, idx) => `
//     <div class="bg-slate-800/80 border border-slate-700 rounded-lg p-2.5 text-xs flex items-center justify-between">
//       <div>
//         <span class="inline-block w-5 h-5 rounded-full bg-sky-500/20 text-sky-400 font-bold text-center leading-5 mr-1">#${wp.id}</span>
//         <span class="font-mono text-slate-300">${wp.lat}, ${wp.lng}</span>
//       </div>
//       <div class="flex items-center space-x-2">
//         <div class="flex items-center bg-slate-900 border border-slate-700 rounded px-1.5 py-0.5">
//           <i class="fa-regular fa-clock text-slate-400 mr-1"></i>
//           <input type="number" value="${wp.stop}" min="0" max="3600"
//                  onchange="updateStopTime(${idx}, this.value)"
//                  class="w-12 bg-transparent text-right font-semibold text-amber-400 focus:outline-none">
//           <span class="text-slate-400 ml-1">s</span>
//         </div>
//         <button onclick="removeWaypoint(${idx})" class="text-slate-500 hover:text-rose-400">
//           <i class="fa-solid fa-xmark"></i>
//         </button>
//       </div>
//     </div>
//   `).join('');
// }

// // ================= 3. KẾT NỐI MQTT (HIVEMQ PUBLIC BROKER) =================
// let client = null;
// const TOPIC_TELEMETRY = "lct_pbl3_rover/telemetry"; // Đã đổi Topic để tránh trùng lặp
// const TOPIC_COMMAND   = "lct_pbl3_rover/command";

// function toggleMqttConfig() {
//   const el = document.getElementById('mqtt-config');
//   el.classList.toggle('hidden');
// }

// function connectMQTT() {
//   const host = document.getElementById('mqtt-host').value.trim();

//   const options = {
//     clientId: 'web_gcs_' + Math.random().toString(16).substring(2, 8),
//     clean: true,
//     connectTimeout: 4000,
//     // Không cần truyền username/password cho Public Broker
//   };

//   if (client) client.end();

//   console.log("Đang kết nối tới HiveMQ Public...", host);
//   client = mqtt.connect(host, options);

//   client.on('connect', () => {
//     const badge = document.getElementById('mqtt-status');
//     badge.className = "px-2.5 py-0.5 text-xs font-medium rounded-full bg-emerald-500/20 text-emerald-400 border border-emerald-500/30";
//     badge.innerHTML = `<i class="fa-solid fa-circle text-[8px] mr-1"></i> MQTT Connected`;
    
//     console.log('✅ Đã kết nối thành công tới HiveMQ Public!');
//     client.subscribe(TOPIC_TELEMETRY);
//   });

//   client.on('message', (topic, message) => {
//     if (topic === TOPIC_TELEMETRY) {
//       try {
//         const data = JSON.parse(message.toString());
//         updateTelemetryUI(data);
//       } catch (e) {
//         console.error("Invalid JSON from STM32:", e);
//       }
//     }
//   });

//   client.on('error', (err) => {
//     console.error('MQTT Error:', err);
//     alert('Lỗi kết nối MQTT. Vui lòng kiểm tra lại mạng hoặc Host URL.');
//   });
// }

// // function updateTelemetryUI(data) {
// //   if (data.lat && data.lng) {
// //     roverMarker.setLatLng([data.lat, data.lng]);
// //     document.getElementById('val-gps').innerText = `${data.lat.toFixed(8)}, ${data.lng.toFixed(8)}`;
// //   }
// //   if (data.bat !== undefined) {
// //     document.getElementById('val-bat').innerText = `${data.bat}%`;
// //     document.getElementById('bar-bat').style.width = `${data.bat}%`;
// //   }
// //   if (data.spd !== undefined) {
// //     document.getElementById('val-speed').innerText = data.spd.toFixed(1);
// //   }
// //   if (data.status) {
// //     document.getElementById('val-status').innerText = data.status;
// //   }
// // }

// // ================= 4. GỬI LỆNH XUỐNG STM32 =================
// function publishCommand(payloadObj) {
//   if (!client || !client.connected) {
//     alert("Chưa kết nối tới MQTT Broker!");
//     return;
//   }
//   const msg = JSON.stringify(payloadObj);
//   client.publish(TOPIC_COMMAND, msg);
//   console.log("Published to STM32:", msg);
// }

// function uploadMission() {
//   if (waypoints.length === 0) {
//     alert("Vui lòng chọn ít nhất 1 điểm giao hàng trên bản đồ!");
//     return;
//   }
  
//   const targetSpeed = parseFloat(document.getElementById('rng-speed').value);
  
//   // 1. Gửi lệnh xóa lộ trình cũ và set tốc độ trước
//   publishCommand({ cmd: "CLEAR_MISSION", speed: targetSpeed });
  
//   // 2. Gửi từng điểm waypoint cách nhau 500ms
//   waypoints.forEach((wp, index) => {
//       setTimeout(() => {
//           publishCommand({
//               cmd: "ADD_WP",
//               lat: wp.lat,
//               lng: wp.lng,
//               stop: wp.stop
//           });
//       }, (index + 1) * 500); 
//   });
  
//   // Thay thế hàm alert() bằng console.log() để không chặn trình duyệt,
//   // hoặc dùng thư viện Toast (thông báo nhỏ góc màn hình) nếu bạn muốn UI đẹp hơn.
//   console.log("Đang nạp hành trình xuống xe...");
// }

// // Bổ sung thêm tính năng trượt camera theo xe vào hàm updateTelemetryUI
// // Bổ sung thêm tính năng trượt camera theo xe vào hàm updateTelemetryUI
// function updateTelemetryUI(data) {
//   if (data.lat && data.lng) {
//     const newLatLng = [data.lat, data.lng];
//     roverMarker.setLatLng(newLatLng);
//     map.panTo(newLatLng, {animate: true, duration: 1.0}); 
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
  
//   // ==========================================
//   // XỬ LÝ YAW - ĐÃ HIỆU CHỈNH GÓC QUAY
//   // ==========================================
//   if (data.yaw !== undefined) {
//     // 1. Cập nhật con số trên thanh Header
//     document.getElementById('val-yaw').innerText = data.yaw.toFixed(1);
    
//     // 2. Xoay icon trên bản đồ theo góc Yaw
//     const markerUI = document.getElementById('rover-marker-ui');
//     if (markerUI) {
      
//       // Bù trừ 45 độ vì icon mặc định bị nghiêng
//       const yawOffset = -45; 
      
//       // Tính toán góc quay cuối cùng
//       // LƯU Ý: Nếu ngoài đời bạn cầm xe xoay sang phải mà mũi tên trên web lại xoay sang trái, 
//       // hãy đổi thành: const finalAngle = -data.yaw + yawOffset;
//       const finalAngle = data.yaw + yawOffset; 

//       markerUI.style.transform = `rotate(${finalAngle}deg)`;
//     }
//   }
// }

// function sendSpeedCommand() {
//   const targetSpeed = parseFloat(document.getElementById('rng-speed').value);
//   publishCommand({
//     cmd: "SET_SPEED",
//     speed: targetSpeed
//   });
// }

// function sendQuickCmd(action) {
//   publishCommand({ cmd: action });
// }

// function renderWaypointList() {
//   const container = document.getElementById('wp-list');
//   document.getElementById('wp-count').innerText = waypoints.length;

//   if (waypoints.length === 0) {
//     container.innerHTML = `<p class="text-xs text-slate-500 text-center mt-10">Chưa có điểm giao hàng nào.<br>Hãy click lên bản đồ để thêm điểm.</p>`;
//     return;
//   }

//   container.innerHTML = waypoints.map((wp, idx) => `
//     <div class="bg-white border border-slate-200 shadow-sm rounded-lg p-2.5 text-xs flex items-center justify-between">
//       <div>
//         <span class="inline-block w-5 h-5 rounded-full bg-sky-100 text-sky-600 font-bold text-center leading-5 mr-1">#${wp.id}</span>
//         <span class="font-mono text-slate-600">${wp.lat}, ${wp.lng}</span>
//       </div>
//       <div class="flex items-center space-x-2">
//         <div class="flex items-center bg-slate-50 border border-slate-200 rounded px-1.5 py-0.5">
//           <i class="fa-regular fa-clock text-slate-400 mr-1"></i>
//           <input type="number" value="${wp.stop}" min="0" max="3600"
//                  onchange="updateStopTime(${idx}, this.value)"
//                  class="w-12 bg-transparent text-right font-semibold text-amber-600 focus:outline-none">
//           <span class="text-slate-500 ml-1">s</span>
//         </div>
//         <button onclick="removeWaypoint(${idx})" class="text-slate-400 hover:text-rose-500">
//           <i class="fa-solid fa-xmark"></i>
//         </button>
//       </div>
//     </div>
//   `).join('');
// }

// window.onload = () => {
//   connectMQTT();
// };