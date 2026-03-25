/* ═══════════════════════════════════════════════════════════
   SRD 통합 관제 센터 - web_ui.js
   Firebase + ROS Bridge + web_video_server
═══════════════════════════════════════════════════════════ */

// ── Firebase 초기화 ──────────────────────────────────────
const firebaseConfig = {
  apiKey: "AIzaSyBoWy1kWi5lwMnRVs2MeOS96udqWhe-MJU",
  authDomain: "rokey-1f8f3.firebaseapp.com",
  databaseURL: "https://rokey-1f8f3-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "rokey-1f8f3",
  storageBucket: "rokey-1f8f3.firebasestorage.app",
  messagingSenderId: "405786322615",
  appId: "1:405786322615:web:b75b9bcd4207a4eda60f32"
};

firebase.initializeApp(firebaseConfig);
const auth = firebase.auth();

// 페이지 진입 시 로그인 여부 확인 - 미로그인이면 login.html로 강제 이동
auth.onAuthStateChanged(user => {
  if (!user) window.location.href = 'login.html';
});

// Firebase 로그아웃 후 login.html로 이동
function doLogout() {
  auth.signOut().then(() => {
    window.location.href = 'login.html';
  });
}

// ── ROS Bridge ───────────────────────────────────────────
let ros = null;
let rosTopics = []; // 구독 중인 토픽 목록 (연결 해제 시 일괄 unsubscribe용)

// ROS Bridge WebSocket 연결
// 연결 성공 시 subscribeRosTopics() 호출하여 모든 토픽 구독 시작
function connectRosBridge() {
  const url      = document.getElementById('ros-url').value.trim();
  const statusEl = document.getElementById('ros-status');

  if (ros) { ros.close(); ros = null; }

  statusEl.style.color = '#6b7280';
  statusEl.textContent = '연결 중...';

  ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    document.getElementById('ros-badge').textContent  = 'ROS 온라인';
    document.getElementById('ros-badge').className    = 'badge badge-online';
    statusEl.style.color = '#059669';
    statusEl.textContent = '✓ 연결됨';
    subscribeRosTopics();
    addLog('ROS Bridge 연결됨', 'success');
  });

  ros.on('error', () => {
    statusEl.style.color = '#dc2626';
    statusEl.textContent = '✗ 연결 실패';
    addLog('ROS Bridge 연결 실패', 'danger');
  });

  // 연결 해제 시 구독 중인 모든 토픽 unsubscribe
  ros.on('close', () => {
    document.getElementById('ros-badge').textContent = 'ROS 오프라인';
    document.getElementById('ros-badge').className   = 'badge badge-offline';
    statusEl.style.color = '#6b7280';
    statusEl.textContent = '연결 끊김';
    rosTopics.forEach(t => t.unsubscribe());
    rosTopics = [];
  });
}

// ROS Bridge 연결 후 필요한 모든 토픽을 한번에 구독
function subscribeRosTopics() {
  // 배터리 상태 구독 - robot5, robot6 각각 updateBattery()로 UI 갱신
  [{ name: '/robot5/battery_state', num: 5 },
   { name: '/robot6/battery_state', num: 6 }].forEach(({ name, num }) => {
    const bat = new ROSLIB.Topic({ ros, name, messageType: 'sensor_msgs/BatteryState' });
    bat.subscribe(msg => updateBattery(num, msg));
    rosTopics.push(bat);
  });

  // 도킹 상태 구독 - is_docked 값으로 '도킹 중' / '주행 중' 표시
  [{ name: '/robot5/dock_status', num: 5 },
   { name: '/robot6/dock_status', num: 6 }].forEach(({ name, num }) => {
    const dock = new ROSLIB.Topic({ ros, name, messageType: 'irobot_create_msgs/DockStatus' });
    dock.subscribe(msg => updateDock(num, msg));
    rosTopics.push(dock);
  });

  // 오도메트리 구독 - linear.x 속도를 m/s 단위로 표시
  [{ name: '/robot5/odom', num: 5 },
   { name: '/robot6/odom', num: 6 }].forEach(({ name, num }) => {
    const odom = new ROSLIB.Topic({ ros, name, messageType: 'nav_msgs/Odometry' });
    odom.subscribe(msg => updateOdom(num, msg));
    rosTopics.push(odom);
  });

  // HAZARD MAP 구독 - OccupancyGrid 수신 시 Canvas에 렌더링
  const mapTopic = new ROSLIB.Topic({
    ros,
    name: '/robot5/map',
    messageType: 'nav_msgs/OccupancyGrid'
  });
  mapTopic.subscribe(msg => renderMap(msg));
  rosTopics.push(mapTopic);
}

// ── Map Render ────────────────────────────────────────────
// OccupancyGrid 데이터를 Canvas에 픽셀 단위로 렌더링
// ROS 좌표계(원점 좌하단)를 Canvas 좌표계(원점 좌상단)로 y축 반전 처리
// val -1: 미탐색(회색), 0: 자유공간(흰색), 1~100: 장애물(검정)
function renderMap(msg) {
  const canvas = document.getElementById('map-canvas');
  const placeholder = document.getElementById('map-placeholder');
  if (!canvas) return;

  const { width, height } = msg.info;
  canvas.width  = width;
  canvas.height = height;

  const ctx = canvas.getContext('2d');
  const imageData = ctx.createImageData(width, height);

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      // ROS: 원점 좌하단 → Canvas: 원점 좌상단 (y 반전)
      const rosIdx    = (height - 1 - y) * width + x;
      const canvasIdx = (y * width + x) * 4;
      const val = msg.data[rosIdx];

      let r, g, b;
      if (val === -1)      { r = g = b = 180; }  // 미탐색 - 회색
      else if (val === 0)  { r = g = b = 255; }  // 자유공간 - 흰색
      else                 { r = g = b = Math.max(0, 255 - Math.round(val * 2.55)); } // 장애물 - 검정

      imageData.data[canvasIdx]     = r;
      imageData.data[canvasIdx + 1] = g;
      imageData.data[canvasIdx + 2] = b;
      imageData.data[canvasIdx + 3] = 255;
    }
  }

  ctx.putImageData(imageData, 0, 0);
  canvas.style.display = 'block';
  if (placeholder) placeholder.style.display = 'none';
}

// ── Battery ───────────────────────────────────────────────
// 배터리 퍼센트/전압 UI 갱신
// 20% 이하: 빨간색(low), 50% 이하: 노란색(mid), 그 이상: 기본색
function updateBattery(num, msg) {
  const pct     = Math.round((msg.percentage ?? 0) * 100);
  const voltage = msg.voltage != null ? msg.voltage.toFixed(1) + 'V' : '-';

  document.getElementById(`battery-pct-${num}`).textContent = `${pct} %`;

  const bar = document.getElementById(`battery-bar-${num}`);
  bar.style.width = `${pct}%`;
  bar.className   = 'battery-bar-fill' + (pct <= 20 ? ' low' : pct <= 50 ? ' mid' : '');

  document.getElementById(`battery-info-${num}`).textContent = voltage;
}

// ── Dock Status ───────────────────────────────────────────
// is_docked: true → '도킹 중'(초록), false → '주행 중'(파랑)
function updateDock(num, msg) {
  const el = document.getElementById(`dock-status-${num}`);
  if (!el) return;
  if (msg.is_docked) {
    el.textContent = '도킹 중';
    el.className = 'meta-item dock-badge docked';
  } else {
    el.textContent = '주행 중';
    el.className = 'meta-item dock-badge driving';
  }
}

// ── Odom Speed ────────────────────────────────────────────
// twist.twist.linear.x의 절댓값을 속도(m/s)로 표시
function updateOdom(num, msg) {
  const el = document.getElementById(`odom-speed-${num}`);
  if (!el) return;
  const linear = msg.twist.twist.linear.x;
  el.textContent = `${Math.abs(linear).toFixed(2)} m/s`;
}

// ── 카메라 스트림 ────────────────────────────────────────
// CAM_CONFIGS: 카메라별 토픽명과 web_video_server 스트림 타입 정의
// type 'mjpeg'        : raw sensor_msgs/Image 토픽 직접 구독
// type 'ros_compressed': 베이스 토픽에 /compressed를 붙여 CompressedImage 구독
const CAM_CONFIGS = [
  { label: 'Robot A',       topic: '/robot5/oakd/rgb/image_raw', type: 'ros_compressed' },
  { label: 'Robot B',       topic: '/robot6/oakd/rgb/image_raw', type: 'ros_compressed' },
  { label: 'CAM01 Overlay', topic: '/cam01/overlay',             type: 'ros_compressed' },
  { label: 'CAM02 Overlay', topic: '/cam02/overlay',             type: 'ros_compressed' },
];

const camRetryTimers = {}; // 카메라별 재시도 타이머 ID 저장

// 개별 카메라 스트림 연결 (num: 2~5, HTML 버튼의 connectCam(num)과 대응)
// web_video_server의 /stream 엔드포인트에 img.src를 연결
// 연결 실패 시 5초 후 자동 재시도
function connectCam(num) {
  if (camRetryTimers[num]) {
    clearTimeout(camRetryTimers[num]);
    delete camRetryTimers[num];
  }

  const cam  = CAM_CONFIGS[num];              // num: 0~3, 배열 인덱스 직접 사용
  const base = document.getElementById('video-url').value.trim().replace(/\/$/, '');
  const pad  = String(num + 2).padStart(2, '0'); // DOM ID: cam-slot-02~05

  const img         = document.getElementById(`cam-img-${pad}`);
  const slot        = document.getElementById(`cam-slot-${pad}`);
  const placeholder = slot.querySelector('.cam-empty');
  const statusEl    = document.getElementById(`cam-status-${pad}`);
  const dotEl       = document.getElementById(`cam-dot-${pad}`);

  const url = `${base}/stream?topic=${cam.topic}&type=${cam.type}`;

  setCamDot(dotEl, 'connecting');
  statusEl.textContent = '연결 중...';

  img.src = '';
  img.src = url;

  img.onload = () => {
    img.classList.add('active');
    if (placeholder) placeholder.style.display = 'none';
    setCamDot(dotEl, 'connected');
    statusEl.textContent = '● LIVE';
    statusEl.style.color = '#059669';
    addLog(`${cam.label} 스트림 연결됨`, 'success');
  };

  img.onerror = () => {
    img.classList.remove('active');
    if (placeholder) placeholder.style.display = 'block';
    setCamDot(dotEl, 'error');
    statusEl.textContent = '재시도 중...';
    statusEl.style.color = '#d97706';
    camRetryTimers[num] = setTimeout(() => connectCam(num), 5000);
  };
}

// 카메라 연결 상태 표시 도트 색상 변경
// connected: 초록, connecting: 노랑, error: 빨강
function setCamDot(el, state) {
  if (!el) return;
  el.className = 'btn-cam-dot';
  if (state === 'connected')  el.classList.add('dot-live');
  if (state === 'connecting') el.classList.add('dot-wait');
  if (state === 'error')      el.classList.add('dot-err');
}

// ── Incident Log ─────────────────────────────────────────
let alertCount = 0;

// INCIDENT LOG에 항목 추가 (최대 50개)
// type: 'info'(기본) | 'success'(초록) | 'danger'(빨강)
// danger 타입은 헤더 ALERTS 카운터도 증가
function addLog(message, type = 'info') {
  const logEl = document.getElementById('incident-log');
  const placeholder = logEl.querySelector('.empty-text-sm');
  if (placeholder) placeholder.remove();

  const time = new Date().toLocaleTimeString('ko-KR');
  const item = document.createElement('div');
  item.className = `log-item ${type}`;
  item.innerHTML = `<div>${message}</div><div class="log-time">${time}</div>`;
  logEl.prepend(item);

  const items = logEl.querySelectorAll('.log-item');
  if (items.length > 50) items[items.length - 1].remove();

  if (type === 'danger') {
    alertCount++;
    document.getElementById('alerts-count').textContent = alertCount;
  }
}

// INCIDENT LOG 전체 초기화 및 ALERTS 카운터 리셋
function clearLog() {
  document.getElementById('incident-log').innerHTML = '<span class="empty-text-sm">대기 중...</span>';
  alertCount = 0;
  document.getElementById('alerts-count').textContent = '0';
}
