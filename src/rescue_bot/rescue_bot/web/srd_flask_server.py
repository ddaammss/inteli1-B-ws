from flask import Flask, render_template_string

app = Flask(__name__)

# slam_toolbox 및 TurtleBot 4 환경에 최적화된 ROS 2 연동 라이브 대시보드
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SRD 라이브 구조 대시보드 (SLAM Toolbox)</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/lucide/0.263.0/lucide.min.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@300;400;500;700&display=swap" rel="stylesheet">
    
    <!-- ROS 2 Web 연동 라이브러리 -->
    <script src="https://code.createjs.com/1.0.0/easeljs.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/ros2d@0/build/ros2d.min.js"></script>

    <style>
        body { font-family: 'Noto Sans KR', sans-serif; background-color: #f4f6f8; }
        ::-webkit-scrollbar { width: 6px; }
        ::-webkit-scrollbar-track { background: transparent; }
        ::-webkit-scrollbar-thumb { background: #cbd5e1; border-radius: 6px; }
        
        .pulse-red { animation: pulse-red 2s infinite; }
        @keyframes pulse-red {
            0% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.4); }
            70% { box-shadow: 0 0 0 6px rgba(239, 68, 68, 0); }
            100% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0); }
        }

        #ros-map-container canvas {
            width: 100% !important;
            height: 100% !important;
            object-fit: contain;
            background-color: #e2e8f0;
            border-radius: 0.75rem;
            cursor: grab;
        }
        #ros-map-container canvas:active { cursor: grabbing; }
    </style>
</head>
<body class="min-h-screen p-6 flex flex-col gap-6">

    <!-- 상단 헤더 영역 -->
    <div class="grid grid-cols-12 gap-6 h-[140px]">
        <div class="col-span-8 bg-white rounded-2xl shadow-sm border border-slate-100 p-6 flex items-center justify-between h-full">
            <div>
                <div class="flex items-center gap-2 mb-2">
                    <span class="bg-blue-50 text-blue-600 text-[10px] font-bold px-2 py-1 rounded">TurtleBot 4 / SLAM Toolbox</span>
                    <span id="ros-status-badge" class="bg-red-50 text-red-600 text-[10px] font-bold px-2 py-1 rounded">ROS 오프라인</span>
                </div>
                <h1 class="text-2xl font-bold text-slate-800 mb-1">SRD 실시간 관제 대시보드</h1>
                <p class="text-xs text-slate-500">slam_toolbox 기반 온라인 맵핑 및 요구조자 위급도 분석 모니터링</p>
            </div>
            
            <div class="flex gap-4">
                <div class="flex flex-col items-center justify-center bg-slate-50 rounded-xl px-4 py-2 w-28 border border-slate-100">
                    <span class="text-[10px] text-slate-500 mb-1">ROS 연결</span>
                    <div id="connection-status" class="flex items-center gap-1 font-bold text-red-500 text-sm font-mono">
                        <i data-lucide="wifi-off" class="w-4 h-4"></i> DISCONN
                    </div>
                </div>
                <div class="flex flex-col items-center justify-center bg-red-50 rounded-xl px-4 py-2 w-28 border border-red-100">
                    <span class="text-[10px] text-red-500 mb-1">탐지 생존자</span>
                    <div class="flex items-center gap-1 font-bold text-red-600 text-sm">
                        <i data-lucide="alert-circle" class="w-4 h-4"></i> <span id="survivor-count">0</span> 명
                    </div>
                </div>
            </div>
        </div>

        <div class="col-span-4 bg-white rounded-2xl shadow-sm border border-slate-100 p-5 flex flex-col h-full overflow-hidden">
            <div class="flex items-center gap-2 mb-3 shrink-0">
                <i data-lucide="list-video" class="w-5 h-5 text-slate-400"></i>
                <h2 class="font-bold text-slate-700 text-sm">미션 로그</h2>
            </div>
            <div id="event-log" class="flex-1 overflow-y-auto space-y-1.5 text-[11px] font-mono pr-2 min-h-0">
                <div class="text-slate-400 italic">시스템 기동 대기 중...</div>
            </div>
        </div>
    </div>

    <!-- 하단 메인 영역 -->
    <div class="grid grid-cols-12 gap-6 flex-1 min-h-[500px]">
        <div class="col-span-8 bg-white rounded-2xl shadow-sm border border-slate-100 p-6 flex flex-col relative">
            <div class="flex justify-between items-center mb-4">
                <div class="flex items-center gap-2">
                    <div class="bg-slate-800 p-1.5 rounded-lg text-white"><i data-lucide="map" class="w-4 h-4"></i></div>
                    <div>
                        <h3 class="font-bold text-sm text-slate-800">Online SLAM Live Map</h3>
                        <p class="text-[10px] text-slate-500">slam_toolbox /map 토픽 실시간 동기화</p>
                    </div>
                </div>
                <div class="flex gap-2">
                    <button onclick="resetMapView()" class="bg-slate-100 hover:bg-slate-200 text-slate-600 text-[10px] font-bold px-3 py-1.5 rounded-lg transition-colors flex items-center gap-1">
                        <i data-lucide="maximize" class="w-3 h-3"></i> 시점 초기화
                    </button>
                </div>
            </div>

            <div id="ros-map-container" class="flex-1 bg-slate-200 rounded-xl border border-slate-300 relative overflow-hidden flex items-center justify-center">
                <div id="map-loading" class="absolute flex flex-col items-center text-slate-400 z-10">
                    <i data-lucide="loader-2" class="w-8 h-8 animate-spin mb-2"></i>
                    <span class="text-sm font-bold">SLAM 맵 수신 대기 중...</span>
                </div>
            </div>
        </div>

        <div class="col-span-4 flex flex-col gap-6">
            <div class="bg-white rounded-2xl shadow-sm border border-slate-100 p-5 flex flex-col">
                <h3 class="font-bold text-sm text-slate-800 mb-4">ROS Bridge 통신 설정</h3>
                <div class="space-y-3">
                    <div>
                        <label class="text-[10px] text-slate-500 font-bold block mb-1">WebSocket URL</label>
                        <input type="text" id="ws-url" value="ws://localhost:9090" class="w-full bg-slate-50 border border-slate-200 text-slate-700 text-xs px-3 py-2 rounded-lg focus:outline-none focus:border-blue-400 font-mono">
                    </div>
                    <button id="btn-connect" onclick="connectROS()" class="w-full bg-blue-600 hover:bg-blue-700 text-white font-bold text-xs py-2.5 rounded-lg transition-all duration-200">
                        ROS 2 연결하기
                    </button>
                </div>
            </div>

            <div class="flex-1 bg-white rounded-2xl shadow-sm border border-slate-100 p-5 flex flex-col">
                <div class="flex items-center gap-2 mb-3">
                    <i data-lucide="camera" class="w-4 h-4 text-slate-400"></i>
                    <h3 class="font-bold text-sm text-slate-800">탐지 현황 요약</h3>
                </div>
                <div id="detection-summary" class="flex-1 bg-slate-900 rounded-xl p-4 border border-slate-800 flex flex-col gap-3 overflow-y-auto">
                    <div class="text-slate-500 text-xs italic text-center mt-10">생존자 탐지 시 이곳에<br>상세 정보가 표시됩니다.</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        lucide.createIcons();

        var rosClient = null;
        var viewer = null;
        var gridClient = null;
        var mapGrid = null;
        var survivorCount = 0;

        function logEvent(message, isAlert = false) {
            var logDiv = document.getElementById('event-log');
            var time = new Date().toLocaleTimeString('ko-KR', { hour12: false });
            var entry = document.createElement('div');
            entry.className = isAlert ? 'text-red-500 font-bold border-l-2 border-red-500 pl-2' : 'text-slate-600';
            entry.innerHTML = `[${time}] ${message}`;
            logDiv.prepend(entry);
            if(logDiv.children.length > 50) logDiv.removeChild(logDiv.lastChild);
        }

        function connectROS() {
            var url = document.getElementById('ws-url').value;
            var btn = document.getElementById('btn-connect');
            
            if (!url) return;

            try {
                if (rosClient !== null) {
                    try { rosClient.close(); } catch (e) {}
                }

                btn.innerText = "연결 시도 중...";
                btn.disabled = true;
                btn.classList.add('opacity-70', 'cursor-wait');
                logEvent(`서버(${url}) 연결 시도...`);
                
                rosClient = new ROSLIB.Ros({ url : url });

                rosClient.on('connection', function() {
                    btn.innerText = "ROS 2 연결됨";
                    btn.disabled = false;
                    btn.classList.remove('opacity-70', 'cursor-wait', 'bg-blue-600');
                    btn.classList.add('bg-green-600');
                    logEvent('ROS 2 연결 성공!', true);
                    document.getElementById('connection-status').innerHTML = '<i data-lucide="wifi" class="w-4 h-4"></i> CONNECTED';
                    document.getElementById('connection-status').className = 'flex items-center gap-1 font-bold text-green-600 text-sm font-mono';
                    document.getElementById('ros-status-badge').innerText = '온라인 (Slam Toolbox)';
                    document.getElementById('ros-status-badge').className = 'bg-green-50 text-green-600 text-[10px] font-bold px-2 py-1 rounded';
                    lucide.createIcons();
                    initMap();
                    subscribeAlerts();
                });

                rosClient.on('error', function(error) {
                    btn.innerText = "연결 실패";
                    btn.disabled = false;
                    btn.classList.remove('bg-green-600');
                    btn.classList.add('bg-red-600');
                    logEvent('ROS 2 연결 오류.', true);
                });

                rosClient.on('close', function() {
                    btn.innerText = "ROS 2 연결하기";
                    btn.disabled = false;
                    btn.classList.remove('bg-green-600', 'bg-red-600');
                    btn.classList.add('bg-blue-600');
                    logEvent('ROS 2 연결 종료.');
                    document.getElementById('connection-status').innerHTML = '<i data-lucide="wifi-off" class="w-4 h-4"></i> DISCONN';
                    document.getElementById('connection-status').className = 'flex items-center gap-1 font-bold text-red-500 text-sm font-mono';
                    lucide.createIcons();
                });

            } catch (e) {
                logEvent(`스크립트 오류: ${e.message}`, true);
            }
        }

        function initMap() {
            try {
                var container = document.getElementById('ros-map-container');
                var oldCanvas = container.querySelector('canvas');
                if(oldCanvas) oldCanvas.remove();

                var loading = document.getElementById('map-loading');
                if(loading) loading.style.display = 'flex';

                logEvent('SLAM 맵 뷰어 초기화...');

                viewer = new ROS2D.Viewer({
                    divID : 'ros-map-container',
                    width : container.clientWidth || 800,
                    height : container.clientHeight || 500,
                    background : '#e2e8f0' 
                });

                logEvent('/map 토픽(Transient Local) 구독 중...');

                // slam_toolbox의 특징인 Transient Local 및 Reliable QoS 강제 지정
                var mapListener = new ROSLIB.Topic({
                    ros : rosClient,
                    name : '/map',
                    messageType : 'nav_msgs/msg/OccupancyGrid',
                    qos : { durability: 'transient_local', reliability: 'reliable' }
                });

                mapListener.subscribe(function(message) {
                    if(loading) loading.style.display = 'none';
                    
                    if (mapGrid) {
                        viewer.scene.removeChild(mapGrid);
                    }
                    
                    mapGrid = new ROS2D.OccupancyGrid({ message: message });
                    viewer.scene.addChild(mapGrid);

                    try {
                        if (mapGrid.width > 0) {
                            // 맵 업데이트 시 자동 시점 조정
                            viewer.scaleToDimensions(mapGrid.width, mapGrid.height);
                            viewer.shift(mapGrid.pose.position.x, mapGrid.pose.position.y);
                        }
                    } catch(err) {}
                });

            } catch (e) {
                logEvent('맵 초기화 오류: ' + e.message, true);
            }
        }

        function resetMapView() {
            if(mapGrid && viewer) {
                viewer.scaleToDimensions(mapGrid.width, mapGrid.height);
                viewer.shift(mapGrid.pose.position.x, mapGrid.pose.position.y);
                logEvent('시점 초기화 완료.');
            }
        }

        function subscribeAlerts() {
            var alertListener = new ROSLIB.Topic({
                ros : rosClient,
                name : '/srd/severity_data',
                messageType : 'std_msgs/String'
            });

            alertListener.subscribe(function(message) {
                try {
                    var data = JSON.parse(message.data);
                    var summaryDiv = document.getElementById('detection-summary');
                    
                    // 첫 데이터 수신 시 안내 문구 제거
                    if(survivorCount === 0) summaryDiv.innerHTML = '';

                    var severityColor = "text-green-400";
                    var isCritical = false;

                    if(data.severity === "CRITICAL") {
                        severityColor = "text-red-500 font-bold";
                        isCritical = true;
                        survivorCount++;
                        document.getElementById('survivor-count').innerText = survivorCount;
                        document.body.classList.add('pulse-red');
                        setTimeout(() => document.body.classList.remove('pulse-red'), 3000);
                        logEvent(`🚨 위급 상황 감지! (ID: ${data.track_id})`, true);
                    } else if(data.severity === "WARNING") {
                        severityColor = "text-orange-400";
                    }

                    // 탐지 요약 리스트 추가
                    var item = document.createElement('div');
                    item.className = "bg-slate-800/50 border border-slate-700 rounded-lg p-3 text-[11px]";
                    item.innerHTML = `
                        <div class="flex justify-between mb-1">
                            <span class="text-slate-400">ID: ${data.track_id}</span>
                            <span class="${severityColor}">${data.severity}</span>
                        </div>
                        <div class="text-white mb-1">${data.status_msg}</div>
                        <div class="text-slate-500 font-mono">Emotion: ${data.emotion.toUpperCase()} / Motion: ${data.motion_score.toFixed(1)}</div>
                    `;
                    summaryDiv.prepend(item);
                } catch(e) {}
            });
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

if __name__ == '__main__':
    print("🚀 SRD Flask 서버 시작 (SLAM Toolbox 모드)")
    print("👉 접속 주소: http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)