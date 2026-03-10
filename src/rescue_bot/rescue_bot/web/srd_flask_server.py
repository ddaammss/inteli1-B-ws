from flask import Flask, render_template_string

app = Flask(__name__)

# 가독성을 높이고 영상 스트림 안정성을 강화한 v3.2 레이아웃
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SRD 통합 관제 시스템 v3.2</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/lucide/0.263.0/lucide.min.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@300;400;500;700&display=swap" rel="stylesheet">
    
    <script src="https://code.createjs.com/1.0.0/easeljs.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/ros2d@0/build/ros2d.min.js"></script>

    <style>
        body { font-family: 'Noto Sans KR', sans-serif; background-color: #f1f5f9; height: 100vh; display: flex; flex-direction: column; overflow: hidden; }
        ::-webkit-scrollbar { width: 8px; height: 8px; }
        ::-webkit-scrollbar-track { background: transparent; }
        ::-webkit-scrollbar-thumb { background: #cbd5e1; border-radius: 6px; }
        ::-webkit-scrollbar-thumb:hover { background: #94a3b8; }
        
        .pulse-red { animation: pulse-red 2s infinite; }
        @keyframes pulse-red {
            0% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.4); }
            70% { box-shadow: 0 0 0 10px rgba(239, 68, 68, 0); }
            100% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0); }
        }

        #ros-map-container canvas {
            width: 100% !important;
            height: 100% !important;
            object-fit: contain;
            background-color: #e2e8f0;
            border-radius: 1rem;
        }
        
        .cam-slot {
            background-color: #0f172a;
            border: 2px solid #1e293b;
            position: relative;
            aspect-ratio: 16 / 9;
            overflow: hidden;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .cam-feed {
            width: 100%;
            height: 100%;
            object-fit: contain;
            display: none;
            background-color: #000;
        }

        .bottom-panel-card {
            background: white;
            border-radius: 1rem;
            border: 1px solid #e2e8f0;
            display: flex;
            flex-direction: column;
            padding: 1.25rem;
            height: 100%;
        }
    </style>
</head>
<body class="p-4 gap-4">

    <!-- [1] 상단 헤더 -->
    <header class="bg-white rounded-2xl shadow-sm border border-slate-200 p-5 flex items-center justify-between">
        <div class="flex items-center gap-5">
            <div class="bg-slate-800 p-3 rounded-xl text-white shadow-lg shadow-slate-200">
                <i data-lucide="shield-alert" class="w-8 h-8"></i>
            </div>
            <div>
                <h1 class="text-2xl font-bold text-slate-800 tracking-tight leading-none mb-2">SRD 통합 관제 센터 <span class="text-slate-400 font-medium ml-2 text-base">v3.2</span></h1>
                <div class="flex items-center gap-3">
                    <span id="ros-status-badge" class="bg-red-50 text-red-600 text-xs font-bold px-3 py-1 rounded border border-red-100 uppercase">ROS 오프라인</span>
                    <p class="text-sm text-slate-500 font-medium uppercase tracking-tight">Active Surveillance Pipeline</p>
                </div>
            </div>
        </div>
        
        <div class="flex gap-8 items-center">
            <div class="flex flex-col items-end">
                <span class="text-xs text-slate-400 font-black uppercase mb-1">Surveillance Status</span>
                <div id="connection-status" class="font-mono text-red-500 text-lg font-black flex items-center gap-1.5">
                    <i data-lucide="wifi-off" class="w-5 h-5"></i> DISCONNECTED
                </div>
            </div>
            <div class="h-10 w-px bg-slate-200"></div>
            <div class="flex flex-col items-center bg-red-50 px-6 py-3 rounded-2xl border border-red-100">
                <span class="text-xs text-red-500 font-black mb-1 uppercase">Alerts</span>
                <div class="flex items-center gap-2 font-black text-red-600 text-3xl leading-none">
                    <i data-lucide="alert-circle" class="w-6 h-6"></i> <span id="survivor-count">0</span>
                </div>
            </div>
        </div>
    </header>

    <!-- [2] 메인 시각화 영역 -->
    <div class="flex-1 grid grid-cols-12 gap-4 min-h-0">
        <!-- 맵 영역 -->
        <div class="col-span-7 bg-white rounded-2xl shadow-sm border border-slate-200 p-5 flex flex-col min-h-0">
            <div class="flex justify-between items-center mb-4">
                <div class="flex items-center gap-3">
                    <div class="bg-blue-600 p-2 rounded-lg text-white"><i data-lucide="map" class="w-5 h-5"></i></div>
                    <h3 class="font-black text-slate-800 text-lg uppercase tracking-wider">Hazard Map</h3>
                </div>
                <button onclick="resetMapView()" class="bg-slate-100 hover:bg-slate-200 text-slate-700 text-xs font-black px-5 py-2.5 rounded-xl transition-all flex items-center gap-2 border border-slate-200">
                    <i data-lucide="refresh-cw" class="w-4 h-4"></i> 시점 초기화
                </button>
            </div>
            <div id="ros-map-container" class="flex-1 bg-slate-200 rounded-2xl border border-slate-300 relative overflow-hidden flex items-center justify-center font-black text-slate-400">
                지도 수신 대기 중...
            </div>
        </div>

        <!-- 카메라 매트릭스 -->
        <div class="col-span-5 bg-white rounded-2xl shadow-sm border border-slate-200 p-5 flex flex-col min-h-0">
            <div class="flex items-center gap-3 mb-4">
                <div class="bg-purple-600 p-2 rounded-lg text-white"><i data-lucide="layout-grid" class="w-5 h-5"></i></div>
                <h3 class="font-black text-slate-800 text-lg uppercase tracking-wider">Live Vision</h3>
            </div>
            <div class="grid grid-cols-2 gap-3 flex-1 min-h-0">
                <div class="cam-slot rounded-2xl shadow-inner overflow-hidden" id="slot-01">
                    <span class="absolute top-3 left-3 bg-blue-600/90 text-[10px] text-white px-2 py-1 rounded-md font-black font-mono z-20 uppercase">CAM 01: ANALYZER</span>
                    <!-- 영상 로드 실패 시 자동 재시도를 위한 onerror 속성 추가 -->
                    <img id="stream-01" class="cam-feed" src="" alt="Stream" onerror="retryStream(this)">
                    <div id="placeholder-01" class="text-slate-700 opacity-20 flex flex-col items-center gap-2">
                        <i data-lucide="video-off" class="w-12 h-12"></i>
                        <span class="text-[10px] font-black uppercase">Waiting for Stream</span>
                    </div>
                </div>
                <div class="cam-slot rounded-2xl shadow-inner"><div class="text-slate-700 opacity-10"><i data-lucide="bot" class="w-12 h-12"></i></div></div>
                <div class="cam-slot rounded-2xl shadow-inner"><div class="text-slate-700 opacity-10"><i data-lucide="video" class="w-12 h-12"></i></div></div>
                <div class="cam-slot rounded-2xl shadow-inner"><div class="text-slate-700 opacity-10"><i data-lucide="video" class="w-12 h-12"></i></div></div>
            </div>
        </div>
    </div>

    <!-- [3] 하단 패널 -->
    <div class="h-[240px] grid grid-cols-12 gap-4 shrink-0">
        <div class="col-span-4 bottom-panel-card overflow-hidden">
            <div class="flex items-center gap-3 mb-3 shrink-0"><i data-lucide="scroll-text" class="w-5 h-5 text-slate-400"></i><h2 class="font-black text-slate-700 text-sm uppercase">Incident Log</h2></div>
            <div id="event-log" class="flex-1 overflow-y-auto space-y-2 text-[12px] font-mono pr-2 min-h-0 text-slate-600">대기 중...</div>
        </div>

        <div class="col-span-4 bottom-panel-card">
            <div class="flex items-center gap-3 mb-4 shrink-0"><i data-lucide="settings-2" class="w-5 h-5 text-slate-400"></i><h2 class="font-black text-slate-700 text-sm uppercase">System Config</h2></div>
            <div class="space-y-4 flex-1">
                <div>
                    <label class="text-[11px] text-slate-400 font-black uppercase mb-1 block">Rosbridge WebSocket</label>
                    <input type="text" id="ws-url" value="ws://localhost:9090" class="w-full bg-slate-50 border border-slate-200 text-slate-800 text-sm px-4 py-3 rounded-xl focus:outline-none focus:border-blue-500 font-mono font-bold shadow-inner">
                </div>
                <button id="btn-connect" onclick="connectROS()" class="w-full bg-blue-600 hover:bg-blue-700 text-white font-black text-sm py-3.5 rounded-xl shadow-lg uppercase tracking-widest transition-all active:scale-95">브릿지 서버 연결</button>
            </div>
        </div>

        <div class="col-span-4 bottom-panel-card overflow-hidden">
            <div class="flex items-center gap-3 mb-3 shrink-0"><i data-lucide="bar-chart-3" class="w-5 h-5 text-slate-400"></i><h2 class="font-black text-slate-700 text-sm uppercase">Detections</h2></div>
            <div id="detection-summary" class="flex-1 overflow-y-auto space-y-3 pr-1 min-h-0"></div>
        </div>
    </div>

    <script>
        lucide.createIcons();
        var rosClient = null, viewer = null, mapGrid = null, survivorCount = 0;
        
        // 동적 스트림 URL 생성 (현재 접속한 IP/호스트 자동 인식 및 MJPEG 타입 명시)
        function getStreamUrl() {
            const host = window.location.hostname;
            return `http://${host}:8080/stream?topic=/srd/processed_image&type=mjpeg&width=640&height=480`;
        }

        // 이미지 로드 실패 시 재시도 로직
        function retryStream(imgElement) {
            console.warn("영상 스트림 수신 실패, 2초 후 재시도...");
            setTimeout(() => {
                imgElement.src = getStreamUrl() + "&t=" + new Date().getTime();
            }, 2000);
        }

        function logEvent(message, isAlert = false) {
            var logDiv = document.getElementById('event-log');
            var time = new Date().toLocaleTimeString('ko-KR', { hour12: false });
            var entry = document.createElement('div');
            entry.className = isAlert ? 'text-red-500 font-black border-l-4 border-red-500 pl-3 py-1 bg-red-50/50 rounded-r' : 'text-slate-600';
            entry.innerHTML = `<span class="opacity-50">[${time}]</span> ${message}`;
            logDiv.prepend(entry);
            if(logDiv.children.length > 100) logDiv.removeChild(logDiv.lastChild);
        }

        function connectROS() {
            var url = document.getElementById('ws-url').value;
            var btn = document.getElementById('btn-connect');
            try {
                if (rosClient) rosClient.close();
                btn.innerText = "Connecting...";
                rosClient = new ROSLIB.Ros({ url : url });
                
                rosClient.on('connection', function() {
                    btn.innerText = "서버 연결됨";
                    btn.className = "w-full bg-green-600 text-white font-black text-sm py-3.5 rounded-xl uppercase";
                    logEvent('브릿지 서버 통신 성공!', true);
                    document.getElementById('connection-status').innerHTML = '<i data-lucide="wifi" class="w-5 h-5"></i> CONNECTED';
                    document.getElementById('connection-status').className = "font-mono text-green-600 text-lg font-black flex items-center gap-1.5";
                    
                    // 영상 스트림 활성화 (동적 주소 적용)
                    const img = document.getElementById('stream-01');
                    const placeholder = document.getElementById('placeholder-01');
                    img.src = getStreamUrl();
                    img.style.display = 'block';
                    placeholder.style.display = 'none';
                    
                    initMap(); subscribeAlerts(); lucide.createIcons();
                });
                rosClient.on('error', function() { logEvent('서버 연결 실패. 네트워크 확인 필요.'); });
                rosClient.on('close', function() { 
                    logEvent('연결이 종료되었습니다.');
                    document.getElementById('stream-01').style.display = 'none';
                    document.getElementById('placeholder-01').style.display = 'flex';
                });
            } catch (e) { logEvent('스크립트 오류: ' + e.message); }
        }

        function initMap() {
            var container = document.getElementById('ros-map-container');
            container.innerHTML = "";
            viewer = new ROS2D.Viewer({ divID: 'ros-map-container', width: container.clientWidth, height: container.clientHeight, background: '#e2e8f0' });
            logEvent('📍 /map 토픽 수신 대기 중...');
            var mapListener = new ROSLIB.Topic({ ros: rosClient, name: '/map', messageType: 'nav_msgs/msg/OccupancyGrid', qos: { durability: 'transient_local', reliability: 'reliable' } });
            mapListener.subscribe(function(message) {
                if (mapGrid) viewer.scene.removeChild(mapGrid);
                mapGrid = new ROS2D.OccupancyGrid({ message: message });
                viewer.scene.addChild(mapGrid);
                if (mapGrid.width > 0) { viewer.scaleToDimensions(mapGrid.width, mapGrid.height); viewer.shift(mapGrid.pose.position.x, mapGrid.pose.position.y); }
            });
        }

        function subscribeAlerts() {
            var alertListener = new ROSLIB.Topic({ ros: rosClient, name: '/srd/severity_data', messageType: 'std_msgs/String' });
            alertListener.subscribe(function(message) {
                try {
                    var data = JSON.parse(message.data);
                    var summaryDiv = document.getElementById('detection-summary');
                    if(data.severity === "CRITICAL") {
                        survivorCount++; document.getElementById('survivor-count').innerText = survivorCount;
                        document.body.classList.add('pulse-red'); setTimeout(() => document.body.classList.remove('pulse-red'), 2000);
                        logEvent(`🚨 [위급] ID ${data.track_id} 요구조자 감지!`, true);
                    }
                    var item = document.createElement('div');
                    item.className = "bg-slate-50 border border-slate-200 rounded-xl p-4 text-[12px] shadow-sm mb-2";
                    item.innerHTML = `<div class="flex justify-between font-black"><span class="text-slate-400">ID: ${data.track_id}</span><span class="text-red-600">${data.severity}</span></div><div class="mt-1 font-bold">Emotion: ${data.emotion.toUpperCase()}</div>`;
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
    app.run(host='0.0.0.0', port=5000, debug=False)