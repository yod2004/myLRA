const HEADER_MANUAL = 0x01, HEADER_AUTO = 0x02, HEADER_PARAM = 0x03, HEADER_MANUAL2 = 0x04, DIR_STOP = 0;
const HEADER_VOLTAGE = 0x05; 
let videoElement, canvas, ctx;
let bleDevice, bleCharacteristic;
let cv; // cvReadyは使わず、cv変数の有無で管理

let currentMode = 3, isSending = false, isVideoFileMode = false;
let lastCommandId = -1; // 直前に送ったコマンドを記憶する変数
let lastSendTime = 0;   // 最後に送信した時間
let recordedData = [], isRecording = false, recordStartTime = 0;

let targetX = -1, targetY = -1; // 目標地点の絶対座標（-1なら設定なし）
const TARGET_TOLERANCE = 20;    // 目標にどれくらい近づけばOKとするか（ピクセル）
let normX=0, normY=0;
let pixelX=0, pixelY=0;
let latestAngle=0, currentDirStr="";
let latestCenterX=0, latestCenterY=0, latestCenterTime=0; // 自動チューニング用の最新検出位置(canvas座標)

const COLOR_CENTER_LOW = [20, 100, 100];
const COLOR_CENTER_HIGH = [40, 255, 255];
const COLOR_FRONT_LOW1 = [0, 120, 70];
const COLOR_FRONT_HIGH1 = [10, 255, 255];
const COLOR_FRONT_LOW2 = [165, 120, 70];
const COLOR_FRONT_HIGH2 = [180, 255, 255];

const DETECT_SCALE = 0.5; // 色検出は1/2解像度で行う(計算量1/4)
let detectCanvas, detectCtx; // 検出用の縮小オフスクリーンキャンバス
let mats = null; // 毎フレーム使い回すOpenCV Mat群(確保/解放を繰り返さない)

// 解像度が変わったときだけMatを作り直す
function ensureMats(sw, sh) {
    if (mats && mats.sw === sw && mats.sh === sh) return;
    freeMats();
    const bound = (c) => new cv.Mat(sh, sw, cv.CV_8UC3, new cv.Scalar(c[0], c[1], c[2], 0));
    mats = {
        sw, sh,
        src: new cv.Mat(sh, sw, cv.CV_8UC4),
        rgb: new cv.Mat(),
        hsv: new cv.Mat(),
        mask: new cv.Mat(),
        mask2: new cv.Mat(),
        centerLow: bound(COLOR_CENTER_LOW),  centerHigh: bound(COLOR_CENTER_HIGH),
        frontLow1: bound(COLOR_FRONT_LOW1),  frontHigh1: bound(COLOR_FRONT_HIGH1),
        frontLow2: bound(COLOR_FRONT_LOW2),  frontHigh2: bound(COLOR_FRONT_HIGH2),
    };
}

function freeMats() {
    if (!mats) return;
    for (const k of Object.keys(mats)) {
        if (mats[k] && typeof mats[k].delete === 'function') mats[k].delete();
    }
    mats = null;
}

// 新しいビデオフレームが来たときだけ処理する(rAFだと同じフレームを二重処理する)
function scheduleNext() {
    if (videoElement.requestVideoFrameCallback) {
        videoElement.requestVideoFrameCallback(processLoop);
    } else {
        requestAnimationFrame(processLoop);
    }
}

// ファームがWrite Without Responseに対応していれば応答待ちなしで送る(低遅延)
function bleWrite(bytes) {
    if (bleCharacteristic.properties && bleCharacteristic.properties.writeWithoutResponse) {
        return bleCharacteristic.writeValueWithoutResponse(bytes);
    }
    return bleCharacteristic.writeValue(bytes);
}

function waitForOpenCV() {
    if (window.cv && window.cv.Mat) {
        cv = window.cv;
        document.getElementById('loading').style.display = 'none';
    } else {
        setTimeout(waitForOpenCV, 500);
    }
}

window.onload = () => {
    videoElement = document.getElementById('videoElement');
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d', { willReadFrequently: true });
    detectCanvas = document.createElement('canvas');
    detectCtx = detectCanvas.getContext('2d', { willReadFrequently: true });

    canvas.addEventListener('mousedown', (e) => {
        const rect = canvas.getBoundingClientRect();
        // キャンバス内のクリック位置を計算
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        // 実際の解像度(640x480)に合わせて座標変換
        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;
        
        targetX = Math.round(x * scaleX);
        targetY = Math.round(y * scaleY);
        
        console.log(`目標セット: ${targetX}, ${targetY}`);
    });

    document.getElementById('cameraButton').onclick = startWebcam;
    document.getElementById('connectButton').onclick = connectBluetooth;
    document.getElementById('btn-update').onclick = sendParamUpdate;
    document.getElementById('btn-voltage').onclick = requestVoltage;
    
    [1,2,3,4,5,6].forEach(m => document.getElementById(`mode${m}Btn`).onclick = () => setMode(m));

    document.getElementById('t-start').onclick = startAutoTune;
    document.getElementById('t-abort').onclick = () => { tuneAbort = true; };
    document.getElementById('t-save').onclick = saveTuneCSV;

    const videoInput = document.getElementById('videoInput');
    videoInput.addEventListener('change', handleFileSelect, false);
    
    document.getElementById('v-play').onclick = () => videoElement.paused ? videoElement.play() : videoElement.pause();
    document.getElementById('v-reset').onclick = () => { videoElement.currentTime = 0; videoElement.pause(); recordedData=[]; updateLogCount(); };
    document.getElementById('saveBtn').onclick = saveCSV;
    document.getElementById('clearBtn').onclick = () => { recordedData=[]; updateLogCount(); };

    setupDpad();
    setMode(3); 
    
    waitForOpenCV();
    scheduleNext();
};

async function startWebcam() {
    if (!cv) {
        alert("OpenCVのロードが終わっていません。画面上の「ロード中...」が消えてからもう一度押してください！");
        return;
    }
    try {
        isVideoFileMode = false;
        const stream = await navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 480 }, audio: false });
        videoElement.srcObject = stream;
        
        videoElement.setAttribute('autoplay', '');
        videoElement.setAttribute('muted', '');
        videoElement.setAttribute('playsinline', '');
        
        videoElement.play(); 
        canvas.style.display = 'block';
        document.getElementById('cameraButton').textContent = "Webcam動作中";
        if(currentMode === 4) setMode(3);
    } catch (err) { 
        alert("カメラエラー: " + err); 
    }
}

function handleFileSelect(event) {
    const file = event.target.files[0];
    if (!file) return;
    isVideoFileMode = true;
    videoElement.srcObject = null;
    videoElement.src = URL.createObjectURL(file);
    videoElement.load();
    canvas.style.display = 'block';
    document.getElementById('cameraButton').textContent = "動画ファイルモード";
    setMode(4); recordedData = []; updateLogCount();
}

function setMode(mode) {
    if (isTuning && mode !== 6) tuneAbort = true; // チューニング中にモードを離れたら中止

    currentMode = mode;
    document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
    document.getElementById(`mode${mode}Btn`).classList.add('active');

    const dpad = document.getElementById('dpad-area');
    const saveArea = document.getElementById('save-area');
    const paramArea = document.getElementById('param-area');
    const fileContainer = document.getElementById('file-input-container');
    const tuneArea = document.getElementById('tune-area');
    const status = document.getElementById('status');
    const canvasEl = document.getElementById('canvas');

    dpad.classList.remove('mode1-active', 'mode3-active', 'mode5-active');
    fileContainer.style.display = (mode === 4) ? "block" : "none";
    paramArea.style.display = (mode === 4 || mode === 6) ? "none" : "block";
    tuneArea.style.display = (mode === 6) ? "block" : "none";

    let color = "#fff";
    if (mode === 3) { status.textContent="Mode 3: 動作確認"; color="#4CAF50"; dpad.style.display="block"; dpad.classList.add('mode3-active'); saveArea.style.display="none"; }
    else if (mode === 5) { status.textContent="Mode 5: 別波形動作"; color="#00BCD4"; dpad.style.display="block"; dpad.classList.add('mode5-active'); saveArea.style.display="none"; }
    else if (mode === 1) { status.textContent="Mode 1: 3秒記録"; color="#2196F3"; dpad.style.display="block"; dpad.classList.add('mode1-active'); saveArea.style.display="block"; }
    else if (mode === 2) { status.textContent="Mode 2: 自動追尾"; color="#9C27B0"; dpad.style.display="none"; saveArea.style.display="none"; }
    else if (mode === 4) { status.textContent="Mode 4: 動画解析"; color="#FF5722"; dpad.style.display="none"; saveArea.style.display="block"; }
    else if (mode === 6) { status.textContent="Mode 6: 自動チューニング"; color="#E91E63"; dpad.style.display="none"; saveArea.style.display="none"; }

    status.style.color = color;
    canvasEl.style.borderColor = color;
}

// 縮小HSV画像から最大の色領域の重心を探し、canvas座標で返す
// 境界値Mat・マスクMatは ensureMats() で確保したものを使い回す
function findLargestColorCenter(hsvMat, lowMat1, highMat1, lowMat2 = null, highMat2 = null) {
    cv.inRange(hsvMat, lowMat1, highMat1, mats.mask);

    if (lowMat2 && highMat2) {
        cv.inRange(hsvMat, lowMat2, highMat2, mats.mask2);
        cv.bitwise_or(mats.mask, mats.mask2, mats.mask);
    }

    let contours = new cv.MatVector();
    let hierarchy = new cv.Mat();
    cv.findContours(mats.mask, contours, hierarchy, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE);

    let maxArea = 0;
    let bestPos = null;
    const minArea = 20 * DETECT_SCALE * DETECT_SCALE; // フル解像度での閾値20px^2相当
    for (let i = 0; i < contours.size(); i++) {
        let cnt = contours.get(i);
        let area = cv.contourArea(cnt, false);
        if (area > minArea && area > maxArea) {
            maxArea = area;
            let M = cv.moments(cnt);
            // 縮小画像の座標をcanvas座標に戻す
            bestPos = { x: (M.m10 / M.m00) / DETECT_SCALE, y: (M.m01 / M.m00) / DETECT_SCALE };
        }
        cnt.delete();
    }
    contours.delete(); hierarchy.delete();
    return bestPos;
}

// オーバーレイ描画ヘルパー(OpenCVではなく2D APIで直接canvasに描く)
function drawCircle(x, y, r, fill, stroke = null) {
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.stroke(); }
}

function drawArrow(x1, y1, x2, y2, color) {
    const ang = Math.atan2(y2 - y1, x2 - x1), len = 10;
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
    ctx.moveTo(x2, y2); ctx.lineTo(x2 - len * Math.cos(ang - 0.4), y2 - len * Math.sin(ang - 0.4));
    ctx.moveTo(x2, y2); ctx.lineTo(x2 - len * Math.cos(ang + 0.4), y2 - len * Math.sin(ang + 0.4));
    ctx.stroke();
}

function processLoop() {
    try {
        if (!cv) { scheduleNext(); return; }

        let vw = videoElement.videoWidth;
        let vh = videoElement.videoHeight;

        if (vw > 0) {
            document.getElementById('resolution-display').textContent = `Resolution: ${vw} x ${vh}`;
        }

        if (videoElement.paused || videoElement.ended) { scheduleNext(); return; }

        if (canvas.width !== 640) {
            let aspect = vh / vw;
            if(isNaN(aspect)) aspect = 0.75;
            canvas.width = 640; canvas.height = 640 * aspect;
        }
        // 表示用はフル解像度で直接描画(OpenCVにフル画像は渡さない)
        ctx.drawImage(videoElement, 0, 0, canvas.width, canvas.height);

        // 色検出は縮小画像に対して行う
        const sw = Math.round(canvas.width * DETECT_SCALE);
        const sh = Math.round(canvas.height * DETECT_SCALE);
        if (detectCanvas.width !== sw || detectCanvas.height !== sh) {
            detectCanvas.width = sw; detectCanvas.height = sh;
        }
        detectCtx.drawImage(videoElement, 0, 0, sw, sh);
        ensureMats(sw, sh);
        mats.src.data.set(detectCtx.getImageData(0, 0, sw, sh).data);
        cv.cvtColor(mats.src, mats.rgb, cv.COLOR_RGBA2RGB);
        cv.cvtColor(mats.rgb, mats.hsv, cv.COLOR_RGB2HSV);

        let centerPos = findLargestColorCenter(mats.hsv, mats.centerLow, mats.centerHigh);
        let frontPos = findLargestColorCenter(mats.hsv, mats.frontLow1, mats.frontHigh1, mats.frontLow2, mats.frontHigh2);

        let hasAngle = false;

        if (centerPos) {
            latestCenterX = centerPos.x; latestCenterY = centerPos.y; latestCenterTime = Date.now();
            normX = Math.round((centerPos.x / canvas.width) * 255);
            normY = Math.round((centerPos.y / canvas.height) * 255);

            if (vw > 0 && vh > 0) {
                pixelX = Math.round((centerPos.x / canvas.width) * vw);
                pixelY = Math.round((centerPos.y / canvas.height) * vh);
            } else {
                pixelX = normX; pixelY = normY;
            }

            drawCircle(centerPos.x, centerPos.y, 6, 'rgb(255,255,0)');
            drawCircle(centerPos.x, centerPos.y, 8, null, 'black');

            if (frontPos) {
                drawCircle(frontPos.x, frontPos.y, 4, 'rgb(255,0,0)');

                let dx = frontPos.x - centerPos.x;
                let dy = frontPos.y - centerPos.y;
                let rad = Math.atan2(dy, dx);
                let deg = rad * (180 / Math.PI);
                if (deg < 0) deg += 360;
                latestAngle = Math.round(deg);
                hasAngle = true;

                drawArrow(centerPos.x, centerPos.y, frontPos.x, frontPos.y, 'rgb(0,255,0)');
            }

 if (currentMode === 2 && bleCharacteristic && !isVideoFileMode) {
                if (targetX === -1 || targetY === -1) {
                    // 目標なし
                } 
                else if (!isSending) {

                    let commandId = 0; // 0:停止

                    // 1. ロボットからターゲットへのベクトル (dx, dy)
                    // targetX/Y も centerPos もcanvas座標系。pixelX/Y(ビデオ実解像度)と混ぜない
                    let dx = targetX - centerPos.x;
                    let dy = targetY - centerPos.y;

                    // 2. 角度をラジアンに変換
                    let rad = latestAngle * (Math.PI / 180);

                    // 3. 座標変換
                    let relFront = dx * Math.cos(rad) + dy * Math.sin(rad);
                    let relRight = -dx * Math.sin(rad) + dy * Math.cos(rad);

                    // 4. 指令の決定
                    if (Math.abs(relFront) < TARGET_TOLERANCE && Math.abs(relRight) < TARGET_TOLERANCE) {
                        commandId = 0; 
                    }
                    else if (Math.abs(relFront) > Math.abs(relRight)) {
                        if (relFront > 0) commandId = 1; // 前方
                        else commandId = 2;              // 後方
                    }
                    else {
                        if (relRight > 0) commandId = 3; // 右へ
                        else commandId = 4;              // 左へ
                    }

                    // コマンド送信
                    const now = Date.now();

                    // ✅ ここで初めて判定する（!isSending は親の else if で確認済みなので外してOK）
                    if (commandId !== lastCommandId || now - lastSendTime > 500) {
                        
                        isSending = true; // ✅ 送信するときだけロックをかける
                        
                        bleWrite(new Uint8Array([HEADER_MANUAL, commandId, 0]))
                            .then(() => {
                                lastCommandId = commandId;
                                lastSendTime = now;
                                isSending = false; // ✅ 送信成功したらロック解除
                            })
                            .catch((error) => {
                                console.error("送信エラー:", error);
                                isSending = false; // ✅ エラー時もロック解除
                            });
                    }
                }
            }

            // ★画面描画の追加（目標地点に×印を描画）
            if (targetX !== -1) {
                ctx.strokeStyle = 'rgb(0,255,0)'; ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(targetX - 10, targetY - 10); ctx.lineTo(targetX + 10, targetY + 10);
                ctx.moveTo(targetX + 10, targetY - 10); ctx.lineTo(targetX - 10, targetY + 10);
                ctx.stroke();
                ctx.fillStyle = 'rgb(0,255,0)'; ctx.font = '14px sans-serif';
                ctx.fillText("TARGET", targetX + 15, targetY);
            }
        }

        let infoText = "";
        if (currentMode === 1 && isRecording) {
            let t = Date.now() - recordStartTime;
            recordedData.push({ t: t, in: currentDirStr, x: pixelX, y: pixelY, angle: latestAngle });
            infoText = "REC";
        } else if (currentMode === 4 && !videoElement.paused) {
            let t = Math.round(videoElement.currentTime * 1000);
            let last = recordedData[recordedData.length - 1];
            if (!last || last.t !== t) {
                recordedData.push({ t: t, in: "Video", x: pixelX, y: pixelY, angle: latestAngle });
            }
            infoText = `Time:${t}`;
            updateLogCount();
        }

        if (infoText) {
            ctx.fillStyle = 'white'; ctx.font = 'bold 22px sans-serif';
            ctx.fillText(infoText, 20, 40);
        }

        let coordsText = `Pos:(${pixelX}, ${pixelY})`;
        if (hasAngle) coordsText += ` Ang:${latestAngle}`;
        else if (centerPos) coordsText += ` Ang:??? (Front Lost)`;
        else coordsText = `Searching Yellow...`;

        ctx.fillStyle = 'rgb(255,255,0)'; ctx.font = '16px sans-serif';
        ctx.fillText(coordsText, 20, 70);

    } catch (e) { console.log(e); }
    scheduleNext();
}

async function connectBluetooth() {
    const SERVICE_UUID = '0000aaaa-0000-1000-8000-00805f9b34fb';
    const CHARACTERISTIC_UUID = '0000bbbb-0000-1000-8000-00805f9b34fb';
    try {
        document.getElementById('status').textContent = "接続試行中...";
        document.getElementById('status').style.color = "#FF9800";

        bleDevice = await navigator.bluetooth.requestDevice({ acceptAllDevices: true, optionalServices: [SERVICE_UUID] });
        
        bleDevice.addEventListener('gattserverdisconnected', () => {
            document.getElementById('status').textContent = "BLE切断されました";
            document.getElementById('status').style.color = "red";
            bleCharacteristic = null;
        });

        const server = await bleDevice.gatt.connect();
        await new Promise(resolve => setTimeout(resolve, 500));

        const service = await server.getPrimaryService(SERVICE_UUID);
        bleCharacteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);
        await bleCharacteristic.startNotifications();
        bleCharacteristic.addEventListener('characteristicvaluechanged', handleReceiveData);
        document.getElementById('status').textContent = "Bluetooth接続OK";
        document.getElementById('status').style.color = "#4CAF50";
    } catch (err) { 
        alert("接続失敗: " + err); 
        document.getElementById('status').textContent = "接続エラー";
        document.getElementById('status').style.color = "red";
    }
}

async function sendManualCommand(id, headerType = HEADER_MANUAL) {
    if (!bleCharacteristic || isVideoFileMode) return;
    try { await bleWrite(new Uint8Array([headerType, id, 0])); } catch(e) {}
}

async function sendParamUpdate() {
    const btn = document.getElementById('btn-update');
    if (!bleCharacteristic) { 
        btn.textContent = "未接続です"; btn.style.backgroundColor = "#9E9E9E"; 
        setTimeout(() => { btn.textContent = "パラメータ更新"; btn.style.backgroundColor = "#FF9800"; }, 1000);
        return; 
    }
    const r1 = parseInt(document.getElementById('p-res1').value)||5;
    const p1 = parseInt(document.getElementById('p-rep1').value)||4;
    const r2 = parseInt(document.getElementById('p-res2').value)||10;
    const p2 = parseInt(document.getElementById('p-rep2').value)||4;
    try {
        await bleCharacteristic.writeValue(new Uint8Array([HEADER_PARAM, r1, p1, r2, p2]));
        btn.textContent = "更新完了!"; btn.style.backgroundColor = "#4CAF50";
        setTimeout(() => { btn.textContent = "パラメータ更新"; btn.style.backgroundColor = "#FF9800"; }, 1000);
    } catch (e) {
        btn.textContent = "エラー"; btn.style.backgroundColor = "#f44336";
        setTimeout(() => { btn.textContent = "パラメータ更新"; btn.style.backgroundColor = "#FF9800"; }, 1000);
    }
}

function setupDpad() {
    const btns = document.querySelectorAll('.d-btn');
    btns.forEach(btn => {
        const id = parseInt(btn.dataset.dir), name = btn.textContent;
        const press = (e) => {
            e.preventDefault(); 
            if(currentMode===3) { 
                sendManualCommand(id, HEADER_MANUAL); 
                document.getElementById('status').textContent = `動作中: ${name}`; 
            }
            else if(currentMode===5) { 
                sendManualCommand(id, HEADER_MANUAL2); 
                document.getElementById('status').textContent = `別波形で動作中: ${name}`; 
            }
            else if(currentMode===1 && !isRecording && !isVideoFileMode) {
                isRecording=true; recordStartTime=Date.now(); currentDirStr=name; 
                sendManualCommand(id, HEADER_MANUAL);
                document.getElementById('status').textContent = `REC中: ${name}`;
                setTimeout(()=>{ 
                    sendManualCommand(DIR_STOP, HEADER_MANUAL); 
                    isRecording=false; 
                    document.getElementById('status').textContent="完了"; 
                    updateLogCount(); 
                }, 3000);
            }
        };
        const release = (e) => { 
            e.preventDefault(); 
            if(currentMode===3) { 
                sendManualCommand(DIR_STOP, HEADER_MANUAL); 
                document.getElementById('status').textContent="待機中"; 
            } 
            else if(currentMode===5) { 
                sendManualCommand(DIR_STOP, HEADER_MANUAL2); 
                document.getElementById('status').textContent="待機中"; 
            } 
        };
        ['mousedown','touchstart'].forEach(ev=>btn.addEventListener(ev, press, {passive:false}));
        ['mouseup','mouseleave','touchend'].forEach(ev=>btn.addEventListener(ev, release));
    });
}

function updateLogCount() { document.getElementById('log-count').textContent = `データ数: ${recordedData.length}行`; }
function saveCSV() {
    if (recordedData.length===0) { alert("データなし"); return; }
    let csv = "Time(ms),Input,PixelX,PixelY,Angle(deg)\n";
    recordedData.forEach(r => csv += `${r.t},${r.in},${r.x},${r.y},${r.angle}\n`);
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([csv], {type: 'text/csv'}));
    a.download = `log_${Date.now()}.csv`; a.click();
}

async function requestVoltage() {
    if (!bleCharacteristic || isVideoFileMode) {
        alert("Bluetoothが接続されていないか、動画モードです。");
        return;
    }
    try {
        document.getElementById('voltage-display').textContent = "取得中...";
        await bleCharacteristic.writeValue(new Uint8Array([HEADER_VOLTAGE]));
    } catch (e) {
        console.log("電圧要求エラー:", e);
        document.getElementById('voltage-display').textContent = "Error";
    }
}

function handleReceiveData(event) {
    const value = event.target.value;
    let data = new Uint8Array(value.buffer);

    if (data.length > 0) {
        if (data[0] === HEADER_VOLTAGE && data.length >= 3) {
            let volts_int = data[1];
            let volts_dec = data[2];
            document.getElementById('voltage-display').textContent = `${volts_int}.${volts_dec} V`;
        }
    }
}

// ===== Mode 6: 自動パラメータチューニング =====
// パラメータ(Res/Rep)の組み合わせを総当たりし、一定時間動かしてカメラで
// 移動距離を計測 → 速度[px/s]でスコア化 → ベストを自動適用する
let isTuning = false, tuneAbort = false;
let tuneResults = [];

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const clampByte = (v) => Math.max(1, Math.min(255, Math.round(v) || 1));

// 一定時間、検出位置を平均してノイズを抑えた位置を返す(検出が途切れていればnull)
async function samplePosition(ms = 300) {
    const xs = [], ys = []; let lastAng = null;
    const t0 = Date.now();
    while (Date.now() - t0 < ms) {
        if (Date.now() - latestCenterTime < 250) {
            xs.push(latestCenterX); ys.push(latestCenterY); lastAng = latestAngle;
        }
        await sleep(50);
    }
    if (xs.length === 0) return null;
    const avg = a => a.reduce((s, v) => s + v, 0) / a.length;
    return { x: avg(xs), y: avg(ys), angle: lastAng };
}

// 1方向に durMs 動かして速度[px/s]と向きのズレ[deg]を計測
async function measureMove(cmdId, durMs) {
    const p0 = await samplePosition(300);
    if (!p0) return null;
    await bleWrite(new Uint8Array([HEADER_MANUAL, cmdId, 0]));
    await sleep(durMs);
    await bleWrite(new Uint8Array([HEADER_MANUAL, DIR_STOP, 0]));
    await sleep(400); // 静定待ち
    const p1 = await samplePosition(300);
    if (!p1) return null;
    const dist = Math.hypot(p1.x - p0.x, p1.y - p0.y);
    let drift = Math.abs(p1.angle - p0.angle);
    if (drift > 180) drift = 360 - drift;
    return { speed: dist / (durMs / 1000), drift };
}

function tuneRange(min, max, step) {
    min = clampByte(min); max = clampByte(max); step = Math.max(1, Math.round(step) || 1);
    const out = [];
    for (let v = min; v <= max; v += step) out.push(v);
    return out;
}

function setTuneProgress(text) { document.getElementById('t-progress').textContent = text; }

function renderTuneResults() {
    const sorted = [...tuneResults].sort((a, b) => b.score - a.score).slice(0, 10);
    let html = "<tr><th>Res</th><th>Rep</th><th>速度[px/s]</th><th>角度ズレ[deg]</th></tr>";
    sorted.forEach(r => {
        html += `<tr><td>${r.res}</td><td>${r.rep}</td><td>${r.score.toFixed(1)}</td><td>${r.drift.toFixed(0)}</td></tr>`;
    });
    document.getElementById('t-results').innerHTML = html;
}

async function startAutoTune() {
    if (isTuning) return;
    if (!bleCharacteristic) { alert("先にBluetoothを接続してください"); return; }
    if (Date.now() - latestCenterTime > 1000) { alert("カメラでマーカー(黄色)が検出できていません"); return; }

    const axis = document.getElementById('t-axis').value; // 'fb' or 'lr'
    const durMs = Math.max(300, parseInt(document.getElementById('t-dur').value) || 1500);
    const resList = tuneRange(
        document.getElementById('t-res-min').value,
        document.getElementById('t-res-max').value,
        document.getElementById('t-res-step').value);
    const repList = tuneRange(
        document.getElementById('t-rep-min').value,
        document.getElementById('t-rep-max').value,
        document.getElementById('t-rep-step').value);

    const total = resList.length * repList.length;
    const estSec = Math.round(total * (0.3 + 2 * (0.3 + durMs / 1000 + 0.7)));
    if (total > 100 && !confirm(`${total}通りで約${Math.round(estSec / 60)}分かかります。実行しますか?`)) return;

    // 対象でない側のパラメータは現在の入力値を維持する
    const curR1 = clampByte(document.getElementById('p-res1').value);
    const curP1 = clampByte(document.getElementById('p-rep1').value);
    const curR2 = clampByte(document.getElementById('p-res2').value);
    const curP2 = clampByte(document.getElementById('p-rep2').value);
    const cmds = (axis === 'fb') ? [1, 2] : [3, 4]; // 前/後 or 右/左 (戻りながら計測)

    isTuning = true; tuneAbort = false; tuneResults = [];
    let consecutiveFails = 0, count = 0;

    try {
        for (const res of resList) {
            for (const rep of repList) {
                if (tuneAbort) break;
                count++;
                setTuneProgress(`${count}/${total} 計測中: Res=${res}, Rep=${rep} (推定残り${Math.round(estSec * (1 - count / total))}秒)`);

                const p = (axis === 'fb') ? [res, rep, curR2, curP2] : [curR1, curP1, res, rep];
                await bleCharacteristic.writeValue(new Uint8Array([HEADER_PARAM, p[0], p[1], p[2], p[3]]));
                await sleep(200);

                const go = await measureMove(cmds[0], durMs);
                if (tuneAbort) break;
                const back = await measureMove(cmds[1], durMs);

                const runs = [go, back].filter(r => r !== null);
                if (runs.length === 0) {
                    consecutiveFails++;
                    if (consecutiveFails >= 3) {
                        alert("マーカーを3回連続で見失ったため中止します。照明や画角を確認してください。");
                        tuneAbort = true; break;
                    }
                    continue;
                }
                consecutiveFails = 0;
                const score = runs.reduce((s, r) => s + r.speed, 0) / runs.length;
                const drift = runs.reduce((s, r) => s + r.drift, 0) / runs.length;
                tuneResults.push({ axis, res, rep, score, drift,
                    fwd: go ? go.speed : NaN, back: back ? back.speed : NaN });
                renderTuneResults();
            }
            if (tuneAbort) break;
        }
    } finally {
        try { await bleWrite(new Uint8Array([HEADER_MANUAL, DIR_STOP, 0])); } catch (e) {}
        isTuning = false;
    }

    if (tuneResults.length > 0) {
        const best = tuneResults.reduce((a, b) => (b.score > a.score ? b : a));
        // ベスト値を入力欄に反映してファームにも送信
        if (axis === 'fb') {
            document.getElementById('p-res1').value = best.res;
            document.getElementById('p-rep1').value = best.rep;
        } else {
            document.getElementById('p-res2').value = best.res;
            document.getElementById('p-rep2').value = best.rep;
        }
        try { await sendParamUpdate(); } catch (e) {}
        setTuneProgress(tuneAbort
            ? `中止しました (${tuneResults.length}件計測済み)。暫定ベスト: Res=${best.res}, Rep=${best.rep} (${best.score.toFixed(1)} px/s) を適用しました`
            : `完了! ベスト: Res=${best.res}, Rep=${best.rep} (${best.score.toFixed(1)} px/s) を適用しました`);
    } else {
        setTuneProgress("計測データなしで終了しました");
    }
}

function saveTuneCSV() {
    if (tuneResults.length === 0) { alert("結果がありません"); return; }
    let csv = "Axis,Res,Rep,AvgSpeed(px/s),FwdSpeed(px/s),BackSpeed(px/s),Drift(deg)\n";
    tuneResults.forEach(r => csv += `${r.axis},${r.res},${r.rep},${r.score.toFixed(2)},${r.fwd.toFixed(2)},${r.back.toFixed(2)},${r.drift.toFixed(1)}\n`);
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
    a.download = `tune_${Date.now()}.csv`; a.click();
}