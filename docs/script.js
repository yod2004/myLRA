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
let latestAngleTime=0; // 前方マーカーが最後に検出できた時刻(角度の鮮度判定用)

// --- 検出パラメータ(実行時に調整・キャリブレーション可能) ---
// 各色は「色相中心 hue ± 幅 hueW」「彩度下限 sMin」「明度下限 vMin」で表現する。
// この表現なら赤(0/180をまたぐ)も自動で2レンジに分割でき、UIも直感的。
const det = {
    center: { hue: 30, hueW: 12, sMin: 80, vMin: 80 }, // 中心マーカー(黄)
    front:  { hue: 0,  hueW: 14, sMin: 90, vMin: 70 }, // 前方マーカー(赤/オレンジ)
};
let detectScale = 0.5;   // 色検出の解像度倍率(小さいマーカーは0.75/1.0で拾いやすい)
let minAreaFull = 20;    // フル解像度基準の最小ブロブ面積[px^2]
let useMorphology = true; // ノイズ除去・穴埋め(開閉処理)
let showMask = false;     // マスクを画面に重ねて可視化
let sampleTarget = null;  // 'center' | 'front': 次のキャンバスクリックで色をサンプリング

let detectCanvas, detectCtx; // 検出用の縮小オフスクリーンキャンバス
let maskCanvas, maskCtx;     // マスク可視化用オフスクリーンキャンバス
let sampleCanvas, sampleCtx; // 色サンプリング用オフスクリーンキャンバス
let mats = null; // 毎フレーム使い回すOpenCV Mat群(確保/解放を繰り返さない)

// 色相中心±幅を、0..179でwrapを考慮した1~2個の[loH,hiH]区間に変換
function hueRanges(hue, w) {
    w = Math.min(w, 89);
    let lo = hue - w, hi = hue + w;
    if (lo < 0)   return [[180 + lo, 179], [0, hi]];
    if (hi > 179) return [[lo, 179], [0, hi - 180]];
    return [[lo, hi]];
}

// detの色定義から、inRange用の境界Mat(lo/hi)の配列を作り直す
function buildColorRanges() {
    if (!mats) return;
    if (mats.colorRanges) {
        for (const key in mats.colorRanges)
            mats.colorRanges[key].forEach(r => { r.lo.delete(); r.hi.delete(); });
    }
    const make = (c) => hueRanges(c.hue, c.hueW).map(([h0, h1]) => ({
        lo: new cv.Mat(mats.sh, mats.sw, cv.CV_8UC3, new cv.Scalar(h0, c.sMin, c.vMin, 0)),
        hi: new cv.Mat(mats.sh, mats.sw, cv.CV_8UC3, new cv.Scalar(h1, 255, 255, 255)),
    }));
    mats.colorRanges = { center: make(det.center), front: make(det.front) };
}

// 解像度が変わったときだけMatを作り直す
function ensureMats(sw, sh) {
    if (mats && mats.sw === sw && mats.sh === sh) return;
    freeMats();
    mats = {
        sw, sh,
        src: new cv.Mat(sh, sw, cv.CV_8UC4),
        rgb: new cv.Mat(),
        hsv: new cv.Mat(),
        maskCenter: new cv.Mat(),
        maskFront: new cv.Mat(),
        maskTmp: new cv.Mat(),
        kernel: cv.getStructuringElement(cv.MORPH_ELLIPSE, new cv.Size(3, 3)),
        colorRanges: null,
    };
    buildColorRanges();
}

function freeMats() {
    if (!mats) return;
    if (mats.colorRanges) {
        for (const key in mats.colorRanges)
            mats.colorRanges[key].forEach(r => { r.lo.delete(); r.hi.delete(); });
    }
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
    maskCanvas = document.createElement('canvas');
    maskCtx = maskCanvas.getContext('2d', { willReadFrequently: true });
    sampleCanvas = document.createElement('canvas');
    sampleCtx = sampleCanvas.getContext('2d', { willReadFrequently: true });

    canvas.addEventListener('mousedown', (e) => {
        const rect = canvas.getBoundingClientRect();
        // キャンバス内のクリック位置を計算
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;

        // 実際の解像度(640x480)に合わせて座標変換
        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;
        const cx = Math.round(x * scaleX);
        const cy = Math.round(y * scaleY);

        // 色サンプリング中ならクリックでHSVを取得(目標設定より優先)
        if (sampleTarget) { sampleColorAt(cx, cy); return; }

        targetX = cx;
        targetY = cy;
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

    setupDetectPanel();

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

// 縮小HSV画像から指定色の最大領域の重心を探し、canvas座標で返す
// ranges: [{lo,hi}, ...] (赤など色相wrap時は複数)、outMask: 結果マスクの保存先
// excludeMask: 指定すると、そのマスクに該当する画素を結果から除外(色レンジが
//   中心色と被っても中心マーカーを誤検出しないための安全策)
function findColor(hsvMat, ranges, outMask, excludeMask = null) {
    ranges.forEach((r, i) => {
        if (i === 0) {
            cv.inRange(hsvMat, r.lo, r.hi, outMask);
        } else {
            cv.inRange(hsvMat, r.lo, r.hi, mats.maskTmp);
            cv.bitwise_or(outMask, mats.maskTmp, outMask);
        }
    });

    if (excludeMask) cv.subtract(outMask, excludeMask, outMask); // 中心色画素を除去(AND NOT)

    if (useMorphology) {
        // 開処理でゴマ塩ノイズを除去、閉処理でマーカー内部の穴を埋める
        cv.morphologyEx(outMask, outMask, cv.MORPH_OPEN, mats.kernel);
        cv.morphologyEx(outMask, outMask, cv.MORPH_CLOSE, mats.kernel);
    }

    let contours = new cv.MatVector();
    let hierarchy = new cv.Mat();
    cv.findContours(outMask, contours, hierarchy, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE);

    let maxArea = 0;
    let bestPos = null;
    const minArea = minAreaFull * detectScale * detectScale; // フル解像度基準の面積に換算
    for (let i = 0; i < contours.size(); i++) {
        let cnt = contours.get(i);
        let area = cv.contourArea(cnt, false);
        if (area > minArea && area > maxArea) {
            maxArea = area;
            let M = cv.moments(cnt);
            // 縮小画像の座標をcanvas座標に戻す
            bestPos = { x: (M.m10 / M.m00) / detectScale, y: (M.m01 / M.m00) / detectScale };
        }
        cnt.delete();
    }
    contours.delete(); hierarchy.delete();
    return bestPos;
}

// 検出マスクを画面に半透明で重ねる(中心=黄、前方=赤)。閾値調整の目視確認用
function drawMaskOverlay() {
    const sw = mats.sw, sh = mats.sh;
    if (maskCanvas.width !== sw || maskCanvas.height !== sh) { maskCanvas.width = sw; maskCanvas.height = sh; }
    const img = maskCtx.createImageData(sw, sh);
    const c = mats.maskCenter.data, f = mats.maskFront.data, d = img.data;
    for (let i = 0, p = 0; i < c.length; i++, p += 4) {
        if (f[i])      { d[p] = 255; d[p+1] = 0;   d[p+2] = 0; d[p+3] = 170; }
        else if (c[i]) { d[p] = 255; d[p+1] = 255; d[p+2] = 0; d[p+3] = 150; }
        else           { d[p+3] = 0; }
    }
    maskCtx.putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(maskCanvas, 0, 0, canvas.width, canvas.height);
    ctx.imageSmoothingEnabled = true;
}

// RGB(0-255) → HSV(H:0-179, S:0-255, V:0-255)。OpenCVのRGB2HSVと同じスケール
function rgb2hsvCV(r, g, b) {
    r /= 255; g /= 255; b /= 255;
    const mx = Math.max(r, g, b), mn = Math.min(r, g, b), df = mx - mn;
    let h = 0;
    if (df !== 0) {
        if (mx === r)      h = 60 * (((g - b) / df) % 6);
        else if (mx === g) h = 60 * ((b - r) / df + 2);
        else               h = 60 * ((r - g) / df + 4);
    }
    if (h < 0) h += 360;
    const s = mx === 0 ? 0 : df / mx;
    return [Math.round(h / 2), Math.round(s * 255), Math.round(mx * 255)];
}

// クリック位置の色を映像から直接サンプリングし、その色のレンジを自動設定
// 描画ループやOpenCVのMatに依存せず、現在のビデオフレームから直接読む(堅牢)
function sampleColorAt(cx, cy) {
    const label = sampleTarget === 'center' ? '中心(黄)' : '前方(赤)';
    const finish = () => {
        sampleTarget = null;
        document.querySelectorAll('.sample-btn').forEach(b => b.classList.remove('sampling'));
    };

    const vw = videoElement.videoWidth, vh = videoElement.videoHeight;
    const usingVideo = vw > 0 && vh > 0 && !videoElement.ended;
    // クリック点(canvas座標)を映像座標へ。映像が無ければcanvasから直接読む
    const R = 4, W = 2 * R + 1;
    sampleCanvas.width = W; sampleCanvas.height = W;
    try {
        if (usingVideo) {
            const vx = Math.round(cx / canvas.width * vw);
            const vy = Math.round(cy / canvas.height * vh);
            sampleCtx.drawImage(videoElement, vx - R, vy - R, W, W, 0, 0, W, W);
        } else {
            // フォールバック: 表示中のcanvasから読む
            sampleCtx.drawImage(canvas, cx - R, cy - R, W, W, 0, 0, W, W);
        }
    } catch (e) {
        document.getElementById('status').textContent = "色の取得に失敗しました(映像を確認)";
        finish(); return;
    }

    const px = sampleCtx.getImageData(0, 0, W, W).data;
    let sumCos = 0, sumSin = 0, n = 0, minS = 255, minV = 255;
    for (let p = 0; p < px.length; p += 4) {
        const [h, s, v] = rgb2hsvCV(px[p], px[p+1], px[p+2]);
        const a = h * 2 * Math.PI / 180;
        sumCos += Math.cos(a); sumSin += Math.sin(a); n++;
        if (s < minS) minS = s;
        if (v < minV) minV = v;
    }
    if (n === 0) { finish(); return; }

    let hue = Math.atan2(sumSin, sumCos) * 180 / (2 * Math.PI);
    if (hue < 0) hue += 180;
    const c = det[sampleTarget];
    c.hue = Math.round(hue);
    c.hueW = 16;
    // サンプル値の約半分を下限に(上限も設けて厳しくなりすぎないように)。
    // マーカーは彩度が高く背景は低彩度なので、Sを主な弁別に使い緩めに取る。
    // minSをそのまま使うと鮮やかな1点で下限が高くなりマスクが空になるため緩める
    c.sMin = Math.min(150, Math.max(40, Math.round(minS * 0.5)));
    c.vMin = Math.min(150, Math.max(30, Math.round(minV * 0.5)));
    buildColorRanges();
    updateDetectUI();
    document.getElementById('status').textContent =
        `${label}の色を取得: H=${c.hue} S≥${c.sMin} V≥${c.vMin}`;
    finish();
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
        const sw = Math.round(canvas.width * detectScale);
        const sh = Math.round(canvas.height * detectScale);
        if (detectCanvas.width !== sw || detectCanvas.height !== sh) {
            detectCanvas.width = sw; detectCanvas.height = sh;
        }
        detectCtx.drawImage(videoElement, 0, 0, sw, sh);
        ensureMats(sw, sh);
        mats.src.data.set(detectCtx.getImageData(0, 0, sw, sh).data);
        cv.cvtColor(mats.src, mats.rgb, cv.COLOR_RGBA2RGB);
        cv.cvtColor(mats.rgb, mats.hsv, cv.COLOR_RGB2HSV);

        let centerPos = findColor(mats.hsv, mats.colorRanges.center, mats.maskCenter);
        // 前方は中心色(黄)を除外して探す → レンジが被っても中心を誤検出しない
        let frontPos = findColor(mats.hsv, mats.colorRanges.front, mats.maskFront, mats.maskCenter);

        if (showMask) drawMaskOverlay();

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
                latestAngleTime = Date.now();
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
        const devName = bleDevice.name || "(名前なし)";

        bleDevice.addEventListener('gattserverdisconnected', () => {
            document.getElementById('status').textContent = `BLE切断: ${devName}`;
            document.getElementById('status').style.color = "red";
            bleCharacteristic = null;
        });

        const server = await bleDevice.gatt.connect();
        await new Promise(resolve => setTimeout(resolve, 500));

        const service = await server.getPrimaryService(SERVICE_UUID);
        bleCharacteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);

        // 通知は電圧表示にしか使わない。CCCD未対応の旧ファームだと
        // startNotifications()が "GATT Error: Not supported" を投げるが、
        // コマンド送信(write)には不要なので失敗しても接続は継続する
        try {
            await bleCharacteristic.startNotifications();
            bleCharacteristic.addEventListener('characteristicvaluechanged', handleReceiveData);
        } catch (e) {
            console.warn("通知の有効化に失敗(コマンド送信は可能):", e);
        }

        // 接続先デバイス名を表示して、どの機体に繋がったか分かるようにする
        document.getElementById('status').textContent = `接続OK: ${devName}`;
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

// --- 検出設定パネル ---
function setupDetectPanel() {
    document.getElementById('detectToggle').onclick = () => {
        const a = document.getElementById('detect-area');
        a.style.display = (a.style.display === 'none' || !a.style.display) ? 'block' : 'none';
    };

    // 各色のスライダー(色相中心/幅/彩度下限/明度下限)を det に反映
    [['center', 'c'], ['front', 'f']].forEach(([key, pre]) => {
        [['hue', 179], ['hueW', 89], ['sMin', 255], ['vMin', 255]].forEach(([prop]) => {
            const el = document.getElementById(`${pre}-${prop}`);
            el.oninput = () => {
                det[key][prop] = parseInt(el.value);
                document.getElementById(`${pre}-${prop}-v`).textContent = el.value;
                buildColorRanges();
            };
        });
        document.getElementById(`${pre}-sample`).onclick = (e) => {
            const on = sampleTarget !== key;
            sampleTarget = on ? key : null;
            document.querySelectorAll('.sample-btn').forEach(b => b.classList.remove('sampling'));
            if (on) {
                e.target.classList.add('sampling');
                document.getElementById('status').textContent =
                    `映像内の${key === 'center' ? '中心(黄)' : '前方(赤)'}マーカーをクリックしてください`;
            }
        };
    });

    document.getElementById('d-scale').onchange = (e) => { detectScale = parseFloat(e.target.value); };
    document.getElementById('d-area').oninput = (e) => {
        minAreaFull = parseInt(e.target.value);
        document.getElementById('d-area-v').textContent = e.target.value;
    };
    document.getElementById('d-morph').onchange = (e) => { useMorphology = e.target.checked; };
    document.getElementById('d-mask').onchange = (e) => { showMask = e.target.checked; };
    updateDetectUI();
}

// det の現在値をスライダー類に反映(キャリブレーション後の同期にも使う)
function updateDetectUI() {
    [['center', 'c'], ['front', 'f']].forEach(([key, pre]) => {
        ['hue', 'hueW', 'sMin', 'vMin'].forEach(prop => {
            const el = document.getElementById(`${pre}-${prop}`);
            if (!el) return;
            el.value = det[key][prop];
            document.getElementById(`${pre}-${prop}-v`).textContent = det[key][prop];
        });
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

// ===== Mode 6: 自動パラメータ計測 =====
// 動かすパターン(1~4)を固定し、パラメータ(Res/Rep)の組み合わせを総当たりして
// 「実際にどの向きに動いたか」(ロボット座標系の前後/左右成分と回転)を記録、
// 最後にまとめを表示する。評価方向は固定せず、自動適用もしない。
let isTuning = false, tuneAbort = false;
let tuneResults = [];

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const clampByte = (v) => Math.max(1, Math.min(255, Math.round(v) || 1));

// 一定時間、検出位置を平均してノイズを抑えた位置を返す(検出が途切れていればnull)
// 角度は前方マーカーが新鮮なサンプルのみ円平均する(取れなければangle=null)
async function samplePosition(ms = 300) {
    const xs = [], ys = []; let cs = 0, sn = 0, na = 0;
    const t0 = Date.now();
    while (Date.now() - t0 < ms) {
        if (Date.now() - latestCenterTime < 250) {
            xs.push(latestCenterX); ys.push(latestCenterY);
            if (Date.now() - latestAngleTime < 250) {
                const r = latestAngle * Math.PI / 180;
                cs += Math.cos(r); sn += Math.sin(r); na++;
            }
        }
        await sleep(50);
    }
    if (xs.length === 0) return null;
    const avg = a => a.reduce((s, v) => s + v, 0) / a.length;
    let angle = null;
    if (na > 0) {
        angle = Math.atan2(sn, cs) * 180 / Math.PI;
        if (angle < 0) angle += 360;
    }
    return { x: avg(xs), y: avg(ys), angle };
}

// 固定パターンで durMs 動かし、開始時のロボット向きを基準に移動を分解して返す
// front: 前(+)/後(-) [px/s], right: 右(+)/左(-) [px/s] (Mode2と同じ向き定義),
// rot: 回転 [deg/s], speed: 移動の速さ [px/s]
async function measureTrial(cmdId, header, durMs) {
    const p0 = await samplePosition(300);
    if (!p0 || p0.angle === null) return null;
    await bleWrite(new Uint8Array([header, cmdId, 0]));
    await sleep(durMs);
    await bleWrite(new Uint8Array([header, DIR_STOP, 0]));
    await sleep(400); // 静定待ち
    const p1 = await samplePosition(300);
    if (!p1 || p1.angle === null) return null;

    const dx = p1.x - p0.x, dy = p1.y - p0.y;
    const th = p0.angle * Math.PI / 180;
    const sec = durMs / 1000;
    const rot = ((p1.angle - p0.angle + 540) % 360) - 180; // [-180,180)
    return {
        front: (dx * Math.cos(th) + dy * Math.sin(th)) / sec,
        right: (-dx * Math.sin(th) + dy * Math.cos(th)) / sec,
        rot: rot / sec,
        speed: Math.hypot(dx, dy) / sec,
    };
}

// ロボットが画面端に近いときは中央に置き直されるまで待つ(中止で抜ける)
async function waitForSafePosition() {
    const mx = canvas.width * 0.12, my = canvas.height * 0.12;
    while (!tuneAbort) {
        const fresh = Date.now() - latestCenterTime < 500;
        if (fresh &&
            latestCenterX > mx && latestCenterX < canvas.width - mx &&
            latestCenterY > my && latestCenterY < canvas.height - my) return true;
        setTuneProgress(fresh
            ? "ロボットが画面端に近いので中央付近に置き直してください…(自動で再開します)"
            : "マーカーを探しています…");
        await sleep(300);
    }
    return false;
}

function tuneRange(min, max, step) {
    min = clampByte(min); max = clampByte(max); step = Math.max(1, Math.round(step) || 1);
    const out = [];
    for (let v = min; v <= max; v += step) out.push(v);
    return out;
}

function setTuneProgress(text) { document.getElementById('t-progress').textContent = text; }

function renderTuneResults() {
    let html = "<tr><th>Res</th><th>Rep</th><th>前(+)/後(-)<br>[px/s]</th><th>右(+)/左(-)<br>[px/s]</th><th>回転<br>[deg/s]</th><th>速さ<br>[px/s]</th></tr>";
    tuneResults.forEach(r => {
        html += `<tr><td>${r.res}</td><td>${r.rep}</td><td>${r.front.toFixed(1)}</td><td>${r.right.toFixed(1)}</td><td>${r.rot.toFixed(1)}</td><td>${r.speed.toFixed(1)}</td></tr>`;
    });
    document.getElementById('t-results').innerHTML = html;
}

// 全計測が終わったら、方向ごとに目立つパラメータを一覧にする
function renderTuneSummary() {
    if (tuneResults.length === 0) return;
    const best = (fn) => tuneResults.reduce((a, b) => (fn(b) > fn(a) ? b : a));
    // ok: その方向に実際に動いた結果かどうか(全結果が逆向きなら「該当なし」)
    const rows = [
        ['前進が最大',   best(r => r.front),          r => `${r.front.toFixed(1)} px/s`,  r => r.front > 0],
        ['後退が最大',   best(r => -r.front),         r => `${r.front.toFixed(1)} px/s`,  r => r.front < 0],
        ['右移動が最大', best(r => r.right),          r => `${r.right.toFixed(1)} px/s`,  r => r.right > 0],
        ['左移動が最大', best(r => -r.right),         r => `${r.right.toFixed(1)} px/s`,  r => r.right < 0],
        ['回転が最大',   best(r => Math.abs(r.rot)),  r => `${r.rot.toFixed(1)} deg/s`,   null],
        ['回転が最小',   best(r => -Math.abs(r.rot)), r => `${r.rot.toFixed(1)} deg/s`,   null],
        ['速さが最大',   best(r => r.speed),          r => `${r.speed.toFixed(1)} px/s`,  null],
    ];
    let html = '<div style="font-weight:bold; margin:8px 0 4px; color:#FFD54F;">まとめ</div>';
    rows.forEach(([label, r, fmt, ok]) => {
        html += `<div>・${label}: ${(!ok || ok(r)) ? `Res=${r.res}, Rep=${r.rep} (${fmt(r)})` : '該当なし'}</div>`;
    });
    document.getElementById('t-summary').innerHTML = html;
}

async function startAutoTune() {
    if (isTuning) return;
    if (!bleCharacteristic) { alert("先にBluetoothを接続してください"); return; }
    if (Date.now() - latestCenterTime > 1000) { alert("カメラでマーカー(黄色)が検出できていません"); return; }
    if (Date.now() - latestAngleTime > 1000) { alert("前方マーカー(赤)が検出できていません。向きの計測に必要です"); return; }

    const cmdId = parseInt(document.getElementById('t-pattern').value);
    const wave = document.getElementById('t-wave').value; // 'normal' or 'new'
    const header = (wave === 'new') ? HEADER_MANUAL2 : HEADER_MANUAL;
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
    const estSec = Math.round(total * (0.5 + durMs / 1000 + 1.0));
    if (total > 100 && !confirm(`${total}通りで約${Math.round(estSec / 60)}分かかります。実行しますか?`)) return;

    // パターン1/2はRes1/Rep1、パターン3/4はRes2/Rep2をファームが使うので、対応する側を振る
    const usesPair1 = (cmdId === 1 || cmdId === 2);
    const curR1 = clampByte(document.getElementById('p-res1').value);
    const curP1 = clampByte(document.getElementById('p-rep1').value);
    const curR2 = clampByte(document.getElementById('p-res2').value);
    const curP2 = clampByte(document.getElementById('p-rep2').value);

    isTuning = true; tuneAbort = false; tuneResults = [];
    document.getElementById('t-summary').innerHTML = "";
    renderTuneResults();
    let consecutiveFails = 0, count = 0;

    try {
        for (const res of resList) {
            for (const rep of repList) {
                if (tuneAbort) break;
                count++;

                // 画面端に近づいていたら置き直しを待つ(自動では戻れない前提)
                if (!(await waitForSafePosition())) break;
                setTuneProgress(`${count}/${total} 計測中: Res=${res}, Rep=${rep} (推定残り${Math.round(estSec * (1 - count / total))}秒)`);

                const p = usesPair1 ? [res, rep, curR2, curP2] : [curR1, curP1, res, rep];
                await bleCharacteristic.writeValue(new Uint8Array([HEADER_PARAM, p[0], p[1], p[2], p[3]]));
                await sleep(200);

                const m = await measureTrial(cmdId, header, durMs);
                if (m === null) {
                    consecutiveFails++;
                    if (consecutiveFails >= 3) {
                        alert("マーカーを3回連続で見失ったため中止します。黄色・赤の両方が見えているか確認してください。");
                        tuneAbort = true; break;
                    }
                    continue;
                }
                consecutiveFails = 0;
                tuneResults.push({ pattern: cmdId, wave, res, rep, ...m });
                renderTuneResults();
            }
            if (tuneAbort) break;
        }
    } finally {
        try { await bleWrite(new Uint8Array([header, DIR_STOP, 0])); } catch (e) {}
        isTuning = false;
    }

    renderTuneSummary();
    setTuneProgress(tuneAbort
        ? `中止しました (${tuneResults.length}/${total}件計測済み)`
        : `完了! ${tuneResults.length}件計測しました。下のまとめとCSVを確認してください`);
}

function saveTuneCSV() {
    if (tuneResults.length === 0) { alert("結果がありません"); return; }
    let csv = "Pattern,Wave,Res,Rep,Front(px/s),Right(px/s),Rotation(deg/s),Speed(px/s)\n";
    tuneResults.forEach(r => csv += `${r.pattern},${r.wave},${r.res},${r.rep},${r.front.toFixed(2)},${r.right.toFixed(2)},${r.rot.toFixed(2)},${r.speed.toFixed(2)}\n`);
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
    a.download = `tune_${Date.now()}.csv`; a.click();
}