#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_MCP4725.h>
#include <math.h>

#define SERVICE_UUID        "0000aaaa-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000bbbb-0000-1000-8000-00805f9b34fb"

bool deviceConnected = false;

Adafruit_MCP4725 dac1;

// --- 状態管理 ---
bool isNewTypeWave = false; // 新波形モードかどうか
bool isAutoRunning = false; // 自動ループ中か
int manualId = 0;           // 手動実行中のパターンID (0=停止, 1~4=実行中)
bool isAdvancing = false; // 自動モードで前進中かどうか
int advanceDir = 0;      // 自動モードでの進行方向(0:右,1:前,2:左,3:後)

// --- パラメータ変数 ---
// 初期値を設定
int paramRes1 = 19;
int paramRep1 = 3;
int paramRes2 = 30;
int paramRep2 = 5;
int loopCount = 200;
int normX = 0;
int normY = 0;
int angle = 0;

// 半周期サイン波テーブル
// ESP32-C3はFPU非搭載でsin()のソフト演算が遅いため、毎サンプル計算せず参照する
uint16_t halfSineLUT[256];

void buildHalfSineLUT() {
  for (int i = 0; i < 256; i++) {
    halfSineLUT[i] = (uint16_t)(4095 * fabs(sin(i * 2 * 3.14 / 256 / 2)));
  }
}

// 4095*|sin(k*2*3.14/res/2)| 相当をテーブル参照で返す (res: 1~255)
static inline uint16_t halfSine(int k, int res) {
  return halfSineLUT[(k * 256) / res];
}

// すべてのGPIOピンをLowに
void stopAll() {
  digitalWrite(D2, LOW); digitalWrite(D3, LOW);
  digitalWrite(D6, LOW); digitalWrite(D7, LOW);
  digitalWrite(D8, LOW); digitalWrite(D10, LOW);
  dac1.setVoltage(0, false, 800000);
}

// 接続コールバック
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Connect!");
    }
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
      // 接続インターバル短縮を要求してコマンド遅延を減らす (単位1.25ms: 6=7.5ms, 12=15ms)
      pServer->updateConnParams(param->connect.remote_bda, 6, 12, 0, 400);
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Disconnect!");
      // 切断時に動作状態が残るとモータが回り続けてしまうので、停止して状態を戻す
      manualId = 0;
      isAutoRunning = false;
      isAdvancing = false;
      stopAll();
      // 即時startだと失敗することがあるため少し待ってから再アドバタイズ
      delay(200);
      pServer->getAdvertising()->start();
    }
};

// 停止指令が来ていないかチェックする関数
// 手動モードで指を離した(manualIdが0になった)ら true を返す
bool checkStop() {
  // 停止(manualId=0)かつ自動モードでもなければ中断
  if (manualId == 0 && !isAutoRunning) return true;

  return false;
}

void move(int res, int repeatCount, bool isDir){
  if(res <= 0) res = 1;
  if(repeatCount < 0) repeatCount = 0;

  // delayMicroseconds(100);
  if(checkStop()) return; // ループの途中でも指が離れたら即終了
  for(int i = 0; i < repeatCount; i++){

    if(isDir){
      digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
    }else{
      digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
    }
    for(uint8_t k=0; k<res; k++){
      dac1.setVoltage(halfSine(k, res), false, 800000);
    }
  }

  // delayMicroseconds(100);
  if(checkStop()) return;

  if(isDir){
    digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
  }else{
    digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
  }
  for(uint8_t k=0; k<res; k++){
    dac1.setVoltage(halfSine(k, res), false, 800000);
  }
}

void move2(int res, int repeatCount, bool isDir){
  if(res <= 0) res = 1;
  if(repeatCount < 0) repeatCount = 0;

  delayMicroseconds(100);
  for(int i = 0; i < repeatCount; i++){
  if(checkStop()) return; // ループの途中でも指が離れたら即終了

    if(isDir){
      digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
    }else{
      digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
    }
    for(uint8_t k=0; k<res; k++){
      dac1.setVoltage(halfSine(k, res), false, 800000);
    }
  }

  delayMicroseconds(100);
  if(checkStop()) return;
  
  if(isDir){
    digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
  }else{
    digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
  }
  for(uint8_t k=0; k<res; k++){
    dac1.setVoltage(halfSine(k, res), false, 800000);
  }
}

void move3(int res1, int res2, bool isDir){
    if(res1 <= 0) res1 = 1;
    if(res2 <= 0) res2 = 1;
  if(checkStop()) return; // ループの途中でも指が離れたら即終了
  if(isDir){
    digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
  }else{
    digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
  }
  for(uint8_t k=0; k<res1; k++){
    dac1.setVoltage(halfSine(k, res1), false, 800000);
  }

  if(isDir){
    digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
  }else{
    digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
  }
  for(uint8_t k=0; k<res2; k++){
    dac1.setVoltage(halfSine(k, res2), false, 800000);
  }
}

void move4(int res1, int res2, bool isDir){
  if(res1 <= 0) res1 = 1;
  if(res2 <= 0) res2 = 1;
  if(checkStop()) return; // ループの途中でも指が離れたら即終了
  if(isDir){
    digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
  }else{
    digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
  }
  for(uint8_t k=0; k<res1; k++){
    dac1.setVoltage(halfSine(k, res1), false, 800000);
  }
  delay(2);
  
  if(isDir){
    digitalWrite(D2,HIGH); digitalWrite(D3,LOW);
  }else{
    digitalWrite(D2,LOW); digitalWrite(D3,HIGH);
  }
  for(uint8_t k=0; k<res2; k++){
    dac1.setVoltage(halfSine(k, res2), false, 800000);
  }
  delay(2);
}

void autoMove(int targetX, int targetY, int nowX, int nowY, int nowAngle){//nowX:0~255 nowY:0~255
  int targetAngle = atan2((targetY - nowY), (targetX - nowX)) * 180 / 3.14;//[deg]
  if(targetAngle < 0) targetAngle += 360;
  int diffAngle = (targetAngle - nowAngle + 360) % 360;//[0~360)
  if(isAdvancing == false){
    if((315 <= diffAngle && diffAngle < 360) || (0 <= diffAngle && diffAngle < 45)){
      advanceDir = 1; // 前進
    }else if(45 <= diffAngle && diffAngle < 135){
      advanceDir = 2; // 左進
    }else if(135 <= diffAngle && diffAngle < 225){
      advanceDir = 3; // 後退
    }else if(225 <= diffAngle && diffAngle < 315){
      advanceDir = 0; // 右進
    }
    isAdvancing = true;
  }else{//isAdvancing == trueなら
    digitalWrite(D6, LOW); digitalWrite(D8, LOW); digitalWrite(D7, LOW); digitalWrite(D10, LOW);
    if(advanceDir == 1){
      // 前進
      digitalWrite(D6, HIGH);  for(uint8_t i=0; i < 10; i++) move(paramRes1, paramRep1, true);
      if(180 <= diffAngle && diffAngle < 360){
        isAdvancing = false;
      }
    }else if(advanceDir == 2){
      // 左進
      digitalWrite(D10, HIGH); move4(paramRes2, paramRep2, false);
      if((0 <= diffAngle && diffAngle < 90) || (270 <= diffAngle && diffAngle < 360)){
        isAdvancing = false;
      }
    }else if(advanceDir == 3){
      // 後退
      digitalWrite(D8, HIGH);  for(uint8_t i=0; i < 10; i++) move(paramRes1, paramRep1, false);
      if(0 <= diffAngle && diffAngle < 180){
        isAdvancing = false;
      }
    }else if(advanceDir == 0){
      // 右進
      digitalWrite(D7, HIGH);  move4(paramRes2, paramRep2, true);
      if(90 <= diffAngle && diffAngle < 270){
        isAdvancing = false;
      }
    }
  }
}

// 指定したパターンの設定で1単位だけ動かす
void runPatternStep(int id) {
  // ピン設定
  digitalWrite(D6, LOW); digitalWrite(D8, LOW); digitalWrite(D7, LOW); digitalWrite(D10, LOW);
  
  switch(id) {
    // move() や move2() にグローバル変数の paramResX, paramRepX を渡すようにしています
    case 1: digitalWrite(D6, HIGH);  for(uint8_t i=0; i < 10; i++) move(paramRes1, paramRep1, false);  break;
    case 2: digitalWrite(D8, HIGH);  for(uint8_t i=0; i < 10; i++) move(paramRes1, paramRep1, true); break;
    case 3: digitalWrite(D7, HIGH);  move4(paramRes2, paramRep2, true);  break;
    case 4: digitalWrite(D10, HIGH); move4(paramRes2, paramRep2, false); break;
  }
}

// 指定したパターンの設定で新波形で1単位だけ動かす
void runPatternStep2(int id) {
  // ピン設定
  digitalWrite(D6, LOW); digitalWrite(D8, LOW); digitalWrite(D7, LOW); digitalWrite(D10, LOW);
  
  switch(id) {
    // move() や move2() にグローバル変数の paramResX, paramRepX を渡すようにしています
    case 1: digitalWrite(D6, HIGH);  for(uint8_t i=0; i < 10; i++) move(paramRes1, paramRep1, false);  break;
    case 2: digitalWrite(D8, HIGH);  for(uint8_t i=0; i < 10; i++) move(paramRes1, paramRep1, true); break;
    case 3: digitalWrite(D7, HIGH);  for(uint8_t i=0; i < 10; i++) move4(paramRes2, paramRep2, true); break;
    case 4: digitalWrite(D10, HIGH); for(uint8_t i=0; i < 10; i++) move4(paramRes2, paramRep2, false); break;
  }
}

float Vbatt(){
  uint32_t vbatt = 0;
  for(uint8_t i=0; i<16; i++){
    vbatt += analogReadMilliVolts(A0);
  }
  float vbattf = 2 * vbatt / 16 / 1000.0; //[V]
  printf("Vbatt: %.2f V\n", vbattf);

  return vbattf;
}

// 書き込みコールバック
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // 生データを取得
    uint8_t* data = pCharacteristic->getData();
    std::string valueStr = pCharacteristic->getValue(); // 長さ取得用
    int len = valueStr.length();

    if (len > 0) {
      int header = data[0]; // 1バイト目で分岐

      // --- 既存の制御 (3バイト) ---
      if (header == 0x01 && len >= 3) {
          // モード1: 手動制御
          isAutoRunning = false;
          manualId = data[1];
          isNewTypeWave = false;
        } 
        else if(header == 0x04 && len >= 3){
          //モード5: 手動モード(新波形モード)
          isAutoRunning = false;
          manualId = data[1];
          isNewTypeWave = true;
      }
      else if (header == 0x02 && len >= 3) {
          // モード2: 自動追尾開始など
          isAutoRunning = true;
          normX = data[1];
          normY = data[2];
          angle = (data[3] << 8) | data[4];
      }
      // --- ★追加: パラメータ更新 (5バイト) ---
      else if (header == 0x03 && len >= 5) {
          // 受信データ: [0x03, Res1, Rep1, Res2, Rep2]
          paramRes1 = data[1];
          paramRep1 = data[2];
          paramRes2 = data[3];
          paramRep2 = data[4];

          // Serial.printf("Params Updated: Res1=%d, Rep1=%d, Res2=%d, Rep2=%d\n", 
          //               paramRes1, paramRep1, paramRes2, paramRep2);
      }
    }
  }
};



void setup() {
  buildHalfSineLUT();
  // MDのGPIOピン設定
  pinMode(D2,OUTPUT); pinMode(D3,OUTPUT);
  pinMode(D6,OUTPUT); pinMode(D7,OUTPUT);
  pinMode(D8,OUTPUT); pinMode(D10,OUTPUT);
  pinMode(A0, INPUT);//電源監視用
  // dacの設定
  dac1.begin(0x62);

  stopAll();

  Serial.begin(115200);
  // 個体ごとに固有のBLE名にする(MAC下位2バイト)。2台以上を見分けられるように
  uint64_t mac = ESP.getEfuseMac();
  char devName[24];
  snprintf(devName, sizeof(devName), "XIAO_LRA_%04X", (uint16_t)(mac & 0xFFFF));
  Serial.printf("BLE name: %s\n", devName);
  BLEDevice::init(devName);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE|
                      BLECharacteristic::PROPERTY_WRITE_NR|
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  // NOTIFYを使うにはCCCD(0x2902)記述子が必要。
  // 無いとブラウザのstartNotifications()が "GATT Error: Not supported" で失敗する
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.println("Waiting for connection...");
}

void loop() {
  // --- 手動モード実行中 (manualIdが1~4の間) ---
  if (manualId > 0) {
    if(isNewTypeWave){
      runPatternStep2(manualId);
    }else{
      runPatternStep(manualId);
      // runPatternStepの中でcheckStop()しているので、指を離せば次回ループでmanualId=0になり止まる
    }
  }
  // 自動モードの処理が必要であればここに記述
  else if (isAutoRunning) {
    autoMove(0, 0, normX, normY, angle);
  }

  // ESP32-C3は単一コア。loop()がyieldせず回り続けるとBLEホストタスクが
  // CPUを取れず、接続維持に失敗して勝手に切れる(アイドル時のbusy-spinも同様)。
  // 毎周回わずかに譲ってBLEスタックに処理時間を与える
  delay(1);
}