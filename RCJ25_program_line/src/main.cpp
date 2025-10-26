#include <Arduino.h>
#include <math.h> // round()関数のために保持

// 閾値の配列
int sikii_1[8] = {515, 506, 507, 498, 504, 510, 518, 523};//1代目白オム
int sikii_2[8] = {515, 500, 480, 490, 505, 538, 452, 480};//2代目黒オムニ (こちらを使用)
int sikii_3[8] = {0, 0, 0, 0, 0, 0, 0, 0};//ピンヘッダなし基盤

void setup() {
  Serial.begin(115200);
  // A0からA7まで、8つのアナログピンを入力に設定
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  // シリアルバッファをクリア
  while (Serial.available()>0){
    Serial.read();
  };

  // メインから開始信号を受け取るまで待機し、受け取ったら読み捨てる
  while (Serial.available()==0);
  Serial.read();
}

void loop(){
  int line_readings[8];
  line_readings[0] = analogRead(A0);
  line_readings[1] = analogRead(A1);
  line_readings[2] = analogRead(A2);
  line_readings[3] = analogRead(A3);
  line_readings[4] = analogRead(A4);
  line_readings[5] = analogRead(A5);
  line_readings[6] = analogRead(A6);
  line_readings[7] = analogRead(A7);

  // 検出されたピンの数をカウント
  int sumPin = 0;
  // line_readings配列を0または1の二値データに変換
  for (int i = 0; i < 8; i++) {
    // line_readings[i]は一時的に生データを保持。
    if (line_readings[i] > sikii_2[i]) {
      line_readings[i] = 0; // 閾値以上 = 反射が強い (白地/ラインなしを想定)
    } else {
      line_readings[i] = 1; // 閾値未満 = 反射が弱い (黒地/ラインありを想定)
      sumPin++;
    }
  }

  // 検出されたピンが0個の場合は処理を中断して次のループへ (void関数のエラー解消)
  if (sumPin == 0) return;

  // FIX 1: 可変長配列 (VLA) を避けるため、最大サイズ8の配列を使用
  // 検出されたピンのインデックス (0-7) を格納
  int OnPin[8];
  int idxOnPin = 0;
  for (int i = 0; i < 8; i++) {
    if (line_readings[i] == 1) {
      OnPin[idxOnPin] = i;
      idxOnPin++;
    }
  }

  // FIX 1: 可変長配列 (VLA) を避けるため、最大サイズ8の配列を使用
  // 検出されたピン間の間隔を格納
  int kankaku[8];
  // 検出されたピンが1個以上あることが sumPin == 0 のチェックで保証されている

  // 隣接するピン間の間隔を計算
  for (int i = 0; i < sumPin - 1; i++) {
    kankaku[i] = OnPin[i + 1] - OnPin[i];
  }
  // ラップアラウンド（最後のピンと最初のピンの間）の間隔を計算
  kankaku[sumPin - 1] = OnPin[0] - OnPin[sumPin - 1] + 8;

  // 最大間隔とそのインデックスを検索
  int max_kankaku = 0;
  int idxmax = 0;
  for (int i = 0; i < sumPin; i++) {
    if (max_kankaku < kankaku[i]) {
      max_kankaku = kankaku[i];
      idxmax = i;
    }
  }

  // 最大間隔の中央を目標方向として計算 (センサーインデックス 0.0〜8.0 の値)
  float go_angle = (float)OnPin[idxmax] + max_kankaku * 0.5f;

  // 角度 (0〜360度) に変換
  go_angle = (go_angle / 8.0f) * 360.0f;
  
  // 360度を超えた場合の正規化 (例: 365 -> 5)
  if (go_angle >= 360.0f) {
    go_angle = go_angle - 360.0f;
  }

  // FIX 4: map()の代わりに手動で浮動小数点数のスケーリングを実行 (0-360 -> 0-255)
  go_angle = go_angle * (255.0f / 360.0f);

  // FIX 3: 意味のないキャスト (int)go_angle; を削除し、
  // round() で丸めてから uint8_t にキャスト
  uint8_t data = (uint8_t)round(go_angle);
  
  // 角度データをシリアル送信
  Serial.write(data);
  //Serial.println(data);
}
