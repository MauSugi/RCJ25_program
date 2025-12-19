// 名古屋大会

//gitテスト

// 標準数学関数のインクルード (PlatformIOでエラーを避けるため)
#include <Arduino.h>
#include <math.h> 

// 角度
float rad(float deg) {
    return (deg / 180.0f) * PI;
}
float deg(float rad) {
    return (rad / PI) * 180.0f;
}

// デバッグ用LEDのピン
const int L1 = 52;
const int L2 = 30;
const int L3 = 27;
const int L4 = 25;
const int L5 = 3;
const int L6 = 7;
const int L7 = 24;
const int L8 = 35;
const int L9 = 36;
const int L10 = 32;
const int L11 = 42;
const int L12 = 44;


// BNO関連
#include <Wire.h>
byte ADDRESS = 0x28; 
byte EULER_REGISTER = 0x1A;
int merge(byte low, byte high){
    int result = low | (high << 8);
    if (result > 32767) { 
        result -= 65536;
    }
    return result; 
}

void writeToBNO(byte reg, byte val, int dly){
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(false);
    delay(dly); 
}
void initBNO(){
    Wire.beginTransmission(ADDRESS); 
    Wire.write(0x00);
    Wire.endTransmission(false); 
    Wire.requestFrom(ADDRESS, 1);
    if (Wire.read() == 0xa0) {
        //Serial.println("BNO055 found.");     
        writeToBNO(0x3d, 0x00, 80);// operating mode = config mode     
        writeToBNO(0x3f, 0x20, 1000);// sys_trigger = rst_sys     
        writeToBNO(0x3e, 0x00, 80);// pwr_mode = normal mode     
        writeToBNO(0x3f, 0x80, 1000);// sys trigger = clk_sel ex_osc     
        writeToBNO(0x3d, 0x0c, 80);// operating mode = ndof   
        Serial.println("BNO055 initialized.");
    } else {
        while (1) {
            //Serial.println("BNO055 not found..");
            digitalWrite(L1, HIGH);
            digitalWrite(L2, HIGH);
            digitalWrite(L3, HIGH);
            digitalWrite(L4, HIGH);
            digitalWrite(L5, HIGH);
            digitalWrite(L6, HIGH);
            digitalWrite(L7, HIGH);  
            digitalWrite(L8, HIGH);
            digitalWrite(L9, HIGH);
            digitalWrite(L10, HIGH);
            digitalWrite(L11, HIGH);
            digitalWrite(L12, HIGH);
            delay(1000);
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
            delay(1000);
        }
    } 
}

// 機体の角度(-180~180)を返す  
float compass(){
    int euler[6];

    Wire.beginTransmission(ADDRESS);
    Wire.write(EULER_REGISTER);
    Wire.endTransmission(false);

    Wire.requestFrom(ADDRESS, 6);
    byte buffer[6];
    Wire.readBytes(buffer, 6);

    euler[0] = merge(buffer[0], buffer[1]);
    euler[1] = merge(buffer[2], buffer[3]);
    euler[2] = merge(buffer[4], buffer[5]);
    
    float yaw = float(euler[0]) / 16.0;
    if (180 < yaw && yaw <= 360) {
        yaw = map(yaw , 180, 360, -180, 0);
    }
    return yaw;
}


// ボールセンサ関連
const int IR1 = A15;
const int IR2 = A10;
const int IR3 = A13;
const int IR4 = A3;
const int IR5 = A4;
const int IR6 = A11;
const int IR7 = A1;
const int IR8 = A0;
const int IR9 = A5;
const int IR10 = A2;
const int IR11 = A6;
const int IR12 = A8;

// ボールの角度(0~360)を返す、反応なし時は-1を返す
float IR_angle(){
    int IR[12];
    IR[0] = analogRead(IR1);
    IR[1] = analogRead(IR2);
    IR[2] = analogRead(IR3);
    IR[3] = analogRead(IR4);
    IR[4] = analogRead(IR5);
    IR[5] = analogRead(IR6);
    IR[6] = analogRead(IR7);
    IR[7] = analogRead(IR8);
    IR[8] = analogRead(IR9);
    IR[9] = analogRead(IR10);
    IR[10] = analogRead(IR11);
    IR[11] = analogRead(IR12);

    /*
    //値出力
    for (int i=0; i<12; i++) {
        Serial.print(i+1);
        Serial.print(" : ");
        Serial.print(IR[i]);
        Serial.print(" ");
    }
    Serial.println("");
    */

    int sum = 0;//値の総和
    int x = 0;//値のx成分の総和
    int y = 0;//y成分

    for (int i=0; i<12; i++) {
        IR[i] = map(IR[i], 0, 1023, 1023, 0);
        //配列を表示
        sum += IR[i];
        x += IR[i] * cos(3.1415 / 6.0 * i);
        y += IR[i] * sin(3.1415 / 6.0 * i);
    }

    //アークタンジェントで角度にする
    float angle = deg(atan2(y, x));

    if (sum > 1000) {
        if(angle < -165 || 165 < angle){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, HIGH);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < -135){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, HIGH);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < -105){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, HIGH);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < -75){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, HIGH);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < -45){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, HIGH);
            digitalWrite(L12, LOW);
        }else if(angle < -15){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, HIGH);
        }else if(angle < 15){
            digitalWrite(L1, HIGH);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < 45){
            digitalWrite(L1, LOW);
            digitalWrite(L2, HIGH);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < 75){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, HIGH);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < 105){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, HIGH);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < 135){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, HIGH);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(angle < 165){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, HIGH);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }
        
        // 角度を0~360の範囲に変換して返す
        if (angle < 0){
            return map(angle, -180, 0, 180, 360);
        }else{
            return angle;
        }

    } else {
        // 反応がない時はLEDを全て消灯
        digitalWrite(L1, LOW);
        digitalWrite(L2, LOW);
        digitalWrite(L3, LOW);
        digitalWrite(L4, LOW);
        digitalWrite(L5, LOW);
        digitalWrite(L6, LOW);
        digitalWrite(L7, LOW);
        digitalWrite(L8, LOW);
        digitalWrite(L9, LOW);
        digitalWrite(L10, LOW);
        digitalWrite(L11, LOW);
        digitalWrite(L12, LOW);
        return -1;
    }
}
// ボールの角度を入れると回り込みの角度を返す
float mawarikomi(float IR){
  if(IR < 180){
    //右側
    if(IR < 10){
      return IR;
    }else if(IR < 30){
      return IR * 2.5 - 15.0;
    }else if(IR < 90){
      return IR * 1.5 + 15.0;
    }else{//180→280
      return IR * 0.833 + 75.0;
    }
  }else{
    //左側
    IR = map(IR, 180, 360, -180, 0);
    if(-10 < IR){
      return IR;
    }else if(-30 < IR){
      IR = IR * 2.5 + 15.0;
      return map(IR, 0, -360, 360, 0);
    }else if(-90 < IR){
      IR = IR * 1.5 - 15.0;
      return map(IR, 0, -360, 360, 0);
    }else{
      IR = IR * 0.833 - 75.0;
      return map(IR, 0, -360, 360, 0);
    }
  }
}



// モーター関連
// 電源レバーでかい方
/*
const int M1a = 4;
const int M1b = 2;
const int M2a = 8;
const int M2b = 6;
const int M3a = 10;
const int M3b = 12;
const int M4a = 45;
const int M4b = 47;
*/
// 電源レバー小さい方
const int M1a = 47;
const int M1b = 45;
const int M2a = 10;
const int M2b = 12;
const int M3a = 8;
const int M3b = 6;
const int M4a = 4;
const int M4b = 2;


// 最小出力と最大出力を決める。
const int M1_MIN = 30;
const int M2_MIN = 30;
const int M3_MIN = 30;
const int M4_MIN = 30;
const int M_MAX = 250;

// 各モーターを動かす関数
void M1move(float spd){
    spd = constrain(spd, -M_MAX, M_MAX);
    if (spd > 1) {
        spd = map(spd, 1, M_MAX, M1_MIN, M_MAX);
        analogWrite(M1a, spd);
        analogWrite(M1b, 0);
    }else if(spd < -1){
        spd = map(spd, -1, -M_MAX, -M1_MIN, -M_MAX);
        analogWrite(M1a, 0);
        analogWrite(M1b, abs(spd));
    }else{
        analogWrite(M1a, 0);
        analogWrite(M1b, 0);
    }
}
void M2move(float spd){
    spd = constrain(spd, -M_MAX, M_MAX);
    if (spd > 1) {
        spd = map(spd, 1, M_MAX, M2_MIN, M_MAX);
        analogWrite(M2a, spd);
        analogWrite(M2b, 0);
    }else if(spd < -1){
        spd = map(spd, -1, -M_MAX, -M2_MIN, -M_MAX);
        analogWrite(M2a, 0);
        analogWrite(M2b, abs(spd));
    }else{
        analogWrite(M2a, 0);
        analogWrite(M2b, 0);
    }
}
void M3move(float spd){
    spd = constrain(spd, -M_MAX, M_MAX);// constrainの結果をspdに再代入
    if (spd > 1) {
        spd = map(spd, 1, M_MAX, M3_MIN, M_MAX);
        analogWrite(M3a, spd);
        analogWrite(M3b, 0);
    }else if(spd < -1){
        spd = map(spd, -1, -M_MAX, -M3_MIN, -M_MAX);
        analogWrite(M3a, 0);
        analogWrite(M3b, abs(spd));
    }else{
        analogWrite(M3a, 0);
        analogWrite(M3b, 0);
    }
}
void M4move(float spd){
    spd = constrain(spd, -M_MAX, M_MAX);
    if (spd > 1) {
        spd = map(spd, 1, M_MAX, M4_MIN, M_MAX);// M4_MINが80になっていますが、定数に合わせるためM4_MINを使用
        analogWrite(M4a, spd);
        analogWrite(M4b, 0);
    }else if(spd < -1){
        spd = map(spd, -1, -M_MAX, -M4_MIN, -M_MAX);
        analogWrite(M4a, 0);
        analogWrite(M4b, abs(spd));
    }else{
        analogWrite(M4a, 0);
        analogWrite(M4b, 0);
    }
}

// 各モーターの出力を保存する配列
float MotorPower[4] = {0, 0, 0, 0};

// 移動の出力と回転の出力の割合を指定する変数
float idou_ratio = 0.9;
float spin_ratio = 0.1;

// 入力された角度に移動するモーター出力を計算して、配列に代入する
void calc_idou(float angle){// degree(0~360)
    MotorPower[0] += sin(rad(angle-45.0)) * M_MAX * idou_ratio;
    MotorPower[1] += sin(rad(angle-135.0)) * M_MAX * idou_ratio;
    MotorPower[2] += sin(rad(angle-225.0)) * M_MAX * idou_ratio;
    MotorPower[3] += sin(rad(angle-315.0)) * M_MAX * idou_ratio;
}
// 姿勢制御で入力された制御値で機体を回転させるモーター出力を計算して、配列に代入する
void calc_spin(float power){
    MotorPower[0] += power * M_MAX * spin_ratio;
    MotorPower[1] += power * M_MAX * spin_ratio;
    MotorPower[2] += power * M_MAX * spin_ratio;
    MotorPower[3] += power * M_MAX * spin_ratio;
}
// 配列に代入された各モーターの出力でモーターを動かす関数
void Move(){
    M1move(MotorPower[0]);
    M2move(MotorPower[1]);
    M3move(MotorPower[2]);
    M4move(MotorPower[3]);
}

// PID制御関連
const float Kp = 2.5;
const float Ki = 1.0;
const float Kd = 0.1;
unsigned long pretime;
float dt;
float P;
float preP = 0.0;
float I;
float D;
float sisei = 0.0;

void setup(){
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(L3, OUTPUT);
    pinMode(L4, OUTPUT);
    pinMode(L5, OUTPUT);
    pinMode(L6, OUTPUT);
    pinMode(L7, OUTPUT);
    pinMode(L8, OUTPUT);
    pinMode(L9, OUTPUT);
    pinMode(L10, OUTPUT);
    pinMode(L11, OUTPUT);
    pinMode(L12, OUTPUT);
    pinMode(M1a, OUTPUT);
    pinMode(M1b, OUTPUT);
    pinMode(M2a, OUTPUT);
    pinMode(M2b, OUTPUT);
    pinMode(M3a, OUTPUT);
    pinMode(M3b, OUTPUT);
    pinMode(M4a, OUTPUT);
    pinMode(M4b, OUTPUT);

    // LEDを光らせる
    digitalWrite(L1, HIGH);
    digitalWrite(L2, HIGH);
    digitalWrite(L3, HIGH);
    digitalWrite(L4, HIGH);
    digitalWrite(L5, HIGH);
    digitalWrite(L6, HIGH);
    digitalWrite(L7, HIGH);
    digitalWrite(L8, HIGH);
    digitalWrite(L9, HIGH);
    digitalWrite(L10, HIGH);
    digitalWrite(L11, HIGH);
    digitalWrite(L12, HIGH);
    // UART通信開始とBNOの初期化
    Serial.begin(115200);
    Wire.begin();
    initBNO();
    // 初期化が終わったらLEDを消す
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    digitalWrite(L3, LOW);
    digitalWrite(L4, LOW);
    digitalWrite(L5, LOW);
    digitalWrite(L6, LOW);
    digitalWrite(L7, LOW);
    digitalWrite(L8, LOW);
    digitalWrite(L9, LOW);
    digitalWrite(L10, LOW);
    digitalWrite(L11, LOW);
    digitalWrite(L12, LOW);
    
    while (digitalRead(33) == LOW);
    delay(5);
    while (digitalRead(33) == HIGH);
    delay(5);
    // スイッチが一回押されるまで待つ


    while (digitalRead(33) == LOW){// もう一度スイッチが押されるまで0度のLEDを光らせる。
        float tyuusin = compass();
        tyuusin = -tyuusin;

        if(tyuusin < -165 || 165 < tyuusin){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, HIGH);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < -135){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, HIGH);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < -105){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, HIGH);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < -75){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, HIGH);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < -45){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, HIGH);
            digitalWrite(L12, LOW);
        }else if(tyuusin < -15){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, HIGH);
        }else if(tyuusin < 15){
            digitalWrite(L1, HIGH);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < 45){
            digitalWrite(L1, LOW);
            digitalWrite(L2, HIGH);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < 75){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, HIGH);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < 105){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, HIGH);
            digitalWrite(L5, LOW);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < 135){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, HIGH);
            digitalWrite(L6, LOW);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }else if(tyuusin < 165){
            digitalWrite(L1, LOW);
            digitalWrite(L2, LOW);
            digitalWrite(L3, LOW);
            digitalWrite(L4, LOW);
            digitalWrite(L5, LOW);
            digitalWrite(L6, HIGH);
            digitalWrite(L7, LOW);
            digitalWrite(L8, LOW);
            digitalWrite(L9, LOW);
            digitalWrite(L10, LOW);
            digitalWrite(L11, LOW);
            digitalWrite(L12, LOW);
        }
    }

    // サブマイコン何かデータを送って、ラインのプログラムも開始させる。
    Serial.write(1);
    pretime = micros();
}

void loop() {
    // モーター出力の配列を0で初期化させる。
    MotorPower[0] = 0;
    MotorPower[1] = 0;
    MotorPower[2] = 0;
    MotorPower[3] = 0;
    
    // ライン反応時
    while (Serial.available() > 0){// 白線を検知した信号の分だけ、白線の反対に動く
        int data = Serial.read();
        float Line_angle = map(data, 0, 255, 0, 360);
        idou_ratio = 0.7;
        calc_idou(Line_angle);
        idou_ratio = 0.3;
        calc_idou(IR_angle());
        Move();// 白線の反対に動くモーター出力と、ボールを追いかけるモーター出力を合成
        MotorPower[0] = 0;
        MotorPower[1] = 0;
        MotorPower[2] = 0;
        MotorPower[3] = 0;
    }
    
    idou_ratio = 0.9;
    spin_ratio = 0.1;

    // 姿勢制御
    float angle = compass();
    //Serial.println(angle);
    dt = float(micros()-pretime) / 1000000.0;
    P = (0.0 - angle) / 180.0;
    I += P * dt;
    D = (P - preP) / dt;
    preP = P;
    sisei = Kp * P + Ki * I + Kd * D;
    if(-3 < angle && angle < 3){
        sisei = 0;
    }
    calc_spin(sisei);// 姿勢制御分の出力を配列に足す
    
    // Serial.println(sisei);
    
    // ボールアプローチ
    float temp_IR = IR_angle();
    Serial.println(temp_IR);
    if (temp_IR != -1){// ボールの反応がある時は回り込み分の出力を配列に足す
        calc_idou(mawarikomi(IR_angle()));
    }

    /*
    for (int i=0; i<4; i++){
        Serial.print(MotorPower[i]);
        Serial.print("  ");
    }
    Serial.println("");
    */

    //if (angle < -20 || 20 < angle){// 機体が大きく中心を向いていないとき、移動の出力割合を0にしてすぐ戻るようにする。
        //idou_ratio = 0.0;
        //spin_ratio = 0.7;
    //}else{
    //    idou_ratio = 0.9;
    //    spin_ratio = 0.1;
    //}

    // 最終モーター出力
    Move();
    pretime = micros();
}
