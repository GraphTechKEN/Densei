//Arduino Micro または Leonard を使用してください

//簡単な説明
//コマンドに対応しました。デリミタ:CR、Baud:115200、DataBits:8、StopBit:1

//更新履歴
//V1 初版
//V6 外部調整対応、I/O部修正

#include <EEPROM.h>
#include <Adafruit_MCP4725.h>

//入力ピンアサイン
#define PIN_Disp_0 11 //故障　未使用
#define PIN_Disp_1 9//直通
#define PIN_Disp_2 10//EB
#define PIN_Disp_3 A2//抑速
#define PIN_Disp_4 A3//電制
#define PIN_Disp_5 A4//ATS白色
#define PIN_Disp_6 A5//ATS警報
#define PIN_JyoyoMax 7//常用最大(小田急)
#define PIN_VPReg 8//電空レギュレータ制御用リレー

//自動ブレーキ関係
#define Pin_In_BP A0  // Analog input pin that the potentiometer is attached to
#define Pin_In_FV A1  // Analog input pin that the potentiometer is attached to

//↓デバッグのコメント(//)を解除するとシリアルモニタでデバッグできます
//#define DEBUG

Adafruit_MCP4725 dac;

int16_t FV_v_min = 100;   //102
int16_t FV_v_max = 800;  //104
int16_t BP_v_min = 100;   //106
int16_t BP_v_max = 800;  //108
int16_t BC_p_min = 10;   //110
int16_t ave_ratio = 95;  //112

int16_t iFV_press_temp = 0;        // value read from the pot
int16_t iBP_press_temp = 0;        // value read from the pot

float fFV_press = 0.0;
float fBP_press = 0.0;

float fBC_press = 0.0;

bool AutoairBrake_Mode = false; //自動ブレーキモード fBC_press圧力が10kPa以上で作動(暫定)
//自動ブレーキ関係

bool EBmode = false; //EB作動
bool EB_JR = false; //EB表示灯
bool EB_JR_move_E = false; //EB作動(E電磁給排弁用)
bool EB_JR_latch = false; //EB作動ラッチ
unsigned long EB_JR_Timer = 0;

bool ATS_ERR = false; //ATS表示灯
bool ATS_ERR_move_E = false; //ATS作動(E電磁給排弁用)
bool ATS_ERR_latch = false; //ATS作動ラッチ
bool ATS_Norm = false; //ATS白色表示灯
bool Densei = false;  //電制表示灯
bool Oer_Kaisei = false;  //回生(小田急)
bool Broken = false; //故障表示灯
bool Chokutsu = false; //直通表示灯
bool Yokusoku = false; //抑速表示灯
bool JyoyoMax = false; //常用最大(小田急)バルブ用
unsigned long ATS_ERR_Timer = 0;

uint16_t b = 0;
uint16_t pressure = 0;
uint16_t pressure_latch = 0;

bool MON = false;
uint32_t MON_Timer = 0;
uint16_t MON_Interval = 100;

//String strbve = "0000/0/ 00000/000000/0000/0";
String strbve = "";

void setup() {

  pinMode(4, INPUT_PULLUP); //ブレーキ弁非常位置入力
  pinMode(5, INPUT_PULLUP); //連動切替
  pinMode(6, INPUT_PULLUP); //JR/小田急切替
  pinMode(7, OUTPUT); //常用最大ブレーキ作用
  pinMode(PIN_VPReg, OUTPUT); //電空レギュレータ作用
  pinMode(12, OUTPUT); //E電磁給排弁作用

  pinMode(PIN_Disp_0, OUTPUT); //故障
  pinMode(PIN_Disp_1, OUTPUT); //直通
  pinMode(PIN_Disp_2, OUTPUT); //EB
  pinMode(PIN_Disp_3, OUTPUT); //抑速
  pinMode(PIN_Disp_4, OUTPUT); //電制
  pinMode(PIN_Disp_5, OUTPUT); //ATS白色
  pinMode(PIN_Disp_6, OUTPUT); //ATS警報

  digitalWrite(PIN_Disp_0, 0); //故障
  digitalWrite(PIN_Disp_1, 0); //直通
  digitalWrite(PIN_Disp_2, 0); //EB
  digitalWrite(PIN_Disp_3, 0); //抑速
  digitalWrite(PIN_Disp_4, 0); //電制
  digitalWrite(PIN_Disp_5, 0); //ATS白色
  digitalWrite(PIN_Disp_6, 0); //ATS警報
  digitalWrite(PIN_JyoyoMax, 0); //常用最大ブレーキ作用
  digitalWrite(PIN_VPReg, 0); //電空レギュレータ作用
  digitalWrite(12, 0); //E電磁給排弁作用

  dac.begin(0x60);

  //初回書き込みチェック
  uint16_t b = 0;
  EEPROM.get(100, b);
  if (b != 1) {
    EEPROM.put(102, 98);    //FV_v_min
    EEPROM.put(104, 800);   //FV_v_max
    EEPROM.put(106, 98);    //BP_v_min
    EEPROM.put(108, 800);   //BP_v_max
    EEPROM.put(110, 10);    //BC_p_min
    EEPROM.put(112, 95);    //ave_ratio
    EEPROM.put(114, 100);    //MON_Interval
    //初回書き込みフラグセット
    EEPROM.put(100, b = 1);
  } else {
    EEPROM.get(102, FV_v_min);
    EEPROM.get(104, FV_v_max);
    EEPROM.get(106, BP_v_min);
    EEPROM.get(108, BP_v_max);
    EEPROM.get(110, BC_p_min);
    EEPROM.get(112, ave_ratio);
    EEPROM.get(114, MON_Interval);
  }

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.setTimeout(10);
  Serial1.setTimeout(10);


  //自動ブレーキ関係
  fFV_press = map(analogRead(Pin_In_FV), FV_v_min, FV_v_max, 0, 490);  //FV制御空気ダメ圧力RAW
  fBP_press = map(analogRead(Pin_In_BP), BP_v_min, BP_v_max, 0, 490);  //fBC_pressブレーキシリンダ圧力RAW
  //自動ブレーキ関係

  EB_JR_Timer = millis();
  ATS_ERR_Timer = millis();

  MON_Timer = millis();
}

void loop() {

  bool lock = !digitalRead(5); //連動切替
  bool JR = !digitalRead(6); //JR/小田急切替
  strbve = "";
  if (Serial.available()) {
    strbve = Serial.readStringUntil('\r');
  }
  if (Serial1.available()) {
    strbve = Serial1.readStringUntil('\r');
  }
  if (strbve.startsWith("WR ")) {
    //設定モード
    String s = "";
    uint8_t device = strbve.substring(3, 6).toInt();
    int16_t num = strbve.substring(7, 12).toInt();
    switch (device) {

      //FV_v_min
      case 102:
        if (num < 0 || num > FV_v_max) {
          s = "E1 " + String(device);
        } else {
          s = rw_eeprom(device, &num, &FV_v_min, true);
        }
        break;

      //FV_v_max
      case 104:
        if (num > 1023 || num < FV_v_min) {
          s = "E1 " + String(device);
        } else {
          s = rw_eeprom(device, &num, &FV_v_max, true);
        }
        break;

      //BP_v_min
      case 106:
        if (num < 0 || num > BP_v_max) {
          s = "E1 " + String(device);
        } else {
          s = rw_eeprom(device, &num, &BP_v_min, true);
        }
        break;

      //BP_v_max
      case 108:
        if (num > 1023 || num < BP_v_min) {
          s = "E1 " + String(device);
        } else {
          s = rw_eeprom(device, &num, &BP_v_max, true);
        }
        break;

      //BC_p_min
      case 110:
        if (num < 0 || num > 50) {
          s = "E1 " + String(device);
        } else {
          s = rw_eeprom(device, &num, &BC_p_min, true);
        }
        break;

      //ave_ratio
      case 112:
        if (num < 1 || num > 99) {
          s = "E1 " + String(device);
        } else {
          s = rw_eeprom(device, &num, &ave_ratio, true);
        }
        break;

      //MON_Interval
      case 114:
        if (num < 1 || num > 65535) {
          s = "E1 " + String(device);
        } else {
          s = rw_eeprom(device, &num, &num, true);
          MON_Interval = (uint32_t) num;
        }
        break;

    }
    Serial.println(s);
  } else if (strbve.startsWith("RD ")) {
    //読取モード
    String s = "";
    uint8_t device = strbve.substring(3, 6).toInt();
    int16_t num = 0;
    s = rw_eeprom(device, &num, &num, false);
    Serial.println(s);
  } else if (strbve.startsWith("MON")) {
    //表示モード
    int16_t num = strbve.substring(3, 6).toInt();
    MON = (num == 1);
  }

  if (strbve.length() > 14 && lock) {

    //ATS白色表示灯抽出
    ATS_Norm = strbve.substring(14, 15).toInt();

    //ATS警報抽出
    ATS_ERR = strbve.substring(15, 16).toInt();

    //直通抽出
    Chokutsu = strbve.substring(17, 18).toInt();

    //抑速抽出
    Yokusoku = strbve.substring(19, 20).toInt();

    //JRモード
    if (JR) {

      //EB抽出
      EB_JR = strbve.substring(16, 17).toInt();//EB(JR)

      //EB作動時 E電磁給排弁開始タイマーセット
      if (EB_JR && !EB_JR_latch) {
        EB_JR_Timer = millis();
      }

      //E電磁給排弁動作フラグセット
      if (EB_JR && ( millis() - EB_JR_Timer > 5650 ) ) {
        EB_JR_move_E = true; //E電磁給排弁用
      }

      //ATS警報作動時 E電磁給排弁開始タイマーセット
      if (ATS_ERR && !ATS_ERR_latch) {
        ATS_ERR_Timer = millis();
      }

      //E電磁給排弁動作フラグセット
      if (ATS_ERR && ( millis() - ATS_ERR_Timer > 5650 ) ) {
        ATS_ERR_move_E = true; //E電磁給排弁用
      }

      //電制(JR)抽出
      Densei = strbve.substring(18, 19).toInt();

      //小田急モード
    } else {
      EBmode = strbve.substring(31, 32).toInt();
      if (!EBmode) {
        Oer_Kaisei = strbve.substring(16, 17).toInt();// 5 回生(小田急)
      }
    }

    if (!JR && !EBmode) {
      JyoyoMax = strbve.substring(26, 27).toInt();// 常用最大ブレーキ作用 (小田急)
    }
  } else if (!lock) {
    Broken = true;//故障表示灯点灯
    Chokutsu = true;//直通表示灯
    EB_JR = true; //直通
    Yokusoku = true; //抑速
    Densei = true; //電制
    ATS_Norm = true;
    ATS_ERR = true;//ATS警報
  }

  //圧力計データ入力がある場合はpressireに圧力計値を入れる
  if ( strbve.length() > 22 ) {
    //圧力値読取り
    pressure = strbve.substring(21, 25).toInt();
    //圧力上限を超えた場合抑える
    if (pressure > 5000 ) {
      pressure = 5000;
    }
  }

  //表示灯制御部
  digitalWrite(PIN_Disp_0, Broken);   //故障
  digitalWrite(PIN_Disp_1, Chokutsu); //直通
  digitalWrite(PIN_Disp_2, EB_JR || EBmode); //EB
  digitalWrite(PIN_Disp_3, Yokusoku); //抑速
  digitalWrite(PIN_Disp_4, Densei || Oer_Kaisei); //電制
  digitalWrite(PIN_Disp_5, ATS_Norm); //ATS白色
  digitalWrite(PIN_Disp_6, ATS_ERR); //ATS警報
  //表示灯制御部ここまで

  //常用最大バルブ制御
  digitalWrite(PIN_JyoyoMax, JyoyoMax); //常用最大ブレーキ作用

  //自動ブレーキ関係
  int16_t FV_sens = analogRead(Pin_In_FV);
  int16_t BP_sens = analogRead(Pin_In_BP);
  iFV_press_temp = map(analogRead(Pin_In_FV), FV_v_min, FV_v_max, 0, 490); //FV圧力値変換
  iBP_press_temp = map(analogRead(Pin_In_BP), BP_v_min, BP_v_max, 0, 490); //BP圧力値変換
  float a = (float)ave_ratio * 0.01;
  fFV_press = a * fFV_press + (1.00 - a) * (float) iFV_press_temp; //FV圧力値を移動平均化
  fBP_press = a * fBP_press + (1.00 - a) * (float) iBP_press_temp; //BP圧力値を移動平均化

  //fBC_press圧力に変換( = (FV圧-fBC_press圧) * 2.5
  fBC_press = (fFV_press - fBP_press) * 2.5 ;

  if (fBC_press > 440) { //BC圧が440以上の時は440に固定
    fBC_press = 440;
  } else if (fBC_press < 0) {
    fBC_press = 0;
  }

  //自動ブレーキモード動作条件 BC圧が10kPa以上
  AutoairBrake_Mode = (fBC_press > BC_p_min);

  //電空レギュレータ制御 自動ブレーキモードON or 電制ON or 回生(小田急)ON
  digitalWrite(PIN_VPReg, AutoairBrake_Mode || Densei || Oer_Kaisei);

  //電空レギュレータ制御用圧力演算
  int pressure_reg = 0;
  if(Densei && AutoairBrake_Mode){
    pressure_reg = pressure;
    if(pressure < (int) (fBC_press * 10)){
      pressure_reg = (int) (fBC_press * 10);
    }
  } else if(Densei && !AutoairBrake_Mode){
    pressure_reg = pressure;
  } else if(!Densei && AutoairBrake_Mode){
    pressure_reg = (int) (fBC_press * 10);
  }
  //電空レギュレータ圧力指令
  dac.setVoltage(map(pressure_reg, 0 , 5000 , 0, 4095), false);

  //BP排気解除条件 EB非動作、ATS非動作、圧力指令に変化があったとき
  if (!EB_JR && EB_JR_latch ) {
    EB_JR_move_E = false;
  }
  if ( !ATS_ERR && ATS_ERR_latch ) {
    ATS_ERR_move_E = false;
  }

  //BP排気条件(E電磁給排弁を作動)
  //EB動作中 or ATS警報動作中 or 非常位置
  digitalWrite(12, (EB_JR_move_E || ATS_ERR_move_E || !digitalRead(4)) && !MON );   //E電磁給排弁作用

  if (MON && (millis() - MON_Timer > MON_Interval) ) {
    Serial.print("FV(V)=");
    Serial.print(space_padding(FV_sens, 4, false));
    Serial.print(" BP(V)=");
    Serial.print(space_padding(BP_sens, 4, false));

    Serial.print(" FV=");
    Serial.print(space_padding(iFV_press_temp, 3, true));
    Serial.print("kPa BP=");
    Serial.print(space_padding(iBP_press_temp, 3, true));
    Serial.print("kPa");

    Serial.print(" (FV-BP)=");
    Serial.print(space_padding(iFV_press_temp - iBP_press_temp, 3, true));
    Serial.print("kPa");

    Serial.print(" BC=");
    Serial.print(fBC_press , 1);
    Serial.print("kPa");

    Serial.print(" AutoairBrake_Mode: ");
    Serial.println(AutoairBrake_Mode);
    MON_Timer = millis();
  }

  //自動ブレーキ関係おわり

  EB_JR_latch = EB_JR;
  ATS_ERR_latch = ATS_ERR;
  pressure_latch = pressure;

  delay(10);
}


String rw_eeprom(uint8_t dev, uint16_t *n, int *param, bool write) {
  if (write) {
    *param = *n;
    EEPROM.put(dev, *param);
  } else {
    EEPROM.get(dev, *param);
  }
  String s = "OK ";
  if (dev < 100) {
    s += "0";
  }
  if (dev < 10) {
    s += "0";
  }
  s += dev;
  s += " ";
  s += *param;

  return s;
}

String space_padding (int num, int digit, bool sign) {
  String s = "";
  if (num < 10000 && digit >= 5) {
    s += " ";
  }
  if (num < 1000 && digit >= 4) {
    s += " ";
  }
  if (num < 100 && digit >= 3) {
    s += " ";
  }
  if (num < 10 && digit >= 2) {
    s += " ";
  }
  if (sign) {
    if (num > 0) {
      s += "+";
    } else if (num == 0) {
      s += " ";
    } else {
      s += "-";
    }
    s += abs(num);
  } else {
    s += num;
  }
  return s;
}
