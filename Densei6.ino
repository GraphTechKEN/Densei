//Arduino Micro または Leonard を使用してください

//簡単な説明
//コマンドに対応しました。デリミタ:CR、Baud:115200、DataBits:8、StopBit:1

//更新履歴
//V1 初版
//V6 外部調整対応、I/O部修正
//V6.0.0.4 小田急自動切換
//V6.0.0.8 ノイズ対策
//V6.0.0.9 電源投入時ATS-P常用最大追加
//V6.0.0.10 阪急自動切換
//V6.0.0.11 ATS警報器、BZ21対応
//V6.0.0.12 ユニット表示版切り替えUnit_disp設定追加 ATS-P基板へ電源投入ON:ATS1とOFF:ATS0を伝送
//V6.0.0.13 電空レギュレータのOFF_Delay時間を調整可能に
//V6.0.0.14 ドア追加
//V6.0.0.15 小田急急行対応修正(D-ATS-Pで判定)
//V6.0.0.16 キー入力上位指令追加
//V6.0.0.17 ATS電源OFFで抑速と電制がOFFになるバグ修正
//V6.0.0.18 ATS未投入防止表示灯修正

#include <EEPROM.h>
#include <Adafruit_MCP4725.h>

#define PIN_JyoyoMax 7       //常用最大(小田急)
#define PIN_Valve_Reg 8      //電空レギュレータ制御用リレー
#define PIN_Valve_Densei 14  //電制制御バルブ用リレー(MI)
#define PIN_Valve_E 12       //E電磁給排弁用リレー
#define PIN_ATS_ERR 16       //ATS警報器用リレー
#define PIN_ATS_BZ21 17      //BZ21警報器用リレー
//入力ピンアサイン
#define PIN_EB_In 4       //EB
#define PIN_BVE_In 5      //連動有効無効切替
#define PIN_Densei_In 6   //連動有効無効切替
#define PIN_Dengen_In 15  //ATS電源
//自動ブレーキ
#define Pin_In_BP A0  // Analog input pin that the potentiometer is attached to
#define Pin_In_FV A1  // Analog input pin that the potentiometer is attached to
//出力ピンアサイン
#define PIN_Door 13   //ドア
int PIN_Disp_0 = A2;  //A:抑速 B:直通
int PIN_Disp_1 = A3;  //A:電制 B:ユニット1
int PIN_Disp_2 = A4;  //A:EB B:ユニット2
int PIN_Disp_3 = A5;  //A:ATS警報 B:ユニット3
int PIN_Disp_4 = 11;  //A:故障 B:ユニット4　未使用
int PIN_Disp_5 = 10;  //A:ATS白色 B:ATS白色
int PIN_Disp_6 = 9;   //A:直通 B:ATS警報

//↓デバッグのコメント(//)を解除するとシリアルモニタでデバッグできます
//#define DEBUG

Adafruit_MCP4725 dac;

uint16_t FV_v_min = 100;    //102
uint16_t FV_v_max = 800;    //104
uint16_t BP_v_min = 100;    //106
uint16_t BP_v_max = 800;    //108
int16_t BC_p_min = 10;      //110
uint16_t ave_ratio_A = 95;  //112

uint16_t FV_P_min = 0;          //124
uint16_t FV_P_max = 490;        //126
uint16_t BP_P_min = 0;          //128
uint16_t BP_P_max = 490;        //130
int16_t BC_P_max_A = 480;       //132
int16_t BC_P_max_norm_A = 380;  //134
uint16_t BC_Multi_A = 200;      //136 10培値
uint16_t BC_Multi_norm_A = 33;  //138 10培値

int16_t iFV_press_temp = 0;  // value read from the pot
int16_t iBP_press_temp = 0;  // value read from the pot

float fFV_press = 0.0;
float fBP_press = 0.0;
float fBP_press_latch = 0.0;
unsigned long BP_timer = 0;

float fBC_press = 0.0;
float fBC_press_latch = 0.0;
float fBC_press_chat = 0.0;

int BC_press = 0;
int BC_press_latch = 0;
unsigned long BC_press_timer = 0;

float BP_velocity = 0.0;
float BP_velocity_latch = 0.0;
float BP_velocity_Kyudou_Threshold = -0.1;  //142 急動部作動BP速度しきい値
uint16_t uintn_BP_velocity_Kyudou_Th = 100;

bool AutoairBrake_Mode = false;  //自動ブレーキモード fBC_press圧力が10kPa以上で作動(暫定)
bool Kyudou = false;
//自動ブレーキ関係

bool Oer_EB = false;        //EB作動
bool EB_JR = false;         //EB表示灯
bool EB_JR_move_E = false;  //EB作動(E電磁給排弁用)

bool ATS_ERR = false;           //ATS表示灯
bool ATS_ERR_move_E = false;    //ATS作動(E電磁給排弁用)
uint8_t ATS_Norm = 0;           //ATS白色表示灯
bool Ats_Conf = 0;              //ATS白色表示灯
bool Densei = false;            //電制表示灯
bool Oer_Kaisei = false;        //回生(小田急)
bool Oer_KaiseiKaihou = false;  //回生開放(小田急)
bool Oer_Datsp = false;         //D-ATS-P(小田急)
bool Broken = false;            //故障表示灯
uint8_t Chokutsu = 0;           //直通表示灯
bool Yokusoku = false;          //抑速表示灯
bool JyoyoMax_JR = false;       //常用最大(JR)バルブ用
bool Oer_JyoyoMax = false;      //常用最大(小田急)バルブ用
int16_t Unit_disp = 0;          //ユニット表示モード
int16_t Unit_num = 0;           //ユニット表示数
bool Unit1 = false;             //ユニット表示灯1
bool Unit2 = false;             //ユニット表示灯2
bool Unit3 = false;             //ユニット表示灯3
bool Unit4 = false;             //ユニット表示灯4
bool DenryuSign = false;        //電流符号
bool Densei_Use = false;
bool Oer_Pettern = false;  //パターン接近
int16_t MON_Interval = 0;  //V6.0.0.5 unsigned longからint16_tに修正
unsigned long MON_Timer = 0;
bool door = false;
bool Oer_Att = false;
bool Oer_Stop = false;

//ATS電源用
bool ATS_Dengen = false;
bool ATS_Dengen_latch = false;
bool ATS_Dengen_SW = false;
bool ATS_Dengen_SW_latch = false;
bool ATS_Dengen_Uart = false;
bool ATS_Dengen_Uart_latch = false;
uint16_t ATS_Dengen_Mode = 0;  //ATS_Dengen_Mode 1以上:ON/OFF 0:ラッチ
uint16_t ATS_Bell_Mode = 0;    //ATS_Bell_Mode 0:鳴動なし 1以上:鳴動許可
unsigned int Dengen_Count_On = 0;
unsigned int Dengen_Count_Off = 0;
bool Dengen_tounyu = false;
unsigned long Dengen_timer = 0;
bool BZ21 = 0;
bool ATS_P_Break = false;
bool flgAtsMitounyuBell = false;
uint16_t ATS_ERR_TIMER = 750;
uint16_t ATS_P_TIMER = 5000;
bool flgAtsErr = false;
bool flgAtsErr_latch = false;
uint16_t ATS_Mitounyu_Mode = 0;  //ATS未投入防止 (1)警報器(0)警報装置

//E電磁給排弁作用
unsigned long EB_timer = 0;
int16_t EB_Interval = EB_Interval;  //V6.0.0.5 追加
bool EB_delay = false;
bool EB_On = false;
int16_t BP_Threshold = 10;  //V6.0.0.5 追加

uint16_t b = 0;
uint16_t pressure = 0;
uint16_t pressure_latch = 0;

bool EB = false;
bool EB_brk = false;
unsigned int EB_Count_On = 0;
unsigned int EB_Count_Off = 0;

//モニターモード
bool MON = false;
bool USB_MON = false;

//String strbve = "0000/0/ 00000/000000/0000/0";
String strbve = "";

uint8_t TrainMode = 0;

//電制用
bool Valve_Reg_Off_Delay = false;
unsigned long Valve_Reg_DelayTimer = 0;
bool Valve_Reg = false;
bool Valve_Reg_latch = false;

//E制御弁
uint16_t Evalve = false;
uint16_t BC_P_max_E = 0;
uint16_t BC_Multi_E = 0;
uint16_t ave_ratio_E = 0;
uint16_t EB_On_delay = 200;
unsigned long EB_On_delay_time = 0;
bool EB_On_delayed = false;
bool flg_EB_On_delay = false;

uint16_t REG_Off_delay = 2000;

void setup() {

  pinMode(PIN_EB_In, INPUT);  //ブレーキ弁非常位置入力
  pinMode(PIN_Dengen_In, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);           //連動有効無効切替
  pinMode(6, INPUT_PULLUP);           //電制有効無効切替
  pinMode(PIN_JyoyoMax, OUTPUT);      //常用最大ブレーキ作用
  pinMode(PIN_Valve_Reg, OUTPUT);     //電空レギュレータ作用
  pinMode(PIN_Valve_Densei, OUTPUT);  //電空レギュレータ作用
  pinMode(PIN_Valve_E, OUTPUT);       //E電磁給排弁作用

  pinMode(PIN_ATS_ERR, OUTPUT);   //ATS警報器用リレー
  pinMode(PIN_ATS_BZ21, OUTPUT);  //BZ21警報器用リレー

  pinMode(PIN_Disp_0, OUTPUT);  //故障
  pinMode(PIN_Disp_1, OUTPUT);  //直通
  pinMode(PIN_Disp_2, OUTPUT);  //EB
  pinMode(PIN_Disp_3, OUTPUT);  //抑速
  pinMode(PIN_Disp_4, OUTPUT);  //電制
  pinMode(PIN_Disp_5, OUTPUT);  //ATS白色
  pinMode(PIN_Disp_6, OUTPUT);  //ATS警報
  pinMode(PIN_Door, OUTPUT);    //ATS警報

  digitalWrite(PIN_Disp_0, 0);        //故障
  digitalWrite(PIN_Disp_1, 0);        //直通
  digitalWrite(PIN_Disp_2, 0);        //EB
  digitalWrite(PIN_Disp_3, 0);        //抑速
  digitalWrite(PIN_Disp_4, 0);        //電制
  digitalWrite(PIN_Disp_5, 0);        //ATS白色
  digitalWrite(PIN_Disp_6, 0);        //ATS警報
  digitalWrite(PIN_Door, 0);          //ドア
  digitalWrite(PIN_JyoyoMax, 0);      //常用最大ブレーキ作用
  digitalWrite(PIN_Valve_Reg, 0);     //電空レギュレータ作用
  digitalWrite(PIN_Valve_E, 0);       //E電磁給排弁作用
  digitalWrite(PIN_Valve_Densei, 0);  //電制バルブ作用

  digitalWrite(PIN_ATS_ERR, 0);   //ATS警報器用リレー
  digitalWrite(PIN_ATS_BZ21, 0);  //BZ21警報器用リレー

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
    EEPROM.put(114, 100);   //MON_Interval
    EEPROM.put(116, 7000);  //EB_Interval 電磁給排弁開放時間 10倍値
    EEPROM.put(118, 10);    //BP_Threshold
    EEPROM.put(120, 0);     //ATS_Dengen_Mode 1以上:ON/OFF 0:ラッチ
    EEPROM.put(122, 0);     //ATS_Bell_Mode
    EEPROM.put(124, 0);     //FV_P_min
    EEPROM.put(126, 490);   //FV_P_max
    EEPROM.put(128, 0);     //BP_P_min
    EEPROM.put(130, 490);   //BP_P_max
    EEPROM.put(132, 490);   //BC_P_max
    EEPROM.put(134, 380);   //BC_P_max_norm 常用
    EEPROM.put(136, 200);   //BC_Multi 10倍値
    EEPROM.put(138, 33);    //BC_Multi_norm 常用 10倍値
    EEPROM.put(140, 100);   //ATS-S電源投入時
    EEPROM.put(142, 100);   //急動部作動BP速度しきい値
    EEPROM.put(144, 0);     //A制御弁(0) E制御弁(1)
    EEPROM.put(146, 0);     //BC_P_max_E
    EEPROM.put(148, 0);     //BC_Multi_E 10倍値
    EEPROM.put(150, 0);     //ave_ratio_E
    EEPROM.put(152, 200);   //EB_On_delay
    EEPROM.put(154, 2000);  //REG_Off_delay
    EEPROM.put(180, 0);     //Unit_disp
    EEPROM.put(182, 0);     //Unit_num
    //初回書き込みフラグセット
    EEPROM.put(100, b = 1);
  } else {
    EEPROM.get(102, FV_v_min);
    EEPROM.get(104, FV_v_max);
    EEPROM.get(106, BP_v_min);
    EEPROM.get(108, BP_v_max);
    EEPROM.get(110, BC_p_min);
    EEPROM.get(112, ave_ratio_A);
    EEPROM.get(114, MON_Interval);
    EEPROM.get(116, EB_Interval);  //EB_Interval 電磁給排弁開放時間 10倍値
    EEPROM.get(118, BP_Threshold);
    EEPROM.get(120, ATS_Dengen_Mode);              //ATS_Dengen_Mode 1以上:ON/OFF 0:ラッチ
    EEPROM.get(122, ATS_Bell_Mode);                //ATS_Bell_Mode 0:鳴動なし 1以上:鳴動許可
    EEPROM.get(124, FV_P_min);                     //FV_P_min
    EEPROM.get(126, FV_P_max);                     //FV_P_max
    EEPROM.get(128, BP_P_min);                     //BP_P_min
    EEPROM.get(130, BP_P_max);                     //BP_P_max
    EEPROM.get(132, BC_P_max_A);                   //BC_P_max
    EEPROM.get(134, BC_P_max_norm_A);              //BC_P_max_norm 常用
    EEPROM.get(136, BC_Multi_A);                   //BC_Multi 10倍値
    EEPROM.get(138, BC_Multi_norm_A);              //BC_Multi_norm 常用 10倍値
    EEPROM.get(140, ATS_ERR_TIMER);                //ATS-S電源投入時間
    EEPROM.get(142, uintn_BP_velocity_Kyudou_Th);  //急動部作動BP速度しきい値
    EEPROM.get(144, Evalve);                       //A制御弁(0) E制御弁(1)
    EEPROM.get(146, BC_P_max_E);                   //BC_P_max_E
    EEPROM.get(148, BC_Multi_E);                   //BC_Multi_E 10倍値
    EEPROM.get(150, ave_ratio_E);                  //ave_ratio_E
    EEPROM.get(152, EB_On_delay);                  //EB_On_delay
    EEPROM.get(154, REG_Off_delay);                //REG_Off_delay
    EEPROM.get(180, Unit_disp);
    EEPROM.get(182, Unit_num);
    EEPROM.get(204, ATS_Mitounyu_Mode);  //ATS未投入防止 (1)警報器(0)警報装置
    BP_velocity_Kyudou_Threshold = (float)uintn_BP_velocity_Kyudou_Th * -0.001;
  }

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.setTimeout(10);
  Serial1.setTimeout(10);
  Serial1.print("ATS0");
  Serial1.print('\r');

  //自動ブレーキ関係
  fFV_press = map(analogRead(Pin_In_FV), FV_v_min, FV_v_max, FV_P_min, FV_P_max);  //FV制御空気ダメ圧力RAW
  fBP_press = map(analogRead(Pin_In_BP), BP_v_min, BP_v_max, BP_P_min, BP_P_max);  //fBP_pressブレーキシリンダ圧力RAW
  //自動ブレーキ関係
}

void loop() {
  if (Unit_disp == 0) {         //115系
    PIN_Disp_0 = 11;            //故障　未使用
    PIN_Disp_1 = 9;             //直通
    PIN_Disp_2 = 10;            //EB
    PIN_Disp_3 = A2;            //抑速
    PIN_Disp_4 = A3;            //電制
    PIN_Disp_5 = A4;            //ATS白色
    PIN_Disp_6 = A5;            //ATS警報
  } else if (Unit_disp == 1) {  //ユニット表示(201系)
    PIN_Disp_0 = A2;            //直通
    PIN_Disp_1 = A3;            //ユニット1
    PIN_Disp_2 = A4;            //ユニット2
    PIN_Disp_3 = A5;            //ユニット3
    PIN_Disp_4 = 11;            //ユニット4　未使用
    PIN_Disp_5 = 10;            //ATS白色
    PIN_Disp_6 = 9;             //ATS警報
  } else if (Unit_disp == 2) {  //小田急1000
    PIN_Disp_0 = A2;            //故障 OER:速度注意1
    PIN_Disp_1 = 11;            //直通 OER:D-ATS-P
    PIN_Disp_2 = 9;             //EB OER:EB1
    PIN_Disp_3 = A5;            //電制 OER:停車1
    PIN_Disp_4 = A4;            //電制 OER:回生開放
    PIN_Disp_5 = 10;            //パターン接近
    PIN_Disp_6 = A3;            //常用最大 OER:動作
  }
  bool lock = !digitalRead(PIN_BVE_In);       //連動切替
  Densei_Use = !digitalRead(PIN_Densei_In);   //電制有効
  bool Dengen = !digitalRead(PIN_Dengen_In);  //ATS電源
  //ATS電源チャタリング防止用
  if (Dengen) {
    Dengen_Count_On++;
    Dengen_Count_Off = 0;
  } else {
    Dengen_Count_Off++;
    Dengen_Count_On = 0;
  }
  if (Dengen_Count_On > 10) {
    ATS_Dengen_SW = true;
  } else if (Dengen_Count_Off > 10) {
    ATS_Dengen_SW = false;
  }

  if (ATS_Dengen_Mode) {  //ME48モード、ME38しなの鉄道タイプ
    ATS_Dengen = ATS_Dengen_SW || ATS_Dengen_Uart;
  } else {  //ME38モード西タイプ
    if (!ATS_Dengen_SW) {
      ATS_Dengen = false;
    } else {
      if (ATS_Dengen_Uart) {
        ATS_Dengen = true;
      }
    }
  }


  ATS_Dengen_SW_latch = ATS_Dengen_SW;
  ATS_Dengen_Uart_latch = ATS_Dengen_Uart;

  if (ATS_Dengen && !ATS_Dengen_latch) {
    Dengen_tounyu = true;
    Dengen_timer = millis();
    Serial1.print("ATS1");
    Serial1.print('\r');
  } else if (!ATS_Dengen && !ATS_Dengen_latch) {
    Dengen_tounyu = false;
    //Yokusoku = false;
    //Densei = false;
    ATS_Norm = false;
    ATS_ERR = false;
    JyoyoMax_JR = false;
  } else if (!ATS_Dengen && ATS_Dengen_latch) {
    Serial1.print("ATS0");
    Serial1.print('\r');
  }

  if (Dengen_tounyu) {
    if (millis() - Dengen_timer < ATS_ERR_TIMER) {
      ATS_Norm = false;
      ATS_ERR = true;
    } else {
      ATS_Norm = true;
      ATS_ERR = false;
    }

    //ATS-P 初期常用最大
    if (millis() - Dengen_timer < ATS_P_TIMER) {
      JyoyoMax_JR = true;
    } else {
      JyoyoMax_JR = false;
      Dengen_tounyu = false;
    }
  }

  strbve = "";
  if (Serial.available()) {
    //6.0.0.16 上位キー入力指令追加
    String str = Serial.readStringUntil('\r');
    if (str.startsWith("KEY")) {
      str.trim();
      Serial1.print(str);
      Serial1.print('\r');
    } else if (str.startsWith("MON")) {
      USB_MON = str.substring(4).toInt();
      Serial.print("OK MON ");
      Serial.println(USB_MON);
    } else {
      strbve = str;
    }
  } else if (Serial1.available()) {
    strbve = Serial1.readStringUntil('\r');
    if (USB_MON) {
      Serial.println(strbve);  //USBのシリアルを出力し、受信側がない場合はタイムアウトして遅延するのでコメントアウト
    }
  }
  if (strbve.startsWith("WR ")) {
    //設定モード
    String s = "";
    if (strbve.length() > 7) {
      uint8_t device = strbve.substring(3, 6).toInt();
      int16_t num = strbve.substring(7, 12).toInt();
      if ((device >= 100 && device < 200) || device == 204) {
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

          //ave_ratio_A
          case 112:
            if (num < 0 || num > 99) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &ave_ratio_A, true);
            }
            break;

          //MON_Interval
          case 114:
            if (num < 1 || num > 65535) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &MON_Interval, true);
              MON_Interval = num;
            }
            break;

            //EB_Interval
          case 116:
            if (num < 1 || num > 65535) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &EB_Interval, true);
              EB_Interval = num;
            }
            break;

            //BP_Threshold
          case 118:
            if (num < 1 || num > 100) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BP_Threshold, true);
              BP_Threshold = num;
            }
            break;

            //ATS_Dengen_Mode
          case 120:
            if (num < 0 || num > 1) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &ATS_Dengen_Mode, true);
              ATS_Dengen_Mode = num;
            }
            break;

            //ATS_Bell_Mode 0:鳴動なし 1以上:鳴動許可
          case 122:
            if (num < 0 || num > 1) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &ATS_Bell_Mode, true);
              ATS_Bell_Mode = num;
            }
            break;

            //FV_P_min
          case 124:
            if (num < 0 || num > FV_P_max) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &FV_P_min, true);
            }
            break;

          //FV_P_max
          case 126:
            if (num > 1023 || num < FV_P_min) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &FV_P_max, true);
            }
            break;

          //BP_P_min
          case 128:
            if (num < 0 || num > BP_P_max) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BP_P_min, true);
            }
            break;

          //BP_P_max
          case 130:
            if (num > 1023 || num < BP_P_min) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BP_P_max, true);
            }
            break;

            //BC_P_max_A
          case 132:
            if (num > 1023 || num < 0) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BC_P_max_A, true);
            }
            break;

            //BC_P_max_norm_A
          case 134:
            if (num > 1023 || num < 0) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BC_P_max_norm_A, true);
            }
            break;

            //BC_Multi_A
          case 136:
            if (num > 1023 || num < 0) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BC_Multi_A, true);
            }
            break;

            //BC_Multi_norm_A
          case 138:
            if (num > 1023 || num < 0) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BC_Multi_norm_A, true);
            }
            break;

            //ATS-S電源投入時間
          case 140:
            if (num > 3000 || num < 0) {

              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &ATS_ERR_TIMER, true);
            }
            break;

            //急動部動作BP速度しきい値
          case 142:
            if (num > 3000 || num < 0) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &uintn_BP_velocity_Kyudou_Th, true);
              BP_velocity_Kyudou_Threshold = (float)uintn_BP_velocity_Kyudou_Th * -0.001;
            }
            break;

            //A制御弁(0) E制御弁(1)
          case 144:
            if (num > 1 || num < 0) {

              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &Evalve, true);
            }
            break;

            //BC_P_max_E
          case 146:
            if (num > 1023 || num < 0) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BC_P_max_E, true);
            }
            break;

            //BC_Multi_E
          case 148:
            if (num > 1023 || num < 0) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &BC_Multi_E, true);
            }
            break;

            //ave_ratio_E
          case 150:
            if (num < 0 || num > 99) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &ave_ratio_E, true);
            }
            break;

            //EB_On_delay
          case 152:
            if (num < 1 || num > 65535) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &EB_On_delay, true);
            }
            break;

            //REG_Off_delay
          case 154:
            if (num < 1 || num > 65535) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &REG_Off_delay, true);
            }
            break;

            //Unit_disp
          case 180:
            if (num < 0 || num > 255) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &num, true);
              Unit_disp = num;
            }
            break;

            //Unit_num
          case 182:
            if (num < 0 || num > 4) {
              s = "E1 " + String(device);
            } else {
              s = rw_eeprom(device, &num, &num, true);
              Unit_num = num;
            }
            break;
            //ATS未投入防止装置モード
          case 204:
            if (num < 0 || num > 4) {
              s = "E1 " + device;
            } else {
              s = rw_eeprom(device, &num, &ATS_Mitounyu_Mode, true);
            }
            break;
        }
        if (USB_MON) {
          Serial.println(s);
        }
        Serial1.print(s);
        Serial1.print('\r');
      }
    }
  } else if (strbve.startsWith("RD ")) {
    //読取モード
    String s = "";
    if (strbve.length() > 5) {
      uint8_t device = strbve.substring(3, 6).toInt();
      int16_t num = 0;
      if (device >= 100 && device < 200) {
        s = rw_eeprom(device, &num, &num, false);
        if (USB_MON) {
          Serial.println(s);
        }
        Serial1.print(s);
        Serial1.print('\r');
      }
    }
  } else if (strbve.startsWith("MON")) {
    //表示モード
    int16_t num = strbve.substring(3).toInt();
    MON = (num == 1);
    if (USB_MON = (num == 2)) {
      Serial.println("MON USB 1");
    }

  } else if (strbve.startsWith("ATS 1")) {  //制御基板からブレーキ弁ATS接点ON情報
    ATS_Dengen_Uart = true;
  } else if (strbve.startsWith("ATS 0")) {  //制御基板からブレーキ弁ATS接点OFF情報
    ATS_Dengen_Uart = false;

    //ATS確認
  } else if (strbve.startsWith("ACF 1")) {  //制御基板からATS確認ボタンON情報
    Ats_Conf = true;
  } else if (strbve.startsWith("ACF 0")) {  //制御基板からATS確認ボタンOFF情報
    Ats_Conf = false;

    //警報持続
  } else if (strbve.startsWith("ACT 1")) {  //制御基板から警報持続ボタンON情報
    BZ21 = true;
  } else if (strbve.startsWith("ACT 0")) {  //制御基板から警報持続ボタンOFF情報
    BZ21 = false;
  } else if (strbve.startsWith("ATSM3")) {
    flgAtsMitounyuBell = true;
  } else if (strbve.startsWith("ATSM2")) {
    if (ATS_Mitounyu_Mode >> 1 & 1) {
      flgAtsMitounyuBell = true;
    } else {
      flgAtsMitounyuBell = false;
    }
  } else if (strbve.startsWith("ATSM1")) {
      flgAtsMitounyuBell = false;
  } else {
    if (strbve.length() > 35 && lock) {
      //電流符号抽出
      DenryuSign = (strbve.substring(7, 8) != "-");

      //ATS白色表示灯抽出
      ATS_Norm = strbve.substring(14, 15).toInt();

      //ATS警報抽出
      ATS_ERR = strbve.substring(15, 16).toInt();

      //直通抽出
      Chokutsu = strbve.substring(17, 18).toInt();

      //BZ21状態抽出
      //BZ21 = strbve.substring(13, 14) == "1";

      //ATS確認ボタン抽出
      //ATS_Dengen_Uart = strbve.substring(20, 21) == "1";
      if (ATS_Dengen_Mode == 0) {
        ATS_Dengen |= ATS_Dengen_Uart;
      }

      Oer_Datsp = (strbve.substring(40, 41).toInt() == 2);

      //小田急モード抽出
      if (Oer_Datsp) {
        TrainMode = 1;  //小田急
      } else {
        //阪急モード抽出
        if (ATS_Norm == 7) {
          TrainMode = 2;  //阪急
        } else if (ATS_Norm == 0 || ATS_Norm == 1) {
          //JRモード抽出
          TrainMode = 0;
        }
      }
      //抑速抽出
      Yokusoku = strbve.substring(19, 20).toInt();


      //ユニット表示抽出
      Unit1 = strbve.substring(40, 41).toInt() & (Unit_num > 0);
      Unit2 = strbve.substring(41, 42).toInt() & (Unit_num > 1);
      Unit3 = strbve.substring(28, 29).toInt() & (Unit_num > 2);

      //ドア抽出
      door = strbve.charAt(5) != '1';

      //JRモード
      if (TrainMode == 0) {
        Oer_EB = false;
        Oer_Kaisei = false;
        Oer_JyoyoMax = false;
        Oer_Pettern = false;
        Oer_KaiseiKaihou = false;
        //EB抽出
        EB_JR = strbve.substring(16, 17).toInt();  //EB(JR)

        //電制(JR)抽出
        Densei = strbve.substring(18, 19).toInt();
        if (strbve.length() > 35) {
          //ATS/EB非常ブレーキ抽出
          EB_JR_move_E = (strbve.substring(36, 37).toInt());

          //ATS-P常用最大
          ATS_P_Break = strbve.substring(31, 32).toInt();
        }
        //小田急モード
      } else if (TrainMode == 1) {
        if (strbve.length() > 35) {
          Oer_EB = strbve.substring(31, 32).toInt();
          if (!Oer_EB) {
            Oer_Kaisei = strbve.substring(16, 17).toInt();        // 5 回生(小田急)
            Oer_JyoyoMax = strbve.substring(26, 27).toInt();      // 常用最大ブレーキ作用 (小田急)
            Oer_Pettern = strbve.substring(28, 29).toInt();       //パターン接近(小田急)
            Yokusoku = strbve.substring(27, 28).toInt();          //停車　
            Oer_Att = strbve.substring(37, 38).toInt();           //速度注意
            Oer_KaiseiKaihou = strbve.substring(38, 39).toInt();  //回生開放SW
            Oer_Datsp = strbve.substring(40, 41).toInt();         //D-ATS-P
            Oer_Stop = Yokusoku;
          }
        }
        //阪急モード
      } else if (TrainMode == 2) {
        Oer_EB = false;
        Oer_Kaisei = false;
        Oer_JyoyoMax = false;
        Oer_Pettern = false;
        Oer_KaiseiKaihou = false;
        Densei = !DenryuSign;
      }

      //圧力計データ入力がある場合はpressireに圧力計値を入れる
      if (strbve.length() > 24) {
        //圧力値読取り
        pressure = strbve.substring(21, 25).toInt();
        //Serial.println(pressure);
        //圧力上限を超えた場合抑える
        if (pressure > 5000) {
          pressure = 5000;
        }
      }
    } else if (!lock) {
      Broken = Dengen;    //故障表示灯点灯
      Chokutsu = Dengen;  //直通表示灯
      EB_JR = Dengen;     //直通
      Yokusoku = Dengen;  //抑速
      Densei = Dengen;    //電制
      ATS_Norm = Dengen;  //ATS
      ATS_ERR = Dengen;   //ATS警報
    }
  }
  ATS_Dengen_latch = ATS_Dengen;
  //表示灯制御部
  digitalWrite(PIN_Disp_0, Broken || ((Chokutsu == 1) && Unit_disp) || (Oer_Att && (TrainMode == 1)));                                                                                                        //故障 OER:速度注意
  digitalWrite(PIN_Disp_1, ((Chokutsu == 1) && !Unit_disp) || (Unit1 && Unit_disp) || Oer_Datsp && (Unit_disp == 2));                                                                                         //直通 OER:D-ATS-P
  digitalWrite(PIN_Disp_2, (EB_JR == 1) || Oer_EB || (Unit2 && Unit_disp));                                                                                                                                   //EB OER:EB
  digitalWrite(PIN_Disp_3, Yokusoku || (Unit3 && Unit_disp) || (Oer_Stop && (TrainMode == 1)));                                                                                                               //抑速 OER:停車
  digitalWrite(PIN_Disp_4, (Densei && !Unit_disp) || (Oer_Kaisei && !(Unit_disp == 2)) || (!Yokusoku && !Densei && Unit1 && !DenryuSign) || (Unit4 && Unit_disp) || (Oer_KaiseiKaihou && (Unit_disp == 2)));  //電制 OER:回生(A)/回生開放(C)
  digitalWrite(PIN_Disp_5, (ATS_Norm && !Ats_Conf && !(Unit_disp == 2)) || (Oer_Pettern && (TrainMode == 1)));                                                                                                //ATS白色 OER:パターン接近
  digitalWrite(PIN_Disp_6, (((ATS_ERR || (!ATS_Dengen && flgAtsMitounyuBell)) && !Unit_disp) || ATS_P_Break && (Unit_disp == 2)) && (TrainMode == 0) || (Oer_JyoyoMax && (TrainMode == 1)));                  //ATS警報 ブレーキ動作                                                       //ATS警報 OER:ブレーキ動作
  digitalWrite(PIN_Door, door);                                                                                                                                                                               //戸閉灯指示
  digitalWrite(PIN_ATS_BZ21, !BZ21 && ATS_Dengen);                                                                                                                                                            //警報持続ボタン

  flgAtsErr = ATS_ERR && ATS_Dengen && ATS_Bell_Mode;
  if (TrainMode == 0) {
    //ATS警報器動作
    if (flgAtsErr && !flgAtsErr_latch) {
      Serial1.print("ATSE");
      Serial1.print('\r');
    } else if (!flgAtsErr && flgAtsErr_latch) {
      Serial1.print("ATSN");
      Serial1.print('\r');
    }
    digitalWrite(PIN_ATS_ERR, flgAtsErr || (!ATS_Dengen && flgAtsMitounyuBell));
  }

  flgAtsErr_latch = flgAtsErr;
  //表示灯制御部ここまで

  /*
  Serial.print("!ATS_Norm=");
  Serial.print(!ATS_Norm);
  Serial.print(" !ATS_ERR=");
  Serial.print(!ATS_ERR);
  Serial.print(" !DenryuSign=");
  Serial.println(!DenryuSign);
*/

  //常用最大バルブ制御
  digitalWrite(PIN_JyoyoMax, Oer_JyoyoMax || JyoyoMax_JR || ATS_P_Break);  //常用最大ブレーキ作用

  uint16_t BC_P_max = BC_P_max_A;
  uint16_t BC_P_max_norm = BC_P_max_norm_A;
  uint16_t BC_Multi = BC_Multi_A;
  uint16_t BC_Multi_norm = BC_Multi_norm_A;
  uint16_t ave_ratio = ave_ratio_A;

  //E制御弁モード時
  if (Evalve) {
    BC_P_max = BC_P_max_E;
    BC_P_max_norm = BC_P_max_E;
    BC_Multi = BC_Multi_E;
    BC_Multi_norm = BC_Multi_E;
    ave_ratio = ave_ratio_E;
  }

  int16_t FV_sens = analogRead(Pin_In_FV);
  int16_t BP_sens = analogRead(Pin_In_BP);
  iFV_press_temp = map(FV_sens, FV_v_min, FV_v_max, FV_P_min, FV_P_max);  //FV圧力値変換
  iBP_press_temp = map(BP_sens, BP_v_min, BP_v_max, BP_P_min, BP_P_max);  //BP圧力値変換
  float a = (float)ave_ratio * 0.01;
  fFV_press = a * fFV_press + (1.00 - a) * (float)iFV_press_temp;  //FV圧力値を移動平均化
  fBP_press = a * fBP_press + (1.00 - a) * (float)iBP_press_temp;  //BP圧力値を移動平均化

  if ((millis() - BP_timer) > 50) {
    BP_velocity = (fBP_press - fBP_press_latch) / (float)(millis() - BP_timer) * 0.1 + BP_velocity_latch * 0.9;
    //Serial.println(BP_velocity);
    BP_velocity_latch = BP_velocity;
    BP_timer = millis();
    fBP_press_latch = fBP_press;
  }

  //fBC_press圧力に変換( = (FV圧-fBC_press圧) * 2.5
  if (Kyudou) {
    //急動時
    fBC_press = (fFV_press - fBP_press) * BC_Multi * 0.1;
  } else {
    //常用時
    fBC_press = (fFV_press - fBP_press) * BC_Multi_norm * 0.1;
  }
  if (fBC_press > BC_P_max) {  //BC圧がBC_P_max(490)以上の時はBC_P_maxに固定
    fBC_press = BC_P_max;
  } else if (fBC_press < 0) {
    fBC_press = 0;
  }
  Kyudou = (fBC_press > BC_P_max_norm);
  if (fBC_press > BC_P_max_norm && fBC_press_latch < BC_P_max && fBP_press > 40 && BP_velocity > BP_velocity_Kyudou_Threshold && (fBC_press - fBC_press_latch) > 0) {
    fBC_press = BC_P_max_norm;
    Kyudou = false;
  }
  //自動ブレーキモード動作条件 BC圧が3kPa以上変化したとき
  if (abs(fBC_press - fBC_press_chat) > 2) {
    //自動ブレーキモード動作条件 BC圧が10kPa以上
    AutoairBrake_Mode = (fBC_press > BC_p_min);
    fBC_press_chat = fBC_press;
  }

  fBC_press_latch = fBC_press;

  //電空レギュレータ制御 自動ブレーキモードON or 電制ON or 回生(小田急)ON
  Valve_Reg = AutoairBrake_Mode || (Densei_Use && (!Yokusoku && Densei || Oer_Kaisei || (!Densei && Unit1 && !DenryuSign)));
  if (!Valve_Reg && Valve_Reg_latch) {
    Valve_Reg_Off_Delay = true;
    Valve_Reg_DelayTimer = millis();
  }
  if (Valve_Reg_Off_Delay && millis() - Valve_Reg_DelayTimer > REG_Off_delay) {
    Valve_Reg_Off_Delay = false;
  }
  Valve_Reg_latch = Valve_Reg;
  digitalWrite(PIN_Valve_Reg, Valve_Reg || Valve_Reg_Off_Delay);
  //電制バルブ制御
  //digitalWrite(PIN_Valve_Densei, AutoairBrake_Mode || (Densei_Use && (Densei || Oer_Kaisei || (!Yokusoku && !Densei && Unit1 && !DenryuSign))));
  digitalWrite(PIN_Valve_Densei, (Densei_Use && (Densei || Oer_Kaisei || (!Yokusoku && !Densei && Unit1 && !DenryuSign))));

  //電空レギュレータ制御用圧力演算
  int pressure_reg = 0;
  if (Densei_Use && Densei && AutoairBrake_Mode) {
    pressure_reg = pressure;
    if (pressure < (int)(fBC_press * 10)) {
      pressure_reg = (int)(fBC_press * 10);
    }
  } else if (Densei_Use && (Densei || Oer_Kaisei || (Unit1 && !DenryuSign))) {
    pressure_reg = pressure;
  } else if (AutoairBrake_Mode) {
    pressure_reg = (int)(fBC_press * 10);
  }
  if (Valve_Reg_Off_Delay) {
    pressure_reg = 0;
  }
  //電空レギュレータ圧力指令
  dac.setVoltage(map(pressure_reg, 0, 5000, 0, 4095), false);

  //BP排気条件(E電磁給排弁を作動)
  //EB動作中 or ATS警報動作中 or 非常位置

  //EB誤作動防止版  2024/03/19
  bool EB_In = !digitalRead(PIN_EB_In);
  if (EB_In) {
    EB_Count_On++;
    EB_Count_Off = 0;
  } else {
    EB_Count_Off++;
    EB_Count_On = 0;
  }
  if (EB_Count_On > 10) {
    EB_brk = true;
  } else if (EB_Count_Off > 10) {
    EB_brk = false;
  }
  //EB誤作動防止版  2024/03/19

  //E電磁給排弁動作
  if ((EB_JR_move_E || ATS_ERR_move_E || EB_brk) && !MON) {
    //E電磁給排弁ON
    EB_On = true;
    //EB遅延制御がなく、BPが[BP_Threshold]kPa以上の場合
    if (!EB_delay && fBP_press > BP_Threshold) {
      //EB遅延制御開始、タイマースタート
      EB_delay = true;
      EB_timer = millis();
    }
  } else {
    //EB遅延制御が開始されてない場合
    if (!EB_delay) {
      //E電磁給排弁OFF
      EB_On = false;

      //EB遅延制御が開始されている場合
    } else {
      //電磁給排弁ON
      EB_On = true;

      //タイマー経過後、またはBPが10kPa未満はEB遅延制御をOFF
      if (millis() - EB_timer > EB_Interval) {
        //タイマー経過後はEB遅延制御をOFF
        EB_delay = false;
      }
    }
  }

  if (EB_On && !flg_EB_On_delay) {
    flg_EB_On_delay = true;
    EB_On_delay_time = millis();
  } else if (EB_On && (millis() - EB_On_delay_time > EB_On_delay)) {
    EB_On_delayed = true;
  } else if (!EB_On) {
    EB_On_delayed = false;
    flg_EB_On_delay = false;
  }
  digitalWrite(PIN_Valve_E, EB_On_delayed);  //E電磁給排弁作用

  if (MON && (millis() - MON_Timer > MON_Interval)) {
    String s = "";
    s += "FV(V)=";
    s += space_padding(FV_sens, 4, false);
    s += " BP(V)=";
    s += space_padding(BP_sens, 4, false);
    Serial1.print(s);
    if (USB_MON) {
      Serial.print(s);
    }

    s = "";
    s += " FV=";
    s += space_padding(iFV_press_temp, 3, true);
    s += "kPa BP=";
    s += space_padding(iBP_press_temp, 3, true);
    s += "kPa";
    Serial1.print(s);
    if (USB_MON) {
      Serial.print(s);
    }

    /*
    s = "";
    s += " (FV-BP)=";
    s += space_padding(iFV_press_temp - iBP_press_temp, 3, true);
    s += "kPa";
    Serial1.print(s);
    if (USB_MON) {
      Serial.print(s);
    }
*/

    s = "";
    s += " BC=";
    s += String(fBC_press, 1);
    s += "kPa";
    Serial1.print(s);
    if (USB_MON) {
      Serial.print(s);
    }

    /*
    s = "";
    s += " AutoairBrake_Mode: ";
    s += String(AutoairBrake_Mode);
    Serial1.print(s);
    Serial1.print('\r');
    */

    if (USB_MON) {
      Serial.println(s);
    }
    MON_Timer = millis();

    //上位にBCを伝送する
  } else {
    BC_press = (int)fBC_press;
    if ((millis() - BC_press_timer > 50) && (abs(BC_press - BC_press_latch) > 3) || (BC_press != BC_press_latch) && (BC_press == 0)) {
      BC_press_timer = millis();
      Serial1.print("BC ");
      Serial1.print(space_padding(BC_press, 3, false));
      Serial1.print('\r');
      BC_press_latch = BC_press;
    }
  }

  //自動ブレーキ関係おわり

  pressure_latch = pressure;
}


String rw_eeprom(uint8_t dev, uint16_t* n, uint16_t* param, bool write) {
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

String space_padding(int num, int digit, bool sign) {
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

String zero_padding(int num, int digit, bool sign) {
  String s = "";
  if (num < 10000 && digit >= 5) {
    s += "0";
  }
  if (num < 1000 && digit >= 4) {
    s += "0";
  }
  if (num < 100 && digit >= 3) {
    s += "0";
  }
  if (num < 10 && digit >= 2) {
    s += "0";
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
