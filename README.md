## 表示灯、バルブ制御、自動帯制御用

- 上位とはRx <- Tx 、 Tx -> Rx　で接続をとること
- 圧力センサは0～500kPa以上で0～5V以内の出力が出るものであれば問題ない
- 電空レギュレータも0～500kPa出力を0～5Vで行えるものであれば問題ない
- 電空レギュレータのリレー出力は24V電源、信号は0～5V出力、DA変換は秋月電子製AE-MCP4725を使用
- 表示灯は5/12/24V対応としているが適宜省略orフォトリレーなどに置き換えてDC/AC100V対応するとよい
- 使用リレーは、秋月電子製ドライバー内蔵リレーモジュールキット(AE-G5V-DRV)を使用
- 予備のリレーはRx端子のため、書き込み時に動作するので注意

  ![実態配線図](Densei6.0.0.6.png)
