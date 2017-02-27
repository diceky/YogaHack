
double R1 = 5.1;

//アナログ入力の数を定義する
#define NUM 1
//アナログ入力の値を格納する配列
int val[NUM]; 
 
void setup() {
  //シリアル通信の開始
  Serial.begin(9600);
}
 
void loop() {
    // 変数の宣言
  double Vo, Rf, fg;
  int ain = analogRead(A0);
  // アナログ入力値から出力電圧を計算
  Vo = ain * 5.0 / 1024;
  // 出力電圧からFRSの抵抗値を計算
  Rf = R1*Vo / (5.0 - Vo);
  // FRSの抵抗値から圧力センサの荷重を計算
  fg = 880.79/Rf + 47.96;
  // 荷重データをシリアル通信で送る
  //Serial.println((int)fg);
  Serial.println((int)ain);
  delay(100);
//  Serial.read();   
}
