//SmileFunctionDecoder rail spark スケッチ
//
//arduinoのアナログ入力は10ビットの分解能です．
//0Vから5Vの電圧は，0から1023の値で表現されます．
//
//http://dcc.client.jp/

#include <NmraDcc.h>
#include <SoftwareSerial.h>


int ledPin = 2;             // LEDはデジタルピン2に接続
int sparkLedPin = 3;
//Task Schedule
unsigned long gPreviousL5 = 0;

#define O1 0            // Atiny85 PB0(5pin) analogin
#define O2 1            // Atiny8 5 PB1(6pin) analogwrite
#define O3 3            // Atint85 PB3(2pin)
#define O4 4            // Atiny85 PB4(3pin) analogwrite

SoftwareSerial mySerial(O1, O2); // RX, TX

//rail spark

unsigned char ptn1[10][4]={{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char ptn2[10][4]={{'I',0,5,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',1,0,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char ptn3[8][4]= {{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'E',0,0,1}};
unsigned char ptn4[6][4]= {{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'E',0,0,1}};
unsigned char ptn5[4][4]= {{'I',0,5,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char (*ptn)[4];

void setup() {
  pinMode(O1, OUTPUT);   // 出力に設定

  TCCR1 = 0<<CTC1 | 0<<PWM1A | 0<<COM1A0 | 1<<CS10;    

   // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
}

void loop() {
  if ( (millis() - gPreviousL5) >= 10) // 100:100msec  10:10msec  Function decoder は 10msecにしてみる。
  {
    RailSparkControl();

    //Reset task
    gPreviousL5 = millis();
  }
}

//---------------------------------------------------------------------
// RailSparkControlステートマシン
// 10ms周期で起動
// unsigned chart ptn[4][5]{{'I',0,0,1},{'S',20,255,1},{'S',40,0,1},{'E',0,0,1}};
//---------------------------------------------------------------------
void RailSparkControl(){
  static char state = 0;    // ステート
  static char adr = 0;      // アドレス
  static int timeUp = 0;    // 時間
  static int nextTimer = 0;  // A/Dする２回目のウエイト
  static float delt_v = 0;  // 100msあたりの増加量 
  static float pwmRef =0;
  static int nextSparkWait =0;  // 点滅間隔 10ms
  long randNumber = 0;
  long sparkSel = 0;
  static int adf = 0;    // a/d First
  int ads = 0;    // a/d Second
  int dif = 200;
  
  S00:  
  switch(state){
    case 0: // S00:idel
      digitalWrite(O1, HIGH);   // 赤外LEDをon
      nextTimer = 1;
      state = 1;
      break;

    case 1: // S01:時間カウント
      nextTimer--;
      if( nextTimer <= 0 ){
        state = 2;
      }
      break;

    case 2:
      adf=analogRead(O3);      //アナログ値を取得
//  mySerial.print("adf:");  
//  mySerial.println(adf);
        
      digitalWrite(O1, LOW);   // 赤外LEDをoff
      nextTimer = 1;
      state = 3;
      break;

    case 3:
      ads=analogRead(O3);      //アナログ値を取得
//  mySerial.print("ads:");  
//  mySerial.println(ads);
     if( ads - adf >= 200 ) {
        state = 4;            //車両検知
     } else {
        state = 0;            //ノイズかな
     }
     break;

    case 4: // A/D値から点灯条件判定    
      adr = 0;
      timeUp = 0;
      pwmRef = 0;
      TCCR1 = 1<<CS10;  //分周比をセット
      //       OCR1B有効   high出力　
      GTCCR = 1 << PWM1B | 2 << COM1B0;

      sparkSel = random(1,5);
      switch(sparkSel){
        case 1:
          ptn = ptn1;
          break;
        case 2:
          ptn = ptn2;
          break;
        case 3:
          ptn = ptn3;
          break;        
        case 4:
          ptn = ptn4;
          break;
        case 5:
          ptn = ptn5;
          break;

        default:
          ptn = ptn1;
          break;
      }
      state = 5;
      mySerial.println("state:5");
      break;
    
    case 5: // S01:コマンド処理
        if( ptn[adr][0]=='I'){ // I:初期化
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          delt_v = 0; // 変化量0
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          analogWrite(O4, (unsigned char)pwmRef); // 0〜255            
          adr++;
          state = 5;
          goto S00;   // 10ms待たずに再度ステートマシーンに掛ける
        } else if( ptn[adr][0]=='E'){ // E:end
          TCCR1 = 0<<CS10;  //分周比をセット   薄く光らないように
          analogWrite(O4,0); // 0〜255     
          state = 0;
        } else if( ptn[adr][0]=='O' ){ // O:出力
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          delt_v = 0;
          state = 6;          
        } else if( ptn[adr][0]=='S' ){ // S:sweep
          timeUp = ptn[adr][1];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット      
          delt_v = (ptn[adr][2]-pwmRef)/timeUp;  // 変化量を算出
          state = 6;
        }
      break;
      
    case 6: // S02:時間カウント
      timeUp--;
      pwmRef = pwmRef + delt_v;
      if(pwmRef<=0){            // 下限、上限リミッタ
          pwmRef = 0;
      } else if(pwmRef>=255){
          pwmRef = 255;
      }
      analogWrite(O4, (unsigned char)pwmRef); // 0〜255         
 
      if( timeUp <= 0 ){
        adr ++;
        state = 5;  //次のコマンドへ
      }
      break;
      
      default:
      break;
  }
}


//-----------------------------------------------------
// Debug用Lﾁｶ
//-----------------------------------------------------
void pulse()
{
  digitalWrite(O4, HIGH); // 短光
  delay(100);
  digitalWrite(O4, LOW);
}

