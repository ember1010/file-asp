// phan cung
#define D_INPUT4 4
#define D_INPUT5 5
#define OUTPUT_SIGNAL1 6
#define OUTPUT_SIGNAL2 7
#define ANALOG_INPUT A0


#define ON HIGH
#define OFF LOW

#define ANALOG_VALUE 14

// setting active la thoi gian nhan nut de tac dong, deactive la thoi gian nha? nut tac dong
#define D4_ACTIVE_TIMER 100
#define D4_DEACTIVE_TIMER 100
#define D5_ACTIVE_TIMER 100
#define D5_DEACTIVE_TIMER 15000
#define ANALOG_ACTIVE_TIMER 100 // active la <1V
#define ANALOG_DEACTIVE_TIMER 100 // deactive la >=1V

#define analogCount_VALUE 10

void setup() {
  Serial.begin(9600);
  pinMode(D_INPUT4, INPUT_PULLUP);
  pinMode(D_INPUT5, INPUT_PULLUP);
  pinMode(OUTPUT_SIGNAL1, OUTPUT);
  pinMode(OUTPUT_SIGNAL2, OUTPUT);
  digitalWrite(OUTPUT_SIGNAL1, OFF);
  digitalWrite(OUTPUT_SIGNAL2, OFF);
}
unsigned long d5TimeRecordActive = 0, d4TimeRecordActive = 0, d5TimeRecordDeactive = 0, d4TimeRecordDeactive = 0, analogRecordBig = 0, analogTimeSmall = 0;
bool d5Input = false, d4Input = false, firstFlag = true;
bool analogFlag = false; // analog >= 1V false; analog < 1V true

void  loop() {
  // Phan doc INPUT

  
  if (digitalRead(D_INPUT4) == LOW) { // neu d4 duoc tac dong
    d4TimeRecordDeactive = millis();
  }
  else
  {
    d4TimeRecordActive = millis();
  }
  if (digitalRead(D_INPUT5) == LOW) { // neu d5 duoc tac dong
    d5TimeRecordDeactive = millis();
  }
  else
  {
    d5TimeRecordActive = millis();
  }

  if (analogRead(ANALOG_INPUT) > ANALOG_VALUE) { // analog > dien ap cho phep
    analogTimeSmall = millis(); // reset thoi gian nho hon di
  }
  else
  {
    d5TimeRecordActive = millis(); // reset thoi gian lon hon di
  }



  if (millis() - d4TimeRecordDeactive > D4_DEACTIVE_TIMER) { // neu tin hieu nhan duoc > timer thi se tra ve thong tin cua input
    d4Input = false; // d4 khong tac dong
  }
  else if (millis() - d4TimeRecordActive > D4_ACTIVE_TIMER) { // neu tin hieu nhan duoc > timer thi se tra ve thong tin cua input
    d4Input = true; // d4  tac dong
  }
  if (millis() - d5TimeRecordDeactive > D5_DEACTIVE_TIMER) { // neu tin hieu nhan duoc > timer thi se tra ve thong tin cua input
    d5Input = false; // d5 khong tac dong
  }
  else if (millis() - d5TimeRecordActive > D5_ACTIVE_TIMER) { // neu tin hieu nhan duoc > timer thi se tra ve thong tin cua input
    d5Input = true; // d5  tac dong
  }
  if (millis() - analogTimeSmall > ANALOG_ACTIVE_TIMER) { // neu tin hieu nhan duoc > timer thi se tra ve thong tin cua input
    analogFlag = true; // du dieu kien < 1V
  }
  else if (millis() - analogTimeBig > ANALOG_DEACTIVE_TIMER) { // neu tin hieu nhan duoc > timer thi se tra ve thong tin cua input
    analogFlag = false; // KHONG du dieu kien < 1V
  }


  // Phan Xu ly logic va OUTPUT 

  if(d4Input == false && analogFlag == true && d5Input == true) // input4 = HIGH ; A0 < 1 ; input 5 = LOW
  {
    digitalWrite(OUTPUT_SIGNAL1,OFF);
    digitalWrite(OUTPUT_SIGNAL2,ON);
    firstFlag = true;
  }
  else if(d4Input == false && d5Input == false) // thoi gian tac dong se chinh bang D5_DEACTIVE_TIMER. dieu kien la input4 va input 5 cung tac dong
  {
    digitalWrite(OUTPUT_SIGNAL1,OFF);
    digitalWrite(OUTPUT_SIGNAL2,OFF);
    firstFlag = true;
  }
  else if(d4Input == true)
  {
    digitalWrite(OUTPUT_SIGNAL2,OFF)
    if(firstFlag){
      digitalWrite(OUTPUT_SIGNAL1,ON);
      delay(100);
      digitalWrite(OUTPUT_SIGNAL1,OFF);
      firstFlag = false;
    }
  }

}
