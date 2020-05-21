
// phan cung
#define INPUT_SIGNAL 5
#define OUTPUT_SIGNAL1 6
#define OUTPUT_SIGNAL2 7
#define ANALOG_INPUT A0


#define ON HIGH
#define OFF LOW

#define ANALOG_VALUE 14
#define TIMER_VALUE 15000

#define analogCount_VALUE 10

void setup() {
  Serial.begin(9600);
  pinMode(INPUT_SIGNAL, INPUT_PULLUP);
  pinMode(OUTPUT_SIGNAL1, OUTPUT);
  digitalWrite(OUTPUT_SIGNAL1, OFF);
  pinMode(OUTPUT_SIGNAL2, OUTPUT);
  digitalWrite(OUTPUT_SIGNAL2, OFF);
}
int analogCount = 0;
bool input,flag = true;
int analog;
unsigned long timer,millisValue;
void  loop() {
  input = digitalRead(INPUT_SIGNAL);
  analog = analogRead(ANALOG_INPUT);
  if (analogCount < 10000 && analog <= ANALOG_VALUE) {
    analogCount++;
  }
  else if (analog > ANALOG_VALUE) analogCount = 0;

  // Phan xac nhan trong 15S neu vi pham dieu kien duoi se reset timer
  if(input == HIGH || analogCount == 0){
    millisValue = millis();
  }
  timer = millis() - millisValue;
  
  if (analogCount > analogCount_VALUE && flag) {
    digitalWrite(OUTPUT_SIGNAL1, OFF);
    digitalWrite(OUTPUT_SIGNAL2, OFF);
    flag =false;
    delay(100);
  }

   else if (millis() - timer > TIMER_VALUE  && !flag)  // neu trong 15s cac dieu kien A0 < value va input tac dong lien tuc thi se millis() - timer > TIMER_VALUE = true 
   {
    digitalWrite(OUTPUT_SIGNAL1, ON);
    digitalWrite(OUTPUT_SIGNAL2, ON); 
    flag=true;
    delay(100);
   }
  Serial.print(input);
  Serial.print(" ");
  Serial.print(analog);
  Serial.print(" ");
  Serial.println(timer);
  delay(1);
}
