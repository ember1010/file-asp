
// phan cung
#define INPUT_SIGNAL 5
#define OUTPUT_SIGNAL1 6
#define OUTPUT_SIGNAL2 7
#define ANALOG_INPUT A0


#define ON HIGH
#define OFF LOW

#define ANALOG_VALUE 14
#define TIMER_VALUE 15000

#define digitalCount_VALUE 10

void setup() {
  Serial.begin(9600);
  pinMode(INPUT_SIGNAL, INPUT_PULLUP);
  pinMode(OUTPUT_SIGNAL1, OUTPUT);
  digitalWrite(OUTPUT_SIGNAL1, OFF);
  pinMode(OUTPUT_SIGNAL2, OUTPUT);
  digitalWrite(OUTPUT_SIGNAL2, OFF);
}
int digitalCount = 0;
bool input,flag = true;
int analog;
unsigned long timer,millisValue;
void  loop() {
  input = digitalRead(INPUT_SIGNAL);
  analog = analogRead(ANALOG_INPUT);
  if (digitalCount < 10000 && analog <= ANALOG_VALUE) {
    digitalCount++;
  }
  else if (analog > ANALOG_VALUE) digitalCount = 0;
  if(input == LOW){
    millisValue = millis();
  }
  timer = millis() - millisValue;
  /*if (input == LOW && flag) {
    digitalWrite(OUTPUT_SIGNAL1, ON);
    digitalWrite(OUTPUT_SIGNAL2, ON);
    flag =false;
    delay(100);
  }*/
  if (digitalCount > digitalCount_VALUE) {
    digitalWrite(OUTPUT_SIGNAL1, OFF);
    digitalWrite(OUTPUT_SIGNAL2, OFF); 
    flag=true;
    delay(100);
    }
   else if (input == LOW && digitalCount < digitalCount_VALUE && !flag)
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
