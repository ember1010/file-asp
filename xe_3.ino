#include <MFRC522.h>
#include <SPI.h>
#include <PID_v1.h>
#include <EEPROM.h>
//#include "clashpoint.h"
#include "Mp3.h"
#include "SRF05.h"

double Setpoint = 320, Input, Output;

double Kp = 0.5, Ki = 0.17  , Kd = 0.025;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
#include "E:/xe  agv full/Header.h"
#include "E:/xe  agv full/InfomationCode.h"
// Define INPUT
# define LIGHTRANGE 250
#define LIGHTSCALE1 0
#define LIGHTSCALE2 +15
#define LIGHTSCALE3 +15
#define LIGHTSCALE4 0
# define LIGHTSENSOR1 A2
# define LIGHTSENSOR2 A3
# define LIGHTSENSOR3 A4
# define LIGHTSENSOR4 A14

# define START1 9
# define START2 7

# define EMG 3
# define BUMP 27
# define SAFETY 33
# define SS1_ECH 24
# define SS2_ECH 28
# define SS3_ECH 25
# define LINETRACK A1
# define AUTO 0
# define MAN 1
# define LALM 35
# define RALM 37
# define RFSTOP 23
# define RFSLOW 25

// Define OUTPUT
# define RM 13
# define RBRK 12
# define LM 11
# define LBRK 10
# define LAMP1 8
# define LAMP2 6
# define SPK 29
# define SS1_TRIG 22
# define SS2_TRIG 26
# define SS3_TRIG 23
# define FORK_L 39
# define FORK_R 41

// Define SPI
# define SS_PIN 53
# define RST_PIN 43
//event

#define BEGINSPEED 30
# define EVENT_TURNLEFT_IN3 14 // normal
# define EVENT_TURNLEFT_IN2 13 // boost
# define EVENT_TURNLEFT_IN1 10 // slow
# define EVENT_TURNLEFT_IN 11 // turn
# define EVENT_TURNLEFT_OUT 12

# define EVENT_TURNRIGHT_IN3 24
# define EVENT_TURNRIGHT_IN2 23
# define EVENT_TURNRIGHT_IN 21
# define EVENT_TURNRIGHT_IN1 20
# define EVENT_TURNRIGHT_OUT 22
# define EVENT_STOP 4
# define NO_EVENT 0
# define EVENT_BOOST 31
# define EVENT_SLOW 32
# define EVENT_NORMAL 30
# define EVENT_TURN 33

# define _NORMAL 80
# define _DEFAULT 80
# define _TURN 60
# define _BOOST 120
# define _SLOW 45

#define UNKNOWSTOP 399
#define EMGSTOP 300
#define LINETRACKSTOP 310
#define LIGHTSENSORSTOP 321
#define SAFETYSTOP 320
#define BUMPERSTOP 330
#define PRESSSTOP 3402
#define AVOIDSTOP 350
#define START 400
#define IOT3 401
#define IOT4 402
#define KHOTRONG 403
#define CAPLIEU 404

# define POINT1 100
# define POINT2 101

# define EPPROM_NAME 0
# define EPPROM_LEFTADD 1
# define EPPROM_RIGHTADD 2
# define EPPROM_LINESELECT 10
Mp3 speaker;
MFRC522 mfrc522(SS_PIN, RST_PIN);
SRF05 ss1(SS1_TRIG, SS1_ECH);
SRF05 ss2(SS2_TRIG, SS2_ECH);

byte closeStatus = 0;
//Specify the links and initial tuning parameters

bool speakerFlag = false, currentSpeedFlag = false, srfFlag = false;
int pos = 0;
const byte NAME = EEPROM.read(EPPROM_NAME);
const byte LEFTADD = EEPROM.read(EPPROM_LEFTADD);
const byte RIGHTADD = EEPROM.read(EPPROM_RIGHTADD);
byte LINESELECT = EEPROM.read(EPPROM_LINESELECT);
bool updateLineFlag = false;

byte closeLampReason = EEPROM.read(99);
int smaxrPrev = 0;
int smaxlPrev = 0;
unsigned long CardID = 1000, CardIDTemp;    // Hien thi so UID dang thap phan
int smaxr = 0;
int smaxl = 0;
int sdow = 0;
int Startdelay = 1000;
byte mode = AUTO;
bool rightALM = HIGH, leftALM = HIGH;
int NORMALSPEED = _NORMAL;
int DEFAULTSPEED = _DEFAULT;
int TURNSPEED = _TURN;
int BOOSTSPEED = _BOOST;
int SLOWSPEED = _SLOW;
double currentSpeed, pointValue = 0;
double valueR;
double valueL;
double prevSpeed = 0;
unsigned long prevCard = 1000;
unsigned long  stopCard = 0 , prevStopCard = 0;
const unsigned long  cardNo1 = 1870775848;
const unsigned long cardNo2 = 1863600424;
const unsigned long cardNo3 = 1396368686;
const unsigned long cardNo4 = 1200942445;
const unsigned long  cardNo5 = 656470381;
const unsigned long cardNo6 = 930540397;
const unsigned long cardNo8 = 661132654;
const unsigned long cardNo9 = 1731786093;
const unsigned long cardNo10 = 3072335214;
const unsigned long cardNo11 = 1607974440;
const unsigned long cardNo12 = 666823789;

const unsigned long card1 = 2501389354;
const unsigned long card2 = 4121118762;
const unsigned long card3 = 1170947626;
const unsigned long card4 = 1701856298;
const unsigned long card5 = 3845799466;
const unsigned long card6 = 2231908394;
const unsigned long card7 = 2779066922;
const unsigned long card8 = 1438101265;
const unsigned long card9 = 3037704209;
const unsigned long card10 = 2510962218;
const unsigned long card11 = 2512206378;
const unsigned long card12 = 2775725354;
const unsigned long card13 = 3848416298;
const unsigned long card14 = 2514959914;
const unsigned long card15 = 903162410;
const unsigned long card16 = 2512011050;
const unsigned long card17 = 352469034;
const unsigned long card18 = 3844025386;
const unsigned long card19 = 1709001514;
const unsigned long card20 = 2504862762;
const unsigned long card21 = 362983185;
const unsigned long card22 = 634209066;
const unsigned long card23 = 3047698730;
const unsigned long card24 = 1161770282;
const unsigned long card25 = 366489130;
const unsigned long card26 = 364656938;
const unsigned long card27 = 1964586026;
const unsigned long card28 = 1709649706;
const unsigned long card29 = 2769108266;
const unsigned long card30 = 3042657322;
const unsigned long card31 = 636628778;
const unsigned long card32 = 895697706;
const unsigned long card33 = 3038262826;
const unsigned long card34 = 4126224682;
const unsigned long card35 = 3585289002;
const unsigned long card36 = 2502903850;
const unsigned long card37 = 2242262826;
const unsigned long card38 = 1965928465;
const unsigned long card39 = 2510631210;
const unsigned long card40 = 3038727210;
const unsigned long card41 = 2378078765;
const unsigned long cardnv = 3297683241;

const unsigned long  IDNo1 = 1345708295;
const unsigned long  IDNo2 = 2169183753;
const unsigned long  IDNo3 = 183269376;
const unsigned long  IDNo4 = 1671576103;
const unsigned long  IDNo5 = 1917949220;
const unsigned long  IDNo6 = 1344967687;
const unsigned long  IDNo7 = 1343335687;
const unsigned long  IDNo8 = 1351433991;
const unsigned long  IDNo9 = 1344585479;
const unsigned long  IDNo10 = 1350706183;
const unsigned long  IDNo11 = 867234087;
const unsigned long  IDNo12 = 870565415;
const unsigned long  IDNo13 = 865235751;
const unsigned long  IDNo14 = 851230244;
const unsigned long  IDNo15 = 1660064036;
const unsigned long  IDNo16 = 866554151;
const unsigned long  IDNo17 = 872260647;
const unsigned long  IDNo18 = 865552935;
const unsigned long  IDNo19 = 1126539815;
const unsigned long  IDNo20 = 869715495;
const unsigned long  IDNo21 = 865366567;
const unsigned long  IDNo22 = 1129932327;
const unsigned long  IDNo23 = 1126712359;
const unsigned long  IDNo24 = 864580135;
const unsigned long  IDNo25 = 867848231;
const unsigned long  IDNo26 = 1127835431;
const unsigned long  IDNo27 = 865686055;
const unsigned long  IDNo28 = 1128798247;
const unsigned long  IDNo29 = 868724263;
const unsigned long  IDNo30 = 869454119;
const unsigned long  IDNo31 = 1404991527;
const unsigned long  IDNo32 = 1407677223;
const unsigned long  IDNo33 = 904892205;
const unsigned long  IDNo34 = 1404290087;
const unsigned long  IDNo35 = 1124592167;
const unsigned long  IDNo36 = 909560866;
const unsigned long  IDNo37 = 1917504292;
const unsigned long  IDNo38 = 1129588263;
const unsigned long  IDNo39 = 1125363751;
const unsigned long IDNo43 = 2691771556;
const unsigned long IDNo44 = 1129837351;
const unsigned long IDNo45 = 644198946;
const unsigned long IDNo46 = 649376290;
const unsigned long IDNo47 = 644597538;
const unsigned long IDNo48 = 645654050;
const unsigned long IDNo49 = 908125986;
const unsigned long IDNo50 = 870354471;
const unsigned long IDNo51 = 641960994;
const unsigned long IDNo52 = 864795175;
const unsigned long IDNo53 = 1924384036;
const unsigned long IDNo54 = 2485213214;
const unsigned long IDNo55 = 2461214497;
const unsigned long IDNo56 = 382526994;
const unsigned long IDNo57 = 3804710;
const unsigned long IDNo58 = 1171485483;
const unsigned long IDNo59 = 2219427870;
const unsigned long IDNo60 = 2475915556;
const unsigned long IDNo61 = 1172171819;
const unsigned long IDNo62 = 1668766247;
const unsigned long IDNo63 = 1402798887;
const unsigned long IDNo64 = 1663632679;
const unsigned long IDNo65 = 1671576103;
const unsigned long IDNo66 = 1408794919;
const unsigned long IDNo67 = 863999783;
const unsigned long IDNo68 = 1131704103;
const unsigned long IDNo69 = 906737442;
const unsigned long IDNo70 = 870631719;
const unsigned long IDNo71 = 1131134247;
const unsigned long IDNo72 = 642094882;
const unsigned long IDNo73 = 640544802;
const unsigned long IDNo74 = 864495655;

const unsigned long IDNo75 = 640638754 ;
const unsigned long IDNo76 = 653320738 ;
const unsigned long IDNo77 = 865896231 ;
const unsigned long IDNo78 = 1358432263 ;
const unsigned long IDNo79 = 1347086855 ;
const unsigned long IDNo80 = 1912395529 ;
const unsigned long IDNo81 = 1346890759 ;
const unsigned long IDNo82 = 1911782409 ;
const unsigned long IDNo83 = 1907722761 ;
const unsigned long IDNo84 = 1904493833 ;
const unsigned long IDNo85 = 1344639239 ;
const unsigned long IDNo86 = 1345546503 ;
const unsigned long IDNo87 = 1342913799 ;
const unsigned long IDNo88 = 1924713764 ;
const unsigned long IDNo89 = 1122658340 ;
const unsigned long IDNo90 = 642510370 ;
const unsigned long IDNo91 = 1916999204 ;
const unsigned long IDNo92 = 1118269476 ;
const unsigned long IDNo93 = 863412263 ;
const unsigned long IDNo94 = 1668822823 ;
const unsigned long IDNo95 = 1159891757 ;
const unsigned long IDNo96 = 1924134180 ;
const unsigned long IDNo97 = 1920996900 ;
const unsigned long IDNo98 = 903611949 ;
const unsigned long IDNo99 = 1432406573 ;
const unsigned long IDNo100 = 652700194 ;


//1200942445 - 4,1870775848 - 1
//1863600424 -2  1396368686 - 3
//656470381 -5 930540397-6 661132654 - 8 1731786093 - 9 3072335214-10
byte point[] = {POINT1, POINT2};
unsigned long cardPoint[] = {IDNo22, IDNo4};
int pointNum = sizeof(point);
unsigned long pointSend = 99, sendTime = 0, informTime = 0 ;
bool pointTaskFlag = true;
String pointMess , responseMess; // "#______data____\n",
bool button1low = false, button2low = false;
int event1 = EVENT_NORMAL;

const int schedule1Point = 34;
const unsigned long schedule1[schedule1Point] = {IDNo1 , IDNo2 , IDNo3 , IDNo94 , IDNo4 , IDNo119 , IDNo12 , IDNo13 , IDNo132 , IDNo124 , IDNo248 , IDNo125 , IDNo201 , IDNo126 , IDNo54 , IDNo127 , IDNo131 , IDNo210 , IDNo123 , IDNo117 , IDNo199 , IDNo115 , IDNo118 , IDNo39 , IDNo22 , IDNo23 , IDNo161 , IDNo250 , IDNo24 , IDNo62 , IDNo202 , IDNo302 , IDNo236 , IDNo190
};
const int schedule1Event[schedule1Point] = {EVENT_SLOW , EVENT_TURNRIGHT_IN3 , EVENT_NORMAL , EVENT_TURNRIGHT_IN2 , EVENT_TURNRIGHT_IN , EVENT_NORMAL , EVENT_TURNLEFT_IN3 , EVENT_NORMAL , EVENT_NORMAL , EVENT_TURN , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_TURNLEFT_IN , EVENT_TURNLEFT_IN , EVENT_TURNRIGHT_IN , EVENT_NORMAL , EVENT_TURNRIGHT_IN3 , EVENT_TURNRIGHT_IN3 , EVENT_NORMAL , EVENT_TURNLEFT_IN3 , EVENT_TURNLEFT_IN3 , EVENT_TURNRIGHT_IN , EVENT_TURNRIGHT_IN , EVENT_NORMAL , EVENT_NORMAL , EVENT_STOP , EVENT_STOP
};
int recentSchedule1 = 0;
bool scheduleFlag = false;
int scheduleSpeed;


int event3 = EVENT_TURN;

const int schedule3Point = 24;
const unsigned long schedule3[schedule3Point] = {IDNo3 , IDNo4 , IDNo94 , IDNo16 , IDNo133 , IDNo134 , IDNo74 , IDNo73 , IDNo62 , IDNo190 , IDNo208 , IDNo249 , IDNo209 , IDNo210 , IDNo131 , IDNo123 , IDNo117 , IDNo199 , IDNo118 , IDNo39 , IDNo22 , IDNo23 , IDNo24 , IDNo25
                                                };
const int schedule3Event[schedule3Point] = {EVENT_NORMAL , EVENT_TURNLEFT_IN3 , EVENT_TURNLEFT_IN2 , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_TURNRIGHT_IN , EVENT_TURNRIGHT_IN3 , EVENT_TURNRIGHT_IN , EVENT_STOP , EVENT_TURN , EVENT_NORMAL , EVENT_TURNRIGHT_IN , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_TURNLEFT_IN , EVENT_TURNLEFT_IN , EVENT_NORMAL , EVENT_TURNRIGHT_IN3 , EVENT_TURNRIGHT_IN3 , EVENT_NORMAL , EVENT_TURNLEFT_IN , EVENT_STOP
                                           };

int event4 = EVENT_TURN;
const int schedule4Point = 24;
const unsigned long schedule4[schedule4Point] = {IDNo3 , IDNo4 , IDNo94 , IDNo16 , IDNo133 , IDNo134 , IDNo74 , IDNo73 , IDNo62 , IDNo190 , IDNo208 , IDNo249 , IDNo209 , IDNo210 , IDNo131 , IDNo123 , IDNo117 , IDNo199 , IDNo118 , IDNo39 , IDNo22 , IDNo23 , IDNo24 , IDNo25
                                                };
const int schedule4Event[schedule4Point] = {EVENT_NORMAL , EVENT_TURNLEFT_IN3 , EVENT_TURNLEFT_IN2 , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_TURNRIGHT_IN , EVENT_TURNRIGHT_IN3 , EVENT_TURNRIGHT_IN , EVENT_STOP , EVENT_TURN , EVENT_NORMAL , EVENT_TURNRIGHT_IN , EVENT_NORMAL , EVENT_NORMAL , EVENT_NORMAL , EVENT_TURNLEFT_IN , EVENT_TURNLEFT_IN , EVENT_NORMAL , EVENT_TURNRIGHT_IN3 , EVENT_TURNRIGHT_IN3 , EVENT_NORMAL , EVENT_TURNLEFT_IN , EVENT_STOP
                                           };




bool brakeFlag = false, timerFlag = false;
bool lamp1Flag = false, bumpActive = false, lineTrack = true , EMGFlag = false, lampStatus1 = false, lampStatus2 = false, lampStatus3 = false, safetyFlag = false;

byte safetySensor = 0;
bool safetySFlag = 1, rfStopFlag = false, rfSlowFlag = false;
int safetyCount = 0, safetyCount1 = 0;
double safetyResult = 0, safetySum = 0;
unsigned long timeCount, bumpDelay = 0, safetyDelay = 0;
int route;
bool a;

class LightSensor {
  private:
    int lightCount = 0;
    unsigned long lightDelay = 0;
    long sensorValue = 0;
    int inputPin;
    int sensorScale;

  public:
    bool lightStatus = true;
    LightSensor(int _inputPin, int _sensorScale);
    bool getStatus();
    void getValue();
};

LightSensor lightSensor1(LIGHTSENSOR1, LIGHTSCALE1);
LightSensor lightSensor2(LIGHTSENSOR2, LIGHTSCALE2);
LightSensor lightSensor3(LIGHTSENSOR3, LIGHTSCALE3);
LightSensor lightSensor4(LIGHTSENSOR4, LIGHTSCALE4);
bool lightSafetySensor;

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define VOLTAGEPIN 15
#define CURRENTPIN 13
#define SCALE_VOLTAGE 0.040385505111
#define OFFSET_ACS 512
//#define SCALE_CURRENT 0.039527272
#define SCALE_CURRENT 0.01
#define OVER_CURRENT 8.00
#define LOW_VOLTAGE 22.00
#define OVERLOAD_COUNT 100
#define LOW_BATTERY_COUNT 150
#define LCD_REFRESH_TIME 500
LiquidCrystal_I2C lcd(0x27, 16, 2);
int firstActiveCount = 0;
double analogValue(byte times, int pin) {
  int readValue = 0;
  double value = 0;
  for (int i = 0; i < times; i++) {
    readValue += analogRead(pin);
  }
  value = (double)readValue / times;
  return value;
}
double voltagePrev[3], currentPrev[3];
int ACSValue = 0;
double currentValue = 0;
unsigned long readValueTimer = 0, sendValueTimer = 0;
double voltageValue = 0;
bool phaseDisplay = false, fristActive = true;
double getVoltage() {
  voltageValue =  analogValue(5, VOLTAGEPIN);
  voltageValue *= SCALE_VOLTAGE;
  double value = voltageValue;
  voltageValue = (voltageValue + voltagePrev[0] + voltagePrev[1] + voltagePrev[2]) / 4;
  voltagePrev[2] = voltagePrev[1];
  voltagePrev[1] = voltagePrev[0];
  voltagePrev[0] = voltageValue;
  return value;
}
double getCurrent() {
  ACSValue = (double)analogValue(5, CURRENTPIN);
  if (ACSValue < OFFSET_ACS - 1) {
    currentValue = (OFFSET_ACS - ACSValue) * SCALE_CURRENT;
  }
  else if (ACSValue > OFFSET_ACS + 1) {
    currentValue = (ACSValue - OFFSET_ACS) * SCALE_CURRENT;
  }
  else currentValue = 0;

  double value = currentValue;
  currentValue = (currentValue + currentPrev[0] + currentPrev[1] + currentPrev[2]) / 4;
  currentPrev[2] = currentPrev[1];
  currentPrev[1] = currentPrev[0];
  currentPrev[0] = currentValue;
  return value;
}
void lcdDisplay() {
  if (millis() - readValueTimer > LCD_REFRESH_TIME) {
    lcd.setCursor(11, 1);
    lcd.print(event1);
    lcd.print(event3);
    lcd.setCursor(2, 0);
    lcd.print(voltageValue);
    lcd.print("V");// pos 10;
    lcd.setCursor(2, 1);
    lcd.print(currentValue);
    lcd.print("A");// pos 10;
    readValueTimer = millis();
  }

  if (millis() - sendValueTimer > 100050 || (fristActive)) {
    batteryInform();
    sendValueTimer = millis();
    fristActive = false;
  }
}
int overCurrentCount, lowVoltageCount;

void batteryCheck() {
  getCurrent();
  getVoltage();
  if (currentValue > OVER_CURRENT && overCurrentCount < 1000) {
    overCurrentCount++;
  }
  else if (currentValue <= OVER_CURRENT) overCurrentCount = 0;
  if (voltageValue < LOW_VOLTAGE && lowVoltageCount < 1000) {
    lowVoltageCount++;
  }
  else if (voltageValue >= LOW_VOLTAGE) lowVoltageCount = 0;
  if (overCurrentCount > OVERLOAD_COUNT) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("QUA TAI: ");
    lcd.print(currentValue);
    lcd.print("A");
    lcd.setCursor(0, 1);
    lcd.print("KIEM TRA LAI AGV");
    analogWrite(LM, 0); analogWrite(RM, 0);
    stopCard = OVERLOAD;
    UART(stopCard);
    delay(1000);
    batteryInform();
    delay(1000);
    batteryInform();
    StopAGV();
    while (1);
  }
  while (lowVoltageCount > LOW_BATTERY_COUNT) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HET PIN: ");
    lcd.print(voltageValue);
    lcd.print("V");
    lcd.setCursor(0, 1);
    lcd.print("CAN THAY THE PIN");
    analogWrite(LM, 0); analogWrite(RM, 0);
    
    stopCard = BATTERYDOWN;
    UART(stopCard);
    delay(1000);
    batteryInform();
    delay(1000);
    batteryInform();
    StopAGV();
    while (1);

  }
}
void batteryInform() {
  char tempChar[10];
  int tempValue = voltageValue * 100;
  sprintf(tempChar, "<%d\n", tempValue);
  String tempString = tempChar;
  Serial.print(tempString);
  Serial1.print(tempString);
  tempValue = currentValue * 100;
  char tempChar2[10];
  sprintf(tempChar2, ">%d\n", tempValue);
  tempString = "";
  tempString = tempChar2;
  Serial.print(tempString);
  Serial1.print(tempString);

}
void setup()
{

  // OUTPUT Setting
  pinMode(RM, OUTPUT);
  pinMode(RBRK, OUTPUT);
  pinMode(LM, OUTPUT);
  pinMode(LBRK, OUTPUT);
  pinMode(SPK, OUTPUT);
  pinMode(LAMP1, OUTPUT);
  pinMode(LAMP2, OUTPUT);

  pinMode(SS1_TRIG, OUTPUT);
  pinMode(SS2_TRIG, OUTPUT);
  pinMode(SS3_TRIG, OUTPUT);
  pinMode(FORK_L, OUTPUT);
  pinMode(FORK_R, OUTPUT);

  // INPUT Setting
  pinMode(RFSTOP, INPUT);
  pinMode(START1, INPUT_PULLUP);
  pinMode(START2, INPUT);
  pinMode(SS1_ECH, INPUT_PULLUP);
  pinMode(SS2_ECH, INPUT_PULLUP);
  pinMode(SS3_ECH, INPUT_PULLUP);
  pinMode(SAFETY, INPUT_PULLUP);
  pinMode(BUMP, INPUT);
  pinMode(EMG, INPUT);

  pinMode(RALM, INPUT);
  pinMode(LALM, INPUT);
  //
  Serial.begin(74880);
  Serial1.begin(74880);
  SPI.begin();
  mfrc522.PCD_Init();

  digitalWrite(LAMP1, LOW);
  digitalWrite(LAMP2, LOW);
  digitalWrite(SPK, LOW);
  digitalWrite(RBRK, LOW);
  digitalWrite(LBRK, LOW);
  digitalWrite(FORK_L, HIGH);
  digitalWrite(FORK_R, LOW);
  //settingSpeed(TURNSPEED);
  //digitalWrite(LED, LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(8);
  nameCall();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("U:");
  lcd.setCursor(0, 1);
  lcd.print("I:");
  lcd.setCursor(11, 1);
  lcd.print("AGV03");
  lcd.setCursor(11, 0);
  lcd.print("MS");

  voltagePrev[0] = getVoltage();
  voltagePrev[1] = voltagePrev[0];
  voltagePrev[2] = voltagePrev[0];

  currentPrev[0] = getCurrent();
  currentPrev[1] = currentPrev[0];
  currentPrev[2] = currentPrev[0];
  Serial1.print('\n');


}
/*********** HAM DIEU KHIEN MOTOR **************/

/********** HAM DUNG DONG CO ***********/
void StopAGV()
{
  smaxr = 0;
  smaxl = 0;
  analogWrite(RM, 0);
  analogWrite(LM, 0);
  digitalWrite(RBRK, HIGH);
  digitalWrite(LBRK, HIGH);

}
/*********** HAM DOC THE RFID *************/
void RFIDread()
{
  // Searching new card
  if ( ! mfrc522.PICC_IsNewCardPresent())
  {
    return;
  }
  // Reading card
  if ( ! mfrc522.PICC_ReadCardSerial())
  {
    return;
  }
  CardID = 0;
  if (CardID == IDNo46) CardID = IDNo6;
  if (EMGFlag) updateLineFlag = true;
  else updateLineFlag = false;
  // Show Card ID
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    CardIDTemp = mfrc522.uid.uidByte[i];
    CardID = CardID * 256 + CardIDTemp;
  }
  CardID = replaceCard(CardID);
  Serial.println(CardID);
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  if ((lampStatus1 && checkCardMSKTr(CardID)) || (lampStatus2 && checkCardMSIOT(CardID))) {

    char mystr[40];
    pointMess = "";
    sprintf(mystr, "#%lu\n", 10001);
    pointMess += mystr;
    Serial1.print(pointMess);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("AGV NHAM DUONG");
    lcd.setCursor(0, 1);
    lcd.print(CardID);

    StopAGV();
    while (1);
  }

}
//1200942445 - 4,1870775848 - 1
//1863600424 -2  1396368686 - 3
//656470381 -5 930540397-6 661132654 - 8 1731786093 - 9 3072335214-10

unsigned long timee;
/*********** MAIN ***************/
void loop()
{

  srfCalculate();
  getInput();
  analyst();
  batteryCheck();
  lcdDisplay();
  if (rightALM == LOW || leftALM == LOW) {
    while (1) {
      
      analogWrite(LM, 0); analogWrite(RM, 0);
      StopAGV();
    }
  }

  UART();
  updateUART1();
  //remoteRequest();
  /* //Serial.println( ss1.getTotalStt());
    if (digitalRead(EMG)) mode = AUTO;
    else mode = MAN;
    switch (mode)
    {
    case AUTO:
      if ( a = ((digitalRead(EMG) == 1) && ( ss1.getTotalStt() || analogRead(A1) < 800 || digitalRead(BUMP) == HIGH)) && digitalRead(LAMP1) == HIGH )
      {
        if ( ss1.getTotalStt() == 1 && analogRead(A1) > 800  && digitalRead(BUMP) != HIGH && digitalRead(LAMP1) == HIGH)
        {
          Speedcontrol2();

        }
        else
        {
          StopAGV();

        }
      }
      else
      {
        Speedcontrol1();
        //Serial.println( " b");
      }
      break;
    case MAN:
    //Serial.println( " a");
      StopAGV();
      while(digitalRead(START2)){analogWrite(RM,60);analogWrite(LM,30);digitalWrite(RBRK, LOW);digitalWrite(LBRK, LOW);}
      while(digitalRead(START3)){analogWrite(RM,30);analogWrite(LM,60);digitalWrite(RBRK, LOW);digitalWrite(LBRK, LOW);}
     // while(digitalRead(START1)){analogWrite(RM,100);analogWrite(LM,100);digitalWrite(RBRK, LOW);digitalWrite(LBRK, LOW);}
      digitalWrite(LAMP1, LOW);
      break;
    }

    //Serial.println(analogRead(A0));*/
Serial.print(EMGFlag);
Serial.print("     ");
Serial.print(lampStatus1);
Serial.print("     ");
    Serial.print(safetyFlag);
    Serial.print("     ");
    Serial.print(safetySensor);
    Serial.print("     ");
    Serial.println(rfStopFlag);
    delay(1);
}


void turnRight() {
  digitalWrite(FORK_L, HIGH);
  digitalWrite(FORK_R, LOW);
}
void turnLeft() {
  digitalWrite(FORK_L, LOW);
  digitalWrite(FORK_R, HIGH);
}
void noTurn() {
  digitalWrite(FORK_L, LOW);
  digitalWrite(FORK_R, LOW);
}
void settingSpeed(int a) {
  smaxr = a;
  smaxl = a;
}
void softStop(int x) {
  if (smaxr > smaxl) smaxr = smaxl;
  else smaxl = smaxr;
  if (( smaxr  >= x || smaxl  >= x ) && millis() - timeCount > 10 ) {
    if (smaxr >= x) smaxr -= x;
    if (smaxl >= x) smaxl -= x;
    timeCount = millis();
  }
  else if ( smaxr  < x && smaxl  < x ) {
    StopAGV();
  }
}
void softStart(int setPoint) {
  if (setPoint == 0) return;
  digitalWrite(RBRK, LOW);
  digitalWrite(LBRK, LOW);
  smaxr = setPoint + setPoint * RIGHTADD / 100;
  smaxl = setPoint + setPoint * LEFTADD / 100;
  motor(Output);


}
void getInput () {

  if (checkButton(START1, LOW)  ) {
    if (lampStatus1 == false && button1low == true) {
      openLamp(LAMP1);
      button1low = false;
    }
    else if (lampStatus1 == true && button1low == true) {
      closeLamp();
      button1low = false;
    }
  }
  else if (checkButton(START2, HIGH)) {
    if (lampStatus2 == false && button2low == true) {
      openLamp(LAMP2);
      button2low = false;
    }
    else if (lampStatus2 == true && button2low == true) {
      closeLamp();
      button2low = false;
    }
  }
  else {
    if (digitalRead(START1) == HIGH) {
      button1low = true;
    }
    if (digitalRead(START2) == LOW) {
      button2low = true;
    }
  }
  
  rfStopFlag = digitalRead(RFSTOP);
  digitalLightSensorActive = getDigitalLightSensor2();
  lightSafetySensor = lightSensor1.getStatus() * lightSensor2.getStatus()  * lightSensor3.getStatus()  * lightSensor4.getStatus() ;
 /* Serial.print("1_____");
    lightSensor1.getValue();
    Serial.print("2_____");
    lightSensor2.getValue();
    Serial.print("3_____");
    lightSensor3.getValue();
    Serial.print("4_____");
    lightSensor4.getValue();*/
    
  if (digitalRead(EMG)) EMGFlag = false; // EMG safe
  else {
    closeLamp(); EMGFlag = true;
  } // EMG unsafe

  if (getLine()) {
    lineTrack = false;
    closeLamp();
  }
  else lineTrack = true;

  safetySensor = ss1.getTotalStt();

  if (lineTrack) pos = analogRead(A0);
  else pos = Setpoint;

  if (((lampStatus1 == HIGH ) || lampStatus2 == HIGH  || lampStatus3 == HIGH  ||  prevSpeed > 0) && (lightSafetySensor && safetySensor != 2 && digitalLightSensorActive)  )Input = inputScale(pos);
  else Input = Setpoint  ;

  myPID.Compute();

  //Serial.println(pos);

  //checkSafetySensor();

  RFIDread();

  safetyFlag = safetySensor != 2  && !EMGFlag  && lineTrack && lightSafetySensor ;  //safetyFlag =    1 - Safe       0 - Unsafe


  //speedModify();

  leftALM = digitalRead(LALM);
  rightALM = digitalRead(RALM);
}
void openLamp(int Lamp) {

  if (Lamp == LAMP1) {
    digitalWrite(LAMP1, HIGH);
    digitalWrite(LAMP2, LOW);
    digitalWrite(SPK, HIGH);

    lampStatus1 = true;
    lampStatus2 = false;
    lampStatus3 = false;
  }
  else if (Lamp == LAMP2) {
    digitalWrite(LAMP1, LOW);
    digitalWrite(LAMP2, HIGH);
    digitalWrite(SPK, HIGH);
    lampStatus1 = false;
    lampStatus2 = true;
    lampStatus3 = false;
  }
  /*else if (Lamp == LAMP3) {
    digitalWrite(LAMP1, LOW);
    digitalWrite(LAMP2, LOW);
    digitalWrite(LAMP3, HIGH);
    digitalWrite(RLAMP, HIGH);
    lampStatus1 = false;
    lampStatus2 = false;
    lampStatus3 = true;
    }*/

}
void closeLamp() {
  //Serial.println(closeStatus);
  ///EEPROM.update(99,closeStatus);
  digitalWrite(LAMP1, LOW);
  digitalWrite(LAMP2, LOW);
  digitalWrite(SPK, LOW);
  lampStatus1 = false;
  lampStatus2 = false;
  lampStatus3 = false;

}
void analyst() {
  if (EMGFlag)
  {
    stopCard = EMGSTOP;
    UART(stopCard);
    while (digitalRead(START2)) {
      analogWrite(RM, 60);
      analogWrite(LM, 30);
      digitalWrite(RBRK, LOW);
      digitalWrite(LBRK, LOW);
      smaxr = 50;
      smaxl = 50;
    }
    while (digitalRead(START2) && !digitalRead(START1)) {
      analogWrite(RM, 30);
      analogWrite(LM, 60);
      digitalWrite(RBRK, LOW);
      digitalWrite(LBRK, LOW);
      smaxr = 50;
      smaxl = 50;
    }
    while (!digitalRead(START1 )) {
      analogWrite(RM, 70 + 70 * RIGHTADD / 100);
      analogWrite(LM, 70 + 70 * LEFTADD / 100);
      digitalWrite(RBRK, LOW);
      digitalWrite(LBRK, LOW);
      smaxr = 100;
      smaxl = 100;
    }
    StopAGV();
    softStop(100);
  }


  else if (lampStatus1 && safetyFlag &&  !rfStopFlag) {
    stopCard = KHOTRONG;
    UART(stopCard);
    event1 = checkSchedule1();
    eventOutput(event1);//scheduleSpeed
    softStart(scheduleSpeed);
  }
  else if (lampStatus2 && safetyFlag &&  !rfStopFlag) {

    stopCard = IOT3;
    UART(stopCard);
    event3 = checkSchedule3(); //Serial.println("2 status");
    eventOutput(event3);//scheduleSpeed
    softStart(scheduleSpeed);
  }
  
  else if (((!lampStatus1 && !lampStatus2) || rfStopFlag) && safetyFlag) { // || rfStopFlag
    holdSpeed(0);
    softStart(scheduleSpeed);
    if ((!lampStatus1 && !lampStatus2)) {
      stopCard = PRESSSTOP;
      UART(stopCard);
    }
    else if (rfStopFlag) {
      stopCard = AVOIDSTOP;
      UART(stopCard);
    }
  }

  else {
    //closeLamp();
    softStop(30);
    stopCard = UNKNOWSTOP;
    if (safetySensor == 2) stopCard = SAFETYSTOP;
    else if (!lineTrack) stopCard = LINETRACKSTOP;
    else if (!lightSafetySensor && !digitalLightSensorActive)  stopCard = LIGHTSENSORSTOP; //safetyFlag = safetySensor != 2  && !EMGFlag  && lineTrack && lightSafetySensor;
    UART(stopCard);
  }
}
int checkSchedule1() {
  if (prevCard != CardID) {
    prevCard = CardID;
    prevStopCard = UNKNOWSTOP;
    stopCard = UNKNOWSTOP;
    pointTaskFlag = true;
    for (int i = 0 ; i < schedule1Point; i++ ) {
      if ( CardID == schedule1[i] ) {
        //if (schedule1Event[i] == EVENT_STOP && (event1 == EVENT_STOP )){return NO_EVENT;}

        return schedule1Event[i];
      }
    }
    return NO_EVENT;
  }
  return event1;
}

/*
      >>>>>>>>>>>>> (A1)123728749 >>>>>>>>>>>>>>>>>>>>>>>>(B1) 1070081065 >>>>>>>>>>>>>>>>>>(C1) 4158125421 >>>>>>>>>>>>>>>>>(D1) 1607974440 >>>>>
      <<<<<(A2) 2400236073 <<<<<<<<<<<<<<<(B2) 1329283881 <<<<<<<<<<<<<<<<<<<<<<<(C2) 3884446573<<<<<<<<<<<<<<<<<<<(D2) 666823789 <<<<<<<<<<<<<<<<
*/
void eventOutput(int evt) {
  switch (evt) {
    case  EVENT_TURNLEFT_IN:
      holdSpeed(TURNSPEED);
      turnLeft();
      break;

    case  EVENT_TURNLEFT_IN1:
      holdSpeed(SLOWSPEED);
      turnLeft();
      break;

    case EVENT_TURNLEFT_OUT:
      scheduleSpeed = NORMALSPEED;
      noTurn();
      break;

    case EVENT_TURNRIGHT_IN:
      holdSpeed(TURNSPEED);
      turnRight();
      break;
    case EVENT_TURNRIGHT_IN1:
      holdSpeed(SLOWSPEED);
      turnRight();
      break;
    case EVENT_TURNLEFT_IN2:
      holdSpeed(BOOSTSPEED);
      turnLeft();
      break;
    case EVENT_TURNRIGHT_IN2:
      holdSpeed(BOOSTSPEED);
      turnRight();
      break;
    case EVENT_TURNLEFT_IN3:
      holdSpeed(NORMALSPEED);
      turnLeft();
      break;
    case EVENT_TURNRIGHT_IN3:
      holdSpeed(NORMALSPEED);
      turnRight();
      break;
    case EVENT_TURNRIGHT_OUT:
      holdSpeed(NORMALSPEED);
      noTurn();
      break;

    case EVENT_STOP:
      scheduleSpeed = SLOWSPEED;
      UART();
      CardID = 200;
      //closeStatus = 5;
      closeLamp();
      noTurn();
      event1 = NO_EVENT;
      break;
    case EVENT_BOOST:
      holdSpeed(BOOSTSPEED);
      noTurn();
      break;
    case EVENT_NORMAL:
      holdSpeed(NORMALSPEED);
      noTurn();
      break;
    case EVENT_SLOW:
      holdSpeed(SLOWSPEED);
      noTurn();
      break;
    case EVENT_TURN:
      holdSpeed(TURNSPEED);
      noTurn();
      break;
    default:
    case NO_EVENT:
      holdSpeed(NORMALSPEED);
      turnLeft();
      break;
  }
}
void speedModify() {
  if (rfSlowFlag) {
    NORMALSPEED = 60;
    DEFAULTSPEED = 60;
    TURNSPEED = 60;
    BOOSTSPEED = 60;
    SLOWSPEED = _SLOW;
  }
  else if (!rfSlowFlag) {
    NORMALSPEED = _TURN;
    DEFAULTSPEED = _TURN;
    TURNSPEED = _TURN ;
    BOOSTSPEED = _TURN;
    SLOWSPEED = _SLOW;
  }
}


int inputScale(int input) {

  if (input > 430) return 660;
  else if (input < 205 ) return 0;
  else if (input >= 310 && input <= 330) return 320;
  //else if (input < 320   && input >= 205) return input-8;
  else return input;
}
void motor (int output) {
  int speedR = smaxr, speedL = smaxl;
  output *= 10;
  if (output < 0) speedR -= map(output * -1, 0, 2550, 3, 92) * speedR / 100;
  if (output > 0) {
    speedL -= map(output, 0, 2550, 3, 92) * speedL / 100;
  }

  analogWrite(RM, speedR);
  analogWrite(LM, speedL);
}
void nameCall() {
  String nameCall;
  switch (NAME) {
    case 1:
      nameCall = "CAT BA";
      break;
    case 2:
      nameCall = "HA LONG";
      break;
    case 3:
      nameCall = "BAI CHAY";
      break;
    case 4:
      nameCall = "DO SON";
      break;
    case 5:
      nameCall = "TRA CO";
      break;
    default:
      nameCall = "NAME HAS NOT BEEN FOUND";
      break;
  }
  Serial.print("AGV NAME: " );
  Serial.println(nameCall);
  Serial.print("RIGHT ADD : " );
  Serial.print(RIGHTADD);
  Serial.print("%" );
  Serial.print("    --    LEFT ADD : " );
  Serial.print(LEFTADD);
  Serial.println("%" );

}
void holdSpeed(int x) {
  if (smaxr >= smaxl) currentSpeed = smaxl ; // lay gia tri toc do hien tai
  else currentSpeed = smaxr ;
  prevSpeed = currentSpeed  ;

  if (currentSpeed > x ) {
    currentSpeed -= (0.42 + pointValue);  // giam gia tri
    pointValue = prevSpeed - currentSpeed;
  }
  else if (currentSpeed == x) {
    currentSpeed = x;  // giu nguyen
    pointValue = 0;
  }
  else if (currentSpeed < x && currentSpeed >= BEGINSPEED) {
    currentSpeed += (0.28 + pointValue);  //  tang gia tri
    pointValue = currentSpeed - prevSpeed;
  }
  else {
    currentSpeed = BEGINSPEED;  // start speed
    pointValue = 0;
  }
  if (x == 0 && currentSpeed < 40) {
    //StopAGV();  // che do dung khi dat gia tri
    pointValue = 0;
    currentSpeed = 0;
  }
  if (pointValue >= 1.0) pointValue -= 1;
  scheduleSpeed = currentSpeed;


}
bool   getLine() {
  for (int i = 1; i <= 5; i++) {
    if (analogRead(LINETRACK) > 750) return 0;
    else delay(20);
  }
  return 1;
}
bool   checkButton(byte button, bool logic) {
  for (int i = 1; i <= 5; i++) {
    if (digitalRead(button) != logic) return 0;
    else delayMicroseconds(100);
  }
  return 1;
}
/*void checkSafetySensor(){
  bool checkSensor = digitalRead(SAFETY);



  if (safetySFlag){

    if (checkSensor == HIGH) {safetySensor= LOW;safetyCount = 0;safetySFlag= true ;return;}
    else safetyCount++;
    if (safetyCount > 10) {safetySensor= HIGH;safetyCount = 0;safetyCount1 = 0;safetySum=0;safetyResult=0;safetySFlag= false ;return;}

    }
  else {

    safetyCount++ ;
    safetySum += checkSensor ;
    safetyResult = safetySum/safetyCount;
    if (safetyCount >= 30 ) {
      if (safetyResult > 0.3) { safetyCount1 ++;}
      else {safetyCount1 = 0;safetyDelay = millis();}
      safetyCount = 0;
      safetySum=0;
      safetyResult=0;
      }

    if (safetyCount1 > 4 && millis() - safetyDelay >5000) {safetySensor= LOW;safetyCount = 0;safetySFlag= true ;return;}
    }


  }
*/
void UART() {

  if (!pointTaskFlag) return;
  pointMess = "";
  char mystr[40];
  sprintf(mystr, "#%lu\n", prevCard);
  pointMess += mystr;
  Serial.print(pointMess);
  Serial1.print(pointMess);
  sendTime = micros();
  pointTaskFlag = false;
  return;
}

void UART(unsigned long stopCard) {

  if ( stopCard == prevStopCard) return;
  pointMess = "";
  char mystr[40];
  sprintf(mystr, "#%lu\n", stopCard);
  pointMess += mystr;
  Serial.print(pointMess);
  Serial1.print(pointMess);
  sendTime = micros();
  prevStopCard = stopCard;

  return;
}
void updateLineChoose() {
  if (EMGFlag && updateLineFlag) {
    if (CardID == 1560729971 || CardID == 1885198707) {
      EEPROM.update(EPPROM_LINESELECT, 10);
      LINESELECT = 10;
    }
    else if (CardID == 3823491187 || CardID == 896229795) {
      EEPROM.update(EPPROM_LINESELECT, 0);
      LINESELECT = 0;
    }
    while (1) {
      if (LINESELECT == 0) digitalWrite(LAMP2, HIGH);
      else digitalWrite(LAMP1, HIGH);
    }
  }
}
void getInput2 () {
  if (checkButton(START1, HIGH)  ) {
    if (lampStatus1 == false && button1low == true) {
      openLamp(LAMP1);

      button1low = false;
    }
    else if (lampStatus1 == true && button1low == true) {
      closeLamp();
      button1low = false;
    }
  }
  /*else if (checkButton(START2,LOW)){


    if (lampStatus2 == false && button2low == true){
      openLamp(LAMP2);
      button2low = false;
    }
    else if(lampStatus2 == true && button2low == true) {
      closeLamp();
      button2low = false;
    }
    }*/
  else {
    if (digitalRead(START1) == LOW) {
      button1low = true;
    }
    /*if (digitalRead(START2)==HIGH){
      button2low = true;
      }*/
  }

  digitalLightSensorActive = getDigitalLightSensor2();
  lightSafetySensor = lightSensor1.getStatus() * lightSensor2.getStatus()  * lightSensor3.getStatus()  * lightSensor4.getStatus() ;
  /*Serial.println("aaaa");
    lightSensor1.getValue();
    lightSensor2.getValue();
    lightSensor3.getValue();
    lightSensor4.getValue(); */

  rfStopFlag = digitalRead(RFSTOP);
  rfSlowFlag = digitalRead(RFSLOW);

  if (digitalRead(EMG)) EMGFlag = false; // EMG safe
  else {
    closeLamp(); EMGFlag = true;
  } // EMG unsafe

  if (getLine()) {
    lineTrack = false;
    closeLamp();
  }
  else lineTrack = true;

  safetySensor = ss1.getTotalStt();

  if (lineTrack) pos = analogRead(A0);
  else pos = Setpoint;

  if (((lampStatus1 == HIGH ) || lampStatus2 == HIGH  || lampStatus3 == HIGH  ||  prevSpeed > 0) && (lightSafetySensor && safetySensor != 2)  )Input = inputScale(pos);
  else Input = Setpoint  ;

  myPID.Compute();

  //Serial.println(pos);

  //checkSafetySensor();

  RFIDread();

  safetyFlag = safetySensor != 2  && !EMGFlag  && lineTrack && lightSafetySensor && digitalLightSensorActive;  //safetyFlag =    1 - Safe       0 - Unsafe


  //speedModify();

  leftALM = digitalRead(LALM);
  rightALM = digitalRead(RALM);
}
void analyst2() {
  if (EMGFlag)
  {
    stopCard = EMGSTOP;
    UART(stopCard);
    while (!digitalRead(START2)) {
      analogWrite(RM, 60);
      analogWrite(LM, 30);
      digitalWrite(RBRK, LOW);
      digitalWrite(LBRK, LOW);
      smaxr = 50;
      smaxl = 50;
    }
    while (digitalRead(START1) && !digitalRead(START2)) {
      analogWrite(RM, 30);
      analogWrite(LM, 60);
      digitalWrite(RBRK, LOW);
      digitalWrite(LBRK, LOW);
      smaxr = 50;
      smaxl = 50;
    }
    while (digitalRead(START1 )) {
      analogWrite(RM, 70 + 70 * RIGHTADD / 100);
      analogWrite(LM, 70 + 70 * LEFTADD / 100);
      digitalWrite(RBRK, LOW);
      digitalWrite(LBRK, LOW);
      smaxr = 100;
      smaxl = 100;
    }
    softStop(100);
  }

  else if (lampStatus1 && safetyFlag &&  !rfStopFlag) {
    stopCard = IOT3;
    UART(stopCard);
    event3 = checkSchedule3(); //Serial.println("2 status");
    eventOutput(event3);//scheduleSpeed
    softStart(scheduleSpeed);
      }
  else if (lampStatus2 && safetyFlag &&  !rfStopFlag) {
    stopCard = IOT3;
    UART(stopCard);
    event4 = checkSchedule4();
    eventOutput(event4);//scheduleSpeed
    softStart(scheduleSpeed);
  }
  else if (((!lampStatus1 && !lampStatus2) || rfStopFlag) && safetyFlag) { // || rfStopFlag
    holdSpeed(0);
    softStart(scheduleSpeed);
    if (!lampStatus1 && !lampStatus2) {
      stopCard = PRESSSTOP;
      UART(stopCard);
    }
    else if (rfStopFlag) {
      stopCard = AVOIDSTOP;
      UART(stopCard);
    }
  }
  else {
    //closeLamp();
    softStop(30);
    stopCard = UNKNOWSTOP;
    if (safetySensor == 2) stopCard = SAFETYSTOP;
    else if (!lineTrack) stopCard = LINETRACKSTOP;
    else if (!lightSafetySensor || !digitalLightSensorActive)  stopCard = LIGHTSENSORSTOP; //safetyFlag = safetySensor != 2  && !EMGFlag  && lineTrack && lightSafetySensor;
    UART(stopCard);
  }

}
int checkSchedule3() {
  if (prevCard != CardID) {
    prevCard = CardID;
    prevStopCard = UNKNOWSTOP;
    stopCard = UNKNOWSTOP;
    pointTaskFlag = true;
    for (int i = 0 ; i < schedule3Point; i++ ) {
      if ( CardID == schedule3[i] ) {
        //if (schedule1Event[i] == EVENT_STOP && (event1 == EVENT_STOP )){return NO_EVENT;}
        return schedule3Event[i];
      }
    }
    return NO_EVENT;
  }
  return event3;
}
int checkSchedule4() {
  if (prevCard != CardID) {
    prevCard = CardID;
    prevStopCard = UNKNOWSTOP;
    stopCard = UNKNOWSTOP;
    pointTaskFlag = true;
    for (int i = 0 ; i < schedule4Point; i++ ) {
      if ( CardID == schedule4[i] ) {
        //if (schedule2Event[i] == EVENT_STOP && (event2 == EVENT_STOP )){return NO_EVENT;}
        return schedule4Event[i];
      }
    }
    return NO_EVENT;
  }
  return event4;
}
LightSensor::LightSensor(int _inputPin, int _sensorScale) {
  inputPin = _inputPin;
  sensorScale = _sensorScale;
}
bool LightSensor::getStatus() {
  sensorValue = analogRead(inputPin) + sensorScale;
  if (sensorValue > LIGHTRANGE ) {
    if (lightCount < 10)  lightCount++;
  }
  else lightCount = 0;
  if (lightCount > 3) {
    lightStatus = false ;
    lightDelay = millis();
  }
  else if (lightCount <= 3 && millis() - lightDelay > 5000) {
    lightStatus =  true;
  }
  return lightStatus;
}
void LightSensor::getValue() {
  Serial.print(lightStatus);
  Serial.print("___");
  Serial.println(sensorValue);
}
void updateUART1() {
  //Serial.println(CardID);
  checkSerial1();
  if (stringComplete) {
    int _length = inputString.length();
    switch (inputString[0]) {
      case '!':
        if (inputString == "!START\r\n" && LINESELECT != 0) {
          if (CardID == 200 ) {
            openLamp(LAMP1);
          }
        }
        if (inputString == "!STOP\r\n" && LINESELECT != 0) {
          if (CardID == 656470381) {
            closeLamp();
            CardID = 200;
          }
        }
        break;
      default: break;
    }
    inputString = "";
    stringComplete = false;
  }
}
void checkSerial1() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
      Serial.println(inputString);
    }
  }
}
void remoteRequest() {
  if ((CardID == 200 )  && !lampStatus1 &&  LINESELECT != 0 && millis() - remoteRequestTimer > 1500) {
    Serial1.println("!KhoTrongRequest");
    Serial.println("!KhoTrongRequest");
    remoteRequestTimer = millis();
  }
  if ((CardID == 656470381 || CardID == 3297683241)  && lampStatus1 &&  LINESELECT != 0  && millis() - remoteRequestTimer > 500) {
    Serial1.println("!KhoTrongStopRequest");
    Serial.println("!KhoTrongStopRequest");
    remoteRequestTimer = millis();
  }
}
void srfCalculate() {
  if (srfFlag == true)ss2.caculate();
  else ss1.caculate();
  srfFlag = !srfFlag;
}
