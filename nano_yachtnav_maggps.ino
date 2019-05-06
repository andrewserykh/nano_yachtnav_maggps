/* v2.30

Watchdog таймер
Простой протокол связи обмен структурой данных
Функции управления выходными реле
Прием I2C данных компаса и данных GPS

TX, RX - связь с esp32
D10, D11 (RX,TX) - подключение GPS
A4, A5 - компас
LED - прием пакетов с GPS
*/

#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include "HMC5883.h"
#include "TinyGPSplus.h"

#define BAUD            9600  //Скорость последовательного порта
#define LED             13    //Светоиод на Arduino

#define COMPASS_COUNT   5     //Количество измерений для усреднения
#define COMPASS_FILTER  10    //Максимальное отклонение для отброса показаний

#define T_MS_COMPASS    500    //Интервал опроса компаса
#define T_MS_SEND       500   //Интервал отправки данных на esp
#define T_MS_RECV       100   //Оттяжка ожидания принятого пакета

#define RELAY_IN1       2     //Выходы на управление актуаторами через релейный модуль
#define RELAY_IN2       3     //на 4 реле
#define RELAY_IN3       4
#define RELAY_IN4       5

HMC5883 compass;
SoftwareSerial SSerialGPS(7, 8); // RX, TX [10,11]
TinyGPSPlus gps_parser;

struct { //Структура пакета данных измерений передаваемых из nano в esp
  byte Header; //заголовок пакета
  byte Length; //длина пакета
  uint16_t CompassCourse;
  uint16_t GpsCourse;
  uint16_t GpsSpeedKmh;
  float GpsSpeedKnots;
  float GpsLat;
  float GpsLng;
  uint16_t GpsAlt;
  uint16_t GpsHour;
  uint16_t GpsMinute;
  uint16_t GpsSecond;
  uint16_t GpsDay;
  uint16_t GpsMonth;
  uint16_t GpsYear;
} datanano;

struct { //Структура пакета команд передаваемых из esp в nano
  byte Header;
  byte Length;
  byte Relay;
} dataesp;

int compass_x,compass_y,compass_z;
float compass_hdg;
float compass_hdg_avg; //среднее
float compass_hdg_arr[COMPASS_COUNT];
int compass_arr_cnt=0;

char SerialEspIn[256];    //буфер приема 
byte SerialEspInLen;     //заполнение буфера

long ms_compass;
long ms_gps;
long ms_recv; //оттяжка для приема пакета
long ms_send; //счетчик для отправки данных в порт

void setup() {
  wdt_disable();
  pinMode(LED, OUTPUT); //D13 LED
  //digitalWrite(LED, HIGH);

  pinMode(RELAY_IN1,OUTPUT);
  pinMode(RELAY_IN2,OUTPUT);
  pinMode(RELAY_IN3,OUTPUT);
  pinMode(RELAY_IN4,OUTPUT);  

  Serial.begin(9600);
  SSerialGPS.begin(9600);  

  wdt_enable(WDTO_4S);  

  compass.init();
  //compass.read(&compass_x,&compass_y,&compass_z,&compass_hdg);
  //for (int i=0; i<COMPASS_COUNT; i++) compass_hdg_arr[i]=compass_hdg;

  digitalWrite(RELAY_IN1, LOW);
  digitalWrite(RELAY_IN2, LOW);
  digitalWrite(RELAY_IN3, LOW);
  digitalWrite(RELAY_IN4, LOW);
  //digitalWrite(LED, LOW);  
  ms_compass = ms_gps = ms_send = millis();
  wdt_reset();
}

void loop() {

  if(gps_parser.time.isValid()) {
    datanano.GpsCourse = (uint16_t)gps_parser.course.deg();
    datanano.GpsSpeedKmh = (uint16_t)gps_parser.speed.kmph();    
    datanano.GpsSpeedKnots = gps_parser.speed.knots();
    datanano.GpsLat = gps_parser.location.lat();
    datanano.GpsLng = gps_parser.location.lng();
    datanano.GpsAlt = (uint16_t)gps_parser.altitude.meters();
    datanano.GpsHour = (uint16_t)gps_parser.time.hour();
    datanano.GpsMinute = (uint16_t)gps_parser.time.minute();
    datanano.GpsSecond = (uint16_t)gps_parser.time.second();
    datanano.GpsDay = (uint16_t)gps_parser.date.day();
    datanano.GpsMonth = (uint16_t)gps_parser.date.month();
    datanano.GpsYear = (uint16_t)gps_parser.date.year();
  }

  if (millis() - ms_send > T_MS_SEND) {
    datanano.Header = 0x65;
    datanano.Length = (byte)sizeof(datanano);
    for (int i=0; i<sizeof(datanano); i++) Serial.write( ((char*)&datanano)[i] );
    ms_send = millis();
  }

  while (SSerialGPS.available()) {
    digitalWrite(LED, HIGH);
    char temp = SSerialGPS.read();
    gps_parser.encode(temp);
    digitalWrite(LED, LOW);
  }

  while (Serial.available() > 0) {
    char SerialChar = (char)Serial.read();
    SerialEspIn[SerialEspInLen] = SerialChar;
    SerialEspInLen++;
    if (SerialEspInLen>255) SerialEspInLen=0;
    ms_recv = millis();
  }
  
  if (SerialEspInLen > 0 && (millis() - ms_recv > T_MS_RECV)) {    
   //for (int q = 0; q < SerialEspInLen; q++) { Serial.print((byte)SerialEspIn[q], HEX); Serial.print(" "); } Serial.println(" ");
   if (SerialEspIn[0] == 0x69){ //Проверка заголовка пакета 0x69
    if (sizeof(dataesp)>=SerialEspInLen) for (int i=0; i<SerialEspInLen; i++) ((byte*)&dataesp)[i] = SerialEspIn[i]; //загрузка структуры данных
   }
   SerialEspInLen=0;
  }

  digitalWrite( RELAY_IN1, !bitRead( dataesp.Relay, 0 ));
  digitalWrite( RELAY_IN2, !bitRead( dataesp.Relay, 1 ));
  digitalWrite( RELAY_IN3, !bitRead( dataesp.Relay, 2 ));
  digitalWrite( RELAY_IN4, !bitRead( dataesp.Relay, 3 ));

  if (millis() - ms_compass > T_MS_COMPASS) {
    digitalWrite(LED, HIGH);
    compass.read(&compass_x,&compass_y,&compass_z,&compass_hdg);
    /*
    compass_hdg_arr[compass_arr_cnt] = compass_hdg;
    compass_arr_cnt++;
    if (compass_arr_cnt => COMPASS_COUNT) compass_arr_cnt=0;
    compass_hdg_avg=0;
    for (int i=0; i<COMPASS_COUNT; i++) compass_hdg_avg+=compass_hdg_arr[i];
    compass_hdg_avg = compass_hdg_avg/COMPASS_COUNT;
    */
    compass_hdg_avg = compass_hdg;
    datanano.CompassCourse = (uint16_t)compass_hdg_avg;
    delay(50);
    ms_compass = millis();
    digitalWrite(LED, LOW);
  } //ms_compass
  
  wdt_reset();

} //loop
