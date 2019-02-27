/* v2.02

Прием I2C данных компаса и данных GPS и передача по MODBUS RTU
с фильтрацией компаса

TX, RX - связь с esp32
D10, D11 (RX,TX) - подключение GPS
A4, A5 - компас
LED - прием пакетов с GPS

Таблица регистров:

00- compass course.deg()
01- (int)

02- gps.course.deg();
03- (int)

04- gps.speed.kmph()
05- (int)

06- gps.speed.knots()
07- (double)
08- 
09-

10- gps.location.lat()
11- (double)
12- 
13- 

14- gps.location.lng()
15-
16-
17-

18- gps.altitude.meters()
19- (int)

20- gps.time.hour()
21-

22- gps.time.minute()
23-

24- gps.time.second()
25-

26- gps.date.day()
27-

28- gps.date.month()
29-

30- gps.date.year()
31-

*/
#include <SoftwareSerial.h>
#include "HMC5883.h"
#include "TinyGPSplus.h"
#include "ModbusRtu.h"

#define BAUD            9600  //Скорость последовательного порта
#define MBADDR          1     //MODBUS Адрес (0-127)
#define TXEN            6     //RS485 RE-DE
#define LED             13    //Светоиод на Arduino

#define COMPASS_COUNT   5     //Количество измерений для усреднения
#define COMPASS_FILTER  10    //Максимальное отклонение для отброса показаний

HMC5883 compass;
SoftwareSerial SSerialGPS(10, 11); // RX, TX
TinyGPSPlus gps_parser;

//для поиска тега раскомментировать тут и в программе область заполнения таблицы регистров
//тогда значения в тегах будут 1,2,3,4 и таким образом можно определить куда "упал" какой канал
//uint16_t mbdata[32] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, ,23 ,24, 25, 26, 27, 28, 29, 30, 31, 32 };
uint16_t mbdata[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
Modbus slave(MBADDR, 0, TXEN);
int8_t state = 0;   //Статус MB

int compass_x,compass_y,compass_z;
float compass_hdg;
float compass_hdg_avg; //среднее
float compass_hdg_arr[COMPASS_COUNT];
int compass_arr_cnt=0;

long ms_compass;
long ms_gps;

void setup() {
  pinMode(LED, OUTPUT);
  //Подсвечиваем запуск контроллера диодом
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);

//  Serial.begin(9600);
  SSerialGPS.begin(9600);
  
  slave.begin( BAUD );

  //Высвечиваем диодом MB адрес устройства
  for (int i=0; i<MBADDR; i++){
    digitalWrite(LED, HIGH); delay(200);
    digitalWrite(LED, LOW); delay(200);
  }
  delay(500);

  compass.init();

  ms_compass = ms_gps = millis();

  compass.read(&compass_x,&compass_y,&compass_z,&compass_hdg);
  for (int i=0; i<COMPASS_COUNT; i++) compass_hdg_arr[i]=compass_hdg;
  
}

void loop() {

  state = slave.poll( mbdata, 32 ); //Обработчик MODBUS

  

  if (millis() - ms_compass > 50) {
    compass.read(&compass_x,&compass_y,&compass_z,&compass_hdg);
    //if (abs(compass_hdg-compass_hdg_avg) <= COMPASS_FILTER) {
      compass_hdg_arr[compass_arr_cnt] = compass_hdg;
      compass_arr_cnt++;
    //}
    if (compass_arr_cnt > COMPASS_COUNT) compass_arr_cnt=0;
    
    compass_hdg_avg=0;
    for (int i=0; i<COMPASS_COUNT; i++) compass_hdg_avg+=compass_hdg_arr[i];
    compass_hdg_avg = compass_hdg_avg/COMPASS_COUNT;

    //Serial.print(compass_hdg);
    //Serial.print(" ");
    //Serial.println(compass_hdg_avg);
    
    ms_compass = millis();
  } //ms_compass


  while (SSerialGPS.available()) {
    digitalWrite(LED, HIGH);
    char temp = SSerialGPS.read();
    gps_parser.encode(temp);
    //Serial.write(SSerialGPS.read());
    digitalWrite(LED, LOW);
  }

  if(gps_parser.time.isValid()) {
    union {
      double u_double;
      uint16_t u_ints[2];
    };
    mbdata[0] = (uint16_t)compass_hdg_avg;
    mbdata[1] = (uint16_t)gps_parser.course.deg();
    mbdata[2] = (uint16_t)gps_parser.speed.kmph();
    
    u_double = gps_parser.speed.knots();
    mbdata[3] = u_ints[0];
    mbdata[4] = u_ints[1];

    u_double = gps_parser.location.lat();
    mbdata[5] = u_ints[0];
    mbdata[6] = u_ints[1];

    u_double = gps_parser.location.lng();
    mbdata[7] = u_ints[0];
    mbdata[8] = u_ints[1];
        
    mbdata[9] = (uint16_t)gps_parser.altitude.meters();

    mbdata[10] = (uint16_t)gps_parser.time.hour();
    mbdata[11] = (uint16_t)gps_parser.time.minute();
    mbdata[12] = (uint16_t)gps_parser.time.second();
    mbdata[13] = (uint16_t)gps_parser.date.day();
    mbdata[14] = (uint16_t)gps_parser.date.month();
    mbdata[15] = (uint16_t)gps_parser.date.year();
   
  }


} //loop
