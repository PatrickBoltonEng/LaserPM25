/*
 * Project LaserPM25
 * Description:
 * Author:
 * Date:
 */

#include "Particle.h"
#include "Seeed_HM330X.h"

SYSTEM_THREAD(ENABLED);

#define SERIAL_OUTPUT Serial

#define UPDATE_INTERVAL 15000  //1 sec = 1000 millis

SerialLogHandler logHandler(LOG_LEVEL_INFO);

int min_time, min_last; 
int PM10S, PM25S, PM100S, PM10E, PM25E, PM100E, PC03, PC05, PC10, PC25, PC50, PC100;
unsigned long UpdateInterval;

HM330X sensor;
uint8_t buf[30];
uint16_t buf2[30];

const char* str[] = {"Sensor#: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                     "Particle Count >0.3um/L: ", "Particle Count >0.5um/L: ", "Particle Count >1.0um/L: ",
                     "Particle Count >2.5um/L: ", "Particle Count >5.0um/L: ", "Particle Count >10 um/L: ",
                    };

uint16_t val[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

void setup() 
{
    Serial.begin(9600);
    delay(100);

    Log.info("Serial start");

    if (sensor.init()) {
        Log.info("HM330X init failed!!!");
        while (1);
    }
    UpdateInterval=millis();
    min_last=Time.minute()-1;
}


void loop() 
{
  Time.zone(-7);

  if(millis() - UpdateInterval > UPDATE_INTERVAL)
  {
    if (sensor.read_sensor_value(buf, 29))
    {
       Log.info("HM330X read result failed!!!");
    }
    parse_result_value(buf);
    parse_result(buf);


     min_time=Time.minute();
     if((min_time!=min_last)&&(min_time==0||min_time==15||min_time==30||min_time==45))
     {
        min_last = min_time;
        Log.info("Last Update: %d", min_last);
     }
     UpdateInterval = millis();
    }
}

HM330XErrorCode print_result(const char* str, uint16_t value) 
{
    if (NULL == str) {
        return ERROR_PARAM;
    }
    Log.info(str);
    Log.info(String(value));
    return NO_ERR;
}

HM330XErrorCode parse_result(uint8_t* data) 
{
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 1; i < 14; i++) 
    {
       value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
       val[i-1] = value;
       print_result(str[i - 1], val[i-1]);  
    }
    PM10S     =   val[1];
    PM25S     =   val[2];
    PM100S    =   val[3];
    PM10E     =   val[4];
    PM25E     =   val[5];
    PM100E    =   val[6];
    PC03       =   val[7];
    PC05       =   val[8];
    PC10       =   val[9];
    PC25       =   val[10];
    PC50       =   val[11];
    PC100      =   val[12];

    Log.info("PM10S: %d", PM10S);
    Log.info("PM25S: %d", PM25S);
    Log.info("PM100S: %d", PM100S);
    Log.info("PM10E: %d", PM10E);
    Log.info("PM25E: %d", PM25E);
    Log.info("PM100E: %d", PM100E);
    Log.info("P03: %d", PC03);
    Log.info("P05: %d", PC05);
    Log.info("P10: %d", PC10);
    Log.info("P25: %d", PC25);
    Log.info("P50: %d", PC50);
    Log.info("P100: %d", PC100);
    return NO_ERR;
}

HM330XErrorCode parse_result_value(uint8_t* data) 
{
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 0; i < 28; i++) {
        Log.info(String(data[i], HEX));
        
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += data[i];
    }
    if (sum != data[28]) {
        Log.info("wrong checkSum!!!!");
    }
    return NO_ERR;
}