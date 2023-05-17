#include <DFRobot_OzoneSensor.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define COLLECT_NUMBER   20              // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3
#define SERIAL_DEBUG
/*   iic slave Address, The default is ADDRESS_3
       ADDRESS_0               0x70      // iic device address
       ADDRESS_1               0x71
       ADDRESS_2               0x72
       ADDRESS_3               0x73
*/
sd.begin();
fo = SD.open("adatok.csv",FILE_APPEND);
SD.close(fo);

DFRobot_OzoneSensor Ozone;

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

//DHT_Unified dht(DHTPIN, DHTTYPE);
void setup_CH4_CO_Ozone();
void setup_CO2_sensor();
void setup_bme();
void read_CH4_CO_Ozone();
void read_CO2_sensor();
void readBME();


///CH4, CO, Ozone Gas Sensors
void setup(){
    Serial.begin(9600); //Set serial baud rate to 9600 bps
    Serial.println("CH4;CO;O3;CO2;C;hum%;hPa");
    fo.write("CH4;CO;O3;CO2;C;hum%;hPa\n");
    setup_CH4_CO_Ozone();
    setup_CO2_sensor();
    setup_bme();
}

void loop(){
    fo = SD.open("adatok.csv",FILE_APPEND);
    read_CH4_CO_Ozone();
    read_CO2_sensor();
    readBME();
    Serial.println();
    fo.write("\n");
    SD.close(fo);
    delay(1000);
}

void setup_CH4_CO_Ozone()
{
    while(!Ozone.begin(Ozone_IICAddress)) {
        Serial.println("I2c device number error !");
        delay(1000);
    }  /* Serial.println("I2c connect success !");
       Set iic mode, active mode or passive mode
       MEASURE_MODE_AUTOMATIC            // active  mode
       MEASURE_MODE_PASSIVE              // passive mode
       */
    Ozone.setModes(MEASURE_MODE_PASSIVE);
}

void setup_bme() {
    if (!bme.begin(0x76,&Wire)) {
    #ifdef SERIAL_DEBUG
    Serial.println("nincs BMP");
    fo.write("nincs BMP\n");
    #endif
  }
}

void read_CH4_CO_Ozone()
{
    int val0;
    val0=analogRead(0);//Read Methane value from analog 0
    /*Serial.print("Methane concentration is ");
  Serial.print(val0,DEC);//Print the value to serial port
  Serial.println(" PPM.");*/
    Serial.print(val0,DEC);
    Serial.print(";");

    fo.print(val0,DEC);
    fo.print(";");

    int val1;
    val1=analogRead(1);//Read CO value from analog 1
    //Serial.print("CO concentration is ");
    Serial.print(val1,DEC);//Print value to serial port
    Serial.print(";");

    fo.print(val1,DEC);
    fo.print(";");
    //Serial.println(" PPM.");
    /*   Smooth data collection
       COLLECT_NUMBER                    // The collection range is 1-100
*/
    int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
    //Serial.print("Ozone concentration is ");
    Serial.print(ozoneConcentration);
    Serial.print(";");

    fo.print(ozoneConcentration);
    fo.print(";");
    //Serial.println(" PPB.");
    //Serial.println("");
    //delay(5000);
}

int sensorIn = A2;

void setup_CO2_sensor(){
    Serial.begin(9600);
    // Set the default voltage of the reference voltage
    analogReference(DEFAULT);
}
void read_CO2_sensor(){
    //Read voltage
    int sensorValue = analogRead(sensorIn);
    // The analog signal is converted to a voltage
    float voltage = sensorValue*(5000/1024.0);
    if(voltage == 0)
    {
        Serial.println("Fault");
        fo.println("CO2 hiba");
    }
    else if(voltage < 400)
    {
        Serial.println("preheating");
        fo.println("preheating");
    }
    else
    {
        int voltage_diference=voltage-400;
        float concentration=voltage_diference*50.0/16.0;
        // Print Voltage
        //Serial.print("The measured voltage is: ");
        //Serial.print(voltage);
        //Serial.println(" mV.");
        //Print CO2 concentration
        //Serial.print("CO2 concentration is ");
        Serial.print(concentration);
        Serial.print(";");

        fo.print(concentration);
        fo.print(";");
        //Serial.println(" PPM");
        //Serial.println("");
    }
    //delay(5000);
}

/*void readtemp() {
    sensors_event_t event;
    //dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println(F("Error reading temperature!"));
    }
    else {
        //Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.print(";");
        //Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    //dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
    }
    else {
        //Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.print("\n");
        //Serial.println(F("%"));
    }
}*/

void readBME() {
    String toReturn = "";

    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    toReturn += temp_event.temperature;
    toReturn += ";";

    toReturn += humidity_event.relative_humidity;
    toReturn += ";";

    toReturn += pressure_event.pressure;
    toReturn += ";";

    Serial.print(toReturn);
    fo.print(toReturn);
}
