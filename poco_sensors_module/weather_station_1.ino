/**
* Autor: Juraj Dudak
* web: http://www.kiwiki.info/index.php/NSoric_Poco
*/

#include <VirtualWire.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <BH1750FVI.h>
#include "DHT.h"
#include <LowPower.h>
//------------- private constants-----------------
#define LED_PIN           13
#define TRANSMIT_PIN      9
/**
* 0xF0 => 1111 0000 in binary
* The meaning of bits:
*  bit     meaning
*  7       Temperature sensor present
*  6       Humidity sensor present
*  5       Pressure sensor present
*  4       Light sensor present
*  3 - 0   Reserved
*/
#define SENSOR_TYPE       0xF0
#define SENSOR_ADDRESS    0x2
#define BATERY_PIN        A0
#define DHTPIN            2
#define DHTTYPE           DHT22
/**
* Real altitude.
* Set your own value!
*/
#define ALTITUDE          284.0
// 75 => 75*8s = 10min, 1 sleep slot is 8s
#define SLEEP_TIME          75
//#define DEBUG


//------------- private variables-----------------
uint8_t     vw_frame[30];   // data packet
DHT         dht(DHTPIN, DHTTYPE);
BH1750FVI   LightSensor;
SFE_BMP180  pressure;

/**
* Low power mode
*/
void sleep(int sleep8time)
{
    for (int i = 0; i < sleep8time; i++)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
}

/** Chybova nekonecna slucka */
void error()
{
    while (1)
    {
        digitalWrite(LED_PIN, 1);
        delay(100);
        digitalWrite(LED_PIN, 0);
        delay(100);
    }

}

void setup(void)
{

#ifdef DEBUG
    Serial.begin(19200);
    Serial.println("Start Radio::sensor");
#endif


    vw_set_tx_pin(TRANSMIT_PIN);
    vw_set_ptt_inverted(false);
    vw_setup(1000);

    LightSensor.begin();
    LightSensor.SetAddress(Device_Address_L); //Address 0x23
    LightSensor.SetMode(Continuous_H_resolution_Mode);

    pressure.begin();

    dht.begin();
    // prerare the data frame: status byte and address
    vw_frame[0] = SENSOR_TYPE;
    vw_frame[1] = (SENSOR_ADDRESS >> 8) & 0xFF;
    vw_frame[2] = (SENSOR_ADDRESS) & 0xFF;

#ifdef DEBUG
    Serial.println("station init done.");
#endif

}

void loop(void)
{
    // Read temperature as Celsius
    float t = dht.readTemperature();
    uint16_t tc = t;            // integer part of temperature
    float t_des = t - tc;       // decimal part
    uint8_t tdes = t_des * 10;  // first digit in decimal part

    // encoding fotmat for temperature:
    // T_high byte (th) = sign,2^9,2^8,2^7,2^6,2^5,2^4,2^3
    // T_low byte (tl)  = 2^2,2^1,2^0,2^(-1),2^(-2),2^(-3),2^(-4),Reserved
    uint8_t th = ((tc & 0b01111111) >> 3) | (tc & 0b10000000);
    uint8_t tl = ((tc & 0b00000111) << 5) ;
    addFloatPart(tdes, tl);


    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    uint8_t hc = h;             // integer part
    hc <<= 1;                   // 1 bit shift left
    float h_des = h - hc;       // decimal part
    uint8_t hdes = t_des * 10;  //first digit in decimal part
    if (hdes > 4)               // the LSB bit is 2^(-1), therefore resolution is 0.5
    {
        hc |= 0B1;
    }

    uint16_t p = getPressure();
    uint16_t l = LightSensor.GetLightIntensity();
    uint16_t b = analogRead(BATERY_PIN);

    // data frame
    vw_frame[3] = b >> 3;       // battery
    vw_frame[4] = th;           // temperature first byte
    vw_frame[5] = tl;           // temperature second byte
    vw_frame[6] = hc;           // humidity
    vw_frame[7] = p >> 8;       // pressure first byte
    vw_frame[8] = p & 0xFF;     // pressure second byte
    vw_frame[9] = l >> 8;       // light first byte
    vw_frame[10] = l & 0xFF;    // light second byte
    vw_frame[11] = CRC8(vw_frame, 11);  // CRC8 checksum

    digitalWrite(LED_PIN, HIGH);        // switch on LED while transmitting
    vw_send((uint8_t *)vw_frame, 12);   // send data packet
    vw_wait_tx();                       // Wait until the whole message is gone
    digitalWrite(LED_PIN, LOW);         // switch off LED


#ifdef DEBUG
    Serial.println("Data frame: ");
    for (int i = 0; i < 10; i++)
    {
        Serial.print(vw_frame[i]);
        Serial.print(" ");
    }
    Serial.println();
    Serial.print("battery\t\t: ");
    Serial.print(b / 1024.0 * 5.0);
    Serial.println(" V");
    Serial.print("Temperature\t: ");
    Serial.print(t);
    Serial.println(" C");
    Serial.print("Humidity\t: ");
    Serial.print(h);
    Serial.println(" per100");
    Serial.print("Light\t\t: ");
    Serial.print(l);
    Serial.println(" lux");
    Serial.print("Pressure\t: ");
    Serial.print(p);
    Serial.println(" hPa");
    Serial.println();
    delay(2000);    // 2 seconds to success of USART transmission
#endif


    sleep(SLEEP_TIME);      // Low-power state
    delay(2000);            // 2 secs to waku up (for DHT sensor)
}


/**
* Encode the  decimal part of temerature.
* The result from DHT sensor is coded in BCD.
* It is necessary (for this communication protocol) encode to binary form.
* @param tdes Decimal part of temperature
* @param tl second byte of encoded temperature
* @return None
*/
void addFloatPart(int tdes, uint8_t &tl)
{
    switch (tdes)
    {
    case 1:
        tl |= 0b00100;
        break;
    case 2:
        tl |= 0b00110;
        break;
    case 3:
        tl |= 0b01010;
        break;
    case 4:
        tl |= 0b01110;
        break;
    case 5:
        tl |= 0b10000;
        break;
    case 6:
        tl |= 0b10100;
        break;
    case 7:
        tl |= 0b11000;
        break;
    case 8:
        tl |= 0b11010;
        break;
    case 9:
        tl |= 0b11110;
        break;
    }
}


/**
* Mesaure the atmosferic pressure
* @return atmosferic pressure
*/
float getPressure()
{
    char status;
    double T, P, p0=-1;
    status = pressure.startTemperature();
    if (status != 0)
    {
        // Wait for the measurement to complete:
        delay(status);
        status = pressure.getTemperature(T);
        if (status != 0)
        {
            // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
            // If request is successful, the number of ms to wait is returned.
            status = pressure.startPressure(3);
            if (status != 0)
            {
                // Wait for the measurement to complete:
                delay(status);
                // Retrieve the completed pressure measurement:
                status = pressure.getPressure(P, T);
                if (status != 0)
                {
                    // computed pressure related to real altitude
                    p0 = pressure.sealevel(P, ALTITUDE);
                }
            }
        }
    }
    return p0;

}


//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
uint8_t CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    while (len--)
    {
        uint8_t extract = *data++;
        for (uint8_t tempI = 8; tempI; tempI--)
        {
            uint8_t sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum)
            {
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
    }
    return crc;
}