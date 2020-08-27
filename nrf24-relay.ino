

// "Wire" library
#include <Wire.h>

// "RF24" by TMRh20
//   https://tmrh20.github.io/RF24/
#include <printf.h>
#include <RF24.h>

// "Adafruit BME280 library"
#include <Adafruit_BME280.h>

// "Adafruit SSD1306"
#include <Adafruit_SSD1306.h>

// "Adafruit GFX Library"
#include <Adafruit_GFX.h>

#define DEBUG_PRINT
#include "debug.h"

#define PIN_RELAY_1            4              // PIN for turning relay #1 on/off
#define PIN_RELAY_2            5              // PIN for turning relay #2 on/off

#define PIN_RF24_CSN           9              // CSN PIN for RF24 module.
#define PIN_RF24_CE           10              // CE PIN for RF24 module.

#define NRF24_CHANNEL         1               // 0 ... 125
#define NRF24_CRC_LENGTH      RF24_CRC_16     // RF24_CRC_DISABLED, RF24_CRC_8, RF24_CRC_16 for 16-bit
#define NRF24_DATA_RATE       RF24_250KBPS    // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
#define NRF24_PAYLOAD_SIZE    32              // Max. 32 bytes.
#define NRF24_PA_LEVEL        RF24_PA_LOW     // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX    
#define NRF24_RETRY_DELAY     5               // Delay bewteen retries, 1..15.  Multiples of 250µs.
#define NRF24_RETRY_COUNT     15              // Number of retries, 1..15.

byte tx_addr[6] = "6LD57";                    // Transmission address for readings.
byte rx_addr[6] = "20RLY";                    // Address for getting ON/OFF command.
byte payload[32];                             // Buffer for payload data.

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);        // NRF24L01 radio.

#define BME_ADDR              0x76            // I2C address for the BME280 sensor.
Adafruit_BME280 bme;                          // BME280 temperature, humidity, and pressure sensor. 

#define SCREEN_WIDTH 128                      // OLED display width, in pixels
#define SCREEN_HEIGHT 64                      // OLED display height, in pixels
#define OLED_RESET     -1                     // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_ADDR      0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define TX_DELAY_SECONDS      60
unsigned long tx_ms = 0;
unsigned long tx_count = 0;

#define ALIVE_DELAY_SECONDS    2
unsigned long alive_ms = 0;

const byte unsigned protocol = 0x05;

int failed_count = 0;

const char *str_on = "ON";
const char *str_off = "OFF";
const char *str_relay = "Relay #";
const char *str_temp = "Temperature";
const char *str_pressure = "Pressure";
const char *str_humidity = "Humidity";
const char *str_error = "Error";
const char *str_tx = "TX:";
const char *str_rx = "RX:";
const char *str_hpa = " hPa";
const char *str_one = "1";
const char *str_two = "2";
const char *str_percent = "%";
const char *str_hash = "#";

const char *str_pa[5] = {"MIN", "LOW", "HIGH", "MAX"};
const char *str_rate[3] = {"2Mb", "1Mb", "250K"};

void setup() {

  // Initialize debugging.
  debug_begin();
  debug_println(F("\n\nArduino relay, v1.0"));

  // Initialize relay PIN to OFF (LOW).
  debug_println(F("- RELAY PIN #1"));
  pinMode(PIN_RELAY_1, OUTPUT);
  digitalWrite(PIN_RELAY_1, LOW);

  // Initialize relay PIN to OFF (LOW).
  debug_println(F("- RELAY PIN #2"));
  pinMode(PIN_RELAY_2, OUTPUT);
  digitalWrite(PIN_RELAY_2, LOW);

  // Initialize NRF24L01 module.
  debug_println(F("- NRF24"));
  nrf24_setup();
  debug_nrf24(radio);

  // Initialize BME280.
  debug_println(F("- BME280"));
  unsigned bme_status;
  if (!bme.begin(BME_ADDR))
    debug_println(F("  Failed."));
  else
    debug_println(F("  OK."));
    
  // Initialize OLED
  debug_println(F("- OLED"));
  if(display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    debug_println(F("  OK."));
  else
    debug_println(F(" Failed."));
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.display();
  
  // Get milliseconds control transmission interval.
  tx_ms = millis();
  alive_ms = millis();

  // Send current status first thing ...
  send_status();

  // Start listening for commands ...
  radio.startListening();
}

void loop() {

  
  if (radio.available()) {
    
    // Data available.
    debug_println(F("\nRecv:"));

    // Read payload.
    radio.read(&payload, sizeof(payload));

    // Write payload for debugging.
    for (int i = 0; i < sizeof(payload); i++) {
     if (payload[i] <= 0x0F) debug_print('0');
     debug_print(payload[i], HEX); 
     debug_print(' ');
    }
    debug_println();
      
    if (payload[0] == 0x04) {
      // We have received a message with protocol id = 0x04.
      // debug_println("Message with protocol id 0x04 received.");
      
      update_relay(PIN_RELAY_1, payload[1]);
      update_relay(PIN_RELAY_2, payload[2]);
            
      // Status have changed. Send status.
      send_status();
    }
    else {
      // debug_println(F("Invalid payload received."));
    }
  }
  
  // Transmit status and sensor readings.
  if (millis() - tx_ms > (unsigned long)(TX_DELAY_SECONDS * 1000L)) {
    send_status();
    tx_ms = millis();
  }
  
  // Print debug information to check that Arduino loop is running.
  if (millis() - alive_ms > (ALIVE_DELAY_SECONDS * 1000L)) {
    debug_print('.');
    alive_ms = millis();  
  }
  
}

bool update_relay(int relay, byte value) {
  // debug_print("update_relay(): relay="); debug_print(relay);
  // debug_print(", value="); debug_println(value);
  if (relay != PIN_RELAY_1 && relay != PIN_RELAY_2) return false;
  
  if (value == 0) 
    digitalWrite(relay, LOW);
  else if (value == 1) 
    digitalWrite(relay, HIGH); 
  else if (value == 2) {
    if (digitalRead(relay) == LOW) 
      digitalWrite(relay, HIGH);
    else
      digitalWrite(relay, LOW);
  }
  else 
    return false;
      
  return true;  
}

void send_status() {
    byte unsigned relay_1, relay_2;
    float temperature, humidity, pressure;
    
    // Stop listening in order to be able to send.
    radio.stopListening();
    radio.flush_tx();
    
    debug_print(F("\nTransmit: "));
    debug_println(++tx_count);

    // Read values of sensors ...
    relay_1 = digitalRead(PIN_RELAY_1) == LOW ? 0 : 1;
    relay_2 = digitalRead(PIN_RELAY_2) == LOW ? 0 : 1;
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    
    // Pack payload.
    int offset = 0;
    memcpy(payload + offset, &protocol, sizeof(protocol)); offset += sizeof(protocol);
    memcpy(payload + offset, &tx_count, sizeof(tx_count)); offset += sizeof(tx_count);
    memcpy(payload + offset, &relay_1, sizeof(relay_1)); offset += sizeof(relay_1);
    memcpy(payload + offset, &relay_2, sizeof(relay_2)); offset += sizeof(relay_2);
    memcpy(payload + offset, &temperature, sizeof(temperature)); offset += sizeof(temperature);
    memcpy(payload + offset, &humidity, sizeof(humidity)); offset += sizeof(humidity);
    memcpy(payload + offset, &pressure, sizeof(pressure)); offset += sizeof(pressure);
    
    // Send payload specifying number of bytes and retries. 
    // The offset variable holds number of bytes.
    int rc = nrf24_send(payload, offset, 5);
    
    if (rc > -1) {
      debug_print(F("Retries=")); debug_println(rc);
      failed_count = 0;
    }
    else {
      
      debug_println(F("Failed."));
      failed_count++;
    }

    update_display(temperature, humidity, pressure, digitalRead(PIN_RELAY_1), digitalRead(PIN_RELAY_2));
    
    // Go back to listening.
    radio.startListening();   
}

void update_display(float t, float h, float p, int r1, int r2) {
  char buf[10];

  // Clear display.
  display.clearDisplay();  

  if (failed_count < 1) {
      // Temperature (label).
      display.setCursor(0, 0);  
      display.write(str_temp);
    
      // Temperature (value)
      display.setCursor(0 + 5, 20);
      display.setTextSize(2);
      dtostrf(t, 0, 1, buf);  
      display.write(buf);
      display.setTextSize(1);
      display.write(248);             // Degree.
    
      // Pressure (label)
      display.setCursor(0, 47);
      display.write(str_pressure);
    
      // Pressure (value)
      dtostrf(p, 0, 0, buf);  
      display.setCursor(0 + 5, 57);
      display.write(buf); display.write(str_hpa);
    
      // Humidity (label)
      display.setCursor(75, 47);
      display.write(str_humidity);
    
      // Humidity (value)
      dtostrf(h, 0, 0, buf);  
      display.setCursor(75 + 5, 57);
      display.write(buf); display.write(str_percent);
       
      // Relay #1 (label)
      display.setCursor(75, 0);
      display.write(str_relay); display.write(str_one);
      
      // Relay #1 (value)
      display.setCursor(75 + 5 , 10);
      r1 == HIGH ? display.write(str_on) : display.write(str_off);
    
      // Relay #2 (label)
      display.setCursor(72, 24);
      display.write(str_relay); display.write(str_two);
    
      // Relay #2 (value).
      display.setCursor(75 + 5, 34);
      r2 == HIGH ? display.write(str_on) : display.write(str_off);    
  }
  else {
      // Temperature (label).
      display.setCursor(0, 0);  
      display.write(str_temp);
    
      // Temperature (value)
      display.setCursor(0 + 5, 11);
      dtostrf(t, 0, 1, buf);  
      display.write(buf);
      display.write(248);             // Degree.
    
      // Pressure (label)
      display.setCursor(0, 24);
      display.write(str_pressure);
    
      // Pressure (value)
      dtostrf(p, 0, 0, buf);  
      display.setCursor(0 + 5, 35);
      display.write(buf); display.write(str_hpa);
    
      // Humidity (label)
      display.setCursor(0, 48);
      display.write(str_humidity);
    
      // Humidity (value)
      dtostrf(h, 0, 0, buf);  
      display.setCursor(0 + 5, 57);
      display.write(buf); display.write(str_percent);

      display.setCursor(75, 0); display.write(str_error);
      display.setCursor(75 + 5, 11); display.print(str_tx); display.write((char *)tx_addr);
      display.setCursor(75 + 5, 22); display.print(str_rx); display.write((char *)rx_addr);
      display.setCursor(75 + 5, 33); display.print(str_hash); display.print(NRF24_CHANNEL); display.print(","); display.print(NRF24_CRC_LENGTH);
      display.setCursor(75 + 5, 44); display.print(str_rate[NRF24_DATA_RATE]);
      display.setCursor(75 + 5, 55); display.print(str_pa[NRF24_PA_LEVEL]);       
  }
  
  display.display();
 
}

int nrf24_send(byte *buf, int bytes, int retries)
{
  int max_retries = retries;
  
  while (retries > 0)
  {
    delay((max_retries - retries) * 50);
    if (radio.write(payload, bytes))
      break;
    retries--;      
  }

  if (retries == 0)
    return -1;
  else
    return max_retries - retries;
}

void nrf24_setup()
{
  radio.begin();
  
  radio.setAutoAck(true);                 
  radio.enableDynamicPayloads();          
  radio.setPALevel(NRF24_PA_LEVEL);
  radio.setRetries(NRF24_RETRY_DELAY, NRF24_RETRY_COUNT);              
  
  radio.setDataRate(NRF24_DATA_RATE);          
  radio.setChannel(NRF24_CHANNEL);
  radio.setCRCLength(NRF24_CRC_LENGTH);
  radio.setPayloadSize(NRF24_PAYLOAD_SIZE);
  
  radio.openWritingPipe(tx_addr);  
  radio.openReadingPipe(1, rx_addr);
  radio.stopListening();                  
}
