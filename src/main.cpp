#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>
#include "BluetoothA2DPSink.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


#define SERVICE_UUID        "f45a050b-7483-4bc9-8fae-49f9c9d83279"
#define CHARACTERISTIC_UUID "e89177bf-c2dd-4c1f-89d5-a224beddf67a"

BluetoothA2DPSink a2dp_sink;
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

std::string blVal = "b";

// when mixking colors with red we need different rows to take different voltage

// cube specs
#define ROWS 8
#define COLUMNS 64
#define FIRST_ROW 0x8000
#define ZEROS 0x0000000000000000

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
/*****/
//pins
int latchPin = 27;
int clockPin = 26;
int dataPin = 25;

uint64_t leds = 0xFFFFFFFFFFFFFFFF;
uint16_t anodes = 0x8000;
bool latchPinState = 0;

typedef enum {RED = 1, GREEN, BLUE, TEAL, PINK} color_state_t;
color_state_t current_color = RED;

void updateColorState(void){
  switch(current_color){
    case RED:
      current_color = GREEN;
      break;
    case GREEN:
      current_color = BLUE;
      break;
    case BLUE:
      current_color = TEAL;
      break;
    case TEAL:
      current_color = RED;
      break;
    case PINK:
      current_color = RED;
  }
}

void shiftOutAnodes(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint16_t val)
{
      val &= 0xFF00; //mask out the bottom 8 bits(don't need them for 8x8x8 LEB matrix
      
      uint8_t i;

      for (i = 0; i < 16; i++)  {
            if (bitOrder == LSBFIRST)
                  digitalWrite(dataPin, !!(val & (1 << i)));
            else      
                  digitalWrite(dataPin, !!(val & (1 << (15 - i))));
                  
            digitalWrite(clockPin, HIGH);
            digitalWrite(clockPin, LOW);            
      }
} 

void _shiftOut64(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint64_t val)
{
      for (int i = 0; i < 64; i++)  {
            if (bitOrder == LSBFIRST) digitalWrite(dataPin, !!(val & ((uint64_t)1 << i)));
            else digitalWrite(dataPin, !!(val & ((uint64_t)1 << (63 - i))));
            digitalWrite(clockPin, HIGH);
            digitalWrite(clockPin, LOW);   
      }
}
void shiftOut64(uint64_t val) { _shiftOut64(dataPin, clockPin, LSBFIRST, val);}

void shiftOut8_Monochrome(int row, int64_t val, color_state_t color) {
  digitalWrite(latchPin, LOW);
   shiftOutAnodes(dataPin, clockPin, LSBFIRST, row);
   
   switch(color){
    case RED:
      shiftOut64(ZEROS);
      shiftOut64(ZEROS);
      shiftOut64(val);
      
      break;
    case GREEN:
      shiftOut64(ZEROS);
      shiftOut64(val);
      shiftOut64(ZEROS);
      break;
    case BLUE:
      shiftOut64(val);
      shiftOut64(ZEROS);
      shiftOut64(ZEROS);
      break;
      case TEAL:
      shiftOut64(val);
      shiftOut64(val);
      shiftOut64(ZEROS);
      break;
      case PINK:
      shiftOut64(val);
      shiftOut64(ZEROS);
      shiftOut64(val);
  }
   digitalWrite(latchPin, HIGH);
}
/*FFT*/
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const uint16_t FFTsamples = 8; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 4;
const double samplingFrequency = 8;
const uint8_t amplitude = 175;

double vReal[FFTsamples];
double vImag[FFTsamples];

byte peak[] = {0,0,0,0,0,0,0,0};
int16_t sample_l_int;
int16_t sample_r_int;
/*****/

/******Matrix Control******/
// void displayBand(int band, int dsize){
//   int dmax = ROWS;
//   int columns = band + 0x0001;
//   printf("%x, %x", dmax, columns);
// //   dsize /= amplitude;
//   if (dsize > dmax) dsize = dmax;
//   // for (int s = 0; s <= dsize; s=s+2){display.drawFastHLine(1+10*band,64-s, 8, WHITE);}
//   // if (dsize > peak[band]) {peak[band] = dsize;}
// }
/************************/
#define CHARBIT 8
int abs(int n) {
  int const mask = n >> (sizeof(int) * CHARBIT - 1);
  return ((n + mask) ^ mask);
}

void read_data_stream(const uint8_t *data, uint32_t length)
{
  //TODO: add a hearing variable here to pause animation on pause of phone
  int16_t *samples = (int16_t*) data;
  uint32_t sample_count = length/2;
  // Do something with the data packet
  // Serial.println("hearing data");
  // Serial.println(data[0]);
 int byteOffset = 0;
    for (int i = 0; i < FFTsamples; i++) {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
      vImag[i] = 0;
      byteOffset = byteOffset + 4;
    }
    
}

void LEBluetooth() {
    BLEDevice::init("ESP32-BLE-Server");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  
  /* BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );*/

  pCharacteristic->setValue("Hello, World!");
  pService->start();
  //BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
    i2s_pin_config_t my_pin_config = {
        .bck_io_num = 18,
        .ws_io_num = 23,
        .data_out_num = 22,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    a2dp_sink.set_pin_config(my_pin_config);
    
  LEBluetooth();
  a2dp_sink.start("TheMatrix");
  a2dp_sink.set_stream_reader(read_data_stream);

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
}

int64_t getColHex(int col) {
  return 0xFF << (col * 8);
}

int getRowHex(int row) {
  return FIRST_ROW >> (row-1);
}

int includeRowsBelow(int row){
  return (short)0x8000 >> (row - 1);
}

void animateFFT(int col, int dsize) {
  int row = 1;
  if (dsize == 0) return;
  dsize /= amplitude;
  dsize = abs(dsize);
  if (dsize < 10) row =1;
  else if (dsize < 100) row = 2;
  else if (dsize < 500) row = 3;
  else if (dsize < 1000) row = 4;
  else if (dsize < 10000) row = 5;
  else if (dsize < 100000) row = 6;
  else if (dsize < 1000000) row = 7;
  else row = 8;
  color_state_t color = BLUE;

  
  if (row >= ROWS) { row = ROWS; }
  int rowHex = includeRowsBelow(row);
  int64_t word = getColHex(col);
  color = PINK;
  shiftOut8_Monochrome(rowHex, 0xC3C300000000C3C3, color);
  delay(25);
}

int64_t getOneColHex(int col) {
  return (int64_t)0x1 << col;
}



void shiftOne(int row, int col, color_state_t color) {
  int rowHex = getRowHex(row);
  int64_t colHex = getOneColHex(col);
  
  shiftOut8_Monochrome(rowHex, colHex, color);
}

void shiftOneExample() {
   for (int i=1; i<9; i++) {
    for (int n=1; n<64; n++) {
      shiftOne(i, n, (color_state_t)(n%4+1));
      delay(20);
    }
  }
}

void shiftCornersExample() {
 shiftOne(1, 63, BLUE);
  shiftOne(1, 56, BLUE);
  shiftOne(1,0, BLUE);
  shiftOne(1,7, BLUE);
}

void shiftOneColumn(int col, color_state_t color, int rows = 0xFF00) {
  int64_t colHex = getOneColHex(col);
  shiftOut8_Monochrome(rows, colHex, color);
}

void shiftOneColumnExample() {
  shiftOneColumn(0, RED);
  shiftOneColumn(7, RED);
  shiftOneColumn(63, RED);
  shiftOneColumn(56, RED);
}

void box(color_state_t color) {
  shiftOut8_Monochrome(0xFF00, 0xFF818181818181FF, color);
}

void runFFT() {
  FFT.Windowing(vReal, FFTsamples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, FFTsamples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, FFTsamples);
  // display.clearDisplay();
  for(int i = 0; i < (FFTsamples); i++)
  {
    animateFFT(i+1, (int)vReal[i]);
  }
}

void loop() {
  blVal = pCharacteristic->getValue();
  Serial.println(blVal.c_str());

  if (blVal == "1") {
    box(BLUE);
    Serial.println("Box animation");
  } else if (blVal == "2") {
    shiftCornersExample();
    Serial.println("shift corners animation");
  } else {
    runFFT();
  }
}
