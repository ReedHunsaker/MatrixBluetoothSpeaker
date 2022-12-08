
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>
#include "BluetoothA2DPSink.h"
#include <HardwareSerial.h>
// #ifdef CORE_DEBUG_LEVEL
// #undef CORE_DEBUG_LEVEL
// #endif

// #define CORE_DEBUG_LEVEL 5
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

BluetoothA2DPSink a2dp_sink;

char cVal = '3';

// when mixking colors with red we need different rows to take different voltage

// cube specs
#define ROWS 8
#define COLUMNS 64
#define FIRST_ROW 0x8000
#define ZEROS 0x0000000000000000
#define ALL_COLUMNS 0xFFFFFFFFFFFFFFFF
#define TOP_BIT_MASK 0x8000000000000000
#define CHARBIT 8
#define RXp2 16
#define TXp2 17

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
/*****/
//pins
int latchPin = 27;
int clockPin = 26; //26
int dataPin = 25; // 24
int dimmingPin = 33;

uint64_t leds = 0xFFFFFFFFFFFFFFFF;
uint16_t anodes = 0x8000;
bool latchPinState = 0;
void (*animation)();

typedef enum {RED = 1, GREEN, BLUE, TEAL, PINK, YELLOW} color_state_t;
color_state_t current_color = RED;

/*FFT*/
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const uint16_t FFTsamples = 1024; //This value MUST ALWAYS be a power of 2 1024
const double samplingFrequency = 8;
const uint8_t amplitude = 180;

double vReal[FFTsamples];
double vImag[FFTsamples];

byte peak[] = {0,0,0,0,0,0,0,0};
int16_t sample_l_int;
int16_t sample_r_int;
/*****/

//debug
__attribute__ ((__noinline__))
void * get_pc () { return __builtin_return_address(0); }

void dimAnimation(int8_t val = 255) {
  for(int i=0; i<val; i++){
    analogWrite(dimmingPin, i);
    delay(1);
  }
  for(int i=val; i>0; i--){
    analogWrite(dimmingPin, i);
    delay(1);
  }
}

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
      current_color = PINK;
      break;
    case PINK:
      current_color = YELLOW;
      break;
    case YELLOW:
      current_color = RED;
      break;
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

/// @brief Shift out one instance of 
/// @param rows Hex of rows ex 0xFF00 is all rows
/// @param word1 64 bit hex word for blue values
/// @param word2 64 bit hex word for green values
/// @param word3 64 bit hex word for red values
void shiftOut(int rows, int64_t word1, int64_t word2, int64_t word3) {
  digitalWrite(latchPin, LOW);
  shiftOutAnodes(dataPin, clockPin, LSBFIRST, rows);
  shiftOut64(word1);
  shiftOut64(word2);
  shiftOut64(word3);
  digitalWrite(latchPin, HIGH);
}

/// @brief shifts out a row of one color
/// @param row rowHex
/// @param val 64bit value
/// @param color color of row
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
      case YELLOW:
      shiftOut64(val);
      shiftOut64(val);
      shiftOut64(ZEROS);
  }
   digitalWrite(latchPin, HIGH);
}

int abs(int n) {
  int const mask = n >> (sizeof(int) * CHARBIT - 1);
  return ((n + mask) ^ mask);
}

void read_data_stream(const uint8_t *data, uint32_t length)
{
  int16_t *samples = (int16_t*) data;
  uint32_t sample_count = length/2;
 int byteOffset = 0;
    for (int i = 0; i < FFTsamples; i++) {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
      vImag[i] = 0;
      byteOffset = byteOffset + 4;
    }
    
}

void initA2DPBluetooth() {
  static const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = 44100, // corrected by info from bluetooth
        .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
    };
  i2s_pin_config_t my_pin_config = {
        .bck_io_num = 19,
        .ws_io_num = 22,
        .data_out_num = 21,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    a2dp_sink.set_pin_config(my_pin_config);
    a2dp_sink.set_i2s_config(i2s_config);
    a2dp_sink.start("TheMatrix");
    a2dp_sink.set_stream_reader(read_data_stream);
}

void box() {
  // shiftOut8_Monochrome(0xFF00, 0xFF818181818181FF, current_color);
  shiftOut(0xFF00, 0xFF818181818181FF, 0x007E626262627E00, 0x00003C3C3C3C0000);
}
//FreeRTOS function that runs the current animation
void currentAnimation( void * parameter) {
  while (true)
  {
    // Serial.println("Animation Task running");
    // Serial.printf("%#x\n", get_pc());
    animation();
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  // put your setup code here, to run once:
  initA2DPBluetooth();
    // set starting animation
  animation = &box;
  
  

  xTaskCreate(currentAnimation, "Current_Anmiation", 50000, NULL, 0, NULL);

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  pinMode(dimmingPin, OUTPUT);
}

int64_t getColHex(int col) { return 0xFF << (col * 8); }

int getRowHex(int row) { return FIRST_ROW >> (row-1); }

int includeRowsBelow(int row){ return (short)0x8000 >> (row - 1); }

int64_t corners = 0xC3C300000000C3C3;
bool cornersActive = 0;

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
  if (cornersActive) word = corners;
  color = PINK;
  shiftOut8_Monochrome(rowHex, word, color);
}

int64_t getOneColHex(int col) { return (int64_t)0x1 << col; }



void shiftOne(int row, int col, color_state_t color) {
  int rowHex = getRowHex(row);
  int64_t colHex = getOneColHex(col);
  
  shiftOut8_Monochrome(rowHex, colHex, color);
}

void shiftOneExample() {
   for (int i=1; i<9; i++) {
    for (int n=1; n<64; n++) {
      shiftOne(i, n, (color_state_t)(n%4+1));
      delay(50);
    }
  }
}

void shiftAllOut() {
  for (int i = 0; i <64; i ++) {
    shiftOut8_Monochrome(0xFF00, ALL_COLUMNS << i, current_color); //bottom two 0x1100
    // delay(20);
    // shiftOut8_Monochrome(0x3000, ALL_COLUMNS << i+2, current_color); //middle two 0x0011
    // delay(20);
    // shiftOut8_Monochrome(0x0C00, ALL_COLUMNS << i+3, current_color);// middle top two 0x1100
    // delay(20);
    // shiftOut8_Monochrome(0x0300, ALL_COLUMNS << i +4, current_color);// top two 0x0011
    // delay(20);
    delay(50);
  }
}



void shiftCornersExample() {
  int64_t four_columns = 0x8100000000000018;
  for (int i =0; i < 64; i++) {
    if (TOP_BIT_MASK & four_columns) four_columns += 1;
    four_columns << i;
    shiftOut8_Monochrome(0xFF00, four_columns, current_color);
  }
//  shiftOne(1, 63, BLUE);
//   shiftOne(1, 56, BLUE);
//   shiftOne(1,0, BLUE);
//   shiftOne(1,7, BLUE);
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



void runFFT() {
  FFT.Windowing(vReal, FFTsamples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, FFTsamples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, FFTsamples);
  int col = 1;
  // display.clearDisplay();
  for(int i = 0; i < (FFTsamples); i++)
  {
    col = (i % 8) + 1;
    animateFFT(col, (int)vReal[i]);
  }
}

void _shiftAndDimAllCols(int rowHex, color_state_t color) {
  shiftOut8_Monochrome(rowHex, ALL_COLUMNS, color);
}

void colorGradiant() {
  color_state_t colors[8] = { 
    BLUE, BLUE, TEAL, TEAL, GREEN, GREEN, RED, PINK
    };
  for (int i =0; i<8; i++) { _shiftAndDimAllCols(0x0100 << i, colors[i]); }
  dimAnimation();
}

void checkColor(char cVal) {
  if (cVal == 'R') current_color = RED;
  else if (cVal == 'G') current_color = GREEN;
  else if (cVal == 'B') current_color = BLUE;
  else if (cVal == 'T') current_color = TEAL;
  else if (cVal =='P') current_color = PINK;
  else if (cVal = 'Y') current_color = YELLOW;
}



void checkAnimation(int8_t val) {
  switch (val) {
    case 0x31:
    // Serial.println("BOX set");
    animation = &box;
    break;
    case 0x32:
    // Serial.println("Corners set");
    animation = &shiftOneExample;
    break;
    case 0x33:
    // Serial.println("FFT set");
    animation = &runFFT;
    break;
    case 0x34:
    animation = &colorGradiant;
    break;
    case 0x35:
    cornersActive = (cornersActive) ? 0 : 1;
    break;
    case 0x36:
    animation = &shiftAllOut;
    break;
  }
}



void loop() {
  // blVal = pCharacteristic->getValue();
  cVal = Serial2.readStringUntil('\n')[0];
  // Serial.printf("char %c\n", cVal);
  int8_t val = cVal;
  // Serial.println(blVal.c_str());
  // Serial.printf("hex: %x\n", val);
  
  //if the hex part of the alphabet then cheange color
  if (val > 0x40) checkColor(cVal);
  else checkAnimation(val);

  
}
