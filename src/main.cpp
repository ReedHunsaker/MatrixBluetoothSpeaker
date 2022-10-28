#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>
#include "BluetoothA2DPSink.h"
#include <map>

BluetoothA2DPSink a2dp_sink;

/*OLED*/
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
/*****/

/*FFT*/
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const uint16_t FFTsamples = 1024; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
// rows of the matrix
const int rows = 4;
// 256 = 1024 (samples) / 4(rows)
const int amplitude = 256;

double vReal[FFTsamples];
double vImag[FFTsamples];

// the peak in a given column update for 4x4x4 or 8x8x8x
byte peak[] = {0,0,0,0,0,0,0,0};
int16_t sample_l_int;
int16_t sample_r_int;
/*****/

// pin mapping
// column to pin
std::map<int, int> pinMap = {
    {0, 18},
    {1, 19}
};

/******Matrix Control******/
// display music
// -param1 column (int): Numbered column -TODO: mapped to a pin
// -param2 dsize (int): FFT information (snapshot of frequency)
void displayBand(int column, int dsize){
  dsize /= amplitude;
  if (dsize > rows) dsize = rows;
//   in loop ight up row for column and row 
  for (int s = 0; s <= dsize; s=s+2){display.drawFastHLine(1+10*column,64-s, 8, WHITE);}
  if (dsize > peak[column]) {peak[column] = dsize;}
}
/************************/

void read_data_stream(const uint8_t *data, uint32_t length)
{
  int16_t *samples = (int16_t*) data;
  uint32_t sample_count = length/2;
  // Do something with the data packet
 int byteOffset = 0;
    for (int i = 0; i < FFTsamples; i++) {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
      vImag[i] = 0;
      byteOffset = byteOffset + 4;
    }
    
}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
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
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = 22,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    a2dp_sink.set_pin_config(my_pin_config);
    a2dp_sink.set_i2s_config(i2s_config);
    a2dp_sink.start("TheMatrix");
    a2dp_sink.set_stream_reader(read_data_stream);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

void loop() {
  FFT.Windowing(vReal, FFTsamples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, FFTsamples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, FFTsamples);
  display.clearDisplay();
//   split into bins
  for(int i = 2; i < (FFTsamples/2); i++)
  {
     // Each array element represents a frequency and its value, is the amplitude. Note the frequencies are not discrete.
      if (i<=2 ) {
          displayBand(0,(int)vReal[i]); // 125Hz
      }
      if (i>2 && i<=4 ) {
          displayBand(1,(int)vReal[i]); // 125Hz
      }   
      if (i>4 && i<=6 ) {
          displayBand(2,(int)vReal[i]); // 125Hz
      }   
      if (i>6 && i<=15) {
          displayBand(3,(int)vReal[i]); // 125Hz
      }   
      if (i>15 && i<=30 ) {
          displayBand(4,(int)vReal[i]); // 125Hz
      }   
      if (i>30 && i<=60) {
          displayBand(5,(int)vReal[i]); // 125Hz
      }   
      if (i>60 && i<=100 ) {
          displayBand(6,(int)vReal[i]); // 125Hz
      }   
      if (i>100 && i<=175 ) {
          displayBand(7,(int)vReal[i]); // 125Hz
      }   
      if (i>175 && i<=225 ) {
          displayBand(8,(int)vReal[i]); // 125Hz
      }   
      if (i>225 && i<=275 ) {
          displayBand(9,(int)vReal[i]); // 125Hz
      }   
      if (i>275 && i<=325 ) {
          displayBand(10,(int)vReal[i]); // 125Hz
      }   
      if (i>325 ) {
          displayBand(11,(int)vReal[i]); // 125Hz
      }   
      // if (i>350 && i<=400 ) {
      //     displayBand(12,(int)vReal[i]); // 125Hz
      // }   
      // if (i>400 && i<=450 ) {
      //     displayBand(13,(int)vReal[i]); // 125Hz
      // }   
      // if (i>450 && i<=500 ) {
      //     displayBand(14,(int)vReal[i]); // 125Hz
      // }   
      // if (i>500) {
      //     displayBand(15,(int)vReal[i]); // 125Hz
      // }   
                  
      // if (i >2   && i<=4 )   displayBand(1,(int)vReal[i]); // 250Hz
      // if (i >4   && i<=7 )   displayBand(2,(int)vReal[i]); // 500Hz
      // if (i >7   && i<=15 )  displayBand(3,(int)vReal[i]); // 1000Hz
      // if (i >15  && i<=40 )  displayBand(4,(int)vReal[i]); // 2000Hz
      // if (i >40  && i<=70 )  displayBand(5,(int)vReal[i]); // 4000Hz
      // if (i >70  && i<=288 ) displayBand(6,(int)vReal[i]); // 8000Hz
      // if (i >288           ) displayBand(7,(int)vReal[i]); // 16000Hz
      
    //   if(i){

    //     }
    //    Serial.print("Max: ");
    //    Serial.println(tempi);
    //    //Serial.println(i);
    Serial.print((int)vReal[i]);
    Serial.print(",");
  }
  display.display();
  
  
}