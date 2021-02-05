#include "misakiUTF16.h"

#include <SPI.h>
#include "soc/spi_reg.h"

#include <XPT2046_Touchscreen.h>
//赤外線関連

#include <Arduino.h>
#include <assert.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>

#include <stdio.h>
#include <string>
#include <sstream>


//赤外線関連ここまで
 
#define SPI_NUM 0x2 //VSPI=0x3, HSPI=0x2

#define TONEPIN 25

const int sck = 14; //SCLK (SPI Clock)
const int miso = 12;//MOSI (Master Output Slave Input)
const int mosi = 13; // MOSI(master output slave input) pin//VSPI_D
const int cs_pin = 15;//OLED ChipSelect
const int dc_pin = 4;//OLED DC (Data/Command)
const int rst_pin = 2;//OLED Reset
const uint8_t lcd_led_pin = 32;

const uint8_t CS_SD = 5; //SD card CS ( Chip Select )
const int t_cs_pin = 21;
const int tonePIN = 25;

XPT2046_Touchscreen ts(t_cs_pin);
boolean wastouched = true;

SPIClass hspi(HSPI);

//赤外線関連
const uint16_t kRecvPin = 34;//受信ピン
//const uint16_t kIrLedPin = 17;//赤外線LEDのピン
const uint32_t kBaudRate = 115200;//Serial通信速度
const uint16_t kCaptureBufferSize = 1024;    //バッファサイズ
const uint8_t kTimeout = 50;   //タイムアウト時間
const uint16_t kFrequency = 38000;   //赤外線周波数
//IRsend irsend(kIrLedPin); //送信オブジェクト
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false); //受信オブジェクト
decode_results results; //結果格納変数

//const uint16_t kRecvPin = 34;
long recv_data;
//const uint8_t kTimeout = 50;
//const uint16_t kCaptureBufferSize = 1024;
//const uint32_t kBaudRate = 115200;
//const uint16_t kMinUnknownSize = 12;
//const uint8_t kTolerancePercentage = kTolerance;  // kTolerance is normally 25%
//IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
//decode_results results;  // Somewhere to store the results
//赤外線関連ここまで

// 音階配列
int tones[12] = { 262,294,330,349,392,440,494,523,587,659,698,784 };

//音関連
#define LEDC_TIMER_13_BIT  13
#define LEDC_BASE_FREQ     5000
//#define LEDC_CHANNEL_1     1//led用
//#define LEDC_CHANNEL_2     2//ステレオ用
#define LEDC_CHANNEL_3     3

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define PINK 0xF81F
#define ORANGE 0xFBE0

uint8_t red_max = 31, green_max = 63, blue_max = 31;

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("Booting");

  ILI9341_Init();
  Display_Clear(0, 0, 319, 239);
  Brightness(255); //LCD LED Full brightness
  delay(2000);

  ts.begin();
  ts.setRotation(1);//T 2.4inch=0, 2.8inch=2
}

void loop(){

  //IRloop2();  
  boolean istouched = ts.touched();
  if (istouched) {
    TS_Point p = ts.getPoint();

    //Y
    //px = 320*(p.x-400)/(3990-400);
    //py = 240*(p.y-260)/(3900-260);

    //T 2.8inch
    int px = 320 * (p.x-350)/(3890-350) ;
    int py = 240 * (p.y-260)/(3790-260) ;

//    int px = p.x;
//    int py = p.y;

    Display_Clear(0, 0, 319, 239);
    
    drawText(0,24,px,WHITE);
    drawText(0,40,py,WHITE);
    
    Draw_Circle_Line(px, py, 5, red_max, green_max, blue_max);
    
    tone(tonePIN,tones[1]);
    delay(100);
    tone(tonePIN,0);
  }
  else{
    if (wastouched) 
    {
      
    }
    //Serial.println("no touch");
  }
}

//****** LCD ILI9341 ディスプレイ初期化 ***********
void ILI9341_Init(){
  Brightness(0);
 
  pinMode(rst_pin, OUTPUT); //Set RESET pin
  pinMode(dc_pin, OUTPUT); //Set Data/Command pin
 
  hspi.begin(sck, miso, mosi, cs_pin); //VSPI setting
 
  hspi.setBitOrder(MSBFIRST);
  //ILI9341 のSPI Clock Cycle Time (Write) Minimun 100ns=10MHz
  //※Arduino core ESP32 v1.0.4では10MHz以上にすると正常に表示されないので注意
  hspi.setFrequency(10000000);
  hspi.setDataMode(SPI_MODE0);
  hspi.setHwCs(true); //Set Hardware CS pin
 
  //Hardware Reset------------
  digitalWrite(rst_pin, HIGH);
  delay(5);
  digitalWrite(rst_pin, LOW);
  delay(10);
  digitalWrite(rst_pin, HIGH);
  delay(121);
 
  digitalWrite(dc_pin, HIGH);
 
  CommandWrite(0x38); //Idle mode OFF
  CommandWrite(0x3A); //COLMOD: Pixel Format Set
    DataWrite(0b01010101); //RGB 16 bits / pixel, MCU 16 bits / pixel
  //CommandWrite(0x36); //MADCTL: Memory Access Control
    Disp_Rotation(7);
    //DataWrite(0b00001000); //D3: BGR(RGB-BGR Order control bit )="1"
    //DataWrite(0b11101000); //D3: BGR(RGB-BGR Order control bit )="1"
  CommandWrite(0x11); //Sleep OUT
  delay(10);
  //CommandWrite(0x21); //ILI9342C M5Stack LCD Inversion.
  CommandWrite(0x29); //Display ON
 
  Display_Clear(0, 0, 319, 239);
  Brightness(100);
}
//********* 4wire SPI Data / Command write************
void CommandWrite(uint8_t b){
  digitalWrite(dc_pin, LOW);
  hspi.write(b);
  digitalWrite(dc_pin, HIGH);
}
 
void DataWrite(uint8_t b){
  hspi.write(b);
}
 
void DataWrite16(uint16_t b){
  hspi.write16(b);
}
 
void DataWrite32(uint32_t b){
  hspi.write32(b);
}
//******** Set Column and Page Address ( X Y range setting )***********
void XY_Range(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
  uint32_t X = (uint32_t)x0<<16 | x1;
  uint32_t Y = (uint32_t)y0<<16 | y1;
 
  CommandWrite( 0x2A ); //Set Column Address
    DataWrite32(X);
  CommandWrite( 0x2B ); //Set Page Address
    DataWrite32(Y);
}
//********* Display All Black Clear ******************************
void Display_Clear(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
  uint16_t Width_x = x1 - x0 + 1;
  uint16_t Width_y = y1 - y0 + 1;
  uint32_t Total = Width_x * Width_y ;
 
  Block_SPI_Fast_Write(x0, y0, x1, y1, 0, 0, 0, Total);
}
//********* Display Color Pixel Block Fast Write *****************
void spiWriteBlock(uint16_t color, uint32_t repeat){
  uint16_t color16 = (color >> 8) | (color << 8);
  uint32_t color32 = color16 | color16 << 16;
 
  if (repeat > 15) {
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 255,
                      SPI_USR_MOSI_DBITLEN_S);
 
    while (repeat > 15) {
      while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM)) & SPI_USR)
        ;
      for (uint32_t i = 0; i < 16; i++)
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), color32);
      SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
      repeat -= 16;
    }
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM)) & SPI_USR)
      ;
  }
 
  if (repeat) {
    repeat = (repeat << 4) - 1;
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, repeat,
                      SPI_USR_MOSI_DBITLEN_S);
    for (uint32_t i = 0; i < 16; i++)
      WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), color32);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM)) & SPI_USR)
      ;
  }
}
//*********** LCD ILE9341 Block Pixel SPI Fast Write *****************
void Block_SPI_Fast_Write(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue, uint32_t repeat){
  uint16_t ColorDot = (red << 11) | (green << 5) | blue;
  XY_Range(x0, y0, x1, y1);
  CommandWrite( 0x2C ); //LCD RAM write
  spiWriteBlock(ColorDot, repeat);
}
//*********** 65k Color Pixel (Dot) Write ****************************
void Draw_Pixel_65k_DotColor(uint16_t x0, uint16_t y0, uint16_t DotColor){
  hspi.setFrequency(10000000);
  XY_Range(x0, y0, x0, y0);
  CommandWrite( 0x2C ); //RAM write
  DataWrite16( DotColor );
}
//*********** 65k Pixel RGB color Write ****************************
void Draw_Pixel_65k_3Color(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue){
  hspi.setFrequency(10000000);
  XY_Range(x0, y0, x0, y0);
 
  uint16_t Dot = ((uint16_t)red << 11) | ((uint16_t)green << 5) | (uint16_t)blue;
  CommandWrite( 0x2C ); //RAM write
  DataWrite16( Dot );
}
//***************************************
void Draw_Rectangle_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  Draw_Horizontal_Line(x0, x1, y0, red, green, blue);
  Draw_Horizontal_Line(x0, x1, y1, red, green, blue);
  Draw_Vertical_Line(x0, y0, y1, red, green, blue);
  Draw_Vertical_Line(x1, y0, y1, red, green, blue);
}
//***************************************
void Draw_Horizontal_Line(int16_t x0, int16_t x1, int16_t y0, uint8_t red, uint8_t green, uint8_t blue){
  if(x0 > 319) x0 = 319;
  if(x1 > 319) x1 = 319;
  if(y0 > 239) y0 = 239;
  if(x1 < x0){
    uint16_t dummy = x1;
    x1 = x0;
    x0 = dummy;
  }
 
  uint32_t Width_x = x1 - x0 + 1;
  Block_SPI_Fast_Write(x0, y0, x1, y0, red, green, blue, Width_x);
}
//***************************************
void Draw_Vertical_Line(int16_t x0, int16_t y0, int16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  if(x0 > 319) x0 = 319;
  if(y0 > 239) y0 = 239;
  if(y1 > 239) y1 = 239;
  if(y1 < y0){
    uint16_t dummy = y1;
    y1 = y0;
    y0 = dummy;
  }
 
  uint16_t Width_y = y1 - y0 + 1;
  Block_SPI_Fast_Write(x0, y0, x0, y1, red, green, blue, Width_y);
}
//***************************************
void Draw_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  if(x0 > 319) x0 = 319;
  if(x1 > 319) x1 = 319;
  if(y0 > 239) y0 = 239;
  if(y1 > 239) y1 = 239;
  if(x0 == x1 && y0 == y1) return;
 
  int i;
 
  int16_t Y = 0, X = 0;
  int16_t length_x = x1 - x0;
  int16_t length_y = y1 - y0;
 
  uint16_t Dot = (red << 11) | (green << 5) | blue;
 
  if(abs(length_x) > abs(length_y)){
    float degY = ((float)length_y) / ((float)length_x);
    if(x0 < x1){
      for(i = x0; i < (x1 + 1); i++){
        Y = y0 + round((i-x0) * degY);
        Draw_Pixel_65k_DotColor(i, Y, Dot);
      }
    }else{
      for(i = x0; i >= x1; i--){
        Y = y0 + round((i-x0) * degY);
        Draw_Pixel_65k_DotColor(i, Y, Dot);
      }
    }
  }else{
    float degX = ((float)length_x) / ((float)length_y);
   
    if(y0 < y1){
      for(i = y0; i < (y1 + 1); i++){
        X = x0 + round((i-y0) * degX);
        Draw_Pixel_65k_DotColor(X, i, Dot);
      }
    }else{
      for(i = y0; i >= y1; i--){
        X = x0 + round((i-y0) * degX);
        Draw_Pixel_65k_DotColor(X, i, Dot);
      }
    }
  }
}
//***************************************
void Draw_Rectangle_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  uint16_t Width_x = x1 - x0 + 1;
  uint16_t Width_y = y1 - y0 + 1;
  uint32_t Total = Width_x * Width_y ;
  Block_SPI_Fast_Write(x0, y0, x1, y1, red, green, blue, Total);
}
//***************************************
void Draw_Circle_Line(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue){
  uint16_t x1, y1;
  float i;
  float deg = 1.0;
  if( r > 50 ) deg = 0.5;
  if( r > 110) deg = 0.25;
 
  uint16_t Dot = ((uint16_t)red << 11) | ((uint16_t)green << 5) | (uint16_t)blue;
 
  for(i = 0; i < 360; i = i + deg){
    x1 = round( (float)(x0 + (r * cos(radians(i)))) );
    y1 = round( (float)(y0 + (r * sin(radians(i)))) );
    Draw_Pixel_65k_DotColor(x1, y1, Dot);
  }
}
//***************************************
void Draw_Circle_Fill(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue){
  //red (0-31), green (0-63), blue (0-31)
  uint16_t x1, y1;
  float i;
  float deg = 1.0;
  //半径が大きくなると、角度の刻み方を細かくしないと、完全に塗りつぶせないので注意。
  if( r > 50 ) deg = 0.5;
  if( r > 110) deg = 0.25;
 
  for( i = 0; i < 360; i = i + deg ){
    x1 = round( (float)(x0 + (r * cos(radians(i)))) );
    y1 = round( (float)(y0 + (r * sin(radians(i)))) );
    Draw_Vertical_Line(x1, y0, y1, red, green, blue);
  }
}
//********* LCD Display LED Brightness **************
void Brightness(uint8_t brightness){
  uint8_t ledc_ch = 0;
  uint32_t valueMax = 255;
  uint32_t duty = (8191 / valueMax) * brightness;
  ledcSetup(ledc_ch, 5000, 13);
  ledcAttachPin(lcd_led_pin, ledc_ch);
  ledcWrite(ledc_ch, duty);
}

//void drawUI()
//{
//  
//  boolean istouched = ts.touched();
//  if (istouched) 
//  {
//    TS_Point p = ts.getPoint();
//
//    //Y
////    px = 320*(p.x-400)/(3990-400);
////    py = 240*(p.y-260)/(3900-260);
//
//    //T 2.8inch
//    int px = 240 * (p.x-400)/(3990-400) +5;
//    int py = 320 * (p.y-260)/(3790-260) +5;
//
//  
//  drawTest(px, py);
//    
//  }else 
//  {
//    if (wastouched) 
//    {
//
//    }
//    //Serial.println("no touch");
//  }
//}

void drawTest(int px, int py){
  int i, j;
  //red (0-31), green (0-63), blue (0-31)
  

  Display_Clear(0, 0, 319, 239);
  Draw_Rectangle_Line(40, 0, 279, 239, 0, 0, blue_max);
  Draw_Line(40, 0, 279, 239, 0, 0, blue_max);
  Draw_Line(40, 239, 279, 0, 0, 0, blue_max);
  Draw_Circle_Line(159, 119, 119, red_max, green_max, blue_max);
  Draw_Circle_Line(25, 25, 25, red_max, 0, 0);
  Draw_Circle_Line(294, 25, 25, 0, green_max, 0);
  Draw_Circle_Line(25, 214, 25, red_max, green_max, 0);
  Draw_Circle_Line(294, 214, 25, red_max, 0, blue_max);
  for(i = 0; i < 120; i = i + 5){
    Draw_Circle_Line(px, py, 119 - i, red_max, green_max, blue_max);
  }
}

void IRloop2(){
    if (irrecv.decode(&results)) {  //データを受け取った場合
    drawText(0, 48, 77777, YELLOW);
    irrecv.resume();  //受信再開
  }
  yield();
}

void IRloop(){
  
  if (irrecv.decode(&results)) {
    //drawTest(100, 100);
    
    if(results.value!=-1)
    {
      recv_data = results.value;
    }

//    drawText(0,38,777,WHITE);
    
    drawText(0, 48, results.value, YELLOW);
    
    if(recv_data == 33441975)
    {
      
      //game1->minoVr = -1;
//      tone(tonePIN,tones[0]);
//      delay(100);
//      tone(tonePIN,0);
    }
    else if(recv_data == 33446055)
    {
      tone(TONEPIN,tones[1]);
      delay(100);
      tone(TONEPIN,0);
//        if(marioBgNo[3] != 52||marioBgNo[4] != 52)
//        {
//          if(charaMario->pos.x>48)charaMario->pos.x-=4;
//          else charaMario->pos.x=48;
//        }
      
    }else if(recv_data == 33454215){
      tone(TONEPIN,tones[2]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33456255){
      tone(TONEPIN,tones[3]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33439935){
      tone(TONEPIN,tones[4]);
      delay(100);
      tone(TONEPIN,0);
//        if(marioBgNo[5] != 52||marioBgNo[4] != 52){
//          if(charaMario->pos.x<352)charaMario->pos.x+=4;
//          else charaMario->pos.x=352;
//        }
      
    }else if(recv_data == 33472575){
      tone(TONEPIN,tones[5]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33431775){
      tone(TONEPIN,tones[6]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33464415){
      tone(TONEPIN,tones[7]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33448095){
      tone(TONEPIN,tones[0]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33480735){
      tone(TONEPIN,tones[1]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33427695){
      tone(TONEPIN,tones[2]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33460335){
      tone(TONEPIN,tones[3]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33444015){
      tone(TONEPIN,tones[4]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33478695){
      tone(TONEPIN,tones[5]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33486855){
      tone(TONEPIN,tones[6]);
      delay(100);
      tone(TONEPIN,0);
    }
    
    if(recv_data == 33435855){
      tone(TONEPIN,tones[7]);
      delay(100);
      tone(TONEPIN,0);
//      jumpF = true;
//      objBg->drawBgXLine(tft,charaMario->pos.x);
//      charaMario->pos.y = 80;
//    
    }else if(recv_data == 33468495){
      tone(TONEPIN,tones[0]);
      delay(100);
      tone(TONEPIN,0);
    }else if(recv_data == 33452175){
      tone(TONEPIN,tones[1]);
      delay(100);
      tone(TONEPIN,0);
    }
    //Serial.println(irrecv.decode(&results));
    //Serial.println("");

    irrecv.resume();  // Receive the next value
    //printinfo();
  }
}

void tone(int pin, int freq)
{
  ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT) ;
  ledcAttachPin(pin, LEDC_CHANNEL_3) ;
  ledcWriteTone(LEDC_CHANNEL_3, freq) ;
}

void drawText(int _x, int _y, int _val, uint16_t color) {

  char buf[128];
  std::ostringstream oss;
  int num = _val;
  oss << "" << num;
  const char *charP = oss.str().c_str();
  strcpy(buf, charP);
  drawJPChar(_x, _y, buf, color);
}

unsigned char animation_offsetY=0;
// ビットパターン表示
// d: 8ビットパターンデータ
//
void bitdisp(byte x, byte y, uint8_t d ,uint16_t color) {
  for (byte i=0; i<8;i++) {
    if (d & 0x80>>i) {
      if(x + i < 320 && y < 240){
          Draw_Pixel_65k_DotColor(x + i , y , color);
      }
    }      
  }
}
// フォントパターンを画面に表示する
// 引数
// x,y 表示位置
//  pUTF8 表示する文字列
// ※半角文字は全角文字に置き換えを行う
//
void drawJPChar(byte x, byte y, char * pUTF8, uint16_t color) {
  int n;
  uint16_t pUTF16[40];
  byte buf[20][8];  //160x8ドットのバナー表示パターン
  n = Utf8ToUtf16(pUTF16, pUTF8);  // UTF8からUTF16に変換する

  // バナー用パターン作成
  for (byte i=0; i < n; i++) {
    getFontDataByUTF16(&buf[i][0], utf16_HantoZen(pUTF16[i]));  // フォントデータの取得    
  }
  
  // ドット表示
  for (byte i=0; i < 8; i++) {
    for (byte j=0; j < n; j++){
        bitdisp(x + (j * 8) ,y + i , buf[j][i], color);
    }
  }
}


int _Max_Width_x = 0;
int _Max_Width_y = 0;

void Disp_Rotation(uint8_t rot){
  uint8_t b = 0b0001000;
  switch( rot ){
    case 0:
      b = 0b0001000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 1:
      b = 0b10101000;
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    case 2: //M5stack 横表示、上下逆
      b = 0b11001000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 3: //M5stack 縦表示、上下逆
      b = 0b01101000;
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    //------------------------
    case 4: //M5stack 横表示、左右反転
      b = 0b01001000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 5: //M5stack 縦表示、左右反転
      b = 0b00101000;
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    case 6: //M5stack 横表示、上下逆、左右反転
      b = 0b10001000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 7: //M5stack 縦表示、上下逆、左右反転
      b = 0b11101000;
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    //-------------------------------
    case 8: //M5stack 横表示、デフォルト、上下反転
      b = 0b10001000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 9: //M5stack 縦表示、デフォルト、上下反転
      b = 0b11101000;
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    case 10: //M5stack 横表示、上下逆、上下反転
      b = 0b01001000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 11: //M5stack 縦表示、上下逆、上下反転
      b = 0b00101000;
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    //-------------------------------
    case 12: //M5stack で表示変わらず
      b = 0b00001100;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 13: //M5stack で表示変わらず
      b = 0b00011000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;

    //------------------------
    case 250:
      b = 0b00101000; //ESP32_mgo_tec bv1.0.69 HiLetgo 2.8", サインスマート販売のILI9341横正常表示
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 251:
      b = 0b10001000; //ESP32_mgo_tec bv1.0.69 HiLetgo 2.8", サインスマート販売のILI9341の縦方向表示
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    case 252: //ESP32_mgo_tec bv1.0.69 HiLetgo 2.8"　横表示　上下逆
      b = 0b11101000;
      _Max_Width_x = 320;
      _Max_Width_y = 240;
      break;
    case 253: //ESP32_mgo_tec bv1.0.69 HiLetgo 2.8"、 縦表示　上下逆
      b = 0b01001000;
      _Max_Width_x = 240;
      _Max_Width_y = 320;
      break;
    case 254:
      b = 0b00101000; //ESP32_mgo_tec bv1.0.71～ HiLetgo 2.8" 横正常表示
      _Max_Width_x = 320;
      _Max_Width_y = 240;
//      ESP32_LCD_ILI9341_SPI::dispInversionOn();
      break;
    default:
      break;
  }
//  _Max_Pix_X = _Max_Width_x - 1;
//  _Max_Pix_Y = _Max_Width_y - 1;
//  _txt_H_max = _Max_Width_x / 8;
//  ESP32_LCD_ILI9341_SPI::SPI_set_change();
  CommandWrite(0x36); //MADCTL: Memory Access Control
  DataWrite(b); //M5stack only. D3: BGR(RGB-BGR Order control bit )="1"
//  if(!_Hw_cs) digitalWrite(_cs, HIGH);
}
