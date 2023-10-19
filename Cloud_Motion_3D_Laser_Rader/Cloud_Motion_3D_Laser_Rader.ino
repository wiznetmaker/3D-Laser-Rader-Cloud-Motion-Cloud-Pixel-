#include "AdafruitIO_Ethernet.h"
#include <SPI.h>
#include <Ethernet.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <EEPROM.h>

//#define Adafruit_IO
#define Cloud_Pixel

#ifdef Adafruit_IO
/************************ Adafruit IO Config *******************************/
// visit io.adafruit.com if you need to create an account, or if you need your Adafruit IO key.
#define IO_USERNAME "Maker_Gavin"
#define IO_KEY "aio_beBL08QUXWTBF53eREKhO***"
AdafruitIO_Ethernet io(IO_USERNAME, IO_KEY);
// set up the 'counter' feed
AdafruitIO_Feed *tof_feed = io.feed("3D Laser Radar Tower");
#endif

// start reading from the first byte (address 0) of the EEPROM
byte value;

#include "TouchyTouch.h"
const int touch_threshold_adjust = 300;
const int touch_pins[] = {9,10,11};
const int touch_count = sizeof(touch_pins) / sizeof(int);
TouchyTouch touches[touch_count];

SoftwareSerial DT(0, 1); //RX--2  TX--3

// Declaration for SH1306 display connected using software I2C (default case):
#define SCREEN_WIDTH  128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SERVO_PIN1 21
Servo servo1;
#define SERVO_PIN2 29
Servo servo2;
uint8_t servo_position_x = 0;
uint8_t servo_position_y = 0;

uint32_t time_hold = 0;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x00, 0x08, 0xdc, 0xEF, 0xFE, 0xED
};
IPAddress ip(10, 0, 1, 72);

char server[] = "10.0.1.193"; // IP address PC
//char server[] = "10.0.1.42";    // IP address cloud pixel
uint32_t port = 80;

//////////////////////////////////// Instruction Set ///////////////////////////////////////////////
const byte SYSCMD[4][8] = {
  {0x01, 0x06, 0x00, 0x20, 0x00, 0x8C, 0x89, 0xA5},// 01 06 00 20 00 8C 89 A5 Set offset calibration distance to 140mm
  {0x01, 0x06, 0x00, 0x21, 0x00, 0x64, 0xD8, 0x2B},// 01 06 00 21 00 64 D8 2B Set xtalk calibration distance to 100mm
  {0x01, 0x06, 0x00, 0x06, 0x00, 0x01, 0xA8, 0x0B},// 01 06 00 06 00 01 A8 0B Load calibration
  {0x01, 0x06, 0x00, 0x01, 0x10, 0x00, 0xD5, 0xCA},// 01 06 00 01 10 00 D5 CA Restart distance measuring module
};
const byte distanceCMD[2][8] = {
  {0x01, 0x06, 0x00, 0x04, 0x00, 0x00, 0xC8, 0x0B},// 01 06 00 04 00 00 C8 0B Set TOF400 measuring range to 1.3m / Set TOF200 measuring range to default
  {0x01, 0x06, 0x00, 0x04, 0x00, 0x01, 0x09, 0xCB},// 01 06 00 04 00 01 09 CB Set TOF400 measuring range to 4.0m / Set TOF200 measuring range to high precision
};
//const byte timeCMD[1][8] = {
//  {0x01, 0x06, 0x00, 0x05, 0x01, 0xF4, 0x99, 0xDC},// 01 06 00 05 01 F4 99 DC Set continuous output and output time interval to 500MS
//};
const byte timeCMD[1][8] = {
  {0x01, 0x06, 0x00, 0x05, 0x00, 0x32, 0x18, 0x1E},// 01 06 00 05 00 32 18 1E Set continuous output and output time interval to 50MS
};
//const byte timeCMD[1][8] = {
//  {0x01, 0x06, 0x00, 0x05, 0x00, 0x10, 0x98, 0x07},//01 06 00 05 01 F4 99 DC Set continuous output and output time interval to 15MS
//};

//*********************TOF SYS System Settings*************************//
//*********Distance Module Restart Function
void modRST()// Distance module restart
{
  for (int i = 0; i < 8; i++)
    DT.write(SYSCMD[3][i]);
}
//*********Distance Output Time Interval Function
void outputTIME500MS()// Output time interval 500ms
{
  for (int i = 0; i < 8; i++)
    DT.write(timeCMD[0][i]);
}
//*************Mode Selection****************************//
void shortdistance()// Short distance mode
{
  for (int i = 0; i < 8; i++)
    DT.write(distanceCMD[0][i]);
}
void longdistance()// Long distance mode
{
  for (int i = 0; i < 8; i++)
    DT.write(distanceCMD[1][i]);
}

uint16_t TOF_DATA[72][54];
String TOF_DATA_String;
uint8_t TOF_MODE = 0;
uint8_t TOF_X_Y = 0;
uint16_t tof_data_uint16;
bool ETH_HARDWARE = 0;

// Define thresholds and corresponding characters
const int thresholds[] = {600, 400, 350, 300, 250, 200, 150, 100, 50};
const char characters[] = {' ', '.', ':', '-', '=', '+', '*', '#', '%'};

EthernetClient client;

void setup() {
  for (int i = 0; i < touch_count; i++) {
    touches[i].begin( touch_pins[i] );
    touches[i].threshold += touch_threshold_adjust; // make a bit more noise-proof
  }
  display.begin(i2c_Address, true); // Address 0x3C default
  display.ScreenVertically();
  display_logo();
  
  servo1.attach(SERVO_PIN1);
  servo_position_x = 118;
  servo1.write(servo_position_x); // horizontal center position // 76<===>118
  servo2.attach(SERVO_PIN2);
  servo_position_y = 130;
  servo2.write(servo_position_y); //vertical center position // 130  
  
  Serial.begin(115200);
// while(! Serial);
//   delay(1000);
  Serial.println("Cloud motion");
  
  DT.begin(115200);
  
  Ethernet.init(17);  // WIZnet W5100S-EVB-Pico
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  delay(2000);
  // Check for Ethernet hardware present
  if (Ethernet.linkStatus() == LinkOFF){
    Serial.println("Ethernet hardware have some problem or Ethernet cable is not connected. :(");
    ETH_HARDWARE = false;
  }
  else
  {
    Serial.println("Ethernet cable is connected. :)");
    ETH_HARDWARE = true;
  }
  shortdistance();
  //longdistance();
  delay(1000);  
  outputTIME500MS();//Output timeout 50MS
  delay(1000);  
  modRST();//Reset TOF module
  delay(1000);
  if(ETH_HARDWARE == true)
  {    
#ifdef Adafruit_IO
    // connect to io.adafruit.com
    io.connect();
    // attach message handler for the counter feed.
    tof_feed->onMessage(handleCount);
    while(io.status() < AIO_CONNECTED){
      Serial.print("please check Ethernet Hardware.");
      delay(500);
    }
    // make sure all feeds get their current values right away
    tof_feed->get();
#endif

#ifdef Cloud_Pixel
    // start the web server on port port
    // server_http.begin();
#endif
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());
  }
}

void loop(){
  if(ETH_HARDWARE == true)
  {
#ifdef Adafruit_IO
    // process messages and keep connection alive
    io.run();
#endif
  }
  // Check for touch button input Add your touch button logic here
   Touch_handling();
   if(TOF_MODE == 1)
   {    
    TOF_DATA_String= "";
    servo_position_x = 36;
    servo1.write(servo_position_x); // horizontal center position // 76<===>118
    servo_position_y = 72;
    servo2.write(servo_position_y); //vertical center position // 130
    for(int i =36; i<180; i=i+4 )
    {
      Touch_handling();
      if(TOF_MODE == 0)
      {
        break;
      }
      servo_position_x = i;
      servo1.write(servo_position_x); 
      delay(5);
      Serial.print("position:"); 
      Serial.println((i-36)/2);
      for(int m =72; m<180; m=m+2)
      {
        Touch_handling();
        if(TOF_MODE == 0)
        {
          break;
        }        
        while(DT.read() != -1)
        {
        }
        while(DT.available() < 6)
        {
        }
        char a = DT.read();
        if(a != 0x01)
        {
          m--;
          continue;
        }
        byte Buf[6];
        DT.readBytes(Buf, 6);
        if (Buf[2] == 0xFF)
        {
          TOF_DATA[(i-36)/2][(m-72)/2] = 4000;
        }
        else
        {
          TOF_DATA[(i-36)/2][(m-72)/2] = Buf[2] * 256 + Buf[3];
        }
        servo_position_y= m+2;        
        servo2.write(servo_position_y);  
      }
      servo_position_x = i+2;
      servo1.write(servo_position_x);
      delay(5);
      Serial.print("position:"); 
      Serial.println((i-34)/2);
      for(int n=180; n>72; n=n-2)
      {
        Touch_handling();
        while(DT.read() != -1)
        {
        }
        while(DT.available() < 6)
        {
        }
        //Serial.print("CMD: ");
        char a = DT.read();
        if(a != 0x01)
        {
          n--;
          continue;
        }
        byte Buf[6];
        DT.readBytes(Buf, 6);
        if (Buf[2] == 0xFF)
        {
          TOF_DATA[(i-34)/2][(n-72)/2-1] = 4000;
        }
        else
        {
          TOF_DATA[(i-34)/2][(n-72)/2-1] = Buf[2] * 256 + Buf[3];
        }
        servo_position_y = n;      
        servo2.write(servo_position_y);
      }
    }
    servo_position_x = 118;
    servo1.write(servo_position_x); 
    servo_position_y = 130;
    servo2.write(servo_position_y); //vertical center position // 130
  
#ifdef Cloud_Pixel
    if (client.connect(server, port)) 
    {
      send_tof_to_cloud_motion();
    }
    else
    {
      Serial.println("connection failed");
    }
#endif
    
#ifdef Adafruit_IO
    {
      tof_feed->save("3D Laser Rader(Scan mode)");
      for(int q = 0; q<54; q++)
      {
        for(int p = 0; p<72; p++)
        {
          Touch_handling();
          // Find the appropriate character and print it
          char toPrint = '@'; // Default character
          for (int i = 0; i < sizeof(thresholds) / sizeof(thresholds[0]); i++) {
            if (TOF_DATA[71-p][q] > thresholds[i]) {
              toPrint = characters[i];
              break;
            }
          }
          TOF_DATA_String += toPrint;
        }
        if(ETH_HARDWARE == true)
        {
          tof_feed->save((TOF_DATA_String.substring(q*72, (q+1)*72-1)));
          delay(2000);
          Serial.println("");
        }
      }
    }
    Serial.println(TOF_DATA_String);
#endif    
    TOF_MODE = 0;
   }
   else
   {
    delay(50);
   }
}

void display_logo()
{
  uint8_t cloud_motion[5*12]=
  {
    0b00111110,0b01000001,0b01000001,0b01000001,0b00100010, // C
    0b00000000,0b00000000,0b01111111,0b00000000,0b00000000, // l
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00011110,0b00000001,0b00000001,0b00000001,0b00011111, // u
    0b00001110,0b00010001,0b00010001,0b00010001,0b01111111, // d
    //0b00000000,0b00000000,0b00000000,0b00000000,0b00000000, // space
    0b01111111,0b00100000,0b00011000,0b00100000,0b01111111, // M
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00010000,0b00111110,0b00010001,0b00010001,0b00000010, // t
    0b00000000,0b00000000,0b01011111,0b00000000,0b00000000, // i
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00011111,0b00010000,0b00010000,0b00010000,0b00001111, // n
  };
  uint16_t _x = 40;
  uint16_t _y = 0;
  for(uint8_t i=0;i<11;i++)
  {
    if((i == 1)||(i ==2))
    {
      _x = _x - 2;
    }
    if(i==8||(i==9))
    {
      _x = _x - 3;
    }
    if(i == 5)
    {
      _x = 36;
    }
    for(uint8_t m=0;m<5;m++)
    {
      if(i >= 5)
      {
        _y = 16;
      }
      else
      {
        _y = 0;
      }
      for(uint8_t n=0;n<8;n++)
      {
        if((cloud_motion[i*5+m]>>(7-n))&0x01)
        {
          display.fillRect(_x+1, _y+1, 1, 1, SSD1306_WHITE); 
        }
        _y += 2;
      }
      _x = _x + 2;
    }
  }
  display.display();
}

void display_tof(uint16_t num)
{
  uint16_t _x = 0;
  uint16_t _y = 0;
  uint8_t between_space = 1;
  uint8_t display_byte[4];
  const uint8_t Repetition_Scrolling[10][5] = {
  {0b00111110,0b01000001,0b01000001,0b01000001,0b00111110}, // 0
  {0b00000000,0b00100001,0b01111111,0b00000001,0b00000000}, // 1
  {0b00100001,0b01000011,0b01000101,0b01001001,0b00110001}, // 2
  {0b00100010,0b01001001,0b01001001,0b01001001,0b00110110}, // 3
  {0b00001100,0b00010100,0b00100100,0b01111111,0b00000100}, // 4
  {0b01110010,0b01010001,0b01010001,0b01010001,0b01001110}, // 5
  {0b00011110,0b00101001,0b01001001,0b01001001,0b00000110}, // 6
  {0b01000011,0b01000100,0b01001000,0b01010000,0b01100000}, // 7
  {0b00110110,0b01001001,0b01001001,0b01001001,0b00110110}, // 8
  {0b00110000,0b01001001,0b01001001,0b01001010,0b00111100}  // 9
  };
  const uint8_t Repetition_Scrolling_mm[5] = {
  0b00011111,0b00010000,0b00011111,0b00010000,0b00001111 //m
  };
  if(num>4000){
    num = 4000;
  }  
  display_byte[0] = (num/1000)%10;
  display_byte[1] = (num/100)%10;
  display_byte[2] = (num/10)%10;
  display_byte[3] = num%10;
 
  display.clearDisplay();
  if(display_byte[0] != 0)
  {
    _x = 40;//+(9 + between_space)*3;;
    _y = 0;
    for(uint8_t i =0;i<4;i++)
    {
      for(uint8_t m=0;m<5;m++)
      {
        _y = 0;
        for(uint8_t n=0;n<8;n++)
        {
          if((Repetition_Scrolling[display_byte[i]][m]>>(7-n))&0x01)
          {
            display.fillRect(_x+1, _y+1, 1, 1, SSD1306_WHITE); 
          }
          _y += 2;
        }
        _x +=2;
      }
      _x += (between_space);
    }
  }
  else if(display_byte[1] != 0)
  {
    _x = 40 + 9 + between_space;
    _y = 0;
    for(uint8_t i =0;i<3;i++)
    {
      for(uint8_t m=0;m<5;m++)
      {
        _y = 0;
        for(uint8_t n=0;n<8;n++)
        {
          if((Repetition_Scrolling[display_byte[i+1]][m]>>(7-n))&0x01)
          {
            display.fillRect(_x+1, _y+1, 1, 1, SSD1306_WHITE); 
          }
          _y += 2;
        }
        _x +=2;
      }
      _x += between_space;
    }
  }
  else if(display_byte[2] != 0)
  {
    _x = 40 + (9 + between_space)*2;
    _y = 0;
    for(uint8_t i =0;i<2;i++)
    {
      for(uint8_t m=0;m<5;m++)
      {
        _y = 0;
        for(uint8_t n=0;n<8;n++)
        {
          if((Repetition_Scrolling[display_byte[i+2]][m]>>(7-n))&0x01)
          {
            display.fillRect(_x+1, _y+1, 1, 1, SSD1306_WHITE); 
          }
          _y += 2;
        }
        _x += 2;
      }
      _x += between_space;
    }
  }
  else
  {
    _x = 40 + (9 + between_space)*3;
    _y = 0;
    for(uint8_t m=0;m<5;m++)
    {
      _y = 0;
      for(uint8_t n=0;n<8;n++)
      {
        if((Repetition_Scrolling[display_byte[3]][m]>>(7-n))&0x01)
        {
          display.fillRect(_x+1, _y+1, 1, 1, SSD1306_WHITE); 
        }
        _y += 2;
      }
      _x += 2;
    }    
  }
  _x = 40 + (9 + between_space)*1;
  _y = 14;
  for(uint8_t m=0;m<5;m++)
  {
    _y = 14;
    for(uint8_t n=0;n<8;n++)
    {
      if((Repetition_Scrolling_mm[m]>>(7-n))&0x01)
      {
        display.fillRect(_x+1, _y+1, 1, 1, SSD1306_WHITE); 
        display.fillRect(_x+1+11, _y+1, 1, 1, SSD1306_WHITE); 
      }
      _y += 2;
    }
    _x += 2;
  }   
  display.display();
}

// you can also attach multiple feeds to the same
// meesage handler function. both counter and counter-two
// are attached to this callback function, and messages
// for both will be received by this function.
void handleCount(AdafruitIO_Data *data) {
  Serial.print("received <- ");
  // since we are using the same function to handle
  // messages for two feeds, we can use feedName() in
  // order to find out which feed the message came from.
  Serial.print(data->feedName());
  Serial.print(" ");
  // print out the received count or counter-two value
  Serial.println(data->value());
}

void Touch_handling()
{
  // key handling
  for ( int i = 0; i < touch_count; i++) {
    touches[i].update();
    if ( touches[i].rose() ) {
      Serial.print("Button:");
      Serial.println(i);
      if ( i == 0) {
        if(TOF_MODE == 0)
        {
          if(TOF_X_Y == 0)
          {
            if(servo_position_x -5 >= 36)
            {
              servo_position_x = servo_position_x -5;
              servo1.write(servo_position_x);
              tof_data_uint16 = Read_tof();
#ifdef Cloud_Pixel
    if (client.connect(server, port)) 
    {
      send_tof_to_cloud_motion();
    }
    else
    {
      Serial.println("connection failed");
    }
#endif
            }
          }
          else
          {
            if(servo_position_y -5 >= 72)
            {
              servo_position_y = servo_position_y - 5;
              servo2.write(servo_position_y);
              tof_data_uint16 = Read_tof();
#ifdef Cloud_Pixel
              if (client.connect(server, port)) 
              {
                send_tof_to_cloud_motion();
              }
              else
              {
                Serial.println("connection failed");
              }
#endif
            }
          }
        }
      }
      if ( i == 1) {
        time_hold = millis();        
      }
      if ( i == 2) {
        if(TOF_MODE == 0)
        {
          if(TOF_X_Y == 0)
          {
            if(servo_position_x +5 <= 180)
            {
              servo_position_x = servo_position_x +5;
              servo1.write(servo_position_x);
              tof_data_uint16 = Read_tof();
#ifdef Cloud_Pixel
              if (client.connect(server, port)) 
              {
                send_tof_to_cloud_motion();
              }
              else
              {
                Serial.println("connection failed");
              }
#endif
            }
          }
          else
          {
            if(servo_position_y +5 <= 180)
            {
              servo_position_y = servo_position_y + 5;
              servo2.write(servo_position_y);
              tof_data_uint16 = Read_tof();
#ifdef Cloud_Pixel
              if (client.connect(server, port)) 
              {
                send_tof_to_cloud_motion();
              }
              else
              {
                Serial.println("connection failed");
              }
#endif
            }
          }
        }
      }
    }
    if ( touches[i].fell() ) {
     Serial.printf("Release:");
     Serial.println(i);
     if ( i == 0) {
      }
      if ( i == 1) {
         if((millis()-time_hold)>500)
         {
          Serial.print("Time_enough");
          if(TOF_MODE == 0)
          {
            TOF_MODE = 1;
          }
          else
          {
            TOF_MODE = 0;
          }
         }
         else
         {
          if(TOF_X_Y == 0)
          {
            TOF_X_Y = 1; 
            Serial.print("TOF_X_Y = 1");
          }
          else
          {
            TOF_X_Y = 0;
            Serial.print("TOF_X_Y = 0");
          }
        }
      }
      if ( i == 2) {
      }
    }
  }
}

uint16_t Read_tof(void)
{   
  while(DT.read() != -1)
  {
  }
  while((DT.available()<6))
  {
     delay(10);
  }
  char a = DT.read();
  byte Buf[6];
  DT.readBytes(Buf, 6);
  if (Buf[2] == 0xFF)
  {
    display_tof(4000);
  }
  else
  {
    Serial.print("TOF:");
    Serial.print(Buf[2] * 256 + Buf[3]);
    Serial.println("mm");
    display_tof(Buf[2] * 256 + Buf[3]);
  }
  return (Buf[2] * 256 + Buf[3]);
}

void send_tof_to_cloud_motion(void)
{
#ifdef Cloud_Pixel
  uint16_t tof_uint16_t[486];
  if(TOF_MODE == 0)
  {
    client.print(String("{MODE:0}{TOF_DATA_NUM:1}{TOF_X:")+String((servo_position_x-36)/2)+String("}{TOF_Y:")+String((servo_position_y-72)/2)+String("}{TOF_DATA:")+tof_data_uint16+String("}"));
//    client.print(String("{MODE:1}{TOF_DATA_NUM:3888}{TOF_DATA:"));
//    for(int q = 0; q<72; q++)
//    {
//      for(int p = 0; p<54; p++)
//      {
//        tof_uint16_t[(q%9)*54+p] =  ((uint16_t)tof_tof[(q * 54 + p) * 2 +1] << 8) | tof_tof[(q * 54 + p) * 2];
//        Serial.println(tof_uint16_t[(q%9)*54+p]);
//      }
//      if((q+1)%9 == 0)
//      {
//        client.write((uint8_t *)tof_uint16_t,972);
//        delay(200);
//      }
//    }
  }
  else
  {
    client.print(String("{MODE:1}{TOF_DATA_NUM:3888}{TOF_DATA:"));
    delay(200);
    for(int q = 0; q<72; q++)
    {
      for(int p = 0; p<54; p++)
      {
        tof_uint16_t[(q%9)*54+p] = TOF_DATA[q][p];
      }
      if((q+1)%9 == 0)
      {
        client.write((uint8_t *)tof_uint16_t,972);
        Serial.write((uint8_t *)tof_uint16_t,972);
        delay(200);
      }
    }
    //client.print(String("}"));
  }
#endif
}
