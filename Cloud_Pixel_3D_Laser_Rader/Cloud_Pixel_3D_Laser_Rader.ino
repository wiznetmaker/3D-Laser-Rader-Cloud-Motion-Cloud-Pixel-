/*
 WizFi360 example: Cloud_Printer_ChatGPT
*/

#include "WizFi360.h"
#include "html_page.h"
#include <LittleFS.h>
#include <SD.h>

#include <Arduino_GFX_Library.h>
Arduino_GFX *tft = create_default_Arduino_GFX();

File myFile;

#include <PNGdec.h>
PNG png;
uint8_t image_x;
uint8_t image_y;

//#define debug_msg
#define buttonPin 14
#define ledPin    12
#define backLightPin 22
#define sdCsPin 17

typedef enum 
{
  listen_to_cloud_motion = 0,
  get_tof_from_cloud_motion,
  display_tof_on_cloud_pixel
}STATE_;
STATE_ currentState;

/* Wi-Fi info */
char ssid[] = "wiznet";       // your network SSID (name)
char pass[] = "KUvT5sT1Ph";   // your network password
IPAddress ip;

String json_String; 
uint16_t dataStart = 0;
uint16_t dataEnd = 0;

uint8_t TOF_MODE =  2;
uint8_t TOF_MODE_LAST;
uint8_t TOF_X;
uint8_t TOF_Y;
uint16_t TOF_DATA;
String TOF_String; 
uint16_t TOF_SCAN_DATA[3888];

// variables will change:
bool buttonState = false;
bool fillScreen_ = true;

int status = WL_IDLE_STATUS;  // the Wifi radio's status

WiFiClient client;
WiFiServer server(80);

void setup() {
  // initialize serial for debugging  
  Serial.begin(115200);
//  while(! Serial);
//    delay(1000);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT); 
  digitalWrite(ledPin, LOW); 
           
  tft->begin();
  tft->fillScreen(WHITE);
  pinMode(backLightPin, OUTPUT); 
  digitalWrite(backLightPin, HIGH); 

  display_logo(260,120,BLACK);

  display_wifi_status(240,250);
  
  // initialize serial for WizFi360 module
  Serial2.setFIFOSize(4096);
  Serial2.begin(2000000);
  WiFi.init(&Serial2);
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  display_wifi_status(240,250);
  
  // print your WiFi shield's IP address
  ip = WiFi.localIP();
  currentState = listen_to_cloud_motion;
  attachInterrupt(digitalPinToInterrupt(buttonPin), onChange, CHANGE); 
  // start the web server on port 80
  server.begin();
}

void loop(){
  switch(currentState){
    case listen_to_cloud_motion:
       {
          // listen for incoming clients
          client = server.available();
          if (client)
          {
            Serial.println("new client");
            // an http request ends with a blank line
            bool currentLineIsBlank = true;
            if (client.connected())
            {
              currentState = get_tof_from_cloud_motion;
            }
          }
       }
       break;
    case get_tof_from_cloud_motion:
       {          
          while (client.available()){
            char c = client.read();
            //Serial.print(c);
            json_String += c;
          }
          if((json_String.length()> 0)&&(TOF_MODE == 2))
          {
            dataStart = json_String.indexOf("{MODE:") + strlen("{MODE:");
            dataEnd = json_String.indexOf("}{", dataStart); 
            TOF_MODE = json_String.substring(dataStart, dataEnd).toInt();
          }
          if(TOF_MODE == 0)
          {
             Serial.println(json_String);
             Serial.println(TOF_MODE);
             currentState = display_tof_on_cloud_pixel;            
          }
          else if(TOF_MODE == 1)
          {
            uint16_t num = 0;
            while (num!= 7776)
            {
              while (client.available()){                
                *((uint8_t*)(TOF_SCAN_DATA) + num) = client.read();
                 num++;
              }
            }
            currentState = display_tof_on_cloud_pixel;       
          }
       }
       break;
    case display_tof_on_cloud_pixel:
       {
          display_dashboard();
          TOF_MODE_LAST = TOF_MODE;
          TOF_MODE = 2;
          json_String= "";          
          if (client.connected())
          {
            currentState = get_tof_from_cloud_motion;
          }
          else
          {
            currentState = listen_to_cloud_motion;
          }
       }
       break;
  }
}

void onChange()
{
  if(digitalRead(buttonPin)== LOW)
  {
    buttonState = true;
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    buttonState = false;
    digitalWrite(ledPin, LOW);
  }
}

void display_wifi_status(uint8_t x,uint8_t y)
{
  if( status != WL_CONNECTED)
  {
    tft->fillCircle(x,y,3,DARKGREY);
    tft->fillArc(x,y, 5, 7, 225, 315, DARKGREY); 
    tft->fillArc(x,y, 9, 11, 225, 315, DARKGREY); 
    tft->fillArc(x,y, 13, 15, 225, 315, DARKGREY); 
  }
  else
  {
    tft->fillCircle(x,y,3,GREEN);
    tft->fillArc(x,y, 5, 7, 225, 315, GREEN); 
    tft->fillArc(x,y, 9, 11, 225, 315, GREEN); 
    tft->fillArc(x,y, 13, 15, 225, 315, GREEN); 
  }
}

void display_dashboard()
{
  uint16_t _x = 0;
  uint16_t _y = 0;
  int32_t pixelVal;
  int16_t lutIndex;
  if(fillScreen_ == true)
  {
    tft->fillScreen(WHITE);
    for(int q = 0; q<72; q++)
    {
      for(int p = 0; p<54; p++)
      {
        _x = q*5+15;
        _y = p*5+35;
        tft->fillRect(_x,_y,4,4,LIGHTGREY);
      }
    }
    fillScreen_ = false;
  }
  tft->setTextColor(DARKGREY);
  tft->setTextSize(2);
  tft->setCursor(160, 4);
  tft->print(String("3D Laser Rader"));// (Cloud Pixel & Cloud motion)"));
  tft->setCursor(390, 35);
  tft->print(String("Mode:"));
  tft->fillRect(390,55,75,20,LIGHTGREY);
  tft->setCursor(390, 170);
  tft->print(String("Dist:"));
  tft->fillRect(390,190,75,20,LIGHTGREY);
  tft->setCursor(385, 285);
  tft->print(String("unit:mm"));
  
  tft->drawLine(0,21,480,21,DARKGREY);

  if((TOF_MODE_LAST == 1)&&(TOF_MODE == 0))
  {
    for(int q = 0; q<72; q++)
    {
      for(int p = 0; p<54; p++)
      {
        _x = q*5+15;
        _y = p*5+35;
        tft->fillRect(_x,_y,4,4,LIGHTGREY);
      }
    }
  }
  if(TOF_MODE == 0)
  {
    tft->fillRect(TOF_X*5+15,TOF_Y*5+35,4,4,LIGHTGREY);
    dataStart = json_String.indexOf("{TOF_X:") + strlen("{TOF_X:");
    dataEnd = json_String.indexOf("}{", dataStart); 
    TOF_X = json_String.substring(dataStart, dataEnd).toInt();
    dataStart = json_String.indexOf("{TOF_Y:") + strlen("{TOF_Y:");
    dataEnd = json_String.indexOf("}{", dataStart); 
    TOF_Y = json_String.substring(dataStart, dataEnd).toInt();
    dataStart = json_String.indexOf("TOF_DATA:") + strlen("TOF_DATA:");
    dataEnd = json_String.indexOf("}", dataStart); 
    TOF_DATA = json_String.substring(dataStart, dataEnd).toInt();
    
    tft->fillRect(390,80,75,20,WHITE);
    tft->setCursor(390, 80);    
    tft->print(String("X:"));
    tft->fillRect(390,100,75,20,LIGHTGREY);
    tft->fillRect(390,125,75,20,WHITE);
    tft->setCursor(390, 125);    
    tft->print(String("Y:"));
    tft->fillRect(390,145,75,20,LIGHTGREY);
    
    tft->setTextColor(RED);
    tft->setCursor(395, 58);
    tft->print(String("Once"));
    tft->setTextColor(GREEN);
    tft->setCursor(395, 103);
    tft->print(TOF_X);
    tft->setCursor(395, 148);
    tft->print(TOF_Y); 
    tft->setCursor(395, 193);
    tft->print(TOF_DATA); 
    tft->fillRect(TOF_X*5+15,TOF_Y*5+35,4,4,RED);
  }
  if(TOF_MODE == 1)
  {
    uint16_t tof_min = 1000;
    uint16_t tof_max = 0;
    for(int q = 0; q<72; q++)
    {
      for(int p = 0; p<54; p++)
      {
        if(TOF_SCAN_DATA[q*54+p]!=4000)
        {
          if(TOF_SCAN_DATA[q*54+p]>tof_max)
          {
            tof_max = TOF_SCAN_DATA[q*54+p];
          }
          if(tof_min > TOF_SCAN_DATA[q*54+p])
          {
            tof_min = TOF_SCAN_DATA[q*54+p];
          }
        }
      }
    }
    Serial.print("tof_min:");
    Serial.println(tof_min);
    Serial.print("tof_max:");
    Serial.println(tof_max);    
    
    tft->setTextColor(RED);
    tft->setCursor(395, 58);
    tft->print(String("Scan"));
    tft->fillRect(390,80,75,20,WHITE);
    tft->setTextColor(DARKGREY);
    tft->setCursor(390, 80);
    tft->print(String("Min:"));
    tft->fillRect(390,100,75,20,LIGHTGREY);
    tft->setTextColor(GREEN);    
    tft->setCursor(395, 103);
    tft->print(tof_min);
    
    tft->fillRect(390,125,75,20,WHITE);
    tft->setTextColor(DARKGREY);
    tft->setCursor(390, 125);    
    tft->print(String("Max:"));
    tft->fillRect(390,145,75,20,LIGHTGREY);    
    tft->setTextColor(GREEN);    
    tft->setCursor(395, 148);
    tft->print(tof_max);
    
    tft->setCursor(395, 193);
    tft->print("--"); 
    tft->fillRect(TOF_X*5+15,TOF_Y*5+35,4,4,RED);
    
    for(int q = 0; q<72; q++)
    {
      for(int p = 0; p<54; p++)
      {
        if(TOF_SCAN_DATA[q*54+p]==4000)
        {
          TOF_SCAN_DATA[q*54+p] = tof_max;
        }
        if(TOF_SCAN_DATA[q*54+p]==0)
        {
          TOF_SCAN_DATA[q*54+p] = tof_min;
        }
      }
    }
    // =========================================================================
    // Colour conversion - one pixel at a time
    // The draw_buffer starts as 16 bit sensor data
    // At the end it is 16 bit RGB (5-6-5)
    // =========================================================================
    for(int q = 0; q<72; q++)
    {
      for(int p = 0; p<54; p++)
      {
        Serial.print(q);
        Serial.print("-");
        Serial.print(p);
        Serial.print("-");
        Serial.print(q*54+p);
        Serial.print(":");
        Serial.println(TOF_SCAN_DATA[q*54+p]);
        lutIndex = map (TOF_SCAN_DATA[q*54+p], tof_min, tof_max , 0x00, 0xff);  // Data is in range so map it LUT index between low and high defaults        
        _x = (71-q)*5+15;
        _y = p*5+35;
        tft->fillRect(_x,_y,4,4,palette[255-lutIndex]);
      }
    }
  }
}

void display_logo(uint16_t x,uint16_t y,uint16_t color)
{
  uint8_t cloud_pixel[5*11]=
  {
    0b00111110,0b01000001,0b01000001,0b01000001,0b00100010, // C
    0b00000000,0b00000000,0b01111111,0b00000000,0b00000000, // l
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00011110,0b00000001,0b00000001,0b00000001,0b00011111, // u
    0b00001110,0b00010001,0b00010001,0b00010001,0b01111111, // d
    0b00000000,0b00000000,0b00000000,0b00000000,0b00000000, // space
    0b01111111,0b01001000,0b01001000,0b01001000,0b00110000, // P
    0b00000000,0b00000000,0b01011111,0b00000000,0b00000000, // i
    0b00010001,0b00001010,0b00000100,0b00001010,0b00010001, // x
    0b00001110,0b00010101,0b00010101,0b00010101,0b00001100, // e
    0b00000000,0b00000000,0b01111111,0b00000000,0b00000000  // l
  };
  uint8_t cloud_motion[5*12]=
  {
    0b00111110,0b01000001,0b01000001,0b01000001,0b00100010, // C
    0b00000000,0b00000000,0b01111111,0b00000000,0b00000000, // l
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00011110,0b00000001,0b00000001,0b00000001,0b00011111, // u
    0b00001110,0b00010001,0b00010001,0b00010001,0b01111111, // d
    0b00000000,0b00000000,0b00000000,0b00000000,0b00000000, // space
    0b01111111,0b00100000,0b00011000,0b00100000,0b01111111, // M
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00010000,0b00111110,0b00010001,0b00010001,0b00000010, // t
    0b00000000,0b00000000,0b01011111,0b00000000,0b00000000, // i
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00011111,0b00010000,0b00010000,0b00010000,0b00001111, // n
  };
  uint16_t _x = x - (5*5*5) - 46;
  uint16_t _y = y - 40;
  for(uint8_t i=0;i<11;i++)
  {    
    if(i == 1 || i == 2 || i ==5 || i==6 ||i==7 ||i==8 || i == 10)
    {
       _x = _x -6;
    }
    else
    {
       _x = _x+4;
    }   
    for(uint8_t m=0;m<5;m++)
    {
      _x = _x +5;
      _y = y - 40;
      for(uint8_t n=0;n<8;n++)
      {
        if((cloud_pixel[i*5+m]>>(7-n))&0x01)
        {
          tft->fillRect(_x+1,_y+1,4,4,color);
        }
        _y += 5;
      }
    }    
  }
  _x = x - (5*5*5) - 46;
  _y = y;
  for(uint8_t i=0;i<12;i++)
  {    
    if(i == 1 || i == 2 || i ==5 || i==6 ||i==9 ||i==10)
    {
       _x = _x -6;
    }
    else
    {
       _x = _x+4;
    }   
    for(uint8_t m=0;m<5;m++)
    {
      _x = _x +5;
      _y = y;
      for(uint8_t n=0;n<8;n++)
      {
        if((cloud_motion[i*5+m]>>(7-n))&0x01)
        {
          tft->fillRect(_x+1,_y+1,4,4,color);
        }
        _y += 5;
      }
    }    
  }
}
