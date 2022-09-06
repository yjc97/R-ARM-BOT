//Color code
#include "SoftWire.h"
#include "Adafruit_APDS9960.h"
#include "U8g2lib.h"

SoftWire sw(6,7);   //sda,scl
Adafruit_APDS9960 apds=Adafruit_APDS9960(sw);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define RED   1
#define GREEN 2
#define BLUE  3
#define WHITE 4

int R_F = 13000;
int G_F = 19000;
int B_F = 25000;
int r_f = 768;
int g_f = 1024;
int b_f = 1280;
int ii=1;
int j=1;


//Doll Machine Code
#include "SoftwareSerial.h"
#include "LobotServoController.h"

#define LED 13
#define TOUCH 12      //Touch sensor
#define  RxPin 8     //Define soft serial port
#define  TxPin 9

int posStep = 5;

SoftwareSerial mySerial(RxPin, TxPin);       //Set an example of SoftwareSerial
LobotServoController myController(mySerial);//Set an example of LobotSerialController 


//Distance Code
#include "Ultrasound.h"
#include "U8g2lib.h"

Ultrasound ultrasound;  //Instantiate the ultrasonic class

float distance;
int i;                //Current data location
uint16_t r;
uint16_t g;
uint16_t b;

void setup(){
  mySerial.begin(9600);
  Serial.begin(9600);
  digitalWrite(LED, HIGH);
  myController.moveServo(1, 500, 1000);
  myController.moveServo(2, 500, 1000);
  myController.moveServo(3, 260, 1000);
  myController.moveServo(4, 750, 1000);
  myController.moveServo(5, 445, 1000);
  myController.moveServo(6, 500, 1000);
  delay(1500);
}

void getDistance() //Function from Distance.INO
 {
    distance = ((float)ultrasound.GetDistance())/10;
  }

int colorDetect() //from Discriminate_color.IDO
{
  uint16_t r, g, b, c;
  int t;
  //wait for color data to be ready
  while(!apds.colorDataReady()){
    delay(5);
  }
  apds.getColorData(&r, &g, &b, &c);

  r = map(r, r_f, R_F, 0, 255);  
  g = map(g, g_f, G_F, 0, 255);
  b = map(b, b_f, B_F, 0, 255);
  
  //Find the largest value in R, G, B. For example, the maximum is R means that the object is Red
  if (r > g)
    t = RED;
  else
    t = GREEN;
  if (t == GREEN && g < b)
    t = BLUE;
  if (t == RED && r < b)
    t = BLUE;
  Serial.print("R:"); Serial.print(r);    //Serial print and detects rgb value
  Serial.print("G:"); Serial.print(g);
  Serial.print("B:"); Serial.println(b);

  //Returns the color only if the RGB value is greater than 30, otherwise returns 0
  if(t == BLUE && b > 80)
    return t;
  else if(t == GREEN && g > 80)
    return t;
  else if(t == RED && r > 80)         //If the robot color is recognized as red, set the judgment value of "r"in this line to be slightly larger than the r value printed by the serial port above
    return t;
  else 
    return 0;
  return 0;
}



void run()
{
  static uint32_t timer;
//  if (timer < millis())
//  {
      if (distance < 10 & ii==1) {
        myController.moveServo(4,700,500);
        ii=2;
        
          int t; //start of Discriminate_color.INO code
          t = colorDetect();
            u8g2.firstPage();
          do
          {
            u8g2.setFont(u8g2_font_courB24_tf);
            u8g2.setCursor(0, 38);
            //Display color
            if (t == RED & j!=1) {
              myController.moveServo(6,300,500);
              j=1;
            }
            else if (t == BLUE & j!=2) {
              myController.moveServo(6,700,500);
              j=2;
            }
            else if (t == GREEN & j!=3) {
              myController.moveServo(6,900,500);
              j=3;
            }
          } while ( u8g2.nextPage());
        }
      else if (distance > 10 & ii==2) {
        myController.moveServo(4,500,500);
        ii=1;
      }
//      timer = millis() + 250;
      
    }



void loop() {
  getDistance();
  run();
}
