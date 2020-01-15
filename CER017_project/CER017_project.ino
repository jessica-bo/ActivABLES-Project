#include <Wire.h>
#include <math.h> 
#include <Mouse.h>
#include <Keyboard.h>


#define DEVICE_A (0x1D)    //first ADXL345 device address
#define DEVICE_B (0x53)    //second ADXL345 device address
#define TO_READ (6)        //number of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ];      //6 bytes buffer for saving data read from the device
char str[512];            //string buffer to transform data before sending it to the serial port

const int ledg = 4;           // Green LED for extension
const int ledy = 7;           // Yellow LED for flexion
const int for_thres = 15;
const int back_thres = -15;

const int REDPIN1 = 5;          // Red LED strip light
const int GREENPIN1 = 6;        // Green LED strip light
const int BLUEPIN1 = 9;         // Blue LED strip light

const int bendin1 = A0;       // Bending sensor pin
int bendvol1;                  // Bending sensor reading

char UpArrow=KEY_UP_ARROW;
char DownArrow=KEY_DOWN_ARROW;
char LeftArrow=KEY_LEFT_ARROW;
char RightArrow=KEY_RIGHT_ARROW;

const float pi = 3.14159265;


void setup()
{
  Keyboard.begin();
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  
  //Turning on the both ADXL345s
  writeTo(DEVICE_A, 0x2D, 24);   
  writeTo(DEVICE_B, 0x2D, 24);

  pinMode(ledy,OUTPUT);
  pinMode(ledg,OUTPUT);
}

int regAddress = 0x32;      //first axis-acceleration-data register on the ADXL345
int xa = 0, ya = 0, za = 0;  
int xb = 0, yb = 0, zb = 0;
  
void loop()
{  
  //analogWrite(bendled1,0);
  readFrom(DEVICE_A, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  xa = (((int)buff[1]) << 8) | buff[0];   
  ya = (((int)buff[3])<< 8) | buff[2];
  za = (((int)buff[5]) << 8) | buff[4];

  double xRest1 = -0.740124528;
  double xconversion1 = 0.037006226;
  double yRest1 = -0.555093396;
  double yconversion1 = 0.037006226;
  double zRest1 = 0.316343548;
  double zconversion1 = 0.039542944;

  double x2a = xconversion1*xa + xRest1;
  double y2a = yconversion1*ya + yRest1;
  double z2a = zconversion1*za + zRest1;
  
  Serial.print(x2a);
  Serial.print(", ");
  Serial.print(y2a);
  Serial.print(", ");
  Serial.println(z2a);
  
  readFrom(DEVICE_B, regAddress, TO_READ, buff); //read the acceleration data from the second ADXL345
  xb = (((int)buff[1]) << 8) | buff[0];   
  yb = (((int)buff[3])<< 8) | buff[2];
  zb = (((int)buff[5]) << 8) | buff[4];

  double xRest2 = -0.740124528;
  double xconversion2 = 0.037006226;
  double yRest2 = -0.555093396;
  double yconversion2 = 0.037006226;
  double zRest2 = 0.316343548;
  double zconversion2 = 0.039542944;

  double x2b = xconversion2*xb + xRest2;
  double y2b = yconversion2*yb + yRest2;
  double z2b = zconversion2*zb + zRest2;
  
  Serial.print(x2b);
  Serial.print(", ");
  Serial.print(y2b);
  Serial.print(", ");
  Serial.println(z2b);

  Serial.print(x2b);
  Serial.print(", ");
  Serial.print(y2b);
  Serial.print(", ");
  Serial.println(z2b); 

  double vector1=sqrt(sq(x2a)+sq(y2a)+sq(z2a));
  double vector2=sqrt(sq(x2b)+sq(y2b)+sq(z2b));

  // Angles of tilt 
  double a1 = acos(x2a/vector1)/pi*180;
  double b1 = acos(y2a/vector1)/pi*180;
  double c1 = acos(z2a/vector1)/pi*180;

  double a2 = acos(x2b/vector2)/pi*180;
  double b2 = acos(y2b/vector2)/pi*180;
  double c2 = acos(z2b/vector2)/pi*180;

  double a12 = a1-a2;
  double b12 = b1-b2;
  double c12 = c1-c2;

  Serial.print("The x angle is: ");
  Serial.println(a12);
  Serial.print("The y angle is: ");
  Serial.println(b12);
  Serial.print("The z angle is: ");
  Serial.println(c12);
  Serial.println("");
  //we send the x y z values as a string to the serial port
  //sprintf(str, "%d %d %d %d %d %d ", xa, ya, za, xb, yb, zb);  
  //Serial.print(str);
  if (b12 > for_thres) {
    Serial.println("Forwards!");
    Serial.println(" ");
    digitalWrite(ledy, HIGH); 
    digitalWrite(ledg, LOW);
    Keyboard.write(UpArrow);
  }

  if (b12 < back_thres) {
    Serial.println("Backwards!");
    Serial.println(" ");
    digitalWrite(ledg, HIGH); 
    digitalWrite(ledy, LOW);
    Keyboard.press(DownArrow);
  }

  if (for_thres > b12 && b12 > back_thres) {
    Serial.println("Neutral!");
    Serial.println(" ");
    digitalWrite(ledg, LOW); 
    digitalWrite(ledy, LOW);
  }
  Serial.write(10);

  
  int rval1, gval1, bval1;
  //bending sensor
  bendvol1 = analogRead(bendin1);
  //Serial.println(bendvol1);
  

  int bend_map1 = map(bendvol1, 70, 290, 0, 255);
  Serial.println(bend_map1);
  
  if(bend_map1<127){
     Keyboard.write(LeftArrow);}
  else if (bend_map1<154){
      //neutral  
      }
  else if (bend_map1<200){
    Keyboard.write(RightArrow);}
  
  if (bend_map1 < 153) {
    rval1 = -5*(bend_map1 - 102);
    gval1 = 5*bend_map1;
    bval1 = 5*(bend_map1 - 102);
  }
  else {
    rval1 = 5*(bend_map1 - 204);
    gval1 = -5*(bend_map1 - 204);
    bval1 = 255;
  }
  rval1 = constrain(rval1, 0, 255);
  gval1 = constrain(gval1, 0, 255);
  bval1 = constrain(bval1, 0, 255);

  analogWrite(REDPIN1, rval1);
  analogWrite(GREENPIN1, gval1);
  analogWrite(BLUEPIN1, bval1);


  //bending sensor
  //bendvol1=analogRead(bendin1);
  //int newbendvol1=map(bendvol1,0,10,0,1023);
  //Serial.println(newbendvol1);
  //if (bendvol1>450){
    //analogWrite(bendled1,1023);
    //Mouse.move(0,-40);
    //Keyboard.press(UpArrow);
  //}
  //It appears that delay is needed in order not to clog the port
  delay(1000);
  Keyboard.releaseAll();
}





