
 /* (c) Jonathan Shulgach - Cite and Notice license:
   All modifications to this code or use of it must include this notice and give credit for use.
   Credit requirements:
    All publications using this code must cite all contributors to this code.
    A list must be updated below indicating the contributors alongside the original or modified code appropriately.
    All code built on this code must retain this notice. All projects incorporating this code must retain this license text alongside the original or modified code.
    All projects incorporating this code must retain the existing citation and license text in each code file and modify it to include all contributors.
    Web, video, or other presentation materials must give credit for the contributors to this code, if it contributes to the subject presented.
    All modifications to this code or other associated documentation must retain this notice or a modified version which may only involve updating the contributor list.
    Primary Authors:
      - Jonathan Shulgach, PhD Student - Neuromechatronics Lab, Carnegie Mellon University
   Contributor List:
   
   Other than the above, this code may be used for any purpose and no financial or other compensation is required.
   Contributors do not relinquish their copyright(s) to this software by virtue of offering this license.
   Any modifications to the license require permission of the authors.
   
   Description:
      This Arduino code handles serial comamnds from an NHP task to operate a reward dispenser. It also measures the weight on
      a scale and updates a 2-segment LED matrix with the change in weight, displayed in mL. 

      MD_MAX72XX library can be found at https://github.com/MajicDesigns/MD_MAX72XX

*/

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <Q2HX711.h>

#define REWARD_LED_PIN_OUT 13
#define LED_DATA_PIN 12
#define LED_CLK_PIN 11
#define LED_CS_PIN 10
#define CONNECT_LED_PIN_OUT 7
#define RELAY_PIN_OUT 6
#define HX711_CLK_PIN 5
#define HX711_BUTTON_PIN_IN 4
#define HX711_DATA_PIN 3
#define BUTTON_PIN_IN 2 // origin: 2

#define MIN_RELAY_OPEN_TIME 0.1  // milliseconds
#define BUFFER_SIZE 16
#define MAX_DEVICES 4 // number of matrix segments connected
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

MD_Parola lc=MD_Parola(HARDWARE_TYPE, LED_DATA_PIN, LED_CLK_PIN, LED_CS_PIN, MAX_DEVICES);
Q2HX711 hx711(HX711_DATA_PIN, HX711_CLK_PIN); // prep hx711

// ====== Sprite Definitions and LED matrix variables =========
const uint8_t F_PMAN1 = 6;
const uint8_t W_PMAN1 = 8;
const uint8_t PROGMEM pacman1[F_PMAN1 * W_PMAN1] =  // gobbling pacman animation
{
  0x00, 0x81, 0xc3, 0xe7, 0xff, 0x7e, 0x7e, 0x3c,
  0x00, 0x42, 0xe7, 0xe7, 0xff, 0xff, 0x7e, 0x3c,
  0x24, 0x66, 0xe7, 0xff, 0xff, 0xff, 0x7e, 0x3c,
  0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c,
  0x24, 0x66, 0xe7, 0xff, 0xff, 0xff, 0x7e, 0x3c,
  0x00, 0x42, 0xe7, 0xe7, 0xff, 0xff, 0x7e, 0x3c,
};

uint8_t scrollSpeed = 25;    // default frame delay value
textEffect_t scrollEffect = PA_SCROLL_LEFT; // or PA_SPRITE
textPosition_t scrollAlign = PA_CENTER; //PA_LEFT, PA_CENTER, PA_RIGHT
uint16_t scrollPause = 2000; // in milliseconds


// ==============HX711 and Loac Cell Variables ===========================
unsigned long delaytime=100;
float y1 = 0.15; // calibrated mass to be added
long x1 = 0L;
long x0 = 0L;
float avg_size = 3.0; // amount of averages for each mass measurement
float mass; // in Kg
int volume; // in mL
float offset = 0;


// ============= Global state-tracking variables =========================
bool VERBOSE = false;
bool idle=false;
volatile bool do_dispense;
unsigned long previousMillis = 0;        // will store last time relay was enabled
unsigned long currentMillis = 0;
volatile double dispense_duration_ms;
double x;
char route, x_str; 
int n_repeat, t_start;
const char *msg[] = {};


void setup(void)
{
  // serial setup
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial.flush();

  // pins setup
  setupPins();
  setupLEDMatrix();
  setupLoadCell();
  
  t_start = millis();
  n_repeat = 1;
  do_dispense = false;
  
}

void loop(void)
{ 
  // Make sure the computer is connected
  checkConnection();

  // Check and parse any incoming serial messages
  parseSerial();

  // Measure the current weight and subtract from the starting weight
  checkWeight();

  // Update the total weight of the water (in mL) used
  updateLEDMatrix();

}

void checkConnection(void)
{
  // Indicate if connected via USB
  if (Serial) 
  {
    digitalWrite(CONNECT_LED_PIN_OUT, HIGH);
  } 
  else
  {
    digitalWrite(CONNECT_LED_PIN_OUT, LOW);
  }
}

void parseSerial(void)
{
  // If interrupt tells us to dispense, then we use a blocking dispense to handle it.
  // ============ Uncomment lines below ============
  if (do_dispense) {
    _handle_blocking_dispense();
  }

  if (Serial.available() > 0) 
  {    
    // read the incoming byte:
    route = Serial.read();    // The first byte is a char that determines how the following bytes are parsed.
    switch (route) 
    {
      case 'c': // "continuous" request
        _start_dispensing();
        break;
      case 'q': // quit "continuous" request
        _stop_dispensing();
        break;
      case 'd': // "duration" request
        x = Serial.parseInt();
        if (VERBOSE)
        {
          Serial.println(x);
        }
        _handle_duration_request(x);
        break;
      case 'r': // "reset" request
        Serial.flush();
        setup();
        return;
      case 'v': // "volume" request
        x = Serial.parseInt();
        _handle_volume_request(x);
        break;
      default:  // unrecognized request
        route = Serial.read();
        break;
    }
    
    if (Serial.available()) 
    {
      if (Serial.find("x")) 
      {
        n_repeat = Serial.parseInt();
      }
      else 
      {
        n_repeat = 1;
      }
      while (Serial.available()) 
      {
        Serial.read();   // "Flush" serial buffer
      }
    }
  }
  delay(1);
  Serial.flush();
}

void setupPins(void) 
{
  // Setup input and output pins
  pinMode(RELAY_PIN_OUT, OUTPUT);  
  pinMode(BUTTON_PIN_IN, INPUT_PULLUP);
  pinMode(HX711_BUTTON_PIN_IN, INPUT_PULLUP);
  
  pinMode(CONNECT_LED_PIN_OUT, OUTPUT);
  pinMode(REWARD_LED_PIN_OUT, OUTPUT);

  // The HX711 pins are already prepared using SPI
  // So are the LED pins
  
  digitalWrite(RELAY_PIN_OUT, LOW);
  digitalWrite(CONNECT_LED_PIN_OUT, LOW);
  digitalWrite(REWARD_LED_PIN_OUT, LOW);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_IN), _handle_manual_dispense, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(HX711_BUTTON_PIN_IN), _handle_manual_dispense2, CHANGE);
//attachInterrupt(digitalPinToInterrupt(HX711_RESET_PIN), _reset_load, CHANGE);
}

void setupLEDMatrix(void)
{
    lc.begin();
    lc.setIntensity(2); // Set the brightness to a medium values ( [0-15] == [low-high] )
    lc.print("Ready!");
    delay(500);
    lc.displayReset(); // and clear the display
}

void setupLoadCell(void)
{
  // calibrateLoadCell()
  x0 = 8584600; // zero point
  x1 = 8739450;

}

void calibrateLoadCell()
{
  // tare procedure
  for (int ii=0;ii<int(avg_size);ii++){
    delay(10);
    x0+=hx711.read();
  }
  x0/=long(avg_size);
  if (VERBOSE)
  {
    Serial.println("Add Calibrated Mass");
  }
  // calibration procedure (mass should be added equal to y1)
  int ii = 1;
  while(true){
    if (hx711.read()<x0+10000){
    } else {
      ii++;
      delay(2000);
      for (int jj=0;jj<int(avg_size);jj++){
        x1+=hx711.read();
      }
      x1/=long(avg_size);
      break;
    }
  }
  if (VERBOSE)
  {
    Serial.println("Calibration Complete");
  }
}

void checkWeight(void)
{
  // Check if reset button is pressed
  if (digitalRead(HX711_BUTTON_PIN_IN) == LOW) {
    _reset_load();
  }
  
  // averaging reading
  long reading = 0;
  for (int jj=0;jj<int(avg_size);jj++)
  {
    reading+=hx711.read();
  }
  reading/=long(avg_size);
  // calculating mass based on calibration and linear fit
  float ratio_1 = (float) (reading-x0);
  float ratio_2 = (float) (x1-x0);
  float ratio = ratio_1/ratio_2;
  mass = (1000*y1*ratio + 1000*offset)/1000;
  if (VERBOSE)
  {
    Serial.print("Raw: ");
    Serial.print(reading);
    Serial.print(", ");
    Serial.print(mass);
    Serial.print(", offset: ");
    Serial.println(offset);
  }
}

void updateLEDMatrix(void)
{
  // Convert the mass number into mL of water  (1Kg = 1000mL)
  volume = mass * 1000;
  
  // Update number displayed to new volume;
  char msg[16];
  itoa(volume, msg, 10);
  lc.print(msg);
  //lc.displayText(msg, PA_CENTER, P.getSpeed(), 500, PA_NO_EFFECT, PA_NO_EFFECT);

  // If 2 hours have passed, play pacman sprite until reset button pressed
  lc.displayText(msg, scrollAlign, scrollSpeed, scrollPause, scrollEffect, scrollEffect);
  lc.setSpriteData(pacman1, W_PMAN1, F_PMAN1, pacman1, W_PMAN1, F_PMAN1);

  if (lc.displayAnimate())
  {
    lc.displayReset();
  }
}

void _handle_blocking_dispense(void) 
{
  //_HANDLE_BLOCKING_DISPENSE This handles running the blocking dispense routine, with notifications to Serial monitor.
  for (int i = 0; i < n_repeat; i++) {
    _blocking_dispense(); 
    delay(50);
  }
  do_dispense = false;
}

void _start_dispensing(void) 
{
  //_START_DISPENSING Set the correct pin(s) to HIGH related to circuit configuration.
  digitalWrite(RELAY_PIN_OUT, HIGH);
  digitalWrite(REWARD_LED_PIN_OUT, HIGH);
}

void _stop_dispensing(void) 
{
  //_STOP_DISPENSING Set the correct pin(s) to LOW related to circuit configuration.
  digitalWrite(RELAY_PIN_OUT, LOW);
  digitalWrite(REWARD_LED_PIN_OUT, LOW);
}

void _handle_manual_dispense(void) 
{
  if (digitalRead(BUTTON_PIN_IN) == LOW) {
    _start_dispensing();
  } else {
    _stop_dispensing();
  }
}

void _blocking_dispense(void) 
{ 
  // Turn the correct pin on, wait, then turn it off
  
  _start_dispensing();
  
  currentMillis = millis();
  previousMillis = currentMillis;
  while (currentMillis - previousMillis < dispense_duration_ms) { 
    currentMillis = millis();
    continue;
  }
  
  _stop_dispensing();
}

void _reset_load(void)
{
  //Serial.println("reset button pressed");
  // reset button pressed and new zero found
  offset = -mass;
  //x0 = hx711.read();
  //digitalWrite(REWARD_LED_PIN_OUT, HIGH);
  //delay(250);
  //digitalWrite(REWARD_LED_PIN_OUT, LOW);
}

void _handle_duration_request(double x) 
{ 
  // Handles requests led with 'd'
  dispense_duration_ms = max(x, MIN_RELAY_OPEN_TIME);
  do_dispense = true; 
}

void _handle_volume_request(double x) 
{ 
  // Handles requests led with 'v'
  x = x / 1000.0;
  dispense_duration_ms = max(x * 777.0 - 93.0, MIN_RELAY_OPEN_TIME); // Uses linear regression from empirical calibration.
  do_dispense = true; 
}
