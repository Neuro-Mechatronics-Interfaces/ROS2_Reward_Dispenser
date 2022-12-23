 /* (c) Jonathan Shulgach - Cite and Notice license:
   All modifications to this code or use of it must include this notice and give credit for use.

   Credit requirements:
    All publications using this code must cite all contributors to this code.
    A list must be updated below indicating the contributors alongside the original or modified code appropriately.
    All code built on this code must retain this notice. All projects incorporating this code must retain this license text alongside the original or modified code.
    All projects incorporating this code must retain the existing citation and license text in each code file and modify it to include all contributors.
    Web, video, or other presentation materials must give credit for the contributors to this code, if it contributes to the subject presented.
    All modifications to this code or other associated documentation must retain this notice or a modified version which may only involve updating the contributor list.

   Contributor List:
    - Jonathan Shulgach, PhD Student - Neuromechatronics Lab, Carnegie Mellon University
    - Max D. Murphy, Postdoctoral Researcher - Neuromechatronics Lab, Carnegie Mellon University

   Other than the above, this code may be used for any purpose and no financial or other compensation is required.
   Forrest, and other contributors if there are any, do not relinquish their copyright(s) to this software by virtue of offering this license.
   Any modifications to the license require permission of the authors.
*/


#define MIN_RELAY_OPEN_TIME 0.1  // milliseconds
#define BUFFER_SIZE 16
#define BUTTON_PIN_IN 2
#define REWARD_LED_PIN_OUT 13
#define CONNECT_LED_PIN_OUT 4
#define RELAY_PIN_OUT 6

// Global state-tracking variables
volatile bool do_dispense;
unsigned long previousMillis = 0;        // will store last time relay was enabled
unsigned long currentMillis = 0;
volatile double dispense_duration_ms;
double x;
char route, x_str; 
int n_repeat, t_start;

void setup()
{
  t_start = millis();
  n_repeat = 1;
  do_dispense = false;
  
  setupPins();
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial.flush();
  // Commented out since not sure if client needs to clear received bytes before sending new commands
  //Serial.begin("NML-NHP Reward"); //Bluetooth device name
}

void loop(){ 
  
  // Indicate if connected via USB
  if (Serial) {
    digitalWrite(CONNECT_LED_PIN_OUT, HIGH);
  } else {
    digitalWrite(CONNECT_LED_PIN_OUT, LOW);
  }

  // If interrupt tells us to dispense, then we use a blocking dispense to handle it.
  // ============ Uncomment lines below ============
  if (do_dispense) {
    _handle_blocking_dispense();
  }
  // ===============================================
  
  if (Serial.available() > 0) {
    
    // read the incoming byte:
    route = Serial.read();    // The first byte is a char that determines how the following bytes are parsed.
    switch (route) {
      case 'c': // "continuous" request
        _start_dispensing();
        break;
      case 'q': // quit "continuous" request
        _stop_dispensing();
        break;
      case 'd': // "duration" request
        x = Serial.parseInt();
        Serial.println(x);
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
    

    if (Serial.available()) {
      if (Serial.find("x")) {
        n_repeat = Serial.parseInt();
      } else {
        n_repeat = 1;
      }
      while (Serial.available()) {
        Serial.read();   // "Flush" serial buffer
      }
    }
  }
  delay(1);
  Serial.flush();
}

// BEGIN: SETUP FUNCTIONS
void setupPins() { // Setup input and output pins
  pinMode(RELAY_PIN_OUT, OUTPUT);  
  pinMode(CONNECT_LED_PIN_OUT, OUTPUT);
  pinMode(REWARD_LED_PIN_OUT, OUTPUT);
  pinMode(BUTTON_PIN_IN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN_OUT, LOW);
  digitalWrite(CONNECT_LED_PIN_OUT, LOW);
  digitalWrite(REWARD_LED_PIN_OUT, LOW);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_IN), _handle_manual_dispense, CHANGE);
}
// END: SETUP FUNCTIONS

// BEGIN: DISPENSING FUNCTIONS

void _handle_blocking_dispense() {
  //_HANDLE_BLOCKING_DISPENSE This handles running the blocking dispense routine, with notifications to Serial monitor.
  for (int i = 0; i < n_repeat; i++) {
    _blocking_dispense(); 
    delay(50);
  }
  do_dispense = false;
}

void _start_dispensing() {
  //_START_DISPENSING Set the correct pin(s) to HIGH related to circuit configuration.
  digitalWrite(RELAY_PIN_OUT, HIGH);
  digitalWrite(REWARD_LED_PIN_OUT, HIGH);
}

void _stop_dispensing() {
  //_STOP_DISPENSING Set the correct pin(s) to LOW related to circuit configuration.
  digitalWrite(RELAY_PIN_OUT, LOW);
  digitalWrite(REWARD_LED_PIN_OUT, LOW);
}

void _handle_manual_dispense() {
  if (digitalRead(BUTTON_PIN_IN) == LOW) {
    _start_dispensing();
  } else {
    _stop_dispensing();
  }
}

void _blocking_dispense() { // Turn the correct pin on, wait, then turn it off
  
  _start_dispensing();
  
  currentMillis = millis();
  previousMillis = currentMillis;
  while (currentMillis - previousMillis < dispense_duration_ms) { 
    currentMillis = millis();
    continue;
  }
  
  _stop_dispensing();
}

// END: DISPENSING FUNCTIONS

// BEGIN: HANDLE FUNCTIONS
void _handle_duration_request(double x) { // Handles requests led with 'd'
  dispense_duration_ms = max(x, MIN_RELAY_OPEN_TIME);
  do_dispense = true; 
}

void _handle_volume_request(double x) { // Handles requests led with 'v'
  x = x / 1000.0;
  dispense_duration_ms = max(x * 777.0 - 93.0, MIN_RELAY_OPEN_TIME); // Uses linear regression from empirical calibration.
  do_dispense = true; 
}
// END: HANDLE FUNCTIONS
