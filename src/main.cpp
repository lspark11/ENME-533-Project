
#include <Stepper.h>
#include <Arduino.h>
#define le_SW 2 //is the output of the push button switch (active low). When the knob is depressed, the voltage goes LOW.
#define le_DT 3 //is similar to CLK output, but it lags behind CLK by a 90° phase shift. This output is used to determine the direction of rotation.
#define le_CLK 4//is the primary output pulse used to determine the amount of rotation. Each time the knob is turned in either direction by just one detent (click), the ‘CLK’ output goes through one cycle of going HIGH and then LOW.


#define re_SW 5 
#define re_DT 6 
#define re_CLK 7 

int re_currentStateCLK = 0;
int x_counter = 0;
unsigned long x_lastButtonPressed = 0;
bool x_btn_state = false;

int le_currentStateCLK = 0;
int y_counter =0;
unsigned long y_lastButtonPressed = 0;
bool y_btn_state = false;

// initalize stepper motor pins
const int stepsPerRevolution = 2100;
Stepper myStepper_x(stepsPerRevolution, 31, 33, 35, 37);
Stepper myStepper_y(stepsPerRevolution, 30, 32, 34, 36);
int dir = 1;

// put function declarations here:
void encoderCounter(bool, bool&, int&, int&, int, int);
void encoderButtonState(int, bool&, unsigned long&);

void setup() {
  //Setting up right encoder
  pinMode(re_SW, INPUT_PULLUP);
  pinMode(re_DT, INPUT);
  pinMode(re_CLK, INPUT);

  //setting up left encoder
  pinMode(le_SW, INPUT_PULLUP);
  pinMode(le_DT, INPUT);
  pinMode(le_CLK, INPUT);

  re_currentStateCLK = digitalRead(re_CLK);
  le_currentStateCLK = digitalRead(le_CLK);

  // set speed for the motors
  myStepper_x.setSpeed(10);
  myStepper_y.setSpeed(10);
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  encoderButtonState(re_SW, x_btn_state, x_lastButtonPressed);
  encoderButtonState(le_SW, y_btn_state, y_lastButtonPressed);


  encoderCounter(true, x_btn_state, x_counter, re_currentStateCLK, re_DT, re_CLK);
  encoderCounter(false, y_btn_state, y_counter, le_currentStateCLK, le_DT, le_CLK);

  // stepper_x(x_counter);
  
}

// put function definitions here:
void encoderCounter(bool xcounter, bool& btn_state, int &counter, int &lastState, int CLK, int DT) {
  delay(1);
  int currentStateCLK  = digitalRead(CLK);
  // Serial.print("entering function");
  // Serial.println(state);
  // Serial.println(lastState);


  if (currentStateCLK != lastState && currentStateCLK == 1 && !btn_state)
  {
    // Serial.print("changing counter value");
    if(digitalRead(DT) != currentStateCLK)
    {
      // Serial.print("counter++ CW");
      counter++;
      dir = 1;
    }
    else
    {
      // Serial.print("counter-- CCW");
      counter--;
      dir = -1;
    }

    if (xcounter)
    {
      Serial.print("x movement:");
      myStepper_x.step(dir * 200);
    }
    else
    {
      Serial.print("y movement:");
      myStepper_y.step(dir * 200);
    }
  Serial.println(counter);
  }
  lastState = currentStateCLK;
  return;
}

void encoderButtonState(int SW_PIN, bool &btn_state, unsigned long &lastButtonPress)
{
  bool btnPressed = false;
  	// Read the button state
  int btnState = digitalRead(SW_PIN);
  
  if (btnState == LOW)
  //If we detect LOW signal, button is pressed
  {
    //if 50ms have passed since last LOW pulse, it means that the
		//button has been pressed, released and pressed again
		if (millis() - lastButtonPress > 50) 
    {
      btnPressed = true;
			Serial.println("Button pressed!");
		}
    // Remember last button press event
		lastButtonPress = millis();
  }
  if (btnPressed)
  {
    if (btn_state)
    {
      btn_state = false;
    }
    else  
    {
      btn_state = true;
    }
  }
  // Put in a slight delay to help debounce the reading
	delay(1);
}
//---------------------References---------------------
//For encoders: https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/