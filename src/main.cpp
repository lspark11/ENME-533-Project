#include <Stepper.h>
#include <Servo.h>
#include <Arduino.h>

#define le_SW 2 //is the output of the push button switch (active low). When the knob is depressed, the voltage goes LOW.
#define le_DT 3 //is similar to CLK output, but it lags behind CLK by a 90° phase shift. This output is used to determine the direction of rotation.
#define le_CLK 4//is the primary output pulse used to determine the amount of rotation. Each time the knob is turned in either direction by just one detent (click), the ‘CLK’ output goes through one cycle of going HIGH and then LOW.

#define re_SW 5 
#define re_DT 6 
#define re_CLK 7 

//LED PINS
#define y_yled 53 
#define y_gled 51

#define servo_bled 49
#define io_rled 47
#define io_gled 45

#define x_yled 43
#define x_gled 41

//Servo pin
#define servo 23

//Button Pins
#define servo_button 8
#define io_button 9

//Stepper motor pins
#define stepperx_int1 31
#define stepperx_int2 33
#define stepperx_int3 35
#define stepperx_int4 37

#define steppery_int1 30
#define steppery_int2 32
#define steppery_int3 34
#define steppery_int4 36

//Button
int ioButtonState;
int servoButtonState;

//Encoder
int re_currentStateCLK = 0;
int x_counter = 0;
unsigned long x_lastButtonPressed = 0;
bool x_btn_state = false;

int le_currentStateCLK = 0;
int y_counter =0;
unsigned long y_lastButtonPressed = 0;
bool y_btn_state = false;

//Button Booleans
bool systemOn = false;
bool servoDown = false;

// initalize stepper motor pins
const int stepsPerRevolution = 2100;
Stepper myStepper_x(stepsPerRevolution, stepperx_int1, stepperx_int2, stepperx_int3, stepperx_int4); 
Stepper myStepper_y(stepsPerRevolution, steppery_int1, steppery_int2, steppery_int3, steppery_int4); 
int dir = 1;

// put function declarations here:
void encoderCounter(bool, bool&, int&, int&, int, int);
void encoderButtonState(int, bool&, unsigned long&);
void ioButtonActions();
void servoButtonActions();

Servo myServo;

void setup() {
  //Setting up right encoder
  pinMode(re_SW, INPUT_PULLUP);
  pinMode(re_DT, INPUT);
  pinMode(re_CLK, INPUT);

  //setting up left encoder
  pinMode(le_SW, INPUT_PULLUP);
  pinMode(le_DT, INPUT);
  pinMode(le_CLK, INPUT);

  //Setting up LED's
  pinMode(y_yled, OUTPUT);
  pinMode(y_gled, OUTPUT);
  pinMode(servo_bled, OUTPUT);
  pinMode(io_rled, OUTPUT);
  pinMode(io_gled, OUTPUT);
  pinMode(x_yled, OUTPUT);
  pinMode(x_gled, OUTPUT);

  //Setting up servo
  myServo.attach(servo); 

  //Setting up Buttons
  pinMode(io_button, INPUT);
  pinMode(servo_button, INPUT);

  re_currentStateCLK = digitalRead(re_CLK);
  le_currentStateCLK = digitalRead(le_CLK);

  digitalWrite(io_rled, HIGH);

  // set speed for the motors
  myStepper_x.setSpeed(15);
  myStepper_y.setSpeed(15);
  
  Serial.begin(9600);
}

void loop() {

  ioButtonActions();
  servoButtonActions();
  // put your main code here, to run repeatedly:
  encoderButtonState(re_SW, x_btn_state, x_lastButtonPressed);
  encoderButtonState(le_SW, y_btn_state, y_lastButtonPressed);

  encoderCounter(true, x_btn_state, x_counter, re_currentStateCLK, re_DT, re_CLK);
  encoderCounter(false, y_btn_state, y_counter, le_currentStateCLK, le_DT, le_CLK);

}


void ioButtonActions()
{
  ioButtonState = digitalRead(io_button);
  // Serial.print(ioButtonState);
  if (ioButtonState == HIGH && !systemOn) {
    // turn LED on:
    digitalWrite(io_gled, HIGH);
    digitalWrite(io_rled, LOW);

   
    systemOn = true;
    delay(500);
    Serial.print("System on!\n");

  } 
  else if (ioButtonState == HIGH && systemOn) 
  {
    // turn LED off:
    digitalWrite(io_gled, LOW);
    digitalWrite(io_rled, HIGH);
    digitalWrite(x_yled, LOW);
    digitalWrite(x_gled, LOW);
    digitalWrite(y_yled, LOW);
    digitalWrite(y_gled, LOW);
    if (servoDown)
    {
      Serial.print("Raising pen for shutoff. \n");
      myServo.write(90);
      digitalWrite(servo_bled, LOW);
      servoDown = false;
    }

    systemOn = false;
    delay(500);
    Serial.print("System off!\n");
  }
}

void servoButtonActions()
{
  if (systemOn)
  {
    servoButtonState = digitalRead(servo_button);
    // Serial.print(ioButtonState);
    if (servoButtonState == HIGH && !servoDown) {
      // turn LED on:
      digitalWrite(servo_bled, HIGH);
      servoDown = true;
      myServo.write(-90);
      Serial.print("Pen in contact!\n");
      delay(500);
    } 
    else if (servoButtonState == HIGH && servoDown) 
    {
      digitalWrite(servo_bled, LOW);
      servoDown = false;
      myServo.write(90);
      Serial.print("Pen Raised.\n");
      delay(500);
    }
  
    if (servoDown)
    {
      digitalWrite(servo_bled, HIGH);
    }
    else
    {
      digitalWrite(servo_bled, LOW);
    }
  }
  
}

// put function definitions here:
void encoderCounter(bool xcounter, bool& btn_state, int &counter, int &lastState, int CLK, int DT) {
  delay(1);
  int currentStateCLK  = digitalRead(CLK);
  // Serial.print("entering function");
  // Serial.println(state);
  // Serial.println(lastState);
  if (systemOn)
  {
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
      Serial.println(dir*200);
      myStepper_x.step(dir * 200);

    }
    else 
    {
      Serial.print("y movement:");
      myStepper_y.step(dir * 200);

    }
  Serial.println(counter);
  }
  else if(!btn_state) //Allowed to move
  {
    if(xcounter)
    {
      digitalWrite(x_yled, LOW);
      digitalWrite(x_gled, HIGH);
    }
    else
    {
      digitalWrite(y_gled, HIGH);
      digitalWrite(y_yled, LOW);
    }
  }
  else //Not allowed to move. Pressed down
  {
    if(xcounter)
    {
      digitalWrite(x_yled, HIGH);
      digitalWrite(x_gled, LOW);
    }
    else
    {
      digitalWrite(y_yled, HIGH);
      digitalWrite(y_gled, LOW);
    }
  }
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
	delay(5);
}
//---------------------References---------------------
//For encoders: https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/