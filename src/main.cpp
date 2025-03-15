#include <Arduino.h>
#define re_SW 2 //is the output of the push button switch (active low). When the knob is depressed, the voltage goes LOW.
#define re_DT 3 //is similar to CLK output, but it lags behind CLK by a 90° phase shift. This output is used to determine the direction of rotation.
#define re_CLK 4 //is the primary output pulse used to determine the amount of rotation. Each time the knob is turned in either direction by just one detent (click), the ‘CLK’ output goes through one cycle of going HIGH and then LOW.

int re_currentStateCLK = 0;
int re_counter = 0;

// put function declarations here:
void encoderCounter(int&, int&, int, int);


void setup() {
  //Setting up right encoder
  pinMode(re_SW, INPUT);
  pinMode(re_DT, INPUT);
  pinMode(re_CLK, INPUT);

  re_currentStateCLK = digitalRead(re_CLK);
  Serial.begin (9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  encoderCounter(re_counter, re_currentStateCLK, re_DT, re_CLK);

}

// put function definitions here:
void encoderCounter(int &counter, int &lastState, int CLK, int DT) {
  delay(1);
  int currentStateCLK  = digitalRead(CLK);
  // Serial.print("entering function");
  // Serial.println(state);
  // Serial.println(lastState);

  if (currentStateCLK != lastState && currentStateCLK == 1)
  {
    // Serial.print("changing counter value");
    if(digitalRead(DT) != currentStateCLK)
    {
      // Serial.print("counter++ CW");
      counter++;
    }
    else
    {
      // Serial.print("counter-- CCW");
      counter--;
    }
  }
  Serial.println(counter);
  lastState = currentStateCLK;
  return;
}