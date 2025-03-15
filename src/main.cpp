#include <Arduino.h>
#define right_encoder_button 0
#define right_encoder_channelA 1
#define right_encoder_channelB 2

int right_encoder_aLastState = 0;

// put function declarations here:
int myFunction(int, int);


void setup() {
  //Setting up right encoder
  pinMode(right_encoder_button, INPUT);
  pinMode(right_encoder_channelA, INPUT);
  pinMode(right_encoder_channelB, INPUT);

  int right_encoder_aLastState = 0;

  Serial.begin (9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  right_encoder_aLastState = digitalRead(right_encoder_channelA);   
  Serial.println(right_encoder_aLastState);

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}