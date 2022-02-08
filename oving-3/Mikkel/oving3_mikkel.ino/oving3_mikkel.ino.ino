#include <ezButton.h>

ezButton button1(2);  // create ezButton object that attach to pin 6;
ezButton button2(3);  // create ezButton object that attach to pin 7;

const int red_light_pin = 9;
const int green_light_pin = 10;
const int blue_light_pin = 11;
const int buz = 5;
const int redWinner = 6;
const int blueWinner = 7;



void setup() {
  button1.setDebounceTime(10); // set debounce time to 50 milliseconds
  button2.setDebounceTime(10); // set debounce time to 50 milliseconds
  pinMode(redWinner, OUTPUT);
  pinMode(blueWinner, OUTPUT);
  Serial.begin(9600);
  randomSeed(analogRead(A0));
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}

void winSound() {
  for (int i = 750; i < 1500; i++) {
    tone(buz, i);
    delay(1);
  }
  noTone(buz);

}

void loseSound() {
  for (int i = 0; i < 750; i++) {
    tone(buz, 200);
    delay(1);
  }
  noTone(buz);
}


void loop() {

  unsigned long startTime = millis();
  RGB_color(255, 0, 0); // Red
  bool goGreen = true;
  bool gameOn = true;

  unsigned long rndTime = random(3000, 6000);
  while (millis() - startTime < rndTime) {
    button1.loop(); // MUST call the loop() function first
    button2.loop(); // MUST call the loop() function first
    if (button1.isPressed()) {
      if (gameOn) {
        digitalWrite(redWinner, LOW);
        digitalWrite(blueWinner, HIGH);
        goGreen = false;
        gameOn = false;
        break;
      }

    }
    if (button2.isPressed()) {
      if (gameOn) {
        digitalWrite(redWinner, HIGH);
        digitalWrite(blueWinner, LOW);
        goGreen = false;
        gameOn = false;
        break;
      }
    }
  }


  while (goGreen && gameOn) {
    RGB_color(0, 255, 0); // Green
    button1.loop(); // MUST call the loop() function first
    button2.loop(); // MUST call the loop() function first

    if (button1.isPressed()) {
      if (gameOn) {
        digitalWrite(redWinner, HIGH);
        digitalWrite(blueWinner, LOW);
        gameOn = false;
        break;
      }
    }
    if (button2.isPressed()) {
      if (gameOn) {
        digitalWrite(blueWinner, HIGH);
        digitalWrite(redWinner, LOW);
        gameOn = false;
        break;
      }
    }
  }
  if (goGreen) {
    winSound();
  }
  else {
    loseSound();
  }

  delay(1000);
}
