//for å inpute til serial monitor på vs, klikk ctrl + shift + P
// velgt arduino: send text ti serial

char inByte;
const int yellowLed = 11;
const int greenLed = 9;
const int blueLed = 10;

void setup()
{
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  Serial.begin(9600);
  Serial.println("Med dette programmet kan du styre 3 LED's");
  Serial.println("Y - slår på rød LED");
  Serial.println("G - slår på grønn LED");
  Serial.println("B - slår på blå LED");
  Serial.println("O - slår av alle LED's");
}
void loop()
{
  
  if(Serial.available() > 0) {
    inByte = Serial.read(); // ja - les det inn i inByte
  
  // tester på inByte hva tegnet var:
  switch(inByte){
    case 'Y': case 'y': //sjekker både stor og liten "r"
    digitalWrite(yellowLed, HIGH);
    break;
    
    case 'G': case 'g':
    digitalWrite(greenLed, HIGH);
    break;
    
    case 'B': case 'b':
    digitalWrite(blueLed, HIGH);
    break;
    
    case 'O': case 'o':
    digitalWrite(yellowLed, LOW);
    digitalWrite(greenLed, LOW);
    digitalWrite(blueLed, LOW);
    break;
    
    default:
    Serial.println("Ugyldig kommando!");
  	}
  }
  
  /*Serial.print(inByte);
  delay(1500);*/
  
}
