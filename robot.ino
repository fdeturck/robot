#include <AFMotor.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

#define TRIGGER_PIN  14
#define ECHO_PIN     15
#define BT_RX_PIN 16
#define BT_TX_PIN 17
#define BTN_PIN 13
#define IR_PIN 12
#define LED_PINRED 9
#define LED_PIN 2

#define MAX_DISTANCE 200
#define MIN_DISTANCE 15

NewPing DistanceSensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

AF_DCMotor Motor1(3);
AF_DCMotor Motor2(1);

int ledState = LOW;
const int ledPin =  LED_PIN;
const int ledPinRed =  LED_PINRED;

int bAvance = 0;

const int buttonPin =  BTN_PIN;
int buttonState = 0;

const int irPin =  IR_PIN;
int irState = 0;

unsigned long previousMillis = 0;
long interval = 1000;

int incomingByte = 0;   // for incoming BTserial data

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPinRed, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(irPin,INPUT);
  delay(500);
}

void loop()
{
  if (BTSerial.available() > 0) {
                // read the incoming byte:
                incomingByte = BTSerial.read();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
                BTSerial.print("I received: ");
                BTSerial.println(incomingByte, DEC);
        }
    
  buttonState = digitalRead(buttonPin);
  irState = digitalRead(irPin);
  if ((buttonState == HIGH) or (incomingByte==0) or (irState==0)){
    // Attente
    interval = 3000;
    blink(1);
    if (bAvance == 1)
    {
      Serial.println("* STOP");
      Motor1.setSpeed(0);
      Motor2.setSpeed(0);
      Motor1.run(BRAKE);
      Motor2.run(BRAKE);
      bAvance = 0;
      digitalWrite(ledPinRed, LOW);
    }
    return;
  }
  else {
    // Go
    interval = 500;
    blink(0);
  }

  //avance
  if (bAvance == 0)
  {
    digitalWrite(ledPinRed, LOW);
    Serial.println("* AVANCE");
    BTSerial.println("* AVANCE");
    Motor1.setSpeed(200);
    Motor2.setSpeed(200);
    Motor1.run(FORWARD);
    Motor2.run(FORWARD);
    bAvance = 1;
  }
  //test distance
  delay(400);
  unsigned int cm = DistanceSensor.ping_cm();
  irState = digitalRead(irPin);
  Serial.print("Distance: ");
  BTSerial.print("Distance: ");
  Serial.print(cm);
  BTSerial.print(cm);
  Serial.println("cm");
  BTSerial.println("cm");
  //erreur de mesure
  if (cm == 0)
    return;


  if (((cm <= MIN_DISTANCE) && (bAvance == 1))||(irState==1))
  {
    //stop
    bAvance = 0;
    digitalWrite(ledPinRed, HIGH);
    Serial.println("* STOP");
    BTSerial.println("* STOP");
    Motor1.setSpeed(0);
    Motor2.setSpeed(0);
    Motor1.run(BRAKE);
    Motor2.run(BRAKE);
    //recule
    Serial.println("*  RECULE");
    BTSerial.println("*  RECULE");
    delay(1000);
    Motor1.setSpeed(255);
    Motor2.setSpeed(255);
    Motor1.run(BACKWARD);
    Motor2.run(BACKWARD);
    // demi tour
    Serial.println("*  DEMI-TOUR");
    BTSerial.println("*  DEMI-TOUR");
    delay(2000);
    Motor1.setSpeed(150);
    Motor2.setSpeed(150);
    Motor1.run(BACKWARD);
    Motor2.run(FORWARD);
    delay(2000);
  }

}

void blink(int echo)
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    if (echo == 1)
    { Serial.println("BLINK");
      BTSerial.println("BLINK");
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}
