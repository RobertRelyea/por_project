#define FLYWHEEL_PIN 5
#define FEEDER_PIN 6

bool spun_up = false;
bool prime_trigger = false, fire_trigger = false;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FLYWHEEL_PIN, OUTPUT);
  pinMode(FEEDER_PIN, OUTPUT);
  pinMode(4, INPUT);
}

// the loop function runs over and over again forever
void loop() {

  prime_trigger = digitalRead(4) == HIGH;

  if(prime_trigger)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    if(!spun_up)
      revFlywheel(170);
    analogWrite(FEEDER_PIN, 150);
    delay(125);
    analogWrite(FEEDER_PIN, 0);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    haltFlywheel();
  }
  /*
  revFlywheel(123);

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  analogWrite(FEEDER_PIN, 123);
  delay(2500);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  analogWrite(FEEDER_PIN, 0);

  analogWrite(FLYWHEEL_PIN, 0);
  delay(10000);*/
}

void revFlywheel(int spd)
{
  analogWrite(FLYWHEEL_PIN, spd);
  delay(400);
  spun_up = true;
}

void haltFlywheel()
{
  analogWrite(FLYWHEEL_PIN, 0);
  spun_up = false;
}
