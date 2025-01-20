#define ENC_COUNT_REV 600

#define ENC_IN_RIGHT_A 2
#define ENC_IN_RIGHT_B 4

boolean Direction_right = true;
volatile long right_wheel_pulse_count = 0;
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

float rpm_right = 0;
float ang_velocity_right = 0;
float velocity_mm_per_s = 0;

const float rpm_to_radians = 0.10471975512;
const float SHAFT_RADIUS = 2.0; 

void setup() {
  Serial.begin(9600); 

  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);

    ang_velocity_right = rpm_right * rpm_to_radians;

    velocity_mm_per_s = ang_velocity_right * SHAFT_RADIUS;

    // In thông tin qua Serial
    Serial.print(" Pulses: ");
    Serial.println(right_wheel_pulse_count);
    Serial.print(" Speed: ");
    Serial.print(velocity_mm_per_s);
    Serial.println(" mm/s");

    right_wheel_pulse_count = 0;
  }
}

void right_wheel_pulse() {
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false; 
  } else {
    Direction_right = true;
  }

  if (Direction_right) {
    right_wheel_pulse_count++;
  } else {
    right_wheel_pulse_count--;
  }
}