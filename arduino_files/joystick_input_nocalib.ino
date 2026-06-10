const int VRX_1 = A0;
const int VRY_1 = A1;
const int VRX_2 = A2;
const int VRY_2 = A3;

float centerX_1 = 0;
float centerY_1 = 0;
float centerX_2 = 0;
float centerY_2 = 0;

const float DEADZONE = 0.05;

void setup() {
  Serial.begin(9600);

  delay(1000); // time to open serial monitor

  Serial.println("Calibrating... keep joystick still");

  const int samples = 100;
  float sumX_1 = 0;
  float sumY_1 = 0;

  float sumX_2 = 0;
  float sumY_2 = 0;

  for (int i = 0; i < samples; i++) {
    int x_1 = analogRead(VRX_1);
    int y_1 = analogRead(VRY_1);

    int x_2 = analogRead(VRX_2);
    int y_2 = analogRead(VRY_2);

    float nx_1 = (x_1 - 512) / 512.0;
    float ny_1 = (y_1 - 512) / 512.0;

    float nx_2 = (x_2 - 512) / 512.0;
    float ny_2 = (y_2 - 512) / 512.0;

  

    sumX_1 += nx_1;
    sumY_1 += ny_1;
    sumX_2 += nx_2;
    sumY_2 += ny_2;

    delay(10);
  }

  centerX_1 = sumX_1 / samples;
  centerY_1 = sumY_1 / samples;
  centerX_2 = sumX_2 / samples;
  centerY_2 = sumY_2 / samples;

  Serial.print("First Center X: ");
  Serial.println(centerX_1, 4);
  Serial.print("First Center Y: ");
  Serial.println(centerY_1, 4);

  Serial.print("Second Center X: ");
  Serial.println(centerX_2, 4);
  Serial.print("Second Center Y: ");
  Serial.println(centerY_2, 4);

  Serial.println("Calibration done");
}

float applyDeadzone(float v) {
  if (abs(v) < DEADZONE) return 0;
  return v;
}

void loop() {
  int x_1 = analogRead(VRX_1);
  int y_1 = analogRead(VRY_1);

  int x_2 = analogRead(VRX_2);
  int y_2 = analogRead(VRY_2);

  // normalize raw values to [-1, 1]
  float nx_1 = (x_1 - 512) / 512.0;
  float ny_1 = (y_1 - 512) / 512.0;

  float nx_2 = (x_2 - 512) / 512.0;
  float ny_2 = (y_2 - 512) / 512.0;

  // subtract center offset + rescale
  nx_1 = (nx_1 - centerX_1) / (1.0 - abs(centerX_1));
  ny_1 = (ny_1 - centerY_1) / (1.0 - abs(centerY_1));

  nx_2 = (nx_2 - centerX_2) / (1.0 - abs(centerX_2));
  ny_2 = (ny_2 - centerY_2) / (1.0 - abs(centerY_2));

  // deadzone
  nx_1 = applyDeadzone(nx_1);
  ny_1 = applyDeadzone(ny_1);
  nx_2 = applyDeadzone(nx_2);
  ny_2 = applyDeadzone(ny_2);

  // clamp to [-1, 1]
  nx_1 = constrain(nx_1, -1.0, 1.0);
  ny_1 = constrain(ny_1, -1.0, 1.0);
  nx_2 = constrain(nx_2, -1.0, 1.0);
  ny_2 = constrain(ny_2, -1.0, 1.0);

  // output (CSV format for Python)
  Serial.print(nx_1, 3);
  Serial.print(nx_1, 3);
  Serial.print(",");
  Serial.println(ny_1, 3);
  Serial.print(nx_2, 3);
  Serial.print(",");
  Serial.println(ny_2, 3);

  delay(20); // ~50Hz
}