// Control LED Ring
// listen to estop button
// listen to power button, control power system on same button, control power button LED


// Power Button Pins
#define POWERPIN 22
#define POWERLED 23

// ESTOP Pins
#define ESTOPC 18
#define ESTOPNO 19
#define ESTOPNC 21

// NeoPixel Ring
#define LEDPIN 2
#include <Adafruit_NeoPixel.h>
#define NUMPIXELS 16               // Number of LEDs in your NeoPixel ring
#define NUM_SPOKES 2               // Number of spokes
#define TRAIL_LENGTH 7             // Length of the fading trail behind each spoke
#define UPDATE_INTERVAL 50         // Time in milliseconds between updates
unsigned long previousMillis = 0;  // will store last time LED was updated
float position = 0;                // Use a floating-point for sub-pixel position accuracy
float breathingPeriod = 3000;      // Breathing cycle length in milliseconds (3 seconds for a full cycle)
Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);


// Included just to turn off power to wifi
#include <WiFi.h>
bool connected = false;

void setup() {
  // put your setup code here, to run once:

  pinMode(POWERLED, OUTPUT);
  digitalWrite(POWERLED, HIGH);
  pinMode(POWERPIN, INPUT_PULLUP);

  pinMode(ESTOPC, OUTPUT);
  digitalWrite(ESTOPC, HIGH);
  pinMode(ESTOPNO, INPUT_PULLDOWN);
  pinMode(ESTOPNC, INPUT_PULLDOWN);

  Serial.begin(115200);
  analogReadResolution(12);

  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)

  WiFi.mode(WIFI_OFF);
  btStop();

  delay(1000);  //allow system to settle
}

void update_connection_status() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        if (input == "CONNECTED") {
            connected = true;
            // Serial.println("Connected to ROS");  // Acknowledge in the serial monitor
        } else if (input == "DISCONNECTED") {
            connected = false;
            // Serial.println("Disconnected from ROS");  // Acknowledge in the serial monitor
        }
    }
}

void handle_leds() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > UPDATE_INTERVAL) {
        pixels.clear();  // Clear all pixels for fresh update
        previousMillis = currentMillis;  // Update the last time LEDs were refreshed

        if (digitalRead(ESTOPNO) || !digitalRead(ESTOPNC)) {
            // Emergency stop button is activated
            for (int spoke = 0; spoke < NUM_SPOKES; spoke++) {
                float spokePosition = fmod((position + spoke * (NUMPIXELS / (float)NUM_SPOKES)), NUMPIXELS);
                for (int trail = 0; trail < TRAIL_LENGTH; trail++) {
                    float trailPosition = spokePosition - trail;
                    if (trailPosition < 0) trailPosition += NUMPIXELS;

                    int pixelIndex = (int)trailPosition % NUMPIXELS;
                    float brightness = (255.0 * (1.0 - (float)trail / TRAIL_LENGTH));

                    pixels.setPixelColor(pixelIndex, pixels.Color((int)brightness, 0, 0));  // Red color for emergency
                }
            }
            Serial.println("Stopped");  // Send "Stopped" state to Python
        } else {
            // No emergency stop
            if (connected) {  // Check if connected to robot
                for (int i = 0; i < NUMPIXELS; i++) {
                    pixels.setPixelColor(i, pixels.Color(0, 255, 0));  // Green color for normal operation
                }
                Serial.println("Normal");  // Send "Normal" state to Python
            } else {
                // Not connected to robot
                for (int i = 0; i < NUMPIXELS; i++) {
                    float cyclePosition = (float)((currentMillis % (unsigned long)breathingPeriod) / breathingPeriod);
                    pixels.setPixelColor(i, pixels.Color(0, 0, 255 * (sin(cyclePosition * 2 * PI) + 1) / 2));  // Blue color to indicate disconnected
                }
                Serial.println("Disconnected");  // Send "Disconnected" state to Python if needed
            }
        }
        pixels.show();  // Update the strip to show the new colors
        position += 0.5;  // Increment position for smoother motion
        if (position >= NUMPIXELS) position -= NUMPIXELS;
    }
}

void handle_powerbutton() {

  if (!digitalRead(POWERPIN)) {  // Power Button Pressed
    Serial.printf("SHUTDOWN");
    pinMode(POWERPIN, OUTPUT);
    digitalWrite(POWERPIN, LOW);
    for (int i = 0; i <= 12; i++) {
      digitalWrite(POWERLED, HIGH);
      delay(255);
      digitalWrite(POWERLED, LOW);
      delay(255);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  int analogVolts = analogReadMilliVolts(34);
  //Serial.printf("ADC millivolts value = %d\n", analogVolts);
  // less than 500 when charged, above 2000 when discharged
  if (analogVolts >= 2000) {
    Serial.printf("Battery Low");
  }

  update_connection_status();
  handle_leds();
  handle_powerbutton();
}
