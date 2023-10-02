#include <WiFi.h>
#include <ESPAsyncE131.h>
#include <FastLED.h>
#include <SD.h>
#include <FS.h>
#include <SPI.h>
#include <Audio.h>

const char* ssid = "Beezernet2.0";
const char* password = "Oliver2729$";G

// I2S Connections
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26
 
 // Create Audio object
Audio audio;
 

// Number of DMX universes and channels per universe
#define UNIVERSE_COUNT 2
#define CHANNELS_PER_UNIVERSE 510
#define CS_PIN 5  // Chip Select Pin for SD Card

// Universe settings
const uint16_t universes[UNIVERSE_COUNT] = {1, 2};

// NeoPixel settings
#define PIXEL_PIN 16
#define PIXEL_COUNT 256
#define RELAY_PIN 2
#define BUTTON_PIN 4

// on off led settings 
const int ledPin = 13;
const int pwmChannel = 0;  // You can choose any channel from 0-15
const int freq = 5000;     // Frequency for PWM signal
const int resolution = 8;  // Resolution in bits for PWM signal (e.g., 8 bits gives values from 0 to 255)
int pwmDutyCycle = 64;  

bool relayState = true;

// E1.31 instance
ESPAsyncE131 e131(UNIVERSE_COUNT);

// FastLED strip instance
CRGB leds[PIXEL_COUNT];

void writeLEDsToSD() {
  File file = SD.open("/leds.bin", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file.write((uint8_t *)leds, PIXEL_COUNT * sizeof(CRGB));
  file.close();
}

void readLEDsFromSD() {
  File file = SD.open("/leds.bin", FILE_READ);
  if (!file || file.size() != PIXEL_COUNT * sizeof(CRGB)) {
    Serial.println("Failed to open file for reading or incorrect file size");
    // You could initialize leds to a default state here if desired
    memset(leds, 0, sizeof(leds)); // Set all LEDs to off
    return;
  }

  file.read((uint8_t *)leds, PIXEL_COUNT * sizeof(CRGB));
  file.close();
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize E1.31
  if (e131.begin(E131_UNICAST)) {
    Serial.println("E1.31 listener started");
  } else {
    Serial.println("Error starting E1.31 listener");
    while (1);
  }

  // Configure the PWM functionality
  ledcSetup(pwmChannel, freq, resolution);
  // Attach the LED pin to the PWM channel
  ledcAttachPin(ledPin, pwmChannel);
  // Write the initial PWM value
  ledcWrite(pwmChannel, pwmDutyCycle);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN, HIGH);
  

  // Initialize FastLED strip
  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(leds, PIXEL_COUNT);
  FastLED.setBrightness(85);
  if (!SD.begin(CS_PIN)) {
    Serial.println("Card Mount Failed");
    return;
  }

    // Setup I2S 
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  // Set Volume
  audio.setVolume(25);
    
  // Open music file
  // audio.connecttoFS(SD,"/ring.mp3");
  
  readLEDsFromSD(); // Load saved LED data from SD card
  FastLED.show();
}

void loop() {
  // Process incoming E1.31 packets
  e131_packet_t packet;
  audio.loop();
  
  while (!e131.isEmpty()) {
    e131.pull(&packet);
    uint16_t incoming_universe = htons(packet.universe);
    uint16_t num_channels = htons(packet.property_value_count)-1;

    // Find the index of the incoming universe in our universes array
    int universe_index = -1;
    for (uint8_t i = 0; i < UNIVERSE_COUNT; i++) {
      if (universes[i] == incoming_universe) {
        universe_index = i;
        break;
      }
    }

    if (universe_index >= 0) {
      // Read pixel data from the E1.31 packet and update FastLED strip
      for (uint16_t i = 0; i < num_channels / 3; i++) {
        leds[i + (universe_index * (CHANNELS_PER_UNIVERSE / 3))] = CRGB(
          packet.property_values[1 + i * 3],      // Red
          packet.property_values[1 + i * 3 + 1],  // Green
          packet.property_values[1 + i * 3 + 2]   // Blue
        );
      }
      // Update the LEDs
      
      FastLED.show();
    }
  }
  // Read the button state (LOW when pressed due to pull-up resistor)
  int buttonState = digitalRead(BUTTON_PIN);

  // Check if the button was pressed
  if (buttonState == LOW) {
    relayState = !relayState; // Toggle the relay state
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW); // Set the relay pin according to the new state
  //    writeLEDsToSD(); // Save current LED data to SD card
  //    readLEDsFromSD(); // Load saved LED data from SD card
  //    FastLED.show();  // Show the loaded LED data
    // Open music file
    audio.connecttoFS(SD,"/ring.mp3");
    delay(500);
  }
}
