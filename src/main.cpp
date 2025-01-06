#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Button2.h>

// BLE UUIDs
#define COORDINATES_SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define COORDINATES_CHAR_UUID "abcd1234-5678-9abc-def0-1234567890ab"

#define ZOOM_SERVICE_UUID "87654321-4321-4321-4321-9876543210ab"
#define ZOOM_CHAR_UUID "fedcba98-7654-3210-0987-654321fedcba"

#define MODE_SERVICE_UUID "56781234-1234-1234-1234-1234567890ab"
#define MODE_CHAR_UUID "dcba1234-5678-9abc-def0-1234567890ab"

// Potentiometer pin
  #define POTENTIOMETER_PIN 15
  //Filter size for potentiometer data
  #define FILTER_SIZE 10 

  //Rotary Encoder pins
  #define ROTARY_PIN_CLK 12
  #define ROTARY_PIN_DT 13
  #define ROTARY_PIN_SW 14


//Resistive Touch pins
#define X1_PIN 32 // Positive X
#define X2_PIN 33 // Negative X
#define Y1_PIN 25 // Positive Y
#define Y2_PIN 26 // Negative Y

int xThreshold = 450;  // Threshold for X coordinate
int yThreshold = 450;  // Threshold for Y coordinate
int filterSize = 10;   // Number of samples for the moving average filter

int xReadings[10]; // Array to store X readings
int yReadings[10]; // Array to store Y readings
int xIndex = 0;    // Current index for X readings
int yIndex = 0;    // Current index for Y readings

// Rotary Encoder variables
volatile int mode = 0;           // Mode value (0 to 10)
volatile int lastStateCLK;       // Last state of the CLK pin
volatile int stepCounter = 0;    // Tracks steps for threshold
const int stepThreshold = 3;     // Number of steps required to change the mode
const int minMode = 0;           // Minimum mode value
const int maxMode = 10;          // Maximum mode value

//filter for potentiometer data
int filterPotentiometerData(int newValue) {
  static int filterBuffer[FILTER_SIZE] = {0};
  static int index = 0;
  static int sum = 0;

  sum -= filterBuffer[index];

  filterBuffer[index] = newValue;
  sum += newValue;

  index = (index + 1) % FILTER_SIZE;

  return sum / FILTER_SIZE;
}

//Read the X coordinate
int readX() {
  pinMode(Y1_PIN, INPUT_PULLDOWN);  // Enable internal pull-down resistor
  pinMode(Y2_PIN, INPUT_PULLDOWN);  // Enable internal pull-down resistor
  pinMode(X1_PIN, OUTPUT);
  pinMode(X2_PIN, OUTPUT);

  digitalWrite(X1_PIN, HIGH);
  digitalWrite(X2_PIN, LOW);

  int value = analogRead(Y2_PIN);
  
  // If the value is below the threshold, consider it as no touch
  if (value < xThreshold) {
    return -1;  // No touch
  }

  return value;
}

// Function to read Y coordinate
int readY() {
  pinMode(X1_PIN, INPUT_PULLDOWN);  // Enable internal pull-down resistor
  pinMode(X2_PIN, INPUT_PULLDOWN);  // Enable internal pull-down resistor
  pinMode(Y1_PIN, OUTPUT);
  pinMode(Y2_PIN, OUTPUT);

  digitalWrite(Y1_PIN, HIGH);
  digitalWrite(Y2_PIN, LOW);

  int value = analogRead(X2_PIN);
  
  // If the value is below the threshold, consider it as no touch
  if (value < yThreshold) {
    return -1;  // No touch
  }

  return value;
}

int getAverageX(int newX) {
  xReadings[xIndex] = newX;
  xIndex = (xIndex + 1) % filterSize;

  int sumX = 0;
  for (int i = 0; i < filterSize; i++) {
    sumX += xReadings[i];
  }
  return sumX / filterSize;
}

// Function to update and calculate the moving average for Y readings
int getAverageY(int newY) {
  yReadings[yIndex] = newY;
  yIndex = (yIndex + 1) % filterSize;

  int sumY = 0;
  for (int i = 0; i < filterSize; i++) {
    sumY += yReadings[i];
  }
  return sumY / filterSize;
}

// Rotary Encoder interrupt service routine
void IRAM_ATTR handleRotation() {
  int currentStateCLK = digitalRead(ROTARY_PIN_CLK);
  if (currentStateCLK != lastStateCLK && currentStateCLK == HIGH) {
    int stateDT = digitalRead(ROTARY_PIN_DT);
    if (stateDT != currentStateCLK) {
      stepCounter++; // Increment step counter for clockwise
    } else {
      stepCounter--; // Decrement step counter for counterclockwise
    }

    // Handle clockwise rotation
    if (stepCounter >= stepThreshold) {
      if (mode < maxMode) {
        mode++;
        Serial.printf("Clockwise - Mode: %d\n", mode);
      }
      stepCounter = 0; // Reset step counter
    }

    // Handle counterclockwise rotation
    if (stepCounter <= -stepThreshold) {
      if (mode > minMode) {
        mode--;
        Serial.printf("Counterclockwise - Mode: %d\n", mode);
      }
      stepCounter = 0; // Reset step counter
    }
  }
  lastStateCLK = currentStateCLK;
}

BLECharacteristic *coordinatesCharacteristic;
BLECharacteristic *zoomCharacteristic;
BLECharacteristic *modeCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    pServer->startAdvertising();
  }
};

void zoomTask(void *parameter) {
  int previousZoomValue = 0;

  while (true) {
    if (deviceConnected) {
      int potValue = analogRead(POTENTIOMETER_PIN);
      // Serial.print("> Potentiometer value:");
      // Serial.println(potValue); 
      int filteredPotValue = filterPotentiometerData(potValue);
      int zoomValue = map(filteredPotValue, 0, 4095, 0, 200);
      if (abs(zoomValue - previousZoomValue) > 2) {
        String message = String("{\"zoom\": ") + String(zoomValue) + "}";

        zoomCharacteristic->setValue(message.c_str());
        zoomCharacteristic->notify();

        previousZoomValue = zoomValue;

        Serial.println(message);
      }
    } else {
      Serial.println("Device not connected - Zoom task idle"); // Debugging output
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


void modeTask(void *parameter) {
 int lastMode = -1; // Tracks the last mode value
  while (true) {
    if (deviceConnected) {
      if (mode != lastMode) {
        String message = String("{\"mode\": ") + String(mode) + "}";
        modeCharacteristic->setValue(message.c_str());
        modeCharacteristic->notify();
        lastMode = mode;
        Serial.println(message);
      }
    } else {
      Serial.println("Device not connected - Mode task idle"); // Debugging output
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void touchPanelTask(void *parameter) {
  while (true) {
    if (deviceConnected) {
      // Read raw X and Y values
      int rawX = readX();
      int rawY = readY();

      // Only proceed if both X and Y values are valid (touched)
      if (rawX != -1 && rawY != -1) {
        // Apply moving average filter for X and Y
        int smoothedX = getAverageX(rawX);  // Apply moving average filter for X
        int smoothedY = getAverageY(rawY);  // Apply moving average filter for Y

        // Map the smoothed values to pixel coordinates
        int pixelX = map(smoothedX, 450, 3900, 0, 700); // Map X value to screen width
        int pixelY = map(smoothedY, 450, 3300, 0, 400); // Map Y value to screen height

        // Create the JSON message for X, Y, and Pixel values
        String message = String("{\"X\":") + pixelX + String(",\"Y\":") + pixelY + String("}");


        // Set the value and notify the characteristic
        coordinatesCharacteristic->setValue(message.c_str());
        coordinatesCharacteristic->notify();

        // Print the message to the Serial Monitor for debugging
        Serial.println(message);
      } else {
        // If not touched, print "Not touched"
        String message = String("{\"X\":-1,\"Y\":-1}");
        coordinatesCharacteristic->setValue(message.c_str());
        coordinatesCharacteristic->notify();

      }
    } else {
      // If device is not connected, print idle message
      Serial.println("Device not connected - Touch panel task idle");
    }

    // Delay to prevent task from running too fast
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


void setup() {
  pinMode(POTENTIOMETER_PIN, INPUT);
    
  pinMode(ROTARY_PIN_CLK, INPUT);
  pinMode(ROTARY_PIN_DT, INPUT);
  
  Serial.begin(115200);

  // Attach interrupt for rotary encoder
  lastStateCLK = digitalRead(ROTARY_PIN_CLK);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_CLK), handleRotation, CHANGE);

  // Initialize BLE
  BLEDevice::init("ESP32 BLE Device");

  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Coordinate Service
  BLEService *coordinateService = pServer->createService(COORDINATES_SERVICE_UUID);
  coordinatesCharacteristic = coordinateService->createCharacteristic(
    COORDINATES_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  coordinatesCharacteristic->addDescriptor(new BLE2902());
  coordinateService->start();
  Serial.println("Coordinate Service started");

  // Zoom Service
  BLEService *zoomService = pServer->createService(ZOOM_SERVICE_UUID);
  zoomCharacteristic = zoomService->createCharacteristic(
    ZOOM_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  zoomCharacteristic->addDescriptor(new BLE2902());
  zoomService->start();
  Serial.println("Zoom Service started");

  // Mode Service
  BLEService *modeService = pServer->createService(MODE_SERVICE_UUID);
  modeCharacteristic = modeService->createCharacteristic(
    MODE_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  modeCharacteristic->addDescriptor(new BLE2902());
  modeService->start();
  Serial.println("Mode Service started");

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(COORDINATES_SERVICE_UUID);
  pAdvertising->addServiceUUID(ZOOM_SERVICE_UUID);
  pAdvertising->addServiceUUID(MODE_SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("Waiting for client to connect...");

  // // Create tasks
  // xTaskCreate(zoomTask, "zoomTask", 15000, NULL, 1, NULL);
  // xTaskCreate(modeTask, "modeTask", 15000, NULL, 1, NULL);
  // xTaskCreate(touchPanelTask, "touchPanelTask", 15000, NULL, 1, NULL);


  xTaskCreatePinnedToCore(zoomTask, "zoomTask", 15000, NULL, 1, NULL, 0);  // Pin to core 0
  xTaskCreatePinnedToCore(modeTask, "modeTask", 15000, NULL, 1, NULL, 0);  // Pin to core 0
  xTaskCreatePinnedToCore(touchPanelTask, "touchPanelTask", 15000, NULL, 1, NULL, 1);  // Pin to core 1
}

void loop() {
}
