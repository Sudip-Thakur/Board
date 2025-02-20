#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Button2.h>

// BLE UUIDs
#define COORDINATES_SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define COORDINATES_CHAR_UUID "abcd1234-5678-9abc-def0-1234567890ab"

#define ZOOM_SERVICE_UUID "87654321-4321-4321-4321-9876543210ac"
#define ZOOM_CHAR_UUID "fedcba98-7654-3210-0987-654321fedcbb"

#define MODE_SERVICE_UUID "56781234-1234-1234-1234-1234567890ac"
#define MODE_CHAR_UUID "dcba1234-5678-9abc-def0-1234567890ac"

#define REDO_UNDO_SERVICE_UUID "98765432-4321-4321-4321-1234567890ab"
#define REDO_UNDO_CHAR_UUID "efdcba98-7654-3210-0987-654321fedcbb"

BLECharacteristic *coordinatesCharacteristic;
BLECharacteristic *zoomCharacteristic;
BLECharacteristic *modeCharacteristic;
BLECharacteristic *redoUndoCharacteristic;

bool deviceConnected = false;

// Potentiometer pin
#define POTENTIOMETER_PIN 15

// Filter size for potentiometer data
#define FILTER_SIZE 10

// Rotary Encoder pins
#define ROTARY_PIN_CLK 12
#define ROTARY_PIN_DT 13
#define ROTARY_PIN_SW 14

// REDO UNDO
#define REDO_PIN 22
#define UNDO_PIN 23

// Resistive Touch pins
#define X1_PIN 32 // Positive X
#define X2_PIN 33 // Negative X
#define Y1_PIN 25 // Positive Y
#define Y2_PIN 26 // Negative Y

int xThreshold = 700; // Threshold for X coordinate
int yThreshold = 700; // Threshold for Y coordinate
int filterSize = 3;   // moving average filter

int xReadings[3];
int yReadings[3];
int xIndex = 0;
int yIndex = 0;

// Rotary Encoder variables
volatile int mode = 0;
volatile int lastStateCLK;
volatile int stepCounter = 0;
const int stepThreshold = 1;
const int minMode = 0;
const int maxMode = 7;
int lastMode = -1;
unsigned long lastDebounceTime = 0;

// filter for potentiometer data
int filterPotentiometerData(int newValue)
{
  static int filterBuffer[FILTER_SIZE] = {0};
  static int index = 0;
  static int sum = 0;

  sum -= filterBuffer[index];

  filterBuffer[index] = newValue;
  sum += newValue;

  index = (index + 1) % FILTER_SIZE;

  return sum / FILTER_SIZE;
}

// Read the X coordinate
int readX()
{
  pinMode(Y1_PIN, INPUT_PULLDOWN); // Enable internal pull-down resistor
  pinMode(Y2_PIN, INPUT_PULLDOWN); // Enable internal pull-down resistor
  pinMode(X1_PIN, OUTPUT);
  pinMode(X2_PIN, OUTPUT);

  digitalWrite(X1_PIN, HIGH);
  digitalWrite(X2_PIN, LOW);

  int value = analogRead(Y2_PIN);

  if (value < xThreshold)
  {
    return -1; // No touch
  }

  return value;
}

// Function to read Y coordinate
int readY()
{
  pinMode(X1_PIN, INPUT_PULLDOWN);
  pinMode(X2_PIN, INPUT_PULLDOWN);
  pinMode(Y1_PIN, OUTPUT);
  pinMode(Y2_PIN, OUTPUT);

  digitalWrite(Y1_PIN, HIGH);
  digitalWrite(Y2_PIN, LOW);

  int value = analogRead(X2_PIN);

  if (value < yThreshold)
  {
    return -1; // No touch
  }

  return value;
}

int getAverageX(int newX)
{
  xReadings[xIndex] = newX;
  xIndex = (xIndex + 1) % filterSize;

  int sumX = 0;
  for (int i = 0; i < filterSize; i++)
  {
    sumX += xReadings[i];
  }
  return sumX / filterSize;
}

int getAverageY(int newY)
{
  yReadings[yIndex] = newY;
  yIndex = (yIndex + 1) % filterSize;

  int sumY = 0;
  for (int i = 0; i < filterSize; i++)
  {
    sumY += yReadings[i];
  }
  return sumY / filterSize;
}

void IRAM_ATTR handleRotation()
{
  const unsigned long debounceDelay = 2; // 5ms debounce time
  int currentStateCLK = digitalRead(ROTARY_PIN_CLK);

  if ((millis() - lastDebounceTime) > debounceDelay) // Debounce Check
  {
    if (currentStateCLK != lastStateCLK && currentStateCLK == HIGH)
    {
      int stateDT = digitalRead(ROTARY_PIN_DT);
      if (stateDT != currentStateCLK)
      {
        stepCounter++; // Clockwise
      }
      else
      {
        stepCounter--; // Counterclockwise
      }

      if (stepCounter >= stepThreshold)
      {
        mode++;             // Increment mode
        if (mode > maxMode) // Wrap around to 0 if mode exceeds maxMode (7)
        {
          mode = minMode;
        }
        stepCounter = 0;
      }

      if (stepCounter <= -stepThreshold)
      {
        mode--;             // Decrement mode
        if (mode < minMode) // Wrap around to maxMode if mode is less than minMode (0)
        {
          mode = maxMode;
        }
        stepCounter = 0;
      }

      // Bluetooth Mode Update (Only if changed)
      if (deviceConnected && mode != lastMode)
      {
        String message = String("{\"mode\": ") + String(mode) + "}";
        modeCharacteristic->setValue(message.c_str()); // Assuming modeCharacteristic is defined elsewhere
        modeCharacteristic->notify();
        lastMode = mode;
        Serial.println(message);
      }
    }
    lastDebounceTime = millis(); // Update debounce time
  }

  lastStateCLK = currentStateCLK;
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Device connected");
    pServer->startAdvertising();
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Device disconnected");
    pServer->startAdvertising();
  }
};

class ModeCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    mode = atoi(value.c_str());
    lastMode = mode;
    String message = String("{\"mode\": ") + String(mode) + "}";
    modeCharacteristic->setValue(message.c_str());
    modeCharacteristic->notify();
    if (value.length() > 0)
    {
      Serial.print("Mode Characteristic written: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(value[i]);
      }
      Serial.println();
    }
  }
};

void zoomTask(void *parameter)
{
  int previousZoomValue = 0;

  while (true)
  {
    if (deviceConnected)
    {
      int potValue = analogRead(POTENTIOMETER_PIN);
      // Serial.print("> Potentiometer value:");
      // Serial.println(potValue);
      int filteredPotValue = filterPotentiometerData(potValue);
      int zoomValue = map(filteredPotValue, 0, 4095, 50, 200);
      if (abs(zoomValue - previousZoomValue) > 2)
      {
        String message = String("{\"zoom\": ") + String(zoomValue) + "}";

        zoomCharacteristic->setValue(message.c_str());
        zoomCharacteristic->notify();

        previousZoomValue = zoomValue;

        Serial.println(message);
      }
    }
    else
    {
      Serial.println("Device not connected - Zoom task idle");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void modeTask(void *parameter)
{
  while (true)
  {
    if (deviceConnected)
    {
      if (mode != lastMode)
      {
        String message = String("{\"mode\": ") + String(mode) + "}";
        modeCharacteristic->setValue(message.c_str());
        modeCharacteristic->notify();
        lastMode = mode;
        Serial.println(message);
      }
    }
    else
    {
      Serial.println("Device not connected - Mode task idle");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void touchPanelTask(void *parameter)
{
  bool wait = false;
  int waitDelay = 200;
  while (true)
  {
    if (deviceConnected)
    {
      int rawX = readX();
      int rawY = readY();

      if (rawX == -1 && rawY == -1)
      {
        wait = true;
      }else {
        wait = false;
      }

      // Serial.print("X: ");
      // Serial.print(rawX);
      // Serial.print(" Y: ");
      // Serial.println(rawY);
      if (rawX != -1 && rawY != -1)
      {
        if(wait){
          vTaskDelay(waitDelay / portTICK_PERIOD_MS);
          continue;
        }
        // moving average filter
        int smoothedX = getAverageX(rawX);
        int smoothedY = getAverageY(rawY);

        // int smoothedX = rawX;
        // int smoothedY = rawY;

        // Map the values to pixel coordinates
        int pixelX = map(smoothedX, xThreshold, 3900, 0, 570); //380
        int pixelY = map(smoothedY, yThreshold, 3300, 0, 320); //220

        // Create the JSON message for X, Y, and Pixel values
        String message = String("{\"X\":") + pixelX + String(",\"Y\":") + pixelY + String("}");

        coordinatesCharacteristic->setValue(message.c_str());
        coordinatesCharacteristic->notify();

        Serial.println(message);
      }
      else
      {
        // // If not touched, print "Not touched"
        // String message = String("{\"X\":-1,\"Y\":-1}");
        // coordinatesCharacteristic->setValue(message.c_str());
        // coordinatesCharacteristic->notify();
      }
    }
    else
    {
      Serial.println("Device not connected - Touch panel task idle");
    }

    // Delay to prevent task from running too fast
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void redoUndoTask(void *parameter)
{
  const unsigned long debounceDelay = 250;
  unsigned long lastRedoPressTime = 0;
  unsigned long lastUndoPressTime = 0;

  pinMode(REDO_PIN, INPUT_PULLUP);
  pinMode(UNDO_PIN, INPUT_PULLUP);

  while (true)
  {
    if (deviceConnected)
    {
      int redoButton = digitalRead(REDO_PIN);
      int undoButton = digitalRead(UNDO_PIN);

      unsigned long currentTime = millis();

      // Check if the redo button is pressed and debounce time has passed
      if (redoButton == LOW && (currentTime - lastRedoPressTime > debounceDelay))
      {
        lastRedoPressTime = currentTime;
        String message = String("{\"action\": 1}");
        redoUndoCharacteristic->setValue(message.c_str());
        redoUndoCharacteristic->notify();
        Serial.println("Redo button pressed");
      }

      // Check if the undo button is pressed and debounce time has passed
      if (undoButton == LOW && (currentTime - lastUndoPressTime > debounceDelay))
      {
        lastUndoPressTime = currentTime;
        String message = String("{\"action\": 0}");
        redoUndoCharacteristic->setValue(message.c_str());
        redoUndoCharacteristic->notify();
        Serial.println("Undo button pressed");
      }
    }
    else
    {
      Serial.println("Device not connected - Redo Undo task idle");
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void handleButtonTask(void *parameter)
{
  static unsigned long pressStartTime = 0;
  static unsigned long releaseTime = 0;
  static int clickCount = 0;
  unsigned long currentTime;

  pinMode(ROTARY_PIN_SW, INPUT_PULLUP); // Configure button pin as input with internal pull-up

  while (true)
  {
    if (deviceConnected) // Check if the device is connected
    {
      currentTime = millis();

      if (digitalRead(ROTARY_PIN_SW) == LOW)
      { // Button pressed
        if (pressStartTime == 0)
        {
          pressStartTime = currentTime;
        }

        // Long Press Detection
        if ((currentTime - pressStartTime) > 1000)
        {
          Serial.println("Long Press Detected");
          clickCount = 0; // Reset click count
          pressStartTime = 0;
        }
      }
      else
      { // Button released
        if (pressStartTime > 0)
        {
          releaseTime = currentTime;
          clickCount++;
          pressStartTime = 0;
        }
      }

      // Detect Single and Double Click
      if (clickCount > 0 && (currentTime - releaseTime > 300))
      {
        if (clickCount == 1)
        {
          Serial.println("Single Click Detected");
          String message = String("{\"action\": 2}");
          redoUndoCharacteristic->setValue(message.c_str());
          redoUndoCharacteristic->notify();
        }
        else if (clickCount == 2)
        {
          Serial.println("Double Click Detected");
        }
        clickCount = 0; // Reset click counter
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to reduce CPU usage
  }
}

void blinkTask(void *parameter)
{
  int ledPin = 2;
  pinMode(ledPin, OUTPUT);
  while (true)
  {
    if (deviceConnected)
    {
      digitalWrite(ledPin, HIGH);
      vTaskDelay(300 / portTICK_PERIOD_MS);
      digitalWrite(ledPin, LOW);
      vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    else
    {
      digitalWrite(ledPin, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void setup()
{
  pinMode(POTENTIOMETER_PIN, INPUT);

  pinMode(ROTARY_PIN_CLK, INPUT);
  pinMode(ROTARY_PIN_DT, INPUT);

  Serial.begin(115200);

  lastStateCLK = digitalRead(ROTARY_PIN_CLK);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_CLK), handleRotation, CHANGE);

  // Initialize BLE
  BLEDevice::init("TrackWrite");

  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Coordinate Service
  BLEService *coordinateService = pServer->createService(COORDINATES_SERVICE_UUID);
  coordinatesCharacteristic = coordinateService->createCharacteristic(
      COORDINATES_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  coordinatesCharacteristic->addDescriptor(new BLE2902());
  coordinateService->start();
  Serial.println("Coordinate Service started");

  // Zoom Service
  BLEService *zoomService = pServer->createService(ZOOM_SERVICE_UUID);
  zoomCharacteristic = zoomService->createCharacteristic(
      ZOOM_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  zoomCharacteristic->addDescriptor(new BLE2902());
  zoomService->start();
  Serial.println("Zoom Service started");

  // Mode Service
  BLEService *modeService = pServer->createService(MODE_SERVICE_UUID);
  modeCharacteristic = modeService->createCharacteristic(
      MODE_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
  modeCharacteristic->addDescriptor(new BLE2902());
  modeService->start();
  Serial.println("Mode Service started");

  // Redo Undo Service
  BLEService *redoUndoService = pServer->createService(REDO_UNDO_SERVICE_UUID);
  redoUndoCharacteristic = redoUndoService->createCharacteristic(
      REDO_UNDO_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  redoUndoCharacteristic->addDescriptor(new BLE2902());
  modeCharacteristic->setCallbacks(new ModeCharacteristicCallbacks());
  redoUndoService->start();
  Serial.println("Redo Undo Service started");

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(COORDINATES_SERVICE_UUID);
  pAdvertising->addServiceUUID(ZOOM_SERVICE_UUID);
  pAdvertising->addServiceUUID(MODE_SERVICE_UUID);
  pAdvertising->addServiceUUID(REDO_UNDO_SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("Waiting for client to connect...");

  // // Create tasks
  // xTaskCreate(zoomTask, "zoomTask", 15000, NULL, 1, NULL);
  // xTaskCreate(modeTask, "modeTask", 15000, NULL, 1, NULL);
  // xTaskCreate(touchPanelTask, "touchPanelTask", 15000, NULL, 1, NULL);
  // xTaskCreate(blinkTask, "blinkTask", 15000, NULL, 1, NULL);

  xTaskCreatePinnedToCore(zoomTask, "zoomTask", 15000, NULL, 1, NULL, 0); // Pin to core 0
  // xTaskCreatePinnedToCore(modeTask, "modeTask", 15000, NULL, 1, NULL, 0);             // Pin to core 0
  xTaskCreatePinnedToCore(handleButtonTask, "handleButtonTask", 15000, NULL, 1, NULL, 0); // Pin to core 0
  xTaskCreatePinnedToCore(redoUndoTask, "redoUndoTask", 15000, NULL, 2, NULL, 0);         // Pin to core 0
  xTaskCreatePinnedToCore(touchPanelTask, "touchPanelTask", 15000, NULL, 1, NULL, 1);     // Pin to core 1, higher priority

  xTaskCreatePinnedToCore(blinkTask, "blinkTask", 15000, NULL, 2, NULL, 1); // Pin to core 0
}

void loop()
{
}
