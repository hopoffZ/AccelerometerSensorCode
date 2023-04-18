#include "SparkFun_SinglePairEthernet.h"
#include "arduinoFFT.h"
#include "Sparkfun_LIS2DH12.h"

// Config Defines
// #define DEBUG_MODE

#ifndef DEBUG_MODE
#define ChipSelPin CS
#define InterruptPin G1
#define ResetPin G0
#endif

SinglePairEthernet adin1110;

arduinoFFT FFT;

byte deviceMAC[6] = { 0x00, 0xE0, 0x22, 0xFE, 0xDA, 0x03 };
byte destinationMAC[6] = { 0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xCA };
SPARKFUN_LIS2DH12 accel;       //Create instance

const uint16_t samples = 1024;
const double samplingFrequency = 10000;
double vReal[samples];
double vImag[samples];
byte outputBuffer[samples / 2];

unsigned long lastBlink = 0;

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(9600);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  if (accel.begin() == false)
  {
    #ifdef DEBUG_MODE
    Serial.println("Accelerometer not detected. Check address jumper and wiring. Freezing...");
    #endif
    bool f = true;
    while (f)
      f = accel.begin(); //loop until success
  }
  accel.setDataRate(LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP);
  delay(500);
#ifndef DEBUG_MODE
  if (!adin1110.begin(deviceMAC, LED_BUILTIN, InterruptPin, ResetPin, ChipSelPin)) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  while (adin1110.getLinkStatus() != true)
    ;
#endif
}

void loop() {
  collectData();
  calculateFFT();
  sendSensData();

#ifndef DEBUG_MODE
  unsigned long now = millis();
  if ((now - lastBlink >= 1000) && (adin1110.getLinkStatus())) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = now;
  }
#endif
}

void collectData() {
  for (int i = 0; i < samples; i++) {
    vImag[i] = 0;
  }
  long int delay1, delay2, delay3, delay4;
  for (int i = 0; i < samples; i++) {
    if (accel.available())
  {
    vReal[i] = accel.getZ();
    #ifdef DEBUG_MODE
    Serial.print("Acc [mg]: ");
    Serial.print(accelX, 1);
    Serial.print(" x, ");
    Serial.print(accelY, 1);
    Serial.print(" y, ");
    Serial.print(accelZ, 1);
    Serial.print(" z, ");
    Serial.print(tempC, 1);
    Serial.print("C");
    Serial.println();
    #endif
  }
    delayMicroseconds(19);
    delay1++;
    delay2++;
    delay3++;
    delay4++;
  
}

void calculateFFT() {
  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  FFT.Compute(FFT_FORWARD);                        /* Compute FFT */
  FFT.ComplexToMagnitude();                        /* Compute magnitudes */
}

void sendSensData() {
  for (int i = 0; i < samples/2; i++) {
    outputBuffer[i] = (byte)vReal[i];

#ifdef DEBUG_MODE
    Serial.print(outputBuffer[i]);
    Serial.print(" ");
#endif
  }
#ifdef DEBUG_MODE
  Serial.println();
#endif

#ifndef DEBUG_MODE
  if (adin1110.getLinkStatus()) {
    adin1110.sendData(outputBuffer, sizeof(outputBuffer), destinationMAC);
  }
#endif
  delay(2000);
}
