#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---- LCD I2C ----
#define I2C_SDA 21
#define I2C_SCL 22
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---- TDS ----
#define TdsSensorPin 34
#define VREF 3.3
#define SCOUNT 30

int   analogBuffer[SCOUNT];
int   analogBufferTemp[SCOUNT];
int   analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

// ---- DS18B20 ----
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ---- Calibração (seus dados experimentais) ----
const int N_CAL = 24;
float calibTDS[N_CAL]    = {   103,  132  ,   146, ,     262,    429,    680,   854,      886,   898  ,     943  , 1003,  1036 ,  1039   , 1066  , 1084   , 1178    ,   1183,    1191, 1199   , 1210   , 1223   ,  1232   ,1235  , 1251  };
float calibFactor[N_CAL] = {0.3678, 0.4125,0.4563, ,  0.4295, 0.4290, 0.4755, 0.4566, 0.47127, 0.4776 ,  0.5058  ,0.4327, 0.4465,  0.4478 , 0.4595 ,0.4059, 0.4315  , 0.4333,  0.4363, 0.4392 , 0.3878 , 0.3920 ,  0.3510 , 0.3519 , 0.3564 };

// ---- Função para interpolar fator ----
float getFactorFromTDS(float tds) {
  if (tds <= calibTDS[0]) return calibFactor[0];
  if (tds >= calibTDS[N_CAL - 1]) return calibFactor[N_CAL - 1];

  for (int i = 0; i < N_CAL - 1; i++) {
    if (tds >= calibTDS[i] && tds <= calibTDS[i + 1]) {
      float t = (tds - calibTDS[i]) / (calibTDS[i + 1] - calibTDS[i]);
      return calibFactor[i] + t * (calibFactor[i + 1] - calibFactor[i]);
    }
  }
  return calibFactor[N_CAL - 1]; // fallback
}

// ---- Mediana ----
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];

  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if (iFilterLen & 1) return bTab[(iFilterLen - 1) / 2];
  return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

void setup() {
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("TDS + Temp");
  lcd.setCursor(0, 1);
  lcd.print("Inicializando...");
  
  sensors.begin();
  delay(1200);
  lcd.clear();
}

void loop() {
  // --- Amostragem ---
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 50U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  // --- Processamento ---
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000U) {
    printTimepoint = millis();

    // Mediana
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 4095.0;

    // Temperatura
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    if (temperatureC == DEVICE_DISCONNECTED_C) {
      temperatureC = 25.0;
    }

    // Compensação
    float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    // Conversão TDS
    tdsValue = (133.42 * pow(compensationVoltage, 3)
              - 255.86 * pow(compensationVoltage, 2)
              + 857.39 * compensationVoltage) * 0.5;

    // Busca fator calibrado
    float factor = getFactorFromTDS(tdsValue);

    // EC final
    float ecValue = tdsValue / (factor * 1000.0);

    // Serial
    Serial.print("Temp: ");
    Serial.print(temperatureC, 2);
    Serial.print(" C | TDS: ");
    Serial.print((int)tdsValue);
    Serial.print(" ppm | Fator: ");
    Serial.print(factor, 4);
    Serial.print(" | EC: ");
    Serial.print(ecValue, 3);
    Serial.println(" mS/cm");

    // LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EC: ");
    lcd.print(ecValue, 2);
    lcd.print(" mS/cm");

    lcd.setCursor(0, 1);
    lcd.print("Temp:");
    lcd.print(temperatureC, 1);
    lcd.print((char)223);
    lcd.print("C");
  }
}
