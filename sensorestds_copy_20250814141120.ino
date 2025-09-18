#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---- LCD I2C ----
#define I2C_SDA 21
#define I2C_SCL 22
// Endereço comum: 0x27 (se não aparecer nada, tente 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---- TDS ----
#define TdsSensorPin 34      // GPIO34 (ADC1_CH6)
#define VREF 3.3             // Referência ADC ESP32
#define SCOUNT 30            // Amostras p/ mediana

int   analogBuffer[SCOUNT];
int   analogBufferTemp[SCOUNT];
int   analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

// ---- DS18B20 ----
#define ONE_WIRE_BUS 4       // GPIO do DATA do DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

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

  // I2C do LCD nos pinos estáveis do ESP32
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("TDS + Temp");
  lcd.setCursor(0, 1);
  lcd.print("Inicializando...");
  
  // DS18B20
  sensors.begin();
  delay(1200);
  lcd.clear();
}

void loop() {
  // --- Amostragem do ADC (a cada ~40 ms) ---
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 50U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  // --- Processamento/Exibição (a cada ~800 ms) ---
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000U) {
    printTimepoint = millis();

    // Copia e calcula mediana para reduzir ruído
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 4095.0;

    // Lê temperatura real do DS18B20
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    if (temperatureC == DEVICE_DISCONNECTED_C) {
      temperatureC = 25.0; // fallback se sensor desconectado
    }

    // Compensação de temperatura (ref 25°C)
    float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    // Conversão para TDS (ppm) - curva do fabricante
    tdsValue = (133.42 * pow(compensationVoltage, 3)
              - 255.86 * pow(compensationVoltage, 2)
              + 857.39 * compensationVoltage) * 0.5;

    // Serial
    
    Serial.print("Temp: ");
    Serial.print(temperatureC, 2);
    Serial.print(" C | TDS: ");
    Serial.print((int)tdsValue);
    Serial.println(" ppm");

    float factor = 0.35; // ajuste se quiser 0.64 ou 0.7

      if ((int)tdsValue > 80 && (int)tdsValue < 130){
       Serial.print("calculo com fator 0,3678");
       
      factor = 0.3678;
    }


     if ((int)tdsValue > 240 && (int)tdsValue < 270){
       Serial.print("calculo com fator 0,4295");
       
      factor = 0.4295;
    }

    if ((int)tdsValue > 390 && (int)tdsValue < 449){
       Serial.print("calculo com fator 0,429");
       
      factor = 0.429;
    }
    if ((int)tdsValue > 600 && (int)tdsValue < 690){
       Serial.print("calculo com fator 0,475");
       
      factor = 0.475;
    }

    if ((int)tdsValue > 810 && (int)tdsValue < 895){
       Serial.print("calculo com fator 0,4852");
       
      factor = 0.4852;
    }

     if ((int)tdsValue > 1010 && (int)tdsValue < 1080){
       Serial.print("calculo com fator 0,4532");
       
      factor = 0.4532;
    }

      if ((int)tdsValue > 1080 && (int)tdsValue < 1100){
       Serial.print("calculo com fator 0,4059");
       
      factor = 0.4059;
    }

    float ecValue = tdsValue / (factor * 1000.0);

  
    Serial.print("Fator: ");
    Serial.print(factor); 

    Serial.print("EC Value: ");
    Serial.print(ecValue, 2); // duas casas decimais
    Serial.println(" mS/cm");


  // LCD CM
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("v: ");
    lcd.print((float)ecValue);
    lcd.print(" mS/cm");

    // LCD tds
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("TDS:");
    // lcd.print((int)tdsValue);
    // lcd.print("ppm");




    lcd.setCursor(0, 1);
    lcd.print("Temp:");
    lcd.print(temperatureC, 1);
    lcd.print((char)223);
    lcd.print("C");
  }
}
