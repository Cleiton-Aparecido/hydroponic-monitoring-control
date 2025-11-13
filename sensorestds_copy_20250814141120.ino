#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

#define TdsSensorPin 34          // Pino do sensor de TDS/EC
#define ONE_WIRE_BUS 4           // Pino do sensor DS18B20
#define BOMB1_PIN 5              // Bomba 1
#define BOMB2_PIN 18             // Bomba 2
#define BOMB3_PIN 19             // Bomba 3
#define CHECK_INTERVAL 60000     // Tempo entre verificações (ms)
// #define CHECK_INTERVAL 2000   // Tempo entre verificações (ms)
#define VREF 3.3                 // Referência ADC do ESP32
#define SCOUNT 30                // Amostras de leitura analógica
#define BOMB_FLOW 1.4286         // Vazão da bomba (mL/s)


// Botões (você pode trocar os pinos conforme sua ligação)
#define BTN_UP_PIN 12
#define BTN_DOWN_PIN 14
#define BTN_OK_PIN 27

// --- VARIÁVEIS GLOBAIS ---
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float ecValue = 0;
bool editMode = false;
float ecMin = 1.51;             // Valor mínimo de EC desejado
bool bombActive = false;
unsigned long startTimeBomb = 0;
unsigned long lastCheck = 0;
float tempo;


// --- OBJETOS DOS SENSORES ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- FUNÇÕES AUXILIARES ---
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (int i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int temp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = temp;
      }
    }
  }
  if (iFilterLen & 1)
    return bTab[(iFilterLen - 1) / 2];
  else
    return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

float getFactorFromTDS(float tds) {
  // Fator aproximado para converter TDS -> EC
  if (tds < 500) return 0.5;
  if (tds < 1000) return 0.64;
  return 0.7;
}

// --- Cálculo do volume (equação do professor) ---
float calcularVolume(float ec) {
  // Volume (mL) = 0,0507*EC² + 0,2745*EC + 0,0266
  return 0.0507 * pow(ec, 2) + 0.2745 * ec + 0.0266;
}


float calcularTempoLigada(float ec) {
  float diferenca = ecMin - ec; // quanto falta para o alvo
  if (diferenca < 0) diferenca = 0; // evita valores negativos
  float volume = 0.0507 * pow(diferenca, 2) + 0.2745 * diferenca + 0.0266;
  return volume / BOMB_FLOW; // tempo em segundos
}


// --- SETUP ---
void setup() {
  Serial.begin(115200);
  pinMode(BOMB1_PIN, OUTPUT);
  pinMode(BOMB2_PIN, OUTPUT);
  pinMode(BOMB3_PIN, OUTPUT);
  digitalWrite(BOMB1_PIN, LOW);
  digitalWrite(BOMB2_PIN, LOW);
  digitalWrite(BOMB3_PIN, LOW);

  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_OK_PIN, INPUT_PULLUP);

  sensors.begin();
  lcd.init();
  lcd.backlight();

  Serial.println("Sistema de Controle de EC iniciado!");

}

// --- LOOP PRINCIPAL ---
void loop() {
  // Amostragem do sensor de EC
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 50U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }


   if (digitalRead(BTN_OK_PIN) == LOW) {
    delay(200);
    editMode = !editMode;
    Serial.print("Modo alterado para: ");
  }

  if (editMode) {
    if (digitalRead(BTN_UP_PIN) == LOW) {
      ecMin += 0.01;
      delay(200);
    }
    if (digitalRead(BTN_DOWN_PIN) == LOW) {
      ecMin -= 0.01;
      if (ecMin < 0) ecMin = 0;
      delay(200);
    }
  }


  // Processa e mostra resultados a cada 1 segundo
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000U) {
    printTimepoint = millis();

    for (int i = 0; i < SCOUNT; i++)
      analogBufferTemp[i] = analogBuffer[i];

    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 4095.0;

    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    if (temperatureC == DEVICE_DISCONNECTED_C) temperatureC = 25.0;

    float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    tdsValue = (133.42 * pow(compensationVoltage, 3)
              - 255.86 * pow(compensationVoltage, 2)
              + 857.39 * compensationVoltage) * 0.5;

    float factor = getFactorFromTDS(tdsValue);
    ecValue = tdsValue / (factor * 1000.0);

    // // --- Controle automático da bomba a cada 5 segundos ---
      Serial.println("|==============================================|");
      Serial.print("Segundos() -> ");
      Serial.print(millis()/1000);
      Serial.print("| bombActive() -> ");
      Serial.println(bombActive);
       Serial.println("|==============================================|");
    if ((millis() - lastCheck > CHECK_INTERVAL) && bombActive == false ) {

      lastCheck = millis();

      if ((ecValue < ecMin) && bombActive == false) {
          Serial.println("=============ligar bombas=============");
        tempo = calcularTempoLigada(ecValue); // segundos
        Serial.print("EC baixo (");
        Serial.print(ecValue);
        Serial.print(" mS/cm). Ligando bomba por ");
        Serial.print(tempo, 2);
        Serial.println(" s...");

        startTimeBomb = millis();
        bombActive = true;

        digitalWrite(BOMB1_PIN, HIGH);
        digitalWrite(BOMB2_PIN, HIGH);
        digitalWrite(BOMB3_PIN, HIGH);


    
     

      } else {
        Serial.print("EC OK: ");
        Serial.print(ecValue);
        Serial.println(" mS/cm (bomba off)");
      }
       Serial.println("=============FINALIZADA Realizando analise=============");
    }
    
    if(bombActive){
        
        
        Serial.println("|==============================================|");
        Serial.print("startTimeBomb: ");
        Serial.println(startTimeBomb);

        
        Serial.print("lastCheck: ");
        Serial.println(lastCheck);

        Serial.print("millis() ");
        Serial.println(millis());

        Serial.print("tempo restante: ");
        Serial.print( (millis() - startTimeBomb)/1000);
        Serial.println( " seg");

        Serial.print("Tempo ligada: ");
        Serial.print((tempo * 100000)/1000);
        Serial.println( " seg");

      if(( millis() - startTimeBomb)  > (tempo * 100000) ){
          Serial.print("Desativar bomba! "); 
          

          digitalWrite(BOMB1_PIN, LOW);
          digitalWrite(BOMB2_PIN, LOW);
          digitalWrite(BOMB3_PIN, LOW);

          Serial.println("Bomba desligada.");
          
          bombActive = false;
          lastCheck = millis();
          
        
      }
      else{
           Serial.println("Bombas ligadas");
      }


    }else{
       Serial.println("Bombas Desligadas");
    }
    
    Serial.println("|==============================================|");

    if(editMode){

      lcd.setCursor(0, 0);
      lcd.print("Modo altecao:  ");
      lcd.setCursor(0, 1);
      lcd.print(ecMin, 2);
      lcd.print(" mS/cm     ");

    }
    else{
        // --- Exibe informações no LCD ---
        lcd.setCursor(0, 0);
        lcd.print(ecValue, 2);
        lcd.print(" mS/cm ");



        if(bombActive){
          lcd.setCursor(11, 0);
          lcd.print("ON ");
        }else{
          lcd.setCursor(11, 0);
          lcd.print("OFF ");
        }

        lcd.setCursor(0, 1);
        lcd.print(temperatureC, 1);
        lcd.print((char)223);
        lcd.print("C");

        lcd.setCursor(7, 1);
        lcd.print("M:");
        lcd.print(ecMin, 2);
        lcd.print("ms");
    }

  

  }
}
