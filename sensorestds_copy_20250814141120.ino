#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// --- DEFINIÇÕES DE PINOS ---
#define TdsSensorPin 34
#define ONE_WIRE_BUS 4

#define BOMB1_PIN 5
#define BOMB2_PIN 18
#define BOMB3_PIN 19

#define BTN_UP_PIN 12
#define BTN_DOWN_PIN 14
#define BTN_OK_PIN 27

// --- CONFIGURAÇÕES ---
#define VREF 3.3
#define SCOUNT 30
// #define BOMB_FLOW 1.4286 // mL/s

// --- VARIÁVEIS ---
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float ecValue = 0;

bool editMode = false;
bool intervalEditMode = false;

float ecMin = 1.60;

bool bombActive = false;
unsigned long startTimeBomb = 0;
unsigned long lastCheck = 0;
float tempo;

unsigned long checkIntervalMinutes = 1;
unsigned long CHECK_INTERVAL = 60000UL;

// Controle de botões
unsigned long okPressTime = 0;
bool okHeld = false;

unsigned long comboPressTime = 0;
bool comboHeld = false;


bool reservoirEditMode = false;
unsigned long comboDownPressTime = 0;
bool comboDownHeld = false;


const int N_CAL = 24;
float calibTDS[N_CAL]    = {   103,  132  ,   146,      262,    429,    680,   854,      886,   898  ,     943  , 1003,  1036 ,  1039   , 1066  , 1084   , 1178    ,   1183,    1191, 1199   , 1210   , 1223   ,  1232   ,1235  , 1251  };
float calibFactor[N_CAL] = {0.3678, 0.4125,0.4563,   0.4295, 0.4290, 0.4755, 0.4566, 0.47127, 0.4776 ,  0.5058  ,0.4327, 0.4465,  0.4478 , 0.4595 ,0.4059, 0.4315  , 0.4333,  0.4363, 0.4392 , 0.3878 , 0.3920 ,  0.3510 , 0.3519 , 0.3564 };

// ---- Função para interpolar fator ----

// --- OBJETOS ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -----------------------------------------------------------------------------
//  FUNÇÕES AUXILIARES
// -----------------------------------------------------------------------------

int getMedianNum(int bArray[], int len) {
  int bTab[len];
  for (int i = 0; i < len; i++) bTab[i] = bArray[i];
  
  for (int j = 0; j < len - 1; j++)
    for (int i = 0; i < len - j - 1; i++)
      if (bTab[i] > bTab[i + 1]) {
        int temp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = temp;
      }

  if (len & 1)
    return bTab[(len - 1) / 2];
  else
    return (bTab[len / 2] + bTab[len / 2 - 1]) / 2;
}

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

// Volume do reservatório em litros
float RESERVOIR_VOLUME = 5.0;  // altere conforme sua caixa de água

// Vazão da bomba (mL por segundo)
float BOMB_FLOW = 1.4286;

// Quantidade de bombas trabalhando juntas
int NUM_BOMBS = 3;
float tempoLigado;

// --- Cálculo do tempo necessário para as 3 bombinhas ---
float calcularTempoLigada(float ecAtual) {

  float V = (ecMin - ecAtual) / ecMin;
  float V2 = (8.4800 * RESERVOIR_VOLUME); 
  float resultadoML = V * V2;
  float tempoLigado = resultadoML / BOMB_FLOW;


  Serial.println("===== CALCULO DE FERTILIZACAO (3 Bombas) =====");
  Serial.print("EC Atual: "); Serial.println(ecAtual,4);
  Serial.print("EC Alvo : "); Serial.println(ecMin,4);
  Serial.print("RESERVOIR_VOLUME : "); Serial.println(RESERVOIR_VOLUME);
  Serial.print(" (ecMin - ecAtual) / ecMin "); Serial.println(V,4);
  Serial.print("(8,48 * RESERVOIR_VOLUME) "); Serial.println(V2,4);
  Serial.print("resultadoML: "); Serial.println(resultadoML,4);
  Serial.print("Tempo necessário (s) "); Serial.println(tempoLigado);
  Serial.println("==============================================");

  return tempoLigado;
}


// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------

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

  Serial.println("Sistema iniciado!");
}

// -----------------------------------------------------------------------------
//  LOOP PRINCIPAL
// -----------------------------------------------------------------------------

void loop() {

  // ---------------------------------------------------------------------------
  //  LEITURA DO SENSOR
  // ---------------------------------------------------------------------------
  static unsigned long analogSampleTime = millis();
  if (millis() - analogSampleTime > 50U) {
    analogSampleTime = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

   // ---------------------------------------------------------------------------
  //  LEITURA DOS BOTÕES
  // ---------------------------------------------------------------------------
  bool okPressed   = (digitalRead(BTN_OK_PIN)   == LOW);
  bool upPressed   = (digitalRead(BTN_UP_PIN)   == LOW);
  bool downPressed = (digitalRead(BTN_DOWN_PIN) == LOW);

  // Se OK soltar, reseta todos os timers de hold
  if (!okPressed) {
    okPressTime = 0;
    okHeld = false;
    comboPressTime = 0;
    comboHeld = false;
    comboDownPressTime = 0;
    comboDownHeld = false;
  }

  // --- COMBO OK + UP (2s) → EDITAR INTERVALO ---
  if (okPressed && upPressed && !downPressed) {

    if (comboPressTime == 0)
      comboPressTime = millis();

    if (!comboHeld && millis() - comboPressTime > 2000UL) {
      comboHeld = true;

      intervalEditMode  = !intervalEditMode;
      editMode          = false;
      reservoirEditMode = false;

      Serial.println(intervalEditMode ?
        ">>> EDITANDO INTERVALO <<<" :
        ">>> SAINDO DO MODO INTERVALO <<<");

      delay(300);
    }
  }

  // --- COMBO OK + DOWN (2s) → EDITAR RESERVOIR_VOLUME ---
  else if (okPressed && downPressed && !upPressed) {

    if (comboDownPressTime == 0)
      comboDownPressTime = millis();

    if (!comboDownHeld && millis() - comboDownPressTime > 2000UL) {
      comboDownHeld = true;

      reservoirEditMode = !reservoirEditMode;
      editMode          = false;
      intervalEditMode  = false;

      Serial.println(reservoirEditMode ?
        ">>> EDITANDO VOLUME RESERVATORIO <<<" :
        ">>> SAINDO DO MODO VOLUME <<<");

      delay(300);
    }
  }

  // --- APENAS OK (2s) → EDITAR EC_MIN ---
  else if (okPressed && !upPressed && !downPressed) {

    if (okPressTime == 0)
      okPressTime = millis();

    if (!okHeld && millis() - okPressTime > 2000UL) {
      okHeld = true;

      if (intervalEditMode) {
        intervalEditMode = false;
        Serial.println(">>> SAINDO DO MODO INTERVALO <<<");
      }
      else if (reservoirEditMode) {
        reservoirEditMode = false;
        Serial.println(">>> SAINDO DO MODO VOLUME <<<");
      }
      else {
        editMode = !editMode;
        Serial.println(editMode ?
          ">>> EDITANDO EC_MIN <<<" :
          ">>> SAINDO DO MODO EC_MIN <<<");
      }
      delay(300);
    }
  }

  


  // ---------------------------------------------------------------------------
  //  MODO EDITAR EC_MIN
  // ---------------------------------------------------------------------------
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

  // ---------------------------------------------------------------------------
  //  MODO EDITAR INTERVALO
  // ---------------------------------------------------------------------------
  if (intervalEditMode) {

    if (digitalRead(BTN_UP_PIN) == LOW && !okPressed) {
      checkIntervalMinutes++;
      if (checkIntervalMinutes > 120) checkIntervalMinutes = 120;
      delay(200);
    }

    if (digitalRead(BTN_DOWN_PIN) == LOW) {
      if (checkIntervalMinutes > 1)
        checkIntervalMinutes--;
      delay(200);
    }

    CHECK_INTERVAL = checkIntervalMinutes * 60000UL;
  }

  // ---------------------------------------------------------------------------
  //  MODO EDITAR VOLUME DO RESERVATÓRIO
  // ---------------------------------------------------------------------------
  if (reservoirEditMode) {

    if (digitalRead(BTN_UP_PIN) == LOW && !okPressed) {
      RESERVOIR_VOLUME += 5.0;   // aumenta 0.5 L
      if (RESERVOIR_VOLUME > 100.0) RESERVOIR_VOLUME = 100.0;
      delay(200);
    }

    if (digitalRead(BTN_DOWN_PIN) == LOW && !okPressed) {
      RESERVOIR_VOLUME -= 5.0;   // diminui 0.5 L
      if (RESERVOIR_VOLUME < 1.0) RESERVOIR_VOLUME = 1.0;
      delay(200);
    }
  }

  // ---------------------------------------------------------------------------
  //  PROCESSAMENTO DOS VALORES (a cada 1 segundo)
  // ---------------------------------------------------------------------------
  static unsigned long printTime = millis();
  if (millis() - printTime > 1000U) {
    printTime = millis();

    for (int i = 0; i < SCOUNT; i++)
      analogBufferTemp[i] = analogBuffer[i];

    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 4095.0;

    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    if (temperatureC == DEVICE_DISCONNECTED_C) temperatureC = 25.0;

    float compCoeff = 1.0 + 0.02 * (temperatureC - 25.0);
    float compVolt = averageVoltage / compCoeff;

    tdsValue = (133.42 * pow(compVolt, 3)
              - 255.86 * pow(compVolt, 2)
              + 857.39 * compVolt) * 0.5;


    Serial.println("tds");
    Serial.print(tdsValue);
    Serial.println("|==============================================|");
  
    float factor = getFactorFromTDS(tdsValue);
    ecValue = tdsValue / (factor * 1000.0);


    Serial.println("|==============================================|");
    Serial.print("Segundos() -> ");
    Serial.print(millis());
    Serial.print("| bombActive() -> ");
    Serial.println(bombActive);
    Serial.println("|==============================================|");
    // -----------------------------------------------------------------------
    //  CONTROLE DAS BOMBAS
    // -----------------------------------------------------------------------
    if (!bombActive && (millis() - lastCheck > CHECK_INTERVAL)) {
      lastCheck = millis();

      if (ecValue < ecMin) {
        tempo = calcularTempoLigada(ecValue);
        startTimeBomb = millis();
        bombActive = true;

        digitalWrite(BOMB1_PIN, HIGH);
        digitalWrite(BOMB2_PIN, HIGH);
        digitalWrite(BOMB3_PIN, HIGH);
      }
    }

    if (bombActive) {

        tempoLigado = tempo * 1000;

        Serial.println("|==============================================|");
        Serial.print("startTimeBomb: ");
        Serial.println(startTimeBomb);

        
        Serial.print("lastCheck: ");
        Serial.println(lastCheck);

        Serial.print("millis() ");
        Serial.println(millis());

        Serial.print("tempo atual: ");
        Serial.print( (startTimeBomb - millis()));
        Serial.println( " seg");

       Serial.print("Tempo ligada segund: ");
       Serial.println(tempo );

        Serial.print("Tempo ligada: ");
        Serial.print(tempoLigado);
        Serial.println( " seg");

      if ((millis() - startTimeBomb) > tempoLigado) {

        Serial.print("Desativar bomba! "); 
          

        digitalWrite(BOMB1_PIN, LOW);
        digitalWrite(BOMB2_PIN, LOW);
        digitalWrite(BOMB3_PIN, LOW);

        bombActive = false;
        lastCheck = millis();
      }
    }else{
      Serial.println("Bombas Desligadas");
    }

    Serial.println("|==============================================|");

    // -----------------------------------------------------------------------
    //  DISPLAY LCD
    // -----------------------------------------------------------------------
    if (editMode) {
      lcd.setCursor(0, 0);
      lcd.print("Editar EC_MIN   ");

      lcd.setCursor(0, 1);
      lcd.print(ecMin, 2);
      lcd.print(" mS/cm      ");
    }
    else if (intervalEditMode) {
      lcd.setCursor(0, 0);
      lcd.print("Editar Intervalo");

      lcd.setCursor(0, 1);
      lcd.print(checkIntervalMinutes);
      lcd.print(" min        ");
    }
    else if (reservoirEditMode) {
      lcd.setCursor(0, 0);
      lcd.print("Editar Volume   ");

      lcd.setCursor(0, 1);
      lcd.print(RESERVOIR_VOLUME, 1);
      lcd.print(" L          ");
    }
    else {

      lcd.setCursor(0, 0);
      lcd.print(ecValue, 2);
      lcd.print(" mS/cm ");

      lcd.setCursor(11, 0);
      lcd.print(bombActive ? "ON   " : "OFF  ");

      lcd.setCursor(0, 1);
      lcd.print(temperatureC, 1);
      lcd.print((char)223);
      lcd.print("C ");

      lcd.setCursor(7, 1);
      lcd.print("M:");
      lcd.print(ecMin, 2);
    }
  }
}
