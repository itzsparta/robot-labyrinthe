/*
 * TEST DU CÂBLAGE — à lancer AVANT le code du robot
 * Cytron Maker Nano + 3x VL6180X (Pololu #2489) + 2x FS90R
 * Bibliothèque : "VL6180X" by Pololu
 *
 * Ouvre le moniteur série à 115200 bauds.
 *  - Au démarrage : vérifie chaque capteur (OK / ÉCHEC)
 *      buzzer : 1 bip = tout est bon, 3 bips = problème
 *      (le petit interrupteur du buzzer sur la carte doit être sur ON)
 *  - Ensuite : affiche les 3 distances en mm en continu
 *  - Servos à l'arrêt. Pour trouver la valeur STOP de chaque roue,
 *    tape dans le moniteur série par exemple :  g1500  ou  d1480
 *    et ajuste jusqu'à ce que la roue ne bouge plus du tout.
 */

#include <Wire.h>
#include <VL6180X.h>
#include <Servo.h>

const char* NOM[3]       = {"Diag gauche", "Avant", "Diag droite"};
const uint8_t PIN_CE[3]  = {2, 3, 4};
const uint8_t ADRESSE[3] = {0x30, 0x31, 0x32};
const uint8_t PIN_BUZZER = 8;   // buzzer intégré de la Maker Nano

VL6180X capteur[3];
bool capteurOK[3];
Servo servoG, servoD;
int stopG = 1485, stopD = 1470;

void bip(int n) {
  for (int i = 0; i < n; i++) {
    tone(PIN_BUZZER, 2000, 120);
    delay(250);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // évite que la Nano se fige si SDA/SCL sont mal branchés

  Serial.println(F("\n=== Test du cablage ==="));

  // Tous les capteurs en veille
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], OUTPUT);
    digitalWrite(PIN_CE[i], LOW);
  }
  delay(10);

  // Réveil un par un + nouvelle adresse
  bool toutOK = true;
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], INPUT);   // jamais HIGH : CE n'accepte pas le 5 V
    delay(10);
    capteur[i].setTimeout(100);
    capteur[i].init();
    capteur[i].configureDefault();
    capteur[i].setAddress(ADRESSE[i]);

    // Le VL6180X répond 0xB4 quand on lui demande son identifiant
    capteurOK[i] = (capteur[i].readReg(VL6180X::IDENTIFICATION__MODEL_ID) == 0xB4);
    if (!capteurOK[i]) toutOK = false;

    Serial.print(NOM[i]);
    Serial.print(F(" (CE sur D"));
    Serial.print(PIN_CE[i]);
    Serial.print(F(") : "));
    Serial.println(capteurOK[i] ? F("OK") : F("ECHEC -> verifie 5V, GND, SDA=A4, SCL=A5 et son fil CE"));
  }

  servoG.attach(9);
  servoD.attach(10);
  servoG.writeMicroseconds(stopG);
  servoD.writeMicroseconds(stopD);

  bip(toutOK ? 1 : 3);
  Serial.println(F("\nGauche\tAvant\tDroite   (mm, 255 = rien devant)"));
}

void loop() {
  // Réglage des valeurs STOP depuis le moniteur série
  if (Serial.available()) {
    char roue = Serial.read();
    int val = Serial.parseInt();
    if (val >= 1000 && val <= 2000) {
      if (roue == 'g') { stopG = val; servoG.writeMicroseconds(stopG); }
      if (roue == 'd') { stopD = val; servoD.writeMicroseconds(stopD); }
      Serial.print(F(">> STOP_G = ")); Serial.print(stopG);
      Serial.print(F("   STOP_D = ")); Serial.println(stopD);
    }
  }

  for (int i = 0; i < 3; i++) {
    if (capteurOK[i]) {
      int d = capteur[i].readRangeSingleMillimeters();
      if (capteur[i].timeoutOccurred()) Serial.print(F("TIMEOUT"));
      else Serial.print(d);
    } else {
      Serial.print(F("--"));
    }
    Serial.print('\t');
  }
  Serial.println();
  delay(200);
}
