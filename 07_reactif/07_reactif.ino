/*
 * Robot labyrinthe - conduite réactive, sans arrêt (adapté d'un code fourni, bibliothèque Adafruit -> Pololu)
 * Cytron Maker Nano + 3x VL6180X (Pololu #2489) + 2x servos FS90R
 *
 * Principe :
 *   - rien devant (> 220 mm) : tout droit, vite
 *   - mur devant : on braque vers le côté le plus dégagé, d'autant plus fort que le mur est proche
 *     (roue intérieure ralentie, puis arrêtée, puis en marche arrière = pivot au contact)
 *
 * Le code d'origine commandait les servos en degrés (90 = arrêt, 1° ~ 10 µs).
 * Ici les vitesses restent en degrés autour de l'arrêt, converties en µs autour des STOP mesurés.
 *
 * Câblage : CE gauche->D2, avant->D3, droite->D4 | servos gauche->D9, droite->D10 | buzzer D8
 */

#include <Wire.h>
#include <VL6180X.h>
#include <Servo.h>

// ---------- Brochage ----------
enum { GAUCHE = 0, AVANT = 1, DROITE = 2 };
const uint8_t PIN_CE[3]  = {2, 3, 4};
const uint8_t ADRESSE[3] = {0x30, 0x31, 0x32};
const uint8_t PIN_SERVO_G = 9;
const uint8_t PIN_SERVO_D = 10;
const uint8_t PIN_BUZZER  = 8;

// ---------- Réglages ----------
const int STOP_G = 1485;         // milieu de la zone morte roue gauche
const int STOP_D = 1470;         // milieu de la zone morte roue droite
const int US_PAR_DEGRE = 10;     // conversion des "degrés" du code d'origine

const int V_DROIT = 45;          // vitesse tout droit (degrés autour de l'arrêt ; 45 = quasi plein régime)
const int AVANT_LIBRE = 220;     // au-delà : rien devant (mm)

VL6180X capteur[3];
Servo servoG, servoD;

// g, d : vitesse de chaque roue en "degrés" autour de l'arrêt, positif = marche avant
void moteurs(int g, int d) {
  servoG.writeMicroseconds(STOP_G + g * US_PAR_DEGRE);
  servoD.writeMicroseconds(STOP_D - d * US_PAR_DEGRE);   // servo droit monté en miroir
}

// distance en mm (255 = rien à portée), -1 si le capteur ne répond pas
int lire(int i) {
  int d = capteur[i].readRangeSingleMillimeters();
  if (capteur[i].timeoutOccurred()) return -1;
  return d;
}

void initCapteurs() {
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], OUTPUT);
    digitalWrite(PIN_CE[i], LOW);
  }
  delay(10);
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], INPUT);   // jamais HIGH : CE ne supporte pas le 5 V
    delay(10);
    capteur[i].setTimeout(100);
    capteur[i].init();
    capteur[i].configureDefault();
    capteur[i].setAddress(ADRESSE[i]);
    capteur[i].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);
  }
}

void setup() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  initCapteurs();
  servoG.attach(PIN_SERVO_G);
  servoD.attach(PIN_SERVO_D);
  servoG.detach();               // immobile pendant qu'on pose le robot
  servoD.detach();
  tone(PIN_BUZZER, 2000, 150);
  delay(2000);
  servoG.attach(PIN_SERVO_G);
  servoD.attach(PIN_SERVO_D);
}

void loop() {
  int dG  = lire(GAUCHE);
  int dAv = lire(AVANT);
  int dD  = lire(DROITE);

  // Capteur muet : on garde la dernière commande plutôt que de décider sur une fausse valeur
  if (dG < 0 || dAv < 0 || dD < 0) return;

  if (dAv > AVANT_LIBRE) {       // rien devant
    moteurs(V_DROIT, V_DROIT);
    return;
  }
  if (dAv <= 5) return;          // mesure aberrante

  // Plus le mur devant est proche, plus on braque (map extrapole : à 220 mm, kC = -15)
  int kC = map(dAv, 100, 10, 5, 20);
  int kD = map(dD, 100, 10, 5, 7);
  int kG = map(dG, 100, 10, 5, 7);

  if (dD < dG) {                 // plus de place à gauche : roue gauche ralentie
    moteurs(10 - kC - kD, 20);
  } else if (dD > dG) {          // plus de place à droite : roue droite ralentie
    moteurs(30, 15 - kC - kG);
  } else {                       // égalité : pivot à droite, comme dans le code d'origine
    moteurs(30, -30);
  }
}
