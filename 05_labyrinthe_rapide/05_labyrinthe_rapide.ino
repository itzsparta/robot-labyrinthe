/*
 * Robot labyrinthe — parcours connu, version rapide
 * Arduino Nano + 3x VL6180X (Pololu #2489) + 2x servos FS90R (rotation continue)
 * Bibliothèque requise : "VL6180X" by Pololu
 *
 * Placement des capteurs (à 2-3 cm du sol, à plat, fixés sans jeu) :
 *   AVANT      : centré au bout du nez, droit devant        -> déclenche les virages
 *   DIAG_G / D : coins avant, orientés à 45° vers l'avant   -> gardent le robot droit
 *
 * Câblage :
 *   VL6180X (x3, en parallèle) : VIN->5V  GND->GND  SDA->A4  SCL->A5
 *   GPIO0/CE : diag gauche->D2  avant->D3  diag droite->D4
 *   Servos : signal gauche->D9, droite->D10, rouge -> batterie 4,8-6 V, masses communes
 */

#include <Wire.h>
#include <VL6180X.h>
#include <Servo.h>

#define DEBUG 1   // 1 = affiche les distances dans le moniteur série (9600). Mets 0 pour la course

// ---------- Parcours : un virage à chaque mur rencontré droit devant ----------
// 'G' = gauche, 'D' = droite, dans l'ordre.
// Exemple pour un départ en bas du zigzag en montant le long du bord droit : À ADAPTER à ton parcours.
const char PARCOURS[] = "GDDGGDDGG";
const int NB_VIRAGES = sizeof(PARCOURS) - 1;

// ---------- Brochage ----------
enum { DIAG_G = 0, AVANT = 1, DIAG_D = 2 };
const uint8_t PIN_CE[3]  = {2, 3, 4};
const uint8_t ADRESSE[3] = {0x30, 0x31, 0x32};
const uint8_t PIN_SERVO_G = 9;
const uint8_t PIN_SERVO_D = 10;

// ---------- Réglages à calibrer ----------
const int STOP_G = 1485;        // roue gauche immobile (zone morte 1468-1502, mesure)
const int STOP_D = 1470;        // roue droite immobile (mesuré)
const int V_MAX = 200;          // vitesse en ligne droite (µs autour du STOP) : monte tant que la roue accélère
const int V_MIN = 80;           // vitesse juste avant un virage
const int V_PIVOT = 150;        // vitesse pendant le pivot
const int T_90 = 280;           // durée d'un pivot de 90° en ms (dépend de la charge des piles !)
const int DIST_VIRAGE = 90;     // distance au mur devant à laquelle on tourne (mm)
const int DIST_RALENTIR = 190;  // on commence à ralentir sous cette distance (mm, portée max ~200)
const int DIAG_OUVERT = 200;    // lecture diagonale au-delà de laquelle il n'y a pas de mur (mm)
const int CONSIGNE_DIAG = 140;  // lecture diagonale visée quand un seul mur est vu (~10 cm de côté)
const float KP = 1.5;           // gain de la correction de trajectoire

VL6180X capteur[3];
Servo servoG, servoD;
int etape = 0;

// g, d : vitesse de chaque roue en µs autour du STOP, positif = marche avant
void moteurs(int g, int d) {
  servoG.writeMicroseconds(STOP_G + g);
  servoD.writeMicroseconds(STOP_D - d);   // servo droit monté en miroir
}

int lire(int i) {
  int d = capteur[i].readRangeSingleMillimeters();   // même mode que le test, qui marche
  if (capteur[i].timeoutOccurred()) return 999;
  return d;   // 255 = rien à moins de ~20 cm
}

void tourner(char sens) {
  if (sens == 'G') moteurs(-V_PIVOT, V_PIVOT);
  else             moteurs(V_PIVOT, -V_PIVOT);
  delay(T_90);

  // Repart doucement et jette les mesures prises pendant le pivot
  // (sinon une vieille lecture "mur devant" déclencherait le virage suivant)
  moteurs(V_MIN, V_MIN);
  for (int i = 0; i < 3; i++) lire(i);
}

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // évite que la Nano se fige si le bus I2C bloque

  // Les 3 capteurs démarrent à la même adresse I2C (0x29) :
  // tous en veille, puis réveil un par un avec une adresse unique.
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], OUTPUT);
    digitalWrite(PIN_CE[i], LOW);
  }
  delay(10);

  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], INPUT);   // réveil par la pull-up du module. Jamais HIGH : CE n'est pas tolérante au 5 V
    delay(10);
    capteur[i].setTimeout(100);
    capteur[i].init();
    capteur[i].configureDefault();
    capteur[i].setAddress(ADRESSE[i]);
  }

  servoG.attach(PIN_SERVO_G);
  servoD.attach(PIN_SERVO_D);
  moteurs(0, 0);
  delay(2000);   // 2 s pour poser le robot au départ
}

void loop() {
  int dG  = lire(DIAG_G);
  int dAv = lire(AVANT);
  int dD  = lire(DIAG_D);

#if DEBUG
  Serial.print(dG);  Serial.print('\t');
  Serial.print(dAv); Serial.print('\t');
  Serial.println(dD);
#endif

  // Mur devant : virage suivant du parcours, ou arrêt si c'était le dernier
  if (dAv < DIST_VIRAGE) {
    if (etape >= NB_VIRAGES) {
      moteurs(0, 0);
      while (true) {}
    }
    tourner(PARCOURS[etape++]);
    return;
  }

  // Vitesse : plein gaz, puis ralentit en approchant du mur
  int v = map(constrain(dAv, DIST_VIRAGE, DIST_RALENTIR),
              DIST_VIRAGE, DIST_RALENTIR, V_MIN, V_MAX);

  // Correction : centrage si 2 murs, distance fixe si 1 seul, tout droit si aucun
  bool murG = dG < DIAG_OUVERT;
  bool murD = dD < DIAG_OUVERT;
  int err = 0;                                   // > 0 : il faut tourner à droite
  if (murG && murD) err = dD - dG;
  else if (murG)    err = CONSIGNE_DIAG - dG;
  else if (murD)    err = dD - CONSIGNE_DIAG;

  int corr = constrain((int)(KP * err), -v / 2, v / 2);
  moteurs(v + corr, v - corr);
}
