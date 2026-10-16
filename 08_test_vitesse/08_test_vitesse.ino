/*
 * Test de vitesse des servos FS90R : à partir de quelle commande la vitesse n'augmente plus ?
 * Pour chaque vitesse de la liste : N bips (N = numéro de l'essai), 6 s pour poser le robot
 * sur la ligne de départ, puis 2 s tout droit et arrêt. Mesurer la distance parcourue à la règle.
 * Si la distance n'augmente plus d'une vitesse à la suivante, les servos sont saturés.
 *
 * Servos gauche->D9, droite->D10 | buzzer D8
 */

#include <Servo.h>

const uint8_t PIN_SERVO_G = 9;
const uint8_t PIN_SERVO_D = 10;
const uint8_t PIN_BUZZER  = 8;

const int STOP_G = 1485;
const int STOP_D = 1470;
const int US_PAR_DEGRE = 10;

const int VITESSES[] = {30, 45, 60, 90};   // en "degrés" comme dans 07 (45 = vitesse actuelle)
const int NB = sizeof(VITESSES) / sizeof(VITESSES[0]);
const unsigned long T_ROULE = 2000;        // ms
const unsigned long T_POSE = 6000;         // ms pour reposer le robot

Servo servoG, servoD;

void bips(int n) {
  for (int i = 0; i < n; i++) {
    tone(PIN_BUZZER, 2000, 120);
    delay(300);
  }
}

void setup() {
  for (int i = 0; i < NB; i++) {
    bips(i + 1);
    delay(T_POSE);
    servoG.attach(PIN_SERVO_G);
    servoD.attach(PIN_SERVO_D);
    servoG.writeMicroseconds(STOP_G + VITESSES[i] * US_PAR_DEGRE);
    servoD.writeMicroseconds(STOP_D - VITESSES[i] * US_PAR_DEGRE);   // servo droit monté en miroir
    delay(T_ROULE);
    servoG.detach();   // au sol, le STOP ne suffit pas à immobiliser le robot
    servoD.detach();
  }
  tone(PIN_BUZZER, 1000, 600);   // fin du test
}

void loop() {}
