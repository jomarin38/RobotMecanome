#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <ServoInput.h>
#include "PilotageMoteur.h"

// --------------------------------------- Brochage de l'Arduino -----------------------------------------------------
// Pour la télécommande, on peut utiliser n'importe quelle entrée car le module 'ServoInput' utilisé
// n'est pas restreint aux broches acceptant les interruptions. Il y a même un option permettant d'utiliser
// PinChangeInterrupt à la place de l'implémentation intégrée. Ici, nous ne l'utilisons pas.
#define PIN_TELECOMMANDE_AV_AR    2
#define PIN_TELECOMMANDE_DR_GA    9
#define PIN_TELECOMMANDE_ROTATION 10

#define PIN_CODEUR_MOTEUR_A       13
#define PIN_CODEUR_MOTEUR_B       14
#define PIN_CODEUR_MOTEUR_C       15
#define PIN_CODEUR_MOTEUR_D       16 

// Brochage des commandes de moteurs imposé par le shield L298P KeyStudio
#define PIN_VITESSE_MOTEUR_A  3
#define PIN_SENS_MOTEUR_A     4
#define PIN_VITESSE_MOTEUR_B  11
#define PIN_SENS_MOTEUR_B     12
#define PIN_VITESSE_MOTEUR_C  5
#define PIN_SENS_MOTEUR_C     8
#define PIN_VITESSE_MOTEUR_D  6
#define PIN_SENS_MOTEUR_D     7

// -------------------------------------- Gestion de la télécommande -------------------------------------------------
#define PULSE_MINIMUM             850  // Durée minimum des pulses du récepteur de télécommande (µs)
#define PULSE_MAXIMUM             2150  // Durée maximum des pulses du récepteur de télécommande (µs)

#define ZONE_MORTE_TELECOMMANDE   0.05   // Ratio de la pleine échelle en dessous duquel on ne bouge pas

#define COEFF_ROTATION            1.0

ServoInputPin<PIN_TELECOMMANDE_AV_AR>     commande_av_ar(PULSE_MINIMUM, PULSE_MAXIMUM);
ServoInputPin<PIN_TELECOMMANDE_DR_GA>     commande_dr_ga(PULSE_MINIMUM, PULSE_MAXIMUM);
ServoInputPin<PIN_TELECOMMANDE_ROTATION>  commande_rotation(PULSE_MINIMUM, PULSE_MAXIMUM);

// -------------------------------------- Gestion des moteurs --------------------------------------------------------
#define VITESSE_MAX_MOTEURS           255.0
#define CONSIGNE_MAX                  255  // Valeur de consigne maximum fournie par la télécommande
// Coefficients appliqués aux consignes de la télécommande pour ajuster la sensibilité
#define COEFFICIENT_REDUCTION_AV_AR   1.0 
#define COEFFICIENT_REDUCTION_DR_GA   1.0
#define COEFFICIENT_REDUCTION_ROT     0.01

PilotageMoteur<PIN_CODEUR_MOTEUR_A> moteur_A(PIN_VITESSE_MOTEUR_A, PIN_SENS_MOTEUR_A, 5, 20, 0);
PilotageMoteur<PIN_CODEUR_MOTEUR_B> moteur_B(PIN_VITESSE_MOTEUR_B, PIN_SENS_MOTEUR_B, 5, 20, 0);
PilotageMoteur<PIN_CODEUR_MOTEUR_C> moteur_C(PIN_VITESSE_MOTEUR_C, PIN_SENS_MOTEUR_C, 5, 20, 0);
PilotageMoteur<PIN_CODEUR_MOTEUR_D> moteur_D(PIN_VITESSE_MOTEUR_D, PIN_SENS_MOTEUR_D, 5, 20, 0);

enum MoveType {
  AVANT,
  ARRIERE,
  TDROITE,
  TGAUCHE,
  RDROITE,
  RGAUCHE
};

long prev_time;
int index_move_type;
enum MoveType move_types[6] = {AVANT, ARRIERE, TDROITE, TGAUCHE, RDROITE, RGAUCHE};

// ---------------------------------------------- SETUP --------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TELECOMMANDE_AV_AR,     INPUT);
  pinMode(PIN_TELECOMMANDE_DR_GA,     INPUT);
  pinMode(PIN_TELECOMMANDE_ROTATION,  INPUT);
  Serial.print("Configuration des interruptions de la télécommande ... ");
  commande_av_ar.attach();
  commande_dr_ga.attach();
  commande_rotation.attach();
  Serial.println("OK");

  pinMode(PIN_CODEUR_MOTEUR_A,        INPUT);
  pinMode(PIN_VITESSE_MOTEUR_A,       OUTPUT);
  pinMode(PIN_SENS_MOTEUR_A,          OUTPUT);
  pinMode(PIN_CODEUR_MOTEUR_B,        INPUT);
  pinMode(PIN_VITESSE_MOTEUR_B,       OUTPUT);
  pinMode(PIN_SENS_MOTEUR_B,          OUTPUT);
  pinMode(PIN_CODEUR_MOTEUR_C,        INPUT);
  pinMode(PIN_VITESSE_MOTEUR_C,       OUTPUT);
  pinMode(PIN_SENS_MOTEUR_C,          OUTPUT);
  pinMode(PIN_CODEUR_MOTEUR_D,        INPUT);
  pinMode(PIN_VITESSE_MOTEUR_D,       OUTPUT);
  pinMode(PIN_SENS_MOTEUR_D,          OUTPUT);
  
  analogWrite(PIN_VITESSE_MOTEUR_A, 0);
  analogWrite(PIN_VITESSE_MOTEUR_B, 0);
  analogWrite(PIN_VITESSE_MOTEUR_C, 0);
  analogWrite(PIN_VITESSE_MOTEUR_D, 0);

  moteur_A.set_DebugMode(false);
  moteur_A.set_InverseSensMoteur(true);

  Serial.begin(115200);
  Serial.println("Initialisation OK !");

  prev_time = millis();
  index_move_type = 0;
}

// ----------------------------- Calcul des consignes pour 3 roues holonomes -----------------------------------------
#define RAYON_ROUES       32.5
#define RAYON_PLATEFORME  120.0  // Distance des roues au centre de la plateforme (supposé le même pour les 3 roues)

void calcule_consignes_moteurs(
  float consigne_x, float consigne_y, float consigne_rotation,
  float &consigne_moteur_A, float &consigne_moteur_B, float &consigne_moteur_C, float &consigne_moteur_D)
{
  // Mise à l'échelle des consignes
  consigne_x = map(consigne_x, -255, 255, -CONSIGNE_MAX, CONSIGNE_MAX);
  consigne_y = map(consigne_y, -255, 255, -CONSIGNE_MAX, CONSIGNE_MAX);
  consigne_rotation = map(consigne_rotation, -255, 255, -CONSIGNE_MAX, CONSIGNE_MAX);;

  /*
  Serial.print("consigne_x = ");
  Serial.print(consigne_x);
  Serial.print("consigne_y = ");
  Serial.println(consigne_y);
  Serial.print("consigne_rotation = ");
  Serial.println(consigne_rotation);*/

  /*
  float terme_rotation = COEFFICIENT_REDUCTION_ROT * RAYON_PLATEFORME * consigne_rotation / RAYON_ROUES;
  float terme_x   = 0.866 * consigne_x / RAYON_ROUES;
  float terme_y1  = consigne_y * 0.5 / RAYON_ROUES;
  float terme_y2  = terme_y1 * 2;*/

  float consigne_moteur_A_x = consigne_x;
  float consigne_moteur_B_x = consigne_x;
  float consigne_moteur_C_x = consigne_x;
  float consigne_moteur_D_x = -consigne_x;

  float consigne_moteur_A_y = consigne_y;
  float consigne_moteur_B_y = -consigne_y;
  float consigne_moteur_C_y = -consigne_y;
  float consigne_moteur_D_y = -consigne_y;

  float consigne_moteur_A_rotation = consigne_rotation;
  float consigne_moteur_B_rotation = consigne_rotation;
  float consigne_moteur_C_rotation = -consigne_rotation;
  float consigne_moteur_D_rotation = consigne_rotation;

  consigne_moteur_A = consigne_moteur_A_x + consigne_moteur_A_y + consigne_moteur_A_rotation;
  consigne_moteur_B = consigne_moteur_B_x + consigne_moteur_B_y + consigne_moteur_B_rotation;
  consigne_moteur_C = consigne_moteur_C_x + consigne_moteur_C_y + consigne_moteur_C_rotation;
  consigne_moteur_D = consigne_moteur_D_x + consigne_moteur_D_y + consigne_moteur_D_rotation;

  /*
  float k = VITESSE_MAX_MOTEURS / max(consigne_moteur_A, max(consigne_moteur_B, max(consigne_moteur_C, consigne_moteur_D)));

  consigne_moteur_A *= k;
  consigne_moteur_B *= k;
  consigne_moteur_C *= k;
  consigne_moteur_D *= k;*/

  /*
  // Normalisation si dépassement de la vitesse max
  float consigne_moteurs_max = max(max(max(consigne_moteur_A, consigne_moteur_B), consigne_moteur_C), consigne_moteur_D);
  float consigne_moteurs_min = min(min(min(consigne_moteur_A, consigne_moteur_B), consigne_moteur_C), consigne_moteur_D);
  if (consigne_moteurs_max > VITESSE_MAX_MOTEURS) {
    float correction = VITESSE_MAX_MOTEURS / consigne_moteurs_max;
    consigne_moteur_A *= correction;
    consigne_moteur_B *= correction;
    consigne_moteur_C *= correction;
    consigne_moteur_D *= correction;
  } else if (consigne_moteurs_min < -VITESSE_MAX_MOTEURS) {
    float correction = VITESSE_MAX_MOTEURS / -consigne_moteurs_min;
    consigne_moteur_A *= correction;
    consigne_moteur_B *= correction;
    consigne_moteur_C *= correction;
    consigne_moteur_D *= correction;
  }
}*/

  // Normalisation si dépassement de la vitesse max
  float consigne_moteurs_max = max(max(max(abs(consigne_moteur_A), abs(consigne_moteur_B)), abs(consigne_moteur_C)), abs(consigne_moteur_D));
  if (consigne_moteurs_max > VITESSE_MAX_MOTEURS) {
    float correction = VITESSE_MAX_MOTEURS / consigne_moteurs_max;
    consigne_moteur_A *= correction;
    consigne_moteur_B *= correction;
    consigne_moteur_C *= correction;
    consigne_moteur_D *= correction;
  }
}

// -------------------------------------------------------------------------------------------------------------------
void loop() {
  
  float consigne_x         = commande_av_ar.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
  float consigne_y         = commande_dr_ga.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
  float consigne_rotation  = commande_rotation.mapDeadzone(-CONSIGNE_MAX, CONSIGNE_MAX, ZONE_MORTE_TELECOMMANDE);
  
  /*
  Serial.print("consigne_x = ");
  Serial.print(consigne_x);
  Serial.print(" ; ");
  Serial.print("consigne_y = ");
  Serial.print(consigne_y);
  Serial.print(" ; ");
  Serial.print("consigne_rotation = ");
  Serial.println(consigne_rotation);*/
  float consigne_moteur_A, consigne_moteur_B, consigne_moteur_C, consigne_moteur_D;

  
  calcule_consignes_moteurs(
    consigne_x, consigne_y, consigne_rotation, 
    consigne_moteur_A, consigne_moteur_B, consigne_moteur_C, consigne_moteur_D
  );
  /*
  Serial.print("consigne_A = ");
  Serial.print(consigne_moteur_A);
  Serial.print(" ; ");
  Serial.print("consigne_B = ");
  Serial.print(consigne_moteur_B);
  Serial.print(" ; ");
  Serial.print("consigne_C = ");
  Serial.print(consigne_moteur_C);
  Serial.print(" ; ");
  Serial.print("consigne_D = ");
  Serial.println(consigne_moteur_D);*/
  
  set_vitesse(consigne_moteur_A, consigne_moteur_B, consigne_moteur_C, consigne_moteur_D);
}

void set_vitesse(int moteur_a, int moteur_b, int moteur_c, int moteur_d){
  moteur_A.set_ConsigneVitesse(moteur_a);
  moteur_B.set_ConsigneVitesse(moteur_b);
  moteur_C.set_ConsigneVitesse(moteur_c);
  moteur_D.set_ConsigneVitesse(moteur_d);

  moteur_A.execute();
  moteur_B.execute();
  moteur_C.execute();
  moteur_D.execute();
} 

void toto(enum MoveType move_type) {
  switch (move_type) {
    case AVANT:
      set_vitesse(50, 50, -50, -50);
      break;
      
    case ARRIERE:
      set_vitesse(-50, -50, 50, 50);
      break;

    case TDROITE:
      set_vitesse(-50, 50, 50, -50);
      break;
      
    case TGAUCHE:
      set_vitesse(50, -50, -50, 50);
      break;

    case RDROITE:
      set_vitesse(50, 50, 50, 50);
      break;
      
    case RGAUCHE:
      set_vitesse(-50, -50, -50, -50);
      break;
  }
}
