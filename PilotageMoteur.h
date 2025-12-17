#ifndef PilotageMoteur_h
#define PilotageMoteur_H

#include <PID_v1.h>

// Nombre de pulses du codeur cumulées pour mesurer la période de rotation
#define NB_PULSES_CODEUR_MOTEURS  4
// Macro calculant la vitesse de rotation du moteur en fonction de la période mesurée sur le codeur
#define vitesse_rotation_moteur(periode) (1000000.0 / (periode * 11.0 / NB_PULSES_CODEUR_MOTEURS))
// Temps (en µs) au bout duquel on considère que la vitesse de rotation est nulle si on n'a pas reçu de pulse.
#define TIMEOUT_CODEUR_MOTEURS    10000

// Periode d'écriture sur la liaison série en mode debug (µs)
#define PERIODE_ECRITURE_LS 20000

//--------------------------------------------------------------------------------------------------------------------
template <int pinCodeur> class PilotageMoteur {
  public:
    PilotageMoteur(int8_t pin_vitesse_moteur, int8_t pin_sens_moteur, float kp, float ki, float kd);

    // Fournit la vitesse de rotation du moteur en tours/s
    double get_VitesseRotation();
    void set_ParametresPID(float kp, float ki, float kd);
    void set_ConsigneVitesse(float consigne);
    void execute(void);

    void set_DebugMode(bool debugMode) { m_debug_mode = debugMode; }
    void set_InverseSensMoteur(bool inverseSensMoteur) { m_InverseSensMoteur = inverseSensMoteur; }

  private:
    int8_t m_PinVitesseMoteur;
    int8_t m_PinSensMoteur;
    bool m_InverseSensMoteur = false;

    static int             m_Compteur_impulsions_codeur;
    static unsigned long   m_Duree_impulsion_codeur;
    static unsigned long   m_Date_derniere_impulsion_codeur;

    double m_VitesseMoteurMesuree;
    double m_ConsigneVitesseMoteur;
    double m_CommandeMoteur;

    float m_Kp;
    float m_Ki;
    float m_Kd;

    int m_PID_SampleTime = 20;

    bool m_debug_mode = false;

    PID PID_moteur;

    unsigned long m_DateEcritureLiaisonSerie = 0;

    static void routineInterruptionCodeur(void);
};

//--------------------------------------------------------------------------------------------------------------------
template <int pinCodeur> int
PilotageMoteur<pinCodeur>::m_Compteur_impulsions_codeur;

template <int pinCodeur> unsigned long
PilotageMoteur<pinCodeur>::m_Duree_impulsion_codeur = 1000000;

template <int pinCodeur> unsigned long
PilotageMoteur<pinCodeur>::m_Date_derniere_impulsion_codeur;

template <int pinCodeur> 
PilotageMoteur<pinCodeur>::PilotageMoteur(
    int8_t pin_vitesse_moteur, 
    int8_t pin_sens_moteur, 
    float kp, 
    float ki, 
    float kd ) :
  m_Kp(kp),
  m_Ki(ki),
  m_Kd(kd),
  PID_moteur(&m_VitesseMoteurMesuree, &m_CommandeMoteur, &m_ConsigneVitesseMoteur, kp, ki, kd, DIRECT) 
{
  m_PinVitesseMoteur = pin_vitesse_moteur;
  m_PinSensMoteur = pin_sens_moteur;

  attachPinChangeInterrupt(digitalPinToPCINT(pinCodeur), routineInterruptionCodeur, RISING);
  m_Date_derniere_impulsion_codeur = micros();
  m_Compteur_impulsions_codeur = 0;
  PID_moteur.SetOutputLimits(0, 255);
  PID_moteur.SetSampleTime(m_PID_SampleTime);
  PID_moteur.SetMode(AUTOMATIC);
}

template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::set_ParametresPID(float kp, float ki, float kd) {
  m_Kp = kp;
  m_Ki = ki;
  m_Kd = kd;
  PID_moteur.SetTunings(m_Kp, m_Ki, m_Kd);
}

template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::set_ConsigneVitesse(float consigne) {
  m_ConsigneVitesseMoteur = abs(consigne);
  digitalWrite(m_PinSensMoteur, (consigne > 0) != m_InverseSensMoteur);
}

template <int pinCodeur> 
double PilotageMoteur<pinCodeur>::get_VitesseRotation() {
  // if ((micros() - m_Date_derniere_impulsion_codeur) > TIMEOUT_CODEUR_MOTEURS)
  //   return 0;
  return (1000000.0 * NB_PULSES_CODEUR_MOTEURS / (double)(m_Duree_impulsion_codeur * 11)); 
}

template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::execute() {
  m_VitesseMoteurMesuree = get_VitesseRotation();
  PID_moteur.Compute();

  analogWrite(m_PinVitesseMoteur, m_CommandeMoteur);

  if (m_debug_mode) {
    unsigned long date_courante = micros();
    if ((date_courante - m_DateEcritureLiaisonSerie) > PERIODE_ECRITURE_LS) {
      // Serial.print("consigne = ");
      // Serial.print(m_ConsigneVitesseMoteur);
      // Serial.print(" ; commande = ");
      // Serial.print(m_CommandeMoteur);
      // Serial.print(" ; vitesse = ");
      // Serial.println(m_VitesseMoteurMesuree);
      Serial.print(m_ConsigneVitesseMoteur);
      Serial.print(" ; ");
      Serial.print(m_CommandeMoteur);
      Serial.print(" ; ");
      Serial.println(m_VitesseMoteurMesuree);
      m_DateEcritureLiaisonSerie = date_courante;
    }
  }
}

template <int pinCodeur> 
void PilotageMoteur<pinCodeur>::routineInterruptionCodeur(void) {
  if (++m_Compteur_impulsions_codeur == NB_PULSES_CODEUR_MOTEURS) {
    unsigned long date_courante = micros();
    m_Duree_impulsion_codeur = date_courante - m_Date_derniere_impulsion_codeur;
    m_Date_derniere_impulsion_codeur = date_courante;
    m_Compteur_impulsions_codeur = 0;
  }
}

#endif // #ifndef PilotageMoteur_h