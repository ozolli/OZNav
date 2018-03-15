#ifndef boat_h
#define boat_h

struct Cal {
  // Données de calibrage en eeprom
  double mag = -1.0;    // Déclinaison magnétique en degrés décimaux, négatif pour une déclinaison W
  int hdg = 0;    // Compas en degrés
  int roll = 0;   // Gite en degrés
  int pitch = 0;  // Tangage en degrés
  int leeway = 8; // Leeway en degrés
  int baro = -110;    // Pression atmosphérique en Pa (hPa*100)
  
  // Coefficients d'amortissement du lissage exponentiel simple
  const double alpha_sogcog = 0.2; // Lissage environ 10 secondes
  const double alpha_setdrift = 0.07; // Lissage environ 30 secondes
  const double alpha_roll = 0.06;
  const double alpha_pitch = 0.06;
  const double alpha_hdg = 0.03;
};

struct IMU {
  double d_hdg = 0.0;
  double d_hdt = 0.0;
  double d_roll = 0.0;
  double d_pitch = 0.0;
  double r_hdg = 0.0;
  double r_hdt = 0.0;
  double r_roll = 0.0;
  double r_pitch = 0.0;
  uint8_t syscal = 0;
  uint8_t gyrocal = 0;
  uint8_t accelcal = 0;
  uint8_t magcal = 0;
  //sensors_event_t compass_event;
};

struct Wind {
  boolean hasVWR = false;
  char LR;
  double aws = 0.0;
  double tws = 0.0;
  double d_awa = 0.0;
  double d_awa_rel = 0.0;
  double d_twa = 0.0;
  double d_twa_rel = 0.0;
  double r_awa = 0.0;
  double r_awa_cor = 0.0;
  double r_twa = 0.0;
  double r_awd = 0.0;
  double r_twd = 0.0;
};

struct Baro {
  double temp = 0.0;
  double pres = 0.0;
};

struct Speedo {
  boolean hasVHW = false;
  double bsp = 0.0;
  double vmg = 0.0;
};

struct GPS {
  boolean hasGPS = false;
  double sog = 0.0;
  double d_cog = 0.0;
  double r_cog = 0.0;
  double drift = 0.0;
  double d_set = 0.0;
  double r_set = 0.0;
};

struct Boat {
  Cal cal;
  IMU imu;
  Wind wind;
  Baro baro;
  Speedo speedo;
  GPS gps;
};
#endif

