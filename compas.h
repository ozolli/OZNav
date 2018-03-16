#include <Adafruit_FXOS8700.h>
#include "filtre.h"

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

FPB Froll, Fpitch, Fhdg;

// Résultats obtenus avec OZNav_Calib_Mag et MotionCal.exe ou magneto12.exe

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -85.952304f, -144.312971f, 76.945700f };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.110872f, -0.034282f, -0.001749f },
                                    { -0.034282f, 1.121473f, 0.006872f },
                                    { -0.001749f, 0.006872f, 1.160688f } };

// float mag_field_strength        = 47.128f;

// Avec Magneto 1.2
// dans son boitier avec RS-422 et câble, maison

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0f, 0.0f, 0.0f };

// Offsets X, Y et Z de l'accéléromètre entrés directement dans le FXOS8700
int accel_offset[3]             = { -14, 20, -27 };

// Coefficients de l'équation de déviation du compas (CurveExpert)
/*  15032018.cxp
    a = -2.822200891339419E+00
    b = -2.671455888028069E-01
    c = 2.182855608749967E+00
    d = -1.288112431251045E-01
    e = 1.193057725181622E-01
  Standard Error          : 7.535990506554416E-01
  Correlation Coefficient : 9.072402092315716E-01
*/
double A = -2.822200;
double B = -0.267145;
double C =  2.182855;
double D = -0.128811;
double E =  0.119305;

void send_poztx(char * s) {
  // Calcul du checksum nmea et envoi phrase vers port série RS422.
  // La phrase peut également être lue sur le pin TX en TTL.
  
  char nmeaOut[80];
  char temps[70];

  sprintf(nmeaOut, "$POZTX,");
  strcpy(temps, s);
  strcat(nmeaOut, temps);
  byte checksum = 0;
  char csum[4];
  for (byte i = 1; i < strlen(nmeaOut); i++ ) {
    checksum ^= nmeaOut[i];
  }
  sprintf(csum, "*%02X", checksum);
  Serial.write(nmeaOut);
  Serial.write(csum);
  Serial.println();
}

void imu_init() {
  if(!accelmag.begin(ACCEL_RANGE_2G, accel_offset[0], accel_offset[1], accel_offset[2])) {
    char msg[] = "Défaut compas";
    send_poztx(msg);
  }
}

int count = 0;

void imu_query(Boat& mus) {
count++;
  //sensors_event_t gyro_event;
  sensors_event_t aevt;
  sensors_event_t mevt;

  // Get new data samples
  accelmag.getEvent(&aevt, &mevt);

  // Apply mag offset compensation (base values in uTesla)
  float x = mevt.magnetic.x - mag_offsets[0];
  float y = mevt.magnetic.y - mag_offsets[1];
  float z = mevt.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Calcul gite
  mus.imu.r_roll = (float) atan2(-aevt.acceleration.y, aevt.acceleration.z);
  mus.imu.r_roll = filtre(Froll, mus.imu.r_roll, mus.cal.alpha_roll);
  mus.imu.d_roll = mus.imu.r_roll * RAD_TO_DEG;
  mus.imu.d_roll += mus.cal.roll;

  // Calcul tangage
  mus.imu.r_pitch = (float) atan2(-aevt.acceleration.x, sqrt(aevt.acceleration.y*aevt.acceleration.y
                          + aevt.acceleration.z*aevt.acceleration.z));
  mus.imu.r_pitch = filtre(Fpitch, mus.imu.r_pitch, mus.cal.alpha_pitch);
  mus.imu.d_pitch = mus.imu.r_pitch * RAD_TO_DEG;
  mus.imu.d_pitch += mus.cal.pitch;

  // Calcul cap
  float norm = sqrt(aevt.acceleration.x * aevt.acceleration.x
                    + aevt.acceleration.y * aevt.acceleration.y
                    + aevt.acceleration.z * aevt.acceleration.z);
  float pitchA = asin(-aevt.acceleration.x / norm);
  float rollA = asin(-aevt.acceleration.y / cos(pitchA) / norm);
  norm = sqrt(mx * mx + my * my + mz * mz);
  mx = mx / norm;
  my = -1 * my / norm;
  mz = mz / norm;
  
  // Tilt-compensation
  float Mx = mx * cos(pitchA) + mz * sin(pitchA);
  float My = mx * sin(rollA) * sin(pitchA) + my * cos(rollA) - mz * sin(rollA) * cos(pitchA);
  mus.imu.r_hdg = atan2(-My, Mx);
  if (!isnan(mus.imu.r_hdg)) mus.imu.r_hdg = filtre(Fhdg, mus.imu.r_hdg, mus.cal.alpha_hdg);

  // Courbe de déviation
  mus.imu.d_hdg = mus.imu.r_hdg * RAD_TO_DEG;
  mus.imu.d_hdg -= A + B * sin(mus.imu.r_hdg) + C * cos(mus.imu.r_hdg) + D * sin(2*mus.imu.r_hdg) + E * cos(2*mus.imu.r_hdg);

  // Calage du zéro
  mus.imu.d_hdg += mus.cal.hdg;

  if (mus.imu.d_hdg < 0.0f) mus.imu.d_hdg += 360.0f;
  else if (mus.imu.d_hdg >= 360.0f) mus.imu.d_hdg -= 360.0f;
  mus.imu.r_hdg = mus.imu.d_hdg * DEG_TO_RAD;
  mus.imu.d_hdt = mus.imu.d_hdg + mus.cal.mag;
  if (mus.imu.d_hdt < 0.0f) mus.imu.d_hdt += 360.0f;
  else if (mus.imu.d_hdt >= 360.0f) mus.imu.d_hdt -= 360.0f;
  mus.imu.r_hdt = mus.imu.d_hdt * DEG_TO_RAD;
  if (count == 6000) Serial.println("######################### 1 mn #######################");
}
