#include <Adafruit_FXOS8700.h>
#include "filtre.h"

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

FPB Froll, Fpitch, Fhdg;

// Résultats obtenus avec OZNav_Calib_Mag et MotionCal.exe ou magneto12.exe

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -83.981975f, -143.862941f, 74.983069f };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.198396f, -0.021559f, 0.029449f },
                                    { -0.021559f, 1.202093f, -0.000364f },
                                    { 0.029449f, -0.000364f, 1.193448f } };

// float mag_field_strength        = 47.128f;

// Avec Magneto 1.2
// dans son boitier avec RS-422 et câble, bureau

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0f, 0.0f, 0.0f };

// Offsets X, Y et Z de l'accéléromètre entrés directement dans le FXOS8700
int accel_offset[3]             = { -14, 20, -27 };

// Coefficients de l'équation de déviation du compas (CurveExpert)
/*  12032018.cxp
    a = -2.126020994861641E+00
    b = 1.991367698456988E+00
    c = -1.454724778794229E+00
    d = 5.077575902172844E-01
    e = 2.967244547636485E+00
  Standard Error          : 4.827243315951540E-01
  Correlation Coefficient : 9.860858930771847E-01
*/
double A = -2.126020;
double B =  1.991367;
double C = -1.454724;
double D =  0.507757;
double E =  2.967244;

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
  mus.imu.r_hdg = filtre(Fhdg, mus.imu.r_hdg, mus.cal.alpha_hdg);

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
