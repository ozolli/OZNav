#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Madgwick.h>

// Create sensor instances.
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Résultats obtenus avec OZNav_Calib_Mag et MotionCal.exe ou magneto12.exe

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -83.981975f, -143.862941f, 74.983069f };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 0.835224f, 0.014973f, -0.020605f },
                                    { 0.014973f, 0.832151f, -0.000116f },
                                    { -0.020605f, -0.000116f, 0.838417f } };

float mag_field_strength        = 47.128f;

// Avec Magneto 1.2
// dans son boitier avec RS-422 et câble, bureau

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0f, 0.0f, 0.0f };

// Offsets X, Y et Z de l'accéléromètre entrés directement dans le FXOS8700
int accel_offset[3]             = { -14, 20, -27 };

Madgwick filter;

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

  // Initialize the sensors.
  if(!gyro.begin(GYRO_RANGE_250DPS)) {
    char msg[] = "Défaut gyroscope";
    send_poztx(msg);
  }

  if(!accelmag.begin(ACCEL_RANGE_2G, accel_offset[0], accel_offset[1], accel_offset[2])) {
    char msg[] = "Défaut compas";
    send_poztx(msg);
  }

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(50.0f);

  // Coef d'atténuation du filtre à affiner.
  // Voir https://forums.adafruit.com/viewtopic.php?f=19&t=126734&p=632174#p632187
  // Attention aux mises à jour de Adafruit_AHRS car Madgwick.h est modifié pour ajouter setBeta !
  // Sauvegarde du fichier modifié dans C:\Users\Olivier\Documents\Arduino\Muscadet
  filter.setBeta(0.5f);
}

void imu_query(Boat& mus) {

  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= RAD_TO_DEG;
  gy *= RAD_TO_DEG;
  gz *= RAD_TO_DEG;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  // Pas de gimbal lock possible en bateau car gite et tangage < 90°

  // Calcul gite
  mus.imu.d_roll = filter.getRoll();
  mus.imu.d_roll += mus.cal.roll;
  mus.imu.r_roll = mus.imu.d_roll * DEG_TO_RAD;

  // Calcul tangage
  mus.imu.d_pitch = filter.getPitch();
  mus.imu.d_pitch += mus.cal.pitch;
  mus.imu.r_pitch = mus.imu.d_pitch * DEG_TO_RAD;

  // Calcul cap
  mus.imu.d_hdg = 180.0f - filter.getYaw();
  mus.imu.d_hdg  += mus.cal.hdg;
  if (mus.imu.d_hdg < 0.0f) mus.imu.d_hdg += 360.0f;
  else if (mus.imu.d_hdg >= 360.0f) mus.imu.d_hdg -= 360.0f;
  mus.imu.r_hdg = mus.imu.d_hdg * DEG_TO_RAD;
  mus.imu.d_hdt = mus.imu.d_hdg + mus.cal.mag;
  if (mus.imu.d_hdt < 0.0f) mus.imu.d_hdt += 360.0f;
  else if (mus.imu.d_hdt >= 360.0f) mus.imu.d_hdt -= 360.0f;
  mus.imu.r_hdt = mus.imu.d_hdt * DEG_TO_RAD;
}
