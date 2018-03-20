#ifndef IO_h
#define IO_h

#include <TimeLib.h>
#include <SdFat.h>

#define RS422 Serial1

SdFat SD;
SdFile logfile;
char logfileName[21] = "";
unsigned long offset = 0;
boolean logfileNameSet = false;
boolean sdCardReady = false;

// Implémentation dtostrf pour Cortex M0
#if 0
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
#else
#include <string.h>
#include <stdlib.h>
char *dtostrf(double val, int width, unsigned int prec, char *sout)
{
  int decpt, sign, reqd, pad;
  const char *s, *e;
  char *p;
  s = fcvt(val, prec, &decpt, &sign);
  if (prec == 0 && decpt == 0) {
    s = (*s < '5') ? "0" : "1";
    reqd = 1;
  } else {
    reqd = strlen(s);
    if (reqd > decpt) reqd++;
    if (decpt == 0) reqd++;
  }
  if (sign) reqd++;
  p = sout;
  e = p + reqd;
  pad = width - reqd;
  if (pad > 0) {
    e += pad;
    while (pad-- > 0) *p++ = ' ';
  }
  if (sign) *p++ = '-';
  if (decpt <= 0 && prec > 0) {
    *p++ = '0';
    *p++ = '.';
    e++;
    while ( decpt < 0 ) {
      decpt++;
      *p++ = '0';
    }
  }    
  while (p < e) {
    *p++ = *s++;
    if (p == e) break;
    if (--decpt == 0) *p++ = '.';
  }
  if (width < 0) {
    pad = (reqd + width) * -1;
    while (pad-- > 0) *p++ = ' ';
  }
  *p = 0;
  return sout;
}
#endif

char *d2s(double val, char width, char prec) {
  // Conversion double vers string pour sprintf arduino car %f n'est pas supporté

  static char str_temp[8];
  dtostrf(val, width, prec, str_temp);
  return str_temp;
}

char *timestamp() {
  // Construit un logfile timestamp modèle SailGrib
  // 2018-02-21T15:41:24.241Z<espace>
  
  static char TStamp[30];
  sprintf(TStamp,"%04u-%02u-%02uT%02u:%02u:%02u.%03luZ ", year(),month(),day(),hour(),minute(),second(),(millis() - offset) % 1000);
  return TStamp; 
}

void add_to_log(char *nmea) {
  if (logfileNameSet && sdCardReady) {
    logfile.open(logfileName, FILE_WRITE);
    logfile.write(nmea);
    logfile.println();
    logfile.close();
  }
}

char *add_checksum(char *nmea) {
  // Calcul du checksum nmea

  byte checksum = 0;
  char csum[4];
  for (byte i = 1; i < strlen(nmea); i++ ) {
    checksum ^= nmea[i];
  }
  sprintf(csum, "*%02X", checksum);
  return (strcat(nmea, csum));
}

void send_nmea(char *nmea) {
  // Envoi phrase vers port série RS485.

#ifdef DEBUG
  Serial.write(nmea);
  Serial.println();
#else
  RS422.write(nmea);
  RS422.println();
#endif
}

void send_poztx(char *s) {
  // Calcul du checksum nmea et envoi phrase vers port série RS422.
  // La phrase peut également être lue sur le pin TX en TTL.
  
  char nmeaOut[80];
  sprintf(nmeaOut, "$POZTX,%s", s);
  //strcpy(temps, s);
  //strcat(nmeaOut, temps);
  send_nmea(add_checksum(nmeaOut));
}

void sdcard_init() {
  if (SD.begin(4, SD_SCK_MHZ(50))) {
    sdCardReady = true;
  } else {
    char msg[] = "Carte MicroSD absente ou défectueuse";
    send_poztx(msg);
  }
}

void dateTime(uint16_t* date, uint16_t* time) {
  // User gets date and time from GPS or real-time
  // clock in real callback function

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}

char *filename() {
  // Construit un nom de fichier log YYYYMMDD-hhmmss.txt
  
  static char FName[21];
  sprintf(FName,"%04u%02u%02u-%02u%02u%02uZ.txt", year(), month(), \
                    day(), hour(), minute(), second());
  return FName; 
}

void time_GPS(TinyGPSPlus tgps) {
  // Récupère l'heure TU depuis le GPS et synchronise les millisecondes
  
  // Moyen le plus fiable de tester : Date valide si != 2000
  if (tgps.date.year() != 2000) {
    int Year = tgps.date.year();
    byte Month = (byte)tgps.date.month();
    byte Day = (byte)tgps.date.day();
    byte Hour = (byte)tgps.time.hour();
    byte Minute = (byte)tgps.time.minute();
    byte Second = (byte)tgps.time.second();

    offset = millis();
    setTime(Hour, Minute, Second, Day, Month, Year);

    // Règle la date de création du logfile
    SdFile::dateTimeCallback(dateTime);

    // Crée le nom du logfile
    if (!logfileNameSet) {
      strcpy(logfileName, filename());
      logfileNameSet = true;
    }
  }
}

int fromHex(char a)
// Renvoie la valeur décimale d'un hexa (TinyGPS++)

{
  if (a >= 'A' && a <= 'F') return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f') return a - 'a' + 10;
  else return a - '0';
}

int readGPS(int readch, char *buffer, int len) {
  // Acquisition nmea GPS et contrôle checksum

  static int pos, lg = 0;
  int rpos;
  byte i, ck, checksum = 0;

  if (readch > 0) {
    switch (readch) {
      case '$':  // OZ
        pos = 0; // Reset position index immediately
        buffer[pos++] = readch;
        buffer[pos] = 0;
        checksum = 0;
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        lg = strlen(buffer) - 3;
        for (i = 1; i < lg; i++ ) checksum ^= (uint8_t)buffer[i];
        ck = 16 * fromHex(buffer[strlen(buffer) - 2]) + fromHex(buffer[strlen(buffer) - 1]);
        if (ck == checksum) return rpos;
      default:
        if (pos < len - 1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}
#endif

