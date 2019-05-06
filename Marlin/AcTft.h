#ifndef ACTFTHEADER_H
#define ACTFTHEADER_H
#include "MyHardwareSerial.h"
#include "music.h"
#include "AcVars.h"

#define NEW_SERIAL_PROTOCOL(x) (NewSerial.print(x))
#define NEW_SERIAL_PROTOCOL_F(x,y) (NewSerial.print(x,y))
#define NEW_SERIAL_PROTOCOLPGM(x) (NewSerialprintPGM(PSTR(x)))
#define NEW_SERIAL_(x) (NewSerial.print(x),NewSerial.write('\n'))
#define NEW_SERIAL_PROTOCOLLN(x) (NewSerial.print(x),NewSerial.write('\r'),NewSerial.write('\n'))
#define NEW_SERIAL_PROTOCOLLNPGM(x) (NewSerialprintPGM(PSTR(x)),NewSerial.write('\n'))

#define TFT_SERIAL_START() (NewSerial.write('\r'),NewSerial.write('\n'))
#define TFT_SERIAL_CMD_SEND(x) (NewSerialprintPGM(PSTR(x)),NewSerial.write('\r'),NewSerial.write('\n'))
#define TFT_SERIAL_ENTER() (NewSerial.write('\r'),NewSerial.write('\n'))
#define TFT_SERIAL_SPACE() (NewSerial.write(' '))

const char newErr[] PROGMEM ="ERR ";
const char newSucc[] PROGMEM ="OK";
#define NEW_SERIAL_ERROR_START (NewSerialprintPGM(newErr))
#define NEW_SERIAL_ERROR(x) NEW_SERIAL_PROTOCOL(x)
#define NEW_SERIAL_ERRORPGM(x) NEW_SERIAL_PROTOCOLPGM(x)
#define NEW_SERIAL_ERRORLN(x) NEW_SERIAL_PROTOCOLLN(x)
#define NEW_SERIAL_ERRORLNPGM(x) NEW_SERIAL_PROTOCOLLNPGM(x)

//##define NEW_SERIAL_ECHO_START (NewSerialprintPGM(newSucc))
#define NEW_SERIAL_ECHOLN(x) NEW_SERIAL_PROTOCOLLN(x)
#define NEW_SERIAL_SUCC_START (NewSerialprintPGM(newSucc))
#define NEW_SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))
#define NEW_SERIAL_ECHOPGM(x) NEW_SERIAL_PROTOCOLPGM(x)
#define NEW_SERIAL_ECHO(x) NEW_SERIAL_PROTOCOL(x)

FORCE_INLINE void NewSerialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    NewSerial.write(ch);
    ch=pgm_read_byte(++str);
  }
}
#endif // ACTFTHEADER_H