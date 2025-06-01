#ifndef IRremote_h
#define IRremote_h

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#ifndef __AVR_ATmega32U4__
#define MARK  0
#define SPACE 1

#define NEC_BITS 32
#define USECPERTICK 50 // microsecunde per tick de ceas
#define RAWBUF 80 // lungimea bufferului raw pentru durata

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

// Timing-ul protocolului NEC
#define NEC_HDR_MARK 9000
#define NEC_HDR_SPACE 4500
#define NEC_BIT_MARK 560
#define NEC_ONE_SPACE 1600
#define NEC_ZERO_SPACE 560
#define NEC_RPT_SPACE 2250

#define _GAP 5000 // Interval minim între transmisii
#define GAP_TICKS (_GAP/USECPERTICK)

// Stările IR
#define STATE_IDLE 2
#define STATE_MARK 3
#define STATE_SPACE 4
#define STATE_STOP 5

// Tipurile de protocol
#define NEC 1
#define UNKNOWN -1

#define TOPBIT 0x80000000

#ifdef F_CPU
#define SYSCLOCK F_CPU // ceasul principal Arduino
#else
#define SYSCLOCK 16000000 // valoare implicită 16MHz
#endif

// Macro-uri pentru configurarea timerului
#define TIMER_DISABLE_INTR (TIMSK2 = 0)
#define TIMER_ENABLE_PWM (TCCR2A |= _BV(COM2B1))
#define TIMER_DISABLE_PWM (TCCR2A &= ~(_BV(COM2B1)))
#define TIMER_ENABLE_INTR (TIMSK2 = _BV(OCIE2A))
#define TIMER_INTR_NAME TIMER2_COMPA_vect

#define TIMER_CONFIG_KHZ(val) ({ \
  const uint8_t pwmval = F_CPU / 2000 / (val); \
  TCCR2A = _BV(WGM20); \
  TCCR2B = _BV(WGM22) | _BV(CS20); \
  OCR2A = pwmval; \
  OCR2B = pwmval / 3; \
})

#define TIMER_COUNT_TOP (SYSCLOCK * USECPERTICK / 1000000)
#if (TIMER_COUNT_TOP < 256)
#define TIMER_CONFIG_NORMAL() ({ \
  TCCR2A = _BV(WGM21); \
  TCCR2B = _BV(CS20); \
  OCR2A = TIMER_COUNT_TOP; \
  TCNT2 = 0; \
})
#else
#define TIMER_CONFIG_NORMAL() ({ \
  TCCR2A = _BV(WGM21); \
  TCCR2B = _BV(CS21); \
  OCR2A = TIMER_COUNT_TOP / 8; \
  TCNT2 = 0; \
})
#endif

typedef struct {
  uint8_t recvpin; // pinul pentru datele IR de la detector
  volatile uint8_t rcvstate; // automatul finit de stare
  volatile uint32_t lastTime;
  unsigned int timer;
  volatile uint8_t rawbuf[RAWBUF];
  volatile uint8_t rawlen;
} irparams_t;

class IRremote {
public:
  IRremote(int pin);
  ErrorStatus decode();
  void begin();
  void end();
  void loop();
  boolean keyPressed(unsigned char r);
  
  String getString();
  unsigned char getCode();
  String getKeyMap(byte keycode, byte ir_type = 1);
  byte getIrKey(byte keycode, byte ir_type = 1);
  void sendString(String s);
  void sendString(float v);
  void sendNEC(unsigned long data, int nbits);
  void sendRaw(unsigned int buf[], int len, uint8_t hz);
  void enableIROut(uint8_t khz);
  void enableIRIn();
  void mark(uint16_t us);
  void space(uint16_t us);

private:
  ErrorStatus decodeNEC();

  int16_t irIndex;
  char irRead;
  char floatString[5];
  boolean irReady;
  boolean irPressed;
  String irBuffer;
  String Pre_Str;
  double irDelayTime;

  volatile uint8_t *rawbuf;
  int rawlen;
  unsigned long value;
  uint8_t bits;
  int8_t decode_type;
};

#endif // !__AVR_ATmega32U4__
#endif // IRremote_h
