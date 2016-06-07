// *******************************************************************
//  
//  SENSOR TRANSMISOR BS (ATTINY85)
//
// Uso simple de la libreria VirtualWire para transmitir mensajes.
// Implementa transmision tipo simplex (one-way).
//
//********************************************************************

#include <VirtualWire.h>

#include <stdlib.h>
#include <EEPROM.h>

#include <avr/io.h>        // Adds useful constants

#define F_CPU 8000000      // Es usado por la libreria delay.h
#include <util/delay.h>    // Añade la funciones delay_ms y delay_us

#include <avr/sleep.h>
#include <avr/interrupt.h>

// Routines to set and claer bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Valores para representar Verdadero y Falso
#define TRUE 255  
#define FALSE 0

const int led_pin = 0;
const int txPin = 3;
const int rxPin = 5;
const int wakePin = 1;
const int extBattPin = A1;
const int transmit_en_pin = 3;

volatile byte count = 1; // count cycles for watchdog

// Variables for the Sleep/power down modes:
volatile boolean f_wdt = 1;

long intBatt = 0; // Usado para random
long extBatt = 0; // Usado para sensor de volage externo

struct Sensor_STRUCT// Data Structure
{
  int    id;  // dentificador del sensor
  long   batt;// Nivel de la bateria del sensor en milivoltios
  int    sen1;// Valor de la sonda 1
  int    sen2;// Valor de la sonda 2
  float  sen3;// Valor de la sonda 3
};

struct Sensor_STRUCT sData;

//
// Lee el voltaje interno de AVR
//
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


//
//   Usa el divisor de frecuencia del osclidador
// interno del chip
//
//  Ejemplos:
//  setPrescaler(0); //  8 MHz
//  setPrescaler(1); // div 1, i.e. 4 MHz
//  setPrescaler(15); // div 8, i.e. 1 MHz div 256 (15) = 32kHz
//  setPrescaler(3); // div 8, i.e. 1 MHz
//
static void setPrescaler (uint8_t mode) {
    cli();
    CLKPR = bit(CLKPCE);
    CLKPR = mode;
sei();
}

//
// Setup
//
void setup()
{
  // Initialise the IO and ISR
  vw_set_tx_pin(txPin);
  vw_set_rx_pin(rxPin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(300);             // Bits per sec
  pinMode(led_pin, OUTPUT);
  pinMode(wakePin, INPUT); // sonda de interrupción
  // Valores iniciales 
  sData.id = 102;
  sData.batt = 0;
  sData.sen1 = 0;
  sData.sen2 = 0;
  sData.sen3 = 0.0;
  pinMode(extBattPin, INPUT);
  analogReference(INTERNAL);

  setup_watchdog(9); // Aproximadamente 8 segundos sleep
  
}

//
// Setup del modo sleep
//
void sleep() {

  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT1);                   // Use PB1 as interrupt pin
  ADCSRA &= ~_BV(ADEN);                   // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep

  cli();                                  // Disable interrupts
  PCMSK &= ~_BV(PCINT1);                  // Turn off PB1 as interrupt pin
  sleep_disable();                        // Clear SE bit
  ADCSRA |= _BV(ADEN);                    // ADC on

  sei();                                  // Enable interrupts
} // sleep

ISR(PCINT1_vect) {
  // This is called when the interrupt occurs, but I don't need to do anything in it
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt = 1; // set global flag
  // Cuenta tres interrupciones de watchdog
  if (count == 100)
    count = 1;
  else
    count = count + 1;
}


//
// Enviar datos del sensor
//
void sendData() {
  digitalWrite(led_pin, HIGH); // Flash a light to show transmitting

  vw_send((uint8_t *)&sData, sizeof(sData));
  vw_wait_tx(); // Wait until the whole message is gone

  digitalWrite(led_pin, LOW);

} //sendData()

//
//   Leer los valores de las sondas del sensor 
//
void readSensors() {
  intBatt = readVcc();
  sData.batt = intBatt;
  if (digitalRead(wakePin) == HIGH)
    sData.sen1 = TRUE;
  else
    sData.sen1 = FALSE;
  sData.sen2 = count;
  float v1 = 1.1; // valor de referencia 1.1 real de la alimentacion de Arduino, Vcc
  float r1 = 1000000; // 1M
  float r2 = 100000; // 100K
  float v = (analogRead(extBattPin) * v1) / 1024.0; //A4 pin3 PB4
  float v2 = v / (r2 / (r1 + r2));
  sData.sen2 = analogRead(extBattPin);
  sData.sen3 = v2;
  //sData.sen3 = highByte(intBatt << 8);

} //readSensors()


//
// Bucle continuo
//
void loop()
{
  _delay_ms(100);
  if (count == 1)
  {
    readSensors(); // Leer valores de sensores
    sendData();    // Envia info sensor
  }
  
  sleep();
} //loop()

// 
// Setup del watchdog
//
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
//
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;

  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

