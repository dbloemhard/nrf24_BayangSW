// Use serial monitor to see PPM values
//#define DEBUGPPM
// Use serial plotter to see channel output
//#define DEBUGCHANNELS
//#define DEBUGTELEMETRY
//
// ##########################################
// #####   MultiProtocol nRF24L01 Tx   ######
// ##########################################
// #        by goebish on rcgroups          #
// #                                        #
// #   Parts of this project are derived    #
// #     from existing work, thanks to:     #
// #                                        #
// #   - PhracturedBlue for DeviationTX     #
// #   - victzh for XN297 emulation layer   #
// #   - Hasi for Arduino PPM decoder       #
// #   - hexfet, midelic, closedsink ...    #
// ##########################################
//
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License.
// If not, see <http://www.gnu.org/licenses/>.

#include <EEPROM.h>
#include "iface_nrf24l01.h"

// ######### TX Aux Switch to PPM Channel Map ##########
// Use debug channels with the serial plotter (Arduino IDE) to see which switches
// map to which ppm channel (5-12). Following values are for the default Lite
// Radio 2
#define SwitchA AUX1
#define SwitchB AUX2
#define SwitchC AUX3
#define SwitchD AUX4

// ######### Channel Mapping for Silverware ############
// This is where you map switch positions to Silverware channels. Normally this 
// can be done from your model configuration in the TX, but on the Lite Radio we
// cannot switch models in on the TX, so you have to share the configuration with
// the Frsky transmitter
// Examples
// #define CHAN_5 UP(SwitchA)
// - CHAN_5 is active when SwitchA is UP
//
// More complex
// #define CHAN_6 DOWN(SwitchB) || MIDDLE(SwitchB)
// - CHAN_6 is active when SwitchB is in the DOWN position OR the MIDDLE position
//
// Using multiple switches to send one channel:
// #define CHAN_12 DOWN(SwitchA) && DOWN(SwitchD)
// - CHAN_12 is active when both SwitchA and SwitchD are in the DOWN position
//
// Always send channel as active:
// #define CHAN_6 true
//
// Never send channel/not used:
// #define CHAN_12 false
//
#define CHAN_5 UP(SwitchA)
#define CHAN_6 DOWN(SwitchB) || MIDDLE(SwitchB)
#define CHAN_7 UP(SwitchD)
#define CHAN_8 MIDDLE(SwitchB)
#define CHAN_9 DOWN(SwitchC)
#define CHAN_10 UP(SwitchC)
#define CHAN_11 UP(SwitchB) || MIDDLE(SwitchB)
#define CHAN_12 false // always off


// ######### PPM configuration ############
// tune ppm input for "special" transmitters
// #define SPEKTRUM // TAER, 1100-1900, AIL & RUD reversed

// PPM stream settings
// Use DEBUGPPM and DEBUGCHANNELS to confirm channels are streamed in the order below
// And that the Min/Max values are correct
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    AILERON,
    ELEVATOR,
    THROTTLE,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7), emergency stop (Bayang, Silverware)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700

#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)
#define UP(ch) (ppm[ch] > PPM_MAX_COMMAND)
#define MIDDLE(ch) (PPM_MIN_COMMAND < ppm[ch] && ppm[ch] < PPM_MAX_COMMAND)
#define DOWN(ch) (ppm[ch] < PPM_MIN_COMMAND)
#define REBINDGESTURE (ppm[RUDDER] < PPM_MIN_COMMAND && ppm[THROTTLE] < PPM_SAFE_THROTTLE && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)

// ############ Board Selection ############
#define STM32_BOARD
// #define ATMEGA328P_BOARD

// ############ Wiring #####################
#ifdef STM32_BOARD
  #define PPM_pin   PB11  // PPM in
  
  //SPI Comm.pins with nRF24L01
  #define MOSI_pin  PB15  // MOSI - D3
  #define SCK_pin   PB13  // SCK  - D4
  #define CE_pin    PA8  // CE - D5
  #define MISO_pin  PB14 // MISO - A0
  #define CS_pin    PB12 // CS   - A1
  
  // LED
  #define ledPin    PC13 // LED  - D13
  #define LED_on digitalWrite(ledPin, LOW)
  #define LED_off digitalWrite(ledPin, HIGH)

  // SPI outputs
  #define MOSI_on digitalWrite(MOSI_pin,HIGH)
  #define MOSI_off digitalWrite(MOSI_pin,LOW)
  #define SCK_on digitalWrite(SCK_pin,HIGH)
  #define SCK_off digitalWrite(SCK_pin,LOW)
  #define CE_on digitalWrite(CE_pin,HIGH)
  #define CE_off digitalWrite(CE_pin,LOW)
  #define CS_on digitalWrite(CS_pin,HIGH)
  #define CS_off digitalWrite(CS_pin,LOW)
  // SPI input
  #define  MISO_on (digitalRead(MISO_pin)==HIGH)

  // Need to manually configure timer on the STM32 board
  HardwareTimer HWTimer2(TIM2);
  #define TCNT1 TIM2->CNT
  
#else //ATMEGA328P_BOARD
  #define PPM_pin   2  // PPM in
  
  //SPI Comm.pins with nRF24L01
  #define MOSI_pin  3  // MOSI - D3
  #define SCK_pin   4  // SCK  - D4
  #define CE_pin    5  // CE   - D5
  #define MISO_pin  A0 // MISO - A0
  #define CS_pin    A1 // CS   - A1
  
  // LED
  #define ledPin    13 // LED  - D13
  #define LED_on digitalWrite(ledPin, HIGH)
  #define LED_off digitalWrite(ledPin, LOW)
  
  // SPI outputs
  #define MOSI_on PORTD |= _BV(3)  // PD3
  #define MOSI_off PORTD &= ~_BV(3)// PD3
  #define SCK_on PORTD |= _BV(4)   // PD4
  #define SCK_off PORTD &= ~_BV(4) // PD4
  #define CE_on PORTD |= _BV(5)    // PD5
  #define CE_off PORTD &= ~_BV(5)  // PD5
  #define CS_on PORTC |= _BV(1)    // PC1
  #define CS_off PORTC &= ~_BV(1)  // PC1
  // SPI input
  #define  MISO_on (PINC & _BV(0)) // PC0

  #include <util/atomic.h>
#endif

#define RF_POWER TX_POWER_80mW 

// supported protocols
enum {
    PROTO_BAYANG_SILVERWARE = 0, // Bayang for Silverware with frsky telemetry
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_E010,         // EAchine E010, NiHui NH-010, JJRC H36 mini
    PROTO_END
};

// EEPROM locationss
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

// Telemetry definition
struct {
    uint16_t volt1; // Uncompensated voltage
    uint16_t volt2; // Compensated voltage
    uint16_t rssi;  // TX RSSI - how many telemetry packets received per second
    uint16_t rx_rssi;  // RX RSSI - how packets received per second the receiver sees
    uint8_t updated;
    uint32_t lastUpdate;
} telemetry_data;

uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};
uint16_t rebindCounter=0;

void setup()
{
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(ledPin, OUTPUT);
    LED_off; //start LED off
    pinMode(PPM_pin, INPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    #if defined(DEBUGPPM) || defined(DEBUGCHANNELS) || defined(DEBUGTELEMETRY)
      Serial.begin(9600);
      //Delay so that USB can be recognised/com port opened
      delay(5000);
    //#else
      //frskyInit();    
    #endif
    
    // PPM ISR setup
    attachInterrupt(digitalPinToInterrupt(PPM_pin), ISR_ppm, CHANGE);

    // Timers
    #ifdef STM32_BOARD
      init_HWTimer();               //0.5us
    #else //ATMEGA328P_BOARD
      TCCR1A = 0;  //reset timer1
      TCCR1B = 0;
      TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz
    #endif
    
    set_txid(false);
}

void loop()
{
    uint32_t timeout=0;    
    // reset / rebind
    if(reset || ppm[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        selectProtocol();
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();
    }
    if(rebindCounter > 1000){
        rebindCounter = 0;
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();      
    }
        
    telemetry_data.updated = 0;
    // process protocol
    switch(current_protocol) {
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            timeout = process_Bayang();
            break;
        case PROTO_E010:
            timeout = process_MJX();
            break;
    }
    // updates ppm values out of ISR
    update_ppm();

    if(REBINDGESTURE)
      rebindCounter++;
    else
      rebindCounter=0;
   
    while(micros() < timeout) {
        if(telemetry_data.updated) {
            //To do - output telemetry to a small LCD screen?
            //frskyUpdate();
          #ifdef DEBUGTELEMETRY
            Serial.print("volt1: ");
            Serial.println(telemetry_data.volt1);
            Serial.print("volt2: ");
            Serial.println(telemetry_data.volt2);
            Serial.print("RX RSSI: ");
            Serial.println(telemetry_data.rx_rssi);
            Serial.print("TX RSSI: ");
            Serial.println(telemetry_data.rssi);
          #endif
        }            
    }
    telemetry_data.updated = 0;
}

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

void selectProtocol()
{
    // wait for multiple complete ppm frames
    ppm_ok = false;
    uint8_t count = 10;
    while(count) {
        while(!ppm_ok) {} // wait
        update_ppm();
        if(ppm[AUX8] < PPM_MAX_COMMAND) // reset chan released
            count--;
        ppm_ok = false;
    }
    
    // startup stick commands (protocol selection / renew transmitter ID)
    if(ppm[RUDDER] > PPM_MAX_COMMAND) // rudder right
        current_protocol = PROTO_E010; // EAchine E010, NiHui NH-010, JJRC H36 mini
        
    else if(ppm[RUDDER] < PPM_MIN_COMMAND)   // Rudder left
        set_txid(true);                      // Renew Transmitter ID
    
    else if(ppm[AILERON] < PPM_MIN_COMMAND) // Aileron left
        current_protocol = PROTO_BAYANG;    // EAchine H8(C) mini, BayangToys X6/X7/X9, JJRC JJ850 ...
          
    else // Default to bayang (silverware)
        current_protocol = PROTO_BAYANG_SILVERWARE; // Bayang protocol for Silverware with frsky telemetry
        // (or read last used protocol from eeprom)
        //current_protocol = constrain(EEPROM.read(ee_PROTOCOL_ID),0,PROTO_END-1);      
        
    // update eeprom with selected protocol
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    // wait for safe throttle
   
    while(ppm[THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        update_ppm();
    }
}

void init_protocol()
{
    switch(current_protocol) {
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            Bayang_init();
            Bayang_bind();
            break;
        case PROTO_E010:
            MJX_init();
            MJX_bind();
            break;
    }
}

// update ppm values out of ISR    
void update_ppm()
{
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
    #ifdef STM32_BOARD
            ppm[ch] = Servo_data[ch];
    #else // ATMEGA328P
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    #endif
    }

#ifdef SPEKTRUM
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        if(ch == AILERON || ch == RUDDER) {
            ppm[ch] = 3000-ppm[ch];
        }
        ppm[ch] = constrain(map(ppm[ch],1120,1880,PPM_MIN,PPM_MAX),PPM_MIN,PPM_MAX);
    }
#endif
}

void ISR_ppm()
{
  #ifdef STM32_BOARD
    #define PPM_SCALE 1L
  #else // ATMEGA328P
    #if F_CPU == 16000000
        #define PPM_SCALE 1L
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L
    #else
        #error // 8 or 16MHz only !
    #endif
  #endif
  
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    static unsigned long debugValue;
    counterPPM = TCNT1;
    TCNT1 = 0;
    ppm_ok=false;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    #ifdef DEBUGPPM
        Serial.print("Pulse (");
        Serial.print(counterPPM);
        Serial.println(")");
    #endif
    }
    else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    #ifdef DEBUGPPM
        Serial.print("Sync Frame (");
        Serial.print(counterPPM);
        Serial.println(")");
    #endif
    #ifdef DEBUGCHANNELS
        Serial.print(Servo_data[0]);
        Serial.print(" ");
        Serial.print(Servo_data[1]);
        Serial.print(" ");
        Serial.print(Servo_data[2]);
        Serial.print(" ");
        Serial.print(Servo_data[3]);
        Serial.print(" ");
        Serial.print(Servo_data[4]);
        Serial.print(" ");
        Serial.print(Servo_data[5]);
        Serial.print(" ");
        Serial.print(Servo_data[6]);
        Serial.print(" ");
        Serial.print(Servo_data[7]);
        Serial.println();
    #endif
    }
    else{  //servo values between 510us and 2420us will end up here
      #ifdef DEBUGPPM
        debugValue = constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
        Serial.print("Channel ");
        Serial.print(chan);
        Serial.print(": ");
        Serial.print(debugValue);
        Serial.print(" (PPM Count ");
        Serial.print(counterPPM);
        Serial.print(", pulse ");
        Serial.print(pulse);
        Serial.println(")");
      #endif
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
            if(chan==3)
                ppm_ok = true; // 4 first channels Ok
        }
        chan++;
    }
}

#ifdef STM32_BOARD
  void init_HWTimer()
  { 
    HWTimer2.pause();                 // Pause the timer2 while we're configuring it
    TIM2->PSC = 35;                // 36-1;for 72 MHZ /0.5uSec/(35+1)
    TIM2->ARR = 0xFFFF;              // Count until 0xFFFF
    HWTimer2.setMode(1, TIMER_OUTPUT_COMPARE);  // Main scheduler
    TIM2->SR = 0x1E5F & ~TIM_SR_CC2IF;     // Clear Timer2/Comp2 interrupt flag
    TIM2->DIER &= ~TIM_DIER_CC2IE;       // Disable Timer2/Comp2 interrupt
    HWTimer2.refresh();                 // Refresh the timer's count, prescale, and overflow
    HWTimer2.resume();
  }
#endif
