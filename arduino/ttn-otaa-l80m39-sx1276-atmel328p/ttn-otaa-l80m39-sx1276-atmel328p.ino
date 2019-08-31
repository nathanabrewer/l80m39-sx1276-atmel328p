#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define LORA_RESET 8
#define LED_1 4
#define LED_2 6
#define LED_3 7


//LSB
static const u1_t PROGMEM DEVEUI[8]={  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0  };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

//LSB
static const u1_t PROGMEM APPEUI[8]={ 0x4F, 0x19, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

//MSB
static const u1_t PROGMEM APPKEY[16] = {  };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

uint8_t mydata[16];

static osjob_t sendjob;

// Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RESET,
    .dio = {2, 3, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 1000000,
};


void setup() {

    pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LORA_RESET, HIGH);   
  
  Serial.begin(9600);

  os_init();
  LMIC_reset();
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7,14);
  LMIC_selectSubBand(1);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  delay(2000);
  
  do_send(&sendjob);
  
}


void loop() { 
  if((millis()%1000) == 0){
    do_send(&sendjob);
  }
  os_runloop_once();
}


void onEvent (ev_t ev) {
    //Serial.print(os_getTime());
    //Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            //Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            //Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            //Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            //Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            //Serial.println(F("EV_JOINING"));
            digitalWrite(LED_2, LOW);
            delay(500);
            digitalWrite(LED_2, HIGH);

            break;
        case EV_JOINED:
            //Serial.println(F("EV_JOINED"));
            {

              digitalWrite(LED_2, LOW);
              
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              
              //Serial.print("netid: ");
              //Serial.println(netid, DEC);
              //Serial.print("devaddr: ");
              //Serial.println(devaddr, HEX);
              //Serial.print("artKey: ");
              
              for (int i=0; i<sizeof(artKey); ++i) {
                //if (i != 0)
                  //Serial.print("-");
                  //Serial.print(artKey[i], HEX);
              }
              //Serial.println("");
              //Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                      //if (i != 0)
                              //Serial.print("-");
                      //Serial.print(nwkKey[i], HEX);
              }
              //Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            //Serial.println(F("EV_JOIN_FAILED"));
            digitalWrite(6, LOW);
            delay(1000);
            digitalWrite(6, HIGH);
            delay(1000);            
            digitalWrite(6, LOW);
            delay(1000);
            digitalWrite(6, HIGH);
            delay(1000);
             digitalWrite(6, LOW);
            delay(1000);
            digitalWrite(6, HIGH);
            delay(1000);            
            digitalWrite(6, LOW);
            delay(1000);
            digitalWrite(6, HIGH);
            delay(1000);           
            break;
        case EV_REJOIN_FAILED:
            //Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            //Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              //Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              //Serial.println(F("Received "));
              //Serial.println(LMIC.dataLen);
              //Serial.println(F(" bytes of payload"));
            
                for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                        //Serial.print(F("0"));
                    }
                    //Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                }

                if(LMIC.frame[LMIC.dataBeg] == 0x01){
                  digitalWrite(6, LOW);
                }
                if(LMIC.frame[LMIC.dataBeg] == 0x02){
                   digitalWrite(6, HIGH);
                }
                     
            }

              digitalWrite(7, HIGH);   // turn the LED on (HIGH is the voltage level)
              delay(1000);                       // wait for a second
              digitalWrite(7, LOW);    // turn the LED off by making the voltage LOW
              delay(1000);  
            
            
            
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            
            break;
        case EV_LOST_TSYNC:
            //Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            //Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            //Serial.println(F("EV_TXSTART"));
            break;
        default:
            //Serial.print(F("Unknown event: "));
            //Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        //Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        
        mydata[0] = 123;
        
                
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        //Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
