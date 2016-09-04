/*
  Multple Serial test

 Receives from the main serial port, sends to the others.
 Receives from serial port 1, sends to the main serial (Serial 0).

 This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc

 The circuit:
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:

 created 30 Dec. 2008
 modified 20 May 2012
 by Tom Igoe & Jed Roach
 modified 27 Nov 2015
 by Arturo Guadalupi

 This example code is in the public domain.

 */
 #include "ExtSoftSerial.h"

ExtSoftSerial SoftSerial(3,A0,false);

#define DEBUG_DUMP_PPM

#ifdef DEBUG_DUMP_PPM
uint8_t ppmDump   = 0;
uint32_t lastDump = 0;
#endif

#define MULTI_HEADER 0x55
#define MULTI_PROTOCOL_FRSKY 0x3

uint8_t frameIndex;
volatile uint16_t PPM[16] = { 512, 512, 512, 512, 512, 512, 512, 512 , 512, 512, 512, 512, 512, 512, 512, 512 };
volatile uint8_t ppmAge = 0; // age of PPM data



void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial.println("Begin with ExtSoftSerial");

/*  SoftSerial.begin(100000,SERIAL_8N1);
  delay(1000);


  SoftSerial.begin(100000, SERIAL_8N2);
  delay(1000);
    SoftSerial.begin(100000, SERIAL_8E1);
  delay(1000);*/
    SoftSerial.begin(100000, SERIAL_8E2);
  //delay(1000);
    
}
uint8_t mode, currmode = 0;
uint8_t centerdel = 1;



void loop() {
  
//currmode = (millis()/1000) % (0x3E + 1) + SERIAL_8E2 ;



	static const unsigned long REFRESH_INTERVAL = 10000; // ms
	static unsigned long lastRefreshTime = 0;
	
	if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
	{
		lastRefreshTime += REFRESH_INTERVAL;
                
        //        SoftSerial.setCentering(centerdel++);
	}

  if (currmode != mode) {
      
/*  switch ( mode)
  {
  case 0:
    Serial.println("Begin with SERIAL_8N1");
    SoftSerial.begin(100000,SERIAL_8N1);
    break;
    case 1:
    Serial.println("Begin with SERIAL_8N2");
    SoftSerial.begin(100000,SERIAL_8N2);
    break;
    case 2:
    Serial.println("Begin with SERIAL_8E1");
    SoftSerial.begin(100000,SERIAL_8E1);
    break;
    case 3:
    Serial.println("Begin with SERIAL_8E2");
    SoftSerial.begin(100000,SERIAL_8E2);

    break;
  default:
    break;
  }*/
  
  mode = currmode;
  //SoftSerial.begin(100000,mode);
  }


  if (SoftSerial.available()) {
    uint8_t inByte = SoftSerial.read();
    processMULTI(inByte);
    //Serial.write(inByte);
  }
#ifdef DEBUG_DUMP_PPM
  if (ppmDump) {
    uint32_t timeTMP = millis();
    Serial.print(timeTMP - lastDump);
    lastDump = timeTMP;
    Serial.print(':');
    for (uint8_t i = 0; i < 16; i++) {
      Serial.print(PPM[i]);
      Serial.print(',');
    }
    Serial.println();
    ppmDump = 0;
  }
#endif


}


struct sbus_help {
  uint16_t ch0 : 11;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
} __attribute__ ((__packed__));

struct sbus {
  struct sbus_help ch[2];
  uint8_t status;
}  __attribute__ ((__packed__));

// This is common temporary buffer used by all PPM input methods
union ppm_msg {
  uint8_t  bytes[32];
  uint16_t words[16];
  struct sbus sbus;
} ppmWork;


static uint16_t calculateChannel(uint16_t input) {
  uint32_t value;
  if (input < 204) {
    value = 204;
  } else {
    value = input - 204;
  }
  value = (value * 1000) / 1639;
  if (value > 999) {
    value = 999;
  }

  return (uint16_t)value + 12;
}

static inline void processMULTI(uint8_t c)
{
  if (frameIndex == 0) {
    if (c == MULTI_HEADER) {
      frameIndex++;
      //Serial.println("Sync");
    }
    else {
//Serial.println("S");
    Serial.print(" S:");
    Serial.print(c, HEX);
    }
    
  } else if (frameIndex == 1) {
    /*Serial.print(" ");
    Serial.print(c, HEX);
    Serial.print(" ");*/
    if ((c & 0x1F) == MULTI_PROTOCOL_FRSKY) {
      frameIndex++;
      //Serial.print(" FRSky");
    } else {
      frameIndex = 0;
      Serial.print(" F ");
    Serial.print(c, HEX);
    }
  } else if (frameIndex == 2) {
    frameIndex++;
  } else if (frameIndex == 3) {
    frameIndex++;
  } else if (frameIndex <= 26) {
    ppmWork.bytes[(frameIndex++) - 4] = c;
    
    if (frameIndex == 26) {
      if ((ppmWork.sbus.status & 0x08) == 0) {
        uint8_t set;
        for (set = 0; set < 2; set++) {
          PPM[(set << 3)] = calculateChannel(ppmWork.sbus.ch[set].ch0);
          PPM[(set << 3) + 1] = calculateChannel(ppmWork.sbus.ch[set].ch1);
          PPM[(set << 3) + 2] = calculateChannel(ppmWork.sbus.ch[set].ch2);
          PPM[(set << 3) + 3] = calculateChannel(ppmWork.sbus.ch[set].ch3);
          PPM[(set << 3) + 4] = calculateChannel(ppmWork.sbus.ch[set].ch4);
          PPM[(set << 3) + 5] = calculateChannel(ppmWork.sbus.ch[set].ch5);
          PPM[(set << 3) + 6] = calculateChannel(ppmWork.sbus.ch[set].ch6);
          PPM[(set << 3) + 7] = calculateChannel(ppmWork.sbus.ch[set].ch7);
        }

#ifdef DEBUG_DUMP_PPM
        ppmDump = 1;
#endif
        ppmAge = 0;
      }

      frameIndex = 0;
    }
  } else {
    frameIndex = 0;
  }
}