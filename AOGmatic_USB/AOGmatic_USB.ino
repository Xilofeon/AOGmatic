#define VERSION 2.92
    /*  20/04/2025 - Daniel Desmartins
     *  in collaboration and test with Lolo85 and BricBric
     *  Connected to the Relay Port in AgOpenGPS
     *  If you find any mistakes or have an idea to improove the code, feel free to contact me. N'hésitez pas à me contacter en cas de problème ou si vous avez une idée d'amélioration.
     *
     *  simulateur   ecrivain
     *  Mode Demo, launches the demonstration of the servomotors. 
     *  Advanced in a field in simulation in AOG... admired the result!!!
     *  To activate do pulse on A0 to ground!
     *  bric bric  29/07/2023
     */

//pins:
#define NUM_OF_SECTIONS 7 //16 relays max for PCA9685
#define PinAogReady 9 //Pin AOG Ready
#define AutoSwitch 10  //Switch Mode Auto On/Off
#define ManuelSwitch 11 //Switch Mode Manuel On/Off
#define PinDemoMode A0 //launches the demonstration of the servomotors. Advanced in a field in simulation in AOG... admired the result!!!
const uint8_t switchPinArray[] = {2, 3, 4, 5, 6, 7, 8, 12, A1, A2, A3}; //Pins, Switch activation sections    //<- max 11 sections
#define WORK_WITHOUT_CONTROL //commenter si vous voulez utiliser le code avec controle par intérrupteur
bool readyIsActive = LOW;

///////////Régler ici la position d'ouverture, fermeture et neutre de vos servotmoteur (dans l'orde de 1 à 16)/////////////
uint8_t positionOpen[] =    { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};   //position ouvert en degré
uint8_t positionNeutral[] = { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};   //position neutre en degré
uint8_t positionClosed[] =  {155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155};   //position fermé en degré
#define TIME_RETURN_NEUTRAL 10 //temps de retour au neutre en cycle 10 = 1000ms //0 = pas de retour au neutre et trop court pas le temps d'aller à la position demandée!

#define ANGLE_MIN 0
#define ANGLE_MAX 180

//Réglage des servomoteurs Attention les positions max et min doivent être définies au préalable par vous même pour vos servomoteurs.
#define SERVO_MIN 90
#define SERVO_MAX 540
#define SERVO_FREQ 50 //Généralement par defaut 50hz pour les SG90

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#include <EEPROM.h>
#define EEP_Ident 0x5400

//Program counter reset
void(* resetFunc) (void) = 0;

//Variables for config - 0 is false  
struct Config {
    uint8_t raiseTime = 2;
    uint8_t lowerTime = 4;
    uint8_t enableToolLift = 0;
    uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

    uint8_t user1 = 0; //user defined values set in machine tab
    uint8_t user2 = 0;
    uint8_t user3 = 0;
    uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes

uint8_t fonction[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
bool fonctionState[] = { false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false };

//Demo Mode
const uint8_t message[] = { 0, 0, 0, 126, 9, 9, 126, 0, 0, 62, 65, 65, 62, 0, 0, 62, 65, 73, 58, 0, 0, 127, 2, 4, 2, 127, 0, 0, 126, 9, 9, 126, 0, 1, 1, 127, 1, 1, 0, 0, 65, 127, 65, 0, 0, 62, 65, 65, 34, 0, 0, 0, 0};
const uint16_t tableau = 530;
uint16_t boucle = 0;
bool demoMode = false;

//Variables:
const uint8_t loopTime = 100; //10hz
uint32_t lastTime = loopTime;
uint32_t currentTime = loopTime;
uint8_t lastTimeSectionMove[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
bool lastPositionMove[] = {true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true};

//Comm checks
uint8_t watchdogTimer = 12;     //make sure we are talking to AOG
uint8_t serialResetTimer = 0;   //if serial buffer is getting full, empty it

bool isRaise = false;
bool isLower = false;

//Communication with AgOpenGPS
uint16_t EEread = 0;

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0;
int16_t tempHeader = 0;

//hello from AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };
bool helloUDP = false;
//show life in AgIO
uint8_t helloAgIO[] = { 0x80, 0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0x6D };
uint8_t helloCounter = 0;

uint8_t AOG[] = { 0x80, 0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

//The variables used for storage
uint8_t sectionLo = 0, sectionHi = 0, tramline = 0, hydLift = 0, geoStop = 0;

uint8_t count = 0;

boolean autoModeIsOn = false;
boolean manuelModeIsOn = false;
boolean aogConnected = false;
boolean firstConnection = true;

uint8_t onLo = 0, offLo = 0, onHi = 0, offHi = 0, mainByte = 0;

uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;
//End of variables

void setup() {  
  pinMode(PinAogReady, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(AutoSwitch, INPUT_PULLUP);  //INPUT_PULLUP: no external Resistor to GND or to PINx is needed, PULLUP: HIGH state if Switch is open! Connect to GND and D0/PD0/RXD
  pinMode(ManuelSwitch, INPUT_PULLUP);
  pinMode(PinDemoMode, INPUT_PULLUP);
  #ifdef WORK_WITHOUT_CONTROL
  for (count = 0; count < NUM_OF_SECTIONS; count++) {
    pinMode(switchPinArray[count], OUTPUT);
    digitalWrite(switchPinArray[count], LOW);
  }
  #else
  for (count = 0; count < NUM_OF_SECTIONS; count++) {
    pinMode(switchPinArray[count], INPUT_PULLUP);
  }
  #endif

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(PinAogReady, !readyIsActive);
  
  Serial.begin(38400);  //set up communication
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB
  }
  Serial.println(F(""));
  Serial.println(F("Firmware : AOGmatic USB"));
  Serial.print(F("Version : "));
  Serial.println(VERSION);
  
  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(6, aogConfig);
    EEPROM.put(20, fonction);
  }
  else
  {
    EEPROM.get(6, aogConfig);
    EEPROM.get(20, fonction);
  }

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(200); //wait for IO chips to get ready
  switchSectionsOff();
  setSection();
} //end of setup

void loop() {
  currentTime = millis();
  if (currentTime - lastTime >= loopTime) {  //start timed loop
    lastTime = currentTime;
    
    //avoid overflow of watchdogTimer:
    if (watchdogTimer++ > 250) watchdogTimer = 12;
    
    //clean out serial buffer to prevent buffer overflow:
    if (serialResetTimer++ > 20) {
      while (Serial.available() > 0) Serial.read();
      serialResetTimer = 0;
    }
    
    if (TIME_RETURN_NEUTRAL) returnNeutralPosition();
    
    if (watchdogTimer > 20) {
      if (aogConnected && watchdogTimer > 60) {
        aogConnected = false;
        firstConnection = true;
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(PinAogReady, !readyIsActive);
      } else if (watchdogTimer > 240) digitalWrite(LED_BUILTIN, LOW);
    }
    
    //emergency off:
    if (watchdogTimer > 10) {
      switchSectionsOff();
      
      //show life in AgIO
      if (++helloCounter > 10 && !helloUDP) {
        Serial.write(helloAgIO, sizeof(helloAgIO));
        Serial.flush();   // flush out buffer
        helloCounter = 0;
      }
    #ifdef WORK_WITHOUT_CONTROL
      for (count = 0; count < NUM_OF_SECTIONS; count++) {
        digitalWrite(switchPinArray[count], LOW);
      }
    } else {
      for (count = 0; count < NUM_OF_SECTIONS; count++) {
        if (count < 8) {
          fonctionState[count] = bitRead(sectionLo, count); //Open or Close sectionLo if AOG requests it in auto mode
          digitalWrite(switchPinArray[count], bitRead(sectionLo, count));
        } else {
          fonctionState[count] = bitRead(sectionHi, count-8); //Open or Close  le sectionHi if AOG requests it in auto mode
          digitalWrite(switchPinArray[count], bitRead(sectionHi, count-8));
        }
      }
      
      //Add For control Master swtich
      if (NUM_OF_SECTIONS < 16) {
        fonctionState[15] = (sectionLo || sectionHi);
      }
      
      //show life in AgIO
      if (++helloCounter > 10 && !helloUDP) {
        Serial.write(helloAgIO, sizeof(helloAgIO));
        Serial.flush();   // flush out buffer
        helloCounter = 0;
      }
      
      //Demo Mode
      if (demoMode) {
        if (boucle++ > tableau) boucle = 0;
        uint8_t boom = message[boucle/10];

        AOG[9] = (uint8_t)boom; //onLo;
        AOG[10] = (uint8_t)~boom; //offLo;

        //checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(AOG) - 1; i++)
        {
          CK_A = (CK_A + AOG[i]);
        }
        AOG[sizeof(AOG) - 1] = CK_A;
        
        Serial.write(AOG, sizeof(AOG));
        Serial.flush();   // flush out buffer
      } else {
        demoMode = !digitalRead(PinDemoMode);
      }
   	}
    #else
    } else {
      //check Switch if Auto/Manuel:
      autoModeIsOn = !digitalRead(AutoSwitch); //Switch has to close for autoModeOn, Switch closes ==> LOW state ==> ! makes it to true
      if (autoModeIsOn) {
        mainByte = 1;
      } else {
        mainByte = 2;
        manuelModeIsOn = !digitalRead(ManuelSwitch);
        if (!manuelModeIsOn) firstConnection = false;
      }
      
      if (!autoModeIsOn) {
        if(manuelModeIsOn && !firstConnection) { //Mode Manuel
          for (count = 0; count < NUM_OF_SECTIONS; count++) {
            if (!digitalRead(switchPinArray[count])) { //Signal LOW ==> switch is closed
              if (count < 8) {
                bitClear(offLo, count);
                bitSet(onLo, count);
              } else {
                bitClear(offHi, count-8);
                bitSet(onHi, count-8);
              }
              fonctionState[count] =  true; //Section ON
            } else {
              if (count < 8) {
                bitSet(offLo, count);
                bitClear(onLo, count);
              } else {
                bitSet(offHi, count-8);
                bitClear(onHi, count-8);
              }
              fonctionState[count] = false; //Section OFF
            }
          }
        } else { //Mode off
          switchRelaisOff(); //All relays off!
        }
      } else if (!firstConnection) { //Mode Auto
        onLo = onHi = 0;
        for (count = 0; count < NUM_OF_SECTIONS; count++) {
          if (digitalRead(switchPinArray[count])) {
            if (count < 8) {
              bitSet(offLo, count); //Info for AOG switch OFF
            } else {
              bitSet(offHi, count-8); //Info for AOG switch OFF
            }
            fonctionState[count] = false; //Close the section
          } else { //Signal LOW ==> switch is closed
            if (count < 8) {
              bitClear(offLo, count);
              fonctionState[count] = bitRead(sectionLo, count); //Open or Close sectionLo if AOG requests it in auto mode
            } else {
              bitClear(offHi, count-8); 
              fonctionState[count] = bitRead(sectionHi, count-8); //Open or Close  le sectionHi if AOG requests it in auto mode
            }
          }
        }
      } else { //FirstConnection
        switchRelaisOff(); //All relays off!
        mainByte = 2;
      }

      //Add For control Master swtich
      if(NUM_OF_SECTIONS < 16) {
        fonctionState[15] = (sectionLo || sectionHi);
      }
      
      //Send to AOG
      AOG[5] = (uint8_t)mainByte;
      AOG[9] = (uint8_t)onLo;
      AOG[10] = (uint8_t)offLo;
      AOG[11] = (uint8_t)onHi;
      AOG[12] = (uint8_t)offHi;
      
      //add the checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < sizeof(AOG)-1; i++)
      {
        CK_A = (CK_A + AOG[i]);
      }
      AOG[sizeof(AOG)-1] = CK_A;
      
      Serial.write(AOG, sizeof(AOG));
      Serial.flush();   // flush out buffer
    }
    #endif
    
    //hydraulic lift

    if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
    {
        lastTrigger = hydLift;
        lowerTimer = 0;
        raiseTimer = 0;

        //100 msec per frame so 10 per second
        switch (hydLift)
        {
            //lower
        case 1:
            lowerTimer = aogConfig.lowerTime * 10;
            break;

            //raise
        case 2:
            raiseTimer = aogConfig.raiseTime * 10;
            break;
        }
    }

    //countdown if not zero, make sure up only
    if (raiseTimer)
    {
        raiseTimer--;
        lowerTimer = 0;
    }
    if (lowerTimer) lowerTimer--;

    //if anything wrong, shut off hydraulics, reset last
    if ((hydLift != 1 && hydLift != 2) || watchdogTimer > 10) //|| gpsSpeed < 2)
    {
        lowerTimer = 0;
        raiseTimer = 0;
        lastTrigger = 0;
    }

    if (aogConfig.isRelayActiveHigh)
    {
        isLower = isRaise = false;
        if (lowerTimer) isLower = true;
        if (raiseTimer) isRaise = true;
    }
    else
    {
        isLower = isRaise = true;
        if (lowerTimer) isLower = false;
        if (raiseTimer) isRaise = false;
    }

    //set sections
    setSection();
  }

  // Serial Receive
  //Do we have a match with 0x8081?    
  if (Serial.available() > 4 && !isHeaderFound && !isPGNFound) 
  {
    uint8_t temp = Serial.read();
    if (tempHeader == 0x80 && temp == 0x81)
    {
      isHeaderFound = true;
      tempHeader = 0;
    }
    else
    {
      tempHeader = temp;     //save for next time
      return;
    }
  }

  //Find Source, PGN, and Length
  if (Serial.available() > 2 && isHeaderFound && !isPGNFound)
  {
    Serial.read(); //The 7F or less
    pgn = Serial.read();
    dataLength = Serial.read();
    isPGNFound = true;
    
    if (!aogConnected) {
      watchdogTimer = 12;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  //The data package
  if (Serial.available() > dataLength && isHeaderFound && isPGNFound)
  {
    if (pgn == 239) // EF Machine Data
    {
      Serial.read();
      Serial.read();
      
      hydLift = Serial.read();
      tramline = Serial.read();  //bit 0 is right bit 1 is left
      
      geoStop = Serial.read();
      Serial.read();
      
      sectionLo = Serial.read(); // read section control from AgOpenGPS
      sectionHi = Serial.read();
      
      if (aogConfig.isRelayActiveHigh)
      {
          tramline = 255 - tramline;
          sectionLo = 255 - sectionLo;
          sectionHi = 255 - sectionHi;
      }
      
      //Bit 13 CRC
      Serial.read();
      
      //reset watchdog
      watchdogTimer = 0;
  
      //Reset serial Watchdog
      serialResetTimer = 0;

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
      
      if (!aogConnected) {
        digitalWrite(PinAogReady, readyIsActive);
        aogConnected = true;
      }
    }
    else if (pgn == 200) // Hello from AgIO
    {
      helloUDP = true;
      
      Serial.read(); //Version
      Serial.read();
      
      if (Serial.read())
      {
        sectionLo -= 255;
        sectionHi -= 255;
        watchdogTimer = 0;
      }
	  
      //crc
      Serial.read();
      
      helloFromMachine[5] = sectionLo;
      helloFromMachine[6] = sectionHi;
      
      Serial.write(helloFromMachine, sizeof(helloFromMachine));
      
      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }
    else if (pgn == 238) //EE Machine Settings 
    {
        aogConfig.raiseTime = Serial.read();
        aogConfig.lowerTime = Serial.read();
        aogConfig.enableToolLift = Serial.read();

        //set1 
        uint8_t sett = Serial.read();  //setting0     
        if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;

        aogConfig.user1 = Serial.read();
        aogConfig.user2 = Serial.read();
        aogConfig.user3 = Serial.read();
        aogConfig.user4 = Serial.read();

        //crc
        Serial.read();

        //save in EEPROM and restart
        EEPROM.put(6, aogConfig);
        resetFunc();

        //reset for next pgn sentence
        isHeaderFound = isPGNFound = false;
        pgn = dataLength = 0;
    }
    else if (pgn == 236) //Sections Settings 
    {
        for (uint8_t i = 0; i < 24; i++)
        {
          if (i < 16)
            fonction[i] = Serial.read();
          else
            Serial.read();
        }

        EEPROM.put(20, fonction);

        //reset for next pgn sentence
        isHeaderFound = isPGNFound = false;
        pgn = dataLength = 0;
    }
    else { //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }
  }
} //end of main loop

void switchSectionsOff() {  //that are the sections, switch all off
  for (count = 0; count < NUM_OF_SECTIONS; count++) {
    fonctionState[count] = false;
  }
  onLo = onHi = 0;
  offLo = offHi = 0b11111111;
  
  //Add For control Master swtich
  if(NUM_OF_SECTIONS < 16) {
    fonctionState[count] = false;
  }
}

void setSection() {
  fonctionState[16] = isLower;
  fonctionState[17] = isRaise;
  
  //Tram
  fonctionState[18] = bitRead(tramline, 0); //right
  fonctionState[19] = bitRead(tramline, 1); //left
  
  //GeoStop
  fonctionState[20] =  geoStop;
  
  bool t_sectionActive = false;
  for (count = 0; count < 16; count++) {
    bool t_sectionActive = fonctionState[fonction[count] - 1];
    if (t_sectionActive && !lastPositionMove[count]) {
      setPosition(count, positionOpen[count]);
      lastPositionMove[count] = true;
      lastTimeSectionMove[count] = 0;
    } else if (!t_sectionActive && lastPositionMove[count]) {
      setPosition(count, positionClosed[count]);
      lastPositionMove[count] = false;
      lastTimeSectionMove[count] = 0;
    }
  }
}

void returnNeutralPosition() {
  uint8_t tmp = 0;
  for (count = 0; count < 16; count++) {
    tmp = lastTimeSectionMove[count];
    if (tmp != 255) {
      if (tmp < TIME_RETURN_NEUTRAL) {
        tmp++;
      } else {
        setPosition(count, positionNeutral[count]);
        tmp = 255;
      }
    }
    lastTimeSectionMove[count] = tmp;
  }
}

void setPosition(uint8_t t_section, uint16_t angle) {
  uint16_t t_position = map(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(t_section, 0, t_position);
}
