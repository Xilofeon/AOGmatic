    /* V2.70 - 30/07/2023 - Daniel Desmartins
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

//pins:                                                                                    UPDATE YOUR PINS!!!    //<-
#define NUM_OF_SECTIONS 7 //16 relays max for PCA9685                                                             //<-
#define PinAogReady 9 //Pin AOG Conntected                                                                        //<- 
#define AutoSwitch 10  //Switch Mode Auto On/Off                                                                  //<-
#define ManuelSwitch 11 //Switch Mode Manuel On/Off                                                               //<-
#define WorkWithoutAogSwitch A5 //Switch for work without AOG (optional)                                          //<-
#define PinDemoMode A0 //launches the demonstration of the servomotors. Advanced in a field in simulation in AOG... admired the result!!!
const uint8_t switchPinArray[] = {2, 3, 4, 5, 6, 7, 8, 12, A1, A2, A3, A4}; //Pins, Switch activation sections    //<- max 12 sections
//#define WORK_WITHOUT_AOG //Allows to use the box without aog connected (optional)
#define WORK_WITHOUT_CONTROL //commenter si vous voulez utiliser le code avec controle par intérrupteur
//#define EEPROM_USE //comment out if not use EEPROM and AOG config machine (not currently used in this code)
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
uint8_t sectionLo = 0, sectionHi = 0;

uint8_t count = 0;

boolean autoModeIsOn = false;
boolean manuelModeIsOn = false;
boolean aogConnected = false;
boolean firstConnection = true;

uint8_t onLo = 0, offLo = 0, onHi = 0, offHi = 0, mainByte = 0;
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
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(200); //wait for IO chips to get ready
  switchRelaisOff();
} //end of setup

void loop() {
  currentTime = millis();
  if (currentTime - lastTime >= loopTime) {  //start timed loop
    lastTime = currentTime;
    
    //avoid overflow of watchdogTimer:
    if (watchdogTimer++ > 250) watchdogTimer = 12;
    
    #ifdef WORK_WITHOUT_AOG
    while ((watchdogTimer > 10) && !analogRead(WorkWithoutAogSwitch)) {
      for (count = 0; count < NUM_OF_SECTIONS; count++) {
        if (digitalRead(switchPinArray[count]) || (digitalRead(AutoSwitch) && digitalRead(ManuelSwitch))) {
          setSection(count, false); //Section OFF
        } else {
          setSection(count, true); //Section ON
        }
      }
      if (serialResetTimer++ < 100 || serialResetTimer > 110) {
        watchdogTimer = serialResetTimer = 100;
        if (timeOfReturnNeutral) returnNeutralPosition();
      }
      delay(10);
      if (Serial.available() > 4) {
        lastTime = 200 + millis();
        break;
      }
    }
    #endif
    
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
      switchRelaisOff();
      
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
          setSection(count, bitRead(sectionLo, count)); //Open or Close sectionLo if AOG requests it in auto mode
          digitalWrite(switchPinArray[count], bitRead(sectionLo, count));
        } else {
          setSection(count, bitRead(sectionHi, count-8)); //Open or Close  le sectionHi if AOG requests it in auto mode
          digitalWrite(switchPinArray[count], bitRead(sectionHi, count-8));
        }
      }
      
      //Add For control Master swtich
      if (NUM_OF_SECTIONS < 16) {
        setSection(15, (sectionLo || sectionHi));
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
              setSection(count, true); //Section ON
            } else {
              if (count < 8) {
                bitSet(offLo, count);
                bitClear(onLo, count);
              } else {
                bitSet(offHi, count-8);
                bitClear(onHi, count-8);
              }
              setSection(count, false); //Section OFF
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
            setSection(count, false); //Close the section
          } else { //Signal LOW ==> switch is closed
            if (count < 8) {
              bitClear(offLo, count);
              setSection(count, bitRead(sectionLo, count)); //Open or Close sectionLo if AOG requests it in auto mode
            } else {
              bitClear(offHi, count-8); 
              setSection(count, bitRead(sectionHi, count-8)); //Open or Close  le sectionHi if AOG requests it in auto mode
            }
          }
        }
      } else { //FirstConnection
        switchRelaisOff(); //All relays off!
        mainByte = 2;
      }

      //Add For control Master swtich
      if(NUM_OF_SECTIONS < 16) {
        setSection(15, (sectionLo || sectionHi));
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
      Serial.read();
      Serial.read();
      Serial.read();   //high,low bytes
      Serial.read();
      
      sectionLo = Serial.read();          // read relay control from AgOpenGPS
      sectionHi = Serial.read();
      
      //Bit 13 CRC
      Serial.read();
      
      //reset watchdog
      watchdogTimer = 0;
  
      //Reset serial Watchdog
      serialResetTimer = 0;

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn=dataLength=0;
      
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
    else { //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn=dataLength=0;
    }
  }
} //end of main loop

void switchRelaisOff() {  //that are the relais, switch all off
  for (count = 0; count < NUM_OF_SECTIONS; count++) {
    setSection(count, false);
  }
  onLo = onHi = 0;
  offLo = offHi = 0b11111111;
  
  //Add For control Master swtich
  if(NUM_OF_SECTIONS < 16) {
    setSection(15, false);
  }
}

void setSection(uint8_t section, bool sectionActive) {
  if (sectionActive && !lastPositionMove[section]) {
    setPosition(section, positionOpen[section]);
    lastPositionMove[section] = true;
    lastTimeSectionMove[section] = 0;
  } else if (!sectionActive && lastPositionMove[section]) {
    setPosition(section, positionClosed[section]);
    lastPositionMove[section] = false;
    lastTimeSectionMove[section] = 0;
  }
}

void returnNeutralPosition() {
  uint8_t tmp = 0;
  for (count = 0; count < NUM_OF_SECTIONS; count++) {
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

void setPosition(uint8_t section, uint16_t angle) {
  uint16_t t_position = map(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(section, 0, t_position);
}
