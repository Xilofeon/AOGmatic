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

//-----------------------------------------------------------------------------------------------
// Change this number to reset and reload default parameters To EEPROM
#define EEP_Ident 0x5425  

//the default network address
struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes
//-----------------------------------------------------------------------------------------------

#include <EEPROM.h> 
#include <Wire.h>
#include "EtherCard_AOG.h"
#include <IPAddress.h>

// ethernet interface ip address
static uint8_t myip[] = { 0,0,0,123 };

// gateway ip address
static uint8_t gwip[] = { 0,0,0,1 };

//DNS- you just need one anyway
static uint8_t myDNS[] = { 8,8,8,8 };

//mask
static uint8_t mask[] = { 255,255,255,0 };

//this is port of this autosteer module
uint16_t portMy = 5123;

//sending back to where and which port
static uint8_t ipDestination[] = { 0,0,0,255 };
uint16_t portDestination = 9999; //AOG port that listens

// ethernet mac address - must be unique on your network
static uint8_t mymac[] = { 0x00,0x00,0x56,0x00,0x00,0x7B };

uint8_t Ethernet::buffer[200]; // udp send and receive buffer

//Program counter reset
void(*resetFunc) (void) = 0;

//ethercard 10,11,12,13 Nano = 10 depending how CS of ENC28J60 is Connected
#define CS_Pin 10

//Communication with AgOpenGPS
uint16_t EEread = 0;

//pins:
#define NUM_OF_SECTIONS 7 //16 relays max for PCA9685
#define PinAogReady 9 //Pin AOG Ready
#define PinAogConnected A3 //Pin AOG Connected
#define AutoSwitch A1  //Switch Mode Auto On/Off
#define ManuelSwitch A2 //Switch Mode Manuel On/Off
#define PinDemoMode A0 //launches the demonstration of the servomotors. Advanced in a field in simulation in AOG... admired the result!!!
const uint8_t switchPinArray[] = {2, 3, 4, 5, 6, 7, 8}; //Pins, Switch activation sections or led    //<- max 7 sections
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

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

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

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0;
int16_t tempHeader = 0;

//hello from AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

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
  pinMode(PinAogConnected, OUTPUT);
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

  digitalWrite(PinAogConnected, HIGH);
  digitalWrite(PinAogReady, !readyIsActive);
  
  Serial.begin(38400);  //set up communication
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB
  }
  Serial.println(F(""));
  Serial.println(F("Firmware : AOGmatic UDP"));
  Serial.print(F("Version : "));
  Serial.println(VERSION);
  
  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(6, aogConfig);
    EEPROM.put(20, fonction);
    EEPROM.put(50, networkAddress);
  }
  else
  {
    EEPROM.get(6, aogConfig);
    EEPROM.get(20, fonction);
    EEPROM.get(50, networkAddress);
  }

  if (ether.begin(sizeof Ethernet::buffer, mymac, CS_Pin) == 0)
      Serial.println(F("Failed to access Ethernet controller"));

  //grab the ip from EEPROM
  myip[0] = networkAddress.ipOne;
  myip[1] = networkAddress.ipTwo;
  myip[2] = networkAddress.ipThree;

  gwip[0] = networkAddress.ipOne;
  gwip[1] = networkAddress.ipTwo;
  gwip[2] = networkAddress.ipThree;

  ipDestination[0] = networkAddress.ipOne;
  ipDestination[1] = networkAddress.ipTwo;
  ipDestination[2] = networkAddress.ipThree;

  //set up connection
  ether.staticSetup(myip, gwip, myDNS, mask);
  ether.printIp("_IP_: ", ether.myip);
  ether.printIp("GWay: ", ether.gwip);
  ether.printIp("AgIO: ", ipDestination);

  //register to port 8888
  ether.udpServerListenOnPort(&udpSteerRecv, 8888);

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
        digitalWrite(PinAogConnected, LOW);
        digitalWrite(PinAogReady, !readyIsActive);
      } else if (watchdogTimer > 240) digitalWrite(PinAogConnected, LOW);
    }
    
    //emergency off:
    if (watchdogTimer > 10) {
      switchSectionsOff();
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
        
        //off to AOG
        ether.sendUdp(AOG, sizeof(AOG), portMy, ipDestination, portDestination);
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
      
      //off to AOG
      ether.sendUdp(AOG, sizeof(AOG), portMy, ipDestination, portDestination);
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
  delay(1);

  //this must be called for ethercard functions to work. Calls udpSteerRecv() defined way below.
  ether.packetLoop(ether.packetReceive());
}

//callback when received packets
void udpSteerRecv(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, uint8_t* udpData, uint16_t len)
{
  //The data package
  if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
  {
    if (!aogConnected) {
      watchdogTimer = 12;
      digitalWrite(PinAogConnected, HIGH);
    }

    if (udpData[3] == 239) //machine Data
    {
      hydLift = udpData[7];
      tramline = udpData[8];  //bit 0 is right bit 1 is left
      geoStop = udpData[9];
      
      sectionLo = udpData[11];          // read relay control from AgOpenGPS
      sectionHi = udpData[12];
      
      if (aogConfig.isRelayActiveHigh)
      {
          tramline = 255 - tramline;
          sectionLo = 255 - sectionLo;
          sectionHi = 255 - sectionHi;
      }
      
      //reset watchdog
      watchdogTimer = 0;
  
      if (!aogConnected) {
        digitalWrite(PinAogReady, readyIsActive);
        aogConnected = true;
      }
    }
    else if (udpData[3] == 200) // Hello from AgIO
    {
      helloFromMachine[5] = sectionLo;
      helloFromMachine[6] = sectionHi;
      
      if (udpData[7] == 1)
      {
        sectionLo -= 255;
        sectionHi -= 255;
        watchdogTimer = 0;
      }

      helloFromMachine[5] = sectionLo;
      helloFromMachine[6] = sectionHi;

      ether.sendUdp(helloFromMachine, sizeof(helloFromMachine), portMy, ipDestination, portDestination);
    }
    else if (udpData[3] == 238)
    {
        aogConfig.raiseTime = udpData[5];
        aogConfig.lowerTime = udpData[6];
        aogConfig.enableToolLift = udpData[7];

        //set1 
        uint8_t sett = udpData[8];  //setting0     
        if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;

        aogConfig.user1 = udpData[9];
        aogConfig.user2 = udpData[10];
        aogConfig.user3 = udpData[11];
        aogConfig.user4 = udpData[12];

        //crc

        //save in EEPROM and restart
        EEPROM.put(6, aogConfig);
        resetFunc();
    }
    else if (udpData[3] == 201)
    {
        //make really sure this is the subnet pgn
        if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
        {
            networkAddress.ipOne = udpData[7];
            networkAddress.ipTwo = udpData[8];
            networkAddress.ipThree = udpData[9];

            //save in EEPROM and restart
            EEPROM.put(50, networkAddress);
            resetFunc();
        }
    }
    //Scan Reply
    else if (udpData[3] == 202)
    {
        //make really sure this is the subnet pgn
        if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
        {
            uint8_t scanReply[] = { 128, 129, 123, 203, 7, 
                networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 123,
                src_ip[0], src_ip[1], src_ip[2], 23   };

            //checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
            {
                CK_A = (CK_A + scanReply[i]);
            }
            scanReply[sizeof(scanReply)-1] = CK_A;

            static uint8_t ipDest[] = { 255,255,255,255 };
            uint16_t portDest = 9999; //AOG port that listens

            //off to AOG
            ether.sendUdp(scanReply, sizeof(scanReply), portMy, ipDest, portDest);
        }
    }
    else if (udpData[3] == 236) //Sections Settings 
    {
        for (uint8_t i = 0; i < 16; i++)
        {
            fonction[i] = udpData[i + 5];
        }

        //save in EEPROM and restart
        EEPROM.put(20, fonction);
    }
  }
}

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
