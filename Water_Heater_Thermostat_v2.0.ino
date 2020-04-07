/*

Water Heater Thermostat is a device that controls any kind of pump in a solar system 
or in a classical boiler

The operating principle is as follows:
- a DS18B20 probe at the heat source compares the temperature with a 
second DS18B20 probe placed in hot water tank.
- beyond a deltaTon threshold the pump is started, stops below deltaToff
- pump starts when Tmini freezes
- an alarm system reports the Tmaxi temperature exceedance
- the thresholds - deltaTon, deltaToff, Tmaxi and Tmini - as well as the pump model are 
configurable and backed up without power
- support of the PWM control for externally controlled pumps.
  Following the measurements made on the Steca TR 503 regulator, the PWM signal has the 
  the following characteristics:
  - 250Hz with variable cyclic ratio from 0 (stop) to 100% (total pump operation) under 10V,
  - the pump is continuously supplied by the 230V,
  - 50% at start-up, the ratio gradually increases to 100%,
  - when Tcumulus reaches deltaTpwm, the cyclic ratio may decrease down to 25%.

The programme provides for:
- 1 Arduino Uno R3
- 2 DS18B20 temperature sensor 
- 1 resistor of 4.7k between data bus and +5V of DS18B20 sensors
- 1 SRG
- 1 16x2 LCD display for Arduino with I2C extension for serial mode,
- 3 push buttons

_________________________________________________________________
|                                                               |
|       author : Philippe de Craene <dcphilippe@yahoo.fr>       |
|           any feedback is welcome                             |
_________________________________________________________________

Hardware pinup:
---------------

numeric output  2   => SSR 		= logic I active SSR drive
numeric output  3   => PWM 		= drives the 250Hz PWM signal, output 3 required for Timer2
numeric output  4   => ALARM 	= red LED
numeric output  5   => SSR_INV  = logic 0 active SSR drive
numeric output  7   => DS18B20 	= 1-Wire data bus for DS18B20
numeric output  8   => ENTRY 	= push-button
numeric output  9   => PLUS 	= push-button
numeric output 10   => MINUS	= bouton poussoir
analog input   A4   => SDA for display
analog input   A5   => SCL for display


Versions history:
-----------------

version 1    - 18 août 2018       - transformation pour sondes numériques et le LCD avec I2C
version 1.11 - 20 août 2018       - correction  + optimisation 
version 1.2  - 28 août 2018       - corrections de bugs
version 1.3  - 25 nov. 2018       - adaptation pour SSR actif à LOW
version 1.4  - 20 mai  2019       - mise à jour des liens pour télécharger les bibliothèques
version 1.41 - 22 aout 2019       - mises à jour diverses
version 2.0  - 5 april 2020       - update for boiler driven by SSR + ALARM management + English translation
 
*/

#include <EEPROM.h>
#include <Wire.h>  
#include <OneWire.h>             // https://github.com/PaulStoffregen/OneWire
#include <LiquidCrystal_I2C.h>   // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library

// Inputs/outputs:
// A4 => SDA for display
// A5 => SCL for display
const byte ENTRYpin   = 8;    // 3 push-buttons
const byte MINUSpin   = 9;
const byte PLUSpin    = 10;
const byte DS18B20    = 7;    // 1-Wire data for DS18B20
const byte SSRpin     = 2;    // SSR drive + yellow LED
const byte SSR_INVpin = 5;    // inverted SSR drive
const byte PWMpin     = 3;    // 244Hz PWM signal + blue LED
const byte ALARMpin   = 4;    // red LED for ALARM

// default parameters:
int Tmaxi_d      = 85;        // first use maximum temperature = 85°C
int Tmini_d      = -1;        // first use frost temperature = -1°C
int deltaTon_d   =  5;        // first use starting threshold = 5°K
int deltaToff_d  =  2;        // first use stoping threshold = 2°K
int deltaTpwm_d  =  1;        // first use PWM ration management threshold = 1°K
byte pump_model_d = 0;        // pump model, first use = 0
                              // 0: non-PWM pump, 1: rising PWM , 2: falling PWM
byte ssr_rule_d   = 0;        // SSR usage drive, first use =0
                              // 0: pump, 1: boiler, 2: Tmini alarm, 3: Tmaxi alarm

// LCD with I2C declaration :
LiquidCrystal_I2C lcd(0x27, 16, 2);

// specific symbols to display:
byte degree[8] =        { 0b11100,  0b10100,  0b11100,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000};
byte fleche_bas[8] =    { 0b00100,  0b00100,  0b00100,  0b00100,  0b00100,  0b11111,  0b01110,  0b00100};
byte fleche_haut[8] =   { 0b00100,  0b01110,  0b11111,  0b00100,  0b00100,  0b00100,  0b00100,  0b00100};
byte fleche_stable[8] = { 0b00100,  0b01110,  0b11111,  0b00100,  0b00100,  0b11111,  0b01110,  0b00100};

// DS18B20 declaration:
OneWire sondeDS(DS18B20);

// variables for temperature management:
float Tsource, Tcumulus, memo_Tcumulus;  // past value for tendency calculation
int deltaTon, deltaToff, deltaTpwm;
int Tmaxi, Tmini;
byte tendency = 4;             // tendency 2=increase, 3=decrese, 4=stable

// variables for process management:
byte pump_model;               // 0: non PWM, 1: rising PWM, 2: falling PWM
byte ssr_rule;                 // 0: pump, 1: boiler, 2: Tmini alarm, 3: Tmaxi alarm
byte state, memo_state = 0;    // give the overoll state of the device:
                               // 0: off, 1: on, 2: Tmini, 3: Tmaxi, 4: forced, 5: boiler
bool mforce = LOW;             // flag if pump always ON
bool boiler = LOW;             // flag if a boiler is driven
unsigned long memo_tempo = 0;  // flag for time count

// variables for PWM:
byte ratio = 0;                // initial PWM duty cycle
byte PWM;                      // final PWM (take in account if Rising or Falling PWM)
const byte PWMmini = 25;       // minimum ratio when pup is active

// variables for push-buttons & menus
const char label_pump[] = {'A', 'M', 'G', 'T', 'F', 'S' };
       // label displayed : A: off, M: on, G: Tmini, T: Tmaxi, F: forced, S: boiler
byte ret_push_button, memo_ret_push_button = 0;   // function return value when a push-button is pushed
String label;
byte window = 0;               // index of window to display
byte windowsTotal = 11;        // total number of windows (including window 0)
byte count_before_timeout = 0;
byte timeout = 60;             // display switch off delay
bool eeprom = false;           // flag a backup needed

//
// SETUP
//_____________________________________________________________________________________________

void setup() {

  lcd.begin();

  pinMode (ENTRYpin,   INPUT_PULLUP);
  pinMode (PLUSpin,    INPUT_PULLUP);
  pinMode (MINUSpin,   INPUT_PULLUP);
  pinMode (ALARMpin,   OUTPUT);
  pinMode (SSRpin,     OUTPUT);
  pinMode (SSR_INVpin, OUTPUT);
  pinMode (PWMpin,     OUTPUT);

// Timer2 set for PWM at 244Hz
  TCCR2A = 0b00100011;  // Fast PWM Mode
  TCCR2B = 0b00000110;  // formula => binary = 62500 / (F * 256)
  OCR2B = 0;            // duty ration is 0 at startup
  
// LCD => special homemade characters are assigned
  lcd.createChar(1, degree );        // =1, etc...
  lcd.createChar(2, fleche_haut );
  lcd.createChar(3, fleche_stable );
  lcd.createChar(4, fleche_bas );

// get data from the EEPROM, type byte. So offset of 50 for negative temperature values :
// defaut EEPROM content is 255 at very first usage.
  if(EEPROM.read(0) < 70) { deltaTon = EEPROM.read(0) - 50; }
  else { EEPROM.write(0, deltaTon_d + 50); deltaTon = deltaTon_d; }
  if(EEPROM.read(1) < 60) { deltaToff = EEPROM.read(1) - 50; }
  else { EEPROM.write(1, deltaToff_d + 50); deltaToff = deltaToff_d; }
  if(EEPROM.read(2) < 60) { deltaTpwm = EEPROM.read(2) - 50; }
  else { EEPROM.write(2, deltaTpwm_d + 50); deltaTpwm = deltaTpwm_d; }
  if(EEPROM.read(3) < 150) { Tmaxi = EEPROM.read(3) - 50; }
  else { EEPROM.write(3, Tmaxi_d + 50); Tmaxi = Tmaxi_d; }
  if(EEPROM.read(4) < 60) { Tmini = EEPROM.read(4) - 50; }
  else { EEPROM.write(4, Tmini_d + 50); Tmini = Tmini_d; }
  if(EEPROM.read(5) < 3) { pump_model = EEPROM.read(5); }
  else { EEPROM.write(5, pump_model_d); pump_model = pump_model_d; }
  if(EEPROM.read(6) < 4) { ssr_rule = EEPROM.read(6); }
  else { EEPROM.write(6, ssr_rule_d); ssr_rule = ssr_rule_d; }

// console and LCD initialisation
  Serial.begin(9600);
  Serial.println("ready ...");
  Serial.println ();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Thermostat by");
  lcd.setCursor(0, 1);
  lcd.print("PhilippeDC");
  delay(500); 
}      // end of setup

//
// LOOP
//____________________________________________________________________________________________

void loop() {

  unsigned long tempo = millis()/800;       // time count a littke less than one seconds
  if( memo_tempo == tempo ) return;         // everything after is done every second
  memo_tempo = tempo;
  static unsigned int heatTimeOut = 0;      // counter for boiler offline alarm

// STEP 1: get the temperature values
//____________________________________________________________________________________________

// as it is needed 1.6s to get the temperature for both sensors, this is done 
// only when we are out of the parameters setting
  if(window== 0) { 
     GetTemperature(&Tsource, true);         // true only for the first reading
     GetTemperature(&Tcumulus, false); }

// STEP 2: compare temperature with the thresholds
//____________________________________________________________________________________________

// deltaTonn is reached : the pump is starting
  memo_state = state;                        // remaind the past state
  if( Tsource-Tcumulus-deltaTon >= 0 ) {
     state = 1;
     if(memo_state == 0) ratio = 50; }        // duty cycle 50% when pump is starting

// the pump is running, PWM increases until 100%
  if((state==1)&&(Tsource-Tcumulus-deltaToff-deltaTpwm >0)&&(ratio < 100)) 
    ratio++;

// deltaTpwm is reached so PWM is starting to decrease
  if((state==1)&&(Tsource-Tcumulus-deltaToff-deltaTpwm <=0)&&(tendency==2)&&(ratio > PWMmini)) 
    ratio--;

// deltaToff is reached so the pump stops
  if(Tsource-Tcumulus-deltaToff <=0) { 
    state = 0;
    ratio = 0; }

// Tmini reached : the pump starts + ALARM
  if(Tsource <= Tmini) {
    state = 2;
    ratio = 50;
    Alarm(3); }                               // red LED blinks 3 times/second

// pump is always on
  if(mforce == HIGH) {
    state = 4;
    ratio = 50;
    Alarm(1); }                              // red LED blinks 1 time/second

// drive a boiler
  if(ssr_rule == 1) {
    state = 5;                               // start the boiler
    if(Tsource < 40) {                       // check the heat of the source   
      PWM = 0;                               // pump stop if no heat
      heatTimeOut++;                         // count seconds
      if( heatTimeOut > 375 ) Alarm(2); }    // after 375 count = 10 minutes
    else heatTimeOut = 0; }
      
// Tmaxi reached : the pump is stopped + ALARM
  if(Tcumulus >= Tmaxi || Tsource > Tmaxi ) {
    state = 3;
    ratio = 0;
    Alarm(5); }                               // red LED blinks 5 times/second

//  Step 3: outputs management
//____________________________________________________________________________________________

// PWM management:
// pump_model == 0 for NON-PWM pump
// pump_model == 1 for Rising PWM pump
// pump_model == 2 for Falling PWM pump

  if(pump_model == 1)      PWM = map(ratio, 0, 100, 0, 255);
  else if(pump_model == 2) PWM = map(ratio, 0, 100, 255, 0);
  else                     PWM = 0;

  analogWrite(PWMpin, PWM);              // commande du PWM / mli

// SSR management
// ssr_rule == 0 to drive pump (require if pump_model =0)
// ssr_rule == 1 to drive the boiler,
// ssr_rule == 2 to drive Tmini alarm,
// ssr_rule == 3 to drive Tmaxi alarm
  if( ssr_rule == 0 ) {                  // case NON-PWM pump
    if((state == 1)||(state==2)||(state==4)) {
      digitalWrite(SSRpin, HIGH); digitalWrite(SSR_INVpin, LOW); }
    else {
      digitalWrite(SSRpin, LOW); digitalWrite(SSR_INVpin, HIGH); }
  }    // end of ssr_rule == 0

  else if( ssr_rule == 1 ) {             // case boiler
    if( state == 5 ) {
      digitalWrite(SSRpin, HIGH); digitalWrite(SSR_INVpin, LOW); }
    else {
      digitalWrite(SSRpin, LOW); digitalWrite(SSR_INVpin, HIGH); }
  }    // end of ssr_rule == 1

  else if( ssr_rule == 2 ) {             // case alarm Tmini
    if( state == 2 ) {
      digitalWrite(SSRpin, HIGH); digitalWrite(SSR_INVpin, LOW); }
    else {
      digitalWrite(SSRpin, LOW); digitalWrite(SSR_INVpin, HIGH); }
  }    // end of ssr_rule == 2

  else if( ssr_rule == 3 ) {             // case alarm Tmaxi
    if( state == 3 ) {
      digitalWrite(SSRpin, HIGH); digitalWrite(SSR_INVpin, LOW); }
    else {
      digitalWrite(SSRpin, LOW); digitalWrite(SSR_INVpin, HIGH); }
  }    // end of ssr_rule == 2

// Step 4 : tendency calculation
//____________________________________________________________________________________________

  static unsigned int cycleCounter = 0;     // counter for tendencies
  if(++cycleCounter > 5) {
    cycleCounter = 0;
    if( Tcumulus >  memo_Tcumulus ) tendency = 2;    // draw rising arrow on display
    if( Tcumulus == memo_Tcumulus ) tendency = 3;    // draw stable arrow on display
    if( Tcumulus <  memo_Tcumulus ) tendency = 4;    // draw falling arrow on display
    memo_Tcumulus = Tcumulus;                        // keep past measure
  }    // end of test cycleCounter

// Step 5: display & parameters management
//____________________________________________________________________________________________

// check push-buttons & display ON / OFF 
//____________________________________________________________________________________________

  memo_ret_push_button = ret_push_button; // memorize past value
  ret_push_button = push_button();        // reading push-button status 
                                          // 0: nothing, 1: ENTRY, 2: PLUS, 3: MINUS
  count_before_timeout++;                 // timeout before switch off the display
  if( count_before_timeout > timeout ) {  // timeout reached
      window = 0;                         // return to first display
      lcd.clear();
      lcd.noBacklight();                  // light off display
  }

  if((memo_ret_push_button == 1)&&(ret_push_button == 1)) next_window();

// regular case: no push-buttonactivity
//____________________________________________________________________________________________

  if( window== 0 ) {
    lcd.setCursor(0, 0);
    lcd.print("SOURCE:");  
    if( Tsource < 100 ) lcd.print(" ");
    lcd.print(String(Tsource, 1));  
    lcd.write(1);                             // display character '°'
    lcd.print("C ");  
    lcd.setCursor(15, 0);   
    lcd.write(label_pump[state]);
    lcd.setCursor(0, 1);
    lcd.print("BALLON: ");
    lcd.print(String(Tcumulus, 1));
    lcd.write(1);
    lcd.print("C ");  
    lcd.setCursor(15, 1);
    lcd.write(tendency);                      // dram the arrow
  }    // end of windows == 0

// ENTRY button pushed
//____________________________________________________________________________________________

  if( window == 1 ) { 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LISTE DES");
    lcd.setCursor(0, 1);
    lcd.print("PARAMETRES :");
    delay(800);
    next_window();
  }    // end of windows == 1

// parameters setting

  if( window == 2 ) {  
    if(ret_push_button == 2) {
      deltaTon++;
      deltaTon = constrain(deltaTon, 0, 9); }    // limit value between 0 to 9
    if(ret_push_button == 3) {
      deltaTon--;
      deltaTon = constrain(deltaTon, deltaToff, 9); } 
    lcd.setCursor(0, 0);
    lcd.print("DELTA T MARCHE :");
    lcd.setCursor(0, 1);
    lcd.print(deltaTon);
    lcd.write(1);
    lcd.print("K  "); 
  }    // end of window == 2

  if( window == 3 ) {  
    if(ret_push_button == 2) {
      deltaToff++; 
      deltaToff = constrain(deltaToff, 0, deltaTon); } 
    if(ret_push_button == 3) {
      deltaToff--;
      deltaToff = constrain(deltaToff, 0, 9); } 
    lcd.setCursor(0, 0);
    lcd.print("DELTA T ARRET :");
    lcd.setCursor(0, 1);           
    lcd.print(deltaToff);                   
    lcd.write(1);
    lcd.print("K"); 
  }    // end of window == 3

  if( window == 4 ) {  
    if(ret_push_button == 2) {
      deltaTpwm++;
      deltaTpwm = constrain(deltaTpwm, 0, 9); }
    if(ret_push_button == 3) {
      deltaTpwm--;
      deltaTpwm = constrain(deltaTpwm, 0, 9); }
    lcd.setCursor(0, 0);
    lcd.print("DELTA T PWM :");
    lcd.setCursor(0, 1);
    lcd.print(deltaTpwm);
    lcd.write(1);
    lcd.print("K  "); 
  }    // end of window == 4

  if( window == 5 ) {  
    if(ret_push_button == 2) {
      Tmaxi++;
      Tmaxi = constrain(Tmaxi, 40, 100); }
    if(ret_push_button == 3) {
      Tmaxi--;
      Tmaxi = constrain(Tmaxi, 40, 100); }
    lcd.setCursor(0, 0);
    lcd.print("T MAXI ALARME :");
    lcd.setCursor(0, 1);
    lcd.print(Tmaxi);
    lcd.write(1);
    lcd.print("C  "); 
  }    // end of windows == 5

  if( window == 6 ) {  
    if(ret_push_button == 2) {
      Tmini++;
      Tmini = constrain(Tmini, -9, 9); }
    if(ret_push_button == 3) {
      Tmini--;
      Tmini = constrain(Tmini, -9, 9); }
    lcd.setCursor(0, 0);
    lcd.print("T HORS GEL :");
    lcd.setCursor(0, 1);
    lcd.print(Tmini);
    lcd.write(1);
    lcd.print("C  "); 
  }    // end of window == 6

  if( window == 7 ) {
    if(ret_push_button == 2) pump_model = (pump_model +1)%3;
    if(ret_push_button == 3) pump_model = (pump_model -1)%3;
    if(pump_model == 0)      label = "NON-PWM        ";
    else if(pump_model == 1) label = "PWM CROISSANT  ";
    else if(pump_model == 2) label = "PWM DECROISSANT";
    lcd.setCursor(0, 0);
    lcd.print("MODELE DE POMPE:");
    lcd.setCursor(0, 1);
    lcd.print(label);
  }    // end of window == 7

  if( window == 8 ) {
    if( pump_model == 0 ) label = "POMPE NON-PWM   ";
    else {
      if(ret_push_button == 2) ssr_rule = (ssr_rule +1)%4;
      if(ret_push_button == 3) ssr_rule = (ssr_rule -1)%4;
      if(ssr_rule == 0)      label = "POMPE NON-PWM   ";
      else if(ssr_rule == 1) label = "CHAUDIERE       ";
      else if(ssr_rule == 2) label = "PROTECTION GEL  ";
      else if(ssr_rule == 3) label = "PROT. EBULLITION";
    }
    lcd.setCursor(0, 0);
    lcd.print("USAGE COMMANDE :");
    lcd.setCursor(0, 1);
    lcd.print(label);
  }    // end of window == 8

  if( window == 9 ) {
    lcd.setCursor(0, 0);
    lcd.print("REINITIALISATION");
    lcd.setCursor(0, 1);
    lcd.print("POUR VALIDER : +");
    if(ret_push_button == 2) {
       lcd.setCursor(0, 0);
       lcd.print("REINITIALISATION");
       lcd.setCursor(0, 1);
       lcd.print("FAITE           ");
       deltaTon = deltaTon_d;    // default value to all parameters
       deltaToff = deltaToff_d;
       deltaTpwm = deltaTpwm_d;
       Tmaxi = Tmaxi_d;
       Tmini = Tmini_d;
       pump_model = pump_model_d;
       ssr_rule = ssr_rule_d;
       mforce = LOW;             // stop always ON capability
       window= 0; }
  }    // end of windows == 9

  if(  window == 10 ) {
    lcd.setCursor(0, 0);
    lcd.print("MARCHE FORCEE ? ");
    lcd.setCursor(0, 1);
    lcd.print("OUI= + / NON= - ");
    if(ret_push_button == 2) {
      mforce = HIGH;
      lcd.setCursor(0, 0);
      lcd.print("MARCHE FORCEE   ");
      lcd.setCursor(0, 1);
      lcd.print("CONFIRMEE       "); 
      window = 0; }
    if(ret_push_button == 3) {
      mforce = LOW;
      lcd.setCursor(0, 0);
      lcd.print("MARCHE FORCEE   ");
      lcd.setCursor(0, 1);
      lcd.print("ARRETEE         ");
      window = 0; }
  }    // end of windows == 10

// EEPROM backup
//____________________________________________________________________________________________

  if( eeprom = true ) {                // EEPROM backup only if needed
    EEPROM.update(0, deltaTon + 50);
    EEPROM.update(1, deltaToff + 50);
    EEPROM.update(2, deltaTpwm + 50);
    EEPROM.update(3, Tmaxi + 50);
    EEPROM.update(4, Tmini + 50);
    EEPROM.update(5, pump_model);
    EEPROM.update(6, ssr_rule);
  }

// for debugging
//____________________________________________________________________________________________

   Serial.print("ssr_rule: "); Serial.print(ssr_rule);
   Serial.print("  state: "); Serial.print(state);
   Serial.print("  ratio: "); Serial.print(ratio);
   Serial.print("  PWM: "); Serial.print(PWM);
   Serial.print("  Ts: "); Serial.print(Tsource);
   Serial.print("  Tc: "); Serial.print(Tcumulus);
   Serial.print("  Ts-Tc-deltaToff: "); Serial.print(Tsource-Tcumulus-deltaToff);
   Serial.println();
 
}    // end of loop

//============================================================================================
// list of functions
//============================================================================================

//
// next_window() : procedure to change the window display
//____________________________________________________________________________________________

void next_window() {
  window = (window+1) % windowsTotal;    // next window
  lcd.clear();
}      // end of next_window()

//
// push_button() : return value depending of the state of the 3 push-buttons
//____________________________________________________________________________________________

byte push_button() {
  if ( digitalRead(ENTRYpin) == LOW ) {
    count_before_timeout = 0;         // reset the timeout counter
    lcd.backlight();                  // switch on display
    lcd.clear();
    return 1;
  }
  if ( digitalRead(PLUSpin) == LOW ) {
    count_before_timeout = 0;
    lcd.backlight();
    eeprom = true;
    return 2;
  }
  if ( digitalRead(MINUSpin) == LOW ) {
    count_before_timeout = 0;
    lcd.backlight();
    eeprom = true;
    return 3;
  }
  return 0;
}     // end of push_button()

//
// GetTemperature() : reading measures of DS18B20
//____________________________________________________________________________________________

byte GetTemperature(float *temperature, byte reset_search) {
  byte data[9], addr[8];           // data[] : data read from scratchpad
                                   // addr[] : Address of detected 1-Wire device
  
// reset the list of adresses for the first sensor
// char argument = "true" or "false"
  if(reset_search) sondeDS.reset_search();
  sondeDS.search(addr);
  sondeDS.reset();
  sondeDS.select(addr);
  sondeDS.write(0x44, 1);          // getting the temperature takes as long as 1/2 second for each sensor
  delay(800);                      // required, see above comment
  sondeDS.reset();
  sondeDS.select(addr);
  sondeDS.write(0xBE);             // send request from scratchpad
 
// the scratchpad content makes 9 bytes:
  for (byte i = 0; i < 9; i++) { data[i] = sondeDS.read(); }

// miraculous formula to get the temperature in °C with a 12 bits resolution, accurancy of 0,06°K:
// it is possible to modify this formula to get °F
  *temperature = ((data[1] << 8) | data[0]) * 0.0625; 
}      // end of getTemperature()

//
// Alarm() red LED lighting management
//____________________________________________________________________________________________

void Alarm(byte c) {
  for( byte i=0; i<c; i++ ) {
    digitalWrite(ALARMpin, HIGH); delay(10);
    digitalWrite(ALARMpin, LOW); delay(200);
  }
}      // end of Alarm()
