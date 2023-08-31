#include <SPI.h>
#include <avr/pgmspace.h>
#include <cstring>
#include <cstdlib>

// Automatically increment to equal ms since last set to 0
elapsedMillis sinceLastRead;
elapsedMillis sinceLastMessage;
elapsedMicros sinceLastLoop;
elapsedMillis sinceValveOn;
elapsedMillis sinceValveOff;

byte initComplete=0;
byte Motion = 0;
byte xH;
byte xL;
byte yH;
byte yL;
int xydat[2];
int xy2dat[2];
double dP;
double dR;
double dY;

const int ncs = 0;
const int ncs2 = 1;
const int pVelPin = 3;
const int rVelPin = 4;
const int yVelPin = 5;
// const int valve1Pin = 14;
// const int valve2Pin = 15;
const int lickPin = 16;
// const int lick2Pin = 17;
const int valvePin = 22;

const double px1 = 0.8151;
const double rx1 = 0.1849;
const double yx1 = -0.5959;
const double py1 = 0.2414;
const double ry1 = -0.2414;
const double yy1 = -0.7779;
const double px2 = -0.1849;
const double rx2 = -0.8151;
const double yx2 = 0.5959;
const double py2 = -0.2414;
const double ry2 = 0.2414;
const double yy2 = -0.7779;

char* usbBuffer = NULL;
size_t usbBufferLength;

byte valveState = 0;
byte holdOpen = 0;
unsigned long valveDur = 0;
const int timeoutDur = 100; // ms to hold between drops, to prevent nonlinear effects w/ multiple drops when calibrating
int dropsToDispense = 0;

// long lickCount1 = 0;
// long lickCount2 = 0;
long lickCount = 0;

// unsigned int lastLick1 = 0;
// unsigned int lastLick2 = 0;
unsigned int lastLick = 0;
unsigned long dt;

// To maintain mean since last read
double loopsSinceRead = 0.0;

//const double px1 = 1;
//const double rx1 = 0;
//const double yx1 = 0;
//const double py1 = 0;
//const double ry1 = 0;
//const double yy1 = -1.557;
//const double px2 = 0;
//const double rx2 = -1;
//const double yx2 = 1.1918;
//const double py2 = 0;
//const double ry2 = 0;
//const double yy2 = 0;

// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

// uncomment to only read velocity every X ms
// #define READ_INTERVAL_MS 10
#define ADNS_SETTINGS SPISettings(2000000, MSBFIRST, SPI_MODE3)
// #define DEBUG

extern const unsigned short firmware_length;
extern prog_uchar firmware_data[];

void setup() {

// set up serial and print so that matlab knows it is working
  while(!Serial) delay(10);
  Serial.print("Baud rate: ");
  Serial.println(Serial.baud());

  analogWriteFrequency(pVelPin,11500);
  analogWriteFrequency(rVelPin,11500);
  analogWriteFrequency(yVelPin,11500);
  analogWriteResolution(12);
  pinMode(ncs, OUTPUT);
  pinMode(ncs2, OUTPUT);
  
  pinMode(valvePin,OUTPUT);
  pinMode(lickPin,INPUT);

  adns_reset_cs();
  SPI.begin();
  
  delay(1000);
  performStartup();
  delay(10);
  performStartup2();
  delay(10);
  adns_write_reg(REG_Configuration_I, 0x10);
  //adns_write_reg(REG_Configuration_I, 0x29); // maximum resolution
  //adns_write_reg(REG_Configuration_I, 0x09); // default resolution
  //adns_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  delay(10);
  adns2_write_reg(REG_Configuration_I, 0x10);
  //adns2_write_reg(REG_Configuration_I, 0x09); // default resolution
  //adns2_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  delay(1500);  
  // dispRegisters();
  // delay(1500);
  // dispRegisters2();
  // delay(1500);

  // Initialize usbBuffer to hold 10 characters - should realistically never have to expand
  setBufferSize(10);

  initComplete=9;

}

void setBufferSize(size_t n) {
  if (usbBuffer == NULL) {
    usbBuffer = (char*) malloc((n + 1) * sizeof(char));
  } else {
    usbBuffer = (char*) realloc(usbBuffer, (n + 1) * sizeof(char));
  }

  if (usbBuffer == NULL) {
    Serial.println("Unable to allocate memory - aborting!");
    while (true) {
      delay(100);
    }
  } else {
    usbBufferLength = n;
  }
}

void adns_reset_cs() {
  // Reset chip select lines to pre-transaction state
  digitalWrite(ncs, HIGH);
  digitalWrite(ncs2, HIGH);
}

void adns_com_begin(){
  SPI.beginTransaction(ADNS_SETTINGS);
  digitalWrite(ncs, LOW);
}

void adns2_com_begin(){
  SPI.beginTransaction(ADNS_SETTINGS);
  digitalWrite(ncs2, LOW);
}

void adns_com_end(){
  digitalWrite(ncs, HIGH);
  SPI.endTransaction();
}

void adns2_com_end(){
  digitalWrite(ncs2, HIGH);
  SPI.endTransaction();
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

byte adns2_read_reg(byte reg_addr){
  adns2_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns2_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns2_write_reg(byte reg_addr, byte data){
  adns2_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns2_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
//  Serial.println("Uploading firmware to chip 1...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
}

void adns2_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  //Serial.println("Uploading firmware to chip 2...");
  // set the configuration_IV register in 3k firmware mode
  adns2_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns2_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns2_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns2_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns2_com_end();
}


void performStartup(void){
  // adns_com_end(); // ensure that the serial port is reset - this is really a digital pin
  adns_com_begin(); // ensure that the serial port is reset - digital pin
  adns_com_end(); // ensure that the serial port is reset - digital pin
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(10);

  // Serial.println("Optical Chip 1 Initialized");
}

void performStartup2(void){
  // adns2_com_end(); // ensure that the serial port is reset
  adns2_com_begin(); // ensure that the serial port is reset
  adns2_com_end(); // ensure that the serial port is reset
  adns2_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns2_read_reg(REG_Motion);
  adns2_read_reg(REG_Delta_X_L);
  adns2_read_reg(REG_Delta_X_H);
  adns2_read_reg(REG_Delta_Y_L);
  adns2_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns2_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0_2 = adns2_read_reg(REG_LASER_CTRL0);
  adns2_write_reg(REG_LASER_CTRL0, laser_ctrl0_2 & 0xf0 );
  
  delay(10);

  //Serial.println("Optical Chip 2 Initialized");
}


// void dispRegisters(void){
//   int oreg[7] = {
//     0x00,0x3F,0x2A,0x0F  };
//   const char* oregname[] = {
//     "Product_ID","Inverse_Product_ID","SROM_Version","CPI"  };
//   byte regres;

//   adns_com_begin();

//   int rctr=0;
//   for(rctr=0; rctr<4; rctr++){
//     SPI.transfer(oreg[rctr]);
//     delay(1);
//     // Serial.println("---");
//     // Serial.println(oregname[rctr]);
//     // Serial.println(oreg[rctr],HEX);
//     regres = SPI.transfer(0);
//     // // Serial.println(regres,BIN);  
//     // Serial.println(regres,HEX);  
//     delay(1);
//   }
//   adns_com_end();
// }

// void dispRegisters2(void){
//   int oreg[7] = {
//     0x00,0x3F,0x2A,0x0F  };
//   const char* oregname[] = {
//     "Product_ID2","Inverse_Product_ID2","SROM_Version2","CPI2"  };
//   byte regres;

//   adns2_com_begin();

//   int rctr=0;
//   for(rctr=0; rctr<4; rctr++){
//     SPI.transfer(oreg[rctr]);
//     delay(1);
//     // Serial.println("---");
//     // Serial.println(oregname[rctr]);
//     // Serial.println(oreg[rctr],HEX);
//     regres = SPI.transfer(0);
//     // Serial.println(regres,BIN);  
//     // Serial.println(regres,HEX);  
//     delay(1);
//   }
//   adns2_com_end();
// }

void readXY(int *xy){
  //digitalWrite(ncs,LOW);
  
  Motion = (adns_read_reg(REG_Motion) & (1 << 7)) != 0;
  xL = adns_read_reg(REG_Delta_X_L);
  xH = adns_read_reg(REG_Delta_X_H);
  yL = adns_read_reg(REG_Delta_Y_L);
  yH = adns_read_reg(REG_Delta_Y_H);
  xy[0] = (xH << 8) + xL;
  xy[1] = (yH << 8) + yL;

  if(xy[0] & 0x8000){
    xy[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy[1] & 0x8000){
    xy[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  
  //digitalWrite(ncs,HIGH);     
}

void readXY2(int *xy){
  //digitalWrite(ncs2,LOW);
  
  Motion = (adns2_read_reg(REG_Motion) & (1 << 7)) != 0;
  xL = adns2_read_reg(REG_Delta_X_L);
  xH = adns2_read_reg(REG_Delta_X_H);
  yL = adns2_read_reg(REG_Delta_Y_L);
  yH = adns2_read_reg(REG_Delta_Y_H);
  xy[0] = (xH << 8) + xL;
  xy[1] = (yH << 8) + yL;

  if(xy[0] & 0x8000){
    xy[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy[1] & 0x8000){
    xy[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  
  //digitalWrite(ncs2,HIGH);     
}


// Valve control
void startRelease() {
  digitalWrite(valvePin, HIGH);
  sinceValveOn = 0;
  valveState = 1;
}

void stopRelease() {
  digitalWrite(valvePin, LOW);
  sinceValveOff = 0;
  valveState = 0;
}


void interpretCommand(char* message) {
  size_t len = strlen(message);
  if (len > 0) {
    char char1 = message[0];
    if (char1 == 'O') {
      // Hold open
      holdOpen = 1;
      startRelease();
#ifdef DEBUG
      Serial.println("Holding valve open");
#endif
    } else if (char1 == 'C') {
      // Stop holding
      holdOpen = 0;
      stopRelease();
#ifdef DEBUG
      Serial.println("Closing valve");
#endif
    } else {
      bool succeeded = false;
      // Get # of ms to open valve for each drop
      char* tok = strtok(message, "xX");
      if (tok != NULL) {
        int parsedDur = atoi(tok);
        if (parsedDur > 0) {
          // Get # of drops to release
          tok = strtok(NULL, "xX");
          if (tok == NULL) {
            valveDur = (unsigned long) parsedDur;
            dropsToDispense = 1;
            succeeded = true;
          } else {
            int parsedN = atoi(tok);
            if (parsedN > 0) {
              valveDur = (unsigned long) parsedDur;
              dropsToDispense = parsedN;
              succeeded = true;
            }
          }
        }
      }

#ifdef DEBUG
      if (succeeded) {
        Serial.print("Dispensing ");
        Serial.print(dropsToDispense);
        Serial.print(" drop(s) for ");
        Serial.print(valveDur);
        Serial.println(" ms each");
      } else {
        Serial.println("Command not understood");
      }
#endif
    }
  }

  // Print each piece of data to the serial port
  Serial.print("dp:");
  Serial.print(dP, 3);
  Serial.print(";dr:");
  Serial.print(dR, 3);
  Serial.print(";dy:");
  Serial.print(dY, 3);
  Serial.print(";licks:");
  Serial.print(lickCount);
  Serial.print(";valveState:");
  Serial.print(valveState);
  Serial.print(";dta:");
  Serial.print(dt);
  Serial.print(";dtmsg:");
  Serial.println(sinceLastMessage);
  Serial.send_now();
  sinceLastMessage = 0;
  loopsSinceRead = 0;
  lickCount = 0;
}

void loop() {
  static double thisdP, thisdR, thisdY;

  dt = sinceLastLoop;
  sinceLastLoop = 0;

#ifdef READ_INTERVAL_MS
  if (sinceLastRead >= READ_INTERVAL_MS) {
    sinceLastRead = 0;
#endif
    readXY(&xydat[0]);
    readXY2(&xy2dat[0]);
    thisdP = px1*xydat[0] + py1*xydat[1] + px2*xy2dat[0] + py2*xy2dat[1];
    thisdR = rx1*xydat[0] + ry1*xydat[1] + rx2*xy2dat[0] + ry2*xy2dat[1];
    thisdY = yx1*xydat[0] + yy1*xydat[1] + yx2*xy2dat[0] + yy2*xy2dat[1];
    analogWrite(pVelPin, thisdP+2048);
    analogWrite(rVelPin, thisdR+2048);
    analogWrite(yVelPin, thisdY+2048);

    // update averages
    loopsSinceRead++;
    dP *= (loopsSinceRead - 1) / loopsSinceRead;
    dP += thisdP / loopsSinceRead;
    dR *= (loopsSinceRead - 1) / loopsSinceRead;
    dR += thisdR / loopsSinceRead;
    dY *= (loopsSinceRead - 1) / loopsSinceRead;
    dY += thisdY / loopsSinceRead;
#ifdef READ_INTERVAL_MS
  }
#endif

  // update licks
  int lickRead = digitalRead(lickPin);
  if (lastLick == 0 && lickRead == 1) {
    lickCount = lickCount + 1;
  }
  lastLick = lickRead;

  unsigned int nWritten = 0;
  while (Serial.available() > 0) {
    // read next char if available
    char inByte = Serial.read();
    if (inByte == '\n') {
      // the new-line character ('\n') indicates a complete message
      // so close the string and interpret the message
      // should always maintain room for a string ending character
      usbBuffer[nWritten] = '\0';
      interpretCommand(usbBuffer);
    } else {
      // append character to message buffer
      if (nWritten == usbBufferLength) {
        setBufferSize(usbBufferLength * 2);
      }
      usbBuffer[nWritten] = inByte;
      nWritten++;
    }
  }

  // update valve
  // If valve is off and valve trigger turns on, start release
  if (valveState == 0 && dropsToDispense > 0 && sinceValveOff >= timeoutDur) {
    dropsToDispense--;
    startRelease();
#ifdef DEBUG
    Serial.println("dropping");
#endif
  } else if (!holdOpen && valveState == 1 && sinceValveOn >= valveDur) {
    stopRelease();
  }

  delayMicroseconds(10);
}

