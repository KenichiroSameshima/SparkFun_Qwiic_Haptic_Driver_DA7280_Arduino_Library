/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <kenichiro.sameshima@alpsalpine.com> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return Poul-Henning Kamp
 * ----------------------------------------------------------------------------
 */

#ifndef _SPARKFUN_HAPTIC_DRIVER_DA7280_
#define _SPARKFUN_HAPTIC_DRIVER_DA7280_

#include <Wire.h>
#include <Arduino.h>

#define DEF_ADDR 0x4A
#define CHIP_REV 0xBA
#define ENABLE 0x01
#define UNLOCKED 0x01
#define DISABLE 0x00
#define LOCKED 0x00
#define LRA_TYPE 0x00
#define ERM_TYPE 0x01
#define RAMP 0x01
#define STEP 0x00

struct hapticSettings {

  uint8_t motorType; 
  float nomVolt;
  float absVolt; 
  float currMax;
  float impedance; 
  float lraFreq;  

};

typedef enum {

  HAPTIC_SUCCESS,
  E_SEQ_CONTINUE = 0x01,
  E_UVLO = 0x02,
  HAPTIC_HW_ERROR,
  E_SEQ_DONE = 0x04,
  HAPTIC_INCORR_PARAM,
  HAPTIC_UNKNOWN_ERROR,
  E_OVERTEMP_CRIT = 0x08,
  E_SEQ_FAULT = 0x10,
  E_WARNING = 0x20,
  E_ACTUATOR_FAULT = 0x40,
  E_OC_FAULT = 0x80

} event_t;

typedef enum
{
  NO_WARN_DIAG = 0x00,
  E_OVERTEMP_WARN = 0x08,
  E_MEM_TYPE = 0x10,
  E_LIM_DRIVE_ACC = 0x40,
  E_LIM_DRIVE = 0x80
} warn_diag_status_t;

typedef enum {

  NO_SEQ_DIAG = 0x00,
  E_PWM_FAULT = 0x20,
  E_MEM_FAULT = 0x40,
  E_SEQ_ID_FAULT = 0x80

} seq_diag_status_t;

typedef enum {

  STATUS_NOM = 0x00,
  STA_SEQ_CONTINUE = 0x01,
  STA_UVLO_VBAT_OK = 0x02,
  STA_PAT_DONE = 0x04,
  STA_OVERTEMP_CRIT = 0x08,
  STA_PAT_FAULT = 0x10,
  STA_WARNING = 0x20,
  STA_ACTUATOR = 0x40,
  STA_OC = 0x80

} status_t;

enum OPERATION_MODES {

   INACTIVE  = 0x00, 
   DRO_MODE, 
   PWM_MODE,
   RTWM_MODE, 
   ETWM_MODE

};

enum SNPMEM_ARRAY_POS {

  BEGIN_SNP_MEM = 0x00,
  NUM_SNIPPETS = 0x00,
  NUM_SEQUENCES = 0x01,
  ENDPOINTERS = 0x02,
  SNP_ENDPOINTERS = 0x02,  
  //SEQ_ENDPOINTERS = 0x11, //SNP_ENDPOINTERS + 14 = 0x10
  TOTAL_MEM_REGISTERS = 0x64,

};

enum SNPMEM_REGS {

  NUM_SNIPPETS_REG = 0x84,
  NUM_SEQUENCES_REG = 0x85,
  SNP_ENDPOINTERS_REGS = 0x88, // Up to 15 endpointers can be addressed
  //SEQ_ENDPOINTERS_REGS = 0x89,
  END_OF_MEM = 0xE7 // 0x84 + 99 = 0xE7
};

enum REGISTERS {

  CHIP_REV_REG = 0x00, //whoami?
  IRQ_EVENT1 = 0x03, 
  IRQ_EVENT_WARN_DIAG, 
  IRQ_EVENT_SEQ_DIAG, 
  IRQ_STATUS1, 
  IRQ_MASK1, 
  CIF_I2C1, 
  FRQ_LRA_PER_H = 0x0A, 
  FRQ_LRA_PER_L, 
  ACTUATOR1, 
  ACTUATOR2, 
  ACTUATOR3, 
  CALIB_V2I_H, 
  CALIB_V2I_L = 0x10, 
  CALIB_IMP_H, 
  CALIB_IMP_L, 
  TOP_CFG1,
  TOP_CFG2,
  TOP_CFG3,
  TOP_CFG4,
  TOP_INT_CFG1,
  TOP_INT_CFG6_H = 0x1C,
  TOP_INT_CFG6_L,
  TOP_INT_CFG7_H,
  TOP_INT_CFG7_L,
  TOP_INT_CFG8 = 0x20,
  TOP_CTL1 = 0x22,
  TOP_CTL2,
  SEQ_CTL1,
  SWG_C1,
  SWG_C2,
  SWG_C3,
  SEQ_CTL2,
  GPI_0_CTL,
  GPI_1_CTL,
  GPI_2_CTL,
  MEM_CTL1,
  MEM_CTL2,
  ADC_DATA_H1,
  ADC_DATA_L1,
  POLARITY = 0x43,
  LRA_AVR_H,
  LRA_AVR_L,
  FRQ_LRA_PER_ACT_H,
  FRQ_LRA_PER_ACT_L,
  FRQ_PHASE_H,
  FRQ_PHASE_L,
  FRQ_CTL = 0x4C,
  TRIM3 = 0x5F,
  TRIM4,
  TRIM6 = 0x62,
  TOP_CFG5 = 0x6E,
  IRQ_EVENT_ACTUATOR_FAULT = 0x81,
  IRQ_STATUS2,
  IRQ_MASK2,
  SNP_MEM_X

};


class Haptic_Driver
{  
  public:
    
    // Public Variables
    uint8_t snpMemCopy[100] {}; 
    uint8_t lastPosWritten = 0; 
    
    //Function declarations
    Haptic_Driver(uint8_t address = DEF_ADDR); // I2C Constructor

    bool begin(TwoWire &wirePort = Wire); // begin function

    bool setActuatorType(uint8_t);
    bool setOperationMode(uint8_t mode = DRO_MODE ); 
    uint8_t getOperationMode(); 

    bool defaultMotor();
    bool setMotor(hapticSettings userSettings);
    hapticSettings getSettings();
    bool setActuatorABSVolt(float);
    float getActuatorABSVolt();
    bool setActuatorNOMVolt(float);
    float getActuatorNOMVolt();
    bool setActuatorIMAX(float);
    float getActuatorIMAX();
    bool setActuatorImpedance(float);
    float getActuatorImpedance();
    uint16_t readImpAdjus();
    bool setActuatorLRAfreq(float);
    float getActuatorLRAfreq();
    bool setLoopFilterCapTrim(float);
    uint8_t getLoopFilterCapTrim();
    bool setLoopFilterResTrim(float);
    uint8_t getLoopFilterResTrim();
    bool enableDoubleRange(bool);
    bool enableLoopFiltLowBandwidth(bool);
    bool enableEmbeddedOperation(uint8_t);
    bool enableAcceleration(bool);
    bool enableRapidStop(bool);
    bool enableAmpPid(bool);
    bool enableFreqTrack(bool);
    bool setBemfFaultLimit(bool);
    bool enableV2iFactorFreeze(bool);
    bool calibrateImpedanceDistance(bool);
    bool setVibratePower(uint8_t);
    uint8_t getVibratePower();
    bool setFullBrakeThreshold(uint8_t);
    float getFullBrakeThreshold();
    bool setBemf(uint8_t val);
    float getBemf();
    void clearIrqEvent(uint8_t);
    event_t getIrqEvent();
    uint8_t getEventWarningDiag();
    seq_diag_status_t getEventSeqDiag();
    status_t getIrqStatus();
    bool setIrqMask(uint8_t);
    uint8_t getIrqMask();

    hapticSettings sparkSettings;



  private:
    
    // Private Variables
    uint8_t _address;
    TwoWire *_i2cPort;

    // This generic function handles I2C write commands for modifying individual
    // bits in an eight bit register. Paramaters include the register's address, a mask 
    // for bits that are ignored, the bits to write, and the bits' starting
    // position.
    bool _writeRegister(uint8_t, uint8_t, uint8_t, uint8_t);

    // This generic function reads an eight bit register. It takes the register's
    // address as its' parameter. 
    uint8_t _readRegister(uint8_t);
};

#endif  // _SPARKFUN_HAPTIC_DRIVER_DA7280_
