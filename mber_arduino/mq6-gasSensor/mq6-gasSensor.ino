/*==============Demo for MQ-6 Gas Sensor Module =======
www.circuits4you.com
======================================================*/
 
#include <stdint.h>
#include <math.h>
#include <Wire.h>
#include "I2C_16.h"

#define MQ6_VOBJ  0x00
#define MQ6_TAMB 0x01
#define MQ6_CONFIG 0x02

/*========Hardware Related Macros=====================*/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (20)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (10)    //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
 
/*============Software Related Macros ===============*/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation
 
/*=============Application Related Macros============*/
#define         GAS_LPG                      (0)
#define         GAS_CH4                      (1)
 
/*===============Globals=============================*/
float           LPGCurve[3]  =  {3,   0,  -0.4};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg1000, lg1), point2: (lg10000, lg0.4) 
float           CH4Curve[3]  =  {3.3, 0,  -0.38};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg2000, lg1), point2: (lg5000,  lg0.7) 
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


uint8_t sensor2 = 0x41;

// Configuration Settings
#define TMP006_CFG_RESET    0x8000
#define TMP006_CFG_MODEON   0x7000
#define TMP006_CFG_1SAMPLE  0x0000
#define TMP006_CFG_2SAMPLE  0x0200
#define TMP006_CFG_4SAMPLE  0x0400
#define TMP006_CFG_8SAMPLE  0x0600
#define TMP006_CFG_16SAMPLE 0x0800
#define TMP006_CFG_DRDYEN   0x0100
#define TMP006_CFG_DRDY     0x0080

void setup()
{
  Wire.begin();
  Serial.begin(9600);                               //UART setup, baudrate = 9600bps
  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
  write16(sensor2, MQ6_CONFIG, TMP006_CFG_8SAMPLE | TMP006_CFG_MODEON | TMP006_CFG_DRDYEN); //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
}
 
void loop()
{
   float      raw_data;
   uint32_t mq6_data;
   uint16_t to_i2c;
   Serial.print("LPG:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
   Serial.print( "ppm\n" );
   Serial.print("        ");   
   Serial.print("CH4::"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CH4) );
   raw_data = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CH4);
   mq6_data = (uint32_t)raw_data;
   to_i2c = (uint16_t)mq6_data;
   write16(sensor2, MQ6_TAMB, to_i2c);
   Serial.print(" - ");
   Serial.print(to_i2c);
   Serial.print( "ppm" );
   Serial.print("\n");
   delay(200);
}
 
/*=============== MQResistanceCalculation ==================
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
============================================================*/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/*===================== MQCalibration ======================
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
============================================================*/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}
/*==================== MQRead ===============================
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
============================================================*/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*=======================  MQGetGasPercentage ==================
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
===============================================================*/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CH4 ) {
      return MQGetPercentage(rs_ro_ratio,CH4Curve);
  }    
 
  return 0;
}
 
/*========================  MQGetPercentage ===================
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
===============================================================*/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, (((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}