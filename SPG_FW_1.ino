HardwareSerial &pc  = Serial;
HardwareSerial &bt  = Serial1;
float valores[6]={10.15,20.23,30.31,40.44,50.46,90.56};       // Pines de lectura para sensores de flexión.

#include<Wire.h>                                   //Librería comunicación I2C.
#define degconvert 57.2957786                      //57 grados en un radian.
#define adcvoltaje 0.004887                        //Conversión ADC 10bits Vref=3.3v. //0.003225 0.00488759 si son 5V
    
const int MPU_direccion=0x68;                      // Direccion I2C del MPU-6050 Poner AD0 a GND.
const int SensorFlexion[5]={A15,A14,A13,A12,A11};       // Pines de lectura para sensores de flexión.
float ValorResistenciaFlexion[5];                 // Guarda el valor de la resistencia en Ohms de cada sensor.
float ValorAnguloFlexion[5];                      // Guarda el valor en grados de la flexion de cada sensor.
float ValorAnalogicoFlexion[5];                      // Guarda el valor binario de la lectura analógica.
float Vcc=4.162;                                     // Voltaje de alimentación.
double R2=10000;                                   // Valor de la resistencia PULL-UP de cada sensor de flexión.

int GENERAL[8];                                 // Datos a enviar utilizados en el guante, simplificados
//double MACROS[14];                               // Todos los datos en un arreglo, 5 sensores + MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;               // Valores leídos del sensor MPU-6050
double roll;                                       // Valor del ángulo de giro sobre eje X en grados
double pitch;                                      // Valor del ángulo de giro sobre el eje Y en grados 
double gyroXrate;                                  // Valor de la aceleración angular en grados/segundo del eje X
double gyroYrate;                                  // Valor de la aceleración angular en grados/segundo del eje Y
double gyroZrate;                                  // Valor de la aceleración angular en grados/segundo del eje Z

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_direccion);
  Wire.write(0x6B);  // PWR_MGMT_1 registro
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Serial.begin(9600);
  pc.begin(9600);
  //bt.begin(115200);
  bt.begin(9600);
}

void loop() {

  for(int i=0;i<5;i++){                                                                                 //Leer sensores
    ValorAnalogicoFlexion[i] = (analogRead(SensorFlexion[i]))*adcvoltaje;                                 //Conversión ADC a Voltaje
    ValorResistenciaFlexion[i]= (ValorAnalogicoFlexion[i]*R2)/(Vcc-ValorAnalogicoFlexion[i]);           //Cálculo resistencia en ohms        
  }
   
  //Calcula el valor de flexion en grados por cada sensor. 0 y 4 son los sensores de flexion cortos. 1,2,3 son los sensores largos.
  ValorAnguloFlexion[0]=((ValorResistenciaFlexion[0]-R2)*0.0036)+36;        // 180 grados de rango / 30000 ohms = 0.006
  ValorAnguloFlexion[1]=((ValorResistenciaFlexion[1]-R2)*0.0036)+36;       // 180 grados de rango / 40000 ohms = 0.0036
  ValorAnguloFlexion[2]=((ValorResistenciaFlexion[2]-R2)*0.0036)+36;       // 180 grados de rango / 40000 ohms
  ValorAnguloFlexion[3]=((ValorResistenciaFlexion[3]-R2)*0.0036)+36;       // 180 grados de rango / 40000 ohms
  ValorAnguloFlexion[4]=((ValorResistenciaFlexion[4]-R2)*0.0036)+36;        // 180 grados de rango / 30000 ohms

  Wire.beginTransmission(MPU_direccion);
  Wire.write(0x3B);                                                   // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_direccion,14,true);                            // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();                                     // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();                                     // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();                                     // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();                                     // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();                                     // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();                                     // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();                                     // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  roll = atan2(AcY, AcZ)*degconvert; 
  pitch = atan2(-AcX, AcZ)*degconvert;
  gyroXrate = GyX/131.0; //deg/second
  gyroYrate = GyY/131.0; //deg/second
  gyroZrate = GyZ/131.0; //deg/second

  //DEFINICIÓN DEL FORMATO DEL ARREGLO A ENVIAR VÍA BT
  //GENERAL[0]=0;
  GENERAL[0]=ValorAnguloFlexion[0];//ValorAnguloFlexion[0];
  GENERAL[1]=ValorAnguloFlexion[1];
  GENERAL[2]=ValorAnguloFlexion[2];
  GENERAL[3]=ValorAnguloFlexion[3];
  GENERAL[4]=ValorAnguloFlexion[4];
  GENERAL[5]=roll;
  //GENERAL[7]=0;

  while (bt.available() > 0) {
   if (bt.read() == '\n') {
      int recibido = bt.parseInt(); 
      //pc.println(recibido); 
      //pc.println("X"+String(GENERAL[1])+","+String(GENERAL[2])+","+String(GENERAL[3])+","+String(GENERAL[4])+","+String(GENERAL[5])+","+String(GENERAL[6])+"X");  
      for(int d=0;d<6;d++){                                                                                 //Leer sensores
      bt.println(GENERAL[d]);
      pc.println(GENERAL[d]);
      }
      
      //bt.println("X"+String(GENERAL[1])+","+String(GENERAL[2])+","+String(GENERAL[3])+","+String(GENERAL[4])+","+String(GENERAL[5])+","+String(GENERAL[6])+"X");        
      //pc.println("X"+String(valores[1])+","+String(valores[2])+","+String(valores[3])+","+String(valores[4])+","+String(valores[5])+","+String(valores[6])+"X");  
      //bt.println("X"+String(valores[1])+","+String(valores[2])+","+String(valores[3])+","+String(valores[4])+","+String(valores[5])+","+String(valores[6])+"X");            
    }   
  }
}


