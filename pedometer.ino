#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
 #include <Wire.h>

#define FILTER_N 12 
FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
float oldxx,oldyy,oldzz,olddd;
int StepNumber0=0,StepNumber1=0;
void setup() 
{ 
  Serial.begin(9600);
  Wire.begin();
   
  delay(5);
  sixDOF.init();   
  delay(5);
}
void loop() {
  float xx = Filter_x();
  float yy = Filter_y();
  float zz = Filter_z();
  float dot = (oldxx * xx)+(oldyy * yy)+(oldzz * zz);
  float olda = abs(sqrt(oldxx*oldxx+oldyy*oldyy+oldzz*oldzz));
  float newa = abs(sqrt(xx*xx+yy*yy+zz*zz));
  dot /=(olda*newa);
  StepNumber0=StepNumber1;
  if(abs(dot-olddd) >= 0.05)
  {bool isChange;
    if(!isChange){isChange = 1;
  StepNumber1 +=1;}
  else isChange = 0;
  }
  oldxx = xx;
  oldyy = yy;
  oldzz = zz;
  olddd = dot;
  delay(60);
  
  if(StepNumber0!=StepNumber1)
  {
  Serial.println(StepNumber1);
  }
  
  delay(100); 
        
 
}

int filter_buf[FILTER_N + 1];
float Filter_x() {
  int i;
  float filter_sum = 0.0;
  
  for(i = 0; i < FILTER_N; i++) {
    sixDOF.getRawValues(rawSixDof);
    filter_buf[i] = rawSixDof[0];
    
    filter_sum += filter_buf[i];
    delay(5);
  }
  return (float)(filter_sum / FILTER_N);
}
 
float Filter_y() {
  int i;
  float filter_sum = 0.0;
  
  for(i = 0; i < FILTER_N; i++) {
    sixDOF.getRawValues(rawSixDof);
    filter_buf[i] = rawSixDof[1];
    
    filter_sum += filter_buf[i];
    delay(5);
  }
  return (float)(filter_sum / FILTER_N);
}
 
float Filter_z() {
  int i;
  float filter_sum = 0.0;
  
  for(i = 0; i < FILTER_N; i++) {
    sixDOF.getRawValues(rawSixDof);
    filter_buf[i] = rawSixDof[2];
    
    filter_sum += filter_buf[i];
    delay(5);
  }
  return (float)(filter_sum / FILTER_N);
}
 
 
