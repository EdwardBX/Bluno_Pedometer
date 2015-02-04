#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

#define FILTER_N 12

FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
float old_accx, old_accy, old_accz, old_dot;
int StepNum = 0, StepNumber = 0;

void setup() {
Serial.begin(9600);
Wire.begin();

delay(5);
sixDOF.init();
delay(5);
}

void loop() {
float accx = Filter(0);//读取滤波后的加速度x分量
float accy = Filter(1);
float accz = Filter(2);
float dot = (old_accx * accx)+(old_accy * accy)+(old_accz * accz);
float old_acc = abs(sqrt(old_accx*old_accx+old_accy*old_accy+old_accz*old_accz));
float new_acc = abs(sqrt(accx*accx+accy*accy+accz*accz));
dot /= (old_acc * new_acc);//计算加速度变化程度

StepNum = StepNumber;
if(abs(dot - old_dot) >= 0.05) {
//变化程度超过阈值，则判定步数增加
bool isChange;
if(!isChange) {
isChange = 1;
StepNumber += 1;
}
else isChange = 0;
}
old_accx = accx;
old_accy = accy;
old_accz = accz;
old_dot = dot;
delay(60);

if(StepNum != StepNumber) {
//步数增加则打印出来
Serial.println(StepNumber);
}

delay(100);
}

int filter_buf[FILTER_N + 1];

float Filter(int s) {
//平均滤波法
float filter_sum = 0.0;

for(int i = 0; i < FILTER_N; i++) {
sixDOF.getRawValues(rawSixDof);
filter_buf[i] = rawSixDof[s];
filter_sum += filter_buf[i];
delay(5);
}
return (float)(filter_sum / FILTER_N);
}
