#include <DFRobot_WT61PC.h>
#include <SoftwareSerial.h>
#include <math.h>

//Use software serial port RX：10，TX：11
SoftwareSerial mySerial(10, 11);
SoftwareSerial mySerial1(12, 13);
SoftwareSerial mySerial2(50, 51);
SoftwareSerial mySerial3(52, 53);
SoftwareSerial mySerial4(62, 63);
SoftwareSerial mySerial5(64, 65);

DFRobot_WT61PC sensor(&mySerial);
DFRobot_WT61PC sensor1(&mySerial1);
DFRobot_WT61PC sensor2(&mySerial2);
DFRobot_WT61PC sensor3(&mySerial3);
DFRobot_WT61PC sensor4(&mySerial4);
DFRobot_WT61PC sensor5(&mySerial5);

union eX {
  float Values[54];// 54 for 6 sensors
  byte Bytes[216]; // 216 for 6 sensors
} Obj;

void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  delay(500);

  while (!Serial) {
    ;
  }
  mySerial.begin(9600);
  mySerial1.begin(9600);
  mySerial2.begin(9600);
  mySerial3.begin(9600);
  mySerial4.begin(9600);
  mySerial5.begin(9600);

  sensor.modifyFrequency(FREQUENCY_20HZ);
  sensor1.modifyFrequency(FREQUENCY_20HZ);
  sensor2.modifyFrequency(FREQUENCY_20HZ);
  sensor3.modifyFrequency(FREQUENCY_20HZ);
  sensor4.modifyFrequency(FREQUENCY_20HZ);
  sensor5.modifyFrequency(FREQUENCY_20HZ);
}

void loop()
{
  mySerial.listen();
  if (sensor.available ()) {

    Obj.Values[0] = sensor.Acc.X;
    Obj.Values[1] = sensor.Acc.Y;
    Obj.Values[2] = sensor.Acc.Z;
    Obj.Values[3] = sensor.Gyro.X;
    Obj.Values[4] = sensor.Gyro.Y;
    Obj.Values[5] = sensor.Gyro.Z;
    Obj.Values[6] = sensor.Angle.X;
    Obj.Values[7] = sensor.Angle.Y;
    Obj.Values[8] = sensor.Angle.Z;
    
    Serial.print("Acc\t"); Serial.print(Obj.Values[0]); Serial.print("\t"); Serial.print(Obj.Values[1]); Serial.print("\t"); Serial.println(Obj.Values[2]);
    Serial.print("Gyro\t"); Serial.print(Obj.Values[3]); Serial.print("\t"); Serial.print(Obj.Values[4]); Serial.print("\t"); Serial.println(Obj.Values[5]);
    Serial.print("Angle\t"); Serial.print(Obj.Values[6]); Serial.print("\t"); Serial.print(Obj.Values[7]); Serial.print("\t"); Serial.println(Obj.Values[8]);
    Serial.println(" ");


  }

  mySerial1.listen();
  if (sensor1.available ()) {
    Obj.Values[9] = sensor1.Acc.X;
    Obj.Values[10] = sensor1.Acc.Y;
    Obj.Values[11] = sensor1.Acc.Z;
    Obj.Values[12] = sensor1.Gyro.X;
    Obj.Values[13] = sensor1.Gyro.Y;
    Obj.Values[14] = sensor1.Gyro.Z;
    Obj.Values[15] = sensor1.Angle.X;
    Obj.Values[16] = sensor1.Angle.Y;
    Obj.Values[17] = sensor1.Angle.Z;
    
    Serial.print("Acc1\t"); Serial.print(Obj.Values[9]); Serial.print("\t"); Serial.print(Obj.Values[10]); Serial.print("\t"); Serial.println(Obj.Values[11]);
    Serial.print("Gyro1\t"); Serial.print(Obj.Values[12]); Serial.print("\t"); Serial.print(Obj.Values[13]); Serial.print("\t"); Serial.println(Obj.Values[14]);
    Serial.print("Angle1\t"); Serial.print(Obj.Values[15]); Serial.print("\t"); Serial.print(Obj.Values[16]); Serial.print("\t"); Serial.println(Obj.Values[17]);
    Serial.println(" ");

  }

  mySerial2.listen();
  if (sensor2.available ()) {
    Obj.Values[18] = sensor2.Acc.X;
    Obj.Values[19] = sensor2.Acc.Y;
    Obj.Values[20] = sensor2.Acc.Z;
    Obj.Values[21] = sensor2.Gyro.X;
    Obj.Values[22] = sensor2.Gyro.Y;
    Obj.Values[23] = sensor2.Gyro.Z;
    Obj.Values[24] = sensor2.Angle.X;
    Obj.Values[25] = sensor2.Angle.Y;
    Obj.Values[26] = sensor2.Angle.Z;
    
    Serial.print("Acc2\t"); Serial.print(Obj.Values[18]); Serial.print("\t"); Serial.print(Obj.Values[19]); Serial.print("\t"); Serial.println(Obj.Values[20]);
    Serial.print("Gyro2\t"); Serial.print(Obj.Values[21]); Serial.print("\t"); Serial.print(Obj.Values[22]); Serial.print("\t"); Serial.println(Obj.Values[23]);
    Serial.print("Angle2\t"); Serial.print(Obj.Values[24]); Serial.print("\t"); Serial.print(Obj.Values[25]); Serial.print("\t"); Serial.println(Obj.Values[26]);
    Serial.println(" ");

  }

  mySerial3.listen();
  if (sensor3.available ()) {
    Obj.Values[27] = sensor3.Acc.X;
    Obj.Values[28] = sensor3.Acc.Y;
    Obj.Values[29] = sensor3.Acc.Z;
    Obj.Values[30] = sensor3.Gyro.X;
    Obj.Values[31] = sensor3.Gyro.Y;
    Obj.Values[32] = sensor3.Gyro.Z;
    Obj.Values[33] = sensor3.Angle.X;
    Obj.Values[34] = sensor3.Angle.Y;
    Obj.Values[35] = sensor3.Angle.Z;
    
    Serial.print("Acc3\t"); Serial.print(Obj.Values[27]); Serial.print("\t"); Serial.print(Obj.Values[28]); Serial.print("\t"); Serial.println(Obj.Values[29]);
    Serial.print("Gyro3\t"); Serial.print(Obj.Values[30]); Serial.print("\t"); Serial.print(Obj.Values[31]); Serial.print("\t"); Serial.println(Obj.Values[32]);
    Serial.print("Angle3\t"); Serial.print(Obj.Values[33]); Serial.print("\t"); Serial.print(Obj.Values[34]); Serial.print("\t"); Serial.println(Obj.Values[35]);
    Serial.println(" ");

  }

  mySerial4.listen();
  if (sensor4.available ()) {
    Obj.Values[36] = sensor4.Acc.X;
    Obj.Values[37] = sensor4.Acc.Y;
    Obj.Values[38] = sensor4.Acc.Z;
    Obj.Values[39] = sensor4.Gyro.X;
    Obj.Values[40] = sensor4.Gyro.Y;
    Obj.Values[41] = sensor4.Gyro.Z;
    Obj.Values[42] = sensor4.Angle.X;
    Obj.Values[43] = sensor4.Angle.Y;
    Obj.Values[44] = sensor4.Angle.Z;
    
    Serial.print("Acc4\t"); Serial.print(Obj.Values[36]); Serial.print("\t"); Serial.print(Obj.Values[37]); Serial.print("\t"); Serial.println(Obj.Values[38]);
    Serial.print("Gyro4\t"); Serial.print(Obj.Values[39]); Serial.print("\t"); Serial.print(Obj.Values[40]); Serial.print("\t"); Serial.println(Obj.Values[41]);
    Serial.print("Angle4\t"); Serial.print(Obj.Values[42]); Serial.print("\t"); Serial.print(Obj.Values[43]); Serial.print("\t"); Serial.println(Obj.Values[44]);
    Serial.println(" ");

  }

  mySerial5.listen();
  if (sensor5.available ()) {
    Obj.Values[45] = sensor5.Acc.X;
    Obj.Values[46] = sensor5.Acc.Y;
    Obj.Values[47] = sensor5.Acc.Z;
    Obj.Values[48] = sensor5.Gyro.X;
    Obj.Values[49] = sensor5.Gyro.Y;
    Obj.Values[50] = sensor5.Gyro.Z;
    Obj.Values[51] = sensor5.Angle.X;
    Obj.Values[52] = sensor5.Angle.Y;
    Obj.Values[53] = sensor5.Angle.Z;
    
    Serial.print("Acc5\t"); Serial.print(Obj.Values[45]); Serial.print("\t"); Serial.print(Obj.Values[46]); Serial.print("\t"); Serial.println(Obj.Values[47]);
    Serial.print("Gyro5\t"); Serial.print(Obj.Values[48]); Serial.print("\t"); Serial.print(Obj.Values[49]); Serial.print("\t"); Serial.println(Obj.Values[50]);
    Serial.print("Angle5\t"); Serial.print(Obj.Values[51]); Serial.print("\t"); Serial.print(Obj.Values[52]); Serial.print("\t"); Serial.println(Obj.Values[53]);
    Serial.println(" ");

  }

  Serial2.write(Obj.Bytes, sizeof(Obj.Values));
  Serial2.write(0xFF);
  delay(500);
}
