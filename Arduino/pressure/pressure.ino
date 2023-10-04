#include "DFRobot_GP8403.h"
DFRobot_GP8403 dac(&Wire,0x5C);

void setup() {
  Serial.begin(9600);
  while(dac.begin()!=0){
    Serial.println("init error");
    delay(100);
   }
//  Serial.println("init succeed");
  dac.begin();
  dac.setDACOutRange(dac.eOutputRange10V);//Set the output range as 0-10V
  dac.setDACOutVoltage(8160,0);

}

String V_string;
int v_str;
float temp_dac_in;
float V;
int dac_in;
boolean newData = false;
const byte numChars = 6;
char receivedChars[numChars];
  
void loop(){

  if (Serial.available())
  { 
    recvWithStartEndMarkers();
    V_string = showNewData();
    if (V_string != "")
    {
      V = V_string.toFloat();
      //Serial.println(V);      
      
          
      temp_dac_in = V*1000;        
      dac_in = (int)temp_dac_in;
            
      dac.setDACOutVoltage(dac_in, 0);
    }
  }
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}


String showNewData() {
    String s;
    if (newData == true) {
        newData = false;

        int ch_size;
        char temp = 'a';
        
        ch_size = sizeof(receivedChars)/sizeof(temp);
        s = convertToString(receivedChars, ch_size);        
    }
    return s;
}


String convertToString(char* a, int size)
{
    int i;
    String s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}
