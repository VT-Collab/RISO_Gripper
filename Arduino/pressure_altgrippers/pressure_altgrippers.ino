#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;
Adafruit_MCP4725 dac3;

float P;
float V;
float temp_dac_input;
float dac_in_1;
float dac_in_2;
float dac_in_3;
float I;


const byte numChars = 12;
char receivedChars[numChars];
int chr_size;
char ex_char = 'a';
String p_string;
boolean newData = false;

void setup() {
    Serial.begin(19200);
    Serial.setTimeout(1000);
    dac1.begin(0x60);
    dac2.begin(0x62);
    dac3.begin(0x63);

    dac1.setVoltage(0.0, false);    
    dac2.setVoltage(0.0, false);      
    dac3.setVoltage(0.0, false);
    
    
}


void loop() {
  if (Serial.available())
  { 
    recvWithStartEndMarkers();
    p_string = showNewData();
    if (p_string != "")
    {
      Serial.println(p_string);
      String p_1 = getValue(p_string,';',0);
      //String p_2 = getValue(p_string,';',1);
      Serial.println(p_1);
      //String p_3 = getValue(p_string,';',2);
     
      dac_in_1 = getDAC_Input(p_1);
      //dac_in_2 = getDAC_Input(p_2);
      //dac_in_3 = getDAC_Input(p_3);
          
      dac1.setVoltage(dac_in_1, false);     
      //dac2.setVoltage(dac_in_2, false);      
      //dac3.setVoltage(dac_in_3, false);   
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
                if (ndx > numChars) {
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



int getDAC_Input(String p)
{
      P = p.toFloat();      
      I = (P+7.5)*8.0/15.0;            
      V = (I-4.0)/3.2;              
      temp_dac_input = V*4095.0/5.0;        
      return (int)temp_dac_input;
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
