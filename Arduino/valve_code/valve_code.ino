const byte numChars = 12;
char receivedChars[numChars];
int chr_size;
char ex_char = 'a';
String p_string;
boolean newData = false;


void setup() {
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

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
      Serial.println(p_1);
      if (p_1 == "0"){
        digitalWrite(11, LOW);
        digitalWrite(12, HIGH);
      }
      else if (p_1 == "1") {
        digitalWrite(11, HIGH);
        digitalWrite(12, LOW);
      }
      else if (p_1 == "2"){
        digitalWrite(11, HIGH);
        digitalWrite(12, HIGH);
      }
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
