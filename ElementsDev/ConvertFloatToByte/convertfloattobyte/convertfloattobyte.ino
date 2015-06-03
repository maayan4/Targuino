
unsigned int lTrack = 5179;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  
  
  byte msgToSend[7] = {0, 0, 0, 0, 0, 0, 0};
  
  msgToSend[5] = (byte) (lTrack & 0xFF00) >> 8;
  msgToSend[6] = (byte) (lTrack & 0x00FF); 
  
  byte lTrackTemp[2] = {msgToSend[5], msgToSend[6]};

 unsigned int res = (unsigned int) lTrackTemp;

 unsigned int res = (unsigned int) lTrackTemp;

 Serial.println(res);
}

void loop() {


}
