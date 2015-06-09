
boolean nextLine = false;
void setup() 
{
  Serial.begin(57600);
  Serial2.begin(57600);
  Serial2.println("begin");
}
 
void loop() 
{
  while(Serial.available()){ // there is data that is being sent from the serial monitor
    Serial2.print(char(Serial.read())); // send the data to xBee
  }
  while(Serial2.available()){ // there is data being sent from the xBee
    Serial.print(char(Serial2.read())); // display the data on serial monitor
    nextLine = true;
  }
  if (nextLine){ // applies carriage return only when data is available
    Serial.println();
    nextLine = false;
  }
  delay(1000);
}
