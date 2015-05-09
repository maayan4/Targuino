

int Led=13;
 
int SENSOR=52;
 
int sensorValue;
 
void setup(){
  
  pinMode(Led,OUTPUT);
  pinMode (SENSOR,INPUT);
  Serial.begin(9600);

}
 
void loop(){
 
  sensorValue=digitalRead(SENSOR);
  if(sensorValue==HIGH){ 
    digitalWrite(Led, HIGH);
  }
 else{
  digitalWrite(Led, LOW);
 }
 Serial.println(sensorValue);
}
