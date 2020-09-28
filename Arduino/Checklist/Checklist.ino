int degree;
String inputCmd ="";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  readCommands();
  delay(100);
    
  switch(inputCmd[0]){
    case 'W':
    {
      goForward();
      break;
    }
    case 'A':
    {
      turnLeft();
      break;
    }
    case 'D':
    {
      turnRight();
      break;
    }
    case 'L':
    {
      rotateLeft(degree);
      break;
    }
    case 'R':
    {
      rotateRight(degree);
      break;
    }
    case 'C':
    {
      calibration();
      break;
    }
    case 'V':
    {
      Serial.print("SENSORS");
      break;
    }
  }
  delay(1000);
}

void readCommands(){
  char newChar;
  inputCmd="";
  while (Serial.available()){
    newChar = Serial.read();
    inputCmd.concat(newChar);
    if (newChar == '\n'){
      break;
    }
  }
}

void goForward(){
  Serial.println("arduino,forward");
}
void turnLeft(){
  Serial.println("arduino,left");
}
void turnRight(){
  Serial.println("arduino,right");
}
void rotateLeft(int degree){
  Serial.print(degree);
  Serial.println("arduino,Rotate Left");
}
void rotateRight(int degree){
  Serial.print(degree);
  Serial.println("arduino,Rotate right");
}
void calibration(){
  Serial.println("arduino,Calibrate");
}
