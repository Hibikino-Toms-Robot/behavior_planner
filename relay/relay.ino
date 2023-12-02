#define SIG_PIN2 2
#define SIG_PIN4 4
#define SIG_PIN6 6
char receivedChar;

void Module_ON(void){
  digitalWrite(SIG_PIN2, HIGH);
  digitalWrite(SIG_PIN4, HIGH);
  digitalWrite(SIG_PIN6, HIGH);
}

void All_Module_OFF(void){
  digitalWrite(SIG_PIN2, LOW);
  digitalWrite(SIG_PIN4, LOW);
  digitalWrite(SIG_PIN6, LOW);
}
void setup(){
  pinMode(13, OUTPUT);
  pinMode(SIG_PIN2, OUTPUT);
  pinMode(SIG_PIN4, OUTPUT);
  pinMode(SIG_PIN6, OUTPUT);
  digitalWrite(SIG_PIN2, LOW);
  digitalWrite(SIG_PIN4, LOW);
  digitalWrite(SIG_PIN6, LOW);
  Serial.begin(115200);
}
void loop(){
  if(Serial.available() > 0)
  {
    receivedChar = Serial.read();

    // 受信したコマンドに応じてリレーを制御
    if (receivedChar == '1') {
      digitalWrite(13, HIGH); 
      Module_ON();
      Serial.println("setup ok!");
    } else if (receivedChar == '0') {
      digitalWrite(13, LOW);
      All_Module_OFF() ;
      Serial.println("Communication Error"); 
    }
  }
}
