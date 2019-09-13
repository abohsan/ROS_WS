
int led = 12;
long savedTime = 0;
int diffTime = 0 ;

void blinkLed(int duration) {
  if (duration > 0) {
    diffTime = (millis() - savedTime);
    if ( diffTime > duration) {
      digitalWrite(led, !digitalRead(led));
      savedTime = millis();
    }
  }else{
     digitalWrite(led, HIGH);
  }
}
