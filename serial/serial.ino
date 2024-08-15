void setup(){
    Serial.begin(115200);
}

void loop(){
    Serial.printf("%f %f %f %f %f\n",micros(),micros(),micros(),micros(),micros());
    delay(10);
}