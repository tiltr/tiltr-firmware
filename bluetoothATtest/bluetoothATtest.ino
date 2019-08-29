HardwareSerial Serial5(PD2, PC12); // RX | TX


void setup()

{

Serial.begin(9600); Serial5.begin(57600); //Baud Rate for command Mode. 
Serial.println("Enter AT commands!");

}

void loop()

{

// Feed any data from bluetooth to Terminal.

if (Serial5.available())

Serial.write(Serial5.read());

// Feed all data from termial to bluetooth

if (Serial.available())

Serial5.write(Serial.read());

}
