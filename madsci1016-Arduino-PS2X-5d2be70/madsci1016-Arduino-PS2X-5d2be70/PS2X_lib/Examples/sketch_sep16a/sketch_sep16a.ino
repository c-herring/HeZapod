void setup()
{
	//DDRB = 0x01;
	pinMode(8, OUTPUT);
	pinMode(9, INPUT);
	Serial.begin(9600);	
	analogWrite(0, 20);
}

void loop()
{
	delay(500);	
	Serial.println("HEl0lo");
	//PORTB ^= 0x01;
	if (digitalRead(9) == HIGH)
	//if (PINB & 0x02)
	{
		analogWrite(0, 255);
		digitalWrite(8, HIGH);
		//PORTB |= 0x01;
	}
	else
	{
		analogWrite(0, 0);
		digitalWrite(8, LOW);
		//PORTB &= 0xFE;
	}
}
