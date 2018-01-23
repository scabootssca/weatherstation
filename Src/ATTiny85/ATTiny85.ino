#include <TinyWireS.h>

#define SLAVE_ADDR 0x2
#define LED_PIN 3
#define LED_PIN2 4
int requestCount = 0;
int i = 0;

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

void setup()
{
  TinyWireS.begin(SLAVE_ADDR);
  TinyWireS.onRequest(requestEvent);
  TinyWireS.onReceive(receiveEvent);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(LED_PIN2, OUTPUT);
  digitalWrite(LED_PIN2, HIGH);
}


void loop()
{
  TinyWireS_stop_check();
}

void requestEvent()
{
  if(requestCount%2==0)
  {
    digitalWrite(LED_PIN2, LOW);
  } else {
    digitalWrite(LED_PIN2, HIGH);
  }

  TinyWireS.send(requestCount);
  requestCount++;
}

void receiveEvent(uint8_t howMany) {
  if (howMany < 1)
  {
      // Sanity-check
      return;
  }
  if (howMany > TWI_RX_BUFFER_SIZE)
  {
      // Also insane number
      return;
  }

  i = TinyWireS.receive();

  if(i%2==0)
  {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}
