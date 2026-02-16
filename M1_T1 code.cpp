// ===========================================
// Brightness Control System
// 3 Buttons (PCI) + Timer1 Interrupt
// ===========================================

#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_PIN 5   // Move LED to D5 (PWM)
#define BUTTON1 8   // Increase brightness
#define BUTTON2 9   // Decrease brightness
#define BUTTON3 10  // Turn OFF

volatile bool buttonEvent = false;
volatile bool timerEvent = false;

volatile uint8_t lastPortBState = 0;

int brightness = 0;

// =================================
// Setup Functions
// =================================
void setupPins()
{
  pinMode(LED_PIN, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
}

void setupPCI()
{
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);

  lastPortBState = PINB;
}

void setupTimer()
{
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  OCR1A = 15624;

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);

  TIMSK1 |= (1 << OCIE1A);

  sei();
}

// =================================
// Interrupt Service Routines
// =================================

ISR(PCINT0_vect)
{
  buttonEvent = true;
}

ISR(TIMER1_COMPA_vect)
{
  timerEvent = true;
}

// =================================
// Button Logic
// =================================
void processButtons()
{
  uint8_t currentState = PINB;
  uint8_t changed = currentState ^ lastPortBState;

  // Increase brightness
  if (changed & (1 << PB0))
  {
    if (!(currentState & (1 << PB0)))
    {
      brightness += 85;
      if (brightness > 255)
        brightness = 255;

      Serial.println("Brightness Increased");
    }
  }

  // Decrease brightness
  if (changed & (1 << PB1))
  {
    if (!(currentState & (1 << PB1)))
    {
      brightness -= 85;
      if (brightness < 0)
        brightness = 0;

      Serial.println("Brightness Decreased");
    }
  }

  // Turn OFF
  if (changed & (1 << PB2))
  {
    if (!(currentState & (1 << PB2)))
    {
      brightness = 0;
      Serial.println("LED Turned OFF");
    }
  }

  analogWrite(LED_PIN, brightness);

  lastPortBState = currentState;
}

// =================================
// Setup & Loop
// =================================
void setup()
{
  Serial.begin(9600);

  setupPins();
  setupPCI();
  setupTimer();

  Serial.println("Brightness Control System Started");
}

void loop()
{
  if (buttonEvent)
  {
    buttonEvent = false;
    processButtons();
  }

  if (timerEvent)
  {
    timerEvent = false;

    Serial.print("System Status Update -> Brightness Level: ");
    Serial.println(brightness);
  }
}