// Encoder bench test: one motor encoder wired straight to the Uno.
// VCC->5V, GND->GND, A->D2, B->D3 (motor wires unconnected).
// Prints every 250 ms: "ENC ms count edgesA edgesB pinA pinB".
// count is quadrature (4 per encoder line); edgesA/B show whether each
// channel toggles at all. Reflash firmware/dunebroom_motor afterwards.

const uint8_t PIN_A = 2;
const uint8_t PIN_B = 3;

volatile long count = 0;
volatile unsigned long edgesA = 0, edgesB = 0;
volatile uint8_t lastState = 0;

// Gray-code step table indexed by (old << 2) | new.
const int8_t STEP[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void update() {
  uint8_t s = (digitalRead(PIN_A) << 1) | digitalRead(PIN_B);
  count += STEP[(lastState << 2) | s];
  lastState = s;
}

void isrA() { edgesA++; update(); }
void isrB() { edgesB++; update(); }

void setup() {
  Serial.begin(115200);
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  lastState = (digitalRead(PIN_A) << 1) | digitalRead(PIN_B);
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), isrB, CHANGE);
  Serial.println("READY ENCODER_TEST");
}

void loop() {
  static unsigned long next = 0;
  if (millis() >= next) {
    next = millis() + 250;
    noInterrupts();
    long c = count;
    unsigned long ea = edgesA, eb = edgesB;
    interrupts();
    Serial.print("ENC ");
    Serial.print(millis());
    Serial.print(' ');
    Serial.print(c);
    Serial.print(' ');
    Serial.print(ea);
    Serial.print(' ');
    Serial.print(eb);
    Serial.print(' ');
    Serial.print(digitalRead(PIN_A));
    Serial.print(' ');
    Serial.println(digitalRead(PIN_B));
  }
}
