// implements smart spindle controller with PID and auto tune, and EEPROM save.
// To be used between Klipper and CNC spindle BLDC motor controller.
// Receives target speed from Klipper as G-code via UART, sends back debug data.
// Reads 3 Hall sensors on motor, calculates speed and adjusts PWM signal to motor driver to match target speed.
// PID Auto tune based on Ziegler-Nichols ultimate gain tuning method is triggered by push button.

// author: Valtteri Wikström

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define HALL_PIN0 PD2
#define HALL_PIN1 PD3
#define HALL_PIN2 PD4
#define BUTTON_PIN PD5
#define ESC_PIN PB1

#define PULSES_PER_REV 84
#define PWM_MIN 2000
#define PWM_MAX 4000

#define RPM_FILTER_ALPHA 0.2
#define RPM_CHECK_TIMEOUT 3000

volatile int pulseCount = 0;
uint8_t lastHallState = 0;

float Kp, Ki;
volatile float targetRPM = 4000;
float integral = 0;
float pwmOutput = 0;
float currentRPM = 0;
float filteredRPM = 0;

uint32_t lastRPMCalc = 0;
uint32_t lastPID = 0;
uint32_t lastButtonPress = 0;
uint32_t lastRPMNonZero = 0;

float EEMEM ee_Kp;
float EEMEM ee_Ki;

#define RX_BUF_SIZE 64
char rxBuffer[RX_BUF_SIZE];
volatile uint8_t rxHead = 0, rxTail = 0;

volatile uint32_t timer0_millis = 0;
ISR(TIMER0_COMPA_vect) { timer0_millis++; }
ISR(USART_RX_vect) {
    uint8_t next = (rxHead + 1) % RX_BUF_SIZE;
    if (next != rxTail) { rxBuffer[rxHead] = UDR0; rxHead = next; }
}

void setupTimers() {
    TCCR0A = (1 << WGM01);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
    TCCR0B = (1 << CS01) | (1 << CS00);
}

void uart_init(uint16_t ubrr) {
    UBRR0H = (ubrr >> 8); UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = c; }
void uart_print(const char* s) { for (; *s; s++) uart_putc(*s); }
void uart_println(const char* s) { uart_print(s); uart_putc('\n'); }
void uart_print_float(float x) { char buf[16]; dtostrf(x, 1, 2, buf); uart_print(buf); }

bool uart_available() { return rxHead != rxTail; }
char uart_read() {
    if (rxHead == rxTail) return 0;
    char c = rxBuffer[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return c;
}

uint8_t readHallState() {
    return (((PIND & (1 << HALL_PIN0)) ? 1 : 0) << 2) |
           (((PIND & (1 << HALL_PIN1)) ? 1 : 0) << 1) |
           ((PIND & (1 << HALL_PIN2)) ? 1 : 0);
}

uint32_t millis() { uint32_t m; cli(); m = timer0_millis; sei(); return m; }

void pwm_setup() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11);
    ICR1 = 39999;
}
void pwm_write(uint16_t val) { OCR1A = val; }
uint16_t pwm_map(float pwm) { return PWM_MIN + (uint16_t)(pwm * ((PWM_MAX - PWM_MIN) / 255.0)); }

void doPID() {
    float error = targetRPM - currentRPM;
    integral += error * 0.02;
    if (integral > 500) integral = 500;
    if (integral < -500) integral = -500;
    pwmOutput += Kp * error + Ki * integral;
    if (pwmOutput < 0) pwmOutput = 0;
    if (pwmOutput > 255) pwmOutput = 255;
    pwm_write(pwm_map(pwmOutput));
}

void relayAutoTune() {
    float relayHigh = 140, relayLow = 100, halfPeriods[6];
    uint8_t periodCount = 0;
    bool above = currentRPM > targetRPM;
    pwmOutput = relayLow; pwm_write(pwm_map(pwmOutput)); _delay_ms(1000);
    uint32_t lastCrossTime = millis();
    while (periodCount < 6) {
        if (above && currentRPM < targetRPM) {
            pwmOutput = relayHigh; pwm_write(pwm_map(pwmOutput));
            uint32_t now = millis(); halfPeriods[periodCount++] = (now - lastCrossTime) / 1000.0; lastCrossTime = now; above = false;
        } else if (!above && currentRPM > targetRPM) {
            pwmOutput = relayLow; pwm_write(pwm_map(pwmOutput));
            uint32_t now = millis(); halfPeriods[periodCount++] = (now - lastCrossTime) / 1000.0; lastCrossTime = now; above = true;
        }
    }
    float sum = 0; for (uint8_t i=0;i<6;i++) sum += halfPeriods[i];
    float Pu = (sum/6.0) * 2.0, deltaPWM = relayHigh-relayLow, deltaRPM = targetRPM*0.2;
    float Ku = (4.0/3.1415)*(deltaPWM/(deltaRPM>0?deltaRPM:1));
    Kp=0.6*Ku; Ki=1.2*Ku/Pu;
    eeprom_write_block((const void*)&Kp, &ee_Kp, sizeof(float));
    eeprom_write_block((const void*)&Ki, &ee_Ki, sizeof(float));
    uart_println("Relay auto-tune complete:");
    uart_print("Pu: "); uart_print_float(Pu); uart_println("");
    uart_print("Ku: "); uart_print_float(Ku); uart_println("");
    uart_print("Kp: "); uart_print_float(Kp); uart_println("");
    uart_print("Ki: "); uart_print_float(Ki); uart_println("");
}

void handleSerial() {
    static char line[32]; static uint8_t pos = 0;
    while (uart_available()) {
        char c = uart_read();
        if (c == '\n' || c == '\r') {
            line[pos] = 0; pos = 0;
            if (line[0] == 'S') {
                float newRPM = atof(&line[1]);
                if (newRPM >= 0 && newRPM <= 20000) {
                    targetRPM = newRPM; integral = 0;
                    uart_print("New targetRPM: "); uart_print_float(targetRPM); uart_println("");
                }
            } else if (line[0] == 'P') {
                char* kpStr = strtok(&line[1], ",");
                char* kiStr = strtok(NULL, ",");
                if (kpStr && kiStr) {
                    float newKp = atof(kpStr), newKi = atof(kiStr);
                    if (newKp >= 0 && newKi >= 0 && newKp < 100 && newKi < 100) {
                        Kp = newKp; Ki = newKi;
                        eeprom_write_block(&Kp, &ee_Kp, sizeof(float));
                        eeprom_write_block(&Ki, &ee_Ki, sizeof(float));
                        uart_print("Saved Kp: "); uart_print_float(Kp);
                        uart_print(" Ki: "); uart_print_float(Ki); uart_println("");
                    }
                }
            }
        } else if (pos < sizeof(line) - 1) line[pos++] = c;
    }
}

int main() {
    DDRD &= ~((1 << HALL_PIN0)|(1 << HALL_PIN1)|(1 << HALL_PIN2)|(1 << BUTTON_PIN));
    PORTD |= (1 << HALL_PIN0)|(1 << HALL_PIN1)|(1 << HALL_PIN2)|(1 << BUTTON_PIN);
    DDRB |= (1 << ESC_PIN);
    setupTimers(); pwm_setup(); uart_init(16); sei();
    eeprom_read_block(&Kp, &ee_Kp, sizeof(float));
    eeprom_read_block(&Ki, &ee_Ki, sizeof(float));
    if (isnan(Kp) || isnan(Ki) || Kp<0 || Ki<0 || Kp>100 || Ki>100) { Kp=0.02; Ki=0.1; }
    uart_println("Spindle PID with EEPROM save & auto-tune Ready.");
    lastHallState = readHallState();
    while (1) {
        uint32_t now = millis();
        handleSerial();
        if (!(PIND & (1 << BUTTON_PIN)) && (now - lastButtonPress > 500)) {
            lastButtonPress = now; uart_println("Relay auto-tune triggered..."); relayAutoTune();
        }
        uint8_t hallState = readHallState();
        if (hallState != lastHallState) { pulseCount++; lastHallState = hallState; }
        if (now - lastRPMCalc >= 100) {
            cli(); int count=pulseCount; pulseCount=0; sei();
            currentRPM=(count*600.0)/PULSES_PER_REV;
            filteredRPM = RPM_FILTER_ALPHA*currentRPM + (1-RPM_FILTER_ALPHA)*filteredRPM;
            currentRPM = filteredRPM;
            if (currentRPM > 10) lastRPMNonZero = now;
            lastRPMCalc=now;
        }
        if (now - lastRPMNonZero > RPM_CHECK_TIMEOUT && pwmOutput>0) {
            pwmOutput=0; pwm_write(pwm_map(pwmOutput)); uart_println("Safety Stop: RPM lost!");
        }
        if (now - lastPID >= 20) { doPID(); lastPID=now; }
    }
}
