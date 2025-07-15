// implements smart spindle controller with PID and auto tune, and EEPROM save.
// To be used between Klipper and CNC spindle BLDC motor controller.
// Receives target speed from Klipper as G-code via UART, sends back debug data.
// Reads 3 Hall sensors on motor, calculates speed and adjusts PWM signal to motor driver to match target speed.
// PID Auto tune based on Ziegler-Nichols ultimate gain tuning method is triggered by push button.

// author: Valtteri Wikström

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define HALL_PIN0            PD2
#define HALL_PIN1            PD3
#define HALL_PIN2            PD4
#define BUTTON_PIN           PD5
#define ESC_PIN              PB1

#define PULSES_PER_REV       84

#define PWM_MIN              2000
#define PWM_MAX              4000
#define PWM_SCALE            255.0f

#define RPM_FILTER_ALPHA     0.2f

#define RPM_CHECK_TIMEOUT_MS 3000

#define AUTO_TUNE_INIT_DELAY_MS 1000
#define AUTO_TUNE_PERIODS       6
#define AUTO_TUNE_RELAY_HIGH    140
#define AUTO_TUNE_RELAY_LOW     100

#define DEFAULT_KP           0.02f
#define DEFAULT_KI           0.1f
#define K_MAX                100.0f
#define INTEGRAL_MAX         500.0f

#define TARGET_RPM_MIN       0.0f
#define TARGET_RPM_MAX       20000.0f

// Clamp macro
#define constrain(x, lo, hi) (((x)<(lo))?(lo):(((x)>(hi))?(hi):(x)))

// ——————— EEPROM variables ————————
float EEMEM ee_Kp;
float EEMEM ee_Ki;

// —————— Runtime state ———————
volatile uint32_t timer0_millis = 0;
volatile int      pulseCount     = 0;
volatile uint8_t  lastHallState  = 0;
volatile uint32_t lastRPMNonZero = 0;

// Auto-tune state machine
volatile bool     autoTuneActive = false;
enum {
    AT_IDLE,
    AT_WAIT_INIT,
    AT_TUNING
} atState = AT_IDLE;
uint32_t  atLastCrossTime = 0;
uint8_t   atPeriodCount   = 0;
float     atHalfPeriods[AUTO_TUNE_PERIODS];
bool      atAbove        = false;

// Serial Rx buffer
#define RX_BUF_SIZE 64
char rxBuffer[RX_BUF_SIZE];
volatile uint8_t rxHead = 0, rxTail = 0;

// Target and measured speed
volatile float targetRPM = 4000.0f;
float currentRPM = 0.0f, filteredRPM = 0.0f;

// ————— PID Struct —————
typedef struct {
    float Kp, Ki;
    float integral;
    uint32_t lastTime;
} PID_t;

PID_t pid;

// ——————— Helper: print from Flash ——————
void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print_P(const __FlashStringHelper* flash) {
    const char* p = (const char*)flash;
    char c;
    while ((c = pgm_read_byte(p++))) uart_putc(c);
}

void uart_println_P(const __FlashStringHelper* flash) {
    uart_print_P(flash);
    uart_putc('\n');
}

void uart_print(const char* s) {
    while (*s) uart_putc(*s++);
}

void uart_println(const char* s) {
    uart_print(s);
    uart_putc('\n');
}

void uart_print_float(float x) {
    char buf[16];
    dtostrf(x, 1, 2, buf);
    uart_print(buf);
}

// ————— Timer0: 1 ms tick ————
ISR(TIMER0_COMPA_vect) {
    timer0_millis++;
}

// ——————— Hall sensor PCINT ————————
ISR(PCINT2_vect) {
    uint8_t state = (((PIND & (1 << HALL_PIN0)) ? 1 : 0) << 2)
                  | (((PIND & (1 << HALL_PIN1)) ? 1 : 0) << 1)
                  |  ((PIND & (1 << HALL_PIN2)) ? 1 : 0);
    if (state != lastHallState) {
        pulseCount++;
        lastHallState = state;
        lastRPMNonZero = timer0_millis;
    }
}

// —————— UART Rx ————
ISR(USART_RX_vect) {
    uint8_t next = (rxHead + 1) % RX_BUF_SIZE;
    if (next != rxTail) {
        rxBuffer[rxHead] = UDR0;
        rxHead = next;
    }
}

// ——————— Millis helper (atomic) ————————
uint32_t millis() {
    uint32_t m;
    cli();
    m = timer0_millis;
    sei();
    return m;
}

// —————— PWM setup ————————
void pwm_setup() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM12)  | (1 << WGM13) | (1 << CS11);
    ICR1   = 39999;        // 50 Hz base
}

void pwm_write(uint16_t val) {
    OCR1A = val;
}

uint16_t pwm_map(float pwm) {
    return PWM_MIN
         + (uint16_t)(pwm * ((PWM_MAX - PWM_MIN) / PWM_SCALE));
}

// ——————— PID update (called every ~20 ms) —————
void doPID() {
    uint32_t now = millis();
    float dt = (now - pid.lastTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    pid.lastTime = now;

    float error = targetRPM - currentRPM;
    pid.integral = constrain(pid.integral + error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);

    float output = pid.Kp * error + pid.Ki * pid.integral;
    output = constrain(output, 0.0f, PWM_SCALE);
    pwm_write(pwm_map(output));
}

// ————— Serial command parser ——————
bool uart_available() {
    return rxHead != rxTail;
}

char uart_read() {
    if (rxHead == rxTail) return 0;
    char c = rxBuffer[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return c;
}

void handleSerial() {
    static char line[32];
    static uint8_t pos = 0;

    while (uart_available()) {
        char c = uart_read();
        if (c == '\n' || c == '\r') {
            line[pos] = '\0';
            pos = 0;
            if (line[0] == 'S') {
                float v = atof(&line[1]);
                if (v >= TARGET_RPM_MIN && v <= TARGET_RPM_MAX) {
                    targetRPM = v;
                    pid.integral = 0;
                    uart_print_P(PSTR("New targetRPM: "));
                    uart_print_float(targetRPM);
                    uart_putc('\n');
                }
            }
            else if (line[0] == 'P') {
                char* kpStr = strtok(&line[1], ",");
                char* kiStr = strtok(NULL, ",");
                if (kpStr && kiStr) {
                    float nk = atof(kpStr), ni = atof(kiStr);
                    if (nk >= 0 && ni >= 0 && nk < K_MAX && ni < K_MAX) {
                        pid.Kp = nk; pid.Ki = ni;
                        eeprom_update_block(&pid.Kp, &ee_Kp, sizeof(pid.Kp));
                        eeprom_update_block(&pid.Ki, &ee_Ki, sizeof(pid.Ki));
                        uart_print_P(PSTR("Saved Kp: "));
                        uart_print_float(pid.Kp);
                        uart_print_P(PSTR(" Ki: "));
                        uart_print_float(pid.Ki);
                        uart_putc('\n');
                    }
                }
            }
        }
        else if (pos < sizeof(line)-1) {
            line[pos++] = c;
        }
    }
}

int main() {
    // GPIO setup
    DDRD  &= ~((1<<HALL_PIN0)|(1<<HALL_PIN1)|(1<<HALL_PIN2)|(1<<BUTTON_PIN));
    PORTD |=  (1<<HALL_PIN0)|(1<<HALL_PIN1)|(1<<HALL_PIN2)|(1<<BUTTON_PIN);
    DDRB  |=  (1<<ESC_PIN);

    // Timer0 (millis)
    TCCR0A = (1<<WGM01);
    OCR0A  = 249;               // 1 kHz @ F_CPU=16 MHz
    TIMSK0 = (1<<OCIE0A);
    TCCR0B = (1<<CS01)|(1<<CS00);

    pwm_setup();

    // UART @115200 UBRR=16 @16 MHz
    uint16_t ubrr = 16;
    UBRR0H = ubrr>>8; UBRR0L = ubrr;
    UCSR0B = (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

    // Hall PCINT on PD2,PD3,PD4
    PCICR  |= (1<<PCIE2);
    PCMSK2 |= (1<<PCINT18)|(1<<PCINT19)|(1<<PCINT20);

    sei();

    // Load PID gains from EEPROM
    eeprom_read_block(&pid.Kp, &ee_Kp, sizeof(pid.Kp));
    eeprom_read_block(&pid.Ki, &ee_Ki, sizeof(pid.Ki));
    if (isnan(pid.Kp) || isnan(pid.Ki)
        || pid.Kp < 0 || pid.Ki < 0
        || pid.Kp > K_MAX || pid.Ki > K_MAX) {
        pid.Kp = DEFAULT_KP;
        pid.Ki = DEFAULT_KI;
    }
    pid.integral = 0;
    pid.lastTime = millis();

    uart_println_P(PSTR("Spindle PID with EEPROM save & auto-tune Ready."));

    lastHallState = (PIND>>HALL_PIN0)&0x07;

    uint32_t lastRPMCalc = millis();

    while (1) {
        uint32_t now = millis();

        handleSerial();

        // Button --> start auto-tune
        if (!(PIND & (1<<BUTTON_PIN))
            && (now - atLastCrossTime > 500)
            && !autoTuneActive)
        {
            atLastCrossTime = now;
            atState         = AT_WAIT_INIT;
            autoTuneActive  = true;
            uart_println_P(PSTR("Relay auto-tune triggered..."));
        }

        // Auto-tune state machine non-blocking
        if (autoTuneActive) {
            switch (atState) {
                case AT_WAIT_INIT:
                    if (now - atLastCrossTime >= AUTO_TUNE_INIT_DELAY_MS) {
                        // kick off at low
                        pwm_write(pwm_map(AUTO_TUNE_RELAY_LOW));
                        atAbove        = (currentRPM > targetRPM);
                        atLastCrossTime= now;
                        atPeriodCount  = 0;
                        atState        = AT_TUNING;
                    }
                    break;

                case AT_TUNING:
                    if (atPeriodCount < AUTO_TUNE_PERIODS) {
                        if (atAbove && currentRPM < targetRPM) {
                            pwm_write(pwm_map(AUTO_TUNE_RELAY_HIGH));
                            atHalfPeriods[atPeriodCount++] = (now - atLastCrossTime)/1000.0f;
                            atLastCrossTime = now;
                            atAbove = false;
                        }
                        else if (!atAbove && currentRPM > targetRPM) {
                            pwm_write(pwm_map(AUTO_TUNE_RELAY_LOW));
                            atHalfPeriods[atPeriodCount++] = (now - atLastCrossTime)/1000.0f;
                            atLastCrossTime = now;
                            atAbove = true;
                        }
                    }
                    else {
                        // compute Z-N parameters
                        float sum=0;
                        for(uint8_t i=0;i<AUTO_TUNE_PERIODS;i++) sum+=atHalfPeriods[i];
                        float Pu = (sum/AUTO_TUNE_PERIODS)*2.0f;
                        float deltaPWM = AUTO_TUNE_RELAY_HIGH - AUTO_TUNE_RELAY_LOW;
                        float deltaRPM = targetRPM*0.2f;
                        float Ku = (4.0f/3.14159265f)*(deltaPWM/(deltaRPM>0?deltaRPM:1.0f));
                        pid.Kp = 0.6f*Ku;
                        pid.Ki = 1.2f*Ku/Pu;
                        eeprom_update_block(&pid.Kp, &ee_Kp, sizeof(pid.Kp));
                        eeprom_update_block(&pid.Ki, &ee_Ki, sizeof(pid.Ki));

                        uart_println_P(PSTR("Relay auto-tune complete:"));
                        uart_print_P(PSTR("Pu: ")); uart_print_float(Pu); uart_putc('\n');
                        uart_print_P(PSTR("Ku: ")); uart_print_float(Ku); uart_putc('\n');
                        uart_print_P(PSTR("Kp: ")); uart_print_float(pid.Kp); uart_putc('\n');
                        uart_print_P(PSTR("Ki: ")); uart_print_float(pid.Ki); uart_putc('\n');

                        autoTuneActive = false;
                    }
                    break;
            }
        }

        // RPM calculation every 100 ms
        if (now - lastRPMCalc >= 100) {
            cli();
            int cnt = pulseCount;
            pulseCount = 0;
            sei();
            float rpm = (cnt * 600.0f) / PULSES_PER_REV;
            filteredRPM = RPM_FILTER_ALPHA * rpm + (1 - RPM_FILTER_ALPHA) * filteredRPM;
            currentRPM  = filteredRPM;
            if (currentRPM > 10.0f) lastRPMNonZero = now;
            lastRPMCalc = now;
        }

        //Safety stop if RPM lost 
        if ((now - lastRPMNonZero > RPM_CHECK_TIMEOUT_MS)) {
            pwm_write(pwm_map(0));
        }

        // PID every ~20 ms.
        if (now - pid.lastTime >= 20) {
            doPID();
        }
    }
}
