/**
 * USB Fan HWMON - ATmega3244 Firmware (4 Fan Version)
 * 
 * Hardware Mapping:
 *  - Fan 1: PWM=PB5 (OC1A), Tacho=PD2 (INT1, Arduino Pin 2)
 *  - Fan 2: PWM=PB6 (OC1B), Tacho=PD1 (INT2, Arduino Pin 1/TX)
 *  - Fan 3: PWM=PC6 (OC3A), Tacho=PD0 (INT3, Arduino Pin 0/RX)
 *  - Fan 4: PWM=PD7 (OC4D), Tacho=PE6 (INT6, Arduino Pin 7)
 * 
 * PWM Frequency: 25kHz
 *  - Timer1 (Fan 1-3): Fast PWM, ICR1 TOP
 *  - Timer4 (Fan 4): Fast PWM (Mode 7), OC4D TOP, OC4D Output
 */

#include <Arduino.h>
#include <HID-Project.h>

// --- Configuration ---
#define PWM_FREQ 25000L
#define CPU_FREQ 16000000L
#define PWM1_TOP (CPU_FREQ / PWM_FREQ) // 640
#define PWM0_TOP 79 // 16MHz / (8 * (79+1)) = 25kHz

// --- Globals ---
volatile uint16_t fan_rpm[4] = {0, 0, 0, 0}; 
volatile uint32_t tacho_period_us[4] = {0, 0, 0, 0}; // Time between last two pulses (us)
volatile uint32_t tacho_last_edge_us[4] = {0, 0, 0, 0}; // Timestamp of last pulse

uint8_t rawhid_rx[64];
uint8_t rawhid_tx[64];

uint8_t active_fan_index = 0; // 0..3

// --- Protocol Constants ---
#define CMD_FIRMWARE_ID     0x01
#define CMD_FAN_SELECT      0x10
#define CMD_FAN_FIXED_PWM   0x13
#define CMD_FAN_READ_RPM    0x16
#define CMD_FAN_MAX_RPM     0x17

#define OP_WRITE_ONE        0x06
#define OP_READ_TWO         0x09
#define OP_WRITE_THREE      0x0A

// --- Interrupt Service Routines ---

void handle_tacho(uint8_t index) {
    uint32_t now = micros();
    tacho_period_us[index] = now - tacho_last_edge_us[index];
    tacho_last_edge_us[index] = now;
}

ISR(INT1_vect) { handle_tacho(0); } // Fan 1 (PD1 / Pin 2)
ISR(INT2_vect) { handle_tacho(1); } // Fan 2 (PD2 / Pin 0 RX)
ISR(INT3_vect) { handle_tacho(2); } // Fan 3 (PD3 / Pin 1 TX)
ISR(INT6_vect) { handle_tacho(3); } // Fan 4 (PE6/7)

// --- Initialization ---

void setup() {
    // 1. Configure Tacho Pins (Inputs with Pullup)
    // INT1(PD1), INT2(PD2), INT3(PD3)
    DDRD &= ~((1 << PD1) | (1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD1) | (1 << PD2) | (1 << PD3);
    
    // INT6(PE6)
    DDRE &= ~(1 << PE6);
    PORTE |= (1 << PE6);

    // 2. Configure PWM Pins (Outputs)
    // Timer1: PB5 (Fan 1), PB6 (Fan 2)
    DDRB |= (1 << PB5) | (1 << PB6);
    // Timer3: PC6 (Fan 3 - Pin 5)
    DDRC |= (1 << PC6);
    // Timer4: PD7 (Fan 4 - Pin 6)
    // Note: Pin 3 (PD0) is released back to Timer0 (System)
    DDRD |= (1 << PD7);

    // 3. Configure Timer1 for 25kHz PWM (Fan 1-2)
    // Mode 14: Fast PWM, TOP=ICR1, Scaling=1
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); 
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    ICR1 = PWM1_TOP; // 640

    // 4. Configure Timer3 for 25kHz PWM (Fan 3) on Pin 5 (OC3A)
    // Mode 14: Fast PWM, TOP=ICR3, Scaling=1
    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS30);
    ICR3 = PWM1_TOP; // 640

    // 5. Configure Timer4 for 25kHz PWM (Fan 4) on Pin 6 (OC4D)
    // Clock: PLL (64MHz). Target 25kHz.
    // Divisor needed: 64M / 25k = 2560.
    // Prescaler 4: 16MHz count frequency.
    // TOP = 16M / 25k = 640.
    // Max TOP is 10-bit (1023), so 640 fits.
    
    // TCCR4B: Prescaler 4 (CS42=0, CS41=1, CS40=1) -> 0x3 ? No set bits CS43..0.
    // Atmega32u4 datasheet: CS43:0. 0001=/1, 0010=/2, 0011=/4.
    TCCR4B = (1 << CS41) | (1 << CS40); // Prescaler 4
    
    // TCCR4C: Enable PWM4D (COM4D1=1, COM4D0=0).
    // Note: Shadow bits might be needed for high resolution, but 25kHz is standard.
    // PWM4D is in TCCR4C.
    TCCR4C = (1 << COM4D1) | (1 << PWM4D);
    
    // TCCR4D: WGM41:0 = 00 (Fast PWM)
    TCCR4D = 0; 
    
    // Set TOP in OCR4C (10-bit)
    uint16_t top = 640;
    TC4H = top >> 8;
    OCR4C = top & 0xFF;
    
    // Initial Duty
    // OCR4D = 128; // ~20% of 640 is 128
    uint16_t pwm4_duty = 128;
    TC4H = pwm4_duty >> 8;
    OCR4D = pwm4_duty & 0xFF;

    // Reset Timer0 to defaults? Arduino does it. We just don't touch it.

    // Initial Duty
    uint16_t start_pwm1 = 20UL * PWM1_TOP / 100;
    OCR1A = start_pwm1;
    OCR1B = start_pwm1;
    OCR3A = start_pwm1;
    // OCR4D set above

    // 6. Attach Interrupts
    EICRA = (1 << ISC31) | (1 << ISC21) | (1 << ISC11);
    EICRB = (1 << ISC61);
    EIMSK = (1 << INT1) | (1 << INT2) | (1 << INT3) | (1 << INT6);

    // 7. USB HID
    RawHID.begin(rawhid_rx, sizeof(rawhid_rx));

}

// --- Helper Functions ---

void set_pwm(uint8_t fan, uint8_t duty_byte) {
    if (fan < 3) {
        // Timer 1 (0-640)
        uint16_t val = (uint32_t)duty_byte * PWM1_TOP / 255;
        if (fan == 0) OCR1A = val;
        else if (fan == 1) OCR1B = val;
        else if (fan == 2) OCR3A = val; 
    } else if (fan == 3) {
        // Timer 4 (0-640)
        // duty_byte 0..255 -> 0..640
        uint16_t val = (uint32_t)duty_byte * 640 / 255;
        if (val > 640) val = 640;
        
        // Write 10-bit Timer4 Register (TC4H High Byte FIRST)
        TC4H = val >> 8;
        OCR4D = val & 0xFF;
    }
}

// --- Main Loop ---

void loop() {
    static unsigned long last_rpm_calc_time = 0;
    
    // Periodic RPM Calculation (Every 200ms maybe? or 1000ms is fine)
    // Period measurement is instant, so we can update faster if desired.
    // Keeping 1000ms for display stability, or faster? Let's check user intent.
    // User code said: if ((millis() - last_rpm_calc_time) >= 1020)
    // We stick to ~1s update.
    
    if ((millis() - last_rpm_calc_time) >= 1000) {
        last_rpm_calc_time = millis();
        
        for (uint8_t i = 0; i < 4; i++) {
            uint32_t period;
            uint32_t last_edge;
            
            // Atomic Read
            uint8_t sreg = SREG;
            cli();
            period = tacho_period_us[i];
            last_edge = tacho_last_edge_us[i];
            SREG = sreg;
            
            // Check for Stall (No pulse in last 1.5 seconds)
            // Using 1.5s margin to be safe.
            if ((micros() - last_edge) > 1500000UL) {
                fan_rpm[i] = 0;
            } else {
                if (period > 0) {
                    // RPM = 30,000,000 / Period_us (For 2 pulses per rev)
                    fan_rpm[i] = (uint16_t)(30000000UL / period);
                } else {
                    fan_rpm[i] = 0;
                }
            }
        }
    }

    if (RawHID.available() > 0) {
        int count = 0;
        while (RawHID.available() && count < 64) {
            rawhid_rx[count++] = RawHID.read();
        }
        if (count == 0) return; 

        uint8_t len = rawhid_rx[0];
        if (len > 62) len = 62; 

        uint8_t ptr = 1;
        uint8_t resp_ptr = 1;
        memset(rawhid_tx, 0, 64);
        
        while (ptr <= len) {
            uint8_t seq = rawhid_rx[ptr++];
            if (ptr > len) break;
            uint8_t op = rawhid_rx[ptr++];
            if (ptr > len + 1 && op != 0) break;
            
            rawhid_tx[resp_ptr++] = seq;
            rawhid_tx[resp_ptr++] = op;
            
            uint8_t cmd = rawhid_rx[ptr++];
            rawhid_tx[resp_ptr++] = cmd;
            
            switch (op) {
                case OP_READ_TWO: // 0x09
                    if (cmd == CMD_FIRMWARE_ID) {
                        rawhid_tx[resp_ptr++] = 0x00;
                        rawhid_tx[resp_ptr++] = 0x24; // 2.4 (4 Fans)
                    } else if (cmd == CMD_FAN_READ_RPM) {
                        // Return Cached RPM (Calculated in background loop)
                        uint8_t index = active_fan_index;
                        if (index > 3) index = 0;
                        
                        uint16_t rpm = fan_rpm[index];
                        rawhid_tx[resp_ptr++] = index; // Add Fan ID to response
                        rawhid_tx[resp_ptr++] = rpm & 0xFF;
                        rawhid_tx[resp_ptr++] = (rpm >> 8) & 0xFF;
                    } else if (cmd == CMD_FAN_MAX_RPM) {
                        uint16_t max_rpm = 2500;
                        rawhid_tx[resp_ptr++] = max_rpm & 0xFF;
                        rawhid_tx[resp_ptr++] = (max_rpm >> 8) & 0xFF;
                    } else {
                        rawhid_tx[resp_ptr++] = 0;
                        rawhid_tx[resp_ptr++] = 0;
                    }
                    break;
                
                case OP_WRITE_ONE: // 0x06
                    if (ptr + 0 > len + 1) break;
                    {
                        uint8_t val = rawhid_rx[ptr++];
                        if (cmd == CMD_FAN_SELECT) {
                            if (val <= 3) active_fan_index = val;
                        } else if (cmd == CMD_FAN_FIXED_PWM) {
                            set_pwm(active_fan_index, val);
                        }
                    }
                    break;
            }
        }
        
        rawhid_tx[0] = resp_ptr - 1; 
        RawHID.write(rawhid_tx, 64);
    }
}
