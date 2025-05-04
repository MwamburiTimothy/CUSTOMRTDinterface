#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>


// configuring UART for virtual terminal
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void uart_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<TXEN0);  // Enable transmitter
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // 8-bit data
}

void uart_transmit(char data) {
    while (!( UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str) {
    while(*str) {
        uart_transmit(*str++);
    }
}

void uart_print_float(float val) {
    char buffer[16];
    dtostrf(val, 6, 2, buffer);
    uart_print(buffer);
}

// SPI configuration for PGA and ADC
void spi_init(void) {
    DDRB |= (1<<PB2)|(1<<PB3)|(1<<PB5); // SS, MOSI, SCK as outputs
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // Enable SPI, Master mode, Fosc/16
}

uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

// Configure the PGA
void pga_config(uint8_t gain) {
    PORTB &= ~(1<<PB2);  // CS low
    spi_transfer(0x40);  // Write to gain register
    spi_transfer(gain);  // Gain: 0x00 = 1x, 0x01 = 2x, 0x03 = 8x
    PORTB |= (1<<PB2);   // CS high

    _delay_ms(1);        // Allow PGA to settle
}

void pga_select_channel(uint8_t ch) {
    PORTB &= ~(1<<PB2);
    spi_transfer(0x20);  // Write to channel select register
    spi_transfer(ch);    // 0x00 = CH0
    PORTB |= (1<<PB2);
}

//MCP3004 ADC reading
uint16_t adc_read_mcp3004(uint8_t channel) {
    uint8_t command = 0b11000000 | (channel << 3);  // Start + single-ended + channel

    PORTB &= ~(1<<PB2);
    spi_transfer(command);
    uint8_t high = spi_transfer(0x00);
    uint8_t low = spi_transfer(0x00);
    PORTB |= (1<<PB2);

    return ((high & 0x0F) << 8) | low;
}

int main(void) {
    uart_init(MYUBRR);
    spi_init();

    DDRD &= ~(1<<PD2);  // RTD type select pin as input
    PORTD &= ~(1<<PD2); // Has external pull-down

    DDRB |= (1<<PB2);   // CS pin for SPI
    PORTB |= (1<<PB2);  // CS idle high

    uart_print("RTD Temperature Reader Starting...\r\n");

    const float VREF = 3.3;
    const float ADC_RES = 1023.0;
    const float I_EXC = 0.001; // 0.1 mA
    const float ALPHA = 0.00385;

    while (1) {
        // RTD type selection
        uint8_t is_pt100 = (PIND & (1<<PD2));
        float R0 = is_pt100 ? 100.0 : 1000.0;
        uint8_t gain = is_pt100 ? 0x03 : 0x00; // 8x for Pt100, 1x for Pt1000

        //PGA Setup
        pga_select_channel(0x00); // CH0
        pga_config(gain);

        _delay_ms(10);

        //ADC Read
        uint16_t raw = adc_read_mcp3004(0);

        float v_rtd = (raw / ADC_RES) * VREF;
        float r_rtd = v_rtd / I_EXC;
        float temp = (r_rtd - R0) / (R0 * ALPHA);

        // Output to virtual terminal
        uart_print("RTD: ");
        uart_print(is_pt100 ? "Pt100" : "Pt1000");
        uart_print(" | Temp: ");
        uart_print_float(temp);
        uart_print(" C\r\n");

        _delay_ms(1000);
    }
}

