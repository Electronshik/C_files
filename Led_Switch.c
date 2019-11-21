#include <mega328p.h>
#include <stdlib.h>
#include <delay.h>
#include <stdio.h>

unsigned char main_value = 0;
unsigned int switch_off_counter = 0;
unsigned char switch_off_counter_reset = 0;
char str[4] = "";

enum Bool
{
    FALSE,
    TRUE
};

enum State
{
    OFF,
    ON
};
unsigned char op_state = OFF;
unsigned char flag_of_change = FALSE;

interrupt [EXT_INT0] void ext_int0_isr(void)
{
    if ( !(PIND & (1 << PIND2)) )
    {
        delay_ms(50);
        if ( !(PIND & (1 << PIND2)) )
        {
            if ( PINC & (1 << PINC0) )
            {
                main_value++;
                if (main_value > 100)
                {
                	main_value = 100;
                }
                else
                {
                	flag_of_change = TRUE;
                }
            }   
            else
            {
                if (main_value > 0)
                {
                	main_value--;
                	flag_of_change = TRUE;
                }
            }
        } 
    }
}

interrupt [EXT_INT1] void ext_int1_isr(void)
{
// Place your code here

}

#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

#define RX_BUFFER_SIZE0 8
char rx_buffer0[RX_BUFFER_SIZE0];

unsigned char rx_wr_index0=0,rx_rd_index0=0;
unsigned char rx_counter0=0;

bit rx_buffer_overflow0;

// USART Receiver interrupt service routine
interrupt [USART_RXC] void usart_rx_isr(void)
{
char status,data;
status=UCSR0A;
data=UDR0;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   		rx_buffer0[rx_wr_index0++]=data;
	  	if (rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
	  	if (++rx_counter0 == RX_BUFFER_SIZE0)
	    {
		    rx_counter0=0;
		    rx_buffer_overflow0=1;
	    }
   }
}

char getchar(void)
{
	char data;
	while (rx_counter0==0);
	data=rx_buffer0[rx_rd_index0++];
	if (rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
	#asm("cli")
	--rx_counter0;
	#asm("sei")
	return data;
}

#define TX_BUFFER_SIZE0 8
char tx_buffer0[TX_BUFFER_SIZE0];

unsigned char tx_wr_index0=0,tx_rd_index0=0;
unsigned char tx_counter0=0;

// USART Transmitter interrupt service routine
interrupt [USART_TXC] void usart_tx_isr(void)
{
	if (tx_counter0)
	{
		--tx_counter0;
		UDR0=tx_buffer0[tx_rd_index0++];
		if (tx_rd_index0 == TX_BUFFER_SIZE0) tx_rd_index0=0;
	}
}

void putchar(char c)
{
	while (tx_counter0 == TX_BUFFER_SIZE0);
	#asm("cli")
	if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY)==0))
	{
	   tx_buffer0[tx_wr_index0++]=c;
	   if (tx_wr_index0 == TX_BUFFER_SIZE0) tx_wr_index0=0;
	   ++tx_counter0;
	}
	else
	   UDR0=c;
	#asm("sei")
}

char RS485_Receive_Byte();
void RS485_Transmit_Byte(char Byte);
void to_lamp_value(char lamp_code, unsigned char value);

interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
    if (op_state == ON)
    {
        if ( PINB & (1 << PINB7) )
        {
            switch_off_counter++;
        }
        else
        {
            switch_off_counter_reset++;
        }
    }
}

void main(void)
{
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 62,500 kHz
// Mode: Normal top=0xFF
// OC2A output: Disconnected
// OC2B output: Disconnected
// Timer Period: 4,096 ms
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (1<<CS22) | (0<<CS21) | (1<<CS20);
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2);

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Any change
// INT1: On
// INT1 Mode: Any change
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: Off
EICRA=(0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00);
EIMSK=(1<<INT1) | (1<<INT0);
EIFR=(1<<INTF1) | (1<<INTF0);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART0 Mode: Asynchronous
// USART Baud Rate: 9600
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(1<<RXCIE0) | (1<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x33;

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
ADCSRB=(0<<ACME);
// Digital input buffer on AIN0: On
// Digital input buffer on AIN1: On
DIDR1=(0<<AIN0D) | (0<<AIN1D);

// ADC initialization
// ADC disabled
ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/512k
// Watchdog timeout action: Reset
#pragma optsize-
WDTCSR=(0<<WDIF) | (0<<WDIE) | (1<<WDP3) | (1<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
WDTCSR=(0<<WDIF) | (0<<WDIE) | (1<<WDP3) | (0<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

#asm("sei")

delay_ms(250);  //delay for the switching on while button is pressed
//switching on
if ( PINB & (1 << PINB7) )
{
    DDRD |=   ( 1 << DDD5 );
    PORTD |=   (1 << PORTD5 ); //turn on the power relay  
}
else
{
    delay_ms(50);     //one more check
    if ( PINB & ( 1 << PINB7 ))
    {
        DDRD |=   ( 1 << DDD5 );
        PORTD |=   (1 << PORTD5 ); //turn on the power relay      
    }
}
op_state = ON;

to_lamp_value('A', main_value);
to_lamp_value('B', main_value);

while (1)
      {
      	if (flag_of_change)
      	{
      		to_lamp_value('A', main_value);
            to_lamp_value('B', main_value);
      	}
        if (op_state == ON)
        {
            if (switch_off_counter_reset > 20)
            {
                switch_off_counter = 0; //if button is released more than 4*20 = 80ms the switch off counter will be resetted
                switch_off_counter_reset = 0;
            }
            if (switch_off_counter > 400)
            {
                op_state = OFF;               //if button is pressed more than 4*400 = 1600ms then switching off     
                to_lamp_value('A', 0);
                to_lamp_value('B', 0);
                switch_off_counter = 0;
                switch_off_counter_reset = 0; 
                PORTD &= ~(1 << PORTD5); //turn off the power relay  
            }
        }

        #asm("wdr")
      }
}

void to_lamp_value(char lamp_code, unsigned char value)
{
	RS485_Transmit_Byte('L');
	RS485_Transmit_Byte(lamp_code);
	RS485_Transmit_Byte(':');
    itoa(value, str);
    if (value < 10)
    {
        RS485_Transmit_Byte(str[0]);
    }
    else if (value < 100)
    {
        RS485_Transmit_Byte(str[0]);
        RS485_Transmit_Byte(str[1]);
    }
    else
    {
        RS485_Transmit_Byte(str[0]);
        RS485_Transmit_Byte(str[1]);
        RS485_Transmit_Byte(str[2]);
    }
    RS485_Transmit_Byte(0x0A); //\n
}

void RS485_init(char Baud)
{
	DDRD  |=  (1 << DDD4);           // PA1 Output 
	                               // DE (Driver Enable ) MAX485
                                   // ~RE (Receive Enable) MAX485			
	UBRR0H = (unsigned char)( Baud >> 8 ); // Set Baudrate for USART0
	UBRR0L = (unsigned char)( Baud );
	//UCSR0B |= (1<<RXEN0)|(1<<TXEN0);         // Enable the Transmitter
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);    // Set frame: 8data, 1 stop			   
}

void RS485_Transmit_Byte(char Byte)
{
	//DDRD  |=  (1 << DDD4);           // PA1 Output  
	PORTD |=  (1 << PORTD4);         // PA1 -->  DE,Logic High 
	delay_ms(1);	
	UCSR0B |= (1<<TXEN0);         // Enable the Transmitter

	while ( !( UCSR0A & (1<<UDRE0)) ); // Is TX buffer Empty
    UDR0 = Byte;                      // Then send data     
    delay_ms(2);
    PORTD &=  ~(1 << PORTD4);         // PA1 --> ~RE,DE,Logic Low
}

char RS485_Receive_Byte()
{
	char Rxed_Data = '0';
	
	DDRD  |=   (1 << DDD4);           // PA1 Output  
	PORTD &=  ~(1 << PORTD4);         // PA1 --> ~RE,DE,Logic Low
	
	//UCSR0B |= (1<<RXEN0);           // Enable the Receiver
	
	//while ( !( UCSR0A & (1<<RXC0)) ); // Is RX buffer Empty
	//Rxed_Data   = UDR0 ;             // Read RXed data
	                     
    Rxed_Data = getchar();
    
	return Rxed_Data;
}