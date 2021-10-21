#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define WS_PORT			PORTB
#define WS_DIR			DDRB
#define WS_PIN			(1<<PB5)

#define WHITE_LED_ON_12
#define BLINK_MM

#define BOUD			9600

#define RED			3
#define GREEN			2
#define BLUE			3

#define COLOR_SS		r
#define COLOR_MM		b
#define COLOR_HH		g

#define INTERRUPT		INT0
#define INTERRUPT_EDGE		ISC00
#define INTERRUPT_CONFIG	EICRA
#define INTERRUPT_MASK		EIMSK
#define INTERRUPT_VECT		INT0_vect
#define INTERRUPT_DDR		DDRD
#define INTERRUPT_PIN		(1<<PD2)
#define INTERRUPT_PORT		PORTD

#define RX_DDR			DDRD
#define	RX_PIN			(1<<PD0)

#define DEFAULT_SS		0
#define DEFAULT_MM		0
#define DEFAULT_HH		0

void ws281x_asm_send( void *data, uint16_t datlen, uint8_t pin );

void init_uart();
void uart_process_time();
void uart_rx();

void init_rtc();

//Note: ws leds accept green then red then blue values
typedef struct {
	uint8_t g;
	uint8_t r;
	uint8_t b;
} TRGB;

typedef struct {
	uint8_t ss;
	uint8_t mm;
	uint8_t hh;
} TTIME;

typedef struct {
	uint8_t pos;
	uint8_t buff[16];
} TRX;

//Note: global variables are reset to 0.
TRX rx;
TRGB leds[60];
TTIME time = {DEFAULT_SS,DEFAULT_MM,DEFAULT_HH};

int main() {
	//Overclock internal oscilator 224 to operate near 11.0592 MHz
	OSCCAL = 224;

	//Set INT and RXD pins to input
	RX_DDR &= ~RX_PIN;

	INTERRUPT_DDR &= ~INTERRUPT_PIN;
	//Pullup INT pin to high with internal resisitor
	INTERRUPT_PORT |= INTERRUPT_PIN;

	//Set INT0 trigger method to any logical change
	INTERRUPT_CONFIG |= (1<<INTERRUPT_EDGE);
	//Enable INT0 interrupt
	INTERRUPT_MASK |= (1<<INTERRUPT);

	//Inits
	init_uart();
	init_rtc();

	while(1) {
		//Make AVR sleepy
		sleep_mode();

		//Process uart rx
		uart_rx();

		//Write dummies to control register of Timer2
		TCCR2B |= (1<<CS22)|(1<<CS20);
		//Wait for busy flags to process
		while(ASSR & ((1 << TCN2UB) | (1 << OCR2AUB) | (1 << TCR2AUB)));

		//Process ws leds
		//Clear leds buffer
		for( uint8_t i = 0; i < 60*3; i++ ) *((uint8_t*)leds + i) = 0;

		//Set led on position time.ss
		leds[time.ss].COLOR_SS = RED;

		//Set led on position time.mm
		//Blink mm led
	#ifdef BLINK_MM
		//Note: last bit (2^0 = 1) define if number is odd or even. Much better way than %2 == 0
		if( time.ss & 1 ) {
			leds[time.mm].COLOR_MM = BLUE;
		}
	#else
		leds[time.mm].COLOR_MM = BLUE;
	#endif
		//Process hour led
		//5*time.hh maps hour range from 0-12 to 0-60
		//Then we add minutes dividev by 5 to get smooth hour's led stepping.
		//And make sure that we not exited 60 led range
		leds[ ( 5*time.hh + time.mm/12 ) % 60 ].COLOR_HH = GREEN;

		//Turn on white led towards upside
		//If led 0 is turn off, then set it to white
		//If there is any color, then skip
	#ifdef WHITE_LED_ON_12
		if( !leds[0].r && !leds[0].g && !leds[0].b ) {
			leds[0].r = RED; leds[0].g = GREEN; leds[0].b = BLUE;
		}
	#endif

		//Send led buffer to leds on WS_PIN
		ws281x_asm_send( leds, 60, WS_PIN );

	}
	return 0;
}

void init_uart() {
	#define __UBRR__ F_CPU/16/BOUD-1

	//Set uart to 9600
	uint16_t ubrr = __UBRR__;
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)(ubrr);

	//Set 1bit stop and enable receiver
	UCSR0B = (1<<RXEN0);
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
}

void init_rtc() {
	//Wait some time for stabilize clocks
	_delay_ms(100);

	//Turn off Timer2 in time of init
	TIMSK2 &= ~( (1 << OCIE2A) | (1<<OCIE2B) | (1 << TOIE2) );
	//Turn on async mode
	ASSR |= (1<<AS2);
	//Reset timer
	TCNT2 = 0;

	//Set 128 prescaler: 32.768 / 128 = 1sec precisely
	TCCR2B |= (1<<CS22)|(1<<CS20);

	//Wait for busy flags to start operate
	while(ASSR & ((1 << TCN2UB) | (1 << OCR2AUB) | (1 << TCR2AUB)));

	//Enable Timer2 overflow interrupt
	TIMSK2 |= (1<<TOIE2);

	//Turn on global interrupts
	sei();

	//Setup sleep mode on PowerSave. (On PowerSave mode Timer2 can operate in async mode)
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	//Enable sleep mode
	sleep_enable();
}

ISR(TIMER2_OVF_vect) {
	//Handle passing one second

	if( ++time.ss == 60 ) {
		time.ss = 0;
		if( ++time.mm == 60 ) {
			time.mm = 0;
			if( ++time.hh == 24 ) {
				time.hh = 0;
			}
		}
	}
}

void uart_rx() {
	//Check if there is any data to receive in UDR register
	if( !(UCSR0A & (1<<RXC0) ) ) {
		//If not, return
		return;
	} else {
		//If yes, process time
		//uart_process_time();
		register uint8_t udr = UDR0;
		rx.buff[rx.pos++] = udr;

		//Process rx buffer data if enter was pressed
		if( udr == '\r' || udr == '\n' ) {
			uart_process_time();
			rx.pos = 0;
		}
	}
}

void uart_process_time() {
	//Time from uart comes in format: ssmmhh[enter]
	//Without : or spaces

	//If in buffer is not 7 symbols (ssmmhh\n), then return because of error
	if( rx.pos != 7 ) return;

	uint8_t t[3] = {0,0,0};

	//Convert ascii numbers to numbers
	//Note: ascii (for example) 15 is two bytes: '1' and '5'. '1' has value of (48 + 1 = 49) , '5' is (48 + 5 = 53). So to get number from ascii we have to subtract 48.
	for( uint8_t i = 0; i < 3; i++ ) {
		for( uint8_t p = 0; p < 2; p++ ) {
			t[i] += ( ( p == 0 ) ? 10 : 1 ) * ( rx.buff[2*i+p] - 48 );
		}
	}

	//Set new time
	time.hh = t[2];
	time.mm = t[1];
	time.ss = t[0];
}

ISR( INTERRUPT_VECT ) {
	//Dummy interrupt routine to wake up core when data is sent to uart rx
}

//Function to send buffer to ws leds.
void ws281x_asm_send( void *data, uint16_t datlen, uint8_t pin ) {

	uint8_t databyte=0, cnt, pinLO=~pin;
	WS_DIR |= pin;
	datlen *= 3;

#if F_CPU == 11059200

	asm volatile(
	"		lds		%[cnt],%[ws_port]	\n\t"			
	"		or		%[pinHI],%[cnt]		\n\t"	
	"		and		%[pinLO],%[cnt]		\n\t"		
	"mPTL%=:subi	%A6,1					\n\t"			
	"		sbci	%B6,0				\n\t"			
	"		brcs	exit%=				\n\t"			
	"		ld		%[databyte],X+		\n\t"			
	"		ldi		%[cnt],8			\n\t"		

	"oPTL%=:sts		%[ws_port],	%[pinHI]		\n\t"		
			"		nop				\n\t"	

	"		lsl		%[databyte]			\n\t"			
	"		brcs	.+2					\n\t"			
	"		sts		%[ws_port],	%[pinLO]\n\t"			
	"		nop						\n\t"		

	"		dec		%[cnt]				\n\t"		
	"		sts		%[ws_port],	%[pinLO]\n\t"			
	"		breq	mPTL%=				\n\t"			
													

	"		rjmp	oPTL%=				\n\t"			
	"exit%=:							\n\t"		
	:	[cnt]"=&d" (cnt)
	:	[databyte]"r" (databyte), [ws_port]"M" (_SFR_MEM_ADDR(WS_PORT)), [pinHI]"r" (pin), 
		[pinLO]"r" (pinLO), [data]"x" (data), [datlen]"r" (datlen)
	);

#endif

}
