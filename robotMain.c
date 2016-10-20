#include <avr/io.h>
#include <avr/interrupt.h>


#ifndef F_CPU
#define F_CPU 16000000UL            /* Crystal 16.000 Mhz */
#endif
#define UART_BAUD_RATE 9600         /* 9600 baud */
#define BUF_SIZE 3


#define UART_BAUD_SELECT (F_CPU/(UART_BAUD_RATE*16l)-1)

/*-------GLOBAL DECLARATIONS--------*/

/* uart globals */


unsigned char debugMsg=0;
unsigned char data_send = 1; //data send with uart

unsigned char bufferTab[BUF_SIZE];
int vitesse;
unsigned char angle;


struct circularBuffer {
	unsigned char *bufIn;
	unsigned char *bufOut;
	unsigned char *bufStart;
	unsigned char *bufEnd;
}buffer;
/*----------------------------------*/

void bufferPush (unsigned char inData);


/*-------------INTERRUPTS-----------*/
ISR(USART_TXC_vect)
/* signal handler for uart txd ready interrupt */
{
   data_send = 1;
}

ISR(USART_RXC_vect)
/* signal handler for receive complete interrupt */
{
    unsigned char echo = UDR;
    bufferPush(echo);
	if (debugMsg == 0){
		UDR = echo;
	}
}

ISR(TIMER1_OVF_vect)
/* signal handler for timer1 overflow*/
{
        OCR1A = vitesse;
        OCR1B = vitesse;
}
/*------------------------------------*/

/*-------------FUNCTIONS--------------*/
void bufferPush (unsigned char inData){

	*(buffer.bufIn) = inData;
	if(buffer.bufIn==buffer.bufEnd){	//End of buffer
		buffer.bufIn = buffer.bufStart;
	}
	else
		buffer.bufIn += 1;
}

unsigned char bufferPull (){

	unsigned char outValue = 0;
	outValue = *(buffer.bufOut);
	if(buffer.bufOut==buffer.bufEnd){
		buffer.bufOut = buffer.bufStart;
	}
	else{
		buffer.bufOut += 1;
	}
	return outValue;
}

void uartDebugMsg(char *msg,int msgSize){

	debugMsg = 1;
	data_send = 0;
	UDR = 0xFE;
	for(int i=0;i<msgSize;i++){
		while(data_send == 0);
		data_send = 0;
		UDR = *msg;
		msg++;
	}
	UDR = 0xFF;
	debugMsg = 0;
}

void uart_init(void)
/* initialize uart */
{
   /* configure asynchronous operation (UMSEL), no parity(UPM1,UPM1), 1 stop bit(USBS), 8 data bits(UCSZ1,UCSZ0), Tx on rising edge(UCPOL) */
   UCSRC = ((1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0)|(0<<UCPOL));
   /* enable RxD/TxD(RXEN,TXEN) and ints(RXCIE,TXCIE), 8 data bits(UCSZ2) */
   UCSRB = ((1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN)|(0<<UCSZ2));
   /* set baud rate */
   UBRRH = (unsigned char)(UART_BAUD_SELECT >> 8);
   UBRRL = (unsigned char)(UART_BAUD_SELECT & 0x00FF);
}

void pwm_init(void)
/* initialize PWM */
{
    /* configure niveau haut au débordement et bas à la comparaison (COM1A1/B1 = 1 ; COMA0/B0 = 0)
        Mode Fast PWM (WGM11 = 1 ; WGM10 = 0 )*/
    TCCR1A = ((1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(0<<WGM10));
    /* Mode Fast PWM (WGM13/12 = 1) fréquence du compteur = 16MHz/8 (CS12/10 = 0 ; CS11 = 1) */
    TCCR1B = ((1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10));
    /*ICR1=TOP=10000=0x2710 for fpwm = 200Hz */
    ICR1H = 0x27;
    ICR1L = 0x10;

    /* Timer1 Overflow enable*/
    TIMSK = (1<<TOIE1);
}

void init(void)
/* initialize GPIO*/
{
    DDRD = ((1<<DDD7)|(1<<DDD6)|(1<<DDD5)|(1<<DDD4)|(1<<DDD3)|(1<<DDD2));
}

void calcule_vitesse(unsigned char v)
{
    if (v >= 0 && v <= 200){
        if (v == 100){
            vitesse = 0;
            /* Direction null */
			 PORTD &= ~(1<<PORTD7);
			 PORTD &= ~(1<<PORTD6);
			 PORTD &= ~(1<<PORTD3);
			 PORTD &= ~(1<<PORTD2);
        }
        else if (v < 100){
            vitesse = (-1)*(v-100)*100;
            /* Direction recule */
			PORTD = ((1<<PORTD7)|(1<<PORTD3));
			PORTD &= ~(1<<PORTD6);
			PORTD &= ~(1<<PORTD2);
        }
        else if (v > 100){
            vitesse = (v-100)*100;
            /* Direction Avance */
            PORTD = ((1<<PORTD6)|(1<<PORTD2));
			PORTD &= ~(1<<PORTD7);
			PORTD &= ~(1<<PORTD3);
        }
    }
}

int main(void){
    unsigned char etat = 0;
    unsigned char cmd;
	unsigned char vitesseTmp = 0;

	buffer.bufIn = &bufferTab[0];
	buffer.bufOut = &bufferTab[0];
	buffer.bufStart = &bufferTab[0];
	buffer.bufEnd = &bufferTab[BUF_SIZE-1];

    uart_init();
    init();
    pwm_init();
    sei();

    while(1){
        switch(etat){
            /* Etat Reception Commande */
            case 0 : cmd = bufferPull();
					 if(cmd == 0xF1){
                        etat = 1;
					 }
                     break;
            /* Etat Reception Vitesse */
            case 1 : cmd = bufferPull();
					 if(cmd == 0xF1 | cmd == 0xF0){
                        etat = 1; //Securite
						break;
					 }
					 else{
					 	vitesseTmp = cmd;
					 }
                     etat = 2;
                     break;
            /* Etat Reception Angle */
            case 2 : cmd = bufferPull();
					 if(cmd == 0xF1 | cmd == 0xF0){
                        etat = 1; //Securite
						break;
					 }
					 else{
						//TO DO : calcul angle
						calcule_vitesse(vitesseTmp);
					 }
                     etat = 0;
                     break;
        }
    }

}
/*------------------------------------*/



