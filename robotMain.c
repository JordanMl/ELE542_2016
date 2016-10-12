#include <avr/io.h>
#include <avr/interrupt.h>


#ifndef F_CPU
#define F_CPU 16000000UL            /* Crystal 16.000 Mhz */
#endif
#define UART_BAUD_RATE 9600         /* 9600 baud */
#define BUF_SIZE 4


#define UART_BAUD_SELECT (F_CPU/(UART_BAUD_RATE*16l)-1)


/* CIRCULAR BUFFER */
struct circularBuffer {
	char *bufIn;
	char *bufOut;
	char *bufStart;
	char *bufEnd;
}buffer;


char bufferTab[BUF_SIZE];
char vitesse;
char angle;

void bufferPush (char inData){

	*(buffer.bufIn) = inData;
	if(buffer.bufIn==buffer.bufEnd){	//End of buffer
		buffer.bufIn = buffer.bufStart;
	}
	else
		buffer.bufIn += 1;
}

char bufferPull (){

	char outValue;
	if(buffer.bufOut==buffer.bufIn){  //Buffer is empty
		return 0;
	}
	outValue = *(buffer.bufOut);
	if(buffer.bufOut==buffer.bufEnd){
		buffer.bufOut = buffer.bufStart;
	}
	return outValue;
}

ISR(USART_TXC_vect)
/* signal handler for uart txd ready interrupt */
{

}


ISR(USART_RXC_vect)
/* signal handler for receive complete interrupt */
{
    unsigned char echo = UDR;
    bufferPush(echo);
    UDR = echo;
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

int main(void)
{
    char etat = 0;
    char cmd;

	buffer.bufIn = &bufferTab[0];
	buffer.bufOut = &bufferTab[0];
	buffer.bufStart = &bufferTab[0];
	buffer.bufEnd = &bufferTab[BUF_SIZE];

    uart_init();	/* init the UART transmit buffer */
    sei();			/* enable interrupts */

    while(1){
        switch(etat){
            case 0 : cmd = bufferPull();
                     if(cmd == 0xF1)
                        etat = 1;
                     break;

            case 1 : vitesse = bufferPull();
                     etat = 2;
                     break;

            case 2 : angle = bufferPull();
                     etat = 0;
                     break;
        }
    }

}
