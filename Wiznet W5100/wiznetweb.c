/************************************************************
Wiznet W5100 web server example for CodeVisionAVR V2.60
Based on code from: http://www.ermicro.com/blog/?p=1773

Hardware: Arduino Uno ATmega328 board + Wiznet W5100 shield
from www.dfrobot.com

Connect a LED on PORTD bit 7 to +5V using a 1k resistor.
A LM35DZ temperature sensor connected to analog input
AD0 (PORTC bit 0) is used to measure ambient temperature.
************************************************************/

#include <io.h>
#include <iobits.h>
#include <string.h>
#include <stdio.h>
#include <delay.h>
#include <stdint.h>

#define sei() #asm("sei")
#define F_CPU _MCU_CLOCK_FREQUENCY_

// ATmega328 SPI I/O
#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define SPI_CS   PORTB2

#define LED_OFF SETBIT(PORTD,7) // LED on PORTD bit 7 off
#define LED_ON CLRBIT(PORTD,7) // LED on PORTD bit 7 on

// Wiznet W5100 Op Codes
#define WIZNET_WRITE_OPCODE 0xF0
#define WIZNET_READ_OPCODE 0x0F

// Wiznet W5100 Register Addresses
#define MR         0x0000      // Mode Register
#define GAR        0x0001      // Gateway Address: 0x0001 to 0x0004
#define SUBR       0x0005      // Subnet mask Address: 0x0005 to 0x0008
#define SAR        0x0009      // Source Hardware Address (MAC): 0x0009 to 0x000E
#define SIPR       0x000F      // Source IP Address: 0x000F to 0x0012
#define RMSR       0x001A      // RX Memory Size Register
#define TMSR       0x001B      // TX Memory Size Register
#define S0_MR	   0x0400      // Socket 0: Mode Register Address
#define S0_CR	   0x0401      // Socket 0: Command Register Address
#define S0_IR	   0x0402      // Socket 0: Interrupt Register Address
#define S0_SR	   0x0403      // Socket 0: Status Register Address
#define S0_PORT    0x0404      // Socket 0: Source Port: 0x0404 to 0x0405
#define SO_TX_FSR  0x0420      // Socket 0: Tx Free Size Register: 0x0420 to 0x0421
#define S0_TX_RD   0x0422      // Socket 0: Tx Read Pointer Register: 0x0422 to 0x0423
#define S0_TX_WR   0x0424      // Socket 0: Tx Write Pointer Register: 0x0424 to 0x0425
#define S0_RX_RSR  0x0426      // Socket 0: Rx Received Size Pointer Register: 0x0425 to 0x0427
#define S0_RX_RD   0x0428      // Socket 0: Rx Read Pointer: 0x0428 to 0x0429
#define TXBUFADDR  0x4000      // W5100 Send Buffer Base Address
#define RXBUFADDR  0x6000      // W5100 Read Buffer Base Address// S0_MR values
#define MR_CLOSE	  0x00    // Unused socket
#define MR_TCP		  0x01    // TCP
#define MR_UDP		  0x02    // UDP
#define MR_IPRAW	  0x03	  // IP LAYER RAW SOCK
#define MR_MACRAW	  0x04	  // MAC LAYER RAW SOCK
#define MR_PPPOE	  0x05	  // PPPoE
#define MR_ND		  0x20	  // No Delayed Ack(TCP) flag
#define MR_MULTI	  0x80	  // support multicating// S0_CR values
#define CR_OPEN          0x01	  // Initialize or open socket
#define CR_LISTEN        0x02	  // Wait connection request in tcp mode(Server mode)
#define CR_CONNECT       0x04	  // Send connection request in tcp mode(Client mode)
#define CR_DISCON        0x08	  // Send closing reqeuset in tcp mode
#define CR_CLOSE         0x10	  // Close socket
#define CR_SEND          0x20	  // Update Tx memory pointer and send data
#define CR_SEND_MAC      0x21	  // Send data with MAC address, so without ARP process
#define CR_SEND_KEEP     0x22	  // Send keep alive message
#define CR_RECV          0x40	  // Update Rx memory buffer pointer and receive data// S0_SR values
#define SOCK_CLOSED      0x00     // Closed
#define SOCK_INIT        0x13	  // Init state
#define SOCK_LISTEN      0x14	  // Listen state
#define SOCK_SYNSENT     0x15	  // Connection state
#define SOCK_SYNRECV     0x16	  // Connection state
#define SOCK_ESTABLISHED 0x17	  // Success to connect
#define SOCK_FIN_WAIT    0x18	  // Closing state
#define SOCK_CLOSING     0x1A	  // Closing state
#define SOCK_TIME_WAIT	 0x1B	  // Closing state
#define SOCK_CLOSE_WAIT  0x1C	  // Closing state
#define SOCK_LAST_ACK    0x1D	  // Closing state
#define SOCK_UDP         0x22	  // UDP socket
#define SOCK_IPRAW       0x32	  // IP raw mode socket
#define SOCK_MACRAW      0x42	  // MAC raw mode socket
#define SOCK_PPPOE       0x5F	  // PPPOE socket
#define TX_BUF_MASK      0x07FF   // Tx 2K Buffer Mask:
#define RX_BUF_MASK      0x07FF   // Rx 2K Buffer Mask:
#define NET_MEMALLOC     0x05     // Use 2K of Tx/Rx Buffer
#define TCP_PORT         80       // TCP/IP Port

// Debugging Mode, 0 - Debug OFF, 1 - Debug ON
#define _DEBUG_MODE      0

#if _DEBUG_MODE
  #define BAUD_RATE 19200
#endif

flash uint8_t mac_addr[] = {0x00,0x16,0x36,0xDE,0x58,0xF6};
flash uint8_t ip_addr[] = {192,168,2,10};
flash uint8_t sub_mask[] = {255,255,255,0};
flash uint8_t gtw_addr[] = {192,168,2,1};
    
// Define W5100 Socket Register and Variables Used
uint8_t sockreg;

#define MAX_BUF 1024

uint8_t buf[MAX_BUF];
int8_t temperature;

void SPI_Write(uint16_t addr,uint8_t data)
{
  // Activate the CS pin
  SPI_PORT &= ~(1<<SPI_CS);  
  // Start Wiznet W5100 Write OpCode transmission
  SPDR = WIZNET_WRITE_OPCODE;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));  
  // Start Wiznet W5100 Address High Bytes transmission
  SPDR = (addr & 0xFF00) >> 8;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));  
  // Start Wiznet W5100 Address Low Bytes transmission
  SPDR = addr & 0x00FF;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));   

  // Start Data transmission
  SPDR = data;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));  
  // CS pin is not active
  SPI_PORT |= (1<<SPI_CS);
}

unsigned char SPI_Read(uint16_t addr)
{
  // Activate the CS pin
  SPI_PORT &= ~(1<<SPI_CS);  
  // Start Wiznet W5100 Read OpCode transmission
  SPDR = WIZNET_READ_OPCODE;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));  
  // Start Wiznet W5100 Address High Bytes transmission
  SPDR = (addr & 0xFF00) >> 8;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));  
  // Start Wiznet W5100 Address Low Bytes transmission
  SPDR = addr & 0x00FF;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));   

  // Send Dummy transmission for reading the data
  SPDR = 0x00;  
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));  

  // CS pin is not active
  SPI_PORT |= (1<<SPI_CS);  
  return(SPDR);
}

void W5100_Init(void)
{
  // Ethernet Setup
  // Setting the Wiznet W5100 Mode Register: 0x0000
  SPI_Write(MR,0x80);            // MR = 0b10000000;  
  // Setting the Wiznet W5100 Gateway Address (GAR): 0x0001 to 0x0004
  SPI_Write(GAR + 0,gtw_addr[0]);
  SPI_Write(GAR + 1,gtw_addr[1]);
  SPI_Write(GAR + 2,gtw_addr[2]);
  SPI_Write(GAR + 3,gtw_addr[3]);  
  // Setting the Wiznet W5100 Source Address Register (SAR): 0x0009 to 0x000E
  SPI_Write(SAR + 0,mac_addr[0]);
  SPI_Write(SAR + 1,mac_addr[1]);
  SPI_Write(SAR + 2,mac_addr[2]);
  SPI_Write(SAR + 3,mac_addr[3]);
  SPI_Write(SAR + 4,mac_addr[4]);
  SPI_Write(SAR + 5,mac_addr[5]);  
  // Setting the Wiznet W5100 Sub Mask Address (SUBR): 0x0005 to 0x0008
  SPI_Write(SUBR + 0,sub_mask[0]);
  SPI_Write(SUBR + 1,sub_mask[1]);
  SPI_Write(SUBR + 2,sub_mask[2]);
  SPI_Write(SUBR + 3,sub_mask[3]);  
  // Setting the Wiznet W5100 IP Address (SIPR): 0x000F to 0x0012
  SPI_Write(SIPR + 0,ip_addr[0]);
  SPI_Write(SIPR + 1,ip_addr[1]);
  SPI_Write(SIPR + 2,ip_addr[2]);
  SPI_Write(SIPR + 3,ip_addr[3]);    

  // Setting the Wiznet W5100 RX and TX Memory Size (2KB),
  SPI_Write(RMSR,NET_MEMALLOC);
  SPI_Write(TMSR,NET_MEMALLOC);
}

void close(uint8_t sock)
{
   if (sock != 0) return;
   // Send Close Command
   SPI_Write(S0_CR,CR_CLOSE);   
   // Waiting until the S0_CR is clear
   while(SPI_Read(S0_CR));
}

void disconnect(uint8_t sock)
{
   if (sock != 0) return;   
   // Send Disconnect Command
   SPI_Write(S0_CR,CR_DISCON);   
   // Wait for Disconecting Process
   while(SPI_Read(S0_CR));
}

uint8_t socket(uint8_t sock,uint8_t eth_protocol,uint16_t tcp_port)
{
    uint8_t retval=0;    
    if (sock != 0) return retval;

    // Make sure we close the socket first
    if (SPI_Read(S0_SR) == SOCK_CLOSED) {
      close(sock);
    }    
    // Assigned Socket 0 Mode Register
    SPI_Write(S0_MR,eth_protocol);

    // Now open the Socket 0
    SPI_Write(S0_PORT,((tcp_port & 0xFF00) >> 8 ));
    SPI_Write(S0_PORT + 1,(tcp_port & 0x00FF));
    SPI_Write(S0_CR,CR_OPEN);                   
    // Open Socket    
    // Wait for Opening Process
    while(SPI_Read(S0_CR));    
    // Check for Init Status
    if (SPI_Read(S0_SR) == SOCK_INIT)
      retval=1;
    else
      close(sock);

    return retval;
}

uint8_t listen(uint8_t sock)
{
   uint8_t retval = 0;   
   if (sock != 0) return retval;   
   if (SPI_Read(S0_SR) == SOCK_INIT) {
     // Send the LISTEN Command
     SPI_Write(S0_CR,CR_LISTEN);

     // Wait for Listening Process
     while(SPI_Read(S0_CR));     
     // Check for Listen Status
     if (SPI_Read(S0_SR) == SOCK_LISTEN)
       retval=1;
     else
       close(sock);
    }
    return retval;
}

uint16_t send(uint8_t sock,const uint8_t *buf,uint16_t buflen)
{
    uint16_t ptr,offaddr,realaddr,txsize,timeout;   

    if (buflen <= 0 || sock != 0) return 0;
    
#if _DEBUG_MODE
    printf("Send Size: %d\r\n",buflen);
#endif
    
    // Make sure the TX Free Size Register is available
    txsize=SPI_Read(SO_TX_FSR);
    txsize=(((txsize & 0x00FF) << 8 ) + SPI_Read(SO_TX_FSR + 1));
    
#if _DEBUG_MODE
    printf("TX Free Size: %d\r\n",txsize);
#endif 
   
    timeout=0;
    while (txsize < buflen) {
     delay_ms(1);     txsize=SPI_Read(SO_TX_FSR);
     txsize=(((txsize & 0x00FF) << 8 ) + SPI_Read(SO_TX_FSR + 1));     
     // Timeout for approx 1000 ms
     if (timeout++ > 1000) {

#if _DEBUG_MODE
       printf("TX Free Size Error!\r\n");
#endif

       // Disconnect the connection
       disconnect(sock);
       return 0;
     }
   }	

   // Read the Tx Write Pointer
   ptr = SPI_Read(S0_TX_WR);
   offaddr = (((ptr & 0x00FF) << 8 ) + SPI_Read(S0_TX_WR + 1));

#if _DEBUG_MODE
    printf("TX Buffer: %x\r\n",offaddr);
#endif	

    while(buflen) {
      buflen--;
      // Calculate the real W5100 physical Tx Buffer Address
      realaddr = TXBUFADDR + (offaddr & TX_BUF_MASK);      
      // Copy the application data to the W5100 Tx Buffer
      SPI_Write(realaddr,*buf);
      offaddr++;
      buf++;
    }

    // Increase the S0_TX_WR value, so it point to the next transmit
    SPI_Write(S0_TX_WR,(offaddr & 0xFF00) >> 8 );
    SPI_Write(S0_TX_WR + 1,(offaddr & 0x00FF));	

    // Now Send the SEND command
    SPI_Write(S0_CR,CR_SEND);

    // Wait for Sending Process
    while(SPI_Read(S0_CR));	

    return 1;
}

uint16_t recv(uint8_t sock,uint8_t *buf,uint16_t buflen)
{
    uint16_t ptr,offaddr,realaddr;   	

    if (buflen <= 0 || sock != 0) return 0;   

    // If the request size > MAX_BUF,just truncate it
    if (buflen > MAX_BUF)
      buflen=MAX_BUF - 2;    
    // Read the Rx Read Pointer
    ptr = SPI_Read(S0_RX_RD);
    offaddr = (((ptr & 0x00FF) << 8 ) + SPI_Read(S0_RX_RD + 1));
    
#if _DEBUG_MODE
    printf("RX Buffer: %x\r\n",offaddr);
#endif	

    while(buflen) {
      buflen--;
      realaddr=RXBUFADDR + (offaddr & RX_BUF_MASK);
      *buf = SPI_Read(realaddr);
      offaddr++;
      buf++;
    }
    *buf='\0';        // String terminated character

    // Increase the S0_RX_RD value, so it points to the next receive
    SPI_Write(S0_RX_RD,(offaddr & 0xFF00) >> 8 );
    SPI_Write(S0_RX_RD + 1,(offaddr & 0x00FF));	

    // Now Send the RECV command
    SPI_Write(S0_CR,CR_RECV);
    delay_us(5);    // Wait for Receive Process

    return 1;
}

uint16_t recv_size(void)
{
  return ((SPI_Read(S0_RX_RSR) & 0x00FF) << 8 ) + SPI_Read(S0_RX_RSR + 1);
}

interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
  static unsigned char tenms=1;  
  
  tenms++;                  
  // Read ADC every 20 x 10ms = 200 milisecond
  if (tenms >= 20) {
    // Use internal 1.1V voltage reference
    // Set ADMUX Channel to read analog input 0
    ADMUX=(1<<REFS0) | (1<<REFS1) | 0;   
    // Start conversion by setting ADSC on ADCSRA Register
    ADCSRA |= (1<<ADSC);    
    // wait until convertion complete ADSC=0 -> Complete
    while (ADCSRA & (1<<ADSC));    
    // Get the ADC Result and calculate the temperature in degrees C
    temperature = (int8_t) (ADCW*(1100.0/1024.0/10.0));

    tenms=1;	 
  }  // Start counter from 0x94, overflow at 10 mSec
  TCNT0=0x94;
}

void main(void){
  uint8_t ledstate;
  uint8_t sockstat;
  uint16_t rsize;
  char *getidx,*postidx;
  uint8_t radiostat0[10],radiostat1[10],temp[4];

  SETBIT(DDRD,7); // Set PORTD bit 7 as Output
  LED_OFF; // Turn LED on PORTD bit 7 off
  ledstate = 0;	     

#if _DEBUG_MODE
  // Init USART
  UBRR0H = (((F_CPU/BAUD_RATE)/16)-1)>>8;		// set baud rate
  UBRR0L = (((F_CPU/BAUD_RATE)/16)-1);
  UCSR0B = (1<<RXEN0)|(1<<TXEN0); 				// enable Rx & Tx
  UCSR0C=  (1<<UCSZ01)|(1<<UCSZ00);  	        // config USART; 8N1
#endif  
  
  // Initialize ATMega328 ADC Peripheral
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);  // Free running ADC Mode
  ADCSRB = 0x00;  
  
  // Initialize the AVR ATMega328 SPI Peripheral
  // Set MOSI (PORTB3),SCK (PORTB5) and PORTB2 (SS) as output, others as input
  SPI_DDR = (1<<PORTB3)|(1<<PORTB5)|(1<<PORTB2);  // CS pin is not active
  SPI_PORT |= (1<<SPI_CS);  // Enable SPI, Master Mode 0, set the clock rate fck/2
  SPCR = (1<<SPE)|(1<<MSTR);
  SPSR |= (1<<SPI2X);  

  // Initialize ATMega328 Timer/Counter0 Peripheral
  TCCR0A=0x00;                  // Normal Timer0 Operation
  TCCR0B=(1<<CS02)|(1<<CS00);   // Use maximum prescaller: Clk/1024
  TCNT0=0x94;                   // Start counter from 0x94, overflow at 10 mSec
  TIMSK0=(1<<TOIE0);            // Enable Counter Overflow Interrupt
  sei();                        // Enable Interrupt
  
  // delay necessary for the Wiznet W5100 chip to init after reset
  delay_ms(500);
  
  // Initialize the W5100 Ethernet
  W5100_Init();  
  
  // Initialize used variables
  sockreg=0;
  temperature=0;
  
#if _DEBUG_MODE
  printf("\r\n\r\nWEB Server Debug Mode\r\n\r\n");
#endif

  // Loop forever
  for(;;){
    sockstat=SPI_Read(S0_SR);
    switch(sockstat) {
    
    case SOCK_CLOSED:
        if (socket(sockreg,MR_TCP,TCP_PORT) > 0) {
	       // Listen to Socket 0
	       if (listen(sockreg) <= 0) delay_ms(1);
        
#if _DEBUG_MODE
           printf("Socket Listen!\r\n");
#endif

	       }
	break;     
    
    case SOCK_ESTABLISHED:
	// Get the client request size
    rsize=recv_size();
        
#if _DEBUG_MODE
	printf("Size: %d\r\n",rsize);
#endif

    delay_ms(10); // delay needed to make the buttons work

	if (rsize > 0) {
	  // Now read the client Request
	  if (recv(sockreg,buf,rsize) <= 0) break;
      
#if _DEBUG_MODE
      {
      char *p;
  	  printf("Content:\r\n");
      // display strings longer than 255 characters
      p=buf;
      while (*p) putchar(*p++);
      printf("\r\n");
      }
#endif
      // Check the Request Header
	  getidx=strstrf((char *)buf,"GET /");
	  postidx=strstrf((char *)buf,"POST /");	  
      if (getidx || postidx) {
      
#if _DEBUG_MODE
	    printf("Req. Check!\r\n");
#endif

        // Now check the Radio Button for POST request
	    if (postidx) {
	        if (strstrf((char *)buf,"radio=0"))
               {
               LED_OFF;
               ledstate = 0;
               }
            else
            if (strstrf((char *)buf,"radio=1"))
               {
               LED_ON;
               ledstate = 1;
               }
            }
            
#if _DEBUG_MODE
	    printf("Req. Send!\r\n");
#endif

	    // Create the HTTP Response	Header
	    strcpyf((char *)buf,"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
	    strcatf((char *)buf,"<html><body><span style=\"color:#0000A0\">\r\n");
	    strcatf((char *)buf,"<h1>CodeVisionAVR Embedded Web Server</h1>\r\n");
	    strcatf((char *)buf,"<h3>Arduino Uno Mega328 and Ethernet Shield from www.dfrobot.com</h3>\r\n");
	    strcatf((char *)buf,"<p><form method=\"POST\">\r\n");	    
        // Now Send the HTTP Response
	    if (send(sockreg,buf,strlen((char *)buf)) <= 0) break;

	    // Create the HTTP Temperature Response
	    sprintf((char *)temp,"%d",temperature);        
        // Convert temperature value to string
        strcpyf((char *)buf,"<strong>Temp: <input type=\"text\" size=2 value=\"");
	    strcat((char *)buf,temp);
	    strcatf((char *)buf,"\"> <sup>O</sup>C\r\n");									

	    if (ledstate == 1) {
	      radiostat0[0] = 0;
	      strcpyf(radiostat1,"checked");
	    } else {
	      strcpyf(radiostat0,"checked");
	      radiostat1[0] = 0;
	    }            
        // Create the HTTP Radio Button 0 Response
	    strcatf((char *)buf,"<p><input type=\"radio\" name=\"radio\" value=\"0\" ");
	    strcat((char *)buf,radiostat0);
	    strcatf((char *)buf,">PORTD.7 LED=Off\r\n");
	    strcatf((char *)buf,"<br><input type=\"radio\" name=\"radio\" value=\"1\" ");
	    strcat((char *)buf,radiostat1);
	    strcatf((char *)buf,">PORTD.7 LED=On\r\n");
 	    strcatf((char *)buf,"</strong><p>\r\n");
	    strcatf((char *)buf,"<input type=\"submit\">\r\n");
	    strcatf((char *)buf,"</form></span></body></html>\r\n");            
        
        // Now Send the HTTP Remaining Response
	    if (send(sockreg,buf,strlen((char *)buf)) <= 0) break;
          }	  
      // Disconnect the socket
	  disconnect(sockreg);
        } else
	  delay_ms(2); //delay_us(10);    // Wait for request	
      break;
      
      case SOCK_FIN_WAIT:
      case SOCK_CLOSING:
      case SOCK_TIME_WAIT:
      case SOCK_CLOSE_WAIT:
      case SOCK_LAST_ACK:
      // Force to close the socket
	  close(sockreg);
    
#if _DEBUG_MODE
	  printf("Socket Close!\r\n");
#endif

      break;
    }
  }
}
