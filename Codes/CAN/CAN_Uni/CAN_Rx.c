//Code uses CAN protocol to Recieve data from another Bluepill to blink the onboard led PC13
//Setup:
//CAN pins: Tx->PA12, Rx->PA11
//LED: PC13 (onboard)

#include "stm32f10x.h"

/*int power(int d, int p)
{
	int i = 0;
	int product = 0;
	for(i = 1; i <= p; i++)
		product = product*d;
	
	return product;
}*/

void GPIO_Initialize( )   //called by start up code
{
	RCC->APB2ENR |= (1<<4);   // Set clock for Port C
	GPIOC->CRH |= ( (1<<20) | (1<<21) );   //Output (50 Mhz) 
	GPIOC->CRH &= ~( (1<<22) | (1<<23) );   //General Purpose Output
}
void delay(int count)
{
  int i;
  for (i = 0; i < (count); i++); 
}
void test( ) //Flash LEDs to test
{
		GPIOC->BSRR = (1<<13);   //Set Pin13 High
		delay(8000000);   //1 Second Delay  
		GPIOC->BSRR = 1<<(13 + 16);   //Set Pin13 Low
		delay(8000000); 
}
void initCAN1( )
{
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;	       // enable clock for CAN1
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;            // enable clock for Alternate Function
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;           // enable clock for GPIO C
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;           // enable clock for GPIO A

	/*AFIO->MAPR   &= ~(3<<13);     // reset CAN remap
	AFIO->MAPR   |= 3<<13;            //   set CAN remap, use PD0, PD1

	GPIOD->CRL &= ~(0x0F<<0);
	GPIOD->CRL |=  (0x08<<0);          // CAN RX pin PD.0 Input Push Pull   
	GPIOD->CRL &= ~(0x0F<<4);
	GPIOD->CRL |=  (0x0B<<4);         //TX pin PD.1 AF Output Push Pull 
	*/
	
	
	GPIOA->CRH |= GPIO_CRH_MODE12_0;   //PA12 as AF Output Push Pull
	GPIOA->CRH |= GPIO_CRH_CNF12_1;
	
	GPIOA->CRH |= GPIO_CRH_CNF11_0;   //PA11 as Floating Input (reset state)
	GPIOA->CRH &= ~(GPIO_CRH_CNF11_1);
	GPIOA->CRH &=~(GPIO_CRH_MODE11);   //PA11 Input Mode (reset state)
	
	

	CAN1->MCR = 1;                            //Initialisation Mode  	
	CAN1->MCR |= 1<<4;                     //NART: No Automatic Retransmission  
	CAN1->IER =  1<<1;                       // FIFO 0 msg pending
	CAN1->BTR =  0x031C0009;              //see Q2
	CAN1->MCR &= 0;                      //Reset INRQ, Normal Operating Mode Started
	
	NVIC->ISER[0] |= (1 << 20);       // enable CAN1_Rx interrupt
}																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																													
#define Id0 0x1badcafe

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1

typedef struct  {
  unsigned int   id;                 // 29 bit identifier
  unsigned char  data[8];            // Data field
  unsigned char  len;                // Length of data field in bytes
  unsigned char  format;             // 0 - STANDARD, 1- EXTENDED IDENTIFIER
  unsigned char  type;               // 0 - DATA FRAME, 1 - REMOTE FRAME
} CAN_msg;

CAN_msg  CAN_TxMsg, CAN_RxMsg;

/*void CAN1_wrMsg (CAN_msg *msg)  {
            	CAN1->sTxMailBox[0].TIR  = (unsigned int)(msg->id << 3) | 4; 
              if (msg->type == DATA_FRAME)   // DATA FRAME
                              CAN1->sTxMailBox[0].TIR &= ~(1<<1);
                          else     // REMOTE FRAME
                   CAN1->sTxMailBox[0].TIR |= 1<<1;
            CAN1->sTxMailBox[0].TDLR = (((unsigned int)msg->data[3] << 24) | 
                             ((unsigned int)msg->data[2] << 16) |
                             ((unsigned int)msg->data[1] <<  8) | 
                             ((unsigned int)msg->data[0])        );
    	CAN1->sTxMailBox[0].TDHR = (((unsigned int)msg->data[7] << 24) | 
                             ((unsigned int)msg->data[6] << 16) |
                             ((unsigned int)msg->data[5] <<  8) |
                             ((unsigned int)msg->data[4])        );
              	CAN1->sTxMailBox[0].TDTR &= ~0xf; // Setup length
   	CAN1->sTxMailBox[0].TDTR |=  (msg->len & 0xf);
   	CAN1->sTxMailBox[0].TIR |=  1;                     // Transmit Message
}
void CAN1_Tx_mess( void)
{
  	char m[8] = "CAN Test";
  	int i; 
   	  	CAN_TxMsg.id = Id0;     // initialise message to send
  	for (i = 0; i < 8; i++) CAN_TxMsg.data[i] = m[i];   //Entering Data
  	CAN_TxMsg.len = 8;
  	CAN_TxMsg.format = EXTENDED_FORMAT;
  	CAN_TxMsg.type = DATA_FRAME;
  	CAN1_wrMsg (&CAN_TxMsg);   // transmit message
}
*/




//Initialise Filters
void CAN_wrFilter (unsigned int id, unsigned char format, unsigned char mess_type)  {
  static unsigned short CAN_filterIdx = 0;
         unsigned int   CAN_msgId     = 0;
  
  if (CAN_filterIdx > 13) {                       // check if Filter Memory is full
    return;
  }
                                                  // Setup identifier information
  if (format == STANDARD_FORMAT)  {               // Standard ID
      CAN_msgId  |= (unsigned int)(id << 21) | 0; //CAN_ID_STD;
  }  else  {                                      // Extended ID
      CAN_msgId  |= (unsigned int)(id <<  3) | 4; //CAN_ID_EXT;
  }
  if (mess_type == REMOTE_FRAME)	CAN_msgId  |= 2;

  CAN1->FA1R &=  ~(unsigned int)(1 << CAN_filterIdx); // deactivate filter

                                                  // initialize filter   
  CAN1->FS1R |= (unsigned int)(1 << CAN_filterIdx);// set 32-bit scale configuration
  CAN1->FM1R |= (unsigned int)(1 << CAN_filterIdx);// set 2 32-bit identifier list mode

  CAN1->sFilterRegister[CAN_filterIdx].FR1 = CAN_msgId; //  32-bit identifier
  CAN1->sFilterRegister[CAN_filterIdx].FR2 = CAN_msgId; //  32-bit identifier
    													   
  CAN1->FFA1R &= ~(unsigned int)(1 << CAN_filterIdx);  // assign filter to FIFO 0
  CAN1->FA1R  |=  (unsigned int)(1 << CAN_filterIdx);  // activate filter
  
    
  CAN_filterIdx += 1;                             // increase filter index
}
void init_filters(void) {
  CAN1->FMR = 1;                                   //init mode for filters
  CAN_wrFilter (Id0, EXTENDED_FORMAT,DATA_FRAME);      // Enable reception of messages 
  //other filters if required
  CAN1->FMR &= ~1;                         // reset Initialisation mode for filter banks
 // CAN_start ();                                   // leave init mode 
 // CAN_waitReady ();                               // wait til mbx is empty 
}


/*----------------------------------------------------------------------------
CAN1 Receive code
 *----------------------------------------------------------------------------*/

void CAN_rdMsg ( CAN_msg *msg)  
{
	int data[8], sum;
	int i = 0;
  if ((CAN1->sFIFOMailBox[0].RIR & 0x4)==0) { //CAN_ID_EXT) == 0) { // Standard ID
    msg->format = STANDARD_FORMAT;
    msg->id     = (unsigned int) 0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
  }  else  {                                          // Extended ID
    msg->format = EXTENDED_FORMAT;
    msg->id     = (unsigned int) 0x1FFFFFFF & ((CAN1->sFIFOMailBox[0].RIR >> 3));
  }
                                                  // Read type information
  if ((CAN1->sFIFOMailBox[0].RIR & 0x2) ==0) 
	{ //CAN_RTR_REMOTE) == 0) {
    msg->type =   DATA_FRAME;                     // DATA   FRAME
  }  else  {
    msg->type = REMOTE_FRAME;                     // REMOTE FRAME
  }
                                                  // Read length (number of received bytes)
  msg->len = (unsigned char)0x0000000F & CAN1->sFIFOMailBox[0].RDTR;
                                                  // Read data bytes
  msg->data[0] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
  msg->data[1] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  msg->data[2] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  msg->data[3] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 24);

  msg->data[4] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
  msg->data[5] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  msg->data[6] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  msg->data[7] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 24);
	
	for (i = 0; i < 8; i++) data[i] = msg->data[i];   //Entering Data
	sum = data[8];
	if(sum == 10)
		test();
		
}

//CAN1 receiver interrupt
void CAN1_RX0_IRQHandler(void) {
	//GPIOE->BSRR = 1<<10;   //LED ON for observation on MSO  
	if (CAN1->RF0R & 3) {			      // message pending ?
	CAN_rdMsg (&CAN_RxMsg);                       // read the message
	CAN1->RF0R |= 0x20;                    // Release FIFO 0 output mailbox 
	//   CAN_RxRdy = 1;                                // set receive flag
	//GPIOE->BSRR = 1<<(10+16);  // LED OFF
  }
}

//TALKER = 0
int main( )
{
	GPIO_Initialize();
	initCAN1( );
	init_filters();
	while (1) 
		{			 
		}			
}
