#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_usart.h>
#include <system_stm32f30x.h>
#include <stm32f30x.h>
#include <stm32f30x_misc.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/stat.h>
#include <ADCF3.h>

#define USARTBUFFSIZE 1025

typedef struct {
	uint16_t in;
	uint16_t out;
	uint16_t count;
	uint8_t buff[USARTBUFFSIZE];
} FIFO_Tra_TypeDef;

typedef struct {
	uint16_t in;
	uint16_t out;
	uint16_t count;
	uint8_t buff[USARTBUFFSIZE];
} FIFO_Rec_TypeDef;

//initialize buffers
volatile FIFO_Tra_TypeDef U1TX;
volatile FIFO_Tra_TypeDef U2TX;
volatile FIFO_Tra_TypeDef U3TX;
volatile FIFO_Rec_TypeDef U1RX;
volatile FIFO_Rec_TypeDef U2RX;
volatile FIFO_Rec_TypeDef U3RX;

void initBuffers(void);
void putChar1(int c);
void putChar2(int c);
void putChar3(int c);
void printStr1(char *s);
void printStr2(char *s);
void printStr3(char *s);
void pushch1(uint8_t rx);
void pushch2(uint8_t rx);
void pushch3(uint8_t rx);
bool isAvailable1(char *buffer);
bool isAvailable2(char *buffer);
bool isAvailable3(char *buffer);
void Usart1PutStr(char *s);
void Usart2PutStr(char *s);
void Usart3PutStr(char *s);
void Usart1Put(uint8_t ch);
void Usart2Put(uint8_t ch);
void Usart3Put(uint8_t ch);

void BufferTra_Init(__IO FIFO_Tra_TypeDef *buffer);
void BufferRec_Init(__IO FIFO_Rec_TypeDef *buffer);
ErrorStatus BufferRec_Put(__IO FIFO_Rec_TypeDef *buffer, uint8_t ch);
ErrorStatus BufferRec_Get(__IO FIFO_Rec_TypeDef *buffer, uint8_t *ch);
ErrorStatus BufferTra_Put(__IO FIFO_Tra_TypeDef *buffer, uint8_t ch);
ErrorStatus BufferTra_Get(__IO FIFO_Tra_TypeDef *buffer, uint8_t *ch);
ErrorStatus BufferRec_IsEmpty(__IO FIFO_Rec_TypeDef buffer);
ErrorStatus BufferTra_IsEmpty(__IO FIFO_Tra_TypeDef buffer);

//extern "C" int fputc(int ch, FILE *f)
//{
	//while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	//USART_SendData(USART3, ch);
//}

extern "C" int _write(int fd, char *ptr, int len)
{
   /* Write "len" of char from "ptr" to file id "fd"
    * Return number of char written.
    * Need implementing with UART here. */
	for (int i = 0; i < len; i++)
	{
		putChar2(*ptr);
		ptr++;
	}
	return len;
}

extern "C" void _ttywrch(int ch) {
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
			;  // Wait for Empty
		USART_SendData(USART2, ch);
	}
}

extern "C" int _fstat(int fd, struct stat *pStat)
{
	pStat->st_mode = S_IFCHR;
	return 0;
}

extern "C" int _close(int)
{
	return -1;
}

extern "C" int _isatty(int fd)
{
	return 1;
}

extern "C" int _lseek(int, int, int)
{
	return -1;
}

extern "C" int _read(int fd, char *pBuffer, int size)
{
	for (int i = 0; i < size; i++)
	{
		if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		{
			pBuffer[i] = USART_ReceiveData(USART1);
		}
	}
	return size;
}

extern "C" caddr_t _sbrk(int increment)
{
	extern char end asm("end");
	register char *pStack asm("sp");

	static char *s_pHeapEnd;

	if (!s_pHeapEnd)
		s_pHeapEnd = &end;

	if (s_pHeapEnd + increment > pStack)
		return (caddr_t) - 1;

	char *pOldHeapEnd = s_pHeapEnd;
	s_pHeapEnd += increment;
	return (caddr_t)pOldHeapEnd;
}

void Delay()
{
	int i;
	for (i = 0; i < 1000000; i++)
		asm("nop");
}

// STM32 USART2 (USART2 TX PA.02, RX PA.03) STM32F10x

/**************************************************************************************/

void NM_RCC_Configuration(void)
{
	/* Enable GPIO, AFIO clock */
//	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

}

/**************************************************************************************/

void NM_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable UART clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);	// USART1
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);	// USART2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);	// USART3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);

	/* Configure USART Tx as alternate function push-pull */
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  // PA.9 USART1.TX
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // PA.10 USART1.RX
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  // PA.02 USART2.TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  // PA.03 USART2.RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // PC.10 USART3.TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  // PC.11 USART3.RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

}

/**************************************************************************************/

void NM_USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	/* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
	/* USART configured as follow:
	- BaudRate = 9600 baud
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);
	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);
	/* Enable transmit and receive interrupts for the USART1. */
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	/* Enable RXNE interrupt */
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	/* Enable USART2 global interrupt */
	NVIC_EnableIRQ(USART1_IRQn);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);
	/* Enable the USART2 */
	USART_Cmd(USART2, ENABLE);
	/* Enable transmit and receive interrupts for the USART2. */
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	/* Enable RXNE interrupt */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	/* Enable USART2 global interrupt */
	NVIC_EnableIRQ(USART2_IRQn);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART3, &USART_InitStructure);
	/* Enable the USART2 */
	USART_Cmd(USART3, ENABLE);
	/* Enable transmit and receive interrupts for the USART2. */
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	/* Enable RXNE interrupt */
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	/* Enable USART2 global interrupt */
	NVIC_EnableIRQ(USART3_IRQn);
}

void NM_NVIC_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************************/

int main(void)
{
	char data[] = "Hello World";
	char test[] = "Hello World, The quick brown fox jumps over the lazy dog.";
	SystemInit();
//	SystemCoreClockUpdate();
//	SysTick_Config(SystemCoreClock / 1000); //Set up a systick interrupt 
	initBuffers();
	NM_RCC_Configuration();
	NM_NVIC_init();
	NM_GPIO_Configuration();
	NM_USART_Configuration();
	printStr2(data);
	Usart2PutStr(test);
	ADC1_config();
	uint16_t temp = readTemp();
	uint16_t vBat = readVBat();
	uint16_t vRef = readintRef();
	uint16_t vInput = readVinput();
	printf("temp %03d\r\n", temp);
	printf("vBat %03d\r\n", vBat);
	printf("vRef %03d\r\n", vRef);
	printf("vInput %03d\r\n", vInput);

	while (1)
	{
		char str[] = "Welcome to wherever you are\r\n";
		char *s = str;
		char data;
		char getdata[256];

		if (isAvailable2(&getdata[0]) > 0)
		{
			strcat(getdata, "\r\n");
			printStr2(getdata);
		}
	}
	while (1); // Don't want to exit
}

void putChar1(int c)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;  // Wait for Empty
	USART_SendData(USART1, c); 
}

void putChar2(int c)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);  // Wait for Empty
	USART_SendData(USART2, c); 
}

void putChar3(int c)
{
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);  // Wait for Empty
	USART_SendData(USART3, c); 
}

void printStr1(char *s)
{
	int i;
	i = 0;
	while ((*s) && (i < 255))
	{
		putChar1((u16)*s++);
		i++;
	}
}

void printStr2(char *s)
{
	int i;
	i = 0;
	while ((*s) && (i < 255))
	{
		putChar2((u16)*s++);
		i++;
	}
}

void printStr3(char *s)
{
	int i;
	i = 0;
	while ((*s) && (i < 255))
	{
		putChar3((u16)*s++);
		i++;
	}
}

/**********************************************************
* USART2 interrupt request handler: on reception of a
* character 't', toggle LED and transmit a character 'T'
*********************************************************/
extern "C" void USART1_IRQHandler(void)
{
	/* RXNE handler */
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* RXNE handler */
		uint8_t rx =  USART_ReceiveData(USART1);
		pushch1(rx);
	}
//	else if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
}

extern "C" void USART2_IRQHandler(void)
{
	uint8_t ch;
	/* RXNE handler */
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		ch =  USART_ReceiveData(USART2);
		pushch2(ch);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
		if (BufferTra_Get(&U2TX, &ch) == SUCCESS)//if buffer read
		{
			USART_SendData(USART2, ch);
		}
		else//if buffer empty
		{
			//disable Transmit Data Register empty interrupt
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			USART_ClearITPendingBit(USART2, USART_IT_TXE);
		}
	}
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{
		ch = USART_ReceiveData(USART2);
		USART_ClearITPendingBit(USART2, USART_IT_ORE);
	}
}

extern "C" void USART3_IRQHandler(void)
{
	uint8_t ch;
	/* RXNE handler */
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		ch =  USART_ReceiveData(USART3);
		pushch3(ch);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
	if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
		if (BufferTra_Get(&U3TX, &ch) == SUCCESS)//if buffer read
		{
			USART_SendData(USART3, ch);
		}
		else//if buffer empty
		{
		   //disable Transmit Data Register empty interrupt
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
			USART_ClearITPendingBit(USART2, USART_IT_TXE);
		}
	}
	if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
	{
		ch = USART_ReceiveData(USART3);
		USART_ClearITPendingBit(USART3, USART_IT_ORE);
	}
}

#define LINEMAX 255 // Maximal allowed/expected line length
 
static char line_buffer1[LINEMAX + 1]; // Holding buffer with space for terminating NUL
static char line_buffer2[LINEMAX + 1]; // Holding buffer with space for terminating NUL
static char line_buffer3[LINEMAX + 1]; // Holding buffer with space for terminating NUL
volatile int line_valid1 = 0;
volatile int line_valid2 = 0;
volatile int line_valid3 = 0;
static char rx_buffer1[LINEMAX];   // Local holding buffer to build line
static char rx_buffer2[LINEMAX];   // Local holding buffer to build line
static char rx_buffer3[LINEMAX];   // Local holding buffer to build line
static int rx_index1 = 0;
static int rx_index2 = 0;
static int rx_index3 = 0;

void pushch1(uint8_t rx)
{
	if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
	{
		if (rx_index1 != 0) // Line has some content?
		{
			memcpy((void *)line_buffer1, rx_buffer1, rx_index1); // Copy to static line buffer from dynamic receive buffer
			line_buffer1[rx_index1] = 0; // Add terminating NUL
			line_valid1++; // flag new line valid for processing
			rx_index1 = 0; // Reset content pointer
			memset(rx_buffer1, 0, sizeof(rx_buffer1)); // Clear buffer
		}
	}
	else
	{
		if (rx_index1 == LINEMAX) // If overflows pull back to start
			rx_index1 = 0;
		rx_buffer1[rx_index1++] = rx; // Copy to buffer and increment
	}
	return;
}

void pushch2(uint8_t rx)
{
	if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
	{
		if (rx_index2 != 0) // Line has some content?
		{
			memcpy((void *)line_buffer2, rx_buffer2, rx_index2); // Copy to static line buffer from dynamic receive buffer
			line_buffer2[rx_index2] = 0; // Add terminating NUL
			line_valid2++; // flag new line valid for processing
			rx_index2 = 0; // Reset content pointer
			memset(rx_buffer2, 0, sizeof(rx_buffer2)); // Clear buffer
		}
	}
	else
	{
		if (rx_index2 == LINEMAX) // If overflows pull back to start
			rx_index2 = 0;
		rx_buffer2[rx_index2++] = rx; // Copy to buffer and increment
	}
	return;
}

void pushch3(uint8_t rx)
{
	if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
	{
		if (rx_index3 != 0) // Line has some content?
		{
			BufferRec_Put(&U3RX, rx);
			memcpy((void *)line_buffer3, rx_buffer3, rx_index3); // Copy to static line buffer from dynamic receive buffer
			line_buffer3[rx_index3] = 0; // Add terminating NUL
			line_valid3++; // flag new line valid for processing
			rx_index3 = 0; // Reset content pointer
			memset(rx_buffer3, 0, sizeof(rx_buffer3)); // Clear buffer
		}
	}
	else
	{
		if (rx_index3 == LINEMAX) // If overflows pull back to start
			rx_index3 = 0;
		rx_buffer3[rx_index3++] = rx; // Copy to buffer and increment
	}
	return;
}

bool isAvailable1(char buffer[])
{
	static int size;
	if (line_valid1 > 0)
	{
		size = strlen(line_buffer1);
		memcpy((void *)buffer, line_buffer1, size); // Copy to static line buffer from dynamic receive buffer
		buffer[size] = 0;
		line_valid1--;
		return true;
	}
	else
		return false;
}

bool isAvailable2(char buffer[])
{
	static int size;
	if (line_valid2 > 0)
	{
		size = strlen(line_buffer2);
		memcpy((void *)buffer, line_buffer2, size); // Copy to static line buffer from dynamic receive buffer
		buffer[size] = 0;
		line_valid2--;
		return true;
	}
	else
		return false;
}

bool isAvailable3(char buffer[])
{
	static int size;
	if (line_valid3 > 0)
	{
		size = strlen(line_buffer3);
		memcpy((void *)buffer, line_buffer3, size); // Copy to static line buffer from dynamic receive buffer
		buffer[size] = 0;
		line_valid3--;
		return true;
	}
	else
		return false;
}


void BufferRec_Init(__IO FIFO_Tra_TypeDef *buffer)
{
	buffer->count = 0;//0 bytes in buffer
	buffer->in = 0;//index points to start
	buffer->out = 0;//index points to start
}

void BufferTra_Init(__IO FIFO_Rec_TypeDef *buffer)
{
	buffer->count = 0;//0 bytes in buffer
	buffer->in = 0;//index points to start
	buffer->out = 0;//index points to start
}

void initBuffers()
{
	BufferRec_Init(&U1TX);
	BufferTra_Init(&U1RX);
	BufferRec_Init(&U2TX);
	BufferTra_Init(&U2RX);
	BufferRec_Init(&U3TX);
	BufferTra_Init(&U3RX);
}

ErrorStatus BufferRec_Get(__IO FIFO_Rec_TypeDef *buffer, uint8_t *ch)
{
	if (buffer->count == 0)
		return ERROR;//buffer empty
	*ch = buffer->buff[buffer->out++];
	buffer->count--;
	if (buffer->out == USARTBUFFSIZE)
		buffer->out = 0;//start from beginning
	return SUCCESS;
}

ErrorStatus BufferRec_Put(__IO FIFO_Rec_TypeDef *buffer, uint8_t ch)
{
	if (buffer->count == USARTBUFFSIZE)
		return ERROR;//buffer full
	buffer->buff[buffer->in++] = ch;
	buffer->count++;
	if (buffer->in == USARTBUFFSIZE)
		buffer->in = 0;//start from beginning
	return SUCCESS;
}

ErrorStatus BufferTra_Get(__IO FIFO_Tra_TypeDef *buffer, uint8_t *ch)
{
	if (buffer->count == 0)
		return ERROR;//buffer empty
	*ch = buffer->buff[buffer->out++];
	buffer->count--;
	if (buffer->out == USARTBUFFSIZE)
		buffer->out = 0;//start from beginning
	return SUCCESS;
}

ErrorStatus BufferTra_Put(__IO FIFO_Tra_TypeDef *buffer, uint8_t ch)
{
	if (buffer->count == USARTBUFFSIZE)
		return ERROR;//buffer full
	buffer->buff[buffer->in++] = ch;
	buffer->count++;
	if (buffer->in == USARTBUFFSIZE)
		buffer->in = 0;//start from beginning
	return SUCCESS;
}
void pushch4(uint8_t rx)
{
	if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
	{
		if (rx_index3 != 0) // Line has some content?
		{
			memcpy((void *)line_buffer3, rx_buffer3, rx_index3); // Copy to static line buffer from dynamic receive buffer
			line_buffer3[rx_index3] = 0; // Add terminating NUL
			line_valid3++; // flag new line valid for processing
			rx_index3 = 0; // Reset content pointer
			memset(rx_buffer3, 0, sizeof(rx_buffer3)); // Clear buffer
		}
	}
	else
	{
		if (rx_index3 == LINEMAX) // If overflows pull back to start
			rx_index3 = 0;
		rx_buffer3[rx_index3++] = rx; // Copy to buffer and increment
	}
	return;
}

ErrorStatus Buffer_IsEmpty(__IO FIFO_Rec_TypeDef buffer)
{
	if (buffer.count == 0)
		return SUCCESS;//buffer full
	return ERROR;
}

void Usart1Put(uint8_t ch)
{
    //put char to the buffer
	BufferTra_Put(&U1TX, ch);
	//enable Transmit Data Register empty interrupt
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void Usart2Put(uint8_t ch)
{
    //put char to the buffer
	BufferTra_Put(&U2TX, ch);
	//enable Transmit Data Register empty interrupt
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

void Usart3Put(uint8_t ch)
{
    //put char to the buffer
	BufferTra_Put(&U3TX, ch);
	//enable Transmit Data Register empty interrupt
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

void Usart2PutStr(char *s)
{
	int i;
	i = 0;
	while ((*s) && (i < 255))
	{
		Usart2Put((u16)*s++);
		i++;
	}
}

volatile uint32_t Milliseconds = 0;
volatile uint32_t Seconds = 0;

extern "C" void SysTick_Handler(void)
{
	Milliseconds++; //Increment millisecond variable
	if (Milliseconds % 1000 == 999) { //If 1000 milliseconds have passed, increment seconds
		Seconds++;
	}
}

uint32_t millis(void)
{
	return Milliseconds;
}

//Delay function for millisecond delay
void DelayMil(uint32_t MilS) {
	volatile uint32_t MSStart = Milliseconds;
	while ((Milliseconds - MSStart) < MilS) asm volatile("nop");
}

//Delay function for second delay
void DelaySec(uint32_t S) {
	volatile uint32_t Ss = Seconds;
	while ((Seconds - Ss) < S) asm volatile("nop");
}

/**************************************************************************************/

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**************************************************************************************/
