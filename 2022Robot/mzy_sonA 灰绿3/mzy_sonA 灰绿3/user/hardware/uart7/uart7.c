#include "sys.h"
#include "uart7.h"
#include "vision.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//ucos ʹ��	  
#endif

#if EN_UART7_RX   //���ʹ���˽���

vision_buffer_t vision_buffer;

//��ʼ��IO ����1 

//bound:������

int Uart7_Clean_IDLE_Flag = 0;  //������SR DR�Ĵ���

uint8_t  Vision_Buffer[2][ VISION_BUFFER_LEN ];  //����ϵͳ�������������ݴ�������

void uart7_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	//����ʹ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//ʹ��USART7ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//ʹ��DMAʱ��
	
	//����6��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //GPIOE7����ΪUSART7  RX
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //GPIOE8����ΪUSART7  TX
	
	//USART6�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOE7 RX     DMA1ͨ��5������3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //������
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��GPIOE7
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIOE8 TX   DMA1ͨ��5������7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //������
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��GPIOE8
	
	USART_DeInit(UART7);
	
	 //USART6 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART7, &USART_InitStructure); //��ʼ������7
	
	USART_ClearFlag(UART7, USART_FLAG_IDLE);  //���������·�жϱ�־λ
	USART_ITConfig(UART7, USART_IT_IDLE, ENABLE); //ʹ�ܴ��ڿ����ж�
	
	DMA_DeInit(DMA1_Stream3);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;                              //DMAͨ��5����8��ͨ����ѡ��һ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART7->DR);       //�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Vision_Buffer[0];             //�洢��0��ַ��˫����ģʽ��Ҫʹ��M1AR
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     //���赽�洢��ģʽ
  DMA_InitStructure.DMA_BufferSize = 100;                                     //���ݴ�������������������Ϊ��λ
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //�����ַ���ֲ���
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //�洢����ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //��������λ��8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //�洢������λ��8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //ѭ��ģʽ
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //�����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;                      //ʹ��FIFOģʽ
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //���δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //���δ���
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
  DMA_DoubleBufferModeConfig(DMA1_Stream3, (uint32_t)Vision_Buffer[1], DMA_Memory_0);  //˫����ģʽ��buffer0�ȱ�����
  DMA_DoubleBufferModeCmd(DMA1_Stream3, ENABLE);                              //ʹ��˫����ģʽ
  DMA_Cmd(DMA1_Stream3, DISABLE); //Add a disable
  DMA_Cmd(DMA1_Stream3, ENABLE);
	
	USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE);  //DMA���ڽ��������ж�ʹ��
	USART_DMACmd( UART7, USART_DMAReq_Tx, ENABLE );
	
	USART_Cmd(UART7, ENABLE);  //ʹ�ܴ���7 
		
	#if EN_UART7_RX	 //���ʹ���˽���
	//USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//����7�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//��ռ���ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif

	
}

void UART7_IRQHandler(void)                	//����7�жϷ������
{

 if (USART_GetITStatus(UART7, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(UART7);
    }
    else if (USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)
    {
//        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(UART7);

        if(DMA_GetCurrentMemoryTarget(DMA1_Stream3) == 0)    //���DMA���ڷ��ʴ洢��0 ����CPU���Է��ʴ洢��1
        {
            //��������DMA
            DMA_Cmd(DMA1_Stream3, DISABLE);
//            this_time_rx_len = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);  //��õ�ǰ�յ����ֽ�������ΪDMA�ж�ֻ�н��ճ�����ʱ�ᴥ���ж�
            DMA_SetCurrDataCounter(DMA1_Stream3, VISION_BUFFER_LEN);  //��������DMA�Ķ�ȡ����������
            DMA1_Stream3->CR |= DMA_SxCR_CT;  //���ڴ�1����Ϊ��ǰ�ڴ��ַ
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);
					
						Vision_Read_Data(Vision_Buffer[0]);		//��ȡ�Ӿ�����
					  
//					  Vision_Send_Data(0xB1);
//					
					  memset(Vision_Buffer[0], 0, 200);    //����   ����  ����
					
            DMA_Cmd(DMA1_Stream3, ENABLE);
					 

        }
        else
        {
            //��������DMA
            DMA_Cmd(DMA1_Stream3, DISABLE);
//            this_time_rx_len = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);
            DMA_SetCurrDataCounter(DMA1_Stream3, VISION_BUFFER_LEN);
            DMA1_Stream3->CR &= ~(DMA_SxCR_CT);  //���ڴ�0����Ϊ��ǰ�ڴ��ַ
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);
					
					  Vision_Read_Data(Vision_Buffer[1]);		//��ȡ�Ӿ�����
					
//					  Vision_Send_Data(0xB2);
//					
					  memset(Vision_Buffer[1], 0, 200);    //����   ����  ����
					
            DMA_Cmd(DMA1_Stream3, ENABLE);
					

        }
    }

} 

/**
  * @brief  ����һ�η���һ���ֽ�����
  * @param  ����
  * @retval void
  * @attention  8λ
  */
void Uart7_SendChar(uint8_t Data)
{
	while (USART_GetFlagStatus(UART7, USART_FLAG_TC) == RESET);
	
	USART_SendData(UART7, Data);   
}

/*****************  ����һ���ַ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}


#endif	
