#include "sys.h"
#include "usart6.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//ucos ʹ��	  
#endif

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//__use_no_semihosting was requested, but _ttywrch was referenced, �������º���, ����2
int _ttywrch(int ch)
{
ch = ch;
		return 0;
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif

#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	


int Usart6_Clean_IDLE_Flag = 0;  //������SR DR�Ĵ���

uint8_t  Judge_Buf[2][ JUDGE_BUF_LEN ];  //����ϵͳ�������������ݴ�������

//��ʼ��IO ����1 
//bound:������
void usart6_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	//����ʹ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOGʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��
	
	//����6��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //GPIOG9����ΪUSART6
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //GPIOG14����ΪUSART6
	
	//USART6�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9��GPIOG14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOG,&GPIO_InitStructure); //��ʼ��PG9��PG14
	
	 //USART6 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART6, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���6 
	
	#if EN_USART1_RX	 //���ʹ���˽���
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif


  DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;                              //DMAͨ��5����8��ͨ����ѡ��һ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);       //�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Judge_Buf[0];             //�洢��0��ַ��˫����ģʽ��Ҫʹ��M1AR
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     //���赽�洢��ģʽ
  DMA_InitStructure.DMA_BufferSize = 100;                                     //���ݴ�������������������Ϊ��λ
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //�����ַ���ֲ���
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //�洢����ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //��������λ��8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //�洢������λ��8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //ѭ��ģʽ
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //�����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                      //��ʹ��FIFOģʽ
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //���δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //���δ���
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)Judge_Buf[1], DMA_Memory_0);  //˫����ģʽ��buffer0�ȱ�����
  DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);                              //ʹ��˫����ģʽ
  DMA_Cmd(DMA2_Stream1, DISABLE); //Add a disable
  DMA_Cmd(DMA2_Stream1, ENABLE);
	
}
void USART6_IRQHandler(void)                	//����6�жϷ������
{

 if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART6);
    }
    else if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
//        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART6);
		
        if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)    //���DMA���ڷ��ʴ洢��0 ����CPU���Է��ʴ洢��1
        {
            //��������DMA
            DMA_Cmd(DMA2_Stream1, DISABLE);
//            this_time_rx_len = JUDGE_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);  //��õ�ǰ�յ����ֽ�������ΪDMA�ж�ֻ�н��ճ�����ʱ�ᴥ���ж�
            DMA_SetCurrDataCounter(DMA2_Stream1, JUDGE_BUF_LEN);  //��������DMA�Ķ�ȡ����������
            DMA2_Stream1->CR |= DMA_SxCR_CT;  //���ڴ�1����Ϊ��ǰ�ڴ��ַ
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
					
            DMA_Cmd(DMA2_Stream1, ENABLE);
					 

        }
        else
        {
            //��������DMA
            DMA_Cmd(DMA2_Stream1, DISABLE);
//            this_time_rx_len = JUDGE_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
            DMA_SetCurrDataCounter(DMA2_Stream1, JUDGE_BUF_LEN);
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);  //���ڴ�0����Ϊ��ǰ�ڴ��ַ
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
										
            DMA_Cmd(DMA2_Stream1, ENABLE);
					

        }
    }
} 
#endif	

 



