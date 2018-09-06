/*    ��Ӱ���оƬ��ͷ�ļ�    */
#include <iostm8s103f3.h>
#include <string.h>
#include <stdio.h>
#include "B_LUX_V20.h"

#define uint unsigned int
#define uchar unsigned char

#define DATA_SET PA_ODR_bit.ODR2 = 1
#define DATA_CLR PA_ODR_bit.ODR2 = 0
#define DATA_GET PA_IDR_bit.IDR2

uchar datatime = 0;          //��¼��ƽ���
uchar datareg = 0;           //�洢��λ����
uchar datanum = 0;           //��ȡ��ǰ���������ɸѡ��һ�ֽ�����
uchar data_dh11[3] = {0x00}; //�������е�����
uchar outline;               //�ȴ���ʱ����
uchar DHTFLAG = 0;           //�ж����߶�ȡ״̬�ı�־λ

void UART1_SendData(uchar data)
{
    while (!(UART1_SR & 0X80))
        ;            //�жϷ������ݼĴ����Ƿ�Ϊ��
    UART1_DR = data; //���ͼĴ���д������
}

int fputc(int ch, FILE *f)
{
    UART1_SendData(ch);
    return ch;
}

/*
	PD5��Ӧtx��� UART1���ݷ���
	PD6��Ӧrx���� UART1���ݽ���
	���Ǻ���Ҫ����
*/
void UART1_Init(unsigned long baudrate)
{
    //uint baudrate
    uint baud;
    baud = 16000000 / baudrate;                                              //�趨������Ӧ�������봮��ʱ�ӵķ�Ƶ��
    UART1_BRR2 = ((uchar)((baud & 0xf000) >> 8)) | ((uchar)(baud & 0x000f)); //��д���ر��ʵĸ�4λ���4λ
    UART1_BRR1 = ((uchar)((baud & 0x0ff0) >> 4));                            //��д���ر��ʵ��м�8λ
    UART1_CR1_bit.UART0 = 0;                                                 //ʹ��UART0
    UART1_CR2_bit.RIEN = 1;                                                  //ʹ���жϽ���
    UART1_CR2_bit.TEN = 1;                                                   //ʹ�ܷ���
    UART1_CR2_bit.REN = 1;                                                   //����ʹ��

    PD_DDR_bit.DDR5 = 1; //���ģʽ
    PD_CR1_bit.C15 = 1;  //

    PD_DDR_bit.DDR6 = 0; //����ģʽ
    PD_CR1_bit.C16 = 0;  //
}

void Lora_SendData(char *data)
{

    int len = strlen(data);
    if (len > 247)
        return; //overflow

    uint crc = 0x3C + 0x02 + len;

    uint i;
    for (i = 0; i < len; i++)
    {
        crc += data[i];
    }
    crc &= 0xFF; // ȡ�Ͱ�λ

    UART1_SendData(0x3C); //��ʼ��־
    UART1_SendData(0x02); //type����������
    UART1_SendData(len);
    for (i = 0; i < len; i++)
    {
        UART1_SendData(data[i]);
    }
    UART1_SendData((uchar)crc); //�ֽ�У���
    UART1_SendData(0x0D);       //������־
}

//�ο�18B20���������������IOģʽ
void DATA_OUT()
{
    PA_DDR_bit.DDR2 = 1; //һ��ʼ�����ģʽ
    PA_CR1_bit.C12 = 1;  //ʹ���������ģʽ
    PA_CR2_bit.C22 = 0;
}

//����ģʽ
void DATA_IN() //�ж���������ģʽ
{
    PA_DDR_bit.DDR2 = 0;
    PA_CR1_bit.C12 = 1;
    PA_CR2_bit.C22 = 1;
}
//�ⲿ�жϵ�����
void EXTI_PA()
{
    EXTI_CR1 = 0X02; //PD�ڶ����ⲿ�ж��½��ش���
}

//��ʱ���趨 ʹ��ͨ�ö�ʱ��2���о�ȷ��΢����ʱ
void TIM2_Config()
{
    TIM2_PSCR = 0x00; //Ԥ��Ƶ���趨���üĴ�������λ�Ĵ�����
    TIM2_ARRH = 0x00; //16000 ��ʾΪ 0x0010 ����˵�� 16000000��Ƶ���£�1΢��ͻ����
    TIM2_ARRL = 0x10;

    TIM2_CNTRH = 0x00; //����������
    TIM2_CNTRL = 0x00;
    TIM2_SR1 = 0x00; //������б�־λ
}

//���������趨����ʱ����  ����������˲�ѯ�ȴ���ʽ���о�ȷ��ʱ ����Ҳ��ȷ����ȫռ����CPU��ʱ�䣬Ҳ����һ�ַǳ��ֱ�����ʱ��ʽ
void TIM2_Delayus(uint xus) //��Χ 0~65535����
{
    TIM2_CR1 = 0x81; //��ARPE�Զ�Ԥװ��ʹ�ܣ�ʹ�ܼ�����
    while (xus--)
    {
        while (!(TIM2_SR1 & 0X01))
            ;                //�ȴ���������ARRH/L����ƥ����� Ҳ����һ����ʱ�䵽��
        TIM2_SR1 &= ~(0X01); //�����־λ��Ψһ���㷽ʽ��
    }
    TIM2_CR1 = 0x00; //�رն�ʱ��
}

//��ʱ���趨 ʹ�û�����ʱ��4���е�ƽ��Ȳɼ�
void TIM4_Config()
{
    TIM4_PSCR = 0x04; //Ԥ��Ƶ���趨���üĴ�������λ�Ĵ�����16��Ƶ�� һ������һ��΢��
    TIM4_CNTR = 0x00; //����������
    TIM4_SR = 0x00;   //������б�־λ
    TIM4_CR1 = 0x00;  //���ڹر�״̬
}

//��λDHT11
void DHT11_RST()
{
    TIM4_CR1 = 0x00;     //�رն�ʱ��
    TIM4_CNTR = 0;       //��֤�´εĵ�һ������λ��׼ȷ
    DATA_SET;            //ODR����Ϊ1
    DATA_OUT();          //�������ģʽ����ʱ����ߵ�ƽ
    DATA_CLR;            //��ʱ�����������ģʽ����������
    TIM2_Delayus(18000); //����20����
    DATA_SET;            //�ͷ�����
    TIM2_Delayus(20);    //�ͷ������Ժ�ȴ�40΢��DHT�ᷢ����Ӧ�ź�
}

//���DHT11�Ƿ���Ӧ
uchar DHT11_CHECK()
{

    if (!DATA_GET) //���˳�����ͣ���˵��������Ӧ
    {
        while ((!DATA_GET) && (outline < 100)) //���ǵ͵�ƽ
        {
            TIM2_Delayus(1);
        }
        if (outline > 90) //��ʼ�źų�ʱ�˳�
            return 0;
        outline = 0;
        while ((DATA_GET) && (outline < 100)) //�����Ǹߵ�ƽ
        {
            TIM2_Delayus(1);
        }
        if (outline < 90)
            TIM4_CR1 = 0x81; //���̴򿪶�ʱ����ʼ��ʱ��һ������λ
        else
            return 0;
        DATA_IN(); //��������Ϊ�ⲿ�ж�ģʽ
        outline = 0;

        return 1; //һ�гɹ�����1
    }
    else
        return 0;
}

void UART1_DHT_Display()
{
    if (DHTFLAG) //������ʼ�������������
    {
        if (data_dh11[0] + data_dh11[1] == data_dh11[2])
        {
			printf("DH11_W:");
            UART1_SendData(data_dh11[0] / 100 + 48); //ʪ��
			//printf("�¶�%d %d | �¶� %d %d \n", data_da11[0] /100 + 48);
            UART1_SendData((data_dh11[0] / 10) % 10 + 48);
            UART1_SendData(data_dh11[0] % 10 + 48);
			printf("\nDH11_T:");
            UART1_SendData(data_dh11[1] / 100 + 48); //�¶�
            UART1_SendData((data_dh11[1] / 10) % 10 + 48);
            UART1_SendData(data_dh11[1] % 10 + 48);
            UART1_SendData('\n');
        }
        else
            printf("\r\n����У��ʧ��\r\n");
    }
    else
        printf("\r\n������ȡʧ��\r\n");
}

void delay(unsigned int ms)
{
  unsigned int x , y;
  for(x = ms; x > 0; x--)           /*  ͨ��һ������ѭ��������ʱ*/
    for(y = 3000 ; y > 0 ; y--);
}

void ADC_Init()
{
  PC_DDR_bit.DDR4 = 0;   //����PC->4 Ϊ����
  PC_CR1_bit.C14 = 0;    //����Ϊ��������
  PC_CR2_bit.C24 = 0;    //�����жϽ�ֹ

  ADC_CR1_bit.SPSEL = 3;    //fmaster / 18 = 16MHZ / 18 = 888888HZ ��Ƶ
  ADC_CR2_bit.ALIGN = 1;    //RIGHT ALIGN �Ҷ���
  ADC_CSR_bit.CH = 2;       //SELECT AIN2 ѡ��ת��ͨ��2

  ADC_CR1_bit.ADON = 1;     //����ADC
}

void ADC_Data_Read(unsigned int *AD_Value)
{
  ADC_CR1_bit.ADON = 1;       //����ADC

  while(ADC_CSR_bit.EOC == 0);  //�ȴ�ת������
  *AD_Value = ADC_DRH;          //�ȶ�ȡ��8λ
  *AD_Value = (unsigned int)((*AD_Value << 8) + ADC_DRL);   //��8λ���8λ��ӣ��ճ�16λ����
}

void main()
{
    asm("sim");        //�ر�ϵͳ���ж�
    CLK_CKDIVR = 0x00; //CPUDIV = 1 HSIDIV = 1  �ڲ�ʱ�� = 16Mhz
    DATA_SET;
    DATA_OUT();         //����Ϊ�������״̬
    UART1_Init(115200); //���ô��ڳ�ʼ�������������ò�����Ϊ115200 bps
    TIM2_Config();      //�������ھ�ȷ��ʱ�Ķ�ʱ��2
    TIM4_Config();      //�������ڿ�Ȳ�׽�Ķ�ʱ��4
	ADC_Init();
	B_LUX_Init();
    EXTI_PA();          //�����ж�û�д򿪵�ʱ����Դ��ⲿ�ж���
    asm("rim");         //��ϵͳ���ж�
    while (1)           //������ѭ�����ȴ����ڽ����ж�
    {
        //Lora_SendData("0213test\r\n");
		for (int i = 0; i < 20; i++)
        TIM2_Delayus(50000);    //�����ʱһ��
		
        DHT11_RST();             //��λ
        DHTFLAG = DHT11_CHECK(); //��ȡ����ֵ �Ƿ�ɹ�  1�ɹ� 0 ʧ��
		
		for (int i = 0; i < 11; i++)
        	TIM2_Delayus(50000);    //�����ʱһ��
		
		//���¶�
        UART1_DHT_Display();     //���ô�����ʾ��ʪ��
		
		//����
		unsigned int ADCDATA;  //����һ��16λ����
		ADC_Data_Read(&ADCDATA);//�Ѳ���������ͨ�����ڷ��͵�PC�崮�����������ʾ
    	printf("YL69:%d\n" , ADCDATA);
		
		//����
		unsigned long result = 0;
		B_LUX_GetLux(&result);
   		printf("BH1750:%ld\n",result);
		
        for (int i = 0; i < 11; i++)
        	TIM2_Delayus(50000);    //�����ʱһ��*/
    }
}

#pragma vector = 0x05 //PA���ж�����λ
__interrupt void GPIOA_IRQHandler()
{
    datatime = TIM4_CNTR;                   //��ȡ�����½���֮������ݿ��
    TIM4_CNTR = 0;                          //���㣬�ٴλ�ȡ��һλ
    datareg <<= 1;                          //��λ�ȳ������Ʋ���
    if ((datatime > 75) && (datatime < 85)) //����0  �Ҿ�Ĭ�ϸ�λ��ʼ��ȡ��
        datareg &= 0xfe;
    if ((datatime > 120) && (datatime < 130)) //����1
        datareg |= 0x01;
    if (datanum == 7)
        data_dh11[0] = datareg; //��ȡ��һ���ֽ�Ҳ����ʪ������λ
    if (datanum == 23)          //��ȡ�������ֽ�Ҳ�����¶�����λ
        data_dh11[1] = datareg;
    if (datanum == 39) //��ȡ������ֽ�Ҳ����У�飨�¶�+ʪ�ȣ�λ
        data_dh11[2] = datareg;
    datanum++;         //ÿ�ζ�ȡһλ��1
    if (datanum >= 40) //���ݽ������˽���
        datanum = 0;
}

/*    ���ڽ��������жϷ�����    */
/*
#pragma vector = 0x14              //���ô��ڽ����ж������� = 0X14 = 20
__interrupt void UART1_RX_RXNE(void)
{
  	unsigned char ch1;

  	UART1_SR_RXNE = 1;    //����жϱ�־
  	ch1 = UART1_DR;       //�������ڽ��յ�������
  	UART1_SendData(ch1);   //�ѽ��յ���������ͨ�����ڷ��ͳ�ȥ
}*/