/*    添加包含芯片的头文件    */
#include <iostm8s103f3.h>
#include <string.h>
#include <stdio.h>
#include "B_LUX_V20.h"

#define uint unsigned int
#define uchar unsigned char

#define DATA_SET PA_ODR_bit.ODR2 = 1
#define DATA_CLR PA_ODR_bit.ODR2 = 0
#define DATA_GET PA_IDR_bit.IDR2

uchar datatime = 0;          //记录电平宽度
uchar datareg = 0;           //存储八位数据
uchar datanum = 0;           //获取当前数据情况，筛选出一字节数据
uchar data_dh11[3] = {0x00}; //储存所有的数据
uchar outline;               //等待超时重来
uchar DHTFLAG = 0;           //判断总线读取状态的标志位

void UART1_SendData(uchar data)
{
    while (!(UART1_SR & 0X80))
        ;            //判断发送数据寄存器是否为空
    UART1_DR = data; //向发送寄存器写入数据
}

int fputc(int ch, FILE *f)
{
    UART1_SendData(ch);
    return ch;
}

/*
	PD5对应tx输出 UART1数据发送
	PD6对应rx输入 UART1数据接收
	但是好像要反接
*/
void UART1_Init(unsigned long baudrate)
{
    //uint baudrate
    uint baud;
    baud = 16000000 / baudrate;                                              //设定串口相应波特率与串口时钟的分频数
    UART1_BRR2 = ((uchar)((baud & 0xf000) >> 8)) | ((uchar)(baud & 0x000f)); //先写波特比率的高4位与低4位
    UART1_BRR1 = ((uchar)((baud & 0x0ff0) >> 4));                            //后写波特比率的中间8位
    UART1_CR1_bit.UART0 = 0;                                                 //使能UART0
    UART1_CR2_bit.RIEN = 1;                                                  //使能中断接收
    UART1_CR2_bit.TEN = 1;                                                   //使能发送
    UART1_CR2_bit.REN = 1;                                                   //接收使能

    PD_DDR_bit.DDR5 = 1; //输出模式
    PD_CR1_bit.C15 = 1;  //

    PD_DDR_bit.DDR6 = 0; //输入模式
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
    crc &= 0xFF; // 取低八位

    UART1_SendData(0x3C); //开始标志
    UART1_SendData(0x02); //type：发送数据
    UART1_SendData(len);
    for (i = 0; i < len; i++)
    {
        UART1_SendData(data[i]);
    }
    UART1_SendData((uchar)crc); //字节校验符
    UART1_SendData(0x0D);       //结束标志
}

//参考18B20设置输出输入两种IO模式
void DATA_OUT()
{
    PA_DDR_bit.DDR2 = 1; //一开始是输出模式
    PA_CR1_bit.C12 = 1;  //使用推挽输出模式
    PA_CR2_bit.C22 = 0;
}

//输入模式
void DATA_IN() //中断上拉输入模式
{
    PA_DDR_bit.DDR2 = 0;
    PA_CR1_bit.C12 = 1;
    PA_CR2_bit.C22 = 1;
}
//外部中断的配置
void EXTI_PA()
{
    EXTI_CR1 = 0X02; //PD口对于外部中断下降沿触发
}

//定时器设定 使用通用定时器2进行精确地微秒延时
void TIM2_Config()
{
    TIM2_PSCR = 0x00; //预分频器设定（该寄存器是四位寄存器）
    TIM2_ARRH = 0x00; //16000 表示为 0x0010 就是说在 16000000的频率下，1微秒就会溢出
    TIM2_ARRL = 0x10;

    TIM2_CNTRH = 0x00; //计数器清零
    TIM2_CNTRL = 0x00;
    TIM2_SR1 = 0x00; //清除所有标志位
}

//基于以上设定的延时函数  本程序采用了查询等待方式进行精确延时 不过也的确是完全占用了CPU的时间，也算是一种非常粗暴的延时方式
void TIM2_Delayus(uint xus) //范围 0~65535毫秒
{
    TIM2_CR1 = 0x81; //打开ARPE自动预装载使能，使能计数器
    while (xus--)
    {
        while (!(TIM2_SR1 & 0X01))
            ;                //等待计数器和ARRH/L发生匹配溢出 也就是一毫秒时间到了
        TIM2_SR1 &= ~(0X01); //清除标志位（唯一清零方式）
    }
    TIM2_CR1 = 0x00; //关闭定时器
}

//定时器设定 使用基本定时器4进行电平宽度采集
void TIM4_Config()
{
    TIM4_PSCR = 0x04; //预分频器设定（该寄存器是四位寄存器）16分频道 一步就是一个微秒
    TIM4_CNTR = 0x00; //计数器清零
    TIM4_SR = 0x00;   //清除所有标志位
    TIM4_CR1 = 0x00;  //处于关闭状态
}

//复位DHT11
void DHT11_RST()
{
    TIM4_CR1 = 0x00;     //关闭定时器
    TIM4_CNTR = 0;       //保证下次的第一个数据位的准确
    DATA_SET;            //ODR设置为1
    DATA_OUT();          //推挽输出模式，此时输出高电平
    DATA_CLR;            //此时处于主机输出模式，总线拉低
    TIM2_Delayus(18000); //拉低20毫秒
    DATA_SET;            //释放总线
    TIM2_Delayus(20);    //释放总线以后等待40微秒DHT会发出响应信号
}

//检测DHT11是否响应
uchar DHT11_CHECK()
{

    if (!DATA_GET) //如果顺利拉低，就说明有了响应
    {
        while ((!DATA_GET) && (outline < 100)) //先是低电平
        {
            TIM2_Delayus(1);
        }
        if (outline > 90) //起始信号超时退出
            return 0;
        outline = 0;
        while ((DATA_GET) && (outline < 100)) //接着是高电平
        {
            TIM2_Delayus(1);
        }
        if (outline < 90)
            TIM4_CR1 = 0x81; //立刻打开定时器开始计时第一个数据位
        else
            return 0;
        DATA_IN(); //引脚设置为外部中断模式
        outline = 0;

        return 1; //一切成功返回1
    }
    else
        return 0;
}

void UART1_DHT_Display()
{
    if (DHTFLAG) //器件初始化正常的情况下
    {
        if (data_dh11[0] + data_dh11[1] == data_dh11[2])
        {
			printf("DH11_W:");
            UART1_SendData(data_dh11[0] / 100 + 48); //湿度
			//printf("温度%d %d | 温度 %d %d \n", data_da11[0] /100 + 48);
            UART1_SendData((data_dh11[0] / 10) % 10 + 48);
            UART1_SendData(data_dh11[0] % 10 + 48);
			printf("\nDH11_T:");
            UART1_SendData(data_dh11[1] / 100 + 48); //温度
            UART1_SendData((data_dh11[1] / 10) % 10 + 48);
            UART1_SendData(data_dh11[1] % 10 + 48);
            UART1_SendData('\n');
        }
        else
            printf("\r\n数据校验失败\r\n");
    }
    else
        printf("\r\n器件读取失败\r\n");
}

void delay(unsigned int ms)
{
  unsigned int x , y;
  for(x = ms; x > 0; x--)           /*  通过一定周期循环进行延时*/
    for(y = 3000 ; y > 0 ; y--);
}

void ADC_Init()
{
  PC_DDR_bit.DDR4 = 0;   //设置PC->4 为输入
  PC_CR1_bit.C14 = 0;    //设置为悬空输入
  PC_CR2_bit.C24 = 0;    //设置中断禁止

  ADC_CR1_bit.SPSEL = 3;    //fmaster / 18 = 16MHZ / 18 = 888888HZ 分频
  ADC_CR2_bit.ALIGN = 1;    //RIGHT ALIGN 右对齐
  ADC_CSR_bit.CH = 2;       //SELECT AIN2 选择转化通道2

  ADC_CR1_bit.ADON = 1;     //启动ADC
}

void ADC_Data_Read(unsigned int *AD_Value)
{
  ADC_CR1_bit.ADON = 1;       //启动ADC

  while(ADC_CSR_bit.EOC == 0);  //等待转换结束
  *AD_Value = ADC_DRH;          //先读取高8位
  *AD_Value = (unsigned int)((*AD_Value << 8) + ADC_DRL);   //高8位与低8位相加，凑成16位数据
}

void main()
{
    asm("sim");        //关闭系统总中断
    CLK_CKDIVR = 0x00; //CPUDIV = 1 HSIDIV = 1  内部时钟 = 16Mhz
    DATA_SET;
    DATA_OUT();         //设置为推挽输出状态
    UART1_Init(115200); //调用串口初始化函数，并设置波特率为115200 bps
    TIM2_Config();      //设置用于精确延时的定时器2
    TIM4_Config();      //设置用于宽度捕捉的定时器4
	ADC_Init();
	B_LUX_Init();
    EXTI_PA();          //在总中断没有打开的时候可以打开外部中断了
    asm("rim");         //打开系统总中断
    while (1)           //进入死循环，等待串口接收中断
    {
        //Lora_SendData("0213test\r\n");
		for (int i = 0; i < 20; i++)
        TIM2_Delayus(50000);    //大概延时一会
		
        DHT11_RST();             //复位
        DHTFLAG = DHT11_CHECK(); //获取返回值 是否成功  1成功 0 失败
		
		for (int i = 0; i < 11; i++)
        	TIM2_Delayus(50000);    //大概延时一会
		
		//温温度
        UART1_DHT_Display();     //利用串口显示温湿度
		
		//土壤
		unsigned int ADCDATA;  //定义一个16位变量
		ADC_Data_Read(&ADCDATA);//把采样的数据通过串口发送到PC板串口软件进行显示
    	printf("YL69:%d\n" , ADCDATA);
		
		//光照
		unsigned long result = 0;
		B_LUX_GetLux(&result);
   		printf("BH1750:%ld\n",result);
		
        for (int i = 0; i < 11; i++)
        	TIM2_Delayus(50000);    //大概延时一会*/
    }
}

#pragma vector = 0x05 //PA的中断向量位
__interrupt void GPIOA_IRQHandler()
{
    datatime = TIM4_CNTR;                   //获取两次下降沿之间的数据宽度
    TIM4_CNTR = 0;                          //清零，再次获取下一位
    datareg <<= 1;                          //高位先出，左移操作
    if ((datatime > 75) && (datatime < 85)) //数据0  我就默认高位开始获取了
        datareg &= 0xfe;
    if ((datatime > 120) && (datatime < 130)) //数据1
        datareg |= 0x01;
    if (datanum == 7)
        data_dh11[0] = datareg; //获取第一个字节也就是湿度整数位
    if (datanum == 23)          //获取第三个字节也就是温度整数位
        data_dh11[1] = datareg;
    if (datanum == 39) //获取第五个字节也就是校验（温度+湿度）位
        data_dh11[2] = datareg;
    datanum++;         //每次读取一位进1
    if (datanum >= 40) //数据接收完了结束
        datanum = 0;
}

/*    串口接收数据中断服务函数    */
/*
#pragma vector = 0x14              //设置串口接收中断向量号 = 0X14 = 20
__interrupt void UART1_RX_RXNE(void)
{
  	unsigned char ch1;

  	UART1_SR_RXNE = 1;    //清除中断标志
  	ch1 = UART1_DR;       //读出串口接收到的数据
  	UART1_SendData(ch1);   //把接收到的数据再通过串口发送出去
}*/