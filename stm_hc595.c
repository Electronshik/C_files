#ifndef _HC595_INCLUDED_
#define _HC595_INCLUDED_

//если нет include в main.c
//#include <stm32f10x_gpio.h>
//#include <stm32f10x_tim.h>

//общий катод/общий анод, нужное раскомментировать
//#define     CATHODE_COM
#define     ANODE_COM

//количество разрядов индикатора
#define     WIDTH   8

//к каким пинам подключена hc595
#define 	DS_PORT		GPIOA
#define 	DS_PIN 		GPIO_Pin_3
#define     SH_CP_PORT  GPIOA
#define		SH_CP_PIN	GPIO_Pin_4
#define     ST_CP_PORT  GPIOA
#define		ST_CP_PIN	GPIO_Pin_5
#define		RCC_PORT 	RCC_APB2Periph_GPIOA

#define 	DS_WRITE	GPIO_WriteBit(DS_PORT, DS_PIN, ds & 0x01);
#define		SH_CP_LOW	GPIO_ResetBits(SH_CP_PORT, SH_CP_PIN);
#define		SH_CP_HIGH	GPIO_SetBits(SH_CP_PORT, SH_CP_PIN);
#define		ST_CP_LOW	GPIO_ResetBits(ST_CP_PORT, ST_CP_PIN);
#define		ST_CP_HIGH	GPIO_SetBits(ST_CP_PORT, ST_CP_PIN);

//вывод с точкой
#define     WITH_DOT    | DOT

//показывать незначащие нули в разрядах
#define     ZERO_SHOW   0

//таблица символов
#define     CLEAR   0x00
#define     DOT     0x80
#define     MINUS   0x40
#define     CH_O    0x63   //градус
#define     CH_o    0x5C
#define     CH_A    0x77
#define     CH_b    0x7C
#define     CH_c    0x58
#define     CH_C    0x39
#define     CH_d    0x5E
#define     CH_E    0x79
#define     CH_F    0x71
#define     CH_h    0x74
#define     CH_H    0x76
#define     CH_L    0x38
#define     CH_n    0x54
#define     CH_U    0x3E
#define     CH_v    0x1C
#define     NUM_0   0x3F
#define     NUM_1   0x06
#define     NUM_2   0x5B
#define     NUM_3   0x4F
#define     NUM_4   0x66
#define     NUM_5   0x6D
#define     NUM_6   0x7D
#define     NUM_7   0x07
#define     NUM_8   0x7F
#define     NUM_9   0x6F

//подключение выходов к индикатору
//выход микросхемы Qn к какому разряду подключен по факту, слева направо q7>>q0 микросхемы
#define     Q1      0b00001000     //1  0b00000001
#define     Q2      0b00000100     //2  0b00000010
#define     Q3      0b00000010     //3  0b00000100
#define     Q4      0b00000001     //4  0b00001000
#define     Q5      0b10000000     //5  0b00010000
#define     Q6      0b01000000     //6  0b00100000
#define     Q7      0b00100000     //7  0b01000000
#define     Q8      0b00010000     //8  0b10000000
//выход микросхемы Sn к какому сегменту индикатора подключен, слева направо q7>>q0 микросхемы
#define     S1      0b00000001     //a  0b00000001
#define     S2      0b00010000     //b  0b00000010 
#define     S3      0b00001000     //c  0b00000100
#define     S4      0b10000000     //d  0b00001000
#define     S5      0b00000100     //e  0b00010000
#define     S6      0b01000000     //f  0b00100000
#define     S7      0b00000010     //g  0b01000000
#define     S8      0b00100000     //dp 0b10000000

#ifdef      CATHODE_COM
#define     OUT
#endif

#ifdef      ANODE_COM
#define     OUT     ds = ~(ds && 1);
#endif

//массив символов для вывода(буфер)
char display[WIDTH];

//вызвать из main.c, использует TIM4 (для stm32f103c8t6)
void hc595_timer_gpio_init(void);

//функция работы динамической индикации, сдвигает индикацию на следующий символ по кругу
//вызывать, например, каждую миллисекунду
void hc595_clock();

void TIM4_IRQHandler()
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    hc595_clock();
  }
}

//записать в буфер вывода символ из таблицы символов или напрямую байт по сегментам
//младший бит сегмент a, старший - точка
//char symbol - байт или константа для записи
//char number - позиция символа от 1 до WIDTH
//например hc595_char(CH_F, 3), hc595_char(0x02, 1);
void hc595_char(char symbol, unsigned char number);

//записать в буфер вывода цифру от 0 до 9
//char num - цифра 0-9
//char number - позиция символа от 1 до WIDTH
//например hc595_num(2, 1);
void hc595_num(char num, unsigned char number);

//записать в буфер вывода целое число от 0 до 65535(0x00 - 0xFFFF)
//unsigned int x - число для записи
//char number - позиция первого символа
//char width - количество разрядов числа
//char dot - количество разрядов после точки; 0 - целое, 1 - с десятыми, 2 - с сотыми
void hc595_int(unsigned int x, unsigned char number, char width, char dot);

//запись двух байтов в две hc595
void hc595_write(char data, char number)
{
	char ds;
    number--;
    number = 1 << number;
    //ST_CP_HIGH
    
    ds = number & Q8;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = number & Q7;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = number & Q6;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = number & Q5;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = number & Q4;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = number & Q3;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = number & Q2;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = number & Q1;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH

    ds = data & S8;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = data & S7;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = data & S6;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = data & S5;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = data & S4;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = data & S3;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = data & S2;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    ds = data & S1;
    OUT
    DS_WRITE
    SH_CP_LOW
    SH_CP_HIGH
    
    ST_CP_HIGH
    SH_CP_LOW
    ST_CP_LOW
}

void hc595_int(unsigned int x, unsigned char number, char width, char dot)
{             
    char s1,s2;
        switch (width)
        {
            case 1:
                hc595_num(x%10, number);
                break;
            case 2:
                if(dot == 1)
                {
                    hc595_num((x%100)/10 WITH_DOT, number);
                }
                else
                {  
                    s1 = (x%100)/10;
                    if(ZERO_SHOW || s1)
                    {
                        hc595_num(s1, number); 
                    }
                    else
                    {
                        hc595_num(10, number); 
                    }
                }
                hc595_num(x%10, number+1);
                break;
            case 3: 
                if(dot == 2)
                {
                    hc595_num((x%1000)/100 WITH_DOT, number); 
                }
                else
                {
                    s1 = (x%1000)/100;
                    if(ZERO_SHOW || s1)
                    {
                        hc595_num(s1, number); 
                    }
                    else
                    {
                        hc595_num(10, number); 
                    }
                }
                
                if(dot == 1)
                {
                   hc595_num((x%100)/10 WITH_DOT, number+1);
                }
                else
                {
                    //hc595_num((x%100)/10, number+1);
                    s2 = (x%100)/10;
                    if(ZERO_SHOW || dot || s1 || s2)
                    {   
                        hc595_num(s2, number + 1);
                    }
                    else
                    {
                        hc595_num(10, number + 1); 
                    }
                }
                
                hc595_num(x%10, number+2);
                break;
            case 4:
                hc595_num((x%10000)/1000, number);
                hc595_num((x%1000)/100, number + 1);
                hc595_num((x%100)/10, number + 2);
                hc595_num(x%10, number + 3);
                if(!ZERO_SHOW)
                {   
                    number--;
                    if(display[number] == 0x3F)display[number]=0x00;
                    if(display[number+1] == 0x3F)display[number+1]=0x00;
                    if(display[number+2] == 0x3F)display[number+2]=0x00;     
                }
                break;
            case 5:
                hc595_num(x/10000, number);
                hc595_num((x%10000)/1000, number + 1);
                hc595_num((x%1000)/100, number + 2);
                hc595_num((x%100)/10, number + 3);
                hc595_num(x%10, number + 4);
                if(!ZERO_SHOW)
                {   
                    number--;
                    if(display[number] == 0x3F)display[number]=0x00;
                    if(display[number+1] == 0x3F)display[number+1]=0x00;
                    if(display[number+2] == 0x3F)display[number+2]=0x00; 
                    if(display[number+3] == 0x3F)display[number+3]=0x00;     
                }
                break;
            default:
                break;
        };
} 

void hc595_num(char num, unsigned char number)
{
    char dot_mask = 0;
    if(num & DOT)  
    {
        dot_mask = 1;
        num &= ~DOT;
    }
    switch(num)
    {       
        case 0:
            num = NUM_0;
            break;
        case 1:
            num = NUM_1;
            break;  
        case 2:
            num = NUM_2;
            break;
        case 3:
            num = NUM_3;
            break;
        case 4:
            num = NUM_4;
            break;
        case 5:
            num = NUM_5;
            break;
        case 6:
            num = NUM_6;
            break;
        case 7:
            num = NUM_7;
            break;
        case 8:
            num = NUM_8;
            break;
        case 9:
            num = NUM_9;
            break;
        case 10:
            num = CLEAR;
            break;
        default:
            break;
    } 
    if(dot_mask)
    {
        num |= DOT;
    }
        number--;
        display[number] = num;  
}

void hc595_char(char symbol, unsigned char number)
{   if(number <= WIDTH)
    {   
        number--;
        display[number] = symbol;
    }
}

void hc595_clock()
{
    static unsigned char display_count=0;
    hc595_write(display[display_count], display_count + 1);
    display_count++;
    if(display_count >= WIDTH)
    {
        display_count = 0;
    }  
}

void hc595_timer_gpio_init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_PORT, ENABLE);

	GPIO_InitStructure.GPIO_Pin=DS_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(DS_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=SH_CP_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(SH_CP_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=ST_CP_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(ST_CP_PORT,&GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseInitTypeDef hc595_timer;
	TIM_TimeBaseStructInit(&hc595_timer);
	hc595_timer.TIM_Prescaler = 1600-1;
	hc595_timer.TIM_Period = 100;
	TIM_TimeBaseInit(TIM4, &hc595_timer);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn);
}

#endif
