#include <intrinsics.h>
#include "io430.h"
#include "config.h"

#define TIPO_ANALOGICO      0x02

// calibracao do sensor de temperatura
#define CAL_ADC_15T85         (*(unsigned int *)0x10e4)
#define CAL_ADC_15T30         (*(unsigned int *)0x10e2)
#define CAL_ADC_15VREF_FACTOR (*(unsigned int *)0x10e0)
#define CAL_ADC_OFFSET        (*(unsigned int *)0x10de)
#define CAL_ADC_GAIN_FACTOR   (*(unsigned int *)0x10dc)

//Define a porta de comunicação e seu bit
#define TX_PSEL P1SEL
#define TX_PDIR P1DIR
#define TX_POUT P1OUT
#define TX_BIT BIT2
#define TX_PULL BIT1

//Portas de comunicação e bit do botão
#define BT_PSEL P1SEL
#define BT_PDIR P1DIR
#define BT_PIN P1IN
#define BT_BIT BIT6

/*! \brief Taxa do oscilador */
#define CLK 1000000

/*!  \brief Taxa de transmissão dos dados */
#define BAUD 1200

/*!  |brief Numero de ciclos ate que ocorra uma interrupção de timer */
#define TIMER_CNTR CLK/BAUD

/*! \brief Número de ciclos do debounce */
#define DB_LIMIT  10

//Estados da máquina de transmissão
#define START_BIT   0
#define SEND_DATA   1
#define SEND_CHK    2
#define STOP_BIT    3
#define TX_WAIT     4
#define RESYNC_HIGH 5
#define RESYNC_LOW  6

/*!  \brief Habilita o watchdog */
#define WDT_FEED (WDTCTL = WDT_MDLY_32)

/*!  \brief Pino de TX em alto */
#define TX_HIGH (TX_POUT &= ~TX_BIT)

/*!  \brief Pino de TX em baixo */
#define TX_LOW (TX_POUT |= TX_BIT)

/*!  \brief Delay de aproximadamente o tamanho de um pacote */
#define DELAY 64

/*!  \brief Estrutura com os dados do sensor que serão transmitidos */
#pragma pack(1)
typedef struct SENSOR_DATA_ST
{
  unsigned char identificador[4];
  unsigned char tipo;
  unsigned char valor[2];
  unsigned char chk;
}SENSOR_DATA;

//Protótipo das funções de escopo local
static void serialInit(void);
static void serialInitIO(void);
static void btInitIO(void);
static void adc10Init(void);
static void clockInit(void);
static void timerInit(void);
static unsigned char chkCalc(unsigned char *,unsigned char);

/*!  \brief Armazena os dados que serão enviados */
SENSOR_DATA data;

/*!  \brief Armazena o valor atual do sensor */
unsigned char sensorVal = 1;

/*!  \brief Debounce state machine */
unsigned char dbSt = 0;

/*!  \brief Contador para rotina de debounce */
unsigned char dbCount = 0;

/*!  \brief Indica se é necessário transmitir um bytes com zero */
unsigned char dbTxOffPending = 0;

/*!  \brief Ponteiro para os dados que serão enviados */
unsigned char * dataPt = (unsigned char *)(&data);

/*!  \brief Número de bytes já transmitidos no pacote */
unsigned char countSendBytes = 0;

/*!  \brief Número de bits já transmitidos em um determinado byte */
unsigned char countSendBits = 0;

/*!  \brief Armazena estado da máquina de comunicação serial */
unsigned char txState = 0;

/*!  \brief Contador dos tempo de delay */
unsigned char countDelay = 0;

/*!  \brief Variavel temporaria para o calculo da temperatura */
long temp;

/*!  \brief Valor da temperatura em oC */
volatile long IntDegC;

/*!  \brief valor calibrado do slope do sensor de temperatura */
long tempSlope;

/*!  \brief valor calibrado do 0degC calibrado */
long temp0degC;

/*!  \brief Sinaliza que tem novo valor de temperatura */
volatile unsigned char tempReady = 0;

#pragma segment="ID_ADDR"
const volatile unsigned char * NUMERO_SERIE = (unsigned char const volatile *)0xF800; //{0x00,0x00,0x00,0x07};

void main(void)
{
   //Para watchdog
   WDTCTL = WDTPW + WDTHOLD;

   //Inicia DCO
   clockInit();

   //Inicia periféricos
   adc10Init();
   btInitIO();
   serialInit();
   
   // calcula os parametros do sensor de temperatura
   tempSlope = ((CAL_ADC_15T85 - CAL_ADC_15T30) * 1000L)/55; // slope * 1000
   temp0degC = CAL_ADC_15T30 - ((tempSlope * 30)/1000);     // valor de referencia para 0degC
   
   //Inicia o watchdog
   WDT_FEED;

   //Modo de baixo consumo com interrupção habilitada
   __enable_interrupt();
   ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start

   while(1)
   {
      
      __bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
      
      if (tempReady)
      {
         tempReady = 0;
         
         // calibracao da leitura do ADC
         // ADC (corrected) =( (ADC(raw) x CAL_ADC15VREF_FACTOR / 2^15) x CAL_ADC_GAIN_FACTOR / 2^15 )+ CAL_ADC_OFFSET
         temp = ADC10MEM;
         temp = (temp * CAL_ADC_15VREF_FACTOR) / 32767; // ajuste do VREF
         temp = (temp * CAL_ADC_GAIN_FACTOR) / 32767;       // ajuste do fator de ganho
         temp += CAL_ADC_OFFSET;                            // ajuste do offset
         
         // calculo da temperatura
         IntDegC = (((temp - temp0degC) * 10000) / tempSlope);
         // oC = ((A10/1024)*1500mV)-986mV)*1/3.55mV = A10*423/1024 - 278
         //IntDegC = ((temp - 673) * 423) / 1024;

         __no_operation();                       // SET BREAKPOINT HERE
      
         ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
      }
  }
}

/*!  \brief Configura o DCO do MSP */
static void clockInit(void)
{
  //DCO em 1 MHz
  BCSCTL1= CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;
}

/*!  \brief Configura o hw do timer */
static void timerInit(void)
{
  //Habilita interrupção
  TACCTL0 = CCIE;

  //Define o período com sendo o tempo de bit dos dados
  TACCR0 = TIMER_CNTR;
  TACTL = TASSEL_2 + MC_1;
}

/*!  \brief Configura o hw da serial e o timer */
static void serialInit(void)
{
   serialInitIO();
   timerInit();

   //Preenche a estrutura
   data.identificador[0] = NUMERO_SERIE[0];
   data.identificador[1] = NUMERO_SERIE[1];
   data.identificador[2] = NUMERO_SERIE[2];
   data.identificador[3] = NUMERO_SERIE[3];

   data.tipo = TIPO_ANALOGICO;
}

/*!  \brief Configura os pinos de transmissão do sensor */
static void serialInitIO(void)
{
  //Configura pino como IO
  TX_PSEL &= ~TX_BIT;
  TX_PSEL &= ~TX_PULL;

  //Configura pino como saída em nível alto
  TX_PDIR |= TX_BIT;
  TX_HIGH;

  //Configura Pull up como saída em nível alto
  TX_PDIR |= TX_PULL;
  TX_POUT |= TX_PULL;
}

/*!  \brief Configura o pino conectado ao botão */
static void btInitIO(void)
{
  //Configura como GPIO e entrada
  BT_PSEL &= ~BT_BIT;
  BT_PDIR &= ~BT_BIT;
}

/*! \brief Inicializa o ADC10 para medir a temperatura */
static void adc10Init(void)
{
   ADC10CTL1 = INCH_10 + ADC10DIV_3;         // Temp Sensor ADC10CLK/4
   ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
  tempReady = 1;
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
  WDT_FEED;

  switch(dbSt)
  {
    case 0:
      if(BT_PIN & BT_BIT)
      {
        dbCount++;
        if(dbCount >= DB_LIMIT)
        {
          dbTxOffPending = 1;
          sensorVal = 0;
          dbSt = 1;
        }
      }
      else
      {
        dbCount = 0;
        sensorVal = 1;
      }
    break;

    case 1:
      if(!(BT_PIN & BT_BIT))
      {
        dbCount++;
        if(dbCount >= DB_LIMIT)
        {
          dbSt = 0;
          sensorVal = 1;
        }
      }
      else
      {
        dbCount = 0;
        sensorVal = 0;
      }
    break;
  }

  switch(txState)
  {
    case START_BIT:
      TX_LOW;
      countSendBits = 0;
      countSendBytes = 0;
      txState = SEND_DATA;

      data.valor[0] = (IntDegC>>8) & 0x00FF;
      data.valor[1] = (IntDegC) & 0x00FF;


      //Calcula o checksum dos dados
      data.chk = (unsigned char)chkCalc((unsigned char *)(&data),7);
    break;

    case SEND_DATA:
      if(dataPt[countSendBytes] & (1 << (7 - countSendBits)))
      {
        TX_HIGH;
      }
      else
      {
        TX_LOW;
      }

      countSendBits++;

      if(countSendBits > 7)
      {
        countSendBits = 0;
        countSendBytes++;
        txState = RESYNC_HIGH;
      }

      if(countSendBytes > 7)
      {
        countSendBytes++;
        txState = STOP_BIT;
      }
    break;

    case RESYNC_HIGH:
        TX_HIGH;
        txState = RESYNC_LOW;
    break;

    case RESYNC_LOW:
        TX_LOW;
        txState = SEND_DATA;
    break;

    case STOP_BIT:
      TX_HIGH;
      txState = TX_WAIT;
      countDelay = 0;
    break;

    case TX_WAIT:
      if(++countDelay > DELAY)
      {
        txState = START_BIT;
      }
    break;

    default:
      txState = START_BIT;
    break;
  }
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

/*!  \brief Calcula o checksum dos dados do identificador
 *   \param pmsg Ponteiro para os dados do identificador
 *   \param msgSize Quantidade de bytes do identificador
 */
static unsigned char chkCalc(unsigned char * pmsg,unsigned char msgSize)
{
  unsigned int chk;
  unsigned char i;

  chk = 0;
  for(i=0;i<msgSize;i++)
  {
    chk += pmsg[i];
  }

  return (unsigned char)chk;
}