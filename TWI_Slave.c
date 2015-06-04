//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <math.h>
#include "twislave.c"
#include "lcd.c"
//#include "linsin.c"

#include "adc.c"
#include "onewire.c"
#include "ds18x20.c"
#include "crc8.c"
#include "version.c"

//***********************************
//Estrich							*
#define SLAVE_ADRESSE 0x58 //		*
//									*
//***********************************
#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC


#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR		DDRD
#define LOOPLEDPIN		4 // 



#define SDAPIN		1 // PORT C
#define SCLPIN		0

#define TWI_WAIT_BIT		3
#define TWI_OK_BIT		4



#define ESTRICH_OUTPORT	PORTD		// Ausgangsport fuer Estrich
#define ESTRICH_OUTDDR	DDRD
#define ESTRICH_INPORT	PORTB		// Eingangsport fuer Estrich
#define ESTRICH_INPIN	PINB
#define ESTRICH_INDDR	DDRB

#define TWI_WAIT_BIT		3
#define TWI_OK_BIT		4


#define UHRBIT 0

//#define SERVOPORT	PORTD			// Ausgang fuer Servo
//#define SERVOPIN0 5				// Impuls für Servo 1.					Geändert 17.11.10
//#define SERVOPIN1 4				// Enable fuer Servo, Active H	Geändert 17.11.10

// Definitionen Slave Estrich
#define UHREIN 0
#define UHRAUS 1

//
#define SPI_CONTROL_DDR			DDRA
#define SPI_CONTROL_PORT		PORTA
#define SPI_CONTROL_PORTPIN	PINA

#define SPI_CLK_PIN		0 // INT0
#define SPI_DATA_PIN		1

#define SPI_CLK_WAIT		30
#define SPI_CLK_DELAY		1

#define DATENBREITE	10  // Uebertragungsbreite ADC 
#define ADC_OFFSET	160 // ADC-Wert fuer 0 Grad


#define PAUSEOK	15	// Anzahl Schlaufen von timer0 bis Pause erkannt
#define STARTOK	1	//Bit 1 von spistatus
#define ENDOK		2	//Bit 2 von spistatus
#define READOK		3	//Bit 3 von spistatus

#define OSZIPORT				PORTB
#define OSZIPORTDDR			DDRB
#define OSZIPORTPIN			PINB
#define OSZIA					0

#define OSZILO OSZIPORT &= ~(1<<OSZIA)
#define OSZIHI OSZIPORT |= (1<<OSZIA)
#define OSZITOG OSZIPORT ^= (1<<OSZIA)

// Defines von DS1820
#define SENSORB0	0x8A	// Kollektor-Vorlauf
#define SENSORB1	0x3E	// Kollektor-Ruecklauf

#define SENSORA0	0xBA	// Boiler unten
#define SENSORA1	0x9B	// Boiler Mitte
#define SENSORA2	0x22	// Boiler oben

//

#define TASTE1		38
#define TASTE2		46
#define TASTE3		54
#define TASTE4		72
#define TASTE5		95
#define TASTE6		115
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		225
#define TASTE0		235
#define TASTER		245
#define TASTATURPORT PORTC

#define TASTATURPIN		3

#define BUZZERPIN		2

#define KOLLEKTORVORLAUF			0	//	Byte fuer KOLLEKTORVORLAUFtemperatur
#define KOLLEKTORRUECKLAUF			1	//	Byte fuer KOLLEKTORRUECKLAUFtemperatur
#define BOILERUNTEN					2	//	Byte fuer Boilertemperatur unten
#define BOILERMITTE					3	//	Byte fuer Boilertemperatur mitte
#define BOILEROBEN					4	//	Byte fuer Boilertemperatur oben
#define ERROR							7	//	Byte fuer Errors

// PIN 2 von Port B als Eingang fuer DS1802
#define PUMPEPIN			3	//	PIN 3 von PORT B als Eingang fuer Pumpestatus
#define ELEKTROPIN		4	// Pin 4 von Port B als Eingang fuer Elektroeinsatzstatus
#define WASSERALARMPIN	7	// Pin 7 von Port B als Eingang fuer Wasseralarmsensor

#define KOLL_TEMP_OFFSET 163

#define STARTDELAYBIT	0
#define HICOUNTBIT		1

#define WDTBIT				7

// Wertetabelle fuer Temperatur zu Spannungswert. 186 Werte (Index 0-185)
uint8_t TempWerte[] EEMEM ={0, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 28, 
29, 30, 31, 32, 34, 35, 36, 37, 38, 40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58, 60, 61, 62, 64, 
65, 66, 67, 68, 70, 71, 72, 74, 75, 76, 78, 79, 80, 82, 83, 84, 85, 86, 88, 89, 90, 92, 93, 94, 96, 97, 98, 100, 101, 
102, 104, 105, 106, 108, 109, 110, 112, 113, 114, 116, 118, 119, 120, 122, 123, 124, 126, 127, 128, 130, 131, 132, 
134, 136, 137, 138, 140, 141, 142, 144, 145, 146, 148, 150, 151, 152, 154, 155, 156, 158, 160, 161, 162, 164, 165, 
166, 168, 170, 171, 172, 174, 176, 177, 178, 180, 182, 183, 184, 186, 188, 189, 190, 192, 194, 195, 196, 198, 200, 
201, 202, 204, 206, 207, 208, 210, 212, 213, 214, 216, 218, 220, 221, 222, 224, 226, 227, 228, 230, 232, 234, 235, 
236, 238, 240, 242, 243, 244, 246, 248, 250, 251, 252, 254};


//#define MAXSENSORS 2
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;

// SPI
static volatile uint8_t adcbuffer=0;

static volatile uint8_t spistatus=0;
uint8_t bitpos=0xFF; // Erkennen von Wait

static volatile uint8_t SlaveStatus=0x00; //status

void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);

volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];

void delay_ms(unsigned int ms);
uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

volatile uint8_t Estrichstatus=0x00;

volatile uint16_t Servotakt=20;					//	Abstand der Impulspakete
volatile uint16_t Servopause=0x00;				//	Zaehler fuer Pause
volatile uint16_t Servoimpuls=0x00;				//	Zaehler fuer Impuls
volatile uint8_t Servoimpulsdauer=20;			//	Dauer des Servoimpulses Definitiv
volatile uint8_t ServoimpulsdauerPuffer=22;		//	Puffer fuer Servoimpulsdauer
volatile uint8_t ServoimpulsdauerSpeicher=0;	//	Speicher  fuer Servoimpulsdauer
volatile uint8_t Potwert=45;
volatile uint8_t TWI_Pause=1;
volatile uint8_t ServoimpulsOK=0;				//	Zaehler fuer richtige Impulsdauer
uint8_t ServoimpulsNullpunkt=23;
uint8_t ServoimpulsSchrittweite=10;
uint8_t Servoposition[]={23,33,42,50,60};
volatile uint16_t ADCImpuls=0;

volatile uint8_t status=0;
uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset



// Code 1_wire start
//uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];


uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	
	ow_reset();
	
	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) 
	{
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("No Sensor found\0" );
						
			delay_ms(800);
			lcd_clr_line(1);
			break;
		}
		
		if( diff == OW_DATA_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("Bus Error\0" );
			break;
		}
		lcd_gotoxy(4,1);

		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
			{
				//lcd_gotoxy(15,1);
				//lcd_puthex(id[i]);

			gSensorIDs[nSensors][i] = id[i];
			//delay_ms(100);
			}
			
		nSensors++;
	}
	
	return nSensors;
}

// start a measurement for all sensors on the bus:
void start_temp_meas(void)
{

        gTemp_measurementstatus=0;
        if ( DS18X20_start_meas(NULL) != DS18X20_OK) 
		  {
                gTemp_measurementstatus=1;
        }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
        uint8_t i;
        uint8_t subzero, cel, cel_frac_bits;
        for ( i=0; i<gNsensors; i++ ) 
		  {
			  
			  if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
											 &cel, &cel_frac_bits) == DS18X20_OK ) 
			  {
				  gTempdata[i]=cel*10;
				  gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
				  if (subzero)
				  {
					  gTempdata[i]=-gTempdata[i];
				  }
			  }
			  else
			  {
				  gTempdata[i]=0;
			  }
        }
}

uint8_t lastSensorID(uint8_t index)
{
	return gSensorIDs[index][7];
}


// Code 1_wire end


void RingD2(uint8_t anz)
{
	uint8_t k=0;
	for (k=0;k<2*anz;k++)
	{
		PORTD |=(1<<BUZZERPIN);
		twidelay_ms(2);
		PORTD &=~(1<<BUZZERPIN);
		twidelay_ms(2);
		
	}
	PORTD &=~(1<<BUZZERPIN);
}


uint8_t Tastenwahl(uint8_t Tastaturwert)
{
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}



void slaveinit(void)
{
 	//DDRD |= (1<<DDD0);						//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	//	DDRD |= (1<<DDD1);						//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	//	DDRD |= (1<<DDD2);						//Pin 2 von PORT D als Ausgang fuer Buzzer
	// 	DDRD |= (1<<DDD3);						//Pin 3 von PORT D als Ausgang fuer LED TWI
	LOOPLEDDDR |= (1<<LOOPLEDPIN);		//Pin 4 von PORT D als Ausgang fuer LED
	//DDRD |= (1<<DDD5);						//Pin 5 von Port D als Pin fuer DS1820
 	//DDRD |= (1<<SERVOPIN1);					//Pin 6 von PORT D als Ausgang fuer Servo-Enable
	//DDRD |= (1<<SERVOPIN0);					//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
	//DDRD |= (1<<DDD2);						//Pin 2 von PORT D als Ausgang fuer Buzzer
	
	/*
	 DDRA &= ~(1<<PORTA0);	//Bit 0 von PORT A als Eingang für Taste 0
	 PORTA |= (1<<PORTA0);	//Pull-up
	 
	 DDRA &= ~(1<<PORTA1);	//Bit 1 von PORT A als Eingang für Taste 1
	 PORTA |= (1<<PORTA1);	//Pull-up
	 */
	
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	
	DDRC &= ~(1<<PORTC1);	//Bit 4 von PORT C als Eingang für SDA
	PORTC |= (1<<PORTC1);	//Pull-up
	DDRC &= ~(1<<PORTC0);	//Bit 5 von PORT C als Eingang für SCL
	PORTC |= (1<<PORTC0);	//Pull-up
	
	/*
	 
	 DDRA &= ~(1<<DDA2);	//Pin 2 von PORT A als Eingang fuer ADC 	
	 PORTA |= (1<<DDA2);	//Pull-up
	 DDRA &= ~(1<<DDA3);	//Pin 3 von PORT A als Eingang fuer Tastatur 	
	 PORTA |= (1<<DDA3);	//Pull-up
	 */
	
	ESTRICH_INDDR &= ~(1<< PUMPEPIN); // Eingang fuer Pumpe-Status
	ESTRICH_INPORT |= (1<<PUMPEPIN);  // HI
	
	ESTRICH_INDDR &= ~(1<< ELEKTROPIN); // Eingang fuer Elektro-Status
	ESTRICH_INPORT |= (1<<ELEKTROPIN);  // HI
	
	ESTRICH_INDDR &= ~(1<< WASSERALARMPIN); // Eingang fuer Wasseralarm-Status (Active LO)
	ESTRICH_INPORT |= (1<<WASSERALARMPIN);  // HI
	
	// Kollektor-Tempmessung
	
	// SPI 
	SPI_CONTROL_DDR |= (1<<SPI_CLK_PIN);	// Ausgang fuer Clock von SPI
	SPI_CONTROL_PORT |= (1<<SPI_CLK_PIN);	// HI
	SPI_CONTROL_DDR &= ~(1<<SPI_DATA_PIN);	// Eingang fuer Data von SPI
	SPI_CONTROL_PORT |= (1<<SPI_DATA_PIN);	// HI
	
	// OSZI
	OSZIPORTDDR |= (1<<OSZIA);					// Ausgang
	OSZIPORT |= (1<<OSZIA);						// HI
   
   SlaveStatus=0;
	SlaveStatus |= (1<<TWI_WAIT_BIT);

	
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
//	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	
//Timer fuer Servo	
	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
	TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//Rücksetzen des Timers
	
}


ISR (TIMER0_OVF_vect)
{ 
	ADCImpuls++;
	Servopause++;
	//lcd_clr_line(1);

	//lcd_gotoxy(10,1);
	//lcd_puts("Tim\0");
	//delay_ms(400);
	//lcd_cls();
	//lcd_clr_line(0);
	//lcd_gotoxy(0,1);
	//lcd_puts("Stop Servo\0");
	//lcd_puts(" TP\0");
	//lcd_putint1(TWI_Pause);
	//	Intervall 64 us, Overflow nach 16.3 ms
	if (Servopause==3)	// Neues Impulspaket nach 48.9 ms
	{

		if (TWI_Pause)
		{
//			lcd_gotoxy(19,0);
//			lcd_putc(' ');
//			timer2(Servoimpulsdauer);	 // setzt die Impulsdauer
         /*
			if (SERVOPPIN &  (1<<SERVOPIN1)) // Servo ist ON
			{
				SERVOPORT |= (1<<SERVOPIN0); // Schaltet Impuls an SERVOPIN0 ein
			}
			SERVOPORT |= (1<<5);// Kontrolle auf PIN D5
          */
		}
		Servopause=0;
	}
}



void timer2 (uint8_t wert) 
{ 

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 


uint16_t readThermometer(void)
{
// #define DATENBREITE
//	#define ADC_OFFSET
	//lcd_gotoxy(0,0);
	//lcd_puts("rt\0");
	
	uint16_t temperatur=0;
	SPI_CONTROL_PORT |= (1<<SPI_CLK_PIN); // CLK-Pin sicher HI
	SPI_CONTROL_PORT |= (1<<SPI_DATA_PIN); // DATA-Pin sicher HI
	delay_ms(2);
	
	// CLK auf LO ziehen
	SPI_CONTROL_PORT &= ~(1<<SPI_CLK_PIN);
	delay_ms(SPI_CLK_WAIT); // lange Pause zur Ankuendigung des Read
	
	// CLK wieder HI
	SPI_CONTROL_PORT |= (1<<SPI_CLK_PIN);
	delay_ms(SPI_CLK_DELAY);
	
	uint8_t i=0;
	for (i=0;i<DATENBREITE;i++)
	{
		// CLK auf LO ziehen
		SPI_CONTROL_PORT &= ~(1<<SPI_CLK_PIN);
		delay_ms(SPI_CLK_DELAY);
		if (SPI_CONTROL_PORTPIN & (1<<SPI_DATA_PIN)) // Bit ist HI
		{
			temperatur |= (1<<(DATENBREITE-1-i));
		}
		else
		{
			temperatur &= ~(1<<(DATENBREITE-1-i));
		}
		delay_ms(SPI_CLK_DELAY);
		// CLK wieder HI
		SPI_CONTROL_PORT |= (1<<SPI_CLK_PIN);
		delay_ms(SPI_CLK_DELAY);
	}// for i
	delay_ms(SPI_CLK_DELAY);
	SPI_CONTROL_PORT |= (1<<SPI_DATA_PIN); // DATA-Pin sicher HI

	return temperatur;
	
}



int main (void) 
{
	/* 
	 in Start-loop in while
	 init_twi_slave (SLAVE_ADRESSE);
	 sei();
	 */
	
	slaveinit();
	wdt_reset();
	WDTCR |= (1<<WDTOE) | (1<<WDE);
	WDTCR = 0x00;
	//MCRSR&=~(1<<WDRF);
	wdt_disable();
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	
	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts("READY\0");
	lcd_gotoxy(16,0);
   lcd_puts(VERSION);
	ESTRICH_OUTPORT &= ~(1<<UHREIN);//	UHREIN sicher low
	ESTRICH_OUTPORT &= ~(1<<UHRAUS);//	UHRAus sicher low
	ESTRICH_OUTPORT |= (1<<UHRAUS);
	delay_ms(10);
	ESTRICH_OUTPORT &= ~(1<<UHRAUS);
	
	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	//	uint8_t Servowert=0;
	//	uint8_t Servorichtung=1;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	uint8_t Schalterposition=0;
	//timer0();
	
	//initADC(TASTATURPIN);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint8_t loopcount1=0;
   
	//	uint8_t twierrcount=0;
	LOOPLEDPORT |=(1<<LOOPLEDPIN);
	
	delay_ms(800);
	//eeprom_write_byte(&WDT_ErrCount0,0);
	
	//	Zaehler fuer Wartezeit nach dem Start
	uint16_t startdelay0=0x00AF;
	//uint16_t startdelay1=0;
	
	//Zaehler fuer Zeit von (SDA || SCL = LO)
	//	uint16_t twi_LO_count0=0;
	//	uint16_t twi_LO_count1=0;
	
	//Zaehler fuer Zeit von (SDA && SCL = HI)
	//	uint16_t twi_HI_count0=0;
	
	uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0); 
	if (eepromWDT_Count0==0xFF)
	{
		eepromWDT_Count0=0;	
	}
	
	//	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
	/*
	 eepromWDT_Count0: Zaehler der wdt-Resets mit restart. 
	 
	 Neu:
	 Wenn ein wdt-Reset abläuft, wird Bit 7 in eepromWDT_Count0 gesetzt. 
	 Dadurch wartet der Prozessor mit dem Initialisieren des TWI-Slave, bis eine neue Startbedingung erscheint.
	 Anschliessend wird das Bit 7 wieder zurückgesetzt.
	 
	 alt:
	 eepromWDT_Count1: Zaehler fuer neuen wdt-Reset. Wenn wdt anspricht, wird der Zaheler erhoeht.
	 Beim Restart wird bei anhaltendem LO auf SDA oder SCL gewartet.
	 Wenn SCL und SDA beide HI sind, wird der Zaehler auf den Wert von eepromWDT_Count0 gesetzt 
	 und der TWI-Slave gestartet.
	 
	 */
	
	uint16_t Kollektortemperatur=0;
	uint16_t redKollektortemperatur=0;
	spistatus=0;
	
	uint8_t SlaveStatus=0x00; //status
	
	// Ankuendigen, dass schon ein wdt erfolgte
	if (eepromWDT_Count0 & (1<<WDTBIT))
	{
		//		lcd_gotoxy(18,1);
		//		lcd_puts("W\0");
		
	}
	
	
	//	uint8_t wertecounter=0;
	//	uint8_t wert1=40;
	//	uint8_t wert2=30;
	
	/*
	 Bit 0: 1 wenn wdt ausgelöst wurde
	 
	 */ 
	
	lcd_clr_line(0);  
	lcd_puts("Solar\0");
	
#pragma mark DS1820 init
	// DS1820 init-stuff begin
	uint8_t i=0;
	//	uint8_t nSensors=0;
	//lcd_send_char('A');
	ow_reset();
	gNsensors = search_sensors();
	
	delay_ms(100);
	lcd_gotoxy(0,1);
	lcd_puts("Sensors: \0");
	lcd_putint1(gNsensors);
	delay_ms(1000);
	if (gNsensors>0)
	{
		//lcd_clr_line(1);
		start_temp_meas();
	}
	i=0;
	while(i<MAXSENSORS)
	{
		gTempdata[i]=0;
		i++;
	}
	// DS1820 init-stuff end
	lcd_clr_line(0);  
	lcd_puts("Solar\0");
   lcd_puts(" wait\0");
	
#pragma mark while
	while (1)
	{	
		//Blinkanzeige
		loopcount0++;
		if (loopcount0==0xFFFF)
		{
			loopcount0=0;
         loopcount1++;
			LOOPLEDPORT ^=(1<<LOOPLEDPIN);
			//		uint8_t temp=eeprom_read_byte(&TempWerte[1]);
			//lcd_gotoxy(0, 1);
			//lcd_putc('a'+(loopcount1 & 0x07));
			//delay_ms(10);
			// DS1820 loop-stuff begin
			// verschoben in rxdata
		}
		
		/**	Beginn Startroutinen	***********************/
		
		// Startfunktion: SCL und SDA pruefen, ob lang genug beide HI sind
      
		// wenn Startbedingung vom Master:  TWI_slave initiieren
		if (SlaveStatus & (1<<TWI_WAIT_BIT)) 
		{
			if ((TWI_PIN & (1<<SCLPIN))&&(!(TWI_PIN & (1<<SDAPIN))))// Startbedingung vom Master: SCL HI und SDA LO
			{
            
            init_twi_slave (SLAVE_ADRESSE);
            sei();
            SlaveStatus &= ~(1<<TWI_WAIT_BIT);
            SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
            lcd_gotoxy(5,0);
            lcd_puts("     \0");
            // StartDelayBit zuruecksetzen
            
			}
		}
		/**	Ende Startroutinen	***********************/
		
		
		/* **** rx_buffer abfragen **************** */
		
		if (rxdata)				//	Daten von TWI vorhanden, Befehle ausfuehren, Daten fuer naechsten Aufruf bereitstellen
		{
			
			Estrichstatus=rxbuffer[0];
			//lcd_gotoxy(0,1);
			//lcd_puts("St:\0");
			//lcd_puthex(Estrichstatus);
			//delay_ms(20);
			if ( Estrichstatus  & (1<<UHRBIT))
			{
				//delay_ms(1000);
				//Schaltuhr ein
				//				lcd_gotoxy(17,1);
				//				lcd_puts("ON \0");
				ESTRICH_OUTPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
				ESTRICH_OUTPORT &= ~(1<<UHREIN);//	UHREIN sicher low
				ESTRICH_OUTPORT |= (1<<UHREIN);
				delay_ms(20);
				ESTRICH_OUTPORT &= ~(1<<UHREIN);
			}
			else
			{
				//delay_ms(1000);
				//Schaltuhr aus
				//				lcd_gotoxy(17,1);
				//				lcd_puts("OFF\0");
				
				ESTRICH_OUTPORT &= ~(1<<UHREIN);//	UHREIN sicher low
				ESTRICH_OUTPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
				ESTRICH_OUTPORT |= (1<<UHRAUS);
				delay_ms(20);
				ESTRICH_OUTPORT &= ~(1<<UHRAUS);
			}
			
#pragma mark ADC			
			// tx_buffer laden
			
			// Temperatur lesen
			
			//initADC(KOLLEKTORVORLAUF);
			//uint16_t temperaturBuffer=(readKanal(KOLLEKTORVORLAUF));
			
#pragma mark DS lesen			
			// DS lesen start
			
			// DS1820 loop-stuff begin
			
			lcd_gotoxy(19,1);
			lcd_putc('*');
			start_temp_meas();
			delay_ms(800);
			read_temp_meas();
			
			
			
			// Sensoren abfragen	
			uint8_t index=0;
			//lcd_clr_line(1);
			for (index=0;index<gNsensors;index++)
			{
				uint8_t last= lastSensorID(index); // Letztes Byte der ID
				//lcd_gotoxy(0,0);
				//lcd_puthex(last);
				
				uint8_t indexTemperatur=2*((gTempdata[index]/10) & 0x00FF);
				if (gTempdata[index]%10 >=5) // Dezimalstelle ist >=05: Wert  aufrunden, 1 addieren
				{
					indexTemperatur+= 1;
				}
				// Temperaturwerte zuteilen
				switch (last)
				{
					case SENSORB0: // Kollektor-Vorlauf
					{
						//Sensor B0
						txbuffer[KOLLEKTORVORLAUF]=indexTemperatur;
						lcd_gotoxy(7,3);
						lcd_puts("V:     \0");
						if (gTempdata[index]/10>=100)
						{
							lcd_gotoxy(8,3);
							lcd_putint((gTempdata[index]/10));
						}
						else
						{
							lcd_gotoxy(9,3);
							lcd_putint2((gTempdata[index]/10));
						}
						
						lcd_putc('.');
						lcd_putint1(gTempdata[index]%10);
						
					}break;
						
					case SENSORB1: // Kollektor-Ruecklauf
					{
						//Sensor B1
						txbuffer[KOLLEKTORRUECKLAUF]=indexTemperatur;
						lcd_gotoxy(14,3);
						lcd_puts("R:   \0");
						if (gTempdata[index]/10>=100)
						{
							lcd_gotoxy(15,3);
							lcd_putint((gTempdata[index]/10));
						}
						else
						{
							lcd_gotoxy(16,3);
							lcd_putint2((gTempdata[index]/10));
						}
						
						lcd_putc('.');
						lcd_putint1(gTempdata[index]%10);
						
					}break;
						
					case SENSORA0: // Boiler unten
					{
						//Sensor A0
						txbuffer[BOILERUNTEN]=indexTemperatur;
						lcd_gotoxy(0,2);
						lcd_puts("U:     \0");
						if (gTempdata[index]/10>=100)
						{
							lcd_gotoxy(1,2);
							lcd_putint((gTempdata[index]/10));
						}
						else
						{
							lcd_gotoxy(2,2);
							lcd_putint2((gTempdata[index]/10));
						}
						
						lcd_putc('.');
						lcd_putint1(gTempdata[index]%10);
						
					}break;
						
					case SENSORA1: // Boiler mitte
					{
						//Sensor A1
						txbuffer[BOILERMITTE]=indexTemperatur;
						lcd_gotoxy(7,2);
						lcd_puts("M:   \0");
						if (gTempdata[index]/10>=100)
						{
							lcd_gotoxy(8,2);
							lcd_putint((gTempdata[index]/10));
						}
						else
						{
							lcd_gotoxy(9,2);
							lcd_putint2((gTempdata[index]/10));
						}
						
						lcd_putc('.');
						lcd_putint1(gTempdata[index]%10);
						
					}break;
						
					case SENSORA2: // Boiler oben
					{
						//Sensor A2
						txbuffer[BOILEROBEN]=indexTemperatur;
						lcd_gotoxy(14,2);
						lcd_puts("O:   \0");
						if (gTempdata[index]/10>=100)
						{
							lcd_gotoxy(15,2);
							lcd_putint((gTempdata[index]/10));
						}
						else
						{
							lcd_gotoxy(16,2);
							lcd_putint2((gTempdata[index]/10));
						}
						
						lcd_putc('.');
						lcd_putint1(gTempdata[index]%10);
						
					}break;
					default:
					{
						lcd_gotoxy(0,2);
						lcd_puts("nix\0");
					}
						
				}	// switch last
				//delay_ms(1000);
			}// for index
			
			// end Sensoren abfragen
			
			
			// DS1820 loop-stuff end
			
			// DS lesen end
			lcd_gotoxy(19,1);
			lcd_putc(' ');
			
#pragma mark Kollektortemperatur
			// if (spistatus & (1<< READOK)) // durch rxdata ersetzt
			{
				/*
				 lcd_gotoxy(0, 1);
				 lcd_puts("           \0");
				 lcd_gotoxy(0, 1);
				 lcd_puts("Wert \0");
				 */
				//txbuffer[2]=0;
				//txbuffer[3]=0;
				//txbuffer[4]=0;
				Kollektortemperatur=0;
				Kollektortemperatur=readThermometer();
				redKollektortemperatur=Kollektortemperatur;

				// Eventuell Fehler
            
            if (redKollektortemperatur >= (4* KOLL_TEMP_OFFSET))
				{
               txbuffer[7]= 1;
					redKollektortemperatur -= (4* KOLL_TEMP_OFFSET); // Wert bei 0 Grad wegzaehlen
				}
				else 
				{
               txbuffer[7]= 2;
					redKollektortemperatur=0; // 255 bei negativen Zahlen verhindern
				}
				
				//redKollektortemperatur &= 0x00FF; // 8 Bit
				//lcd_clr_line(3);
				//delay_ms(50);
				//lcd_gotoxy(0,3);
				//lcd_puts("TKr:\0");
				//lcd_putint(redKollektortemperatur & 0x00FF);
				
				//lcd_gotoxy(0,2);
				//lcd_puts("TK:\0");
				//		uint8_t KollektortemperaturH=Kollektortemperatur>>2;
				//		uint8_t KollektortemperaturL=Kollektortemperatur & 0x003;
				
				//lcd_putint(KollektortemperaturH);
				//lcd_putc('*');
				//lcd_putint(KollektortemperaturL);
				
				//txbuffer[2]= KollektortemperaturH;
				//txbuffer[3] |= KollektortemperaturL; // Bits 0,1
				uint8_t KollektortemperaturIndex=redKollektortemperatur & 0x00FF;
            
				if (KollektortemperaturIndex >185) // nur 185 Werte
				{
               txbuffer[7]= 3;
					KollektortemperaturIndex =185;
					
				}
				uint8_t temperatur=eeprom_read_byte(&TempWerte[KollektortemperaturIndex]);
				lcd_gotoxy(0,3);
				
				//lcd_putint2(KollektortemperaturIndex);
				//lcd_putc(' ');
				lcd_puts("T:\0");
				lcd_putint(temperatur/2);
				//txbuffer[4]=temperatur; 
				txbuffer[5]=temperatur;
				
				spistatus &= ~(1<< READOK);
			}
			
#pragma mark Status abfragen	
			txbuffer[6]=0;
			//	Pumpe abfragen
			if (ESTRICH_INPIN & (1<<PUMPEPIN)) // Pumpe ist OFF
			{
				txbuffer[6] &= ~(1<<PUMPEPIN); // Bit ist LO
			}
			else 
			{
				txbuffer[6] |= (1<<PUMPEPIN); // Bit ist HI
			}
			
			
			
			// Elektroeinsatz abfragen
			if (ESTRICH_INPIN & (1<<ELEKTROPIN))
			{
				txbuffer[6] &= ~(1<<ELEKTROPIN); // Bit ist LO
			}
			else 
				
			{
				txbuffer[6] |= (1<<ELEKTROPIN); // Bit ist HI
			}
			
			// Wasseralarm abfragen
			if (ESTRICH_INPIN & (1<<WASSERALARMPIN)) // Pin 7 PIN ist HI, alles OK
			{
				txbuffer[6] &= ~(1<<WASSERALARMPIN); // Bit ist LO
			}
			else // Wasser laeuft
				
			{
				txbuffer[6] |= (1<<WASSERALARMPIN);
			}
			
			//txbuffer[7]= eeprom_read_byte(&WDT_ErrCount0);
			
			rxdata=0;
			
		} // if rxdata
		
		
		
		if (!(PINB & (1<<PB0))) // Taste 0
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P0 Down\0");
			
			if (! (TastenStatus & (1<<PB0)))			//Taste 0 war nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<PB0);
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount ++;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
					// Aktion hier
					// DS1820 init-stuff begin
					uint8_t i=0;
					//		uint8_t nSensors=0;
					ow_reset();
					gNsensors = search_sensors();
					delay_ms(100);
					lcd_gotoxy(0,1);
					lcd_puts("Sensors: \0");
					lcd_putint1(gNsensors);
					if (gNsensors>0)
					{
						start_temp_meas();
					}
					i=0;
					while(i<MAXSENSORS)
					{
						gTempdata[i]=0;
						i++;
					}
					// DS1820 init-stuff end
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<PB0);
				}
			}//else
			
		}	// Taste 0
		
		
		if (!(PINB & (1<<PB1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P1 Down\0");
			
			if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<PB1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("B1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount ++;
				if (Tastencount >= Tastenprellen)
				{
					// Aktion hier
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<PB1);
				}
			}//	else
			
		} // Taste 1
		
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
		
		if (Tastenwert>23)
		{
			/*
			 0: 
			 1: 
			 2: 
			 3: 
			 4: 
			 5: 
			 6: 
			 7: 
			 8: 
			 9: 
			 */
			
			TastaturCount++;
			if (TastaturCount>=50)
			{
				
				//lcd_clr_line(1);
				//lcd_gotoxy(8,1);
				//lcd_puts("T:\0");
				//lcd_putint(Tastenwert);
				
				uint8_t Taste=Tastenwahl(Tastenwert);
				
				//lcd_gotoxy(18,1);
				//lcd_putint2(Taste);
				//delay_ms(600);
				// lcd_clr_line(1);
				
				
				TastaturCount=0;
				Tastenwert=0x00;
				//		uint8_t i=0;
				//		uint8_t pos=0;
				
				switch (Taste)
				{
					case 0://
					{ 
						
					}break;
						
					case 1://
					{ 
					}break;
						
					case 2://
					{ 
						
					}break;
						
					case 3://
					{ 
						
					}break;
						
					case 4://
					{ 
					}break;
						
					case 5://
					{ 
					}break;
						
					case 6://
					{ 
					}break;
						
					case 7://
					{ 
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9://
					{ 
					}break;
						
						
				}//switch Tastatur
			}//if TastaturCount	
			
		}//	if Tastenwert
		
	}//while
	
	
	// return 0;
}
