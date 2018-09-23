#ifndef SPI_PINS_H_
#define SPI_PINS_H_

/*
NXP FRDM-K64F SPI Available busses:
----------------------------------------------------
                    MISO	MOSI	CLK     CS0
----------------------------------------------------                
SPI0    | CPU	    PTD2	PTD3	PTD1	PTD0
        | HEADER    J2.8	J2.10	J2.12	J2.6	
----------------------------------------------------
SPI1	| CPU       PTB16	PTB17	PTB11	PTB10	// Pins used for JTAG/UART
        | HEADER  
----------------------------------------------------
SPI2	| CPU       PTB22	PTB23	PTB21	PTB20   // Pins used for LED RGB
        | HEADER  
----------------------------------------------------
*/

#define SCK_PIN   PTD1
#define MISO_PIN  PTD2
#define MOSI_PIN  PTD3
#define SS_PIN    PTD0

#endif /* SPI_PINS_H_ */