/*
 * SPI.h
 *
 *  Created on: 13/ago/2014
 *      Author: Mihamed Hammouda
 */

#ifndef SPI_H_
#define SPI_H_
#ifdef __cplusplus
extern "C" {
#endif
void initSPIA0(void);/*Inizializzazione del Hw*/
void disableSPIA0(void);
void spi_senddata(short int data);
unsigned char spi_recievedata(void);
void spi_csh(void); /*Drive CS High*/
void spi_csl(void); /* Drive CS LOW */
#endif /* SPI_H_ */
