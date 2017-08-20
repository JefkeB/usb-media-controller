/*
 * ScanPort.h
 *
 *  Created on: 20 aug. 2017
 *      Author: JefPC
 */

#ifndef SCANPORT_H_
#define SCANPORT_H_

//
//
//
#include <stdint.h>


//
//
//
typedef struct TPort
{
   uint8_t input  ;
   uint8_t xor    ;
   uint8_t level  ;
   uint8_t vorig  ;

   uint8_t nieuw  ;
   uint8_t oud    ;
   uint8_t up     ;
   uint8_t down   ;
} TPort ;


//
//
//
void ScanPort( TPort *p );


#endif /* SCANPORT_H_ */
