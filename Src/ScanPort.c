/*
 * ScanPort.c
 *
 *  Created on: 20 aug. 2017
 *      Author: JefPC
 */

//
//
//
#include "ScanPort.h"


//
//
//
void ScanPort( TPort *p )
{
   p->level = ( p->input ^ p->xor ) & ( p->level | p->vorig ) | ( p->level & p->vorig ) ;
   p->vorig = ( p->input ^ p->xor ) ;
   p->nieuw = p->level & ( p->nieuw | p->oud ) | ( p->nieuw & p->oud ) ;
   p->oud   = p->level ;

   p->up   |= p->nieuw & ( p->oud   ^ 0xff ) ;
   p->down |= p->oud   & ( p->nieuw ^ 0xff ) ;
}
