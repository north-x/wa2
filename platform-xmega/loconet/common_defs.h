/****************************************************************************
    Copyright (C) 2002 Alex Shepherd

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

 Title :   Commin definitions
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     16-Mar-2003
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       LocoNet throttle common definitions

*****************************************************************************/

#ifndef __COMMON_DEFS_DEFINED
#define __COMMON_DEFS_DEFINED

#ifndef __BYTE_DEFINED
#define __BYTE_DEFINED
typedef unsigned char byte;
#endif


#ifndef __WORD_DEFINED
#define __WORD_DEFINED
#ifdef __BORLANDC__
typedef unsigned short word ;
#else
typedef unsigned int word ;
#endif
#endif

#ifndef __DWORD_DEFINED
#define __DWORD_DEFINED
#ifdef __BORLANDC__
typedef unsigned int dword;
#else
typedef unsigned long dword;
#endif
#endif

#ifndef __PBYTE_DEFINED
#define __PBYTE_DEFINED
typedef	unsigned char*		pbyte;
#endif

#ifndef __TSHORT_DEFINED
#define __TSHORT_DEFINED
typedef	signed char*		pshort;
#endif

#ifndef __TBOOL_DEFINED
#define __TBOOL_DEFINED
typedef	unsigned char		  bool;
typedef	unsigned char*		pbool;
#endif


#ifndef __BOOLEAN_DEFINED
#define __BOOLEAN_DEFINED
#ifndef TRUE
#define FALSE                     0x00
#define TRUE                      0x01
#endif
#endif

#ifndef __HIBYTE_DEFINED
#define __HIBYTE_DEFINED
#define HIBYTE(a)                 (byte) ((a & 0xff00) >> 8)
#endif

#ifndef __LOBYTE_DEFINED
#define __LOBYTE_DEFINED
#define LOBYTE(a)                 (byte) ((a & 0x00ff))
#endif



#endif
