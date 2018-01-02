////////////////////////////////////////////////////////////////////////////
//
//  basic_cfg.h
//
//
//	Author 	: Rene B�llhoff
//  Created : around Jul 2010
//
//	(c) 2005-2010 by Rene B�llhoff - reneboellhoff (at) gmx (dot) de - 
//
//////////////////////////////////////////////////////////////////////////////
//
//  This source file may be used and distributed without
//  restriction provided that this copyright statement is not
//  removed from the file and that any derivative work contains
//  the original copyright notice and the associated disclaimer.
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//										
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU
//  General Public License for more details.
//
//  You should have received a copy of the GNU General Public
//  License along with this source; if not, download it	from
//  http://www.gnu.org/licenses/gpl.html	
//
////////////////////////////////////////////////////////////////////////////


	// Dieser Konfigurationsabschnitt ist f�r die beiden Schichten
	// Tokenizer und Basic-Interpreter wie f�r den Compiler ma�gebend.
	// Hier lassen sich der Umfang des Komplexes konfigurieren
	#ifdef COMMON_CONFIG


		// Globaler TOKEN-Datentyp.
		// Steht f�r alle erkannten Tokens (inkl. Error und Ende)

		#define TOKEN char

			
		// Mit diesem Schalter werden die C-Erweiterungen aktiviert,
		// mit denen es m�glich ist C-Funktionen aufzurufen, sowie
		// C-Variablen und C-Arrays zu lesen und zu beschreiben.
		// An die C-Funktionen k�nnen Parameter �bergeben werden,
		// wobei die Art der Parameter (String oder Zahl/Ausdruck) 
		// wahlweise �bergeben werden ohne Pr�fung des Aufrufers
		// d.h. die Funktion hat ggf die korrekte Art der Parameter zu
		// �berpr�fen. F�r das zu �bergebende Parameter-Array existiert
		// ein eigener Typ der der Funktion mit �bergeben wird.
		// (T_USER_FUNC_DATAS)

		#define BASIC_USES_EXTENSIONS


		// Mit diesem Schalter wird die Compiler/Byte-Code-Interpreter
		// Funktion aktiviert. Mit dem Compiler lassen sich die
		// Basic-Quellcodes in einen kleineren und deutlich schneller zu 
		// interpretierenden Byte(Token)-Code �bersetzen.
		// Der Compiler besteht aus einer Funktion die den Basic-Quellcode �hnlich 
		// der interpretierenden Ausf�hrungsart durchl�uft, sich dabei aber
		// Goto/Gosub Sprungmarken merkt, und in einem zweiten Durchlauf
		// die erkannten Tokens in einen Speicher geschrieben, sowie
		// diese vorher ermittelten Sprungmarken durch direkte Sprungadressen 
		// (bezogen auf den Quellcodeanfang) ersetzt. 

		#define BASIC_USES_COMPILER
		
		//#define BASIC_VM_ONLY


		// Mit diesem Schalter wird die Unterst�tzung f�r 
		// Zeicheneingabeger�te eingeschaltet. Damit lassen sich die
		// Basic-Quellcodes (oder auch der Byte(Token)-Bin�rcode
		// �ber gesonderte Funktionen von jedem beliebigen Medium aus
		// ausf�hren. Ohne diese Funktion steht "nur" die Zeiger-Variante
		// offen, die die L�nge des ausf�hrbaren Codes von der Gr��e des uCs
		// abh�ngig macht. Das Zeicheneingabeger�t enth�lt eine Reihe von
		// Funktionszeigern die den Lese-Zugriff auf die Zeichen erlaubt.

		//#define BASIC_USES_CHARDEVICE
			//#define CHARDEVICE_RAM
			//#define CHARDEVICE_BLOCK

		 

		// Dieser Schalter erlaubt die direkte Text-Einblendung von Operationen
		// der im Abschnitt BASIC_TOKEN_OPS enthaltenen Operatoren.
		// ist dieser Schalter nicht gesetzt so wird die angegebene Funktion
		// ausgef�hrt. Dies erlaubt Checks (z.b. Division durch 0 ...)

		#define BASIC_USES_FUNC_AS_OPS


		// Dieser Schalter erm�glicht das die Basic-Fehlermeldungen
		// im Klartext mitcompiliert werden. 
		// Andernfalls werden nur die Fehlercodes ausgegeben.

		//#define BASIC_USES_ERROR_TEXTS



		// Dieses Define legt die printf_P Funktion f�r Basic-Ausgaben fest.
		#define BASIC_PRINTF_P 		PRINTF_P

		// Dieses Makro lenkt die Abort-Routine fest
		// die bei einem kritischen Interpreter-Abbruch (falsches Token bekommen,
		// Sprungziel unbekannt, usw) aufgerufen wird.
		extern void fatal_abort (char x);
		#define FATAL_ABORT(x)  fatal_abort (x)

	#endif


	// Dieser Konfigurationsabschnitt definiert Parameter die speziell 
	// die Tokenizer-Schicht betreffen
	#ifdef TOKENIZER_CONFIG

		// Dieser Typ wird in Tokenizer genauso wie im Basic selbst verwendet.
		// Hier wird der grundlegende Datentyp f�r die Positionierung
		// festgelegt. Da der Typ ohne weitere Verarbeitung weitergereicht bzw
		// als tempor�rer Zwischenspeicher ben�tigt wird kann hier ohne Probleme
		// auch ein char* verwendet werden. Dies kann von Vorteil sein wenn 
		// keine Zeicheneingabeger�te verwendet werden. Ansonsten lassen sich nur
		// mit entsprechender Kenntnis direkte Zeiger und Zeicheneingabeger�te
		// verwenden (immerhin ist es ja Sinn und Zweck der Zeicheneingaberger�te
		// die interne Verwendung zu "verschleiern" um eine Abstraktion zu schaffen)
	  
		// Hinweis : Wenn man Integer-Zahlen unter Windows verwendet, sollte man 
		//           sofern man kein Zeichenger�t hat, U32 nehmen. (4-Bytezeiger unter WIN32)
#ifdef WIN32
		typedef U32 CPOS;
#endif

#ifdef AVR
		typedef U16 CPOS;
#endif

		//typedef char * CPOS;

		// Maximale L�nge von Zahlen
	  #define MAX_NUMLEN 						5
		// Maximale L�nge des C-Extension-Keywords (f�r Funktionen gleicherma�en 
		// wie f�r Variablen/Arrays sowie �bergebene Strings)
		#define MAX_USER_KEYWORD_LEN	16
		// Maximale Anzahl an der C-Funktion �bergebenen Parametern 
		#define MAX_USER_PARAMS				3

	#endif



	// Dieser Konfigurationsabschnitt definiert Parameter die speziell 
	// die Basic-Schicht betreffen
	#ifdef BASIC_CONFIG

		// Bestimmt die Maximale L�nge des Text-Strings beim PRINT-Befehl
		#define MAX_STRINGLEN					40

		// Maximale Tiefe des Gosub-Stacks
		#define MAX_GOSUB_STACK_DEPTH 2

		// Maximale Tiefe des FOR/NEXT-Stacks
		#define MAX_FOR_STACK_DEPTH		4

		// Maximale Anzahl der 1-Buchstaben-Variablen (alphabetisch)
		#define MAX_VARNUM						26

		// Maximale Programml�nge (nur f�r das demo)
		#define MAX_PROGRAM_LEN 			255

		#define UBASIC_SCRIPT_COUNT	3

	#endif


	// Dieser Konfigurationsabschnitt definiert Parameter die speziell 
	// die Compiler-Schicht betreffen
	#ifdef COMPILER_CONFIG

		// Hier wird die Gr��e des Zeilennummernbuffers f�r den Compiler definiert
		// Anzumerken bleibt das der Buffer lokal (!!) angelegt wird und keineswegs global
		// vorhanden ist. Daher mu� gew�hrleistet sein das der Stack entsprechend Platz
		// bietet !!
		#define MAX_LINE_BUFFER				50
		
		// Gr��e des Ausgabespeichers f�r Compilierte Programme (nur f�r das demo)
		#define MAX_COMPILER_BUFFER   300

	

	#endif



	#ifdef TOKENIZER_MEDIUM
	//TOKENIZER_MEDIUM  (name,              	workchar, 					
	//																				nextchar, 
	//																				getrelpos, 
	//																				setrelpos)
#ifdef CHARDEVICE_RAM
		TOKENIZER_MEDIUM 	(CHARDEV_SIMPLERAM,  	ram_tokenizer_settext,
																						ram_tokenizer_getchar,   
																						ram_tokenizer_nextchar,
																						ram_tokenizer_getrelpos,
																						ram_tokenizer_setrelpos)
#endif

#ifdef CHARDEVICE_BLOCK
		TOKENIZER_MEDIUM 	(CHARDEV_BLOCKDEV,  	blk_tokenizer_settext,
																						blk_tokenizer_getchar,
																						blk_tokenizer_nextchar,
																						blk_tokenizer_getrelpos,
																						blk_tokenizer_setrelpos)
#endif
	#endif


	// Dieser Konfigurationsabschnitt enth�lt alle Fehlermeldungen 
	// Dabei wird keine Unterscheidung zwischen Tokenizer/Compiler und Basic-Fehlern
	// gemacht. Ob die Texte tats�chlich mit �bersetzt werden bestimmt der Compilerschalter
	// BASIC_USES_ERROR_TEXTS
	#ifdef BASIC_ERROR
		BASIC_ERROR (FA_UNIMPLEMENTED_FUNCTION,						"function not implemented"		)
		BASIC_ERROR (FA_TOO_MANY_LINES,										"too many lines"							)
		BASIC_ERROR (FA_LINE_IN_CACHE_NOT_FOUND,					"line in cache not found"			)
		BASIC_ERROR (FA_ERROR_IN_PARAMETER_LIST,					"error in parameterlist"			)
		BASIC_ERROR (FA_UNEXPECTED_TOKEN,									"unexpected token"						)
		BASIC_ERROR (FA_EXTENSION_WORD_NOT_FOUND,					"extension word not found"		)
		BASIC_ERROR (FA_ERROR_IN_FACTOR,									"error in factor"							)
		BASIC_ERROR (FA_ERROR_IN_TERM,										"error in term"								)
		BASIC_ERROR (FA_ERROR_IN_EXPRESSION,							"error in expression"					)
		BASIC_ERROR (FA_ERROR_JUMP_DESTINATION_NOT_FOUND, "jump destination not found"	)
		BASIC_ERROR (FA_ERROR_GOSUB_STACK_EMPTY,          "no more gosubs on stack"			)    
	#endif


	// Dieser Konfigurationsabschnitt definiert die Schl�sselw�rter und
	// dazugeh�rige Funktionen (z.b. Print inkl Verarbeitung, For if/then/else)
	// Dabei k�nnen Schl�sselw�rter genauso wie einzelne Zeichen definiert werden.
	#ifdef BASIC_TOKENS
	//TOKEN_WORD	(name, text, function)
	//TOKEN_CHAR	(name, char, function)
		//TOKEN_WORD	(TOKENIZER_LET,			"let"		, let_statement_2		)		
		TOKEN_WORD	(TOKENIZER_WAIT,		"wait"	, wait_statement		)
		TOKEN_WORD	(TOKENIZER_PRINT,		"print" , print_statement		)
		TOKEN_WORD	(TOKENIZER_IF,			"if"		, if_statement			)
		TOKEN_WORD	(TOKENIZER_THEN,		"then"	, NO_FUNCTION				)
		TOKEN_WORD	(TOKENIZER_ELSE,		"else"	, NO_FUNCTION				)
		TOKEN_WORD	(TOKENIZER_FOR,			"for"		, for_statement			)
		TOKEN_WORD	(TOKENIZER_TO,			"to"		, NO_FUNCTION				)
		TOKEN_WORD	(TOKENIZER_NEXT,		"next"	, next_statement		)
		TOKEN_WORD	(TOKENIZER_GOTO,		"goto"	, goto_statement		)
		TOKEN_WORD	(TOKENIZER_GOSUB,		"gosub"	, gosub_statement		)
		TOKEN_WORD	(TOKENIZER_RETURN,	"return", return_statement	)
		TOKEN_WORD	(TOKENIZER_END,			"end"		, end_statement			)

//		TOKEN_WORD	(TOKENIZER_SHL,			"shl"		, NO_FUNCTION  			)
//		TOKEN_WORD	(TOKENIZER_SHR,			"shr"		, NO_FUNCTION  			)
		TOKEN_WORD	(TOKENIZER_XOR,			"xor"		, NO_FUNCTION  			)
		TOKEN_WORD	(TOKENIZER_AND,		"and"		, NO_FUNCTION  			)
		TOKEN_WORD	(TOKENIZER_OR, 		"or" 		, NO_FUNCTION  			)
	
		TOKEN_WORD	(TOKENIZER_NE, 			"<>"		, NO_FUNCTION 			)
		TOKEN_WORD	(TOKENIZER_GE, 			">="		, NO_FUNCTION 			)
		TOKEN_WORD	(TOKENIZER_LE, 			"<="		, NO_FUNCTION 			)
	
	
		TOKEN_CHAR	(TOKENIZER_COMMA,				','			)
		//TOKEN_CHAR	(TOKENIZER_SEMICOLON,		';'			)
		TOKEN_CHAR	(TOKENIZER_PLUS,				'+'			)
		TOKEN_CHAR	(TOKENIZER_MINUS,				'-'			)
		TOKEN_DUP	(TOKENIZER_AND,					'&'			)
		TOKEN_DUP	(TOKENIZER_OR,					'|'			)
		TOKEN_DUP	(TOKENIZER_XOR,					'^'			)
		TOKEN_CHAR	(TOKENIZER_ASTR,				'*'			)
		TOKEN_CHAR	(TOKENIZER_SLASH,				'/'			)
		TOKEN_CHAR	(TOKENIZER_MOD,					'%'			)
		TOKEN_CHAR	(TOKENIZER_LEFTPAREN,		'('			)
		TOKEN_CHAR	(TOKENIZER_RIGHTPAREN,	')'			)
#ifdef BASIC_USES_EXTENSIONS
		TOKEN_CHAR	(TOKENIZER_LEFTBRACK,		'['			)
		TOKEN_CHAR	(TOKENIZER_RIGHTBRACK,	']'			)
#endif	
		TOKEN_CHAR	(TOKENIZER_LT,					'<'			)
		TOKEN_CHAR	(TOKENIZER_GT,					'>'			)
		TOKEN_CHAR	(TOKENIZER_EQ,					'='			)
		TOKEN_CHAR	(TOKENIZER_CR,					'\n'		)
																				
	#endif	
	
	// In diesem Abschnitt werden die Operatoren (+ - * / usw) definiert.
	// Es kann die im Makro angegebene Funktion ausf�hren (bei Operationen mit + - * / usw)
	// Dann mu� BASIC_USES_FUNC_AS_OPS definiert sein. Ansonsten wird der Operator 
	// im Makro direkt verwendet, also reine Textersetzung.
	// TOKEN_CHAR_TERM wird in der Funktion "term" verwendet. (Faktoren)
	// TOKEN_CHAR_EXPR wird in der Funktion "expr" verwendet. (Produkte)
	// TOKEN_CHAR_REL wird in der Funktion "realtion" verwendet. (Vergleiche)
	#ifdef BASIC_TOKEN_OPS
	
		// Faktor-Operatoren in Basic-Ausdr�cken
		TOKEN_CHAR_TERM	(TOKENIZER_ASTR,		*  , basic_mul			)
		TOKEN_CHAR_TERM	(TOKENIZER_SLASH,		/  , basic_div			)
		TOKEN_CHAR_TERM	(TOKENIZER_MOD,			%  , basic_mod			)
	
		// Produkt-Operatoren in Basic-Ausdr�cken
		TOKEN_CHAR_EXPR	(TOKENIZER_PLUS,		+  , basic_add			)
		TOKEN_CHAR_EXPR	(TOKENIZER_MINUS,		-  , basic_sub			)
		TOKEN_CHAR_EXPR	(TOKENIZER_AND,			&  , basic_and			)
		TOKEN_CHAR_EXPR	(TOKENIZER_OR,			|  , basic_or				)
		//TOKEN_CHAR_EXPR	(TOKENIZER_ANDT,		&  , basic_and			)
		//TOKEN_CHAR_EXPR	(TOKENIZER_ORT,			|  , basic_or				)
		TOKEN_CHAR_EXPR	(TOKENIZER_XOR,			^  , basic_xor			)

		// Vergleichs-Operatoren
		TOKEN_CHAR_REL	(TOKENIZER_EQ, 			== , basic_rel_eq		)
		TOKEN_CHAR_REL	(TOKENIZER_GT, 			>  , basic_rel_gt		)
		TOKEN_CHAR_REL	(TOKENIZER_LT, 			<  , basic_rel_lt		)
		TOKEN_CHAR_REL	(TOKENIZER_NE, 			!= , basic_rel_ne		)
		TOKEN_CHAR_REL	(TOKENIZER_GE, 			>= , basic_rel_ge		)
		TOKEN_CHAR_REL	(TOKENIZER_LE, 			<= , basic_rel_le		)	  
	
	#endif	
	
	// Hier werden die C-Funktionen, C-Variablen, C-Arrays 
	// deklariert und in den Basic-Sprachumfang integriert.
	// C-Funktionen haben generell den Aufbau : 
	// "signed short NAME (T_USER_FUNC_DATAS stDatas)"
	// wobei T_USER_FUNC_DATAS aus einem Array von MAX_USER_PARAMS Eintr�gen.
	// Jeder Eintrag enth�lt ein Byte �ber den Datentyp (String/Integer) und
	// eine Union aus einem 16-Bit signed integer Wert und einem String der 
	// L�nge MAX_USER_KEYWORD_LEN. Die Anzahl der an die Funktion �bergebenen
	// Parameter ist ebenfalls definierbar und wird vom Basic gepr�ft.
	// C-Variablen sind 16 Bit Signed, und k�nnen gelesen und beschrieben werden
	// C-Arrays sind 16 Bit Signed, und k�nnen ebenfalls gelesen und beschrieben werden.
	// Indizes 
	#ifdef BASIC_USER_EXTENSION_INCLUDES
		#include "port.h"
		#include "servo.h"
		#include "ln_support.h"
	#endif
	
	#ifdef BASIC_USER_EXTENSIONS
	//USER_FUNCTION 	(name,text,func,param)
	//USER_VARIABLE 	(name,text,var,size)
	//USER_ARRAY    	(name,text,var,size)
	/*
		USER_FUNCTION 	(LCD_PRINT,		"lcdout",   lcdoutfunc,   3	)
		USER_FUNCTION 	(RESET_FUNC,	"reset",    resetfunc,		0	)
	
		USER_FUNCTION 	(HALF_FUNC,		"half",     halffunc,			1	)
		USER_FUNCTION 	(DBL_FUNC,		"double",   dblfunc,			1	)
		
		USER_VARIABLE 	(SYSTICK,			"systick",  systick, 1					)
	
		USER_ARRAY			(ADC_ARRAY,		"adc",      adc,    			8	) */
USER_VARIABLE(1, "ln_gpio_status", ln_gpio_status[0], 1)
USER_VARIABLE(2, "ln_gpio_status_tx", ln_gpio_tx[0], 1)
USER_VARIABLE(3, "relay_request", relay_request, 1)
USER_VARIABLE(4, "servo1_time_delta", servo[0].time_delta, 1)
USER_VARIABLE(5, "servo1_setpoint", servo[0].position_setpoint, 1)
USER_VARIABLE(6, "servo1_position", servo[0].position_actual, 1)
USER_VARIABLE(7, "servo2_time_delta", servo[1].time_delta, 1)
USER_VARIABLE(8, "servo2_setpoint", servo[1].position_setpoint, 1)
USER_VARIABLE(9, "servo2_position", servo[1].position_actual, 1)
USER_VARIABLE(10, "port_di", port_di, 1)
USER_VARIABLE(11, "port_do", port_do, 1)
USER_VARIABLE(12, "port_do_select", port_do_select, 1)
USER_VARIABLE(13, "port_mode", port_mode, 1)
USER_VARIABLE(14, "ln_gpio_opcode_tx", ln_gpio_tx[0], 1)
USER_VARIABLE(15, "ln_gpio_opcode_tx2", ln_gpio_tx[1], 1)
USER_VARIABLE(16, "pwm_port1_target", pwm_port[0].dimm_target, 1)
USER_VARIABLE(17, "pwm_port2_target", pwm_port[1].dimm_target, 1)
USER_VARIABLE(18, "pwm_port3_target", pwm_port[2].dimm_target, 1)
USER_VARIABLE(19, "pwm_port4_target", pwm_port[3].dimm_target, 1)
USER_VARIABLE(20, "pwm_port5_target", pwm_port[4].dimm_target, 1)
USER_VARIABLE(21, "pwm_port6_target", pwm_port[5].dimm_target, 1)
USER_VARIABLE(22, "pwm_port7_target", pwm_port[6].dimm_target, 1)
USER_VARIABLE(23, "pwm_port1_delta", pwm_port[0].dimm_delta, 1)
USER_VARIABLE(24, "pwm_port2_delta", pwm_port[1].dimm_delta, 1)
USER_VARIABLE(25, "pwm_port3_delta", pwm_port[2].dimm_delta, 1)
USER_VARIABLE(26, "pwm_port4_delta", pwm_port[3].dimm_delta, 1)
USER_VARIABLE(27, "pwm_port5_delta", pwm_port[4].dimm_delta, 1)
USER_VARIABLE(28, "pwm_port6_delta", pwm_port[5].dimm_delta, 1)
USER_VARIABLE(29, "pwm_port7_delta", pwm_port[6].dimm_delta, 1)
USER_FUNCTION(30, "rand", ubasic_random,0)
USER_FUNCTION(31, "sv_read", ubasic_sv_read,1)
USER_FUNCTION(32, "sv_write", ubasic_sv_write,2)
USER_ARRAY(33, "pwm_delta", pwm_delta,1,7)
USER_ARRAY(34, "pwm_targert", pwm_target,1,7)
USER_VARIABLE(35, "pwm_update_trig", pwm_update_trig, 3)
USER_VARIABLE(36, "pwm_update_cont", pwm_update_cont, 3)
USER_VARIABLE(37, "pwm_at_setpoint", pwm_at_setpoint, 3)
USER_FUNCTION(38, "pwm_set", ubasic_pwm_update,3)



	#endif
