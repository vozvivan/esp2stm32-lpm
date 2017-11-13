/*
 * commands.h
 *
 *  Created on: 2 окт. 2017 г.
 *      Author: twist
 */

#ifndef COMMANDS_H
#define COMMANDS_H
/*
 * TODO:
 * 1. Fix commands descriptions
 */
/*
 * ***************************************************************************
 * command		:	AT
 * description	:	Returns OK answer if esp8266 is ready to listen commands.
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		AT		"AT\r\n"

/*
 * ***************************************************************************
 * command		:	ATE0
 * description	:	Turn off echo
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATE0		"ATE0\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CWMODE=3
 * description	:	AP + client mode
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCWMODE1	"AT+CWMODE=1\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CIPMODE=0
 * description	:	Enable the two-way communication ability of TCP server/client (server<->client)
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATECIPMODE0	"AT+CIPMODE=0\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CWJAP="nameap","password"
 * description	:	Connect to AP
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCWJAP		"AT+CWJAP=\"AI-THINKER_9D1DB8\",\"\"\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CWJAP="nameap","password"
 * description	:	Connect to AP
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		TOP_ROOM		"AT+CWJAP=\"TOP_ROOM\",\"mkoldA1991\"\r\n"

#define		MARAT		"AT+CWJAP=\"Marat\",\"12345678\"\r\n"

/*
 * ***************************************************************************
 * command		:	AT+RST
 * description	:	Reboot device
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATRST		"AT+RST\r\n"

/*
 * ***************************************************************************
 * command		:	AT+RESTORE
 * description	:	Restore defaults
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATRESTORE	"AT+RESTORE\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CWQAP
 * description	:	Disconnect from AP
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCWQAP		"AT+CWQAP\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CWJAP?
 * description	:	Is connected to AP?
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCWJAP		"AT+CWJAP?\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CWMODE?
 * description	:	Selected mode
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCWMODE	"AT+CWMODE?\r\n"

#define		ATCWMODE3	"AT+CWMODE=3\r\n"


/*
 * ***************************************************************************
 * command		:	AT+CIPMODE?
 * description	:	Selected TCP mode
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCIPMODE	"AT+CIPMODE?\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CIPSEND=n
 * description	:	Send n bytes
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCIPSEND	"AT+CIPSEND="

/*
 * ***************************************************************************
 * command		:	AT+CIPMUX=0
 * description	:	Set single connection mode
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCIPMUX	"AT+CIPMUX=0"

/*
 * ***************************************************************************
 * command		:	AT+CIPMUX=0
 * description	:	Set single connection mode
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCIPMUX1	"AT+CIPMUX=1\r\n"


/*
 * ***************************************************************************
 * command		:	AT+CIPSTART
 * description	:	Start TCP clien session
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCIPSTART	"AT+CIPSTART=\"UDP\",\"192.168.0.105\",89\r\n"

/*
 * ***************************************************************************
 * command		:	AT+CIPSTART
 * description	:	Start TCP clien session
 * answer		:	OK
 * byte answer	:	\r\r\nOK
 * ***************************************************************************
 */
#define		ATCIPSERVER	"AT+CIPSERVER=1,88\r\n"

#define TPLINK "AT+CWJAP=\"TP-LINK_AFF0\",\"19965099\r\n"


#endif /*COMMANDS_H*/

