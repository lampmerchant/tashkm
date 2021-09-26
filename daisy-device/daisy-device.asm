;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashKM: Daisy-Chain-Controlled ADB Device
;;;
;


;;; License ;;;

;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <https://www.gnu.org/licenses/>.


;;; Connections ;;;

;;;                                                                   ;;;
;                           .--------.                                  ;
;                   Supply -|01 \/ 08|- Ground                          ;
;     ADB Data <-->    RA5 -|02    07|- RA0/TX ---> To Next RX          ;
;    ADB Power <---    RA4 -|03    06|- RA1/RX <--- From Previous TX    ;
;              --->    RA3 -|04    05|- RA2    ----                     ;
;                           '--------'                                  ;
;;;                                                                   ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1840, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1840.inc
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_CPD_OFF	Data memory protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _LVP_OFF
			;_WRT_OFF	Write protection off
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

				;Timer0 values to time out after:
PULSE_DNS	equ	-8	;A short down pulse (first half of a '1')
PULSE_DNL	equ	-16	;A long down pulse (first half of a '0')
PULSE_UPS	equ	-8	;A short up pulse (second half of a '0')
PULSE_UPL	equ	-16	;A long up pulse (second half of a '1')

			;RX_FLAGS:
SRQ_ON	equ	7	;Set when holding the line low for a service request
RX_RST	equ	6	;Set when the receiver has detected a reset condition
RX_CMD	equ	5	;Set when ADB_CMD contains a command byte
RX_DATA	equ	4	;Set when ADB_BUF-ADB_BUF8 contain a data packet
RX_ABRT	equ	3	;Set when a data packet has been aborted
RX_SIZ2	equ	2	;Number of bytes received minus one
RX_SIZ1	equ	1	; "
RX_SIZ0	equ	0	; "

			;TX_FLAGS:
TX_RDY	equ	7	;Set when data is set up to transmit
TX_ON	equ	6	;Set when transmitter is running
TX_COL	equ	5	;Set when transmitter detected a collision
TX_SIZ2	equ	2	;Number of bytes to be transmitted minus one
TX_SIZ1	equ	1	; "
TX_SIZ0	equ	0	; "

			;COLLISN:
COL_KBD	equ	7	;Set when there was a collision on keyboard's address
COL_MSE	equ	6	;Set when there was a collision on mouse's address

			;U_FLAGS:
U_HEADR	equ	7	;Set when we've received and are working a header byte
U_FWD	equ	6	;Set when we're just forwarding (not queuing) bytes
U_CNTR3	equ	3	;Counter of bytes to forward or queue
U_CNTR2	equ	2	; "
U_CNTR1	equ	1	; "
U_CNTR0	equ	0	; "

			;KBD_2_H:
K2H_DEL	equ	6	;Delete
K2H_CAP	equ	5	;CapsLock
K2H_RST	equ	4	;Reset
K2H_CTL	equ	3	;Control
K2H_SHF	equ	2	;Shift
K2H_OPT	equ	1	;Option
K2H_CMD	equ	0	;Command

			;KBD_2_L:
K2L_CLR	equ	7	;Clear
K2L_SLK	equ	6	;ScrollLock

			;KBD_MODS:
KMD_LCT	equ	7	;Left Control
KMD_RCT	equ	6	;Right Control
KMD_LSH	equ	5	;Left Shift
KMD_RSH	equ	4	;Right Shift
KMD_LOP	equ	3	;Left Option
KMD_ROP	equ	2	;Right Option


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	RX_FLAGS	;Receiver flags
	TX_FLAGS	;Transmitter flags
	COLLISN		;Collision flags
	U_FLAGS		;UART flags
	RX_TDOWN	;Time spent down in the last down-up transition
	RX_TUP		;Time spent up in the last down-up transition
	ADB_PTR		;Receiver/transmitter state machine pointer
	ADB_CMD		;Receiver command buffer
	ADB_BUF		;Receiver/transmitter buffer
	ADB_BUF2	;2nd byte of transmitter buffer
	ADB_BUF3	;3rd byte of transmitter buffer
	ADB_BUF4	;4th byte of transmitter buffer
	ADB_BUF5	;5th byte of transmitter buffer
	ADB_BUF6	;6th byte of transmitter buffer
	ADB_BUF7	;7th byte of transmitter buffer
	ADB_BUF8	;8th byte of transmitter buffer
	
	endc

	cblock	0xD0	;Upper half of bank 1 registers
	
	KBD_2_H		;Keyboard register 2 high byte
	KBD_2_L		;Keyboard register 2 low byte
	KBD_MODS	;Keyboard modifier key state
	KBD_3_H		;Keyboard register 3 high byte
	KBD_3_L		;Keyboard register 3 low byte
	MSE_3_H		;Mouse register 3 high byte
	MSE_3_L		;Mouse register 3 low byte
	
	endc


;;; Vectors ;;;

	org	0x0		;Reset vector
	bra	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

Interrupt
	btfsc	RX_FLAGS,SRQ_ON	;If the SRQ_ON flag is set, branch into the
	bra	IntSrqDone	; logic to end the service request condition
	btfsc	TX_FLAGS,TX_ON	;If the TX_ON flag is set, branch into the
	bra	IntXmit		; transmitter logic
	;fall through

IntRecv
	movlb	0		;Quick as we can, capture the current value
	movf	TMR0,W		; from Timer0 and reset it
	clrf	TMR0		; "
	btfsc	INTCON,TMR0IF	;If Timer0 overflowed, set the captured value
	movlw	0xFF		; to 0xFF instead
	movlb	7		;If we aren't here because of an on-change
	btfss	IOCAF,5		; interrupt, we're here because of a timer
	bra	IntRecvTimeout	; overflow while the bus is high - a timeout
	btfss	IOCAN,5		;If the on-change interrupt is set to capture
	bra	IntRecvRising	; rising edges, branch into rising-edge logic
	;fall through

IntRecvFalling
	bcf	IOCAN,5		;Switch the on-change interrupt to capture
	bsf	IOCAP,5		; rising edges
	bcf	IOCAF,5		;Clear the on-change interrupt flag
	bcf	INTCON,TMR0IF	;Reset the Timer0 interrupt if it was set
	bcf	INTCON,TMR0IE	;Don't interrupt on bus timeout when low
	addlw	33		;Adjust the time the line spent floating high
	movwf	RX_TUP		; so it's zero-based and store it
	addwf	RX_TDOWN,W	;Check if the sum of the time spent up and the
	btfsc	STATUS,C	; time spent down is greater than 132 us; if it
	bra	IntRecvNotBit	; is, then it's out of range for a 1 or 0 bit
	addlw	-34		; cell and must be something else
	btfsc	STATUS,C	; "
	bra	IntRecvNotBit	; "
	;fall through

IntRecvBitCell
	addlw	34		;If it is 132 us or less, halve it to see what
	lsrf	WREG,W		; the threshold is to tell a 1 from a 0
	subwf	RX_TDOWN,W	;If the down time is less than the threshold,
	movlp	high RxFsa0	; point PCLATH to the state machine for
	btfss	STATUS,C	; receiving a 1, else a zero
	movlp	high RxFsa1	; "
	movf	ADB_PTR,W	;Jump into the appropriate state machine
	callw			; "
	movwf	ADB_PTR		; "
	retfie

IntRecvNotBit
	movf	RX_TDOWN,W	;Bucketize the time the line spent pulled down;
	movlp	high RxFsaR	; if none of these apply, treat as a reset
	addlw	-24	;232	;If <= 23 (70us +30%), treat as 0 (stop bit)
	btfss	STATUS,C	; "
	movlp	high RxFsa0	; "
	addlw	-75	;181	;If <= 98 (300us +30%), treat as service req
	btfss	STATUS,C	; "
	movlp	high RxFsaS	; "
	addlw	-108	;148	;If <= 206 (800us +3%), treat as attention
	btfss	STATUS,C	; "
	movlp	high RxFsaA	; "
	movf	ADB_PTR,W	;Jump into the appropriate state machine
	callw			; "
	movwf	ADB_PTR		; "
	retfie

IntRecvRising
	btfsc	TX_FLAGS,TX_RDY	;If the mainline code is ready to transmit,
	bra	IntXmitReady	; a rising edge is where we switch modes
	bcf	IOCAP,5		;Switch the on-change interrupt to capture
	bsf	IOCAN,5		; falling edges
	bcf	IOCAF,5		;Clear the on-change interrupt flag
	movwf	RX_TDOWN	;Store the time the line spent pulled low
	bcf	INTCON,TMR0IF	;Reset the Timer0 interrupt if it was set
	movlb	0		;Set Timer0 to time out after 132 us, just
	movlw	-33		; a bit longer than the maximum bit cell time,
	movwf	TMR0		; so we catch the end of a transaction quickly
	bsf	INTCON,TMR0IE	;Use Timer0 interrupt to detect bus timeout
	retfie

IntRecvTimeout
	bcf	INTCON,TMR0IE	;Only interrupt on bus timeout once
	movlp	high RxFsaT	;Jump into the timeout state machine
	movf	ADB_PTR,W	; "
	callw			; "
	movwf	ADB_PTR		; "
	retfie

IntSrqDone
	bcf	RX_FLAGS,SRQ_ON	;Clear the SRQ_ON flag so we don't return here
	bcf	INTCON,TMR0IF	;Clear the timer interrupt that brought us here
	bcf	INTCON,TMR0IE	;Disable timer interrupt as the line is low
	movlb	1		;Release the ADB line to end the service
	movlw	B'00100000'	; request and cause an on-change interrupt
	iorwf	TRISA,F		; "
	movlb	0		;Set Timer0 to 75 (300 us), same nominal value
	movlw	75		; as if the service request had been made by
	movwf	TMR0		; another device so receiver reads it as such
	retfie

IntXmitReady
	bcf	IOCAP,5		;Switch on-change interrupt to capture falling
	bsf	IOCAN,5		; edges (trigger on other devices pulling low)
	bcf	IOCAF,5		;Clear the interrupt that brought us here
	bcf	TX_FLAGS,TX_RDY	;Clear the TX_RDY flag and raise the TX_ON flag
	bsf	TX_FLAGS,TX_ON	; to indicate that we've switched modes
	clrf	ADB_PTR		;Clear state pointer now we've changed machines
	movlb	0		;Get a pseudorandom number between 0 and 15,
	movf	TMR1H,W		; turn it into a number between 199 and 214;
	xorwf	TMR1L,W		; that will make Timer0 overflow in between
	andlw	B'00001111'	; 168 us and 228 us, which is close enough to
	addlw	-57		; the specced range of 160 us to 240 us to wait
	movwf	TMR0		; before transmitting
	bcf	INTCON,TMR0IF	;Reset the Timer0 interrupt if it was set
	bsf	INTCON,TMR0IE	;Use Timer0 interrupt to cause transmit start
	retfie

IntXmit
	btfsc	INTCON,IOCIF	;If we're in the transmit logic because of IOC,
	bra	IntXmitCol	; pin changed under us and we have a collision
	movlb	1		;If not, we're here because of a timer
	movlw	B'00100000'	; interrupt that signifies we should toggle
	xorwf	TRISA,F		; whether we're pulling the ADB pin low
	btfss	TRISA,5		;If we're pulling the pin low, we can't detect
	bra	IntXmitNoCol	; a collision, so assume there isn't one
	movlb	0		;If we're letting the pin float but it's still
	btfsc	PORTA,5		; low, something else is driving it at the same
	bra	IntXmitNoCol	; time as us and there's a collision
	;fall through

IntXmitCol
	bsf	TX_FLAGS,TX_COL	;Raise the collision flag
	bcf	TX_FLAGS,TX_ON	;Reset everything into receiver mode as though
	clrf	ADB_PTR		; the line were idle; the receiver will ignore
	movlb	7		; the rest of the conflicting transmission and
	movlw	B'00100000'	; await the next attention or reset pulse
	movwf	IOCAN		; "
	clrf	IOCAP		; "
	clrf	IOCAF		; "
	clrf	RX_TDOWN	; "
	movlw	B'00001000'	; "
	movwf	INTCON		; "
	retfie

IntXmitNoCol
	movlb	0		;Call into the transmitter state machine
	movlp	high TxFsa	; "
	movf	ADB_PTR,W	; "
	callw			; "
	movwf	ADB_PTR		; "
	bcf	INTCON,TMR0IF	;Clear Timer0 interrupt that brought us here
	movlb	7		;Clear the on-change interrupt in case we
	bcf	IOCAF,5		; pulled the line low and caused one
	retfie


;;; Mainline ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON
	
	banksel	IOCAN		;RA5 sets IOCAN[5] on negative edge
	movlw	B'00100000'
	movwf	IOCAN
	
	banksel	RCSTA		;UART async mode, 115.2 kHz
	movlw	B'01001000'
	movwf	BAUDCON
	clrf	SPBRGH
	movlw	68
	movwf	SPBRGL
	movlw	B'00100110'
	movwf	TXSTA
	movlw	B'10010000'
	movwf	RCSTA
	
	banksel	OPTION_REG	;Timer0 uses instruction clock, 1:32 prescaler,
	movlw	B'01010100'	; thus ticking every 4 us; weak pull-ups on
	movwf	OPTION_REG
	
	banksel	T1CON		;Timer1 ticks with instruction clock
	movlw	B'00000001'
	movwf	T1CON
	
	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA
	
	banksel	LATA		;Ready to pull RA5-4 low when outputs
	movlw	B'00001111'
	movwf	LATA
	
	banksel	TRISA		;TX output, RX and unused pins inputs, RA5-4
	movlw	B'00111110'	; open-collector outputs, currently off
	movwf	TRISA
	
	movlb	1
	movlw	0x22		;Keyboard register 3 puts it at address 0x2,
	movwf	KBD_3_H		; with SRQ enabled and handler ID 2
	movlw	0x02
	movwf	KBD_3_L
	movlw	0x23		;Mouse register 3 puts it at address 0x3, with
	movwf	MSE_3_H		; SRQ enabled and handler ID 1
	movlw	0x01
	movwf	MSE_3_L
	
	movlw	0x20		;Set up UART receiver queue (0x2000-0x207F),
	movwf	FSR0H		; for which FSRs are push (FSR0) and pop (FSR1)
	movwf	FSR1H		; pointers
	clrf	FSR0L
	clrf	FSR1L
	
	clrf	TX_FLAGS	;Particularly important that these are zero
	clrf	RX_FLAGS
	clrf	COLLISN
	clrf	U_FLAGS
	clrf	ADB_PTR
	
	movlw	B'10001000'	;On-change interrupt and interrupt subsystem on
	movwf	INTCON
	
	bra	Main

GotUartByte
	btfss	U_FLAGS,U_HEADR	;If we're awaiting a header byte, this is it
	bra	GUNewHeader	; "
	btfsc	U_FLAGS,U_FWD	;If we're forwarding bytes, forward this one,
	bra	GUForwardByte	; else queue it
	;fall through

GUQueueByte
	movlb	3		;Push the byte that came in over the UART onto
	movf	RCREG,W		; the receiver queue
	movwi	FSR0++		; "
	bcf	FSR0L,7		; "
	decf	U_FLAGS,F	;Decrement the counter in the UART flags
	movf	U_FLAGS,W	;If the counter has reached zero, lower all
	andlw	B'00001111'	; flags and await a new header byte
	btfsc	STATUS,Z	; "
	clrf	U_FLAGS		; "
	bra	Main

GUForwardByte
	movlb	3		;Retransmit the byte that came in over the UART
	movf	RCREG,W		; "
	movwf	TXREG		; "
	decf	U_FLAGS,F	;Decrement the counter in the UART flags
	movf	U_FLAGS,W	;If the counter has reached zero, lower all
	andlw	B'00001111'	; flags and await a new header byte
	btfsc	STATUS,Z	; "
	clrf	U_FLAGS		; "
	bra	Main

GUNewHeader
	movlb	3		;Grab the byte that came in over the UART
	movf	RCREG,W		; "
	btfsc	WREG,7		;If any of its address bits are set, then we
	bra	GUForwardHeader	; need to forward it
	btfsc	WREG,6		; "
	bra	GUForwardHeader	; "
	btfsc	WREG,5		; "
	bra	GUForwardHeader	; "
	btfsc	WREG,4		; "
	bra	GUForwardHeader	; "
	movwi	FSR0++		;Its address counter is zero, so push it onto
	bcf	FSR0L,7		; the receiver queue as is
	andlw	B'00000111'	;Mask off its counter, increment it by one to
	incf	WREG,W		; adjust it to be a normal countdown of bytes,
	iorlw	B'10000000'	; set the have-header flag, and store this as
	movwf	U_FLAGS		; the new value of UART flags register
	bra	Main

GUForwardHeader
	swapf	WREG,W		;Decrement the address counter so we can send
	decf	WREG,W		; this packet forward to its intended
	swapf	WREG,W		; recipient
	movlb	3		;Send the header byte forward with decremented
	movwf	TXREG		; address counter
	andlw	B'00000111'	;Mask off its counter, increment it by one to
	incf	WREG,W		; adjust it to be a normal countdown of bytes,
	iorlw	B'11000000'	; set the have-header and forward flags, and
	movwf	U_FLAGS		; store this as the new value of UART flags reg
	bra	Main

GotReset
	movlb	1
	movlw	0x22		;Keyboard register 3 puts it at address 0x2,
	movwf	KBD_3_H		; with SRQ enabled and handler ID 2
	movlw	0x02
	movwf	KBD_3_L
	movlw	0x23		;Mouse register 3 puts it at address 0x3, with
	movwf	MSE_3_H		; SRQ enabled and handler ID 1
	movlw	0x01
	movwf	MSE_3_L
	bcf	RX_FLAGS,RX_RST
	bra	Main

GotCmd
	bcf	RX_FLAGS,RX_CMD	;Clear the command flag
	movf	ADB_CMD,W	;If the low four bits of the command are zeroes
	andlw	B'00001111'	; then this is a SendReset command and should
	btfsc	STATUS,Z	; be treated the same as a long reset pulse
	bra	GotReset	; "
	andlw	B'00001100'	;If bits 3-2 of the command are 0b11, this is
	xorlw	B'00001100'	; a talk command; otherwise, it's a flush which
	btfss	STATUS,Z	; we ignore or a listen which we'll handle when
	bra	Main		; we get the data that comes along with it
	movlb	1		;Compare the address of the talk command with
	swapf	ADB_CMD,W	; the stored address for the keyboard; if they
	xorwf	KBD_3_H,W	; match, this is a talk command for the
	andlw	B'00001111'	; keyboard
	btfsc	STATUS,Z	; "
	bra	KeyboardTalk	; "
	swapf	ADB_CMD,W	;Compare the address of the talk command with
	xorwf	MSE_3_H,W	; the stored address for the mouse; if they
	andlw	B'00001111'	; match, this is a talk command for the mouse
	btfsc	STATUS,Z	; "
	bra	MouseTalk	; "
	movf	FSR0L,W		;If the UART receiver queue is not empty, we
	xorwf	FSR1L,W		; need our devices to be polled, so send a
	btfss	STATUS,Z	; service request
	call	ServiceRequest	; "
	bra	Main

Main
	movlb	0
	btfsc	PIR1,RCIF
	bra	GotUartByte
	btfsc	RX_FLAGS,RX_RST
	bra	GotReset
	btfsc	RX_FLAGS,RX_CMD
	bra	GotCmd
	btfsc	RX_FLAGS,RX_DATA
	bra	GotData
	btfsc	RX_FLAGS,RX_ABRT
	bra	GotDataEnd
	btfsc	TX_FLAGS,TX_COL
	bra	GotCollision
	bra	Main

KeyboardTalk
	movf	ADB_CMD,W	;If this is a talk register 0, handle that
	andlw	B'00000011'	; "
	btfsc	STATUS,Z	; "
	bra	KeyboardTalk0	; "
	addlw	-1		;If this is a talk register 1, we have no
	btfsc	STATUS,Z	; handler for that, so done
	bra	Main		; "
	addlw	-1		;If this is a talk register 2, handle that
	btfsc	STATUS,Z	; "
	bra	KeyboardTalk2	; "
	bra	KeyboardTalk3	;Else it's a talk register 3, handle that

KeyboardTalk0
	movf	FSR0L,W		;If the UART receiver queue is empty, we have
	xorwf	FSR1L,W		; no data for either mouse or keyboard, so bail
	btfsc	STATUS,Z	; "
	bra	Main		; "
	btfsc	INDF1,3		;If the byte on the top of the queue has bit 3
	call	ServiceRequest	; set (mouse data) effect a service request
	btfsc	INDF1,3		; condition and send no data so the computer
	bra	Main		; polls the mouse instead
	;fall through

Talk0FromQueue
	movlw	B'11111000'	;Zero the counter of bytes to transmit
	andwf	TX_FLAGS,F	; "
	moviw	FSR1++		;Pop the control byte off the top of the queue
	bcf	FSR1L,7		; and keep only its bottom three bits (length
	andlw	B'00000111'	; of transmission minus one)
	iorwf	TX_FLAGS,F	;Copy its length field into the transmit count
	movwf	ADB_BUF8	; and use the last buffer byte as a counter
	moviw	FSR1++		;Move the first data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF		; "
	bsf	TX_FLAGS,TX_RDY	;Signal that we're ready to transmit
	movf	ADB_BUF8,F	;If the byte counter is zero, then we only had
	btfsc	STATUS,Z	; one byte of data (this shouldn't happen) to
	bra	Main		; transmit and we're done
	moviw	FSR1++		;Move the second data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF2	; "
	decf	ADB_BUF8,F	;Decrement the byte counter; if it is now zero,
	btfsc	STATUS,Z	; we had two bytes of data to transmit and
	bra	Main		; we're done
	moviw	FSR1++		;Move the third data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF3	; "
	decf	ADB_BUF8,F	;Decrement the byte counter; if it is now zero,
	btfsc	STATUS,Z	; we had three bytes of data to transmit and
	bra	Main		; we're done
	moviw	FSR1++		;Move the fourth data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF4	; "
	decf	ADB_BUF8,F	;Decrement the byte counter; if it is now zero,
	btfsc	STATUS,Z	; we had four bytes of data to transmit and
	bra	Main		; we're done
	moviw	FSR1++		;Move the fifth data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF5	; "
	decf	ADB_BUF8,F	;Decrement the byte counter; if it is now zero,
	btfsc	STATUS,Z	; we had five bytes of data to transmit and
	bra	Main		; we're done
	moviw	FSR1++		;Move the sixth data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF6	; "
	decf	ADB_BUF8,F	;Decrement the byte counter; if it is now zero,
	btfsc	STATUS,Z	; we had six bytes of data to transmit and
	bra	Main		; we're done
	moviw	FSR1++		;Move the seventh data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF7	; "
	decf	ADB_BUF8,F	;Decrement the byte counter; if it is now zero,
	btfsc	STATUS,Z	; we had seven bytes of data to transmit and
	bra	Main		; we're done
	moviw	FSR1++		;Move the eighth data byte from the queue into
	bcf	FSR1L,7		; the ADB buffer
	movwf	ADB_BUF8	; "
	bra	Main		;Can't have more than eight, so we're done

KeyboardTalk2
	movlb	1		;Move the contents of keyboard register 2 into
	movf	KBD_2_H,W	; the ADB buffer and signal the transmitter
	movwf	ADB_BUF		; that we're ready to transmit two bytes
	bsf	TX_FLAGS,TX_RDY	; "
	bcf	TX_FLAGS,TX_SIZ2; "
	bcf	TX_FLAGS,TX_SIZ1; "
	bsf	TX_FLAGS,TX_SIZ0; "
	movf	KBD_2_L,W	; "
	movwf	ADB_BUF2	; "
	bra	Main

KeyboardTalk3
	movlb	1		;Move the high byte of keyboard register 3 into
	movf	KBD_3_H,W	; the ADB buffer with its low nibble masked off
	andlw	B'11110000'	; "
	movwf	ADB_BUF		; "
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the ADB buffer; this
	xorwf	TMR1L,W		; way we replace address (which the host
	andlw	B'00001111'	; already knows) with a random number, which
	iorwf	ADB_BUF,F	; helps with collision detection
	bsf	TX_FLAGS,TX_RDY	;Signal the transmitter that we're ready to
	bcf	TX_FLAGS,TX_SIZ2; transmit two bytes
	bcf	TX_FLAGS,TX_SIZ1; "
	bsf	TX_FLAGS,TX_SIZ0; "
	movlb	1		;Move the low byte of keyboard register 3 into
	movf	KBD_3_L,W	; the ADB buffer
	movwf	ADB_BUF2	; "
	bra	Main

MouseTalk
	movf	ADB_CMD,W	;If this is a talk register 0, handle that
	andlw	B'00000011'	; "
	btfsc	STATUS,Z	; "
	bra	MouseTalk0	; "
	addlw	-1		;If this is a talk register 1, we have no
	btfsc	STATUS,Z	; handler for that, so done
	bra	Main		; "
	addlw	-1		;If this is a talk register 2, we have no
	btfsc	STATUS,Z	; handler for that, so done
	bra	Main		; "
	bra	MouseTalk3	;Else it's a talk register 3, handle that

MouseTalk0
	movf	FSR0L,W		;If the UART receiver queue is empty, we have
	xorwf	FSR1L,W		; no data for either mouse or keyboard, so bail
	btfsc	STATUS,Z	; "
	bra	Main		; "
	btfss	INDF1,3		;If the byte on the top of the queue has bit 3
	call	ServiceRequest	; clear (keyboard data) effect a service
	btfss	INDF1,3		; request condition and send no data so the
	bra	Main		; computer polls the keyboard instead
	bra	Talk0FromQueue

MouseTalk3
	movlb	1		;Move the high byte of mouse register 3 into
	movf	MSE_3_H,W	; the ADB buffer with its low nibble masked off
	andlw	B'11110000'	; "
	movwf	ADB_BUF		; "
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the ADB buffer; this
	xorwf	TMR1L,W		; way we replace address (which the host
	andlw	B'00001111'	; already knows) with a random number, which
	iorwf	ADB_BUF,F	; helps with collision detection
	bsf	TX_FLAGS,TX_RDY	;Signal the transmitter that we're ready to
	bcf	TX_FLAGS,TX_SIZ2; transmit two bytes
	bcf	TX_FLAGS,TX_SIZ1; "
	bsf	TX_FLAGS,TX_SIZ0; "
	movlb	1		;Move the low byte of mouse register 3 into the
	movf	MSE_3_L,W	; ADB buffer
	movwf	ADB_BUF2	; "
	bra	Main

GotData
	bcf	RX_FLAGS,RX_DATA;Clear the data flag
	movf	ADB_CMD,W	;If bits 3-2 of the command are 0b10, this is
	andlw	B'00001100'	; a listen command and we've now got the data
	xorlw	B'00001000'	; for it; it should be if we're here, but check
	btfss	STATUS,Z	; just in case
	bra	Main		; "
	movlb	1		;Compare the address of the listen command with
	swapf	ADB_CMD,W	; the stored address for the keyboard; if they
	xorwf	KBD_3_H,W	; match, this is a talk command for the
	andlw	B'00001111'	; keyboard
	btfsc	STATUS,Z	; "
	bra	KeyboardListen	; "
	swapf	ADB_CMD,W	;Compare the address of the listen command with
	xorwf	MSE_3_H,W	; the stored address for the mouse; if they
	andlw	B'00001111'	; match, this is a talk command for the mouse
	btfsc	STATUS,Z	; "
	bra	MouseListen	; "
	bra	Main		;Else, this is not for one of our devices

KeyboardListen
	movf	ADB_CMD,W	;We only care about listen commands for
	andlw	B'00000011'	; register 3, so if this isn't one of those,
	xorlw	B'00000011'	; we're done
	btfss	STATUS,Z	; "
	bra	Main		; "
	movf	ADB_BUF2,W	;If the low byte of the listen register 3 data
	btfsc	STATUS,Z	; is 0x00, handle this
	bra	KeyboardL3_00	; "
	addlw	2		;If the low byte of the listen register 3 data
	btfsc	STATUS,Z	; is 0xFE, handle this
	bra	KeyboardL3_FE	; "
	addlw	251		;If the low byte of the listen register 3 data
	btfsc	STATUS,Z	; is 0x02 or 0x03, these are handler IDs that
	bra	KeyboardL3ChgH	; we (as an extended keyboard) understand, so
	addlw	1		; change our handler ID accordingly
	btfsc	STATUS,Z	; "
	bra	KeyboardL3ChgH	; "
	bra	Main

KeyboardL3_00
	movf	ADB_BUF,W	;Change the address and flags in register 3
	movlb	1		; according to the first (high) data byte
	movwf	KBD_3_H		; "
	bra	Main

KeyboardL3_FE
	btfsc	COLLISN,COL_KBD	;If there was a collision, do not change the
	bra	KL3FENo		; address
	movlw	B'00001111'	;Snuff the top nibble of the first data byte,
	andwf	ADB_BUF,F	; we don't care about those bits
	movlb	1		;Copy the address from the first (high) data
	movf	KBD_3_H,W	; byte into keyboard register 3
	andlw	B'11110000'	; "
	iorwf	ADB_BUF,W	; "
	movwf	KBD_3_H		; "
KL3FENo	bcf	COLLISN,COL_KBD	;Clear the collision flag if it was set
	bra	Main

KeyboardL3ChgH
	movf	ADB_BUF2,W	;Change our handler ID (the low byte of
	movlb	1		; register 3) to the byte given in the listen
	movwf	KBD_3_L		; register 3 command
	bra	Main

MouseListen
	movf	ADB_CMD,W	;We only care about listen commands for
	andlw	B'00000011'	; register 3, so if this isn't one of those,
	xorlw	B'00000011'	; we're done
	btfss	STATUS,Z	; "
	bra	Main		; "
	movf	ADB_BUF2,W	;If the low byte of the listen register 3 data
	btfsc	STATUS,Z	; is 0x00, handle this
	bra	MouseL3_00	; "
	addlw	2		;If the low byte of the listen register 3 data
	btfsc	STATUS,Z	; is 0xFE, handle this
	bra	MouseL3_FE	; "
	bra	Main

MouseL3_00
	movf	ADB_BUF,W	;Change the address and flags in register 3
	movlb	1		; according to the first (high) data byte
	movwf	MSE_3_H		; "
	bra	Main

MouseL3_FE
	btfsc	COLLISN,COL_MSE	;If there was a collision, do not change the
	bra	ML3FENo		; address
	movlw	B'00001111'	;Snuff the top nibble of the first data byte,
	andwf	ADB_BUF,F	; we don't care about those bits
	movlb	1		;Copy the address from the first (high) data
	movf	MSE_3_H,W	; byte into keyboard register 3
	andlw	B'11110000'	; "
	iorwf	ADB_BUF,W	; "
	movwf	MSE_3_H		; "
ML3FENo	bcf	COLLISN,COL_MSE	;Clear the collision flag if it was set
	bra	Main

GotCollision
	bcf	TX_FLAGS,TX_COL	;Clear the collision flag
	movlb	1		;Compare the address of the last command with
	swapf	ADB_CMD,W	; the stored address for the keyboard; if they
	xorwf	KBD_3_H,W	; match, this is a collision on the keyboard's
	andlw	B'00001111'	; address
	btfsc	STATUS,Z	; "
	bsf	COLLISN,COL_KBD	; "
	swapf	ADB_CMD,W	;Compare the address of the last command with
	xorwf	MSE_3_H,W	; the stored address for the mouse; if they
	andlw	B'00001111'	; match, this is a collision on the mouse's
	btfsc	STATUS,Z	; address
	bsf	COLLISN,COL_MSE	; "
	bra	Main

GotDataEnd
	bcf	RX_FLAGS,RX_ABRT
	bra	Main


;;; Subprograms ;;;

ServiceRequest
	movlb	0		;Set Timer0 to overflow after 300 us, the
	movlw	-75		; nominal service request time
	movwf	TMR0		; "
	bsf	RX_FLAGS,SRQ_ON	;Signal that we're making a service request
	bcf	INTCON,TMR0IF	;Enable the Timer0 interrupt so we end the
	bsf	INTCON,TMR0IE	; service request after the timer overflows
	movlb	1		;Pull the ADB line low
	movlw	B'11011111'	; "
	andwf	TRISA,F		; "
	return

Update2KeyDown
	addlw	-51		;0x33 Delete/Backspace
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_2_H,K2H_DEL	; mark key as down
	addlw	-3		;0x36 LeftControl
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_MODS,KMD_LCT; mark key as down
	addlw	-1		;0x37 Command
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_2_H,K2H_CMD	; mark key as down
	addlw	-1		;0x38 LeftShift
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_MODS,KMD_LSH; mark key as down
	addlw	-1		;0x39 CapsLock
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_2_H,K2H_CAP	; mark key as down
	addlw	-1		;0x3A LeftOption
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_MODS,KMD_LOP; mark key as down
	addlw	-13		;0x47 Clear/NumLock
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_2_L,K2L_CLR	; mark key as down
	addlw	-36		;0x6B F14/ScrollLock
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_2_L,K2L_SLK	; mark key as down
	addlw	-16		;0x7B RightShift
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_MODS,KMD_RSH; mark key as down
	addlw	-1		;0x7C RightOption
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_MODS,KMD_ROP; mark key as down
	addlw	-1		;0x7D RightControl
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_MODS,KMD_RCT; mark key as down
	addlw	-2		;0x7F Reset
	btfsc	STATUS,Z	; if keycode matches,
	bcf	KBD_2_H,K2H_RST	; mark key as down
	return

Update2KeyUp
	andlw	B'01111111'	;Clear MSB so we can check released key's code
	addlw	-51		;0x33 Delete/Backspace
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_2_H,K2H_DEL	; mark key as up
	addlw	-3		;0x36 LeftControl
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_MODS,KMD_LCT; mark key as up
	addlw	-1		;0x37 Command
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_2_H,K2H_CMD	; mark key as up
	addlw	-1		;0x38 LeftShift
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_MODS,KMD_LSH; mark key as up
	addlw	-1		;0x39 CapsLock
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_2_H,K2H_CAP	; mark key as up
	addlw	-1		;0x3A LeftOption
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_MODS,KMD_LOP; mark key as up
	addlw	-13		;0x47 Clear/NumLock
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_2_L,K2L_CLR	; mark key as up
	addlw	-36		;0x6B F14/ScrollLock
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_2_L,K2L_SLK	; mark key as up
	addlw	-16		;0x7B RightShift
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_MODS,KMD_RSH; mark key as up
	addlw	-1		;0x7C RightOption
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_MODS,KMD_ROP; mark key as up
	addlw	-1		;0x7D RightControl
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_MODS,KMD_RCT; mark key as up
	addlw	-2		;0x7F Reset
	btfsc	STATUS,Z	; if keycode matches,
	bsf	KBD_2_H,K2H_RST	; mark key as up
	return

Update2Mods
	bsf	KBD_2_H,K2H_CTL	;If either control key is down, reflect that in
	btfsc	KBD_MODS,KMD_LCT; the appropriate bits in keyboard register 2
	btfss	KBD_MODS,KMD_RCT; "
	bcf	KBD_2_H,K2H_CTL	; "
	bsf	KBD_2_H,K2H_SHF	;If either shift key is down, reflect that in
	btfsc	KBD_MODS,KMD_LSH; the appropriate bits in keyboard register 2
	btfss	KBD_MODS,KMD_RSH; "
	bcf	KBD_2_H,K2H_SHF	; "
	bsf	KBD_2_H,K2H_OPT	;If either option key is down, reflect that in
	btfsc	KBD_MODS,KMD_LOP; the appropriate bits in keyboard register 2
	btfss	KBD_MODS,KMD_ROP; "
	bcf	KBD_2_H,K2H_OPT	; "
	return


;;; State Machines ;;;

TxFsa	org	0x900

TStartU	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TStartD
TStartD	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData7D
TData7D	movlw	PULSE_DNL
	btfsc	ADB_BUF,7
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData7U
TData7U	movlw	PULSE_UPS
	btfsc	ADB_BUF,7
	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData6D
TData6D	movlw	PULSE_DNL
	btfsc	ADB_BUF,6
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData6U
TData6U	movlw	PULSE_UPS
	btfsc	ADB_BUF,6
	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData5D
TData5D	movlw	PULSE_DNL
	btfsc	ADB_BUF,5
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData5U
TData5U	movlw	PULSE_UPS
	btfsc	ADB_BUF,5
	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData4D
TData4D	movlw	PULSE_DNL
	btfsc	ADB_BUF,4
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData4U
TData4U	movlw	PULSE_UPS
	btfsc	ADB_BUF,4
	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData3D
TData3D	movlw	PULSE_DNL
	btfsc	ADB_BUF,3
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData3U
TData3U	movlw	PULSE_UPS
	btfsc	ADB_BUF,3
	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData2D
TData2D	movlw	PULSE_DNL
	btfsc	ADB_BUF,2
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData2U
TData2U	movlw	PULSE_UPS
	btfsc	ADB_BUF,2
	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData1D
TData1D	movlw	PULSE_DNL
	btfsc	ADB_BUF,1
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData1U
TData1U	movlw	PULSE_UPS
	btfsc	ADB_BUF,1
	movlw	PULSE_UPL
	movwf	TMR0
	retlw	low TData0D
TData0D	movlw	PULSE_DNL
	btfsc	ADB_BUF,0
	movlw	PULSE_DNS
	movwf	TMR0
	retlw	low TData0U
TData0U	movlw	PULSE_UPS
	btfsc	ADB_BUF,0
	movlw	PULSE_UPL
	movwf	TMR0
	movf	ADB_BUF2,W	;Rotate the next byte into position, if there
	movwf	ADB_BUF		; is one
	movf	ADB_BUF3,W	; "
	movwf	ADB_BUF2	; "
	movf	ADB_BUF4,W	; "
	movwf	ADB_BUF3	; "
	movf	ADB_BUF5,W	; "
	movwf	ADB_BUF4	; "
	movf	ADB_BUF6,W	; "
	movwf	ADB_BUF5	; "
	movf	ADB_BUF7,W	; "
	movwf	ADB_BUF6	; "
	movf	ADB_BUF8,W	; "
	movwf	ADB_BUF7	; "
	movf	TX_FLAGS,W	;If the counter in the three least significant
	andlw	B'00000111'	; bits of TX_FLAGS has hit zero, next move is
	btfsc	STATUS,Z	; to send a stop bit
	retlw	low TStopD	; "
	decf	TX_FLAGS,F	;Otherwise, decrement the counter and send
	retlw	low TData7D	; another byte
TStopD	movlw	PULSE_DNL
	movwf	TMR0
	retlw	low TStopU
TStopU	bcf	TX_FLAGS,TX_ON	;Reset everything into receiver mode
	movlb	7		; "
	movlw	B'00100000'	; "
	movwf	IOCAN		; "
	clrf	IOCAP		; "
	clrf	IOCAF		; "
	clrf	RX_TDOWN	; "
	movlw	B'00001000'	; "
	movwf	INTCON		; "
	retlw	low TStartU


RxFsa0	org	0xA00

R0idle	retlw	low R0idle
	nop
R0cmd7	bcf	ADB_CMD,7
	retlw	low R0cmd6
R0cmd6	bcf	ADB_CMD,6
	retlw	low R0cmd5
R0cmd5	bcf	ADB_CMD,5
	retlw	low R0cmd4
R0cmd4	bcf	ADB_CMD,4
	retlw	low R0cmd3
R0cmd3	bcf	ADB_CMD,3
	retlw	low R0cmd2
R0cmd2	bcf	ADB_CMD,2
	retlw	low R0cmd1
R0cmd1	bcf	ADB_CMD,1
	retlw	low R0cmd0
R0cmd0	bcf	ADB_CMD,0
	bsf	RX_FLAGS,RX_CMD
	retlw	low R0cmdP
R0cmdP	retlw	low R0dataS
	nop
R0dataS	retlw	low R0idle
	nop
	nop
R0data7	bcf	ADB_BUF8,7
	retlw	low R0data6
	nop
R0data6	bcf	ADB_BUF8,6
	retlw	low R0data5
	nop
R0data5	bcf	ADB_BUF8,5
	retlw	low R0data4
	nop
R0data4	bcf	ADB_BUF8,4
	retlw	low R0data3
	nop
R0data3	bcf	ADB_BUF8,3
	retlw	low R0data2
	nop
R0data2	bcf	ADB_BUF8,2
	retlw	low R0data1
	nop
R0data1	bcf	ADB_BUF8,1
	retlw	low R0data0
	nop
R0data0	bcf	ADB_BUF8,0
	movf	ADB_BUF8,W
R0stxxx	btfsc	RX_FLAGS,RX_SIZ2
	bra	R0st1xx
R0st0xx	btfsc	RX_FLAGS,RX_SIZ1
	bra	R0st01x
R0st00x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R0st001
R0st000	movwf	ADB_BUF
	retlw	low R0dataN
R0st001	movwf	ADB_BUF2
	retlw	low R0dataN
R0st01x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R0st011
R0st010	movwf	ADB_BUF3
	retlw	low R0dataN
R0st011 movwf	ADB_BUF4
	retlw	low R0dataN
R0st1xx	btfsc	RX_FLAGS,RX_SIZ1
	bra	R0st11x
R0st10x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R0st101
R0st100	movwf	ADB_BUF5
	retlw	low R0dataN
R0st101	movwf	ADB_BUF6
	retlw	low R0dataN
R0st11x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R0st111
R0st110	movwf	ADB_BUF7
	retlw	low R0dataN
R0st111	bsf	RX_FLAGS,RX_DATA
	retlw	low R0idle
R0dataN	incf	RX_FLAGS,F
	bcf	ADB_BUF8,7
	retlw	low R0data6


RxFsa1	org	0xB00

R1idle	retlw	low R1idle
	nop
R1cmd7	bsf	ADB_CMD,7
	retlw	low R1cmd6
R1cmd6	bsf	ADB_CMD,6
	retlw	low R1cmd5
R1cmd5	bsf	ADB_CMD,5
	retlw	low R1cmd4
R1cmd4	bsf	ADB_CMD,4
	retlw	low R1cmd3
R1cmd3	bsf	ADB_CMD,3
	retlw	low R1cmd2
R1cmd2	bsf	ADB_CMD,2
	retlw	low R1cmd1
R1cmd1	bsf	ADB_CMD,1
	retlw	low R1cmd0
R1cmd0	bsf	ADB_CMD,0
	bsf	RX_FLAGS,RX_CMD
	retlw	low R1cmdP
R1cmdP	retlw	low R1idle
	nop
R1dataS	movlw	B'11111000'
	andwf	RX_FLAGS,F
	retlw	low R1data7
R1data7	bsf	ADB_BUF8,7
	retlw	low R1data6
	nop
R1data6	bsf	ADB_BUF8,6
	retlw	low R1data5
	nop
R1data5	bsf	ADB_BUF8,5
	retlw	low R1data4
	nop
R1data4	bsf	ADB_BUF8,4
	retlw	low R1data3
	nop
R1data3	bsf	ADB_BUF8,3
	retlw	low R1data2
	nop
R1data2	bsf	ADB_BUF8,2
	retlw	low R1data1
	nop
R1data1	bsf	ADB_BUF8,1
	retlw	low R1data0
	nop
R1data0	bsf	ADB_BUF8,0
	movf	ADB_BUF8,W
R1stxxx	btfsc	RX_FLAGS,RX_SIZ2
	bra	R1st1xx
R1st0xx	btfsc	RX_FLAGS,RX_SIZ1
	bra	R1st01x
R1st00x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R1st001
R1st000	movwf	ADB_BUF
	retlw	low R1dataN
R1st001	movwf	ADB_BUF2
	retlw	low R1dataN
R1st01x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R1st011
R1st010	movwf	ADB_BUF3
	retlw	low R1dataN
R1st011 movwf	ADB_BUF4
	retlw	low R1dataN
R1st1xx	btfsc	RX_FLAGS,RX_SIZ1
	bra	R1st11x
R1st10x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R1st101
R1st100	movwf	ADB_BUF5
	retlw	low R1dataN
R1st101	movwf	ADB_BUF6
	retlw	low R1dataN
R1st11x	btfsc	RX_FLAGS,RX_SIZ0
	bra	R1st111
R1st110	movwf	ADB_BUF7
	retlw	low R1dataN
R1st111	bsf	RX_FLAGS,RX_DATA
	retlw	low R1idle
R1dataN	incf	RX_FLAGS,F
	bsf	ADB_BUF8,7
	retlw	low R1data6


RxFsaA	org	0xC00

RAidle	retlw	low RAcmd7
	nop
RAcmd7	retlw	low RAcmd7
	nop
RAcmd6	retlw	low RAcmd7
	nop
RAcmd5	retlw	low RAcmd7
	nop
RAcmd4	retlw	low RAcmd7
	nop
RAcmd3	retlw	low RAcmd7
	nop
RAcmd2	retlw	low RAcmd7
	nop
RAcmd1	retlw	low RAcmd7
	nop
RAcmd0	retlw	low RAcmd7
	nop
	nop
RAcmdP	retlw	low RAcmd7
	nop
RAdataS	retlw	low RAcmd7
	nop
	nop
RAdata7	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
RAdata6	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
RAdata5	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
RAdata4	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
RAdata3	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
RAdata2	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
RAdata1	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
RAdata0	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
RAdataN	bsf	RX_FLAGS,RX_ABRT
	retlw	low RAcmd7
	nop


RxFsaS	org	0xD00

RSidle	retlw	low RSidle
	nop
RScmd7	retlw	low RSidle
	nop
RScmd6	retlw	low RSidle
	nop
RScmd5	retlw	low RSidle
	nop
RScmd4	retlw	low RSidle
	nop
RScmd3	retlw	low RSidle
	nop
RScmd2	retlw	low RSidle
	nop
RScmd1	retlw	low RSidle
	nop
RScmd0	retlw	low RSidle
	nop
	nop
RScmdP	retlw	low RSdataS
	nop
RSdataS	retlw	low RSidle
	nop
	nop
RSdata7	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
RSdata6	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
RSdata5	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
RSdata4	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
RSdata3	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
RSdata2	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
RSdata1	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
RSdata0	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
RSdataN	bsf	RX_FLAGS,RX_ABRT
	retlw	low RSidle
	nop


RxFsaR	org	0xE00

RRidle	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd7	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd6	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd5	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd4	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd3	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd2	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd1	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRcmd0	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
	nop
RRcmdP	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdataS	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
	nop
RRdata7	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdata6	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdata5	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdata4	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdata3	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdata2	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdata1	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
RRdata0	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
RRdataN	bsf	RX_FLAGS,RX_ABRT
	bsf	RX_FLAGS,RX_RST
	retlw	low RRidle


RxFsaT	org	0xF00

RTidle	retlw	low RTidle
	nop
RTcmd7	retlw	low RTidle
	nop
RTcmd6	retlw	low RTidle
	nop
RTcmd5	retlw	low RTidle
	nop
RTcmd4	retlw	low RTidle
	nop
RTcmd3	retlw	low RTidle
	nop
RTcmd2	retlw	low RTidle
	nop
RTcmd1	retlw	low RTidle
	nop
RTcmd0	retlw	low RTidle
	nop
	nop
RTcmdP	retlw	low RTcmdP	;A timeout during 'Tlt' is not a timeout
	nop
RTdataS	retlw	low RTidle
	nop
	nop
RTdata7	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
RTdata6	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
RTdata5	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
RTdata4	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
RTdata3	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
RTdata2	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
RTdata1	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
RTdata0	bsf	RX_FLAGS,RX_ABRT
	retlw	low RTidle
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
RTdataN	bsf	RX_FLAGS,RX_DATA
	retlw	low RTidle
	nop


;;; End of Program ;;;
	end
