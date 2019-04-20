; *************************************************************************************************************
; *************************************************************************************************************
; $$\                                           $$$$$$\                       $$\                         $$\ 
; $$ |                                         $$  __$$\                      $$ |                        $$ |
; $$ |      $$$$$$\  $$$$$$$\   $$$$$$$\       $$ /  \__| $$$$$$\  $$$$$$$\ $$$$$$\    $$$$$$\   $$$$$$\  $$ |
; $$ |     $$  __$$\ $$  __$$\ $$  _____|      $$ |      $$  __$$\ $$  __$$\\_$$  _|  $$  __$$\ $$  __$$\ $$ |
; $$ |     $$$$$$$$ |$$ |  $$ |\$$$$$$\        $$ |      $$ /  $$ |$$ |  $$ | $$ |    $$ |  \__|$$ /  $$ |$$ |
; $$ |     $$   ____|$$ |  $$ | \____$$\       $$ |  $$\ $$ |  $$ |$$ |  $$ | $$ |$$\ $$ |      $$ |  $$ |$$ |
; $$$$$$$$\\$$$$$$$\ $$ |  $$ |$$$$$$$  |      \$$$$$$  |\$$$$$$  |$$ |  $$ | \$$$$  |$$ |      \$$$$$$  |$$ |
; \________|\_______|\__|  \__|\_______/        \______/  \______/ \__|  \__|  \____/ \__|       \______/ \__|
; **************************************************************************************************************
; **************************************************************************************************************
; ** Program:	 Lens Pelco-D Controller
; ** Device:	 Atmega48PA
; ** Frequency:  8 MHz
; ** Created:	 22 Nov 2018
; ** Author:	 Rampopula	  
; **************************************************************************************************************
; **************************************************************************************************************
; ****************************************** Macro definitions *************************************************
; **************************************************************************************************************
.macro LED_ON
	sbi port_led,pin_led
.endm 

.macro LED_OFF
	cbi port_led,pin_led
.endm 

.macro ZOOM_IN
	cbi port_zoom,pin_zoom_a
	sbi port_zoom,pin_zoom_b	
	rcall TIMER_Zoom_Start
	FOCUS_P
.endm

.macro ZOOM_OUT
	cbi port_zoom,pin_zoom_b
	sbi port_zoom,pin_zoom_a	
	rcall TIMER_Zoom_Start
	FOCUS_N
.endm

.macro ZOOM_FREE
	cbi port_zoom,pin_zoom_a
	cbi port_zoom,pin_zoom_b
	out TCCR0B,zero
	clr tim0_cmp_zoom	
.endm

.macro FOCUS_P
	cbi port_focus,pin_focus_b
	sbi port_focus,pin_focus_a
	rcall TIMER_Focus_Start
.endm

.macro FOCUS_N
	cbi port_focus,pin_focus_a
	sbi port_focus,pin_focus_b
	rcall TIMER_Focus_Start
.endm

.macro FOCUS_FREE
	cbi port_focus,pin_focus_a
	cbi port_focus,pin_focus_b
	sts TCCR2B,zero
	clr tim0_cmp_focus
.endm
; ************************************************************************************************
; ************************************************************************************************
.def zero					= R2
.def one					= R3
.def tim_focus_status		= R4
.def tim_zoom_status		= R5
.def temp_uart				= R16
.def temp_sreg				= R17
.def temp_2					= R18
.def temp_3					= R19
.def temp_4					= R20
.def counter				= R21
.def tim0_cmp_zoom			= R22
.def tim0_cmp_focus			= R23
.def temp_sreg2				= R24
.def temp_sreg3				= R25
; ************************************************************************************************
.equ BAUD					= 9600 
.equ XTAL					= 8000000 
.equ UBRRN					= XTAL/(16*BAUD)-1 
.equ free_time_zoom			= 60/2					; motor zoom enable time ms  ; 180 <-> 420
.equ free_time_focus		= 140/2					; motor focus enable time ms
.equ camera_id				= 0x02					; set camera id
.equ sync_byte				= 0xFF					; The synchronization byte, always FF
; ************************************************************************************************
.equ port_led				= PORTC
	.equ ddr_led			= port_led-1
		.equ pin_led		= 1
.equ port_focus				= PORTC
	.equ ddr_focus			= port_focus-1
		.equ pin_focus_a	= 3
		.equ pin_focus_b	= 2
.equ port_zoom				= PORTB
	.equ ddr_zoom			= port_zoom-1
		.equ pin_zoom_a		= 1
		.equ pin_zoom_b		= 2
.equ port_rs485				= PORTC
	.equ ddr_rs485			= port_rs485-1	
		.equ pin_rs485		= 5
; ************************************************************************************************
.dseg
	.org 0x0100			input_buffer: .byte 256

.cseg
	.org 0x0000			rjmp RESET
	.org OC2Aaddr       rjmp TIMER_FOCUS_COMPA
	.org OC0Aaddr       rjmp TIMER_ZOOM_COMPA
	.org URXCaddr		rjmp UART_RX_COMPLETE

	; ************* command list and descriptions *****************
	focus_near:			.db sync_byte,camera_id,0x01,0x00,0x00,0x00		; focus-
	zoom_tele:			.db sync_byte,camera_id,0x00,0x20,0x00,0x00		; zoom+	
	zoom_wide:			.db sync_byte,camera_id,0x00,0x40,0x00,0x00		; zoom-
	focus_far:			.db sync_byte,camera_id,0x00,0x80,0x00,0x00		; focus+	
	; **************************************************************
	focus_near_addr:	.db high(focus_near_act),low(focus_near_act)	; focus- label address
	zoom_tele_addr:		.db high(zoom_tele_act),low(zoom_tele_act)		; zoom+  label address
	zoom_wide_addr:		.db high(zoom_wide_act),low(zoom_wide_act)		; zoom-  label address
	focus_far_addr:		.db high(focus_far_act),low(focus_far_act)		; focus+ label address
	; **************************************************************

; ************************************************************************************************
; ************************************ Interrupt Handlers ****************************************
; ************************************************************************************************
TIMER_ZOOM_COMPA:
	in temp_sreg3,SREG
	inc tim0_cmp_zoom
	cpi tim0_cmp_zoom,free_time_zoom
		brne exit_zoom_compa
	ZOOM_FREE
	exit_zoom_compa:
	out SREG,temp_sreg3
reti

TIMER_FOCUS_COMPA:
	in temp_sreg2,SREG
	inc tim0_cmp_focus
	cpi tim0_cmp_focus,free_time_focus
		brne exit_focus_compa
	FOCUS_FREE
	exit_focus_compa:
	out SREG,temp_sreg2
reti

UART_RX_COMPLETE:
	in temp_sreg,SREG
	lds temp_uart,UDR0
	st X,temp_uart
	inc XL
	out SREG,temp_sreg	
reti
; ************************************************************************************************
; ****************************************** MCU Init ********************************************
; ************************************************************************************************
TIMER_Focus_Init:								
	ldi temp_2,(1<<WGM21)
	sts TCCR2A,temp_2
	ldi temp_2,(0<<CS22)|(0<<CS21)|(0<<CS20)
	sts TCCR2B,temp_2
	ldi temp_2,0xFA							; 2 ms						
	sts OCR2A,temp_2
	ldi temp_2,(1<<OCIE2A)
	sts TIMSK2,temp_2
	ldi temp_2,(0<<AS2)
	sts ASSR,temp_2
ret

TIMER_Focus_Start:
	ldi temp_2,(1<<OCF2A)
	out TIFR2,temp_2
	sts TCNT2,zero
	mov tim_focus_status,one
	ldi temp_2,(0<<CS22)|(1<<CS21)|(1<<CS20)
	sts TCCR2B,temp_2	
ret

TIMER_Focus_Stop:
	ldi temp_2,(0<<CS22)|(0<<CS21)|(1<<CS20)
	sts TCCR2B,temp_2
ret
; ************************************************************************************************
TIMER_Zoom_Init:								
	ldi temp_2,(1<<WGM01)
	out TCCR0A,temp_2
	ldi temp_2,(0<<CS22)|(0<<CS21)|(0<<CS20)
	out TCCR0B,temp_2
	ldi temp_2,0xFA							; 2 ms
	out OCR0A,temp_2
	ldi temp_2,(1<<OCIE0A)
	sts TIMSK0,temp_2
ret

TIMER_Zoom_Start:
	ldi temp_2,(1<<OCF0A)
	out TIFR0,temp_2
	out TCNT0,zero
	mov tim_zoom_status,one
	ldi temp_2,(0<<CS22)|(1<<CS21)|(1<<CS20)
	out TCCR0B,temp_2	
ret

TIMER_Zoom_Stop:
	ldi temp_2,(0<<CS00)|(0<<CS02)
	out TCCR0B,temp_2
ret
; ************************************************************************************************
USART_Init:
	ldi temp_2,low(UBRRN)
	sts UBRR0L,temp_2
	ldi temp_2,high(UBRRN)
	sts UBRR0H,temp_2
	ldi temp_2,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)
	sts UCSR0B,temp_2
	ldi temp_2,(1<<UCSZ01)|(1<<UCSZ00)
	sts UCSR0C,temp_2
ret

DDR_Init:
	sbi ddr_led,pin_led
	sbi ddr_focus,pin_focus_a
	sbi ddr_focus,pin_focus_b
	sbi ddr_zoom,pin_zoom_a
	sbi ddr_zoom,pin_zoom_b
	sbi ddr_rs485,pin_rs485
ret
; ************************************************************************************************
; *************************************** Main Program *******************************************
; ************************************************************************************************
RESET:
	out SREG, zero
	sts UCSR0B,zero
	sts UCSR0C,zero
	sts UBRR0L,zero
	sts UBRR0H,zero
	sts TIMSK0,zero
	out TCNT0,zero
	out TCCR0B,zero
	out TIFR0,zero
	out TCCR0B,zero
	out OCR0A,zero
	out SPMCSR,zero
	out SPL,zero
	out SPH,zero
	out DDRB,zero
	out PORTB,zero
	clr temp_2

	; register\sram flush
	ldi ZL,low(SRAM_START)
	ldi ZH,high(SRAM_START)	
	Flush_SRAM:
		st Z+,temp_2
		cpi ZH,high(RAMEND+1)
		brne Flush_SRAM
		cpi ZL,low(RAMEND+1)
		brne Flush_SRAM
		clr ZL
		clr ZH
		ldi ZL,30
		clr ZH
	Flush_Regs:
		dec ZL
		st Z,ZH
		brne Flush_Regs	
	ldi temp_2,low(RAMEND)
	out SPL,temp_2
	ldi temp_2,high(RAMEND)
	out SPH,temp_2
	inc one
	; *************************
	rcall USART_Init
	rcall TIMER_Zoom_Init
	rcall TIMER_Focus_Init
	rcall DDR_Init
	; *************************

	; input buffer init
	ldi XL,low(input_buffer)
	ldi XH,high(input_buffer)
	mov YL,XL
	mov YH,XH
	; *************************

	sei

; ************************************************************************************************
forever:
	cp XL,YL
		breq forever
	cp counter,zero
		brne start_compare
	; first compare command init
	ldi ZL,low(focus_near<<1)
	ldi ZH,high(focus_near<<1)
	; *************************
	start_compare:
		lpm temp_2,Z
		ld  temp_3,Y
		cp temp_2,temp_3
			brne compare_select
		inc YL
		cpi counter,5
			breq command_accepted
		inc ZL
		inc counter
		rjmp forever


	compare_select:
		cp counter,zero
			breq reset_ident
		cpi temp_4,3
			brsh reset_ident
		adiw Z,6
		inc temp_4
		rjmp start_compare

	reset_ident:
		inc YL
		clr counter
		clr temp_4
		rjmp forever

	command_accepted:
		ldi ZL,low(focus_near_addr<<1)
		ldi ZH,high(focus_near_addr<<1)
		cp temp_4,zero
			breq load_address
		loop_addr:
			adiw Z,2
			dec temp_4
			brne loop_addr
		load_address:
			lpm temp_2,Z+
			lpm temp_3,Z+
			mov ZH,temp_2
			mov ZL,temp_3
			clr counter
			clr temp_4
			ijmp
; ************************************************************************************************
	zoom_tele_act:
		ZOOM_IN
		rjmp forever

	zoom_wide_act:
		ZOOM_OUT	
		rjmp forever

	focus_far_act:
		FOCUS_P
		rjmp forever

	focus_near_act:
		FOCUS_N
		rjmp forever
; ************************************************************************************************
; ************************************************************************************************