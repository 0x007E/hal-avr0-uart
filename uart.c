/**
 * @file uart.c
 * @brief Source file with implementation of polling-based UART functions and macros.
 *
 * This file contains the definitions of function implementations for polling-based UART communication on AVR-0/1/2-Series microcontrollers. Supports configurable baud rates, frame formats, optional echo, and stdio integration (printf/scanf).
 * 
 * @author g.raf
 * @date 2025-12-07
 * @version 1.0 Release
 * @copyright
 * Copyright (c) 2025 g.raf
 * Released under the GPLv3 License. (see LICENSE in repository)
 *
 * @note This file is part of a larger project and subject to the license specified in the repository. For updates and the complete revision history, see the GitHub repository.
 * 
 * @important Interrupts are NOT implemented. Define UART_RXCIE/UART_TXCIE/UART_UDRIE macros to disable polling functions for custom interrupt handling.
 * 
 * @see uart.h for declarations, configuration macros, and related information.
 * @see uart_enums.h for UART status and error enumerations.
 * @see [https://github.com/0x007e/hal-avr0-mega](https://github.com/0x007e/hal-avr0-mega) "AVR ATmega GitHub Repository"
 */

#include "uart.h"

#if UART_STDMODE > 0
    // Initialize FILE stream
    #if !defined(UART_TXCIE) && !defined(UART_UDRIE) && !defined(UART_RXCIE) && UART_STDMODE == 1
        static FILE std_uart = FDEV_SETUP_STREAM(uart_printf, uart_scanf, _FDEV_SETUP_RW);
    #elif !defined(UART_TXCIE) && !defined(UART_UDRIE) && UART_STDMODE == 2
        static FILE std_uart = FDEV_SETUP_STREAM(uart_printf, NULL, _FDEV_SETUP_WRITE);
    #elif !defined(UART_RXCIE) && UART_STDMODE == 3
        static FILE std_uart = FDEV_SETUP_STREAM(NULL, uart_scanf, _FDEV_SETUP_READ);
    #endif
#endif

#if !defined(UART_RXCIE) && !defined(UART_TXCIE) && !defined(UART_UDRIE)
    #if UART_HANDSHAKE > 0
        static UART_Handshake uart_handshake_sending = UART_Ready;
    #endif
#endif

/**
 * @brief Initialize the UART hardware interface with configured parameters.
 *
 * @details
 * This function configures the USART0 peripheral for polling-based operation:
 * - Sets PORTMUX for alternate pin routing (if configured)
 * - Configures TX (output) and RX (input) pin directions
 * - Sets hardware handshake pins (RTS/CTS) if UART_HANDSHAKE==2
 * - Calculates and applies baud rate using UART_BAUD_RATE() macro
 * - Configures frame format: data bits, parity, stop bits
 * - Enables TX/RX with optional RXC echo and stdio stream assignment
 *
 * All configuration is derived from uart.h preprocessor macros.
 *
 * @note Call this function once during system initialization before using UART functions.
 * @note stdio streams (stdout/stdin) are assigned only when UART_STDMODE > 0 and no interrupts defined.
 */
void uart_init(void)
{
#ifdef TWI_PORTMUX
	#if !defined(__AVR_ATtiny202__) && !defined(__AVR_ATtiny402__)
	
		PORTMUX.CTRLB &= ~PORTMUX_USART0_bm;
		PORTMUX.CTRLB |= UART_PORTMUX;
	
	#elif defined(__AVR_ATmega3208__) || defined(__AVR_ATmega3209__) || defined(__AVR_ATmega4808__) || defined(__AVR_ATmega4809__)
	
		PORTMUX.USARTROUTEA &= ~PORTMUX_USART0_bm;
		PORTMUX.USARTROUTEA |= UART_PORTMUX;
	
	#else
		#error "PORTMUX: Invalid CPU!"
	#endif
#endif
    
    UART_PORT.DIRSET = UART_TX_PIN;
    UART_PORT.DIRCLR = UART_RX_PIN;
    
    // Check if hardware flow control is enabled
    #if UART_HANDSHAKE == 2
        // Setup RTS (output)/CTS (input)
        UART_HANDSHAKE_PORT.DIRSET = UART_HANDSHAKE_RTS_PIN;
        UART_HANDSHAKE_PORT.DIRCLR = UART_HANDSHAKE_CTS_PIN;
    #endif
    
    // Check which bit sampling mode should be activated
    #if UART_SAMPLE == 1
        USART0.CTRLB |= (UART_SAMPLE<<1);	// Setup 8 samples/bit
	#else
		USART0.CTRLB &= ~(UART_SAMPLE<<1);	// Setup 16 samples/bit
    #endif

    USART0.BAUD = (unsigned int)UART_BAUDRATE_CALCULATION(UART_BAUDRATE);
    USART0.CTRLC = UART_DATASIZE | UART_PARITY | UART_STOPBITS;

    // Interrupt control
    USART0.CTRLA = 0x00;
    
    // Enable transmitter/receiver
    USART0.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;

    // Receiver interrupt setup
    #ifdef UART_RXCIE
        USART0.CTRLA |= USART_RXCIE_bm;
    #endif

    // Transmitter interrupt setup
    #if defined(UART_TXCIE) && !defined(UART_UDRIE)
        USART0.CTRLA |= USART_TXCIE_bm;
    #endif

    // Transmitter interrupt setup
    #if !defined(UART_TXCIE) && defined(UART_UDRIE)
        USART0.CTRLA |= USART_DREIE_bm;
    #endif

    #if !defined(UART_TXCIE) && !defined(UART_UDRIE) && (UART_STDMODE == 1 || UART_STDMODE == 2)
        stdout = &std_uart;
    #endif
    
    #if !defined(UART_RXCIE) && UART_STDMODE == 1 || UART_STDMODE == 3
        stdin = &std_uart;
    #endif
}

/**
 * @brief Disable the UART hardware interface and reset configuration.
 *
 * @details
 * This function completely disables the USART0 peripheral by clearing TXEN/RXEN bits and all interrupt enables. Call before reconfiguring UART or entering power-save modes.
 */
void uart_disable(void)
{
    USART0.CTRLB &= ~(USART_TXEN_bm | USART_RXEN_bm);
    USART0.CTRLA &= ~(USART_RXCIE_bm | USART_TXCIE_bm | USART_DREIE_bm);
    USART0.STATUS = USART_TXCIF_bm | USART_RXSIF_bm | USART_ISFIF_bm;

#if UART_HANDSHAKE == 2
    UART_HANDSHAKE_PORT.DIRCLR = UART_HANDSHAKE_RTS_PIN | UART_HANDSHAKE_CTS_PIN;
#endif

#ifdef TWI_PORTMUX
	#if !defined(__AVR_ATtiny202__) && !defined(__AVR_ATtiny402__)

		PORTMUX.CTRLB &= ~PORTMUX_USART0_bm;

	#elif defined(__AVR_ATmega3208__) || defined(__AVR_ATmega3209__) || defined(__AVR_ATmega4808__) || defined(__AVR_ATmega4809__)

		PORTMUX.USARTROUTEA &= ~PORTMUX_USART0_bm;

	#else
		#error "PORTMUX: Invalid CPU!"
	#endif
#endif
}

#if !defined(UART_TXCIE) && !defined(UART_UDRIE)
    /**
     * @brief Transmit a single character via UART (blocking).
     *
     * @param data Character byte to transmit (0-255).
     * @return Always returns 0 (success indicator for stdio compatibility).
     *
     * @details
     * Polling implementation waits for DREIF (Data Register Empty) flag before writing to TXDATAL register. Blocks until transmission completes.
     *
     * @note Only available when no TX interrupts defined (UART_TXCIE/UART_UDRIE).
     */
    char uart_putchar(char data)
    {
        // Wait until last transmission completed
        while (!(USART0.STATUS & USART_DREIF_bm))
        {
            ;
        }
        USART0.TXDATAL = data; // Write data to transmission register
    
        return 0;   // Return that there was no fault
    }

    #if (UART_STDMODE == 1 || UART_STDMODE == 2)
        /**
         * @brief UART printf stream handler for stdout redirection.
         *
         * @param data Character to transmit.
         * @param stream FILE stream pointer (unused).
         * @return Result of uart_putchar().
         *
         * @details
         * Internal callback used by avr-libc fdevopen() for printf() redirection. Only compiled when UART_STDMODE == 1 or 2 (write support).
         */
        int uart_printf(char data, FILE *stream)
        {
            return uart_putchar(data);
        }
    #endif

#endif

#if !defined(UART_RXCIE)
    /**
     * @brief Non-blocking check for received UART data with error handling.
     *
     * @param[out] data Pointer to store received byte (valid only if UART_Received returned).
     * @return UART_Data status: UART_Empty, UART_Received, or UART_Fault.
     *
     * @details
     * Checks RXCIF flag and validates frame using uart_error_flags(). Handles XON/XOFF software handshake if enabled. Echoes received data if UART_RXC_ECHO defined.
     *
     * @note Does NOT block. Returns immediately with status.
     */
    UART_Data uart_scanchar(char *data)
    {
        // If data has been received
        if(USART0.STATUS & USART_RXCIF_bm)
        {
            // Check if an error during receiving data occurred
            if(uart_error_flags() != UART_None)
            {
                *data = 0;
                return UART_Fault;
            }
        
            #if UART_HANDSHAKE == 1
                if (*data == UART_HANDSHAKE_XON)
                {
                    uart_handshake_sending = UART_Ready;
                    return UART_Empty;
                }
                else if (*data == UART_HANDSHAKE_XOFF)
                {
                    uart_handshake_sending = UART_Pause;
                    return UART_Empty;
                }
            #endif
        
            *data = USART0.RXDATAL;
        
            #if defined(UART_RXC_ECHO) && !defined(UART_TXCIE) && !defined(UART_UDRIE)
                // Send echo of received data to UART
                uart_putchar(*data);
            #endif
        
            return UART_Received;
        }
        return UART_Empty;
    }

    /**
     * @brief Blocking receive single character via UART.
     *
     * @param[out] status Pointer to receive UART_Data status (UART_Received/UART_Fault).
     * @return Received character byte.
     *
     * @details
     * Loops calling uart_scanchar() until data available or error occurs. Status indicates if data valid (UART_Received) or error (UART_Fault).
     */
    char uart_getchar(UART_Data *status)
    {
        char data;
    
        // Wait until data has been received or an error occurred
        do
        {
            *status = uart_scanchar(&data);
        } while (*status == UART_Empty);
		
        return data;
    }

    #if (UART_STDMODE == 1 || UART_STDMODE == 3)
        /**
         * @brief UART scanf stream handler for stdin redirection.
         *
         * @param stream FILE stream pointer (unused).
         * @return Received character as int (for stdio compatibility).
         *
         * @details
         * Internal callback used by avr-libc fdevopen() for scanf() redirection. Only compiled when UART_STDMODE == 1 or 3 (read support).
         */
        int uart_scanf(FILE *stream)
        {
            UART_Data temp;
    
            return (int)uart_getchar(&temp);
        }

        /**
         * @brief Clear UART input stream errors and discard pending character.
         *
         * @details
         * Calls clearerr(stdin) and getchar() to reset stream state and discard any buffered input. Used to recover from scanf() failures.
         */
        void uart_clear(void)
        {
            clearerr(stdin);    // Clear error on stream
            getchar();          // Remove character from stream
        }

    /**
     * @brief Check and clear UART receive error flags.
     *
     * @return UART_Error code: UART_None, UART_Frame, UART_Overrun, or UART_Parity.
     *
     * @details
     * Reads RXDATAH error bits (FERR, BUFOVF, PERR) and clears by reading RXDATAL. Returns first detected error or UART_None if no errors.
     */
    UART_Error uart_error_flags(void)
    {
        // UART_Frame error
        if(USART0.RXDATAH & USART_FERR_bm)
        {
            USART0.RXDATAL;			// Clear UART data register
            return UART_Frame;		// Return NUL
        }
        // Data UART_Buffer overflow error
        else if(USART0.RXDATAH & USART_BUFOVF_bm)
        {
            USART0.RXDATAL;			// Clear UART data register
            return UART_Overrun;	// Return NUL
        }
        // UART_Parity error
        else if(USART0.RXDATAH & USART_PERR_bm)
        {
            USART0.RXDATAL;         // Clear UART data register
            return UART_Parity;		// Return NUL
        }
        return UART_None;
    }
    #endif
#endif

#if !defined(UART_RXCIE) && !defined(UART_TXCIE) && !defined(UART_UDRIE)
    #if UART_HANDSHAKE > 0
        /**
         * @brief Manage UART hardware/software flow control signaling.
         *
         * @param status UART_Handshake command: UART_Ready, UART_Pause, or UART_Status (query).
         * @return Current handshake state (for UART_Status query).
         *
         * @details
         * Controls flow control signals:
         * - UART_Ready: Send XON or assert RTS
         * - UART_Pause: Send XOFF or deassert RTS  
         * - UART_Status: Return remote CTS state or XON/XOFF status
         */
        UART_Handshake uart_handshake(UART_Handshake status)
        {
            if(status == UART_Ready)
            {
                #if UART_HANDSHAKE == 1
                    uart_putchar(UART_HANDSHAKE_XON);
                #elif UART_HANDSHAKE == 2
                    UART_HANDSHAKE_PORT &= ~(1<<UART_HANDSHAKE_RTS_PIN);
                #endif
            }
            else if(status == UART_Pause)
            {
                #if UART_HANDSHAKE == 1
                    uart_putchar(UART_HANDSHAKE_XOFF);
                #elif UART_HANDSHAKE == 2
                    UART_HANDSHAKE_PORT |= (1<<UART_HANDSHAKE_RTS_PIN);
                #endif
            }
            else
            {
                #if UART_HANDSHAKE == 1
                    return uart_handshake_sending;
                #elif UART_HANDSHAKE == 2
                    if (!(UART_HANDSHAKE_PORT.IN & (1<<UART_HANDSHAKE_CTS_PIN)))
                    {
                        return UART_Ready;
                    }
                    return UART_Pause;
                #endif
            }
            return UART_Status;
        }
    #endif
#endif