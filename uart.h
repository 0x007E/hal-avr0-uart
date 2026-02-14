/**
 * @file uart.h
 * @brief Header file with declarations and macros for hardware UART.
 * 
 * This file provides function prototypes, type definitions, and constants for
 * hardware-based UART communication on AVR0 microcontrollers.
 * 
 * @author g.raf
 * @date 2025-09-27
 * @version 1.0 Release
 * @copyright
 * Copyright (c) 2025 g.raf
 * Released under the GPLv3 License. (see LICENSE in repository)
 * 
 * @note This file mostly becomes part of larger projects and subject to the license specified in the repository. For updates and the complete revision history, see the GitHub repository.
 *
 * @see https://github.com/0x007e/hal-avr0-mega "AVR ATmega GitHub Repository"
 */

#ifndef UART_H_
#define UART_H_

    #ifndef F_PER
        /**
         * @def F_PER
         * @brief Peripheral clock frequency definition.
         *
         * @details
         * This macro defines the operating frequency of the microcontroller's peripheral clock in Hertz. It is used for timing calculations. The value should match the actual peripheral hardware clock frequency to ensure correct timing behavior.
         */
        #define F_PER 10000000UL
    #endif

    #ifndef UART_BAUDRATE
        /**
         * @def UART_BAUDRATE
         * @brief Default UART communication baud rate.
         *
         * @details
         * Sets the default serial communication speed to 9600 baud. Common values are 9600, 19200, 38400, 57600, 115200.
         *
         * @note Override this macro before including uart.h for different communication speeds.
         */
        #define UART_BAUDRATE 9600UL
    #endif

    #ifndef UART_SAMPLE	// Samples/bit
        /**
         * @def UART_SAMPLE
         * @brief UART oversampling rate (samples per bit).
         *
         * @details
         * Configures the number of samples taken per bit during reception:
         * - 0 = 16 samples/bit (higher noise immunity, default)
         * - 1 = 8 samples/bit (faster processing)
         *
         * Higher sampling improves noise rejection.
         *
         * @note Only values 0 or 1 are valid. Affects baud rate calculation.
         */
        #define UART_SAMPLE 0
    #endif

    #ifndef UART_BAUDRATE_CALCULATION
        /**
         * @def UART_BAUDRATE_CALCULATION
         * @brief Calculates UART baud rate value.
         *
         * @details
         * This macro computes the value for `UART` baudrate register used to set the bitrate on receiving and transmitting data on UART. The calculation is based on the peripheral clock frequency (`F_PER´), the selected sample rate (`UART_SAMPLE`) and the defined baudrate (`UART_BAUDRATE`).
         *
         * The formula accounts for AVR's asynchronous normal mode with U2X=0.
         * UART_SAMPLE selects between 16x (0) or 8x (1) oversampling.
         *
         * @param BAUD Target baud rate in bits per second.
         * @return Calculated baudrate register value (integer).
         */
        #define UART_BAUDRATE_CALCULATION(BAUD) (((float)F_PER * 64 / ((1<<(4UL - (UART_SAMPLE & 0x01))) * (float)BAUD)) + 0.5)
    #endif
	
    #ifndef UART_PORTMUX
        /**
         * @def SPI_PORTMUX
         * @brief Selects the alternate port location for SPI pins.
         *
         * @details
         * This macro configures which physical port pins are used for the SPI interface. It determines the pin mapping of SPI signals such as MOSI, MISO, SCK, and SS.
         *
         * The value can be set to predefined routing codes:
         * ATTINY:
         * - `PORTMUX_USART0_DEFAULT_gc`: Default mapping referenced in datasheet (e.g. ATTiny1606 -> PB[2:3]).
         * - `PORTMUX_USART0_ALTERNATE_gc`: Default mapping referenced in datasheet (e.g. ATTiny1606 -> PA[1:2]).
         * ATMEGA(4808):
         * - `PORTMUX_USART0_DEFAULT_gc`: Default mapping referenced in datasheet (e.g. ATmega4808 -> PA[3:0]).
         * - `PORTMUX_USART0_ALT1_gc`: Alternative 1 referenced in datasheet (e.g. ATmega4808 -> PA[7:4]).
         * - `PORTMUX_USART0_NONE_gc`: Signals not connected to any pins
         *
         * By default, `SPI_ALTERNATE_PORT` is set to `PORTMUX_SPI0_ALTERNATE_gc`. Override this macro prior to including the SPI module to select a different port mapping.
         *
         * @note Changing this affects the hardware SPI pin assignment and may require corresponding changes in the PCB or wiring.
         */
        #define UART_PORTMUX PORTMUX_USART0_DEFAULT_gc
    #endif

    #ifndef UART_PORT
        /**
         * @def UART_PORT
         * @brief PORT register for UART TX/RX pins.
         *
         * @details
         * Specifies the PORT register containing UART transmit (TX) and receive (RX) pins. Default is PORTB. Used for pin direction, output values, and input reading.
         *
         * @note Must match the physical pin connections on your hardware.
         */
        #define UART_PORT PORTB
    #endif

    #ifndef UART_TX_PIN
        /**
         * @def UART_TX_PIN
         * @brief Bitmask for UART Transmit (TX) pin.
         *
         * @details
         * Defines the specific pin within UART_PORT used for data transmission.  Default is PIN2_bm (PORTB pin 2). This pin must be configured as output.
         *
         * @note TX pin is driven by the UART hardware when USARTD.DREIF is set.
         */
        #define UART_TX_PIN PIN2_bm
    #endif

    #ifndef UART_RX_PIN
        /**
         * @def UART_RX_PIN
         * @brief Bitmask for UART Receive (RX) pin.
         *
         * @details
         * Defines the specific pin within UART_PORT used for data reception. Default is PIN3_bm (PORTB pin 3). This pin must be configured as input.
         *
         * @note RX pin is read by the UART hardware when USARTD.RXCIF is set.
         */
        #define UART_RX_PIN PIN3_bm
    #endif

    #ifndef UART_ALTERNATE_PORT
        /**
         * @def UART_ALTERNATE_PORT
         * @brief Selects UART port multiplexing/routing option.
         *
         * @details
         * Configures the physical pin location of UART TX/RX signals via PORTMUX:
         * - `PORTMUX_USART0_DEFAULT_gc`: Default pin mapping (standard location)
         * - `PORTMUX_USART0_ALTERNATE_gc`: Alternate pin mapping
         *
         * This macro controls the hardware routing of USART0 signals to different port pins. The selected PORTMUX value determines which physical pins carry TXD0 and RXD0 signals, independent of UART_PORT/UART_TX_PIN settings.
         *
         * @note 
         * Override before including uart.h to select desired pin location. Consult MCU datasheet for exact pin mappings of each PORTMUX option. UART_PORT may need adjustment to match selected pinmux.
         */
        #define UART_ALTERNATE_PORT PORTMUX_USART0_DEFAULT_gc
    #endif

    #ifndef UART_DATASIZE
        /**
         * @def UART_DATASIZE
         * @brief Number of data bits per frame (5-8).
         *
         * @details
         * Configures the character length transmitted/received:
         * Valid values: 5, 6, 7, 8, 9H, 9L (default: 8).
         * - USART_CHSIZE_5BIT_gc   = 5 Bits
         * - USART_CHSIZE_6BIT_gc   = 6 Bits
         * - USART_CHSIZE_7BIT_gc   = 7 Bits
         * - USART_CHSIZE_8BIT_gc   = 8 Bits
         * - USART_CHSIZE_9BITH_gc  = 9 Bits (high bit first)
         * - USART_CHSIZE_9BITL_gc  = 9 Bits (low bit first)
         *
         * @note 8 data bits (8N1) is the most common serial configuration.
         */
        #define UART_DATASIZE USART_CHSIZE_8BIT_gc
    #endif

    #ifndef UART_PARITY
        /**
         * @def UART_PARITY
         * @brief Parity bit configuration.
         *
         * @details
         * Selects parity checking for error detection:
         * - USART_PMODE_DISABLED_gc    = 0 (no parity, default)
         * - USART_PMODE_EVEN_gc        = 1 (even parity)
         * - USART_PMODE_ODD_gc         = 2 (odd parity)
         *
         * @note Parity adds one bit to each frame for basic error checking.
         */
        #define UART_PARITY USART_PMODE_DISABLED_gc
    #endif

    #ifndef UART_STOPBITS
        /**
         * @def UART_STOPBITS
         * @brief Number of stop bits per frame (1-2).
         *
         * @details
         * Configures stop bits signaling frame end:
         * - USART_SBMODE_1BIT_gc   = 1 Bit (default)
	     * - USART_SBMODE_2BIT_gc   = 2 Bits
         *
         * @note Stop bits must be idle (high) level.
         */
        #define UART_STOPBITS USART_SBMODE_1BIT_gc
    #endif

    #ifndef UART_RXC_ECHO
        /**
         * @def UART_RXC_ECHO
         * @brief Enables local echo of received characters.
         *
         * @details
         * When defined, each received character is automatically transmitted back through TX (echo effect). Useful for terminal applications.
         *
         * @note Disabled automatically if UART_TXCIE or UART_UDRIE interrupts are enabled.
         */
        //#define UART_RXC_ECHO

        #ifdef _DOXYGEN_    // Used for documentation, can be ignored
            #define UART_RXC_ECHO
        #endif
    #endif

    #ifndef UART_HANDSHAKE
        /**
         * @def UART_HANDSHAKE
         * @brief Flow control / handshaking mode.
         *
         * @details
         * Configures flow control between communicating systems:
         * - 0 = Disabled (default)
         * - 1 = Software flow control (XON/XOFF)
         * - 2 = Hardware flow control (RTS/CTS)
         *
         * @note Enables reliable data transfer when receiver buffer overflows.
         */
        #define UART_HANDSHAKE 0

        #if UART_HANDSHAKE == 2

            /**
             * @def UART_HANDSHAKE_PORT
             * @brief PORT register for hardware handshake pins (RTS/CTS).
             */
            #ifndef UART_HANDSHAKE_PORT
                #define UART_HANDSHAKE_PORT PORTC
            #endif

            /**
             * @def UART_HANDSHAKE_CTS_PIN
             * @brief Clear To Send input pin bitmask.
             *
             * @details
             * CTS pin signals when remote device is ready to receive data. Transmission pauses when CTS is inactive (low).
             */
            #ifndef UART_HANDSHAKE_CTS_PIN
                #define UART_HANDSHAKE_CTS_PIN  PIN1_bm
            #endif

            /**
             * @def UART_HANDSHAKE_RTS_PIN
             * @brief Request To Send output pin bitmask.
             *
             * @details
             * RTS pin signals to remote device that local receiver is ready. Set active (high) when buffer has space.
             */
            #ifndef UART_HANDSHAKE_RTS_PIN
                #define UART_HANDSHAKE_RTS_PIN  PIN0_bm
            #endif

        #endif

        #ifndef UART_HANDSHAKE_XON
            /**
             * @def UART_HANDSHAKE_XON
             * @brief XON character (transmit when ready to receive).
             */
            #define UART_HANDSHAKE_XON 0x11
        #endif
        
        #ifndef UART_HANDSHAKE_XOFF
            /**
             * @def UART_HANDSHAKE_XOFF
             * @brief XOFF character (transmit when not ready to receive).
             */
            #define UART_HANDSHAKE_XOFF 0x13
        #endif
    #endif

    #ifndef UART_STDMODE
        /**
         * @def UART_STDMODE
         * @brief Standard I/O integration mode (printf/scanf).
         *
         * @details
         * Enables avr-libc stdio functions over UART:
         * - 0 = None
         * - 1 = printf + scanf (default)
         * - 2 = printf only
         * - 3 = scanf only
         *
         * @note Requires fdevopen() setup in application code.
         */
        #define UART_STDMODE 1
    #endif

    /**
     * @defgroup UART_Interrupts UART Interrupt Control Macros
     * @brief Configuration macros for interrupt-based UART processing.
     *
     * @attention 
     * !!! Interrupts are NOT implemented in this library !!!
     * If interrupts are used, polling functions will be disabled. Users must implement ISR handlers separately.
     */
    /* @{ */
    #ifndef UART_RXCIE
        /**
         * @def UART_RXCIE
         * @brief Enable or disable UART RX Complete interrupt processing.
         *
         * @details
         * When defined, enables USART Receive Complete interrupt (RX_CIE). Intended for interrupt-driven receive handling via ISR(USART0_RXC_vect).
         *
         * @attention Interrupts are not implemented in this library. Defining this macro disables polling-based receive functions.
         *
         * @note Implement custom ISR for interrupt handling.
         */
         // #define UART_RXCIE
    #endif

    #ifndef UART_TXCIE
        /**
         * @def UART_TXCIE
         * @brief Enable or disable UART TX Complete interrupt processing.
         *
         * @details
         * When defined, enables USART Transmit Complete interrupt (TX_CIE).  Intended for interrupt-driven transmit handling via ISR(USART0_TXC_vect).
         *
         * @attention Interrupts are not implemented in this library. Defining this macro disables polling-based transmit functions. Mutually exclusive with UART_UDRIE.
         *
         * @note Implement custom ISR for interrupt handling.
         */
        // #define UART_TXCIE
    #endif

    #ifndef UART_UDRIE
        /**
         * @def UART_UDRIE
         * @brief Enable or disable UART Data Register Empty interrupt processing.
         *
         * @details
         * When defined, enables USART Data Register Empty interrupt (UDRIE). Intended for interrupt-driven transmit via ISR(USART0_DRE_vect). Automatically loads next byte when transmit buffer is empty.
         *
         * @attention Interrupts are not implemented in this library. Only available when UART_TXCIE is NOT defined.
         *
         * @note Implement custom ISR for interrupt handling.
         */
        #ifndef UART_TXCIE
            // #define UART_UDRIE
        #else
            #error "UART_TXCIE and UART_UDRIE cannot be used together"
        #endif
    #endif
    /* @} */

    #include <stdio.h>
    #include <avr/io.h>
    
    #include "../../common/enums/UART_enums.h"

    void uart_init(void);
    void uart_disable(void);

    #if !defined(UART_TXCIE) && !defined(UART_UDRIE)
        char uart_putchar(char data);

        #if UART_STDMODE == 1 || UART_STDMODE == 2
            int uart_printf(char data, FILE *stream);
        #endif
    #endif

    #if !defined(UART_RXCIE)
        char uart_getchar(UART_Data *status);
        UART_Data uart_scanchar(char *data);
        UART_Error uart_error_flags(void);

        #if UART_STDMODE == 1 || UART_STDMODE == 3
            int uart_scanf(FILE *stream);
            void uart_clear(void);
        #endif
    #endif

    #if !defined(UART_TXCIE) && !defined(UART_UDRIE) && !defined(UART_RXCIE)
        #if UART_HANDSHAKE > 0
            UART_Handshake uart_handshake(UART_Handshake status);
        #endif
    #endif

#endif /* UART_H_ */