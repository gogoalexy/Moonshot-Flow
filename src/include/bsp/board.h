/****************************************************************************
 * include/board/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/* This header file contains function prototypes for the interfaces between
 * (1) the nuttx core-code, (2) the microprocessor specific logic that
 * resides under the arch/ sub-directory, and (3) the board-specific logic
 * that resides under configs/
 *
 * Naming conventions:
 *
 * 1. Common Microprocessor Interfaces.
 *
 *    Any interface that is common across all microprocessors should be
 *    prefixed with up_ and prototyped in this header file. These
 *    definitions provide the common interface between NuttX and the
 *    architecture-specific implementation in arch/
 *
 *    These definitions are retained in the the header file nuttx/include/arch.h
 *
 *    NOTE: up_ is supposed to stand for microprocessor; the u is like the
 *    Greek letter micron: µ. So it would be µP which is a common shortening
 *    of the word microprocessor.
 *
 * 2. Microprocessor-Specific Interfaces.
 *
 *    An interface which is unique to a certain microprocessor should be
 *    prefixed with the name of the microprocessor, for example stm32_,
 *    and be prototyped in some header file in the arch/ directories.
 *
 *    There is also a arch/<architecture>/include/<chip>/chip.h header file
 *    that can be used to communicate other microprocessor-specific
 *    information between the board logic and even application logic.
 *    Application logic may, for example, need to know specific capabilities
 *    of the chip.  Prototypes in that chip.h header file should follow the
 *    microprocessor specific naming convention.
 *
 * 3. Common Board Interfaces.
 *
 *    Any interface that is common across all boards should be prefixed
 *    with board_ and should be prototyped in this header file. These
 *    board_ definitions provide the interface between the board-level
 *    logic and the architecture-specific logic.
 *
 *    Board related declarations are retained in this header file.
 *
 *    There is also a configs/<board>/include/board.h header file that
 *    can be used to communicate other board-specific information between
 *    the architecture logic and even application logic.  Any definitions
 *    which are common between a single architecture and several boards
 *    should go in this board.h header file; this file is reserved for
 *    board-related definitions common to all architectures.
 *
 * 4. Board-Specific Interfaces.
 *
 *    Any interface which is unique to a board should be prefixed with
 *    the board name, for example stm32f4discovery_. Sometimes the board
 *    name is too long so stm32_ would be okay too. These should be
 *    prototyped in configs/<board>/src/<board>.h and should not be used
 *    outside of that board directory since board-specific definitions
 *    have no meaning outside of the board directory.
 */

#ifndef __INCLUDE_NUTTX_BOARD_H
#define __INCLUDE_NUTTX_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <compiler.h>
__BEGIN_DECLS
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BOARD_SERIALNUMBER_SIZE 12
/****************************************************************************
 * Public Function Prototypes
 *
 * These are all standard board interfaces that are exported from board-
 * specific logic to OS internal logic.  These should never be accessed
 * directly from application code but may be freely used within the OS
 *
 ****************************************************************************/

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/
void board_initialize(void);

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 ****************************************************************************/

int board_app_initialize(void);


/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  This function may or may not be supported by a
 *   particular board architecture.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *     meaning of this status information is board-specific.  If not used by
 *     a board, the value zero may be provided in calls to board_reset.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

int board_reset(int status);

/****************************************************************************
 * Name: board_get_serialnumber
 *
 * Description:
 *   Get the Boards notion of it's unique serial number
 *
 * Input Parameters:
 *   serial - Array to hold the serial number.
 *
 * Returned Value:
 *   The length of the serial number.
 *
 ****************************************************************************/

int board_get_serialnumber(uint8_t serial[BOARD_SERIALNUMBER_SIZE]);


/****************************************************************************
 * Name: board_led_initialize
 *
 * Description:
 *   This functions is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, board_led_initialize() is called from
 *   board-specific initialization logic.  But there are a few architectures
 *   where this initialization function is still called from common chip
 *   architecture logic.  This interface is not, however, a common board
 *   interface in any event and, hence, the usage of the name
 *   board_led_initialize is deprecated.
 *
 *   WARNING: This interface name will eventually be removed; do not use it
 *   in new board ports.  New implementations should use the naming
 *   conventions for "Microprocessor-Specific Interfaces" or the "Board-
 *   Specific Interfaces" as described above.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_led_initialize(void);

/****************************************************************************
 * Name: board_led_on
 *
 * Description:
 *   Set the LED configuration into the ON condition for the state provided
 *   by the led parameter.  This may be one of:
 *
 *     LED_STARTED       NuttX has been started
 *     LED_HEAPALLOCATE  Heap has been allocated
 *     LED_IRQSENABLED   Interrupts enabled
 *     LED_STACKCREATED  Idle stack created
 *     LED_INIRQ         In an interrupt
 *     LED_SIGNAL        In a signal handler
 *     LED_ASSERTION     An assertion failed
 *     LED_PANIC         The system has crashed
 *     LED_IDLE          MCU is in sleep mode
 *
 *   Where these values are defined in a board-specific way in the standard
 *   board.h header file exported by every architecture.
 *
 * Input Parameters:
 *   led - Identifies the LED state to put in the ON state (which may or may
 *         not equate to turning an LED on)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_led_on(int led);

/****************************************************************************
 * Name: board_led_off
 *
 * Description:
 *   Set the LED configuration into the OFF condition for the state provided
 *   by the led parameter.  This may be one of:
 *
 *     LED_INIRQ         Leaving an interrupt
 *     LED_SIGNAL        Leaving a signal handler
 *     LED_ASSERTION     Recovering from an assertion failure
 *     LED_PANIC         The system has crashed (blinking).
 *     LED_IDLE          MCU is not in sleep mode
 *
 *   Where these values are defined in a board-specific way in the standard
 *   board.h header file exported by every architecture.
 *
 * Input Parameters:
 *   led - Identifies the LED state to put in the OFF state (which may or may
 *         not equate to turning an LED off)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_led_off(int led);

/****************************************************************************
 * Name: board_led_rgb
 *
 * Description:
 *
 * Input Parameters:
 *
 *  red   - intensity of the red led
 *  green - intensity of the green led
 *  blue  - intensity of the blue led
 *  hz    - number of flashes per second
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#if defined(CONFIG_ARCH_BOARD_PX4FLOW_V1)
#define board_led_rgb(red, green , blue, hz)
#endif
#if defined(CONFIG_ARCH_BOARD_PX4FLOW_V2)
weak_function void board_led_rgb(uint16_t red, uint16_t green , uint16_t blue,
                   uint16_t hz);
#endif

/****************************************************************************
 * Name: board_crashdump
 *
 * Description:
 *   If CONFIG_BOARD_CRASHDUMP is selected then up_asseert will call out to
 *   board_crashdump prior to calling exit in the case of an assertion failure.
 *   Or in the case of a hardfault looping indefinitely. board_crashdump then
 *   has a chance to save the state of the machine. The provided
 *   board_crashdump should save as much information as it can about the cause
 *   of the fault and then most likely reset the system.
 *
 *   N.B. There are limited system resources that can be used by the provided
 *   board_crashdump function. The tems from the fact that most critical/fatal
 *   crashes are because of a hard fault or during interrupt processing.
 *   In these cases, up_assert is running from the context of an interrupt
 *   handlerand it is impossible to use any device driver in this context.
 *
 *   Also consider the following: Who knows what state the system is in? Is
 *   memory trashed? Is the Heap intact? Therefore all we can expect to do in
 *   board_crashdump is save the "machine state" in a place where on the next
 *   reset we can write it to more sophisticated storage in a sane operating
 *   environment.
 *
 ****************************************************************************/

void board_crashdump(uint32_t currentsp, void *tcb, uint8_t *filename,
                     int lineno);
__END_DECLS
#endif /* __INCLUDE_NUTTX_BOARD_H */
