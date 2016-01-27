/*
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define DBG_COMP ARADBG_SVC

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <arch/board/board.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <ara_debug.h>

#include "ara_board.h"
#include "timesync.h"

#define TIMESYNC_TASK_PRIORITY      (40)
#define TIMESYNC_TASK_STACK_SIZE    (1024)

#define CLOCKID CLOCK_REALTIME
#define SIG SIGRTMIN

/* The current local frame-time */
static uint64_t timesync_frame_time;
static uint64_t timesync_strobe_time[TIMESYNC_MAX_STROBES];
static uint64_t timesync_strobe_time_counter[TIMESYNC_MAX_STROBES];

/* TimeSync finite state machine */
enum timesync_state {
        TIMESYNC_STATE_INVALID       = 0,
        TIMESYNC_STATE_INACTIVE      = 1,
        TIMESYNC_STATE_SYNCING       = 2,
        TIMESYNC_STATE_ACTIVE        = 3,
        TIMESYNC_STATE_DEBUG_INIT    = 4,
        TIMESYNC_STATE_DEBUG_ACTIVE  = 5,
};

static int timesync_state;
static uint32_t timesync_strobe_mask;
static uint32_t timesync_strobe_delay;
static uint32_t timesync_refclk;
static uint32_t timesync_clock_mult;
static int timesync_strobe_count;
static int timesync_strobe_index;
static pthread_mutex_t timesync_lock;
static pthread_cond_t timesync_cv;
static struct stm32_tim_dev_s *timesync_rollover_master_timer;
static struct stm32_tim_dev_s *timesync_rollover_slave_timer;
static struct stm32_tim_dev_s *timesync_strobe_timer;
static int timesync_rollover_timer_irq = -1;
static int timesync_strobe_timer_irq = -1;
static uint64_t overflows = 0;

/*
 * Define TIMESYNC_DEBUG to get printouts locally and a 1 second strobe
 * after completion of the intial synchronization operation
 */
#define TIMESYNC_DEBUG

/* TIME_SYNC strobe timer */
#define TIMESYNC_STROBE_TIMER_ID            6
#define TIMESYNC_STROBE_TIMER_PERIOD        1000000

/* Cascaded rollover timer */
#define TIMESYNC_ROLLOVER_TIMER_MASTER_ID   8 
#define TIMESYNC_ROLLOVER_TIMER_SLAVE_ID    4 
#define TIMESYNC_ROLLOVER_TIMER_PERIOD      0xFFFF
#define TIMESYNC_ROLLOVER_CASCADE_TOTAL     0xFFFFFFFFUL 

#define NSEC_PER_SEC                        1000000000ULL

/**
 * @brief Perform a coarse conversion from clocks to nanoseconds
 *
 * This conversion should be considered coarse because it only accounts for
 * the integer component of the conversion.
 *
 */
static uint64_t timesync_clocks2ns(uint32_t clocks) {
    uint64_t mult_clocks = clocks;

    mult_clocks *= timesync_clock_mult;
    return mult_clocks;
}

/**
 * @brief Return the current hardware count of clocks - accounting for lower 16 bit over-flow during read
 *
 * Latch the two-step hardware read
 * This code is based on linux/drivers/clocksource/arm_arch_timer.c
 *
 */
uint32_t svc_timesync_get_counter(void) {
    uint16_t hi, tmp, lo;

    do {
        hi = STM32_TIM_GETCOUNTER(timesync_rollover_slave_timer);
        lo = STM32_TIM_GETCOUNTER(timesync_rollover_master_timer);
        tmp = STM32_TIM_GETCOUNTER(timesync_rollover_slave_timer);
    } while (hi != tmp);

    return ((uint32_t) hi << 16) | lo;
};

/**
 * @brief Return a 64 bit frame time expressed in nanoseconds - accounting for lower 32 bit over-flow during read
 *
 * Latch lower 32 bits (hardware) and upper 32 bits soft-IRQ tracked
 * to derive a 64 frame-time - expressed in nanoseconds.
 * This code is based on linux/drivers/clocksource/arm_arch_timer.c
 *
 */
uint64_t svc_timesync_get_frame_time(void) {
    uint64_t hi, tmp;
    uint32_t lo;

    do {
        hi = timesync_frame_time;
        lo = svc_timesync_get_counter();
        tmp = timesync_frame_time;
    } while (hi != tmp);

    return (hi + timesync_clocks2ns(lo));
}

/**
 * @brief Responsible for handling roll-over of the lower 32 bit integer into the 64 bit frame-time
 *
 * Handily the amount to append to the frame-time will always be clocks2ns(0xFFFFFFFF)
 * i.e. the maximum count value of both 16 bit counters expressed in nanoseconds.
 * frame_time += clocks2ns(TIM3_COUNT << 16 | TIM8_COUNT) where both count values are set
 * to the maximum 0xFFFF
 *
 */
static int timesync_frame_time_rollover_handler(int irq, void *regs) {

    STM32_TIM_ACKINT(timesync_rollover_slave_timer, irq);
    timesync_frame_time += timesync_clocks2ns(TIMESYNC_ROLLOVER_CASCADE_TOTAL);
    overflows++;
}

static void timesync_log_frame_time(void) {
    if (timesync_strobe_index < TIMESYNC_MAX_STROBES) {
        timesync_strobe_time[timesync_strobe_index] = svc_timesync_get_frame_time();
#ifdef TIMESYNC_DEBUG
        timesync_strobe_time_counter[timesync_strobe_index] = svc_timesync_get_counter();
#endif
        timesync_strobe_index += 1;
    }
}

static int timesync_strobe_handler(int irq, void *regs) {
    irqstate_t flags;

    flags = irqsave();

    STM32_TIM_ACKINT(timesync_strobe_timer, irq);

    timesync_log_frame_time();
    board_timesync_strobe(timesync_strobe_mask);
      
    if (timesync_strobe_index == timesync_strobe_count) {

        STM32_TIM_DISABLEINT(timesync_strobe_timer, 0);
        STM32_TIM_SETMODE(timesync_strobe_timer, STM32_TIM_MODE_DISABLED);
        STM32_TIM_SETCLOCK(timesync_strobe_timer, 0);
#ifdef TIMESYNC_DEBUG
        timesync_state = TIMESYNC_STATE_DEBUG_INIT;
        pthread_cond_signal(&timesync_cv);
#else
        timesync_state = TIMESYNC_STATE_ACTIVE;
#endif
    }

    irqrestore(flags);
    return 0;
}

/*
 * Note this thread exists in-lieu of a hardware timer based
 * solution - considered to be a WIP and not a real-time solution
 */
static int timesync_debugd_main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    uint8_t i;
    uint64_t ftime;

    while (timesync_state != TIMESYNC_STATE_INVALID) {

        /* Don't block when in the active debug state */
        if (timesync_state != TIMESYNC_STATE_DEBUG_ACTIVE) {
            pthread_mutex_lock(&timesync_lock);
            pthread_cond_wait(&timesync_cv, &timesync_lock);
        }

        if (timesync_state == TIMESYNC_STATE_INVALID)
            break;

        switch (timesync_state) {
            case TIMESYNC_STATE_INVALID:
            case TIMESYNC_STATE_INACTIVE:
            case TIMESYNC_STATE_SYNCING:
            case TIMESYNC_STATE_ACTIVE:
                break;
            case TIMESYNC_STATE_DEBUG_ACTIVE:
                board_timesync_strobe(timesync_strobe_mask);
                ftime = svc_timesync_get_frame_time();
                lldbg("overflows=%d running frame-time is %llu\n",
                      overflows, ftime);
                usleep(1000000);
                break;
            case TIMESYNC_STATE_DEBUG_INIT:
                for (i = 1; i < timesync_strobe_index; i++) {
                    lldbg("frame-time diff %llu-useconds %llu-%llu counter-diff is %llu %llu-%llu\n",
                           timesync_strobe_time[i] - timesync_strobe_time[i-1],
                           timesync_strobe_time[i], timesync_strobe_time[i-1],
                           timesync_strobe_time_counter[i] - timesync_strobe_time_counter[i-1],
                           timesync_strobe_time_counter[i], timesync_strobe_time_counter[i-1]);
                }
                timesync_state = TIMESYNC_STATE_DEBUG_ACTIVE;
                break;
           default:
                break;
        }
    }

    return 0;
}

/**
 * @brief Enable TimeSync at the given reference frame_time
 *
 * Setup TIM8 as master clock cascading into TIM4 see DocID018909 Rev 10 page 612
 * https://www.rapitasystems.com/blog/chaining-two-16-bit-timers-together-stm32f4
 * 622/1728 DocID018909 Rev 10 - details which timers can be cascaded
 * TIM8 has been selected since we can clock that @ 96MHz and divide down by 5 to
 * get to 19.2MHz with just a little work @ the PLL configuration.
 */
int svc_timesync_enable(uint8_t strobe_count, uint64_t frame_time,
                        uint32_t strobe_delay, uint32_t strobe_mask,
                        uint32_t refclk) {
                       
    if (!strobe_count || strobe_count > TIMESYNC_MAX_STROBES) 
        return -EINVAL;

    if (!strobe_delay)
        return -EINVAL;

    if (!refclk || STM32_TIM18_FREQUENCY % refclk) {
        lldbg("Error Time-Sync clock %dHz doesn't divide APB clock %dHz evenly\n",
               refclk, STM32_TIM18_FREQUENCY);
        return -ENODEV;
    }

    svc_timesync_disable();
   
    timesync_refclk = refclk; 
    timesync_strobe_count = strobe_count;
    timesync_strobe_index = 0;
    timesync_strobe_mask = strobe_mask;
    timesync_strobe_delay = strobe_delay;
    timesync_clock_mult = NSEC_PER_SEC / timesync_refclk;
    timesync_state = TIMESYNC_STATE_SYNCING;
    timesync_frame_time = frame_time;

    /*******************************************************************
     * Enable TIM8 against the hardware input clock output on TIM8_TRGO
     *******************************************************************/

    /* Configure MMS=010 in TIM8_CR2 output TIM8_TRGO on rollover (master mode) */
    STM32_TIM_SETMASTER_MODE(timesync_rollover_master_timer, STM32_TIM_MASTER_MODE_UPDATE);
    /* Configures TIM8_ARR - auto reload register */
    STM32_TIM_SETPERIOD(timesync_rollover_master_timer, TIMESYNC_ROLLOVER_TIMER_PERIOD);
    /* Configures TIM8_PSC - prescaler value to get our desired master clock */
    STM32_TIM_SETCLOCK(timesync_rollover_master_timer, timesync_refclk);

    /*********************************************************************
     * Enable TIM4 with clock input source ITR2 (TIM8_TRGO), no pre-scaler
     * interrupt on over-flow
     *********************************************************************/

    /* Configure ITR0 as internal trigger TIM4_SMCR:TS=011(TIM8) external clock mode TIM4_SMCR:SMS=111 (slave mode) */
    STM32_TIM_SETSLAVE_MODE(timesync_rollover_slave_timer, STM32_TIM_SLAVE_EXTERNAL_MODE, STM32_TIM_SLAVE_INTERNAL_TRIGGER3);
    /* Configures TIM4_ARR - auto reload register */
    STM32_TIM_SETPERIOD(timesync_rollover_slave_timer, TIMESYNC_ROLLOVER_TIMER_PERIOD);
    /* Configures TIM4_PSC - set to STM32_TIM18_FREQUENCY to get a prescaler value of 0 */
    STM32_TIM_SETCLOCK(timesync_rollover_slave_timer, STM32_TIM18_FREQUENCY);

    /****************************************************************************
     * Enable TIM6 as a simple up-counter at the strobe_delay specified by the AP
     * strobe_delay is expressed in microseconds.
     ****************************************************************************/

    /* Delay is expressed in microseconds so apply directly to TIM6_ARR */
    STM32_TIM_SETPERIOD(timesync_strobe_timer, strobe_delay);
    /* Clock is simply the fundamental input Hz (clocks per second) / 1000 */
    STM32_TIM_SETCLOCK(timesync_strobe_timer, 1000000UL);

    /***************
     * Enable timers
     ***************/

    /* Configures TIM8_CR1 - mode up-counter */
    STM32_TIM_SETMODE(timesync_rollover_master_timer, STM32_TIM_MODE_UP);
    /* Configures TIM4_CR1 - mode up-counter */
    STM32_TIM_SETMODE(timesync_rollover_slave_timer, STM32_TIM_MODE_UP);
    /* Configures TIM6_CR1 - mode up-counter */
    STM32_TIM_SETMODE(timesync_strobe_timer, STM32_TIM_MODE_UP);

    /* Enable roll-over timer interrupt */
    STM32_TIM_ACKINT(timesync_rollover_slave_timer, timesync_rollover_timer_irq);
    STM32_TIM_ENABLEINT(timesync_rollover_slave_timer, 0);
 
    /* Enable strobe timer interrupt */
    STM32_TIM_ACKINT(timesync_strobe_timer, timesync_strobe_timer_irq);
    STM32_TIM_ENABLEINT(timesync_strobe_timer, 0);

    lldbg("ref-clk-freq=%dHz timer-freq=%dHz period=%d mult=%lu\n",
          STM32_TIM18_FREQUENCY, timesync_refclk,
          TIMESYNC_ROLLOVER_TIMER_PERIOD, timesync_clock_mult);
    lldbg("strobe-clk=%dHz period=%d\n", STM32_TIM18_FREQUENCY / 1000UL, strobe_delay);

    return 0;
}

/**
 * @brief Disable TimeSync
 */
int svc_timesync_disable(void) {

    irqstate_t flags;

    flags = irqsave();

    STM32_TIM_DISABLEINT(timesync_rollover_master_timer, 0);
    STM32_TIM_SETMODE(timesync_rollover_master_timer, STM32_TIM_MODE_DISABLED);
    STM32_TIM_SETCLOCK(timesync_rollover_master_timer, 0);
    STM32_TIM_DISABLEINT(timesync_rollover_slave_timer, 0);
    STM32_TIM_SETMODE(timesync_rollover_slave_timer, STM32_TIM_MODE_DISABLED);
    STM32_TIM_SETCLOCK(timesync_rollover_slave_timer, 0);
    timesync_state = TIMESYNC_STATE_INACTIVE;

    irqrestore(flags);

    return 0;
}

/**
 * @brief Return each authoritative frame_time to the passed frame_time output
 */
int svc_timesync_authoritative(uint64_t *frame_time) {
    memcpy(frame_time, timesync_strobe_time, sizeof(uint64_t) * TIMESYNC_MAX_STROBES);
    return 0;
}

void svc_timesync_exit(void) {}

/*
 * System entrypoint. CONFIG_USER_ENTRYPOINT should point to this function.
 */
int svc_timesync_init(void) {
    int rc;

    timesync_rollover_master_timer = stm32_tim_init(TIMESYNC_ROLLOVER_TIMER_MASTER_ID);
    if (timesync_rollover_master_timer == NULL) {
        lldbg("error initializing master time-sync rollover timer #%d\n",
              TIMESYNC_ROLLOVER_TIMER_MASTER_ID);
        PANIC();
        return ERROR;
    }
    timesync_rollover_slave_timer = stm32_tim_init(TIMESYNC_ROLLOVER_TIMER_SLAVE_ID);
    if (timesync_rollover_slave_timer == NULL) {
        lldbg("error initializing slave time-sync rollover timer #%d\n",
              TIMESYNC_ROLLOVER_TIMER_SLAVE_ID);
        PANIC();
        return ERROR;
    }

    timesync_strobe_timer = stm32_tim_init(TIMESYNC_STROBE_TIMER_ID);
    if (timesync_strobe_timer == NULL) {
        lldbg("error initializing timesync strobe timer #%d\n", TIMESYNC_STROBE_TIMER_ID);
        PANIC();
        return ERROR;
    }

    /* TIM8 fundamental frequency */
    lldbg("Frame-time basic-frequency %dHz\n", STM32_TIM18_FREQUENCY);

    svc_timesync_disable();
    timesync_rollover_timer_irq = STM32_TIM_SETISR(timesync_rollover_slave_timer,
                                    timesync_frame_time_rollover_handler, 0);
    if (timesync_rollover_timer_irq == ERROR) {
        lldbg("Unable to latch timer #%d interrupt!\n",
              TIMESYNC_ROLLOVER_TIMER_SLAVE_ID);
        PANIC();
        return;
    }
    timesync_strobe_timer_irq = STM32_TIM_SETISR(timesync_strobe_timer,
                                    timesync_strobe_handler, 0);
    if (timesync_strobe_timer_irq == ERROR) {
        lldbg("Unable to latch timer #%d interrupt!\n",
              TIMESYNC_STROBE_TIMER_ID);
        PANIC();
        return;
    }

#ifdef TIMESYNC_DEBUG
    /* timesync_debug exists for debug and informational purposes only */
    pthread_mutex_init(&timesync_lock, NULL);
    pthread_cond_init(&timesync_cv, NULL);

    rc = task_create("timesync_debugd", TIMESYNC_TASK_PRIORITY,
                     TIMESYNC_TASK_STACK_SIZE, timesync_debugd_main, NULL);
    if (rc == ERROR) {
        dbg_error("failed to start timesync_debugd\n");
        return rc;
    }
#endif

    return 0;
}
