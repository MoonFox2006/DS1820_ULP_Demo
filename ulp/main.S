#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"

    .set DS_GPIO, 13

    .set SKIP_ROM, 0xCC
    .set DS_CONVERT, 0x44
    .set DS_READ, 0xBE

.macro input
    WRITE_RTC_REG(RTC_GPIO_ENABLE_REG, RTC_GPIO_ENABLE_S + DS_GPIO, 1, 0)
    WRITE_RTC_REG(RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_FUN_IE_S, 1, 1)
.endm

.macro output
    WRITE_RTC_REG(RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_FUN_IE_S, 1, 0)
    WRITE_RTC_REG(RTC_GPIO_ENABLE_REG, RTC_GPIO_ENABLE_S + DS_GPIO, 1, 1)
.endm

    .bss

    .global stack
stack:
    .skip 16
    .global stack_end
stack_end:
    .long 0
    .global temperature
temperature:
    .long 0

    .text

    .global entry
entry:
    move r3, stack_end
//loop:
    call ow_reset
    and r2, r2, 1
    jump skip_read, eq
    move r2, SKIP_ROM
    call ow_write
    move r2, DS_READ
    call ow_write
    call ow_read
    move r0, r2
    push r0
    call ow_read
    pop r0
    lsh r2, r2, 8
    or r0, r0, r2
    move r2, temperature
    st r0, r2, 0
skip_read:
    call ow_reset
    and r2, r2, 1
    jump not_detected, eq
    move r2, SKIP_ROM
    call ow_write
    move r2, DS_CONVERT
    call ow_write
not_detected:
//    move r2, 750
//    call delay_ms
//    jump loop
    halt

delay_ms: // duration in R2
    wait 8000 - 20 // 2 + n + 4
    sub r2, r2, 1 // 2 + 4
    jump delay_exit, eq // 2 + 2
    jump delay_ms // 2 + 2
delay_exit:
    ret

ow_reset: // pressence to R2
//    push r0
    output
    wait 8 * 480 - 6 // 2 + n + 4
    input // 24
    wait 8 * 70 - 6 // 2 + n + 4
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + DS_GPIO, 1) // 8 + 4
    jumpr presence, 1, lt // 2 + 2
    move r2, 0
    jump reset_exit
presence:
    wait 8 * 410 - 28 // 2 + n + 4
    move r2, 1 // 2 + 4
reset_exit:
//    pop r0
    ret

ow_write: // data in R2
    stage_rst
write_bit:
    output // 24
    and r1, r2, 1 // 2 + 4
    jump write0, eq // 2 + 2
//    wait 8 * 2 - 16 // 2 + n + 4
    nop // 2 + 4
    input // 24
    wait 8 * 118 - 10 // 2 + n + 4
    jump next_bit // 2 + 2
write0:
    wait 8 * 120 - 16 // 2 + n + 4
    input // 24
next_bit:
    rsh r2, r2, 1 // 2 + 4
    stage_inc 1 // 2 + 4
    jumps write_bit, 8, lt // 2 + 2
    ret

ow_read: // out data to R2
//    push r0
    move r2, 0
    stage_rst
read_bit:
    rsh r2, r2, 1 // 2 + 4
    output // 24
    wait 8 * 2 - 6 // 2 + n + 4
    input // 24
    wait 8 * 12 - 6 // 2 + n + 4
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + DS_GPIO, 1) // 8 + 4
    jumpr read0, 1, lt // 2 + 2
    or r2, r2, 0x80 // 2 + 4
read0:
    wait 8 * 106 - 38 // 2 + n + 4
    stage_inc 1 // 2 + 4
    jumps read_bit, 8, lt // 2 + 2
//    pop r0
    ret
