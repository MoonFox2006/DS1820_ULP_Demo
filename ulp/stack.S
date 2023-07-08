.macro push rx
    st \rx, r3, 0
    sub r3, r3, 1
.endm

.macro pop rx
    add r3, r3, 1
    ld \rx, r3, 0
.endm

.macro call addr sr=r1 pos=.
    .set _next, (\pos + 16)
    move \sr, _next
    push \sr
    jump \addr
.endm

.macro ret sr=r1
    pop \sr
    jump \sr
.endm
