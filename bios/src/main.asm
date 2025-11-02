; Denote we're in the text section
section .text
global main
main:
    xor r0, r0
    xor r1, r1
    lad r2, [rel hello]
    mov r3, byte 7
    zext r3, byte r3
    lad r4, byte [rel main]
    and r2, r4
    scall
.loop:
    jmp .loop

section .data
hello: db "Hello", 0x0a, 0