@echo off

nasm src\boot.asm -f bin -o boot.img

if "%1"=="/r" (
    rem qemu-system-i386 boot.img
)
