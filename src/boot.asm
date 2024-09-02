[ORG 7C00h]

;;  007C00h  +-----------------+
;;           |   bootloader    |
;;  007E00h  |-----------------|
;;           |                 |
;;           |   text + data   |
;;           |                 |
;;  00FC00h  |-----------------|
;;           |                 |
;;           |   back buffer   |
;;           |                 |
;;  01F600h  |-----------------|
;;           |       ...       |
;;           |-----------------|
;;           |      stack      |
;;  0A0000h  |-----------------|
;;           |                 |
;;           |   video memory  |
;;           |                 |
;;  0B2C00h  +-----------------+

;; Bootloader
;; ---------------------------------------------------------------------------

[BITS 16]

    cli                                 ; clear interrupts

    mov     byte [drive_letter], dl     ; save drive letter

    mov     ax, 2401h                   ; A20 bit
    int     15h

    xor     ax, ax
    mov     ds, ax                      ; setup real mode segments

.disk_read:                             ; read entire image into memory
    mov     ah, 2h                      ; read sectors
    mov     al, 63                      ; number of sectors
    mov     ch, 0                       ; cylinder
    mov     cl, 2                       ; sector 2
    mov     dh, 0                       ; head 0
    mov     bx, 7E00h                   ; offset to sector 2
    int     13h                         ; read sector
    jc      .disk_read

    mov     ah, 0h
    mov     al, 13h                     ; VGA mode 13h (320x200 256 colors)
    int     10h

    lgdt    [gdt_desc]                  ; load GDT register
    mov     eax, cr0
    or      al, 1
    mov     cr0, eax                    ; switch to protected mode

    lidt    [idtr]                      ; load interrupt table
    sti

    jmp     08h:main                    ; jump to 32-bit protected mode segment

drive_letter dw 0                       ; drive letter from BIOS

;; GDT -----------------------------------------------------------------------
gdt_desc:
    dw gdt_end - gdt_start - 1
    dd gdt_start

gdt_start: dd 0, 0                      ; 00h - null GDT

gdt_code_seg:                           ; 08h
    dw 0FFFFh, 0
    db 0, 10011010b, 11000000b, 0

gdt_data_seg:                           ; 10h
    dw 0FFFFh, 0
    db 0, 10010010b, 11001111b, 0

gdt_end:

;; IDT -----------------------------------------------------------------------
idtr:
    dw (10 * 8) - 1
    dd idt

idt:
    dq 0                                ; int 0 - divide by 0
    dq 0                                ; int 1 - debug
    dq 0                                ; int 2 - non-maskable interrupt
    dq 0                                ; int 3 - breakpoint
    dq 0                                ; int 4 - overflow
    dq 0                                ; int 5 - bound range exceeded
    dq 0                                ; int 6 - invalid opcode
    dq 0                                ; int 7 - device not available
    dw timer_handler, 08h, 8E00h, 0     ; IRQ 0 - timer (PIT)
    dw key_handler,   08h, 8E00h, 0     ; IRQ 1 - keyboard
    dq 0

;; ---------------------------------------------------------------------------
    times 510-($-$$) db 0
    db 0x55
    db 0xAA

;; Sector 2
;; ---------------------------------------------------------------------------

%define SCR_PITCH 320
%define SCR_LINES 200

%define BTN_FORWARD     01h
%define BTN_BACK        02h
%define BTN_LOOKLEFT    04h
%define BTN_LOOKRIGHT   08h
%define BTN_STRAFELEFT  10h
%define BTN_STRAFERIGHT 20h

%define MAP_W    30
%define MAP_H    12
%define TEX_SIZE 32

[BITS 32]

main:
    cli

    mov     eax, 10h                    ; setup segments in protected mode
    mov     ds, eax
    mov     es, eax
    mov     fs, eax
    mov     gs, eax
    mov     ss, eax

    mov     ebp, 9FFFFh                 ; stack (right below video memory)
    mov     esp, ebp

    mov     eax, cr0                    ; enable SSE
    and     ax, 0FFFh
    or      ax, 2
    mov     cr0, eax
    mov     eax, cr4
    or      ax, 3 << 9
    mov     cr4, eax

    ;; initialize PICs

    mov     al, 10h | 1h                ; ICW1
    out     20h, al
    out     0A0h, al

    mov     al, 8                       ; ICW2
    out     21h, al                     ; IRQ0-7 on int 8-15
    mov     al, 8 + 8                   ; IRQ8-15 on int 16-23
    out     0A1h, al

    mov     al, 4                       ; ICW3
    out     021h, al
    mov     al, 2
    out     0A1h, al

    mov     al, 1h
    out     21h, al
    out     0A1h, al                    ; ICW4

    mov     al, 11111100b               ; unmask IRQ 0 and IRQ 1
    out     021h, al
    mov     al, 11111111b
    out     0A1h, al

    ;; initialize mouse

    mov     al, 0A8h                    ; enable keyboard aux port
    out     64h, al

    mov     al, 020h                    ; 20h - get compaq status byte
    out     64h, al
    in      al, 60h                     ; status byte available on port 60
    mov     ah, al
    in      al, 60h                     ; read port 60 again just in case
    or      ah, 2                       ; enable IRQ 12
    and     ah, ~20h                    ; clear bit 5, disable mouse clock

    mov     al, 60h                     ; set compaq status
    out     64h, al
    mov     al, ah
    out     60h, al                     ; send the updated status byte
    in      al, 60h                     ; might get an ACK

    mov     al, 0D4h
    out     64h, al
    mov     al, 0F6h
    out     60h, al
    in      al, 60h

    mov     al, 0D4h                    ; use default mouse settings
    out     64h, al
    mov     al, 0F4h                    ; enable the mouse
    out     60h, al
    in      al, 60h

    ;; timer

    mov     al, 00110100b               ; high/low frequency generator on chan 0
    out     43h, al                     ; PIT control

    mov     ax, 03FFFh                  ; PIT frequency (hi/lo)
    out     40h, al
    mov     al, ah
    out     40h, al

    sti                                 ; Enable interrupts

    call    r_init_palette              ; Init VGA palette

frame:
    call    mouse_input
    call    read_input

    ;call    draw_clear
    ;call    draw_world
    ;call    draw_player
    ;call    draw_fill_screen
    ;call    draw_texture

    call    r_perspective_view
    call    r_debug_register

    call    r_present

.wait_for_frame:
    hlt                                 ; sleep until an interrupt wakes us up
    mov     eax, [start_frame]
    test    eax, eax                    ; check if we got a frame signal from the timer
    jz      .wait_for_frame             ; if not, go back to sleep
    mov     dword [start_frame], 0      ; reset frame signal

    jmp frame

;; Player
;; ---------------------------------------------------------------------------

;; void read_input(void)
read_input:
    pushad

    xorps   xmm0, xmm0
    xorps   xmm1, xmm1

.forward:
    mov     eax, dword [buttons]
    and     eax, BTN_FORWARD
    jz     .strafe_left

    addss   xmm0, [forward_x]
    addss   xmm1, [forward_y]

.strafe_left:
    mov     eax, dword [buttons]
    and     eax, BTN_STRAFELEFT
    jz     .strafe_right

    subss   xmm0, [right_x]
    subss   xmm1, [right_y]

.strafe_right:
    mov     eax, dword [buttons]
    and     eax, BTN_STRAFERIGHT
    jz     .look_left

    addss   xmm0, [right_x]
    addss   xmm1, [right_y]

.look_left:
    mov     eax, dword [buttons]
    and     eax, BTN_LOOKLEFT
    jz     .back
    mov     esi, dword [angle]
    add     esi, 2                      ; esi = angle to clamp
    call    clamp_angle
    mov     dword [angle], eax          ; eax = clamped angle
    call    face_direction

.back:
    mov     eax, dword [buttons]
    and     eax, BTN_BACK
    jz     .look_right

    subss   xmm0, [forward_x]
    subss   xmm1, [forward_y]

.look_right:
    mov     eax, dword [buttons]
    and     eax, BTN_LOOKRIGHT
    jz     .break
    mov     esi, dword [angle]
    add     esi, -2                     ; esi = angle to clamp
    call    clamp_angle
    mov     dword [angle], eax          ; eax = clamped angle
    call    face_direction

.break:
    call    normalize                   ; normalize(wishdir)
    comiss  xmm7, [fzero]               ; zero length
    jnz      .havespeed
    xorps   xmm0, xmm0
    xorps   xmm1, xmm1

.havespeed:
    movss   xmm2, xmm0
    movss   xmm3, xmm1

    movss   xmm0, [velocity_x]
    movss   xmm1, [velocity_y]
    mulss   xmm0, [friction]
    mulss   xmm1, [friction]
    movss   [velocity_x], xmm0
    movss   [velocity_y], xmm1

    call    walk_move                   ; XMM2, XMM3 = wishdir, XMM7 = wishspeed

.return:
    popad
    ret

;; Updates forward and right vectors based on the angle.
;; void face_direction(void)
face_direction:
    push    eax
    push    ebp
    mov     ebp, esp
    sub     esp, 12

    movss   [ebp - 4], xmm0
    movss   [ebp - 8], xmm1

    fild    dword [angle]               ; ST0 = angle
    fmul    dword [fdeg2rad]            ; ST0 = randians(angle)
    fsincos
    fstp    dword [forward_x]           ; forward_x = cos(angle)
    fchs
    fstp    dword [forward_y]           ; forward_y = -sin(angle)

    xorps   xmm0, xmm0                  ; right_x = -forward_y
    subss   xmm0, [forward_y]
    movss   [right_x], xmm0

    movss   xmm1, [forward_x]
    movss   [right_y], xmm1             ; right_y = forward_x

    movss   xmm0, [ebp - 4]
    movss   xmm1, [ebp - 8]

    mov     esp, ebp
    pop ebp
    pop eax
    ret

;; void walk_move(xmm2 wishdir_x, xmm3 wishdir_y)
walk_move:
    pushad

    movss   xmm0, [velocity_x]
    movss   xmm1, [velocity_y]
    mulss   xmm0, xmm2
    mulss   xmm1, xmm3
    addss   xmm0, xmm1                  ; XMM0 = speed = dot(velocity, wishdir)

    movss   xmm1, [maxspeed]
    subss   xmm1, xmm0                  ; XMM1 = maxspeed - speed

    maxss   xmm1, [fzero]
    minss   xmm1, [maxaccel]            ; XMM1 = accel = clamp(XMM1, 0, maxaccel * dt)
    movss   xmm5, xmm1                  ; XMM5 = accel

    movss   xmm0, xmm2
    movss   xmm1, xmm3
    mulss   xmm0, xmm5
    mulss   xmm1, xmm5
    addss   xmm0, [velocity_x]
    addss   xmm1, [velocity_y]
    movss   [velocity_x], xmm0
    movss   [velocity_y], xmm1          ; velocity = madd(velocity, wishdir, accel)

    movss   xmm2, xmm0
    movss   xmm3, xmm1

    movss   xmm0, [origin_x]
    cvttss2si esi, xmm0                 ; ESI = map origin_x
    shr     esi, 5
    addss   xmm0, xmm2                  ; XMM0 = wish x

    movss   xmm1, [origin_y]
    cvttss2si edi, xmm1
    shr     edi, 5                      ; EDI = map origin_y
    imul    edi, MAP_W                  ; index in the map
    addss   xmm1, xmm3                  ; XMM1 = wish y

    cvttss2si eax, xmm0                 ; EAX = wish x in world units
    comiss  xmm2, [fzero]               ; check if negative direction
    jb      .sub_x
    add     eax, 16                     ; add 16 units for the player bounding box
    jmp     .add_x

.sub_x:
    sub     eax, 16                     ; negative direction: sub 16 units for the bounds

.add_x:
    shr     eax, 5                      ; EAX = map wish x

    cvttss2si edx, xmm1                 ; EDX = wish y in world units
    comiss  xmm3, [fzero]
    jb      .sub_y
    add     edx, 16
    jmp     .add_y

.sub_y:
    sub     edx, 16

.add_y:
    shr       edx, 5                    ; EDX = map wish y
    imul      edx, MAP_W                ; scale by map size

    mov     cl, [map + edi + eax]
    cmp     cl, '.'
    jne     .clamp_x
    movss   [origin_x], xmm0

.clamp_x:
    mov     cl, [map + edx + esi]
    cmp     cl, '.'
    jne     .clamp_y
    movss   [origin_y], xmm1

.clamp_y:
    popad
    ret

;; void mouse_input(void)
mouse_input:
    in      al, 64h                     ; read PS2 status byte
    and     al, 20h                     ; bit 5 set if input is from a mouse
    jz      .return

    in      al, 60h                     ; read mouse press
    mov     bl, al                      ; AL = buttons
    in      al, 60h                     ; read mouse X
    mov     ah, al                      ; AH = mouse X
    in      al, 60h                     ; AL = mouse Y

    mov     ebx, [enablemouse]
    test    ebx, ebx
    jz      .return                     ; mouse disabled

    xor     edx, edx
    mov     dl, ah

    mov     cl, ah
    and     cl, 80h
    jz      .positive_x

    not     dl
    shr     dl, 1                       ; lower sensitivity

    mov     esi, [angle]
    add     esi, edx
    call    clamp_angle
    mov     [angle], esi
    call    face_direction
    jmp     .return

.positive_x:
    shr     dl, 1
    mov     esi, [angle]
    sub     esi, edx
    call    clamp_angle
    mov     [angle], esi
    call    face_direction

.return:
    ret

;; Math routines
;; ---------------------------------------------------------------------------

;; xmm0, xmm1, xmm7 normalize(xmm0 x, xmm1 y)
normalize:
    movss   xmm2, xmm0
    movss   xmm3, xmm1

    mulss   xmm2, xmm2                  ; dot(xmm0, xmm1)
    mulss   xmm3, xmm3
    addss   xmm2, xmm3

    sqrtss  xmm7, xmm2                  ; XMM7 = length(xmm0, xmm1)
    divss   xmm0, xmm7
    divss   xmm1, xmm7
    ret

;; eax clamp_angle(esi angle)
clamp_angle:
    mov     eax, esi
    cmp     eax, 360
    jl      .zero
    sub     eax, 360

.zero:
    cmp     eax, 0
    jge     .end
    add     eax, 360

.end:
    ret

;; xmm0 clamp_anglef(xmm0 angle)
clamp_anglef:
    comiss  xmm0, [f360]
    jb      .zero
    subss   xmm0, [f360]

.zero:
    comiss  xmm0, [fzero]
    jae     .end
    addss   xmm0, [f360]

.end:
    ret

;; Renderer
;; ---------------------------------------------------------------------------

;; void r_init_palette(void)
r_init_palette:
    pushad

    xor     ecx, ecx

    mov     dx, 3C8h                    ; change VGA palette value
    mov     al, 0                       ; start changing from index 0
    out     dx, al                      ; index is incremented every 3 bytes written

    mov     dx, 3C9h                    ; write RGB color

    mov     ecx, 256
    mov     ebx, pal

.loop:
    mov     edi, [ebx]

    mov     eax, edi
    shr     eax, 12
    out     dx, al                      ; R

    mov     eax, edi
    shr     eax, 6
    and     eax, 0FFh
    out     dx, al                      ; G

    mov     eax, edi
    and     eax, 0FFh
    out     dx, al                      ; B

    add     ebx, 4
    dec     ecx
    jnz     .loop

    popad
    ret

;; Copies the backbuffer to video memory.
;; void r_present(void)
r_present:
    pushad

    mov     dx, 3DAh

.wait_end:
    in      al, dx                      ; wait for previous vblank to finish
    test    al, 08h
    jnz     .wait_end

.wait_start:
    in      al, dx                      ; wait for next vblank
    test    al, 08h
    jz      .wait_start

    mov     edi, 0A0000h
    mov     esi, screen
    mov     ecx, (SCR_PITCH / 4) * SCR_LINES

.copy:                                  ; copy entire buffer
    mov     eax, [esi]
    mov     [edi], eax
    add     edi, 4
    add     esi, 4
    dec     ecx
    jnz     .copy

    popad
    ret

;; void r_column(esi x, edi wall_y, ebx wall_height, ecx color)
r_column:
    pushad

    mov     edx, screen

    cmp     esi, 0
    jl      .nodraw

    cmp     esi, SCR_PITCH
    jge     .nodraw

    xor     eax, eax
    add     ebx, edi

    test    edi, edi
    jz      .wall                       ; don't draw ceiling if wall starts above it

.ceiling:
    mov     byte [edx + esi], 09h
    add     edx, SCR_PITCH
    inc     eax
    cmp     eax, edi
    jl     .ceiling

.wall:
    mov     byte [edx + esi], cl
    add     edx, SCR_PITCH
    inc     eax
    cmp     eax, ebx
    jl      .wall

    cmp     eax, SCR_LINES-1
    jge     .nodraw

.floor:
    mov     byte [edx + esi], 245
    add     edx, SCR_PITCH
    inc     eax
    cmp     eax, SCR_LINES
    jl      .floor

.nodraw:
    popad
    ret

;; void r_perspective_view(void)
r_perspective_view:
    pushad
    mov     ebp, esp
    sub     esp, 64h

    ; XMM2 = ray origin[X]
    ; XMM3 = ray origin[Y]
    ; XMM4 = ray offset[X]
    ; XMM5 = ray offset[Y]
    ; XMM6 = radians(angle)
    ; XMM7 = tan(angle)

    cvttss2si eax, [origin_x]
    cvttss2si edx, [origin_y]
    shr     eax, 5
    shr     edx, 5

    imul    edx, MAP_W
    mov     al, byte [map + eax + edx]
    cmp     al, '.'
    jne     .ray_cast_break             ; starting inside a wall

    xor     eax, eax
    mov     [ebp - 04h], eax            ; 04 = ray origin[X] (shortest)
    mov     [ebp - 08h], eax            ; 08 = ray origin[Y] (shortest)

    movss   xmm0, [flarge]
    movss   [ebp - 0Ch], xmm0           ; 0C = shortest distance

    mov     dword [ebp - 10h], 0        ; 10 = N
    mov     dword [ebp - 14h], 4        ; 14 = DOF
    mov     [ebp - 1Ch], eax            ; 1C = scratch

    mov     [ebp - 20h], eax            ; 20 = sin(angle)
    mov     [ebp - 24h], eax            ; 24 = cos(angle)

    cvtsi2ss xmm0, [angle]
    addss   xmm0, [f30]
    call    clamp_anglef
    movss   [ebp - 28h], xmm0           ; 28 = angle
                                        ; 2Ch = color
                                        ; 30h, 34h = scratch
    movss   [ebp - 38h], xmm0           ; texture step
    movss   [ebp - 3Ch], xmm0           ; texture offset

.ray_cast:
    mov     eax, [ebp - 10h]            ; N
    cmp     eax, 320                    ; number of columns
    je      .ray_cast_break

    movss   xmm6, [ebp - 28h]
    movss   xmm0, [fdeg2rad]
    mulss   xmm6, xmm0                  ; XMM6 = radians(angle)

    movss   [ebp - 30h], xmm6
    fld     dword [ebp - 30h]
    fsincos                             ; ST0 = cos, ST1 = sin
    fstp    dword [ebp - 24h]           ; 24 = cos(angle)
    fstp    dword [ebp - 20h]           ; 20 = sin(angle)

    movss   xmm7, [ebp - 20h]
    divss   xmm7, [ebp - 24h]           ; XMM7 = tan(angle)

    mov     dword [ebp - 2Ch], 0        ; 32 = wall color

.vertical:
    cvttss2si eax, [origin_x]           ; EAX = (int)origin_x
    shr     eax, 5
    shl     eax, 5                      ; snap origin_x to grid
    mov   [ebp - 1Ch], eax              ; 1C = eax

    movss   xmm0, [ebp - 24h]           ; XMM0 = cos(angle)
    comiss  xmm0, [feps]                ; compare to 0.001
    ja      .vertical_positive          ; > EPS
    comiss  xmm0, [fneps]               ; compare to -EPS
    jb      .vertical_negative          ; < EPS
    jmp     .horizontal                 ; asymptote

.vertical_positive:
    cvtsi2ss xmm2, [ebp - 1Ch]          ; XMM2 = (float)origin_x
    addss   xmm2, [ftile_size]          ; XMM2 += -TILE_SIZE

                                        ; find ray_y based on ray_x
    movss   xmm3, [origin_x]            ; XMM3 = player origin_x
    subss   xmm3, xmm2                  ; origin_x - ray_origin_x
    mulss   xmm3, xmm7                  ; * tan(angle)
    addss   xmm3, [origin_y]            ; XMM3 = ray_y

    movss   xmm4, [ftile_size]          ; XMM4 = ray_offset_x = TILE_SIZE
    movss   xmm5, [fntile_size]         ; XMM5 = -ray_offsey_x
    mulss   xmm5, xmm7                  ; XMM5 = ray_offset_y

   jmp     .vertical_continue

.vertical_negative:
    cvtsi2ss xmm2, [ebp - 1Ch]          ; XMM2 = (float)origin_x
    addss   xmm2, [fneps]               ; XMM2 += -EPS

    movss   xmm3, [origin_x]            ; XMM2 = player origin_y
    subss   xmm3, xmm2                  ; ray_origin_y - origin_y
    mulss   xmm3, xmm7                  ; * 1/tan(angle)
    addss   xmm3, [origin_y]            ; XMM2 = ray_x

    movss   xmm4, [fntile_size]         ; XMM5 = ray_offset_y -TILE_SIZE
    movss   xmm5, [ftile_size]          ; XMM4 = -ray_offsey_y
    mulss   xmm5, xmm7
    cvttss2si eax, xmm4

    jmp     .vertical_continue

.vertical_continue:
    movss   xmm0, [ebp - 20h]           ; param XMM0 = sin(angle)
    movss   xmm1, [ebp - 24h]           ; param XMM1 = cos(angle)
    call    r_trace_step
    movss   [ebp - 04h], xmm2           ; save the current best guess
    movss   [ebp - 08h], xmm3
    movss   [ebp - 0Ch], xmm0           ; current shortest distance

.horizontal:
    movss   xmm0, [flt1]
    divss   xmm0, xmm7                  ; XMM7 = 1.0 / tan(angle)
    movss   xmm7, xmm0

    cvttss2si eax, [origin_y]           ; EAX = (int)origin_y
    shr     eax, 5
    shl     eax, 5                      ; snap origin_y to grid
    mov   [ebp - 1Ch], eax              ; 1C = eax

    movss   xmm0, [ebp - 20h]           ; XMM0 = sin(angle)
    comiss  xmm0, [feps]                ; compare to 0.001
    ja      .horizontal_positive        ; > EPS
    comiss  xmm0, [fneps]               ; compare to -EPS
    jb      .horizontal_negative        ; < EPS
    jmp     .horizontal_break           ; asymptote

.horizontal_positive:
    cvtsi2ss xmm3, [ebp - 1Ch]          ; XMM0 = (float)origin_y
                                        ; -eps so when we truncate we test the sector above
                                        ; instead of the current sector
    addss   xmm3, [fneps]               ; XMM0 += -EPS

    movss   xmm2, [origin_y]            ; XMM2 = player origin_y
    subss   xmm2, xmm3                  ; ray_origin_y - origin_y
    mulss   xmm2, xmm7                  ; * 1/tan(angle)
    addss   xmm2, [origin_x]            ; XMM2 = ray_x

    movss   xmm5, [fntile_size]         ; XMM5 = ray_offset_y -TILE_SIZE
    movss   xmm4, [ftile_size]          ; XMM4 = -ray_offsey_y
    mulss   xmm4, xmm7

    jmp     .horizontal_continue

.horizontal_negative:
    cvtsi2ss xmm3, [ebp - 1Ch]          ; XMM3 = (float)origin_y
    addss   xmm3, [ftile_size]          ; XMM3 += -TILE_SIZE

    movss   xmm2, [origin_y]            ; XMM2 = player origin_y
    subss   xmm2, xmm3                  ; ray_origin_y - origin_y
    mulss   xmm2, xmm7                  ; * 1/tan(angle)
    addss   xmm2, [origin_x]            ; XMM2 = ray_x

    movss   xmm5, [ftile_size]          ; XMM5 = ray_offset_y -TILE_SIZE
    movss   xmm4, [fntile_size]         ; XMM4 = -ray_offsey_y
    mulss   xmm4, xmm7

    jmp     .horizontal_continue

.horizontal_continue:
    movss   xmm0, [ebp - 20h]           ; param XMM0 = sin(angle)
    movss   xmm1, [ebp - 24h]           ; param XMM1 = cos(angle)
    call    r_trace_step                ; XMM0 = length(ray)

    comiss  xmm0, [ebp - 0Ch]           ; check if horizontal is shorter
    ja      .horizontal_break           ; use vertical result instead

    movss   [ebp - 04h], xmm2           ; shortest ray
    movss   [ebp - 08h], xmm3
    movss   [ebp - 0Ch], xmm0
    mov     dword [ebp - 2Ch], 2        ; lighter wall color shade

.horizontal_break:
    cvtsi2ss xmm0, [angle]
    movss    xmm1, [ebp - 28h]
    subss   xmm0, xmm1                  ; angle - ray_angle
    call    clamp_anglef
    movss   [ebp - 30h], xmm0

    fld     dword [ebp - 30h]
    fmul    dword [fdeg2rad]
    fcos
    fstp    dword [ebp - 30h]

    movss   xmm0, [ebp - 30h]           ; 30 = cos(angle - ray_angle)
    mulss   xmm0, [ebp - 0Ch]           ; XMM0 = length(ray)*cos(angle)

    cvttss2si ebx, xmm0                 ; EDX = truncated distance

    test    ebx, ebx
    jz      .ray_cast_break

    mov     eax, 50 * 200               ; EAX = scaled screen size
    xor     edx, edx
    idiv    ebx                         ; EAX = (32 * 320) / distance

    cvtsi2ss xmm0, eax                  ; XMM0 = (float)line height
    movss   xmm1, [ftile_size]
    divss   xmm1, xmm0                  ; XMM1 = texture y step size = 32 / line height
    xorps   xmm0, xmm0                  ; XMM0 = texture y offset

    cmp     eax, 200
    jb      .distance_clamped

    cvtsi2ss xmm0, eax
    subss   xmm0, [f200]                ; offset the texture y offset when clamping
    cvtss2si eax, xmm0
    mov     eax, 200                    ; clamp line height
    mulss   xmm0, [fhalf]               ; by half the screen height

.distance_clamped:
    mov     ebx, eax                    ; EBX = line height
    shr     eax, 1                      ; EAX = line height / 2
    mov     edi, 100
    sub     edi, eax                    ; EDX = line offset = screen y offset

    mov     esi, [ebp - 10h]            ; column index
    mov     ecx, [ebp - 2Ch]            ; color
    mov     edx, screen

    cmp     esi, 0
    jl      .ray_cast_continue          ; offscreen

    cmp     esi, SCR_PITCH
    jge     .ray_cast_continue          ; offscreen

    xor     eax, eax                    ; EAX = column offset
    add     ebx, edi

    test    edi, edi
    jz      .wall_start                 ; don't draw ceiling if wall starts above it

    mov     eax, edi                    ; wall offset
    mov     edx, eax
    imul    edx, SCR_PITCH
    add     edx, screen

.wall_start:
    push    edi
    movss   xmm7, xmm0                  ; XMM7 = v
    mulss   xmm7, xmm1                  ; XMM7 = texture y offset * texture y step

    mov     ecx, [ebp - 2Ch]            ; which wall direction to shade
    test    ecx, ecx                    ; 1 = horizontal
    jz      .shade_horizontal

    movss   xmm6, [ebp - 04h]
    cvttss2si edi, xmm6                 ; EDI = u = (int)ray_x
    and     edi, 31                     ; EDI = u = (int)ray_x % 32

    jmp     .wall                       ; ready for shading

.shade_horizontal:
    movss   xmm6, [ebp - 08h]
    cvttss2si edi, xmm6                 ; EDI = u = (int)ray_x
    and     edi, 31                     ; EDI = u = (int)ray_x % 32

.wall:
    push    eax

    cvtss2si eax, xmm7
    and     eax, 31
    shl     eax, 5                      ; (int)v * 32
    add     eax, edi                    ; offset = u + v * 32

    mov     ch, byte [textures + eax]
    mov     cl, byte [textures + 0400h + eax]

    mov     eax, [ebp - 2Ch]
    test    eax, eax
    jz      .nodarken

    mov     cl, ch

.nodarken:
    mov     byte [edx + esi], cl

    addss   xmm7, xmm1
    add     edx, SCR_PITCH

    pop     eax
    inc     eax
    cmp     eax, ebx
    jl      .wall

    cmp     eax, SCR_LINES-1
    jge     .ray_cast_continue

    pop     edi                         ; EDI = wall start
    test    edi, edi                    ; check if wall started at y = 0
    jz      .ray_cast_continue          ; if so no need to draw floors and ceilings

    imul    edi, SCR_PITCH              ; EDI = wall start screen offset
    lea     ebx, [screen + edi]

.floor_and_ceiling:
    cvtsi2ss xmm0, eax
    subss   xmm0, [f100]                ; XMM0 = dy = y - SCR_LINES/2

    movss   xmm1, [fplane]              ; XMM1 = 3200 / dy / cos(angle - ray_angle)
    divss   xmm1, xmm0
    divss   xmm1, [ebp - 30h]           ; 30 = cos(angle - ray_angle)

    movss   xmm0, [ebp - 24h]           ; XMM0 = cos(angle)
    mulss   xmm0, xmm1
    movss   xmm6, [origin_x]
    addss   xmm6, xmm0                  ; XMM6 = u

    movss   xmm0, [ebp - 20h]           ; XMM0 = sin(angle)
    mulss   xmm0, xmm1
    movss   xmm7, [origin_y]
    subss   xmm7, xmm0                  ; XMM0 = v

    push    eax
    cvttss2si eax, xmm6
    and     eax, 31
    cvttss2si ecx, xmm7
    and     ecx, 31
    shl     ecx, 5                      ; [v * 32]
    add     eax, ecx                    ; [u + v * 32]

    mov     cl, [textures+0C00h + eax]  ; ceiling texture
    mov     byte [ebx + esi], cl

    mov     cl, [textures+0800h + eax]  ; floor texture
    mov     byte [edx + esi], cl

    pop     eax
    add     edx, SCR_PITCH
    sub     ebx, SCR_PITCH
    inc     eax
    cmp     eax, SCR_LINES+1
    jl      .floor_and_ceiling

.ray_cast_continue:
    movss   xmm0, [ebp - 28h]
    subss   xmm0, [fanglestep]
    call    clamp_anglef                ; clamp in [0,360)
    movss   [ebp - 28h], xmm0

    inc     dword [ebp - 10h]
    jmp     .ray_cast

.ray_cast_break:

    mov     esp, ebp
    popad
    ret

;; ecx r_darken_color(ecx)
r_darken_color:
    push    eax
    mov     eax, ecx

    cmp     cl, 127
    jg      .reversed                   ; quake palette goes from light to dark in the bottom half

    and     eax, 0Fh                    ; already the darkest shade for this color
    jz      .return
    dec     cl
    jmp     .return

.reversed:
    cmp     eax, 0Fh
    je      .return                     ; already the darkest shade for this color
    inc     cl

.return:
    pop     eax
    ret

;; xmm0 r_trace_step(xmm0 sin, xmm1 cos, xmm2 ray_x, xmm3 ray_y, xmm4 ofx, xmm5 ofy)
;; modifies xmm2, xmm3 with new ray_x, ray_y
r_trace_step:
    push    ebp
    push    ecx
    mov     ebp, esp
    sub     esp, 08h

    movss   [ebp - 04h], xmm6           ; save non-volatile
    movss   [ebp - 08h], xmm7

    mov     ecx, 16                     ; sectors to check

    movss   xmm6, xmm0                  ; sin(angle)
    movss   xmm7, xmm1                  ; cos(angle)

.continue:
    test    ecx, ecx
    jz      .break

                                        ; must truncate instead of round
    cvttss2si esi, xmm2                 ; ESI = (int)ray_origin_x
    cvttss2si edi, xmm3                 ; EDI = (int)ray_origin_y

    shr     esi, 5                      ; align to map grid
    shr     edi, 5

    mov     ebx, MAP_W
    imul    ebx, edi
    add     ebx, esi                    ; EBX = ray_x + ray_y * width

    cmp     ebx, MAP_W*MAP_H-1
    ja      .break

    mov     al, byte [map + ebx]
    cmp     al, '.'
    je      .not_wall
    jmp     .break

.not_wall:
    addss   xmm2, xmm4
    addss   xmm3, xmm5

    dec     ecx
    jmp     .continue

.break:
    movss   xmm0, xmm2                  ; find the ray length
    subss   xmm0, [origin_x]            ; XMM0 = ray_x - origin_x
    mulss   xmm0, xmm7                  ; XMM0 *= cos(angle)

    movss   xmm1, xmm3
    subss   xmm1, [origin_y]            ; XMM1 = ray_y - origin_y
    mulss   xmm1, xmm6                  ; XMM1 *= sin(angle)

    subss   xmm0, xmm1                  ; XMM0 = ray length

    movss   xmm6, [ebp - 04h]
    movss   xmm7, [ebp - 08h]
    mov     esp, ebp
    movss   xmm6, [ebp - 04h]
    pop     ecx
    pop     ebp
    ret

;; Draws [preg] to the top of screen
;; void r_debug_register(void)
r_debug_register:
    pushad
    mov     ebp, esp
    sub     esp, 16

    mov     dword [ebp - 4], 0          ; -4  = sentence offset
    mov     dword [ebp - 8], 0F0000000h ; -8  = nibble mask
    mov     dword [ebp - 12], 32        ; -12 = nibble position

.loop_char:
    sub     dword [ebp - 12], 4         ; next nibble

    mov     ebx, dword [preg]           ; EBX = register to draw
    mov     edx, dword [ebp - 8]        ; EDX = register mask
    and     ebx, edx                    ; EBX = masked character index
    mov     ecx, dword [ebp - 12]
    shr     ebx, cl                     ; EBX = character index
    lea     ebx, [font + 8 * ebx]       ; EBX = font offset

    xor     eax, eax                    ; EAX = font y offset
    mov     edi, 2 * SCR_PITCH          ; EDI = screen y offset

.loop_y:
    mov     ecx, 100h                   ; ECX = font bit mask
    mov     esi, 2                      ; ESI = screen x offset

.loop_x:
    shr     ecx, 1                      ; next horizontal pixel mask
    mov     edx, ecx                    ; EDX = scratch
    and     edx, [ebx + eax]            ; mask off font pixel
    jz      .nodraw

    lea     edx, [screen + edi + esi]   ; [x + y * pitch]
    add     edx, dword [ebp - 4]        ; + character offset
    mov     byte [edx], 0FFh

.nodraw:
    inc     esi                         ; screen x offset
    test    ecx, ecx                    ; check when mask is 0
    jnz     .loop_x

    inc     eax
    add     edi, SCR_PITCH              ; next scanline to draw
    cmp     eax, 8                      ; draw 8 scanlines per character
    jne     .loop_y

    add     dword [ebp - 4], 8          ; next character x position
    mov     edx, dword [ebp - 4]
    cmp     edx, 64                     ; draw 8 characters
    shr     dword [ebp - 8], 4          ; shift character mask
    jne     .loop_char

    mov     esp, ebp
    popad
    ret

;; 2D drawing routines
;; ---------------------------------------------------------------------------

;; void draw_rect(esi x, edi y, edx w, ebx h, ecx color)
draw_rect:
    pushad

    cmp     ebx, 0                      ; 0 width, height
    jle     .end

    cmp     edx, 0
    jle     .end

    cmp     esi, 0                      ; test bounds
    jl      .end
    cmp     esi, SCR_PITCH
    jge     .end

    cmp     edi, 0
    jl      .end
    cmp     edi, SCR_LINES
    jge     .end                        ; entirely offscreen

    mov     eax, esi                    ; clipping
    add     eax, edx
    cmp     eax, SCR_PITCH
    jl      .valid_x                    ; not offscreen

    mov     edx, SCR_PITCH
    sub     edx, esi                    ; new clipped height
    cmp     edx, 0
    jle     .end                        ; entirely offscreen

.valid_x:
    mov     eax, edi
    add     eax, ebx
    cmp     eax, SCR_LINES
    jl      .valid_y                    ; not offscreen

    mov     ebx, SCR_LINES
    sub     ebx, edi                    ; new clipped height
    cmp     ebx, 0
    jle     .end

.valid_y:
    mov     eax, ecx                    ; EAX = color

    mov     ecx, edi
    imul    ecx, SCR_PITCH              ; scale y position by PITCH

    mov     edi, screen                 ; EDI = scanline
    add     edi, esi                    ; x offset
    add     edi, ecx                    ; scaled y offset

.loop_y:
    xor     ecx, ecx                    ; ECX = scanline offset
.loop_x:
    mov     byte [edi + ecx], al        ; write color

    inc     ecx
    cmp     ecx, edx
    jnz     .loop_x

    add     edi, SCR_PITCH              ; next scanline
    dec     ebx                         ; scanlines remaining
    jnz     .loop_y

.end:
    popad
    ret

;; draws a 32x32 texture
draw_texture:
    pushad

    mov     edi, screen
    mov     esi, textures
    mov     edx, TEX_SIZE               ; EDX = scanlines to copy

.loop_y:
    xor     ecx, ecx                    ; ECX = x offset

.loop_x:
    mov     al, byte [esi + ecx]
    mov     byte [edi + ecx], al

    inc     ecx                         ; next x offset
    cmp     ecx, TEX_SIZE
    jl      .loop_x

    add     edi, SCR_PITCH              ; next scanline
    add     esi, TEX_SIZE               ; next texture line
    dec     edx
    jnz     .loop_y

    popad
    ret

;; Clears the backbuffer
;; void draw_clear(void)
draw_clear:
    mov     edx, screen
    mov     cx,  0xFFFF
    xor     ax, ax

.loop:
    mov     byte [edx], 06h
    inc     edx
    dec     cx
    jnz     .loop
    ret

;; Renders the full palette range to the backbuffer
;; void draw_fill_screen
draw_fill_screen:
    mov     edx, screen
    mov     cx,  0xFFFF
    xor     ax, ax

.loop:
    mov     byte [edx], al
    inc     edx
    inc     al
    dec     cx
    jnz     .loop
    ret

;; 2D top-down view of the map for debugging
;; void draw_world(void)
draw_world:
    pushad

    xor     edx, edx
    xor     eax, eax                    ; map y offset
    xor     edi, edi                    ; render line offset

.loop_y:
    xor     ecx, ecx                    ; map x offset
    xor     esi, esi                    ; render x offset

.loop_x:
    mov     al, byte [map + ecx + edx]
    cmp     al, '.'
    je      .floor

    push    edx
    push    ecx                         ; save ecx

    mov     edx, TEX_SIZE               ; width
    mov     ebx, TEX_SIZE               ; height
    mov     ecx, 0Ah                    ; color
    call    draw_rect                   ; draw

    pop     ecx
    pop     edx
.floor:
    add     esi, TEX_SIZE
    inc     ecx
    cmp     ecx, MAP_W
    jne     .loop_x

    add     edi, TEX_SIZE
    add     edx, MAP_W
    cmp     edx, MAP_W*MAP_H
    jne     .loop_y

    popad
    ret

;; 2D player position and angle for debugging
;; void draw_player(void)
draw_player:
    pushad
    mov     ebp, esp
    sub     esp, 8

    fld     dword [origin_x]
    fistp   dword [ebp - 4]
    fld     dword [origin_y]
    fistp   dword [ebp - 8]

    mov     esi, [ebp - 4]
    mov     edi, [ebp - 8]
    mov     edx, 4
    mov     ebx, 4
    mov     ecx, 0Bh
    call    draw_rect

    fld     dword [forward_x]
    fmul    dword [ftile_size]
    fistp   dword [ebp - 4]
    fld     dword [forward_y]
    fmul    dword [ftile_size]
    fistp   dword [ebp - 8]

    add     esi, [ebp - 4]
    add     edi, [ebp - 8]
    mov     edx, 2
    mov     ebx, 2
    mov     ecx, 0Fh
    call    draw_rect

    mov     esp, ebp
    popad
    ret

;; Interrupts
;; ---------------------------------------------------------------------------

int_handler:
    iret

timer_handler:
    pushad

    mov     dword [start_frame], 1

    mov     al, 20h                     ; finished with interrupt
    out     20h, al

    popad
    iret

key_handler:
    pushad

    in      al, 60h                     ; read PS/2 keyboard
    mov     bl, al                      ; copy the original
    and     al, ~80h                    ; disable release bit for comparisons

    cmp     al, 11h                     ; W
    jne     .lookleft
    mov     edx, BTN_FORWARD
    jmp     .pressed

.lookleft:
    cmp     al, 4Bh                     ; left arrow
    jne     .back
    mov     edx, BTN_LOOKLEFT
    jmp     .pressed

.back:
    cmp     al, 1Fh                     ; S
    jne     .lookright
    mov     edx, BTN_BACK
    jmp     .pressed

.lookright:
    cmp     al, 4Dh                     ; right arrow
    jne     .strafeleft
    mov     edx, BTN_LOOKRIGHT
    jmp     .pressed

.strafeleft:
    cmp     al, 1Eh                     ; A
    jne     .straferight
    mov     edx, BTN_STRAFELEFT
    jmp     .pressed

.straferight:
    cmp     al, 20h                     ; D
    jne     .mouse_enable
    mov     edx, BTN_STRAFERIGHT
    jmp     .pressed

.mouse_enable:
    cmp     al, 3Ch                     ; F2
    jne     .end
    and     bl, 80h
    jz      .end                        ; only handle release
    mov     eax, 1
    xor     [enablemouse], eax          ; turn mouse on or off
    jmp     .end

.pressed:
    and     bl, 80h                     ; release bit
    jnz     .released
    or      dword [buttons], edx
    jmp     .end

.released:
    not     edx
    and     dword [buttons], edx

.end:
    mov     al, 20h                     ; finished with interrupt
    out     20h, al

    popad
    iret

mouse_handler:
    pushad
    in      al, 60h

    mov     al, 20h                     ; finished with interrupt
    out     20h, al

    popad
    iret

;; input
buttons     dd  0
start_frame dd  1                       ; signal to begin a new frame
enablemouse dd  1

;; player
origin_x    dd  48.0
origin_y    dd  48.0
angle       dd  0
forward_x   dd  1.0
forward_y   dd  0.0
right_x     dd  0.0
right_y     dd  1.0
velocity_x  dd  0.0
velocity_y  dd  0.0

;; common float values
fzero       dd  0.0
fnzero      dd -0.0
flt1        dd  1.0
fhalf       dd  0.5
fquarter    dd  0.25
fanglestep  dd  0.1875                  ; 60 / SCR_PITCH
feps        dd  0.001
fneps       dd -0.001
ftile_size  dd  32.0
fntile_size dd -32.0
flarge      dd  4096.0
f30         dd  30.0
fdeg2rad    dd  0.017453292
f200        dd  200.0
f180        dd  180.0
f90         dd  90.0
f270        dd  270.0
f360        dd  360.0
f100        dd  100.0
fplane      dd  5000.0

;; physics (values are per frame)
friction    dd 0.80
accel       dd 2.0
maxspeed    dd 2.0
stopspeed   dd 3.125
maxaccel    dd 1.333

;; use this address to draw a value on screen
preg        dd 0Ah

;; world
map:
    db "##############################"
    db "#...#.#....##........#.......#"
    db "#...#................#.......#"
    db "#.......#...#........#.......#"
    db "#...#......##........#.......#"
    db "#####.##########.#######.#####"
    db "#...#......#.................#"
    db "#...#...............#........#"
    db "#...#......#.......###.......#"
    db "#..........#........#........#"
    db "#..........#.................#"
    db "##############################"

;; 8x8 pixel font
font:
    dq 007CE6F6DECEC67Ch, 007E181818187818h, 00FE6630180CC67Ch, 007CC6063C06C67Ch ; 0-3
    dq 000C0CFE6C3C1C0Ch, 007CC60606FCC0FEh, 007CC6C6FCC0C67Ch, 001818180C06C6FEh ; 4-7
    dq 007CC6C67CC6C67Ch, 007CC6067EC6C67Ch, 00C6C6FEC6C66C38h, 00FC66667C6666FCh ; 8-B
    dq 003C66C0C0C0663Ch, 00F86C6666666CF8h, 00FEC2C0F8C0C2FEh, 00F060607C6062FEh ; C-F

;; Quake palette converted to 18bit color
pal:
    dd 0000000h, 0004104h, 0008208h, 000C30Ch, 0010410h, 00134D3h, 0016596h, 001A69Ah
    dd 001E79Eh, 00228A2h, 00269A6h, 002AAAAh, 002EBAEh, 0032CB2h, 0036DB6h, 003AEBAh
    dd 00040C2h, 0006103h, 0008183h, 000A1C4h, 000C245h, 000E2C6h, 0010306h, 0013387h
    dd 00153C7h, 0016448h, 00184C8h, 001A548h, 001C548h, 001E5C9h, 0020649h, 00236C9h
    dd 00030C4h, 0005147h, 00071CAh, 000A28Dh, 000C310h, 000E393h, 0010415h, 0012499h
    dd 001451Ch, 001659Fh, 0018622h, 001A6A5h, 001C728h, 001E7ABh, 002082Eh, 00228B2h
    dd 0000000h, 0002080h, 00030C0h, 0005140h, 00071C0h, 0009240h, 000B2C2h, 000C302h
    dd 000E382h, 0010402h, 0012482h, 00134C3h, 0015543h, 0016583h, 0018603h, 001A684h
    dd 0002000h, 0004000h, 0006000h, 0008000h, 000A000h, 000C000h, 000E000h, 0010000h
    dd 0012000h, 0014000h, 0015000h, 0017000h, 0019000h, 001B000h, 001D000h, 001F000h
    dd 0005140h, 00071C0h, 0009240h, 000C2C0h, 000E300h, 0011380h, 00133C2h, 0015442h
    dd 0017482h, 001A4C3h, 001D544h, 0020545h, 0022585h, 00255C7h, 0028608h, 002B649h
    dd 0009142h, 000C183h, 000F204h, 0013245h, 00152C6h, 0018308h, 001C389h, 001F3CBh
    dd 002344Dh, 002750Dh, 002B60Ch, 002F74Ch, 00338CBh, 0037A8Ah, 003BC88h, 003FF07h
    dd 0003080h, 0007140h, 000B244h, 000E2C5h, 0012347h, 0015389h, 001840Bh, 001B48Dh
    dd 001F550h, 00225D2h, 0026695h, 0029797h, 002D85Ah, 003091Eh, 0034A22h, 0038B25h
    dd 002A8A8h, 00277E5h, 0024721h, 002265Eh, 001F59Bh, 001D558h, 001A4D5h, 0017413h
    dd 0015391h, 001330Eh, 001128Ch, 000E209h, 000B187h, 0009145h, 00060C3h, 0004082h
    dd 002E727h, 002B6A3h, 00285E0h, 002555Dh, 002251Ah, 001F4D7h, 001C455h, 001A3D3h
    dd 0017350h, 00152CEh, 001224Bh, 000F209h, 000C187h, 0009145h, 00060C3h, 0004082h
    dd 0036C2Eh, 0032B29h, 002FA26h, 002B962h, 002885Eh, 002579Bh, 00216D7h, 001E615h
    dd 001A552h, 00174CFh, 001540Dh, 001134Ah, 000E2C8h, 000A206h, 0007144h, 00040C2h
    dd 001B81Eh, 001979Bh, 0017719h, 0015697h, 0014615h, 0012594h, 0010552h, 000E4D0h
    dd 000C44Eh, 000B3CCh, 000934Ah, 00082C8h, 0006246h, 00041C5h, 0003143h, 00020C2h
    dd 003FF07h, 003BDC6h, 0036C85h, 0032B44h, 002EA44h, 002A943h, 0026802h, 0022702h
    dd 001E602h, 001A540h, 0016480h, 0013380h, 000F2C0h, 000B200h, 0007100h, 0003080h
    dd 000003Fh, 00030FBh, 0005177h, 00071F3h, 000926Fh, 000B2EBh, 000C327h, 000C323h
    dd 000C31Fh, 000C31Bh, 000C317h, 000B2D4h, 0009250h, 00071CCh, 0005148h, 00030C4h
    dd 000B000h, 000F000h, 0013080h, 0017080h, 001B100h, 001F182h, 0024202h, 0028283h
    dd 002D344h, 00304C7h, 003360Bh, 00367CFh, 0038954h, 0039A97h, 003BBDDh, 003DD22h
    dd 002978Fh, 002D98Eh, 0031C0Eh, 0039E15h, 001FBFFh, 002AE7Fh, 0035FFFh, 0019000h
    dd 0022000h, 002C000h, 0035000h, 003F000h, 003FF24h, 003FF71h, 003FFFFh, 0027595h

textures:
    %include "src/textures.asm"

;; 64 sectors read from disk
    times (64*512)-($-$$) db 0

;; Data
;; ---------------------------------------------------------------------------

screen:
    times (SCR_PITCH * SCR_LINES) db 0

;; Floppy
;; ---------------------------------------------------------------------------
    times (360*1024)-($-$$) db 0
