#!/usr/bin/env python

def rx(i):
    return i + 2

def ry(i):
    return i + 12

def emit(line, *args):
    s = '"' + line + r' \n\t"'
    print s % args

#### set up registers
emit("adiw r30, 10")
emit("adiw r28, 10")

for i in xrange(10):
    emit("ld r%s, x+", rx(i))
for i in xrange(10):
    emit("ld r%s, y+", ry(i))

#### first two multiplications of initial block (x = 0-9, y = 10-19)
emit("ldi r25, 0")
print ""
emit("ldi r23, 0")
emit("mul r2, r12")
emit("st z+, r0")
emit("mov r22, r1")
print ""
emit("ldi r24, 0")
emit("mul r2, r13")
emit("add r22, r0")
emit("adc r23, r1")
emit("mul r3, r12")
emit("add r22, r0")
emit("adc r23, r1")
emit("adc r24, r25")
emit("st z+, r22")
print ""

#### rest of initial block, with moving accumulator registers
acc = [23, 24, 22]
for r in xrange(2, 10):
    emit("ldi r%s, 0", acc[2])
    for i in xrange(0, r+1):
        emit("mul r%s, r%s", rx(i), ry(r - i))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r25", acc[2])
    emit("st z+, r%s", acc[0])
    print ""
    acc = acc[1:] + acc[:1]
for r in xrange(1, 9):
    emit("ldi r%s, 0", acc[2])
    for i in xrange(0, 10-r):
        emit("mul r%s, r%s", rx(r+i), ry(9 - i))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r25", acc[2])
    emit("st z+, r%s", acc[0])
    print ""
    acc = acc[1:] + acc[:1]
emit("mul r%s, r%s", rx(9), ry(9))
emit("add r%s, r0", acc[0])
emit("adc r%s, r1", acc[1])
emit("st z+, r%s", acc[0])
emit("st z+, r%s", acc[1])
print ""

#### reset y and z pointers (x still points to left + 10)
emit("sbiw r30, 30")
emit("sbiw r28, 20")

#### load y registers
for i in xrange(10):
    emit("ld r%s, y+", ry(i))
print ""

#### do x = 0-9, y = 0-9 multiplications
emit("ldi r23, 0")
emit("mul r2, r12")
emit("st z+, r0")
emit("mov r22, r1")
print ""
emit("ldi r24, 0")
emit("mul r2, r13")
emit("add r22, r0")
emit("adc r23, r1")
emit("mul r3, r12")
emit("add r22, r0")
emit("adc r23, r1")
emit("adc r24, r25")
emit("st z+, r22")
print ""

acc = [23, 24, 22]
for r in xrange(2, 10):
    emit("ldi r%s, 0", acc[2])
    for i in xrange(0, r+1):
        emit("mul r%s, r%s", rx(i), ry(r - i))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r25", acc[2])
    emit("st z+, r%s", acc[0])
    print ""
    acc = acc[1:] + acc[:1]

#### now we need to start shifting x and loading from z
x_regs = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
for r in xrange(0, 10):
    x_regs = x_regs[1:] + x_regs[:1]
    emit("ld r%s, x+", x_regs[9]) # load next byte of left
    emit("ldi r%s, 0", acc[2])
    for i in xrange(0, 10):
        emit("mul r%s, r%s", x_regs[i], ry(9 - i))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r25", acc[2])
    emit("ld r0, z") # load stored value from initial block, and add to accumulator (note z does not increment)
    emit("add r%s, r0", acc[0])
    emit("adc r%s, r25", acc[1])
    emit("adc r%s, r25", acc[2])
    emit("st z+, r%s", acc[0]) # store next byte (z increments)
    print ""
    acc = acc[1:] + acc[:1]

# done shifting x, start shifting y
y_regs = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
for r in xrange(0, 10):
    y_regs = y_regs[1:] + y_regs[:1]
    emit("ld r%s, y+", y_regs[9]) # load next byte of right
    emit("ldi r%s, 0", acc[2])
    for i in xrange(0, 10):
        emit("mul r%s, r%s", x_regs[i], y_regs[9 -i])
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r25", acc[2])
    emit("ld r0, z") # load stored value from initial block, and add to accumulator (note z does not increment)
    emit("add r%s, r0", acc[0])
    emit("adc r%s, r25", acc[1])
    emit("adc r%s, r25", acc[2])
    emit("st z+, r%s", acc[0]) # store next byte (z increments)
    print ""
    acc = acc[1:] + acc[:1]

# done both shifts, do remaining corner
for r in xrange(1, 9):
    emit("ldi r%s, 0", acc[2])
    for i in xrange(0, 10-r):
        emit("mul r%s, r%s", x_regs[r+i], y_regs[9 - i])
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r25", acc[2])
    emit("st z+, r%s", acc[0])
    print ""
    acc = acc[1:] + acc[:1]
emit("mul r%s, r%s", x_regs[9], y_regs[9])
emit("add r%s, r0", acc[0])
emit("adc r%s, r1", acc[1])
emit("st z+, r%s", acc[0])
emit("st z+, r%s", acc[1])
emit("eor r1, r1")
