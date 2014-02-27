#!/usr/bin/env python

def rx(i):
    return i + 2

def ry(i):
    return i + 12

#### set up registers
print r'"adiw zh:zl, 10\n\t"'
print r'"adiw yh:yl, 10\n\t"'

for i in xrange(10):
    print r'"ld r%s, x+ \n\t"' % (rx(i))
for i in xrange(10):
    print r'"ld r%s, y+ \n\t"' % (ry(i))

#### first two multiplications of initial block (x = 0-9, y = 10-19)
print r'"ldi r25, $0 \n\t"'
print ""
print r'"ldi r23, $0 \n\t"'
print r'"mul r2, r12 \n\t"'
print r'"st r0, z+ \n\t"'
print r'"mov r22, r1 \n\t"'
print ""
print r'"ldi r24, $0 \n\t"'
print r'"mul r2, r13 \n\t"'
print r'"add r22, r0 \n\t"'
print r'"adc r23, r1 \n\t"'
print r'"mul r3, r12 \n\t"'
print r'"add r22, r0 \n\t"'
print r'"adc r23, r1 \n\t"'
print r'"adc r24, r25 \n\t"'
print r'"st r22, z+ \n\t"'
print ""

#### rest of initial block, with moving accumulator registers
acc = [23, 24, 22]
for r in xrange(2, 10):
    print r'"ldi r%s, $0 \n\t"' % (acc[2])
    for i in xrange(0, r+1):
        print r'"mul r%s, r%s \n\t"' % (rx(i), ry(r - i))
        print r'"add r%s, r0 \n\t"' % (acc[0])
        print r'"adc r%s, r1 \n\t"' % (acc[1])
        print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"st r%s, z+ \n\t"' % (acc[0])
    print ""
    acc = acc[1:] + acc[:1]
for r in xrange(1, 9):
    print r'"ldi r%s, $0 \n\t"' % (acc[2])
    for i in xrange(0, 10-r):
        print r'"mul r%s, r%s \n\t"' % (rx(r+i), ry(9 - i))
        print r'"add r%s, r0 \n\t"' % (acc[0])
        print r'"adc r%s, r1 \n\t"' % (acc[1])
        print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"st r%s, z+ \n\t"' % (acc[0])
    print ""
    acc = acc[1:] + acc[:1]
print r'"mul r%s, r%s \n\t"' % (rx(9), ry(9))
print r'"add r%s, r0 \n\t"' % (acc[0])
print r'"adc r%s, r1 \n\t"' % (acc[1])
print r'"st r%s, z+ \n\t"' % (acc[0])
print r'"st r%s, z+ \n\t"' % (acc[1])
print ""

#### reset y and z pointers (x still points to left + 10)
print r'"sbiw zh:zl, 30\n\t"'
print r'"sbiw yh:yl, 20\n\t"'

#### load y registers
for i in xrange(10):
    print r'"ld r%s, y+ \n\t"' % (ry(i))
print ""

#### do x = 0-9, y = 0-9 multiplications
print r'"ldi r23, $0 \n\t"'
print r'"mul r2, r12 \n\t"'
print r'"st r0, z+ \n\t"'
print r'"mov r22, r1 \n\t"'
print ""
print r'"ldi r24, $0 \n\t"'
print r'"mul r2, r13 \n\t"'
print r'"add r22, r0 \n\t"'
print r'"adc r23, r1 \n\t"'
print r'"mul r3, r12 \n\t"'
print r'"add r22, r0 \n\t"'
print r'"adc r23, r1 \n\t"'
print r'"adc r24, r25 \n\t"'
print r'"st r22, z+ \n\t"'
print ""

acc = [23, 24, 22]
for r in xrange(2, 10):
    print r'"ldi r%s, $0 \n\t"' % (acc[2])
    for i in xrange(0, r+1):
        print r'"mul r%s, r%s \n\t"' % (rx(i), ry(r - i))
        print r'"add r%s, r0 \n\t"' % (acc[0])
        print r'"adc r%s, r1 \n\t"' % (acc[1])
        print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"st r%s, z+ \n\t"' % (acc[0])
    print ""
    acc = acc[1:] + acc[:1]

#### now we need to start shifting x and loading from z
x_regs = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
for r in xrange(0, 10):
    x_regs = x_regs[1:] + x_regs[:1]
    print r'"ld r%s, x+ \n\t"' % (x_regs[9]) # load next byte of left
    print r'"ldi r%s, $0 \n\t"' % (acc[2])
    for i in xrange(0, 10):
        print r'"mul r%s, r%s \n\t"' % (x_regs[i], ry(9 - i))
        print r'"add r%s, r0 \n\t"' % (acc[0])
        print r'"adc r%s, r1 \n\t"' % (acc[1])
        print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"ld r0, z \n\t"' # load stored value from initial block, and add to accumulator (note z does not increment)
    print r'"add r%s, r0 \n\t"' % (acc[0])
    print r'"adc r%s, r25 \n\t"' % (acc[1])
    print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"st r%s, z+ \n\t"' % (acc[0]) # store next byte (z increments)
    print ""
    acc = acc[1:] + acc[:1]

# done shifting x, start shifting y
y_regs = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
for r in xrange(0, 10):
    y_regs = y_regs[1:] + y_regs[:1]
    print r'"ld r%s, y+ \n\t"' % (y_regs[9]) # load next byte of right
    print r'"ldi r%s, $0 \n\t"' % (acc[2])
    for i in xrange(0, 10):
        print r'"mul r%s, r%s \n\t"' % (x_regs[i], y_regs[9 -i])
        print r'"add r%s, r0 \n\t"' % (acc[0])
        print r'"adc r%s, r1 \n\t"' % (acc[1])
        print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"ld r0, z \n\t"' # load stored value from initial block, and add to accumulator (note z does not increment)
    print r'"add r%s, r0 \n\t"' % (acc[0])
    print r'"adc r%s, r25 \n\t"' % (acc[1])
    print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"st r%s, z+ \n\t"' % (acc[0]) # store next byte (z increments)
    print ""
    acc = acc[1:] + acc[:1]

# done both shifts, do remaining corner
for r in xrange(1, 9):
    print r'"ldi r%s, $0 \n\t"' % (acc[2])
    for i in xrange(0, 10-r):
        print r'"mul r%s, r%s \n\t"' % (x_regs[r+i], y_regs[9 - i])
        print r'"add r%s, r0 \n\t"' % (acc[0])
        print r'"adc r%s, r1 \n\t"' % (acc[1])
        print r'"adc r%s, r25 \n\t"' % (acc[2])
    print r'"st r%s, z+ \n\t"' % (acc[0])
    print ""
    acc = acc[1:] + acc[:1]
print r'"mul r%s, r%s \n\t"' % (x_regs[9], y_regs[9])
print r'"add r%s, r0 \n\t"' % (acc[0])
print r'"adc r%s, r1 \n\t"' % (acc[1])
print r'"st r%s, z+ \n\t"' % (acc[0])
print r'"st r%s, z+ \n\t"' % (acc[1])
