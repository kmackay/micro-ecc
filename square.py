#!/usr/bin/env python

def r(i):
    return i + 2

def emit(line, *args):
    s = '"' + line + r' \n\t"'
    print s % args

#### set up registers

for i in xrange(20):
    emit("ld r%s, x+", r(i))

#### first few columns
emit("ldi r27, 0") # zero register
print ""
emit("ldi r23, 0")
emit("mul r2, r2")
emit("st z+, r0")
emit("mov r22, r1")
print ""
emit("ldi r24, 0")
emit("mul r2, r3")
emit("lsl r0")
emit("rol r1")
emit("adc r24, r27") # put carry bit in r24
emit("add r22, r0")
emit("adc r23, r1")
emit("adc r24, r27")
emit("st z+, r22")
print ""
emit("ldi r22, 0")
emit("mul r2, r4")
emit("lsl r0")
emit("rol r1")
emit("adc r22, r27") # put carry bit in r22
emit("add r23, r0")
emit("adc r24, r1")
emit("adc r22, r27")
emit("mul r3, r3")
emit("add r23, r0")
emit("adc r24, r1")
emit("adc r22, r27")
emit("st z+, r23")
print ""

acc = [23, 24, 22]
old_acc = [25, 26]
for i in xrange(3, 20):
    emit("ldi r%s, 0", acc[0])
    emit("ldi r%s, 0", old_acc[0])
    emit("ldi r%s, 0", old_acc[1])
    tmp = [acc[1], acc[2]]
    acc = [acc[0], old_acc[0], old_acc[1]]
    old_acc = tmp
    
    # gather non-equal words
    for j in xrange(0, (i+1)//2):
        emit("mul r%s, r%s", r(j), r(i-j))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r27", acc[2])
    # multiply by 2
    emit("lsl r%s", acc[0])
    emit("rol r%s", acc[1])
    emit("rol r%s", acc[2])
    
    # add equal word (if any)
    if ((i+1) % 2) != 0:
        emit("mul r%s, r%s", r(i//2), r(i//2))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r27", acc[2])
    
    # add old accumulator
    emit("add r%s, r%s", acc[0], old_acc[0])
    emit("adc r%s, r%s", acc[1], old_acc[1])
    emit("adc r%s, r27", acc[2])
    
    # store
    emit("st z+, r%s", acc[0])
    print ""

for i in xrange(1, 17):
    emit("ldi r%s, 0", acc[0])
    emit("ldi r%s, 0", old_acc[0])
    emit("ldi r%s, 0", old_acc[1])
    tmp = [acc[1], acc[2]]
    acc = [acc[0], old_acc[0], old_acc[1]]
    old_acc = tmp

    # gather non-equal words
    for j in xrange(0, (20-i)//2):
        emit("mul r%s, r%s", r(i+j), r(19-j))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r27", acc[2])
    # multiply by 2
    emit("lsl r%s", acc[0])
    emit("rol r%s", acc[1])
    emit("rol r%s", acc[2])

    # add equal word (if any)
    if ((20-i) % 2) != 0:
        emit("mul r%s, r%s", r(i + (20-i)//2), r(i + (20-i)//2))
        emit("add r%s, r0", acc[0])
        emit("adc r%s, r1", acc[1])
        emit("adc r%s, r27", acc[2])

    # add old accumulator
    emit("add r%s, r%s", acc[0], old_acc[0])
    emit("adc r%s, r%s", acc[1], old_acc[1])
    emit("adc r%s, r27", acc[2])

    # store
    emit("st z+, r%s", acc[0])
    print ""

acc = acc[1:] + acc[:1]
emit("ldi r%s, 0", acc[2])
emit("mul r19, r21")
emit("lsl r0")
emit("rol r1")
emit("adc r%s, r27", acc[2])
emit("add r%s, r0", acc[0])
emit("adc r%s, r1", acc[1])
emit("adc r%s, r27", acc[2])
emit("mul r20, r20")
emit("add r%s, r0", acc[0])
emit("adc r%s, r1", acc[1])
emit("adc r%s, r27", acc[2])
emit("st z+, r%s", acc[0])
print ""

acc = acc[1:] + acc[:1]
emit("ldi r%s, 0", acc[2])
emit("mul r20, r21")
emit("lsl r0")
emit("rol r1")
emit("adc r%s, r27", acc[2])
emit("add r%s, r0", acc[0])
emit("adc r%s, r1", acc[1])
emit("adc r%s, r27", acc[2])
emit("st z+, r%s", acc[0])
print ""

emit("mul r21, r21")
emit("add r%s, r0", acc[1])
emit("adc r%s, r1", acc[2])
emit("st z+, r%s", acc[1])

emit("st z+, r%s", acc[2])
emit("eor r1, r1")
