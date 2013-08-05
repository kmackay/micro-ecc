default: makekeys
all: default

ecc.o: ../ecc.c ../ecc.h
	gcc -o ecc.o -c ../ecc.c

makekeys.o: makekeys.c ../ecc.h
	gcc -I.. -o makekeys.o -c makekeys.c

makekeys: ecc.o makekeys.o
	gcc -o makekeys makekeys.o ecc.o

clean:
	rm -f ecc.o makekeys.o makekeys

.PHONY: default all clean
