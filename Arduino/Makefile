.PHONY: all

CC = gcc
CFLAGS = -Wall
LFLAGS = -lm
OBJS = *.o
MAIN = test

all: *.c *.h
	$(CC) $(CFLAGS) -c *.c
	

test: 
	make all
	$(CC) $(CFLAGS) $(LFLAGS) -o $(MAIN) $(OBJS)
	./test
clean: 
	rm *.o test
