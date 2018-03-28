.PHONY: all

CC = gcc
CFLAGS = #-Wall
LFLAGS = -lm
OBJS = *.o
CFILES = libraries/fault_sdk/*.c
TESTFILE = test.c
HFILES = libraries/fault_sdk/*.h
TESTMAIN = test


all: $(CFILES) $(HFILES)
	$(CC) $(CFLAGS) -c $(CFILES)	
	$(CC) $(CFLAGS) -c $(TESTFILE)	
	@echo "Compilation complete"
	

test: 
	make all
	$(CC) $(CFLAGS) $(LFLAGS) -o $(TESTMAIN) $(OBJS)
	@echo "Linking complete!"
	@echo "Executing test!"
	./test
clean: 
	rm -f $(OBJS)
	rm -f *.o test
	@echo "Cleanup complete!"
