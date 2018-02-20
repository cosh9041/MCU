.PHONY: all

CC = gcc
CFLAGS = -Wall
LFLAGS = -lm
OBJS = LLAMAS/*.o
CFILES = LLAMAS/*.c
HFILES = LLAMAS/*.h
MAIN = test


all: $(CFILES) $(HFILES)
	$(CC) $(CFLAGS) -c $(CFILES)
	@echo "Compilation complete"
	

test: 
	make all
	$(CC) $(CFLAGS) $(LFLAGS) -o $(MAIN) $(OBJS)
	@echo"Linking complete!"
	@echo "Executing test!"
	./test
clean: 
	rm -f $(OBJS)
	@echo "Cleanup complete!"
