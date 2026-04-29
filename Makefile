CC = gcc
BIN = main
CFLAGS = -Wall -pthread -lpigpiod_if2 -lrt

all:	$(BIN)

c.o:
	$(CC) $(CFLAGS) -c $< -o $@

main:	main.o motor.o
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f *.o $(BIN)