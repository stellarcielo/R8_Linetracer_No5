CC = gcc
BIN = main manual_drive main_t2
CFLAGS = -Wall -pthread

all:	$(BIN)

c.o:
	$(CC) $(CFLAGS) -c $< -o $@ -lpigpiod_if2 -lrt

main:	main.o motor.o
	$(CC) $(CFLAGS) -o $@ $^ -lpigpiod_if2 -lrt

main_t:	main_t.o
	$(CC) $(CFLAGS) -o $@ $^ -lpigpiod_if2 -lrt

main_t2:	main_t2.o
	$(CC) $(CFLAGS) -o $@ $^ -lpigpiod_if2 -lrt

manual_drive:	manual_drive.o
	$(CC) $(CFLAGS) -o $@ $^ -lpigpiod_if2 -lrt

clean:
	rm -f *.o $(BIN)