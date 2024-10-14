CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -pedantic
DEPS = pid_controller.h
OBJ = pid_controller.o main.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

pid_controller: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o pid_controller