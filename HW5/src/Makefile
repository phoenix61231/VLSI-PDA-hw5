CC = gcc
CFLAGS = -Wall -ansi
SOURCES = main.c
HEADERS = $(SOURCES:.c=.h)
OBJECTS = $(SOURCES:.c=.o)
EXECUTABLE = ../bin/hw5

$(EXECUTABLE): $(OBJECTS)
	$(CC) -o $(EXECUTABLE) $(OBJECTS) -lm

main.o : main.c $(HEADERS)
	$(CC) $(CFLAGS) -c main.c -lm

clean:
	rm $(EXECUTABLE) $(OBJECTS) 
