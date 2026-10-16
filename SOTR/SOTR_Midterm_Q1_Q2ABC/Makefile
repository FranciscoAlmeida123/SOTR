CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic
LDLIBS = -pthread -lrt

all: exercise1 exercise2_ABC

exercise1: exercise1.c
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

exercise2_ABC: exercise2_ABC.c
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

.PHONY: all
