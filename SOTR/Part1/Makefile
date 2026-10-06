CC = gcc
CFLAGS = -Wall -Wextra -g -pthread
LDLIBS = -lrt -lm

all: ptrt
.PHONY: all

ptrt: periodicTaskRT.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)
	
.PHONY: clean 

clean:
	rm -f *~ 
	rm -f ptrt
		
# Some notes
# $@ represents the left side of the ":"
# $^ represents the right side of the ":"
# $< represents the first item in the dependency list   

