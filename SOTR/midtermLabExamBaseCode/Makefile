CC = gcc
CFLAGS = -Wall -Wextra -g -pthread
LDLIBS = -lrt -lm

all: templateCodeThreads
.PHONY: all

templateCodeThreads: templateCodeThreads.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)
	
.PHONY: clean 

clean:
	rm -f *~ 
	rm -f templateCodeThreads
			
# Some notes
# $@ represents the left side of the ":"
# $^ represents the right side of the ":"
# $< represents the first item in the dependency list   

