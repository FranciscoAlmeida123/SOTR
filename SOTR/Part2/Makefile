DEFAULT_TARGET=inverted_pendulum
OBJECTS = inverted_pendulum.o

CFLAGS = -g -Wall -O2 
LDLIBS= -lm -lpthread
CC=gcc


# Generate application
$(DEFAULT_TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(DEFAULT_TARGET) $(OBJECTS) $(LDLIBS)
	
%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@
	
clean:
	rm -f *.o $(DEFAULT_TARGET)

run:	$(DEFAULT_TARGET)
	./$(DEFAULT_TARGET)
