CC=gcc

BUILD_CONFIG = -g -DDEBUG

CFLAGS=   $(BUILD_CONFIG)  -Wall -Wextra 
CFLAGS+=  -pedantic  -D_REENTRANT


CFLAGS+=  -I../src/
# -I/usr/local/include/GroundBase -I/usr/local/include/FlyKit/ 

#LDFLAGS=  -lpthread -lFlyKit -lGroundBase 


SOURCES= main.c \
		../src/RosMessages.c \
		../src/TurtleBotSerial.c \

OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE= Example1

ll: $(SOURCES) $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS) 

.c.o:
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS)
	rm -f $(EXECUTABLE)

fclean: clean
	rm -f $(EXECUTABLE)

re: clean all

