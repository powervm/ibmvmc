
OBJS = HvHmcConnect.o  HvHmcLessCmd.o

.PHONY: all clean
all: connect test1 test2

connect:    $(OBJS)
	$(CXX) -g HvHmcConnect.o  HvHmcLessCmd.o -o connect
HvHmcConnect.o  :   HvHmcLessCmd.H HmcLessTypes.H
HvHmcLessCmd.o  :   HvHmcLessCmd.H

test1: test1.c
	$(CC) -g test1.c -o $@
test2: test2.c
	$(CC) -g test2.c -o $@

clean:
	rm -f *.o connect test1 test2
