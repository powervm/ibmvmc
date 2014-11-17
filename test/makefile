
DEBUG = -g
CXXFLAGS = $(DEBUG)
CCFLAGS = $(DEBUG)
EXES = connect

connect_OBJS = HvHmcConnect.o  HvHmcLessCmd.o

.PHONY: all clean
all: $(EXES)

connect:    $(connect_OBJS)
	$(CXX) $(CXXFLAGS) $(connect_OBJS) -o $@
HvHmcConnect.o  :   HvHmcLessCmd.H HmcLessTypes.H
HvHmcLessCmd.o  :   HvHmcLessCmd.H

clean:
	rm -f *.o $(EXES)
