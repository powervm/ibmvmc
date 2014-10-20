
OBJS = HvHmcConnect.o  HvHmcLessCmd.o

connect:    $(OBJS)
		g++ -g HvHmcConnect.o  HvHmcLessCmd.o -o connect
HvHmcConnect.o  :   HvHmcLessCmd.H HmcLessTypes.H
HvHmcLessCmd.o  :   HvHmcLessCmd.H
