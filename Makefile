CXXFLAGS =	-O3 -g -Wall -fmessage-length=0

OBJS =		Main.o Framework.o

LIBS =

TARGET =	Main

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
