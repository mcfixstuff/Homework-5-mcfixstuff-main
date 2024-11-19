CPP = g++
CPPFLAGS = -O3 -fPIC -Wno-deprecated-declarations
TARGET = main
OS = $(shell uname)
SOURCE = $(wildcard *.cpp)

default:
	$(CPP) $(SOURCE) $(LIB) -o $(TARGET)

clean:
	rm -f *.o $(TARGET)
