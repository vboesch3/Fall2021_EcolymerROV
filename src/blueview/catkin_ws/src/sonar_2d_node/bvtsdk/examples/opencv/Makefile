LDFLAGS=-L../../lib -Wl,-rpath ../../lib -L/usr/local/lib
LDLIBS=-lbvtsdk3 -lcxcore -lcv -lhighgui -lcvaux
CPPFLAGS=-I../../include -I/usr/local/include/opencv/
CC=g++

all: opencv
clean:
	rm opencv opencv.o

opencv: opencv.o

