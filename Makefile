#DEBUG  = -g -O0
DEBUG   = -O3
CC      = g++
INCLUDEPATH = -I/usr/local/include -I.
CFLAGS  = $(DEBUG) -Wall $(INCLUDE) -Winline -pipe

LDFLAGS = -L/usr/local/lib
LIBS    = -lwiringPi

SRC     =       rfm70_test.c

OBJ     =       rfm70_test.o 

all:            rfm70 rfm70_test

rfm70:		rfm70.o
	@echo [link]
	$(CC) -c rfm70.cpp -o librfm70.o $(LDFLAGS) $(LIBS)
	ar rvs librfm70.a librfm70.o

rfm70_test:        rfm70_test
	@echo [link]
	$(CC) rfm70_test.cpp -o $@ -I. $(LDFLAGS) -L. $(LIBS) -lrfm70
        
.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ) *~ core tags rfm70_test

tags:   $(SRC)
	@echo [ctags]
	@ctags $(SRC)

depend:
	makedepend -Y $(SRC)

