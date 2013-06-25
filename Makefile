#DEBUG  = -g -O0
DEBUG   = -O3
CC      = g++
INCLUDEPATH = -I/usr/local/include -I.
CFLAGS  = $(DEBUG) -Wall $(INCLUDE) -Winline -pipe

LDFLAGS = -L/usr/local/lib
LIBS    = -lwiringPi

SRC     =       rpi_rfm70_test.c

OBJ     =       rpi_rfm70_test.o 

all:            rpi_rfm70_test

rfm70:		rfm70.o
	@echo [link]
	$(CC) -o $@ rfm70.o $(LDFLAGS) $(LIBS)

rpi_rfm70_test:        rpi_rfm70_test.o
	@echo [link]
	$(CC) -o $@ rpi_rfm70_test.o $(LDFLAGS) $(LIBS)
        
.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ) *~ core tags rpi_rfm70_test

tags:   $(SRC)
	@echo [ctags]
	@ctags $(SRC)

depend:
	makedepend -Y $(SRC)

# DO NOT DELETE
