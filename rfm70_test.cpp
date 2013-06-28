#include "rfm70.h"
#include <stdio.h>
#include <time.h>
#include <unistd.h>

RFM70 rfm70(10, 0, -1, RFM77_DEFAULT_SPI_CLOCK_DIV);

void receiveEvent(void) {
}

void setup() {
	rfm70.begin();
	rfm70.onReceive(receiveEvent);
}

void loop() {
	printf("Sending data\n\n");
	usleep(10000);
	rfm70.send((uint8_t*) "hello!", 6);
	rfm70.tick();
}

int main(void) {
	setup();

	for (;;) {
		loop();
	}

}

