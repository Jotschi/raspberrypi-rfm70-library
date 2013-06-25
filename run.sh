
rm *.o
rm *.a
rm test
rm rfmTest
clear

g++ -c rfm70.cpp -L/usr/local/lib -o librfm70.o
#g++ -c rfm70.cpp -L/usr/local/lib -lwiringPi -o librfm70.o

ar rvs librfm70.a librfm70.o

#g++ rpi_rfm70_test.cpp  -I. -I/usr/local/include -L/root/wiringPi/rfm70/ -L/usr/local/lib/ -lrfm70 -o test
g++ rpi_rfm70_test.cpp  -I. -I/usr/local/include -L/root/wiringPi/rfm70/ -L/usr/local/lib/ -lrfm70 -lwiringPi -o rfmTest
