all: test

clean: 
	rm -f test

test: test.cc DT5742.cxx DT5742.h
	g++ -fPIC -g -O2 -c test.cc DT5742.cxx -I.
	g++ -fPIC -g -O2 -o test test.o DT5742.o -L/usr/lib/ -L. -lCAENComm -lCAENDigitizer -lm -lpthread
	rm *.o

#test_Na22: test_Na22.cc DT5742_Na22.cxx DT5742_Na22.h
#	g++ -fPIC -g -O2 -c test_Na22.cc DT5742_Na22.cxx -I.
#	g++ -fPIC -g -O2 -o test_test test_Na22.o DT5742_Na22.o -L/usr/lib/ -L. -lCAENComm -lCAENDigitizer -lm -lpthread
#	rm *.o
