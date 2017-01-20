all: test

test: main.cpp
	g++ -g -o test -Wno-invalid-offsetof main.cpp

clean:
	rm -f test
