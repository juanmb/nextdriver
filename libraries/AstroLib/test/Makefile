
LIBS=../

test.run: main.o astrolib.o
	g++ -o $@ $^ -lm -lgtest -lpthread

astrolib.o: $(LIBS)/AstroLib.cpp $(LIBS)/AstroLib.h
	g++ -o $@ -c -I$(LIBS) $(LIBS)/AstroLib.cpp

main.o: main.cpp
	g++ -o $@ -c -I$(LIBS) main.cpp

format:
	astyle --style=stroustrup -p -c -s4 *.cpp

clean:
	rm -f test.run
