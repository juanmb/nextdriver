
LIBS=../
TIMELIB=../../Time

test.run: main.o astrolib.o
	g++ -o $@ $^ -lm -lgtest -lpthread

#time.o: $(TIMELIB)/Time.cpp $(TIMELIB)/Time.h
	#g++ -o $@ -c -I$(TIMELIB) $(TIMELIB)/Time.cpp

astrolib.o: $(LIBS)/AstroLib.cpp $(LIBS)/AstroLib.h
	g++ -o $@ -c -I$(LIBS) $(LIBS)/AstroLib.cpp

main.o: main.cpp
	g++ -o $@ -c -I$(LIBS) main.cpp

format:
	astyle --style=stroustrup -p -c -s4 *.cpp

clean:
	rm -f test.run
