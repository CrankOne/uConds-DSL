a.out: spd-filters-bsp-2.cpp
	g++ -g -Wall -std=gnu++14 spd-filters-bsp-2.cpp

clean:
	rm -rf a.out

.PHONY: clean
