all: test-uc-dsl

test-uc-dsl: test-uc-dsl.cpp
	g++ -c -g -Wall -std=gnu++14 $^ -o $@

clean:
	rm -rf uc-dsl-test


.PHONY: clean all
