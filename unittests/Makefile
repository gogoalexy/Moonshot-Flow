
CC=g++
CFLAGS=-I. -I../src -I../src/include -lm

all: tests

TEST_FILES=../src/modules/flow/sonar_mode_filter.c \
		tests.c

tests: $(TEST_FILES)
	$(CC) -o tests $(TEST_FILES) $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ tests
