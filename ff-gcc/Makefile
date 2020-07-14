ALL = ff-gcc
current_dir = $(shell pwd)
CXXFLAGS = -std=c++11 -DDIR=\"$(current_dir)/\"

all: $(ALL)

ff-gcc: ff-gcc.o
	$(CXX) $(CXXFLAGS) -o $@ $<

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: clean
clean:
	rm -f $(ALL) *.o
