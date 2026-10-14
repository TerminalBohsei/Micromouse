# Host-side tests (no ESP32 needed):  make test
FW  = ../firmware/Micromouse
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I$(FW) -Isim/shim

all: test_unit sim_run

test_unit: test_unit.cpp $(FW)/Decision.h $(FW)/RangeFilter.h $(FW)/MotorMap.h
	$(CXX) -std=c++11 -Wall -Wextra -Werror -pedantic -I$(FW) $< -o $@

sim_run: sim/sim_main.cpp sim/world.cpp sim/mocks.cpp $(FW)/Navigator.cpp $(FW)/Floor.cpp $(FW)/Stepper.cpp
	$(CXX) $(CXXFLAGS) -Isim $^ -o $@

test: all
	./test_unit
	./sim_run

clean:
	rm -f test_unit sim_run
