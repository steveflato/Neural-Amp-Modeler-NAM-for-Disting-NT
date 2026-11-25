NT_API_PATH ?= ../../ext/distingNT_API
EMB ?= ../../ext/nam-embedded
DEFS = -DNAM_A2LITE_MAX_BUFFER_SIZE=48
all: host_test ref_engine
nam_a2lite.o: ../engine/nam_a2lite.c
	gcc -O2 -ffast-math $(DEFS) -c -o $@ $<
namb_reader.o: ../engine/namb_reader.c
	gcc -O2 -c -o $@ $<
host_test: host_test.cpp ../nam_nt.cpp ../nam_json.h nam_a2lite.o namb_reader.o
	g++ -std=c++14 -O2 -Wall -Wno-unused-function -Wno-missing-field-initializers $(DEFS) -I$(NT_API_PATH)/include -o $@ host_test.cpp nam_a2lite.o namb_reader.o
ref_engine: ref_engine.c nam_a2lite.o
	gcc -O2 $(DEFS) -o $@ ref_engine.c nam_a2lite.o
clean:
	rm -f *.o host_test ref_engine
