# DXD-e tools makefile

CC := gcc

utils := generate-dsd generate-dxd extract-dsd

all: $(utils)

generate-dsd: generate-dsd.c modulator.c modulator.h Makefile
	$(CC) -Wall -Ofast generate-dsd.c modulator.c workers.c -lm -o generate-dsd

generate-dxd: generate-dxd.c decimator.c biquad.c decimator.h biquad.h 
	$(CC) -Wall -O2 generate-dxd.c decimator.c biquad.c -lm -o generate-dxd

extract-dsd: extract-dsd.c
	$(CC) -Wall -O2 extract-dsd.c -lm -o extract-dsd

clean:
	rm -f generate-dsd generate-dxd extract-dsd
