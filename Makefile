# DXD-e tools makefile

CC := gcc

utils := generate-dsd generate-dxd extract-dsd corrupt-dxd decode-dxd

all: $(utils)

generate-dsd: generate-dsd.c modulator.c modulator.h dsd-utils.c dsd-utils.h Makefile
	$(CC) -Wall -Ofast generate-dsd.c modulator.c dsd-utils.c biquad.c workers.c -lm -o generate-dsd

generate-dxd: generate-dxd.c dsd-utils.c dsd-utils.h biquad.c biquad.h Makefile
	$(CC) -Wall -O2 generate-dxd.c dsd-utils.c biquad.c -lm -o generate-dxd

decode-dxd: decode-dxd.c modulator.c modulator.h dsd-utils.c dsd-utils.h Makefile
	$(CC) -Wall -Ofast decode-dxd.c modulator.c dsd-utils.c biquad.c workers.c -lm -o decode-dxd

extract-dsd: extract-dsd.c dsd-utils.c dsd-utils.h Makefile
	$(CC) -Wall -O2 extract-dsd.c dsd-utils.c biquad.c -lm -o extract-dsd

corrupt-dxd: corrupt-dxd.c Makefile
	$(CC) -Wall -O2 corrupt-dxd.c -lm -o corrupt-dxd

clean:
	rm -f generate-dsd generate-dxd extract-dsd corrupt-dxd decode-dxd
