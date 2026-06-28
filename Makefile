# DXD-e tools makefile

CC := gcc

utils := generate-dsd generate-dxd extract-dsd corrupt-dxd decode-dxd

all: $(utils)

generate-dsd: generate-dsd.c modulator.c modulator.h decimator.c biquad.c decimator.h biquad.h Makefile
	$(CC) -Wall -Ofast generate-dsd.c modulator.c decimator.c biquad.c workers.c -lm -o generate-dsd

generate-dxd: generate-dxd.c decimator.c biquad.c decimator.h biquad.h  Makefile
	$(CC) -Wall -O2 generate-dxd.c decimator.c biquad.c -lm -o generate-dxd

decode-dxd: decode-dxd.c modulator.c modulator.h decimator.c biquad.c decimator.h biquad.h Makefile
	$(CC) -Wall -Ofast decode-dxd.c modulator.c decimator.c biquad.c workers.c -lm -o decode-dxd

extract-dsd: extract-dsd.c Makefile
	$(CC) -Wall -O2 extract-dsd.c -lm -o extract-dsd

corrupt-dxd: corrupt-dxd.c Makefile
	$(CC) -Wall -O2 corrupt-dxd.c -lm -o corrupt-dxd

clean:
	rm -f generate-dsd generate-dxd extract-dsd corrupt-dxd decode-dxd
