#in makefiles, a hash indicates a comment instead
all: period
hellopi: period.c
	gcc -o period period.c -lwiringPi