#in makefiles, a hash indicates a comment instead
all: period
period: period.c
	gcc -o period period.c -lwiringPi
