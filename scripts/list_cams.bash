#!/bin/bash

process_line() {
	VEND=$(echo "$@" | sed 's/.*ID \([0-9a-z]*\).*/\1/' & sed 's/\(\).*/\1/')
	PROD=$(echo "$@" | sed 's/.*ID [0-9a-z]*:\([0-9a-z]*\).*/\1/' & sed 's/\(\).*/\1/')
	BUS=$(echo "$@" | sed 's/.*Bus 0*\([0-9a-z]*\).*/\1/' & sed 's/\(\).*/\1/')
	DEV=$(echo "$@" | sed 's/.*Device 0*\([0-9a-z]*\).*/\1/' & sed 's/\(\).*/\1/')
	echo "$VEND/$PROD@$BUS/$DEV"
}

export -f process_line

lsusb -d 1d27: | xargs -d '\n' -I '{}' bash -c 'process_line {}'
