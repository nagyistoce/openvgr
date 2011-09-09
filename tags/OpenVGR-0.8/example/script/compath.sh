#!/bin/sh
rtfind / -i $1 | sed "s/$1//g"
