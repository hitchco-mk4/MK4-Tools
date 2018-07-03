_This repo is a part of the Hitchco MK4 project. You can read more about the project [here](http://www.esologic.com/hitchco-mk4/)._
# MK4-Tools

These are a few development tools for making development on this project a little bit easier.

## test-sketches

Various sketches to test different parts of the board. There are comments on the top of each of the files that describes purpose. Use these if problems are being had and you want to quickly debug problems.

## cli

The file `mk4-cli.py` is a command line interface for getting data off of the Arduino in the most minimal way possible. All it does is read in the full 64 bytes and parses them into their actual float/int/byte representation. 

Used for debugging. 