# Base code - inverted pendulum
C programs that simulates the dynamics of an inverted pendulum and provides a suitable controller. 

As is, the application don't use Linux RT services

#Compile with:
gcc -O2 -Wall inverted_pendulum.c -o inverted_pendulum -lpthread -lm

#Run:
./inverted_pendulum

And in another terminal stress the PC, e.g.

sudo nice -n -20 stress-ng     --cpu $(nproc)   --cache 2  --switch 2     --schedmix 2   --vm 6  --timeout 30s

Actual impact depends on HW. Some tunning can be required

