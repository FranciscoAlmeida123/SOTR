/******************************************************************
 * Paulo Pedreiras, Oct/2026
 * DETI/UA/IT - Real-Time Operating Systems course
 *
 * Base code for creating two threads
 * No RT services
 * 
 * Student1 : ID#:     Name:
 * Student2 : ID#:     Name:
 * 
 ******************************************************************/
#define _GNU_SOURCE  /* Must precede #include <sched.h> for sched_setaffinity */ 

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <sched.h> 
#include <pthread.h>
#include <errno.h>
#include <signal.h> 
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <math.h>

/******************************************************************
 * Application-specific defines
 ******************************************************************/
#define NS_IN_SEC 1000000000L

/******************************************************************
 * Function prototypes
 ******************************************************************/
/* Auxiliary functions to manipulate timespec structures */
struct  timespec TsAdd(struct  timespec  ts1, struct  timespec  ts2);
struct  timespec TsSub(struct  timespec  ts1, struct  timespec  ts2);


/******************************************************************
 * Global variables
 ******************************************************************/


/******************************************************************
 * Thread_1_code()
 ******************************************************************/
void * Thread_1_code(void *arg)
{
	(void)arg;
		
	/* Local variables */
	int niter = 0;              /* Activation counter */

	/* Initialization code */
	printf("Thread 1 starting ... \n");

	/* Periodic jobs */ 
	while(1) {
		/* Wait for one second (or more) */
		sleep(1);
		printf("Thread 1 activation %d\n", niter++);
	}  
  
	return NULL;
}

/******************************************************************
 * Thread_2_code()
 ******************************************************************/
void * Thread_2_code(void *arg)
{
	(void)arg;
		
	/* Local variables */
	int niter = 0;              /* Activation counter */

	/* Initialization code */
	printf("Thread 2 starting ... \n");

	/* Periodic jobs */ 
	while(1) {
		/* Wait for one second (or more) */
		sleep(1);
		printf("Thread 2 activation %d\n", niter++);
	}  
  
	return NULL;
}

/******************************************************************
 * main()
 * Application main: creates and joins Thread 1 and Thread 2
 ******************************************************************/
int main(int argc, char *argv[])
{
	pthread_t threadid1, threadid2;
	int err;

	/* Suppress compiler warnings for unused parameters */
	(void)argc;
	(void)argv; 
	
	/* Create threads */
	err = pthread_create(&threadid1, NULL, Thread_1_code, NULL);
	if(err != 0) {
		printf("\n\r Error creating Thread 1: %s\n", strerror(err));
		return -1;
	}

	err = pthread_create(&threadid2, NULL, Thread_2_code, NULL);
	if(err != 0) {
		printf("\n\r Error creating Thread 2: %s\n", strerror(err));
		return -1;
	}

	/* Wait for threads to terminate */
	pthread_join(threadid1, NULL);
	pthread_join(threadid2, NULL);
		
	return 0;
}


/******************************************************************
 * TsAdd()
 * Adds two timespec structures (ts1 + ts2).
 * Handles nanosecond overflow to seconds.
 ******************************************************************/
struct  timespec  TsAdd(struct  timespec  ts1, struct  timespec  ts2){
	
	struct  timespec  tr;
	
	/* Add the two timespec variables */
	tr.tv_sec = ts1.tv_sec + ts2.tv_sec ;
	tr.tv_nsec = ts1.tv_nsec + ts2.tv_nsec ;

	/* Check for nanosecond overflow and carry over to seconds */	
	if (tr.tv_nsec >= NS_IN_SEC) {
		tr.tv_sec++ ;
		tr.tv_nsec = tr.tv_nsec - NS_IN_SEC ;
	}

	return (tr) ;
}

/******************************************************************
 * TsSub()
 * Subtracts two timespec structures (ts1 - ts2).
 * Clamps to 0 if ts1 <= ts2; handles nanosecond borrow otherwise.
 ******************************************************************/
struct  timespec  TsSub (struct  timespec  ts1, struct  timespec  ts2) {
	struct  timespec  tr;

	/* If ts1 <= ts2, result would be negative; return 0 */
	if ((ts1.tv_sec < ts2.tv_sec) || ((ts1.tv_sec == ts2.tv_sec) && (ts1.tv_nsec <= ts2.tv_nsec))) {
		tr.tv_sec = tr.tv_nsec = 0 ;  
	} else {						
		/* If ts1 > ts2, proceed with subtraction and handle borrow if needed */ 
		tr.tv_sec = ts1.tv_sec - ts2.tv_sec ;
		if (ts1.tv_nsec < ts2.tv_nsec) {
			tr.tv_nsec = ts1.tv_nsec + NS_IN_SEC - ts2.tv_nsec ;
			tr.tv_sec-- ;				
		} else {
			tr.tv_nsec = ts1.tv_nsec - ts2.tv_nsec ;
		}
	}

	return (tr) ;
}


