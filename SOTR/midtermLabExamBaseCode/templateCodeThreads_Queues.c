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
#include <mqueue.h> //Bilioteca para as queues
#include <fcntl.h> //Para criar as queues com O_CREAT, O_EXCL e O_RDWR
#include <math.h>
#include <stdatomic.h> //Biblioteca para a variavel atomicbool
#include <stdbool.h>   //Biblioteca para a variavel atomicbool


/******************************************************************
 * Application-specific defines
 ******************************************************************/
#define NS_IN_SEC 1000000000L

#define Period_MS 100
#define Priority 50

/******************************************************************
 * Function prototypes
 ******************************************************************/
/* Auxiliary functions to manipulate timespec structures */
struct  timespec TsAdd(struct  timespec  ts1, struct  timespec  ts2);
struct  timespec TsSub(struct  timespec  ts1, struct  timespec  ts2);

struct timespec period={
	.tv_sec=Period_MS/1000,
	.tv_nsec=(Period_MS%1000) * 1000000L
};

struct timespec next_activation;

static atomic_bool g_running=true; /*Para controlar que quando termina o ciclo de cada thread*/

mqd_t sample_queue;
mqd_t average_queue;
	
/******************************************************************
 * Global variables
 ******************************************************************/
 
struct timespec start_time; //Variavel global 


/******************************************************************
 * Thread_1_code() Periodic (Time triggerd) every 100ms
 ******************************************************************/
 
/*Counter*/
void * Thread_1_code(void *arg)
{
	(void)arg;
	/* Local variables */	
	uint32_t counter=0;
	
	if(clock_gettime(CLOCK_MONOTONIC,&next_activation)==-1){
		perror("clock_gettime");
		exit(EXIT_FAILURE);
	};
	
	/* Activation counter */
	
	/* Initialization code */
	printf("Thread 1 starting ... \n");
	printf("Thread 1 a executar no CPU %d\n", sched_getcpu());

	/* Periodic jobs */ 
	
	while(g_running) {
		/* Wait for one second (or more) */
		//sleep(1);
		next_activation=TsAdd(next_activation,period);
		
		//Cycle to wait until the instant where we qant to use the thread
		int err;
		do{
			err=clock_nanosleep(
			    CLOCK_MONOTONIC,
			    TIMER_ABSTIME,
			    &next_activation,
			    NULL
			);
		}while (err==EINTR && g_running);
				
		/*Check the result after the do..while();*/
		if(err!=0){
			printf("Clock nanosleept: %s\n", strerror(err));
			g_running=false;
			return NULL;
		}
		
		counter++;
				
		/*Queue send Thread1->Thread2*/
		if (mq_send(sample_queue,
			    (const char *)&counter,  //Aponta para onde esta a informação do counter
			    sizeof(counter),  //Dimensao do counter
			    0)==-1){
			    perror("sample mq_send");
			    exit(EXIT_FAILURE);
		}
				
	};
  
	return NULL;
}

/******************************************************************
 * Thread_2_code() Processor Event triggerd
 ******************************************************************/
void * Thread_2_code(void *arg)
{
	(void)arg;
		
	/* Local variables */              
	uint32_t values[3]; /*array para guardar os dados recebidos*/
	int position=0;
	int reps=0;
	uint64_t sum=0;

	/* Initialization code */
	printf("Thread 2 starting ... \n");
	printf("Thread 2 a executar no CPU %d\n", sched_getcpu());

	/* Periodic jobs */ 
	while(g_running) {
		uint32_t VPass2;
		/*Queue receive Thread1->Thread2*/
		if (mq_receive(sample_queue,
			      (char *)&VPass2,  //Aponta para onde vai guardar a informação do VPass2
			      sizeof(VPass2),  //Dimensao do VPass2
			      NULL)==-1){
			      perror("sample mq_receive");
			      exit(EXIT_FAILURE);
		}
		
		/*Guardaa valores no array*/
		values[position]=VPass2;
		position=(position+1)%3; /*Escolhe a posiçao do array [0;1;2]*/
		
		if(reps<3){
			reps++;
		}
		
		/*Media*/
		
		sum=0;
		
		for(int i=0; i<reps;i++){
			sum+=values[i];
		}
		float average=(float)sum/reps;
		
		
		/*Queue send Thread2->Thread3*/
		if (mq_send(average_queue,
			    (const char *)&average,  //Aponta para onde esta a informação do average
			    sizeof(average),  //Dimensao do average
			    0)==-1){
			    perror("average mq_send");
			    exit(EXIT_FAILURE);
		}
		
	}  
  
	return NULL;
}

/******************************************************************
 * Thread_3_code() OutPut Event triggerd
 ******************************************************************/
void * Thread_3_code(void *arg)
{
	(void)arg;
		
	/* Local variables */
	
	/* Initialization code */
	printf("Thread 3 starting ... \n");
	printf("Thread 3 a executar no CPU %d\n", sched_getcpu());

	/* Periodic jobs */ 
	while(g_running) {
		/*Receber aa informação*/	
		float VPassFinal;
		
		/*Queue receive Thread2->Thread3*/
		if (mq_receive(average_queue,
			      (char *)&VPassFinal,  //Aponta para onde vai guardar a informação do VPass2
			      sizeof(VPassFinal),  //Dimensao do VPass2
			      NULL)==-1){
			      perror("average mq_receive");
			      exit(EXIT_FAILURE);
		}
		
		printf("\n");
		printf("The average is: %.2f\n", VPassFinal);
		
		/*Funçao para apresentar o tempo decorrido*/
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);

		struct timespec elapsed = TsSub(now, start_time);

		printf("Time: %02lld.%03ld s\n",(long long)elapsed.tv_sec,elapsed.tv_nsec/1000000L);
		
	}  
  
	return NULL;
}

/******************************************************************
 * main()
 * Application main: creates and joins Thread 1 and Thread 2
 ******************************************************************/
int main(int argc, char *argv[])
{

	pthread_t threadid1, threadid2, threadid3;
	pthread_attr_t attr;
	struct sched_param parm;
	int err;
	
	if(argc!=2){
		printf("Usage: <priority>");
		return -1;
	}
	
	int prio=atoi(argv[1]);	


	/* Suppress compiler warnings for unused parameters */
	(void)argc;
	(void)argv; 
	
	/*Obter tempo real do codigo*/
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	
	/*Dimensao e capacidade da queue thread 1-> Thread 2*/
	struct mq_attr sample_attr = {
		.mq_maxmsg=10,
		.mq_msgsize=sizeof(uint32_t)
	};

	/*Dimensao e capacidade da queue Thread 2 -> Thread 3*/
	struct mq_attr average_attr = {
		.mq_maxmsg=10,
		.mq_msgsize=sizeof(float)
	};
	
	/*Criar a queue*/
	sample_queue= mq_open("/sample_queue",	//Nome da queue
			     O_CREAT | O_EXCL | O_RDWR,  /*O_CREAT-> cria a fila, O_EXCL-> da Erro se o nome da queue ja existir, O_RDWR->abre a fila para ler e escrever*/
			     0600, //permissao apenas o utilizador dono da fila pode ler e escrever
			     &sample_attr
			     ); //Endereço da struct que define o tamanho e a capacidade da queue
			     
	/*Erro caso nao tenha sido possivel abrir a queue*/
	if(sample_queue ==(mqd_t)-1){
		perror("Sample mq_open");
		return EXIT_FAILURE;
	}
	
	/*Apaga o nome porque ja temos um "pointer" para chegar a queue*/
	mq_unlink("/sample_queue");
	
	
	/*Criar a queue*/
	average_queue= mq_open("/average_queue",	//Nome da queue
			     O_CREAT | O_EXCL | O_RDWR,  /*O_CREAT-> cria a fila, O_EXCL-> da Erro se o nome da queue ja existir, O_RDWR->abre a fila para ler e escrever*/
			     0600, //permissao apenas o utilizador dono da fila pode ler e escrever
			     &average_attr
			     ); //Endereço da struct que define o tamanho e a capacidade da queue
			     
	/*Erro caso nao tenha sido possivel abrir a queue*/
	if(average_queue ==(mqd_t)-1){
		perror("Average mq_open");
		mq_close(sample_queue); //Liberta o "pointer" para a queue das asample caso a queue das averages de erro
		return EXIT_FAILURE;
	}
	
	/*Apaga o nome porque ja temos um "pointer" para chegar a queue*/
	mq_unlink("/average_queue");	     
	
	/* Create threads */
	
	/*atribuir prioritys*/
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr,SCHED_FIFO);
	parm.sched_priority=prio;
	pthread_attr_setschedparam(&attr,&parm);
	
	/*Definir as Threads no mesmo CPU*/
	
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(0,&cpuset);
	
	err = pthread_attr_setaffinity_np(&attr,sizeof(cpuset),&cpuset); //Configuração dos CPUs onde as futuras threads podem executar
	
	/*Erro de afinidade*/
	if(err !=0){
		printf("Erro de afinidade: %s\n", strerror(err));
		return EXIT_FAILURE;
	}
	
	
	/*Keep all current and future memory resident in RAM*/
	
	if(mlockall(MCL_CURRENT | MCL_FUTURE) ==-1){
		perror("mlockall");
		return EXIT_FAILURE;
	}
	
	err = pthread_create(&threadid1, &attr, Thread_1_code, NULL);

	if(err != 0) {
		printf("\n\r Error creating Thread 1: %s\n", strerror(err));
		return -1;
	}

	err = pthread_create(&threadid2, &attr, Thread_2_code, NULL);
	if(err != 0) {
		printf("\n\r Error creating Thread 2: %s\n", strerror(err));
		return -1;
	}
	
	err = pthread_create(&threadid3, &attr, Thread_3_code, NULL);
	if(err != 0) {
		printf("\n\r Error creating Thread 3: %s\n", strerror(err));
		return -1;
	}
	
	pthread_attr_destroy(&attr);

	/* Wait for threads to terminate */
	pthread_join(threadid1, NULL);
	pthread_join(threadid2, NULL);
	pthread_join(threadid3, NULL);
		
		
	/* close queues*/
	mq_close(sample_queue);
	mq_close(average_queue);
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


