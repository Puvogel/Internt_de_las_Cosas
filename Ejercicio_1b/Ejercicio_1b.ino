/* ======================================================================= *\
 *  Ejemplo ChibiOS-3
 *  Este ejemplo muestra cómo estimar la carga computacional de cada hebra
 *  
 *  IMPORTANTE: en ChRt/src/rt/templates/chconf.h
 *    - CH_DBG_THREADS_PROFILING debe activarse (TRUE) 
 *    - CH_CFG_NO_IDLE_THREAD debe activarse (TRUE)
 *    
 *  Requiere el uso de la librería ChRt de Bill Greiman
 *    https://github.com/greiman/ChRt
 *    
 *  Asignatura (GII-IoT)
\* ======================================================================= */
#include <ChRt.h>
#include <math.h>
#include <Arduino.h>
//------------------------------------------------------------------------------
// Parametrization
//------------------------------------------------------------------------------
#define USE_DOUBLE FALSE  // Change to TRUE to use double precision (heavier)

#define CYCLE_MS 1000
#define NUM_THREADS 5  // Three working threads + loadEstimator (top) + \
                       // loop (as the idle thread) \
                       // TOP thread is thread with id 0


////////////////////////////////////////
//// Additional Defines ////////////////
////////////////////////////////////////

#define LOAD_REDUCTION_FACTOR 20   // Define a factor by which the load balancer should reduce theard load
#define LOAD_INCREASE_FACTOR 20    // Define factor by which the load balancer should increase thread load
#define CPU_USAGE_TARGET_PER 85                          // Define target CPU utilisation to 85%
#define CPU_USAGE_TARGET_DEC CPU_USAGE_TARGET_PER / 100  // Define target CPU utilisation as decimal
#define NUM_WORKER_THREADS NUM_THREADS - 2               // Worker threads are all but "top", and "loop"
#define K_P 0.1                                         // Proportional gain for adjusting load


///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////


char thread_name[NUM_THREADS][15] = { "top", "worker_1", "worker_2", "worker_3", "idle" };

volatile uint32_t threadPeriod_ms[NUM_THREADS] = { CYCLE_MS, 200, 100, 200, 0 };
volatile int threadLoad[NUM_THREADS] = { 0, 50, 50, 50, 0 };

volatile int idleCycles = 0;

///////////////////////////////////////
////// New min-max load ///////////////
///////////////////////////////////////

// Define the minimum and maximum load values for each thread (in ticks)
const int threadMinLoad_i[NUM_THREADS] = { 0, 10, 10, 10, 0 };
const int threadMaxLoad_i[NUM_THREADS] = { 0, 300, 150, 350, 0 };

///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

volatile uint32_t threadEffectivePeriod_ms[NUM_THREADS] = { 0, 0, 0, 0, 0 };
volatile uint32_t threadCycle_ms[NUM_THREADS] = { 0, 0, 0, 0, 0 };

// Struct to measure the cpu load using the ticks consumed by each thread
typedef struct {
  thread_t *thd;
  systime_t lastSampleTime_i;
  sysinterval_t lastPeriod_i;
  sysinterval_t ticksTotal;
  sysinterval_t ticksPerCycle;
  float loadPerCycle_per;
} threadLoad_t;

typedef struct {
  threadLoad_t threadLoad[NUM_THREADS];
  uint32_t idling_per;
} systemLoad_t;

systemLoad_t sysLoad;


///////////////////////////////////////
///// Load balancer ///////////////////
///////////////////////////////////////

void top_load_balance() {
  // Calculate the total CPU load and utilization of each thread
  float workerLoad = 0;
  // Initialize Array to hold new iteration values for then ext cycle
  int newWorkerLoad[NUM_THREADS];
  // Variable to hold over/undershoot of load ticks across threads
  int excessLoad_i = 0;

  // Iterate over all worker processes: 
  // Get the thread load of the last cycle
  for (int tid = 1; tid < NUM_THREADS - 1; tid++) {
    workerLoad += sysLoad.threadLoad[tid].loadPerCycle_per;
  }
  

  ///////// DEBUGGING //////////
  SerialUSB.print("workerLoad in top_load_balancer: ");
  SerialUSB.println(workerLoad);
  //////////////////////////////

  float loadAdjust = computeLoadAdjustment(workerLoad);

  SerialUSB.print("loadAdjust: ");
  SerialUSB.println(loadAdjust);

  // Adjust the load of each thread based on its utilization
  for (int tid = 1; tid < NUM_THREADS - 1; tid++) {
    int newLoad;
    SerialUSB.print("Thread: ");
    SerialUSB.println(tid);

    if (workerLoad > CPU_USAGE_TARGET_PER) {
      // Reduce the load if the thread's utilization is too high
      newLoad = abs((int)(threadLoad[tid]) + loadAdjust * LOAD_REDUCTION_FACTOR);
      newWorkerLoad[tid] = newLoad;
      SerialUSB.print(" reduce to ");
    } else if (workerLoad < CPU_USAGE_TARGET_PER) {
      // Increase the load if the thread's utilization is too low
      newLoad = abs((int)(threadLoad[tid]) + loadAdjust * LOAD_INCREASE_FACTOR);
      newWorkerLoad[tid] = newLoad;
      SerialUSB.print(" increase to ");
    } else {
      newLoad = abs(threadLoad[tid]);  // Keep the load unchanged
      newWorkerLoad[tid] = newLoad;
      SerialUSB.print(" unchanged with ");
    }
    SerialUSB.print(newLoad);

    // Ensure the new load is within the specified range
    if (newWorkerLoad[tid] < threadMinLoad_i[tid]) {
            excessLoad_i += threadMinLoad_i[tid] - newWorkerLoad[tid];
            SerialUSB.print(". Thread ");
            SerialUSB.print(tid);
            SerialUSB.print(" below bound, increase allowed ticks.");
            newLoad -= excessLoad_i;
        } else if (newWorkerLoad[tid] > threadMaxLoad_i[tid]) {
            excessLoad_i -= newWorkerLoad[tid] - threadMaxLoad_i[tid];
            SerialUSB.print(". Thread ");
            SerialUSB.print(tid);
            SerialUSB.print(" above bound, decrease allowed ticks.");
            newLoad += excessLoad_i;
        }
        excessLoad_i = 0;
        ///////////////
        // Edge Case: Last thread goes out of bounds !!!
        // Cannot ensure bounds can be met within one cycle
        //////////////
    SerialUSB.print("\n");

    // Update the load of the thread
    threadLoad[tid] = newLoad;
  }
}


float computeLoadAdjustment(float currentLoad) {
  // Calculate the error (deviation from the target)
  float error = (float)(CPU_USAGE_TARGET_PER - currentLoad);
  SerialUSB.print("error: ");
  SerialUSB.println(error);

  // Compute the load adjustment value using proportional control
  float adjustment = K_P * error;

  // Ensure that the adjustment value is within a reasonable range
  if (adjustment > 1.0) {
    adjustment = 1.0;  // Maximum adjustment value
  } else if (adjustment < -1.0) {
    adjustment = -1.0;  // Minimum adjustment value
  }

  return adjustment;
}
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

//------------------------------------------------------------------------------
// Load estimator (top)
// High priority thread that executes periodically
//------------------------------------------------------------------------------
BSEMAPHORE_DECL(top_sem, true);
static THD_WORKING_AREA(waTop, 256);

static THD_FUNCTION(top, arg) {
  (void)arg;
  bool ledState = LOW;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);

  // Initialize sysLoad struct
  memset(&sysLoad, 0, sizeof(sysLoad));

  systime_t lastTime_i = 0;
  systime_t period_i = TIME_MS2I(CYCLE_MS);

  // Reset top_sem as "taken"
  chBSemReset(&top_sem, true);

  while (!chThdShouldTerminateX()) {

    ///////////////////////////////////////
    ////// Call load balance funciton//////
    ///////////////////////////////////////

    top_load_balance();

    ///////////////////////////////////////
    ///////////////////////////////////////
    ///////////////////////////////////////

    // Wait a certain amount of time
    // chThdSleepMilliseconds(CYCLE_MS);
    systime_t deadline_i = lastTime_i + period_i;
    if (deadline_i > chVTGetSystemTimeX()) {
      chBSemWaitTimeout(&top_sem, sysinterval_t(deadline_i - chVTGetSystemTimeX()));
    }

    // Accumulated ticks for this cycle
    uint32_t accumTicks = 0;

    // This assumes that no other thread will accumulate ticks during this sampling
    // so we can use this timestamp for all threads
    lastTime_i = chVTGetSystemTimeX();

    // tid starts at 1 because we do not include this thread (top)
    for (int tid = 1; tid < NUM_THREADS; tid++) {
      threadLoad_t *thdLoad = &(sysLoad.threadLoad[tid]);
      thdLoad->lastSampleTime_i = lastTime_i;
      systime_t ticks = chThdGetTicksX(thdLoad->thd);
      /*     
      SerialUSB.print(tid);
      SerialUSB.print(" ");
      SerialUSB.println(ticks);
*/
      thdLoad->ticksPerCycle = ticks - thdLoad->ticksTotal;
      thdLoad->ticksTotal = ticks;
      accumTicks += thdLoad->ticksPerCycle;
    }

    for (int tid = 1; tid < NUM_THREADS; tid++) {
      threadLoad_t *thdLoad = &sysLoad.threadLoad[tid];
      thdLoad->loadPerCycle_per = (100 * (float)thdLoad->ticksPerCycle) / accumTicks;
      SerialUSB.print(thread_name[tid]);
      SerialUSB.print("  ticks(last cycle): ");
      SerialUSB.print(thdLoad->ticksPerCycle);
      SerialUSB.print("  CPU(%): ");
      SerialUSB.print(thdLoad->loadPerCycle_per);
      SerialUSB.print("   Cycle duration(ms): ");
      SerialUSB.print(threadCycle_ms[tid]);
      SerialUSB.print("  period(ms): ");
      SerialUSB.println(threadEffectivePeriod_ms[tid]);
    }
    SerialUSB.println();

    // Switch the led state
    ledState = (ledState == HIGH) ? LOW : HIGH;
    digitalWrite(LED_BUILTIN, ledState);
  }
}

//------------------------------------------------------------------------------
// Worker thread executes periodically
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waWorker1, 256);
static THD_WORKING_AREA(waWorker2, 256);
static THD_WORKING_AREA(waWorker3, 256);

static THD_FUNCTION(worker, arg) {
  int worker_ID = (int)arg;
  sysinterval_t period_i = TIME_MS2I(threadPeriod_ms[worker_ID]);
  systime_t deadline_i = chVTGetSystemTimeX();
  systime_t lastBeginTime_i = 0;

  while (!chThdShouldTerminateX()) {
    systime_t beginTime_i = chVTGetSystemTimeX();
    threadEffectivePeriod_ms[worker_ID] = TIME_I2MS(beginTime_i - lastBeginTime_i);

    int niter = threadLoad[worker_ID];
#if USE_DOUBLE
    double num = 10;
#else
    float num = 10;
#endif

    for (int iter = 0; iter < niter; iter++) {
#if USE_DOUBLE
      num = exp(num) / (1 + exp(num));
#else
      num = expf(num) / (1 + expf(num));
#endif
    }

    deadline_i += period_i;

    /*
    SerialUSB.print(worker_ID);
    SerialUSB.print(" ");
    SerialUSB.print(deadline_i);
    SerialUSB.print(" ");
    SerialUSB.println(chVTGetSystemTimeX());
*/
    lastBeginTime_i = beginTime_i;
    threadCycle_ms[worker_ID] = TIME_I2MS(chVTGetSystemTimeX() - beginTime_i);
    if (deadline_i > chVTGetSystemTimeX()) {
      chThdSleepUntil(deadline_i);
    }
  }
}

//------------------------------------------------------------------------------
// Continue setup() after chBegin() and create the two threads
//------------------------------------------------------------------------------
void chSetup() {
  // Here we assume that CH_CFG_ST_TIMEDELTA is set to zero
  // All SAMD-based boards are only supported in “tick mode”

  // Check first if ChibiOS configuration is compatible
  // with a non-cooperative scheme checking the value of CH_CFG_TIME_QUANTUM
  if (CH_CFG_TIME_QUANTUM == 0) {
    SerialUSB.println("You must set CH_CFG_TIME_QUANTUM to a non-zero value in");
#if defined(__arm__)
    SerialUSB.print("src/<board type>/chconfig<board>.h");
#elif defined(__AVR__)
    SerialUSB.print("src/avr/chconfig_avr.h");
#endif
    SerialUSB.println(" to enable round-robin scheduling.");
    while (true) {}
  }
  SerialUSB.print("CH_CFG_TIME_QUANTUM: ");
  SerialUSB.println(CH_CFG_TIME_QUANTUM);

  // Check we do not spawn the idle thread
  if (CH_CFG_NO_IDLE_THREAD == FALSE) {
    SerialUSB.println("You must set CH_CFG_NO_IDLE_THREAD to TRUE");
  }

  // Start top thread
  sysLoad.threadLoad[0].thd = chThdCreateStatic(waTop, sizeof(waTop),
                                                NORMALPRIO + 2, top, (void *)threadPeriod_ms[0]);

  // Start working threads.
  sysLoad.threadLoad[1].thd = chThdCreateStatic(waWorker1, sizeof(waWorker1),
                                                NORMALPRIO + 1, worker, (void *)1);

  sysLoad.threadLoad[2].thd = chThdCreateStatic(waWorker2, sizeof(waWorker2),
                                                NORMALPRIO + 1, worker, (void *)2);

  sysLoad.threadLoad[3].thd = chThdCreateStatic(waWorker3, sizeof(waWorker3),
                                                NORMALPRIO + 1, worker, (void *)3);

  // This thread ID
  sysLoad.threadLoad[4].thd = chThdGetSelfX();
}

//------------------------------------------------------------------------------
// setup() function
//------------------------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  SerialUSB.begin(115200);
  while (!SerialUSB) { ; }

  SerialUSB.println("Hit any key + ENTER to start ...");
  while (!SerialUSB.available()) { delay(10); }

  // Initialize OS and then call chSetup.
  // chBegin() never returns. Loop() is invoked directly from chBegin()
  chBegin(chSetup);
}

//------------------------------------------------------------------------------
// loop() function. It is considered here as the idle thread
//------------------------------------------------------------------------------
void loop() {
  idleCycles++;
  if (idleCycles > 1000000){
    if(sysLoad.threadLoad[NUM_THREADS -1].loadPerCycle_per <= 16){
      SerialUSB.print("Idle Cycles since last intervention: ");
      SerialUSB.println(idleCycles);
      idleCycles = 0;
      int tid = random(1,3);
      threadLoad[tid] += 20;
      SerialUSB.print("Increase Thead ");
      SerialUSB.print(tid);
      SerialUSB.println(" by 20 ticks");
    }
  }
}
