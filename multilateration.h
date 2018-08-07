#include <math.h>
#include <stdint.h>
#include <stddef.h>

/* Multilateration state structure. This structure contains the current radio signal strength log and location data. */
typedef struct multilateration_state_s // TODO a log of when each record was added
{
    uint32_t   n; /* Size of the x, y, and r arrays. */
    float*     x; /* x position of each source. */
    float*     y; /* y position of each source. */
    float*     r; /* Distance to each source, calculated from signal level. */
    float      X; /* Last calculated x position of receiver. */
    float      Y; /* Last calculated y position of receiver. */
    uint32_t   start; /* Start position in the ring buffer. */
    uint32_t   stop;  /* End position in the ring buffer. */
} multilateration_state_t;

/* Initialize the multilateration state structure. It dynamically allocates the record buffer. */
uint32_t multilateration_init(multilateration_state_t* state, uint32_t n);

/* Uninitialize the multilateration state structure. It frees the record buffer. */
void multilateration_uninit(multilateration_state_t* state);

/* Add a new record to the multilateration buffer. */
uint32_t multilateration_insert(multilateration_state_t* state, float x, float y, float r);

/* Remove records older than a given limit value from the multilateration buffer. */
uint32_t multilateration_removeold(multilateration_state_t* state, uint32_t ticks);

/* Perform 2D multilaterateration through linear regression.
 *
 * It uses the data stored in the state
 *
 * Returns 0 if successful, 1 if too few points in the buffer, 2 if the matrix is singular. */
uint32_t multilateration(multilateration_state_t* state);

/**@brief Give the XY location of the closest beacon. */
uint32_t multilateration_getnearest(multilateration_state_t* state, float* X, float* Y);

/**@brief Get the last calculated X Y location. */
uint32_t multilateration_getXY(multilateration_state_t* state, float* X, float* Y);