#include "multilateration.h"

#include <stdlib.h>


uint32_t multilateration_init(multilateration_state_t* state, uint32_t n)
{
    float* mem = (float*)malloc(sizeof(float)*3*n);
    if (mem==NULL) {
        return 1;
    }
    state->n = n;
    state->X = NAN;
    state->Y = NAN;
    state->x = mem;
    state->y = mem+n;
    state->r = mem+2*n;
    state->start = 0;
    state->stop  = 0;
    return 0;
}


void multilateration_uninit(multilateration_state_t* state)
{
    free(state->x);
}


uint32_t multilateration_insert(multilateration_state_t* state, float x, float y, float r) // TODO lock during the insert
{
    state->stop = (state->stop+1)%state->n;
    if (state->start == state->stop)
    {
        state->start = (state->start+1)%state->n;
    }
    state->x[state->stop] = x;
    state->y[state->stop] = y;
    state->r[state->stop] = r;
    return 0;
}


uint32_t multilateration_removeold(multilateration_state_t* state, uint32_t ticks) // TODO lock during the remove
{
    // TODO not implemented
    return 1;
}


uint32_t multilateration_getnearest(multilateration_state_t* state, float* X, float* Y) // TODO lock during the calculation
{
    float min_r = 10000;
    int min_i = 0;
    int start = state->start;
    int stop = state->stop;
    int n = state->n;
    int records;
    if (stop > start){
        records = stop-start+1;
    } else {
        records = n-start+stop+1;
    }
    if (records < 3 || stop==start)
    {
        // Not enough records to perform triangulation.
        return 1;
    }

    for (uint32_t j=1;j<records;j++)
    {
        uint32_t i = (j+start)%n;
        if (state->r[i]<min_r){
            min_i = i;
            min_r = state->r[i];
        }
    }
    *X = state->x[min_i];
    *Y = state->y[min_i];
    return 0;
}


uint32_t multilateration_getXY(multilateration_state_t* state, float* X, float* Y) // TODO lock during the get
{
    *X = state->X;
    *Y = state->Y;
    return 0;
}


/* Perform 2D linear regression with zero intercept.
 *
 * Find the X and Y that minimize the sum (over i) of (x[i]*X+y[i]*Y-r[i])^2
 * for given arrays x[], y[], and r[]. Implemented for reference and testing purposes.
 * n is the length of the arrays x[i], y[i], and r[i].
 * 
 * Returns 0 if successful, 1 on failure.
 */
static uint32_t linreg(uint32_t n, const float x[], const float y[], const float r[], float* X, float* Y)
{
    float sumx2 = 0.0;                     /* sum of x**2  */
    float sumxy = 0.0;                     /* sum of x * y */
    float sumy2 = 0.0;                     /* sum of y**2  */
    float sumxr = 0.0;                     /* sum of x * r */
    float sumyr = 0.0;                     /* sum of y * r */

    for (uint32_t i=0;i<n;i++)
    { 
        sumx2 += x[i]*x[i]; 
        sumxy += x[i]*y[i]; 
        sumy2 += y[i]*y[i]; 
        sumxr += x[i]*r[i]; 
        sumyr += y[i]*r[i]; 
    } 

    float det = sumx2*sumy2 - sumxy*sumxy;
    if (det*det < 1e-8)
    {
        // Singular(ish) matrix, can't solve the problem.
        return 1;
    }

    *X = (sumy2*sumxr - sumxy*sumyr) / det;
    *Y = (sumx2*sumyr - sumxy*sumxr) / det;

    return 0; 
}


/* Perform 2D multilaterateration through linear regression.
 *
 * For location arrays x[] and y[] and corresponding distance array r[]
 * to a central unknown point (X,Y), calculate the most probable (X,Y). It is
 * performed by approximately solving a system of n equations
 * (x[i]-X)^2 + (y[i]-Y)^2 = r[i]^2 which is linearized by substracting the
 * zeroth equation from all the others. Then the solution is derived with
 * linear least squares. The arrays are treated as ring buffers with current data
 * between positions start and stop.
 *
 * n is the length of the arrays x[i], y[i], and r[i].
 * start and stop are the starting end ending point in the ring buffer arrays.
 *
 * Returns 0 if successful, 1 if too few points in the buffer, 2 if the matrix is singular.
 */
static uint32_t multilateration_ringbuffer(uint32_t n, const float x[], const float y[], const float r[], float* X, float* Y, uint32_t start, uint32_t stop)
{
    float sumx2 = 0.0;                     /* sum of x**2  */
    float sumxy = 0.0;                     /* sum of x * y */
    float sumy2 = 0.0;                     /* sum of y**2  */
    float sumxr = 0.0;                     /* sum of x * r */
    float sumyr = 0.0;                     /* sum of y * r */

    float xx, yy, rr;

    uint32_t records;
     
    if (stop>start){
        records = stop-start+1;
    } else {
        records = n-start+stop+1;
    }

    if (records < 3 || stop==start)
    {
        // Not enough records to perform triangulation.
        return 1;
    }

    for (uint32_t j=1;j<records;j++)
    {
        uint32_t i = (j+start)%n;
        xx = x[i]-x[start];
        yy = y[i]-y[start];
        rr = (r[start]*r[start]-r[i]*r[i] + x[i]*x[i]-x[start]*x[start] + y[i]*y[i]-y[start]*y[start])/2;
        sumx2 += xx*xx;
        sumxy += xx*yy;
        sumy2 += yy*yy;
        sumxr += xx*rr;
        sumyr += yy*rr;
    } 

    float det = sumx2*sumy2 - sumxy*sumxy;
    if (det*det < 1e-8)
    {
        // Singular(ish) matrix, can't solve the problem.
        return 2;
    }

    *X = (sumy2*sumxr - sumxy*sumyr) / det;
    *Y = (sumx2*sumyr - sumxy*sumxr) / det;

    return 0; 
}


/* Perform 2D multilaterateration through linear regression.
 *
 * For location arrays x[] and y[] and corresponding distance array r[]
 * to a central unknown point (X,Y), calculate the most probable (X,Y). It is
 * performed by approximately solving a system of n equations
 * (x[i]-X)^2 + (y[i]-Y)^2 = r[i]^2 which is linearized by substracting the
 * zeroth equation from all the others. Then the solution is derived with
 * linear least squares. See `multilateration_ringbuffer` for details.
 */
static uint32_t multilateration_wholearray(uint32_t n, const float x[], const float y[], const float r[], float* X, float* Y)
{
    return multilateration_ringbuffer(n,x,y,r,X,Y,0,n-1);
}


uint32_t multilateration(multilateration_state_t* state) // TODO lock during the calculation
{
    return multilateration_ringbuffer(state->n, state->x, state->y, state->r, &(state->X), &state->Y, state->start, state->stop);
}


// Some test code available for reference.
#ifdef MULTILATERATION_TESTS
int test_linreg()
{
    uint32_t n = 5;
    float x[] = {1,2,3,4,5};
    float y[] = {11,19,31,42,48};
    float r[] = {101,202,295,399,510};
    float X,Y;
    uint32_t retcode = linreg(n, x, y, r, &X, &Y);
    NRF_LOG_DEBUG("TEST: linreg " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(X-126.10909091), NRF_LOG_FLOAT(Y+2.54545455));
    // double check in python with np.linalg.lstsq(np.array([[1,2,3,4,5],[11,19,31,42,48]]).T,[101,202,295,399,510])
    // result is X=126.10909091, Y=-2.54545455
}

int test_multilateration_wholearray()
{
    uint32_t n = 4;
    float x[] = {0,1,1,0};
    float y[] = {0,0,1,1};
    float r[] = {0.707,0.707,0.707,0.707};
    float X,Y;
    uint32_t retcode = multilateration_wholearray(n, x, y, r, &X, &Y);
    NRF_LOG_DEBUG("TEST: multilateration_wholearray " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(X), NRF_LOG_FLOAT(Y));
}

int test_multilateration()
{
    multilateration_state_t state;
    multilateration_init(&state, 4);
    multilateration_insert(&state, 0, 0, 0.707);
    multilateration_insert(&state, 0, 1, 0.707);
    multilateration_insert(&state, 1, 1, 0.707);
    multilateration_insert(&state, 1, 0, 0.707);
    uint32_t retcode = multilateration(&state);
    float X,Y;
    uint32_t retcode1 = multilateration_getXY(&state, &X, &Y);
    NRF_LOG_DEBUG("TEST: multilateration " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(X), NRF_LOG_FLOAT(Y));
    multilateration_uninit(&state);
}

int testall_multilateration()
{
    test_linreg();
    test_multilateration_wholearray();
    test_multilateration();
}
#endif