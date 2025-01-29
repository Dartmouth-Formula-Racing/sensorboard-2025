/*Author: Sasha Ries
 * Date: 1/25/25
 * File: filter.h
 * Description: h file
 */

#include <main.h>
#include <stdbool.h>

#define WINDOW_SIZE 8 // Number of samples used in filtering
#define FILTER_TYPE 1 // To pick what kind of filter used in preprocessing
#define RPM_SCALE_FACTOR 1000 // Scale factor to increase resolution


// Window struct to store an array of size WINDOW_SIZE and a position indicator
typedef struct {
    float data_array[WINDOW_SIZE];
    uint32_t position;
} sample_window;


/* Function that stores input speed in a rolling storage array.
Then averages all the values in the array and returns a filtered value */
int32_t Roll_average(sample_window* window, float new_speed);