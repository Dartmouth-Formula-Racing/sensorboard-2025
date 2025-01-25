/*Author: Sasha Ries
 * Date: 1/25/25
 * File: filter.h
 * Description: h file
 */

#include <main.h>
#include <stdbool.h>

#define WINDOW_SIZE 4
#define FILTER_TYPE 1
#define RPM_SCALE_FACTOR 1000

// Window struct to store an array of size WINDOW_SIZE and a position indicator
typedef struct {
    float data_array[WINDOW_SIZE];
    uint32_t sample_count;
} sample_window;


int32_t Roll_average(sample_window* window, float new_speed);