#include "sac_obstacle.h"
/* No distance sensor has been specified or wired. Do not fabricate a reading.
 * WARNING: This firmware cannot yet satisfy SAC's compulsory obstacle task. */
bool sac_obstacle_present(void) { return false; }
