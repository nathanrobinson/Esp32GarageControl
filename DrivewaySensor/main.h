#pragma once

// #define IS_GARAGE
#define IS_DRIVEWAY

#ifdef IS_GARAGE

#define ENABLE_DISPLAY
#define ENABLE_TOUCH

#endif

#ifdef IS_DRIVEWAY

#define AXS15231B_DISPLAY

#endif