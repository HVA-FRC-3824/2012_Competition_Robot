//**************************************************************************
//* WARNING: This file was automatically generated.  Any changes you make  *
//*          to this file will be lost if you generate the file again.     *
//**************************************************************************
#ifndef IMAGEPROCESSING_TASK_INCLUDE
#define IMAGEPROCESSING_TASK_INCLUDE

#include <math.h>

/********************************* Image Defines ******************************/
#define NUMBER_OF_TARGETS                4
#define NUMBER_OF_TARGET_PARAMETERS     11

#define CENTER_OF_MASS_X_INDEX           0
#define CENTER_OF_MASS_Y_INDEX           1
#define BOUNDING_RECT_WIDTH_INDEX        2
#define BOUNDING_RECT_HEIGHT_INDEX       3
#define PARTICAL_AREA_INDEX              4
#define BOUNDING_BOX_AREA_INDEX          5
#define FIRST_PIX_X_INDEX                6
#define FIRST_PIX_Y_INDEX                7
#define ASPECT_RATIO_SCORE_INDEX         8
#define RECTANGLE_SCORE_INDEX            9
#define PARAMETER_INDEX                 10

#define TARGET_RATIO                    1.33

#define TOP_TARGET                       0
#define LEFT_TARGET                      1
#define RIGHT_TARGET                     2
#define BOTTOM_TARGET                    3

/******************************************************************************/

#include "WPILib.h"

int IVA_ProcessImage(Image *image,
      double results[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS]);

#endif // ifndef IMAGEPROCESSING_TASK_INCLUDE
