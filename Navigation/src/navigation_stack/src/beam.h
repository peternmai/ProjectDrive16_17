 /*****************************************************************************

                                                         Author: Jason Ma
                                                         Date:   Sep 30 2017
                                      TODO

 File Name:       beam.h
 Description:     TODO
 *****************************************************************************/

#ifndef BEAM_H
#define BEAM_H

#include "util.h"

using std::vector;

float beamPath(float angleOffset, float width, int numRects, vector<CartesianCoordinate> map);

#endif /* BEAM_H */
