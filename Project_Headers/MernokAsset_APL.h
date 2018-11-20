/*
 * MernokAsset_APL.h
 *
 *  Created on: 28 Mar 2018
 *      Author: NeilPretorius
 */

#ifndef MERNOKASSET_APL_H_
#define MERNOKASSET_APL_H_

#include "Global_Variables.h"
#include "Vision_Parameters.h"

bool MernokAsset_GroupList_populated = false;
int MernokAsset_Groups[255];

int Get_MernokAsset_GroupValues(void);
int Set_Default_MernokAsset_GroupValues(void);



#endif /* MERNOKASSET_APL_H_ */
