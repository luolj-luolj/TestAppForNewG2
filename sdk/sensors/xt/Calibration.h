/*
 * Copyright 2019 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CARDBOARD_SDK_SENSORS_CALIBRATION_H_
#define CARDBOARD_SDK_SENSORS_CALIBRATION_H_

#include "util/logging.h"
#include <stdio.h>
#include <string.h>
#include "Math.h"

class Calibration
{
public:	
	Calibration();
	void Initialize(char* serialNumber,char* accfilepath, char* gyrofilepath);
	void LoadConfigFile(float *AccBias, float* GyroBias, float (*R)[3]);
private:
	void LoadConfigBuffer(const char* buffer,int line_num,float *AccBias, float (*R)[3]);
	void LoadGyroBiasFile(float* GyroBias);
	int FindUnquotedChar(const char* text, unsigned long k);
	char mSerialNumber[PATH_MAX];
	char mAccFilePath[PATH_MAX];
	char mGyroFilePath[PATH_MAX];
};
#endif  //CARDBOARD_SDK_SENSORS_ACCEL_CALIBRATION_H_