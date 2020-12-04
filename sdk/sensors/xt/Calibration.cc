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
#include "Calibration.h"
#include <stdio.h>
#include <stdlib.h>

#define MaxVariableLength 100

Calibration::Calibration()
{

}

void Calibration::Initialize(char* serialNumber,char* accfilepath, char* gyrofilepath)
{
	static char path1[150] = "/data/hmdinfo/AccelBias.txt";
	static char path2[150] = "/data/hmdinfo/GyroBias.txt";
	strcpy(mSerialNumber, serialNumber);
	if (accfilepath) {
		strcpy(mAccFilePath, accfilepath);
	} else {
		strcpy(mAccFilePath, path1);
	}
	if (gyrofilepath) {
		strcpy(mGyroFilePath, gyrofilepath);
	} else {
		strcpy(mGyroFilePath, path2);
	}
}

int Calibration::FindUnquotedChar(const char* text, unsigned long k)
{
	const char* start = text;
	for (;;) {
		unsigned long c = *reinterpret_cast<const unsigned char*>(text);
		if (c == 0) break;
		if (c == k) return (int)(text - start);
		text++;
	}
	return (-1);
}

void Calibration::LoadConfigBuffer(const char* buffer,int line_num,float *AccBias, float (*R)[3])
{
	char line[MaxVariableLength];
	const char* text = buffer;
	int i;

	if(line_num == 5){
		for (i=0;i<line_num;i++) 
		{
			memset(line, 0, MaxVariableLength);
			long length = FindUnquotedChar(text, '\n');

			if (length > 0) 
			{
				strncpy(line, text, length);
				if(i==0)
					sscanf(line,"%f,%f,%f",&AccBias[0],&AccBias[1],&AccBias[2]);
				else if(i==2)
					sscanf(line,"%f,%f,%f",&R[0][0],&R[0][1],&R[0][2]);
				else if(i==3)
					sscanf(line,"%f,%f,%f",&R[1][0],&R[1][1],&R[1][2]);
				else if(i==4)
					sscanf(line,"%f,%f,%f",&R[2][0],&R[2][1],&R[2][2]);
			}
			text += length + 1;
		}
	}else if(line_num == 4){
		for (i=0;i<line_num;i++) 
		{
			memset(line, 0, MaxVariableLength);
			long length = FindUnquotedChar(text, '\n');
			PHONEAR_LOGI("length is %ld", length);
			if (length > 0) 
			{
				strncpy(line, text, length);
				PHONEAR_LOGI("line[%d] is %s", i, line);
				if(i==0)
					sscanf(line,"%f,%f,%f",&AccBias[0],&AccBias[1],&AccBias[2]);
				else if(i==1)
					sscanf(line,"%f,%f,%f",&R[0][0],&R[0][1],&R[0][2]);
				else if(i==2)
					sscanf(line,"%f,%f,%f",&R[1][0],&R[1][1],&R[1][2]);
				else if(i==3){
					if(sscanf(line,"%f,%f,%f",&R[2][0],&R[2][1],&R[2][2])==0)
						//To be compatible with the format that an extra ',' at the begning of the last line, which is a bug of calibration
						sscanf(line,",%f,%f,%f",&R[2][0],&R[2][1],&R[2][2]);
				}
			}
			text += length + 1;
		}
	}
	for(int i = 0 ; i < 3 ; i++)
	{
		if(AccBias[i] < -9.8f || AccBias[i] > 9.8f)
		{
			PHONEAR_LOGE("AccBias value too large!");
			for(int i = 0 ; i < 3 ; i++)
			{
				AccBias[i] = 0;
			}
			R[0][0] = R[1][1] = R[2][2] = 1.0f;
			R[0][1] = R[1][0] = R[2][0] = 0.0f;
			R[0][2] = R[1][2] = R[2][1] = 0.0f;
			return;
		}
	}
	Matrix3f ma(R[0][0],R[0][1],R[0][2],R[1][0],R[1][1], R[1][2],R[2][0] , R[2][1] ,R[2][2] );
	if(fabs(ma.Determinant()-1.0f) > 0.001f)
	{
		PHONEAR_LOGE("Accel Determinant check error!");
		// for some HMD still check error, don't reset calibration data.only give a print info.
		/* AccBias[0] = 0.0f;
		AccBias[1] = 0.0f;
		AccBias[2] = 0.0f;
		R[0][0] = R[1][1] = R[2][2] = 1.0f;
		R[0][1] = R[1][0] = R[2][0] = 0.0f;
		R[0][2] = R[1][2] = R[2][1] = 0.0f;*/
	}
	return;
}

void Calibration::LoadGyroBiasFile(float* GyroBias) {
	//parse gyro calibration file
	FILE* fp;
	int size;
	int temperature;

	char line[MaxVariableLength];
	fp = fopen(mGyroFilePath, "r");
	if (!fp) {
		CARDBOARD_LOGE("Unable to open gyro file: %s", mGyroFilePath);
		return;
	}
	fscanf(fp, "%d,%f,%f,%f", &temperature, &GyroBias[0], &GyroBias[1], &GyroBias[2]);
	PHONEAR_LOGE("GyroBias.txt temperature is %d", temperature);
	return;
}

void Calibration::LoadConfigFile(float *AccBias, float* GyroBias, float (*R)[3])
{
	FILE* fp;
	int line_num;
	char flag;
	char cmdline[100];
	char defaultFilePath[50];
	char strLine[11];
	int offset;

	//parse acc calibration file
	fp = fopen(mAccFilePath, "r");
	if (!fp) {
		PHONEAR_LOGE("Unable to open acc file: %s", mAccFilePath);
		return;
	} else {
		PHONEAR_LOGI("open acc file: %s successfully", mAccFilePath);
	}

	fgets(strLine,11,fp);
	if(strncmp(strLine,"AccelBias",9) != 0){
		PHONEAR_LOGE("AccelBias.txt file is invalid!");
		return;
	} else {
		PHONEAR_LOGI("AccelBias.txt valid ok");
	}
	offset = 10;//sizeof AccelBias + 1(\n)

	fseek(fp, offset, SEEK_END);
	int size = ftell(fp);
	fseek(fp, offset, SEEK_SET);

	char* buf = new char[size + 1];
	fread(buf, 1, size, fp);
	buf[size] = '\0';
	fseek(fp, offset, SEEK_SET);
	line_num = 0;
	while(!feof(fp))
	{
		flag = fgetc(fp);
		if(flag == '\n')
			line_num++;
	}
	fclose(fp);
	PHONEAR_LOGI("line_num is %d", line_num);
	LoadConfigBuffer(buf,line_num,AccBias,R);
	delete[] buf;
	//LoadGyroBiasFile(GyroBias);
}

