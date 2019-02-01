/*****************************************************************************
MeanFilterClass.h

Description:	cpp file of MeanFilterClass

@Version:	1.0
@Author:	Chengxu Zhou (chengxu.zhou@iit.it)
@Release:	2014/08/20
@Update:	Fri 11 Nov 2016 04:08:02 PM CET
*****************************************************************************/
#include "MeanFilterClass.h"
#include <iostream>

MeanFilterClass::MeanFilterClass()
{
	initF = false;
	F_buffer.reserve(300);
}

std::vector<double> MeanFilterClass::applyFilter(int filter_size, const std::vector<double> &raw_data)
{	
	int raw_data_size = raw_data.size();
	
	if(!initF)
	{
		F_buffer.resize(filter_size, std::vector<double> (raw_data_size, 0)); 
		// it_buffer = F_buffer.begin();
		initF = true;
	}

	//std::cout<<F_buffer.size()<<std::endl;

	for(int i=0;i<filter_size-1;i++)
	{
		F_buffer[i] = F_buffer[i+1];
	}
	F_buffer[filter_size-1] = raw_data;  //then push back the latest value into array
	
	std::vector<double> F_filter(raw_data_size, 0), F_sum(raw_data_size, 0);
	
	for (int j=0;j<raw_data_size;j++)
	{	
		int sum0 = 0;
		for(int i=filter_size;i>0;i--)
		{	
			sum0 += i;
			F_sum[j] += i*F_buffer[i-1][j];
		}
		F_filter[j] = F_sum[j]/sum0;
	}

	return F_filter;
}

std::vector<double> MeanFilterClass::returnBuffer(int index)
{
	int F_size = F_buffer.size();
	if (index < F_size)
	{
		return F_buffer[F_size-1-index];	
	}
	else
	{
		printf("Error!!! Invalid Index!!!");
		exit(0);
	}
}

std::vector<double> MeanFilterClass::applyMeanFilter(int filter_size, const std::vector<double> &raw_data)
{	
	int raw_data_size = raw_data.size();
	
	if(!initF)
	{
		F_buffer.resize(filter_size, std::vector<double> (raw_data_size, 0)); 
		// it_buffer = F_buffer.begin();
		initF = true;
	}

	//std::cout<<F_buffer.size()<<std::endl;

	for(int i=0;i<filter_size-1;i++)
	{
		F_buffer[i] = F_buffer[i+1];
	}
	F_buffer[filter_size-1] = raw_data;  //then push back the latest value into array
	
	std::vector<double> F_filter(raw_data_size, 0), F_sum(raw_data_size, 0);
	
	for (int j=0;j<raw_data_size;j++)
	{	
		for(int i=0;i<filter_size;i++)
		{	
			F_sum[j] += F_buffer[i][j];
		}
		F_filter[j] = F_sum[j]/filter_size;
	}

	return F_filter;
}

int MeanFilterClass::returnBufferSize()
{
	return F_buffer.size();	
}

Eigen::Vector3d MeanFilterClass::applyMeanFilter(int filter_size, const Eigen::Vector3d &raw_data)
{
    std::vector<double> raw_temp(3,0);
    raw_temp[0] = raw_data[0];
    raw_temp[1] = raw_data[1];
    raw_temp[2] = raw_data[2];
    raw_temp = applyMeanFilter(filter_size, raw_temp);
    Eigen::Vector3d F_filter(0,0,0);
    F_filter[0] = raw_temp[0];
    F_filter[1] = raw_temp[1];
    F_filter[2] = raw_temp[2];
    return F_filter;
}

Eigen::Vector3d MeanFilterClass::applyFilter(int filter_size, const Eigen::Vector3d &raw_data)
{
    std::vector<double> raw_temp(3,0);
    raw_temp[0] = raw_data[0];
    raw_temp[1] = raw_data[1];
    raw_temp[2] = raw_data[2];
    raw_temp = applyFilter(filter_size, raw_temp);
    Eigen::Vector3d F_filter(0,0,0);
    F_filter[0] = raw_temp[0];
    F_filter[1] = raw_temp[1];
    F_filter[2] = raw_temp[2];
    return F_filter;
}






