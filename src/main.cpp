/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <memory>
#include <fstream>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>

#include "cv_bridge/cv_bridge.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
bool useSampleOutput=false;

using namespace dso;

void parseArgument(char* arg)
{
	int option;
	char buf[1000];

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}


std::unique_ptr<FullSystem> fullSystem;
std::unique_ptr<Undistort> undistorter;
int frameID = 0;

cv::Mat_<double> cameraMatrix;

const cv::Size imageSize(1280, 960);

void vidCb(const sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    //cv::Rect rect(imageSize.width / 4, imageSize.height / 4, imageSize.width / 2, imageSize.height / 2);
    //auto tmp = cv_ptr->image(rect);
    //tmp.copyTo(cv_ptr->image);
    //cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(640, 480));

    //cv::namedWindow("Cropped", cv::WINDOW_AUTOSIZE);
    //cv::imshow("Cropped", cv_ptr->image);
    //cv::waitKey(0);

    {
        cv::Mat orig, undist;
        cv_ptr->image.copyTo(orig);
        //cv::undistort(orig, undist, cameraMatrix, cv::InputArray(std::vector<double>{-0.36846, 0.20339000000000002, -0.0014700000000000002, 0.00151}));
        cv::undistort(orig, undist, cameraMatrix, cv::InputArray(std::vector<double>{-0.375296, 0.188019, 0.000203, -0.000602}));

        cv::Mat diff;
        cv::resize(orig, orig, imageSize / 2);
        cv::resize(undist, undist, imageSize / 2);
        cv::absdiff(orig, undist, diff);

        cv::Mat output;
        cv::hconcat(orig, undist, output);
        cv::hconcat(output, diff, output);

        cv::namedWindow("Undistorted", cv::WINDOW_AUTOSIZE);
        cv::imshow("Undistorted", output);
        cv::waitKey(0);
    }

    //return;

    //cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(640, 480));
    assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);

	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = std::make_unique<FullSystem>();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
	    if(undistorter->photometricUndist != 0)
	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

    MinimalImageB minImg(cv_ptr->image.cols, cv_ptr->image.rows, (unsigned char*) cv_ptr->image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
	fullSystem->addActiveFrame(undistImg, frameID);
	frameID++;
	delete undistImg;

}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "dso_live");

	for(int i = 1; i < argc; i++)
        parseArgument(argv[i]);

    {
        std::ifstream ifs(calib);
        cv::Mat_<double> matrix = cv::Mat_<double>::zeros(3, 3);
        float fx, fy, cx, cy;

        if(!(ifs >> fx >> fy >> cx >> cy))
            throw std::logic_error("invalid calib");

        matrix.at<double>(0, 0) = imageSize.width * fx;
        matrix.at<double>(1, 1) = imageSize.height * fy;
        matrix.at<double>(0, 2) = imageSize.width * cx - 0.5;
        matrix.at<double>(1, 2) = imageSize.height * cy - 0.5;
        matrix.at<double>(2, 2) = 1;

        cameraMatrix = matrix;

        std::cout << "Camera matrix: " << cameraMatrix << std::endl;
    }

	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;


	printf("MODE WITH CALIBRATION, but without exposure times!\n");
	setting_photometricCalibration = 2;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;

    undistorter.reset(Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile));

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());


    fullSystem = std::make_unique<FullSystem>();
    fullSystem->linearizeOperation=false;


    if(!disableAllDisplay)
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]));

    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh;
    ros::Subscriber imgSub = nh.subscribe("/robo/camera/image_color", 1, &vidCb);

    ros::spin();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

	return 0;
}

