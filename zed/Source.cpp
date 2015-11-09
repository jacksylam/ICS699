///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////




/**************************************************************************************************
** This sample demonstrates how to grab images and depth/disparity map with the ZED SDK          **
** Both images and depth/disparity map are displayed with OpenCV                                 **
** Most of the functions of the ZED SDK are linked with a key press event (using opencv)         **
***************************************************************************************************/

//windows work around
#define NOMINMAX

//standard includes
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>


//opencv includes
#include <opencv2/core/core.hpp>

//ZED Includes
#include <zed/Camera.hpp>

//Define the structure and callback for mouse event
typedef struct mouseOCVStruct {
	float* data;
	uint32_t step;
	cv::Size _image;
	cv::Size _resize;
	std::string name;
} mouseOCV;

mouseOCV mouseStruct;

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		mouseOCVStruct* data = (mouseOCVStruct*)param;

		int y_int = (y * data->_image.height / data->_resize.height);
		int x_int = (x * data->_image.width / data->_resize.width);

		float* ptr_image_num = (float*)((int8_t*)data->data + y_int * data->step);
		float dist = ptr_image_num[x_int] / 1000.f;

		if (dist > 0.)
			printf("\n%s : %2.2f m\n", data->name.c_str(), dist);
		else
			printf("\n : NAN\n");
	}
}




// save function using opencv
void saveSbSimage(sl::zed::Camera* zed, std::string filename) {
	sl::zed::resolution imSize = zed->getImageSize();

	cv::Mat SbS(imSize.height, imSize.width * 2, CV_8UC4);
	cv::Mat leftIm(SbS, cv::Rect(0, 0, imSize.width, imSize.height));
	cv::Mat rightIm(SbS, cv::Rect(imSize.width, 0, imSize.width, imSize.height));

	slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::LEFT)).copyTo(leftIm);
	slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::RIGHT)).copyTo(rightIm);

	cv::imshow("Saving Image", SbS);
	cv::cvtColor(SbS, SbS, CV_RGBA2RGB);

	cv::imwrite(filename, SbS);
}

void computeDepthRBGPoints(sl::zed::Camera* zed, sl::zed::Mat depth, cv::Mat leftImage, cv::Mat rightImage){
	float baseline = zed->getParameters()->baseline;
	float focal = zed->getParameters()->LeftCam.fx;
	std::ofstream depthFileRGBPoints("depthFileRGBPoints.txt");

	//Record initial precision for stream purposes
	std::streamsize initialPrecision = std::cout.precision();

	float* ptr_d;
	for (int i = 0; i < depth.height; ++i) {
		ptr_d = (float*)(depth.data + i * depth.step);
		for (int j = 0; j < depth.width * depth.channels; ++j) {

			//Bad solution. Only use left image to compute color.
			cv::Vec4b leftPixel = leftImage.at<cv::Vec4b>(i, j);

			unsigned char leftRed = leftPixel[0];
			unsigned char leftGreen = leftPixel[1];
			unsigned char leftBlue = leftPixel[2];
			unsigned char leftAlpha = leftPixel[3];

			
			float red = ((int)leftRed) / 255.0;
			float green = ((int)leftGreen) / 255.0;
			float blue = ((int)leftBlue) / 255.0;
			float alpha = ((int)leftAlpha) / 255.0;

			if (ptr_d[j] > -1){
				depthFileRGBPoints << std::setprecision(initialPrecision) << j << " " << i << " " << ptr_d[j] << " " << std::setprecision(3) << red << " " << green << " " << blue << " " << alpha << std::endl;
			}
			else{
				depthFileRGBPoints << std::setprecision(initialPrecision) << j << " " << i << " " << "0" << " " << std::setprecision(3) << red << " " << green << " " << blue << " " << alpha << std::endl;
			}
		}
	}

	depthFileRGBPoints.close();
}


//main  function
int main(int argc, char **argv) {

	if (argc > 2){
		std::cout << "Only the path of a SVO can be passed in arg" << std::endl;
		//Sleep(2000);
		return -1;
	}

	sl::zed::SENSING_MODE dm_type = sl::zed::RAW;
	sl::zed::Camera* zed;

	if (argc == 1) // Use in Live Mode
		zed = new sl::zed::Camera(sl::zed::HD1080);
	else // Use in SVO playback mode
		zed = new sl::zed::Camera(argv[1]);

	int width = zed->getImageSize().width;
	int height = zed->getImageSize().height;

	sl::zed::ERRCODE err = zed->init(sl::zed::MODE::PERFORMANCE, 0, true);

	// ERRCODE display
	std::cout << sl::zed::errcode2str(err) << std::endl;



	// Quit if an error occurred
	if (err != sl::zed::SUCCESS) {
		delete zed;
		return 1;
	}

	int key = ' ';
	int ViewID = 2;
	int count = 0;
	int ConfidenceIdx = 100;

	bool DisplayDisp = true;
	bool displayConfidenceMap = false;

	cv::Mat disp(height, width, CV_8UC4);
	cv::Mat anaplyph(height, width, CV_8UC4);
	cv::Mat confidencemap(height, width, CV_8UC4);
	cv::Mat canny(height, width, CV_8UC4);
	cv::Mat myDepth(height, width, CV_8UC4);



	cv::Size DisplaySize(720, 404);
	cv::Mat dispDisplay(DisplaySize, CV_8UC4);
	cv::Mat anaplyphDisplay(DisplaySize, CV_8UC4);
	cv::Mat confidencemapDisplay(DisplaySize, CV_8UC4);
	cv::Mat depthDisplay(DisplaySize, CV_8UC4);



	/* Init mouse callback */
	sl::zed::Mat depth;
	zed->grab(dm_type);
	depth = zed->retrieveMeasure(sl::zed::MEASURE::DEPTH); // Get the pointer
	// Set the structure
	mouseStruct._image = cv::Size(width, height);
	mouseStruct._resize = DisplaySize;
	mouseStruct.data = (float*)depth.data;
	mouseStruct.step = depth.step;
	mouseStruct.name = "DEPTH";
	/***/

	//create Opencv Windows
	cv::namedWindow("DEPTH", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("DEPTH", onMouseCallback, (void*)&mouseStruct);
	cv::namedWindow("VIEW", cv::WINDOW_AUTOSIZE);

	bool pictureTaken = false;

	//loop until 'q' is pressed
	while (key != 'q') {
		// DisparityMap filtering
		//zed->setDispReliability(reliabilityIdx); !!function name has been change in Release 0.8 --see ChangeLog
		zed->setConfidenceThreshold(ConfidenceIdx);


		// Get frames and launch the computation
		bool res = zed->grab(dm_type);

		// The following is the best way to save a disparity map/ Image / confidence map in Opencv Mat.
		// Be Careful, if you don't save the buffer/data on your own, it will be replace by a next retrieve (retrieveImage, NormalizeMeasure, getView....)
		// !! Disparity, Depth, confidence are in 8U,C4 if normalized format !! //
		// !! Disparity, Depth, confidence are in 32F,C1 if only retrieve !! //

		/***************  DISPLAY:  ***************/

		slMat2cvMat(zed->getView(static_cast<sl::zed::VIEW_MODE>(ViewID))).copyTo(anaplyph);
		cv::resize(anaplyph, anaplyphDisplay, DisplaySize);


		slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(myDepth);
		//slMat2cvMat(depth).copyTo(myDepth); //line crashes cv_cvtcolor
		cv::resize(myDepth, depthDisplay, DisplaySize);


		//Take only one picture and output file once.
		if (pictureTaken == false){
			std::ofstream depthFile("depth.txt");
			if (depthFile.is_open()){//testing depth saving
				depthFile << "myDepth = " << std::endl << myDepth << std::endl << std::endl;
				depthFile.close();
			}
			else {
				std::cout << "Unable to open myDepth txt file.";
			}

			std::ofstream depthFileOnly("mmDepth.txt");
			std::ofstream depthFileFormated("mmDepthFormat.txt");

			if (depthFileOnly.is_open()){//save depth in mm
				float* ptr_d;
				for (int i = 0; i < depth.height; ++i)
				{
					ptr_d = (float*)(depth.data + i * depth.step);
					for (int j = 0; j < depth.width * depth.channels; ++j)
					{
						depthFileOnly << ptr_d[j];

						//write to formated file
						depthFileFormated << j << " " << i << " " << ptr_d[j] << std::endl;

						// if not end of the current row, we add a space character
						if (j != (depth.width * depth.channels) - 1)
							depthFileOnly << " ";
					}
					depthFileOnly << "\n";
				}
				depthFileOnly.close();
				depthFileFormated.close();
			}
			else{
				std::cout << "Error with depth only txt.";
			}

			cv::Mat leftImageMat(height, width, CV_8UC4);
			cv::Mat rightImageMat(height, width, CV_8UC4);

			leftImageMat = slMat2cvMat(zed->getView(sl::zed::VIEW_MODE::STEREO_LEFT));
			rightImageMat = slMat2cvMat(zed->getView(sl::zed::VIEW_MODE::STEREO_RIGHT));
			computeDepthRBGPoints(zed, depth, leftImageMat, rightImageMat);

			//write to png file
			std::string depthPictureFileName = "depthPicture.png";
			cv::Mat myDepthPictureMat(height, width, CV_8UC4);
			cv::cvtColor(myDepth, myDepthPictureMat, CV_RGBA2RGB);
			cv::imwrite(depthPictureFileName, myDepthPictureMat);


			std::string sideBySidePictureFileName = "sideBySide.png";
			cv::Mat sideBySidePictureMat(height, 2 * width, CV_8UC4);
			cv::Mat leftIm(sideBySidePictureMat, cv::Rect(0, 0, width, height));
			cv::Mat rightIm(sideBySidePictureMat, cv::Rect(width, 0, width, height));
			slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::LEFT)).copyTo(leftIm);
			slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::RIGHT)).copyTo(rightIm);
			cv::cvtColor(sideBySidePictureMat, sideBySidePictureMat, CV_RGBA2RGB);
			cv::imwrite(sideBySidePictureFileName, sideBySidePictureMat);

			pictureTaken = true;
		}
		imshow(mouseStruct.name, depthDisplay);



		imshow("VIEW", anaplyphDisplay);


#ifdef WIN32
		key = cv::waitKey(5);
#else
		key = cv::waitKey(5) % 256;
#endif

		// Keyboard shortcuts
		switch (key) {
			// ______________  THRESHOLD __________________
		case 'b':
			ConfidenceIdx -= 10;
			break;
		case 'n':
			ConfidenceIdx += 10;
			break;

			//re-compute stereo alignment
		case 'a':
			zed->reset();
			break;

			//Change camera settings (here --> gain)
		case 'g': //increase gain of 1
		{
			int current_gain = zed->getCameraSettingsValue(sl::zed::ZED_GAIN);
			zed->setCameraSettingsValue(sl::zed::ZED_GAIN, current_gain + 1);
			std::cout << "set Gain to " << current_gain + 1 << std::endl;
		}
			break;

		case 'h': //decrease gain of 1
		{
			int current_gain = zed->getCameraSettingsValue(sl::zed::ZED_GAIN);
			zed->setCameraSettingsValue(sl::zed::ZED_GAIN, current_gain - 1);
			std::cout << "set Gain to " << current_gain - 1 << std::endl;
		}
			break;
			// ______________  VIEW __________________
		case '0': // left
			ViewID = 0;
			break;
		case '1': // right
			ViewID = 1;
			break;
		case '2': // anaglyph
			ViewID = 2;
			break;
		case '3': // gray scale diff
			ViewID = 3;
			break;
		case '4': // Side by side
			ViewID = 4;
			break;
		case '5': // overlay
			ViewID = 5;
			break;
			// ______________  Display Confidence Map __________________
		case 's':
			displayConfidenceMap = !displayConfidenceMap;
			break;
			//______________ SAVE ______________
		case 'w': // image
			saveSbSimage(zed, std::string("ZEDImage") + std::to_string(count) + std::string(".png"));
			count++;
			break;
		case 'v': // disparity
		{
			std::string filename = std::string(("ZEDDisparity") + std::to_string(count) + std::string(".png"));
			cv::Mat dispSnapshot;
			disp.copyTo(dispSnapshot);
			cv::imshow("Saving Disparity", dispSnapshot);
			cv::imwrite(filename, dispSnapshot);
			count++;
			break;
		}
		case 'r':
			dm_type = sl::zed::SENSING_MODE::RAW;
			std::cout << "SENSING_MODE: Raw" << std::endl;
			break;
		case 'f':
			dm_type = sl::zed::SENSING_MODE::FULL;
			std::cout << "SENSING_MODE: FULL" << std::endl;
			break;

		case 'd':
			DisplayDisp = !DisplayDisp;
			break;
		}

		ConfidenceIdx = ConfidenceIdx < 1 ? 1 : ConfidenceIdx;
		ConfidenceIdx = ConfidenceIdx > 100 ? 100 : ConfidenceIdx;
	}

	delete zed;
	return 0;
}
