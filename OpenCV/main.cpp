#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>

bool compareRectangles(const cv::RotatedRect& first, const cv::RotatedRect& second) {
	return first.size.area() > second.size.area();
}

void loadcameracalibration(std::string& settingsstring, cv::Mat& cameraMatrix, cv::Mat& distcoeffs, std::vector<std::vector<cv::Point2f>>& imagePoints) {

	cv::FileStorage fs2(settingsstring, cv::FileStorage::READ);
	//method : use(type) operator on filenode.
	int framecount = static_cast<int>(fs2["nr_of_frames"]);
	cv::String date;
	//second method : use filenode::operator >>
	fs2["calibration_time"] >> date;
	fs2["camera_Matrix"] >> cameraMatrix;
	fs2["distortion_coefficients"] >> distcoeffs;
	std::cout << "framecount: " << framecount << std::endl
		<< "calibration date: " << date << std::endl
		<< "camera Matrix: " << cameraMatrix << std::endl
		<< "distortion coeffs: " << distcoeffs << std::endl;
	cv::FileNode features = fs2["features"];
	cv::FileNodeIterator it = features.begin(), it_end = features.end();
	int idx = 0;
	std::vector<uchar> lbpval;
	//iterate through a sequence using filenodeiterator
	for (; it != it_end; ++it, idx++)
	{
		std::cout << "feature #" << idx << ": ";
		std::cout << "x=" << static_cast<int>((*it)["x"]) << ", y=" << static_cast<int>((*it)["y"]) << ", lbp: (";
		//you can also easily read numerical arrays using filenode >> std::vector operator.
		(*it)["lbp"] >> lbpval;
		for (int i = 0; i < static_cast<int>(lbpval.size()); i++)
			std::cout << " " << static_cast<int>(lbpval[i]);
		std::cout << ")" << std::endl;
	}

	cv::Mat imagePtMat;

	fs2["image_points"] >> imagePtMat;

	for (size_t i = 0; i < imagePtMat.rows; i++)
	{
		imagePoints.push_back(imagePtMat.row(i));
	}

	fs2.release();
}

cv::RNG rng(12345);

int main(int argc, char* argv[]) {

	std::string settingsLocation = "E:\\Projects\\OpenCV\\OpenCV\\OpenCV\\OpenCV\\out_camera_data.xml";

	cv::Mat cameraMatrix;
	cv::Mat distCoeff;

	std::vector<std::vector<cv::Point2f>> imagePoints;

	loadcameracalibration(settingsLocation, cameraMatrix, distCoeff, imagePoints);

	cv::Mat frame;
	cv::Mat imageGray;

	cv::VideoCapture capture;

	capture.open(1);

	if (!capture.isOpened()) {
		std::cerr << "error opening camera 1." << std::endl;
		return  -1;
	}

	//This will work fine
	//auto substractor = cv::createBackgroundSubtractorMOG2();

	while (capture.read(frame)) {


		if (frame.empty()) {
			std::cerr << "error capturing frame" << std::endl;
			break;
		}

		//cv::Mat difference;
		//cv::Mat background = cv::imread("E:\\Projects\\OpenCV\\OpenCV\\OpenCV\\OpenCV\\img\\img.png", cv::IMREAD_COLOR);
		//cv::absdiff(background, frame, difference);
		//cv::imshow("Difference", difference);


		cvtColor(frame, imageGray, CV_BGR2GRAY);
		blur(imageGray, imageGray, cv::Size(5, 5));

		cv::Mat mask;
		threshold(imageGray, mask, 50, 255, CV_THRESH_BINARY_INV);
		cv::Canny(mask, mask, 0, 50, 3);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		//Find the rotated rectangles
		std::vector<cv::RotatedRect> minRect(contours.size());

		for (int i = 0; i < contours.size(); i++) {
			if (cv::contourArea(contours[i]) > 2000)
			{
				minRect[i] = cv::minAreaRect(cv::Mat(contours[i]));
			}
		}

		std::sort(minRect.begin(), minRect.end(), compareRectangles);

		//Draw contours + rotated rectangles + ellipses
		cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC3);


		cv::Point2f points[4];
		minRect[0].points(points);

		//for (int i = 0; i < contours.size(); i++) {

		cv::Scalar color = cv::Scalar(rng.uniform(0, 0), rng.uniform(0, 0), rng.uniform(250, 250));

		drawContours(drawing, contours, 0, color, 3, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

		//Rotated rectangle
		cv::Point2f rect_points[4]; minRect[0].points(rect_points);

		//bottomLeft, topLeft, topRight, bottomRight.

		double distancePx = cv::norm(cv::Mat(rect_points[0]) - cv::Mat(rect_points[3]));

		double realDist = 86 * 3.7 / (distancePx / 189);

		for (int j = 0; j < 4; j++) {
			if (rect_points[j].x == 0) {
				continue;
			}
			line(frame, rect_points[j], rect_points[(j + 1) % 4], color, 3, 8);
		}

		//}

		//Sensor physical size
		cv::VideoCapture cap;

		if (!cap.isOpened()) //check if we succeeded
			CV_Assert("Can't open Web-Cam");

		double CamHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		double CamWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);

		// Function for display the object dimension
		int imgW = 650;
		int imgH = 50;
		int fontFace = cv::FONT_HERSHEY_PLAIN;
		double fontScale = 1.5;
		int thickness = 2;
		cv::Point textOrg(imgW / 5, imgH / 2.5);
		cv::Point textOrgTwo(imgW / 5, imgH);
		cv::Point tectDist(imgW / 5, imgH / 0.5);

		std::string someText;

		someText = cv::format("Height: %.2f mm", minRect[0].size.width - minRect[0].size.width * 0.28);
		putText(frame, someText, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		someText = cv::format("Width: %.2f mm", minRect[0].size.height - minRect[0].size.height * 0.28);
		putText(frame, someText, textOrgTwo, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		someText = cv::format("Distance: %.2f mm", realDist);
		putText(frame, someText, tectDist, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		imshow("Object Measurement", frame);

		//cv::imshow("Output", mask);
		//std::string store("E:\\Projects\\OpenCV\\OpenCV\\OpenCV\\OpenCV\\img\\img.png");
		//cv::imwrite(store, frame);

		const auto c = cvWaitKey(10);  //delay in milliseconds
		if (static_cast<char>(c) > 0) {
			break;
		}
	}

	return 0;
}
