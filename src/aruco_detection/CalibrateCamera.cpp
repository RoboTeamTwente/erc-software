#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {

    // --- Parameters ---
    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 0.025f;  // meters
    float markerLength = 0.018f;  // meters
    int calibrationFlags = 0;
    double aspectRatio = 1.0;

    // --- Dictionary & ChArUco board ---
    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::DICT_5X5_250);

    Ptr<aruco::CharucoBoard> board =
        aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;

    // --- Data containers ---
    vector<Mat> allCharucoCorners;
    vector<Mat> allCharucoIds;
    Size imageSize;

    VideoCapture inputVideo("http://145.126.44.42:8080/video"); // or path to video file
    if (!inputVideo.isOpened()) {
        cerr << "Cannot open camera/video.\n";
        return -1;
    }

    cout << "Press 'c' to capture a valid ChArUco frame, or 'q' to quit.\n";

    while (true) {
        Mat image, imageCopy;
        inputVideo >> image;
        if (image.empty())
            break;

        image.copyTo(imageCopy);

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners, rejected;
        aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams, rejected);

        if (!markerIds.empty())
            aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);

        // --- Interpolate ChArUco corners ---
        Mat currentCharucoCorners, currentCharucoIds;
        if (!markerIds.empty()) {
            aruco::interpolateCornersCharuco(
                markerCorners, markerIds, image, board, currentCharucoCorners, currentCharucoIds);
        }

        if (currentCharucoCorners.total() > 3)
            aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

        imshow("ChArUco Detection", imageCopy);
        char key = (char)waitKey(1);

        if (key == 'q')
            break;

        if (key == 'c' && currentCharucoCorners.total() > 3) {
            cout << "Frame captured.\n";
            allCharucoCorners.push_back(currentCharucoCorners);
            allCharucoIds.push_back(currentCharucoIds);
            imageSize = image.size();
        }
    }

    destroyAllWindows();

    if (allCharucoCorners.size() < 3) {
        cerr << "Not enough valid frames for calibration.\n";
        return -1;
    }

    // --- Calibration ---
    Mat cameraMatrix, distCoeffs;
    if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = aspectRatio;
    }

    cout << "Calibrating using " << allCharucoCorners.size() << " valid frames...\n";

    double repError = aruco::calibrateCameraCharuco(
        allCharucoCorners,
        allCharucoIds,
        board,
        imageSize,
        cameraMatrix,
        distCoeffs,
        noArray(),
        noArray(),
        calibrationFlags
    );

    cout << "\n=== Calibration Results ===\n";
    cout << "RMS Reprojection Error: " << repError << endl;
    cout << "Camera Matrix:\n" << cameraMatrix << endl;
    cout << "Distortion Coefficients:\n" << distCoeffs.t() << endl;

    // --- Save calibration ---
    FileStorage fs("charuco_camera_params.yml", FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();
    cout << "Saved calibration to charuco_camera_params.yml\n";

    return 0;
}
