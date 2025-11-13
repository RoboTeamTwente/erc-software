#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    // --- Load calibration ---
    Mat cameraMatrix, distCoeffs;
    FileStorage fs("charuco_camera_params.yml", FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    cout << "Loaded camera matrix:\n" << cameraMatrix << endl;
    cout << "Loaded distortion:\n" << distCoeffs.t() << endl;

    // --- Parameters ---
    float markerLength = 0.05f; // meters (adjust to your printed marker)

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::DICT_5X5_250);

    Ptr<aruco::DetectorParameters> detectorParams =
        aruco::DetectorParameters::create();

    VideoCapture inputVideo("http://145.126.44.42:8080/video", CAP_FFMPEG);
    if (!inputVideo.isOpened()) {
        cerr << "Cannot open stream." << endl;
        return -1;
    }

    while (true) {
        Mat frame, frameCopy;
        if (!inputVideo.read(frame)) {
            cerr << "Frame read failed." << endl;
            break;
        }

        frame.copyTo(frameCopy);

        vector<int> ids;
        vector<vector<Point2f>> corners;
        vector<Vec3d> rvecs, tvecs;

        // Detect markers
        aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams);

        if (!ids.empty()) {
            // Pose estimation using calibrated camera
            aruco::estimatePoseSingleMarkers(
                corners, markerLength,
                cameraMatrix, distCoeffs,
                rvecs, tvecs
            );

            aruco::drawDetectedMarkers(frameCopy, corners, ids);

            // Draw axes
            for (size_t i = 0; i < ids.size(); i++) {
                drawFrameAxes(frameCopy, cameraMatrix, distCoeffs,
                              rvecs[i], tvecs[i], markerLength * 0.5f);
            }
        }

        imshow("Aruco Pose Estimation", frameCopy);
        if ((char)waitKey(1) == 27) break; // ESC
    }

    return 0;
}
