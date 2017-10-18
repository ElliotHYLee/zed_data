#include <sl/Camera.hpp>
#include "ProcessImage.h"

using namespace sl;

typedef struct mouseOCVStruct {
	sl::Mat depth;
	cv::Size _resize;
} mouseOCV;


mouseOCV mouseStruct;
cv::Mat slMat2cvMat(sl::Mat& input);
void printHelp();
//static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param);

void transformPose(sl::Transform &pose, float tx) {
	sl::Transform transform_;
	transform_.setIdentity();
	// Move the tracking frame by tx along the X axis
	transform_.tx = tx;
	// Apply the transformation
	pose = Transform::inverse(transform_) * pose * transform_;
}


int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;
	sl::Pose camera_pose;
	
    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;
	init_params.camera_fps = 60;
	init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
	init_params.sdk_verbose = true;

	// Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        printf("%s\n", errorCode2str(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

	TrackingParameters trackingParameters;
	trackingParameters.initial_world_transform = sl::Transform::identity();
	trackingParameters.enable_spatial_memory = true;     // Enable Spatial memory

	zed.enableTracking(trackingParameters);
	float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;



    // Display help in console
    printHelp();
	
    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 1;
    int new_height = image_size.height / 1;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_zed(new_width, new_height, sl::MAT_TYPE_8U_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);

	sl::Mat image_zed2(new_width, new_height, sl::MAT_TYPE_8U_C4);
	cv::Mat image_ocv2 = slMat2cvMat(image_zed2);


	
 //   sl::Mat depth_image_zed(new_width, new_height, sl::MAT_TYPE_8U_C4);
 //   cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
 //   sl::Mat depth;
    sl::Mat point_cloud;

	cv::Size ds(1280, 720);
	//cv::Size ds(640, 360);
	CalibrationParameters cp = zed.getCameraInformation().calibration_parameters;
	ProcessImage pi(ds, cp);
	cv::namedWindow("rgb", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("rgb", ProcessImage::onMouseCallBack , (void*)&pi);

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedMilliSec = 0;

	// get ticks per second
	QueryPerformanceFrequency(&frequency);

	// start timer
	QueryPerformanceCounter(&t1);

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, new_width, new_height);
			//zed.retrieveImage(image_zed2, VIEW_RIGHT, MEM_CPU, new_width, new_height);
            //zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, new_width, new_height);
			zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_CPU, new_width, new_height);
			//zed.retrieveMeasure(depth, MEASURE_DEPTH); // Retrieve the depth measure (32bits)


			TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);
			//transformPose(camera_pose.pose_data, translation_left_to_center);
			//sl::float4 quaternion = camera_pose.getOrientation();
			sl::float3 rotation = camera_pose.getEulerAngles(); // Only use Euler angles to display absolute angle values. Use quaternions for transforms.
			sl::float3 translation = camera_pose.getTranslation();
			//cv::imshow("rgb", image_zed);
			

			//pi.updateFrames(image_ocv, depth_image_ocv, depth, point_cloud, elapsedMilliSec);
			pi.updateFrames(image_ocv, point_cloud, elapsedMilliSec, translation, rotation);
			//pi.doYourWork();
            

			
			//cout << to_string(translation.x) << " " << to_string(translation.y) << " " << to_string(translation.z) << endl;

			//imshow("depth", depth_image_ocv);
			QueryPerformanceCounter(&t2);
			elapsedMilliSec = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
			cout << 1000 / elapsedMilliSec << " Hz " << endl;
			QueryPerformanceCounter(&t1);

            // Handle key event
            key = cv::waitKey(10);
            //processKeyEvent(zed, key);
		}
		else
		{
			cout << " shit " << endl;
		}
    }
    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
