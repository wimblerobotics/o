#include <atomic>
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
#include <iomanip>
#include <mutex>         // std::mutex, std::unique_lock
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <o_msgs/ObjectFound.h>
#include <o_msgs/ObjectFoundList.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "yolo_v2_class.hpp"    // imported functions from DLL

#ifdef OPENCV

#include <opencv2/opencv.hpp>            // C++
#include <opencv2/core/version.hpp>
#ifndef CV_VERSION_EPOCH     // OpenCV 3.x and 4.x
#include <opencv2/videoio/videoio.hpp>
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)"" CVAUX_STR(CV_VERSION_REVISION)
#ifndef USE_CMAKE_LIBS
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
#endif    // USE_CMAKE_LIBS
#else     // OpenCV 2.x
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_EPOCH)"" CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)
#ifndef USE_CMAKE_LIBS
#pragma comment(lib, "opencv_core" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_imgproc" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_highgui" OPENCV_VERSION ".lib")
#pragma comment(lib, "opencv_video" OPENCV_VERSION ".lib")
#endif    // USE_CMAKE_LIBS
#endif    // CV_VERSION_EPOCH


void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,
    int current_det_fps = -1, int current_cap_fps = -1)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_name = obj_names[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            max_width = std::max(max_width, (int)i.w + 2);
            //max_width = std::max(max_width, 283);
            std::string coords_3d;
            if (!std::isnan(i.z_3d)) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << "x:" << i.x_3d << "m y:" << i.y_3d << "m z:" << i.z_3d << "m ";
                coords_3d = ss.str();
                cv::Size const text_size_3d = getTextSize(ss.str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1, 0);
                int const max_width_3d = (text_size_3d.width > i.w + 2) ? text_size_3d.width : (i.w + 2);
                if (max_width_3d > max_width) max_width = max_width_3d;
            }

            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)),
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
            if(!coords_3d.empty()) putText(mat_img, coords_3d, cv::Point2f(i.x, i.y-1), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);
        }
    }
    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: " + std::to_string(current_cap_fps);
        putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
    }
}
#endif    // OPENCV


std::mutex img_mutex;

static cv_bridge::CvImagePtr cv_image; //###
static uint32_t last_frame_number = 0;
static uint32_t curr_frame_number = 0;

void video_callback(const sensor_msgs::ImageConstPtr &msg) {
    ROS_DEBUG("[darknet::video_callback] frame received.");

    int frame_width; //###
    int frame_height; //###

    img_mutex.lock();
    cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    curr_frame_number++;

    frame_width = cv_image->image.size().width;
    frame_height = cv_image->image.size().height;
    ROS_INFO("darknet::video_callback]  frame_width: %d, frame_height: %d", frame_width, frame_height);
    img_mutex.unlock();

    return;
}

void publish_objects_found(ros::Publisher objectfound_publisher, std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, uint32_t frame_id = -1) {
    o_msgs::ObjectFoundList object_found_list;
    for (auto &i : result_vec) {
        o_msgs::ObjectFound object_found_message;
        object_found_message.object_id = i.obj_id;
        object_found_message.frame_id = frame_id;
        object_found_message.x = i.x;
        object_found_message.y = i.y;
        object_found_message.width = i.w;
        object_found_message.height = i.h;
        object_found_message.probability = i.prob;
        object_found_message.object_name = obj_names.size() > i.obj_id ? obj_names[i.obj_id] : "<BADNAME>";
        object_found_list.objects_found.push_back(object_found_message);
    }

    objectfound_publisher.publish(object_found_list);
}

void show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names, int frame_id = -1) {
    if (frame_id >= 0) std::cout << " Frame: " << frame_id << std::endl;
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
            << ", w = " << i.w << ", h = " << i.h
            << std::setprecision(3) << ", prob = " << i.prob << std::endl;
    }
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

template<typename T>
class send_one_replaceable_object_t {
    const bool sync;
    std::atomic<T *> a_ptr;
public:

    void send(T const& _obj) {
        T *new_ptr = new T;
        *new_ptr = _obj;
        if (sync) {
            while (a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));
    }

    T receive() {
        std::unique_ptr<T> ptr;
        do {
            while(!a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
            ptr.reset(a_ptr.exchange(NULL));
        } while (!ptr);
        T obj = *ptr;
        return obj;
    }

    bool is_object_present() {
        return (a_ptr.load() != NULL);
    }

    send_one_replaceable_object_t(bool _sync) : sync(_sync), a_ptr(NULL)
    {}
};

image_t make_empty_image(int w, int h, int c)
{
    image_t out;
    out.data = 0;
    out.h = h;
    out.w = w;
    out.c = c;
    return out;
}

image_t make_image_custom(int w, int h, int c)
{
    image_t out = make_empty_image(w, h, c);
    out.data = (float *)calloc(h*w*c, sizeof(float));
    return out;
}

image_t mat_to_image_custom(cv::Mat mat)
{
    int w = mat.cols;
    int h = mat.rows;
    int c = mat.channels();
    image_t im = make_image_custom(w, h, c);
    unsigned char *data = (unsigned char *)mat.data;
    int step = mat.step;
    for (int y = 0; y < h; ++y) {
        for (int k = 0; k < c; ++k) {
            for (int x = 0; x < w; ++x) {
                im.data[k*w*h + y*w + x] = data[y*step + x*c + k] / 255.0f;
            }
        }
    }
    return im;
}

image_t mat_to_image(cv::Mat img_src)
{
    cv::Mat img;
    if (img_src.channels() == 4) cv::cvtColor(img_src, img, cv::COLOR_RGBA2BGR);
    else if (img_src.channels() == 3) cv::cvtColor(img_src, img, cv::COLOR_RGB2BGR);
    else if (img_src.channels() == 1) cv::cvtColor(img_src, img, cv::COLOR_GRAY2BGR);
    else std::cerr << " Warning: img_src.channels() is not 1, 3 or 4. It is = " << img_src.channels() << std::endl;
    image_t image = mat_to_image_custom(img);
    return image;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "darknet_node");
	ros::NodeHandle nh;

    image_transport::ImageTransport image_transport(nh);
    image_transport::Subscriber image_subscriber;
    image_subscriber = image_transport.subscribe("/d435_left/color/image_raw"
                                                 , 1
                                                 , boost::bind(video_callback, _1)
                                                 , ros::VoidPtr()
                                                 , image_transport::TransportHints("compressed"));

     std::string  names_file = "data/coco.names";
    std::string  cfg_file = "cfg/yolov3.cfg";
    std::string  weights_file = "yolov3.weights";

    if (argc == 4) {    //voc.names yolo-voc.cfg yolo-voc.weights test.mp4
        names_file = argv[1];
        cfg_file = argv[2];
        weights_file = argv[3];
    } else {
        exit(-1);
    }

    float const thresh = (argc > 5) ? std::stof(argv[5]) : 0.2;

    ros::Publisher objectfound_publisher = nh.advertise<o_msgs::ObjectFoundList>("object_found", 1);

    Detector detector(cfg_file, weights_file);

    auto obj_names = objects_names_from_file(names_file);
    std::string out_videofile = "result.avi";
    bool const save_output_videofile = false;   // true - for history
    bool const use_kalman_filter = false;   // true - for stationary camera

    bool detection_sync = true;             // true - for video-file


    while (ros::ok())
     {
        if (last_frame_number != curr_frame_number) {
            last_frame_number = curr_frame_number;

            try {
                //#####auto img = detector.load_image(filename);
                img_mutex.lock();
                auto  img = mat_to_image(cv_image->image);
                std::vector<bbox_t> result_vec = detector.detect(img);
                publish_objects_found(objectfound_publisher, result_vec, obj_names, last_frame_number);
                detector.free_image(img);
                img_mutex.unlock();
                show_console_result(result_vec, obj_names);
            }
            catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
            catch (...) { std::cerr << "unknown exception \n"; getchar(); }
        }

        ros::spinOnce();
    }

    return 0;
}