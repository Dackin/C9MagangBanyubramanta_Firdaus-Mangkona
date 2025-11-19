#include <filesystem>
#include <memory>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "interfaces/msg/object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Konfigurasi untuk 2 class: Baskom dan Flare
#define N_CLASSES 2
#define INPUT_WIDTH 640
#define INPUT_HEIGHT 640
#define CONF_THRESH 0.4
#define SCORE_THRESH 0.4
#define NMS_THRESH 0.4

using namespace cv;
using namespace std;

// Class names untuk 2 objek
const char* class_names[] = {"Baskom", "Flare"};

// Color list untuk visualization (RGB format, akan dikali 255)
const float color_list[80][3] = {
    {0.000, 0.447, 0.741}, {0.850, 0.325, 0.098}, {0.929, 0.694, 0.125},
    {0.494, 0.184, 0.556}, {0.466, 0.674, 0.188}, {0.301, 0.745, 0.933},
    {0.635, 0.078, 0.184}, {0.300, 0.300, 0.300}, {0.600, 0.600, 0.600},
    {1.000, 0.000, 0.000}, {1.000, 0.500, 0.000}, {0.749, 0.749, 0.000},
    {0.000, 1.000, 0.000}, {0.000, 0.000, 1.000}, {0.667, 0.000, 1.000},
    {0.333, 0.333, 0.000}, {0.333, 0.667, 0.000}, {0.333, 1.000, 0.000},
    {0.667, 0.333, 0.000}, {0.667, 0.667, 0.000}, {0.667, 1.000, 0.000},
    {1.000, 0.333, 0.000}, {1.000, 0.667, 0.000}, {1.000, 1.000, 0.000},
    {0.000, 0.333, 0.500}, {0.000, 0.667, 0.500}, {0.000, 1.000, 0.500},
    {0.333, 0.000, 0.500}, {0.333, 0.333, 0.500}, {0.333, 0.667, 0.500},
    {0.333, 1.000, 0.500}, {0.667, 0.000, 0.500}, {0.667, 0.333, 0.500},
    {0.667, 0.667, 0.500}, {0.667, 1.000, 0.500}, {1.000, 0.000, 0.500},
    {1.000, 0.333, 0.500}, {1.000, 0.667, 0.500}, {1.000, 1.000, 0.500},
    {0.000, 0.333, 1.000}, {0.000, 0.667, 1.000}, {0.000, 1.000, 1.000},
    {0.333, 0.000, 1.000}, {0.333, 0.333, 1.000}, {0.333, 0.667, 1.000},
    {0.333, 1.000, 1.000}, {0.667, 0.000, 1.000}, {0.667, 0.333, 1.000},
    {0.667, 0.667, 1.000}, {0.667, 1.000, 1.000}, {1.000, 0.000, 1.000},
    {1.000, 0.333, 1.000}, {1.000, 0.667, 1.000}, {0.333, 0.000, 0.000},
    {0.500, 0.000, 0.000}, {0.667, 0.000, 0.000}, {0.833, 0.000, 0.000},
    {1.000, 0.000, 0.000}, {0.000, 0.167, 0.000}, {0.000, 0.333, 0.000},
    {0.000, 0.500, 0.000}, {0.000, 0.667, 0.000}, {0.000, 0.833, 0.000},
    {0.000, 1.000, 0.000}, {0.000, 0.000, 0.167}, {0.000, 0.000, 0.333},
    {0.000, 0.000, 0.500}, {0.000, 0.000, 0.667}, {0.000, 0.000, 0.833},
    {0.000, 0.000, 1.000}, {0.000, 0.000, 0.000}, {0.143, 0.143, 0.143},
    {0.286, 0.286, 0.286}, {0.429, 0.429, 0.429}, {0.571, 0.571, 0.571},
    {0.714, 0.714, 0.714}, {0.857, 0.857, 0.857}, {0.000, 0.447, 0.741},
    {0.314, 0.717, 0.741}, {0.50, 0.5, 0}};

struct Config {
    float confThreshold;
    float nmsThreshold;
    float scoreThreshold;
    int inpWidth;
    int inpHeight;
    std::string onnx_path;
};

struct Resize {
    cv::Mat resized_image;
    int dw;
    int dh;
};

struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
};

class YOLOV5 {
   public:
    YOLOV5(Config config);
    ~YOLOV5();
    std::vector<Detection> detect(cv::Mat& frame);
    void draw_detections(cv::Mat& frame, const std::vector<Detection>& detections);

   private:
    float confThreshold;
    float nmsThreshold;
    float scoreThreshold;
    int inpWidth;
    int inpHeight;
    float rx;  // width ratio of original image and resized image
    float ry;  // height ratio of original image and resized image
    std::string onnx_path;
    Resize resize;
    ov::Tensor input_tensor;
    ov::InferRequest infer_request;
    ov::CompiledModel compiled_model;
    
    void initialmodel();
    void preprocess_img(cv::Mat& frame);
    std::vector<Detection> postprocess_img(float* detections, ov::Shape& output_shape);
};

YOLOV5::YOLOV5(Config config) {
    this->confThreshold = config.confThreshold;
    this->nmsThreshold = config.nmsThreshold;
    this->scoreThreshold = config.scoreThreshold;
    this->inpWidth = config.inpWidth;
    this->inpHeight = config.inpHeight;
    this->onnx_path = config.onnx_path;
    this->initialmodel();
}

YOLOV5::~YOLOV5() {}

std::vector<Detection> YOLOV5::detect(Mat& frame) {
    preprocess_img(frame);
    infer_request.infer();
    const ov::Tensor& output_tensor = infer_request.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();
    
    // Fix warning: use const_cast untuk compatibility
    const float* detections_const = output_tensor.data<float>();
    float* detections = const_cast<float*>(detections_const);
    
    return this->postprocess_img(detections, output_shape);
}

void YOLOV5::initialmodel() {
    // OpenVINO preprocessing pipeline
    ov::Core core;
    std::shared_ptr<ov::Model> model = core.read_model(this->onnx_path);
    
    ov::preprocess::PrePostProcessor ppp =
        ov::preprocess::PrePostProcessor(model);
    
    ppp.input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    
    ppp.input()
        .preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255, 255, 255});
    
    ppp.input().model().set_layout("NCHW");
    ppp.output().tensor().set_element_type(ov::element::f32);
    
    model = ppp.build();
    this->compiled_model = core.compile_model(model, "CPU");
    this->infer_request = compiled_model.create_infer_request();
}

void YOLOV5::preprocess_img(cv::Mat& frame) {
    cv::Mat img;
    
    // Letterbox resize ke 640x640
    float scale = std::min(inpWidth / (float)frame.cols, inpHeight / (float)frame.rows);
    int new_w = (int)(frame.cols * scale);
    int new_h = (int)(frame.rows * scale);

    cv::resize(frame, img, cv::Size(new_w, new_h));

    int pad_w = inpWidth - new_w;
    int pad_h = inpHeight - new_h;

    cv::copyMakeBorder(img, img, pad_h / 2, pad_h - pad_h / 2,
                       pad_w / 2, pad_w - pad_w / 2,
                       cv::BORDER_CONSTANT, cv::Scalar(114,114,114));

    // Simpan rasio untuk scale back
    this->rx = frame.cols / (float)new_w;
    this->ry = frame.rows / (float)new_h;

    // BGR -> RGB
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    // Create OpenVINO tensor
    input_tensor = ov::Tensor(ov::element::u8,
                              {1, (size_t)inpHeight, (size_t)inpWidth, 3},
                              img.data);
    infer_request.set_input_tensor(input_tensor);
}

std::vector<Detection> YOLOV5::postprocess_img(float* detections, ov::Shape& output_shape) {
    std::vector<cv::Rect> boxes;
    std::vector<int> class_ids;
    std::vector<float> confidences;
    
    // Parse detections - fix warning dengan size_t
    size_t num_detections = output_shape[1];
    size_t detection_size = output_shape[2];
    
    for (size_t i = 0; i < num_detections; i++) {
        float* detection = &detections[i * detection_size];

        float confidence = detection[4];
        if (confidence >= this->confThreshold) {
            float* classes_scores = &detection[5];
            cv::Mat scores(1, static_cast<int>(detection_size - 5), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            
            if (max_class_score > this->scoreThreshold) {
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);
                
                float x = detection[0];
                float y = detection[1];
                float w = detection[2];
                float h = detection[3];
                float xmin = x - (w / 2);
                float ymin = y - (h / 2);

                boxes.push_back(cv::Rect(static_cast<int>(xmin), static_cast<int>(ymin), 
                                         static_cast<int>(w), static_cast<int>(h)));
            }
        }
    }

    // Non-Maximum Suppression
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold,
                      this->nmsThreshold, nms_result);

    std::vector<Detection> output;
    for (size_t i = 0; i < nms_result.size(); i++) {
        Detection result;
        int idx = nms_result[i];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        
        // Scale box back to original image size
        result.box.x = static_cast<int>(this->rx * result.box.x);
        result.box.y = static_cast<int>(this->ry * result.box.y);
        result.box.width = static_cast<int>(this->rx * result.box.width);
        result.box.height = static_cast<int>(this->ry * result.box.height);
        
        output.push_back(result);
    }
    
    return output;
}

void YOLOV5::draw_detections(cv::Mat& frame, const std::vector<Detection>& detections) {
    for (size_t i = 0; i < detections.size(); i++) {
        const auto& detection = detections[i];
        const auto& box = detection.box;
        int classId = detection.class_id;
        float confidence = detection.confidence;

        int xmax = box.x + box.width;
        int ymax = box.y + box.height;
        
        // Get color for this class
        cv::Scalar color = cv::Scalar(
            color_list[classId][0], 
            color_list[classId][1],
            color_list[classId][2]
        );
        
        // Determine text color based on background brightness
        float c_mean = cv::mean(color)[0];
        cv::Scalar txt_color;
        if (c_mean > 0.5) {
            txt_color = cv::Scalar(0, 0, 0);
        } else {
            txt_color = cv::Scalar(255, 255, 255);
        }
        
        // Draw bounding box
        cv::rectangle(frame, cv::Point(box.x, box.y), 
                     cv::Point(xmax, ymax), color * 255, 2);
        
        // Prepare label text
        int baseLine = 0;
        char text[256];
        sprintf(text, "%s %.1f%%", class_names[classId], confidence * 100);
        
        cv::Size label_size = cv::getTextSize(
            text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
        
        // Draw label background
        cv::Scalar txt_bk_color = color * 0.7 * 255;
        cv::rectangle(
            frame,
            cv::Rect(cv::Point(box.x, box.y),
                    cv::Size(label_size.width, label_size.height + baseLine)),
            txt_bk_color, -1);
        
        // Draw label text
        cv::putText(frame, text, 
                   cv::Point(box.x, box.y + label_size.height),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
    }
}

class ObjectDetectionNode : public rclcpp::Node {
   public:
    ObjectDetectionNode() : Node("object_detection") {
        // Hardcoded configuration
        Config config = {
            CONF_THRESH,    // confThreshold
            NMS_THRESH,     // nmsThreshold  
            SCORE_THRESH,   // scoreThreshold
            INPUT_WIDTH,    // inpWidth
            INPUT_HEIGHT,   // inpHeight
            "src/openvinoros2/models/Taqi.onnx"  // onnx_path
        };
        
        try {
            yolomodel_ = std::make_unique<YOLOV5>(config);
            RCLCPP_INFO(this->get_logger(), "YOLO model initialized successfully");
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO: %s", ex.what());
            throw;
        }
        
        // Subscriber untuk input image
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/raw_image", 10,
            std::bind(&ObjectDetectionNode::image_callback, this, 
                     std::placeholders::_1));
        
        // Publisher untuk detection results (custom message)
        object_pub_ = this->create_publisher<interfaces::msg::Object>(
            "/objects", 10);
        
        RCLCPP_INFO(this->get_logger(), "Object Detection Node ready!");
        RCLCPP_INFO(this->get_logger(), "Detecting: Baskom and Flare");
    }

   private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS Image to cv::Mat
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            // Run detection
            cv::Mat frame = cv_ptr->image.clone();
            std::vector<Detection> detections = yolomodel_->detect(frame);
            
            RCLCPP_INFO(this->get_logger(), "Detected %zu objects", detections.size());
            
            // Publish detection messages
            for (const auto& det : detections) {
                interfaces::msg::Object obj_msg;
                obj_msg.name = class_names[det.class_id];
                obj_msg.x = static_cast<int16_t>(det.box.x);
                obj_msg.y = static_cast<int16_t>(det.box.y);
                obj_msg.width = static_cast<int16_t>(det.box.width);
                obj_msg.height = static_cast<int16_t>(det.box.height);
                obj_msg.confidence = det.confidence;
                
                object_pub_->publish(obj_msg);
            }
            
            // Draw detections on frame
            yolomodel_->draw_detections(frame, detections);
            
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in image_callback: %s", ex.what());
            return;
        }
    }

    std::unique_ptr<YOLOV5> yolomodel_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_with_box_pub_;
    rclcpp::Publisher<interfaces::msg::Object>::SharedPtr object_pub_;
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}