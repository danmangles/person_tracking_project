// The tag frame reported is relative to the optical camera frame
// Thus, distance forward is in z (positive)
//
// The orientation of the tag itself is:
// X - towards the text at the top of the tag
// Y - towards the shorter side of the tag
// Z - into the page the tag is printed on
//
// Can detect a 17cm tag out to 10m (in the camera optical z direction)

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <apriltags/apriltag.h>
#include <apriltags/common/image_u8.h>
#include <apriltags/tag36h11.h>
#include <apriltags/common/zarray.h>
#include <apriltags/common/getopt.h>
#include <Eigen/Dense>

struct TagMatch {
    int id;
    cv::Point2d p0, p1, p2, p3;
    Eigen::Matrix3d H;
};

Eigen::Isometry3d getRelativeTransform(TagMatch const& match, Eigen::Matrix3d const & K, double tag_size)
{
  // Was having problems with float and double conversions.
  // It was necessary to cast to double/float explicitly
  // I think OpenCv is the one which gets confused

  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size/2.;
  objPts.push_back(cv::Point3f(-s,-s, 0.));
  objPts.push_back(cv::Point3f( s,-s, 0.));
  objPts.push_back(cv::Point3f( s, s, 0.));
  objPts.push_back(cv::Point3f(-s, s, 0.));

  imgPts.push_back(match.p0);
  imgPts.push_back(match.p1);
  imgPts.push_back(match.p2);
  imgPts.push_back(match.p3);

  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
                           (float) K(0,0), (float) K(0,1), (float) K(0,2),
                           (float) K(1,0), (float) K(1,1), (float) K(1,2),
                           (float) K(2,0), (float) K(2,1), (float) K(2,2) );

  cv::Vec4f distParam(0.,0.,0.,0.);

  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33f r;

  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  double tvec_x = (double) tvec.at<float>(0);
  double tvec_y = (double) tvec.at<float>(1);
  double tvec_z = (double) tvec.at<float>(2);

  Eigen::Matrix4d T_matrix;
  T_matrix.topLeftCorner(3,3) = wRo;
  T_matrix.col(3).head(3) << tvec_x, tvec_y, tvec_z;
  T_matrix.row(3) << 0,0,0,1;
  Eigen::Isometry3d T(T_matrix);

  return T;
}

class AprilTagDetector {
    public:
    AprilTagDetector(getopt_t *options) : getopt(options) {
        tf = tag36h11_create();
        tf->black_border = getopt_get_int(options, "border");
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);

        td->quad_decimate = getopt_get_double(getopt, "decimate");
        td->quad_sigma = getopt_get_double(getopt, "blur");
        td->nthreads = getopt_get_int(getopt, "threads");
        td->debug = getopt_get_bool(getopt, "debug");
        td->refine_edges = getopt_get_bool(getopt, "refine-edges");
        td->refine_decode = getopt_get_bool(getopt, "refine-decode");
        td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    }

    ~AprilTagDetector() {

        apriltag_detector_destroy(td);
        tag36h11_destroy(tf);
    }

    std::vector<TagMatch> detectTags(image_u8_t *im) {

        const int hamm_hist_max = 10;

        int hamm_hist[hamm_hist_max];
        memset(hamm_hist, 0, sizeof(hamm_hist));
        zarray_t *detections = apriltag_detector_detect(td, im);

        std::vector<TagMatch> tag_matches;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (verbose)
                printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
                       i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);

            for (int x = 0; x < 3 ; x++) {
                image_u8_draw_line(im, det->p[x][0], det->p[x][1], det->p[x+1][0], det->p[x+1][1], 255, 10);
            }
            TagMatch tag_match;
            tag_match.id = det->id; //det->family->d*det->family->d;
            tag_match.p0 = cv::Point2d(det->p[0][0], det->p[0][1]);
            tag_match.p1 = cv::Point2d(det->p[1][0], det->p[1][1]);
            tag_match.p2 = cv::Point2d(det->p[2][0], det->p[2][1]);
            tag_match.p3 = cv::Point2d(det->p[3][0], det->p[3][1]);

            Eigen::Map<Eigen::Matrix3d> H_map(det->H->data);
            tag_match.H = H_map.transpose();
            tag_matches.push_back(tag_match);
            hamm_hist[det->hamming]++;
        }

        apriltag_detections_destroy(detections);

        if (verbose) {
            timeprofile_display(td->tp);
            printf("nedges: %d, nsegments: %d, nquads: %d\n", td->nedges, td->nsegments, td->nquads);
            printf("Hamming histogram: ");
            for (int i = 0; i < hamm_hist_max; i++)
                printf("%5d", hamm_hist[i]);
            printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
            printf("\n");
        }

        return tag_matches;
    }

    bool verbose;
    double tag_size;

    private:
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    getopt_t *getopt;
};

class CameraListener {
    public:
        std::string camera_topic;
        std::string image_transport_mode;
        std::string camera_info_topic;
        int64_t prev_utime_;
        double max_processing_fps;

    CameraListener(ros::NodeHandle node_, image_transport::ImageTransport imageTransport_) :
       node_(node_), imageTransport_(imageTransport_), initialised_(false){

    }

    void setDetector(AprilTagDetector* detector) {
        mDetector = detector;
    }

    bool setup(bool show_window, bool publish_img_with_matches = false) {

        std::cout << "Subscribing to " << camera_topic << std::endl;
        std::cout << "image_transport_mode: " << image_transport_mode << std::endl;
        std::cout << "Subscribing to " << camera_info_topic << " (info)" <<  std::endl;

        imageTransportSub_ = imageTransport_.subscribe(camera_topic, 1, &CameraListener::imageCallback, this, image_transport::TransportHints(image_transport_mode));
        cameraInfoSub_ = node_.subscribe(camera_info_topic, 1, &CameraListener::imageInfoCallback, this);

        detection_pub_ = imageTransport_.advertise("apriltag_detections", 1);

        // example values for a RealSense D435, should not be used
        K_ = Eigen::Matrix3d::Identity();
        K_(0,0) = 618.7470092773438;//focal_length_x
        K_(1,1) = 618.8784790039062;//focal_length_y
        K_(0,2) = 317.176025390625;//principal_x
        K_(1,2) = 246.12997436523438;//principal_y
        K_(2,2) = 1; //should be one

        mPublishImageWithMatches = publish_img_with_matches;

        mShowWindow = show_window;
        printf("Subscriber setup done\n" );
        return true;
    }

    void imageInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){
        if(!initialised_){

            ROS_INFO_STREAM("Cam info received: ");
            K_ = Eigen::Matrix3d::Identity();
            K_(0,0) = msg->K[0]; // focal_length_x
            K_(0,1) = msg->K[1]; // should be zero
            K_(0,2) = msg->K[2]; // principal_x
            K_(1,0) = msg->K[3]; // should be zero
            K_(1,1) = msg->K[4]; // focal_length_y
            K_(1,2) = msg->K[5]; // principal_y
            K_(2,0) = msg->K[6]; // should be zero
            K_(2,1) = msg->K[7]; // should be zero
            K_(2,2) = msg->K[8]; // should be one

            ROS_INFO_STREAM(K_);
            initialised_ = true;
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        if(!initialised_){
            ROS_INFO_STREAM("Camera info not received yet. Ignoring image");
            return;
        }

        int64_t utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);

        if (abs(utime-prev_utime_) < (1E6/max_processing_fps) ){
            //ROS_INFO_STREAM("skip " << utime);
            return;
        }

        prev_utime_ = utime;
        ROS_INFO_STREAM_THROTTLE(1, "Processing image of type " << msg->encoding << " at " << utime);

        cv_bridge::CvImageConstPtr image_ptr;
        image_ptr = cv_bridge::toCvShare (msg, sensor_msgs::image_encodings::RGB8);

        cv::Mat image;
        cv::cvtColor(image_ptr->image, image, CV_RGB2GRAY);

        processImage(image, utime, msg->header.frame_id);
    }

    void processImage(cv::Mat image, int64_t utime, std::string camera_frame_id){
        image_u8_t *image_u8 = fromCvMat(image);

        std::vector<TagMatch> tags = mDetector->detectTags(image_u8);
        cv::cvtColor(image, image, CV_GRAY2RGB);

        for (int i = 0; i < tags.size(); i++) {
            // Draw on the detections:
            cv::line(image, tags[i].p0, tags[i].p1, cv::Scalar(255,0,0), 2, CV_AA);
            cv::line(image, tags[i].p1, tags[i].p2, cv::Scalar(0,255,0), 2, CV_AA);
            cv::line(image, tags[i].p2, tags[i].p3, cv::Scalar(0,0,255), 2, CV_AA);
            cv::line(image, tags[i].p3, tags[i].p0, cv::Scalar(0,0,255), 2, CV_AA);

            Eigen::Vector3d x_axis(2,0,1);
            Eigen::Vector3d y_axis(0,2,1);
            Eigen::Vector3d origin(0,0,1);

            Eigen::Vector3d px = tags[i].H * x_axis;
            Eigen::Vector3d py = tags[i].H * y_axis;
            Eigen::Vector3d o  = tags[i].H * origin;

            px/= px[2];
            py/= py[2];
            o/= o[2];

            cv::line(image, cv::Point2d(o[0], o[1]), cv::Point2d(px[0], px[1]), cv::Scalar(255,0,255), 1, CV_AA);
            cv::line(image, cv::Point2d(o[0], o[1]), cv::Point2d(py[0], py[1]), cv::Scalar(255,255,0), 1, CV_AA);


            // Determine and publish the relative tf
            Eigen::Isometry3d tag_to_camera = getRelativeTransform(tags[i], K_, mDetector->tag_size);
            tf::Transform transform;
            tf::poseEigenToTF( tag_to_camera, transform);
            std::stringstream ss;
            ss << "tag_" <<  tags[i].id ;
            ROS_INFO_STREAM_THROTTLE(1, "Detected " << ss.str() << " at " << utime);
            br_.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(utime * 1E-6), camera_frame_id, ss.str() ));

        }

        if (mShowWindow) {
            cv::imshow("detections", image);
            cv::waitKey(1);
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        detection_pub_.publish(msg);

        image_u8_destroy(image_u8);
    }

    image_u8_t *fromCvMat(const cv::Mat & img) {
        image_u8_t *image_u8 = image_u8_create_alignment(img.cols, img.rows, img.step);
        int size = img.total() * img.elemSize();
        memcpy(image_u8->buf, img.data, size * sizeof(uint8_t));
        return image_u8;
    }

    private:
    bool mShowWindow;
    bool mPublishImageWithMatches;
    AprilTagDetector *mDetector;
    bool initialised_;
    Eigen::Matrix3d K_;

    ros::NodeHandle node_;
    image_transport::Subscriber imageTransportSub_;
    ros::Subscriber cameraInfoSub_;
    image_transport::ImageTransport imageTransport_;
    tf::TransformBroadcaster br_;

    image_transport::Publisher detection_pub_;

};



int main(int argc, char *argv[])
{

    // These old settings have not been changed to Param yet:
    getopt_t *getopt = getopt_create();
    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }


    bool verbose = false;
    std::string camera_topic = "camera/image_raw";
    std::string camera_info_topic = "camera/camera_info";
    std::string image_transport_mode = "raw";
    bool show_window = false;
    double max_processing_fps = 100; // really high basically means "all frames"
    double tag_size =0.1735;


    ros::init(argc, argv, "anakin_detector_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    ////////////////////////////
    AprilTagDetector tag_detector(getopt);
    nh.getParam("verbose", verbose);
    nh.getParam("tag_size", tag_size);

    tag_detector.verbose = verbose;
    tag_detector.tag_size = tag_size;

    ////////////////////////////
    CameraListener camera_listener(nh, it);
    nh.getParam("camera_topic", camera_topic);
    nh.getParam("image_transport_mode", image_transport_mode);
    nh.getParam("camera_info_topic", camera_info_topic);
    nh.getParam("show_window", show_window);
    nh.getParam("max_processing_fps", max_processing_fps);

    camera_listener.camera_topic = camera_topic;
    camera_listener.image_transport_mode = image_transport_mode;
    camera_listener.camera_info_topic = camera_info_topic;
    camera_listener.max_processing_fps = max_processing_fps;

    ////////////////////////////
    camera_listener.setup(show_window);
    camera_listener.setDetector(&tag_detector);

    ROS_INFO("apriltags_node ready");
    ros::spin();

    return 0;
}

