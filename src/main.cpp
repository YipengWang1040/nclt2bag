#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <chrono>
#include <iostream>
#include <string>
#include <experimental/filesystem>
#include <filesystem>
#include <set>

#include <archive.h>
#include <archive_entry.h>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

////////////////////////////////////////// velodyne //////////////////////////////////////////////////////

struct archive_wrapper {
    struct archive* a;
    struct archive_entry* curr;
};

bool archive_init(archive_wrapper& wrapper, const std::string& file_name) {
    wrapper.a = archive_read_new();
    archive_read_support_filter_gzip(wrapper.a);  // since the dataset contains only tar.gz
    archive_read_support_format_tar(wrapper.a);

    int r = archive_read_open_filename(wrapper.a, file_name.c_str(), 10240);
    if (r != ARCHIVE_OK) {
        archive_read_free(wrapper.a);
        return false;
    }
    return true;
}

string archive_iterate(archive_wrapper& wrapper) {
    if (archive_read_next_header(wrapper.a, &wrapper.curr) == ARCHIVE_OK) {
        return archive_entry_pathname(wrapper.curr);
    }
    return "";
}

size_t archive_read(archive_wrapper& wrapper, void* buffer, const size_t& max_size) {
    size_t s = archive_entry_size(wrapper.curr);
    if (s > max_size) return s;
    return archive_read_data(wrapper.a, buffer, s);
}

bool archive_destroy(archive_wrapper& wrapper) {
    int r = archive_read_free(wrapper.a);
    return r == ARCHIVE_OK;
}

// from read_vel_sync.py provided by nclt dataset
void convert(double& x, double& y, double& z) {
    constexpr double scaling = 0.005;
    constexpr double offset = -100;
    x = x * scaling + offset;
    y = (y * scaling + offset) * -1;
    z = (z * scaling + offset) * -1;
}

struct velodyne_point {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    u_int8_t i;
    u_int8_t r;
};

struct ros_point {
    float x;
    float y;
    float z;
    float i;
    int16_t r;
    int16_t _;
};

std::vector<sensor_msgs::PointField> get_point_fileds() {
    sensor_msgs::PointField x_field;
    x_field.name = 'x';
    x_field.offset = 0;
    x_field.datatype = sensor_msgs::PointField::FLOAT32;
    x_field.count = 1;

    sensor_msgs::PointField y_field;
    y_field.name = 'y';
    y_field.offset = 4;
    y_field.datatype = sensor_msgs::PointField::FLOAT32;
    y_field.count = 1;

    sensor_msgs::PointField z_field;
    z_field.name = 'z';
    z_field.offset = 8;
    z_field.datatype = sensor_msgs::PointField::FLOAT32;
    z_field.count = 1;

    sensor_msgs::PointField i_field;
    i_field.name = "intensity";
    i_field.offset = 12;
    i_field.datatype = sensor_msgs::PointField::FLOAT32;
    i_field.count = 1;

    sensor_msgs::PointField r_field;
    r_field.name = "ring";
    r_field.offset = 16;
    r_field.datatype = sensor_msgs::PointField::INT16;
    r_field.count = 1;
    return {x_field, y_field, z_field, i_field, r_field};
}

size_t packet2point_cloud(const size_t& buffer_size, const char* packet_buffer, sensor_msgs::PointCloud2& cloud) {
    size_t curr_pos = 0;
    size_t length = buffer_size / 8;
    constexpr int step_size = 18;  // 4*3 + 4 + 2
    cloud.height = 1;
    cloud.width = length;
    cloud.point_step = step_size;
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.fields = get_point_fileds();
    cloud.data.resize(cloud.row_step);
    for (size_t i = 0; i < length; ++i) {
        const velodyne_point& pt = reinterpret_cast<const velodyne_point*>(packet_buffer)[i];
        double x = pt.x;
        double y = pt.y;
        double z = pt.z;
        double intensity = pt.i;
        convert(x, y, z);

        ros_point point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.i = intensity;
        point.r = pt.r;
        memcpy(cloud.data.data() + step_size * i, &point, step_size);
    }
    return length;
}


////////////////////////////////////////// imu //////////////////////////////////////////////////////
void read_imu(const std::string& path, std::vector<size_t>& time_stamps, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& mags,
              std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& accels, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& rots) {
    std::ifstream in(path);
    size_t time_stamp;
    char comma;
    string line;
    while (getline(in, line)) {
        if (line.find("nan") != line.npos) continue;
        double mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, rot_x, rot_y, rot_z;
        stringstream ss(line);
        ss >> time_stamp >> comma >> mag_x >> comma >> mag_y >> comma >> mag_z >> comma >> accel_x >> comma >> accel_y >> comma >> accel_z >> comma >> rot_x >> comma >> rot_y >> comma >> rot_z;
        if (isnan(mag_x) || isnan(mag_y) || isnan(mag_z) || isnan(accel_x) || isnan(accel_y) || isnan(accel_z) || isnan(rot_x) || isnan(rot_y) || isnan(rot_z)) continue;
        time_stamps.push_back(time_stamp);
        mags.emplace_back(mag_x, mag_y, mag_z);
        accels.emplace_back(accel_x, accel_y, accel_z);
        rots.emplace_back(rot_x, rot_y, rot_z);
    }
}


////////////////////////////////////////// ground_truth //////////////////////////////////////////////////////
void read_ground_truth(const std::string& path, std::vector<size_t>& time_stamps, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& positions,
                       std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& rotations) {
    std::ifstream in(path);
    size_t time_stamp;
    char comma;
    string line;
    while (getline(in, line)) {
        if (line.find("nan") != line.npos) continue;
        double x, y, z, roll, pitch, yaw;
        stringstream ss(line);
        ss >> time_stamp >> comma >> x >> comma >> y >> comma >> z >> comma >> roll >> comma >> pitch >> comma >> yaw;
        if (isnan(x) || isnan(y) || isnan(z) || isnan(roll) || isnan(pitch) || isnan(yaw)) continue;
        time_stamps.push_back(time_stamp);
        positions.emplace_back(x, y, z);
        rotations.emplace_back(roll, pitch, yaw);
    }
}

Eigen::Quaterniond euler2quaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}


std::set<std::string> get_topics(const std::string& bag_path) {
    std::set<std::string> ret;
    if (!std::filesystem::exists(bag_path)) {
        return ret;
    }
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(bag);
    for (auto&& connection_info : view.getConnections()) {
        if (ret.find(connection_info->topic) == ret.end()) {
            ret.insert(connection_info->topic);
        }
    }
    bag.close();
    return ret;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "usage: ./nclt2bag path date" << endl;
        return -1;
    }

    std::cout << std::unitbuf;

    chrono::steady_clock::time_point start = chrono::steady_clock::now();

    string path = argv[1];
    string date = argv[2];

    string ground_truth_path = path + "/ground_truth/groundtruth_" + date + ".csv";
    string imu_path = path + "/sensor_data/" + date + "/ms25.csv";
    //    string ground_truth_cov_path =
    //    path+"/ground_truth_cov/cov_"+date+".csv";
    string velodyne_path = path + "/velodyne_data/" + date + "_vel.tar.gz";
    string bag_path = path + "/rosbags/" + date + ".bag";

    string gt_topic = "/nclt/gt_odometry";
    string imu_topic = "/nclt/imu";
    string lidar_topic = "/nclt/velodyne_points";
    string gt_frame = "nclt_gt";
    string lidar_frame = "velodyne";
    string imu_frame = "imu_link";

    // check what topics are already in the bag
    std::set<std::string> topics = get_topics(bag_path);

    // rosbag
    rosbag::Bag bag;
    if (std::filesystem::exists(bag_path)) {
        cout << "Bag openned in append mode." << endl;
        bag.open(bag_path, rosbag::bagmode::Append);
    } else {
        bag.open(bag_path, rosbag::bagmode::Write);
    }

    // ground_truth
    if (topics.find(gt_topic) == topics.end()) {
        cout << "processing ground truth data..." << endl;
        std::vector<size_t> time_stamps;  // us
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rotations;
        read_ground_truth(ground_truth_path, time_stamps, positions, rotations);

        Eigen::Vector3d initial_position = positions[0];
        Eigen::Quaterniond initial_rotation = euler2quaternion(rotations[0].x(), rotations[0].y(), rotations[0].z());
        Eigen::Quaterniond initial_rotation_inv = initial_rotation.inverse();

        Eigen::Quaterniond q_rectify(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

        for (size_t i = 0; i < time_stamps.size(); ++i) {
            nav_msgs::Odometry odometry;
            odometry.header.seq = i;
            odometry.header.frame_id = gt_frame;
            odometry.header.stamp.fromNSec(time_stamps[i] * 1000);
            Eigen::Vector3d position = initial_rotation_inv * (positions[i] - initial_position);
            odometry.pose.pose.position.x = position.x();
            odometry.pose.pose.position.y = -position.y();
            odometry.pose.pose.position.z = -position.z();
            Eigen::Quaterniond q = q_rectify * initial_rotation_inv * euler2quaternion(rotations[i].x(), rotations[i].y(), rotations[i].z());
            odometry.pose.pose.orientation.x = q.x();
            odometry.pose.pose.orientation.y = q.y();
            odometry.pose.pose.orientation.z = q.z();
            odometry.pose.pose.orientation.w = q.w();
            bag.write(gt_topic, odometry.header.stamp, odometry);
        }
        cout << "Done." << endl;
    } else {
        cout << "Ground truth existed. Skipped." << endl;
    }

    // imu
    if (topics.find(imu_topic) == topics.end()) {
        cout << "processing imu data..." << endl;
        std::vector<size_t> time_stamps;  // us
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> mags;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> accels;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rots;
        read_imu(imu_path, time_stamps, mags, accels, rots);


        for (size_t i = 0; i < time_stamps.size(); ++i) {
            sensor_msgs::Imu imu;
            imu.header.seq = i;
            imu.header.frame_id = imu_frame;
            imu.header.stamp.fromNSec(time_stamps[i] * 1000);

            imu.linear_acceleration.x = -accels[i].y();
            imu.linear_acceleration.y = -accels[i].x();
            imu.linear_acceleration.z = -accels[i].z();
            imu.angular_velocity.x = -rots[i].y();
            imu.angular_velocity.y = -rots[i].x();
            imu.angular_velocity.z = -rots[i].z();

            bag.write(imu_topic, imu.header.stamp, imu);
        }
        cout << "Done." << endl;
    } else {
        cout << "IMU data existed. Skipped." << endl;
    }


    // velodyne
    if (topics.find(lidar_topic) == topics.end()) {
        cout << "Opening velodyne file..." << endl;
        archive_wrapper wrapper;
        if (!archive_init(wrapper, velodyne_path)) {
            cerr << "unable to open lidar data \"" + velodyne_path + "\"." << endl;
            bag.close();
            return -1;
        }

        string entry_name;
        size_t buffer_size = 100 * 1024 * 1024;  // 100 MB
        char* buffer = new char[buffer_size];
        size_t idx = 0;
        while (!(entry_name = archive_iterate(wrapper)).empty()) {
            if (entry_name.find("velodyne_sync") == entry_name.npos) continue;
            size_t time_stamp = stoll(entry_name.substr(entry_name.find_last_of('/') + 1, 16));
            size_t actual_size = archive_read(wrapper, buffer, buffer_size);
            while (actual_size > buffer_size) {
                delete[] buffer;
                buffer_size = actual_size + 1024;
                if (buffer_size > 1024 * 1024 * 1024) {
                    // something must went wrong
                    bag.close();
                    archive_destroy(wrapper);
                    return -1;
                }
                buffer = new char[buffer_size];
                cout << endl << "buffer reallocated to " << buffer_size << endl;
                actual_size = archive_read(wrapper, buffer, buffer_size);
            }
            sensor_msgs::PointCloud2 msg;
            packet2point_cloud(actual_size, buffer, msg);
            msg.header.stamp.fromNSec(time_stamp * 1000);
            msg.header.frame_id = lidar_frame;
            msg.header.seq = idx++;
            bag.write(lidar_topic, msg.header.stamp, msg);
            cout << "\rfinished processing " << idx << "-th frame of point cloud...";
        }
        delete[] buffer;
    } else {
        cout << "Velodyne data existed. Skipped." << endl;
    }


    bag.close();
    cout << endl << "Done." << endl;
    cout << "time consumed: " << chrono::duration<double>(chrono::steady_clock::now() - start).count() << " seconds." << endl;


    return 0;
}