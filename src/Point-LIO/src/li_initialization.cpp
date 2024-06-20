#include "li_initialization.h"

bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::Imu imu_last, imu_next;
// sensor_msgs::Imu::ConstPtr imu_last_ptr;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];



condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::Imu::Ptr> imu_deque;
std::deque<geometry_msgs::Pose::Ptr> gps_deque;

V3D ang_vel_meas = V3D::Zero();

void livox_base_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer.lock();
    livox1_queue.push(msg);
    mtx_buffer.unlock();
}

void livox_ref_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer.lock();
    livox2_queue.push(msg);
    mtx_buffer.unlock();
}

void livox2PointCloud2(const livox_ros_driver::CustomMsgConstPtr &lidarMsg, PointCloudXYZI &pclPointCloud) {
    for (unsigned int i = 0; i < lidarMsg->point_num; i++) {
        PointType point;
        point.x = lidarMsg->points[i].x;
        point.y = lidarMsg->points[i].y;
        point.z = lidarMsg->points[i].z;
        point.intensity = lidarMsg->points[i].reflectivity;
        point.curvature = lidarMsg->points[i].offset_time / float(1000000);
        pclPointCloud.push_back(point);
    }
}

livox_ros_driver::CustomMsg::ConstPtr merge_livox_msg1(const livox_ros_driver::CustomMsg::ConstPtr &livoxMsg1,
                                                       const livox_ros_driver::CustomMsg::ConstPtr &livoxMsg2) {
    livox_ros_driver::CustomMsg::ConstPtr merge_msg;
    PointCloudXYZI pc_msg1, pc_msg2, pc_msg2_transformed, pc_merge;
    livox2PointCloud2(livoxMsg1, pc_msg1);
    livox2PointCloud2(livoxMsg2, pc_msg2);
    /* 1. 对msg2通过外参进行刚体变换 */
    pcl::transformPointCloud(pc_msg2, pc_msg2_transformed, tf_base2ref);
    /* 2. 对msg1和msg2的起始时间进行比对，合并出最终的header信息 */
    pc_merge = pc_msg1 + pc_msg2_transformed;
    sort(pc_merge.begin(), pc_merge.end(), time_list);
    // FIXME 需要通过第二步的比对结果，重新生成header信息
    livox_ros_driver::CustomMsg final_merged;
    if (livoxMsg1->header.stamp.toSec() <= livoxMsg2->header.stamp.toSec())
        final_merged.header = livoxMsg1->header; // ROS standard message header
    else
        final_merged.header = livoxMsg2->header;
//    final_merged.header = livoxMsg1->header; // ROS standard message header
    if (livoxMsg1->timebase <= livoxMsg2->timebase)
        final_merged.timebase = livoxMsg1->timebase; // The time of first point
    else
        final_merged.timebase = livoxMsg2->timebase;

//    final_merged.timebase = livoxMsg1->timebase;
    final_merged.point_num = pc_merge.size(); // Total number of pointclouds
    final_merged.lidar_id = livoxMsg1->lidar_id; //  Lidar device id number
    final_merged.rsvd = livoxMsg1->rsvd;
    /* 3. 两帧点云的整体合并 */
    for (unsigned int i = 0; i < final_merged.point_num; i++) {
        livox_ros_driver::CustomPoint p;
        p.x = pc_merge[i].x;
        p.y = pc_merge[i].y;
        p.z = pc_merge[i].z;
        p.reflectivity = pc_merge[i].intensity;
        p.offset_time = pc_merge[i].curvature * float(1000000);
        final_merged.points.push_back(p);
    }
    merge_msg = boost::make_shared<livox_ros_driver::CustomMsg>(final_merged);
    return merge_msg;
}

livox_ros_driver::CustomMsg::ConstPtr merge_livox_msg2(const livox_ros_driver::CustomMsg::ConstPtr &livoxMsg1,
                                                       const livox_ros_driver::CustomMsg::ConstPtr &livoxMsg2) {
    livox_ros_driver::CustomMsg::ConstPtr merge_msg;
    PointCloudXYZI pc_msg2, pc_msg2_transformed;
    livox2PointCloud2(livoxMsg2, pc_msg2);
    pcl::transformPointCloud(pc_msg2, pc_msg2_transformed, tf_base2ref);

    auto timediff = livoxMsg1->timebase - livoxMsg2->timebase;
    livox_ros_driver::CustomMsg final_merged;
    final_merged.header = livoxMsg1->header;
//    if (timediff <= 0)
//        final_merged.timebase = livoxMsg1->timebase; // The time of first point
//    else
//        final_merged.timebase = livoxMsg2->timebase;
    final_merged.timebase = livoxMsg1->timebase; // The time of first point
    final_merged.point_num = livoxMsg1->point_num + livoxMsg2->point_num; // Total number of pointclouds
    final_merged.lidar_id = livoxMsg1->lidar_id; //  Lidar device id number
    final_merged.rsvd = livoxMsg1->rsvd;
    /* 将livoxmsg2中的点云做刚体变换后，更改每个点云的时间 */
    for (size_t i = 0; i < livoxMsg2->point_num; i++) {
        livox_ros_driver::CustomPoint p;
        p.x = pc_msg2_transformed[i].x;
        p.y = pc_msg2_transformed[i].y;
        p.z = pc_msg2_transformed[i].z;
        p.reflectivity = pc_msg2_transformed[i].intensity;
        p.offset_time = pc_msg2_transformed[i].curvature * float(1000000) + timediff;
//        p.offset_time += timediff;
        final_merged.points.push_back(p);
    }
    for (size_t i = 0; i < livoxMsg1->point_num; i++) {
        final_merged.points.push_back(livoxMsg1->points[i]);
    }
    merge_msg = boost::make_shared<livox_ros_driver::CustomMsg>(final_merged);
    return merge_msg;
}

void sync_multi_avia() {
    while (ros::ok()) {
        mtx_buffer.lock();
        if (!livox1_queue.empty() && !livox2_queue.empty()) {
//            printf("start sync multi lidar\n");
            auto time1 = livox1_queue.front()->header.stamp.toSec();
            auto time2 = livox2_queue.front()->header.stamp.toSec();
            if (time1 < time2 - 0.003) {
                livox1_queue.pop();
                printf("throw livox1\n");
            } else if (time1 > time2 + 0.003) {
                livox2_queue.pop();
                printf("throw livox2\n");
            } else {
                auto msg1 = livox1_queue.front();
//                livox_pcl_cbk(msg1);
                livox1_queue.pop();
                auto msg2 = livox2_queue.front();
//                livox_pcl_multi_cbk(msg2);
                livox2_queue.pop();
//                printf("find msg1 and msg2\n");
                auto msg_merge = merge_livox_msg1(msg1, msg2);
                livox_pcl_cbk(msg_merge);
//                livox1_queue.pop();
//                livox2_queue.pop();
            }
        }
        mtx_buffer.unlock();
    }
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = msg->header.stamp.toSec();
    // printf("check lidar time %f\n", last_timestamp_lidar);
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));
        p_pre->process(msg, ptr);
        if (con_frame) {
            if (frame_ct == 0) {
                time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
            }
            if (frame_ct < 10) {
                for (int i = 0; i < ptr->size(); i++) {
                    ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                    ptr_con->push_back(ptr->points[i]);
                }
                frame_ct++;
            } else {
                PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_con_i = *ptr_con;
                lidar_buffer.push_back(ptr_con_i);
                double time_con_i = time_con;
                time_buffer.push_back(time_con_i);
                ptr_con->clear();
                frame_ct = 0;
            }
        } else {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    // mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
        // lidar_buffer.shrink_to_fit();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    // hr: cut_frame_init always true
    if (cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000)); // unit:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
        p_pre->process(msg, ptr);
        if (con_frame) {
            if (frame_ct == 0) {
                time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
            }
            if (frame_ct < 10) {
                for (int i = 0; i < ptr->size(); i++) {
                    ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                    ptr_con->push_back(ptr->points[i]);
                }
                frame_ct++;
            } else {
                PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_con_i = *ptr_con;
                double time_con_i = time_con;
                lidar_buffer.push_back(ptr_con_i);
                time_buffer.push_back(time_con_i);
                ptr_con->clear();
                frame_ct = 0;
            }
        } else {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
    // mtx_buffer.lock();

    // publish_count ++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(
            msg->header.stamp.toSec() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = msg->header.stamp.toSec();
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    if (timestamp < last_timestamp_imu) {
        ROS_ERROR("imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        // cout << "check time:" << timestamp << ";" << last_timestamp_imu << endl;
        // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }
//    mtx_buffer.lock();
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
//    mtx_buffer.unlock();
    // sig_buffer.notify_all();
}


bool sync_packages(MeasureGroup &meas) {
    {
        /* hr 无IMU */
        if (!imu_en) {
            if (!lidar_buffer.empty()) {
                if (!lidar_pushed) {
                    meas.lidar = lidar_buffer.front();
                    meas.lidar_beg_time = time_buffer.front();
                    lose_lid = false;
                    if (meas.lidar->points.size() < 1) {
                        cout << "lose lidar" << std::endl;
                        // return false;
                        lose_lid = true;
                    } else {
                        double end_time = meas.lidar->points.back().curvature;
                        for (auto pt: meas.lidar->points) {
                            if (pt.curvature > end_time) {
                                end_time = pt.curvature;
                            }
                        }
                        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                        meas.lidar_last_time = lidar_end_time;
                    }
                    lidar_pushed = true;
                }

                time_buffer.pop_front();
                lidar_buffer.pop_front();
                lidar_pushed = false;
                if (!lose_lid) {
                    return true;
                } else {
                    return false;
                }
            }
            return false;
        }

        if (lidar_buffer.empty() || imu_deque.empty()) {
            return false;
        }
        /*** push a lidar scan ***/
        if (!lidar_pushed) {
            lose_lid = false;
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() < 1) {
                cout << "lose lidar" << endl;
                lose_lid = true;
                // lidar_buffer.pop_front();
                // time_buffer.pop_front();
                // return false;
            } else {
                double end_time = meas.lidar->points.back().curvature;
                for (auto pt: meas.lidar->points) {
                    if (pt.curvature > end_time) {
                        end_time = pt.curvature;
                    }
                }
                lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                // cout << "check time lidar:" << end_time << endl;
                meas.lidar_last_time = lidar_end_time;
            }
            lidar_pushed = true;
        }

        if (!lose_lid && (last_timestamp_imu < lidar_end_time)) {
            return false;
        }
        if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) {
            return false;
        }

        if (!lose_lid && !imu_pushed) {
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_) {
                double imu_time = imu_deque.front()->header.stamp.toSec();
                imu_next = *(imu_deque.front());
                // hr: 尽可能地释放内部存储空间，使得容器的内部存储空间大小等于容器中当前元素的数量。这样可以节省内存。
                meas.imu.shrink_to_fit();
                while (imu_time < lidar_end_time) {
                    //ROS_INFO("%lf", imu_deque.front()->angular_velocity.x);
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if (imu_deque.empty()) break;
                    imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    imu_next = *(imu_deque.front());
                }
            }
            imu_pushed = true;
        }

        // hr: 如果当前帧点云数量小于1，依旧接着push imu数据
        if (lose_lid && !imu_pushed) {
            /*** push imu data, and pop from imu buffer ***/
            if (p_imu->imu_need_init_) {
                double imu_time = imu_deque.front()->header.stamp.toSec();
                meas.imu.shrink_to_fit();

                imu_next = *(imu_deque.front());
                while (imu_time < meas.lidar_beg_time + lidar_time_inte) {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if (imu_deque.empty()) break;
                    imu_time = imu_deque.front()->header.stamp.toSec(); // can be changed
                    imu_next = *(imu_deque.front());
                }
            }
            imu_pushed = true;
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        imu_pushed = false;
        return true;
    }
}
