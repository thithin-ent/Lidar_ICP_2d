#include <Lidar_ICP/Lidar_ICP.h>
#include "NN.cpp"

Lidar_ICP::Lidar_ICP()
{
    scan1_sub_ = nh_.subscribe("scan1", 1, &Lidar_ICP::scan1_callback, this);
    scan2_sub_ = nh_.subscribe("scan2", 1, &Lidar_ICP::scan2_callback, this);
}

Lidar_ICP::~Lidar_ICP()
{
}

void Lidar_ICP::scan1_callback(const sensor_msgs::LaserScanConstPtr &data)
{
    //cout << "scan1_callback running " << endl;
    scan1_poses_.clear();
    int index_max = (data->angle_max - data->angle_min) / data->angle_increment;
    for (int i = 0; i < index_max; i++)
    {
        float range_temp = data->ranges[i];
        if (range_temp < data->range_min)
        {
            continue;
        }

        float angle_temp = data->angle_min + i * data->angle_increment;
        Vector2f scan_pose;
        scan_pose << range_temp * cos(angle_temp), range_temp * sin(angle_temp);
        scan1_poses_.push_back(scan_pose);
    }
    //cout << "scan1_callback end " << endl;
    ready1_ = true;
}

void Lidar_ICP::scan2_callback(const sensor_msgs::LaserScanConstPtr &data)
{
    //cout << "scan2_callback running " << endl;
    scan2_poses_.clear();
    int index_max = (data->angle_max - data->angle_min) / data->angle_increment;

    for (int i = 0; i < index_max; i++)
    {
        float range_temp = data->ranges[i];
        if (range_temp < data->range_min)
        {
            continue;
        }

        float angle_temp = data->angle_min + i * data->angle_increment;
        Vector2f scan_pose;
        scan_pose << range_temp * cos(angle_temp), range_temp * sin(angle_temp);
        scan2_poses_.push_back(scan_pose);
    }
    //cout << "scan2_callback end " << endl;
    ready2_ = true;
}

void Lidar_ICP::ICP()
{
    if (!ready1_ || !ready2_){
        return;
    }
    T_ << 1,0,0, 0,1,0, 0,0,1;
    for (int itertate = 0; itertate < 50; itertate++)
    {
        cout << "iteration: " << itertate << endl;

        Kdnode *root = new Kdnode;
        kd_tree(scan1_poses_, 0, root);
        vector<vector<Vector2f>> point_pairs;
        
        //for(int test_1 = 0; test_1 < scan1_poses_.size();test_1++){
        //    cout << scan1_poses_[test_1] << endl;
        //}
        //root->print_kdnode();


        //cout << "scan2_poses_.size: " << scan2_poses_.size() << endl;
        for (int i = 0; i < scan2_poses_.size(); i++)
        {
            //cout << "point paire ready...: " << i << endl;
            Kdnode *best = nn(root, scan2_poses_[i], 0);
            //cout << "best best best...: " << best->data << endl;
            vector<Vector2f> point_pair;
            Vector2f A,B;
            double distance = (best->data - scan2_poses_[i]).transpose() * (best->data - scan2_poses_[i]);

            if (distance > 1000)
                continue;
            A << best->data(0), best->data(1);
            B << scan2_poses_[i](0), scan2_poses_[i](1);


            point_pair.push_back(B);
            point_pair.push_back(A);
            point_pairs.push_back(point_pair);
        }

        //cout << "point paire end" << endl;
        point_base_matching(point_pairs);

        if (point_pairs.size() == 0)
        {
            cout << "error" << endl;
            return;
        }

        for (int i = 0; i < scan2_poses_.size(); i++)
        {
            Vector2d temp;
            temp << scan2_poses_[i](0), scan2_poses_[i](1);
            temp = rotation_ * temp;
            temp(0) = temp(0) + trans_(0);
            temp(1) = temp(1) + trans_(1);
            scan2_poses_.at(i) << temp(0), temp(1);
        }
        //print_rotation();
        //print_trans();
        print_T();
    }
}

void Lidar_ICP::point_base_matching(const vector<vector<Vector2f>> &point_pairs)
{

    long double x_mean = 0;
    long double y_mean = 0;
    long double xp_mean = 0;
    long double yp_mean = 0;
    int index = point_pairs.size();
    if (index == 0)
    {
        cout << "error" << endl;
        return;
    }

    for (int i = 0; i < index; i++)
    {
        x_mean = x_mean + point_pairs[i][0](0);
        y_mean += point_pairs[i][0](1);
        xp_mean += point_pairs[i][1](0);
        yp_mean += point_pairs[i][1](1);
    }

    x_mean /= index;
    y_mean /= index;
    xp_mean /= index;
    yp_mean /= index;

    double s_x_xp = 0;
    double s_y_yp = 0;
    double s_x_yp = 0;
    double s_y_xp = 0;

    for (int i = 0; i < index; i++)
    {
        s_x_xp += (point_pairs[i][0](0) - x_mean) * (point_pairs[i][1](0) - xp_mean);
        s_y_yp += (point_pairs[i][0](1) - y_mean) * (point_pairs[i][1](1) - yp_mean);
        s_x_yp += (point_pairs[i][0](0) - x_mean) * (point_pairs[i][1](1) - yp_mean);
        s_y_xp += (point_pairs[i][0](1) - y_mean) * (point_pairs[i][1](0) - xp_mean);
    }

    double rot_angle = atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp);
    double translation_x = xp_mean - (x_mean * cos(rot_angle) - y_mean * sin(rot_angle));
    double translation_y = yp_mean - (x_mean * sin(rot_angle) + y_mean * cos(rot_angle));

    rotation_ << cos(rot_angle), -sin(rot_angle), sin(rot_angle), cos(rot_angle);
    trans_ << translation_x, translation_y;

    Matrix3d T;
    T << rotation_, trans_, 0,0,1;
    T_ =  T*T_;
}

void Lidar_ICP::print_rotation()
{
    cout << "rotations " << endl;
    cout << rotation_ << endl;
}

void Lidar_ICP::print_trans()
{
    cout << "trans " << endl;
    cout << trans_ << endl;
}

void Lidar_ICP::print_T()
{
    cout << "T " << endl;
    cout << T_ << endl;
}

double Lidar_ICP::get_transx()
{
    return T_(0,2);
}

double Lidar_ICP::get_transy()
{
    return T_(0,2);
}

double Lidar_ICP::get_rotation()
{  
    return acos(T_(0,0));
}