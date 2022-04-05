# Lidar_ICP_2d

두 scan 데이터를 point to point matching을 하여 ICP 정합을 하는 ros package 입니다.

# How to use

    cd ~/catkin_ws/src
    git clone https://github.com/thithin-ent/Lidar_ICP_2d.git
    cd ~/catkin_ws
    catkin_make
    rosrun Lidar_ICP_2d Lidar_ICP_2d


# subscriber topic

/scan1 (sensor_msgs/LaserScan)
  - 정합을 위한 첫번째 scan data입니다. 해당 scan데이터를 기준으로 scan2데이터가 정합됩니다.

/scan2 (sensor_msgs/LaserScan)
  - 정합을 위한 두번째 scan data입니다. scan2 데이터는 scan1데이터로 정합됩니다.

# publisher topic

/tf (tf/tfMessage)
  - Transforms 정보입니다. scan1데이터의 프레임과 scan2데이터의 프레임을 연결하는 tf를 발행합니다.

/caliblation_pointcloud (sensor_msgs/PointCloud)
  - 정합된 두 scan1 데이터와 scan2 데이터를 pointCloud로 발행합니다. 해당 pointCloud는 base_link 프레임으로 발행됩니다.

# example

 scan1data
 scan2data
 정합된 데이터

# how it work

1. scan1 데이터를 받는 경우 해당 데이터를 reference data로 지정하도록 합니다. 이때 데이터들의 검색의 편의성을 위해 Kd-tree구조로 바꾸어 저장합니다.

2. scan2 데이터를 받아 scan1데이터에서 가장 근처의 point 지점을 찾습니다. 실제 ICP 정합을 위해선 같은 물체의 point간의 거리를 알아야 하나, 어떤 데이터지점이 같은 물체인지 모르기 때문에 가장 가까운 point지점의 거리를 error 모델로 사용합니다.

3. 가장 가까운 point지점의 거리를 error 모델로 하여 각 iteration마다 비선형 최적화를 합니다. 비선형 최적화를 통해 rotation과 transration을 찾습니다.

4. 찾은 rotation과 transration을 통해 scan1과 scan2 데이터를 정합합니다.
