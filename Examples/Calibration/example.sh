in the ORB
./Examples/Calibration/recorder_realsense_D435i ./Examples/Calibration/recorder

python3 ./Examples/Calibration/python_scripts/process_imu.py ./Examples/Calibration/recorder/

in the kalibr
source devel/setup.zsh

export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1

rosrun kalibr kalibr_bagcreater --folder ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/recorder/. --output-bag ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/recorder.bag

rosrun kalibr kalibr_calibrate_cameras --bag ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/recorder.bag --topics /cam0/image_raw --models pinhole-radtan --target ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/april_6x6_80x80cm_larues.yaml

rosrun kalibr kalibr_calibrate_imu_camera --bag ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/recorder.bag --cam  ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/camera_calibration.yaml --imu  ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/imu_intrinsics.yaml --target  ~/robocon/ORB/ORB_SLAM3-debug/Examples/Calibration/april_6x6_80x80cm_larues.yaml
