# source /home/chasing/Downloads/prometheussuper/devel/setup.bash

# export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/chasing/Downloads/prometheussuper/devel/lib
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/chasing/Downloads/prometheussuper/Simulator/gazebo_simulator/gazebo_models/uav_models
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/chasing/Downloads/prometheussuper/Simulator/gazebo_simulator/gazebo_models/ugv_models
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/chasing/Downloads/prometheussuper/Simulator/gazebo_simulator/gazebo_models/sensor_models
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/chasing/Downloads/prometheussuper/Simulator/gazebo_simulator/gazebo_models/scene_models
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/chasing/Downloads/prometheussuper/Simulator/gazebo_simulator/gazebo_models/texture

# source /home/chasing/Public/prometheus_px4/Tools/setup_gazebo.bash /home/chasing/Public/prometheus_px4 /home/chasing/Public/prometheus_px4/build/amovlab_sitl_default
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/chasing/Public/prometheus_px4
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/chasing/Public/prometheus_px4/Tools/sitl_gazebo



source ~/prometheussuperp600gazebo/devel/setup.bash

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/prometheussuperp600gazebo/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/prometheussuperp600gazebo/Simulator/gazebo_simulator/gazebo_models/uav_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/prometheussuperp600gazebo/Simulator/gazebo_simulator/gazebo_models/ugv_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/prometheussuperp600gazebo/Simulator/gazebo_simulator/gazebo_models/sensor_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/prometheussuperp600gazebo/Simulator/gazebo_simulator/gazebo_models/scene_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/prometheussuperp600gazebo/Simulator/gazebo_simulator/gazebo_models/texture

source ~/prometheus_px4/Tools/setup_gazebo.bash ~/prometheus_px4 ~/prometheus_px4/build/amovlab_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/prometheus_px4/Tools/sitl_gazebo