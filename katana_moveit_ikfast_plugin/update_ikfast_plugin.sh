rosrun moveit_kinematics create_ikfast_moveit_plugin.py calvin arm katana_moveit_ikfast_plugin $(rospack find katana_moveit_ikfast_plugin)/src/katana_450_6m90a_ikfast_solver.cpp
mv calvin_arm_moveit_ikfast_plugin_description.xml katana_moveit_ikfast_plugin_description.xml
mv src/calvin_arm_ikfast_moveit_plugin.cpp src/katana_450_6m90a_ikfast_plugin.cpp
mv src/calvin_arm_ikfast_solver.cpp src/katana_450_6m90a_ikfast_solver.cpp
sed -i 's/calvin_arm_ikfast_moveit_plugin/katana_450_6m90a_ikfast_plugin/g' $(git ls-files)
sed -i 's/calvin_arm_ikfast_solver/katana_450_6m90a_ikfast_solver/g' $(git ls-files)
sed -i 's/calvin_arm_moveit_ikfast_plugin_description/katana_moveit_ikfast_plugin_description/g' $(git ls-files)
sed -i 's/calvin_arm_moveit_ikfast_plugin/katana_moveit_ikfast_kinematics_plugin/g' $(git ls-files)
sed -i 's/calvin_arm_kinematics/katana_450_6m90a_kinematics/g' $(git ls-files)
sed -i 's/ikfast_kinematics_plugin::IKFastKinematicsPlugin/katana_450_6m90a_kinematics::IKFastKinematicsPlugin/g' $(git ls-files)
sed -i 's/namespace ikfast_kinematics_plugin/namespace katana_450_6m90a_kinematics/g' $(git ls-files)
git checkout update_ikfast_plugin.sh
# TODO: manually remove duplicate moveit_core plugin line from package.xml
