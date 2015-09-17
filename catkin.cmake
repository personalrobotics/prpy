cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
    #catkin_add_nosetests(tests/test_PlanningPipeline.py)
    #catkin_add_nosetests(tests/test_DistanceFieldManager.py)
endif()
