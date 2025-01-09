cmake_minimum_required(VERSION 3.0.2)
project(nmea_navsat_driver)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()
catkin_package()

install(PROGRAMS
   scripts/nmea_serial_driver
   scripts/nmea_socket_driver
   scripts/nmea_topic_driver
   scripts/nmea_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

install(FILES
   LICENSE.txt
   test/requirements.txt
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


