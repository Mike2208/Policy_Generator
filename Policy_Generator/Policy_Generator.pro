TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    algorithm_optimal_single_bot.cpp \
    graph_class.cpp \
    graph_edge.cpp \
    graph_vertice.cpp \
    map_comparator.cpp \
    map_generator.cpp \
    map_height_map.cpp \
    map_path_finder.cpp \
    map_path.cpp \
    map.cpp \
    obstacle_funnel_algorithm.cpp \
    obstacle_map.cpp \
    obstacle_path_finder.cpp \
    opencl_interface.cpp \
    robot_data.cpp \
    test.cpp \
    robot_navigation.cpp \
    policy_generator.cpp \
    policy_data.cpp \
    obstacle_district.cpp \
    occupancy_grid_map.cpp \
    obstacle_district_manager.cpp \
    obstacle_connection.cpp \
    obstacle_connection_manager.cpp \
    obstacle_district_map.cpp \
    tree_class.cpp \
    tree_node.cpp \
    monte_carlo_search.cpp

HEADERS += \
    algorithm_optimal_single_bot.h \
    graph_class.h \
    graph_edge.h \
    graph_vertice.h \
    map_comparator.h \
    map_generator.h \
    map_height_map.h \
    map_path_finder.h \
    map_path.h \
    map_standards.h \
    map.h \
    obstacle_funnel_algorithm.h \
    obstacle_map.h \
    obstacle_path_finder.h \
    opencl_interface.h \
    robot_data.h \
    todo.h \
    robot_navigation.h \
    policy_generator.h \
    policy_data.h \
    obstacle_district.h \
    occupancy_grid_map.h \
    obstacle_district_manager.h \
    obstacle_connection.h \
    obstacle_connection_manager.h \
    obstacle_district_map.h \
    tree_class.h \
    tree_node.h \
    monte_carlo_search.h

unix:!macx: LIBS += -L/lib/beignet/ -lcl

INCLUDEPATH += /lib/beignet/include,/opt/intel/opencl-sdk/include
DEPENDPATH += /lib/beignet/include,/opt/intel/opencl-sdk/include
