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
    obstacle_connections.cpp \
    obstacle_data_single.cpp \
    obstacle_funnel_algorithm.cpp \
    obstacle_map.cpp \
    obstacle_path_finder.cpp \
    opencl_interface.cpp \
    robot_data.cpp \
    test.cpp \
    robot_navigation.cpp \
    obstacle_districts.cpp \
    policy_generator.cpp

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
    obstacle_connections.h \
    obstacle_data_single.h \
    obstacle_funnel_algorithm.h \
    obstacle_map.h \
    obstacle_path_finder.h \
    opencl_interface.h \
    robot_data.h \
    todo.h \
    robot_navigation.h \
    obstacle_districts.h \
    policy_generator.h

unix:!macx: LIBS += -L/lib/beignet/ -lcl

INCLUDEPATH += /lib/beignet/include,/opt/intel/opencl-sdk/include
DEPENDPATH += /lib/beignet/include,/opt/intel/opencl-sdk/include
