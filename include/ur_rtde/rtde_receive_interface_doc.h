/*
  This file contains docstrings for the Python bindings.
  Do not edit! These were automatically extracted by mkdoc.py
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif


static const char *__doc_ur_rtde_RTDEReceiveInterface = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_RTDEReceiveInterface = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualCurrent = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualDigitalInputBits = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualDigitalOutputBits = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualExecutionTime = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualJointVoltage = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualMainVoltage = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualMomentum = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualQ = R"doc(@returns Actual joint positions)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualQd = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualRobotCurrent = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualRobotVoltage = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualTCPForce = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualTCPPose = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualTCPSpeed = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getActualToolAccelerometer = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getJointControlOutput = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getJointMode = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getJointTemperatures = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getRobotMode = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getRobotStatus = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getRuntimeState = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getSafetyMode = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getSpeedScaling = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogInput0 = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogInput1 = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogOutput0 = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getStandardAnalogOutput1 = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetCurrent = R"doc(@returns Target joint currents)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetMoment = R"doc(@returns Target joint moments (torques))doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetQ = R"doc(@returns Target joint positions)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetQd = R"doc(@returns Target joint velocities)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetQdd = R"doc(@returns Target joint accelerations)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetSpeedFraction = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetTCPPose = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTargetTCPSpeed = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_getTimestamp = R"doc(@returns Time elapsed since the controller was started [s])doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_hostname = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_isConnected =
R"doc(@returns Connection status for RTDE, useful for checking for lost
connection.)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_port = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_receiveCallback = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_reconnect =
R"doc(@returns Can be used to reconnect to the robot after a lost
connection.)doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_robot_state = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_rtde = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_stop_thread = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_th = R"doc()doc";

static const char *__doc_ur_rtde_RTDEReceiveInterface_variables = R"doc()doc";

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

