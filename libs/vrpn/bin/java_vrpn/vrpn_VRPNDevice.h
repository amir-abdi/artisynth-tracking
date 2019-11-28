/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class vrpn_VRPNDevice */

#ifndef _Included_vrpn_VRPNDevice
#define _Included_vrpn_VRPNDevice
#ifdef __cplusplus
extern "C" {
#endif
#undef vrpn_VRPNDevice_vrpn_TEXT_NORMAL
#define vrpn_VRPNDevice_vrpn_TEXT_NORMAL 0L
#undef vrpn_VRPNDevice_vrpn_TEXT_WARNING
#define vrpn_VRPNDevice_vrpn_TEXT_WARNING 1L
#undef vrpn_VRPNDevice_vrpn_TEXT_ERROR
#define vrpn_VRPNDevice_vrpn_TEXT_ERROR 2L
/*
 * Class:     vrpn_VRPNDevice
 * Method:    isLive_native
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_isLive_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    doingOkay_native
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_doingOkay_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    isConnected_native
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_isConnected_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    getElapsedTimeSecs_native
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_vrpn_VRPNDevice_getElapsedTimeSecs_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    setReplayRate_native
 * Signature: (F)V
 */
JNIEXPORT void JNICALL Java_vrpn_VRPNDevice_setReplayRate_1native
  (JNIEnv *, jobject, jfloat);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    getReplayRate_native
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_vrpn_VRPNDevice_getReplayRate_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    reset_native
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_reset_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    eof_native
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_eof_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    playToElapsedTime_native
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_playToElapsedTime_1native
  (JNIEnv *, jobject, jlong);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    playToWallTime_native
 * Signature: (Ljava/util/Date;)Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_playToWallTime_1native
  (JNIEnv *, jobject, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    getLengthSecs_native
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_vrpn_VRPNDevice_getLengthSecs_1native
  (JNIEnv *, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    getEarliestTime_native
 * Signature: (Ljava/util/Date;)Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_getEarliestTime_1native
  (JNIEnv *, jobject, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    getLatestTime_native
 * Signature: (Ljava/util/Date;)Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_getLatestTime_1native
  (JNIEnv *, jobject, jobject);

/*
 * Class:     vrpn_VRPNDevice
 * Method:    getTime_native
 * Signature: (Ljava/util/Date;)Z
 */
JNIEXPORT jboolean JNICALL Java_vrpn_VRPNDevice_getTime_1native
  (JNIEnv *, jobject, jobject);

#ifdef __cplusplus
}
#endif
#endif
