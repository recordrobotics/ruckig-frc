#include <jni.h>
#include <vector>
#include <memory>
#include <ruckig/ruckig.hpp>

// Helper to cast jlong to pointer
#define PTR_FROM_JLONG(type, val) reinterpret_cast<type *>(val)
#define JLONG_FROM_PTR(ptr) reinterpret_cast<jlong>(ptr)

#define RuckigJNI(method) Java_org_recordrobotics_ruckig_jni_RuckigJNI_##method

extern "C"
{

#pragma region Ruckig3
    JNIEXPORT jlong JNICALL RuckigJNI(create3)(JNIEnv *, jobject, double delta_time)
    {
        return JLONG_FROM_PTR(new ruckig::Ruckig<3>(delta_time));
    }

    JNIEXPORT void JNICALL RuckigJNI(destroy3)(JNIEnv *, jobject, jlong handle)
    {
        delete PTR_FROM_JLONG(ruckig::Ruckig<3>, handle);
    }

    JNIEXPORT jint JNICALL RuckigJNI(update)(JNIEnv *, jobject, jlong handle, jlong inputHandle, jlong outputHandle)
    {
        auto *ruckig = PTR_FROM_JLONG(ruckig::Ruckig<3>, handle);
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, inputHandle);
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, outputHandle);
        return static_cast<jint>(ruckig->update(*input, *output));
    }

    JNIEXPORT void JNICALL RuckigJNI(reset)(JNIEnv *, jobject, jlong handle)
    {
        auto *ruckig = PTR_FROM_JLONG(ruckig::Ruckig<3>, handle);
        ruckig->reset();
    }

    JNIEXPORT jdouble JNICALL RuckigJNI(getDeltaTime)(JNIEnv *, jobject, jlong handle)
    {
        auto *ruckig = PTR_FROM_JLONG(ruckig::Ruckig<3>, handle);
        return ruckig->delta_time;
    }

    JNIEXPORT void JNICALL RuckigJNI(setDeltaTime)(JNIEnv *, jobject, jlong handle, double delta_time)
    {
        auto *ruckig = PTR_FROM_JLONG(ruckig::Ruckig<3>, handle);
        ruckig->delta_time = delta_time;
    }
#pragma endregion

#pragma region InputParameter3
    JNIEXPORT jlong JNICALL RuckigJNI(createInput3)(JNIEnv *, jobject)
    {
        auto *input = new ruckig::InputParameter<3>();
        for (size_t dof = 0; dof < input->degrees_of_freedom; ++dof)
        {
            input->current_position[dof] = 0.0;
            input->target_position[dof] = 0.0;
        }
        return JLONG_FROM_PTR(input);
    }

    JNIEXPORT void JNICALL RuckigJNI(destroyInput3)(JNIEnv *, jobject, jlong handle)
    {
        delete PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
    }

    JNIEXPORT jint JNICALL RuckigJNI(getDurationDiscretization)(JNIEnv *, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        return static_cast<jint>(input->duration_discretization);
    }

    JNIEXPORT void JNICALL RuckigJNI(setDurationDiscretization)(JNIEnv *, jobject, jlong handle, int discretizationCode)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        input->duration_discretization = static_cast<ruckig::DurationDiscretization>(discretizationCode);
    }

    JNIEXPORT jint JNICALL RuckigJNI(getDefaultSynchronization)(JNIEnv *, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        return static_cast<jint>(input->synchronization);
    }

    JNIEXPORT void JNICALL RuckigJNI(setDefaultSynchronization)(JNIEnv *, jobject, jlong handle, int synchronizationCode)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        input->synchronization = static_cast<ruckig::Synchronization>(synchronizationCode);
    }

    JNIEXPORT jintArray JNICALL RuckigJNI(getPerDoFSynchronization)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jintArray codes = env->NewIntArray(3);
        if (input->per_dof_synchronization.has_value())
        {
            jint *elements = env->GetIntArrayElements(codes, nullptr);
            for (size_t i = 0; i < 3; ++i)
            {
                elements[i] = static_cast<jint>(input->per_dof_synchronization->at(i));
            }
            env->ReleaseIntArrayElements(codes, elements, 0);
        }
        else
        {
            jint *elements = env->GetIntArrayElements(codes, nullptr);
            for (size_t i = 0; i < 3; ++i)
            {
                elements[i] = static_cast<jint>(input->synchronization);
            }
            env->ReleaseIntArrayElements(codes, elements, 0);
        }
        return codes;
    }

    JNIEXPORT void JNICALL RuckigJNI(setPerDoFSynchronization)(JNIEnv *env, jobject, jlong handle, jintArray codes)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jint codesArr[3];
        env->GetIntArrayRegion(codes, 0, 3, codesArr);
        std::array<ruckig::Synchronization, 3> perDofSync;
        for (size_t i = 0; i < 3; ++i)
        {
            perDofSync[i] = static_cast<ruckig::Synchronization>(codesArr[i]);
        }
        input->per_dof_synchronization = perDofSync;
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getMaxVelocity)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->max_velocity.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setMaxVelocity)(JNIEnv *env, jobject, jlong handle, jdoubleArray maxVelocity)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(maxVelocity, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->max_velocity[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getMaxAcceleration)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->max_acceleration.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setMaxAcceleration)(JNIEnv *env, jobject, jlong handle, jdoubleArray maxAcceleration)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(maxAcceleration, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->max_acceleration[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getMaxJerk)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->max_jerk.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setMaxJerk)(JNIEnv *env, jobject, jlong handle, jdoubleArray maxJerk)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(maxJerk, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->max_jerk[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getCurrentPosition)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->current_position.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setCurrentPosition)(JNIEnv *env, jobject, jlong handle, jdoubleArray currentPosition)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(currentPosition, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->current_position[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getCurrentVelocity)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->current_velocity.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setCurrentVelocity)(JNIEnv *env, jobject, jlong handle, jdoubleArray currentVelocity)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(currentVelocity, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->current_velocity[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getCurrentAcceleration)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->current_acceleration.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setCurrentAcceleration)(JNIEnv *env, jobject, jlong handle, jdoubleArray currentAcceleration)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(currentAcceleration, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->current_acceleration[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getTargetPosition)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->target_position.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setTargetPosition)(JNIEnv *env, jobject, jlong handle, jdoubleArray targetPosition)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(targetPosition, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->target_position[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getTargetVelocity)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->target_velocity.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setTargetVelocity)(JNIEnv *env, jobject, jlong handle, jdoubleArray targetVelocity)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(targetVelocity, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->target_velocity[i] = values[i];
        }
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getTargetAcceleration)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, input->target_acceleration.data());
        return arr;
    }

    JNIEXPORT void JNICALL RuckigJNI(setTargetAcceleration)(JNIEnv *env, jobject, jlong handle, jdoubleArray targetAcceleration)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        jdouble values[3];
        env->GetDoubleArrayRegion(targetAcceleration, 0, 3, values);
        for (size_t i = 0; i < 3; ++i)
        {
            input->target_acceleration[i] = values[i];
        }
    }

    JNIEXPORT jdouble JNICALL RuckigJNI(getMinimumDuration)(JNIEnv *, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        if (input->minimum_duration.has_value())
            return static_cast<jdouble>(input->minimum_duration.value());
        else
            return std::numeric_limits<jdouble>::lowest();
    }

    JNIEXPORT void JNICALL RuckigJNI(setMinimumDuration)(JNIEnv *, jobject, jlong handle, jdouble minimumDuration)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        if (minimumDuration != std::numeric_limits<jdouble>::lowest())
        {
            input->minimum_duration = minimumDuration;
        }
        else
        {
            input->minimum_duration.reset();
        }
    }

    JNIEXPORT jboolean JNICALL RuckigJNI(validate)(JNIEnv *, jobject, jlong handle, jboolean check_current_state_within_limits,
                                                   jboolean check_target_state_within_limits)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        return static_cast<jboolean>(input->validate(check_current_state_within_limits, check_target_state_within_limits));
    }

    JNIEXPORT jstring JNICALL RuckigJNI(toStringInput3)(JNIEnv *env, jobject, jlong handle)
    {
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, handle);
        std::string str = input->to_string();
        return env->NewStringUTF(str.c_str());
    }
#pragma endregion

#pragma region OutputParameter3
    JNIEXPORT jlong JNICALL RuckigJNI(createOutput3)(JNIEnv *, jobject)
    {
        return JLONG_FROM_PTR(new ruckig::OutputParameter<3>());
    }

    JNIEXPORT void JNICALL RuckigJNI(destroyOutput3)(JNIEnv *, jobject, jlong handle)
    {
        delete PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getNewPosition)(JNIEnv *env, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, output->new_position.data());
        return arr;
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getNewVelocity)(JNIEnv *env, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, output->new_velocity.data());
        return arr;
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getNewAcceleration)(JNIEnv *env, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, output->new_acceleration.data());
        return arr;
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(getNewJerk)(JNIEnv *env, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(3);
        env->SetDoubleArrayRegion(arr, 0, 3, output->new_jerk.data());
        return arr;
    }

    JNIEXPORT jdouble JNICALL RuckigJNI(getTime)(JNIEnv *, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        return static_cast<jdouble>(output->time);
    }

    JNIEXPORT jboolean JNICALL RuckigJNI(isNewCalculation)(JNIEnv *, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        return static_cast<jboolean>(output->new_calculation);
    }

    JNIEXPORT jdouble JNICALL RuckigJNI(getCalculationDuration)(JNIEnv *, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        return static_cast<jdouble>(output->calculation_duration);
    }

    JNIEXPORT void JNICALL RuckigJNI(passOutputToInput)(JNIEnv *, jobject, jlong outputHandle, jlong inputHandle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, outputHandle);
        auto *input = PTR_FROM_JLONG(ruckig::InputParameter<3>, inputHandle);
        output->pass_to_input(*input);
    }

    JNIEXPORT jstring JNICALL RuckigJNI(toStringOutput3)(JNIEnv *env, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        std::string str = output->to_string();
        return env->NewStringUTF(str.c_str());
    }
#pragma endregion

#pragma region Trajectory3
    JNIEXPORT jdoubleArray JNICALL RuckigJNI(trajectoryAtTime)(JNIEnv *env, jobject, jlong handle, jdouble time)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(9);
        std::array<double, 3> position, velocity, acceleration;
        output->trajectory.at_time(static_cast<double>(time), position, velocity, acceleration);
        env->SetDoubleArrayRegion(arr, 0, 3, position.data());
        env->SetDoubleArrayRegion(arr, 3, 3, velocity.data());
        env->SetDoubleArrayRegion(arr, 6, 3, acceleration.data());
        return arr;
    }

    JNIEXPORT jdouble JNICALL RuckigJNI(trajectoryDuration)(JNIEnv *, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        return static_cast<jdouble>(output->trajectory.get_duration());
    }

    JNIEXPORT jdoubleArray JNICALL RuckigJNI(trajectoryPositionExtrema)(JNIEnv *env, jobject, jlong handle)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        jdoubleArray arr = env->NewDoubleArray(12);
        auto extrema = output->trajectory.get_position_extrema();
        for (size_t i = 0; i < 3; ++i)
        {
            env->SetDoubleArrayRegion(arr, i * 4, 1, &extrema[i].min);
            env->SetDoubleArrayRegion(arr, i * 4 + 1, 1, &extrema[i].max);
            env->SetDoubleArrayRegion(arr, i * 4 + 2, 1, &extrema[i].t_min);
            env->SetDoubleArrayRegion(arr, i * 4 + 3, 1, &extrema[i].t_max);
        }
        return arr;
    }

    JNIEXPORT jdouble JNICALL RuckigJNI(trajectoryFirstTimeAtPosition)(JNIEnv *, jobject, jlong handle, jint dof, jdouble position, jdouble time_after)
    {
        auto *output = PTR_FROM_JLONG(ruckig::OutputParameter<3>, handle);
        auto time = output->trajectory.get_first_time_at_position(static_cast<size_t>(dof), static_cast<double>(position), static_cast<double>(time_after));
        if (time.has_value())
        {
            return static_cast<jdouble>(time.value());
        }
        else
        {
            return std::numeric_limits<jdouble>::lowest();
        }
    }
#pragma endregion

} // extern "C"
