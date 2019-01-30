using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class flightController : MonoBehaviour
{

    void Start()
    {

    }

    void Update()
    {

    }

    class firFilter
    {
        private float[] buf;
        private float[] coeffs;

        public firFilter(float[] buf, float[] coeffs)
        {
            this.buf = buf;
            this.coeffs = coeffs;
        }

        public void firFilterUpdate(float input)
        {
            for (int i = this.buf.Length - 1; i > 0; i--)
            {
                buf[i] = buf[i - 1];
            }
            buf[0] = input;
        }

        public float firFilterApply()
        {
            float ret = 0.0f;
            for (int i = 0; i < this.buf.Length; i++)
            {
                ret += this.coeffs[i] * this.buf[i];
            }
            return ret;
        }


        public class pt1Filter
        {
            public float state;
            public float RC = 0.0f;
            public float dT;

            public float pt1FilterApply4(float input, int f_cut, float dT)
            {
                if (this.RC == 0.0f)
                {
                    this.RC = (float)(1.0f / (2.0f * Math.PI * f_cut));
                }
                this.dT = dT;
                this.state = this.state + this.dT / (this.RC + this.dT) * (input - this.state);
                return this.state;
            }
        }

        /*
        class rateLimitFilter
        {
        }
        */

        /*
        class ptermLpfState
        {
        }
        */

        /*

        COPY PASTA'D FROM SETUP CONFIG FILE FOR CONFIGURATOR:

        # mixer
        mmix 0  1.000 -1.000  1.000 -1.000
        mmix 1  1.000 -1.000 -1.000  1.000
        mmix 2  1.000  1.000  1.000  1.000
        mmix 3  1.000  1.000 -1.000 -1.000
         */

        const float THROTTLE_CLIPPING_FACTOR = 0.33f;

        const int FD_ROLL = 0;
        const int FD_PITCH = 1;
        const int FD_YAW = 2;

        const int RATE_MODE = 0;
        const int ANGLE_MODE = 1;
        const int HORIZON_MODE = 2;

        const int MAX_ANGLE = 45;

        //Get PID LVEL 
        const int PID_LEVEL_P = 0;
        const int PID_LEVEL_I = 0;

        const int stab_roll_rate = 90;
        const int stab_pitch_rate = 90;
        const int stab_yaw_rate = 90;

        int[] control_rates = { 90, 90, 90 };
        int[] attitude = { 0, 0, 0 };

        int holding_angle = 0;

        // custom constrain function for c#
        public static int constrain(int value, int min, int max)
        {
            return (value < min) ? min : (value > max) ? max : value;
        }

        static float motorMixRange = 0.0f;

        public class PIDState
        {
            public float kP;
            public float kI;
            public float kD;
            public float kFF;
            public float kT;

            public float gyroRate;
            public float rateTarget;

            public const int PID_GYRO_RATE_BUF_LENGTH = 5;
            public float[] gyroRateBuf = new float[PID_GYRO_RATE_BUF_LENGTH];
            public firFilter fytoRateFilter;

            public float errorGyroIf;
            public float errorGyroIfLimit; //Set manually

            public pt1Filter angleFilterState;

            //rateLimitFilter axisAccelFilter;
            public pt1Filter ptermLpfState;

            public float output;
        }

        PIDState[] pidState = new PIDState[3];



        void pidController(float gyroRollRate, float gyroPitchRate, float gyroYawRate, float roll_stick, float pitch_stick, float yaw_stick, int flight_mode, float dT)
        {
            pidState[0].gyroRate = gyroRollRate;
            pidState[1].gyroRate = gyroPitchRate;
            pidState[2].gyroRate = gyroYawRate;

            pidState[0].rateTarget = pidRcCommandToRate((int)(roll_stick*500), stab_roll_rate);
            pidState[1].rateTarget = pidRcCommandToRate((int)(pitch_stick*500), stab_pitch_rate);
            pidState[2].rateTarget = pidRcCommandToRate((int)(yaw_stick*500), stab_yaw_rate);

            for (int axis = 0; axis < 3; axis++)
            {
                pidState[axis].rateTarget = constrainf(pidState[axis].rateTarget, -1800, 1800);
            }

            if (flight_mode == ANGLE_MODE)
            {
                pidLevel(pidState[FD_ROLL], FD_ROLL, roll_stick, stab_roll_rate, flight_mode);
                pidLevel(pidState[FD_PITCH], FD_PITCH, pitch_stick, stab_pitch_rate, flight_mode);
            }
            else if (flight_mode == HORIZON_MODE)
            {
                pidLevel(pidState[FD_ROLL], FD_ROLL, roll_stick, stab_roll_rate, flight_mode);
            }

            for (int axis = 0; axis < 3; axis++)
            {
                pidApplySetpointRateLimiting(pidState[axis], axis);
            }
            for (int axis = 0; axis < 3; axis++)
            {
                pidApplyFixedWingRateController(pidState[axis], axis, dT);
            }
        }

        private void pidLevel(PIDState pidState, int axis, float stick, float stab_rate, int mode)
        {
            float angleTarget = pidRcCommandToAngle((int)(stick*500), MAX_ANGLE);
            float rateTarget = pidRcCommandToRate((int)(stick*500), (int)stab_rate);

            float angleRateTarget = 0;
            if (mode == ANGLE_MODE)
            {
                holding_angle = 0;
            }
            if (mode == HORIZON_MODE && axis == FD_ROLL)
            {
                if (attitude[FD_ROLL] > MAX_ANGLE)
                {
                    holding_angle = 1;
                }
                else if (attitude[FD_ROLL] < -MAX_ANGLE)
                {
                    holding_angle = 1;
                }

                if (holding_angle == 1 && stick > 0)
                {
                    holding_angle = 0;
                }
                else if (holding_angle == -1 && stick < 0)
                {
                    holding_angle = 0;
                }

                if (holding_angle != 0)
                {
                    angleTarget = MAX_ANGLE * holding_angle;
                }
            }

            float angleErrorDeg;
            if (mode == HORIZON_MODE && axis == FD_ROLL)
            {
                angleErrorDeg = attitude[FD_ROLL] - angleTarget;
            }
            else
            {
                angleErrorDeg = angleTarget - attitude[FD_ROLL];
            }

            if (mode == ANGLE_MODE || holding_angle != 0) {
                angleRateTarget = constrainf(angleErrorDeg * PID_LEVEL_P, -stab_rate, stab_rate);
                if (PID_LEVEL_I != 0)
                {
                    angleRateTarget = pidState.angleFilterState.pt1FilterApply4(angleRateTarget, PID_LEVEL_I, dT);

                }

            }
            else
            {
                angleRateTarget = rateTarget;
            }
            pidState.rateTarget = angleRateTarget;
        }

        private void pidApplySetpointRateLimiting(PIDState pidState, int axis)
        {
        }

        private void pidApplyFixedWingRateController(PIDState pidState, int axis, float dT)
        {
            float rateError = pidState.rateTarget - pidState.gyroRate;

            float newPTerm = rateError * pidState.kP;
            float newFFTerm = pidState.rateTarget * pidState.kFF;

            pidState.errorGyroIf += rateError * pidState.kI * dT;

            pidState.errorGyroIf = constrainf(pidState.errorGyroIf, -pidState.errorGyroIfLimit, pidState.errorGyroIfLimit);

            int pidSumLimit = 0; //Set manually
            pidState.output = constrainf(newPTerm + newFFTerm + pidState.errorGyroIf, -pidSumLimit, pidSumLimit);
        }


        // todo: get range of input
        private void mixer(float pitch, float roll, float max_output, float max_input)
        {
            int MAX_SUPPORTED_MOTORS = 1;
            int[] rpyMix = new int[MAX_SUPPORTED_MOTORS];
            int max_rpyMix = 0;
            int min_rpyMix = 0;


            for (int i = 0; i < 2; i++)
            {
                rpyMix[i] = pitch * currentMixer[i].pitch + roll * currentMixer[i].roll;
                if (rpyMix[i] > max_rpyMix) max_rpyMix = rpyMix[i];
                if (rpyMix[i] < min_rpyMix) min_rpyMix = rpyMix[i];
            }

            int rpyMixRange = max_rpyMix - min_rpyMix;
            int throttleRange, throttleCommand, throttleMin, throttleMax;
            int throttlePrevious = 0;

            throttleCommand = 0; // todo: fix this, the C code takes in rcCommand[THROTTLE] here
            throttleMin = 0;     // this variable is assigned to motorConfig()->minthrottle
            throttleMax = 0;     // this variable is assigned to motorConfig()->maxthrottle

            throttleRange = throttleMax - throttleMin;

            motorMixRange = (float)rpyMixRange / (float)throttleRange;

            if (motorMixRange > 1.0)
            {
                for (int i = 0; i < 2; i++)
                {
                    rpyMix[i] /= (int)motorMixRange;
                }
                throttleMin = (int)(throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
                throttleMax = (int)(throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
            }
            else
            {
                throttleMin = (int)Math.Min(throttleMin + (rpyMixRange / 2), throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
                throttleMax = (int)Math.Max(throttleMax - (rpyMixRange / 2), throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
            }

            for (int i = 0; i < 2; i++)
            {
                motor[i] = rpyMix[i] + constrain(throttleCommand * currentMixer[i].throttle, throttleMin, throttleMax);
                // next part of code starts with "if(failsafeIsActive()"
            }

        }

        private float pidRcCommandToRate(int axis, int rate)
        {
            float maxRateDPS = rate * 10.0f;
            return scaleRangef((float)axis, -500.0f, 500.0f, -maxRateDPS, maxRateDPS);
        }

        private float pidRcCommandToAngle(int stick, int maxInclination)
        {
            stick = constrain(stick, -500, 500);
            return scaleRangef((float)stick, -500.0f, 500.0f, (float)-maxInclination, (float)maxInclination);
        }

        private float scaleRangef(float x, float srcMin, float srcMax, float destMin, float destMax)
        {
            float a = (destMax - destMin) * (x - srcMin);
            float b = srcMax - srcMin;
            return ((a / b) + destMin);
        }

        /*
        private int constrain(int amt, int low, int high)
        {
            if (amt < low)
                return low;
            else if (amt > high)
                return high;
            else
                return amt;
        }
        */

        private float constrainf(float amt, float low, float high)
        {
            if (amt < low)
                return low;
            else if (amt > high)
                return high;
            else
                return amt;
        }
    }
