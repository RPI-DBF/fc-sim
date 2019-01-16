using System;

class FC
{
    class firFilter
    {
        private float buf[];
        private float coeffs[];

        public firFilter(float buf[], const float coeffs[])
        {
            this.buf = buf;
            this.coeffs = coeffs;
        }

        public void firFilterUpdate(float input)
        {
            for (int i = this.buf.length - 1; i > 0; i--)
            {
                buf[i] = buf[i-1];
            }
            buf[0] = input;
        }

        public float firFilterApply()
        {
            float ret = 0.0f;
            for (int i = 0; i < this.buf.length; i++)
            {
                ret += this.coeffs[i] * this.buf[i];
            }
            return ret;
        }
    }

    class pt1Filter
    {
        private float state;
        private float RC = 0.0f;
        private float dT;

        void pt1FilterApply4(float input, int f_cut, float dT)
        {
            if (!this.RC)
            {
                this.RC = 1.0f / (2.0f * Math.PI *f_cut);
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

    const int FD_ROLL = 0;
    const int FD_PITCH = 1;
    const int FD_YAW = 2;

    const int RATE_MODE = 0;
    const int ANGLE_MODE = 1;
    const int RATE_ANGLE_MODE = 2;

    const int MAX_ANGLE = 45;

    const int stab_roll_rate = 90;
    const int stab_pitch_rate = 90;
    const int stab_yaw_rate = 90;

    const int[] control_rates = {90, 90, 90}
    int[] attitude = {0, 0, 0}

    int holding_angle = 0;

    class PIDState
    {
        float kP;
        float kI;
        float kD;
        float kFF;
        float kT;

        float gyroRate;
        float rateTarget;

        const int PID_GYRO_RATE_BUF_LENGTH = 5;
        float gyroRateBuf[PID_GYRO_RATE_BUF_LENGTH];
        firFilter fytoRateFilter;

        float errorGyroIf;
        float errorGyroIfLimit; //Set manually

        pt1Filter angleFilterState;

        rateLimitFilter axisAccelFilter;
        pt1Filter ptermLpfState;

        float output;
    }

    PIDState pidState[4];


    static void Main(string[] args)
    {
        Console.WriteLine("Hello World");
    }

    void pidController(float gyroRollRate, float gyroPitchRate, float gyroYawRate, float roll_stick, float pitch_stick, float yaw_stick, int flight_mode, float dT)
    {
        pidState[0].gyroRate = gyroRollRate;
        pidState[1].gyroRate = gyroPitchRate;
        pidState[2].gyroRate = gyroYawRate;

        pidState[0].rateTarget = pidRcCommandToRate(roll_stick, stab_roll_rate);
        pidState[1].rateTarget = pidRcCommandToRate(pitch_stick, stab_pitch_rate);
        pidState[2].rateTarget = pidRcCommandToRate(yaw_stick, stab_yaw_rate);

        for (int axis = 0; axis < 3; axis++)
        {
            pidState[axis].rateTarget = constrainf(pidState[axis], -1800, 1800);
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
        float angleTarget = pidRcCommandToAngle(stick, MAX_ANGLE);
        float rateTarget = pidRcCommandToRate(stick, stab_rate);

        float angleRateTarget = 0;
        if (mode == ANGLE_MODE)
        {
            holding_angle = 0;
        }
        if (flight_mode == HORIZON_MODE && axis == FD_ROLL)
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
            else if (holding_angle == -1 && stick < 0) {
                holding_angle = 0;
            }

            if (holding_angle != 0) {
                angleTarget = MAX_ANGLE * holding_angle;
            }
        }

        float angleErrorDeg;
        if (flight_mode == HORIZON_MODE && axis == FD_ROLL) {
            angleErrorDeg = attitude[FD_ROLL] - angleTarget;
        } else {
            angleErrorDeg = angleTarget - attitude[FD_ROLL];
        }

        if (flight_mode == ANGLE_MODE || holding_angle0 {
                angleRateTarget = constrainf(angleErrorDeg * PID_LEVEL_P, -stab_rate, stab_rate);
                if (PID_LEVEL_I) {
                    angleRateTarget = pidState.angleFilterState.pt1FilterApply4(angleRateTarget, PID_LEVEL_I, dT);

                }

        }
        else {
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

        pidSumLimit = 0; //Set manually
        pidState.output = constrainf(newPTerm + newFFTerm + pidState->errorGyroIf, -pidSumLimit, pidSumLimit);
    }

	void mixer(float pitch, float roll)
	{
	}
	
    private float pidRcCommandToRate(int axis, int rate)
    {
        const float maxRateDPS = rate * 10.0f;
        return scaleRangef((float) stick, -500.0f, 500.0f, -maxRateDPS, maxRateDPS);
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

    private int constrain(int amt, int low, int high)
    {
        if (amt < low)
            return low;
        else if (amt > high)
            return high;
        else
            return amt;
    }

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
