// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class Interpolation {
    private static InterpolatingDoubleTreeMap rpm;
    private static InterpolatingDoubleTreeMap angle;

    // {distance, angle, rpm}
    private static double[][] samples = new double[][] {
        // florida values
        // {0.651, 20, 2500},
        // {0.761, 22, 2550},
        // {0.904, 24, 2600},
        // {1, 26, 2650},
        // {1.25, 30, 2750},
        // {1.5, 32.5, 2850},
        // {1.75, 33.5, 2950},
        // {2, 37, 3200},
        // {2.25, 39, 3200},
        // {2.5, 40.5, 3300},
        // {2.75, 41.5, 3450},
        // {3, 42, 3500},
        // {3.25, 43.5, 3600},
        // {3.5, 43, 3900},
        // {3.81, 43, 3900},
        // {4, 44, 3900}

        // bad values!
        {0.69, 15, 2500},
        {1.0, 20, 2600},
        {1.25, 24, 2800},
        {1.5, 28, 2900},
        {1.75, 30, 3000},
        {2.0, 33, 3200},
        {2.25, 35, 3350},
        {2.5, 37, 3400},
        {2.75, 39, 3500},
        {3.0, 42, 3500},
        {3.5, 42.5, 3500},
        {4.0, 43, 3600}
    };

    static {
        rpm = new InterpolatingDoubleTreeMap();
        angle = new InterpolatingDoubleTreeMap();

        for(int i = 0; i < samples.length; i++) {
            rpm.put(samples[i][0], samples[i][2]);
            angle.put(samples[i][0], samples[i][1]);
        }
    }

    public static double getRPM(double dist) {
        return rpm.get(dist);
    }

    public static double getAngle(double dist) {
        return angle.get(dist);
    }
}
