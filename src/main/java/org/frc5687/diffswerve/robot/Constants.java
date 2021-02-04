/* (C)2020-2021 */
package org.frc5687.diffswerve.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.*;

public class Constants {
    public static final int TICKS_PER_UPDATE = 1;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static final double EPSILON = 0.001;

    public static class DriveTrain {

        public static final Transform2d SLAM_TO_ROBOT =
                new Transform2d(
                        new Translation2d(0, 0),
                        new Rotation2d(0)); // TODO: Figure out real values.

        public static final double T265_MEASUREMENT_COVARIANCE = 0.5;

        public static final double WIDTH = 0.6223;
        public static final double LENGTH = 0.6223;
        public static final Translation2d FRONT_LEFT_POSITION =
                new Translation2d(WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d FRONT_RIGHT_POSITION =
                new Translation2d(WIDTH / 2.0, -LENGTH / 2.0);
        public static final Translation2d BACK_LEFT_POSITION =
                new Translation2d(-WIDTH / 2.0, LENGTH / 2.0);
        public static final Translation2d BACK_RIGHT_POSITION =
                new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0);
        public static final boolean RIGHT_INVERTED = false;
        public static final boolean LEFT_INVERTED = false;

        // units are meters and radians.
        // System pose [x, y, theta] certainty TODO: Tune for real bot.
        public static final Matrix<N3, N1> STATE_STD_DEVS =
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        // Heading certainty TODO: Tune for real bot.
        public static final Matrix<N1, N1> LOCAL_MEASUREMENT_STD_DEVS =
                VecBuilder.fill(Units.degreesToRadians(0.01));
        // Vision pose [x, y, theta] certainty TODO: Tune for real bot.
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(20));

        public static final double DEADBAND = 0.08;
        public static final double MAX_MPS = 5.1816;
        //        public static final double MAX_MPS = 1.0;
        public static final double MAX_ANG_VEL = Math.PI * 2.0;
        public static final double MAX_MPSS = 1.7; // accel

        public static final double kP = 2.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double PROFILE_CONSTRAINT_VEL = 2.0 * Math.PI;
        public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI;
    }

    public static class DifferentialSwerveModule {

        public static final double kDt = 0.005;
        public static final int TIMEOUT = 200;
        public static final double GEAR_RATIO_WHEEL = 6.46875;
        public static final double GEAR_RATIO_STEER = 11.5;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.0508; // Meters
        public static final double MAX_MPS = 5.1816;
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTS_TO_ROTATIONS = 3.3;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.007;
        public static final double INERTIA_STEER = 0.007;
        public static final double Q_AZIMUTH_ANG_VELOCITY = 0.2; // radians per sec
        public static final double Q_AZIMUTH = 0.02; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 3; // radians per sec
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = 1.718873; // degrees
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 400.0; // RPM
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 400.0; // RPM
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.02; // degrees
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 114.592; // degrees
        public static final double CONTROL_EFFORT = 12.0;
    }
}
