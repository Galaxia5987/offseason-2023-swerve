package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.SwerveModuleConfigBase;
import frc.robot.valuetuner.WebConstant;

import java.util.HashMap;

import static frc.robot.Ports.SwerveDrive.*;


public final class Constants {
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].

    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final double NOMINAL_VOLTAGE = 10; // [volts]
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]
    public static final double SIMULATION_LOOP_PERIOD = 0.02; // [s]

    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    // The order of modules is ALWAYS front-right (fr), front-left (fl), rear-right (rr), rear-left (rl)
    public static final class SwerveDrive {
        public static final double VELOCITY_MULTIPLIER = 4;
        public static final double ROTATION_MULTIPLIER = 4;

        public static final int TICKS_PER_ROTATION_DRIVE_MOTOR = 2048;
        public static final int TICKS_PER_ROTATION_ANGLE_MOTOR = 1024;
        public static final double GEAR_RATIO_DRIVE_MOTOR = 7.5;
        public static final double GEAR_RATIO_ANGLE_MOTOR = 1;
        public static final double DRIVE_MOTOR_TICKS_PER_METER = GEAR_RATIO_DRIVE_MOTOR * TICKS_PER_ROTATION_DRIVE_MOTOR / (0.11 * Math.PI); // 4 * 0.0254
        public static final double ANGLE_MOTOR_TICKS_PER_RADIAN = GEAR_RATIO_ANGLE_MOTOR * TICKS_PER_ROTATION_ANGLE_MOTOR / (2 * Math.PI);

        public static final int MAX_CURRENT = 15; // [amps]

        // State Space
        public static final double VELOCITY_TOLERANCE = 5; // [rps]
        public static final double COST_LQR = 11;
        // Note that the values of MODEL_TOLERANCE and ENCODER_TOLERANCE should be a lot smaller (something like 1e-6)
        public static final double MODEL_TOLERANCE = 0.01;
        public static final double ENCODER_TOLERANCE = 0.01; // [ticks]

        public static final double HEADING_KP = 5;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
        public static final TrapezoidProfile.Constraints HEADING_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 5); // [rads/sec], [rad/sec^2]

        // The heading is responsible for the angle of the whole chassis, while the angle is used in the angle motor itself.
        public static final double ALLOWABLE_HEADING_ERROR = Math.toRadians(5); // [rad]
        public static final double ALLOWABLE_ANGLE_ERROR = Math.toRadians(3); // [rad]
        public static final double WHEEL_RADIUS = 0.04688; // [m]

        public static final double ROBOT_LENGTH = 0.6624; // [m]
        public static final double ROBOT_WIDTH = 0.5224; // [m]

        // the rotational velocity of the robot, this constant multiplies the rotation output of the joystick
        public static final int ANGLE_CURVE_STRENGTH = 1;
        public static final int ANGLE_CRUISE_VELOCITY = 400;
        public static final int ANGLE_MOTION_ACCELERATION = 1300;

        public static final double JOYSTICK_THRESHOLD = 0.05; // [%]
        public static final double ROTATIONAL_ADDITION_RESTRAINT = 3;
        public static final double DRIFTING_PERIOD = 0.5; // expected period the robot will change its rotation even after commanded to stop. [s]
        public static final double SAMPLE_YAW_PERIOD = 0.1; // expected period the robot will change its rotation even after commanded to stop. [s]
        public static final double ADJUST_CONTROLLER_TOLERANCE = Math.toRadians(0.5);
        public static final WebConstant ADJUST_CONTROLLER_KP = WebConstant.of("Swerve", "kp_heading", 12.5);
        public static final WebConstant ADJUST_CONTROLLER_KI = WebConstant.of("Swerve", "KI_heading", 0);
        public static final WebConstant ADJUST_CONTROLLER_KD = WebConstant.of("Swerve", "Kd_heading", 0.1);

        private static final double Rx = SwerveDrive.ROBOT_LENGTH / 2; // [m]
        private static final double Ry = SwerveDrive.ROBOT_WIDTH / 2; // [m]

        // Axis systems
        public static final Translation2d[] SWERVE_POSITIONS = new Translation2d[]{
                new Translation2d(Rx, -Ry),
                new Translation2d(Rx, Ry),
                new Translation2d(-Rx, -Ry),
                new Translation2d(-Rx, Ry)
        };
    }

    public static class Shooter {
        public static final double TICKS_PER_REVOLUTION = 2048; // Ticks per revolution of the shooter motor. [tick]
        public static final double NEUTRAL_DEADBAND = 0.1; // [%]
        public static final double OUTPUT_MULTIPLIER = 0.1; // Multiplies the output for manual control in the bits. [%]
        public static final double OUTTAKE_POWER = 0.2; // Power to give to the shooter when taking balls out. [%]
        public static final double LOW_GOAL_VELOCITY = 2000;
        public static final double RECOMMENDED_ACCELERATION_TIME = 1.3; // Recommended time for the shooter to get to it's setpoint. [s]
        public static final double CARGO_OFFSET = 0; // Desired offset from the middle of the target where you want the cargo to hit. [m]

        public static final WebConstant kP = WebConstant.of("Shooter", "kP", 0.08);
        public static final WebConstant kI = WebConstant.of("Shooter", "kI", 0.0000075);
        public static final WebConstant kD = WebConstant.of("Shooter", "kD", 9);
        public static final WebConstant kF = WebConstant.of("Shooter", "kf", 0.0465);
//        public static final WebConstant kP = WebConstant.of("Shooter", "kP", 0.03);
//        public static final WebConstant kI = WebConstant.of("Shooter", "kI", 0.00);
//        public static final WebConstant kD = WebConstant.of("Shooter", "kD", 0);
//        public static final WebConstant kF = WebConstant.of("Shooter", "kf", 0.0475);
        public static final WebConstant SHOOTER_VELOCITY_DEADBAND = WebConstant.of("Shooter", "Velocity deadband", 50); // Dead band for shooter velocity setpoint. [rpm]
        public static final HashMap<Double, Double> SHORT_MEASUREMENTS = new HashMap<>() {{
            put(-99999.0, 3530.0);
            put(2.3, 3530.0);
            put(2.6, 3600.0);
            put(2.77, 3650.0);
            put(2.95, 3770.0);
            put(3.11, 3800.0);
            put(3.33, 3900.0);
            put(99999.0, 3900.0);
        }};
        public static final HashMap<Double, Double> LONG_MEASUREMENTS = new HashMap<>() {{
            put(-99999.0, 3675.0);
            put(3.33, 3765.0);
            put(3.52, 3900.0);
            put(3.7, 3965.0);
            put(3.92, 4000.0);
            put(4.13, 4070.0);
            put(4.25, 4155.0);
            put(4.48, 4250.0);
            put(4.81, 4398.0);
            put(5.0, 4575.0);
            put(5.25, 4620.0);
            put(5.6, 4795.0);
            put(6.0, 4890.0);
            put(6.41, 5060.0);
            put(99999.0, 5060.0);
        }};
        public static final double TARMAC_VELOCITY = 3700; // [rpm]


        public static TalonFXConfiguration getConfiguration() {
            final TalonFXConfiguration configuration = new TalonFXConfiguration();
            configuration.neutralDeadband = NEUTRAL_DEADBAND;
            return configuration;
        }
    }

    public static final class SwerveModule {
        public static final int TRIGGER_THRESHOLD_CURRENT = 2; // [amps]

        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // [secs]
        public static final double RAMP_RATE = 0; // seconds from neutral to max

        public static final int[] ZERO_POSITIONS = {-589, -453, -868, -519}; // fr, fl, rr, rl

        public static final SwerveModuleConfigBase frConfig = new SwerveModuleConfigBase.Builder(0)
                .configPorts(DRIVE_MOTOR_FR, ANGLE_MOTOR_FR)
                .configInversions(DRIVE_INVERTED_FR, ANGLE_INVERTED_FR, ANGLE_SENSOR_PHASE_FR)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[0])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase flConfig = new SwerveModuleConfigBase.Builder(1)
                .configPorts(DRIVE_MOTOR_FL, ANGLE_MOTOR_FL)
                .configInversions(DRIVE_INVERTED_FL, ANGLE_INVERTED_FL, ANGLE_SENSOR_PHASE_FL)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[1])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rrConfig = new SwerveModuleConfigBase.Builder(2)
                .configPorts(DRIVE_MOTOR_RR, ANGLE_MOTOR_RR)
                .configInversions(DRIVE_INVERTED_RR, ANGLE_INVERTED_RR, ANGLE_SENSOR_PHASE_RR)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[2])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rlConfig = new SwerveModuleConfigBase.Builder(3)
                .configPorts(DRIVE_MOTOR_RL, ANGLE_MOTOR_RL)
                .configInversions(DRIVE_INVERTED_RL, ANGLE_INVERTED_RL, ANGLE_SENSOR_PHASE_RL)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[3])
                .configJ(0.115)
                .build();
    }

    public static class Autonomous {
        public static final double KP_THETA_CONTROLLER = 8.8;
        public static final double KP_X_CONTROLLER = 3;
        public static final double KP_Y_CONTROLLER = 3;

        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0, 0, 0);
        public static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(0);
        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

        public static final double MAX_VEL = 4; // [m/sec] 4.1
        public static final double MAX_ACCEL = 2; // [m/sec^2] 2.14
    }

    public static class Control {
        public static final int JOYSTICK_FILTER_TAP = 8 + 8 / 2;

        public static final double RIGHT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
        public static final double LEFT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
    }
}
