package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ModuleConstants;

public final class Configs {

        public static final class Lift {
                public static final SparkFlexConfig rightConfig = new SparkFlexConfig();
                public static final SparkFlexConfig leftConfig = new SparkFlexConfig();

                static {
                        rightConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.LiftConstants.kRightInverted)
                                        .smartCurrentLimit(Constants.LiftConstants.kRightCurrentLimit);
                        rightConfig.encoder
                                        .positionConversionFactor(Constants.LiftConstants.kLiftGearRatio)
                                        .velocityConversionFactor(Constants.LiftConstants.kLiftGearRatio / 60);
                        // rightConfig.closedLoop
                        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // .pid(Constants.LiftConstants.kLiftP,
                        // Constants.LiftConstants.kLiftI,
                        // Constants.LiftConstants.kLiftD)
                        // .outputRange(-1, 1);
                        // rightConfig.closedLoop.maxMotion
                        // .maxAcceleration(2)
                        // .maxVelocity(5);

                        leftConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.LiftConstants.kLeftInverted)
                                        .smartCurrentLimit(Constants.LiftConstants.kLeftCurrentLimit);
                        leftConfig.encoder
                                        .positionConversionFactor(Constants.LiftConstants.kLiftGearRatio)
                                        .velocityConversionFactor(Constants.LiftConstants.kLiftGearRatio / 60);
                        // leftConfig.closedLoop
                        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // .pid(Constants.LiftConstants.kLiftP,
                        // Constants.LiftConstants.kLiftI,
                        // Constants.LiftConstants.kLiftD)
                        // .outputRange(-1, 1);
                        // leftConfig.closedLoop.maxMotion
                        // .maxAcceleration(2)
                        // .maxVelocity(5);
                }
        }

        public static final class Outake {
                public static final SparkFlexConfig motorConfig = new SparkFlexConfig();

                static {
                        motorConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.OutakeConstants.kInverted)
                                        .smartCurrentLimit(Constants.OutakeConstants.kCurrentLimit);
                        motorConfig.encoder
                                        .positionConversionFactor(Constants.OutakeConstants.kOutakeGearRatio)
                                        .velocityConversionFactor(Constants.OutakeConstants.kOutakeGearRatio / 60);
                        motorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(Constants.OutakeConstants.kOutakeP,
                                                        Constants.OutakeConstants.kOutakeI,
                                                        Constants.OutakeConstants.kOutakeD)
                                        .outputRange(-1, 1);
                }
        }

        public static final class FourBar {
                public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

                static {
                        motorConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.FourBarConstants.kInverted)
                                        .smartCurrentLimit(Constants.FourBarConstants.kCurrentLimit);
                        motorConfig.encoder
                                        .positionConversionFactor(1)
                                        .velocityConversionFactor(1);
                        motorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(Constants.FourBarConstants.kFourBarP,
                                                        Constants.FourBarConstants.kFourBarI,
                                                        Constants.FourBarConstants.kFourBarD)
                                        .outputRange(-1, 1);
                }
        }

        public static final class Wrist {
                public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

                static {
                        motorConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.WristConstants.kInverted)
                                        .smartCurrentLimit(Constants.WristConstants.kCurrentLimit);
                        motorConfig.encoder
                                        .positionConversionFactor(1)
                                        .velocityConversionFactor(1);
                        motorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(Constants.WristConstants.kWristP,
                                                        Constants.WristConstants.kWristI,
                                                        Constants.WristConstants.kWristD)
                                        .outputRange(-1, 1);
                }
        }

        public static final class Climber {
                public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

                static {
                        motorConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.ClimberConstants.kMotorInverted)
                                        .smartCurrentLimit(Constants.ClimberConstants.kMotorCurrentLimmit);
                }
        }

        public static final class Intake {
                public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
                public static final SparkFlexConfig deliveryConfig = new SparkFlexConfig();

                static {
                        intakeConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.IntakeConstants.kIntakeInverted)
                                        .smartCurrentLimit(Constants.IntakeConstants.kIntakeCurrentLimit);
                        // rightConfig.encoder
                        //                 .positionConversionFactor(1)
                        //                 .velocityConversionFactor(1);
                        // rightConfig.closedLoop
                        //                 .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        //                 .pid(Constants.IntakeConstants.kIntakeP,
                        //                                 Constants.IntakeConstants.kIntakeI,
                        //                                 Constants.IntakeConstants.kIntakeD)
                        //                 .velocityFF(1)
                        //                 .outputRange(-1, 1);

                        deliveryConfig
                                        .idleMode(IdleMode.kBrake)
                                        .inverted(Constants.IntakeConstants.kDeliveryInverted)
                                        .smartCurrentLimit(Constants.IntakeConstants.kDeliveryCurrentLimit);
                        // leftConfig.encoder
                        //                 .positionConversionFactor(1)
                        //                 .velocityConversionFactor(1);
                        // leftConfig.closedLoop
                        //                 .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        //                 .pid(Constants.IntakeConstants.kIntakeP,
                        //                                 Constants.IntakeConstants.kIntakeI,
                        //                                 Constants.IntakeConstants.kIntakeD)
                        //                 .velocityFF(1)
                        //                 .outputRange(-1, 1);
                }
        }

        public static final class MAXSwerveModule {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward gain.
                        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                                        / ModuleConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(0.04, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20);
                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, since the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(1, 0, 0)
                                        .outputRange(-1, 1)
                                        // Enable PID wrap around for the turning motor. This will allow the PID
                                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                                        // to 10 degrees will go through 0 rather than the other direction which is a
                                        // longer route.
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
