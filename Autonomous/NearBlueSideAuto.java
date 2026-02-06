package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Config
@Autonomous(name = "Near Blue Side Auto", group = "Autonomous")
public class NewNearBlueSideAuto extends LinearOpMode {
    public class Shoot {
        private DcMotorEx leftShooter, rightShooter;
        private DcMotor intake, chute;

        public Shoot(HardwareMap hardwareMap) {
            leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
            rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
            intake = hardwareMap.get(DcMotor.class, "intake");
            chute = hardwareMap.get(DcMotor.class, "chute");

            rightShooter.setDirection(DcMotor.Direction.REVERSE);
            chute.setDirection(DcMotor.Direction.REVERSE);
            leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients leftPidfCoefficients = new PIDFCoefficients(200, 0, 0, 12.855);
            PIDFCoefficients rightPidfCoefficients = new PIDFCoefficients(220, 0, 0, 12.815);
            leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftPidfCoefficients);
            rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightPidfCoefficients);
        }

        public class TurnShooterOn implements Action {
            private boolean initialized = false;
            private double velocity;

            public TurnShooterOn(double velocity) {
                this.velocity = velocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftShooter.setVelocity(velocity);
                    rightShooter.setVelocity(velocity);
                    intake.setPower(0.8);
                    chute.setPower(1.0);
                    initialized = true;
                }
                return false;
            }
        }

        public Action turnShooterOn(double velocity) {
            return new TurnShooterOn(velocity);
        }

        public class TurnShooterOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftShooter.setVelocity(0);
                    rightShooter.setVelocity(0);
                    intake.setPower(0);
                    chute.setPower(0);
                    initialized = true;
                }
                return false;
            }
        }

        public Action turnShooterOff() {
            return new TurnShooterOff();
        }
    }

    public class Intake {
        private DcMotor intake, chute;
        private Servo leftBlocker, rightBlocker;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            chute = hardwareMap.get(DcMotor.class, "chute");
            leftBlocker = hardwareMap.get(Servo.class, "leftBlocker");
            rightBlocker = hardwareMap.get(Servo.class, "rightBlocker");

            chute.setDirection(DcMotor.Direction.REVERSE);
        }

        public class TurnIntakeOn implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;
            private ElapsedTime chuteTimer;
            private boolean chuteOscillationState = false;
            private double duration;

            public TurnIntakeOn(double duration) {
                this.duration = duration;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(1.0);
                    timer = new ElapsedTime();
                    chuteTimer = new ElapsedTime();
                    initialized = true;
                }

                if (chuteTimer.seconds() >= 0.1) {
                    chuteOscillationState = !chuteOscillationState;
                    chuteTimer.reset();
                }
                chute.setPower(chuteOscillationState ? 1.0 : -1.0);

                return timer.seconds() < duration;
            }
        }

        public Action turnIntakeOn(double duration) {
            return new TurnIntakeOn(duration);
        }

        public class TurnIntakeOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0.0);
                    chute.setPower(0.0);
                    initialized = true;
                }
                return false;
            }
        }

        public Action turnIntakeOff() {
            return new TurnIntakeOff();
        }

        public class BringArtifacts implements Action {
            private boolean initialized = false;
            private ElapsedTime timer;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer = new ElapsedTime();
                    leftBlocker.setPosition(0.922);
                    rightBlocker.setPosition(0.372);
                    initialized = true;
                }

                if (timer.seconds() < 0.75) {
                    return true;
                } else {
                    leftBlocker.setPosition(0.51);
                    rightBlocker.setPosition(0.82);
                    return false;
                }
            }
        }

        public Action bringArtifacts() {
            return new BringArtifacts();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(21.0, 124.0, Math.toRadians(125));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shoot shoot = new Shoot(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action goToShootFirst = drive.actionBuilder(initialPose)
                .lineToX(41.0)
                .build();
        Action goToIntakeSecond = drive.actionBuilder(new Pose2d(41.0, 124.0, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(38.0, 51.0), Math.toRadians(166))
                .build();
        Action intakeSecond = drive.actionBuilder(new Pose2d(38.0, 51.0, Math.toRadians(166)))
                .lineToY(60.0)
                .build();
        Action goToShootSecond = drive.actionBuilder(new Pose2d(38.0, 60.0, Math.toRadians(166)))
                .lineToY(58.0)
                .strafeToLinearHeading(new Vector2d(58.0, 88.0), Math.toRadians(82))
                .build();
        Action goToIntakeThird = drive.actionBuilder(new Pose2d(58.0, 88.0, Math.toRadians(82)))
                .strafeToLinearHeading(new Vector2d(44.0, 78.0), Math.toRadians(140))
                .build();
        Action intakeThird = drive.actionBuilder(new Pose2d(44.0, 78.0, Math.toRadians(166)))
                .lineToY(86.0)
                .waitSeconds(0.2)
                .lineToY(84.0)
                .strafeTo(new Vector2d(0.0, 76.0))
                .build();
        Action goToShootThird = drive.actionBuilder(new Pose2d(0.0, 76.0, Math.toRadians(162)))
                .strafeToLinearHeading(new Vector2d(43.0, 76.0), Math.toRadians(104))
                .build();
        Action intakeFourth = drive.actionBuilder(new Pose2d(43.0, 76.0, Math.toRadians(104)))
                .strafeToLinearHeading(new Vector2d(6.0, 48.0), Math.toRadians(140))
                .strafeToLinearHeading(new Vector2d(-7.5, 60.0), Math.toRadians(140))
                .build();
        Action goToShootFourth = drive.actionBuilder(new Pose2d(-7.5, 60.0, Math.toRadians(140)))
                .lineToY(54.0)
                .strafeToLinearHeading(new Vector2d(46.5, 67.0), Math.toRadians(103))
                .build();
        Action goToIntakeFifth = drive.actionBuilder(new Pose2d(46.5, 67.0, Math.toRadians(103)))
                .strafeToLinearHeading(new Vector2d(29.5, 17.0), Math.toRadians(170))
                .build();
        Action intakeFifth = drive.actionBuilder(new Pose2d(29.5, 17.0, Math.toRadians(170)))
                .lineToY(25.0)
                .build();
        Action goToShootFifth = drive.actionBuilder(new Pose2d(29.5, 25.0, Math.toRadians(170)))
                .strafeToLinearHeading(new Vector2d(130.0, 80.0), Math.toRadians(10))
                .build();
        Action resetPoseBeforeTurn = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drive.localizer.setPose(new Pose2d(130.0, 80.0, Math.toRadians(10)));
                return false;
            }
        };
        Action intakeSixth = drive.actionBuilder(new Pose2d(130.0, 80.0, Math.toRadians(10)))
                .turnTo(Math.toRadians(102.5),
                        new TurnConstraints(Math.PI, -4 * Math.PI, 4 * Math.PI))
                .lineToY(190.0,
                        new TranslationalVelConstraint(300),
                        new ProfileAccelConstraint(-180, 300)
                )
                .build();
        Action goToShootSixth = drive.actionBuilder(new Pose2d(130.0, 190.0, Math.toRadians(102.5)))
                .strafeToLinearHeading(new Vector2d(60.0, -35.0), Math.toRadians(-50), // (60, -30, -75)
                        new TranslationalVelConstraint(300),
                        new ProfileAccelConstraint(-180, 300))
                .build();
        Action resetPoseAgainBeforeTurn = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drive.localizer.setPose(new Pose2d(60.0, -35.0, Math.toRadians(-50)));
                return false;
            }
        };
        Action goToFinalPosition = drive.actionBuilder(new Pose2d(60.0, -35.0, Math.toRadians(-50)))
                .strafeToLinearHeading(new Vector2d(80.0, -35.0), Math.toRadians(-50), // (70, -35, -75)
                        new TranslationalVelConstraint(300),
                        new ProfileAccelConstraint(-180, 300))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shoot.turnShooterOn(1075),
                        goToShootFirst,
                        //new SleepAction(0.2),
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        goToIntakeSecond,
                        new ParallelAction(
                                intake.turnIntakeOn(1.0),
                                intakeSecond
                        ),
                        shoot.turnShooterOn(1125),
                        goToShootSecond,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        goToIntakeThird,
                        new ParallelAction(
                                intake.turnIntakeOn(1.0),
                                intakeThird
                        ),
                        new SleepAction(0.4),
                        shoot.turnShooterOn(1125),
                        goToShootThird,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        intakeFourth,
                        intake.turnIntakeOn(2.0),
                        shoot.turnShooterOn(1175),
                        goToShootFourth,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        goToIntakeFifth,
                        new ParallelAction(
                                intake.turnIntakeOn(1.0),
                                intakeFifth
                        ),
                        shoot.turnShooterOn(1150),
                        goToShootFifth,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        resetPoseBeforeTurn,
                        new ParallelAction(
                                intake.turnIntakeOn(2.0),
                                intakeSixth
                        ),
                        intake.turnIntakeOn(1.0),
                        shoot.turnShooterOn(1150),
                        goToShootSixth,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        resetPoseAgainBeforeTurn,
                        goToFinalPosition
                )
        );
    }
}
