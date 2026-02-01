package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "New Near Blue Side Auto", group = "Autonomous")
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

            PIDFCoefficients leftPidfCoefficients = new PIDFCoefficients(220, 0, 0, 12.915);
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
                    leftBlocker.setPosition(0.605);
                    rightBlocker.setPosition(0.372);
                    initialized = true;
                }

                if (timer.seconds() < 0.7) {
                    return true;
                } else {
                    leftBlocker.setPosition(0.186);
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
                .strafeToLinearHeading(new Vector2d(38.0, 51.5), Math.toRadians(166))
                .build();
        Action intakeSecond = drive.actionBuilder(new Pose2d(38.0, 51.5, Math.toRadians(166)))
                .lineToY(61.0)
                .build();
        Action goToShootSecond = drive.actionBuilder(new Pose2d(38.0, 61.0, Math.toRadians(166)))
                .lineToY(56.0)
                .strafeToLinearHeading(new Vector2d(56.0, 88.0), Math.toRadians(100))
                .build();
        Action intakeThird = drive.actionBuilder(new Pose2d(46.0, 98.0, Math.toRadians(100)))
                .strafeToLinearHeading(new Vector2d(6.0, 45.0), Math.toRadians(140))
                .strafeToLinearHeading(new Vector2d(-7.0, 64.0), Math.toRadians(140))
                .build();
        Action goToShootThird = drive.actionBuilder(new Pose2d(-7.0, 64.0, Math.toRadians(140)))
                .lineToY(56.0)
                .strafeToLinearHeading(new Vector2d(61.0, 50.0), Math.toRadians(103))
                .build();
        Action intakeForth = drive.actionBuilder(new Pose2d(61.0, 50.0, Math.toRadians(103)))
                .strafeToLinearHeading(new Vector2d(6.0, 45.0), Math.toRadians(140))
                .strafeToLinearHeading(new Vector2d(-5.0, 64.0), Math.toRadians(140))
                .build();
        Action goToShootForth = drive.actionBuilder(new Pose2d(-5.0, 64.0, Math.toRadians(120)))
                .lineToY(56.0)
                .strafeToLinearHeading(new Vector2d(33.0, 98.0), Math.toRadians(100))
                .build();
        Action goToIntakeFifth = drive.actionBuilder(new Pose2d(33.0, 98.0, Math.toRadians(100)))
                .strafeToLinearHeading(new Vector2d(37.0, 71.5), Math.toRadians(162))
                .build();
        Action intakeFifth = drive.actionBuilder(new Pose2d(37.0, 71.5, Math.toRadians(162)))
                .lineToY(83.0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shoot.turnShooterOn(1100),
                        goToShootFirst,
                        new SleepAction(0.2),
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        goToIntakeSecond,
                        new ParallelAction(
                                intake.turnIntakeOn(0.8),
                                intakeSecond
                        ),
                        shoot.turnShooterOn(1100),
                        goToShootSecond,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        intakeThird,
                        intake.turnIntakeOn(1.5),
                        shoot.turnShooterOn(1200),
                        goToShootThird,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff()
//                        intakeForth,
//                        intake.turnIntakeOn(2.0),
//                        shoot.turnShooterOn(),
//                        goToShootForth,
//                        intake.bringArtifacts(),
//                        shoot.turnShooterOff(),
//                        goToIntakeFifth,
//                        new ParallelAction(
//                                intake.turnIntakeOn(0.8),
//                                intakeFifth
//                        )
                )
        );
    }
}
