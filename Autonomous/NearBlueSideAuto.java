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

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftShooter.setVelocity(1100);
                    rightShooter.setVelocity(1100);
                    intake.setPower(0.8);
                    chute.setPower(1.0);
                    initialized = true;
                }
                return false;
            }
        }

        public Action turnShooterOn() {
            return new TurnShooterOn();
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
                .strafeToLinearHeading(new Vector2d(36.5, 51.5), Math.toRadians(166))
                .build();
        Action intakeSecond = drive.actionBuilder(new Pose2d(36.5, 51.5, Math.toRadians(166)))
                .lineToY(61.0)
                .build();
        Action goToShootSecond = drive.actionBuilder(new Pose2d(36.5, 61.0, Math.toRadians(166)))
                .lineToY(58.0)
                .strafeToLinearHeading(new Vector2d(56.0, 98.0), Math.toRadians(77.5))
                .build();
        //Action intakeThird = drive.actionBuilder(new Pose2d(56.0, 98.0, Math.toRadians(75)))
                        //.build()

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shoot.turnShooterOn(),
                        goToShootFirst,
                        new SleepAction(0.3),
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        goToIntakeSecond,
                        new ParallelAction(
                                intake.turnIntakeOn(0.8),
                                intakeSecond
                        ),
                        shoot.turnShooterOn(),
                        goToShootSecond,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff()
                )
        );
    }
}
