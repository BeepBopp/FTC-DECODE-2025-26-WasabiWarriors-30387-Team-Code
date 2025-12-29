package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "Near Blue Side Auto - Roadrunner", group = "Autonomous")
public class NearBlueSideAuto extends LinearOpMode {
    public class Shoot {
        private DcMotorEx leftShooter, rightShooter;

        public Shoot(HardwareMap hardwareMap) {
            leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
            rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
            leftShooter.setDirection(DcMotor.Direction.REVERSE);
            rightShooter.setDirection(DcMotor.Direction.FORWARD);
            leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public class TurnShooterOn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftShooter.setVelocity(1650);
                    rightShooter.setVelocity(1675);
                    initialized = true;
                }
                return false;  // Complete immediately, no wait time
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
        private DcMotor intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.REVERSE);
        }

        public class TurnIntakeOn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0.9);
                    initialized = true;
                }
                return false;
            }
        }

        public Action turnIntakeOn() {
            return new TurnIntakeOn();
        }

        public class TurnIntakeOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0);
                    initialized = true;
                }
                return false;
            }
        }

        public Action turnIntakeOff() {
            return new TurnIntakeOff();
        }

        public class BringThirdBall implements Action {
            private boolean initialized = false;
            private double startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;

                if (elapsed > 0 && elapsed < 0.1) {
                    intake.setPower(0);
                } else if (elapsed > 0.1 && elapsed < 0.2) {
                    intake.setDirection(DcMotor.Direction.FORWARD);
                    intake.setPower(0.5);
                } else if (elapsed > 0.2 && elapsed < 0.35) {
                    intake.setDirection(DcMotor.Direction.REVERSE);
                    intake.setPower(1.0);
                } else if (elapsed >= 0.35) {
                    intake.setPower(0);
                    return false;
                }

                packet.put("Third Ball Sequence Time", elapsed);
                return true;
            }
        }

        public Action bringThirdBall() {
            return new BringThirdBall();
        }
    }

    public class Feed {
        private Servo leftBringUp, rightBringUp;

        public Feed(HardwareMap hardwareMap) {
            leftBringUp = hardwareMap.get(Servo.class, "leftBringUp");
            rightBringUp = hardwareMap.get(Servo.class, "rightBringUp");
        }

        public class BringUp implements Action {
            private boolean initialized = false;
            private double startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftBringUp.setPosition(0.95);
                    rightBringUp.setPosition(0.04);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                double elapsed = System.currentTimeMillis() - startTime;
                packet.put("Feed Up Time", elapsed);
                return elapsed < 500;
            }
        }

        public Action bringUp() {
            return new BringUp();
        }

        public class BringDown implements Action {
            private boolean initialized = false;
            private double startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftBringUp.setPosition(0.5);
                    rightBringUp.setPosition(0.5);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                double elapsed = System.currentTimeMillis() - startTime;
                packet.put("Feed Down Time", elapsed);
                return elapsed < 500;
            }
        }

        public Action bringDown() {
            return new BringDown();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(21.0, 124.0, Math.toRadians(125));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shoot shoot = new Shoot(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Feed feed = new Feed(hardwareMap);

        Action goToShootFirst = drive.actionBuilder(initialPose)
                .lineToX(40.0)
                .build();
        Action goToIntakeSecond = drive.actionBuilder(new Pose2d(40.0, 124.0, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(38.0, 74.5), Math.toRadians(163))
                .build();
        Action intakeSecond = drive.actionBuilder(new Pose2d(38.0, 74.5, Math.toRadians(163)))
                .lineToY(82.0)
                .build();
        Action goToShootSecond = drive.actionBuilder(new Pose2d(38.0, 84.0, Math.toRadians(163)))
                .strafeToLinearHeading(new Vector2d(60.0, 88.0), Math.toRadians(120))
                .build();
        Action goToIntakeThird = drive.actionBuilder(new Pose2d(60.0, 88.0, Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(35.0, 38.0), Math.toRadians(166.5))
                .build();
        Action intakeThird = drive.actionBuilder(new Pose2d(35.0, 49.0, Math.toRadians(163)))
                .lineToY(57.0)
                .build();
        Action goToShootThird = drive.actionBuilder(new Pose2d(35.0, 55.0, Math.toRadians(163)))
                .strafeToLinearHeading(new Vector2d(54.0, 85.0), Math.toRadians(120))
                .build();
        Action goToIntakeForth = drive.actionBuilder(new Pose2d(54.0, 85.0, Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(32.5, 15.0), Math.toRadians(163))
                .build();
        Action intakeForth = drive.actionBuilder(new Pose2d(32.5, 15.0, Math.toRadians(163)))
                .lineToY(25.0)
                .build();
        Action goToShootForth = drive.actionBuilder(new Pose2d(32.0, 25.0, Math.toRadians(163)))
                .strafeToLinearHeading(new Vector2d(70.0, 78.0), Math.toRadians(111.5))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        // shoot first 3 balls
                        shoot.turnShooterOn(),
                        goToShootFirst,
                        feed.bringUp(),
                        feed.bringDown(),
                        intake.bringThirdBall(),
                        new SleepAction(1.0 / 3),
                        feed.bringUp(),
                        feed.bringDown(),
                        shoot.turnShooterOff(),
                        // pick up second set of balls
                        goToIntakeSecond,
                        intake.turnIntakeOn(),
                        intakeSecond,
                        // shoot second set of balls
                        shoot.turnShooterOn(),
                        goToShootSecond,
                        intake.turnIntakeOff(),
                        new SleepAction(0.5),
                        feed.bringUp(),
                        feed.bringDown(),
                        intake.bringThirdBall(),
                        new SleepAction(1.0 / 3),
                        feed.bringUp(),
                        feed.bringDown(),
                        // pick up third set of balls
                        goToIntakeThird,
                        intake.turnIntakeOn(),
                        intakeThird,
                        // shoot third set of balls
                        shoot.turnShooterOn(),
                        goToShootThird,
                        intake.turnIntakeOff(),
                        feed.bringUp(),
                        feed.bringDown(),
                        intake.bringThirdBall(),
                        new SleepAction(1.0 / 3),
                        feed.bringUp(),
                        feed.bringDown(),
                        // pick up forth set of balls
                        goToIntakeForth,
                        intake.turnIntakeOn(),
                        intakeForth,
                        // shoot forth set of balls
                        shoot.turnShooterOn(),
                        goToShootForth,
                        intake.turnIntakeOff(),
                        feed.bringUp(),
                        feed.bringDown(),
                        intake.bringThirdBall(),
                        new SleepAction(1.0 / 3),
                        feed.bringUp(),
                        feed.bringDown()
                )
        );
    }
}
