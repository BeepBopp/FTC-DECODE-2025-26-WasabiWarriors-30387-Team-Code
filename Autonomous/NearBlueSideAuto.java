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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Near Blue Side Auto", group = "Autonomous")
public class NearBlueSideAuto extends LinearOpMode {
    public class Shoot {
        private DcMotorEx leftShooter, rightShooter;

        public Shoot(HardwareMap hardwareMap) {
            leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
            rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

            rightShooter.setDirection(DcMotor.Direction.REVERSE);
            leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients leftPidfCoefficients = new PIDFCoefficients(227, 0, 0, 12.915);
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
        private CRServo chute;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            chute = hardwareMap.get(CRServo.class, "chute");
            chute.setDirection(CRServo.Direction.REVERSE);
        }

        public class TurnIntakeOn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(1.0);
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
                    intake.setPower(0.0);
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
                    initialized = true;
                }

                if (timer.seconds() < 2.1) {
                    intake.setPower(0.8);
                    chute.setPower(1.0);
                    return true;
                } else {
                    intake.setPower(0);
                    chute.setPower(0);
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
                .strafeToLinearHeading(new Vector2d(37.0, 71.5), Math.toRadians(162))
                .build();
        Action intakeSecond = drive.actionBuilder(new Pose2d(37.0, 71.5, Math.toRadians(162)))
                .lineToY(79.0)
                .strafeTo(new Vector2d(10.0, 70.0))
                .strafeTo(new Vector2d(2.5, 70.0))
                .build();
        Action goToShootSecond = drive.actionBuilder(new Pose2d(2.5, 70.0, Math.toRadians(162)))
                .strafeToLinearHeading(new Vector2d(42.0, 88.0), Math.toRadians(124))
                .build();
        Action goToIntakeThird = drive.actionBuilder(new Pose2d(42.0, 88.0, Math.toRadians(124)))
                .strafeToLinearHeading(new Vector2d(32.0, 42.0), Math.toRadians(163))
                .build();
        Action intakeThird = drive.actionBuilder(new Pose2d(32.0, 42.0, Math.toRadians(163)))
                .lineToY(51.0)
                .build();
        Action goToShootThird = drive.actionBuilder(new Pose2d(32.0, 51.0, Math.toRadians(163)))
                .strafeToLinearHeading(new Vector2d(51.0, 80.0), Math.toRadians(110))
                .build();
        Action goToIntakeForth = drive.actionBuilder(new Pose2d(51.0, 80.0, Math.toRadians(110)))
                .strafeToLinearHeading(new Vector2d(29.5, 14.0), Math.toRadians(163))
                .build();
        Action intakeForth = drive.actionBuilder(new Pose2d(29.5, 14.0, Math.toRadians(163)))
                .lineToY(23.0)
                .build();
        Action goToShootForth = drive.actionBuilder(new Pose2d(29.5, 23.0, Math.toRadians(163)))
                .strafeToLinearHeading(new Vector2d(52.0, 80.0), Math.toRadians(114))
                .build();
        Action goToFinalPosition = drive.actionBuilder(new Pose2d(52.0, 80.0, Math.toRadians(109.5)))
                .strafeToLinearHeading(new Vector2d(32.0, 62.0), Math.toRadians(163))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        // shoot first 3 balls
                        shoot.turnShooterOn(),
                        goToShootFirst,
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),
                        // pick up second set of balls
                        goToIntakeSecond,
                        intake.turnIntakeOn(),
                        intakeSecond,
                        new SleepAction(0.5),
                        // shoot third set of balls
                        shoot.turnShooterOn(),
                        goToShootSecond,
                        intake.turnIntakeOff(),
                        intake.bringArtifacts(),
                        // pick up third set of balls
                        goToIntakeThird,
                        intake.turnIntakeOn(),
                        intakeThird,
                        // shoot second set of balls
                        shoot.turnShooterOn(),
                        goToShootThird,
                        intake.turnIntakeOff(),
                        intake.bringArtifacts(),
                        // pick up forth set of balls
                        goToIntakeForth,
                        intake.turnIntakeOn(),
                        intakeForth,
                        // shoot forth set of balls
                        shoot.turnShooterOn(),
                        goToShootForth,
                        intake.turnIntakeOff(),
                        intake.bringArtifacts(),
                        goToFinalPosition
                )
        );
    }
}
