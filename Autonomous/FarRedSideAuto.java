package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;

@Config
@Autonomous(name = "Far Red Side Auto v1", group = "Autonomous")
public class RightFarV2 extends LinearOpMode {
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
                    leftShooter.setVelocity(velocity+70);
                    rightShooter.setVelocity(velocity-70);
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

                if (timer.seconds() < 0.7) {
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

    public class Align {
        private Limelight3A limelight;
        private IMU imu;
        private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

        private final double LIMELIGHT_TURN_GAIN = 0.05;
        private final double LIMELIGHT_TOLERANCE = 1.5;
        private final double LIMELIGHT_TIMEOUT = 1.0;
        private final double LIMELIGHT_SPEED = 1.0;
        private final double TOLERANCE_HOLD_TIME = 0.2;

        public Align(HardwareMap hardwareMap) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            imu = hardwareMap.get(IMU.class, "imu");
            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
            leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
            rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

            limelight.pipelineSwitch(0);
            limelight.start();
        }

        public class AlignToAprilTag implements Action {
            private boolean initialized = false;
            private ElapsedTime alignTimer;
            private ElapsedTime toleranceTimer;
            private boolean isWithinTolerance = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    alignTimer = new ElapsedTime();
                    toleranceTimer = new ElapsedTime();
                    initialized = true;
                }

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double botHeading = orientation.getYaw(AngleUnit.RADIANS);
                limelight.updateRobotOrientation(Math.toDegrees(botHeading));

                LLResult limelightResult = limelight.getLatestResult();

                if (limelightResult != null && limelightResult.isValid()) {
                    double tx = limelightResult.getTx();
                    double turn = tx * LIMELIGHT_TURN_GAIN;

                    if (Math.abs(tx) < LIMELIGHT_TOLERANCE) {
                        if (!isWithinTolerance) {
                            isWithinTolerance = true;
                            toleranceTimer.reset();
                        }

                        if (toleranceTimer.seconds() >= TOLERANCE_HOLD_TIME) {
                            leftFrontDrive.setPower(0);
                            leftBackDrive.setPower(0);
                            rightFrontDrive.setPower(0);
                            rightBackDrive.setPower(0);

                            packet.put("Limelight Status", "ALIGNED (held 0.2s)");
                            packet.put("TX Offset", tx);
                            packet.put("Hold Time", toleranceTimer.seconds());
                            return false;
                        }

                        leftFrontDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);

                        packet.put("Limelight Status", "HOLDING ALIGNMENT");
                        packet.put("TX Offset", tx);
                        packet.put("Hold Time", String.format("%.2f/%.2f", toleranceTimer.seconds(), TOLERANCE_HOLD_TIME));
                        return true;
                    } else {
                        isWithinTolerance = false;

                        double leftFrontPower = turn * LIMELIGHT_SPEED;
                        double leftBackPower = turn * LIMELIGHT_SPEED;
                        double rightFrontPower = -turn * LIMELIGHT_SPEED;
                        double rightBackPower = -turn * LIMELIGHT_SPEED;

                        leftFrontDrive.setPower(leftFrontPower);
                        leftBackDrive.setPower(leftBackPower);
                        rightFrontDrive.setPower(rightFrontPower);
                        rightBackDrive.setPower(rightBackPower);

                        packet.put("Limelight Status", "ALIGNING");
                        packet.put("TX Offset", tx);
                        packet.put("Turn Power", turn);
                        return true;
                    }
                } else {
                    isWithinTolerance = false;


                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);

                    if (alignTimer.seconds() > LIMELIGHT_TIMEOUT) {
                        packet.put("Limelight Status", "TIMEOUT - Tag Lost");
                        return false;
                    }

                    packet.put("Limelight Status", "SEARCHING");
                    return true;
                }
            }
        }

        public Action alignToAprilTag() {
            return new AlignToAprilTag();
        }
    }




    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(12.0, 9.0, Math.toRadians(90));
        Pose2d shootPose = new Pose2d(12.0,20.0, Math.toRadians(80));
        Pose2d strafePose = new Pose2d(12.0,20.0,Math.toRadians(0));
        Pose2d humanPlayerPose = new Pose2d(50.0, 10, Math.toRadians(0));
        Pose2d humanPlayerPoseEnd = new Pose2d(54.0, 10, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shoot shoot = new Shoot(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Align align = new Align(hardwareMap);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(30),
                new AngularVelConstraint(Math.PI)
        ));

        Action goToShootPosition = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(12.0, 20.0), Math.toRadians(80)).build();
        Action reCenter = drive.actionBuilder(shootPose).turnTo(Math.toRadians(0)).build();
        Action reCenter2 = drive.actionBuilder(shootPose).turnTo(Math.toRadians(0)).build();
        Action reCenter3 = drive.actionBuilder(shootPose).turnTo(Math.toRadians(0)).build();
        Action reCenter4 = drive.actionBuilder(shootPose).turnTo(Math.toRadians(0)).build();
        Action goToHumanPlayer = drive.actionBuilder(strafePose).lineToXLinearHeading(50,Math.toRadians(0)).build();
        Action goToHumanPlayer2 = drive.actionBuilder(strafePose).lineToXLinearHeading(50,Math.toRadians(0)).build();
        Action goToHumanPlayer3 = drive.actionBuilder(strafePose).lineToXLinearHeading(50,Math.toRadians(0)).build();
        Action goToHumanPlayer4 = drive.actionBuilder(strafePose).lineToXLinearHeading(50,Math.toRadians(0)).build();
        Action takeBallFromHumanPlayer = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action takeBallFromHumanPlayer2 = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action takeBallFromHumanPlayer3 = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action takeBallFromHumanPlayer4 = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action backOutofHumanPlayer = drive.actionBuilder(humanPlayerPoseEnd).lineToX(52,slowVel, drive.defaultAccelConstraint).build();
        Action backOutofHumanPlayer2 = drive.actionBuilder(humanPlayerPoseEnd).lineToX(52,slowVel, drive.defaultAccelConstraint).build();
        Action backOutofHumanPlayer3 = drive.actionBuilder(humanPlayerPoseEnd).lineToX(52,slowVel, drive.defaultAccelConstraint).build();
        Action backOutofHumanPlayer4 = drive.actionBuilder(humanPlayerPoseEnd).lineToX(52,slowVel, drive.defaultAccelConstraint).build();
        Action gofromBackPosetoPlayerPose = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action gofromBackPosetoPlayerPose2 = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action gofromBackPosetoPlayerPose3 = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action gofromBackPosetoPlayerPose4 = drive.actionBuilder(humanPlayerPose).lineToX(54,slowVel, drive.defaultAccelConstraint).build();
        Action gofromPlayerPosetoShootPose = drive.actionBuilder(humanPlayerPoseEnd).strafeToLinearHeading(new Vector2d(-8.0, 14.0),Math.toRadians(80)).build();
        Action gofromPlayerPosetoShootPose2 = drive.actionBuilder(humanPlayerPoseEnd).strafeToLinearHeading(new Vector2d(-8.0, 14.0),Math.toRadians(80)).build();
        Action gofromPlayerPosetoShootPose3 = drive.actionBuilder(humanPlayerPoseEnd).strafeToLinearHeading(new Vector2d(-8.0, 14.0),Math.toRadians(80)).build();
        Action gofromPlayerPosetoShootPose4 = drive.actionBuilder(humanPlayerPoseEnd).strafeToLinearHeading(new Vector2d(-8.0, 14.0),Math.toRadians(80)).build();
        Action walkingout = drive.actionBuilder(shootPose).lineToY(30).build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shoot.turnShooterOn(1430),
                        goToShootPosition,
                        align.alignToAprilTag(),
                        new SleepAction(0.7),
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),

                        reCenter,
                        new ParallelAction(
                                intake.turnIntakeOn(1.0),
                                new SequentialAction(
                                        goToHumanPlayer,
                                        takeBallFromHumanPlayer,
                                        backOutofHumanPlayer,
                                        gofromBackPosetoPlayerPose
                                )
                        ),

                        shoot.turnShooterOn(1440),
                        gofromPlayerPosetoShootPose,
                        align.alignToAprilTag(),
                        new SleepAction(0.4),
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),

                        reCenter2,
                        new ParallelAction(
                                intake.turnIntakeOn(1.0),
                                new SequentialAction(
                                        goToHumanPlayer2,
                                        takeBallFromHumanPlayer2,
                                        backOutofHumanPlayer2,
                                        gofromBackPosetoPlayerPose2
                                )
                        ),
                        shoot.turnShooterOn(1440),
                        gofromPlayerPosetoShootPose2,
                        align.alignToAprilTag(),
                        new SleepAction(0.4),
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),

                        reCenter3,
                        new ParallelAction(
                                intake.turnIntakeOn(1.0),
                                new SequentialAction(
                                        goToHumanPlayer3,
                                        takeBallFromHumanPlayer3,
                                        backOutofHumanPlayer3,
                                        gofromBackPosetoPlayerPose3
                                )
                        ),
                        shoot.turnShooterOn(1430),
                        gofromPlayerPosetoShootPose3,
                        align.alignToAprilTag(),
                        new SleepAction(0.4),
                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),

                        reCenter4,
                        new ParallelAction(
                                intake.turnIntakeOn(1.0),
                                new SequentialAction(
                                        goToHumanPlayer4,
                                        takeBallFromHumanPlayer4,
                                        backOutofHumanPlayer4,
                                        gofromBackPosetoPlayerPose4
                                )
                        ),
                        shoot.turnShooterOn(1440),
                        gofromPlayerPosetoShootPose4,
                        align.alignToAprilTag(),
                        new SleepAction(0.4),

                        intake.bringArtifacts(),
                        shoot.turnShooterOff(),

                        walkingout
                )
        );
    }
}
