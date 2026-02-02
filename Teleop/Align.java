public class Align {
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    
    private final double LIMELIGHT_TURN_GAIN = 0.05;
    private final double LIMELIGHT_TOLERANCE = 1.5;
    private final double LIMELIGHT_TIMEOUT = 5.0;
    private final double LIMELIGHT_SPEED = 1.0;

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
        private boolean aligned = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                alignTimer = new ElapsedTime();
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
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    
                    packet.put("Limelight Status", "ALIGNED");
                    packet.put("TX Offset", tx);
                    return false;
                }

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
            } else {
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
