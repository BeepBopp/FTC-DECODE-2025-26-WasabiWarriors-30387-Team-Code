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

            // Update robot orientation for Limelight
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double botHeading = orientation.getYaw(AngleUnit.RADIANS);
            limelight.updateRobotOrientation(Math.toDegrees(botHeading));
            
            LLResult limelightResult = limelight.getLatestResult();

            if (limelightResult != null && limelightResult.isValid()) {
                // Get horizontal offset (tx)
                double tx = limelightResult.getTx();
                double turn = tx * LIMELIGHT_TURN_GAIN;

                // Check if aligned
                if (Math.abs(tx) < LIMELIGHT_TOLERANCE) {
                    // Stop all motors
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    
                    packet.put("Limelight Status", "ALIGNED");
                    packet.put("TX Offset", tx);
                    return false; // Action complete
                }

                // Apply rotation to face the tag
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
                
                return true; // Continue action
            } else {
                // Tag not detected - stop and wait
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);

                // Check for timeout
                if (alignTimer.seconds() > LIMELIGHT_TIMEOUT) {
                    packet.put("Limelight Status", "TIMEOUT - Tag Lost");
                    return false; // Action failed/complete
                }

                packet.put("Limelight Status", "SEARCHING");
                return true; // Keep trying
            }
        }
    }

    public Action alignToAprilTag() {
        return new AlignToAprilTag();
    }
