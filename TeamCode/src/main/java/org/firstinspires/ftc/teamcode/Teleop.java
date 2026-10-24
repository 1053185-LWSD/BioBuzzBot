
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends LinearOpMode {

    // Verify against the exact goBILDA motor encoder configuration.
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 1400.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drivetrain motors
        DcMotor frontLeftMotor =
                hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor =
                hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor =
                hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor =
                hardwareMap.dcMotor.get("backRightMotor");

        // Intake motor
        DcMotor intakeMotor =
                hardwareMap.dcMotor.get("intakeMotor");

        // Shooter motor with encoder feedback
        DcMotorEx shooterMotor =
                hardwareMap.get(DcMotorEx.class, "shooterMotor");

        // Reverse right drivetrain motors
        frontRightMotor.setDirection(
                DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(
                DcMotorSimple.Direction.REVERSE);

        // Intake configuration
        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter configuration
        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU setup
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        telemetry.addLine("BioBuzz TeleOp Ready");
        telemetry.addLine("P1: Drive | P2: LT Intake, RT Shooter");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // =====================================
            // PLAYER 1: FIELD-CENTRIC DRIVETRAIN
            // =====================================

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading)
                    - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading)
                    + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX)
                            + Math.abs(rx), 1.0);

            double frontLeftPower =
                    (rotY + rotX + rx) / denominator;
            double backLeftPower =
                    (rotY - rotX + rx) / denominator;
            double frontRightPower =
                    (rotY - rotX - rx) / denominator;
            double backRightPower =
                    (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // =====================================
            // PLAYER 2: INTAKE (LEFT TRIGGER)
            // =====================================

            double intakePower = gamepad2.left_trigger;
            intakeMotor.setPower(intakePower);

            // =====================================
            // PLAYER 2: SHOOTER (RIGHT TRIGGER)
            // FIXED TARGET: 1400 RPM
            // =====================================

            boolean shooterHeld = gamepad2.right_trigger > 0.05;

            if (shooterHeld) {
                double targetTicksPerSecond =
                        TARGET_RPM * TICKS_PER_REV / 60.0;

                shooterMotor.setVelocity(targetTicksPerSecond);
            } else {
                shooterMotor.setPower(0.0);
            }

            // =====================================
            // DRIVER STATION TELEMETRY
            // =====================================

            double measuredRPM = shooterMotor.getVelocity()
                    * 60.0 / TICKS_PER_REV;

            telemetry.addData("Intake Power",
                    "%.0f%%", intakePower * 100.0);
            telemetry.addData("Shooter Target RPM",
                    shooterHeld ? "%.0f" : "OFF",
                    TARGET_RPM);
            telemetry.addData("Shooter Measured RPM",
                    "%.0f", measuredRPM);
            telemetry.update();
        }
    }
}
