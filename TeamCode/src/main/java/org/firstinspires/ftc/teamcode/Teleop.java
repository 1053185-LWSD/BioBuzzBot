package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends LinearOpMode {
    // Proposed names: match each to its own actual motor in Robot Configuration.
    private static final String TRANSFER_MOTOR_NAME = "transferMotor";
    private static final String INTAKE_MOTOR_NAME = "intakeMotor";
    private static final String SHOOTER_MOTOR_NAME = "shooterMotor";

    // Initial test powers; tune on the completed robot.
    private static final double TRANSFER_FEED_POWER = 0.25;
    private static final double TRANSFER_REVERSE_POWER = 0.25;
    private static final double INTAKE_POWER = 0.40;
    private static final double INTAKE_REVERSE_POWER = 0.40;

    // Verify physical directions. Change an individual value to REVERSE if needed.
    private static final DcMotorSimple.Direction TRANSFER_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction INTAKE_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction SHOOTER_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    // 72 mm CAD wheel; provisional ~6 ft shot, ~65-degree exit, ~0.35 m release.
    // The actual launch angle, floor clearance, slip, and pulley ratio need measuring.
    private static final double SHOOTER_FLYWHEEL_RPM = 3500.0; // TEST NUMBER: needs testing on the completed robot and field.
    // External belt ratio ONLY: flywheel pulley teeth / motor pulley teeth.
    // 1.0 assumes equal pulleys; SDK motor ticks/rev already includes its gearbox.
    private static final double SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN = 1.0;
    private static final double SHOOTER_TRIGGER_THRESHOLD = 0.10;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // TRANSFER SETUP: diagram label 1 is the drive motor, not Hub port 1.
        // Assumes its belts mechanically drive rotors 2 and 3.
        // CAD does not establish the actual Hub port or motor direction.
        DcMotor transferMotor = hardwareMap.get(DcMotor.class, TRANSFER_MOTOR_NAME);
        transferMotor.setPower(0);
        transferMotor.setDirection(TRANSFER_DIRECTION);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // INTAKE SETUP: this controls the intake DC motor, not the front servos.
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setPower(0);
        intakeMotor.setDirection(INTAKE_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SHOOTER SETUP: requires the motor encoder cable and correct motor type.
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        shooterMotor.setPower(0);
        shooterMotor.setDirection(SHOOTER_DIRECTION);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double shooterTicksPerMotorTurn = shooterMotor.getMotorType().getTicksPerRev();
        double shooterMotorRpm = SHOOTER_FLYWHEEL_RPM
                * SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN;
        double shooterTargetTicksPerSecond = shooterMotorRpm / 60.0
                * shooterTicksPerMotorTurn;
        double shooterMaxMotorRpm = shooterMotor.getMotorType().getMaxRPM();
        boolean shooterConfigurationValid =
                Double.isFinite(shooterTicksPerMotorTurn) && shooterTicksPerMotorTurn > 0
                && Double.isFinite(SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN)
                && SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN > 0
                && Double.isFinite(shooterTargetTicksPerSecond)
                && shooterTargetTicksPerSecond > 0
                && Double.isFinite(shooterMaxMotorRpm) && shooterMaxMotorRpm > 0
                && shooterMotorRpm <= shooterMaxMotorRpm;

        telemetry.addData("Controls", "RB/LB: transfer; Y/X: intake; RT: shooter (hold)");
        telemetry.addData("Shooter test target (flywheel RPM)", SHOOTER_FLYWHEEL_RPM);
        if (!shooterConfigurationValid) {
            telemetry.addData("Shooter disabled",
                    "Check configured motor type, ticks/rev, RPM limit and pulley ratio");
        }
        telemetry.update();

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        try {
            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                // TRANSFER CONTROLS (controller 1): motor 1 drives rotors 2 and 3.
                // Hold right bumper: feed toward shooter. Hold left: reverse.
                // Release both, or hold both together: stop.
                // Physical feed direction must be verified using TRANSFER_DIRECTION.
                double transferPower = 0.0;
                if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    transferPower = TRANSFER_FEED_POWER;
                } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                    transferPower = -TRANSFER_REVERSE_POWER;
                }
                transferMotor.setPower(transferPower);

                // INTAKE CONTROLS: Y intakes, X reverses; both/neither stops.
                double intakePower = 0.0;
                if (gamepad1.y && !gamepad1.x) {
                    intakePower = INTAKE_POWER;
                } else if (gamepad1.x && !gamepad1.y) {
                    intakePower = -INTAKE_REVERSE_POWER;
                }
                intakeMotor.setPower(intakePower);

                // SHOOTER CONTROL: trigger acts as a hold button, not a speed slider.
                // Release removes power; FLOAT lets the flywheel coast down.
                boolean shooterRequested = gamepad1.right_trigger > SHOOTER_TRIGGER_THRESHOLD;
                if (shooterRequested && shooterConfigurationValid) {
                    shooterMotor.setVelocity(shooterTargetTicksPerSecond);
                } else {
                    shooterMotor.setPower(0);
                }

                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                // Independent controls: allow the shooter to spin up before feeding.
                // No automatic feeding, ball counting, or shooter-ready interlock.
                telemetry.addData("Intake power", intakePower);
                telemetry.addData("Transfer power", transferPower);
                telemetry.addData("Shooter requested", shooterRequested);
                if (shooterConfigurationValid) {
                    double measuredFlywheelRpm = shooterMotor.getVelocity() * 60.0
                            / shooterTicksPerMotorTurn
                            / SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN;
                    telemetry.addData("Shooter flywheel RPM (estimated)", measuredFlywheelRpm);
                    telemetry.addData("Shooter target RPM",
                            shooterRequested ? SHOOTER_FLYWHEEL_RPM : 0.0);
                } else {
                    telemetry.addData("Shooter disabled",
                            "Motor configuration or pulley ratio cannot support target RPM");
                }
                telemetry.update();
            }
        } finally {
            shooterMotor.setPower(0);
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}

