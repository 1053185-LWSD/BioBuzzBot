package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Chosen start (the manual permits multiple starts; it does not fix one pose):
 * - Our side of the field, touching the perimeter beside our Loading Zone.
 * - The nearest part of the robot is 1 inch beyond an end of the zone.
 * - Aim along the wall toward the zone, then 20 degrees into the field.
 * - Keep the whole robot outside the zone and contact four POLLEN preloads.
 *
 * With that pose, a 6-inch forward drive gives 2.05 inches of wall clearance
 * and extends the leading outline 4.64 inches beyond the zone's end.
 * Confirm overlap with the zone's 11-inch depth using the complete physical
 * footprint: the 104mm ground wheels were not identified in the STL, so its
 * previously measured envelope is not a verified overall chassis envelope.
 *
 * Ground contact is via goBILDA 3625-0202-0104, 104mm mecanum wheels. The two
 * belts on each side separately transmit power from motors to wheels; they
 * do not act as tracks or multiply the distance traveled by two.
 * External belt ratio is provisionally 1:1; confirm pulley tooth counts.
 * Confirm the motor configuration, then measure a run to account for slip.
 * This routine cannot detect the zone or guarantee points without field testing.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Forward Leave + Park", group = "BioBuzz")
public class Autonomous extends LinearOpMode {
    private static final double FORWARD_DISTANCE_INCHES = 6.0;
    private static final double START_ANGLE_FROM_WALL_DEGREES = 20.0;
    private static final double START_GAP_FROM_ZONE_INCHES = 1.0;

    // goBILDA GripForce mecanum wheels, SKU 3625-0202-0104.
    private static final double WHEEL_DIAMETER_MM = 104.0;
    // External transmission only: wheel pulley teeth / motor pulley teeth.
    // Equal tooth counts give 1.0. The driven pulley tooth count is unconfirmed.
    // Example: 48T on wheel / 24T on motor -> 2.0, requiring twice the ticks.
    private static final double MOTOR_OUTPUT_TURNS_PER_WHEEL_TURN = 1.0;
    private static final double INCHES_PER_MOTOR_OUTPUT_TURN =
            Math.PI * WHEEL_DIAMETER_MM / 25.4
                    / MOTOR_OUTPUT_TURNS_PER_WHEEL_TURN;

    // Correct residual slip/scale only after verifying the drivetrain model.
    // Example: a requested 6-inch run measures 5.7 inches -> use 6.0 / 5.7.
    private static final double DISTANCE_CORRECTION = 1.0;
    private static final double DRIVE_POWER = 0.25;
    private static final double DRIVE_TIMEOUT_SECONDS = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight};

        // Match Teleop.java's forward motor directions explicitly on both sides.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        int[] targetTicks = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            // SDK ticks/rev describe the GEARMOTOR OUTPUT SHAFT. Its internal
            // gearbox is already included: do not multiply by 19.2 again.
            double ticksPerOutputTurn = motors[i].getMotorType().getTicksPerRev();
            double target = FORWARD_DISTANCE_INCHES / INCHES_PER_MOTOR_OUTPUT_TURN
                    * ticksPerOutputTurn * DISTANCE_CORRECTION;
            if (Double.isNaN(target) || Double.isInfinite(target)
                    || target < 1 || target > Integer.MAX_VALUE) {
                telemetry.addData("Configuration error", "Check motor type for drive motor %d", i);
                telemetry.update();
                return;
            }
            targetTicks[i] = (int) Math.round(target);
        }

        telemetry.addData("Auto", "Forward Leave + Park");
        telemetry.addData("Forward distance (in)", FORWARD_DISTANCE_INCHES);
        telemetry.addData("Ground travel / output turn (in)", "%.4f",
                INCHES_PER_MOTOR_OUTPUT_TURN);
        telemetry.addData("Motor output turns / wheel turn", MOTOR_OUTPUT_TURNS_PER_WHEEL_TURN);
        telemetry.addData("Targets FL / BL / FR / BR", "%d / %d / %d / %d",
                targetTicks[0], targetTicks[1], targetTicks[2], targetTicks[3]);
        telemetry.addData("Power / timeout", "%.2f / %.1f s",
                DRIVE_POWER, DRIVE_TIMEOUT_SECONDS);
                telemetry.addData(
                "Setup",
                "Touch wall, %.1f in beyond zone, aim %.0f deg inward",
                START_GAP_FROM_ZONE_INCHES,
                START_ANGLE_FROM_WALL_DEGREES
        );
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        ElapsedTime timer = new ElapsedTime();

        try {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setTargetPosition(targetTicks[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            timer.reset();

            for (DcMotor motor : motors) {
                motor.setPower(DRIVE_POWER);
            }

            while (opModeIsActive()
                    && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
                boolean anyBusy = false;

                for (DcMotor motor : motors) {
                    anyBusy |= motor.isBusy();
                }

                if (!anyBusy) {
                    break;
                }

                idle();
            }
        } finally {
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }

            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
}