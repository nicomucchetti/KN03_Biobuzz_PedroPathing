package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Swerve tuning opmodes built on Pedro's own CoaxialPod / Swerve classes (revhub 3.0.x).
 * They read everything from the CoaxialPodConfig / SwerveConfig objects in Constants.java,
 * so what you tune here is exactly what Pedro will use.
 *
 *   Analog Min / Max Tuner : rotate each pod 4-5 turns, copy min/max into Constants
 *   SwerveOffsetsTuner     : each pod is driven to wheel-heading 0 (forward); hold A to creep
 *   SwerveTurnTuner        : full Swerve.drive() rotate-in-place; pods should form an "O"
 *   SwervePIDTuner         : press A to toggle every pod between 0 and 90 degrees
 *
 * These are plain @TeleOps, so they show up in the Driver Station list under "Swerve Tuning"
 * without being registered in Tuning.java.
 *
 * SAFETY: wheels OFF THE GROUND for the first runs. Drive motors only run while A is held
 * (except the PID tuner, which never drives the wheels).
 */
public class SwerveTuners {

    static final double CREEP_POWER = 0.15;
    static final double TURN_POWER = 0.2;

    // Order: front left, front right, back left, back right
    static CoaxialPodConfig[] configs() {
        return new CoaxialPodConfig[]{
                Constants.frontLeft, Constants.frontRight, Constants.backLeft, Constants.backRight};
    }

    static CoaxialPod[] buildPods(HardwareMap hw) {
        CoaxialPodConfig[] cfgs = configs();
        CoaxialPod[] pods = new CoaxialPod[cfgs.length];
        for (int i = 0; i < cfgs.length; i++) pods[i] = new CoaxialPod(hw, cfgs[i]);
        return pods;
    }

    static void stopPods(CoaxialPod[] pods) {
        for (CoaxialPod p : pods) {
            p.setMotorPower(0);
            p.setServoPower(0);
        }
    }

    static double wrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    /** Adds one telemetry line per pod: target vs actual in Pedro's encoder space. */
    static void podTelemetry(LinearOpMode op, CoaxialPod[] pods, double wheelTarget) {
        for (CoaxialPod p : pods) {
            double actual = p.getAngleAfterOffsetRad();
            double target = p.adjustThetaForEncoder(wheelTarget);
            double err = wrap(target - actual);
            op.telemetry.addData(p.name(),
                    "raw %.3f rad | actual %.1f deg | target %.1f deg | err %.1f deg",
                    p.getRawAngleRad(), Math.toDegrees(wrap(actual)),
                    Math.toDegrees(wrap(target)), Math.toDegrees(err));
        }
    }

    // ---------------------------------------------------------------------
    @TeleOp(name = "Analog Min / Max Tuner", group = "Swerve Tuning")
    public static class AnalogMinMaxTuner extends LinearOpMode {
        @Override
        public void runOpMode() {
            CoaxialPodConfig[] cfgs = configs();
            AnalogInput[] encs = new AnalogInput[cfgs.length];
            double[] min = new double[cfgs.length];
            double[] max = new double[cfgs.length];
            for (int i = 0; i < cfgs.length; i++) {
                encs[i] = hardwareMap.get(AnalogInput.class, cfgs[i].servoEncoderName.get());
                min[i] = 3.3;
                max[i] = 0;
            }
            telemetry.addLine("After start, slowly rotate each pod 4-5 full turns.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                for (int i = 0; i < cfgs.length; i++) {
                    double v = encs[i].getVoltage();
                    min[i] = Math.min(min[i], v);
                    max[i] = Math.max(max[i], v);
                    telemetry.addData(cfgs[i].name.get(),
                            "now %.3f  min %.3f  max %.3f", v, min[i], max[i]);
                }
                telemetry.update();
            }
        }
    }

    // ---------------------------------------------------------------------
    @TeleOp(name = "SwerveOffsetsTuner", group = "Swerve Tuning")
    public static class SwerveOffsetsTuner extends LinearOpMode {
        @Override
        public void runOpMode() {
            CoaxialPod[] pods = buildPods(hardwareMap);
            telemetry.addLine("Robot OFF THE GROUND. Pods drive to wheel-heading 0 (forward).");
            telemetry.addLine("Hold A to creep the drive motors; all wheels should roll the robot forward.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                double power = gamepad1.a ? CREEP_POWER : 0.0;
                for (CoaxialPod p : pods) p.move(0.0, power, false);
                podTelemetry(this, pods, 0.0);
                telemetry.update();
            }
            stopPods(pods);
        }
    }

    // ---------------------------------------------------------------------
    @TeleOp(name = "SwerveTurnTuner", group = "Swerve Tuning")
    public static class SwerveTurnTuner extends LinearOpMode {
        @Override
        public void runOpMode() {
            // Swerve.drive() needs each pod's podOffset (lateral offset) to build the "O" shape.
            for (CoaxialPodConfig c : configs()) {
                boolean ok;
                try {
                    ok = c.podOffset.get() != null;
                } catch (Exception e) {
                    ok = false;
                }
                if (!ok) {
                    telemetry.addLine("Missing podOffset for pod: " + c.name.get());
                    telemetry.addLine("Add c.podOffset.set(Vector2D.cartesian(x, y)) to every CoaxialPodConfig.");
                    telemetry.update();
                    waitForStart();
                    return;
                }
            }

            CoaxialPod[] pods = buildPods(hardwareMap);
            Swerve swerve = new Swerve(hardwareMap, Constants.driveConfig, pods);

            telemetry.addLine("Robot OFF THE GROUND. Hold A = rotate CCW, hold B = rotate CW.");
            telemetry.addLine("Pods should form an \"O\". If they form an \"X\", flip encoderReversed on all pods.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                double turn = 0.0;
                if (gamepad1.a) turn = TURN_POWER;
                if (gamepad1.b) turn = -TURN_POWER;
                // Must be called every loop: pod steering only runs inside drive()
                swerve.drive(new DrivePowers(0.0, 0.0, turn), true);

                for (CoaxialPod p : pods) {
                    telemetry.addData(p.name(), "angle %.1f deg",
                            Math.toDegrees(wrap(p.getAngleAfterOffsetRad())));
                }
                telemetry.addData("avg angle scaling", swerve.debug().get("averageAngleScaling"));
                telemetry.update();
            }
            swerve.stop();
            stopPods(pods);
        }
    }

    // ---------------------------------------------------------------------
    @TeleOp(name = "SwervePIDTuner", group = "Swerve Tuning")
    public static class SwervePIDTuner extends LinearOpMode {
        @Override
        public void runOpMode() {
            CoaxialPod[] pods = buildPods(hardwareMap);
            double[] targets = {0.0, Math.PI / 2.0};
            int idx = 0;
            boolean lastA = false;

            telemetry.addLine("Press A to toggle all pods between 0 and 90 degrees. Wheels never drive.");
            telemetry.addLine("Edit turnController in Constants.java, redeploy, repeat.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.a && !lastA) idx = (idx + 1) % targets.length;
                lastA = gamepad1.a;

                for (CoaxialPod p : pods) p.move(targets[idx], 0.0, false);

                telemetry.addData("wheel target (deg)", Math.toDegrees(targets[idx]));
                podTelemetry(this, pods, targets[idx]);
                telemetry.update();
            }
            stopPods(pods);
        }
    }
}
