package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Swerve tuning opmodes built on Pedro's own CoaxialPod / Swerve classes (revhub 3.0.x).
 * They read everything from the CoaxialPodConfig / SwerveConfig objects in Constants.java,
 * so what you tune here is exactly what Pedro will use.
 *
 *   Analog Min / Max Tuner : hold A to spin every pod, copy min/max into Constants
 *   Servo Direction Check  : confirms positive servo power increases each encoder angle
 *   Swerve Offset Calibrator: jog each wheel to forward, copy the suggested angleOffsetRad
 *   SwerveOffsetsTuner     : checks offsets: pods drive to FORWARD (PI/2); hold A to creep
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

    // Swerve.computePodPowers() hands each pod theta = atan2(forward, -strafe), so in Pedro's
    // pod-angle frame 0 = robot right, PI/2 = FORWARD, PI = left (CCW positive).
    static final double FORWARD = Math.PI / 2.0;

    static final double CREEP_POWER = 0.15;
    static final double TURN_POWER = 0.2;
    static final double DRIVE_CHECK_SCALE = 0.3;
    static final double CHECK_POWER = 0.15;
    static final double JOG_POWER = 0.2;
    static final double SETTLED_RAD = Math.toRadians(3);
    static final double REACHED_RAD = Math.toRadians(10);

    // Order: front left, front right, back left, back right
    static CoaxialPodConfig[] configs() {
        return new CoaxialPodConfig[]{
                Constants.frontLeft, Constants.frontRight, Constants.backLeft, Constants.backRight};
    }

    static CoaxialPod[] buildPods(HardwareMap hw) {
        // MANUAL bulk reads: one hub read per loop no matter how often each encoder is read.
        // (AUTO re-reads the hub whenever a channel is read twice, which the telemetry does.)
        // Every loop that uses these pods must call clearBulkCache() first.
        for (LynxModule hub : hw.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CoaxialPodConfig[] cfgs = configs();
        CoaxialPod[] pods = new CoaxialPod[cfgs.length];
        for (int i = 0; i < cfgs.length; i++) pods[i] = new CoaxialPod(hw, cfgs[i]);
        return pods;
    }

    /** Swerve.drive() needs each pod's podOffset. Shows a message and returns false if one is missing. */
    static boolean podOffsetsSet(LinearOpMode op) {
        for (CoaxialPodConfig c : configs()) {
            boolean ok;
            try {
                ok = c.podOffset.get() != null;
            } catch (Exception e) {
                ok = false;
            }
            if (!ok) {
                op.telemetry.addLine("Missing podOffset for pod: " + c.name.get());
                op.telemetry.addLine("Add c.podOffset.set(Vector2D.cartesian(x, y)) to every CoaxialPodConfig.");
                op.telemetry.update();
                return false;
            }
        }
        return true;
    }

    static void clearBulkCache(HardwareMap hw) {
        for (LynxModule hub : hw.getAll(LynxModule.class)) hub.clearBulkCache();
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

    /** The error CoaxialPod.move() acts on: it flips the target 180 deg when |err| > 90 deg. */
    static double pedroError(CoaxialPod p, double wheelTarget) {
        double err = wrap(p.adjustThetaForEncoder(wheelTarget) - p.getAngleAfterOffsetRad());
        return Math.abs(err) > Math.PI / 2.0 ? wrap(err + Math.PI) : err;
    }

    /** Adds one telemetry line per pod: target vs actual in Pedro's encoder space. */
    static void podTelemetry(LinearOpMode op, CoaxialPod[] pods, double wheelTarget) {
        for (CoaxialPod p : pods) {
            double actual = p.getAngleAfterOffsetRad();
            double target = p.adjustThetaForEncoder(wheelTarget);
            double err = wrap(target - actual);
            double pedroErr = pedroError(p, wheelTarget);
            op.telemetry.addData(p.name(),
                    "raw %.3f rad | actual %.1f deg | target %.1f deg | err %.1f deg (pedro %.1f)",
                    p.getRawAngleRad(), Math.toDegrees(wrap(actual)),
                    Math.toDegrees(wrap(target)), Math.toDegrees(err), Math.toDegrees(pedroErr));
        }
    }

    // ---------------------------------------------------------------------
    /**
     * Axon servos only output their analog position while they are receiving a servo signal,
     * so the pods are built (which initialises every CRServo) before the encoders are read.
     */
    @TeleOp(name = "Analog Min / Max Tuner", group = "Swerve Tuning")
    public static class AnalogMinMaxTuner extends LinearOpMode {
        @Override
        public void runOpMode() {
            CoaxialPod[] pods = buildPods(hardwareMap);
            CoaxialPodConfig[] cfgs = configs();
            AnalogInput[] encs = new AnalogInput[cfgs.length];
            double[] min = new double[cfgs.length];
            double[] max = new double[cfgs.length];
            double[] last = new double[cfgs.length];
            int[] turns = new int[cfgs.length];
            for (int i = 0; i < cfgs.length; i++) {
                encs[i] = hardwareMap.get(AnalogInput.class, cfgs[i].servoEncoderName.get());
                min[i] = 3.3;
                max[i] = 0;
            }

            // Show live voltages during init so you can confirm every encoder is reporting
            while (opModeInInit()) {
                clearBulkCache(hardwareMap);
                for (CoaxialPod p : pods) p.setServoPower(0);
                telemetry.addLine("Robot OFF THE GROUND. Every voltage below should be non-zero.");
                for (int i = 0; i < cfgs.length; i++) {
                    last[i] = encs[i].getVoltage();
                    telemetry.addData(cfgs[i].name.get(), "now %.3f V", last[i]);
                }
                telemetry.update();
            }

            while (opModeIsActive()) {
                clearBulkCache(hardwareMap);
                if (gamepad1.b) {
                    java.util.Arrays.fill(min, 3.3);
                    java.util.Arrays.fill(max, 0);
                    java.util.Arrays.fill(turns, 0);
                }
                telemetry.addLine("Hold A: all pods spin slowly. Keep going until every pod shows 3+ turns.");
                telemetry.addLine("B: reset. Copy min/max into analogMinVoltage/analogMaxVoltage.");
                for (int i = 0; i < cfgs.length; i++) {
                    pods[i].setServoPower(gamepad1.a ? CHECK_POWER : 0.0);
                    double v = encs[i].getVoltage();
                    // A jump of more than half the range is the encoder rolling over: one turn
                    if (Math.abs(v - last[i]) > 1.6) turns[i]++;
                    last[i] = v;
                    min[i] = Math.min(min[i], v);
                    max[i] = Math.max(max[i], v);
                    telemetry.addData(cfgs[i].name.get(),
                            "now %.3f  min %.3f  max %.3f  turns %d", v, min[i], max[i], turns[i]);
                }
                telemetry.update();
            }
            stopPods(pods);
        }
    }

    // ---------------------------------------------------------------------
    /**
     * Pedro's pod PID assumes positive servo power makes the encoder angle INCREASE.
     * If a pod is backwards it never settles: it gets pushed to the 90-degree flip point and
     * chatters there forever, no matter what the PID gains are. Run this before tuning PID.
     */
    @TeleOp(name = "Servo Direction Check", group = "Swerve Tuning")
    public static class ServoDirectionCheck extends LinearOpMode {
        @Override
        public void runOpMode() {
            CoaxialPod[] pods = buildPods(hardwareMap);
            double[] last = new double[pods.length];
            double[] travel = new double[pods.length];

            telemetry.addLine("Robot OFF THE GROUND. Hold A: every servo gets +" + CHECK_POWER + " power.");
            telemetry.addLine("Press B to reset the travel counters.");
            telemetry.update();
            waitForStart();
            for (int i = 0; i < pods.length; i++) last[i] = pods[i].getRawAngleRad();

            while (opModeIsActive()) {
                clearBulkCache(hardwareMap);
                if (gamepad1.b) java.util.Arrays.fill(travel, 0.0);
                for (int i = 0; i < pods.length; i++) {
                    pods[i].setMotorPower(0);
                    pods[i].setServoPower(gamepad1.a ? CHECK_POWER : 0.0);
                    double raw = pods[i].getRawAngleRad();
                    travel[i] += wrap(raw - last[i]);
                    last[i] = raw;

                    String verdict;
                    if (Math.abs(travel[i]) < Math.toRadians(20)) verdict = "hold A longer";
                    else if (travel[i] > 0) verdict = "OK (servoDirection is right)";
                    else verdict = "BACKWARDS -> flip servoDirection";
                    telemetry.addData(pods[i].name(), "travel %.0f deg | %s",
                            Math.toDegrees(travel[i]), verdict);
                }
                telemetry.update();
            }
            stopPods(pods);
        }
    }

    // ---------------------------------------------------------------------
    /**
     * Measures angleOffsetRad. Jog each wheel until it points straight forward, then copy the
     * suggested offset. The suggestion already includes CoaxialPod's +PI/2 target shift
     * (offset = raw - adjustThetaForEncoder(FORWARD)), so Swerve's forward points forward.
     * Run Analog Min / Max Tuner first: the raw angle depends on analogMin/MaxVoltage.
     */
    @TeleOp(name = "Swerve Offset Calibrator", group = "Swerve Tuning")
    public static class SwerveOffsetCalibrator extends LinearOpMode {
        @Override
        public void runOpMode() {
            CoaxialPod[] pods = buildPods(hardwareMap);
            int sel = 0;
            boolean lastUp = false, lastDown = false;

            telemetry.addLine("Robot OFF THE GROUND. Wheels never drive.");
            telemetry.addLine("Dpad up/down: pick pod. Left stick X: jog it (push gently).");
            telemetry.addLine("Point every wheel straight forward, bevel gears all on the SAME side.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                clearBulkCache(hardwareMap);
                if (gamepad1.dpad_up && !lastUp) sel = (sel + pods.length - 1) % pods.length;
                if (gamepad1.dpad_down && !lastDown) sel = (sel + 1) % pods.length;
                lastUp = gamepad1.dpad_up;
                lastDown = gamepad1.dpad_down;

                for (int i = 0; i < pods.length; i++) {
                    pods[i].setMotorPower(0);
                    pods[i].setServoPower(i == sel ? gamepad1.left_stick_x * JOG_POWER : 0.0);

                    double raw = pods[i].getRawAngleRad();
                    double offset = raw - pods[i].adjustThetaForEncoder(FORWARD);
                    offset = ((offset % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI);
                    telemetry.addData((i == sel ? "> " : "  ") + pods[i].name(),
                            "raw %.4f | angleOffsetRad %.4f | current %.4f",
                            raw, offset, configs()[i].angleOffsetRad.get());
                }
                telemetry.update();
            }
            stopPods(pods);
        }
    }

    // ---------------------------------------------------------------------
    @TeleOp(name = "SwerveOffsetsTuner", group = "Swerve Tuning")
    public static class SwerveOffsetsTuner extends LinearOpMode {
        @Override
        public void runOpMode() {
            CoaxialPod[] pods = buildPods(hardwareMap);
            telemetry.addLine("Robot OFF THE GROUND. Pods drive to forward.");
            telemetry.addLine("Hold A to creep the drive motors; all wheels should roll the robot forward.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                clearBulkCache(hardwareMap);
                double power = gamepad1.a ? CREEP_POWER : 0.0;
                for (CoaxialPod p : pods) p.move(FORWARD, power, false);
                podTelemetry(this, pods, FORWARD);
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
            // Swerve.drive() needs each pod's podOffset to build the "O" shape.
            if (!podOffsetsSet(this)) {
                waitForStart();
                return;
            }

            CoaxialPod[] pods = buildPods(hardwareMap);
            Swerve swerve = new Swerve(hardwareMap, Constants.driveConfig, pods);

            telemetry.addLine("Robot OFF THE GROUND. Hold A = rotate CCW, hold B = rotate CW.");
            telemetry.addLine("Pods should form an \"O\". If they form an \"X\", flip encoderReversed on all pods.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                clearBulkCache(hardwareMap);
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
    /**
     * Sends Pedro's own DrivePowers so each sign convention can be checked on the floor.
     * Pedro's convention: forward +, strafe + = LEFT, turn + = CCW (left). The sticks are mapped
     * so that pushing a stick left should move/turn the robot left.
     */
    @TeleOp(name = "Swerve Drive Check", group = "Swerve Tuning")
    public static class SwerveDriveCheck extends LinearOpMode {
        @Override
        public void runOpMode() {
            if (!podOffsetsSet(this)) {
                waitForStart();
                return;
            }
            CoaxialPod[] pods = buildPods(hardwareMap);
            Swerve swerve = new Swerve(hardwareMap, Constants.driveConfig, pods);

            telemetry.addLine("ON THE FLOOR with room. Powers are scaled to " + DRIVE_CHECK_SCALE + ".");
            telemetry.addLine("Left stick: drive (push left = should go LEFT). Right stick X: turn (left = CCW).");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                clearBulkCache(hardwareMap);
                double forward = -gamepad1.left_stick_y * DRIVE_CHECK_SCALE;
                double strafe = -gamepad1.left_stick_x * DRIVE_CHECK_SCALE;
                double turn = -gamepad1.right_stick_x * DRIVE_CHECK_SCALE;
                // Must be called every loop: pod steering only runs inside drive()
                swerve.drive(new DrivePowers(forward, strafe, turn), true);

                telemetry.addData("forward", "%.2f (+ = forward)", forward);
                telemetry.addData("strafe", "%.2f (+ = left)", strafe);
                telemetry.addData("turn", "%.2f (+ = CCW / left)", turn);
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
            // A 90 deg step starts every pod right on CoaxialPod's 90 deg flip boundary, where noise
            // makes it flip back and forth (and each flip is a huge D kick). 45 deg is the default.
            double[] stepSizes = {Math.toRadians(45), Math.toRadians(90)};
            int stepSel = 0;
            boolean atStep = false;

            // Live-tunable gains. CoaxialPod re-reads config.turnController every move(),
            // so swapping the controller here takes effect immediately.
            // Order of the per-pod extras matches configs(): FL, FR, BL, BR
            String[] names = {"kP", "kD", "kV (static push)",
                    "kV extra frontLeft", "kV extra frontRight", "kV extra backLeft", "kV extra backRight"};
            double[] gains = {Constants.turnkP, Constants.turnkD, Constants.turnkV,
                    Constants.kVExtraFrontLeft, Constants.kVExtraFrontRight,
                    Constants.kVExtraBackLeft, Constants.kVExtraBackRight};
            double[] steps = {0.05, 0.002, 0.01, 0.005, 0.005, 0.005, 0.005};
            int sel = 0;
            boolean lastA = false, lastY = false, lastX = false;
            boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

            ElapsedTime loopTimer = new ElapsedTime();
            ElapsedTime stepTimer = new ElapsedTime();
            double settleTime = -1;
            double settleSum = 0;
            int settledSteps = 0, unsettledSteps = 0;
            // Split each step into travel (reach 10 deg) and final creep (10 -> 3 deg),
            // and count which pod is the last one to settle.
            double reachTime = -1, reachSum = 0;
            int reachedSteps = 0;
            int lastPod = -1;
            int[] lastPodCount = new int[pods.length];

            telemetry.addLine("A: toggle pods between 0 and the step. Y: step 45 / 90 deg. Wheels never drive.");
            telemetry.addLine("X: reset settle stats (do this after every gain change).");
            telemetry.addLine("Dpad up/down: pick gain. Dpad left/right: change it.");
            telemetry.addLine("Copy the final gains into Constants.java.");
            telemetry.update();
            waitForStart();
            stepTimer.reset();

            while (opModeIsActive()) {
                clearBulkCache(hardwareMap);
                if (gamepad1.a && !lastA) {
                    // Score the step that just ended before starting the next one
                    if (settleTime >= 0) { settleSum += settleTime; settledSteps++; }
                    else unsettledSteps++;
                    if (reachTime >= 0) { reachSum += reachTime; reachedSteps++; }
                    if (lastPod >= 0) lastPodCount[lastPod]++;
                    atStep = !atStep;
                    stepTimer.reset();
                    settleTime = -1;
                    reachTime = -1;
                    lastPod = -1;
                }
                if (gamepad1.y && !lastY) stepSel = (stepSel + 1) % stepSizes.length;
                if (gamepad1.x && !lastX) {
                    settleSum = 0; settledSteps = 0; unsettledSteps = 0;
                    reachSum = 0; reachedSteps = 0;
                    java.util.Arrays.fill(lastPodCount, 0);
                }
                lastY = gamepad1.y; lastX = gamepad1.x;
                double target = FORWARD + (atStep ? stepSizes[stepSel] : 0.0);
                if (gamepad1.dpad_up && !lastUp) sel = (sel + names.length - 1) % names.length;
                if (gamepad1.dpad_down && !lastDown) sel = (sel + 1) % names.length;
                boolean changed = false;
                if (gamepad1.dpad_right && !lastRight) { gains[sel] += steps[sel]; changed = true; }
                if (gamepad1.dpad_left && !lastLeft) {
                    // Per-pod extras may go negative (less push than the shared kV)
                    gains[sel] = sel < 3 ? Math.max(0, gains[sel] - steps[sel]) : gains[sel] - steps[sel];
                    changed = true;
                }
                lastA = gamepad1.a; lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
                lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;

                if (changed) {
                    Constants.turnkP = gains[0];
                    Constants.turnkD = gains[1];
                    Constants.turnkV = gains[2];
                    Constants.kVExtraFrontLeft = gains[3];
                    Constants.kVExtraFrontRight = gains[4];
                    Constants.kVExtraBackLeft = gains[5];
                    Constants.kVExtraBackRight = gains[6];
                    // Each pod needs its own PID instance (they hold integral/derivative state)
                    CoaxialPodConfig[] cfgs = configs();
                    for (int i = 0; i < cfgs.length; i++) {
                        double kV = Math.max(0, gains[2] + gains[3 + i]);
                        cfgs[i].turnController.set(Controller.pid(gains[0], Constants.turnkI, gains[1])
                                .plus(Controller.proportionalFeedforward(kV)));
                    }
                }

                double worst = 0;
                int worstPod = 0;
                for (int i = 0; i < pods.length; i++) {
                    pods[i].move(target, 0.0, false);
                    double e = Math.abs(pedroError(pods[i], target));
                    if (e > worst) { worst = e; worstPod = i; }
                }
                if (worst < REACHED_RAD && reachTime < 0) reachTime = stepTimer.seconds();
                if (worst < SETTLED_RAD) {
                    if (settleTime < 0) settleTime = stepTimer.seconds();
                } else {
                    settleTime = -1;
                    lastPod = worstPod;
                }

                for (int i = 0; i < names.length; i++) {
                    telemetry.addData((i == sel ? "> " : "  ") + names[i], "%.3f", gains[i]);
                }
                telemetry.addData("wheel target (deg left of forward)", Math.toDegrees(target - FORWARD));
                telemetry.addData("worst pod error (deg)", "%.1f", Math.toDegrees(worst));
                telemetry.addData("step size (deg)", "%.0f", Math.toDegrees(stepSizes[stepSel]));
                telemetry.addData("settled (<3 deg) after", settleTime < 0 ? "not settled" : String.format("%.2f s", settleTime));
                telemetry.addData("avg settle", settledSteps == 0 ? "-" : String.format("%.2f s over %d steps, %d never settled",
                        settleSum / settledSteps, settledSteps, unsettledSteps));
                telemetry.addData("avg reach 10 deg", reachedSteps == 0 ? "-" : String.format("%.2f s", reachSum / reachedSteps));
                StringBuilder last = new StringBuilder();
                for (int i = 0; i < pods.length; i++) {
                    last.append(pods[i].name()).append(' ').append(lastPodCount[i]).append("  ");
                }
                telemetry.addData("last to settle", last.toString());
                telemetry.addData("loop time (ms)", "%.1f", loopTimer.milliseconds());
                loopTimer.reset();
                podTelemetry(this, pods, target);
                telemetry.update();
            }
            stopPods(pods);
        }
    }
}
