package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    // 伸缩机构 (Kraken X44)
    private final TalonFX m_deployLeft = new TalonFX(IntakeConstants.DEPLOY_LEFT_ID, IntakeConstants.CAN_BUS);
    private final TalonFX m_deployRight = new TalonFX(IntakeConstants.DEPLOY_RIGHT_ID, IntakeConstants.CAN_BUS);

    // Intake 滚轴电机
    private final TalonFX m_rollerMaster = new TalonFX(IntakeConstants.ROLLER_MASTER_ID, IntakeConstants.CAN_BUS);
    private final TalonFX m_rollerFollower = new TalonFX(IntakeConstants.ROLLER_FOLLOWER_ID, IntakeConstants.CAN_BUS);
    

    // 声明一个基于电压的位置控制请求对象 (Slot 0)
    private final PositionVoltage m_positionRightRequest = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage m_positionLeftRequest = new PositionVoltage(0).withSlot(0);

    private final double GEAR_RATIO = IntakeConstants.GEAR_RATIO;
    private final double TARGET_DEPLOY_ANGLE_DEGREES = IntakeConstants.FULL_DEPLOY_DEGREES;
    public static final double FULL_DEPLOY_DEGREES = IntakeConstants.FULL_DEPLOY_DEGREES;
    private final double INTAKE_SPEED = IntakeConstants.INTAKE_SPEED;
    private final double SYNC_TOLERANCE_DEGREES = IntakeConstants.SYNC_TOLERANCE_DEGREES;

    private boolean deployExtended = false;

    public IntakeSubsystem() {
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        
        // 1. 配置控制参数 (Slot 0)
        deployConfig.Slot0.kP = IntakeConstants.DEPLOY_KP;
        deployConfig.Slot0.kI = 0.0;
        deployConfig.Slot0.kD = IntakeConstants.DEPLOY_KD;

        // 2. 电流限制和刹车模式
        deployConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
        deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // 3. 【关键】：将这一套完全相同的配置，应用给两台电机！
        // 这样 Follower 内部才会有完全一样的 kP 和 kD 去做追踪
        m_deployLeft.getConfigurator().apply(deployConfig);
        m_deployRight.getConfigurator().apply(deployConfig);

        // 4. 【关键】：必须同时将两台电机的当前位置清零！
        m_deployLeft.setPosition(0);
        m_deployRight.setPosition(0);

        // 5. 设置主从模式（确保在位置重置和配置应用之后）
        // 注意：Master 发送 Position 请求时，Follower 也会在本地闭环到该位置的反向值
        m_rollerFollower.setControl(new Follower(m_rollerMaster.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        double leftDeg = (m_deployLeft.getPosition().getValueAsDouble() / GEAR_RATIO) * 360.0;
        double rightDeg = (-m_deployRight.getPosition().getValueAsDouble() / GEAR_RATIO) * 360.0;
        SmartDashboard.putNumber("Intake/DeployLeftDeg", leftDeg);
        SmartDashboard.putNumber("Intake/DeployRightDeg", rightDeg);
        SmartDashboard.putBoolean("Intake/DeployExtended", deployExtended);
    }

    // --- 角度/位置控制逻辑 ---
    
    /**
     * 将推出机构移动到指定的角度 (以度为单位)
     */
    public void setDeployAngle(double targetDegrees) {
        double targetMechanismRotations = targetDegrees / 360.0;
        double targetMotorRotations = targetMechanismRotations * GEAR_RATIO;
        
        // 【关键】：检查当前误差是否小于 1 度，如果小于，就别给 PID 发指令了！
        double currentLeftRotations = m_deployLeft.getPosition().getValueAsDouble();
        if (Math.abs(currentLeftRotations - targetMotorRotations) < (1.0 / 360.0 * GEAR_RATIO)) {
            return; // 已经在目标附近，什么都不做，电机保持静止
        }
        
        m_deployLeft.setControl(m_positionLeftRequest.withPosition(targetMotorRotations));
        m_deployRight.setControl(m_positionRightRequest.withPosition(-targetMotorRotations));
    }

    public void setRollerSpeed(double speed) {
        m_rollerMaster.set(speed);
    }

    public void setDeployFree() {
        setDeployNeutralMode(NeutralModeValue.Coast);
        m_deployLeft.setControl(new DutyCycleOut(0));
        m_deployRight.setControl(new DutyCycleOut(0));
    }

    public void setDeployBack() {
        setDeployNeutralMode(NeutralModeValue.Brake);
        setDeployAngle(0);
    }

    /** Returns the average deploy angle in degrees (left + right) / 2. */
    public double getDeployAngleDegrees() {
        double leftDeg = (m_deployLeft.getPosition().getValueAsDouble() / GEAR_RATIO) * 360.0;
        double rightDeg = (-m_deployRight.getPosition().getValueAsDouble() / GEAR_RATIO) * 360.0;
        return (leftDeg + rightDeg) / 2.0;
    }

    /** Returns true if deploy is within deadband of the target angle. */
    public boolean isDeployAtTarget(double targetDegrees) {
        return Math.abs(getDeployAngleDegrees() - targetDegrees) <= IntakeConstants.DEPLOY_DEADBAND_DEGREES;
    }

    public void toggleDeploy() {
        if (deployExtended) {
            deployExtended = false;
            setDeployBack();
        } else {
            deployExtended = true;
            setDeployNeutralMode(NeutralModeValue.Brake);
            setDeployAngle(-180);
        }
    }

    public void deployOut() {
        deployExtended = true;
        setDeployNeutralMode(NeutralModeValue.Brake);
        setDeployAngle(-180);
    }

    // // --- 封装为 Commands ---

    /** Deploy intake to target angle, wait until within deadband. No roller. */
    public Command deployIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                double left = m_deployLeft.getPosition().getValueAsDouble();
                double right = -m_deployRight.getPosition().getValueAsDouble();
                double diff = Math.abs((left - right) / GEAR_RATIO * 360.0);

                if (diff > SYNC_TOLERANCE_DEGREES) {
                    double averageDeg = (left + right) / GEAR_RATIO * 360.0;
                    setDeployAngle(averageDeg);
                } else {
                    setDeployAngle(TARGET_DEPLOY_ANGLE_DEGREES);
                }
            }),
            Commands.waitUntil(() -> isDeployAtTarget(TARGET_DEPLOY_ANGLE_DEGREES))
                .withTimeout(2.0)
        );
    }

    // 全自动进件指令：包含部署、等待到位、旋转，以及松开时的强制复位
    public Command acquireFuelCommand(BooleanSupplier reverse) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                double left = m_deployLeft.getPosition().getValueAsDouble();
                double right = -m_deployRight.getPosition().getValueAsDouble();
                double diff = Math.abs((left - right) / GEAR_RATIO * 360.0);

                if (diff > SYNC_TOLERANCE_DEGREES) {
                    double averageDeg = (left + right) / GEAR_RATIO * 360.0;
                    setDeployAngle(averageDeg);
                } else {
                    setDeployAngle(TARGET_DEPLOY_ANGLE_DEGREES);
                }
            }),

            // Wait until deploy is within deadband of target angle (max 2s)
            Commands.waitUntil(() -> isDeployAtTarget(TARGET_DEPLOY_ANGLE_DEGREES))
                .withTimeout(2.0),

            this.run(() -> {
                setRollerSpeed(reverse.getAsBoolean() ? -INTAKE_SPEED : INTAKE_SPEED);
            })
        ).finallyDo((interrupted) -> {
            setRollerSpeed(0);
            setDeployFree();
        });
    }

    public Command retractIntake() {
        return this.runOnce(() -> {
            setRollerSpeed(0);
            setDeployBack();
        });
    }

    /**
     * 安全伸出指令：如果两侧电机误差过大，先去到平均位置同步，再伸出到目标角度
     */
    public Command safeDeployCommand(double targetDegrees) {
        // 使用 deferredProxy 的原因是：我们需要在“每次按下按钮的那一瞬间”再去读取位置，
        // 而不是在 RobotContainer 初始化时读取。
        return Commands.deferredProxy(() -> {
            
            // 1. 获取当前两侧的电机圈数
            double leftMotorRots = m_deployLeft.getPosition().getValueAsDouble();
            double rightMotorRots = m_deployRight.getPosition().getValueAsDouble();

            // 由于右侧电机在指令中是反转的 (-targetMotorRotations)，
            // 我们把右侧的圈数取反，统一到左侧的参考系下进行比较。
            double rightRotsInLeftFrame = -rightMotorRots;

            // 2. 换算回机构的角度度数
            double leftCurrentDeg = (leftMotorRots / GEAR_RATIO) * 360.0;
            double rightCurrentDeg = (rightRotsInLeftFrame / GEAR_RATIO) * 360.0;

            // 3. 计算绝对误差
            double diffDegrees = Math.abs(leftCurrentDeg - rightCurrentDeg);

            // 4. 逻辑判断
            if (diffDegrees > SYNC_TOLERANCE_DEGREES) {
                // --- 误差过大，执行修复序列 ---
                // 计算当前两边的平均角度
                double averageDeg = (leftCurrentDeg + rightCurrentDeg) / 2.0;

                return Commands.sequence(
                    // 步骤 A：发送指令让两边先走向平均位置
                    this.runOnce(() -> setDeployAngle(averageDeg)),
                    
                    // 步骤 B：等待，直到两边的位置差缩小到安全范围内
                    Commands.waitUntil(() -> {
                        double currentLeft = m_deployLeft.getPosition().getValueAsDouble();
                        double currentRight = -m_deployRight.getPosition().getValueAsDouble();
                        double currentDiff = Math.abs(currentLeft - currentRight);
                        
                        // 换算当前误差度数
                        double currentDiffDeg = (currentDiff / GEAR_RATIO) * 360.0;
                        return currentDiffDeg <= 1.0; // 同步到 1 度以内放行
                    }).withTimeout(0.5), // 【关键保护】最多等 0.5 秒。如果机构卡死，不能让它无限等下去
                    
                    // 步骤 C：同步完成，继续前往真正的目标位置
                    this.runOnce(() -> setDeployAngle(targetDegrees))
                );

            } else {
                // --- 误差在安全范围内，直接前往目标位置 ---
                return this.runOnce(() -> setDeployAngle(targetDegrees));
            }
        });
    }

    // 动作 1：一键伸出到指定角度（比如 90度）
    public Command deployToAngleCommand() {
        return Commands.startEnd(
            () -> setDeployAngle(TARGET_DEPLOY_ANGLE_DEGREES),
            () -> setDeployAngle(0.0),
            this
            );
    }

    // 动作 2：一键收回（回到 0度）
    public Command retractToAngleCommand() {
        return this.runOnce(() -> setDeployAngle(0.0));
    }

    public Command runIntakeCommand() {
        return this.runEnd(() -> setRollerSpeed(INTAKE_SPEED),
         () -> setRollerSpeed(0)
        );
    }

    public Command reverseIntakeCommand() {
        return this.runEnd(() -> setRollerSpeed(-INTAKE_SPEED),
         () -> setRollerSpeed(0)
        );
    }

    // 配置覆盖

    /**
    * 动态设置部署电机的中性模式（Brake 或 Coast）
    * Phoenix 6 的 apply 机制非常高效，只更新这一组配置不会阻塞总线
    */
    public void setDeployNeutralMode(NeutralModeValue mode) {
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
        motorOutputConfig.NeutralMode = mode;
        
        m_deployLeft.getConfigurator().apply(motorOutputConfig);
        m_deployRight.getConfigurator().apply(motorOutputConfig);
    }


}