package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    
    // 增加一个变量，记录上一次的状态，防止每20毫秒重复发送数据
    private String m_lastState = "UNKNOWN";

    public LEDSubsystem() {
        m_led = new AddressableLED(0); // RBG
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setSolidColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
        // 1. 获取当前机器人的真实状态
        String currentState = "UNKNOWN";
        if (DriverStation.isDisabled()) {
            currentState = "DISABLED";
        } else if (DriverStation.isAutonomousEnabled()) {
            currentState = "AUTO";
        } else if (DriverStation.isTeleopEnabled()) {
            currentState = "TELEOP";
        }

        // 2. 只有当状态发生切换时，才去改变颜色和发送数据！
        if (!currentState.equals(m_lastState)) {
            if (currentState.equals("DISABLED")) {
                // 红色，亮度降为 50
                setSolidColor(50, 0, 0); 
            } 
            else if (currentState.equals("AUTO")) {
                // 蓝色，亮度降为 50
                setSolidColor(0, 50, 50); 
            } 
            else if (currentState.equals("TELEOP")) {
                // 绿色，亮度降为 50
                setSolidColor(0, 0, 50); 
            }
            // 更新状态锁
            m_lastState = currentState;
        }
    }
}