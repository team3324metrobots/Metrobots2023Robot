package frc.team3324.robot.NodeSelector;

import org.littletonrobotics.junction.AutoLog;

public interface NodeSelectorIO {
    @AutoLog
    public static class NodeSelectorIOInputs {
      public long selectedNode = -1;
      public long coneTipped = -1;
    }

    public default void updateInputs(NodeSelectorIOInputs inputs) {}

    public default void setSelected(long selected) {}
  
    public default void setConeOrientation(boolean tipped) {}
}

