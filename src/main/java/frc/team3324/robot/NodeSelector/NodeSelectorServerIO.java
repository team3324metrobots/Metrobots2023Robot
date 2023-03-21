package frc.team3324.robot.NodeSelector;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;
public class NodeSelectorServerIO implements NodeSelectorIO {
        private final IntegerPublisher nodePublisher;
        private final IntegerSubscriber nodeSubscriber;
        private final BooleanPublisher coneTippedPublisher;
        private final BooleanSubscriber coneTippedSubscriber;
      
        public NodeSelectorServerIO() {
          // Create publisher and subscriber
          var table = NetworkTableInstance.getDefault().getTable("nodeselector");
          nodePublisher = table.getIntegerTopic("node_robot_to_dashboard").publish();
          nodeSubscriber = table.getIntegerTopic("node_dashboard_to_robot").subscribe(-1);
          coneTippedPublisher = table.getBooleanTopic("cone_tipped_robot_to_dashboard").publish();
          coneTippedSubscriber = table.getBooleanTopic("cone_tipped_dashboard_to_robot").subscribe(false);
      
          // Start server
          var app =
              Javalin.create(
                  config -> {
                   
                    config.staticFiles.add(
                        Paths.get(
                                Filesystem.getDeployDirectory().getAbsolutePath().toString(),
                                "nodeselector")
                            .toString(),

                        Location.EXTERNAL);
                      
                      
                  }
                  );
      
          app.start(5800);
        }
      
        public void updateInputs(NodeSelectorIOInputs inputs) {
          for (var value : nodeSubscriber.readQueueValues()) {
            inputs.selectedNode = value;
          }
          for (var value : coneTippedSubscriber.readQueueValues()) {
            inputs.coneTipped = value ? 1 : 0;
          }
        }
      
        public void setSelected(long selected) {
          nodePublisher.set(selected);
        }
      
        public void setConeOrientation(boolean tipped) {
          coneTippedPublisher.set(tipped);
        }
      }

