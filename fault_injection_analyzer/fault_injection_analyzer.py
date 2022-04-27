
import rclpy
from rclpy.node import Node
import csv

from autoware_auto_system_msgs.msg import HazardStatusStamped
from tier4_control_msgs.msg import GateMode


class FaultInjectionAnalyzer(Node):

    def __init__(self):
        super().__init__('fault_injection_analyzer')
        self.sub_hazard = self.create_subscription(
            HazardStatusStamped,
            '/system/emergency/hazard_status',
            self.on_hazard_status,
            10)

        self.sub_gate = self.create_subscription(
            GateMode,
            '/control/current_gate_mode',
            self.on_gate_mode,
            10)

        self.timer = self.create_timer(1.0, self.on_timer)

        self.diags = None
        self.gate_mode = None

    def on_gate_mode(self, msg):
        if msg.data == 0:
            self.gate_mode = "AUTO"
        elif msg.data == 1:
            self.gate_mode = "EXTERNAL"
        else:
            self.gate_mode = "UNKNOWN"

    def on_hazard_status(self, msg):
        self.diags = []
        self.diags += msg.status.diag_no_fault
        self.diags += msg.status.diag_safe_fault
        self.diags += msg.status.diag_latent_fault
        self.diags += msg.status.diag_single_point_fault

    def is_ready(self):
        return self.gate_mode is not None and self.diags is not None

    def on_timer(self):
        if self.is_ready() is False:
            return

        with open("analyzed_" + str(self.gate_mode) + ".csv", mode='w') as f:
            lst = []

            for d in self.diags:
                is_existence = False
                if d.hardware_id != "":
                    name, key = str(d.name).rsplit(sep=": ", maxsplit=1)
                    name = name.rsplit(sep="/", maxsplit=1)[0]

                    # search existence
                    for e in lst:
                        if e[0] == name and e[1] == key:
                            e.append(str(d.hardware_id))
                            is_existence = True
                            break

                    if is_existence is False:
                        lst.append([name, key, str(d.hardware_id)])

            # create header
            header = ["diagnostics", "collecting"]
            for i in range(max([len(x) for x in lst]) - 2):
                header.append("hardware_id_" + str(i))

            # write to csv
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(lst)
        exit(0)


def main():
    rclpy.init()

    analyzer = FaultInjectionAnalyzer()

    rclpy.spin(analyzer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
