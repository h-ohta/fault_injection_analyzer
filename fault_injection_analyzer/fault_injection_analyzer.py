import rclpy
from rclpy.node import Node
import csv

from autoware_auto_system_msgs.msg import HazardStatusStamped


class FaultInjectionAnalyzer(Node):

    def __init__(self):
        super().__init__('fault_injection_analyzer')
        self.subscription = self.create_subscription(
            HazardStatusStamped,
            '/system/emergency/hazard_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        diag_no_fault = msg.status.diag_no_fault
        with open("analyzed.csv", mode='w') as f:
            diags = []

            for d in diag_no_fault:
                is_existence = False
                print(d)
                if d.hardware_id != "":
                    name, key = str(d.name).rsplit(sep=": ", maxsplit=1)
                    name = name.rsplit(sep="/", maxsplit=1)[0]

                    # search existence
                    for e in diags:
                        if e[0] == name and e[1] == key:
                            e.append(str(d.hardware_id))
                            is_existence = True
                            break

                    if is_existence is False:
                        diags.append([name, key, str(d.hardware_id)])

            # create header
            header = ["diagnostics", "collecting"]
            for i in range(max([len(x) for x in diags]) - 2):
                header.append("hardware_id_" + str(i))

            # write to csv
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(diags)
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
