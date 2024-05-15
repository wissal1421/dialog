import rclpy
from rclpy import Node

from dialogflow_ros2_interfaces.srv import DialogflowService
from dialogflow_ros2_interfaces import DialogflowClient

class DialogflowService(Node):
    def __init__(self):
        self._dc = DialogflowClient()
        self._service = self.create_service(DialogflowService, '/dialogflow_client/intent_service', self._service_cb)

    def _service_cb(self, req):
        if req.voice:
            df_msg = self._dc.detect_intent_stream()
        else:
            df_msg = self._dc.detect_intent_text(req.text)
        return DialogflowServiceResponse(success=True, result=df_msg)

def main(args=None):
    rclpy.init(args=args)
    ds = DialogflowService()

    ds = DialogflowClient()
    ds.get_logger().info("DF_CLIENT: Dialogflow Service is running...")
    rclpy.spin(ds)

if __name__ == "__main__":
    main()

