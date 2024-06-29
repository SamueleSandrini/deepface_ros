from typing import List, Dict, Optional
from dataclasses import dataclass, field

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy_cascade_lifecycle.cascade_lifecycle_node import CascadeLifecycleNode

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge
from deepface import DeepFace

from sensor_msgs.msg import Image
from deepface_msgs.srv import FaceAnalysis as FaceAnalysisSrv
from deepface_msgs.msg import Emotion, Race, Gender, FaceDetection
from deepface_msgs.msg import FaceAnalysis as FaceAnalysisMsg

@dataclass
class DeepFaceNode(CascadeLifecycleNode):
    # _face_analysis_srv: Optional[Service] = field(init=False, default_factory=None)
    # _cv_bridge: CvBridge = field(init=False, default_factory=CvBridge)

    def __post_init__(self):
        super().__init__("deepface_node")
        self._face_analysis_pub: Optional[Publisher] = None
        self.first_configuration = True

        self.declare_parameter("model", "yolov8m.pt")


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from {state.label} state...')
        # params
                
        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self._face_analysis_pub = self.create_lifecycle_publisher(FaceAnalysisMsg, "face_analysis", 10)

        self._face_analysis_srv = self.create_service(FaceAnalysisSrv, "face_analysis", self.face_analysis)
        

        # cv bridge
        self.cv_bridge = CvBridge()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from {state.label} state...')
        
        self._image_sub = self.create_subscription(
            Image, "image", self.image_cb,
            self.image_qos_profile
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from {state.label} state...')

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning from {state.label} state...')

        self.destroy_service(self._face_analysis_srv)
        self.destroy_publisher(self._face_analysis_pub)

        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Shutting down from {state.label} state...')
        self.destroy_service(self._srv_face_analysis_srv)
        self.destroy_publisher(self._face_analysis_pub)

        return super().on_shutdown(state)

    def image_cb(self, msg: Image) -> None:
        self.image = self.cv_bridge.imgmsg_to_cv2(msg)
        face_analysis_msg = self.face_analysis(self.image)
        self._face_analysis_pub.publish(face_analysis_msg)

    def face_analysis_callback(self, req: FaceAnalysisSrv.Request, res: FaceAnalysisSrv.Response):
        
        cv_image = self.cv_bridge.imgmsg_to_cv2(req.image)
        
        face_analysis_msg = self.face_analysis(cv_image)
        
        res.face_analysis = face_analysis_msg
        res.success = True
        
        return res
        
    def face_analysis(self, cv_image) -> FaceAnalysisMsg:
        result = DeepFace.analyze(cv_image)

        result_msg = FaceAnalysisMsg()
        result_msg.age = result['age']
        result_msg.dominant_emotion = result['dominant_emotion']
        result_msg.dominant_gender = result['dominant_gender']
        result_msg.dominant_race = result['dominant_race']
        
        result_msg.face_detection = FaceDetection()
        result_msg.face_detection.roi.x_offset = result['region']['x']
        result_msg.face_detection.roi.y_offset = result['region']['y']
        result_msg.face_detection.roi.height = result['region']['h']
        result_msg.face_detection.roi.width = result['region']['w']
        result_msg.face_confidence = result['face_confidence']

        for emotion in result['emotion']:
            emotion_msg = Emotion()
            emotion_msg.type = emotion
            emotion_msg.value = result['emotion'][emotion]
            result_msg.emotions.append(emotion_msg)
        for gender, score in result['gender'].items():
            gender_msg = Gender()
            gender_msg.gender = gender
            gender_msg.score = score
            result_msg.genders.append(gender_msg)
        for race, score in result['race'].items():
            race_msg = Race()
            race_msg.race = race
            race_msg.score = score
            result_msg.races.append(race_msg)            


    
def main():
    rclpy.init()
    node = DeepFaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()