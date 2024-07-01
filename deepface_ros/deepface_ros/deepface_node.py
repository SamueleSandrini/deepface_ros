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
from deepface_msgs.srv import FacesAnalysis as FacesAnalysisSrv
from deepface_msgs.srv import ImageFacesAnalysis
from deepface_msgs.msg import Emotion, Race, Gender, FaceDetection, FaceAnalysis, FacesAnalysis


class DeepFaceNode(CascadeLifecycleNode):

    def __init__(self):
        super().__init__("deepface_node")
        self.face_analysis_pub: Optional[Publisher] = None
        self.image_sub = None
        self.image = None

        self.declare_parameter("publish_online_analysis", True)
        self.declare_parameter("image_reliability",
                            QoSReliabilityPolicy.BEST_EFFORT)


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from {state.label} state...')
        # params
                
        self.publish_online_analysis = self.get_parameter(
            "publish_online_analysis").get_parameter_value().bool_value

        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.face_analysis_pub = self.create_lifecycle_publisher(FacesAnalysis, "faces_analysis", 10)

        self.face_analysis_srv = self.create_service(FacesAnalysisSrv, "faces_analysis", self.face_analysis_callback)
        self.image_face_analysis_srv = self.create_service(ImageFacesAnalysis, "image_faces_analysis", self.image_face_analysis_callback)
        

        # cv bridge
        self.cv_bridge = CvBridge()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from {state.label} state...')
        
        self.image_sub = self.create_subscription(
            Image, "image", self.image_cb,
            self.image_qos_profile
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from {state.label} state...')

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning from {state.label} state...')

        self.destroy_service(self.face_analysis_srv)
        self.destroy_service(self.image_face_analysis_srv)
        self.destroy_publisher(self.face_analysis_pub)

        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Shutting down from {state.label} state...')
        
        self.destroy_service(self.face_analysis_srv)
        self.destroy_service(self.image_face_analysis_srv)
        self.destroy_publisher(self.face_analysis_pub)

        return super().on_shutdown(state)

    def image_cb(self, msg: Image) -> None:
        
        self.image = self.cv_bridge.imgmsg_to_cv2(msg)

        if self.publish_online_analysis:
            faces_analysis_msg = self.face_analysis(self.image)
            self.face_analysis_pub.publish(faces_analysis_msg)

    def image_face_analysis_callback(self, req: ImageFacesAnalysis.Request, res: ImageFacesAnalysis.Response):
        
        cv_image = self.cv_bridge.imgmsg_to_cv2(req.image)
        faces_analysis_msgs = self.face_analysis(cv_image)
        
        res.faces_analysis = faces_analysis_msgs
        res.success = True
        
        return res
    
    def face_analysis_callback(self, req: FacesAnalysisSrv.Request, res: FacesAnalysisSrv.Response):
        
        image_to_analyse = self.image
        
        if image_to_analyse is None:
            res.success = False
            return res
        
        faces_analysis_msgs = self.face_analysis(image_to_analyse)
                
        res.faces_analysis = faces_analysis_msgs
        res.success = True
        
        return res
    
    def face_analysis(self, cv_image) -> FacesAnalysis:
        faces_analysis_msg = FacesAnalysis()
        
        try:
            result = DeepFace.analyze(cv_image)
        except Exception as e:
            return faces_analysis_msg

        for face_analysis in result:
            result_msg = FaceAnalysis()

            result_msg.age = face_analysis['age']
            result_msg.dominant_emotion = face_analysis['dominant_emotion']
            result_msg.dominant_gender = face_analysis['dominant_gender']
            result_msg.dominant_race = face_analysis['dominant_race']
            
            result_msg.face_detection = FaceDetection()
            result_msg.face_detection.roi.x_offset = max(0,face_analysis['region']['x'])
            result_msg.face_detection.roi.y_offset = max(0,face_analysis['region']['y'])
            result_msg.face_detection.roi.height = face_analysis['region']['h']
            result_msg.face_detection.roi.width = face_analysis['region']['w']
            result_msg.face_detection.confidence = face_analysis['face_confidence']

            for emotion in face_analysis['emotion']:
                emotion_msg = Emotion()
                emotion_msg.emotion = emotion
                emotion_msg.score = face_analysis['emotion'][emotion]
                result_msg.emotions.append(emotion_msg)
            for gender, score in face_analysis['gender'].items():
                gender_msg = Gender()
                gender_msg.gender = gender
                gender_msg.score = score
                result_msg.genders.append(gender_msg)
            for race, score in face_analysis['race'].items():
                race_msg = Race()
                race_msg.race = race
                race_msg.score = score
                result_msg.races.append(race_msg)            
            
            faces_analysis_msg.faces_analysis.append(result_msg)
        return faces_analysis_msg
            
    
def main():
    rclpy.init()
    node = DeepFaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()