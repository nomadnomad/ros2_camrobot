from detector_msg.srv import DetecteObjects
from detector_msg.msg import DetectedObject
import rclpy
from rclpy.node import Node

import sys
sys.path.append('/home/t-matsuo/ros2_camrobot/src/detector_srv/detector_srv/yolov5')
import argparse
from utils.general import check_requirements
import torch
import my_detect

class DetectorService(Node):
    def __init__(self, opt):
        super().__init__('detector_service')
        self.srv = self.create_service(DetecteObjects, 'detecte_objects', self.detecte_objects_callback)
        self.opt = opt
    
    def detecte_objects_callback(self, request, response):
        self.get_logger().info(f'Incoming request\nsorce = {request.source}')
        self.opt.source = request.source
        det = my_detect.main(self.opt)
        #self.get_logger().info(f'det = {det}')
        #self.get_logger().info(f'len(det) = {len(det)}')
        self.get_logger().info(f'x1, y1, x2, y2, confidence, class')
        results = []
        for x1, y1, x2, y2, confidence, class_, class_name in det:
            #self.get_logger().info(f'{x1, y1, x2, y2, confidence, class_}')
            detected_object = DetectedObject()
            detected_object.x1 = float(x1)
            detected_object.y1 = float(y1)
            detected_object.x2 = float(x2)
            detected_object.y2 = float(y2)
            detected_object.confidence = float(confidence)
            detected_object.class_no = int(class_)
            detected_object.class_name = class_name
            self.get_logger().info(f'{x1}\t{y1}\t{x2}\t{y2}\t{confidence}\t{int(class_)}')
            results.append(detected_object)
        response.detected_objects = results
        return response

def main(opt=None, args=None):
    if opt is None:
        opt = parse_argument()
    rclpy.init(args=args)
    detector_service = DetectorService(opt)
    rclpy.spin(detector_service)
    rclpy.shutdown()

def parse_argument():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='yolov5s.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='data/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()

    return opt

if __name__ == '__main__':
    opt = parse_argument()
    main(opt)