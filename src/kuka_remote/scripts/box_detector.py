#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        
        # Create CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        # Publisher for detected boxes info
        self.detection_pub = self.create_publisher(
            String,
            '/box_detections',
            10
        )
        
        # Publisher for visualization image
        self.viz_pub = self.create_publisher(
            Image,
            '/box_detections/image',
            10
        )
        
        # Create OpenCV window for live display
        cv2.namedWindow('Box Detections', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Box Detections', 800, 600)
        
        self.get_logger().info('Box Detector Node Started')
        
        # Color ranges in HSV
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 100, 100]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([160, 100, 100]),
                'upper2': np.array([180, 255, 255])
            },
            'green': {
                'lower': np.array([40, 40, 40]),
                'upper': np.array([80, 255, 255])
            },
            'blue': {
                'lower': np.array([90, 50, 50]),
                'upper': np.array([130, 255, 255])
            }
        }
        
    def detect_color(self, hsv_image, color_name):
        """Detect a specific color in HSV image"""
        if color_name == 'red':
            # Red wraps around in HSV, so we need two ranges
            mask1 = cv2.inRange(hsv_image, 
                               self.color_ranges['red']['lower1'], 
                               self.color_ranges['red']['upper1'])
            mask2 = cv2.inRange(hsv_image, 
                               self.color_ranges['red']['lower2'], 
                               self.color_ranges['red']['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_image, 
                              self.color_ranges[color_name]['lower'], 
                              self.color_ranges[color_name]['upper'])
        
        return mask
    
    def find_bounding_box(self, mask):
        """Find bounding box from mask"""
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Calculate center
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Calculate area
        area = cv2.contourArea(largest_contour)
        
        return {
            'x': int(x),
            'y': int(y),
            'width': int(w),
            'height': int(h),
            'center_x': int(center_x),
            'center_y': int(center_y),
            'area': float(area)
        }
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create visualization image
            viz_image = cv_image.copy()
            
            # Detect all colors
            detections = {}
            colors_bgr = {
                'red': (0, 0, 255),
                'green': (0, 255, 0),
                'blue': (255, 0, 0)
            }
            
            for color_name in ['red', 'green', 'blue']:
                # Detect color
                mask = self.detect_color(hsv_image, color_name)
                
                # Apply morphological operations to reduce noise
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # Find bounding box
                bbox = self.find_bounding_box(mask)
                
                if bbox and bbox['area'] > 100:  # Minimum area threshold
                    detections[color_name] = bbox
                    
                    # Draw bounding box on visualization
                    color_bgr = colors_bgr[color_name]
                    cv2.rectangle(viz_image, 
                                 (bbox['x'], bbox['y']), 
                                 (bbox['x'] + bbox['width'], bbox['y'] + bbox['height']), 
                                 color_bgr, 2)
                    
                    # Draw center point
                    cv2.circle(viz_image, 
                              (bbox['center_x'], bbox['center_y']), 
                              5, color_bgr, -1)
                    
                    # Add label
                    label = f"{color_name}: ({bbox['center_x']}, {bbox['center_y']})"
                    cv2.putText(viz_image, label, 
                               (bbox['x'], bbox['y'] - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 
                               0.5, color_bgr, 2)
            
            # Publish detections as JSON
            if detections:
                detection_msg = String()
                detection_msg.data = json.dumps(detections, indent=2)
                self.detection_pub.publish(detection_msg)
                
                self.get_logger().info(f'Detected {len(detections)} boxes')
            
            # Publish visualization image
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
            self.viz_pub.publish(viz_msg)
            
            # Display in OpenCV window
            cv2.imshow('Box Detections', viz_image)
            cv2.waitKey(1)  # Required for window update
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
