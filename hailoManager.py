# hailoManager.py
import numpy as np
import cv2
import os

# HailoRT 라이브러리 로드
try:
    from hailort import VDevice, HEF, InferVStreams, ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType
    HAILO_AVAILABLE = True
except ImportError:
    print("[Error] 'hailort' library not found. Please run: pip install hailort")
    HAILO_AVAILABLE = False

class HailoDetector:
    def __init__(self, hef_path):
        self.active = False
        self.target = None
        self.network_group = None
        
        if not HAILO_AVAILABLE:
            return
        
        if not os.path.exists(hef_path):
            print(f"[Error] Model file not found: {hef_path}")
            return

        try:
            print(f"[Hailo] Loading HEF: {hef_path}")
            self.target = VDevice()
            self.hef = HEF(hef_path)
            
            # 네트워크 설정 (기본값)
            self.configure_params = ConfigureParams.create_from_hef(hef=self.hef, interface=VDevice.DefaultInterface)
            self.network_groups = self.target.configure(self.hef, self.configure_params)
            self.network_group = self.network_groups[0]
            
            # 입출력 스트림 설정 (입력: UINT8, 출력: FLOAT32)
            self.input_vstream_params = InputVStreamParams.make(self.network_group, format_type=FormatType.UINT8)
            self.output_vstream_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)
            
            # 모델 입력 크기 자동 확인 (예: 640x640)
            self.input_info = self.hef.get_input_vstream_infos()[0]
            self.input_h = self.input_info.shape[1]
            self.input_w = self.input_info.shape[2]
            
            print(f"[Hailo] Init Success! Input Size: {self.input_w}x{self.input_h}")
            self.active = True
            
        except Exception as e:
            print(f"[Error] Hailo Init Failed: {e}")
            self.active = False

    def infer(self, frame):
        """
        [입력] OpenCV Frame (numpy)
        [출력] [{'label': 'person', 'bbox': [y1, x1, y2, x2], 'score': 0.9}, ...]
        """
        if not self.active: 
            return []

        # 1. 전처리 (Resize & Expansion)
        resized = cv2.resize(frame, (self.input_w, self.input_h))
        input_data = np.expand_dims(resized, axis=0) # (1, H, W, C)

        detections = []
        try:
            # 2. 추론 (Inference)
            with InferVStreams(self.network_group, self.input_vstream_params, self.output_vstream_params) as pipeline:
                pipeline.send(input_data)
                results = pipeline.recv() # 딕셔너리 형태 {'output_name': array}
                
                # 3. 후처리 (Parsing)
                detections = self._parse_output(results)
                
        except Exception as e:
            print(f"[Warning] Inference Error: {e}")
            
        return detections

    def _parse_output(self, results):
        """
        모델 출력값을 파싱하여 박스 정보로 변환
        (사용하는 .hef 모델이 NMS 후처리가 포함된 버전이라고 가정)
        """
        detections = []
        # results 딕셔너리의 값들 중 하나를 가져옴
        for name, tensor in results.items():
            # 출력 형태가 (1, N, 6) 인지 확인 (Batch, Count, [y1,x1,y2,x2,score,cls])
            # 모델마다 다를 수 있으므로 디버깅 시 print(tensor.shape) 해보는 것이 좋음
            data = np.squeeze(tensor) # (N, 6)
            
            if data.ndim == 2 and data.shape[1] >= 6:
                for det in data:
                    ymin, xmin, ymax, xmax, score, class_id = det[:6]
                    if score < 0.5: continue # 신뢰도 50% 미만 무시
                    
                    label = 'person' if int(class_id) == 0 else 'obstacle'
                    detections.append({
                        'label': label,
                        'bbox': [ymin, xmin, ymax, xmax], # 0.0 ~ 1.0 정규화 좌표
                        'score': score
                    })
        return detections