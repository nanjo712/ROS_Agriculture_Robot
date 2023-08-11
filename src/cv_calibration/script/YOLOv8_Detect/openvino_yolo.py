from ultralytics import YOLO
import cv2
import numpy as np

img_size = (640, 480)

def get_params(camera_matrix, dist_coefs):
    global img_size
    return {
        'camera_matrix' : camera_matrix,
        'dist_coefs' : dist_coefs,
        'new_cammtx' : cv2.getOptimalNewCameraMatrix(
             camera_matrix, dist_coefs, img_size, 0
        )[0]
    }
def undistort(params, frame):
    return cv2.undistort(frame, params['camera_matrix'],
        params['dist_coefs'], None, params['new_cammtx']
    )
g200_params = get_params(camera_matrix=np.array([
        [391.459091, 0.000000, 329.719318],
        [0.000000, 391.714735, 229.722416],
        [0.000000, 0.000000, 1.000000]
    ]),
    dist_coefs=np.array([
        -0.34456177150589806, 0.08938559391911026, 
        0.0026686183140887153, -0.0035206005954522388, 0.0
    ])
)
s908_params = get_params(camera_matrix=np.array([
        [438.3273374513049, 0.0, 347.29674052899026],
        [0.0, 437.0820337808688, 250.16551688612083],
        [0.000000, 0.000000, 1.000000]
    ]),
    dist_coefs=np.array([
        -0.3573633421672698, 0.10407018099293977, 
        0.0034661331203235195, 0.0005119394704358136, 0.0
    ])
)
    

if __name__=="__main__":
    ov_model = YOLO('./best_openvino_model/',task="detect")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_size[1])
    cap.set(cv2.CAP_PROP_FPS, 30)
    while True:
        _, frame = cap.read()
        frame = undistort(g200_params, frame)
        results = ov_model(frame)
        result = results[0]
        cv2.imshow("",result.plot())
        if len(result)>0:
            print(result.boxes.xywhn.numpy()[0])
        if cv2.waitKey(1)==ord('q'):
            break