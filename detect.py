
import pyzed.sl as sl
import numpy as np
import os 
import time
import random
import cv2 
import matplotlib.pyplot as plt
    
    
import tensorrt as trt    
import pycuda.driver as cuda
import pycuda.autoinit
from utils import postprocess


engine_file = 'best130_192_320.engine'


def preprocess_image(raw_bgr_image,input_h,input_w):
        """
        description: Convert BGR image to RGB,
                     resize and pad it to target size, normalize to [0,1],
                     transform to NCHW format.
        param:
            input_image_path: str, image path
        return:
            image:  the processed image
            image_raw: the original image
            h: original height
            w: original width
        """
        image_raw = raw_bgr_image
        h, w, c = image_raw.shape
        image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        # Calculate widht and height and paddings
        r_w = input_w / w
        r_h = input_h / h
        if r_h > r_w:
            tw = input_w
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((input_h - th) / 2)
            ty2 = input_h - th - ty1
        else:
            tw = int(r_h * w)
            th = input_h
            tx1 = int((input_w - tw) / 2)
            tx2 = input_w - tw - tx1
            ty1 = ty2 = 0
        # Resize the image with long side while maintaining ratio
        image = cv2.resize(image, (tw, th))
        # Pad the short side with (128,128,128)
        image = cv2.copyMakeBorder(
            image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, None, (128, 128, 128)
        )
        image = image.astype(np.float32)
        # Normalize to [0,1]
        image /= 255.0
        # HWC to CHW format:
        image = np.transpose(image, [2, 0, 1])
        # CHW to NCHW format
        image = np.expand_dims(image, axis=0)
        # Convert the image to row-major order, also known as "C order":
        image = np.ascontiguousarray(image)
        return image, image_raw, h, w
def plot_one_box(x, img, color=None, label=None, line_thickness=None):
           
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov8 project.
    param: 
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )
def xywh2xyxy(x):
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2  # top left x
    y[..., 1] = x[..., 1] - x[..., 3] / 2  # top left y
    y[..., 2] = x[..., 0] + x[..., 2] / 2  # bottom right x
    y[..., 3] = x[..., 1] + x[..., 3] / 2  # bottom right y
    return y


class yolov8_trt(object):
    #initiate the engine and setup stuff
    def __init__(self,engine_file_path,input_size):
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)
        stream = cuda.Stream()        
        
        #deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        self.context = engine.create_execution_context() 
        
        self.engine = engine
        self.host_inputs = []
        self.cuda_inputs = []
        self.host_outputs = []
        self.cuda_outputs = []
        self.bindings = []
        self.stream = stream
        
        self.input_h = input_size[0]
        self.input_w = input_size[1]
    
    def infer(self,input_img,input_size,output_shape):
        #set the values
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        stream = self.stream
        context = self.context
        
        input_h = self.input_h
        input_w = self.input_w
        
        input_img_raw = input_img
        
        #allocate memory for input and output
        for binding in engine:
            #create page-locked memory buffers and allocate memory on host and device
            size = trt.volume(engine.get_tensor_shape(binding))
            host_mem = cuda.pagelocked_empty(size, np.float32)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
        
             #append the device buffer to device bindings
            bindings.append(int(cuda_mem))
            #append the binding the the input or output list
            if binding == "images":
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)
                
                
        #preprocess
        input_img, _, _, _ = preprocess_image(input_img_raw,input_size[0],input_size[1])
        #copy input image to host buffer
        np.copyto(host_inputs[0],input_img.ravel())    
        #start the counter  
        start = time.time()
        #transfer the input data to gpu for execution
        cuda.memcpy_htod_async(cuda_inputs[0],host_inputs[0],stream)    
         #run inference
        context.execute_async_v2(bindings=bindings,stream_handle=stream.handle)
        #transfer predicitions from the gpu
        cuda.memcpy_dtoh_async(host_outputs[0],cuda_outputs[0],stream)
        stream.synchronize()
        #end the counter
        end = time.time()
        #amount of time spent
        time_spent = str(end - start)
        output = host_outputs[0]                 
        

        final_output = input_img_raw
        
        output = output.reshape(output_shape[0],output_shape[1],output_shape[2])
        results = postprocess(preds = output,img = input_img,orig_img =  input_img_raw,
        OBJ_THRESH = 0.5,NMS_THRESH = 0.3)   
        
        
        box = []
        conf = 0.0
        
        try:
            results = results[0][0][0]  
        except:
            #didnt find img return false
            return final_output, box, conf, False    
        box = results[:4]
        conf = results[4]
        cls = results[5] 
        #plot_one_box(box,final_output,label="helipad")      
        
        #found an iamge return true for found_img
        return final_output,box,conf,True

        
        
        
 
def main():

    INPUT_SIZE = (192,320)
    OUTPUT_SHAPE = [1,5,1260]
    
    #open zed camera and set the parameters
    zed = sl.Camera()
    init_params = sl.InitParameters()  
    init_params_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.sdk_verbose = 1
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.depth_maximum_distance = 10

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    #create sl.Mat objects
    img_zed= sl.Mat()
    depth = sl.Mat()
    #set zed camera settings
    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 70)
    zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 6)

    y_trt = yolov8_trt(engine_file_path=engine_file,input_size=INPUT_SIZE)
    
    
    found_img = False
    
    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        #grab an image in sl.mat
        zed.retrieve_image(img_zed, sl.VIEW.LEFT)
        #use get_data() to get the numpy array
        img_np = img_zed.get_data()
        frame = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)
    
        #run inference
        res,box,conf,found_img = y_trt.infer(frame,INPUT_SIZE,output_shape=OUTPUT_SHAPE)
        
        #if an img is found
        if found_img:     
            
            
            #print(box)
            center_point_x = int((box[0] + box[2]) / 2)
            center_point_y = int((box[1] + box[3]) / 2)
            #find the depth at the center of the target
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            err,depth_value = depth.get_value(center_point_x,center_point_y)

            #get the camera information
            #TODO: Might not need to run during the runtime constantly 
            #never saw the values chnage. but beter safe than sorry.
            camera_info = zed.get_camera_information()
            calibration_params = camera_info.camera_configuration.calibration_parameters

            f_x = calibration_params.left_cam.fx
            f_y = calibration_params.left_cam.fy
            c_x = calibration_params.left_cam.cx
            c_y = calibration_params.left_cam.cy
            
            #convert to 3d coordinates
            u, v = center_point_x, center_point_y
            
            Z = depth_value
            X = ((u - c_x) * Z) / (f_x)
            Y = ((v - c_y) * Z) / (f_y)
            
            

            label_text = f"x:{X:.2f} y:{Y:.2f} z:{Z:.2f}"
            
            #plot the results
            plot_one_box(box,res,label=label_text)
            
            
         
    
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
        
        cv2.imshow("vid",res)

    
    #close everything at the end
    zed.close()  
    cv2.destroyAllWindows()  
    
    
    

if __name__ == "__main__":
    main()









