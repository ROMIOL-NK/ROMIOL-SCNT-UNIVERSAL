import socket
import numpy as np
from ultralytics import YOLO
import cv2
import time
import os
import glob
import math
import torch
import threading
# from message_process import deal_msg

def deal_msg(msg):
    msg_list = msg.strip().split('\n')
    oocyte, injector, holder, soma, polarbody, cytoplasm, cytoplasm_interface, gl_interface, polarbody_likely = [], [], [], [], [], [], [], [], []
    for s in msg_list:
        if not s.strip():
            continue
        try:
            parts = s.split(',')
            if len(parts) < 6:
                continue
            p, x1, y1, x2, y2, name = parts[:6]
            p, x1, y1, x2, y2 = float(p), float(x1), float(y1), float(x2), float(y2)
            name = name.strip()
            position = [x1, y1, x2, y2]
            if name == 'Oocyte':
                oocyte.append([p, position])
            elif name == 'Injector':
                injector.append([p, position])
            elif name == 'Holder':
                holder.append([p, position])
            elif name == 'Soma':
                soma.append([p, position])
            elif name == 'polar_body':
                polarbody.append([p, position])
            elif name == 'Cytoplasm':
                cytoplasm.append([p, position])
            elif name == 'Cytoplasm-Interface':
                cytoplasm_interface.append([p, position])
            elif name == 'GL-Interface':
                gl_interface.append([p, position])
            elif name == 'Polarbody-likely':
                polarbody_likely.append([p, position])

        except ValueError:
            print(f"格式错误，跳过该行: {s}")
            continue

    current_holder, current_injector, current_holder_injector = [], [], []
    if len(holder) + len(injector) > 0:
        if len(holder) > 0:
            for i in holder:
                if i[1][0] <= 3:
                    current_holder.append(i)
        else:
            for i in injector:
                if i[1][0] <= 3:
                    current_holder_injector.append(i)
        for i in injector:
            # if i[1][0] >= 30:
            if (i[1][0] >= 30) and ((abs(i[1][0]-i[1][2])*abs(i[1][1]-i[1][3])) >= 500):# 注射针检测的左侧位置和面积筛选
                current_injector.append(i)
        # current_injector 排序
        # current_injector = sorted(current_injector, key=lambda e: ((e[1][2]-e[1][0])*(e[1][3]-e[1][1])))
        current_injector = sorted(current_injector, key=lambda e: e[0])
    holder_last, injector_last = None, None
    holder_max_p = 0
    if len(current_holder) > 0:
        for i in current_holder:
            if i[0] > holder_max_p:
                holder_last = i
                holder_max_p = i[0]
    else:
        if len(current_holder_injector) > 0:
            x_min = current_holder_injector[0][1][0]
            x_max = current_holder_injector[0][1][2]
            y_min = current_holder_injector[0][1][1]
            y_max = current_holder_injector[0][1][3]
            p = current_holder_injector[0][0]
            if len(current_holder_injector) > 1:
                for i in current_holder_injector:
                    box1 = current_holder_injector[1]
                    box2 = i[1]
                    # box = [x_min, y_min, x_max, y_max]
                    x_left = max(box1[0], box2[0])
                    y_top = max(box1[1], box2[1])
                    x_right = min(box1[2], box2[2])
                    y_bottom = min(box1[3], box2[3])

                    inter_area = max(0, x_right - x_left) * max(0, y_bottom - y_top)
                    union_area = ((box1[2] - box1[0]) * (box1[3] - box1[1]) + (box2[2] - box2[0]) * (box2[3] - box2[1]) - inter_area)
                    area_i = (i[1][2]-i[1][0])*(i[1][3]-i[1][1])
                    if(1.0*area_i/union_area < 0.3):
                        continue
                    
                    if i[1][0] < x_min:
                        x_min = i[1][0]
                    if i[1][2] > x_max:
                        x_max = i[1][2]
                    if i[1][1] < y_min:
                        y_min = i[1][1]
                    if i[1][3] > y_max:
                        y_max = i[1][3]
                    if p < i[0]:
                        p = i[0]
            holder_last = [p, [x_min, y_min, x_max, y_max]]
    if len(current_injector) > 0:
        x_min = current_injector[0][1][0]
        x_max = current_injector[0][1][2]
        y_min = current_injector[0][1][1]
        y_max = current_injector[0][1][3]
        p = current_injector[0][0]
        for idx, val in enumerate(current_injector[1:], start=1):
            print(idx, val)
            if abs(val[1][1]-y_min)<30 and abs(val[1][3]-y_max)<30:
                if val[1][0] < x_min:
                    x_min = val[1][0]
                if val[1][2] > x_max:
                    x_max = val[1][2]
                if val[1][1] < y_min:
                    y_min = val[1][1]
                if val[1][3] > y_max:
                    y_max = val[1][3]
                if p < val[0]:
                    p = val[0]
        injector_last = [p, [x_min, y_min, x_max, y_max]]
    current_cell = None
    if len(oocyte) > 0:
        if holder_last is None:
            current_cell = oocyte[0]
            for i in oocyte:
                if current_cell[0] < i[0]:
                    current_cell = i
        else:
            current_cell = oocyte[0]
            distance_min = (holder_last[1][2] - ((current_cell[1][0] + current_cell[1][2]) / 2)) ** 2 + (
                        ((holder_last[1][1] + holder_last[1][3]) / 2) - (
                            (current_cell[1][1] + current_cell[1][3]) / 2)) ** 2
            for i in oocyte:
                distance = (holder_last[1][2] - ((i[1][0] + i[1][2]) / 2)) ** 2 + (
                            ((holder_last[1][1] + holder_last[1][3]) / 2) - ((i[1][1] + i[1][3]) / 2)) ** 2
                if distance < distance_min:
                    current_cell = i
                    distance_min = distance
    polarbody_current = []
    if current_cell is not None:
        for i in polarbody:
            center_x = (i[1][0] + i[1][2]) / 2
            center_y = (i[1][1] + i[1][3]) / 2
            if current_cell[1][0] < center_x < current_cell[1][2] and current_cell[1][1] < center_y < current_cell[1][3]:
                polarbody_current.append(i)
    polarbody_last = None
    if len(polarbody_current) > 0:
        polarbody_last = polarbody_current[0]
        for i in polarbody_current:
            if i[0] > polarbody_last[0]:
                polarbody_last = i

    msg_final = msg
    if holder_last is not None:
        msg_final = msg_final + "%f,%d,%d,%d,%d,%s\n" % (holder_last[0], holder_last[1][0], holder_last[1][1], holder_last[1][2], holder_last[1][3], "holder_current")
    if injector_last is not None:
        msg_final = msg_final + "%f,%d,%d,%d,%d,%s\n" % (injector_last[0], injector_last[1][0], injector_last[1][1], injector_last[1][2], injector_last[1][3], "injector_current")
    if current_cell is not None:
        msg_final = msg_final + "%f,%d,%d,%d,%d,%s\n" % (current_cell[0], current_cell[1][0], current_cell[1][1], current_cell[1][2], current_cell[1][3], "oocyte_current")
    if polarbody_last is not None:
        msg_final = msg_final + "%f,%d,%d,%d,%d,%s\n" % (polarbody_last[0], polarbody_last[1][0], polarbody_last[1][1], polarbody_last[1][2], polarbody_last[1][3], "polarbody_current")

    return msg_final

# def create_ellipse_mask_from_cell(cell_image):
#     height, width = cell_image.shape[:2]
#     mask = np.zeros((height, width), dtype=np.uint8)
#     center = (width // 2, height // 2)
#     axes = (width // 2, height // 2)
#     cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)
    
#     return mask

def mask_cell_background(cell_image, background_color=(0, 0, 0)):
    height, width = cell_image.shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    center = (width // 2, height // 2)
    axes = (width // 2, height // 2)
    cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)

    # mask = create_ellipse_mask_from_cell(cell_image)
    result = cell_image.copy()
    result[mask == 0] = background_color

    return result


device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
yolomodel = YOLO('F:/SCNT-detect-20250518.pt').to(device)
# yolomodel = YOLO('F:/3_classes.pt').to(device)
# yolomodel = YOLO('F:/yunfu_detect.pt').to(device)
polar_body = YOLO('F:/polar_body.pt').to(device)
print("----------------------------------Model load completed----------------------------------")
img_path = r"F:/.romiol/img.jpg"

class AutoReconnectServer:
    def __init__(self, host='localhost', port=9000, max_retries=10, retry_delay=1):
        self.host = host
        self.port = port
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.running = False
        self.server_socket = None
        
    def start(self):
        """启动服务器，支持自动重启"""
        self.running = True
        retry_count = 0
        
        while self.running and retry_count < self.max_retries:
            try:
                self._start_server()
                retry_count = 0  # 重置重试计数
                
            except OSError as e:
                retry_count += 1
                print(f"[ERROR] 服务器出错: {e}")
                print(f"[INFO] {self.retry_delay}秒后尝试第{retry_count}次重启...")
                time.sleep(self.retry_delay)
                
            except KeyboardInterrupt:
                print("[INFO] 服务器手动停止")
                break
                
    def _start_server(self):
        """启动服务器实例"""
        print(f"[INFO] 启动服务器在 {self.host}:{self.port}")
        
        # 创建并配置socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # 绑定端口
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(1.0)  # 设置超时以便检查运行状态
        
        print(f"[INFO] 服务器已启动，开始接受连接...")
        
        # 主循环接受连接
        while self.running:
            try:
                # 接受新连接
                client_socket, client_addr = self.server_socket.accept()
                print(f"[CONNECT] 新连接来自: {client_addr}")
                
                # 创建新线程处理客户端
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, client_addr)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except socket.timeout:
                # 超时，检查运行状态
                continue
                
            except OSError as e:
                print(f"[ERROR] 接受连接失败: {e}")
                raise  # 重新抛出异常触发重启
                
    def _handle_client(self, client_socket, client_addr):
        """处理客户端连接"""
        try:
            # 简单示例：接收并返回数据
            while True:
                msg = client_socket.recv(7)
                if not msg:
                    raise ConnectionResetError
                sgn = msg.decode("utf-8")
                if sgn:
                    print("Received Signal:", sgn)
                    try:
                        image = cv2.imread(img_path)
                        results = yolomodel.predict(image, conf=0.3)
                        names = results[0].names
                        msg = "\n"# 非空字符串,使得没有物体框时也有返回信息
                        for box in results[0].boxes.data:
                            box = box.cpu().tolist()
                            classId = int(box[-1])
                            name = names[classId]
                            # detections.append(np.array(Box_, np.float64))
                            msg = msg + "%f,%d,%d,%d,%d,%s\n"%(box[4],box[0],box[1],box[2],box[3],name)
                            if name == 'Oocyte':
                                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
                        # if 'Oocyte' in msg : # and 'Polarbody' not in msg:
                                img_Oocyte = image[int(y1): int(y2), int(x1): int(x2)]
                                masked_image = mask_cell_background(img_Oocyte)
                                # cv2.imshow("window", masked_image)
                                # cv2.waitKey(1)
                                # masked_image = img_Oocyte
                                results = polar_body.predict(masked_image, conf=0.3)
                                names_pb = results[0].names
                                for box in results[0].boxes.data:
                                    box = box.cpu().tolist()
                                    classId = int(box[-1])
                                    name = names_pb[classId]
                                    # detections.append(np.array(Box_, np.float64))
                                    msg = msg + "%f,%d,%d,%d,%d,%s\n"%(box[4],box[0] + x1,box[1] + y1,box[2] + x1,box[3] + y1,name)
                        msg_final = deal_msg(msg)
                        
                        client_socket.send(msg_final.encode("UTF-8"))
                        print(msg_final)
                    except:
                        print('image not found!')
                
        except (ConnectionResetError, BrokenPipeError):
            print(f"[DISCONNECT] 客户端 {client_addr} 断开连接")
            
        finally:
            client_socket.close()
            
    def stop(self):
        """停止服务器"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print("[INFO] 服务器已停止")

# 使用示例
if __name__ == "__main__":
    server = AutoReconnectServer(port=9000)
    server.start()
