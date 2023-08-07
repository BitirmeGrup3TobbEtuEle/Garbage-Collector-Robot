import cv2
import numpy as np
from jetbot import Robot
import Jetson.GPIO as GPIO
import time
import nanocamera as nano
import requests

url = 'http://192.168.10.34:5555/veriler'

robot = Robot()

new_data = {'CopTuru': 'Siyah bardak aranıyor...',
            'CopSayisi': 80
         }
response = requests.post(url, json=new_data)

# PID sabitleri
kp = 0.017
ki = 0.00014
kd = 0.004

# PID Kontrol degiskenleri
prev_error = 0
integral = 0

global x
x = 0
global y
y = 0
global bardak_ilerle
bardak_ilerle= 0

# Arduino ile haberlesmeyi saglayan pinler
pin_numarası =15
copu_aldi = 16
metal_pin=40

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_numarası, GPIO.IN)
GPIO.setup(copu_aldi, GPIO.OUT)
GPIO.setup(metal_pin, GPIO.IN)

global giris_degeri
giris_degeri = GPIO.input(pin_numarası)

global servo_kapat
servo_kapat= GPIO.input(copu_aldi)

global metal_deger
metal_deger=GPIO.input(metal_pin)

global servo_acildi
servo_acildi=0

global metal_arama_devam
metal_arama_devam=0

global yesil_devam_et
yesil_devam_et=0

global kirmizi_arama_devam
kirmizi_arama_devam=0
        
video_capture = nano.Camera (flip=0 ,width = 960, height = 540, fps=60)


# 3 renk icin HSV degerleri

lower_black = np.array([0, 0, 0])
upper_black = np.array([73,93,73])


lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])


lower_green = np.array([39, 73, 83])
upper_green = np.array([99, 253, 128])


def calculate_pid(error):
    global integral, prev_error
    integral += error
    derivative = error - prev_error
    output = kp * error + ki * integral + kd * derivative
    prev_error = error
    return output


def metal_detect():
    global metal_deger
    metal_deger= GPIO.input(metal_pin)
    return metal_deger

def mesafeye_bak():
    global giris_degeri
    giris_degeri = GPIO.input(pin_numarası)
    
    return giris_degeri
        
def servo_kapa():
    global servo_kapat
    global copu_aldi
    servo_kapat=1
    GPIO.output(copu_aldi, GPIO.HIGH)   

def servo_ac():
    global servo_acildi
    global copu_aldi
    servo_acildi=1
    GPIO.output(copu_aldi, GPIO.LOW)  
    
def show_camera():
    window_title = "CSI Camera"
	
    global x
    x= 0
	
    global y
    y= 0
	
    global bardak_ilerle
    bardak_ilerle=0
    
    
    global giris_degeri
    giris_degeri = GPIO.input(pin_numarası)
	
    global servo_kapat
    servo_kapat= GPIO.input(copu_aldi)
	
    global metal_deger
    metal_deger= GPIO.input(metal_pin)
	
    global metal_arama_devam
    global yesil_devam_et
    global kirmizi_arama_devam
    
    black_arama_devam=1
    right_yapti=0
    right_say=0
    devam_et=0
    devam_et1=0
    devam_et2=0
    yesil_deneme=1
    yesil_sayac=0
    pixel=82000
    hizli_don=0
    left_durdur_black=0
    kirmizi_yesil_var=0
    siyah_kontur = 45
    ilk_bardak=0
    count = 0
    lefti_duzelt=0
	
    if True:
        try:
            while True:
                if count >= 2:
                    siyah_kontur = 27
                    
                frame = video_capture.read()
                
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                mask_black = cv2.inRange(hsv, lower_black, upper_black)
                contours_black, _ = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours_black = sorted(contours_black, key=lambda x: cv2.contourArea(x), reverse=True)
    
   
                mask_green = cv2.inRange(hsv, lower_green, upper_green)
                contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours_green = sorted(contours_green, key=lambda x: cv2.contourArea(x), reverse=True)
    
                
    
                mask_red = cv2.inRange(hsv, lower_red, upper_red)
                contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours_red = sorted(contours_red, key=lambda x: cv2.contourArea(x), reverse=True)
                
                
                 
                bardak_ilerle=0
                black_arama_devam=1
                
                
                
                
                if len(contours_green) > 50 and yesil_devam_et==1 and kirmizi_arama_devam==0:
                    
                    
                    new_data = {'CopTuru': 'Cam-Plastik bulundu!',
                                                'CopSayisi': 80
                                                }
                    response = requests.post(url, json=new_data)
                    
                    right_yapti=1
                   
                   
                    c_green = max(contours_green, key=cv2.contourArea)
                    M_green = cv2.moments(c_green)
        
                    if M_green["m00"] != 0:
					
                        cX_green = int(M_green["m10"] / M_green["m00"])
                        cY_green = int(M_green["m01"] / M_green["m00"])
                        center = (cX_green, cY_green)
						
                        frame = cv2.circle(frame, center, 5, (0, 255, 0), -1)
                        frame_height, frame_width= frame.shape[:2]
                        robot_center_x = frame_width // 2
                        robot_center_y = frame_height // 2

                        for cnt in contours_green:
                                    area = cv2.contourArea(cnt)
                                    (x, y, w, h) = cv2.boundingRect(cnt)
                                    if area>800:
                                        (x, y, w, h) = cv2.boundingRect(cnt)
                                        break
                        
                        cv2.line(frame, (robot_center_x,0), (robot_center_x,frame_height), (255, 0, 255), 4)
                        
                        dikdortgen_merkez_x = x+w // 2
                        dikdortgen_merkez_y = y+h // 2
                    
                        cv2.line(frame,(robot_center_x,0),(dikdortgen_merkez_x,dikdortgen_merkez_y),(0,255,255),2)  
                        frameile_dikdortgen_mesafe_x=robot_center_x-dikdortgen_merkez_x
                        frameile_dikdortgen_mesafe_y=robot_center_y-dikdortgen_merkez_y
                
                       
                        error = frameile_dikdortgen_mesafe_x
                        output1 = calculate_pid(error)

                        green_pixels = cv2.countNonZero(mask_green)
                        
                        frame_pixels = mask_green.size
                        
                        
                        if green_pixels < (pixel):
                            
                            # Daha hizli
                            motor_hiz_1=0.26
							# Daha yavas
                            motor_hiz_2=0.22
                                                        
                            # 620-310
                            if abs(frameile_dikdortgen_mesafe_x)>570:
							
                                if frameile_dikdortgen_mesafe_x>0:
								
                                    robot.set_motors(0.07,motor_hiz_1)
                                else:
								
                                    robot.set_motors(motor_hiz_1,0.07)
                            elif abs(frameile_dikdortgen_mesafe_x)>285:
							
                                if frameile_dikdortgen_mesafe_x>0:
                                    
                                    robot.set_motors(0.09,motor_hiz_2)
                                else:
								
                                    robot.set_motors(motor_hiz_2,0.09)
                            else:
							
                                '''new_data = {'CopTuru': 'İlgili alana gidiliyor...',
                                'CopSayisi': 80
                                 }
                                response = requests.post(url, json=new_data)'''
								
                                print("forward yesil")
                                robot.forward(speed=0.19)
                            
                        else:
						
                            robot.stop()
                            time.sleep(6)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            new_data = {'CopTuru': 'Bardak bırakıldı!',
                                'CopSayisi': 80
                                 }
                            response = requests.post(url, json=new_data)
                            time.sleep(3)
                            GPIO.output(16, GPIO.LOW)
                            robot.stop()
                            print("yesil_servo_acildi")
                            yesil_devam_et=0
                            robot.set_motors(-0.3,-0.22)
                            time.sleep(1)
                            robot.stop()
                            right_yapti=0
                            right_say=0
                            pixel=pixel+2000
                            hizli_don=0
                            devam_et2=0
                 
                elif len(contours_red) > 50 and kirmizi_arama_devam==1 and yesil_devam_et==0:
				
                    print("kirmizi_reng_arama_kismi")
                    right_yapti=1
                    new_data = {'CopTuru': 'Metal atık bulundu!',
                                                'CopSayisi': 80
                                                }
                    response = requests.post(url, json=new_data)
                   
                    
                    c_red = max(contours_red, key=cv2.contourArea)
                    M_red = cv2.moments(c_red)
        
                    if M_red["m00"] != 0:
					
                        cX_red = int(M_red["m10"] / M_red["m00"])
                        cY_red = int(M_red["m01"] / M_red["m00"])
                        center = (cX_red, cY_red)
						
                        frame = cv2.circle(frame, center, 5, (0, 255, 0), -1)
                        frame_height, frame_width= frame.shape[:2]
                        robot_center_x = frame_width // 2
                        robot_center_y = frame_height // 2
						
                        for cnt in contours_red:
                                    area = cv2.contourArea(cnt)
                                    (x, y, w, h) = cv2.boundingRect(cnt)
                                    if area>800:
                                        (x, y, w, h) = cv2.boundingRect(cnt)
                                        break
                        
                        cv2.line(frame, (robot_center_x,0), (robot_center_x,frame_height), (255, 0, 255), 4)
                        
                        dikdortgen_merkez_x = x+w // 2
                        dikdortgen_merkez_y = y+h // 2
                    
                        cv2.line(frame,(robot_center_x,0),(dikdortgen_merkez_x,dikdortgen_merkez_y),(0,255,255),2)  
                        frameile_dikdortgen_mesafe_x=robot_center_x-dikdortgen_merkez_x
                        frameile_dikdortgen_mesafe_y=robot_center_y-dikdortgen_merkez_y
                
                        error = frameile_dikdortgen_mesafe_x
                        output1 = calculate_pid(error)

                        red_pixels = cv2.countNonZero(mask_red)
                        
                        frame1_pixels = mask_red.size

                       
                        if red_pixels < (pixel):
						
                            print("kirmiziya_dogru_ilerle")
                            motor_hiz_1=0.26
                            motor_hiz_2=0.22
                            
                            if abs(frameile_dikdortgen_mesafe_x)>570:
							
                                if frameile_dikdortgen_mesafe_x>0:
								
                                    robot.set_motors(0.07,motor_hiz_1)
                                else:
								
                                    robot.set_motors(motor_hiz_1,0.07)
                            elif abs(frameile_dikdortgen_mesafe_x)>285:
							
                                if frameile_dikdortgen_mesafe_x>0:
								
                                    robot.set_motors(0.09,motor_hiz_2)
                                else:
								
                                    robot.set_motors(motor_hiz_2,0.09)
                            else:
							
                                '''new_data = {'CopTuru': 'İlgili alana gidiliyor...',
                                'CopSayisi': 80
                                 }
                                response = requests.post(url, json=new_data)'''
								
                                print("forward kirmizi")
                                robot.forward(speed=0.19)
                        else:
						
                            robot.stop()
                            time.sleep(6)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            GPIO.output(16, GPIO.LOW)
                            new_data = {'CopTuru': 'Bardak bırakıldı!',
                                'CopSayisi': 80
                                 }
                            response = requests.post(url, json=new_data)
                            time.sleep(3)
                            GPIO.output(16, GPIO.LOW)
                            robot.stop()
                            robot.set_motors(-0.22,-0.3)
                            time.sleep(1)
                            robot.stop()
                            right_yapti=0
                            pixel=pixel+2000
                            metal_arama_devam=0
                            print("kirmizi_icin_servo_acildi")
                            kirmizi_arama_devam=0
                            hizli_don=0
                            devam_et1=0
                elif len(contours_black) > siyah_kontur and kirmizi_arama_devam==0 and yesil_devam_et==0 and ilk_bardak==0:
                    
                    """
                    and (len(contours_green) == 0 and len(contours_red) == 0)
                    
                    
                    if kirmizi_yesil_var==1:
                        robot.stop()
                        kirmizi_yesil_var=0
                    """    
                    """
                    if (len(contours_green) > 0 or len(contours_red) > 0):
                        black_arama_devam=0
                        robot.set_motors(-0.25,-0.15)
                    """
					
                    if black_arama_devam==1:
                        
                        c_black = max(contours_black, key=cv2.contourArea)
                        M_black = cv2.moments(c_black)
                        if M_black["m00"] != 0:
                                cX_black = int(M_black["m10"] / M_black["m00"])
                                cY_black = int(M_black["m01"] / M_black["m00"])
                                center = (cX_black, cY_black)
                                
                                frame_height, frame_width= frame.shape[:2]
                                robot_center_x = frame_width // 2
                                robot_center_y = frame_height // 2

                                for cnt in contours_black:
                                    area = cv2.contourArea(cnt)
                                    (x, y, w, h) = cv2.boundingRect(cnt)
                                    if area>800:
                                        (x, y, w, h) = cv2.boundingRect(cnt)
                                        break
            
                                cv2.line(frame, (robot_center_x,0), (robot_center_x,frame_height), (255, 0, 255), 4)
                                
                                
                                dikdortgen_merkez_x = x+w // 2
                                dikdortgen_merkez_y = y+h // 2
                            
                                
                                cv2.line(frame,(robot_center_x,0),(dikdortgen_merkez_x,dikdortgen_merkez_y),(0,255,255),2)  
                                frameile_dikdortgen_mesafe_x=robot_center_x-dikdortgen_merkez_x
                                frameile_dikdortgen_mesafe_y=robot_center_y-dikdortgen_merkez_y
                        
                                
                                error = frameile_dikdortgen_mesafe_x
                                
                                output = calculate_pid(error)
            
                                if kirmizi_arama_devam==1:
								
                                    metal_arama_devam=1
                                if yesil_devam_et==1:
								
                                    pass
                                if metal_arama_devam==0 and yesil_devam_et==0:
								
                                    right_yapti=1
                                    if right_yapti==1 and devam_et==0:
									
                                        devam_et=1
                                        robot.stop()
                                    if mesafeye_bak() !=1 :
                                    
                                        motor_hiz_1=0.22
                                        motor_hiz_2=0.16
                                        
                                        if abs(frameile_dikdortgen_mesafe_x)>225:
										
                                            if frameile_dikdortgen_mesafe_x>0:
											
                                                robot.set_motors(0.07,motor_hiz_1)
                                            else:
											
                                                robot.set_motors(motor_hiz_1,0.07)
                                        elif abs(frameile_dikdortgen_mesafe_x)>100:
										
                                            if frameile_dikdortgen_mesafe_x>0:
											
                                                robot.set_motors(0.09,motor_hiz_2)
                                            else:
											
                                                robot.set_motors(motor_hiz_2,0.09)
                                        else:
										
                                            print("forward_black")
                                            robot.forward(speed=0.19)
                                    else:
                                        
                                        print("black mesafe ok")
                                        robot.stop()
                                        time.sleep(6)
                                        GPIO.output(16, GPIO.HIGH)
                                        time.sleep(10)
                                        robot.backward(0.17)
                                        time.sleep(1)
                                        robot.stop()
                                        siyah_kontur = siyah_kontur - 9
                                        count = count + 1
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        if metal_deger==1:
										
                                            kirmizi_arama_devam=1
                                            print("kirmizi_arama_devam")
                                            print(kirmizi_arama_devam)
                                        else:
										
                                            yesil_devam_et=1 
                                            print("yesil_devam_et")
                                        right_yapti=0
                                        hizli_don=1
                                        devam_et=0
                                        ilk_bardak=1
                elif len(contours_black) > siyah_kontur and (len(contours_green) == 0 and len(contours_red) == 0) and kirmizi_arama_devam==0 and yesil_devam_et==0 and ilk_bardak==1:
                        
                    """
                    if (len(contours_green) > 0 or len(contours_red) > 0):
                        black_arama_devam=0
                        robot.set_motors(-0.25,-0.15)
                    """
                    new_data = {'CopTuru': 'Siyah bardak aranıyor...',
                            'CopSayisi': 80
                                 }
                    response = requests.post(url, json=new_data)
                    					   
                    if black_arama_devam==1:
                        
                        c_black = max(contours_black, key=cv2.contourArea)
                        M_black = cv2.moments(c_black)
                        if M_black["m00"] != 0:
						
                                cX_black = int(M_black["m10"] / M_black["m00"])
                                cY_black = int(M_black["m01"] / M_black["m00"])
                                center = (cX_black, cY_black)
                                
                                frame_height, frame_width= frame.shape[:2]
                                robot_center_x = frame_width // 2
                                robot_center_y = frame_height // 2

                                for cnt in contours_black:
                                    area = cv2.contourArea(cnt)
                                    (x, y, w, h) = cv2.boundingRect(cnt)
                                    if area>800:
                                        (x, y, w, h) = cv2.boundingRect(cnt)
                                        break
            
                                cv2.line(frame, (robot_center_x,0), (robot_center_x,frame_height), (255, 0, 255), 4)
                                
                                dikdortgen_merkez_x = x+w // 2
                                dikdortgen_merkez_y = y+h // 2
                            
                                cv2.line(frame,(robot_center_x,0),(dikdortgen_merkez_x,dikdortgen_merkez_y),(0,255,255),2)  ##silecez
                                frameile_dikdortgen_mesafe_x=robot_center_x-dikdortgen_merkez_x
                                frameile_dikdortgen_mesafe_y=robot_center_y-dikdortgen_merkez_y
                        
                                error = frameile_dikdortgen_mesafe_x
                                
                                output = calculate_pid(error)
            
                                if kirmizi_arama_devam==1:
								
                                    metal_arama_devam=1
                                if yesil_devam_et==1:
								
                                    pass
                                if metal_arama_devam==0 and yesil_devam_et==0:
								
                                    right_yapti=1
                                    if right_yapti==1 and devam_et==0:
									
                                        devam_et=1
                                        robot.stop()
                                    if mesafeye_bak() !=1 :
                                    
                                        motor_hiz_1=0.22
                                        motor_hiz_2=0.16
                                        if abs(frameile_dikdortgen_mesafe_x)>225:
										
                                            if frameile_dikdortgen_mesafe_x>0:
											
                                                robot.set_motors(0.07,motor_hiz_1)
                                                print("robot.left.yavas")
												
                                            else:
											
                                                robot.set_motors(motor_hiz_1,0.07)
                                                print("robot.right.hizli")
                                        elif abs(frameile_dikdortgen_mesafe_x)>100:
										
                                            if frameile_dikdortgen_mesafe_x>0:
											
                                                robot.set_motors(0.09,motor_hiz_2)
                                                print("robot.left.yavas1")
                                            else:
											
                                                robot.set_motors(motor_hiz_2,0.09)
                                                print("robot.right.hizli2")
                                        else:
										
                                            print("forward_black")
                                            robot.forward(speed=0.19)
                                        
                                    else:
                                        
                                        print("black mesafe ok")
                                        robot.stop()
                                        time.sleep(6)
                                        GPIO.output(16, GPIO.HIGH)
                                        time.sleep(10)
                                        robot.backward(0.2)
                                        time.sleep(1)
                                        robot.stop()
                                        siyah_kontur = siyah_kontur - 9
                                        count = count + 1
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        metal_deger= GPIO.input(metal_pin)
                                        if metal_deger==1:
										
                                            new_data = {'CopTuru': 'Metal atık bulundu!',
                                                'CopSayisi': 80
                                                }
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            kirmizi_arama_devam=1
                                            print("kirmizi_arama_devam")
                                            print(kirmizi_arama_devam)
											
                                        else:
										
                                            new_data = {'CopTuru': 'Cam-Plastik bulundu!',
                                                'CopSayisi': 80
                                                }
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            response = requests.post(url, json=new_data)
                                            yesil_devam_et=1 
                                            print("yesil_devam_et")
                                        right_yapti=0
                                        hizli_don=1
                                        devam_et=0
                elif bardak_ilerle == 0 and  right_yapti==0 and hizli_don==0:
				
                    print("left yapti")
                    robot.left(0.18)
                elif bardak_ilerle == 0 and  right_yapti==1 and hizli_don==0:
				
                    print("left duzelt yapti")
                    pass
                elif bardak_ilerle == 0 and  right_yapti==0 and hizli_don==1:
				
                    print("hizli left yapti")
                    robot.left(0.19)
                
                    
                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == 27 or keyCode == ord("q"):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
	
        print("Error: Unable to open camera")

if __name__ == "__main__":
    show_camera()
