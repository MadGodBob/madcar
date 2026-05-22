import cv2
camera = cv2.VideoCapture(0, cv2.CAP_V4L2) #打开摄像头(只有一个摄像头则编号为0，若有2个则依次为0,1)
solution = [640,480]
solution = [320,240]
camera.set(cv2.CAP_PROP_FRAME_WIDTH, solution[0])
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, solution[1])
camera.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
camera.set(cv2.CAP_PROP_FPS, 90)
cv2.namedWindow('Video Cam') #创建窗口"Video Cam"
i=0
count = 1
while cv2.waitKey(1)!=27: #esc键 持续间隔1ms等待按键,若有按键跳出循环
      success, frame =camera.read() #读取摄像头数据
      cv2.imshow('Video Cam', frame) # 显示在窗口"Video Cam"上
      count += 1
      print(count)
      # if cv2.waitKey(1)==32: #空格键存图像
      #    i=i+1
      #    cv2.imwrite(str(i)+".jpg",frame) #存图像
camera.release() #断开摄像头
cv2.destroyAllWindows() #释放所有窗口