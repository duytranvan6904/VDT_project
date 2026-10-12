Bước 1: Đổi tên mạng điện thoại

- Tên mạng: NIC STAFF  
- Pass: @Nic1234\!		(pass wifi công ty)  
- Pi 5 kết nối đúng khi đèn chuyển xanh  
- Lưu ý nếu cắm dây type C vào mà đèn vẫn đỏ, mạng đổi đúng rồi thì rút ra xoay dây 180 độ rồi cắm lại


Bước 2: Thiết lập kết nối với PX4

- Mở Windows Power Shell  
- Gõ “ssh vdt_2026@pi5vdt”. Nhấn Enter  
- Gõ pass “anh”. Pass khi gõ sẽ không hiện chữ trên màn hình. Cứ gõ đủ rồi ấn ok  
- Gõ lần lượt các lệnh sau:  
* source /opt/ros/\$ROS\_DISTRO/setup.bash  
* source \~/ros2\_ws/install/setup.bash  
* python3 mavlink\_shell.py /dev/ttyACM0 \--baudrate 57600  
- Sau khi gõ lệnh python3, nhấn Enter tiếp 1-2 lần cho tới khi hiện ra dòng chữ “nsh\>”. Lúc này là vào được MAVLink Console qua Pi 5  
- Gõ tiếp:  
* uxrce\_dds\_client start \-t serial \-d /dev/ttyACM0 \-b 921600  
* mavlink stop \-d /dev/ttyACM0  
- Xong rồi Ctrl \+ C để thoát khỏi cửa sổ Console  
- Gõ MicroXRCEAgent serial \--dev /dev/ttyACM0 \-b 921600 \-v6 để kích hoạt Agent giao tiếp với PX4. Kết quả đúng sẽ thấy cửa sổ màn hình in ra các chữ liên tục  
- Để nguyên cửa sổ không thoát ra


Bước 3: Xây dựng workspace/phát tín hiệu RC ảo

- Mở một cửa sổ Power Shell mới. Đăng nhập vào lại mạng vdt\_2026@pi5vdt như ở bước 2  
- Gõ lần lượt:  
* source /opt/ros/\$ROS\_DISTRO/setup.bash  
* source \~/ros2\_ws/install/setup.bash  
* cd \~/VDT\_project/ros2\_ws  
* colcon build  
* source install/setup.bash  
* python3 land\_arbiter.py \--msg-type vdt\_msgs/msg/RcFsmInput  
- Để nguyên cửa sổ không thoát ra

Bước 4: Chuẩn bị cất cánh

- Mở một cửa sổ Power Shell mới. Đăng nhập vào lại mạng vdt\_2026@pi5vdt như ở bước 2  
- Gõ lần lượt:  
* source /opt/ros/\$ROS\_DISTRO/setup.bash  
* source \~/ros2\_ws/install/setup.bash  
* cd \~/VDT\_project/ros2\_ws  
* colcon build  
* source install/setup.bash  
* ./takeoff.sh \--alt 3  
- Để nguyên cửa sổ không thoát ra


Bước 5: Khởi động toàn bộ hệ thống

- Mở một cửa sổ Power Shell mới. Đăng nhập vào lại mạng vdt\_2026@pi5vdt như ở bước 2  
- Gõ lần lượt:  
* source /opt/ros/\$ROS\_DISTRO/setup.bash  
* source \~/ros2\_ws/install/setup.bash  
* cd \~/VDT\_project/ros2\_ws  
* colcon build  
* source install/setup.bash  
* ros2 launch bringup vdt\_system.launch.py start\_hardware:=false  
- Để nguyên cửa sổ không thoát ra

Bước 6: Kiểm tra thủ công điều kiện trước khi bay 

- Mở một cửa sổ Power Shell mới. Đăng nhập vào lại mạng vdt\_2026@pi5vdt như ở bước 2  
- Gõ lần lượt:  
* source /opt/ros/\$ROS\_DISTRO/setup.bash  
* source \~/ros2\_ws/install/setup.bash  
* cd \~/VDT\_project/ros2\_ws  
* colcon build  
* source install/setup.bash  
* ros2 topic list | grep fmu/out  
* ros2 topic info /rc/fsm\_input  
* ros2 topic echo takeoff\_cmd/state –once  
* python3 preflight\_check.py  
- Yêu cầu: có topic /fmu/out/\*, /rc/fsm\_input đúng 1 publisher, trạng thái idle, và GPS global đạt (exit code 0).  
- Thực hiện cất cánh gõ. Quan sát trạng thái hoạt động của quad. Code hiện tại chỉ mới verify được phần tìm kiếm bám theo và hạ cánh xuống mục tiêu, phần tránh vật cản chưa verify kỹ  
*  ./cmd.sh takeoff

- Hạ cánh qua QGC hoặc qua bằng lệnh  
* ./cmd.sh emergency\_land

Bước 7: Tắt hệ thống

- Đợi disarm \-\> Tắt cửa sổ cất cánh \-\> Tắt cửa sổ khởi động hệ thống \-\> Tắt cửa sổ chuẩn bị cất cánh \-\> Tắt cửa sổ RC ảo \-\> Tắt cửa sổ giao tiếp PX4  
- Thứ tự ngược lúc khởi động

